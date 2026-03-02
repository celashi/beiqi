from flask import Flask, request, jsonify, render_template
import threading
import time
import logging
import pigpio
from queue import Queue, Empty
from DM420 import IndustrialStepper

app = Flask(__name__)

# ---------------- pigpio配置 ----------------
BUTTON_PIN = 27
LIMIT_SWITCH_PIN = 17
HOME_OFFSET_MM = 35.31
DEBOUNCE_TIME = 200
pi = pigpio.pi()

Max_Travel = 20

# ---------------- 日志 ----------------
log = logging.getLogger('werkzeug')
log.setLevel(logging.ERROR)

def debug_log(msg):
    print(f"[{time.strftime('%Y-%m-%d %H:%M:%S')}] {msg}")

# ---------------- GPIO ----------------
last_button_press = 0
button_lock = threading.Lock()
emergency_event = threading.Event()

def button_press_callback(gpio, level, tick):
    global last_button_press
    now = time.time() * 1000
    with button_lock:
        if now - last_button_press < DEBOUNCE_TIME:
            return
        last_button_press = now
    emergency_event.set()

def init_gpio():
    if not pi.connected:
        raise Exception("pigpio 未连接，请先 sudo pigpiod")

    pi.set_mode(BUTTON_PIN, pigpio.INPUT)
    pi.set_pull_up_down(BUTTON_PIN, pigpio.PUD_DOWN)
    pi.callback(BUTTON_PIN, pigpio.RISING_EDGE, button_press_callback)

    pi.set_mode(LIMIT_SWITCH_PIN, pigpio.INPUT)
    pi.set_pull_up_down(LIMIT_SWITCH_PIN, pigpio.PUD_UP)

# ---------------- 电机 ----------------
motor_controller = IndustrialStepper(
    step_pin=13,
    dir_pin=19,
    microstep=1600,
    lead=1
)
motor_controller.pi = pi

# ---------------- 参数 ----------------
MIN_STEP_MM = 0.01
MAX_STEP_MM = 0.05
HOMING_SPEED = 3.0
RELEASE_SPEED = 5.0
AUTO_INIT_GAP_THRESHOLD = 4.99

MIN_MOVE_STEP = 0.01
MAX_MOVE_STEP = 0.5
MIN_SPEED = 0.1
MAX_SPEED = 10.0

# ---------------- 状态 ----------------
state_lock = threading.Lock()
current_position_mm = 0.0
zero_position_established = False
cycle_progress = {"current": 0, "total": 0}

# ---------------- 指令队列 ----------------
cmd_queue = Queue()
STOP_ALL = object()

# ---------------- 工具函数 ----------------
def get_dynamic_step(speed):
    if speed < 0.5:
        return MIN_STEP_MM
    elif speed < 2.0:
        return 0.02
    else:
        return MAX_STEP_MM

def get_dynamic_move_step(speed):
    speed = max(MIN_SPEED, min(MAX_SPEED, speed))
    step = MIN_MOVE_STEP + (speed - MIN_SPEED) / (MAX_SPEED - MIN_SPEED) * (MAX_MOVE_STEP - MIN_MOVE_STEP)
    return round(step, 4)

# ---------------- 运动原语 ----------------
def move_mm_worker(distance_mm, speed):
    global current_position_mm

    if abs(distance_mm) < 0.001:
        return

    direction = 'up' if distance_mm > 0 else 'down'
    abs_dist = abs(distance_mm)

    step = get_dynamic_step(speed)
    remain = abs_dist

    while remain > 0.001:
        if emergency_event.is_set():
            motor_controller.stop()
            return
        
        # 恢复：实时检测间隙阈值
        with state_lock:
            actual_gap = Max_Travel - current_position_mm
        if actual_gap < AUTO_INIT_GAP_THRESHOLD:
            debug_log(f"检测到间隙{actual_gap:.2f}mm < 4.99mm，触发自动找零")
            homing_worker(HOMING_SPEED)
            return

        seg = min(step, remain)
        motor_controller.move(direction, seg, speed)

        with state_lock:
            current_position_mm += seg if direction == 'up' else -seg

        remain -= seg

def homing_worker(speed=HOMING_SPEED):
    global current_position_mm, zero_position_established

    debug_log("开始找零")
    emergency_event.clear()  # 清除急停标志

    while True:
        if emergency_event.is_set():
            motor_controller.stop()
            debug_log("找零过程被急停中断")
            return
        
        if pi.read(LIMIT_SWITCH_PIN) == 0:
            motor_controller.stop()
            break

        motor_controller.move('down', 0.1, speed)
        with state_lock:
            current_position_mm -= 0.1

        if current_position_mm < -100:
            motor_controller.stop()
            debug_log("找零超限停止")
            return

    # 找零触发后向上移动偏移量
    move_mm_worker(HOME_OFFSET_MM, speed)

    with state_lock:
        current_position_mm = 0.0
        zero_position_established = True

    debug_log("找零完成")

def move_to_zero_worker():
    global current_position_mm

    with state_lock:
        if not zero_position_established:
            debug_log("零点未建立，先执行找零再回零")
            # 兜底：先找零再回零
            homing_worker(HOMING_SPEED)
            return
        dist = -current_position_mm

    move_mm_worker(dist, RELEASE_SPEED)

# ---------------- 循环 ----------------
def run_cycle_worker(gap, speed, hold, cycles):
    global cycle_progress, current_position_mm

    # 参数校验（恢复app.py的逻辑）
    gap = max(5.0, min(20.0, gap))
    speed = max(0.1, min(10.0, speed))
    hold = max(0.1, min(60.0, hold))
    cycles = max(1, min(999, cycles))

    with state_lock:
        cycle_progress["current"] = 0
        cycle_progress["total"] = cycles
        base_pos = current_position_mm

    target_pos = Max_Travel - gap

    for i in range(cycles):
        if emergency_event.is_set():
            break

        # 循环前校验间隙
        with state_lock:
            actual_gap = Max_Travel - current_position_mm
        if actual_gap < AUTO_INIT_GAP_THRESHOLD:
            debug_log(f"第{i+1}次循环前检测到间隙{actual_gap:.2f}mm < 4.99，触发自动找零")
            homing_worker(HOMING_SPEED)
            with state_lock:
                base_pos = current_position_mm  # 重新校准基准位置
            continue

        # 移动到目标位置
        with state_lock:
            move_dist = target_pos - current_position_mm
        move_mm_worker(move_dist, speed)
        if emergency_event.is_set():
            break

        # 保持咬合状态
        t0 = time.time()
        while time.time() - t0 < hold:
            if emergency_event.is_set():
                break
            time.sleep(0.01)
        if emergency_event.is_set():
            break

        # 返回基准位置
        with state_lock:
            back = base_pos - current_position_mm
        move_mm_worker(back, speed)
        if emergency_event.is_set():
            break

        # 更新循环进度
        with state_lock:
            cycle_progress["current"] += 1

    # 循环结束/中断后重置进度
    with state_lock:
        cycle_progress["current"] = 0
    debug_log("循环执行完成/被中断")

# ---------------- Worker 主线程 ----------------
def motor_worker():
    debug_log("电机worker启动")

    while True:
        try:
            cmd = cmd_queue.get(timeout=0.1)
        except Empty:
            continue

        if cmd is STOP_ALL:
            motor_controller.stop()
            with cmd_queue.mutex:
                cmd_queue.queue.clear()
            continue

        try:
            cmd()
        except Exception as e:
            debug_log(f"worker异常: {e}")
        finally:
            cmd_queue.task_done()

# ---------------- pigpio 急停监听 ----------------
def emergency_monitor():
    while True:
        emergency_event.wait()
        emergency_event.clear()

        debug_log("急停触发")

        # 立即停止电机
        motor_controller.stop()
        # 清空队列（加锁避免竞态）
        with cmd_queue.mutex:
            cmd_queue.queue.clear()
        # 提交回零任务（确保执行）
        cmd_queue.put(move_to_zero_worker)
        # 等待回零任务完成（避免重复触发）
        time.sleep(0.5)

# ---------------- Flask ----------------
@app.route("/")
def index():
    return render_template("index.html")

@app.route("/move", methods=["POST"])
def move():
    data = request.json
    direction = data["direction"]
    speed = float(data["speed"])

    # 移动前检测间隙（恢复app.py的逻辑）
    with state_lock:
        actual_gap = Max_Travel - current_position_mm
    if actual_gap < AUTO_INIT_GAP_THRESHOLD:
        debug_log(f"move请求：间隙{actual_gap:.2f}mm < 4.99，触发紧急释放")
        with cmd_queue.mutex:
            cmd_queue.queue.clear()
        cmd_queue.put(homing_worker)
        return jsonify({"status": "emergency_release", "message": "间隙过小，已触发紧急释放"})

    step = get_dynamic_move_step(speed)
    dist = step if direction == "up" else -step

    cmd_queue.put(lambda: move_mm_worker(dist, speed))
    return jsonify({"status": "moving"})

@app.route("/stop", methods=["POST"])
def stop():
    motor_controller.stop()
    with cmd_queue.mutex:
        cmd_queue.queue.clear()
    emergency_event.clear()  # 清除急停标志
    return jsonify({"status": "stopped"})

@app.route("/home", methods=["POST"])
def home():
    with cmd_queue.mutex:
        cmd_queue.queue.clear()
    cmd_queue.put(move_to_zero_worker)
    return jsonify({"status": "releasing"})

@app.route("/cycle", methods=["POST"])
def cycle():
    data = request.json
    gap = float(data["gap"])
    speed = float(data["speed"])
    hold = float(data["hold"])
    cycles = int(data["cycles"])

    with cmd_queue.mutex:
        cmd_queue.queue.clear()

    cmd_queue.put(lambda: run_cycle_worker(gap, speed, hold, cycles))
    return jsonify({"status": "cycle_started"})

@app.route("/position", methods=["GET"])
def position():
    with state_lock:
        pos = round(current_position_mm, 2)
        c = cycle_progress["current"]
        t = cycle_progress["total"]
    return jsonify({
        "position": pos,
        "cycle_current": c,
        "cycle_total": t
    })

# ---------------- main ----------------
if __name__ == "__main__":
    try:
        init_gpio()

        # 启动worker线程
        threading.Thread(target=motor_worker, daemon=True).start()
        threading.Thread(target=emergency_monitor, daemon=True).start()

        debug_log("启动找零")
        # 同步执行找零（确保启动后有零点）
        homing_worker()
        
        # 校验找零结果
        with state_lock:
            if not zero_position_established:
                raise Exception("初始找零失败，无法启动服务")

        debug_log("服务启动成功，监听 0.0.0.0:49110")
        app.run(host="0.0.0.0", port=49110, debug=False)

    except Exception as e:
        debug_log(f"启动失败: {e}")
    finally:
        motor_controller.stop()
        if pi.connected:
            pi.stop()
        debug_log("程序退出，资源已清理")