import threading
import time
from dataclasses import dataclass
from queue import Queue, Empty

import importlib.util

if importlib.util.find_spec("pigpio") is not None:
    pigpio = __import__("pigpio")
else:
    pigpio = None



@dataclass(frozen=True)
class MotionConfig:
    button_pin: int = 27
    limit_switch_pin: int = 17
    home_offset_mm: float = 35.31
    debounce_time_ms: int = 200
    max_travel_mm: float = 20.0

    min_step_mm: float = 0.01
    max_step_mm: float = 0.05
    homing_speed: float = 3.0
    release_speed: float = 5.0
    auto_init_gap_threshold: float = 4.99

    min_move_step: float = 0.01
    max_move_step: float = 0.5
    min_speed: float = 0.1
    max_speed: float = 10.0


class SimulatedPi:
    INPUT = 0
    OUTPUT = 1
    PUD_DOWN = 0
    PUD_UP = 1
    RISING_EDGE = 1

    def __init__(self):
        self.connected = True
        self._pins = {}

    def set_mode(self, gpio, mode):
        self._pins.setdefault(gpio, 0)

    def set_pull_up_down(self, gpio, pud):
        self._pins.setdefault(gpio, 0)

    def callback(self, gpio, edge, func):
        return None

    def read(self, gpio):
        return self._pins.get(gpio, 1)

    def write(self, gpio, level):
        self._pins[gpio] = level

    def stop(self):
        return None


class SimulatedStepper:
    """实时模拟电机运动：按距离/速度 sleep，支持 stop 中断。"""

    def __init__(self):
        self._stop_event = threading.Event()

    def move(self, direction, distance_mm, speed_mm_s):
        if distance_mm <= 0 or speed_mm_s <= 0:
            return
        duration = distance_mm / speed_mm_s
        end_at = time.time() + duration
        self._stop_event.clear()
        while time.time() < end_at:
            if self._stop_event.is_set():
                return
            time.sleep(0.002)

    def stop(self):
        self._stop_event.set()


class MotionController:
    def __init__(self, simulate: bool = False):
        self.simulate = simulate
        self.cfg = MotionConfig()

        if not simulate and pigpio is None:
            raise RuntimeError("真实模式需要安装 pigpio；请安装后重试或使用 --simulate")

        self.pi = SimulatedPi() if simulate else pigpio.pi()
        if simulate:
            self.motor = SimulatedStepper()
        else:
            from DM420 import IndustrialStepper
            self.motor = IndustrialStepper(step_pin=13, dir_pin=19, microstep=1600, lead=1)
        if not simulate:
            self.motor.pi = self.pi

        self.state_lock = threading.Lock()
        self.button_lock = threading.Lock()

        self.current_position_mm = 0.0
        self.zero_position_established = False
        self.cycle_progress = {"current": 0, "total": 0}
        self.cycle_running = False

        self.last_button_press = 0
        self.emergency_event = threading.Event()
        self.abort_event = threading.Event()
        self.cmd_queue = Queue()
        self.stop_all_token = object()

    def debug_log(self, msg):
        mode = "SIM" if self.simulate else "REAL"
        print(f"[{time.strftime('%Y-%m-%d %H:%M:%S')}][{mode}] {msg}")

    def button_press_callback(self, gpio, level, tick):
        now = time.time() * 1000
        with self.button_lock:
            if now - self.last_button_press < self.cfg.debounce_time_ms:
                return
            self.last_button_press = now
        self.abort_event.set()
        self.emergency_event.set()

    def _should_abort(self):
        return self.abort_event.is_set()

    def init_gpio(self):
        if not self.pi.connected:
            raise Exception("pigpio 未连接，请先 sudo pigpiod")

        io = pigpio if pigpio is not None else SimulatedPi
        self.pi.set_mode(self.cfg.button_pin, io.INPUT)
        self.pi.set_pull_up_down(self.cfg.button_pin, io.PUD_DOWN)
        self.pi.callback(self.cfg.button_pin, io.RISING_EDGE, self.button_press_callback)

        self.pi.set_mode(self.cfg.limit_switch_pin, io.INPUT)
        self.pi.set_pull_up_down(self.cfg.limit_switch_pin, io.PUD_UP)

    def start_background_threads(self):
        threading.Thread(target=self.motor_worker, daemon=True).start()
        threading.Thread(target=self.emergency_monitor, daemon=True).start()

    def get_dynamic_step(self, speed):
        if speed < 0.5:
            return self.cfg.min_step_mm
        if speed < 2.0:
            return 0.02
        return self.cfg.max_step_mm

    def get_dynamic_move_step(self, speed):
        speed = max(self.cfg.min_speed, min(self.cfg.max_speed, speed))
        step = self.cfg.min_move_step + (speed - self.cfg.min_speed) / (
            self.cfg.max_speed - self.cfg.min_speed
        ) * (self.cfg.max_move_step - self.cfg.min_move_step)
        return round(step, 4)

    def _actual_gap(self):
        with self.state_lock:
            return self.cfg.max_travel_mm - self.current_position_mm

    def _auto_release_if_needed(self):
        actual_gap = self._actual_gap()
        if actual_gap < self.cfg.auto_init_gap_threshold:
            self.debug_log(f"检测到间隙{actual_gap:.2f}mm < 4.99mm，触发自动紧急释放")
            self.move_to_zero_worker()
            return True
        return False

    def move_mm_worker(self, distance_mm, speed, check_auto_release=True):
        if abs(distance_mm) < 0.001:
            return

        direction = 'up' if distance_mm > 0 else 'down'
        remain = abs(distance_mm)
        step = self.get_dynamic_step(speed)

        while remain > 0.001:
            if self._should_abort():
                self.motor.stop()
                return

            if check_auto_release and self._auto_release_if_needed():
                return

            seg = min(step, remain)
            self.motor.move(direction, seg, speed)
            with self.state_lock:
                self.current_position_mm += seg if direction == 'up' else -seg
            remain -= seg

    def homing_worker(self, speed=None):
        if speed is None:
            speed = self.cfg.homing_speed

        self.debug_log("开始找零")
        self.abort_event.clear()

        if self.simulate:
            with self.state_lock:
                self.current_position_mm = 0.0
                self.zero_position_established = True
            self.debug_log("模拟找零完成")
            return

        while True:
            if self._should_abort():
                self.motor.stop()
                self.debug_log("找零过程被急停中断")
                return

            if self.pi.read(self.cfg.limit_switch_pin) == 0:
                self.motor.stop()
                break

            self.motor.move('down', 0.1, speed)
            with self.state_lock:
                self.current_position_mm -= 0.1

            if self.current_position_mm < -100:
                self.motor.stop()
                self.debug_log("找零超限停止")
                return

        self.move_mm_worker(self.cfg.home_offset_mm, speed, check_auto_release=False)

        with self.state_lock:
            self.current_position_mm = 0.0
            self.zero_position_established = True
        self.debug_log("找零完成")

    def move_to_zero_worker(self):
        with self.state_lock:
            if not self.zero_position_established:
                self.debug_log("零点未建立，先执行找零再回零")
                self.homing_worker(self.cfg.homing_speed)
                return
            dist = -self.current_position_mm

        self.move_mm_worker(dist, self.cfg.release_speed, check_auto_release=False)

    def run_cycle_worker(self, gap, speed, hold, cycles):
        gap = max(5.0, min(20.0, gap))
        speed = max(0.1, min(10.0, speed))
        hold = max(0.1, min(60.0, hold))
        cycles = max(1, min(999, cycles))

        with self.state_lock:
            self.cycle_progress["current"] = 0
            self.cycle_progress["total"] = cycles
            self.cycle_running = True
            base_pos = self.current_position_mm

        target_pos = self.cfg.max_travel_mm - gap

        for i in range(cycles):
            if self._should_abort():
                break

            if self._auto_release_if_needed():
                with self.state_lock:
                    base_pos = self.current_position_mm
                continue

            with self.state_lock:
                move_dist = target_pos - self.current_position_mm
            self.move_mm_worker(move_dist, speed)
            if self._should_abort():
                break

            t0 = time.time()
            while time.time() - t0 < hold:
                if self._should_abort():
                    break
                time.sleep(0.01)
            if self._should_abort():
                break

            with self.state_lock:
                back = base_pos - self.current_position_mm
            self.move_mm_worker(back, speed)

            if self._should_abort():
                break
            with self.state_lock:
                self.cycle_progress["current"] += 1

        with self.state_lock:
            self.cycle_progress["current"] = 0
            self.cycle_running = False
        self.debug_log("循环执行完成/被中断")

    def motor_worker(self):
        self.debug_log("电机worker启动")
        while True:
            try:
                cmd = self.cmd_queue.get(timeout=0.1)
            except Empty:
                continue

            if cmd is self.stop_all_token:
                self.motor.stop()
                with self.cmd_queue.mutex:
                    self.cmd_queue.queue.clear()
                self.cmd_queue.task_done()
                continue

            try:
                self.abort_event.clear()
                cmd()
            except Exception as e:
                self.debug_log(f"worker异常: {e}")
            finally:
                self.cmd_queue.task_done()

    def emergency_monitor(self):
        while True:
            self.emergency_event.wait()
            self.emergency_event.clear()
            self.debug_log("急停触发")
            self.abort_event.set()
            self.motor.stop()
            with self.cmd_queue.mutex:
                self.cmd_queue.queue.clear()
            self.cmd_queue.put(self.move_to_zero_worker)
            time.sleep(0.5)

    # API-facing methods
    def enqueue_move(self, direction, speed):
        actual_gap = self._actual_gap()
        if actual_gap < self.cfg.auto_init_gap_threshold:
            self.debug_log(f"move请求：间隙{actual_gap:.2f}mm < 4.99，触发紧急释放")
            self.abort_event.set()
            self.motor.stop()
            with self.cmd_queue.mutex:
                self.cmd_queue.queue.clear()
            self.cmd_queue.put(self.move_to_zero_worker)
            return {"status": "emergency_release", "message": "间隙过小，已触发紧急释放"}

        step = self.get_dynamic_move_step(speed)
        dist = step if direction == "up" else -step
        self.cmd_queue.put(lambda: self.move_mm_worker(dist, speed))
        return {"status": "moving"}

    def enqueue_stop(self):
        self.abort_event.set()
        self.motor.stop()
        with self.cmd_queue.mutex:
            self.cmd_queue.queue.clear()
        return {"status": "stopped"}

    def enqueue_home(self):
        self.abort_event.set()
        self.motor.stop()
        with self.cmd_queue.mutex:
            self.cmd_queue.queue.clear()
        self.cmd_queue.put(self.move_to_zero_worker)
        return {"status": "releasing"}

    def enqueue_cycle(self, gap, speed, hold, cycles):
        with self.cmd_queue.mutex:
            self.cmd_queue.queue.clear()
        self.cmd_queue.put(lambda: self.run_cycle_worker(gap, speed, hold, cycles))
        return {"status": "cycle_started"}

    def get_position_payload(self):
        with self.state_lock:
            return {
                "position": round(self.current_position_mm, 2),
                "cycle_current": self.cycle_progress["current"],
                "cycle_total": self.cycle_progress["total"],
                "cycle_running": self.cycle_running,
                "mode": "simulate" if self.simulate else "real",
            }

    def startup(self):
        self.init_gpio()
        self.start_background_threads()
        self.debug_log("启动找零")
        self.homing_worker()
        with self.state_lock:
            if not self.zero_position_established:
                raise Exception("初始找零失败，无法启动服务")

    def shutdown(self):
        self.motor.stop()
        if self.pi.connected:
            self.pi.stop()
        self.debug_log("程序退出，资源已清理")
