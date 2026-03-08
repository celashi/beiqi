import pigpio
import time


class IndustrialStepper:
    """
    工业标准脉冲驱动器（DM420 / TB6600 / DM542 通用）
    使用 pigpio.wave 输出精确脉冲
    """

    def __init__(self, step_pin=13, dir_pin=19, microstep=1600, lead=1):
        self.step_pin = step_pin
        self.dir_pin = dir_pin
        self.microstep = microstep
        self.lead = lead
        self.unit_ratio = microstep * lead  # mm -> 步数换算
        self.pi = pigpio.pi()

        self.pi.set_mode(step_pin, pigpio.OUTPUT)
        self.pi.set_mode(dir_pin, pigpio.OUTPUT)

    # -------------------------------------
    # 工具函数
    # -------------------------------------
    def _dir(self, direction):
        """
        direction = 'up' or 'down'
        脉冲前先设定方向，等待方向建立时间（DM420 典型 10us）
        已颠倒上下方向：原'up'变为'down'，原'down'变为'up'
        """
        # 核心修改：颠倒up/down对应的电平值（原逻辑是1→up，0→down；现在0→up，1→down）
        level = 0 if direction == 'up' else 1  # 仅修改这一行即可完成方向调转
        self.pi.write(self.dir_pin, level)
        self.pi.wave_clear()
        time.sleep(0.00002)  # 20us 方向建立时间（>10us）

    def _build_pulse_wave(self, freq, steps):
        """
        构建 wave 脉冲：
        - freq 频率 (Hz)
        - steps 步数
        """
        T = 1.0 / freq              # 单周期(s)
        T_us = int(T * 1e6)         # 单周期(µs)

        # 工业驱动要求至少 4µs 脉宽
        on_us = max(4, T_us // 2)
        off_us = max(4, T_us - on_us)

        pulses = []
        for _ in range(steps):
            pulses.append(pigpio.pulse(1 << self.step_pin, 0, on_us))
            pulses.append(pigpio.pulse(0, 1 << self.step_pin, off_us))

        self.pi.wave_add_generic(pulses)
        wave_id = self.pi.wave_create()
        return wave_id

    # -------------------------------------
    # 工业版运动控制
    # -------------------------------------
    def move(self, direction, distance_mm, speed_mm_s):
        """
        单次直线运动，industrial version
        direction: 'up' or 'down'
        """
        if distance_mm <= 0 or speed_mm_s <= 0:
            print("Invalid parameters")
            return

        # 1. 设置方向
        self._dir(direction)

        # 2. 计算步数和频率
        steps = int(distance_mm * self.unit_ratio)
        time_need = distance_mm / speed_mm_s
        freq = steps / time_need

        # 限幅
        freq = max(20, min(freq, 200000))  # 20Hz~200kHz

        # 3. 构建脉冲
        wave_id = self._build_pulse_wave(freq, steps)
        if wave_id < 0:
            raise RuntimeError(f"wave_create 失败, code={wave_id}")

        # 4. 播放脉冲
        self.pi.wave_send_once(wave_id)

        # 5. 阻塞等待完成
        while self.pi.wave_tx_busy():
            time.sleep(0.0001)

        try:
            self.pi.wave_delete(wave_id)
        except pigpio.error:
            pass

    def stop(self):
        self.pi.wave_tx_stop()