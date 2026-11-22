"""
controller.py — Arduino Mega2560 控制器仿真
复现 servoController.ino 的核心逻辑:
  - 串口协议解析 (<255><255> + 4×16bit)
  - mapToRange 行程映射 (pos / 1.7)
  - PWM 脉冲频率计算 (samplePeriod / |target - current|)
  - 位置跟踪仿真
"""

import numpy as np
from config import ARDUINO, SERIAL_PROTOCOL


class ArduinoController:
    """模拟一块 Arduino Mega2560 控制板"""

    def __init__(self, board_id=0, axes_names=None):
        self.board_id = board_id
        self.axes = axes_names if axes_names else []
        self.xingcheng = ARDUINO['xingcheng']
        self.map_divisor = ARDUINO['map_divisor']
        self.center_pos = ARDUINO['center_pos']
        self.max_pos = ARDUINO['max_pos']
        self.sample_period = ARDUINO['sample_period']
        self.min_period = ARDUINO['min_period']
        self.max_period = ARDUINO['max_period']

        # 电机状态 (与 .ino 中的变量对应)
        self.num_motors = len(self.axes)
        self.target_pos = np.zeros(self.num_motors, dtype=np.int32)
        self.current_pos = np.zeros(self.num_motors, dtype=np.int32)
        self.inc_pos = np.zeros(self.num_motors, dtype=np.int32)
        self.is_running = np.zeros(self.num_motors, dtype=bool)
        self.stop_signal = np.zeros(self.num_motors, dtype=bool)
        self.pwm_periods = np.zeros(self.num_motors, dtype=float)

    def map_to_range(self, pos):
        """复现 mapToRange: pos / 1.7, 限幅到 maxPos"""
        mapped = pos / self.map_divisor
        if mapped > self.max_pos:
            mapped = self.max_pos
        return int(mapped)

    def physical_displacement(self, arduino_pos):
        """
        将 Arduino 位置单位转为物理位移 (mm)
        arduino_pos 范围 [0, max_pos=38400]
        物理行程 [0, xingcheng=192mm]
        位移 = arduino_pos * xingcheng / max_pos - xingcheng/2  (相对于中位)
        """
        return arduino_pos * self.xingcheng / self.max_pos - self.xingcheng / 2.0

    def displacement_to_arduino(self, disp_mm):
        """物理位移(mm) → Arduino 位置单位"""
        return int((disp_mm + self.xingcheng / 2.0) / self.xingcheng * self.max_pos)

    # 电缸物理半行程 = 总行程 192mm / 2 = 96mm。
    # 16bit 的 0 ↔ 0 步 ↔ -96mm，65535 ↔ 38400 步 ↔ +96mm（固件除数 65535/38400）。
    # 运动指令仍限幅在 ±95mm（留 1mm 机械余量），但 16bit 映射必须按物理 96mm，
    # 否则 -95mm 指令会被映射到 0 步 = -96mm，产生 1mm 满量程误差。
    PHYS_HALF_STROKE_MM = 96.0

    def displacement_to_flypt_16bit(self, disp_mm, range_mm=None):
        """
        物理位移(mm) → FlyPT 16-bit 输出值（按物理半行程 96mm 映射）。
        16-bit: 0 → -96mm, 32768 → 0mm, 65535 → +96mm。
        range_mm 参数保留以兼容旧调用，映射统一用 PHYS_HALF_STROKE_MM。
        """
        r = self.PHYS_HALF_STROKE_MM
        val = (disp_mm + r) / (2.0 * r) * 65535.0
        return int(round(np.clip(val, 0, 65535)))

    def flypt_16bit_to_displacement(self, val_16bit, range_mm=None):
        """FlyPT 16-bit 值 → 物理位移(mm)（按物理半行程 96mm）"""
        r = self.PHYS_HALF_STROKE_MM
        return (val_16bit / 65535.0) * (2.0 * r) - r

    def parse_serial(self, data_bytes):
        """
        解析串口数据: <255><255> + 4×16bit (每板4轴)
        data_bytes: bytes, 包含起始标记和轴数据
        Returns: 目标位置数组 (Arduino 内部单位)
        """
        if len(data_bytes) < 2 + self.num_motors * 2:
            return None

        # 检查起始标记
        if data_bytes[0] != SERIAL_PROTOCOL['start_mark']:
            return None
        if data_bytes[1] != SERIAL_PROTOCOL['start_mark']:
            return None

        targets = []
        idx = 2
        for i in range(self.num_motors):
            val16 = (data_bytes[idx] << 8) | data_bytes[idx + 1]
            targets.append(self.map_to_range(val16))
            idx += 2

        return np.array(targets, dtype=np.int32)

    def build_serial_frame(self, displacements_mm, range_mm=95.0):
        """
        构建串口帧: <255><255> + 4×16bit
        displacements_mm: 物理位移数组 (mm)
        Returns: bytes
        """
        frame = bytearray()
        frame.append(SERIAL_PROTOCOL['start_mark'])
        frame.append(SERIAL_PROTOCOL['start_mark'])
        for disp in displacements_mm:
            val16 = self.displacement_to_flypt_16bit(disp, range_mm)
            frame.append((val16 >> 8) & 0xFF)
            frame.append(val16 & 0xFF)
        return bytes(frame)

    def move_motor(self, motor_idx, target, dt=0.001):
        """
        模拟单个电机的运动
        复现 .ino 中 move0-move3 的逻辑
        """
        if target == self.target_pos[motor_idx]:
            return self.current_pos[motor_idx]

        # 如果正在运行, 先停止
        if self.is_running[motor_idx]:
            self.stop_signal[motor_idx] = True
            while self.is_running[motor_idx]:
                # 快速推进到当前位置
                if self.stop_signal[motor_idx]:
                    self.is_running[motor_idx] = False
                    self.stop_signal[motor_idx] = False
                    break
            self.stop_signal[motor_idx] = False

        current = self.current_pos[motor_idx]
        if target != current:
            if target > current:
                self.inc_pos[motor_idx] = 1
            else:
                self.inc_pos[motor_idx] = -1
            self.target_pos[motor_idx] = target
            self.is_running[motor_idx] = True
            # PWM 周期 = samplePeriod / |target - current|
            diff = abs(target - current)
            period = self.sample_period / max(diff, 1)
            period = max(self.min_period, min(self.max_period, period))
            self.pwm_periods[motor_idx] = period

        return self.current_pos[motor_idx]

    def step(self, targets, dt=0.001):
        """
        仿真一个时间步
        targets: 目标位置数组 (Arduino 内部单位)
        """
        # 更新目标
        for i in range(self.num_motors):
            if targets[i] != self.target_pos[i]:
                self.move_motor(i, targets[i])

        # 推进电机位置 (模拟 ISR 计数)
        for i in range(self.num_motors):
            if self.is_running[i]:
                self.current_pos[i] += self.inc_pos[i]
                if (self.inc_pos[i] > 0 and self.current_pos[i] >= self.target_pos[i]) or \
                   (self.inc_pos[i] < 0 and self.current_pos[i] <= self.target_pos[i]):
                    self.is_running[i] = False

    def get_physical_positions(self):
        """获取所有电机的物理位置 (mm)"""
        return [self.physical_displacement(self.current_pos[i])
                for i in range(self.num_motors)]

    def get_pwm_frequencies(self):
        """获取PWM频率 (Hz)"""
        freqs = []
        for i in range(self.num_motors):
            if self.pwm_periods[i] > 0:
                freqs.append(1000000.0 / self.pwm_periods[i])
            else:
                freqs.append(0)
        return freqs


class DualBoardController:
    """双板控制器仿真: 两块 Arduino 各控3-4轴"""

    def __init__(self, config=ARDUINO):
        self.cfg = config
        self.board1 = ArduinoController(
            0, config['board1_axes'][:4]
        )
        self.board2 = ArduinoController(
            1, config['board2_axes'][:4]
        )
        self.range_mm = 95.0

    def dispatch_displacements(self, displacements_mm):
        """
        将6个轴的物理位移分发到两块板
        displacements_mm: 长度6的数组
        """
        # 板1: 轴1-4；板2: 轴5-6（实物 2 号板只接了轴5/6，
        # Mover 配置里帧2后两通道虽发 Axis4a/Axis3a 但板2电机2/3未接电缸，
        # 为避免重复驱动同一电缸，这里后两通道填 0）。
        b1_disp = displacements_mm[:4]
        b2_disp = np.zeros(4)
        b2_disp[0] = displacements_mm[4]  # Axis5a
        b2_disp[1] = displacements_mm[5]  # Axis6a

        return b1_disp, b2_disp

    def step(self, displacements_mm, dt=0.001):
        """仿真一个时间步"""
        b1_disp, b2_disp = self.dispatch_displacements(displacements_mm)

        # 转换为 Arduino 位置单位
        b1_targets = np.array([
            self.board1.displacement_to_arduino(d) for d in b1_disp
        ], dtype=np.int32)
        b2_targets = np.array([
            self.board2.displacement_to_arduino(d) for d in b2_disp
        ], dtype=np.int32)

        self.board1.step(b1_targets, dt)
        self.board2.step(b2_targets, dt)

    def get_all_displacements(self):
        """获取6轴物理位移 (mm)"""
        b1 = self.board1.get_physical_positions()
        b2 = self.board2.get_physical_positions()
        # 板2只控制轴5/6, 轴3/4由板1控制
        return np.array([b1[0], b1[1], b1[2], b1[3], b2[0], b2[1]])

    def get_serial_frames(self, displacements_mm):
        """构建两块板的串口帧"""
        b1_disp, b2_disp = self.dispatch_displacements(displacements_mm)
        frame1 = self.board1.build_serial_frame(b1_disp, self.range_mm)
        frame2 = self.board2.build_serial_frame(b2_disp, self.range_mm)
        return frame1, frame2
