"""
config.py — 全局配置参数
所有参数取自 G_chair 工程中的实际文件：
  - 斯图尔特6轴-动态2.Mover  (RIG / POSE / OUTPUT 配置)
  - servoController.ino       (Arduino 控制器参数)
  - DCS_To_FlyPT_Mover.lua    (遥测数据通道定义)
  - 调教翻译.txt               (滤波函数定义)
"""

# ============================ 平台几何参数 ============================
# 源自 Mover 文件中的 RIG 配置:
#   NUDRigL1=670, L2=125, L3=125, L4=950, Range=95, Middle=740
# L1 解释为底座直径 → 半径 = 335mm
# L2 解释为平台半径 = 125mm
# 中位腿长 = 740mm，由此反推高度 ≈ 679mm

RIG = {
    'r_base': 335.0,       # 底座铰点半径 (mm) = L1/2
    'r_plat': 125.0,       # 平台铰点半径 (mm) = L2
    'leg_middle': 740.0,   # 中位腿长 (mm)
    'range': 95.0,         # 电缸行程半幅 (mm)，实际范围 [middle-range, middle+range]
    'angle_offset': 60.0,  # 上下六边形相位差 (度)
    'height': 679.4,       # 中位高度 (mm)，由几何反推
}

# ============================ FlyPT 姿态映射 ============================
# 源自 Mover 文件中 POSE :: FROM MOTION 配置
POSE_FROM_MOTION = {
    'sway':   {'in_gain': 100.0, 'out_gain': 1.0,  'range': 50.0,  'input': 'accel_x'},
    'surge':  {'in_gain': 100.0, 'out_gain': -1.0, 'range': 50.0,  'input': 'accel_z'},
    'heave':  {'in_gain': 50.0,  'out_gain': 1.0,  'range': 30.0,  'input': 'accel_y'},
    'yaw':    {'in_gain': 50.0,  'out_gain': 0.2,  'range': 8.0,   'input': 'rot_y'},
    'roll':   {'in_gain': 50.0,  'out_gain': 0.5,  'range': 6.0,   'input': 'rot_x'},
    'pitch':  {'in_gain': 50.0,  'out_gain': 0.5,  'range': 6.0,   'input': 'rot_z'},
    # 交叉耦合
    'roll_sway_couple': 0.3,
    'pitch_surge_couple': 0.4,
}

# ============================ 滤波器配置 ============================
# 源自 Mover 文件中各 AFC 过滤器设置
FILTERS = {
    # 悬挂姿态滤波: LOGISTIC(EMALP(VALUE;30);50;0.5)
    'suspension': {
        'emalp_period': 30.0,    # ms
        'logistic_max': 50.0,
        'logistic_k': 0.5,
    },
    # 电缸输出滤波: LOGISTIC(VALUE;70;0.9)
    'actuator': {
        'logistic_max': 70.0,
        'logistic_k': 0.9,
    },
    # 旋转自由度滤波: EDYNLP(VALUE;0;20)
    'rotation': {
        'edynlp_period': 0.0,
        'edynlp_strength': 20.0,
    },
}

# ============================ Arduino 控制器参数 ============================
# 源自 servoController.ino
ARDUINO = {
    'sample_period': 4000,     # 总运动时间 (us)
    'min_period': 39,          # 最小 PWM 周期
    'max_period': 4000,        # 最大 PWM 周期
    'pulse_length': 5,         # 脉冲宽度
    'baud_rate': 115200,       # 串口波特率
    'xingcheng': 192,          # 行程 (mm)
    'map_divisor': 1.7,        # mapToRange 除数 = 330/192 ≈ 1.718
    'center_pos': 100 * 192,   # 中心位置 = 19200
    'max_pos': 200 * 192,      # 最大位置 = 38400
    'motors_per_board': 4,     # 每块板控制电机数
    'num_boards': 2,           # 板子数量
    'total_motors': 6,         # 总电机数
    # 引脚分配
    'pulse_pins': [12, 2, 7, 45],   # 脉冲输出引脚 (OCR1B, OCR3B, OCR4B, OCR5B)
    'dir_pins': [30, 31, 32, 33],   # 方向输出引脚
    # 板1→轴1-4, 板2→轴5-6(及3-4视配置)
    'board1_axes': ['Axis1a', 'Axis2a', 'Axis3a', 'Axis4a'],
    'board2_axes': ['Axis5a', 'Axis6a', 'Axis3a', 'Axis4a'],
}

# ============================ 串口协议 ============================
SERIAL_PROTOCOL = {
    'start_mark': 255,         # 起始标记
    'start_mark_count': 2,     # 连续两个255
    'bytes_per_axis': 2,       # 每个轴2字节 (16-bit)
    'axes_per_board': 4,       # 每板4轴
}

# ============================ 遥测数据通道 ============================
# 源自 DCS_To_FlyPT_Mover.lua 的25通道输出
TELEMETRY_CHANNELS = [
    'accel_x', 'accel_z', 'accel_y',       # 0-2: 三轴加速度 (G)
    'speed_x', 'speed_z', 'speed_y',       # 3-5: 三轴速度 (m/s)
    'rot_y', 'rot_x', 'rot_z',             # 6-8: 三轴角速度 (rad/s)
    'heading', 'bank', 'pitch',            # 9-11: 姿态角 (rad)
    'airspeed', 'gear_front', 'gear_turn',  # 12-14: 空速, 前起落架, 转向
    'gear_left', 'gear_right',             # 15-16: 左/右起落架
    'altitude', 'flaps', 'gear',           # 17-19: 高度, 襟翼, 起落架
    'speedbrakes', 'canopy', 'stall',      # 20-22: 减速板, 座舱盖, 失速
    'aoa', 'time',                         # 23-24: 攻角, 时间
]

# ============================ 仿真参数 ============================
SIM = {
    'dt': 0.02,           # 仿真步长 (s) = 50Hz
    'duration': 60.0,     # 总时长 (s)
    'fps': 30,            # 动画帧率
    'dt_filter': 0.02,    # 滤波器时间步长 (s)
}

# ============================ 场景定义 ============================
SCENARIOS = [
    {'name': '直线起步', 'start': 0,   'end': 12},
    {'name': '匀速行驶', 'start': 12,  'end': 24},
    {'name': '急刹车',   'start': 24,  'end': 32},
    {'name': '右转弯',   'start': 32,  'end': 44},
    {'name': '左转弯',   'start': 44,  'end': 56},
    {'name': '颠簸路面', 'start': 56,  'end': 60},
]

OUTPUT_DIR = r'C:\Users\chiehyu\Desktop\G_chair\G_chair_sim\output'
