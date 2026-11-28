"""实测双 Mega 硬件映射（唯一权威，避免各工具各写一份）。

坐标约定：+Y=后方（见 G_chair_mujoco/params.py 与 README），-Y=前方。
模型三对铰点：
  cyl0/cyl1 = 后方一对（右后 / 左后）
  cyl2/cyl3 = 左侧一对（左中 / 左前）
  cyl4/cyl5 = 右侧一对（右前 / 右中）

板1 (ttyUSB1): ch0右前 ch1右中 ch2右后 ch3左后
板2 (ttyUSB2): ch0左中 ch1左前
"""

CYL_NAMES = {
    0: '右后', 1: '左后', 2: '左中',
    3: '左前', 4: '右前', 5: '右中',
}

# sim 缸号 -> (板序号 0/1, 通道 0..3)
CYL_TO_BOARD = {
    0: (0, 2),   # 右后 -> 板1 ch2
    1: (0, 3),   # 左后 -> 板1 ch3
    2: (1, 0),   # 左中 -> 板2 ch0
    3: (1, 1),   # 左前 -> 板2 ch1
    4: (0, 0),   # 右前 -> 板1 ch0
    5: (0, 1),   # 右中 -> 板1 ch1
}

# 反向查表：(板,通道) -> 缸号
BOARD_TO_CYL = {(b, c): cyl for cyl, (b, c) in CYL_TO_BOARD.items()}
