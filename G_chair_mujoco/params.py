"""
params.py — G_chair 六轴平台几何/执行器参数

机构: 6-6 Stewart 平台, 底座与平台各 6 个铰点, 分 3 组"对"布置
      (与实物 0.9m/0.25m 交替边长的六边形底座、500mm 座椅固定板对应)。

关键尺寸来源:
  - 电缸总行程 xingcheng = 192 mm (servoController.ino), 相对中位 ±95 mm
  - 中位腿长 NUDMiddle = 740 mm (斯图尔特6轴-动态2.Mover)
  - 底座 ~1.1 m 六边形, 平台 ~0.6 m (第三步-硬件篇 图纸/成品照片)

注意: 旧仿真 (G_chair_sim) 把 NUDRigL1=670 当作"直径"得到 r_base=335mm,
      且上下铰点角度完全重合, 该几何在数学上是一个奇异机构 (偏航/升降零刚度,
      受重力会"塌陷"到交叉腿分支)。本模型改用非奇异的三对铰点布置, 详见 README。
"""

from dataclasses import dataclass
import numpy as np


@dataclass
class RigParams:
    base_radius: float = 0.540        # 底座铰点分布半径 (m)
    plat_radius: float = 0.300        # 平台铰点分布半径 (m)
    base_half_angle_deg: float = 12.0 # 底座每组"对"铰点半张角 (度)
    plat_half_angle_deg: float = 16.0 # 平台每组"对"铰点半张角 (度)
    # 实物铰点布置（按成品照）：底座后方正中间(+Y=90°)有一组对铰点，
    # 另两组在左右对称的 210° / 330°。平台相对底座错位 60° → 150/270/30。
    # 2026-11 从旧值 (0/120/240)、(60/180/300) 整体旋转 90°，使后方中间有铰点。
    base_pair_centers_deg: tuple = (90.0, 210.0, 330.0)
    plat_pair_centers_deg: tuple = (150.0, 270.0, 30.0)

    leg_middle: float = 0.740         # 中位腿长 (m)
    stroke: float = 0.192             # 电缸总行程 (m)
    range: float = 0.095              # 相对中位的半行程 (m)

    platform_mass: float = 45.0       # 平台 + 座椅 + 人体等效质量 (kg)
    slider_mass: float = 1.2
    root_mass: float = 0.6

    kp: float = 80000.0
    kv: float = 2000.0
    force_max: float = 4000.0

    def base_anchors(self):
        """底座 6 铰点 (6x3, m), 顺序与电缸 Axis1..6 对应。"""
        ang = []
        for c in self.base_pair_centers_deg:
            ang += [c - self.base_half_angle_deg, c + self.base_half_angle_deg]
        ang = np.radians(ang)
        return np.c_[self.base_radius * np.cos(ang),
                     self.base_radius * np.sin(ang),
                     np.zeros(6)]

    def platform_anchors(self):
        """
        平台 6 铰点 (6x3, m), 顺序已与支腿配对:
        第 i 条腿连接 base_i 与 plat_i。配对为相邻一组 (循环错位 60°),
        该配对是唯一同时满足"无交叉 + 6 腿等长 + 良态雅可比"的布局。
        """
        ang = []
        for c in self.plat_pair_centers_deg:
            ang += [c - self.plat_half_angle_deg, c + self.plat_half_angle_deg]
        ang = np.radians(ang)
        raw = np.c_[self.plat_radius * np.cos(ang),
                    self.plat_radius * np.sin(ang),
                    np.zeros(6)]
        # 配对: leg i -> raw[(i-1) mod 6], 等价于平台相对底座整体旋转 60°
        return np.roll(raw, 1, axis=0)

    def neutral_height(self, leg_middle=None):
        """使 6 条腿中位长度均等于 leg_middle 的平台高度 (m)。"""
        lm = self.leg_middle if leg_middle is None else leg_middle
        B = self.base_anchors()
        P = self.platform_anchors()
        horiz = np.linalg.norm((B - P)[:, :2], axis=1)
        if np.ptp(horiz) > 1e-9:
            raise ValueError('铰点布置无法做到 6 腿等长 (水平距离不一致)')
        h2 = lm * lm - horiz[0] * horiz[0]
        if h2 <= 0:
            raise ValueError('腿长过短, 无法形成该高度')
        return float(np.sqrt(h2))


PARAMS = RigParams()
