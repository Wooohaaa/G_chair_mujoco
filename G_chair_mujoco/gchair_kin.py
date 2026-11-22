"""
kinematics.py — Stewart 平台逆/正运动学 (通用铰点布局)

单位: 长度 mm, 角度 rad。位移返回相对中位腿长的差值 (mm)。
"""

import numpy as np


class StewartPlatform:
    def __init__(self, base_points, plat_points, leg_middle, height, range_mm):
        self.base_points = np.asarray(base_points, dtype=float)
        self.plat_points = np.asarray(plat_points, dtype=float)
        self.leg_middle = float(leg_middle)
        self.height = float(height)
        self.range = float(range_mm)
        self.axis_names = ['Axis%da' % (i + 1) for i in range(6)]

    @staticmethod
    def rotation_matrix(roll, pitch, yaw):
        cr, sr = np.cos(roll), np.sin(roll)
        cp, sp = np.cos(pitch), np.sin(pitch)
        cy, sy = np.cos(yaw), np.sin(yaw)
        Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
        Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
        Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
        return Rz @ Ry @ Rx

    def leg_lengths_mm(self, x, y, z, roll, pitch, yaw):
        R = self.rotation_matrix(roll, pitch, yaw)
        T = np.array([x, y, z + self.height])
        Pw = (R @ self.plat_points.T).T + T
        return np.linalg.norm(Pw - self.base_points, axis=1)

    def inverse_kinematics(self, x, y, z, roll, pitch, yaw):
        return self.leg_lengths_mm(x, y, z, roll, pitch, yaw) - self.leg_middle

    def forward_kinematics(self, displacements, seed=None, max_iter=100, tol=1e-7):
        """
        正运动学: 缸位移(mm) -> 平台位姿 [x,y,z,roll,pitch,yaw] (mm, rad)。
        阻尼 Gauss-Newton, 从 seed (默认中位) 出发收敛到最近的装配分支。
        """
        disp = np.asarray(displacements, dtype=float)
        pose = np.zeros(6) if seed is None else np.array(seed, dtype=float)
        eps = np.array([1.0, 1.0, 1.0, 1e-3, 1e-3, 1e-3])
        for _ in range(max_iter):
            f = self.inverse_kinematics(*pose) - disp
            if np.max(np.abs(f)) < tol:
                break
            J = np.zeros((6, 6))
            for j in range(6):
                p1 = pose.copy()
                p1[j] += eps[j]
                J[:, j] = (self.inverse_kinematics(*p1) -
                           self.inverse_kinematics(*pose)) / eps[j]
            # 列尺度天然不同 (mm/mm vs mm/rad), 用 lstsq 直接解并限制步长
            delta = np.linalg.lstsq(J, -f, rcond=1e-8)[0]
            n = np.linalg.norm(delta)
            if n > 0.4:
                delta *= 0.4 / n
            pose += delta
        return pose

    def get_plat_world(self, x, y, z, roll, pitch, yaw):
        R = self.rotation_matrix(roll, pitch, yaw)
        T = np.array([x, y, z + self.height])
        return (R @ self.plat_points.T).T + T


def make_rig(p):
    """由 RigParams (m) 构造运动学对象 (mm)。"""
    return StewartPlatform(
        base_points=p.base_anchors() * 1000.0,
        plat_points=p.platform_anchors() * 1000.0,
        leg_middle=p.leg_middle * 1000.0,
        height=p.neutral_height() * 1000.0,
        range_mm=p.range * 1000.0,
    )
