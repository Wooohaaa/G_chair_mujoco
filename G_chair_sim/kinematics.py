"""
kinematics.py — Stewart 六轴平台运动学
实现逆运动学 (IK) 和正运动学 (FK)，以及工作空间验证。

几何模型: 经典 6-6 Stewart 平台
  - 6 个底座铰点在半径 r_base 的圆上，角度 0/60/120/180/240/300°
  - 6 个平台铰点在半径 r_plat 的圆上，相位偏移 60°
  - 中位时腿长 = leg_middle，高度 = height
"""

import numpy as np


class StewartPlatform:
    def __init__(self, r_base, r_plat, leg_middle, height, range_mm,
                 angle_offset_deg=60.0):
        self.r_base = r_base          # 底座铰点半径 (mm)
        self.r_plat = r_plat          # 平台铰点半径 (mm)
        self.leg_middle = leg_middle  # 中位腿长 (mm)
        self.height = height          # 中位高度 (mm)
        self.range = range_mm         # 行程半幅 (mm)

        # 底座铰点 (固定在地面, z=0)
        angles_base = np.arange(6) * np.pi / 3  # 0, 60, 120, ... 度
        self.base_points = np.array([
            [r_base * np.cos(a), r_base * np.sin(a), 0.0]
            for a in angles_base
        ])

        # 平台铰点 (初始在 z=0 平面, 使用时旋转+平移)
        offset = np.radians(angle_offset_deg)
        angles_plat = angles_base + offset
        self.plat_points = np.array([
            [r_plat * np.cos(a), r_plat * np.sin(a), 0.0]
            for a in angles_plat
        ])

        # 轴名 (与 Mover 配置对应)
        self.axis_names = ['Axis1a', 'Axis2a', 'Axis3a',
                           'Axis4a', 'Axis5a', 'Axis6a']

    @staticmethod
    def rotation_matrix(roll, pitch, yaw):
        """ZYX 欧拉角 → 旋转矩阵 (yaw绕Z, pitch绕Y, roll绕X)"""
        cr, sr = np.cos(roll), np.sin(roll)
        cp, sp = np.cos(pitch), np.sin(pitch)
        cy, sy = np.cos(yaw), np.sin(yaw)
        Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
        Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
        Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
        return Rz @ Ry @ Rx

    def inverse_kinematics(self, x, y, z, roll, pitch, yaw):
        """
        逆运动学: 给定平台位姿 → 6 根腿的位移 (mm)
        位移 = 腿长 - 中位腿长
        """
        R = self.rotation_matrix(roll, pitch, yaw)
        T = np.array([x, y, z + self.height])

        leg_lengths = []
        for i in range(6):
            p_world = R @ self.plat_points[i] + T
            leg_len = np.linalg.norm(p_world - self.base_points[i])
            leg_lengths.append(leg_len)

        leg_lengths = np.array(leg_lengths)
        return leg_lengths - self.leg_middle

    def forward_kinematics(self, displacements, max_iter=200, tol=1e-6):
        """
        正运动学: 给定6根腿位移 → 平台位姿
        坐标轮换牛顿法: 逐DOF优化, 避免雅可比病态
        Stewart平台存在多解分支, 需要好的初始猜测
        """
        pose = np.zeros(6)
        # 初始猜测: z = disp * leg_middle/height
        mean_disp = np.mean(displacements)
        pose[2] = mean_disp * self.leg_middle / self.height

        # 旋转DOF初始猜测
        r_eff = self.r_plat * 0.5
        if r_eff > 0:
            right = np.mean([displacements[0], displacements[1], displacements[5]])
            left = np.mean([displacements[2], displacements[3], displacements[4]])
            pose[3] = (right - left) / (2 * r_eff)  # roll
            front = np.mean([displacements[1], displacements[2]])
            back = np.mean([displacements[4], displacements[5]])
            pose[4] = (front - back) / (2 * r_eff)  # pitch
            diag = (displacements[0] + displacements[3]) / 2
            other = np.mean(displacements[[1, 2, 4, 5]])
            pose[5] = (diag - other) / r_eff  # yaw

        for _ in range(max_iter):
            disp_calc = self.inverse_kinematics(*pose)
            error = displacements - disp_calc
            if np.max(np.abs(error)) < tol:
                break

            # 坐标轮换: 逐DOF做1D牛顿步
            for j in range(6):
                eps = 0.01 if j >= 3 else 1.0  # 旋转用rad, 平移用mm
                p1 = pose.copy()
                p1[j] += eps
                d1 = self.inverse_kinematics(*p1)
                J1 = (d1 - disp_calc) / eps  # 1D雅可比 (6x1)

                # 1D最小二乘步: delta = (J1^T * error) / (J1^T * J1)
                JtE = np.dot(J1, error)
                JtJ = np.dot(J1, J1)
                if abs(JtJ) < 1e-12:
                    continue
                delta = JtE / JtJ

                # 限制步长防止发散
                max_step = 5.0 if j >= 3 else 2.0
                delta = np.clip(delta, -max_step, max_step)
                pose[j] += delta

            # 重新计算残差
            disp_calc = self.inverse_kinematics(*pose)

        return pose

    def validate_workspace(self, num_samples=3000):
        """
        工作空间验证: 随机采样位姿，统计可达比例
        """
        accessible = 0
        overflow = 0
        max_disp = 0
        for _ in range(num_samples):
            x = np.random.uniform(-25, 25)
            y = np.random.uniform(-25, 25)
            z = np.random.uniform(-15, 15)
            roll = np.random.uniform(-0.12, 0.12)
            pitch = np.random.uniform(-0.12, 0.12)
            yaw = np.random.uniform(-0.12, 0.12)

            disp = self.inverse_kinematics(x, y, z, roll, pitch, yaw)
            if np.all(np.abs(disp) <= self.range):
                accessible += 1
            else:
                overflow += 1
            max_disp = max(max_disp, np.max(np.abs(disp)))

        return {
            'accessible_ratio': accessible / num_samples,
            'overflow_count': overflow,
            'max_disp_observed': max_disp,
            'range': self.range,
        }

    def get_plat_world(self, x, y, z, roll, pitch, yaw):
        """获取变换后平台铰点的世界坐标 (用于3D可视化)"""
        R = self.rotation_matrix(roll, pitch, yaw)
        T = np.array([x, y, z + self.height])
        return np.array([R @ self.plat_points[i] + T for i in range(6)])

    def get_base_points(self):
        """获取底座铰点坐标 (用于3D可视化)"""
        return self.base_points.copy()
