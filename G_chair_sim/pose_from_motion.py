"""
pose_from_motion.py — 从运动数据映射到6自由度平台姿态
复现 Mover 文件中 POSE :: FROM MOTION 的映射逻辑。

输入: 游戏遥测数据 (加速度、角速度等)
输出: 6 DOF 平台姿态 (sway, surge, heave, yaw, roll, pitch)
"""

import numpy as np
from config import POSE_FROM_MOTION


class PoseFromMotion:
    def __init__(self, config=POSE_FROM_MOTION):
        self.cfg = config

    def compute(self, telemetry, dt=0.02):
        """
        根据遥测数据计算6自由度姿态

        Args:
            telemetry: dict, 包含遥测通道
                - accel_x: 横向加速度 (G)
                - accel_y: 垂直加速度 (G)  (FlyPT用Z为垂直, Y为前后, 此处已按FlyPT坐标)
                - accel_z: 纵向加速度 (G)
                - rot_x: 滚转角速度 (rad/s)
                - rot_y: 偏航角速度 (rad/s)
                - rot_z: 俯仰角速度 (rad/s)
            dt: 时间步长

        Returns:
            pose: dict, 6个自由度的值
                sway, surge, heave (mm)
                yaw, roll, pitch (度)
        """
        pose = {}

        for dof_name, params in self.cfg.items():
            if dof_name.endswith('_couple'):
                continue

            input_key = params['input']
            input_val = telemetry.get(input_key, 0.0)

            # 输入增益 → 输出增益 → 限幅
            raw = input_val * params['in_gain'] * params['out_gain']
            # 限幅到 range
            raw = np.clip(raw, -params['range'], params['range'])
            pose[dof_name] = raw

        # 交叉耦合: roll 受 sway 影响, pitch 受 surge 影响
        sway_input = telemetry.get('accel_x', 0.0) * self.cfg['sway']['in_gain']
        surge_input = telemetry.get('accel_z', 0.0) * self.cfg['surge']['in_gain']

        pose['roll'] += sway_input * self.cfg['roll_sway_couple'] * self.cfg['roll']['out_gain']
        pose['pitch'] += surge_input * self.cfg['pitch_surge_couple'] * self.cfg['pitch']['out_gain']

        # 再次限幅
        for dof_name, params in self.cfg.items():
            if dof_name.endswith('_couple'):
                continue
            pose[dof_name] = np.clip(
                pose[dof_name],
                -params['range'],
                params['range']
            )

        # 角度输出已经是度 (raw值即为度), 不需要再转换
        # 保留原始值 (raw已经在range限幅内, range单位为度)
        return pose

    def apply_filters(self, pose, filter_chain, dt=0.02):
        """
        对姿态施加滤波:
        - 线性DOF (sway, surge, heave): 经悬挂滤波
        - 旋转DOF (yaw, roll, pitch): 经EDYNLP滤波
        """
        filtered = {}
        # 线性DOF: 悬挂滤波 LOGISTIC(EMALP(VALUE;30);30;0.3)
        for dof in ['sway', 'surge', 'heave']:
            filtered[dof] = filter_chain.filter_suspension(pose[dof], dt)

        # 旋转DOF: EDYNLP(VALUE;0;20)
        for dof in ['yaw', 'roll', 'pitch']:
            filtered[dof] = filter_chain.filter_rotation(pose[dof], dt)

        return filtered
