"""
telemetry.py — 合成游戏遥测数据生成器
生成匹配 DCS_To_FlyPT_Mover.lua 25通道格式的模拟数据。
模拟驾驶场景: 直线/急刹/转弯/颠簸/爬坡
"""

import numpy as np
from config import SIM, SCENARIOS


class TelemetryGenerator:
    """生成模拟驾驶遥测数据"""

    def __init__(self, dt=SIM['dt']):
        self.dt = dt
        self.t = 0.0
        # 车辆状态
        self.speed = 0.0        # 纵向速度 (m/s)
        self.lateral_speed = 0.0
        self.vertical_speed = 0.0
        self.heading = 0.0       # 航向角
        self.pitch = 0.0
        self.roll = 0.0
        # 场景当前索引
        self.scenario_idx = 0

    def _get_scenario(self):
        for i, s in enumerate(SCENARIOS):
            if s['start'] <= self.t < s['end']:
                return i, s
        return len(SCENARIOS) - 1, SCENARIOS[-1]

    def _generate_scenario(self, t_rel, scenario_name):
        """根据场景名称生成遥测数据"""
        accel_x = 0.0   # 横向 (G)
        accel_y = 0.0   # 垂直 (G)
        accel_z = 0.0   # 纵向 (G)
        rot_x = 0.0     # 滚转角速度
        rot_y = 0.0     # 偏航角速度
        rot_z = 0.0     # 俯仰角速度

        if scenario_name == '直线起步':
            # 0-12s: 从静止加速到 60km/h
            accel_z = min(t_rel / 4.0, 1.0) * 1.2  # 纵向加速 1.2G
            if t_rel > 2:
                self.speed = min(t_rel * 2.5, 20.0)  # 0-20 m/s

        elif scenario_name == '匀速行驶':
            # 12-24s: 匀速
            self.speed = 20.0
            # 轻微路面颠簸
            accel_y = 0.05 * np.sin(t_rel * 3.0)
            accel_x = 0.03 * np.sin(t_rel * 2.0)

        elif scenario_name == '急刹车':
            # 24-32s: 急刹车
            decel = 1.5 * min(t_rel / 2.0, 1.0)  # 刹车减速度 1.5G
            accel_z = -decel
            if t_rel < 3:
                rot_z = 0.1 * np.sin(t_rel * 2.0)  # 刹车点头
                self.pitch = -0.06 * min(t_rel / 1.0, 1.0)  # 俯仰
            self.speed = max(self.speed - decel * self.dt * 10, 0)

        elif scenario_name == '右转弯':
            # 32-44s: 右转弯
            steer = 0.8 * min(t_rel / 2.0, 1.0)  # 转向
            self.speed = 12.0
            # 横向加速度 (离心力)
            accel_x = steer * 1.2
            # 滚转 (车身侧倾)
            self.roll = -steer * 0.08
            rot_x = -steer * 0.15
            rot_y = steer * 0.2  # 偏航角速度
            # 转向时纵向减速
            accel_z = -0.2 * steer
            self.heading += rot_y * self.dt

        elif scenario_name == '左转弯':
            # 44-56s: 左转弯
            steer = -0.8 * min(t_rel / 2.0, 1.0)
            self.speed = 12.0
            accel_x = steer * 1.2
            self.roll = -steer * 0.08
            rot_x = -steer * 0.15
            rot_y = steer * 0.2
            accel_z = -0.2 * abs(steer)
            self.heading += rot_y * self.dt

        elif scenario_name == '颠簸路面':
            # 56-60s: 颠簸
            self.speed = 15.0
            accel_y = 0.3 * np.sin(t_rel * 8.0) * min(t_rel, 1.0)
            accel_x = 0.1 * np.sin(t_rel * 6.0)
            rot_z = 0.05 * np.sin(t_rel * 7.0)
            self.pitch = 0.03 * np.sin(t_rel * 5.0)

        return accel_x, accel_y, accel_z, rot_x, rot_y, rot_z

    def generate(self):
        """生成一帧遥测数据 (25通道, 匹配 DCS Lua 脚本)"""
        self.t += self.dt
        idx, scenario = self._get_scenario()
        t_rel = self.t - scenario['start']

        accel_x, accel_y, accel_z, rot_x, rot_y, rot_z = \
            self._generate_scenario(t_rel, scenario['name'])

        # 计算速度分量 (用于飞行动效)
        speed_x = self.speed * np.sin(self.heading)
        speed_z = self.speed * np.cos(self.heading)
        speed_y = 0.0

        telemetry = {
            # 0-2: 三轴加速度 (G)
            'accel_x': accel_x,
            'accel_z': accel_z,
            'accel_y': accel_y,
            # 3-5: 三轴速度 (m/s)
            'speed_x': speed_x,
            'speed_z': speed_z,
            'speed_y': speed_y,
            # 6-8: 三轴角速度 (rad/s)
            'rot_y': rot_y,
            'rot_x': rot_x,
            'rot_z': rot_z,
            # 9-11: 姿态角 (rad)
            'heading': self.heading,
            'bank': self.roll,
            'pitch': self.pitch,
            # 12: 空速 (m/s)
            'airspeed': self.speed,
            # 13-16: 起落架
            'gear_front': 1.0,
            'gear_turn': 0.0,
            'gear_left': 1.0,
            'gear_right': 1.0,
            # 17: 高度 (m)
            'altitude': 0.0,
            # 18-21: 襟翼/起落架/减速板/座舱盖
            'flaps': 0.0,
            'gear': 1.0,
            'speedbrakes': 0.0,
            'canopy': 0.0,
            # 22: 失速告警
            'stall': 0.0,
            # 23: 攻角
            'aoa': accel_z * 0.1,
            # 24: 时间
            'time': self.t,
        }

        # 记录当前场景名
        telemetry['_scenario'] = scenario['name']
        telemetry['_scenario_idx'] = idx

        return telemetry

    def reset(self):
        self.t = 0.0
        self.speed = 0.0
        self.lateral_speed = 0.0
        self.vertical_speed = 0.0
        self.heading = 0.0
        self.pitch = 0.0
        self.roll = 0.0
