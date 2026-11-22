#!/usr/bin/env python3
"""
motion_pipeline.py — 仿真与实机共用的遥测→6轴缸位移 数据链（上位机核心算法）

这是"修好的映射算法"的唯一权威实现，sim_udp.py(仿真/MuJoCo) 和
serial_bridge.py(实机/串口) 都调用它，保证仿真所见 = 实机所得。

链路:
  遥测 dict → [低通] → [加速度驱动映射 / 原始FlyPT映射] → [洗出] → 6DOF位姿
           → IK(实机几何) → 6轴缸位移(mm) → 限幅 → 串口帧

默认用"加速度驱动映射"(motion_map.accel_driven_pose)，已验证左右转对称。
IK 用 G_chair_sim 的 StewartPlatform（实机几何），不是 MuJoCo 的 rig。
"""
import math
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
MJ_DIR = os.path.join(HERE, 'G_chair_mujoco')
SIM_DIR = os.path.join(HERE, 'G_chair_sim')
for p in (MJ_DIR, SIM_DIR):
    if p not in sys.path:
        sys.path.insert(0, p)

from config import RIG, TELEMETRY_CHANNELS, SIM
from kinematics import StewartPlatform
from motion_map import accel_driven_pose, Washout6D
from pose_from_motion import PoseFromMotion
from filters import FilterChain
from config import FILTERS
from speed_limiter import AdaptiveSpeedLimiter

# 主控硬件能力：minPeriod=39us -> 25641Hz -> 每步(192mm/38400)=0.005mm -> 128mm/s
# 默认限速 110mm/s，留 14% 余量给加减速和脉冲实现误差。
DEFAULT_MAX_SPEED = 110.0


class MotionPipeline:
    """可复用的遥测→缸位移管线。"""

    def __init__(self, accel_model=True, lpf_tau=0.15, washout_tau=2.5,
                 max_speed=DEFAULT_MAX_SPEED):
        self.accel_model = accel_model
        self.lpf_tau = lpf_tau
        self.washout_tau = washout_tau
        self.max_speed = max_speed
        self.speed_limiter = None  # 在 rig 建好后初始化

        # 低通状态
        self._lp = {k: 0.0 for k in ('accel_x', 'accel_z', 'accel_y', 'rot_y')}
        self._lp_init = False
        # 洗出 / 原始映射滤波器
        self.wo = Washout6D(washout_tau) if accel_model and washout_tau > 0 else None
        self.mapper = PoseFromMotion()
        self.filt = FilterChain(FILTERS)
        # 实机几何 IK（config.RIG）
        self.rig = StewartPlatform(
            r_base=RIG['r_base'], r_plat=RIG['r_plat'],
            leg_middle=RIG['leg_middle'], height=RIG['height'],
            range_mm=RIG['range'], angle_offset_deg=RIG['angle_offset'])
        # 自适应缸速限制（在 IK 之前回拉位姿，保证几何一致且只削尖峰）
        self.speed_limiter = (AdaptiveSpeedLimiter(self.rig, max_speed_mm_s=max_speed,
                                                   dt=SIM['dt'])
                              if max_speed > 0 else None)

    def _lowpass(self, tele, dt):
        if self.lpf_tau <= 0:
            return tele
        a = dt / (self.lpf_tau + dt)
        out = dict(tele)
        for k in self._lp:
            x = float(tele.get(k, 0.0))
            if not self._lp_init:
                self._lp[k] = x
            else:
                self._lp[k] = (1 - a) * self._lp[k] + a * x
            out[k] = self._lp[k]
        self._lp_init = True
        return out

    def pose(self, tele, dt=SIM['dt']):
        """遥测 dict → 6DOF 位姿 [x,y,z,roll,pitch,yaw] (m, rad)。

        注意：本方法会推进 speed_limiter / washout / 低通的状态。
        pose() 与 displacements_mm() 不要在同一帧混用（后者内部也调 pose），
        否则限速器状态会被推进两次。仿真端只用 pose()，实机桥接只用 displacements_mm()。
        """
        tele = self._lowpass(tele, dt)
        if self.accel_model:
            p = accel_driven_pose(tele, dt, self.wo)
        else:
            pf = self.mapper.apply_filters(self.mapper.compute(tele, dt), self.filt, dt)
            p = np.array([pf['sway']/1000.0, pf['surge']/1000.0, pf['heave']/1000.0,
                          math.radians(pf['roll']), math.radians(pf['pitch']),
                          math.radians(pf['yaw'])])
        # 自适应限速：只削超主控能力的尖峰，正常段零延迟，几何一致
        if self.speed_limiter is not None:
            p = self.speed_limiter.limit(p)
        return p

    def displacements_mm(self, tele, dt=SIM['dt']):
        """遥测 dict → 6轴缸位移(mm)，已限幅到行程。"""
        pose = self.pose(tele, dt)
        x, y, z, roll, pitch, yaw = pose
        disp = self.rig.inverse_kinematics(x*1000.0, y*1000.0, z*1000.0,
                                           roll, pitch, yaw)
        return np.clip(disp, -RIG['range'], RIG['range'])

    def reset(self):
        """断流后重置状态（低通/洗出/限速器回到初始），避免残留偏置。"""
        self._lp = {k: 0.0 for k in self._lp}
        self._lp_init = False
        if self.wo is not None:
            self.wo = Washout6D(self.washout_tau)
        if self.speed_limiter is not None:
            self.speed_limiter.reset()
