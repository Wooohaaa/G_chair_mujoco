#!/usr/bin/env python3
"""
motion_pipeline.py — 仿真与实机共用的遥测→6轴缸位移 数据链（上位机核心算法）

这是"修好的映射算法"的唯一权威实现，sim_udp.py(仿真/MuJoCo) 和
serial_bridge.py(实机/串口) 都调用它，保证仿真所见 = 实机所得。

链路:
  遥测 dict → [低通] → [加速度驱动映射 / 原始FlyPT映射] → [洗出] → 6DOF位姿
           → IK(实机几何) → 6轴缸位移(mm) → 限幅 → 串口帧

默认用"加速度驱动映射"(motion_map.accel_driven_pose)，已验证左右转对称。
IK 用 make_rig(PARAMS) —— 与 MuJoCo 同一套实物几何（非奇异布局）；
仿真端和实机端都通过本管线得到缸位移，保证"仿真所见 = 实机所得"。
（修复前曾用 config.RIG 旧几何，同一位姿差 49.5mm，已废弃。）
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

from config import SIM, FILTERS
from params import PARAMS
from gchair_kin import make_rig
from motion_map import accel_driven_pose, Washout6D, Gains
from pose_from_motion import PoseFromMotion
from filters import FilterChain
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
        self.gains = Gains()       # 在线可调增益（默认与原实现一致）
        self.speed_limiter = None  # 在 rig 建好后初始化

        # 低通状态
        self._lp = {k: 0.0 for k in ('accel_x', 'accel_z', 'accel_y', 'rot_y')}
        self._lp_init = False
        # 洗出 / 原始映射滤波器
        self.wo = Washout6D(washout_tau) if accel_model and washout_tau > 0 else None
        self.mapper = PoseFromMotion()
        self.filt = FilterChain(FILTERS)
        # 统一用【实物几何】PARAMS（与 MuJoCo 模型同源，按成品照对齐的非奇异布局）。
        # 修复前这里用的是 config.RIG（旧几何 r_base=335，奇异），与仿真 IK 不一致，
        # 同一位姿两套几何算出的缸位移可差 49.5mm，导致"仿真=实机"不成立。
        self.rig = make_rig(PARAMS)
        # 自适应缸速限制（在 IK 之前回拉位姿，保证几何一致且只削尖峰）
        self.speed_limiter = (AdaptiveSpeedLimiter(self.rig, max_speed_mm_s=max_speed,
                                                   dt=SIM['dt'])
                              if max_speed > 0 else None)

    @staticmethod
    def _sanitize(tele):
        """净化遥测：NaN/Inf/非数值 → 0.0，并钳位到合理范围。

        一帧异常数据若进入低通/洗出状态，会永久污染后续所有输出（实机安全隐患）。
        这里在入口处拦截，保证滤波器状态始终是有限值。
        """
        out = {}
        limits = {'accel_x': 20.0, 'accel_z': 20.0, 'accel_y': 20.0,
                  'rot_x': 20.0, 'rot_y': 20.0, 'rot_z': 20.0}
        for k, lim in limits.items():
            try:
                x = float(tele.get(k, 0.0))
            except (TypeError, ValueError):
                x = 0.0
            if not np.isfinite(x):
                x = 0.0
            out[k] = float(np.clip(x, -lim, lim))
        return out

    def _lowpass(self, tele, dt):
        tele = self._sanitize(tele)   # 先净化，避免异常值污染状态
        if self.lpf_tau <= 0:
            return tele
        a = dt / (self.lpf_tau + dt)
        out = dict(tele)
        for k in self._lp:
            x = tele[k]
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
            p = accel_driven_pose(tele, dt, self.wo, self.gains)
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
        pose, disp = self.pose_and_disp(tele, dt)
        return disp

    def pose_and_disp(self, tele, dt=SIM['dt']):
        """一次算出 (6DOF位姿, 6轴缸位移)，只推进一次限速器状态。

        仿真端用位姿做 FK 校验/跟踪显示，实机端只用缸位移；
        两者共用同一套实物几何 PARAMS，保证"仿真=实机"。
        """
        pose = self.pose(tele, dt)   # 内部已推进限速器/洗出/低通
        x, y, z, roll, pitch, yaw = pose
        disp = self.rig.inverse_kinematics(x*1000.0, y*1000.0, z*1000.0,
                                           roll, pitch, yaw)
        disp = np.clip(disp, -PARAMS.range * 1000.0, PARAMS.range * 1000.0)
        return pose, disp

    def set_params(self, lpf=None, washout=None, max_speed=None, gains=None):
        """运行时调整滤波/限速/增益参数（供上位机 Web 界面在线调参）。"""
        # 非有限输入（Web 误输 NaN/Inf 或恶意包）直接忽略，保持当前有效参数，
        # 避免一个坏值永久污染低通/洗出/限速状态。
        def finite(v):
            try:
                x = float(v)
            except (TypeError, ValueError):
                return None
            return x if math.isfinite(x) else None

        if lpf is not None:
            x = finite(lpf)
            if x is not None:
                self.lpf_tau = max(0.0, x)
        if washout is not None:
            x = finite(washout)
            if x is not None:
                self.washout_tau = max(0.0, x)
                if self.accel_model:
                    self.wo = Washout6D(self.washout_tau) if self.washout_tau > 0 else None
        if max_speed is not None:
            x = finite(max_speed)
            if x is not None:
                v = max(0.0, x)
                self.max_speed = v
                if v > 0:
                    if self.speed_limiter is None:
                        from speed_limiter import AdaptiveSpeedLimiter
                        self.speed_limiter = AdaptiveSpeedLimiter(
                            self.rig, max_speed_mm_s=v, dt=SIM['dt'])
                    else:
                        self.speed_limiter.max_speed = v
                else:
                    self.speed_limiter = None
        if gains:
            for k, v in gains.items():
                if hasattr(self.gains, k):
                    x = finite(v)
                    if x is not None:
                        setattr(self.gains, k, x)
        return self.get_params()

    def get_params(self):
        p = {'lpf': self.lpf_tau, 'washout': self.washout_tau,
             'max_speed': self.max_speed, 'accel_model': self.accel_model}
        p.update(self.gains.to_dict())
        return p

    def reset(self):
        """断流后重置状态（低通/洗出/限速器回到初始），避免残留偏置。"""
        self._lp = {k: 0.0 for k in self._lp}
        self._lp_init = False
        self.filt = FilterChain(FILTERS)   # 否则 accel_model=False 时 EMA 状态残留
        if self.accel_model:
            self.wo = Washout6D(self.washout_tau) if self.washout_tau > 0 else None
        if self.speed_limiter is not None:
            self.speed_limiter.reset()
