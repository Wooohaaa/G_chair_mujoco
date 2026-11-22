#!/usr/bin/env python3
"""
speed_limiter.py — 自适应缸速限制器（保证几何一致性 + 只削尖峰、正常段零延迟）

原理：
  直接对 6 个缸独立限速会破坏平台几何（FK->IK 残差达 100mm）。
  正确做法：在 IK 之前，对 6DOF 目标位姿做自适应回拉——
  若"目标位姿对应的缸速"超过上限，就迭代地把目标位姿朝上一帧位姿插值，
  直到预测缸速 <= 上限。这样 6 缸仍由同一个位姿经 IK 算出，几何严格一致。

  正常段缸速低于上限 -> 不回拉 -> 零延迟。
  尖峰段 -> 只回拉到刚好达标 -> 平台动作被削顶但不拖慢后续正常段。
"""
import numpy as np


class AdaptiveSpeedLimiter:
    def __init__(self, rig, max_speed_mm_s=110.0, dt=0.02, iters=8):
        self.rig = rig
        self.max_speed = max_speed_mm_s
        self.dt = dt
        self.iters = iters
        self.prev_pose = None
        self.prev_disp = None

    def _disp(self, pose):
        return self.rig.inverse_kinematics(pose[0]*1000, pose[1]*1000, pose[2]*1000,
                                           pose[3], pose[4], pose[5])

    def limit(self, pose):
        pose = np.asarray(pose, dtype=float).copy()
        target_disp = self._disp(pose)
        if self.prev_pose is None:
            self.prev_pose = pose.copy()
            self.prev_disp = np.clip(target_disp, -95, 95).copy()
            return pose

        cap = self.max_speed * self.dt
        out = pose.copy()
        # 若超限，二分回拉插值系数 alpha (0=上一帧, 1=目标)
        lo, hi = 0.0, 1.0
        for _ in range(self.iters):
            mid = 0.5 * (lo + hi)
            cand = self.prev_pose + mid * (pose - self.prev_pose)
            d = np.clip(self._disp(cand), -95, 95)
            speed = np.max(np.abs(d - self.prev_disp)) / self.dt
            if speed > self.max_speed:
                hi = mid
            else:
                lo = mid
        out = self.prev_pose + lo * (pose - self.prev_pose)
        self.prev_pose = out.copy()
        self.prev_disp = np.clip(self._disp(out), -95, 95).copy()
        return out

    def reset(self):
        self.prev_pose = None
        self.prev_disp = None
