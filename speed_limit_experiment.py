#!/usr/bin/env python3
"""实验：标定位姿变化率限制参数，使缸速 <= 主控能力(128mm/s)，同时保留体感。"""
import os, sys, math
import numpy as np
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from motion_pipeline import MotionPipeline
from G_chair_sim.config import TELEMETRY_CHANNELS

a = np.loadtxt(os.path.join(HERE, 'G_chair_mujoco', 'output', 'raw_telemetry.csv'),
               delimiter=',', skiprows=1)
i = {k: j for j, k in enumerate(TELEMETRY_CHANNELS)}
keys = ['accel_x', 'accel_z', 'accel_y', 'rot_y', 'rot_x', 'rot_z']

# 主控能力
STEP_MM = 192.0 / 38400.0            # 0.005 mm/step
MAX_HZ = 1e6 / 39.0                  # 25641 Hz
CAP = STEP_MM * MAX_HZ               # 128 mm/s
print('主控最大缸速 %.0f mm/s' % CAP)

def rate_limit(pose, prev, max_lin_mm_s, max_ang_deg_s, dt):
    """对6DOF位姿做变化率限制(线性mm/s, 角度deg/s)。"""
    p = pose.copy()
    # 线性自由度 (m)
    for j in range(3):
        d = p[j] - prev[j]
        cap = max_lin_mm_s / 1000.0 * dt
        p[j] = prev[j] + np.clip(d, -cap, cap)
    # 角度自由度 (rad)
    for j in range(3, 6):
        d = p[j] - prev[j]
        cap = math.radians(max_ang_deg_s) * dt
        p[j] = prev[j] + np.clip(d, -cap, cap)
    return p

def run(max_lin, max_ang):
    mp = MotionPipeline()
    prev = np.zeros(6)
    speed = []
    poses = []
    for row in a:
        tele = {k: row[j] for k, j in i.items()}
        p = mp.pose(tele, 0.02)
        if max_lin > 0 or max_ang > 0:
            p = rate_limit(p, prev, max_lin, max_ang, 0.02)
        prev = p.copy()
        poses.append(p)
        disp = mp.rig.inverse_kinematics(p[0]*1000, p[1]*1000, p[2]*1000,
                                         p[3], p[4], p[5])
        disp = np.clip(disp, -95, 95)
        if len(poses) > 1:
            prev_disp = mp.rig.inverse_kinematics(poses[-2][0]*1000, poses[-2][1]*1000,
                poses[-2][2]*1000, poses[-2][3], poses[-2][4], poses[-2][5])
            speed.append(np.max(np.abs(disp - np.clip(prev_disp,-95,95))) / 0.02)
    return np.array(speed)

for lin, ang, tag in [
    (0, 0, '无限速(当前)'),
    (60, 60, '60mm/s 60deg/s'),
    (40, 50, '40mm/s 50deg/s'),
    (30, 40, '30mm/s 40deg/s'),
]:
    sp = run(lin, ang)
    print('\n%-18s MAX %5.0f mm/s  P99 %5.0f  P95 %5.0f  超128mm/s帧 %d/%d' %
          (tag, sp.max(), np.percentile(sp, 99), np.percentile(sp, 95),
           (sp > CAP).sum(), len(sp)))
