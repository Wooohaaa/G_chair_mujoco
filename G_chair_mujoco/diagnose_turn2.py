#!/usr/bin/env python3
"""严格诊断：跑完整链路(遥测->姿态->滤波->IK->MuJoCo)，按横向加速度区分左右。"""
import os, sys, math
import numpy as np
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', 'G_chair_sim'))

from config import TELEMETRY_CHANNELS, FILTERS
from pose_from_motion import PoseFromMotion
from filters import FilterChain
from sim import GChairSim

a = np.loadtxt(os.path.join(HERE, 'output', 'raw_telemetry.csv'),
               delimiter=',', skiprows=1)
idx = {k: i for i, k in enumerate(TELEMETRY_CHANNELS)}
ax = a[:, idx['accel_x']]   # 横向加速度: 左转/右转的直接指标
rx = a[:, idx['rot_x']]
ry = a[:, idx['rot_y']]

m = PoseFromMotion()
fc = FilterChain(FILTERS)
sim = GChairSim()

cmd, act = [], []
for row in a:
    tele = {k: row[i] for k, i in idx.items()}
    p = m.apply_filters(m.compute(tele, 0.02), fc, 0.02)
    pose = np.array([p['sway']/1000, p['surge']/1000, p['heave']/1000,
                     math.radians(p['roll']), math.radians(p['pitch']),
                     math.radians(p['yaw'])])
    sim.command_pose(pose, 0.02)
    sim.step()
    cmd.append(pose)
    act.append(sim.platform_pose())
cmd = np.array(cmd)
act = np.array(act)

# 用 accel_x 符号区分左右（|ax|>0.15 才算转弯）
turn = np.abs(ax) > 0.15
L = turn & (ax < 0)   # 左转(假设负=左)
R = turn & (ax > 0)   # 右转
print('转弯帧 L=%d R=%d (按accel_x符号)' % (L.sum(), R.sum()))

names = ['x(sway)', 'y(surge)', 'z(heave)', 'roll', 'pitch', 'yaw']
print('\n=== 指令 cmd 左右对比 ===')
for i, n in enumerate(names):
    scale = 1000 if i < 3 else 57.3
    lm, rm = cmd[L, i].mean()*scale, cmd[R, i].mean()*scale
    print('  %-10s 左 %+8.3f  右 %+8.3f  %s' %
          (n, lm, rm, '反号OK' if lm*rm < 0 else '*** 同向 ***'))

print('\n=== 实际 act(座椅) 左右对比 ===')
for i, n in enumerate(names):
    scale = 1000 if i < 3 else 57.3
    lm, rm = act[L, i].mean()*scale, act[R, i].mean()*scale
    print('  %-10s 左 %+8.3f  右 %+8.3f  %s' %
          (n, lm, rm, '反号OK' if lm*rm < 0 else '*** 同向 ***'))

# 直接看 roll 的构成：rot_x 项 vs accel_x 耦合项
print('\n=== roll 两项符号关系 ===')
roll_rotx = rx * 50 * 0.5
roll_couple = ax * 100 * 0.3 * 0.5
print('  corr(rot_x项, accel_x耦合项) = %+.3f' % np.corrcoef(roll_rotx, roll_couple)[0,1])
print('  两项反号的帧占比: %.1f%%' % (100*np.mean(np.sign(roll_rotx)!=np.sign(roll_couple))))
