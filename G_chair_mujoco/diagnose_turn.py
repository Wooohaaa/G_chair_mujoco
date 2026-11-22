#!/usr/bin/env python3
"""诊断：按左转/右转分组，看 accel_x -> sway, rot_x -> roll 的符号关系。"""
import os, sys
import numpy as np
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', 'G_chair_sim'))
from config import TELEMETRY_CHANNELS

a = np.loadtxt(os.path.join(HERE, 'output', 'raw_telemetry.csv'),
               delimiter=',', skiprows=1)
idx = {k: i for i, k in enumerate(TELEMETRY_CHANNELS)}
ax = a[:, idx['accel_x']]   # 横向加速度 -> sway
rx = a[:, idx['rot_x']]     # 滚转角速度 -> roll
ry = a[:, idx['rot_y']]     # 偏航角速度 -> yaw (用来区分左右转)

# 只取明显在转弯的帧（|yaw rate| 大）
turn = np.abs(ry) > 0.15
print('转弯帧数:', turn.sum())

# 左转: rot_y > 0 ; 右转: rot_y < 0  (GTA 偏航方向约定)
left = turn & (ry > 0)
right = turn & (ry < 0)
print('左转帧:', left.sum(), ' 右转帧:', right.sum())

print('\n=== 横向加速度 accel_x（应左右相反）===')
print('  左转: mean %+.3f  (期望与右转反号)' % np.mean(ax[left]))
print('  右转: mean %+.3f' % np.mean(ax[right]))
print('  -> 左右反号? ', 'YES' if np.mean(ax[left]) * np.mean(ax[right]) < 0 else '*** NO ***')

print('\n=== 滚转角速度 rot_x（应左右相反）===')
print('  左转: mean %+.3f' % np.mean(rx[left]))
print('  右转: mean %+.3f' % np.mean(rx[right]))
print('  -> 左右反号? ', 'YES' if np.mean(rx[left]) * np.mean(rx[right]) < 0 else '*** NO ***')

# 模拟 pose_from_motion 的 roll (含交叉耦合)
in_gain_sway = 100.0
roll_gain = 50.0 * 0.5
couple = 0.3
roll_pose = rx * roll_gain + ax * in_gain_sway * couple * 0.5
print('\n=== 映射后的 roll 角(度, 含交叉耦合) ===')
print('  左转: mean %+.3f' % np.mean(roll_pose[left]))
print('  右转: mean %+.3f' % np.mean(roll_pose[right]))
print('  -> 左右反号? ', 'YES' if np.mean(roll_pose[left]) * np.mean(roll_pose[right]) < 0 else '*** NO ***')

# 如果在模拟 NMS 滤波前的 IK，看平台实际会往哪倒
print('\n=== rot_x 与 accel_x 的相关性（转弯时）===')
print('  转弯帧 corr(rot_x, accel_x) = %+.3f' % np.corrcoef(rx[turn], ax[turn])[0, 1])
print('  同号帧比例: %.1f%%' % (100 * np.mean(np.sign(rx[turn]) == np.sign(ax[turn]))))
