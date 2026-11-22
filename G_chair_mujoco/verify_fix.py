#!/usr/bin/env python3
"""用抓取的真实数据回放，验证修复后左右转对称性。"""
import os, sys, math
import numpy as np
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', 'G_chair_sim'))
from config import TELEMETRY_CHANNELS
from motion_map import accel_driven_pose, Washout6D
from sim import GChairSim

a = np.loadtxt(os.path.join(HERE, 'output', 'raw_telemetry.csv'), delimiter=',', skiprows=1)
i = {k: j for j, k in enumerate(TELEMETRY_CHANNELS)}
ax = a[:, i['accel_x']]

def run(accel_model=True):
    lp = {k: 0.0 for k in ('accel_x','accel_z','accel_y','rot_y')}
    wo = Washout6D(2.5) if accel_model else None
    sim = GChairSim(); poses=[]
    for row in a:
        tele = {k: row[j] for k,j in i.items()}
        al = 0.02/(0.15+0.02)
        for k in lp: lp[k]=(1-al)*lp[k]+al*float(tele.get(k,0))
        tele.update(lp)
        if accel_model:
            p = accel_driven_pose(tele, 0.02, wo)
        else:
            # 原始映射(简化版, 用 config 增益)
            rx=tele.get('rot_x',0); ry=tele.get('rot_y',0)
            p=np.array([np.clip(tele['accel_x']*100,-50,50)/1000,
                        np.clip(-tele['accel_z']*100,-50,50)/1000,
                        np.clip(tele['accel_y']*50,-30,30)/1000,
                        math.radians(np.clip(rx*25+tele['accel_x']*15,-6,6)),
                        0, math.radians(np.clip(ry*10,-8,8))])
        sim.command_pose(p,0.02); sim.step()
        poses.append(sim.platform_pose())
    return np.array(poses)

turn=np.abs(ax)>0.15; L=turn&(ax<0); R=turn&(ax>0)
for tag,model in [('原始 FlyPT 映射',False),('修复: 加速度驱动',True)]:
    poses=run(model)
    print('\n=== %s ===' % tag)
    for nm,j,sc in [('sway',0,1000),('roll',3,57.3),('yaw',5,57.3)]:
        lm,rm=poses[L,j].mean()*sc,poses[R,j].mean()*sc
        r=abs(lm)/max(abs(rm),1e-6)
        flag='OK' if 0.7<r<1.4 else '*** 不对称 ***'
        print('  %-5s 左%+7.2f 右%+7.2f 幅值比%.2f %s' % (nm,lm,rm,r,flag))
