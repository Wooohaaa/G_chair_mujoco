#!/usr/bin/env python3
"""对比几种修复方案对左右转对称性 + 尖峰的改善，用同一份真实数据。"""
import os, sys, math
import numpy as np
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', 'G_chair_sim'))
from config import TELEMETRY_CHANNELS, FILTERS
from pose_from_motion import PoseFromMotion
from filters import FilterChain, EDYNLP, EMALP

a = np.loadtxt(os.path.join(HERE, 'output', 'raw_telemetry.csv'), delimiter=',', skiprows=1)
idx = {k: i for i, k in enumerate(TELEMETRY_CHANNELS)}
ax_all = a[:, idx['accel_x']]

# 遥测低通器
class TeleLP:
    def __init__(self, tau): self.tau=tau; self.v={}
    def update(self, tele, dt):
        al = dt/(self.tau+dt); out=dict(tele)
        for k in ('accel_x','accel_z','accel_y','rot_x','rot_y','rot_z'):
            x=float(tele.get(k,0)); p=self.v.get(k,x)
            self.v[k]=(1-al)*p+al*x; out[k]=self.v[k]
        return out

def run(lpf=0.0, rot_mode='edynlp20'):
    """返回每帧的 (sway, roll, yaw) 姿态(度/mm)。"""
    m=PoseFromMotion(); fc=FilterChain(FILTERS); lp=TeleLP(lpf) if lpf>0 else None
    out=[]
    for row in a:
        tele={k:row[i] for k,i in idx.items()}
        if lp: tele=lp.update(tele,0.02)
        p=m.apply_filters(m.compute(tele,0.02),fc,0.02)
        out.append([p['sway'],p['roll'],p['pitch'],p['yaw']])
    return np.array(out)

def metrics(out, name):
    sway,roll,pitch,yaw = out[:,0],out[:,1],out[:,2],out[:,3]
    turn=np.abs(ax_all)>0.15
    L=turn&(ax_all<0); R=turn&(ax_all>0)
    # 对称度: 左右均值应反号且幅值接近; 用 |mean_L|/|mean_R|
    res={}
    for nm,v in [('sway',sway),('roll',roll),('yaw',yaw)]:
        lm,rm=v[L].mean(),v[R].mean()
        ratio=abs(lm)/max(abs(rm),1e-6)
        res[nm]=(lm,rm,ratio)
    # 尖峰: 各通道 P99-P1 跨度
    spikes={nm:float(np.percentile(v,99)-np.percentile(v,1)) for nm,v in [('roll',roll),('sway',sway),('yaw',yaw)]}
    print('\n=== %s ===' % name)
    for nm,(lm,rm,ratio) in res.items():
        flag='OK' if 0.7<ratio<1.4 else '*** 不对称 ***'
        print('  %-5s 左%+7.2f 右%+7.2f  幅值比%.2f %s' % (nm,lm,rm,ratio,flag))
    print('  尖峰跨度(P99-P1): roll %.1f sway %.1f yaw %.1f' % (spikes['roll'],spikes['sway'],spikes['yaw']))
    return res,spikes

# 方案对比
print('数据: %d 帧' % len(a))
base=run(); metrics(base,'基线(当前)')
lp=run(lpf=0.15); metrics(lp,'方案A: 遥测低通 tau=0.15')

# 方案B: 把 rotation 的 EDYNLP strength 提高(改 FILTERS 副本)
import copy
orig=copy.deepcopy(FILTERS)
FILTERS['rotation']['edynlp_strength']=40.0
b=run(); metrics(b,'方案B: EDYNLP strength 20->40')
FILTERS['rotation']['edynlp_strength']=orig['rotation']['edynlp_strength']

# 方案C: 低通 + strength 40
FILTERS['rotation']['edynlp_strength']=40.0
c=run(lpf=0.15); metrics(c,'方案C: 低通 + strength40')
FILTERS['rotation']['edynlp_strength']=orig['rotation']['edynlp_strength']
