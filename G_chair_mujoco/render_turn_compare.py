#!/usr/bin/env python3
"""渲染左转/右转峰值帧的实际座椅姿态，直观对比方向。"""
import os, sys, math
import numpy as np
import mujoco
from PIL import Image, ImageDraw
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', 'G_chair_sim'))
from config import TELEMETRY_CHANNELS, FILTERS
from pose_from_motion import PoseFromMotion
from filters import FilterChain
from sim import GChairSim

a = np.loadtxt(os.path.join(HERE, 'output', 'raw_telemetry.csv'), delimiter=',', skiprows=1)
idx = {k: i for i, k in enumerate(TELEMETRY_CHANNELS)}
ax = a[:, idx['accel_x']]
# 找左转最强帧和右转最强帧
iL = np.argmin(ax); iR = np.argmax(ax)
print('左转峰值帧 accel_x=%.3f, 右转峰值帧 accel_x=%.3f' % (ax[iL], ax[iR]))

m = PoseFromMotion(); fc = FilterChain(FILTERS)
sim = GChairSim()
cam = mujoco.MjvCamera(); cam.lookat[:] = [0, 0, 0.6]
cam.distance = 1.9; cam.elevation = 20; cam.azimuth = 90
r = mujoco.Renderer(sim.model, height=400, width=560)

def render_at(i, tag):
    # 用前 60 帧稳定滤波器状态，再取第 i 帧
    for k in range(max(0, i-60), i+1):
        tele = {key: a[k, j] for key, j in idx.items()}
        p = m.apply_filters(m.compute(tele, 0.02), fc, 0.02)
        pose = np.array([p['sway']/1000, p['surge']/1000, p['heave']/1000,
                         math.radians(p['roll']), math.radians(p['pitch']),
                         math.radians(p['yaw'])])
        sim.command_pose(pose, 0.02); sim.step()
    act = sim.platform_pose()
    r.update_scene(sim.data, camera=cam)
    img = Image.fromarray(r.render())
    d = ImageDraw.Draw(img)
    d.text((10, 10), '%s  roll=%+.2f sway=%+.1fmm' %
           (tag, math.degrees(act[3]), act[0]*1000), fill=(0,255,0))
    return img

# 左转和右转必须用同一个滤波器实例连续运行才能对比真实状态
# 简化: 分别从头跑两条链
imgL = render_at(iL, 'LEFT')
# 重置滤波器
m = PoseFromMotion(); fc = FilterChain(FILTERS); sim = GChairSim()
imgR = render_at(iR, 'RIGHT')
r.close()

combo = Image.new('RGB', (imgL.width*2, imgL.height))
combo.paste(imgL, (0, 0)); combo.paste(imgR, (imgL.width, 0))
out = os.path.join(HERE, 'output', 'real_gta5', 'turn_compare.png')
combo.save(out)
print('保存', out)
