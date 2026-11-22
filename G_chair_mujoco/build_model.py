"""
build_model.py — 由几何参数生成 MuJoCo MJCF 模型 (model.xml)

模型结构:
  world
   ├─ floor (地面)
   ├─ base_plate (底座, 固定)
   ├─ platform (上平台, free joint, 6 自由度) + 座椅
   └─ legA_i ─ legB_i ─ legC_i(slide)   ×6
        每条支腿: 底座万向节(2 hinge) + 直线滑轨(等效电缸) + 两端球头
  6 个 equality/connect 把支腿顶端约束到平台铰点 → 闭环 Stewart 机构
  6 个 position 执行器驱动滑轨 (目标 = 相对中位的位移, m)

用法:
  python build_model.py            # 生成同目录 model.xml
"""

import os
import numpy as np

from params import PARAMS, RigParams


def quat_z_to(u):
    u = np.asarray(u, dtype=float)
    u = u / np.linalg.norm(u)
    z = np.array([0.0, 0.0, 1.0])
    c = float(np.dot(z, u))
    if c > 1.0 - 1e-9:
        return np.array([1.0, 0.0, 0.0, 0.0])
    if c < -1.0 + 1e-9:
        return np.array([0.0, 1.0, 0.0, 0.0])
    axis = np.cross(z, u)
    s = np.linalg.norm(axis)
    axis /= s
    ang = np.arccos(c)
    return np.concatenate([[np.cos(ang / 2.0)], axis * np.sin(ang / 2.0)])


def _fmt(v):
    return " ".join("%.8g" % x for x in v)


def build_xml(p: RigParams = PARAMS) -> str:
    B = p.base_anchors()
    P = p.platform_anchors()
    H = p.neutral_height()
    deck_r = p.plat_radius + 0.035
    base_r = p.base_radius + 0.045

    L = []
    a = L.append
    a('<mujoco model="gchair_stewart">')
    a('  <compiler angle="radian" autolimits="true"/>')
    a('  <option timestep="0.002" integrator="implicitfast" '
      'gravity="0 0 -9.81" iterations="100" tolerance="1e-9"/>')
    a('  <visual>')
    a('    <global offwidth="1280" offheight="720"/>')
    a('    <quality shadowsize="4096"/>')
    a('    <headlight ambient="0.55 0.55 0.55" diffuse="0.7 0.7 0.7" '
      'specular="0.15 0.15 0.15"/>')
    a('  </visual>')
    a('  <asset>')
    a('    <texture name="grid" type="2d" builtin="checker" '
      'rgb1="0.22 0.24 0.28" rgb2="0.30 0.32 0.36" width="512" height="512"/>')
    a('    <material name="grid" texture="grid" texrepeat="8 8" reflectance="0.05"/>')
    a('    <material name="metal" rgba="0.64 0.66 0.72 1" metallic="0.85" roughness="0.35"/>')
    a('    <material name="rod" rgba="0.85 0.86 0.90 1" metallic="0.9" roughness="0.2"/>')
    a('    <material name="dark" rgba="0.10 0.11 0.13 1" metallic="0.3" roughness="0.65"/>')
    a('    <material name="seat" rgba="0.06 0.06 0.07 1" metallic="0.0" roughness="0.9"/>')
    a('    <material name="joint" rgba="0.20 0.45 0.85 1" metallic="0.7" roughness="0.3"/>')
    # 外观增强用材质（纯视觉，不影响动力学）
    a('    <material name="extrusion" rgba="0.26 0.27 0.30 1" metallic="0.65" roughness="0.45"/>')
    a('    <material name="barrel" rgba="0.20 0.21 0.24 1" metallic="0.5" roughness="0.5"/>')
    a('    <material name="motor" rgba="0.22 0.23 0.25 1" metallic="0.6" roughness="0.4"/>')
    a('    <material name="screen" rgba="0.02 0.03 0.05 1" metallic="0.1" roughness="0.25"/>')
    a('  </asset>')

    a('  <worldbody>')
    a('    <light name="key" pos="1.6 -1.6 2.6" dir="-0.45 0.45 -1" '
      'directional="true" diffuse="0.8 0.8 0.8" castshadow="true"/>')
    a('    <light name="fill" pos="-1.8 1.4 2.0" dir="0.5 -0.4 -0.8" '
      'directional="true" diffuse="0.45 0.45 0.45"/>')
    a('    <geom name="floor" type="plane" size="4 4 0.05" material="grid" '
      'contype="1" conaffinity="1"/>')
    a('    <body name="base_plate" pos="0 0 0">')
    # 实物底座是矩形铝型材框架，不是圆盘。用 4 根型材 + 薄底板近似。
    a('      <geom name="base_panel" type="box" size="%.6g %.6g 0.008" '
      'pos="0 0 -0.014" material="dark" contype="0" conaffinity="0"/>'
      % (base_r * 0.95, base_r * 0.95))
    fw = base_r + 0.01
    prof = 0.025
    for nm, pos, size in [
        ('front', (0.0, -fw, 0.0), (fw, prof, 0.025)),
        ('rear',  (0.0,  fw, 0.0), (fw, prof, 0.025)),
        ('left',  (-fw, 0.0, 0.0), (prof, fw, 0.025)),
        ('right', ( fw, 0.0, 0.0), (prof, fw, 0.025)),
    ]:
        a('      <geom name="base_frame_%s" type="box" size="%s" pos="%s" '
          'material="extrusion" contype="0" conaffinity="0"/>'
          % (nm, _fmt(size), _fmt(pos)))
    # 每个底座铰点：球碗座 + 伺服电机盒（实物中电机安装在型材上）
    for i in range(6):
        b = B[i]
        a('      <geom name="socket_cup_%d" type="cylinder" size="0.038 0.014" '
          'pos="%.6g %.6g 0.004" material="dark" contype="0" conaffinity="0"/>' % (i, b[0], b[1]))
        a('      <geom name="socket_ball_%d" type="sphere" size="0.030" '
          'pos="%.6g %.6g -0.014" material="metal" contype="0" conaffinity="0"/>' % (i, b[0], b[1]))
    a('    </body>')

    # ---------------- platform ----------------
    a('    <body name="platform" pos="0 0 %.8g" quat="1 0 0 0">' % H)
    a('      <freejoint name="platform_free"/>')
    a('      <inertial pos="0 0 0.12" mass="%.6g" diaginertia="1.7 1.7 2.6"/>'
      % p.platform_mass)
    # 上平台同样是铝型材框架 + 座椅安装板
    a('      <geom name="deck" type="box" size="%.6g %.6g 0.008" pos="0 0 0" '
      'material="extrusion" contype="0" conaffinity="0"/>' % (deck_r * 0.92, deck_r * 0.92))
    pw = deck_r + 0.01
    pp = 0.022
    for nm, pos, size in [
        ('front', (0.0, -pw, 0.0), (pw, pp, 0.022)),
        ('rear',  (0.0,  pw, 0.0), (pw, pp, 0.022)),
        ('left',  (-pw, 0.0, 0.0), (pp, pw, 0.022)),
        ('right', ( pw, 0.0, 0.0), (pp, pw, 0.022)),
    ]:
        a('      <geom name="plat_frame_%s" type="box" size="%s" pos="%s" '
          'material="extrusion" contype="0" conaffinity="0"/>'
          % (nm, _fmt(size), _fmt(pos)))
    # 桶形赛车座椅：座垫 + 侧翼 + 靠背 + 肩翼 + 头枕
    a('      <geom name="seat_base" type="box" size="0.23 0.24 0.045" '
      'pos="0 0 0.06" material="seat" contype="0" conaffinity="0"/>')
    for nm, sx in [('l', -1), ('r', 1)]:
        a('      <geom name="seat_bolster_%s" type="box" size="0.045 0.24 0.065" '
          'pos="%.3g 0 0.075" material="seat" contype="0" conaffinity="0"/>'
          % (nm, sx * 0.225))
        a('      <geom name="seat_back_bolster_%s" type="box" size="0.05 0.055 0.28" '
          'pos="%.3g 0.23 0.33" material="seat" contype="0" conaffinity="0"/>'
          % (nm, sx * 0.21))
        a('      <geom name="seat_shoulder_%s" type="box" size="0.06 0.06 0.10" '
          'pos="%.3g 0.23 0.50" material="seat" contype="0" conaffinity="0"/>'
          % (nm, sx * 0.20))
    a('      <geom name="seat_back" type="box" size="0.22 0.05 0.26" '
      'pos="0 0.23 0.32" material="seat" contype="0" conaffinity="0"/>')
    a('      <geom name="seat_head" type="box" size="0.11 0.05 0.09" '
      'pos="0 0.23 0.60" material="seat" contype="0" conaffinity="0"/>')
    # 方向盘 + 显示器支架（座椅前方，-Y 方向）
    for nm, sx in [('l', -1), ('r', 1)]:
        a('      <geom name="wheel_mount_%s" type="box" size="0.02 0.22 0.02" '
          'pos="%.3g -0.20 0.36" material="extrusion" contype="0" conaffinity="0"/>'
          % (nm, sx * 0.19))
    a('      <geom name="steering_wheel" type="cylinder" size="0.155 0.012" '
      'pos="0 -0.42 0.46" quat="0.70710678 0.70710678 0 0" '
      'material="dark" contype="0" conaffinity="0"/>')
    a('      <geom name="wheel_spoke_v" type="box" size="0.012 0.012 0.15" '
      'pos="0 -0.42 0.46" material="dark" contype="0" conaffinity="0"/>')
    a('      <geom name="wheel_spoke_h" type="box" size="0.15 0.012 0.012" '
      'pos="0 -0.42 0.46" material="dark" contype="0" conaffinity="0"/>')
    a('      <geom name="wheel_stem" type="box" size="0.02 0.02 0.06" '
      'pos="0 -0.42 0.41" material="extrusion" contype="0" conaffinity="0"/>')
    # 显示器立柱 + 横撑（否则屏幕悬空）
    for nm, sx in [('l', -1), ('r', 1)]:
        a('      <geom name="monitor_post_%s" type="box" size="0.015 0.015 0.40" '
          'pos="%.3g -0.36 0.44" material="extrusion" contype="0" conaffinity="0"/>'
          % (nm, sx * 0.15))
    a('      <geom name="monitor_brace" type="box" size="0.18 0.02 0.015" '
      'pos="0 -0.56 0.82" material="extrusion" contype="0" conaffinity="0"/>')
    a('      <geom name="monitor_frame" type="box" size="0.36 0.010 0.22" '
      'pos="0 -0.62 0.82" material="dark" contype="0" conaffinity="0"/>')
    a('      <geom name="monitor_screen" type="box" size="0.34 0.006 0.20" '
      'pos="0 -0.615 0.82" material="screen" contype="0" conaffinity="0"/>')
    for i in range(6):
        a('      <site name="plat_%d" pos="%s" size="0.012"/>'
          % (i, _fmt(P[i])))
    a('    </body>')

    # ---------------- legs ----------------
    for i in range(6):
        b = B[i]
        u = np.array([P[i][0], P[i][1], H]) - np.array([b[0], b[1], 0.0])
        q = quat_z_to(u)
        a('    <body name="legA_%d" pos="%s" quat="%s">' % (i, _fmt(b), _fmt(q)))
        a('      <joint name="legA_%d_x" type="hinge" axis="1 0 0" '
          'range="-0.6 0.6" damping="2" armature="0.01"/>' % i)
        a('      <geom name="barrel_%d" type="cylinder" size="0.038 0.22" '
          'pos="0 0 0.23" material="barrel" contype="0" conaffinity="0"/>' % i)
        a('      <geom name="barrel_cap_%d" type="cylinder" size="0.040 0.012" '
          'pos="0 0 0.45" material="metal" contype="0" conaffinity="0"/>' % i)
        a('      <geom name="servo_%d" type="box" size="0.055 0.070 0.085" '
          'pos="0.055 0 0.10" material="motor" contype="0" conaffinity="0"/>' % i)
        a('      <geom name="servo_cap_%d" type="cylinder" size="0.045 0.015" '
          'pos="0.105 0 0.10" quat="0.70710678 0 0.70710678 0" '
          'material="dark" contype="0" conaffinity="0"/>' % i)
        a('      <geom name="ballbase_%d" type="sphere" size="0.024" pos="0 0 0" '
          'material="joint" contype="0" conaffinity="0"/>' % i)
        a('      <body name="legB_%d" pos="0 0 0">' % i)
        a('        <inertial pos="0 0 0.05" mass="0.15" '
          'diaginertia="1e-4 1e-4 1e-4"/>')
        a('        <joint name="legB_%d_y" type="hinge" axis="0 1 0" '
          'range="-0.6 0.6" damping="2" armature="0.01"/>' % i)
        a('        <body name="legC_%d" pos="0 0 0">' % i)
        a('          <joint name="leg_%d" type="slide" axis="0 0 1" '
          'range="%.8g %.8g" damping="0.4" armature="0.01"/>'
          % (i, -p.range, p.range))
        a('          <geom name="rod_%d" type="cylinder" size="0.020 0.22" '
          'pos="0 0 %.8g" material="rod" contype="0" conaffinity="0"/>'
          % (i, p.leg_middle - 0.22))
        a('          <geom name="balltop_%d" type="sphere" size="0.020" '
          'pos="0 0 %.8g" material="joint" contype="0" conaffinity="0"/>'
          % (i, p.leg_middle))
        a('          <site name="tip_%d" pos="0 0 %.8g" size="0.010"/>'
          % (i, p.leg_middle))
        a('        </body>')
        a('      </body>')
        a('    </body>')

    a('  </worldbody>')

    a('  <equality>')
    for i in range(6):
        a('    <connect name="close_%d" site1="tip_%d" site2="plat_%d" '
          'solref="0.002 1" solimp="0.92 0.96 0.001"/>' % (i, i, i))
    a('  </equality>')

    a('  <actuator>')
    for i in range(6):
        a('    <position name="act_%d" joint="leg_%d" kp="%.6g" kv="%.6g" '
          'ctrlrange="%.8g %.8g" forcerange="%.8g %.8g" ctrllimited="true" '
          'forcelimited="true"/>'
          % (i, i, p.kp, p.kv, -p.range, p.range, -p.force_max, p.force_max))
    a('  </actuator>')

    a('  <sensor>')
    a('    <framepos name="platform_pos" objtype="body" objname="platform"/>')
    a('    <framequat name="platform_quat" objtype="body" objname="platform"/>')
    for i in range(6):
        a('    <jointpos name="leg_pos_%d" joint="leg_%d"/>' % (i, i))
        a('    <jointvel name="leg_vel_%d" joint="leg_%d"/>' % (i, i))
    a('  </sensor>')

    a('</mujoco>')
    return "\n".join(L) + "\n"


def main():
    here = os.path.dirname(os.path.abspath(__file__))
    xml = build_xml()
    out = os.path.join(here, 'model.xml')
    with open(out, 'w', encoding='utf-8') as fh:
        fh.write(xml)
    print('wrote %s (%d bytes)' % (out, len(xml)))
    p = PARAMS
    print('neutral height = %.4f m, leg middle = %.3f m, stroke = %.3f m'
          % (p.neutral_height(), p.leg_middle, p.stroke))


if __name__ == '__main__':
    main()
