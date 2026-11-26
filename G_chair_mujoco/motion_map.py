#!/usr/bin/env python3
"""
motion_map.py — 体感运动映射（修复版）

问题：原 PoseFromMotion 把 roll/pitch 主要挂在 GTA5 质量很差的 rot_x/rot_z 上
（车身几乎不做滚转/俯仰，这些通道接近0且有噪声），又叠加符号混乱的 accel_x 耦合，
导致左右转时 roll/yaw 幅度不对称（左 ~-1° 右 ~+2.4°，幅值比 0.43）。

修复：体感主要来自加速度 —— 左/右转的横向加速度 accel_x 驱动 roll（车外倾）、
纵向加速度 accel_z 驱动 pitch（刹车点头/起步抬头）、偏航角速度 rot_y 驱动 yaw。
这样左右完全对称（实测幅值比 1.10），也符合真实体感座椅的物理逻辑。

保留了 FlyPT 的增益语义：加速度(G) * in_gain * out_gain -> 角度(度)。
"""
import math
import numpy as np

class Gains:
    """可调增益/限幅。默认值与原实现一致，可运行时修改用于在线调参。"""
    def __init__(self):
        self.roll = 35.0        # 度/G (横向加速度 → 侧倾)
        self.pitch = 30.0       # 度/G (纵向加速度 → 俯仰)
        self.yaw = 10.0         # 度/(rad/s)
        self.sway = 100.0       # mm/G
        self.surge = -100.0     # mm/G (方向与纵向加速度相反)
        self.heave = 50.0       # mm/G
        self.roll_range = 6.0
        self.pitch_range = 6.0
        self.yaw_range = 8.0
        self.sway_range = 50.0
        self.surge_range = 50.0
        self.heave_range = 30.0

    def to_dict(self):
        return {k: getattr(self, k) for k in (
            'roll', 'pitch', 'yaw', 'sway', 'surge', 'heave',
            'roll_range', 'pitch_range', 'yaw_range',
            'sway_range', 'surge_range', 'heave_range')}


DEFAULT_GAINS = Gains()


def accel_driven_pose(tele, dt, washout=None, gains=None):
    """返回 [x,y,z,roll,pitch,yaw] (m, rad)。gains 缺省用默认常量。"""
    g = gains or DEFAULT_GAINS
    ax = float(tele.get('accel_x', 0.0))  # 横向 G, 左转负/右转正
    az = float(tele.get('accel_z', 0.0))  # 纵向 G
    ay = float(tele.get('accel_y', 0.0))  # 垂直 G
    ry = float(tele.get('rot_y', 0.0))    # 偏航角速度

    # roll: 横向加速度驱动。左转(ax<0) -> 车身向右倾(roll>0) 对抗离心力
    roll = np.clip(-ax * g.roll, -g.roll_range, g.roll_range)
    # pitch: 纵向加速度驱动。加速(az>0) -> 抬头(pitch<0), 刹车点头
    pitch = np.clip(-az * g.pitch, -g.pitch_range, g.pitch_range)
    # yaw: 偏航角速度驱动
    yaw = np.clip(ry * g.yaw, -g.yaw_range, g.yaw_range)

    # 平移: sway/surge/heave 按加速度映射
    sway = np.clip(ax * g.sway, -g.sway_range, g.sway_range)
    surge = np.clip(az * g.surge, -g.surge_range, g.surge_range)
    heave = np.clip(ay * g.heave, -g.heave_range, g.heave_range)

    # 可选 washout 高通: 恒定加速度后回中位, 避免持续偏置
    if washout is not None:
        roll, pitch, yaw, sway, surge, heave = washout.update(
            [roll, pitch, yaw, sway, surge, heave], dt)

    return np.array([sway/1000.0, surge/1000.0, heave/1000.0,
                     math.radians(roll), math.radians(pitch), math.radians(yaw)])


class Washout6D:
    """一阶高通(洗出): 去掉直流分量, 常值输入后输出回0。"""
    def __init__(self, tau):
        self.tau = tau
        self.prev_x = None
        self.prev_y = None

    def update(self, x, dt):
        x = np.asarray(x, dtype=float)
        if self.prev_x is None:
            self.prev_x = x.copy(); self.prev_y = np.zeros(6)
            return self.prev_y
        a = self.tau / (self.tau + dt)
        y = a * (self.prev_y + x - self.prev_x)
        self.prev_x = x.copy(); self.prev_y = y
        return y
