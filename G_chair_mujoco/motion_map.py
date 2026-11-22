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

# 与 config.POSE_FROM_MOTION 一致的增益量级
ACCEL_ROLL_GAIN = 100.0 * 0.5 * 0.7   # =35 度/G 的基础, 再乘 0.7 缩放
ACCEL_PITCH_GAIN = 100.0 * 0.5 * 0.6
YAW_GAIN = 50.0 * 0.2
SWAY_GAIN = 100.0
SURGE_GAIN = 100.0 * (-1.0)   # surge 输出方向与纵向加速度相反(原配置)
HEAVE_GAIN = 50.0
ROLL_RANGE = 6.0
PITCH_RANGE = 6.0
YAW_RANGE = 8.0
SWAY_RANGE = 50.0
SURGE_RANGE = 50.0
HEAVE_RANGE = 30.0


def accel_driven_pose(tele, dt, washout=None):
    """返回 [x,y,z,roll,pitch,yaw] (m, rad)。"""
    ax = float(tele.get('accel_x', 0.0))  # 横向 G, 左转负/右转正
    az = float(tele.get('accel_z', 0.0))  # 纵向 G
    ay = float(tele.get('accel_y', 0.0))  # 垂直 G
    ry = float(tele.get('rot_y', 0.0))    # 偏航角速度

    # roll: 横向加速度驱动。左转(ax<0) -> 车身向右倾(roll>0) 对抗离心力
    roll = np.clip(-ax * ACCEL_ROLL_GAIN, -ROLL_RANGE, ROLL_RANGE)
    # pitch: 纵向加速度驱动。加速(az>0) -> 抬头(pitch<0), 刹车点头
    pitch = np.clip(-az * ACCEL_PITCH_GAIN, -PITCH_RANGE, PITCH_RANGE)
    # yaw: 偏航角速度驱动
    yaw = np.clip(ry * YAW_GAIN, -YAW_RANGE, YAW_RANGE)

    # 平移: sway/surge/heave 仍按加速度积分映射(与原一致)
    sway = np.clip(ax * SWAY_GAIN, -SWAY_RANGE, SWAY_RANGE)
    surge = np.clip(az * SURGE_GAIN, -SURGE_RANGE, SURGE_RANGE)
    heave = np.clip(ay * HEAVE_GAIN, -HEAVE_RANGE, HEAVE_RANGE)

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
