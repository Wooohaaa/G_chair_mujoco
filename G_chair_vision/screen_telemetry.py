#!/usr/bin/env python3
"""
screen_telemetry.py — RK3566 视觉节点：游戏画面 → G_chair 25 浮点 UDP 遥测

设计要点（为什么不是"YOLO 框直接算加速度"）:
  赛车游戏镜头跟随车辆，车在画面里基本居中，车框位置/大小几乎不含加减速信息。
  真正能推算 ego-motion 的是【路面/背景光流】。YOLO(RKNN) 只做语义补充:
    - 判断是否在游戏画面（菜单/加载时抑制输出）
    - 检测前方车辆 → 距离突变产生冲击顿挫
  主链路: 光流 → 速度代理 → 差分得加速度代理 → 低通+洗出 → 6DOF → 25浮点UDP

  输出格式与 DCS_To_FlyPT_Mover.lua / G_chair_sim/udp_telemetry.py 完全一致，
  仿真端直接跑 sim_udp.py 即可，无需改动。

RK3566 部署:
  - 必须 INT8 模型 (yolov8n.rknn)，FP16 只有 3~5 FPS 不可用
  - 固定输入 320/640，后处理放 CPU
  - rknnlite 可选: 没装也能用纯光流模式跑

用法:
  # 本机自测（合成路面动画，不需要摄像头/RKNN）
  ./.venv/bin/python screen_telemetry.py --mock --ip 127.0.0.1 --duration 20

  # 板子没联网时: TCP server + 电脑 adb forward（见 README）
  python3 screen_telemetry.py --mock --transport tcp --port 4124 --hz 20

  # RK3566 实机（HDMI采集卡 /dev/video9，同网段直发 UDP）
  python3 screen_telemetry.py --source /dev/video9 --ip 10.148.82.176
"""

import argparse
import math
import os
import socket
import sys
import time

import numpy as np

# 让它输出与 G_chair_sim 完全一致的通道顺序
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', 'G_chair_sim'))
from config import TELEMETRY_CHANNELS

try:
    import cv2
except ImportError:
    raise ImportError('需要 opencv-python-headless: pip install opencv-python-headless')


# ============================== 可调参数 ==============================
PROC_W, PROC_H = 320, 180        # 处理分辨率（低分辨率换帧率）
ROI_TOP = 0.35                   # 只取画面下半部分（路面区域）
ROI_LEFT, ROI_RIGHT = 0.15, 0.85 # 去掉左右边缘
LP_TAU = 0.20                    # 低通时间常数 (s)，滤掉光流抖动
WASHOUT_TAU = 2.5                # 洗出时间常数 (s)，恒定速度后回中位
SPEED_SCALE = 6.0                # 光流→速度代理 的增益（需实测标定）
ACCEL_CLIP = 1.5                 # 加速度代理限幅 (G)
ROT_CLIP = math.radians(8.0)     # 角速度限幅 (rad/s)


class EMA:
    """一阶低通"""
    def __init__(self, tau):
        self.tau = tau
        self.v = None

    def update(self, x, dt):
        if self.v is None:
            self.v = x
        else:
            a = dt / (self.tau + dt)
            self.v = (1 - a) * self.v + a * x
        return self.v


class Washout:
    """高通/洗出：去掉直流分量，恒速时输出回 0。体感平台必须洗出。"""
    def __init__(self, tau):
        self.tau = tau
        self.prev_x = None
        self.prev_y = 0.0

    def update(self, x, dt):
        if self.prev_x is None:
            self.prev_x = x
            return 0.0
        a = self.tau / (self.tau + dt)
        y = a * (self.prev_y + x - self.prev_x)
        self.prev_x, self.prev_y = x, y
        return y


class VisionMotion:
    """光流 + 可选 YOLO 语义 → 6DOF 运动代理"""

    def __init__(self, use_yolo=False, model_path=None, debug_dir=None):
        self.use_yolo = use_yolo
        self.yolo = None
        if use_yolo:
            self.yolo = YoloGate(model_path)
        self.prev_gray = None
        self.prev_t = None
        self.prev_speed = None
        # 每个自由度: 低通 + 洗出
        self.filters = {
            k: (EMA(LP_TAU), Washout(WASHOUT_TAU))
            for k in ('accel_x', 'accel_y', 'accel_z', 'rot_x', 'rot_y', 'rot_z')
        }
        self.debug_dir = debug_dir
        if debug_dir:
            os.makedirs(debug_dir, exist_ok=True)
        self.frame_idx = 0
        self.last_raw = {}

    def _flow_features(self, frame, gray):
        """返回 (纵向速度代理, 横向速度代理, 颠簸强度, 有效标志)"""
        h, w = gray.shape
        y0 = int(h * ROI_TOP)
        x0, x1 = int(w * ROI_LEFT), int(w * ROI_RIGHT)
        roi = gray[y0:, x0:x1]

        if self.prev_gray is None:
            self.prev_gray = roi.copy()
            return 0.0, 0.0, 0.0, False

        # Farneback 稠密光流：路面纹理的运动
        flow = cv2.calcOpticalFlowFarneback(
            self.prev_gray, roi, None, 0.5, 3, 15, 3, 5, 1.2, 0)
        self.prev_gray = roi.copy()

        fy = flow[:, :, 1]      # 向下为正 → 前进
        fx = flow[:, :, 0]      # 向右为正 → 横向
        # 用中位数抗异常值（其它车的局部运动、压缩噪点）
        v_long = float(np.median(fy))
        v_lat = float(np.median(fx))
        # 垂直方向的离散度 → 颠簸/纹理错乱强度
        bump = float(np.std(fy))
        return v_long, v_lat, bump, True

    def process(self, frame, t):
        proc = cv2.resize(frame, (PROC_W, PROC_H))
        gray = cv2.cvtColor(proc, cv2.COLOR_BGR2GRAY)
        gray = cv2.GaussianBlur(gray, (5, 5), 0)

        dt = 0.033 if self.prev_t is None else max(0.005, min(0.2, t - self.prev_t))
        self.prev_t = t

        v_long, v_lat, bump, valid = self._flow_features(proc, gray)

        # 光流 → 速度代理 → 差分 → 加速度代理
        speed = v_long * SPEED_SCALE if valid else 0.0
        if self.prev_speed is None or not valid:
            accel_long = 0.0
        else:
            accel_long = (speed - self.prev_speed) / dt
        self.prev_speed = speed

        raw = {
            'accel_z': np.clip(accel_long, -ACCEL_CLIP, ACCEL_CLIP),   # 纵向 surge
            'accel_x': np.clip(v_lat * SPEED_SCALE * 2.0, -ACCEL_CLIP, ACCEL_CLIP),  # 横向 sway
            'accel_y': np.clip((bump - 1.0) * 0.25, -ACCEL_CLIP, ACCEL_CLIP),        # heave
            'rot_y':   np.clip(v_lat * 0.15, -ROT_CLIP, ROT_CLIP),    # yaw
            'rot_x':   np.clip(-v_lat * 0.10, -ROT_CLIP, ROT_CLIP),   # roll
            'rot_z':   np.clip((bump - 1.0) * 0.02, -ROT_CLIP, ROT_CLIP),  # pitch
        }

        # YOLO 语义：菜单/过场时抑制，前方车逼近时加冲击
        if self.yolo is not None:
            gate, jolt = self.yolo.analyze(proc)
            if not gate:
                raw = {k: 0.0 for k in raw}
            if jolt > 0:
                raw['accel_z'] = np.clip(raw['accel_z'] - jolt, -ACCEL_CLIP, ACCEL_CLIP)

        # 低通 + 洗出
        out = {}
        for k, (lp, hp) in self.filters.items():
            out[k] = hp.update(lp.update(raw[k], dt), dt)

        self.last_raw = raw
        self.frame_idx += 1
        if self.debug_dir and self.frame_idx % 10 == 0:
            cv2.imwrite(os.path.join(self.debug_dir, 'f%05d.jpg' % self.frame_idx), proc)
        return out, raw, speed, valid


class YoloGate:
    """可选 RKNN 语义门控。rknnlite 未安装时自动禁用。"""
    def __init__(self, model_path=None):
        self.ok = False
        try:
            from rknnlite.api import RKNNLite
            self.rknn = RKNNLite()
            rc = self.rknn.load_rknn(model_path or 'yolov8n.rknn')
            rc2 = self.rknn.init_runtime()
            self.ok = (rc == 0 and rc2 == 0)
            print('[yolo] RKNN 加载成功')
        except Exception as e:
            print('[yolo] 不可用，退化为纯光流: %s' % e)
            self.ok = False

    def analyze(self, img):
        if not self.ok:
            return True, 0.0
        # TODO: 按你的 rknn_model_zoo yolov8 后处理解码，提取:
        #   gate = 检测到"车/路面"类目标
        #   jolt = 前方车辆框面积快速增长量
        return True, 0.0


class MockCapture:
    """合成向前行驶的路面动画，用来在本机验证整条链路。"""
    def __init__(self):
        self.t = 0.0
        self.phase = 0.0
        self.speed = 0.0

    def read(self):
        self.t += 1.0 / 30.0
        # 前 6s 加速，6~10s 匀速，10~14s 急刹，之后循环
        seg = self.t % 18.0
        if seg < 6:
            self.speed = seg / 6.0 * 8.0
        elif seg < 10:
            self.speed = 8.0
        elif seg < 14:
            self.speed = max(0.0, 8.0 - (seg - 10) * 2.0)
        else:
            self.speed = 0.0
        self.phase = (self.phase + self.speed * 0.02) % 1.0

        img = np.full((PROC_H, PROC_W, 3), 60, dtype=np.uint8)
        # 路面（下半部分）灰度渐变模拟纹理
        for y in range(int(PROC_H * ROI_TOP), PROC_H):
            v = 40 + int(60 * (y / PROC_H))
            img[y, :, :] = v
        # 横向条纹随速度向下滚动 → 光流纵向分量
        for k in range(8):
            y = int(PROC_H * ROI_TOP + ((k / 8.0 + self.phase) % 1.0) *
                    PROC_H * (1 - ROI_TOP))
            if y < PROC_H:
                cv2.line(img, (int(PROC_W * ROI_LEFT), y),
                         (int(PROC_W * ROI_RIGHT), y), (200, 200, 200), 2)
        # 车道线（透视线）
        cx = PROC_W // 2
        for side in (-1, 1):
            cv2.line(img, (cx + side * 15, int(PROC_H * ROI_TOP)),
                     (cx + side * 110, PROC_H), (220, 220, 220), 2)
        return True, img


def build_telemetry(m, speed, speed_lat, valid, t):
    """6DOF 代理 → 与 DCS 一致的 25 通道 dict。"""
    d = dict.fromkeys(TELEMETRY_CHANNELS, 0.0)
    d['accel_x'] = m['accel_x']
    d['accel_z'] = m['accel_z']
    d['accel_y'] = m['accel_y']
    d['rot_y'] = m['rot_y']
    d['rot_x'] = m['rot_x']
    d['rot_z'] = m['rot_z']
    d['speed_z'] = speed
    d['speed_x'] = speed_lat
    d['heading'] = 0.0
    d['time'] = t
    d['_scenario'] = '视觉' if valid else '无信号'
    d['_scenario_idx'] = -1
    return d


class TelemetrySender:
    """UDP 直发 或 TCP server（配合 adb forward，板子无网络时用）。"""

    def __init__(self, transport, ip, port):
        self.transport = transport
        self.remote = (ip, port)
        self.sock = None
        self.conn = None
        if transport == 'udp':
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            print('[udp] -> %s:%d' % self.remote)
        else:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self.sock.bind(('0.0.0.0', port))
            self.sock.listen(1)
            self.sock.settimeout(0.5)
            print('[tcp] 监听 0.0.0.0:%d，等待电脑连接（先起板子，再 adb forward + 电脑端）' % port)

    def send(self, payload):
        data = payload.encode('ascii')
        if self.transport == 'udp':
            self.sock.sendto(data, self.remote)
            return True
        # TCP: 等连接 / 处理断线重连
        while True:
            if self.conn is not None:
                try:
                    self.conn.sendall(data + b'\n')
                    return True
                except (BrokenPipeError, ConnectionResetError, OSError):
                    print('[tcp] 客户端断开，等待重连')
                    try:
                        self.conn.close()
                    except Exception:
                        pass
                    self.conn = None
            try:
                self.conn, addr = self.sock.accept()
                print('[tcp] 客户端已连接 %s' % (addr,))
            except socket.timeout:
                return False

    def close(self):
        if self.conn:
            self.conn.close()
        if self.sock:
            self.sock.close()


def main():
    ap = argparse.ArgumentParser(description='RK3566 视觉节点 → 25浮点 UDP/TCP')
    ap.add_argument('--source', default='/dev/video9', help='视频源: /dev/videoN 或视频文件')
    ap.add_argument('--mock', action='store_true', help='合成路面动画（本机自测）')
    ap.add_argument('--ip', default='127.0.0.1', help='仿真PC IP (UDP 模式用)')
    ap.add_argument('--port', type=int, default=4124)
    ap.add_argument('--transport', choices=['udp', 'tcp'], default='udp',
                    help='tcp = 板子作 TCP server，配合电脑 adb forward 使用')
    ap.add_argument('--yolo', action='store_true', help='启用 RKNN YOLO 语义门控')
    ap.add_argument('--model', default=None, help='yolov8n.rknn 路径')
    ap.add_argument('--debug-dir', default=None, help='保存调试帧目录')
    ap.add_argument('--duration', type=float, default=0.0, help='0=一直运行')
    ap.add_argument('--hz', type=float, default=20.0, help='发送频率 (RK3566 建议 15~20)')
    args = ap.parse_args()

    cap = None
    mock = None
    if args.mock:
        mock = MockCapture()
        print('[capture] 合成路面模式')
    else:
        cap = cv2.VideoCapture(args.source)
        if not cap.isOpened():
            print('[capture] 无法打开 %s' % args.source)
            return
        print('[capture] %s' % args.source)

    motion = VisionMotion(use_yolo=args.yolo, model_path=args.model,
                          debug_dir=args.debug_dir)
    sender = TelemetrySender(args.transport, args.ip, args.port)
    print('[send] @ %.0fHz' % args.hz)

    t0 = time.time()
    frames = 0
    try:
        while (args.duration <= 0) or (time.time() - t0 < args.duration):
            loop0 = time.time()
            t = time.time() - t0
            if mock:
                ok, frame = mock.read()
            else:
                ok, frame = cap.read()
            if not ok:
                print('[capture] 读取失败，退出')
                break

            m, raw, speed, valid = motion.process(frame, t)
            tele = build_telemetry(m, speed, 0.0, valid, t)
            payload = ' '.join('%.4f' % tele[k] for k in TELEMETRY_CHANNELS)
            sender.send(payload)
            frames += 1
            if frames % 20 == 0:
                print(' t=%.1fs 速度代理=%.2f  accel=%s  滤波后=%s' %
                      (t, speed, np.round([raw[k] for k in ('accel_z', 'accel_x')], 2),
                       np.round([m['accel_z'], m['accel_x']], 3)), flush=True)
            time.sleep(max(0, 1.0 / args.hz - (time.time() - loop0)))
    except KeyboardInterrupt:
        print('\n停止')
    finally:
        sender.close()
        if cap:
            cap.release()
    print('共发送 %d 帧' % frames)


if __name__ == '__main__':
    main()
