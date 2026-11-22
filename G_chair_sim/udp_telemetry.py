#!/usr/bin/env python3
"""
udp_telemetry.py — 跨电脑 UDP 遥测收发模块。

协议与工程自带 DCS_To_FlyPT_Mover.lua 完全一致:
  一条 UDP 报文 = 25 个空格分隔的浮点数, 顺序见 config.TELEMETRY_CHANNELS。
  例: "-0.012 1.200 0.030 ... 12.34"

游戏PC 运行发送端(该脚本里的 UdpTelemetrySender 或 DCS Lua),
仿真PC 运行 UdpTelemetryReceiver, 把收到的数据喂给现有姿态映射/滤波链。
"""

import socket
import threading
import time

from config import TELEMETRY_CHANNELS


def parse_datagram(data):
    """把一条 UDP 报文解析成遥测 dict; 无效返回 None。"""
    try:
        text = data.decode('ascii', errors='ignore').strip()
        # 兼容用逗号分隔的游戏/插件
        text = text.replace(',', ' ')
        parts = text.split()
        if len(parts) < len(TELEMETRY_CHANNELS):
            return None
        vals = [float(p) for p in parts[:len(TELEMETRY_CHANNELS)]]
    except (ValueError, UnicodeDecodeError):
        return None
    tele = dict(zip(TELEMETRY_CHANNELS, vals))
    tele['_scenario'] = 'UDP远程'
    tele['_scenario_idx'] = -1
    return tele


class UdpTelemetryReceiver:
    """后台线程收 UDP 遥测, get() 取最新一帧。"""

    def __init__(self, host='0.0.0.0', port=4124):
        self.host = host
        self.port = port
        self._lock = threading.Lock()
        self._latest = None
        self._recv_time = 0.0
        self._running = False
        self._thread = None
        self._sock = None
        self.frames = 0
        self.bad_frames = 0
        self.last_error = ''

    def start(self):
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._sock.bind((self.host, self.port))
        self._sock.settimeout(0.2)   # 让 stop() 能及时退出
        self._running = True
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    def _loop(self):
        while self._running:
            try:
                data, _addr = self._sock.recvfrom(8192)
            except socket.timeout:
                continue
            except OSError:
                break
            tele = parse_datagram(data)
            with self._lock:
                if tele is None:
                    self.bad_frames += 1
                    self.last_error = repr(data[:80])
                else:
                    self._latest = tele
                    self._recv_time = time.time()
                    self.frames += 1

    def get(self, max_age=None):
        """返回 (telemetry, age_sec); 无数据或过期返回 None。"""
        with self._lock:
            if self._latest is None:
                return None
            age = time.time() - self._recv_time
            if max_age is not None and age > max_age:
                return None
            return dict(self._latest), age

    def stop(self):
        self._running = False
        if self._thread is not None:
            self._thread.join(timeout=1.0)
        if self._sock is not None:
            self._sock.close()

    def stats(self):
        return '收包 %d, 坏包 %d%s' % (
            self.frames, self.bad_frames,
            (' (最后错误 %s)' % self.last_error) if self.bad_frames else '')


class UdpTelemetrySender:
    """测试/模板用: 把遥测 dict 按 25 浮点格式发到目标 IP:port。"""

    def __init__(self, host, port):
        self.remote = (host, port)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    def send(self, telemetry):
        vals = [float(telemetry.get(k, 0.0)) for k in TELEMETRY_CHANNELS]
        payload = ' '.join('%.4f' % v for v in vals).encode('ascii')
        self.sock.sendto(payload, self.remote)

    def close(self):
        self.sock.close()
