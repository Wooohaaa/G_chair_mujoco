#!/usr/bin/env python3
"""双 Mega 实时控制网页：GTA5 UDP 遥测 -> 映射管线 -> 两块 Mega 串口。

网页: http://<本机IP>:8770
WebSocket: 8771
按钮: 开始 / 停止（停止会先斜坡回 0 再关串口）

安全：软件限速 110mm/s、步数钳位、断流保持位置、异常自动回零。
"""
import asyncio
import json
import os
import socket
import threading
import time

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.join(HERE, '..')
import sys
sys.path.insert(0, ROOT)
sys.path.insert(0, os.path.join(ROOT, 'G_chair_sim'))

from config import TELEMETRY_CHANNELS
from motion_pipeline import MotionPipeline
from udp_telemetry import parse_datagram

CYL_TO_BOARD = {0: (0, 0), 5: (0, 1), 4: (0, 2), 3: (0, 3), 2: (1, 0), 1: (1, 1)}
DT = 0.02
MAX_STEP = int(110.0 * DT / 0.005)
HTML = os.path.join(HERE, 'live_dual_mega_web.html')

state = {
    'running': False, 'starting': False, 'stopping': False,
    'scale': 0.5, 'max_mm': 25.0, 'base_steps': 6000,
    'serial1': '/dev/ttyUSB1', 'serial2': '/dev/ttyUSB2', 'udp_port': 4124,
    'frames': 0, 'tele_frames': 0, 'last_tele': 0.0,
    'target': [0] * 6, 'applied': [0] * 6,
    'error': '', 't0': 0.0,
}
lock = threading.Lock()
_stop_event = threading.Event()
_thread = None


def frames_from_steps(steps):
    out = []
    for bi in range(2):
        ch = [0, 0, 0, 0]
        for cyl, (board, channel) in CYL_TO_BOARD.items():
            if board == bi:
                ch[channel] = int(round(steps[cyl] * 1.7))
        out.append(bytes([255, 255]) + b''.join(
            max(0, min(65535, v)).to_bytes(2, 'big') for v in ch))
    return out


def bridge_loop():
    import serial
    s1 = s2 = sock = None
    with lock:
        p1, p2, uport = state['serial1'], state['serial2'], state['udp_port']
        scale, maxmm, base = state['scale'], state['max_mm'], state['base_steps']
    try:
        print('[bridge] 初始化', p1, p2, uport, flush=True)
        if base * 0.005 + maxmm > 95:
            raise ValueError('偏置+限幅超出物理半行程')
        s1, s2 = [serial.Serial(p, 115200, timeout=0.1) for p in (p1, p2)]
        print('[bridge] 串口已打开', flush=True)
        for s in (s1, s2):
            s.dtr = False; s.rts = True; time.sleep(0.15)
            s.dtr = True; s.rts = False; time.sleep(0.15)
            s.reset_input_buffer()
        time.sleep(0.8)
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.bind(('0.0.0.0', uport)); sock.settimeout(0.005)
        print('[bridge] UDP已绑定', flush=True)
        mp = MotionPipeline()
        applied = np.zeros(6)
        target = np.zeros(6)
        tele = None
        last_tele = time.time()
        with lock:
            state['running'] = True
            state['starting'] = False
            state['t0'] = time.time()
        next_t = time.time()
        while not _stop_event.is_set():
            try:
                while True:
                    data, _ = sock.recvfrom(2048)
                    t = parse_datagram(data)
                    if t is not None:
                        tele = t
                        last_tele = time.time()
            except socket.timeout:
                pass
            if tele is not None and not _stop_event.is_set():
                _, disp = mp.pose_and_disp(tele, DT)
                v = np.clip(disp * scale, -maxmm, maxmm)
                target = np.clip(np.round(base + v / 0.005), 0, 38400)
                if _stop_event.is_set():
                    target = np.zeros(6)
            else:
                target = np.zeros(6) if _stop_event.is_set() else target
            delta = target - applied
            applied += np.clip(delta, -MAX_STEP, MAX_STEP)
            applied = np.clip(applied, 0, 38400)
            f1, f2 = frames_from_steps(applied.astype(int))
            s1.write(f1); s2.write(f2)
            with lock:
                state['frames'] += 1
                if tele is not None:
                    state['tele_frames'] += 1
                    state['last_tele'] = last_tele
                state['applied'] = np.round(applied).astype(int).tolist()
                state['target'] = np.round(target).astype(int).tolist()
            next_t += DT
            time.sleep(max(0, next_t - time.time()))
        # 停止：斜坡回 0
        zero = np.zeros(6)
        for _ in range(200):
            delta = zero - applied
            applied += np.clip(delta, -MAX_STEP, MAX_STEP)
            if np.max(np.abs(applied)) == 0:
                break
            f1, f2 = frames_from_steps(applied.astype(int))
            s1.write(f1); s2.write(f2)
            with lock:
                state['applied'] = np.round(applied).astype(int).tolist()
                state['target'] = [0] * 6
            time.sleep(DT)
        s1.write(frames_from_steps(zero)[0]); s2.write(frames_from_steps(zero)[1])
        time.sleep(0.2)
    except Exception as e:
        with lock:
            state['error'] = '%s: %s' % (type(e).__name__, e)
    finally:
        if s1:
            try: s1.close()
            except Exception: pass
        if s2:
            try: s2.close()
            except Exception: pass
        if sock:
            try: sock.close()
            except Exception: pass
        with lock:
            state['running'] = False
            state['starting'] = False
            state['stopping'] = False
            state['applied'] = [0] * 6
            state['target'] = [0] * 6


def start_bridge():
    global _thread
    with lock:
        if state['running'] or state['starting']:
            return False, '已在运行'
        state['error'] = ''
        state['starting'] = True
        state['frames'] = 0
        state['tele_frames'] = 0
    _stop_event.clear()
    _thread = threading.Thread(target=bridge_loop, daemon=True)
    _thread.start()
    return True, '启动中'


def stop_bridge():
    with lock:
        if not state['running'] and not state['starting']:
            return False, '未运行'
        state['stopping'] = True
    _stop_event.set()
    return True, '停止中（先回零）'


async def http_handler(reader, writer):
    try:
        req = await asyncio.wait_for(reader.read(2048), timeout=2.0)
        if b'live_dual_mega_web.js' in req:
            body = open(os.path.join(HERE, 'live_dual_mega_web.js'), 'rb').read()
            ctype = 'application/javascript'
        else:
            body = open(HTML, 'rb').read()
            ctype = 'text/html; charset=utf-8'
        writer.write(('HTTP/1.1 200 OK\r\nContent-Type: %s\r\nContent-Length: %d\r\n'
                      'Cache-Control: no-store\r\n\r\n' % (ctype, len(body))).encode() + body)
        await writer.drain()
    except Exception:
        pass
    finally:
        writer.close()


async def ws_handler(ws):
    async def push():
        while True:
            with lock:
                snap = dict(state)
                snap['age'] = (round(time.time() - state['last_tele'], 2)
                               if state['last_tele'] > 0 else -1)
            await ws.send(json.dumps(snap))
            await asyncio.sleep(0.1)
    pusher = asyncio.create_task(push())
    try:
        async for msg in ws:
            try:
                d = json.loads(msg)
            except Exception:
                continue
            cmd = d.get('cmd')
            if cmd == 'start':
                with lock:
                    state['scale'] = float(d.get('scale', state['scale']))
                    state['max_mm'] = float(d.get('max_mm', state['max_mm']))
                    state['base_steps'] = int(d.get('base_steps', state['base_steps']))
                ok, msg = start_bridge()
            elif cmd == 'stop':
                ok, msg = stop_bridge()
            elif cmd == 'params':
                with lock:
                    for k in ('scale', 'max_mm'):
                        if k in d:
                            state[k] = float(d[k])
                    if 'base_steps' in d:
                        state['base_steps'] = int(d['base_steps'])
                    if 'serial1' in d:
                        state['serial1'] = str(d['serial1'])
                    if 'serial2' in d:
                        state['serial2'] = str(d['serial2'])
                ok, msg = True, '参数已保存'
            else:
                ok, msg = False, '未知命令'
            await ws.send(json.dumps({'ack': cmd, 'ok': ok, 'msg': msg}))
    except Exception:
        pass
    finally:
        pusher.cancel()


async def main_async(port):
    await asyncio.start_server(http_handler, '0.0.0.0', port)
    import websockets
    await websockets.serve(ws_handler, '0.0.0.0', port + 1)
    print('双 Mega 实时控制台: http://localhost:%d (WS %d)' % (port, port + 1))
    print('UDP 接收 :%d -> %s / %s'
          % (state['udp_port'], state['serial1'], state['serial2']))
    await asyncio.Future()


if __name__ == '__main__':
    import argparse
    ap = argparse.ArgumentParser()
    ap.add_argument('--port', type=int, default=8770)
    ap.add_argument('--serial1', default='/dev/ttyUSB1')
    ap.add_argument('--serial2', default='/dev/ttyUSB2')
    ap.add_argument('--udp-port', type=int, default=4124)
    a = ap.parse_args()
    state['serial1'] = a.serial1
    state['serial2'] = a.serial2
    state['udp_port'] = a.udp_port
    asyncio.run(main_async(a.port))
