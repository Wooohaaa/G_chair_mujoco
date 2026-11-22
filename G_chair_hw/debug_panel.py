#!/usr/bin/env python3
"""
debug_panel.py — 连接真实 ESP32-S3 主控的调试测试面板（后端服务器）

功能:
  - 通过 USB 串口连接 ESP32 主控，发运动指令、收状态回传
  - 手动姿态测试：预设动作(中位/升沉/横滚/俯仰/偏航) + 自定义6缸位移
  - 实时状态：6缸目标/实际步数、缸速、限位、看门狗
  - 数据录制/回放：保存指令与状态到 CSV
  - 无硬件时 --simulate 模拟模式，可先验证界面

运行:
  ./.venv/bin/python debug_panel.py --port 8765                 # 真实串口(自动检测)
  ./.venv/bin/python debug_panel.py --serial /dev/ttyUSB0       # 指定串口
  ./.venv/bin/python debug_panel.py --simulate                  # 模拟模式(无硬件)
  浏览器: http://localhost:8765
"""
import argparse
import asyncio
import glob
import json
import os
import threading
import time

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
SIM_DIR = os.path.join(HERE, '..')
sys_path = SIM_DIR
import sys
sys.path.insert(0, sys_path)
sys.path.insert(0, os.path.join(SIM_DIR, 'G_chair_sim'))

from controller import DualBoardController

# 全局状态
HTTP_PORT = 8765

state = {
    'connected': False,
    'mode': 'simulate',
    'cur': [0] * 6,
    'tgt': [0] * 6,
    'limit': 0,
    'enabled': 0,
    'watchdog': 0,
    'speed': [0] * 6,
    'pending': None,      # 待发送到主控的目标位移(mm)
    'log': [],            # 录制的帧
    'recording': False,
    't0': time.time(),
}

ctl = DualBoardController()
MAX_POS = 38400
CENTER = 19200
STEP_MM = 192.0 / MAX_POS   # 0.005 mm/step


def steps_to_mm(steps):
    return (steps - CENTER) * STEP_MM


def mm_to_16bit(disp_mm):
    """位移mm -> FlyPT 16bit (与 serial_bridge 一致)。"""
    return ctl.board1.displacement_to_flypt_16bit(float(disp_mm))


def build_frame(disp_mm):
    f = bytearray([255, 255])
    for i in range(6):
        v = mm_to_16bit(disp_mm[i])
        f += bytes([(v >> 8) & 0xFF, v & 0xFF])
    return bytes(f)


# ---------- 串口收发线程 ----------
ser = None
rx_thread = None
rx_buf = b''


def detect_serial():
    ports = sorted(glob.glob('/dev/ttyUSB*') + glob.glob('/dev/ttyACM*') +
                   glob.glob('/dev/cu.usbmodem*') + glob.glob('COM*'))
    return ports


def rx_loop():
    """读串口状态帧 '$cur..,tgt..,limit,enabled,wd\\n'。"""
    global rx_buf
    import serial as pyserial
    while ser and ser.is_open:
        try:
            chunk = ser.read(ser.in_waiting or 1)
        except Exception:
            break
        if not chunk:
            time.sleep(0.005)
            continue
        rx_buf += chunk
        while b'\n' in rx_buf:
            line, rx_buf = rx_buf.split(b'\n', 1)
            parse_status(line)


def parse_status(line):
    try:
        s = line.decode('ascii', 'ignore').strip()
        if not s.startswith('$'):
            return
        parts = s[1:].split(',')
        if len(parts) < 15:
            return
        cur = [int(float(x)) for x in parts[0:6]]
        tgt = [int(float(x)) for x in parts[6:12]]
        limit = int(parts[12]); enabled = int(parts[13]); wd = int(parts[14])
        # 计算缸速(mm/s)：以 20Hz 状态更新近似
        now = time.time()
        dt = now - state.get('_last_status_t', now)
        state['_last_status_t'] = now
        if dt > 0:
            speed = [abs(steps_to_mm(cur[i]) - steps_to_mm(state['cur'][i])) / dt
                     for i in range(6)]
        else:
            speed = [0] * 6
        state['cur'] = cur
        state['tgt'] = tgt
        state['limit'] = limit
        state['enabled'] = enabled
        state['watchdog'] = wd
        state['speed'] = [round(x, 1) for x in speed]
        if state['recording']:
            state['log'].append([round(time.time() - state['t0'], 3)] +
                                [steps_to_mm(x) for x in cur] +
                                [steps_to_mm(x) for x in tgt] + [limit, enabled, wd])
    except (ValueError, IndexError) as e:
        pass


def connect_serial(port, baud):
    global ser, rx_thread
    import serial as pyserial
    ser = pyserial.Serial(port, baudrate=baud, bytesize=8, parity='N', stopbits=1, timeout=0.1)
    state['connected'] = True
    state['mode'] = 'serial'
    rx_thread = threading.Thread(target=rx_loop, daemon=True)
    rx_thread.start()
    return True


def disconnect_serial():
    global ser
    state['connected'] = False
    if ser and ser.is_open:
        ser.close()
    ser = None


def send_pending():
    """主线程把 pending 位移通过串口发给主控。"""
    if state['pending'] is None:
        return
    disp = state['pending']
    state['pending'] = None
    frame = build_frame(disp)
    if state['mode'] == 'serial' and ser and ser.is_open:
        ser.write(frame)
    else:
        # 模拟模式：直接更新 cur/tgt
        tgt_steps = [int(np.clip(d / STEP_MM + CENTER, 0, MAX_POS)) for d in disp]
        state['tgt'] = tgt_steps
        state['cur'] = tgt_steps[:]  # 模拟瞬时到位
        state['enabled'] = 1
        state['watchdog'] = 0


# ---------- HTTP + WebSocket ----------
async def handler(websocket):
    # 周期推送状态前端（20Hz）
    async def push():
        while True:
            try:
                await websocket.send(json.dumps(state_snapshot()))
            except Exception:
                break
            await asyncio.sleep(0.05)
    pusher = asyncio.create_task(push())
    try:
        while True:
            try:
                msg = await websocket.recv()
            except Exception:
                break
            try:
                data = json.loads(msg)
            except Exception:
                continue
            cmd = data.get('cmd')
            if cmd == 'get':
                await websocket.send(json.dumps(state_snapshot()))
            elif cmd == 'pose':
                disp = data.get('disp')
                if disp and len(disp) == 6:
                    state['pending'] = [float(x) for x in disp]
            elif cmd == 'preset':
                name = data.get('name')
                presets = {
                    'neutral': [0] * 6,
                    'heave_up': [0, 0, 30, 0, 0, 0],
                    'heave_dn': [0, 0, -30, 0, 0, 0],
                    'roll_l': [0, 0, 0, 5, 0, 0],
                    'roll_r': [0, 0, 0, -5, 0, 0],
                    'pitch_up': [0, 0, 0, 0, 5, 0],
                    'pitch_dn': [0, 0, 0, 0, -5, 0],
                    'yaw_l': [0, 0, 0, 0, 0, 6],
                    'yaw_r': [0, 0, 0, 0, 0, -6],
                }
                if name in presets:
                    state['pending'] = presets[name][:]
            elif cmd == 'connect':
                port = data.get('port')
                baud = int(data.get('baud', 115200))
                if port == '__SIM__':
                    disconnect_serial()
                    state['mode'] = 'simulate'
                    state['connected'] = True
                    state['error'] = ''
                else:
                    try:
                        connect_serial(port, baud)
                        state['error'] = ''
                    except Exception as e:
                        state['error'] = str(e)
            elif cmd == 'disconnect':
                disconnect_serial()
            elif cmd == 'record_start':
                state['log'] = []
                state['recording'] = True
            elif cmd == 'record_stop':
                state['recording'] = False
                save_log()
            elif cmd == 'estop':
                state['pending'] = [0] * 6
    finally:
        pusher.cancel()


def state_snapshot():
    d = {k: v for k, v in state.items()
         if k not in ('log', 'pending', '_last_status_t')}
    d['cur_mm'] = [round(steps_to_mm(x), 2) for x in state['cur']]
    d['tgt_mm'] = [round(steps_to_mm(x), 2) for x in state['tgt']]
    return d


def save_log():
    if not state['log']:
        return
    path = os.path.join(HERE, 'output')
    os.makedirs(path, exist_ok=True)
    out = os.path.join(path, 'debug_panel_log.csv')
    header = 't,' + ','.join('cur%d' % i for i in range(6)) + \
             ',' + ','.join('tgt%d' % i for i in range(6)) + ',limit,enabled,wd'
    arr = np.array(state['log'])
    np.savetxt(out, arr, delimiter=',', header=header, comments='')
    print('[record] saved %s (%d rows)' % (out, len(arr)))


async def http_handler(reader, writer):
    """简单静态文件服务 + 端口列表。"""
    try:
        req = await asyncio.wait_for(reader.read(2048), timeout=2.0)
        line = req.decode('utf-8', 'ignore').split('\r\n')[0]
        if 'GET /ports' in line:
            body = json.dumps({'ports': detect_serial()}).encode()
            ctype = 'application/json'
        else:
            html_path = os.path.join(HERE, 'debug_panel.html')
            with open(html_path, 'rb') as f:
                body = f.read()
            # 注入实际 WebSocket 端口
            ws_port = HTTP_PORT + 1
            body = body.replace(b"const WS = 'ws://' + location.hostname + ':8766';",
                                ("const WS = 'ws://' + location.hostname + ':%d';" % ws_port).encode())
            ctype = 'text/html; charset=utf-8'
        writer.write(('HTTP/1.1 200 OK\r\nContent-Type: %s\r\nContent-Length: %d\r\n'
                      'Access-Control-Allow-Origin: *\r\n\r\n' % (ctype, len(body))).encode())
        writer.write(body)
    except Exception:
        pass
    finally:
        writer.close()


async def main_async(args):
    global HTTP_PORT
    HTTP_PORT = args.port
    # HTTP 服务
    http_server = await asyncio.start_server(http_handler, '0.0.0.0', args.port)
    # WebSocket 服务（用 8765+1）
    import websockets
    ws_port = args.port + 1
    ws_server = await websockets.serve(handler, '0.0.0.0', ws_port)
    print('调试面板: http://localhost:%d  (WebSocket :%d)' % (args.port, ws_port))

    if args.simulate:
        state['mode'] = 'simulate'
        print('模拟模式（无硬件）')
    elif args.serial:
        connect_serial(args.serial, args.baud)
    else:
        ports = detect_serial()
        if ports:
            connect_serial(ports[0], args.baud)
        else:
            state['mode'] = 'simulate'
            print('未检测到串口，进入模拟模式')

    # 周期发送 pending
    async def send_loop():
        while True:
            send_pending()
            await asyncio.sleep(0.05)
    asyncio.create_task(send_loop())

    await asyncio.gather(http_server.wait_closed(), ws_server.wait_closed())


def main():
    ap = argparse.ArgumentParser(description='ESP32 主控调试面板')
    ap.add_argument('--port', type=int, default=8765)
    ap.add_argument('--serial', default=None, help='串口设备路径')
    ap.add_argument('--baud', type=int, default=115200)
    ap.add_argument('--simulate', action='store_true', help='模拟模式(无硬件)')
    args = ap.parse_args()
    asyncio.run(main_async(args))


if __name__ == '__main__':
    main()
