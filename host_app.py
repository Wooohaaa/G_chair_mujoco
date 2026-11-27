#!/usr/bin/env python3
"""
host_app.py — G_chair 一键上位机（实机链路 / 仿真链路 统一入口）

把整条链路串起来，带 Web 实时监控 + 在线调参：
    GTA5 UDP(25浮点) → MotionPipeline(映射/滤波/IK/限速) → 输出
        --mode real : 串口帧 <255><255>+6×16bit → ESP32-S3 主控 → 电缸
        --mode sim  : MuJoCo 闭环平台（无需硬件）

运行:
  # 实机（接 ESP32 USB 串口）
  ./.venv/bin/python host_app.py --mode real --serial /dev/ttyUSB0
  # 仿真（无硬件）
  ./.venv/bin/python host_app.py --mode sim
  # 无游戏数据先验证界面
  ./.venv/bin/python host_app.py --mode sim --mock

  浏览器: http://localhost:8765   （WebSocket 8766）
"""
import argparse
import asyncio
import glob
import json
import os
import socket
import sys
import threading
import time

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

from motion_pipeline import MotionPipeline
from G_chair_sim.udp_telemetry import UdpTelemetryReceiver
from G_chair_sim.config import TELEMETRY_CHANNELS, SIM

DT = SIM['dt']

# ---------------- 全局状态 ----------------
state = {
    'mode': 'sim', 'connected': False, 't0': time.time(),
    'pose': [0.0] * 6,        # x,y,z(m) + r,p,y(rad)
    'disp_mm': [0.0] * 6,     # 6缸位移
    'speed': [0.0] * 6,
    'lost': 0, 'frames': 0,
    'hw': {'limit': 0, 'enabled': 0, 'watchdog': 0, 'frames': 0},
    'hw_last_status': 0.0,   # 最近一帧状态回传的时间，用于检测主控掉线
    'tele_stale': 0.0,       # 遥测数值停滞时长(秒)，检测游戏暂停/mod卡死
    'params': {},
    'recording': False,
    'manual_disp': None,   # 手动测试：设为 [6缸位移mm] 时绕过映射直接输出
}
_state_lock = threading.Lock()
_rec_log = []


def zero_telemetry():
    return dict.fromkeys(TELEMETRY_CHANNELS, 0.0)


# ---------------- 串口输出 + 状态回传 ----------------
ser = None
wifi_sock = None
_rx_buf = b''


def build_frame(disp_mm):
    from G_chair_sim.controller import DualBoardController
    ctl = DualBoardController()
    f = bytearray([255, 255])
    for d in disp_mm:
        v = ctl.board1.displacement_to_flypt_16bit(float(d))
        f += bytes([(v >> 8) & 0xFF, v & 0xFF])
    return bytes(f)


def wifi_status_rx_loop(port):
    """WiFi 模式：在同一 UDP 端口收 ESP32 的 $ 状态帧（与发送端口共用）。"""
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    try:
        s.bind(('0.0.0.0', port))
    except OSError:
        return
    s.settimeout(0.2)
    while True:
        try:
            data, _ = s.recvfrom(2048)
        except socket.timeout:
            continue
        except OSError:
            break
        for line in data.split(b'\n'):
            try:
                t = line.decode('ascii', 'ignore').strip()
                if not t.startswith('$'):
                    continue
                parts = t[1:].split(',')
                if len(parts) >= 15:
                    with _state_lock:
                        state['hw']['limit'] = int(parts[12])
                        state['hw']['enabled'] = int(parts[13])
                        state['hw']['watchdog'] = int(parts[14])
                        state['hw']['frames'] += 1
                        state['hw_last_status'] = time.time()
            except (ValueError, IndexError):
                pass


def status_rx_loop(port):
    global _rx_buf
    while port and port.is_open:
        try:
            chunk = port.read(port.in_waiting or 1)
        except Exception:
            break
        if not chunk:
            time.sleep(0.005)
            continue
        _rx_buf += chunk
        while b'\n' in _rx_buf:
            line, _rx_buf = _rx_buf.split(b'\n', 1)
            try:
                s = line.decode('ascii', 'ignore').strip()
                if not s.startswith('$'):
                    continue
                parts = s[1:].split(',')
                if len(parts) >= 15:
                    with _state_lock:
                        state['hw']['limit'] = int(parts[12])
                        state['hw']['enabled'] = int(parts[13])
                        state['hw']['watchdog'] = int(parts[14])
                        state['hw']['frames'] += 1
                        state['hw_last_status'] = time.time()
            except (ValueError, IndexError):
                pass


# ---------------- 数据链主循环（后台线程） ----------------
def pipeline_loop(args, motion, rx, sim):
    have_signal = False
    _last_sig = None
    next_t = time.time()
    with _state_lock:
        state['params'] = motion.get_params()

    while True:
        item = rx.get(max_age=args.stale)
        if item is None:
            if have_signal:
                motion.reset()
                have_signal = False
            _last_sig = None
            with _state_lock:
                state['tele_stale'] = 0.0
            tele = zero_telemetry()
            with _state_lock:
                state['lost'] += 1
        else:
            tele = item[0]
            have_signal = True
            # 检测遥测数值停滞（游戏暂停 / mod 卡死）：运动通道全相同则累加时长
            sig = tuple(round(float(tele.get(k, 0.0)), 4) for k in
                        ('accel_x', 'accel_z', 'accel_y', 'rot_y', 'rot_x', 'rot_z'))
            if sig == _last_sig:
                with _state_lock:
                    state['tele_stale'] += DT
            else:
                with _state_lock:
                    state['tele_stale'] = 0.0
            _last_sig = sig

        # 手动姿态测试优先：一旦设置就保持，直到 Web 点"恢复遥测"清空
        manual = state.get('manual_disp')
        if manual is not None:
            disp = np.array(manual, dtype=float)
            pose = np.zeros(6)
        else:
            pose, disp = motion.pose_and_disp(tele, DT)
        lim = np.clip(disp, -95, 95)

        if args.mode == 'real' and args.output == 'wifi' and wifi_sock is not None:
            wifi_sock.sendto(build_frame(lim), (args.esp32_ip, args.esp32_udp))
        elif args.mode == 'real' and ser and ser.is_open:
            ser.write(build_frame(lim))
        elif sim is not None:
            sim.command_displacement(lim, DT)
            sim.step()

        # 缸速
        with _state_lock:
            prev = state['disp_mm']
            speed = [abs(float(lim[i]) - prev[i]) / DT for i in range(6)]
            state['pose'] = [float(x) for x in pose]
            state['disp_mm'] = [float(x) for x in lim]
            state['speed'] = [round(x, 1) for x in speed]
            state['frames'] += 1
            if state['recording']:
                _rec_log.append([round(time.time() - state['t0'], 3)] +
                                [float(x) for x in lim] +
                                [float(x) for x in pose] + speed)

        next_t += DT
        time.sleep(max(0, next_t - time.time()))


# ---------------- Web 服务 ----------------
async def ws_handler(websocket):
    async def push():
        while True:
            try:
                with _state_lock:
                    snap = dict(state)
                    snap['hw'] = dict(state['hw'])
                    snap['params'] = dict(state['params'])
                    # 主控状态回传年龄（秒）：掉线时不再更新，前端据此告警
                    snap['hw_age'] = (round(time.time() - state['hw_last_status'], 2)
                                      if state['hw_last_status'] > 0 else -1)
                    snap['tele_stale'] = round(state['tele_stale'], 2)
                await websocket.send(json.dumps(snap))
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
                d = json.loads(msg)
            except Exception:
                continue
            cmd = d.get('cmd')
            if cmd == 'param':
                with _state_lock:
                    gains = {k: d[k] for k in ('roll', 'pitch', 'yaw', 'sway',
                                               'surge', 'heave') if k in d}
                    p = motion_ref.set_params(lpf=d.get('lpf'), washout=d.get('washout'),
                                              max_speed=d.get('max_speed'), gains=gains)
                    state['params'] = p
                    save_tuning(p)   # 调参自动持久化，重启后保留
            elif cmd == 'record_start':
                _rec_log.clear()
                with _state_lock:
                    state['recording'] = True
            elif cmd == 'record_stop':
                with _state_lock:
                    state['recording'] = False
                save_log()
            elif cmd == 'reset':
                with _state_lock:
                    motion_ref.reset()
            elif cmd == 'defaults':
                with _state_lock:
                    p = motion_ref.set_params(lpf=0.15, washout=2.5, max_speed=110.0,
                                              gains={'roll': 35.0, 'pitch': 30.0,
                                                     'yaw': 10.0, 'sway': 100.0})
                    state['params'] = p
                    save_tuning(p)
            elif cmd == 'preset':
                name = d.get('name')
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
                    with _state_lock:
                        state['manual_disp'] = presets[name][:]
            elif cmd == 'manual_mm':
                disp = d.get('disp')
                if disp and len(disp) == 6:
                    with _state_lock:
                        state['manual_disp'] = [float(x) for x in disp]
            elif cmd == 'estop':
                with _state_lock:
                    state['manual_disp'] = [0] * 6
            elif cmd == 'release':
                with _state_lock:
                    state['manual_disp'] = None
    finally:
        pusher.cancel()


def save_log():
    if not _rec_log:
        return
    # 支持环境变量覆盖输出路径（测试用，避免污染真实日志）
    path = os.environ.get('GCHAIR_LOG_PATH',
                          os.path.join(HERE, 'output', 'host_app_log.csv'))
    out_dir = os.path.dirname(os.path.abspath(path))
    os.makedirs(out_dir, exist_ok=True)
    header = (','.join(['t'] + ['leg%d' % i for i in range(6)] +
                       ['pose%d' % i for i in range(6)] +
                       ['speed%d' % i for i in range(6)]))
    np.savetxt(path, np.array(_rec_log), delimiter=',', header=header, comments='')
    print('[record] %s (%d rows)' % (path, len(_rec_log)))


async def http_handler(reader, writer):
    try:
        req = await asyncio.wait_for(reader.read(2048), timeout=2.0)
        line = req.decode('utf-8', 'ignore').split('\r\n')[0]
        if 'GET /ports' in line:
            body = json.dumps({'ports': sorted(
                glob.glob('/dev/ttyUSB*') + glob.glob('/dev/ttyACM*') +
                glob.glob('/dev/cu.usbmodem*'))}).encode()
            ctype = 'application/json'
        else:
            with open(os.path.join(HERE, 'host_app.html'), 'rb') as f:
                body = f.read()
            body = body.replace(b"':WS_PORT'",
                                ("':%d'" % (HTTP_PORT + 1)).encode())
            ctype = 'text/html; charset=utf-8'
        writer.write(('HTTP/1.1 200 OK\r\nContent-Type: %s\r\nContent-Length: %d\r\n'
                      'Access-Control-Allow-Origin: *\r\n\r\n' %
                      (ctype, len(body))).encode())
        writer.write(body)
    except Exception:
        pass
    finally:
        writer.close()


motion_ref = None
HTTP_PORT = 8765
TUNING_FILE = os.path.join(HERE, 'output', 'tuning.json')


def save_tuning(params):
    try:
        os.makedirs(os.path.dirname(TUNING_FILE), exist_ok=True)
        with open(TUNING_FILE, 'w') as f:
            json.dump(params, f, indent=2)
    except Exception as e:
        print('[tuning] 保存失败: %s' % e)


def load_tuning():
    if not os.path.exists(TUNING_FILE):
        return None
    try:
        with open(TUNING_FILE) as f:
            return json.load(f)
    except Exception:
        return None


async def main_async(args):
    global ser, wifi_sock, motion_ref, HTTP_PORT
    HTTP_PORT = args.port

    if args.output == 'wifi':
        wifi_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        print('[wifi] 输出到 udp://%s:%d' % (args.esp32_ip, args.esp32_udp))
        # WiFi 模式：在 esp32_udp 端口收 ESP32 的状态回传（与上位机发帧同端口）
        threading.Thread(target=wifi_status_rx_loop,
                         args=(args.esp32_udp,), daemon=True).start()
        with _state_lock:
            state['connected'] = True

    motion_ref = MotionPipeline(lpf_tau=args.lpf, washout_tau=args.washout,
                                max_speed=args.max_speed)
    saved = load_tuning()
    if saved:
        try:
            p = motion_ref.set_params(lpf=saved.get('lpf'), washout=saved.get('washout'),
                                      max_speed=saved.get('max_speed'), gains=saved)
            print('[tuning] 已从 %s 恢复参数: %s' % (os.path.basename(TUNING_FILE), p))
        except (TypeError, ValueError) as e:
            print('[tuning] 参数文件损坏，使用默认值: %s' % e)
    rx = UdpTelemetryReceiver(args.host, args.udp_port)
    rx.start()

    sim = None
    if args.mode == 'sim':
        from G_chair_mujoco.sim import GChairSim
        sim = GChairSim()
        print('[mode] 仿真 (MuJoCo)')

    if args.replay:
        # 从 raw_telemetry.csv 回放真实遥测（25通道），按 50Hz 发到 UDP
        def replay_loop():
            import csv
            path = args.replay
            with open(path) as fh:
                rows = list(csv.DictReader(fh))
            print('[replay] %s (%d 帧)' % (path, len(rows)), flush=True)
            while True:
                for row in rows:
                    payload = ' '.join('%.4f' % float(row.get(k, 0.0))
                                       for k in TELEMETRY_CHANNELS)
                    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                    s.sendto(payload.encode(), ('127.0.0.1', args.udp_port))
                    s.close()
                    time.sleep(0.02)
        threading.Thread(target=replay_loop, daemon=True).start()

    elif args.mock:
        # 用测试发送端，本机自环
        from G_chair_sim.udp_sender_test import make_frame
        def mock_loop():
            while True:
                tele = make_frame((time.time() - state['t0']) % 60)
                payload = ' '.join('%.4f' % tele.get(k, 0.0)
                                   for k in TELEMETRY_CHANNELS)
                s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                s.sendto(payload.encode(), ('127.0.0.1', args.udp_port))
                s.close()
                time.sleep(0.02)
        threading.Thread(target=mock_loop, daemon=True).start()
        print('[mock] 本机合成遥测')

    if args.mode == 'real' and args.output == 'serial':
        port = args.serial
        if not port:
            ports = sorted(glob.glob('/dev/ttyUSB*') + glob.glob('/dev/ttyACM*'))
            port = ports[0] if ports else None
        if port:
            try:
                import serial as pyserial
                ser = pyserial.Serial(port, baudrate=args.baud, bytesize=8,
                                      parity='N', stopbits=1, timeout=0.1)
                threading.Thread(target=status_rx_loop, args=(ser,), daemon=True).start()
                print('[serial] %s' % port)
                with _state_lock:
                    state['connected'] = True
            except Exception as e:
                print('[serial] 打开 %s 失败: %s — 继续运行但 Web 会告警未连接' % (port, e))
                ser = None
        else:
            print('[serial] 未找到串口，实机模式将只计算不发送（Web 会告警）')

    threading.Thread(target=pipeline_loop, args=(args, motion_ref, rx, sim),
                     daemon=True).start()

    http_server = await asyncio.start_server(http_handler, '0.0.0.0', args.port)
    import websockets
    ws_server = await websockets.serve(ws_handler, '0.0.0.0', args.port + 1)
    print('上位机就绪: http://localhost:%d (WebSocket :%d)' %
          (args.port, args.port + 1))
    await asyncio.gather(http_server.wait_closed(), ws_server.wait_closed())


def self_check(args):
    """启动自检：依赖/映射/换算/帧格式，打印健康状态。"""
    print('===== 上位机自检 =====')
    ok = True
    try:
        from motion_pipeline import MotionPipeline
        m = MotionPipeline()
        tele = {'accel_x': 0.5, 'accel_z': 0.3, 'accel_y': 0.1,
                'rot_y': 0.4, 'rot_x': 0.0, 'rot_z': 0.0}
        peak = 0.0
        for k in range(60):
            t = dict(tele); t['accel_x'] = 0.6 * np.sin(k * 0.1)
            _, d = m.pose_and_disp(t, 0.02)
            peak = max(peak, np.max(np.abs(d)))
        good = np.all(np.isfinite(d)) and peak <= 95 and peak > 1.0
        print('  映射管线: %s (变化输入峰值位移 %.1fmm)' % ('OK' if good else 'FAIL', peak))
        ok &= good
    except Exception as e:
        print('  映射管线: FAIL (%s)' % e); ok = False
    try:
        frame = build_frame([0] * 6)
        good = len(frame) == 14 and frame[0] == 255 and frame[1] == 255
        print('  指令帧: %s (14字节 FF FF 头)' % ('OK' if good else 'FAIL'))
        ok &= good
    except Exception as e:
        print('  指令帧: FAIL (%s)' % e); ok = False
    if args.mode == 'real':
        ports = sorted(glob.glob('/dev/ttyUSB*') + glob.glob('/dev/ttyACM*') +
                       glob.glob('/dev/cu.usbmodem*'))
        print('  串口设备: %s' % (ports if ports else '未检测到（实机请检查接线）'))
    print('======================')
    return ok


def main():
    ap = argparse.ArgumentParser(description='G_chair 一键上位机')
    ap.add_argument('--mode', choices=['real', 'sim'], default='sim')
    ap.add_argument('--host', default='0.0.0.0')
    ap.add_argument('--udp-port', type=int, default=4124)
    ap.add_argument('--serial', default=None)
    ap.add_argument('--baud', type=int, default=115200)
    ap.add_argument('--output', choices=['serial', 'wifi'], default='serial',
                    help='实机输出方式: serial=USB串口, wifi=UDP到ESP32')
    ap.add_argument('--esp32-ip', default='192.168.1.50', help='WiFi模式下ESP32的IP')
    ap.add_argument('--esp32-udp', type=int, default=4125, help='WiFi模式下ESP32的UDP端口')
    ap.add_argument('--port', type=int, default=8765)
    ap.add_argument('--stale', type=float, default=0.5)
    ap.add_argument('--lpf', type=float, default=0.15)
    ap.add_argument('--washout', type=float, default=2.5)
    ap.add_argument('--max-speed', type=float, default=110.0)
    ap.add_argument('--mock', action='store_true', help='本机合成遥测(无游戏机)')
    ap.add_argument('--replay', default=None,
                    help='从 raw_telemetry.csv 回放真实遥测(循环)')
    ap.add_argument('--check', action='store_true', help='只做启动自检后退出')
    args = ap.parse_args()
    if args.check:
        self_check(args)
    else:
        self_check(args)
        asyncio.run(main_async(args))


if __name__ == '__main__':
    main()
