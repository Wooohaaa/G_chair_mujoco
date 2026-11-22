"""
realtime_sim.py — G_chair 实时仿真服务器
启动 HTTP 服务器, 实时运行仿真, 通过浏览器展示3D动画和数据流

运行: python realtime_sim.py
浏览器打开: http://localhost:8765
"""

import asyncio
import websockets
import json
import threading
import time
import numpy as np
import sys
import os
import http.server
import socketserver

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from config import RIG, SIM, SCENARIOS
from pipeline import SimulationPipeline

# ======================== 全局仿真状态 ========================
sim_state = {
    'running': False,
    't': 0.0,
    'scenario': '准备中',
    'scenario_idx': 0,
    'mode': 'SIM_ONLY',
    'telemetry': {},
    'pose_raw': {},
    'pose_filtered': {},
    'displacements': [0]*6,
    'targetDisplacements': [0]*6,
    'actualDisplacements': [0]*6,
    'arduino_positions': [0]*6,
    'pwm_freqs': [0]*8,
    'rig_params': {
        'r_base': RIG['r_base'],
        'r_plat': RIG['r_plat'],
        'leg_middle': RIG['leg_middle'],
        'height': RIG['height'],
        'range': RIG['range'],
    },
    'axis_names': ['Axis1a','Axis2a','Axis3a','Axis4a','Axis5a','Axis6a'],
    'base_points': [],
    'plat_points_neutral': [],
    'frame_count': 0,
}

pipeline = None
sim_thread = None
REALTIME_DT = 0.02  # 实时步长 20ms = 50Hz

# ======================== WebSocket 服务器 ========================
async def handle_websocket(websocket, path):
    global sim_state
    async for message in websocket:
        try:
            data = json.loads(message)
            if 'mode' in data:
                sim_state['mode'] = data['mode']
        except json.JSONDecodeError:
            pass

    while sim_state['running']:
        await websocket.send(json.dumps(sim_state))
        await asyncio.sleep(0.02)  # 50Hz

async def start_websocket_server():
    async with websockets.serve(handle_websocket, "localhost", 8766):
        await asyncio.Future()

# 启动 WebSocket 服务器线程
websocket_server_thread = threading.Thread(target=lambda: asyncio.run(start_websocket_server()))
websocket_server_thread.daemon = True
websocket_server_thread.start()

# ======================== HTTP 服务器 ========================
class SimHandler(http.server.SimpleHTTPRequestHandler):
    def do_GET(self):
        if self.path == '/' or self.path == '/index.html':
            self.serve_html()
        elif self.path == '/data':
            self.serve_data()
        elif self.path == '/reset':
            self.reset_sim()
        else:
            self.send_error(404, "Not Found")

    def serve_html(self):
        html_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'realtime_view.html')
        try:
            with open(html_path, 'rb') as f:
                content = f.read()
            self.send_response(200)
            self.send_header('Content-Type', 'text/html; charset=utf-8')
            self.send_header('Content-Length', len(content))
            self.end_headers()
            self.wfile.write(content)
        except FileNotFoundError:
            self.send_error(500, "HTML file not found")

    def serve_data(self):
        # Convert numpy arrays to lists for JSON serialization
        data = dict(sim_state)
        data['displacements'] = [float(x) for x in sim_state['displacements']]
        data['arduino_positions'] = [float(x) for x in sim_state['arduino_positions']]
        data['pwm_freqs'] = [float(x) for x in sim_state['pwm_freqs']]
        data['base_points'] = [
            [float(x) for x in p] for p in sim_state['base_points']
        ] if sim_state['base_points'] else []
        data['plat_points_neutral'] = [
            [float(x) for x in p] for p in sim_state['plat_points_neutral']
        ] if sim_state['plat_points_neutral'] else []

        # Clean up telemetry dict
        for key in ['telemetry', 'pose_raw', 'pose_filtered']:
            if isinstance(data.get(key), dict):
                data[key] = {
                    k: float(v) if isinstance(v, (int, float, np.floating)) else v
                    for k, v in data[key].items()
                    if not k.startswith('_')
                }

        json_str = json.dumps(data, ensure_ascii=False)
        body = json_str.encode('utf-8')
        self.send_response(200)
        self.send_header('Content-Type', 'application/json; charset=utf-8')
        self.send_header('Content-Length', len(body))
        self.send_header('Access-Control-Allow-Origin', '*')
        self.end_headers()
        self.wfile.write(body)

    def reset_sim(self):
        sim_state['reset'] = True
        self.send_response(200)
        self.send_header('Content-Type', 'application/json')
        self.end_headers()
        self.wfile.write(b'{"status":"reset_requested"}')

    def log_message(self, format, *args):
        pass  # 抑制日志


# ======================== 仿真循环 ========================
def sim_loop():
    global pipeline, sim_state

    pipeline = SimulationPipeline()
    sim_state['running'] = True

    # 初始化几何数据
    base_pts = pipeline.rig.get_base_points()
    sim_state['base_points'] = [[float(x) for x in p] for p in base_pts]
    plat_pts = pipeline.rig.plat_points
    sim_state['plat_points_neutral'] = [[float(x) for x in p] for p in plat_pts]

    step_count = 0
    last_time = time.time()

    while sim_state['running']:
        # 检查是否请求重置
        if sim_state.get('reset'):
            sim_state['reset'] = False
            pipeline = SimulationPipeline()
            step_count = 0
            last_time = time.time()

        # 运行仿真步
        pipeline.run_step()
        step_count += 1

        # 更新状态
        history = pipeline.history
        idx = len(history['time']) - 1

        sim_state['t'] = history['time'][idx]
        sim_state['scenario'] = history.get('scenario', '准备中')[idx] if 'scenario' in history else '准备中'
        sim_state['telemetry'] = {
            k: float(v) if isinstance(v, (int, float, np.floating)) else v
            for k, v in history['telemetry'][idx].items()
            if not k.startswith('_')
        }
        sim_state['pose_raw'] = {k: float(v) for k, v in history['pose_raw'][idx].items()}
        sim_state['pose_filtered'] = {k: float(v) for k, v in history['pose_filtered'][idx].items()}
        sim_state['displacements'] = [float(x) for x in history['displacements'][idx]]
        sim_state['targetDisplacements'] = [float(x) for x in history.get('targetDisplacements', [0] * 6)[idx]]
        sim_state['actualDisplacements'] = [float(x) for x in history.get('actualDisplacements', [0] * 6)[idx]]
        sim_state['arduino_positions'] = [float(x) for x in history['arduino_positions'][idx]]
        sim_state['pwm_freqs'] = [float(x) for x in history['pwm_freqs'][idx]]
        sim_state['frame_count'] = step_count

        # 实时 pacing
        elapsed = time.time() - last_time
        sleep_time = max(0, REALTIME_DT - elapsed)
        time.sleep(sleep_time)
        last_time = time.time()

        # 循环重置
        if sim_state['t'] >= SIM['duration']:
            sim_state['reset'] = True


# ======================== 启动 ========================
def start_server(port=8765):
    # 启动仿真线程
    sim_thread = threading.Thread(target=sim_loop, daemon=True)
    sim_thread.start()

    # 等待仿真初始化
    time.sleep(0.5)

    # 启动 HTTP 服务器
    handler = SimHandler
    try:
        httpd = socketserver.TCPServer(("", port), handler)
        print(f"\n{'='*60}")
        print(f"  G_chair 实时3D仿真已启动!")
        print(f"{'='*60}")
        print(f"  浏览器打开: http://localhost:{port}")
        print(f"  按 Ctrl+C 停止服务器")
        print(f"{'='*60}\n")
        httpd.serve_forever()
    except OSError as e:
        print(f"端口 {port} 被占用, 尝试 {port+1}...")
        start_server(port+1)


if __name__ == '__main__':
    # 启动仿真循环线程
    sim_thread = threading.Thread(target=sim_loop)
    sim_thread.start()
    
    # 启动 HTTP 服务器
    start_server()
