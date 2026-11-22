import asyncio
import websockets
import json
import threading
import time
import numpy as np
import sys
import os

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
    'displacements': [0] * 6,
    'targetDisplacements': [0] * 6,
    'actualDisplacements': [0] * 6,
    'arduino_positions': [0] * 6,
    'pwm_freqs': [0] * 8,
    'rig_params': {
        'r_base': RIG['r_base'],
        'r_plat': RIG['r_plat'],
        'leg_middle': RIG['leg_middle'],
        'height': RIG['height'],
        'range': RIG['range'],
    },
    'axis_names': ['Axis1a', 'Axis2a', 'Axis3a', 'Axis4a', 'Axis5a', 'Axis6a'],
    'base_points': [],
    'plat_points_neutral': [],
    'frame_count': 0,
}

pipeline = None
sim_thread = None
REALTIME_DT = 0.02  # 实时步长 20ms = 50Hz

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
        await asyncio.sleep(REALTIME_DT)

async def start_websocket_server():
    async with websockets.serve(handle_websocket, "localhost", 8766):
        await asyncio.Future()

# 启动 WebSocket 服务器线程
websocket_server_thread = threading.Thread(target=lambda: asyncio.run(start_websocket_server()))
websocket_server_thread.daemon = True
websocket_server_thread.start()

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
        if sim_state.get('reset'):
            sim_state['reset'] = False
            pipeline = SimulationPipeline()
            step_count = 0
            last_time = time.time()

        pipeline.run_step()
        step_count += 1

        # 更新状态
        history = pipeline.history
        idx = len(history['time']) - 1
        sim_state['t'] = history['time'][idx]
        sim_state['telemetry'] = {k: float(v) for k, v in history['telemetry'][idx].items()}
        sim_state['pose_raw'] = {k: float(v) for k, v in history['pose_raw'][idx].items()}
        sim_state['pose_filtered'] = {k: float(v) for k, v in history['pose_filtered'][idx].items()}
        sim_state['displacements'] = [float(x) for x in history['displacements'][idx]]
        sim_state['targetDisplacements'] = [float(x) for x in history['targetDisplacements'][idx]]
        sim_state['actualDisplacements'] = [float(x) for x in history['actualDisplacements'][idx]]
        sim_state['arduino_positions'] = [float(x) for x in history['arduino_positions'][idx]]
        sim_state['pwm_freqs'] = [float(x) for x in history['pwm_freqs'][idx]]
        sim_state['frame_count'] += 1

if __name__ == "__main__":
    sim_thread = threading.Thread(target=sim_loop)
    sim_thread.start()
    sim_thread.join()