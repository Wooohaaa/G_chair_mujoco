#!/usr/bin/env python3
"""test_web.py — Web/WS 控制接口回归测试（手动姿态、调参、录制状态）。

启动一个临时 host_app实例（sim+mock），用 WebSocket 客户端验证：
  1. 状态推送持续（帧数增长）
  2. 预设手动姿态保持
  3. release 恢复遥测
  4. 参数下发生效
  5. 录制开始/停止状态正确
"""
import json
import os
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.join(HERE, '..')
PY = sys.executable
PORT = 8960
WS_PORT = PORT + 1

PASS = FAIL = 0


def check(name, cond, detail=''):
    global PASS, FAIL
    if cond:
        PASS += 1; print('  ✅ %s %s' % (name, detail))
    else:
        FAIL += 1; print('  ❌ %s %s' % (name, detail))


def main():
    import asyncio
    import websockets

    print('=' * 50)
    print('Web/WS 控制接口测试')
    print('=' * 50)
    # 备份 tuning.json（测试会改参数）
    tuning = os.path.join(ROOT, 'output', 'tuning.json')
    backup = None
    if os.path.exists(tuning):
        backup = open(tuning).read()

    proc = subprocess.Popen([PY, os.path.join(ROOT, 'host_app.py'),
                             '--mode', 'sim', '--mock', '--port', str(PORT)],
                            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                            cwd=ROOT)
    try:
        time.sleep(3)
        async def t():
            async with websockets.connect('ws://localhost:%d' % WS_PORT) as ws:
                async def drain(n=4):
                    out = []
                    for _ in range(n):
                        out.append(json.loads(await asyncio.wait_for(ws.recv(), 3)))
                    return out[-1]
                s = await drain(3)
                f0 = s['frames']
                check('状态推送', f0 > 0, '帧=%d' % f0)
                await ws.send(json.dumps({'cmd': 'preset', 'name': 'heave_up'}))
                vals = [(await drain(2))['disp_mm'][2] for _ in range(3)]
                check('手动保持 30mm', all(abs(v - 30) < 0.1 for v in vals),
                      str([round(v, 1) for v in vals]))
                await ws.send(json.dumps({'cmd': 'release'}))
                s = await drain(3)
                check('释放遥测', s['manual_disp'] is None)
                await ws.send(json.dumps({'cmd': 'param', 'lpf': 0.31, 'roll': 44}))
                s = await drain(4)
                check('参数下发', s['params']['lpf'] == 0.31 and s['params']['roll'] == 44.0,
                      'lpf=%s roll=%s' % (s['params']['lpf'], s['params']['roll']))
                await ws.send(json.dumps({'cmd': 'record_start'}))
                await asyncio.sleep(0.3)
                await ws.send(json.dumps({'cmd': 'record_stop'}))
                await asyncio.sleep(1.0)
                s = await drain(10)   # 多读几帧，确保拿到停止后的最新状态
                check('录制状态复位', s['recording'] is False,
                      'recording=%s' % s['recording'])
        asyncio.run(t())
    finally:
        proc.terminate()
        try:
            proc.wait(timeout=3)
        except Exception:
            proc.kill()
        if backup is not None:
            open(tuning, 'w').write(backup)
        elif os.path.exists(tuning):
            os.remove(tuning)   # 测试产生的 tuning.json 不留

    print('\n' + '=' * 50)
    print('结果: %d PASS, %d FAIL' % (PASS, FAIL))
    print('=' * 50)
    return 0 if FAIL == 0 else 1


if __name__ == '__main__':
    sys.exit(main())
