# 一键上位机 host_app.py

把"游戏数据 → 平台动作"整条链路用一个程序串起来，带 **Web 实时监控 + 在线调参**。
替代原来分散的 `sim_udp.py` / `serial_bridge.py` / 手工改参数。

## 运行

```bash
cd sim
./G_chair_mujoco/.venv/bin/python host_app.py --mode sim           # 仿真（MuJoCo，无硬件）
./G_chair_mujoco/.venv/bin/python host_app.py --mode real --serial /dev/ttyUSB0   # 实机（ESP32）
./G_chair_mujoco/.venv/bin/python host_app.py --mode sim --mock    # 本机合成数据，先验证界面
```

浏览器打开 `http://localhost:8765`（WebSocket 8766）。

| 参数 | 默认 | 说明 |
|---|---|---|
| `--mode` | sim | `real`=串口发 ESP32；`sim`=MuJoCo |
| `--udp-port` | 4124 | 收游戏遥测的 UDP 端口 |
| `--serial` | 自动检测 | 实机串口 |
| `--port` | 8765 | Web HTTP 端口 |
| `--mock` | 关 | 本机合成遥测（无游戏机时验证用） |
| `--replay` | 无 | 从 `raw_telemetry.csv` 循环回放真实遥测，用于反复对比参数 |
| `--check` | 关 | 只做启动自检（依赖/映射/帧格式）后退出，不启动服务 |

## Web 界面功能

- **实时状态**：帧数、断流帧、串口连接、使能/看门狗/限位、6 缸位移与速度
- **6 缸曲线**：实时波形
- **在线调参**（实时生效，不用重启）：
  - 低通 tau（0~0.6s）——压尖峰，越大越平滑但延迟越高
  - 洗出 tau（0~8s）——恒定加速度后回中位的速度
  - 最大缸速（0~128mm/s）——保护电机，默认 110
  - **体感增益**：侧倾 roll（度/G）、俯仰 pitch（度/G）、偏航 yaw（度/(rad/s)）、横移 sway（mm/G）——实时生效，不用改代码
- **录制**：保存每帧缸位移/位姿/速度到 `output/host_app_log.csv`
- **复位滤波状态**：断流或调参后手动重置

## 数据链

```
GTA5 mod → UDP 25浮点 :4124 → UdpTelemetryReceiver
        → MotionPipeline(低通→加速度驱动映射→洗出→限速→IK)
        → 缸位移(mm)
           ├─ sim : sim.command_displacement() → MuJoCo
           └─ real: build_frame(<255><255>+6×16bit) → 串口 → ESP32
```

仿真与实机用**同一套** `MotionPipeline` 和**同一套**实物几何（PARAMS），
所以"仿真所见 = 实机所得"。

## 验证记录（2026-11-26）

用 `--mode sim --mock` 实测：

| 项 | 结果 |
|---|---|
| 启动 | 干净，HTTP 200，WS 端口正确注入 |
| 数据流 | 帧数持续增长，断流帧 1（启动瞬间） |
| 在线调参 | lpf 0.15→0.3、washout 2.5→3.5、max_speed 110→90 全部实时生效 |
| 录制 | 保存 15 行到 `output/host_app_log.csv`，状态正确复位 False |
| 在线增益 | roll 35→50、pitch 30→45、yaw 10→20、sway 100→150 实时生效 |
| 回放 | `--replay raw_telemetry.csv` 循环回放，帧数持续增长（144→520）、峰值位移 15.3mm、断流 2 |

## 运行日志分析（调参助手）

录制的 CSV 可以用分析工具量化评估，并给出具体调参建议：

```bash
./run.sh analyze output/host_app_log.csv
```

输出：行程占用/超程、缸速分布与限速帧、体感幅度（roll/pitch/yaw 峰值）、
左右对称性、以及"该加/减哪个增益"的建议。

比较两次调参的效果（例如改增益前后各录一段）：

```bash
./run.sh analyze --compare output/before.csv output/after.csv
```

并排显示缸位移 MAX、缸速 P95/P99、roll MAX 及变化量，用数据确认改进。
实测：roll MAX 从 1.33° → 2.66° 被正确检出。
实测 8 秒回放日志：400 帧、50Hz、缸速 P99=106mm/s（未顶限速）、
roll 峰值 1.33° → 建议加大 roll 增益。

## 与旧脚本的关系

| 旧脚本 | 现在 |
|---|---|
| `sim_udp.py` | 被 `host_app.py --mode sim` 取代（功能更全） |
| `serial_bridge.py` | 被 `host_app.py --mode real` 取代 |
| `debug_panel.py` | 手动测试面板，仍可单独用；`host_app` 覆盖其实时监控+调参功能 |

> 旧脚本保留作诊断/对比用，新链路统一走 `host_app.py`。
