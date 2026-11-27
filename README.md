# G_chair 六轴 Stewart 平台 · 仿真与实机控制

> 🗺️ **系统总览**：`SYSTEM_OVERVIEW.md` —— 整个工程的地图、快速开始、文档索引。
> 📋 **实机部署一页纸**：`QUICKSTART.md` —— 接线、烧录、自检、上手，照着做即可。
> 🔌 **协议规范**：`PROTOCOL.md` —— UDP/串口帧/状态帧格式、换算、时序（调试抓包对照）。
> 🚀 **一键启动**：`./run.sh --help`

用 MuJoCo 验证 6-DOF Stewart 平台的运动学、闭环机构、行程、奇异位形与控制跟踪，
并用同一套算法驱动真实的 ESP32-S3 主控（替代原两块 Mega2560）。

```
FlyPT / 游戏遥测 ──> 6DOF 位姿 ──> 滤波 ──> 逆运动学 IK ──> 6 缸位移 ──> MuJoCo 闭环平台
                                                              │
                                              实际平台位姿 <──┘ (FK 校验)
```

## 目录结构

| 文件/目录 | 说明 |
|---|---|
| **`host_app.py`** | **一键上位机**：GTA5 UDP → 映射/IK/限速 → 串口或 MuJoCo，带 Web 实时监控 + 在线调参（详见 `HOST_APP.md`） |
| `tests/test_chain.py` | 全链路回归测试（19 项，无需硬件） |
| `G_chair_mujoco/` | **主体**：MuJoCo 模型、IK/FK、场景驱动、查看/渲染、校验 |
| `G_chair_sim/` | 上游数据链：遥测生成 → 姿态映射 → FlyPT 滤波 → IK → 控制器/串口仿真 |
| `G_chair_gta5/` | GTA5 游戏机端 UDP 遥测发送脚本（ScriptHookV + SHVDN） |
| `G_chair_esp32/` | ESP32-S3 单板控制器固件（6 路 RMT 步进，替代双 Mega2560）。总体说明见 `REPLACING_DUAL_MEGA.md`，画板规格见 `BOARD_SPEC.md` |
| `motion_pipeline.py` | **仿真与实机共用的映射管线**：遥测→低通→加速度驱动映射→洗出→6DOF→IK→缸位移 |

### 仿真 = 实机：共用一套映射

`motion_pipeline.py` 是上位机核心算法的唯一权威实现，`sim_udp.py`（仿真/MuJoCo）和
`G_chair_hw/serial_bridge.py`（实机/串口）都调用它，**保证仿真所见 = 实机所得**。
默认用修好的"加速度驱动映射"（roll←accel_x，左右对称），可用 `--flypt`/`--no-accel-model`
切回原始 FlyPT 映射对比。主控（Arduino/ESP32）是纯执行层，不参与映射，无需改动。

### 缸速限制（保护主控/电机）

`motion_pipeline.py` 内置 `speed_limiter.py` 的**自适应限速器**：默认最大缸速 **110mm/s**
（主控硬件能力 128mm/s = `minPeriod 39µs → 25641Hz × 0.005mm/步`，留 14% 余量）。
它在 IK **之前**回拉位姿（不是独立限速 6 缸，那会破坏几何），只削掉超标的尖峰、
正常段零延迟。实测：峰值缸速 **1325 → 110mm/s，超主控能力帧 95 → 0**，
左右对称性保持。`--max-speed 0` 可关闭。对比图见
`G_chair_mujoco/output/real_gta5/speed_limit_compare.png`。

## 快速开始

```bash
cd G_chair_mujoco
python3 -m venv .venv
./.venv/bin/pip install -r requirements.txt

./.venv/bin/python sim.py --viewer                         # 交互查看器，播放演示动作
./.venv/bin/python sim.py --validate                       # 静态位姿/工作空间校验
./.venv/bin/python sim.py --scenario combined --plot --save-csv
```

## 跨电脑实时仿真（游戏在另一台机器）

游戏端发 UDP（25 个空格分隔浮点数，端口 4124），本机接收并驱动 MuJoCo：

```bash
cd G_chair_mujoco
./.venv/bin/python sim_udp.py --realtime --plot --save-csv
```

- 协议与工程自带的 `DCS_To_FlyPT_Mover.lua` 完全一致，见 `G_chair_sim/udp_telemetry.py`。
- `G_chair_sim/udp_sender_test.py` 是纯标准库的发送端模板，可先在本机环回验证链路。
- GTA5（故事模式）的安装与字段映射见 `G_chair_gta5/README.md`。
- 断流超过 `--stale`（默认 0.5 s）平台自动回中位。

## 晃动来源诊断

`G_chair_mujoco/wobble_analysis.py` 与 `stream_smooth_check.py` 用实验拆分了运行时晃动：

1. **主因：加速度→位移的纯比例映射 + 硬限幅**（`G_chair_sim/pose_from_motion.py`），
   没有 washout 高通，遥测突变会造成单步 15~17 mm 的缸指令跳变。
2. **次因：执行器欠阻尼**（`kp=80000, kv=2000`，ζ≈0.53）+ 软 `connect` 约束，
   30 mm 阶跃过冲 8.8%、振铃约 0.14 s。
3. **重力静下垂**：中位时平台低 1.4 mm（恒定偏压，非振荡）。

真实（经 ~150 ms 低通滤波的）赛车遥测下，位置跟踪 RMS 1.45 mm / MAX 3.67 mm，无超程。

## 许可

仅供个人学习与仿真验证。使用第三方游戏 mod 请遵守对应游戏与 mod 的条款。
