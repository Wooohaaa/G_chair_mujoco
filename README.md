# G_chair 六轴 Stewart 平台 · MuJoCo 仿真

用 MuJoCo 验证 6-DOF Stewart 平台的运动学、闭环机构、行程、奇异位形与控制跟踪，
不接触实物即可跑通整条数据链。

```
FlyPT / 游戏遥测 ──> 6DOF 位姿 ──> 滤波 ──> 逆运动学 IK ──> 6 缸位移 ──> MuJoCo 闭环平台
                                                              │
                                              实际平台位姿 <──┘ (FK 校验)
```

## 目录结构

| 目录 | 说明 |
|---|---|
| `G_chair_mujoco/` | **主体**：MuJoCo 模型、IK/FK、场景驱动、查看/渲染、校验 |
| `G_chair_sim/` | 上游数据链：遥测生成 → 姿态映射 → FlyPT 滤波 → IK → 控制器/串口仿真 |
| `G_chair_gta5/` | GTA5 游戏机端 UDP 遥测发送脚本（ScriptHookV + SHVDN） |

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
