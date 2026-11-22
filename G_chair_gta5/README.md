# GTA5 → G_chair 遥测通信（游戏机端）

GTA5 没有官方遥测接口，必须靠 mod 从游戏内读取车辆状态。本目录给出一套**基于成熟生态 + 自写小脚本**的方案，直接输出本工程需要的 25 浮点 UDP 格式。

## 一、为什么推荐这套（而不是现成的第三方插件）

| 方案 | 成熟度 | 评价 |
|---|---|---|
| **ScriptHookV + Community SHVDN + 本脚本**（推荐） | ★★★★★ | ScriptHookV 是事实标准（v3889.0 / 2026-07，支持 1.0.3889.0 与 1.0.1158.13）；SHVDN 持续维护（v3.7.0-nightly.202 / 2026-11）。本脚本 ~130 行，协议完全可控，不依赖第三方中间件的调参。 |
| knaufinator/GTAV-Telemetry-Simtools-plugin | ★★ | 现成但仅 5 star、需装 SimTools 并用 tuner 设置 min/max，README 自称 UDP "often" 才中继，可靠性存疑。 |
| 内存直接读（Cheat Engine / 离线指针） | ★ | 游戏每次更新指针就失效，维护成本最高。 |

> ⚠️ ScriptHookV **不支持 GTA Online**（进入线上会被关闭游戏）。必须在**故事模式**下用。

## 二、安装步骤（游戏电脑上，Windows）

1. 确认游戏版本与 ScriptHookV 匹配（当前支持 1.0.3889.0 / 1.0.1158.13）。
2. 安装 **Microsoft .NET Framework 4.8+** 和 **Visual C++ 2019 x64 Redistributable**（Win10 通常已自带）。
3. 下载 [ScriptHookV](https://www.dev-c.com/gtav/scripthookv)，把 `ScriptHookV.dll` 和 `dinput8.dll`（ASI Loader）复制到 GTA5 根目录（与 `GTA5.exe` 同级）。若已有旧的 `dsound.dll` 先删掉。
4. 下载 [Community ScriptHookV .NET](https://www.gta5-mods.com/tools/scripthookv-net)，把 `ScriptHookVDotNet.asi`（或新版 `ScriptHookVDotNet-Loader.asi`）、`ScriptHookVDotNet.dll`、`ScriptHookVDotNet3.dll` 复制到 GTA5 根目录。
   - 若你游戏较新且用 add-on 类 mod，建议用 nightly build（注意新版把加载器改名成 `ScriptHookVDotNet-Loader.asi`，且 `.asi` 和 `.dll` 都要装）。
5. 用 Visual Studio（或 `csc`）把 `GchairTelemetry.cs` 编译成 `GchairTelemetry.dll`，放进 `GTA5根目录\scripts\`（没有就新建）。
   - 目标框架 .NET Framework 4.8，引用 `ScriptHookVDotNet3.dll`（v3 API）。
   - 图省事也可直接把 `GchairTelemetry.cs` 源码（改名 `GchairTelemetry.3.cs`）丢进 `scripts\`，SHVDN 会自动编译，但启动会慢几秒。
6. 修改脚本顶部 `TargetIp` 为仿真电脑的 IP（本机示例 `10.148.82.176`），`TargetPort` 保持 `4124`。
7. 进游戏故事模式 → 上任意车辆 → 脚本每个 tick 发一帧 UDP。

## 三、仿真电脑（本机）接收

```bash
cd sim/G_chair_mujoco
./.venv/bin/python sim_udp.py --realtime --plot --save-csv
```

看到 `[udp] 收到首帧!` 即链路打通。窗口标题/控制台会持续显示收包统计和跟踪误差。

## 四、字段映射（脚本 → 本工程通道）

| 通道 | 来源 | 说明 |
|---|---|---|
| 0 `accel_x` | 车体右向加速度 / 9.81 | 横向 G |
| 1 `accel_z` | 车体前向加速度 / 9.81 | 纵向 G（急刹/起步） |
| 2 `accel_y` | 车体竖向加速度 / 9.81 | 垂直 G（颠簸） |
| 6/7/8 `rot_y/x/z` | 四元数差分 → 车体轴角速度 | yaw / roll / pitch 速率 |
| 24 `time` | 脚本运行时间 | 计时 |

其余通道填 0（本工程的 `PoseFromMotion` 只用上面这些）。

## 五、调试与注意事项

- **收不到包**：先在游戏 PC 用 `udp_sender_test.py --ip <仿真机IP>` 验证链路；再确认 IP/端口、Windows 防火墙出站、两机同网段。
- **GTA 帧率不稳**：脚本用真实 `DateTime` 差分，接收端只取"最新一帧"，所以 30~60Hz 波动不影响；若想严格 50Hz，可在脚本里加发送节流。
- **晃动偏大**：GTA 的加速度尖峰（碰撞、撞墙）会被 `PoseFromMotion` 的硬限幅截断，仍可能出现一次性顿挫。按之前诊断，把遥测先做 ~150ms 低通（游戏端或 `sim_udp.py` 里）可把峰值误差从 ~15mm 降到 ~4mm。
- **下车/切人物**：脚本会清空差分状态，不会发出巨大加速度脉冲；仿真端超过 `--stale`（默认 0.5s）无数据会自动回中位。
