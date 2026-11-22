# 双机仿真连接指南（游戏机 → 仿真机）

```
┌──────────────┐   UDP 25个空格分隔浮点数 :4124   ┌──────────────┐
│  游戏电脑     │ ────────────────────────────────> │  仿真电脑(本机) │
│  GTA5 故事模式 │  100Mbps/1Gbps 局域网             │  MuJoCo 仿真   │
│  SHVDN 脚本   │                                   │  10.148.82.176 │
└──────────────┘                                   └───────┬──────┘
                                                           │ 可选: USB串口
                                                           ▼
                                                  1号板 + 2号板 Mega2560
```

## 一、物理连接（二选一）

1. **推荐**：两台电脑都插同一个路由器/交换机的 LAN 口（同一网段，例如 `192.168.1.x` 或 `10.148.82.x`）。
2. **没有路由器**：用一根网线直连两机网卡（现代网卡支持自动翻转，无需交叉线）。此时两边需手动设同网段静态 IP，例如游戏机 `192.168.1.10`、仿真机 `192.168.1.20`，子网掩码 `255.255.255.0`。

先用 `ipconfig`（游戏机）和 `hostname -I`（仿真机）确认 IP 互相能 ping 通。

## 二、仿真电脑（接收端）设置

本机 IP 当前为 `10.148.82.176`。若换了网络，重新查：

```bash
hostname -I
```

开放 UDP 4124 入站（若 Ubuntu 开了 ufw）：

```bash
echo 100200 | sudo -S ufw allow 4124/udp
echo 100200 | sudo -S ufw status
```

启动仿真（等待游戏机发数据）：

```bash
cd sim/G_chair_mujoco
./.venv/bin/python sim_udp.py --realtime --plot --save-csv
# 看到 "[udp] 收到首帧! 开始驱动 MuJoCo" 即链路打通
```

## 三、游戏电脑（发送端）设置

GTA5 无官方遥测接口，用已写好的方案（详见 `G_chair_gta5/README.md`）：

1. 装 ScriptHookV + Community ScriptHookV .NET（故事模式，不能进 Online）。
2. 把 `GchairTelemetry.cs` 改名 `GchairTelemetry.3.cs` 放进 `GTA5根目录\scripts\`。
3. 改脚本顶部：

```csharp
private const string TargetIp = "10.148.82.176";  // 改成仿真机实际 IP
private const int TargetPort = 4124;
```

4. 进游戏故事模式 → 上任意车辆 → 开始发 UDP。

## 四、验证步骤（按顺序排错）

| 步骤 | 游戏机命令 | 仿真机命令 | 期望结果 |
|---|---|---|---|
| 1. 网络通 | `ping 10.148.82.176` | — | 有回复，<1ms |
| 2. 端口通 | 运行 `python udp_sender_test.py --ip 10.148.82.176` | `./.venv/bin/python -c "import socket;s=socket.socket(2,2);s.bind(('0.0.0.0',4124));print(s.recvfrom(1024)[0][:60])"` | 打印出一串浮点数 |
| 3. 仿真通 | 运行 GTA5 脚本 | `sim_udp.py --realtime` | `[udp] 收到首帧!`，收包数持续增长 |
| 4. 带实机 | — | `serial_bridge.py --port1 /dev/ttyUSB0 --port2 /dev/ttyUSB1` | 电缸按姿态动作 |

`udp_sender_test.py` 在 `sim/G_chair_sim/`，纯标准库，可先代替游戏验证链路。

## 五、如果要同时驱动真实平台

仿真机再开一个终端跑桥接（把收到的位姿转成两块板的串口帧）：

```bash
sim/G_chair_mujoco/.venv/bin/pip install pyserial
cd sim/G_chair_hw
../G_chair_mujoco/.venv/bin/python -u serial_bridge.py \
    --port1 /dev/ttyUSB0 --port2 /dev/ttyUSB1 --verbose
```

板子必须接**两个独立串口**、115200/8N1、共地。断流 >0.5s 自动回中位。

## 六、常见问题

| 现象 | 原因 / 解决 |
|---|---|
| 收不到包 | `TargetIp` 写错；两机不同网段；ufw/防火墙挡了；游戏机不在故事模式/没上车 |
| 收到包但不动 | 帧格式不符（必须是 25 个空格分隔浮点数）；`sim_udp.py` 的 `--port` 与发送端不一致 |
| 画面卡顿 | 用网线代替 WiFi；关掉 `--plot` 实时写盘 |
| 晃动大 | 遥测尖峰导致，对输入做 ~150ms 低通（见 `wobble_analysis.py` 结论） |
| 串口无反应 | 串口名不对（`ls /dev/ttyUSB*`）；CH340 驱动没装；两板挂了同一串口 |

## 数据格式速查（两端一致）

一条 UDP 报文 = 25 个空格分隔浮点数，顺序见 `sim/G_chair_sim/config.py` 的 `TELEMETRY_CHANNELS`：

```
accel_x accel_z accel_y speed_x speed_z speed_y rot_y rot_x rot_z
heading bank pitch airspeed gear_front gear_turn gear_left gear_right
altitude flaps gear speedbrakes canopy stall aoa time
```

仿真真正用到的只有 `accel_x/z/y`（横向/纵向/垂直加速度，单位 G）和 `rot_x/y/z`（roll/yaw/pitch 角速度，rad/s）。
