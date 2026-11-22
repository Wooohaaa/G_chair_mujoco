# G_chair 实机通信：两块 Arduino、职责、游戏机接入、单板替代

本文所有结论来自工程内实际文件：

- 固件：`斯图尔特6轴平台/第二步-控制器/19厘米行程代码/servoController/servoController.ino`
- FlyPT 配置：`斯图尔特6轴平台/第一步-软件/斯图尔特6轴-动态2.Mover`（两个 `<OUTPUT>` 块）
- 单板预留：`sim/G_chair_esp32/include/config.h`

---

## 1. 两块 Arduino 如何互相通讯？

**答案：它们完全不通讯，两块板之间没有任何连线。**

`servoController.ino` 全程只有 `Serial.read()`（接收来自 PC 的串口数据）和 4 个硬件定时器输出脉冲，**没有任何 UART/I2C/SPI/无线收发代码**。两板是各自独立、地位对等的"哑从机"。

真正的"协调者"是运行 FlyPT Mover 的 PC。Mover 配置里有两个独立的串口输出：

| 输出名 | 发送字符串 | 串口参数 |
|---|---|---|
| `1号板` | `<255><255><Axis1a><Axis2a><Axis3a><Axis4a>` | 115200, 8N1, Binary |
| `2号板` | `<255><255><Axis5a><Axis6a><Axis4a><Axis3a>` | 115200, 8N1, Binary |

两块板各插一根 USB 线到 PC 的两个 COM 口，PC 分别给每块板发一帧。**PC 才是大脑，板子只负责执行。**

> 注意：如果两块板挂在同一根串口总线上，它们会收到完全相同的帧、做完全相同的动作（固件没有地址过滤），所以必须是**两个独立串口**。

---

## 2. 两块板各负责什么？

两块板刷的是**同一份固件**，职责由"收到的帧里放了哪几个轴"决定：

- **1 号板**：驱动电缸 Axis1a / Axis2a / Axis3a / Axis4a（4 个伺服，占满 4 路定时器）
- **2 号板**：帧里是 Axis5a / Axis6a / Axis4a / Axis3a，**实际接线通常只用前 2 路**（Axis5/Axis6），后两个值发了但对应输出没接电缸。你实际机器如果接了 4 路，请按接线为准。

每块板内部做 4 件事：

1. **收帧**：等连续两个 `255`，再收 4×16-bit（大端，`read()<<8 | read()`）。
2. **映射行程**：`mapToRange(pos) = pos / 1.7`，把 FlyPT 的 0~65535 映射到 0~`maxPos=38400` 步（总行程 192mm）。中位 32768 → 约 19200 步。
3. **发脉冲**：`move0..move3` 根据 `目标-当前` 的差值设定定时器周期 `4000 / |差值|`（钳位 39~4000，即 4ms 窗口内发多少脉冲）——**离目标越远跑得越快，越近越慢**，天然减速，无加速度规划。
4. **计数到位**：`ISR(TIMER1/3/4/5_COMPA)` 每个脉冲 `currentPos += incPos`，到达目标或收到新目标就停定时器。

引脚（MEGA2560 扩展板需焊排针）：

| 功能 | 引脚 |
|---|---|
| STEP 0~3 | 12, 2, 7, 45（OCR1B/3B/4B/5B） |
| DIR 0~3 | 30, 31, 32, 33（PORT C 位操作） |

**没有位置反馈、没有限位开关、没有急停**（开环）。抗干扰建议：RESET 与 GND 间接 10µF 电容——但接上后无法再刷固件，重刷时要拆板。

> `控制器（嘉立创打板）/全伺服通用版--网口（已验证）/` 里的"网口"指的是**伺服驱动器的网线线序**（差分信号），不是 PC 的以太网通信，别混淆。

---

## 3. 游戏电脑如何连到开发板传递运动数据？

### 方案 A（原方案）：游戏 PC 直接跑 FlyPT Mover

```
游戏 PC ──USB×2──> 1号板 + 2号板
   └─ FlyPT Mover: 遥测→6DOF→滤波→IK→6轴位移→两帧二进制
```
游戏和 FlyPT 在同一台 Windows 机器上，Mover 直接把两帧发到两个 COM 口。这是工程原本的用法。

### 方案 B（你现在的跨机方案）：游戏 PC → 仿真 PC → 串口

```
游戏PC --UDP 25浮点--> 仿真PC --serial_bridge.py--> COM1(1号板) + COM2(2号板)
```

桥接程序 `serial_bridge.py` 现在调用**仿真与实机共用的 `motion_pipeline.py`**：默认用修好的加速度驱动映射（roll←accel_x，左右对称，和 MuJoCo 仿真完全一致），可用 `--flypt` 切回原始 FlyPT 链路对比。输出帧和 FlyPT/Mover 逐字节一致（已验证：帧长 10 字节、`ff ff` 开头、16-bit 解码回 ±95mm 正确）。

```bash
# 默认 (推荐): 与仿真同一套对称映射
./.venv/bin/python serial_bridge.py --single-board --port1 /dev/ttyUSB0 --verbose
# 切回原始 FlyPT 映射对比
./.venv/bin/python serial_bridge.py --single-board --flypt --port1 /dev/ttyUSB0
```

```bash
# 1) 装依赖
sim/G_chair_mujoco/.venv/bin/pip install pyserial

# 2) 游戏机跑 GTA5 的 UDP 发送脚本（见 G_chair_gta5/README.md）

# 3) 仿真机跑桥接（先 dry-run 验证，再真发）
cd sim/G_chair_hw
../G_chair_mujoco/.venv/bin/python -u serial_bridge.py --dry-run --verbose
../G_chair_mujoco/.venv/bin/python -u serial_bridge.py \
    --port1 /dev/ttyUSB0 --port2 /dev/ttyUSB1 --verbose
```

关键点：
- 串口必须 **115200, 8N1, Binary**，与固件一致。
- 两块板必须接**两个独立串口**（CH340/CH341 驱动见工程内压缩包）。
- 所有 GND 共地；长线/干扰环境按工程说明加抗干扰电容。
- 断流超过 `--stale`（默认 0.5s）自动发中位帧，平台平滑回中，不会冻结。
- 仿真 PC 必须物理靠近平台（USB 线长度有限）。若距离远，把桥接放到平台旁的迷你主机/树莓派上，通过网络收 UDP。

---

## 4. 将来用一块开发板代替这两块，怎么做？

工程已经预留了单板配置：`sim/G_chair_esp32/include/config.h`（ESP32-S3，只有配置、还没有主程序）。

| 资源 | 引脚 |
|---|---|
| 6× STEP | 4, 5, 6, 7, 15, 16 |
| 6× DIR | 17, 18, 8, 9, 10, 11 |
| ENABLE | 12 |
| 6× 限位 | 13, 14, 21, 47, 48, 38 |
| 步进 | 40 kHz DDS 时基，单轴最高 15 kHz |

迁移要做 4 件事：

1. **协议合并**：把"两个串口各 4 轴"改成"一个通道收 6 轴"。最省事的做法是保留 PC 做 IK，只改帧格式——用
   `serial_bridge.py --single-board --port1 /dev/ttyUSB0`，发 `<255><255>` + 6×16-bit。
   固件里把现有的 4 路 `move0..3` 复制成 6 路，每路用自己的 STEP/DIR 引脚。

2. **脉冲发生改造**：Mega2560 靠 4 个 16-bit 定时器；ESP32 没有同样的外设，改用
   `hw_timer_t` 中断 + 直接写 GPIO（或 RMT/ledc）实现 `config.h` 里的 40kHz DDS 相位累加，
   保留 `4000/|差值|` 的减速规律。

3. **补齐缺失的安全逻辑**（原固件没有，单板必须加）：
   - 上电默认 **ENA 禁止输出**，收到第一帧合法数据才使能；
   - 6 路限位开关触发立即停对应轴；
   - 看门狗：超过 0.5s 无有效帧 → 全部停或回中位；
   - 硬件急停按钮直接切断使能。

4. **电气匹配**：ESP32 是 3.3V 逻辑，若伺服驱动 STEP/DIR 要求 5V，需加电平转换（如 74HCT245）；
   务必共地，STEP/DIR 线用双绞屏蔽线。

**推荐迁移路径**：先不改上位机，只把 ESP32 当成"一块能 drive 6 轴的板"，用 `--single-board` 验证 6 路脉冲正确；确认机械动作无误后，再考虑把 UDP+滤波+IK 整个搬进 ESP32（那样可以完全甩掉 PC，但要把 Python 的姿态映射/滤波/IK 用 C++ 重写，工作量大得多，不建议一开始做）。

> ⚠️ 我还没写 ESP32 主程序（`main.cpp`），`config.h` 只是引脚/参数定义。上面是完整实施步骤；如果你要我现在把带限位/看门狗的 6 轴固件骨架写出来，说一声即可——但真实电机接线前务必先断开电缸空载测试。
