# FlyPT Mover 游戏支持列表 & 数据来源说明（资料核查）

> 本文来自工程 `FlyPT软件-汉化BY开源Q群12908980/游戏数据输出设置.mhtml`（官方教程）的核查结果。
> **结论先行**：FlyPT 支持 40+ 游戏，但 **GTA5 不在其中**。我们现在的系统完全不依赖 FlyPT，
> 这份资料仅作参考，避免以后翻资料时误解。

---

## 1. FlyPT 支持的游戏（工程资料完整列表）

### 赛车（约占一半）
Assetto Corsa / Competizione、rFactor / rFactor 2、iRacing、Project CARS 1/2/3、
F1 2016 / 2019 / 2020、DiRT Rally / Rally 2 / 4、GRID / GRID 2、RaceRoom、Race07、
GT Legends、Live For Speed、KartKraft、Automobilista、BeamNG.drive、
Richard Burns Rally(NGP6 MOD)、No Limits 2、PiBoSo 系列（Kart Racing Pro / GP Bikes / MX Bikes / WRS）

### 卡车
European Truck Simulator 2 (ETS2)、American Truck Simulator (ATS)

### 飞行
DCS World、Microsoft Flight Simulator X、Flight Simulator 2020、Falcon 4.0 BMS、
Aerofly FS2、IL-2 Sturmovik、Condor 2、VTOL VR

### 其他
Epic Roller Coasters

---

## 2. 数据获取方式分三类（关键：是否"原生"）

### 类型 A：游戏原生 UDP 遥测（设置里开即可，半原生/全原生）

约一半游戏**自己就有 UDP 输出**，FlyPT 只是接收端：

| 游戏 | 怎么开 |
|---|---|
| F1 2019/2020 | 游戏菜单 → Settings → Telemetry → 开 UDP，send rate 60Hz |
| DiRT Rally 2 / DiRT 4 / Grid 2019 | 改 `hardware_settings.xml` 的 `<udp enabled="1" extradata="...">` |
| BeamNG.drive | 选项菜单里开 UDP，可调端口/更新率 |
| Condor 2 | 改 `Settings\UDP.ini`：Enabled=1, Port=55278, ExtendedData=1 |
| KartKraft | 游戏设置里开 UDP |
| Live For Speed | 改 `cfg.txt` |
| No Limits 2 | 启动参数 `--telemetry --telemetryport=15151` |
| PiBoSo 系列 | 改 `proxy_udp.ini`，需管理员运行 |
| Epic Roller Coasters | 启动参数 `-telemetry network -ip ... -port 7701` |
| IL-2 Sturmovik | 游戏目录配置开 UDP |
| Aerofly FS2 | 装 DLL 到 `external_dll`，广播 4321 端口 |

### 类型 B：MMF 共享内存 / DLL 插件（非原生，靠插件读内存）

这些游戏原生不给遥测，FlyPT 靠插件读游戏内存：

| 游戏 | 依赖 |
|---|---|
| ETS2 / ATS | `FlyPT_ETS2_AMTS_MMF.dll` 装到游戏 plugins 目录 |
| rFactor 2 | `FlyPT_RFactor2_MMF.dll` 装到 `Bin64\Plugins` |
| rFactor / ARCA / Stock Car Extreme / Automobilista / GTR2 等 isiMotor2 引擎 | 插件装到 Plugins 目录 |
| Assetto Corsa / iRacing / RaceRoom / Race07 / FSX / Falcon BMS | "Setup: Not required"——游戏本身有共享内存接口（AC shared memory 等），FlyPT 直接读 |

### 类型 C：Lua 脚本

| 游戏 | 依赖 |
|---|---|
| DCS World | `DCS_To_FlyPT_Mover.lua` 放进 DCS 的 `Scripts\Hooks`，脚本读数据发 UDP 4124 |
| Flight Simulator 2020 | 用微软 SimConnect 接口 |

---

## 3. 对我们系统的影响（重要）

**GTA5 不在这个列表里**——FlyPT 原生不支持 GTA5。这正是我们写 `GchairTelemetry.cs`
（ScriptHookV + SHVDN mod）的原因：自己从游戏读数据发 UDP。

所以：

| 我们系统的部分 | 与 FlyPT 的关系 |
|---|---|
| GTA5 数据源（`GchairTelemetry.cs`） | ❌ 无关，自研 mod |
| 姿态映射（`motion_pipeline.py`） | ❌ 无关，自研且修过左右对称 |
| 串口帧 / 主控 | ❌ 无关，协议格式兼容而已 |
| 调试面板 | ❌ 无关，自研替代 FlyPT 调试功能 |
| 那 40+ 游戏的支持资料 | ❌ 完全用不上 |

**结论：整套系统运行时不需要 FlyPT 软件，也不需要装任何 FlyPT 插件/DLL。
FlyPT 目录里唯一保留价值的是参数来源（Mover 配置文件 + 调教翻译.txt 的滤波器原理），
这些数值已固化进 `config.py` / `params.py`。**

---

## 4. 为什么我们不用 FlyPT 的"多游戏支持"

FlyPT 支持 40+ 游戏是靠**为每款游戏写适配插件**实现的。我们只用 GTA5，
只需要一个 mod（已写好）。如果将来要支持别的游戏，思路是：

- 游戏有原生 UDP → 写个 UDP 接收 + 字段映射到我们的 25 通道格式
- 游戏只有内存数据 → 写对应 mod/plugin 读内存发 UDP（就像 GTA5 的 SHVDN 脚本）

不需要 FlyPT 这个中间层。
