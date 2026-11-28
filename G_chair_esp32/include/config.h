#pragma once
#include <Arduino.h>

// ==================== ESP32-S3 单板替代双 Mega2560 ====================
// 一块 ESP32-S3 用 6 路 RMT 硬件产生步进脉冲，替代原来两块 Mega 的 4+2 轴。
// 协议保持与 servoController.ino 一致：<255><255> + 6×16-bit 大端
// （电脑端 serial_bridge.py --single-board 发这个格式）。

#define SERIAL_BAUD         115200
#define FRAME_HEADER_BYTE   0xFF
#define NUM_AXES            6

// 固件版本：串口监视器发 #V 可确认烧录的是哪一版（排查刷错固件/旧换算常数）
#define FW_VERSION          "1.2.0"

// ==================== 机械与行程参数（与 servoController.ino 一致）====================
#define XINGCHENG_MM        192.0f
#define MAX_POS_STEPS       38400L      // 4ms 窗口内的步数上限
#define CENTER_POS_STEPS    19200L
// 修正原 .ino 的 1.7 近似误差：1.7 会让 0mm(16bit=32768) → 19275 步 = +0.37mm 偏置，
// 且 ±95mm 顶到 ±96mm 钳位。精确除数 = 65535/38400 = **1.706640625**（务必写全精度）：
// 历史上误写成 1.7078125（那不是 65535/38400），会造成中位 -0.065mm、满行程 -0.13mm 偏差。
// 正确值使 0mm→19200 步、±95mm→±19200 步，与上位机 controller.py 完全一致。
#define MAP_DIVISOR         1.706640625f  // = 65535/38400 精确值
#define SAMPLE_PERIOD_US    4000UL      // 与 Mega 的 samplePeriod=4000 一致
#define MIN_PULSE_PERIOD_US 39UL
#define MAX_PULSE_PERIOD_US 4000UL

// ==================== 引脚（ESP32-S3-DevKitC-1）====================
// 避开 strapping: GPIO0/3/45/46；USB-CDC: GPIO19/20（保留给下载/日志）
static const uint8_t PIN_STEP[NUM_AXES] = { 4, 5, 6, 7, 15, 16 };
static const uint8_t PIN_DIR[NUM_AXES]  = { 17, 18, 8, 9, 10, 11 };
static const uint8_t PIN_LIMIT[NUM_AXES] = { 13, 14, 21, 47, 48, 38 };
#define PIN_ENABLE          12
#define ENABLE_ACTIVE_LOW   1           // 驱动器低电平使能

// ==================== 安全 ====================
#define WATCHDOG_MS         500         // 超过该时间无有效帧 -> 回中位
#define HOMING_SPEED_US     200         // 回中位时的脉冲周期 (越快周期越小)

// 可选：软件急停 GPIO（画板时预留）。默认关闭，避免未接线时误触发。
// 启用: platformio.ini 加 -DUSE_ESTOP_GPIO；急停按钮常闭，按下拉低此脚。
#define PIN_ESTOP           2
#define ESTOP_ACTIVE_LOW    1

// ==================== 伺服品牌 DB25 引脚定义（按工程网口线序图.pdf）====================
// 不同品牌伺服的 DB25 针脚不同。新板的 RJ45 线序统一， But 若用 DB25 直连
// 需按品牌接线。下面列出工程中已知的三种，供画板/接线时对照。
//
// 品牌        24V+  24V-  使能  脉冲+  方向+  脉冲-/方向-
// 东能 EPS    21    19    9     4      3      15/16
// 日鼎全系列  1     14    2     8      20     7/21
// 一川伺服    9     10    11    3      4      5/4
// 东菱 DS2P   (见工程 PDF 图片, 未录)
//
// 注意：新板输出的是【统一 RJ45 线序】(1=24V+ 2=24V- 3=使能 4=脉冲+ 5=方向+ 6/7=脉冲-/方向-)，
//       接不同品牌伺服时，按上表把 RJ45 的线序转接到对应 DB25 针脚（做一根转接线即可，
//       不需要改板）。若你想板上直接出 DB25，则按所选品牌的表格布线。
#define SERVO_BRAND_DEFAULT  "RIDING"    // 仅作文档标记；实际接线在板外完成
