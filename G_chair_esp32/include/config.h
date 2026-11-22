#pragma once
#include <Arduino.h>

// ==================== ESP32-S3 单板替代双 Mega2560 ====================
// 一块 ESP32-S3 用 6 路 RMT 硬件产生步进脉冲，替代原来两块 Mega 的 4+2 轴。
// 协议保持与 servoController.ino 一致：<255><255> + 6×16-bit 大端
// （电脑端 serial_bridge.py --single-board 发这个格式）。

#define SERIAL_BAUD         115200
#define FRAME_HEADER_BYTE   0xFF
#define NUM_AXES            6

// ==================== 机械与行程参数（与 servoController.ino 一致）====================
#define XINGCHENG_MM        192.0f
#define MAX_POS_STEPS       38400L      // 4ms 窗口内的步数上限
#define CENTER_POS_STEPS    19200L
#define MAP_DIVISOR         1.7f        // 16-bit / 1.7 -> 步数
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
