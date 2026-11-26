#pragma once
// WiFi 配置模板（仅 USE_WIFI 编译时使用）
// 复制为 wifi_config_local.h 并填入你的网络信息，或直接改下面两行。
// 注意：不要把真实密码提交到公开仓库！建议用本地头文件 + .gitignore。

#ifndef WIFI_SSID
#define WIFI_SSID       "YOUR_WIFI_SSID"
#endif
#ifndef WIFI_PASSWORD
#define WIFI_PASSWORD   "YOUR_WIFI_PASSWORD"
#endif

// ESP32 监听的 UDP 端口（上位机往这个端口发 6 轴帧）
#ifndef WIFI_UDP_PORT
#define WIFI_UDP_PORT   4125
#endif

// WiFi 连接超时（ms），超时后退回 AP 配网模式
#ifndef WIFI_CONNECT_TIMEOUT
#define WIFI_CONNECT_TIMEOUT   15000
#endif
