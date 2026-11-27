#pragma once
// WiFi UDP 接收（可选，USE_WIFI 编译）
// 收到 <255><255>+6×16bit 帧后，逐字节喂给 main.cpp 的 parse_byte()，
// 复用同一套解析/映射/限速逻辑，只是把"串口输入"换成"WiFi UDP 输入"。
#ifdef USE_WIFI

#include <WiFi.h>
#include <WiFiUdp.h>
#include "wifi_config.h"
#include "config.h"

extern void parse_byte(uint8_t b);  // main.cpp 提供

static WiFiUDP wifiUdp;
static IPAddress hostIP;          // 最近一次指令包的来源 IP（状态回传目标）
static bool haveHost = false;

void wifi_setup() {
    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    Serial.print("[wifi] 连接 ");
    Serial.println(WIFI_SSID);
    unsigned long t0 = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - t0 < WIFI_CONNECT_TIMEOUT) {
        delay(300);
        Serial.print('.');
    }
    if (WiFi.status() == WL_CONNECTED) {
        Serial.print("\n[wifi] 已连接 IP=");
        Serial.println(WiFi.localIP());
    } else {
        Serial.println("\n[wifi] 连接失败，可检查 wifi_config.h 的 SSID/密码");
    }
    wifiUdp.begin(WIFI_UDP_PORT);
    Serial.printf("[wifi] 监听 UDP %d\n", WIFI_UDP_PORT);
}

void wifi_loop() {
    int n = wifiUdp.parsePacket();
    if (n > 0) {
        hostIP = wifiUdp.remoteIP();   // 记住上位机地址，用于状态回传
        haveHost = true;
        uint8_t buf[64];
        int len = wifiUdp.read(buf, min(n, (int)sizeof(buf)));
        for (int i = 0; i < len; i++) parse_byte(buf[i]);
    }
}

// WiFi 模式状态回传：把 main.cpp 生成的 $ 帧发回最近的上位机 IP。
// 同一个 UDP socket 既收又发（上位机也在同端口收）。
void wifi_send_status(const char *frame, int len) {
    if (!haveHost || len <= 0) return;
    wifiUdp.beginPacket(hostIP, WIFI_UDP_PORT);
    wifiUdp.write((const uint8_t *)frame, len);
    wifiUdp.endPacket();
}

#endif  // USE_WIFI
