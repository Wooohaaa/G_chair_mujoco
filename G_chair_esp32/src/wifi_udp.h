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
        uint8_t buf[64];
        int len = wifiUdp.read(buf, min(n, (int)sizeof(buf)));
        for (int i = 0; i < len; i++) parse_byte(buf[i]);
    }
}

#endif  // USE_WIFI
