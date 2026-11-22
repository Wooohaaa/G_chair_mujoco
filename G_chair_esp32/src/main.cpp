/*
 * main.cpp — ESP32-S3 单板 Stewart 平台控制器（Arduino-ESP32 旧版 RMT API）
 *
 * 一块 ESP32-S3 用 6 路 RMT 硬件产生步进脉冲，替代原来两块 Mega2560。
 *
 * 协议: <255><255> + 6×uint16 大端（电脑端 serial_bridge.py --single-board）
 *   16-bit / 1.7 -> 目标步数 (0~38400)，中位 32768 -> 19200
 * 脉冲周期 = 4000 / |target-current| us，钳位 [39,4000]，与原固件减速曲线一致。
 *
 * 安全: 上电 ENA 禁止 -> 首帧到达才使能；限位触发停该轴；
 *       超过 500ms 无有效帧 -> 回中位。
 */
#include <Arduino.h>
#include <driver/rmt.h>
#include "config.h"

struct Stepper {
    rmt_channel_t chan;
    volatile long current = CENTER_POS_STEPS;
    volatile long target  = CENTER_POS_STEPS;
    volatile int  dir = 1;
    volatile bool running = false;
    volatile uint32_t period_us = SAMPLE_PERIOD_US;
    uint8_t pin_step, pin_dir;
    rmt_item32_t item;
};

static Stepper axes[NUM_AXES];

// 1MHz 分辨率 (80MHz/80) -> 1 tick = 1us, duration 值直接是微秒
static void set_period(Stepper &s, uint32_t period_us) {
    if (period_us < MIN_PULSE_PERIOD_US) period_us = MIN_PULSE_PERIOD_US;
    if (period_us > MAX_PULSE_PERIOD_US) period_us = MAX_PULSE_PERIOD_US;
    s.period_us = period_us;
    const uint32_t high = 5;
    const uint32_t low = period_us > high ? period_us - high : 1;
    s.item.level0 = 1;  s.item.duration0 = high;
    s.item.level1 = 0;  s.item.duration1 = low;
    if (s.running) {
        // 等 RMT 把新 symbol 加载完再返回，避免 DMA 尚未读取就被下次修改导致波形错乱
        rmt_write_items(s.chan, &s.item, 1, true);
    }
}

static void rmt_init_axis(int i) {
    Stepper &s = axes[i];
    s.chan = (rmt_channel_t)i;          // 0~5
    s.pin_step = PIN_STEP[i];
    s.pin_dir = PIN_DIR[i];
    pinMode(s.pin_dir, OUTPUT);
    digitalWrite(s.pin_dir, LOW);

    rmt_config_t cfg = {};
    cfg.rmt_mode = RMT_MODE_TX;
    cfg.channel = s.chan;
    cfg.gpio_num = (gpio_num_t)s.pin_step;
    cfg.mem_block_num = 1;
    cfg.clk_div = 80;                   // 1 MHz
    cfg.tx_config.loop_en = true;       // 无限循环发送
    cfg.tx_config.carrier_en = false;
    cfg.tx_config.idle_output_en = true;
    cfg.tx_config.idle_level = RMT_IDLE_LEVEL_LOW;
    ESP_ERROR_CHECK(rmt_config(&cfg));
    ESP_ERROR_CHECK(rmt_driver_install(s.chan, 0, 0));

    set_period(s, SAMPLE_PERIOD_US);
}

static void rmt_start(Stepper &s) {
    rmt_write_items(s.chan, &s.item, 1, true);    // 等加载完成；loop_en 自动循环
    s.running = true;
}

static void rmt_stop(Stepper &s) {
    rmt_tx_stop(s.chan);
    s.running = false;
}

// ---------------------- 位置控制 ----------------------
static void move_axis(int i, long target) {
    Stepper &s = axes[i];
    s.target = target;
    long diff = target - s.current;
    if (diff == 0) { if (s.running) rmt_stop(s); return; }

    s.dir = diff > 0 ? 1 : -1;
    digitalWrite(s.pin_dir, s.dir > 0 ? HIGH : LOW);

    long p = SAMPLE_PERIOD_US / labs(diff);
    if (p < (long)MIN_PULSE_PERIOD_US) p = MIN_PULSE_PERIOD_US;
    if (p > (long)MAX_PULSE_PERIOD_US) p = MAX_PULSE_PERIOD_US;
    set_period(s, (uint32_t)p);

    if (!s.running) rmt_start(s);
}

static void update_axis(int i, uint32_t dt_us) {
    Stepper &s = axes[i];
    if (!s.running) return;
    long step = (long)((float)dt_us / (float)s.period_us);
    if (step <= 0) return;
    if (s.dir > 0) {
        if (s.current + step >= s.target) { s.current = s.target; rmt_stop(s); return; }
        s.current += step;
    } else {
        if (s.current - step <= s.target) { s.current = s.target; rmt_stop(s); return; }
        s.current -= step;
    }
    // 越近越慢
    long diff = labs(s.target - s.current);
    long p = diff > 0 ? (long)(SAMPLE_PERIOD_US / diff) : MAX_PULSE_PERIOD_US;
    if (p < (long)MIN_PULSE_PERIOD_US) p = MIN_PULSE_PERIOD_US;
    if (p > (long)MAX_PULSE_PERIOD_US) p = MAX_PULSE_PERIOD_US;
    if (p != (long)s.period_us) set_period(s, (uint32_t)p);
}

static void enable_drivers(bool on) {
#if ENABLE_ACTIVE_LOW
    digitalWrite(PIN_ENABLE, on ? LOW : HIGH);
#else
    digitalWrite(PIN_ENABLE, on ? HIGH : LOW);
#endif
}

static void go_neutral() {
    for (int i = 0; i < NUM_AXES; i++) move_axis(i, CENTER_POS_STEPS);
}

// ---------------------- 串口 ----------------------
static uint16_t frame[NUM_AXES];
static int header_count = 0, byte_idx = 0;
static uint16_t acc = 0;
static unsigned long last_frame_ms = 0;
static bool enabled = false;

static void parse_byte(uint8_t b) {
    // 失步自恢复：若已收到帧头但超过 100ms 未凑满一帧，清零状态重新找帧头。
    // 防止字节丢失后状态机卡在 byte_idx 奇数态、后续帧头被当数据吞掉。
    static unsigned long frame_start_ms = 0;
    if (header_count > 0 || byte_idx > 0) {
        if (millis() - frame_start_ms > 100UL) {
            header_count = 0; byte_idx = 0;
        }
    }
    if (header_count < 2) {
        if (b == FRAME_HEADER_BYTE) {
            if (header_count == 0) frame_start_ms = millis();
            header_count++;
        } else { header_count = 0; byte_idx = 0; }
        return;
    }
    if (byte_idx % 2 == 0) acc = (uint16_t)b << 8;
    else {
        acc |= b;
        frame[byte_idx / 2] = acc;
        byte_idx++;
        if (byte_idx >= NUM_AXES * 2) {
            for (int i = 0; i < NUM_AXES; i++) {
                long target = (long)(frame[i] / MAP_DIVISOR);
                if (target > MAX_POS_STEPS) target = MAX_POS_STEPS;
                move_axis(i, target);
            }
            last_frame_ms = millis();
            if (!enabled) { enable_drivers(true); enabled = true; }
            header_count = 0; byte_idx = 0;
        }
    }
}

void setup() {
    Serial.begin(SERIAL_BAUD);
    pinMode(PIN_ENABLE, OUTPUT);
    enable_drivers(false);
    for (int i = 0; i < NUM_AXES; i++) {
        pinMode(PIN_LIMIT[i], INPUT_PULLUP);
        rmt_init_axis(i);
    }
    last_frame_ms = millis();
    Serial.println("ESP32-S3 6-axis ready: <255><255> + 6x16bit");
}

void loop() {
    // 限制每循环解析字节数，避免串口满速灌入时长时间阻塞限位检测/脉冲更新
    int max_bytes = 32;
    while (Serial.available() && max_bytes-- > 0) parse_byte((uint8_t)Serial.read());

    // 限位触发: 立即停该轴
    for (int i = 0; i < NUM_AXES; i++) {
        if (digitalRead(PIN_LIMIT[i]) == LOW && axes[i].running) {
            rmt_stop(axes[i]);
        }
    }

    static unsigned long last_tick = micros();
    unsigned long now = micros();
    uint32_t dt = (uint32_t)(now - last_tick);
    if (dt >= SAMPLE_PERIOD_US) {
        for (int i = 0; i < NUM_AXES; i++) update_axis(i, dt);
        last_tick = now;
    }

    if (enabled && millis() - last_frame_ms > WATCHDOG_MS) go_neutral();

    // 状态回传：每 50ms 发一帧文本，供调试面板实时显示。
    // 格式: $cur0,cur1,...,cur5,tgt0,...,tgt5,limitMask,enabled,watchdog\n
    //   cur/tgt 为步数(0~38400)；limitMask 6位， bit i=1 表示轴 i 限位触发；
    //   enabled 0/1；watchdog 0/1 表示当前处于看门狗回中状态。
    // 以 '$' 开头，与上位机发来的二进制指令帧 <255><255> 区分。
    static unsigned long last_status = 0;
    if (millis() - last_status >= 50) {
        last_status = millis();
        byte limitMask = 0;
        for (int i = 0; i < NUM_AXES; i++) {
            if (digitalRead(PIN_LIMIT[i]) == LOW) limitMask |= (1 << i);
        }
        byte wd = (enabled && (millis() - last_frame_ms > WATCHDOG_MS)) ? 1 : 0;
        Serial.print('$');
        for (int i = 0; i < NUM_AXES; i++) { Serial.print(axes[i].current); Serial.print(','); }
        for (int i = 0; i < NUM_AXES; i++) { Serial.print(axes[i].target);  Serial.print(','); }
        Serial.print(limitMask); Serial.print(',');
        Serial.print(enabled ? 1 : 0); Serial.print(',');
        Serial.print(wd);
        Serial.println();
    }
}
