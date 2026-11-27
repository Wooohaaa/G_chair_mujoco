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
#ifdef USE_WIFI
#include "wifi_udp.h"
#endif

struct Stepper {
    rmt_channel_t chan;
    volatile long current = CENTER_POS_STEPS;
    volatile long target  = CENTER_POS_STEPS;
    volatile int  dir = 1;
    volatile bool running = false;
    volatile uint32_t period_us = SAMPLE_PERIOD_US;
    uint8_t pin_step, pin_dir;
    rmt_item32_t item;
    // 限位锁存：+1=上限位触发(只许向下)，-1=下限位触发(只许向上)，0=正常
    int8_t limitLatch = 0;
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
        // 修 P0-1：loop 模式下 wait_tx_done=true 会等到"所有 item 发完"，
        // 而 loop_en=无限循环永远不会满足 -> 卡死。正确做法（参考 Arduino rmtWriteLooping）：
        // 先停 -> 用 wait=false 重新加载 -> 重启循环。
        rmt_tx_stop(s.chan);
        rmt_write_items(s.chan, &s.item, 1, false);
        rmt_tx_start(s.chan, true);   // true = loop 模式
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
    // 同样用 wait=false（非阻塞），显式以 loop 模式启动，避免 loop 模式下 wait=true 的死锁。
    rmt_write_items(s.chan, &s.item, 1, false);
    rmt_tx_start(s.chan, true);
    s.running = true;
}

static void rmt_stop(Stepper &s) {
    rmt_tx_stop(s.chan);
    s.running = false;
}

// ---------------------- 位置控制 ----------------------
static void move_axis(int i, long target) {
    Stepper &s = axes[i];
    // 限位互锁：锁存后只允许朝"离开限位"的方向运动；反向指令才解锁。
    // 例：上限位(+1)触发后，只接受 target <= current 的指令；一旦收到向下指令，
    // 先解锁再执行，避免持续顶死。
    if (s.limitLatch != 0) {
        long diff = target - s.current;
        if (s.limitLatch > 0 && diff > 0) return;   // 上限位：拒继续向上
        if (s.limitLatch < 0 && diff < 0) return;   // 下限位：拒继续向下
        s.limitLatch = 0;                            // 收到离开方向 → 解锁
    }
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
    for (int i = 0; i < NUM_AXES; i++) {
        axes[i].limitLatch = 0;   // 回中位时清除限位锁存，允许自由运动
        move_axis(i, CENTER_POS_STEPS);
    }
}

// ---------------------- 串口 ----------------------
static uint16_t frame[NUM_AXES];
static int header_count = 0, byte_idx = 0;
static uint16_t acc = 0;
static unsigned long last_frame_ms = 0;
static unsigned long manual_hold_until = 0;  // #J 手动点动保持截止时间(10s上限)
static bool enabled = false;

void parse_byte(uint8_t b) {   // wifi_udp.h 在 USE_WIFI 模式下 extern 引用
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
            manual_hold_until = 0;   // 上位机接管，退出手动点动保持
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
#ifdef USE_ESTOP_GPIO
    pinMode(PIN_ESTOP, INPUT_PULLUP);
#endif
#ifdef USE_WIFI
    wifi_setup();
    Serial.println("ESP32-S3 6-axis (WiFi UDP) ready");
#else
    Serial.println("ESP32-S3 6-axis (Serial) ready: <255><255> + 6x16bit");
#endif
    last_frame_ms = millis();
}

// 文本命令（调试用，'#' 开头，与二进制帧和 '$' 状态帧区分）
static String cmd_buf;
static bool selftest_active = false;
static int selftest_axis = 0, selftest_phase = 0;
static unsigned long selftest_t = 0;
static void handle_text_cmd(const String &c) {
    if (c == "#H") {
        Serial.println("CMD: #T=selftest  #N=neutral  #E=enable  #D=disable");
        Serial.println("     #J<axis>,<steps> e.g. #J0,300 (=+1.5mm on axis0)");
    } else if (c == "#N") {
        manual_hold_until = 0;
        enable_drivers(true); enabled = true;
        go_neutral();
        Serial.println("OK neutral");
    } else if (c == "#E") {
        enable_drivers(true); enabled = true;
        Serial.println("OK enable");
    } else if (c == "#D") {
        manual_hold_until = 0;
        enable_drivers(false); enabled = false;
        for (int i = 0; i < NUM_AXES; i++) rmt_stop(axes[i]);
        Serial.println("OK disable");
    } else if (c.startsWith("#J")) {
        // 单轴点动：#J<axis>,<steps>，steps 为相对中位的步数（1步=0.005mm）
        int comma = c.indexOf(',');
        if (comma > 2) {
            int axis = c.substring(2, comma).toInt();
            long steps = c.substring(comma + 1).toInt();
            if (axis >= 0 && axis < NUM_AXES) {
                enable_drivers(true); enabled = true;
                axes[axis].limitLatch = 0;   // 手动 jog 先清锁存
                long target = CENTER_POS_STEPS + steps;
                target = max(0L, min((long)MAX_POS_STEPS, target));
                move_axis(axis, target);
                // 无上位机帧时，看门狗会在500ms后回中位；手动点动期间暂缓，
                // 最多保持10s（安全上限），收到上位机帧/#N/#D 立即退出。
                manual_hold_until = millis() + 10000UL;
                Serial.printf("OK jog axis%d -> %ld steps (%.2fmm)\n",
                              axis, target, (target - CENTER_POS_STEPS) * 0.005);
            } else {
                Serial.println("ERR axis 0..5");
            }
        } else {
            Serial.println("ERR format: #J<axis>,<steps>");
        }
    } else if (c == "#T") {
        // 启动非阻塞自检状态机（不能用 delay —— 期间 update_axis 不跑，脉冲发不出去）
        enable_drivers(true); enabled = true;
        selftest_active = true;
        selftest_axis = 0;
        selftest_phase = 0;
        selftest_t = millis();
        move_axis(0, CENTER_POS_STEPS + 300);
        Serial.println("OK selftest started (non-blocking)");
    }
}

// 自检状态机：每轴 300ms 伸出 +300 步，再 300ms 回中，逐轴进行。
// 在 loop() 里调用，保证 update_axis 持续运行、脉冲真正发出。
static void selftest_loop() {
    if (!selftest_active) return;
    unsigned long now = millis();
    if (now - selftest_t < 300) return;
    selftest_t = now;
    if (selftest_phase == 0) {
        move_axis(selftest_axis, CENTER_POS_STEPS);   // 回中
        selftest_phase = 1;
    } else {
        selftest_axis++;
        if (selftest_axis >= NUM_AXES) {
            selftest_active = false;
            Serial.println("OK selftest done");
        } else {
            move_axis(selftest_axis, CENTER_POS_STEPS + 300);
            selftest_phase = 0;
        }
    }
}

void loop() {
#ifdef USE_WIFI
    wifi_loop();
#else
    // 限制每循环解析字节数，避免串口满速灌入时长时间阻塞限位检测/脉冲更新
    int max_bytes = 32;
    while (Serial.available() && max_bytes-- > 0) {
        uint8_t b = (uint8_t)Serial.read();
        if (b == '#') {
            cmd_buf = "";
            while (Serial.available()) cmd_buf += (char)Serial.read();
            cmd_buf.trim();
            handle_text_cmd(cmd_buf);
            continue;
        }
        parse_byte(b);
    }
#endif

#ifdef USE_ESTOP_GPIO
    // 软件急停（双保险）：触发时禁止使能并停全部轴。
    // 主保护仍是硬件常闭按钮切 24V；这里是第二道防线。
    if (digitalRead(PIN_ESTOP) == LOW) {
        enable_drivers(false);
        enabled = false;
        for (int i = 0; i < NUM_AXES; i++) rmt_stop(axes[i]);
    }
#endif

    // 限位触发: 立即停该轴并锁存方向（防止下一帧继续顶死）
    for (int i = 0; i < NUM_AXES; i++) {
        if (digitalRead(PIN_LIMIT[i]) == LOW && axes[i].running) {
            // 触发瞬间运动方向即限位端：dir>0=上限位, dir<0=下限位
            axes[i].limitLatch = (axes[i].dir > 0) ? 1 : -1;
            axes[i].target = axes[i].current;   // 同步影子位置，避免累积误差
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

    selftest_loop();   // 非阻塞自检（保证 update_axis 持续运行）

    // 自检 / 手动点动期间跳过看门狗（均不依赖上位机帧）
    bool manual_hold = (millis() < manual_hold_until);
    if (!selftest_active && !manual_hold && enabled &&
        millis() - last_frame_ms > WATCHDOG_MS) go_neutral();

    // 状态回传：每 50ms 发一帧文本，供调试面板实时显示。
    // 格式: $cur0,cur1,...,cur5,tgt0,...,tgt5,limitMask,enabled,watchdog\n
    //   cur/tgt 为步数(0~38400)；limitMask 6位， bit i=1 表示轴 i 限位触发；
    //   enabled 0/1；watchdog 0/1 表示当前处于看门狗回中状态。
    // 以 '$' 开头，与上位机发来的二进制指令帧 <255><255> 区分。
    static unsigned long last_status = 0;
    if (millis() - last_status >= 50) {
        last_status = millis();
        byte limitMask = 0;
        byte latchMask = 0;
        for (int i = 0; i < NUM_AXES; i++) {
            if (digitalRead(PIN_LIMIT[i]) == LOW) limitMask |= (1 << i);
            if (axes[i].limitLatch != 0) latchMask |= (1 << i);
        }
        byte wd = (enabled && !selftest_active && millis() >= manual_hold_until &&
                   (millis() - last_frame_ms > WATCHDOG_MS)) ? 1 : 0;
        // 一次写入，避免多次 Serial.print 造成 loop 阻塞（P0-2：保持脉冲计数准确）
        char buf[160];
        int n = snprintf(buf, sizeof(buf), "$%ld,%ld,%ld,%ld,%ld,%ld,"
                         "%ld,%ld,%ld,%ld,%ld,%ld,%u,%u,%u",
                         axes[0].current, axes[1].current, axes[2].current,
                         axes[3].current, axes[4].current, axes[5].current,
                         axes[0].target,  axes[1].target,  axes[2].target,
                         axes[3].target,  axes[4].target,  axes[5].target,
                         limitMask, (unsigned)(enabled ? 1 : 0), (unsigned)wd);
#ifdef USE_ESTOP_GPIO
        n += snprintf(buf + n, sizeof(buf) - n, ",%u",
                      (unsigned)(digitalRead(PIN_ESTOP) == LOW ? 1 : 0));
#endif
        // 限位锁存掩码（末字段，L前缀与旧 estop 字段区分；旧解析器只读前15字段，兼容）
        n += snprintf(buf + n, sizeof(buf) - n, ",L%u", (unsigned)latchMask);
        buf[n++] = '\n';
#ifdef USE_WIFI
        wifi_send_status(buf, n);   // WiFi 模式：回传给上位机（同一 UDP 端口）
#else
        Serial.write(buf, n);       // 串口模式：回传到调试线
#endif
    }
}
