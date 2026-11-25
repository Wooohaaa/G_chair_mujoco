# ESP32-S3 单板调度可行性分析（是否要 RTOS）

> 结论先行：**当前单 loop 架构基本可行，不需要上 FreeRTOS 多任务**，
> 但有 **2 个必须在上真实电机前修掉的阻塞点**，以及 3 个建议改进项。
> Arduino-ESP32 底层本来就是 FreeRTOS，问题不在"有没有 RTOS"，而在"当前代码有没有被阻塞"。

---

## 1. 当前架构：一块板承担了两块板的功能

| 原两块 Mega 的功能 | 现在 ESP32-S3 谁做 |
|---|---|
| 收串口帧 + 位置映射 | `parse_byte()` 在 loop 里 |
| 4 定时器产生脉冲（×2板） | **RMT 硬件外设**（6 路），不占 CPU |
| 位置计数（原 ISR 中断累加） | `update_axis()` 在 loop 里用 `micros()` 累加 |
| 限位检测 | loop 里轮询 |
| 看门狗 | loop 里轮询 |
| 状态回传 | loop 里 `Serial.print` |

**关键**：脉冲产生（最实时的事）已经是**硬件 RMT**，不依赖 CPU 调度——
这是不需要 RTOS 的核心原因。RTOS 解决的是"多个软件任务争抢 CPU"，
而这里最硬的实时任务已经卸载到硬件了。

---

## 2. 时序量化（实测/文档确认）

| 事件 | 周期 | 单次耗时 | 占用 |
|---|---|---|---|
| 控制 tick (`update_axis`) | 4ms (250Hz) | 6 轴约 0.1ms | 2.5% |
| 状态回传 (`Serial.print`) | 50ms (20Hz) | **6.8ms** (78字节@115200) | 13.5% |
| 指令接收 (14字节帧) | 20ms (50Hz) | 1.2ms | 6.1% |
| 限位轮询 | 每 loop | 6 次 digitalRead ~0.01ms | 忽略 |

**总 CPU 占用 < 25%，单 loop 完全跑得动。**

---

## 3. 必须修的问题（True Blocking Issues）

### 🔴 P0-1：`rmt_write_items(..., wait_tx_done=true)` 在 loop 模式下可能死锁

`main.cpp:41, 70`：
```c
rmt_write_items(s.chan, &s.item, 1, true);  // wait_tx_done = true
```

ESP32 RMT 文档明确：
> wait_tx_done=true: 阻塞直到**所有 items 都发送完**
> wait_tx_done=false: 立即返回，中断继续发送

但我们的 RMT 配置了 **`loop_en=true`（无限循环发送）**——"所有 items 发送完"这个条件**永远不会满足**，
`rmt_write_items(..., true)` 可能永久阻塞或行为未定义。

每次 `update_axis` 变速都调 `set_period` → `rmt_write_items(..., true)`，
也就是**每个轴每次加减速都可能卡住整个 loop**。这是真实的高危 bug。

**修复**：改用 `wait_tx_done=false`（文档推荐的非阻塞方式），
但要保证 `s.item` 在 RMT 仍在使用时不被修改——方法：
- 用 `rmt_tx_stop()` 停 → 改 item → `rmt_write_items(..., false)` 重启；
- 或保留 double-buffer（两个 item 交替）。
- 最简单稳妥：改周期时先 `rmt_tx_stop`，改完 item 再 `rmt_write_items(false)` 重新加载。

### 🔴 P0-2：RMT 硬件脉冲 与 软件 current 计数会脱节

**原 Mega**：定时器 ISR 里 `currentPos += incPos`，计数与脉冲严格同步（同一中断）。

**现在**：`update_axis()` 在 loop 里用 `dt_us / period_us` 估算发出多少脉冲。
问题：
- loop 被 `Serial.print`(6.8ms) 或串口解析阻塞时，`update_axis` 没跑，
  但 **RMT 硬件还在按原周期持续发脉冲** → 实际发出的脉冲数 > 软件 current。
- 结果：`current` 落后于真实位置 → 到位判断错误（平台冲过目标或多走）。

**修复方案（任选）**：
1. **用 RMT 中断计数**：配置 TX 完成中断，每发一个 item 中断累加（最准，但要写 ISR，注意放 IRAM）。
2. **用硬件读回**：读 RMT 实际发送数量（ESP32 RMT 有 `rmt_get_tx_length` 之类 API）——但循环模式下读回的是剩余数，需转换。
3. **保证 loop 不被长阻塞**：修掉 P0-1，把状态回传的非阻塞化（见 P1-2），
   这样 loop 周期稳定在 <1ms，`dt_us/period_us` 估算误差 <1 个脉冲——可接受。
4. ** pulses 不依赖 current 闭环**：如果伺服本身是位置模式、上位机每 20ms 发一次目标，
   位置误差由上位机下一次帧纠正，current 只是"预估"——那当前方案 tolerant 度够。

**建议**：先做方案 3（成本最低），如果实机发现累积误差再做方案 1。

---

## 4. 建议改进（非必须）

### 🟡 P1-1：限位检测用中断而非轮询
当前每 loop 轮询 6 个 GPIO。loop 正常 <1ms，够用；
但限位是安全信号，用 `attachInterrupt` 更可靠（触发立即停 RMT，不依赖 loop）。

### 🟡 P1-2：状态回传用非阻塞串口
`Serial.print` 78 字节在 115200 下占 6.8ms。ESP32 的 HardwareSerial 有 TX buffer（256字节），
通常不会真阻塞 6.8ms（只是写 buffer），但 buffer 满时会阻塞。
建议：把状态帧先格式化进固定 buffer，一次 `Serial.write(buf, len)`；
或降低回传频率到 10Hz。

### 🟡 P1-3：看门狗计时用独立硬件 timer
当前在 loop 里比较 `millis()`。loop 卡住时看门狗也卡住。
若 P0-1 修掉，loop 不会长时间卡住，当前方案可接受。

---

## 5. 关于"要不要 RTOS"的直接回答

**不需要为了这个功能上 FreeRTOS 多任务**，理由：

1. Arduino-ESP32 底层已经是 FreeRTOS（`loop()` 本身是一个 task，跑在 core 1）。
2. 最实时的脉冲产生已经是 **RMT 硬件外设**，不占 CPU、不受任务调度影响。
3. 剩余软件任务（串口解析、位置累加、限位、看门狗、状态回传）总占用 <25%，
   单 loop 顺序执行没有争抢。
4. 上多任务反而引入新问题：共享 `axes[]` 结构需要加锁/临界区，
   增加复杂度和 bug 风险。

**真正需要的是修掉阻塞点（P0-1）和确认位置计数精度（P0-2）**，不是加 RTOS。

**什么时候才需要 RTOS/多任务**：
- 如果要在 ESP32 上同时跑视觉/YOLO + 控制（目前视觉在 RK3566/PC，不在 ESP32）
- 如果要加 WiFi 远程升级 + 控制同时跑且不能互相阻塞
- 如果要做多轴同步插补（需要严格等周期的调度，可用 FreeRTOS 定时器任务）

---

## 6. 修复状态（2026-11-25 更新）

| 优先级 | 问题 | 状态 |
|---|---|---|
| **P0** | `rmt_write_items(true)` 可能死锁 | ✅ **已修**：改 `wait=false`，变速时 `rmt_tx_stop` → rewrite → `rmt_tx_start(true)`（与 Arduino 官方 `rmtWriteLooping` 一致）。编译通过 |
| **P0** | 软件 current 与 RMT 脉冲脱节 | ⚠️ **缓解**：P0-1 修复后 loop 不再被 RMT 阻塞，估算误差 <1 脉冲；实机数脉冲验证后再决定是否加 RMT 中断计数 |
| P1 | 限位轮询 | 暂不改（loop <1ms，轮询够用） |
| P1 | 状态回传阻塞 | 暂不改（写入 TX buffer，13.5% 占用） |
| P1 | 看门狗 | 暂不改（随 loop 稳定） |

---

## 7. 验证方法

修完后必须在实际硬件上验证（不能只靠编译）：
1. 空载，逻辑分析仪看 6 路 STEP 波形：周期是否随目标距离平滑变化、有无突然中断/停发。
2. 给一个大幅目标（如中位→+95mm），用示波器数脉冲个数，对比 `current` 软件计数值：
   两者应一致（±1 脉冲）。
3. 状态回传开着（20Hz）的同时快速连续发指令，看有无卡死（看门狗是否误触发）。
4. 限位触发测试：触发后对应轴立即停发脉冲。

---

## 8. 一句话总结

**单 loop 可行，不用上 RTOS；但 `wait_tx_done=true` 的 RMT 调用是真实死锁风险，
且软件位置计数可能与硬件脉冲脱节——这两个必须在接真实电机前修掉。**
