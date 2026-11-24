# Báo cáo Đánh giá Sâu Từng Module (Deep Code Audit) — Nhánh `giomuadongbac`

**Ngày rà soát:** 22/11/2026 (UTC+7)  
**Nhánh / Commit kiểm chứng:** `giomuadongbac` @ `3d9bcd2`  
**Phạm vi:** Rà soát trực tiếp toàn bộ 43 tệp `src/*.c`, 38 tệp `include/*.h`, linker script `ld/STM32F411CE_FLASH_256K.ld`, `platformio.ini` và bộ công cụ Python Classic SPP (`tools/classic_spp_*.py`).  
**Tài liệu đầu vào:** `docs/PRELIMINARY_ASSESSMENT_GIOMUADONGBAC.md`, `PROJECT_STATUS.md`, `AGENTS.md`.  
**Nguyên tắc:** Mọi phát hiện bên dưới đều đối chiếu trực tiếp từ mã nguồn theo `file:line`; tách bạch rõ **lỗi kỹ thuật thật (bug/race/UB)** với **giới hạn thiết kế chấp nhận được** trong phạm vi đã bàn giao (RC2 MPU/I2C, OLED diag, Motor Rev3, Encoder fixture).

---

## 0. Tổng hợp 9 Phát hiện Mức CRITICAL (Cần xử lý trước khi build/test phần cứng)

| ID | Module | Vị trí (`file:line`) | Phát hiện đã xác minh & Hệ quả thực tế |
|---|---|---|---|
| **CRIT-01** | Core / Entrypoint | `src/main.c:40, 217, 823-825`; `src/main_runtime.c` | **Git merge conflict chưa xử lý** (`<<<<<<< HEAD` / `=======` / `>>>>>>>`) + trùng `int main(void)`. Đặc biệt: toàn bộ mã `OLED_PRODUCTION_STATUS` (`src/main.c:51-124, 286-308, 344-346, 364-367`) **bị thiếu hoàn toàn trong `src/main_runtime.c`**; nếu xóa bừa phần conflict ở `main.c` sẽ làm mất sạch OLED của `production_oled_status_96m`. |
| **CRIT-02** | IR 6 kênh | `src/ir_sensor.c:235`; `include/pinout.h:89-91` | **Lỗi bitmask `ADC1->SMPR2 = 0x04924924UL` làm kênh `IR_RX_L90` (`ADC1_IN9`, PB1) chỉ lấy mẫu 3 chu kỳ ADC (~125 ns) thay vì 84 chu kỳ (~3,5 µs — ngắn hơn 28×).** Copy nhầm hằng 9 kênh của `SMPR1` sang `SMPR2` (10 kênh, cần `0x24924924UL` để set bit `[29:27] = 100`). Gây suy hao biên độ, sai số phi tuyến và ghosting/crosstalk nghiêm trọng trên mắt `L90`. |
| **CRIT-03** | IR 6 kênh | `src/ir_sensor.c:195-211, 260`; `src/control.c:121, 228, 263` | **Race condition preempt giữa TIM11 (prio 0) và `DMA2_Stream0` (prio 2) gây treo vĩnh viễn IR scanning.** `IR_CompleteDma()` gán `state = IR_READY` (`L197`) **trước** `TIM10->CR1 &= ~TIM_CR1_CEN` (`L198`). Nếu TIM11 preempt đúng khe giữa 2 dòng này, TIM11 gọi `IR_GetLatest()` → `IR_StartScan()` (đặt `state = IR_AMBIENT_START`, bật lại `TIM10`), sau đó DMA2 quay lại `L198` **tắt `TIM10`** → `state` kẹt mãi ở `IR_AMBIENT_START` với `TIM10` tắt cho tới khi reset! |
| **CRIT-04** | I2C1 / Bus Recovery | `src/i2c.c:133, 139, 141, 145, 147, 149` | **RMW không nguyên tử trên `GPIOB_ODR` trong `I2C_BusRecovery()` (ngữ cảnh `main`) đua trực tiếp với `GPIOB->BSRR` của TIM11 prio 0 (TB6612 `PB12–PB15`) và TIM10/DMA2 (IR LED `PB2, PB10`).** Nếu I2C timeout lúc đang chạy, thao tác `GPIOB_ODR \|= / &=` có thể **ghi đè xóa mất lệnh `Motor_Stop()`/`Motor_LockOutputs()` vừa phát từ TIM11** hoặc kẹt LED IR. *(Chưa lộ ở 180s RC2 vì 0 timeout).* |
| **CRIT-05** | I2C1 / OLED Shared | `src/i2c.c:453, 480-481, 521-541, 961-974` | **Đảo ưu tiên `DMA1_Stream6` (prio 4) > `I2C1_EV` (prio 5) gây bão ngắt BTF vô hạn treo chết `main()`.** Ở `OLED_ADDR`, `I2C1_EV` đọc `I2C1->SR2` (`L966`) kích DMA chạy **trước** khi đặt `oled_phase = OLED_DMA` (`L967`) và không tắt `ITEVTEN`. Với gói OLED 2 byte (hoặc bị TIM11/TIM10 chen giữa `L966-967`), `DMA1_Stream6` preempt `I2C1_EV`, thấy `oled_phase == OLED_ADDR`, bỏ rơi `TCIF6`; sau đó `BTF=1` nổ `I2C1_EV` vô hạn rơi vào `default: break;` không clear BTF → đóng băng `main()` vĩnh viễn. |
| **CRIT-06** | Flash Storage | `src/flash_storage.c:109-189`; `src/board.c:127-129` | **Không tắt/flush ART Data Cache (`FLASH_ACR_DCEN`/`DCRST`) khi ghi Flash.** `board.c:129` bật `DCEN=1`. `Flash_IsErased()` + verify word 0 nạp cả dòng 128-bit (`addr+0..+15`) vào D-cache với word 1..3 là `0xFFFFFFFF`; verify word 1 (`addr+4`) đọc trúng cache cũ `0xFFFFFFFF` → **100% lần ghi trên MCU thật sẽ fail `FLASH_ERR_VERIFY` ngay tại word thứ 2 (`h.version_type`) và làm hỏng vĩnh viễn 1 slot trống mỗi lần thử!** |
| **CRIT-07** | System Test | `src/system_test.c:26-28, 102, 187` | **Sai hình học bánh xe gấp 2 lần (nhầm đường kính 34 mm thành bán kính `ST_RADIUS_MM = 34.0f`).** Công thức `2*pi*34/1430 = 0,14939 mm/count` tương đương bánh **Ø68 mm** → `MODE_STRAIGHT200` dừng ở 1338,79 xung nhưng thực tế chỉ đi **~100–103 mm**. |
| **CRIT-08** | Maze Capture | `src/maze_sensor_capture.c:59-66` | **Undefined Behavior (`snprintf` thiếu 1 đối số).** Chuỗi `"MC,..."` ở `L60` có **16 đặc tả định dạng** (9 `%u` ở cuối) nhưng chỉ truyền **15 đối số** (`index..ir[5]`), đọc rác stack vào cột 16, sai lệch với header 15 cột `MC,BEGIN` (`L96`). |
| **CRIT-09** | Integration Sweep | `src/integration_sweep.c:77-85, 166-170, 184-187, 218-242`; `src/speed_pid_diag.c:27-28, 50-99` | **Data race TIM11 Priority 0 ↔ `main()` che giấu lỗi IMU thành PASS giả.** `Sweep_ImuFaultFromISR()` (`control.c:295`) ghi `SWEEP_FAULT`/`SWEEP_IMU_STALE` và gọi `SpeedPidDiag_Stop()` (non-reentrant, state non-volatile) từ ISR trong khi `Sweep_Update()`/`Sweep_Arm()` không khóa IRQ — có thể ghi đè ngược về `SWEEP_COAST`/`SWEEP_DRIVE` hoặc `SWEEP_DONE, SWEEP_OK`. |

---

## 1. Phân hệ Hệ thống Cốt lõi (`main.c`, `main_runtime.c`, `board.c`, `system_timer.c`, `control.c`, `pinout.h`)

### Đánh giá kiến trúc
- Phân cấp ngắt (mặc định `PRIGROUP=0`, 4-bit preemption): `TIM11` (**0**) > `TIM10` (**1**) > `DMA2_S0` (**2**) > `I2C1 MPU EV/ER` (**3**) > `DMA1_S6 OLED` (**4**) > `I2C1 OLED EV/ER` (**5**) > `DMA2_S7 USART1 TX` (**7**) > `USART1/USART2` (**8**) > `TIM5` (**14**).
- Clock 96 MHz danh định (HSI trim25, `PLLM=16, PLLN=192, PLLP=2`, `APB1=48 MHz`, `APB2=96 MHz`, `FLASH 3WS + ICEN + DCEN + PRFTEN`), có kiểm tra fail-closed `Board_GetClockFault()` trước `Motor_Init()`.

### Các phát hiện chi tiết
- **[CRITICAL] `CRIT-01` — `src/main.c:40-825` & `src/main_runtime.c`:** Hợp nhất xung đột bắt buộc phải mang theo toàn bộ nhánh `#if defined(OLED_PRODUCTION_STATUS)` từ `src/main.c` (`L51-124, 286-308, 344-346, 364-367`) sang `src/main_runtime.c` (và thêm `!defined(IR_BURST_DIAG)` vào guard đầu `src/main_runtime.c:3-6` để không lệ thuộc gián tiếp `IR_FILTER_DIAG`).
- **[HIGH] `src/board.c:222-238` — `HardFault_Handler()` thả nổi chân PWM và không tắt 6 LED phát IR:**
  - `TIM2->CCER &= ~(CC1E | CC2E); TIM2->CCR1 = CCR2 = 0;` làm chân `PA15/PB3` (`PUPDR=00`) rơi vào trạng thái floating thay vì Hi-Z Stop chuẩn TB6612 (`IN1=IN2=0, PWM=HIGH`).
  - Không tắt 6 chân LED IR (`PA7, PB10, PB2, PA8, PA11, PA12`): nếu HardFault xảy ra đúng lúc một cặp LED IR đang bật xung dòng cao, cặp LED đó sẽ **sáng liên tục 100% DC** trong `while(1){}` → nguy cơ cháy LED/transistor kích.
- **[HIGH] Thiếu Hardware Watchdog (`IWDG`/`WWDG`) trên toàn hệ thống:** Nếu TIM11 bị tắt hoặc CPU treo trong khi TIM2 đang chạy, PWM vẫn phát vô hạn ra H-bridge; lệnh dừng PA0 (nằm trong TIM11) cũng ngừng tác dụng.
- **[MEDIUM] `src/system_timer.c:25` & `include/system_timer.h:73-75` — Lỗi tràn không lũy thừa 2 của `millis()` sau 71,58 phút:**
  - `millis()` trả về `TIM5->CNT / 1000UL`. Khi `TIM5->CNT` tràn `0xFFFFFFFF → 0`, `millis()` nhảy `4,294,967 → 0` chứ không phải `0xFFFFFFFF → 0`. Mọi phép `(uint32_t)(now - prev) >= period` và `delay_elapsed_ms()` đều bị sai tại mốc 71,58 phút uptime (còn `micros()` wrap chuẩn 2³² nên an toàn).
- **[MEDIUM] `src/control.c:147, 149, 299, 301`:** Hardcode hằng số lý do dừng (`1U`, `3U`, `7U`) khớp ngầm vào `enum` ẩn trong `.c` của `system_test.c` và `turn_pid_diag.c`.
- **[MEDIUM] `include/pinout.h:6, 29, 33, 37, 212-213`:** Comment `L6` ghi ngược `IN1=PB14, IN2=PB15` so với macro `L29/33` (`IN1=PB15, IN2=PB14`); comment `L37` nhắc file `hardware.c` không tồn tại; `WHEEL_BASE_MM = 84.0f` chưa được dùng ở bất kỳ file nào trong `src/`; `WHEEL_DIAMETER_MM = 35.0f` bị lệch sang `34.0f` ở `wall_pid_test_core.h`/`maze_sensor_capture.c` và sai 2× ở `system_test.c`.

---

## 2. Module Cảm biến Hồng ngoại 6 kênh (Ưu tiên mặc định)
**Tệp rà soát:** `src/ir_sensor.c`, `include/ir_sensor.h`, `src/ir_filter.c`, `include/ir_filter.h`, `src/ir_wall_signal.c`, `include/ir_wall_signal.h`, `src/ir_filter_diag.c`, `src/ir_filter_burst_diag.c`, `src/ir_oled_test.c`.

### Trả lời trực tiếp yêu cầu audit từ `AGENTS.md`
- **Blocking ADC/DMA wait trong TIM10 ISR:** **ĐÃ ĐƯỢC LOẠI BỎ HOÀN TOÀN.** `TIM1_UP_TIM10_IRQHandler` hiện là state machine bất đồng bộ (chu kỳ 100 µs, `ARR=99`) kết hợp `DMA2_Stream0_IRQHandler`; không có vòng chờ busy-wait nào trong ISR.

### Các phát hiện chi tiết
- **[CRITICAL] `CRIT-02` — `src/ir_sensor.c:235`:** `ADC1->SMPR2 = 0x04924924UL;` đặt SMP9 (bits `[29:27]`) = **`000` (3 chu kỳ ADC = 125 ns @ 24 MHz)** cho kênh `ADC1_IN9` (`PB1 = IR_RX_L90`), trong khi 5 kênh còn lại (`IN1, IN4, IN5, IN6, IN8`) lấy mẫu **84 chu kỳ (3,5 µs)**. Phải sửa thành `0x24924924UL` và **đo lại toàn bộ số liệu calibration `L90`** sau khi sửa.
- **[CRITICAL] `CRIT-03` — `src/ir_sensor.c:195-211`:** Thứ tự cập nhật trong `IR_CompleteDma()` (`IR_PAIR3_DMA`) đặt `state = IR_READY;` (`L197`) trước `TIM10->CR1 &= ~TIM_CR1_CEN;` (`L198`) và trước ghi `diagnostics` (`L201-211`). Do `TIM11` (prio 0) có quyền preempt `DMA2_Stream0` (prio 2), TIM11 có thể tiêu thụ và khởi động scan mới trước khi `L198` tắt ngược `TIM10`, gây **deadlock vĩnh viễn**. Khắc phục: tắt `TIM10`, cập nhật xong `result`/`scan_count`/`completed_us`/`diagnostics`, gọi `__DMB()`, rồi mới công bố `state = IR_READY` sau cùng.
- **[MEDIUM] `src/ir_sensor.c:446-449` — Crosstalk quang học bắt buộc giữa `L0` và `R0`:** `IR_PAIR3_ON` bật đồng thời cả 2 LED phía trước `IR_LED_L0` và `IR_LED_R0`. Khi tiếp cận tường trước (đặc biệt bài đo P0 vách trắng 20 mm), phản xạ từ `L0` đi thẳng vào mắt thu `R0` và ngược lại. Nên đánh giá tách thành cặp chéo hoặc 4 pha/6 pha nếu dữ liệu 20 mm cho thấy crosstalk vượt ngưỡng.
- **[MEDIUM] `src/ir_sensor.c:107-108, 250-252, 351-359`:** `IR_Stop()` không khóa IRQ khi sửa `state`/`TIM10`/`DMA2`; `IR_Fail()` và `ADC_DMA_Init()` tắt `DMA_SxCR_EN` nhưng không đợi `EN == 0` trước khi xóa `LIFCR` hoặc cấu hình lại.
- **[MEDIUM] `src/ir_filter_diag.c` & `src/ir_filter_burst_diag.c` ↔ `src/ir_sensor.c:396, 465, 478, 503`:** Hai profile `ir_filter_diag_96m` và `ir_burst_diag_96m` biên dịch mã đếm `DWT->CYCCNT` trong `ir_sensor.c`, nhưng **không bật `CoreDebug->DEMCR.TRCENA` / `DWT->CTRL.CYCCNTENA`** (do không gọi `Control_Init()`), nên cycle counter luôn là 0 nếu không gắn debugger.
- **[LOW] `src/ir_filter.c:32-33`:** Ở mẫu thứ 2 (`ring_count == 1`), `median3(x0, x0, x1)` luôn trả về `x0` (bỏ qua `x1`).
- **Điểm tốt (`src/ir_wall_signal.c`):** Kiểm tra atomic `> 4095`, wrap-safe `(int32_t)(completed_us - state->completed_us) <= 0`, tự reset EMA khi mất mẫu/gap > 3 ms và bắt buộc warmup 32 mẫu trước `IR_WALL_VALID`.

---

## 3. Module I2C1, MPU6050 (RC2) & OLED SSD1306 (Shared-I2C1)
**Tệp rà soát:** `src/i2c.c`, `include/i2c.h`, `src/mpu6050.c`, `include/mpu6050.h`, `include/mpu6050_i2c_module.h`, `include/mpu_policy.h`, `src/oled_shared.c`, `include/oled_shared.h`, `src/ssd1306.c`, `include/ssd1306.h`, `src/oled_mpu_diag.c`.

### Giải mã 2 sự cố thực địa đã ghi nhận trong `PROJECT_STATUS.md`
1. **Tại sao `cal=0` bị kẹt >40 giây trên phần cứng?**
   - `MPU6050_CalibrationRetry()` (`src/mpu6050.c:107-121`) sau 3 lượt thất bại (~17–27 s) chuyển vĩnh viễn sang trạng thái chết `cal_state = 4, calibrating = 0, calibrated = 0`. Comment ghi *"require a deliberate KEY retry"*, nhưng **không có bất kỳ file nào trong toàn bộ repo cài đặt lệnh gọi lại `MPU6050_Calibrate()`**.
   - Ngưỡng hội tụ `MPU_CAL_BLOCK_MEAN_SPAN_DPS = 0.12f` (`src/mpu6050.c:171`) ở dải `±2000 °/s` (`16.4 LSB/(°/s)`) chỉ tương đương **`1.97 LSB`** trên 8 block liên tiếp (4 s) → rất nhạy với drift nhiệt ngay sau khởi động hoặc rung bàn nhỏ.
2. **Tại sao `SSD1306_NOT_FOUND` lặp lại trên bo thật?**
   - `SSD1306_Init()` (`src/ssd1306.c:143-148`) gọi `I2C_IsDeviceReady()` **TRƯỚC** `delay_ms_blocking(100)`! Ở `oled_mpu_diag.c` và `ir_oled_test.c`, lệnh dò chạy ở ~2–3 ms sau cấp nguồn trước khi mạch POR của module OLED ổn định → trả `SSD1306_NOT_FOUND` ngay không retry.
   - `I2C_WaitBusFree()` (`src/i2c.c:172-179`) khi gặp BUSY gọi `I2C_BusRecovery()` (tắt `I2C1->CR1.PE` và chuyển PB8/PB9 sang GPIO Output) rồi trả thẳng `I2C_BUSY` **mà không gọi lại `I2C_Init()`** → khiến `I2C_IsDeviceReady()` fail và bỏ lại PB8/PB9 ở chế độ GPIO Output!

### Các phát hiện chi tiết (Bug thật vs Giới hạn thiết kế RC2)
- **[CRITICAL] `CRIT-04` — `src/i2c.c:133-149`:** Phải đổi toàn bộ `GPIOB_ODR |= / &=` trong `I2C_BusRecovery()` sang `GPIOB->BSRR` trước mọi thử nghiệm Motor-ON.
- **[CRITICAL] `CRIT-05` — `src/i2c.c:453, 480-481, 521-541, 961-974`:** Trong `OLED_ADDR`, phải đặt `oled_phase = OLED_DMA` và tắt `I2C_CR2_ITEVTEN` **trước** khi bật `DMAEN`/`DMA_SxCR_EN` và đọc `SR2` (đồng thời cân nhắc đặt `DMA1_Stream6_IRQn` không cao hơn `I2C1_EV_IRQn`).
- **[HIGH] `src/oled_shared.c:12, 51-52` ↔ `src/i2c.c:261-269`:** Ở PCLK1=48 MHz, mức fallback 300 kHz (`fast_mode=2`, `ccr=54`) cho tần số thực **`296,296 Hz < OLED_MIN_SCL_HZ (300,000 Hz)`**, vô hiệu hóa OLED ngay ở mức fallback đầu tiên.
- **[HIGH] `src/i2c.c:495-501`:** `I2C_OledAsync_Poll()` khi abort timeout không tắt `DMA1_Stream6`/`I2C1_EV/ER` IRQ (race gọi kép `OLED_Finish`) và không chạy `I2C_BusRecovery()` (khiến MPU kẹt chờ 5 ms > ngưỡng stale 3 ms).
- **[HIGH] `src/oled_shared.c:40` ↔ `src/main.c:93-95`:** `"MPU: CAL FAILED"` không bao giờ lên màn hình vì `CanStart()` khóa OLED khi `!h->calibrated`.
- **[MEDIUM] `src/i2c.c:650`:** `I2C_Write()` khi recovery gọi cứng `I2C_Init(instance, 1)` (400 kHz), phá tốc độ fallback 300/200/100 kHz đã thương lượng lúc boot.
- **[MEDIUM] `src/i2c.c:78, 527-535`:** `OLED_DMA_FAIL` gộp `FEIF6` và kiểm tra trước `TCIF6` (trái ST AN4031 và cách làm đúng ở `src/uart.c:251-258`).
- **[MEDIUM] `src/i2c.c:992-997, 1007-1015, 1027, 1033-1037`:** Các điểm cận biên RM0383 của `I2C1_EV_IRQHandler`: ghi `CR1` (POS/ACK) ngay sau `CR1 |= STOP`; repeated START trên `BTF` có thể tail-chain nếu slave stretch SCL; 1-byte `SR2 → STOP` không bọc `__disable_irq()`.
- **[MEDIUM] `src/mpu6050.c:520, 528-531, 538, 655-657`:** Nhận `I2C_IDLE` ở `sample_phase` 1/2/5 không reset `sample_phase = 0` (nguy cơ kẹt đọc); lỗi đọc `TEMP_OUT` không cập nhật `temperature_last_ms` gây retry 1 kHz và khóa OLED tại `oled_shared.c:42`.
- **[LOW] `src/i2c.c:288-425, 544-575` & `src/ssd1306.c`:** `I2C_ReadReg_DMA` (legacy không dùng) thiếu guard `OLED_SHARED_I2C`; `SSD1306_UpdateScreen/Clear/...` chưa khóa ở mức API khi `OLED_SHARED_I2C` hoạt động; render ~3.500–3.920 `DrawPixel` trong một vòng `main()` (tốn ~0,8–1,5 ms CPU/frame ở 5 Hz) là nguyên nhân gốc của mức giảm `-0,73%` tần số MPU ở `results/oled_mpu_20261024_082616` — cần chia nhỏ (incremental render) nếu sau này vẽ menu/map 16×16.
- **Giới hạn thiết kế RC2 chấp nhận được (không cần sửa gấp):** ±2000 °/s, Euler integration, LPF α=0.7, hard deadzone 0.3 °/s (đã lưu song song `yaw_raw`), temperature chỉ monitor chưa bù bias, `completed_us` theo thời điểm poll `main()`.

---

## 4. Module Motor Driver, Encoder & Integration Sweep/Serial
**Tệp rà soát:** `src/motor.c/h`, `src/motor_startup.c/h`, `src/motor_test.c/h`, `src/encoder.c/h`, `src/integration_sweep.c/h`, `src/integration_serial.c/h`.

### Các phát hiện chi tiết
- **[CRITICAL] `CRIT-09` — `src/integration_sweep.c:77-85, 166-170, 184-187, 218-242` & `src/speed_pid_diag.c:27-28, 50-99`:** Cần khóa IRQ (`PRIMASK`) quanh các thao tác `state` trong `integration_sweep.c` và không gọi hàm `SpeedPidDiag_Stop()` (chạy trong `main`, non-volatile) trực tiếp từ TIM11 ISR (nên để ISR chỉ set cờ fault + `Motor_LockOutputs()`, `main` dọn `SpeedPidDiag_Stop()`).
- **[HIGH] `src/integration_sweep.c:44, 128-129, 227-230` ↔ `src/speed_pid_diag.c:68-71`:** Aliasing hai đồng hồ `now_ms` vs `millis()` ở cùng chu kỳ 10 ms làm rớt nhịp `SpeedPidDiag_Run()` (chu kỳ thực nhảy thành **20 ms** trong khi PID tích phân cố định với `dt = 0.010f`).
- **[HIGH] `src/motor.c:358-369, 396-404, 435-436, 440-461`:**
  - `Motor_Brake()` đặt `target = applied = 0` nên `Motor_Update1ms()` reset `command_age_ms = 0` rồi thoát sớm → **Short Brake miễn nhiễm với command lease 300 ms** vô thời hạn.
  - Cửa sổ quá độ `IN1=0, IN2=0, PWM=LOW` (1 ms khi khởi động `was_stopped` và tối đa 33,3 µs khi dừng) chưa có trong bảng chân lý chính thức TB6612FNG — cần đo oscilloscope trước khi nghiệm thu điện.
- **[HIGH] `src/encoder.c:266-267` ↔ `src/motor.c:61`:** TIM3/TIM4 để `IC1F = IC2F = 0000` (không lọc số, lấy mẫu 96 MHz) trong khi chân **`PB4` (`TIM3_CH1`, Encoder Right)** nằm kề chân **`PB3` (`TIM2_CH2`, PWM Motor Left 30 kHz, `OSPEEDR=3` Very High Speed)** — dễ nhiễu đếm xung giả khi motor tải cao.
- **[HIGH] `src/motor.c:170-212`:** Không nhất quán điều kiện mở khóa sau watchdog: `Motor_SystemTestUnlockOutputs()` và `Motor_TurnDiagUnlockOutputs()` xóa `command_timeout_latched = 0`, trái ngược với `Motor_WallPidTestUnlockOutputs()` từ chối mở khóa khi từng nổ watchdog.
- **[HIGH] `src/motor_test.c:56-68, 80-104, 111-137` & `src/motor_startup.c`:** Cả hai là **mã mồ côi** (chỉ gọi từ unit test, không nối vào firmware nào) và có nhiều lỗi: `MotorTest_BrakePulse()` có TOCTOU race, bypass NaN, trả về `1` dù motor đang lock; `MotorTest_Tick1ms()` gọi `Motor_KeepAlive()` kéo dài lease lên 1500 ms (trái quy tắc 300 ms của `control.c:213`).
- **[MEDIUM] `src/encoder.c:88-91, 205-238, 262-279, 362-365`:**
  - Thiếu acquire barrier (`__DMB()`) giữa check `tail == shadow_qhead` và đọc non-volatile `shadow_samples[tail]` trong `Encoder_ShadowService()`.
  - Không có API snapshot nguyên tử cặp `(L, R)` nên nhiều caller ở `main()` (`main_runtime.c`, `speed_pid_diag.c`, `ble_benchmark.c`) đọc lệch nhau tối đa 1 ms.
  - `Encoder_TimerInit()` không xóa sạch thanh ghi (`PSC/CR1/CCER/CCMR1/SMCR/DIER`); `Encoder_Init()` thiếu dummy read `RCC->APB1ENR`.
  - `Encoder_GetShadowSnapshot()` khóa IRQ thừa khi toàn bộ mảng đã offload sang `main()` (tạo ~2–3 µs jitter cho TIM11).
- **[MEDIUM] `src/motor.c:55-72, 303-309`:** `GPIO_AF()` ghi `MODER` trước `AFR` gây glitch ngắn trên chân JTAG `PA15/PB3`; `fall_rate <= 3` làm ramp-down >300 ms kích nhầm watchdog 300 ms.
- **[MEDIUM] `src/integration_serial.c:49-53, 114-127` & `src/integration_sweep.c:145-172, 202-210`:** Hardcode chuỗi `CMD_PIDCFG` tách rời macro `speed_pid_diag.c`; lỗi clock báo nhầm mã `SWEEP_IMU_STALE` thay vì `SWEEP_CLOCK_FAULT`; `Sweep_Arm()` không kiểm tra `max_irq_cycles` trước khi cấp lệnh `Command(0U)`; lệnh dừng khẩn `'!'` xử lý ở `main` và có thể bị vứt nếu `USART1` có `ORE`.

---

## 5. Module Điều khiển, PID, Turn/Wall/System Test & Maze Capture
**Tệp rà soát:** `src/pid.c/h`, `src/motion.c/h`, `src/motion_primitives.c/h`, `src/speed_pid_diag.c/h`, `src/system_test.c/h`, `src/turn_pid_diag.c/h`, `src/wall_pid_test.c/h`, `src/wall_pid_test_core.c/h`, `src/wall_pid_trace.c/h`, `src/maze_sensor_capture.c`.

### Bảng đối chiếu sai lệch hằng số hình học bánh xe
| Vị trí | Cấu hình | CPR/PPR | `mm/count` | Nhận xét |
|---|---|---:|---:|---|
| `include/pinout.h:212` & `src/motion.c:11` | `WHEEL_DIAMETER_MM = 35.0f` | `1430` | `0.076892` | Chuẩn PCB hiện tại |
| `include/wall_pid_test_core.h:14` | `WP_WHEEL_DIAMETER_MM = 34.0f` | `1430` | `0.074695` | Lệch `-2,86%` (~20,6 mm / 720 mm) |
| `src/maze_sensor_capture.c:96` | `diam_mm=34` | `1430` | `0.074695` | Lệch `-2,86%` |
| `src/system_test.c:26-28` | **`ST_RADIUS_MM = 34.0f`** (**Ø68 mm!**) | `1430` | **`0.149390`** | **CRITICAL (`CRIT-07`): Sai gấp 2×** |

### Các phát hiện chi tiết theo từng module con
- **`src/pid.c` ([HIGH]):** Không kiểm tra `NULL` và `isfinite()` → `NaN` vượt qua clamp và nhiễm độc `integral`/`prev_error` vĩnh viễn; `dt <= 0` vẫn cộng `integral += error * dt` (`L28`); D-term trên `error` không LPF và không có cờ `has_prev` sau `PID_Reset()` tạo derivative kick khổng lồ (`+90` ở tick đầu `TURN 90°`); không anti-windup theo bão hòa đầu ra; `PID_Reset()` không xóa `output`.
- **`src/motion.c` ([HIGH]):**
  - `NormalizeAngle()` (`L44-49`) dùng vòng `while` không chặn giới hạn trong **TIM11 Priority 0 ISR** và không kiểm tra `isfinite()` → `±Inf` hoặc float cực lớn làm treo cứng hệ thống (motor runaway).
  - `MOTION_STRAIGHT` (`L259-287`) không có timeout (dù cộng `elapsed_s`), không stall guard, không giới hạn lệch góc, không kiểm tra `NaN` ở `Motion_CommandStraight()` (`NaN <= 0` là false → chạy vô tận).
  - `MOTION_TURN` (`L289-310`): hỏng khi mục tiêu `180°` (nhích ngược `-0,5°` nhảy sai số sang `-179,5°` đảo chiều toàn lực); ép `magnitude >= TURN_PWM_MIN (20%)` kể cả khi `output == 0` gây giật bang-bang.
  - `static MotionPrimitiveController pivot` (`L42`) không `volatile`, các getter `Motion_GetPrimitive*()` đọc không khóa IRQ; gọi `Motion_CommandStraight/Turn()` đè lúc `MOTION_PIVOT_PRIMITIVE` đang chạy làm `pivot.status` kẹt ở `RUNNING` vĩnh viễn. Toàn bộ API `Motion_RequestPrimitive/Turn90/Turn180` hiện là dead code chưa nối vào `main_runtime.c`.
- **`src/motion_primitives.c` & `src/turn_pid_diag.c` (`SingleBrakeControlTick`) ([HIGH]):**
  - Pha `ACCEL` (`motion_primitives.c:180-200`, `turn_pid_diag.c:1133-1156`) chỉ sang `BRAKE` khi `v > 180.0f && e <= braking_angle`, không có nhánh dự phòng ở tốc độ thấp và không có stall guard → pin yếu/ma sát cao (`v <= 180 dps`) sẽ giữ **90–100% PWM xuyên qua đích**, kẹt bánh giữ 90–100% PWM suốt 4,0–6,5 giây.
  - Hằng số `accel_ff` (`+220 pm`) vẫn cộng cố định suốt pha cruise khi `ref == vmax`, đẩy quay 180° bão hòa 1000 pm sát ngưỡng abort `875 dps`.
  - `SETTLE` không kiểm tra `|w| <= 12 dps`; `turn_pid_diag.c` còn có lỗi **unsigned underflow race** (`L456, 1767, 1783-1788`) bỏ qua 800 ms đo coast/drift khi TIM11 dừng sau lúc `main` lấy `now = millis()`, thiếu 1 trường trong `profiles[TP_TRACK]` (`L288`), multi-phase fall-through (`Flow`/`Glide`/`Sprint`), và in `TP,START` kể cả khi unlock motor thất bại.
- **`src/wall_pid_test.c`, `wall_pid_test_core.c`, `wall_pid_trace.c` ([HIGH]):**
  - **Unsigned underflow race (`wall_pid_test.c:75, 288, 296-297`):** `W,POST` có thể ghi ở 0 ms thay vì 800 ms.
  - `WP_WALL_LOST` và `WP_EXCURSION` là **mã chết** → mất 1 tường ở tốc độ tới 900 mm/s sẽ bẻ lái hết tầm `±200 pm` vào khoảng trống.
  - Stall guard (`wall_pid_test_core.c:76`) dùng `&&` và kiểm tra tổng xung `< 20` từ lúc xuất phát → bỏ sót kẹt 1 bánh và **không phát hiện đâm tường giữa đường** (giữ tới 80% PWM tối đa 5 s).
  - `Clip(base ± steer, 0, 800)` (`L138-139`) cắt bất đối xứng ở cận dưới `0` khi giảm tốc `base < |steer|`; `speed_integral_pm` thiếu anti-windup.
  - `wall_pid_trace.c:21, 78`: `rate_cd` dùng `int16_t` (tràn ở **`±327,67 dps`** dù `WP_Step` cho phép `±720 dps`); `wall_pid_test.c:105` vẫn gọi `WPT_Append()` khi `WP_IMU_STALE` (nguy cơ ép kiểu `NaN`/`Inf` sang int); gói `W,END`/`MW,BEGIN`/`MW,END` dài >20 B không chia chunk BLE; mảng trace chiếm ~48 KB SRAM và replay mất ~10 phút.
- **`src/speed_pid_diag.c` ([HIGH]):** Ngoài race từ TIM11 (`CRIT-09`) và aliasing 10 ms/20 ms, ở `step = 3` (`2400 PPS`) `feedforward = 300 == MAX_CMD (300)` → **0 headroom dương** cho PID trong khi tích phân vẫn windup tới `1250`.
- **`src/system_test.c` ([CRITICAL/HIGH]):** Ngoài lỗi hình học 2× (`CRIT-07`), `pid_turn` là P-only với sàn bang-bang `105 pm` và `Motor_Stop()` bất chấp tốc độ góc → dễ limit-cycle; `st.stop_press_consumed` có thể kẹt khi bấm-nhả nhanh.
- **`src/maze_sensor_capture.c` ([CRITICAL/HIGH]):** Ngoài UB `snprintf` 16 vs 15 đối số (`CRIT-08`), `yaw_cd` và `rate_cd` dùng `int16_t` (tràn ở `±327,67°` / `±327,67 dps`) trong khi `Begin()` không `MPU6050_ResetYaw()`; `Begin()` không kiểm tra `IR_WALL_VALID`; `MC,BEGIN` (~138 B) gửi không chia chunk BLE.

---

## 6. Module Truyền thông (USART1, USART2/JDY-33, Classic SPP Host)
**Tệp rà soát:** `src/uart.c/h`, `src/uart_bench.c/h`, `src/bt_debug.c/h`, `src/ble_benchmark.c`, `src/ble_rx_diag.c`, `tools/classic_spp_*.py`.

### Các điểm đã xác minh ĐẠT
- **Clock/BRR:** USART1 (APB2 96 MHz → `BRR=0x0341`, sai số 0,00%) và USART2 (APB1 48 MHz → 115200 `BRR=0x01A1` +0,08%, 38400 `BRR=0x04E2` 0%) đều chính xác.
- **DMA stream:** Không xung đột (`DMA2_S7 Ch4` cho USART1 TX, `DMA2_S0 Ch0` cho IR ADC1, `DMA1_S6 Ch1` cho OLED I2C1 TX; USART2 dùng TXE/RXNE IRQ).
- **Atomic drop trên `bt_debug.c`:** `BT_TrySendData()` hiện drop nguyên gói (không cắt đuôi như mô tả cũ trong `PROJECT_STATUS.md:52`).
- **RX-only trên Python Classic SPP:** Toàn bộ `classic_spp_dashboard.py`, `classic_spp_link_diag.py`, `classic_spp_rate_pipeline.py` không có bất kỳ lệnh `.write()` nào.

### Các phát hiện chi tiết
- **[HIGH] `src/bt_debug.c:18, 20, 66-74, 118, 127, 139-140` & `src/ble_rx_diag.c:61, 66`:** `tx_buf`/`rx_buf` không `volatile` và thiếu `__DMB()`/compiler barrier trong lock-free SPSC ring (nguy cơ GCC reorder đọc/ghi payload qua `head`/`tail`); đồng thời RMW không khóa IRQ trên `USART2->CR1` giữa `main` (`TXEIE`/`RE`/`RXNEIE`) và `USART2_IRQHandler` (`&= ~TXEIE`).
- **[HIGH] `src/uart.c:205-224` ↔ `src/main_runtime.c:627`:** `UART_Printf()` bỏ qua giá trị trả về của `vsnprintf(buffer, 256, ...)` → cắt cụt âm thầm mất `\r\n` (dòng `SYS_IRV` có 152 ký tự cố định + 25 số nguyên, **vượt 255 byte** khi giá trị lớn) và gửi rác stack nếu `vsnprintf < 0`.
- **[HIGH] `tools/classic_spp_rate_pipeline.py:361-378, 117, 177`:** `decoder.feed(data)` chạy **trước** vòng cập nhật `active_session` từ `!B`/`!E` trong cùng chunk đọc COM → lỗi CRC của session mới không được gán vào `crc_by_session[active_session]` (`crc_case=0`), **có thể chấm nhầm `CANDIDATE` cho case bị hỏng CRC** hoặc gán nhầm sang session trước.
- **[HIGH] `tools/classic_spp_dashboard.py:141-142, 234-240, 440-476`:** Với firmware production (`main_runtime.c:688` phát `t, state, encL, encR, yaw, gz, d, e`), mặc định `plot_a="l0"`, `plot_b="r0"` làm `all_values` rỗng → `draw_plot()` **`return` sớm ở `L446` trước khi cập nhật `self.metric_table` (`L472-475`)**, khiến cả biểu đồ lẫn bảng số liệu trống trơn; combobox không bind `<<ComboboxSelected>>` nên đổi kênh khi dừng/replay không vẽ lại.
- **[MEDIUM] `src/uart.c:73, 105, 260, 265-279`:** (a) Sau lỗi `DMEIF`, `DMA2_Stream7_IRQHandler` tắt `EN` rồi gọi ngay `UART_DMA_StartNext()` mà không đợi `EN==0` → nguy cơ treo TX DMA (`dma_busy=0`, `TCIF7` nổ sau bị bỏ qua). (b) Chân `PA10` (USART1_RX) để floating (`PUPDR=00`), khi rút USB-TTL dễ nhiễu `FE/NE` hoặc nhận giả `'S'`/`'s'` dừng motor. (c) Cả USART1 và USART2 khi gặp `ORE` đều vứt luôn byte hợp lệ cũ trong `DR`.
- **[MEDIUM] `src/ble_benchmark.c:474-494, 554-557` & `tools/classic_spp_rate_pipeline.py:376, 393, 398`:** `due_us = now + step` cộng dồn trễ main-loop (dễ tụt `schedule_achieved_pct < 98%` ở 200–800 Hz); `!S` (~180 B) vẫn phát 1 Hz chen ngang lúc đo tải SPP; `BLE_PROFILE_DIAG` không xả TX queue trước `!E`; Python không dừng khi gặp `!Q,state=ABORT` (treo đủ 600 s) và in chuỗi literal `\\n`.
- **[MEDIUM] `tools/classic_spp_protocol.py:191-192, 211-215` & `tools/classic_spp_dashboard.py:61-66, 271-306, 406, 527-531`:** `MazeState` đổi `self.n = 16` trước khi mở rộng mảng `walls` 5×5 → gây `IndexError` tại `dashboard.py:406` làm chết vòng `redraw()`; `SPPReader._emit` nuốt `queue.Full` âm thầm (mất cả dòng log lẫn sự kiện `"closed"`); `disconnect()`/`quit()` không `join()` thread đọc COM.

---

## 7. Module Flash Storage & Linker 256K
**Tệp rà soát:** `ld/STM32F411CE_FLASH_256K.ld`, `src/flash_storage.c/h`, `src/flash_record_format.c/h`, `src/flash_map_protocol.c/h`, `src/flash_map_diag.c/h`, `include/flash_legacy_types.h`.

### Các điểm đã xác minh ĐẠT
- Phân vùng linker script (`0x08000000`, 256 KiB = Sectors 0–5, `board_upload.maximum_size = 262144`) khớp tuyệt đối với Sector 6 (`0x08040000`, 128 KiB calib) và Sector 7 (`0x08060000`, 128 KiB map); 127 slot append-only (slot 1..127), giữ nguyên slot 0 legacy; header 20 B + payload ≤1000 B + commit marker `0xA53C69F0` tại `+1020`, bảo vệ bởi CRC32/IEEE.

### Các phát hiện chi tiết
- **[CRITICAL] `CRIT-06` — `src/flash_storage.c:109-189` ↔ `src/board.c:127-129`:** Bắt buộc phải tắt `FLASH_ACR_DCEN` hoặc chạy chuỗi reset D-cache (`DCEN=0 → DCRST=1 → DCRST=0 → DCEN=1` + `__DSB(); __ISB();`) trước mọi bước đọc verify sau ghi, nếu không mọi lần ghi trên STM32F411 thật đều fail `FLASH_ERR_VERIFY` ngay ở word thứ 2.
- **[HIGH] `src/flash_storage.c:28, 96-123, 148-188`:**
  - Thiếu `__DSB()` ngay sau ghi word (`L117`) trước `Flash_WaitReady()`.
  - `FLASH_ERR_MASK` (`L28`) thiếu `FLASH_SR_OPERR` (và `FLASH_SR_RDERR`).
  - `Flash_Append()` (`L148`) không kiểm tra `IR_Stop()` (trái hợp đồng `include/flash_storage.h:47`) và không khóa ngắt trong khi Flash single-bank làm stall fetch lệnh từ Flash tới ~100 µs/word (vượt 1 byte 115200 baud = 86,8 µs; vòng `Flash_WaitReady()` nằm trên Flash cũng bị stall khi `BSY=1`).
- **[HIGH] `src/flash_map_diag.c:44-47, 181`:** Undefined Behavior `memcpy(flash_buffer, flash_buffer, n)` (nguồn trùng đích) khi `DoSave()` truyền `flash_buffer` vào `ReadMap()`.
- **[MEDIUM] `src/flash_storage.c:223-263`:** `Flash_SaveMapData` (`MAP1`) và `Flash_SaveMazeChecked` (`Maze_Persistent_t`) dùng chung `REC_MAP = 1U` trên Sector 7 → ghi `MAP1` làm `Flash_LoadMazeChecked` trả `FLASH_ERR_FORMAT` thay vì tương thích ngược.
- **[LOW] `src/flash_storage.c:27, 291-298, 317-347`:** `FLASH_PSIZE_WORD` đòi hỏi `VDD ≥ 2,7 V`; `MPU_Calib_Flash_t` có 3 byte padding cuối chưa xóa 0 trước khi tính CRC; wrapper `Flash_Report` luôn gọi `UART_Printf` dù profile có thể chưa `UART_Init()`.

---

## 8. Lộ trình khắc phục đề xuất (Theo đúng thứ tự ưu tiên kỹ thuật)

1. **Giai đoạn 0 — Gỡ blocker build:**
   - Giải quyết merge conflict `src/main.c` và đưa đầy đủ nhánh `OLED_PRODUCTION_STATUS` vào `src/main_runtime.c`; chạy build thử tất cả 17 env trong `platformio.ini` + suite `tests/regression/`.
2. **Giai đoạn 1 — Sửa các lỗi CRITICAL trên phần cứng & an toàn (bắt buộc trước mọi lần flash/chạy motor):**
   - **IR (`src/ir_sensor.c`):** Sửa `SMPR2 = 0x24924924UL` (`CRIT-02`); dời `TIM10->CR1 &= ~TIM_CR1_CEN` và cập nhật `diagnostics` lên **trước** `state = IR_READY` + `__DMB()` (`CRIT-03`); thêm tắt 6 LED IR vào `HardFault_Handler()`.
   - **I2C1/OLED (`src/i2c.c`):** Chuyển `I2C_BusRecovery()` sang `GPIOB->BSRR` (`CRIT-04`); sửa thứ tự `OLED_ADDR → OLED_DMA` + tắt `ITEVTEN` trước `SR2` (`CRIT-05`); khôi phục `I2C_Init()` sau recovery trong `I2C_WaitBusFree()` và sửa `I2C_Write()` giữ nguyên `i2c_current_speed_mode`.
   - **Flash (`src/flash_storage.c`, `src/flash_map_diag.c`):** Thêm tắt/flush ART `DCEN/DCRST` + `__DSB()/__ISB()` + `FLASH_SR_OPERR` (`CRIT-06`) và sửa overlapping `memcpy`.
   - **Diagnostics bị hỏng số liệu (`system_test.c`, `maze_sensor_capture.c`, `integration_sweep.c`):** Sửa `ST_RADIUS_MM` (`CRIT-07`), sửa `snprintf` 15 đối số (`CRIT-08`), khóa IRQ trong `integration_sweep.c` và không gọi `SpeedPidDiag_Stop()` từ TIM11 (`CRIT-09`).
3. **Giai đoạn 2 — Gỡ 2 lỗi vận hành thực địa & củng cố công cụ Classic SPP:**
   - **MPU/OLED thực địa:** Thêm cơ chế retry thoát `cal_state = 4` + đánh giá lại ngưỡng `0.12 °/s`; đảo `delay_ms_blocking(100)` lên trước `I2C_IsDeviceReady()` trong `SSD1306_Init()`; hạ `OLED_MIN_SCL_HZ` xuống `290000UL` và cho phép hiển thị `CAL FAILED`.
   - **Comms & Classic SPP:** Thêm `__DMB()` / bảo vệ `USART2->CR1` ở `bt_debug.c`; check tràn `vsnprintf` ở `uart.c`; kéo pull-up `PA10`; sửa `crc_by_session` trong `classic_spp_rate_pipeline.py` và sửa `draw_plot()`/`MazeState` trong `classic_spp_dashboard.py`.
4. **Giai đoạn 3 — Củng cố điều khiển chuyển động trước khi chạy mê cung thật:**
   - Thống nhất 1 hằng số đường kính bánh duy nhất (đo thực tế 34 vs 35 mm); bật `IC1F/IC2F` lọc nhiễu cho encoder TIM3/TIM4; bổ sung `isfinite()` / bỏ `while` vô hạn trong `NormalizeAngle()` & `pid.c`; sửa unsigned underflow race (`turn_pid_diag.c`, `wall_pid_test.c`); bổ sung nhánh phanh tốc độ thấp & stall guard trong `motion_primitives.c` và `wall_pid_test_core.c`; cân nhắc bật `IWDG` phần cứng và chuẩn hóa `millis()` tránh tràn ở 71,58 phút.
