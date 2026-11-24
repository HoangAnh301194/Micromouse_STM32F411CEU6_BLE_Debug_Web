# Đánh giá sơ bộ nhánh `giomuadongbac` — Micromouse STM32F411CEU6

**Ngày tổng hợp:** 22/11/2026 (UTC+7)  
**Nhánh làm việc:** `giomuadongbac` (tracking `origin/giomuadongbac`, HEAD `3d9bcd2`)  
**Mục đích tài liệu:** Tổng hợp nhanh hiện trạng mã nguồn, cấu hình build, công cụ, bằng chứng thực nghiệm và các rủi ro/blocker trên nhánh `giomuadongbac` làm đầu vào cho phiên đánh giá sâu từng module (bằng Neon model).

---

## 1. Trạng thái Git & Blocker hiện tại

- **HEAD hiện tại:** `3d9bcd2` (`docs: record Web BLE freeze and Classic SPP host debug priority`).
- **Working tree:** Sạch (chỉ vừa chuyển nhánh từ `main` sang `giomuadongbac`).
- **Blocker nghiêm trọng trên mã nguồn (`src/main.c`):**
  - `src/main.c` đang chứa Git merge conflict markers chưa được giải quyết tại:
    - Dòng `40`: `<<<<<<< HEAD`
    - Dòng `824`: `=======`
    - Dòng `825`: `>>>>>>> e41c1a1f73c3c25492cf61c7dede937855026fcc`
  - **Hệ quả:** Không thể khẳng định build PlatformIO hay flash phần cứng từ mã nguồn hiện tại sẽ thành công cho tới khi rà soát và xử lý dứt điểm xung đột trong `src/main.c`.

---

## 2. Cấu trúc kho lưu trữ trên nhánh `giomuadongbac`

| Thư mục / Tệp | Quy mô & Nội dung chính |
|---|---|
| `src/` | 43 tệp `.c` (core runtime, drivers ngoại vi, các module diagnostic/test opt-in) |
| `include/` | 38 tệp `.h` (bao gồm `pinout.h` cố định theo PCB, `mpu6050_i2c_module.h`, `mpu_policy.h`, v.v.) |
| `platformio.ini` | 17 môi trường build hiện hành (production + active diagnostics) |
| `platformio_legacy.ini` | Lưu các profile A/B, encoder, UART, OLED lịch sử (opt-in) |
| `ld/` | Linker script `STM32F411CE_FLASH_256K.ld` (giới hạn app 256 KiB, chừa sector 6–7 cho Flash storage) |
| `tools/` | 47 tệp (gồm `README.md`, `README_TESTING.md` và các pipeline Python/PowerShell cho Classic SPP, BLE, IR, Encoder, PID, OLED, Flash, Maze) |
| `tests/` | 7 nhóm kiểm thử: `encoder`, `integration`, `ir_sensor`, `motor_driver`, `regression`, `speed_pid`, `web` |
| `web/` | `ble_debug_console.html`, `README.md` (**đang đóng băng phát triển** từ 29/10/2026) |
| `docs/` | Tài liệu kiến trúc và biên bản theo từng phân hệ: `classic_spp`, `clock`, `encoder`, `ir`, `maze`, `motion`, `motor_driver`, `mpu_i2c_rc2`, `system_debug`, `architecture.md`, `flash_storage.md`, `OLED_DRIVER_REVIEW.md`, `PLATFORMIO_MODE_CLEANUP_20261027.md` |
| `results/` | Dữ liệu log/CSV/JSON thực nghiệm được giữ nguyên làm ground truth (`imu_soak_*`, `system_uart_*`, `ir_timing_*`, `ir_qualification_20261027`, `ir_filter_*`, `oled_mpu_*`, `encoder_qual_*`, `encoder_p0_p3_*`, `speed_pid_final_*`, `turn_pid_*`, `ble_one_shot_*`, `classic_spp`, `flash_map_diag`, `ir_full_qualification`) |
| `.github/workflows/` | 4 CI workflows: `ble-link-preflight.yml`, `classic-spp-regression.yml`, `firmware-regression.yml`, `windows-bluetooth-diag.yml` |

---

## 3. Phân nhóm các môi trường PlatformIO (`platformio.ini`)

1. **Production:**
   - `genericSTM32F411CE`: Firmware chính, 96 MHz danh định (HSI trim25), `MPU_SAMPLING_POLICY_B=1`, `MPU_TIM11_TICK_SCHED=1`, `MPU_TEMP_MONITOR=1`, tắt OLED.
   - `production_oled_status_96m`: Bản production opt-in bật màn hình trạng thái OLED chỉ đọc (`OLED_SHARED_I2C=1`, `OLED_PRODUCTION_STATUS=1`).
2. **System / UART / Flash / Integration Diagnostics:**
   - `integration_diag_motor_off_96m`, `integration_diag_motor_on_96m`
   - `flash_map_diag_96m`
3. **Bluetooth (BLE & Classic SPP):**
   - `ble_profile_motor_off_115200_96m`, `ble_profile_motor_off_38400_96m`
   - `ble_profile_motor_on_pid_115200_96m` (giới hạn cứng PWM 30%)
   - `ble_rx_diag_96m`
   - `classic_spp_rate_diag_96m` (benchmark binary throughput 64 mức tải qua Classic SPP, motor-locked)
4. **IR / Turn / Wall / Maze / System Tests (opt-in):**
   - `ir_filter_diag_96m`, `ir_burst_diag_96m`
   - `system_test_96m` (headless PA0: 90° → 180° → 200 mm, PWM ≤ 26%)
   - `turn_pid_diag_96m`
   - `wall_pid_4cell_96m` (4×180 = 720 mm, replay trace qua JDY-33)
   - `maze_sensor_capture_96m` (lăn tay motor-locked, ghi RAM 512 frame @ 200 Hz, replay kiểm tra FNV32)

---

## 4. Tổng hợp đánh giá sơ bộ theo từng Module

### 4.1. Nền tảng hệ thống & Định thời (`board.c`, `system_timer.c`, `control.c`, `include/pinout.h`)
- **Kiến trúc thời gian thực:** `TIM11` chạy ở **1 kHz** (NVIC priority 0, `src/control.c` sở hữu), gọi `Encoder_Update()`, `Motor_Update1ms()`, `MPU6050_NotifyControlTick()`, xử lý IR scan và `Motion_Update()` → `PID_Compute()` → `Motor_SetPair()`.
- **Clock:** PLL 96 MHz danh định từ HSI trim25 (`APB1=48 MHz`, `APB2=96 MHz`), `TIM10/TIM11` tự suy từ cấu hình RCC.
- **Đánh giá sơ bộ:** Clock và TIM11 1 kHz đã qua CI và chạy ổn định trong các bài test tích hợp/IMU; chưa đo tần số HSI/PLL thực tế bằng thiết bị ngoài (oscilloscope/logic analyzer).

### 4.2. I2C1 & MPU6050 (`src/i2c.c`, `src/mpu6050.c`, `include/mpu6050_i2c_module.h`)
- **Trạng thái:** **Đã chốt bàn giao chức năng/tích hợp RC2** (`docs/mpu_i2c_rc2/README.md`). I2C1 (PB8/PB9, 0x68) chạy bất đồng bộ ngoài TIM11.
- **Bằng chứng:** Soak 180s STILL/SHADOW/LIVE (`results/imu_soak_20261019_115313/`, `123733/`) đạt ~1 kHz, 0 lỗi I2C/timeout, 0 TIM11 late/gap.
- **Còn mở:** Độ chính xác góc quay thật theo mốc chuẩn, drift nhiệt lặp lại, P95/P99 khoảng cách lấy mẫu; từng ghi nhận hiện tượng `cal=0` kéo dài trên phần cứng ở một số log system/turn test và fallback I2C ~300 kHz ở một phiên cũ.
- **Lưu ý rà soát:** Không sửa đổi nếu không có phát hiện lỗi cụ thể; tuyệt đối không tạo chủ I2C thứ hai.

### 4.3. OLED SSD1306 dùng chung I2C1 (`src/ssd1306.c`, `src/fonts.c`, `src/oled_shared.c`, `src/oled_mpu_diag.c`)
- **Trạng thái:** Nghiệm thu A/B motor-OFF 15s PASS ngày 24/10/2026 (`results/oled_mpu_20261024_082616/report.json`: 17/17 checks, 1056/1056 DMA TX thành công, gyro giảm 0,73% từ 1000 xuống 992,73 Hz).
- **Còn mở:** `production_oled_status_96m` mới chỉ build PASS; chưa kiểm tra hiển thị pixel thực tế trên panel, chưa có menu/nút bấm, chưa test motor-ON hay soak dài hạn. Trên phần cứng người dùng từng gặp `SSD1306_NOT_FOUND`, nên `system_test_96m` đã chuyển sang chế độ headless PA0.

### 4.4. Motor Driver & Encoder (`src/motor.c`, `src/motor_startup.c`, `src/motor_test.c`, `src/encoder.c`)
- **Motor Driver Rev3 (TIM2 30 kHz):** Đóng băng logic cốt lõi, hồi quy phần mềm PASS, có phản hồi encoder khi test LIVE PWM 80–100% trên bánh kê. Chưa có phép đo điện (dạng sóng, dòng, sụt áp VM 2S, nhiệt, chạy có tải trên sàn).
- **Encoder (TIM3/TIM4, `ENCODER_PPR=1430`):** Đã nghiệm thu trên bệ kê bánh (`results/speed_pid_final_20261024_003343/`, 9/9 lượt speed-PID diagnostic PASS). Đã chốt hệ số lọc `SPEED_LPF_ALPHA=0.10f` trong `src/encoder.c`. Chưa nghiệm thu quãng đường thực trên sàn, độ trượt và sai lệch hai bánh có tải.

### 4.5. Cảm biến hồng ngoại 6 kênh (`src/ir_sensor.c`, `src/ir_filter.c`, `src/ir_wall_signal.c`, `src/ir_filter_diag.c`, `src/ir_filter_burst_diag.c`, `src/ir_oled_test.c`)
- **Đây là module ưu tiên tiếp theo theo `AGENTS.md`.**
- **Trạng thái:** Đạt mức `ACQUISITION_ACCEPTED` cho diagnostic motor-locked (`docs/ir/reports/IR_VALIDATION_20261027.md`, `results/ir_qualification_20261027/acceptance.json`): G0/G1 PASS, scan ~1210 Hz, 0 lỗi/drop/clipping, có capture no-wall 30s và tĩnh 40/50 mm.
- **Còn mở:**
  - Kiểm tra kỹ lại khả năng còn chờ ADC/DMA trong ngữ cảnh TIM10 ISR (mặc dù `PROJECT_STATUS.md` ghi đã chuyển sang async state machine, `AGENTS.md` vẫn yêu cầu audit mục này).
  - Chưa đạt nghiệm thu P0 vách trắng 20 mm, chưa đủ 3 trial độc lập cho mọi cự ly, chưa đo góc/cross-talk/ánh sáng/vật liệu, chưa chốt filter hay LUT cho production/Wall PID.

### 4.6. Truyền thông Debug: USART1, BLE (USART2/JDY-33) & Python Classic SPP (`src/uart.c`, `src/uart_bench.c`, `src/bt_debug.c`, `src/ble_*.c`, `tools/classic_spp_*.py`, `web/`)
- **USART1 (PA9/PA10, 115200):** Đã bàn giao debug (60 Hz khuyến nghị, 80 Hz max PASS với khung ~75 byte, 140 Hz bắt đầu rơi).
- **Quyết định 29/10/2026:** **Đóng băng Web BLE UI** (`web/ble_debug_console.html`, `web/README.md`, `tests/web/`). Không xóa JDY-33 hay `src/bt_debug.c`.
- **Hướng ưu tiên hiện tại:** Python Classic SPP qua cổng COM Outgoing trên Windows (`tools/classic_spp_dashboard.py`, `tools/classic_spp_link_diag.py`, `tools/classic_spp_protocol.py`, `tools/classic_spp_rate_pipeline.py`).
- **Rủi ro/Giới hạn:** Dashboard hiện **RX-only**; `src/bt_debug.c` khi đầy TX queue có thể bỏ phần đuôi mà chưa có thống kê đầy đủ trong production; chiều Python → STM32 chưa được nghiệm thu; chưa test throughput/nhiễu khi motor chạy thực tế.

### 4.7. Flash Storage (`src/flash_storage.c`, `src/flash_record_format.c`, `src/flash_map_protocol.c`, `src/flash_map_diag.c`)
- **Trạng thái:** Lưu bản ghi append-only 1 KiB/slot ở sector 6–7 (tối đa 127 lần ghi/sector trước khi cần maintenance), có CRC32, commit marker, đọc lại xác minh sau ghi, khóa an toàn khi motor chạy/control active. Hỗ trợ đọc layout 5×5 cũ và map 16×16 tới 1000 byte.
- **Còn mở:** Hiện **không kích hoạt** trong runtime chính; chưa nghiệm thu ghi/đọc/reset/mất nguồn trên board thật.

### 4.8. Điều khiển chuyển động, PID & Giải mê cung (`src/motion.c`, `src/motion_primitives.c`, `src/pid.c`, `src/speed_pid_diag.c`, `src/system_test.c`, `src/turn_pid_diag.c`, `src/wall_pid_test*.c`, `src/maze_sensor_capture.c`)
- **Production runtime (`src/motion.c`):** Mới ở mức tối giản (`MOTION_IDLE`, `MOTION_STRAIGHT`, `MOTION_TURN`) với lệnh F/L/R/S/Z. Chưa có flood-fill, wall-follow hay fast-run nghiệm thu.
- **Các module thử nghiệm/chuẩn bị có mặt trên nhánh:**
  - `turn_pid_diag.c`: Có log chạy thật sớm (`results/turn_pid_20261027_*`), đang có pilot `SMOOTH v4` chưa nghiệm thu phần cứng.
  - `wall_pid_test.c` / `wall_pid_test_core.c` / `wall_pid_trace.c`: Bài test bám tường 4 ô (720 mm) dùng L0/R0 EMA 0.10, giới hạn PWM 80%, trace qua BLE — chưa nghiệm thu thực địa.
  - `maze_sensor_capture.c`: Thu thập dữ liệu lăn tay 6 kênh IR + encoder + IMU — chưa nghiệm thu hoàn chỉnh.

---

## 5. Đề xuất thứ tự đánh giá sâu cho phiên làm việc tiếp theo (Neon model)

1. **Bước 0 — Gỡ blocker `src/main.c`:** Diff và hợp nhất sạch 2 nhánh conflict (`HEAD` vs `e41c1a1f73c3c25492cf61c7dede937855026fcc`), kiểm tra build các profile chính trong `platformio.ini`.
2. **Bước 1 — Module IR 6 kênh (ưu tiên mặc định):** Audit `src/ir_sensor.c`, `src/ir_filter.c`, `src/ir_wall_signal.c`, `src/control.c` (TIM10/ADC1/DMA2 timing, ISR safety, chuẩn bị cho calibration 20 mm & filter/LUT).
3. **Bước 2 — Điều khiển & Chuyển động (`motion`, `pid`, `turn_pid_diag`, `wall_pid_test`):** Kiểm tra tính nhất quán hình học bánh xe (chú ý đang có `WHEEL_DIAMETER_MM=35` ở production vs `34 mm` ở `system_test`/`wall_pid_4cell_96m`), logic an toàn, anti-windup, state transitions và đồng bộ IMU/Encoder.
4. **Bước 3 — Truyền thông & Lưu trữ (`bt_debug.c`, `uart.c`, `classic_spp_*`, `flash_storage.c`):** Kiểm tra ring buffer, mất gói, tính toàn vẹn frame và điều kiện an toàn Flash.
5. **Bước 4 — Giao diện tích hợp với các module đã đóng băng (`motor.c`, `encoder.c`, `i2c.c`, `mpu6050.c`, `oled_shared.c`):** Chỉ kiểm tra ranh giới API và xung đột tài nguyên, không viết lại logic lõi đã bàn giao nếu không có lỗi rõ ràng.
