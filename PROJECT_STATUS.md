# PROJECT_STATUS — Micromouse STM32F411CEU6

**Ngày cập nhật:** 24/10/2026 (UTC+7). **Nhánh làm việc:** `giomuadongbac`. **Mã nguồn đã rà soát:** `2fca0d946f751ee4af2f356d9120eaee05c13922`. Đây là hồ sơ ngữ cảnh dự án để agent/chat mới tiếp tục mà không cần đọc toàn bộ lịch sử.

**Nguyên tắc:** trạng thái *đã bàn giao* chỉ áp dụng trong phạm vi đã kiểm tra; CI PASS không thay thế phép đo trên MCU và encoder có xung không chứng minh đã đo được điện áp/dòng điện. Khi có kết quả mới, cập nhật hồ sơ này và giữ nguyên các log gốc.

## 1. Nền tảng và quyền sở hữu ngoại vi

| Thành phần | Trạng thái cấu hình |
|---|---|
| MCU/Build | STM32F411CEU6, bare-metal CMSIS/PlatformIO, flash qua ST-Link, máy host Windows PowerShell |
| Clock | 96 MHz **danh định** (PLL từ HSI trim25), chưa đo độ chính xác bằng thiết bị ngoài |
| TIM11 | Ngắt điều khiển 1 kHz, ưu tiên 0; `src/control.c` sở hữu |
| TIM2 | PWM motor trái/phải 30 kHz; `src/motor.c` sở hữu |
| TIM3 / TIM4 | Encoder hai bánh; dấu tiến đã được chỉnh về dương |
| TIM10 + ADC1 + DMA2 Stream0 | Đọc 6 mắt IR; `src/ir_sensor.c` sở hữu |
| I2C1 | PB8 SCL / PB9 SDA; MPU6050 địa chỉ 0x68; OLED dự kiến 0x3C |
| USART1 | PA9 TX DMA2 Stream7 Ch4 / PA10 RX IRQ, 115200 8N1, production TX queue 1024 byte (2 KiB in `INTEGRATION_DIAG`) |
| USART2/JDY-33 BLE | PA2 TX / PA3 RX, 115200 baud, TXE IRQ, TX buffer 4096 byte |
| Nguồn | Bo thử đã dùng pin 2S cấp VM ở các bài LIVE; chưa có phép đo định lượng dòng/VM sụt áp |
| Sơ đồ chân | `include/pinout.h` là cấu hình PCB hiện hành; không tự ý thay đổi |

`src/main.c` là vòng lặp điều phối. I2C MPU được tiến hành bất đồng bộ trong main/IRQ I2C; TIM11 thông báo nhịp và dùng snapshot, không chờ I2C trong ngắt. Môi trường firmware còn dùng được phải lấy trực tiếp từ `platformio.ini`.

## 2. Ma trận tiến độ nghiệm thu

| Module | Trạng thái đã xác minh | Phạm vi còn mở |
|---|---|---|
| **Clock 96 MHz** | Tích hợp và CI thành công | Tần số thực tế của HSI/PLL chưa đo bên ngoài |
| **USART1 PA9/PA10** | **Bàn giao debug**: 60 Hz khuyến nghị, 80 Hz là tần số tối đa được benchmark PASS đối với khung ~75 byte; thử 140 Hz bắt đầu mất bản tin | Giới hạn thay đổi theo độ dài log và tải ứng dụng; cần kiểm soát đỉnh queue |
| **I2C1 + MPU6050** | **Đã chốt bàn giao chức năng/tích hợp RC2**: 180 giây STILL, SHADOW, LIVE; gần 1 kHz, I2C errors/timeouts 0, TIM11 late/gap 0; có nhiệt độ chip qua I2C | Độ chính xác góc quay thật, drift nhiệt nhiều thử nghiệm, P95/P99 khoảng cách lấy mẫu |
| **Motor Driver Rev3** | Tích hợp, hồi quy phần mềm; LIVE PWM 80–100% có phản hồi encoder khi bánh được cô lập | Dạng sóng, dòng motor, điện áp VM, nhiệt driver, phanh, chạy có tải; không đánh đồng với chứng nhận an toàn điện |
| **Encoder hai bánh** | Có xung thực và dấu tiến dương | PPR/quãng đường theo mốc ngoài, tốc độ RPM/mm/s, miss pulse, sai lệch L/R chưa nghiệm thu đầy đủ |
| **TIM11 và PID** | 1 kHz ổn định trong bài thử; PID giả lập chạy đủ tải không mất chu kỳ | Chưa nghiệm thu bộ PID tốc độ bánh, điều khiển bám tường, thời gian đáp ứng/quá độ khi di chuyển tốc độ cao |
| **IR 6 kênh** | **Acquisition/timing phase đạt trên board**: ADC1/DMA2 Stream0 async, 1 kHz scan trong 30 giây; TIM11 nhận mẫu mới 100%; zero DMA/ADC errors/timeouts, TIM11 late/gap, MPU I2C errors/timeouts, UART drops | Chưa đánh giá độ chính xác/khoảng cách, nhiễu, nhiễu chéo hay phản hồi theo tường; phase này chỉ xác nhận acquisition, nhịp và tải |
| **BLE USART2 PA2/PA3** | Đã có TXE IRQ + 4096B buffer, RX polling; web console hiển thị đồng thời ma trận 16×16 (Flood/Path/Explored, Stats) và terminal log, hai panel kéo chỉnh kích thước được, có Connect/Reconnect/Disconnect | Chưa đo thông lượng, RX overflow, mất bản tin, khi TX queue đầy hàm có thể bỏ phần đuôi mà không có thống kê; web UI mới chỉ kiểm tra trong trình duyệt, chưa nghiệm thu BLE với JDY-33 thật |
| **OLED/MPU I2C1 shared debug** | **Nghiệm thu A/B trên bo, motor khóa: PASS (24/10/2026)** theo `results/oled_mpu_20261024_082616/report.json`: 17/17 checks, TIM11 1000 Hz cả hai pha; MPU 1000 → 992.73 Hz (-0.73%); OLED DMA 1056/1056 thành công, 528 chunks, 73 frames / 15s, max transaction 161 us; không có lỗi I2C/DMA, recovery, mẫu gyro trễ hay gián đoạn TIM11. Phiên trước `081626` FAIL vẫn được lưu nguyên làm đối chứng. | **Đạt nghiệm thu dùng chung I2C trong firmware diag motor-OFF 15s**. Chưa xác minh bằng máy hiện sóng, chưa có bằng chứng hiển thị ký tự đúng bằng quan sát độc lập, chưa nghiệm thu khi motor ON / production / soak dài hạn / menu thao tác. |
| **OLED SSD1306 + renderer** | Driver SSD1306, font, incremental dirty framebuffer và DMA scheduling đã PASS kiểm thử A/B 15 s motor-OFF; log gốc `results/oled_mpu_20261024_082616/`. Không có I2C/DMA/MPU error trong bài nghiệm thu. | Chưa quan sát độc lập pixel/chữ trên panel; OLED disabled trong production; chưa có API UI/menu ứng dụng, không chứng nhận chạy motor ON hoặc soak dài hạn. |
| **Flash storage, Sectors 6/7** | Firmware linker giới hạn 256 KiB (sector 0–5); lưu Flash dùng bản ghi append-only 1 KiB/slot, CRC32, commit marker, xác minh sau ghi, timeout TIM5, PG cleanup, kiểm tra điều kiện Control stopped + Motor locked; dữ liệu legacy đọc tương thích | Chưa nghiệm thu read/write/reset/mất nguồn trên board; mỗi sector 127 lần ghi trước khi cần maintenance; chưa nối save-map khi đến đích; xem [Flash storage](docs/flash_storage.md) |
| **Wall-follow, flood-fill, fast-run** | Core tối giản có trạng thái IDLE/STRAIGHT/TURN và lệnh F/L/R/S/Z | Chưa có nghiệm thu điều khiển mê cung và chạy tốc độ cao; UI BLE cũ không chứng minh tính năng firmware đang hoạt động |

**Tài liệu bàn giao MPU/I2C có giá trị chi tiết nhất:** [docs/mpu_i2c_rc2/README.md](docs/mpu_i2c_rc2/README.md). API tích hợp tại `include/mpu6050_i2c_module.h`; giữ `src/i2c.c` và `src/mpu6050.c` ở vị trí gốc, tránh tạo thêm bộ chủ I2C.

## 3. Kết quả thực địa được lưu nguyên trạng

- **STILL/SHADOW 180 giây:** [summary JSON](results/imu_soak_20261019_115313/summary.json) · [CSV](results/imu_soak_20261019_115313/measurements.csv). 10.799 bản tin IMU 60 Hz mỗi chế độ; yaw thô sau trừ bias trôi **+0,458° STILL / +0,801° SHADOW** trong 180 giây. Yaw áp dụng vùng chết chỉ **+0,003° / −0,011°**, **không** được coi là drift thực bằng không. Đỉnh UART 965 / 979 byte trên 1024 byte, dù không có dropped messages.
- **LIVE PWM 80–100%, 180 giây:** [summary JSON](results/imu_soak_20261019_123733/summary.json) · [CSV](results/imu_soak_20261019_123733/measurements.csv). Real PWM và encoder có hoạt động; gần 1 kHz MPU không có lỗi I2C/timeout hay mất chu kỳ TIM11 trong phép thử; nhiệt độ bên trong MPU khoảng 32,345–32,460 °C. Không đo được trực tiếp từ trường EMI, điện áp/dòng motor hay góc tham chiếu độc lập.
- **Benchmark USART1:** [104312](results/system_uart_20261019_104312/uart/uart_rate_summary.json), [104529](results/system_uart_20261019_104529/uart/uart_rate_summary.json), [105726](results/system_uart_20261019_105726/uart/uart_rate_summary.json). 60 Hz có dự phòng, 80 Hz đạt, các tần số cao hơn làm queue đầy trước khi CPU thành nút thắt.
- **Motor OFF/ON trước đó:** [summary](results/system_uart_20261019_104529/summary.json). Một phiên OFF từng fallback I2C xuống gần 300 kHz; nguyên nhân cụ thể chưa khẳng định, các phiên MPU mới đạt nominal 400 kHz.
- **IR 6 kênh, motor-off, 30 giây (firmware commit `9c7862ab9cc092295c14cab707bb366582db9433`):** [summary JSON](results/ir_timing_20261019_173447/summary.json) · [UART log gốc](results/ir_timing_20261019_173447/uart.log). 29.000 scan/29 s (1 kHz), 116.000/116.000 DMA completions; scan trung bình 723,71 µs, max 724 µs; age mẫu max 293 µs, TIM11 fresh 29.000/29.000 ticks; đo cycle-counter IR ISR tương ứng khoảng 1,5965% nominal 96 MHz. DMA/ADC/timeout, TIM11 late/gap, MPU I2C errors/timeouts và UART drops đều 0; UART queue peak 1049/2047 byte. Có 30 bản tin raw đủ 6 kênh; **không** đánh giá accuracy/nhiễu. Các capture thử trước được giữ lại nhưng đánh dấu invalid trong `summary.json` do telemetry bị truncate hoặc UART queue drop.
- **CI mã nguồn tại commit rà soát `2fca0d9`:** workflow Motor Driver, Clock, System Integration và UART+MPU RC2 đều **success**. Chỉ chứng minh mã build/hồi quy, không tự chạy lại phần cứng.

**Đã dọn sau bàn giao MPU:** `tools/imu_soak_pipeline.py` và `tools/test_imu_soak_pipeline.py` cùng firmware thí nghiệm chuyên biệt không còn nằm trong working tree của nhánh hiện tại. Dữ liệu lịch sử trong `results/` **được giữ nguyên**. Tool UART/clock/motor/system khác còn tồn tại và không được xóa tùy tiện.

## 4. Thứ tự phát triển tiếp

### P0 — IR sáu kênh: acquisition/timing phase

1. **Đã làm:** thay busy-wait ADC/DMA trong TIM10 bằng state machine async, DMA IRQ, timeout/error recovery; chỉ publish đủ scan.
2. **Đã đo:** scan rate, scan/DMA durations, ISR cycles, tuổi/freshness tại TIM11, DMA/ADC lỗi, UART queue/drop, MPU/I2C và raw ADC sáu kênh; xem biên bản và log phía trên.
3. Phase tiếp theo (chưa làm trong phạm vi này): đánh giá theo khoảng cách/tường và nhiễu chéo giữa các cặp emitter; không suy ra độ chính xác từ log timing hiện tại.

### P1 — Encoder + PID điều khiển thực

Kiểm tra encoder/quãng đường bằng mốc ngoài, mm/pulse, RPM, độ lệch hai bánh, motor speed loop, yaw hold, rẽ 90/180 độ, dừng/coast; so sánh kết quả chuyển động thực với mục tiêu, không suy từ PID SHADOW.

### P1 — USART2 BLE debug

Nghiệm thu TX/RX hai chiều, hàng đợi, gói rơi khi đầy, mất byte, độ trễ lệnh STOP dưới tải; xem xét kiểm tra clock USART2, tránh chặn CPU trong các API nhận/gửi; WebBLE UI chỉ là công cụ host, chưa chứng nhận chức năng mapping.

### P2 — MPU nâng cao và OLED

MPU: kiểm tra góc quay thật ±90/180/360 độ, drift theo nhiệt độ có thử lặp, so sánh DLPF 42/98 Hz cả ở tĩnh và động. OLED: chia lịch I2C1 và refresh DMA thấp đã PASS bài A/B motor-OFF 15 s ngày 24/10/2026; giai đoạn sau mới đóng gói OLED UI API, bật trong production và nghiệm thu ảnh thật/motor ON/soak. Nguồn 2S, watchdog, an toàn motor và EMI có kế hoạch thử riêng.

## 5. Hướng dẫn cho agent / chat mới

1. Đọc **file này trước**, rồi `README.md` và tài liệu của **module đang thực hiện**; không cần đọc mọi commit cũ. Khi làm IR, đọc `src/ir_sensor.c`, `include/ir_sensor.h`, `src/control.c`, `include/pinout.h`.
2. Luôn xác minh branch/head/working tree và `platformio.ini` trên GitHub/trên máy trước khi chỉnh. Mặc định làm trên **`giomuadongbac`**. Không giả định rằng lệnh của chiến dịch đã dọn còn tồn tại.
3. Không sửa trái phép pinout, cấu hình MCU đã nghiệm thu hoặc xóa/sửa log cũ. Mọi cập nhật đo mới phải lưu thời gian, tham số, commit, các phép kiểm đạt/chưa đạt và cách tái tạo.
4. Nếu thử motor, chỉ dùng firmware cố ý cho motor chạy, khóa mặc định trong thí nghiệm sensor, bánh nâng lên và có khả năng cắt nguồn VM độc lập. Phân biệt **lệnh PWM**, **encoder phản hồi**, **đo điện**, **chuyển động thật trên sàn**.
5. Không tuyên bố nghiệm thu điều khiển tốc độ cao từ build PASS, từ 180 giây quét cảm biến tĩnh, hoặc từ bài PID giả lập.
6. Sau mỗi mốc bàn giao, cập nhật `PROJECT_STATUS.md`, README module, danh sách giới hạn còn mở. Không force-push làm mất thay đổi đồng thời.

**Câu mở đầu khuyến nghị cho chat/agent kế tiếp:** “Tiếp tục module IR trên nhánh giomuadongbac. Đọc PROJECT_STATUS.md và docs/system_debug/IR_SENSOR_TIMING_20261019.md; acquisition/timing đã đạt smoke-test board, phase sau mới kiểm tra tường/accuracy/noise. Giữ nguyên dữ liệu nghiệm thu I2C–MPU6050.”

## 6. Encoder software pipeline — khởi tạo 20/10/2026

- Pipeline mới tại `tools/encoder_qualification_pipeline.py` thực hiện audit nguồn, kiểm thử C-native bằng mã thực `src/encoder.c`, build firmware, nghiên cứu PPS mô phỏng, thu UART và phân tích sweep PWM lịch sử. Xem `docs/encoder/QUALIFICATION_PIPELINE_20261020.md`.
- Profile riêng `integration_diag_encoder_motor_off_96m` kế thừa khóa motor đầu ra, bổ sung `SYS_EN` khoảng 50 Hz trong main. Production, Motor Driver, I2C/MPU và IR không thay thuật toán.
- Có workflow CI host/build nhưng **chưa nghiệm thu profile mới trên board**. Phân tích 50 Hz không chứng minh không mất cạnh xung hoặc độ chính xác PPS ở 1 kHz.
- Giữ nguyên `ENCODER_PPR=1430` và dấu encoder. Chưa thay EMA 1 ms, IC1F/IC2F hoặc PID khi chưa có phép đo so sánh thực.
- Motor-on phải cấp quyền riêng: bánh nâng/cố định, cắt VM độc lập, người giám sát và chấp nhận sweep thực tới 100% PWM.

**Cập nhật Encoder motor-on pipeline:** Profile mới `integration_diag_encoder_motor_on_96m` cho phép ghi `SYS_EN` khoảng 50 Hz trong sweep motor thật có `MODE ON` + `START` qua UART. Lệnh tổng hợp dùng `--motor-on-sweep --fixture-ready`, không cần quay tay và không yêu cầu xác nhận riêng cho 100% PWM. Các điều kiện bánh nâng/cố định, cắt nguồn VM độc lập và có người trực vẫn bắt buộc. Chưa chạy nghiệm thu trên board bằng profile mới; kết quả CI/build chưa được xác minh tại thời điểm cập nhật.

- **Bổ sung chẩn đoán I2C cho lượt Encoder OFF/ON kế tiếp:** riêng các profile `ENCODER_PROFILE_DIAG` xuất `SYS_IPB` mỗi giây, gồm tốc độ MPU chọn và trạng thái ACK/WHO_AM_I của các mức 400/300/200/100 kHz. Lý do: log `encoder_qual_20261020_014805` ghi ON `iclk=296296` thay vì 400000, trong khi `i2cerr=to=0`; không kết luận do motor EMI khi chưa có probe và bằng chứng độc lập. Wrapper giữ nghiêm điều kiện I2C 400 kHz, tách cảnh báo tốc độ khỏi kết luận dữ liệu Encoder khi toàn bộ 33 bước và telemetry 50 Hz đầy đủ. Chưa chạy firmware mới trên phần cứng.

### Encoder EMA alpha + I2C multi-diagnostic A/B — source ready 20/10/2026

- Added **diagnostic-only** `ENCODER_ALPHA_AB_DIAG=1` to OFF/ON Encoder PlatformIO profiles; production motor/PID and `SPEED_LPF_ALPHA=0.30` remain unchanged.
- Every TIM11 1-ms tick: run shadow EMA `0.05/0.10/0.20/0.30/0.40/0.60/0.80` plus 5/10/20-ms rolling encoder count windows on both wheels. Cumulative per-interval error-versus-boxcar and 1-ms variation are emitted gradually via `SYS_AM`; snapshot comparisons via `SYS_AB` at 10 Hz.
- I2C startup probe `SYS_IPB` supplemented with diagnostic-only `SYS_IB` (SR1/SR2, BUSY timeout/recovery counters, PB8/PB9 levels). No production I2C timing or recovery policy changed. Baseline `022526` ON saw initial 400-kHz I2C_BUSY and successful 296-kHz fallback.
- New `tools/encoder_shadow_ab.py` + synthetic regression generate per-wheel/per-PWM alpha comparisons and **provisional** top two. Scores use a dependent trailing 20-ms Encoder count reference, not externally measured RPM; never auto-apply recommended alpha.
- Retain 13 prior gates plus shadow regression and both diagnostic builds, 33×OFF +33×ON stages, all UART safety checks, and locked-OFF firmware restoration. Raw and CSV logs remain available even on a flagged I2C fallback.
- GitHub Actions run **39315053774** on commit `5906d0f3` completed successfully for host/native checks, diagnostic builds and production build (software CI only). **Hardware A/B and I2C 400-kHz acceptance have not yet been run/verified for the new implementation.**
- Detailed protocol: `docs/encoder/ALPHA_AB_I2C_MULTIDIAG_20261020.md`.

### Lượt chẩn đoán Encoder A/B 030513 — thực nghiệm 20/10/2026

- Lượt đầu tiên với `ENCODER_ALPHA_AB_DIAG` (commit đo `697d9dd`):
  phần mềm/build PASS, OFF 33 giai đoạn PASS, ON 33 giai đoạn hoàn tất;
  trả `ABORTED_OR_INCOMPLETE` ở host do health UART ON:
  `rx_err=24`, `tx_drop=1`. Không phải lỗi đếm xung Encoder hay mất
  chu kỳ TIM11 (`late=gap=0`); không được tuyên bố nghiệm thu hệ thống.
- I2C 400 kHz **đã được chọn** ở cả OFF/ON, lỗi đọc/timeout bằng 0.
  Công đo A/B tăng chi phí Encoder IRQ: ON `encmaxcy=8235` vs
  baseline `971`; toàn TIM11 max `8965` vs `1905`.
- Không thể chốt `alpha`: pipeline cũ abort trước bước phân tích shadow,
  và CSV đo tổng quát bỏ các cột `SYS_AB/SYS_AM`. Các log UART gốc
  chưa được push trong bộ `030513`.
- Cập nhật host tiếp theo: lưu `encoder_shadow_off.csv/on.csv` đầy đủ,
  bảo toàn phân tích alpha khi health UART thất bại, vẫn báo chưa nghiệm thu.
  Có thể chạy lại riêng `tools/encoder_shadow_ab.py` trên log địa phương,
  **không cần chạy lại motor**. Chưa thay `SPEED_LPF_ALPHA=0.30`.

### 2026-10-20 continuation: refined alpha bank and RX isolation

- Ten EMA candidates in diagnostic-only build: `0.05,0.08,0.10,0.12,0.15,0.20,0.30,0.40,0.60,0.80`; source remains `SPEED_LPF_ALPHA=0.30` in production and PID.
- Emission of optional `SYS_AB` snapshots reduced from 10 to 5 Hz and gated by UART queued-byte watermark; repeated `SYS_IPB/SYS_IB` interval extended to 10 seconds. Required 1kHz bank statistics and safety counters retained, no UART baud/IRQ priority changes.
- Added `SYS_UR` ORE/FE/NE/RX-ring overflow evidence to distinguish USART hardware error classes; strict RX/TX health acceptance unchanged.
- Decoder supports historical 7-alpha and refined 10-alpha records, includes invalid frame checks; reports actual alpha ranking separately per wheel and PWM. No alpha is deployed to production.
- Verified host/native simulation, Encoder OFF/ON build and production build by GitHub Actions run `39354165109` at commit `23694ef5`: PASS. **Physical ten-alpha/UART optimized run not yet performed**; do not claim a hardware pass.
- Protocol: `docs/encoder/ALPHA_AB_I2C_MULTIDIAG_20261020.md`.

### 2026-10-20 — TIM11 Encoder shadow A/B IRQ offload (software ready)

- Last captured full 10-alpha implementation: `083902` (and partial `085212`);
  `083902` Motor-ON stopped after stage 14 with
  `SWEEP_UART_LINK_LOST=9`, and 18 hardware UART RX ORE flags.
  Measured ON `TIM11 maxcy=11741`, Encoder `encmaxcy=10999`
  at nominal 96 MHz (~122.3/114.6 us), both longer than one
  115200-8N1 byte (~86.8 us). TIM11 NVIC priority 0 can delay
  RXNE IRQ at priority 8. This remains a causal **hypothesis**, not
  a measured proof of when RXNE was delayed.
- Moved ten diagnostic-only EMA and boxcar calculations out of TIM11:
  1-ms ISR now enqueues signed left/right count deltas in a static
  256-slot SPSC ring (255 effective slots); main processes maximum
  12 ticks each iteration and up to 32 at stage telemetry boundaries.
  Production Encoder raw counts and `SPEED_LPF_ALPHA=0.30f`
  are unchanged.
- UART PING parser executes before unchanged 750-ms sweep lease
  enforcement. No watchdog timeout inflation, UART baud/NVIC rewrite,
  Motor Driver changes or MPU speed policy changes.
- Per-second `SYS_AQ` records accepted/consumed/overflow/backlog/
  stage edge alignment; hardware PASS requires zero `lost` and
  zero `edge`. Host retains these in raw/CSV and reports.
- Native regression extended to compile **both** regular Encoder and
  diagnostic Encoder source and exercise FIFO overflow, delayed
  consumer, bounded service and 10k quadrature updates.
  Host audit/diagnostic builds/production build are validated by
  CI; **real TIM11 runtime reduction and UART zero-ORE are NOT
  yet hardware-qualified.**
- Motor-OFF-only next test (physically disconnect VM):
  `python tools/system_uart_pipeline.py --port COM7 --flash --encoder-telemetry`.
  Inspect `SYS_T.encmaxcy`, `SYS_UR.ore`, `SYS_AQ`,
  and firmware heartbeat before considering any live motor sweep.
- Full protocol: `docs/encoder/ALPHA_AB_I2C_MULTIDIAG_20261020.md`.

### 2026-10-20 — P0→P3 sequential Encoder qualification (source/CI qualified; physical pending)

- Added `tools/encoder_p0_p3_pipeline.py` as **one gated command**: offline/native
  tests + capped profile build, **P0 Motor-OFF** real TIM11/Encoder/FIFO/UART
  acceptance, **P1 12-stage LEFT/RIGHT/BOTH 0–30%** motor response,
  **P2 original 33+33-stage 0–100%** OFF/ON benchmark, **P3 ten-alpha** A/B
  shortlist (no PID/production alpha modifications).
- New `integration_diag_encoder_motor_on_limited_30_96m` uses a compile-time
  `SWEEP_STEPS_PER_GROUP=4` vs default 11, is motor-ON/Encoder-diagnostic
  only, and never emits duty above 30%. `CMD_STATUS.max_pwm/total` is
  checked before START; host halts on wrong profile.
- Before flashing ANY motor-ON firmware, the OFF phase must PASS its 33
  normal health gates *and* measured Encoder TIM11 max under 2,000 cycles.
  P1 must then PASS all 12 stages, zero UART RX/TX faults, no heartbeat
  degradation, zero FIFO loss/edge and sensor/clock/watchdog faults.
- P2 full-range is enabled **only after P0/P1**, and requires the operator's
  explicit `--fixture-ready --independent-vm-cutoff --allow-100-percent`
  acknowledgement. This is a real 100%-PWM test after limited gate, not
  automatically physically safe.
- P3 is an **open-loop 20-ms count-proxy** recommendation from the P2
  `792/792` ten-alpha shadow metrics and per-wheel rank consensus. It
  intentionally **does not** alter `SPEED_LPF_ALPHA=0.30` or PID
  coefficients without a separate closed-loop validation.
- Per-step logs, raw UART, shadow CSV, summary, shortlist and attempted
  motor-OFF firmware restoration are saved under a new
  `results/encoder_p0_p3_YYYYMMDD_HHMMSS` directory.
- CI run `39360810947` verified host/orchestrator tests, limited-profile
  build and production build; motor integration workflow `39360810939`
  verified native state-machine tests. **No physical P1/P2 rerun is
  claimed by these checks.**
- Operational protocol: `docs/encoder/ENCODER_P0_P3_SEQUENTIAL_20261020.md`.

### 2026-10-23 — Final Encoder EMA test uses a TRUE diagnostic closed-loop speed PID

- Audited `src/motion.c`: existing Motion PID controls heading and turn,
  not wheel speed; prior PWM/alpha P0–P3 comparison was open-loop.
  Treating the old A/B score as final PID speed proof would be incorrect.
- Added three strictly isolated opt-in builds `speed_pid_alpha_008_96m`,
  `speed_pid_alpha_010_96m`, `speed_pid_alpha_030_96m` with the same
  `PID_Compute`, fixed exploratory gains Kp 0.050 / Ki 0.080 / Kd 0,
  10ms main-context speed control and 800/1600/2400 PPS targets.
  Firmware limits commanded PWM to 30%, retains 750ms PING/STOP/FAULT
  sticky lock, Motor Driver 300ms lease and MPU/TIM11/I2C safety.
- Diagnostic-only compile-time Encoder `SPEED_LPF_ALPHA` is 0.08, 0.10
  or 0.30; **production remains 0.30** and its heading/turn PID code
  and gains are untouched.
- `tools/speed_pid_final_pipeline.py`: 3 counterbalanced repeats each,
  9 total supervised real closed-loop runs. Per-run exact STM32
  `CMD_PIDCFG` firmware alpha/gains/PWM ceiling validation before
  `START`, 20Hz `SYS_PID` target/EMA/50ms count-delta/commanded+applied
  PWM, strict UART/TIM11/heartbeat/Encoder stage gating, independent
  motor power cutoff prompts before/after each firmware flash.
- Decision based on fixed normalized tracking MAE, overshoot, settling
  and ripple metrics. Requires >=5% score advantage and winning >=2/3
  repeats, otherwise `INCONCLUSIVE`. No automatic production update.
  This test cannot establish external shaft RPM or traction under load.
- Offline CI tests and builds pass in Encoder Actions run `39989165294`;
  **new 9-run physical trial NOT yet performed**. Source audit,
  protocol and commands: `docs/encoder/CLOSED_LOOP_SPEED_PID_FINAL_20261023.md`.


### Flash map 5×5/16×16 USART1 / WebBLE diagnostic

Source added: dedicated motor-locked `flash_map_diag_96m` profile, deterministic fake maps, UART `FM_*` read/write/read-back telemetry, BLE `MAPCFG/MAPROW/MAPEND` auto-size and CRC32 gate, serial operator pipeline. Write requires `SAVE N CONFIRM`, no sector erase or boot-time writes. On-board Flash/JDY-33 tests remain pending; see `docs/system_debug/FLASH_MAP_WEB_PIPELINE.md`.

### 2026-10-24 — Encoder acceptance on lifted-wheel fixture; production EMA selected

- **Accepted on fixture:** Encoder 1kHz sampling/counts/PPS estimate and
  bounded TIM11 integration, based on `results/speed_pid_final_20261024_003343`.
  9/9 live closed-loop speed-PID diagnostic runs PASS; 12/12 stages each;
  zero hardware UART, TIM11-gap and loss-of-link faults; P0 Motor-OFF PASS.
- **Engineering selection:** default `src/encoder.c` production
  `SPEED_LPF_ALPHA=0.10f` (previously 0.30f). Retain the three isolated
  A/B diagnostic overrides 0.08, 0.10, 0.30; no production wheel-speed
  PID has been implemented or tuned here.
- Original score: 0.10=0.03224, 0.08=0.03270, 0.30=0.03383.
  The best-vs-runner-up margin is **1.41%, below the predefined 5%**:
  machine decision correctly remains `INCONCLUSIVE`.
  Thus acceptance covers Encoder operation, **not proof of a
  unique EMA optimum**; alpha 0.10 is an explicit engineering choice.
- Bench raised-wheel scope only: ground-distance metrology, slip,
  production motion control and loaded validation still pending.
  Follow full acceptance record
  `docs/encoder/ENCODER_ACCEPTANCE_20261024.md`.

## 2026-10-24 — BLE USART2 one-shot profiler (software handoff; hardware PENDING)

- Created isolated `BLE_PROFILE_DIAG` builds and a one-command `tools/ble_full_pipeline.py` harness on feature branch. The host connects JDY-33 directly by Bleak FFE1, runs 18 packet/rate tests at 115200, six at 38400, and optionally six during a guarded 30%-PWM real 10ms speed-PID integration sweep. It restores the motor-locked image after automated bench phases.
- USART2 PA2/PA3 now accepts/rejects complete TX frames atomically, receives PA3 using RXNE interrupt ring and counts TX queue, hardware errors and ISR cycles. Historical production BLE debug remains text, 10 Hz. TIM11 runtime histogram is diagnostic-only.
- **No real BLE hardware log or CPU/latency qualification yet.** GitHub CI/Python synthetic tests do not establish real over-air performance; DMA1 Stream6 conflicts with I2C1_TX and is deliberately not changed. The browser text Web Console does not parse this binary protocol. See [BLE one-shot procedure](docs/system_debug/BLE_ONE_SHOT_PIPELINE_20261024.md).
- Motor-ON phase remains optional and requires explicit `--motor-on --fixture-ready --independent-vm-cutoff --allow-30-percent`, supervised lifted wheels, separate VM cutoff, and real BLE heartbeat. Do not run free-moving tests using the attached ST-Link multi-flash procedure.


### 2026-10-24 — Cleanup of completed host debug tooling

By user request, all 33 files in `tools/` were retired from the
active `giomuadongbac` branch. The full pre-cleanup state is preserved at
`archive/debug-tools-precleanup-20261024`. The `results/` logs and data are **unchanged**.
The functioning JDY-33 5x5/16x16 WebBLE map viewer is retained
unchanged at `web/ble_debug_console.html`, the independent native C
regression runners are moved to `tests/regression/` and the Web map
contract check to `tests/web/`. The 6 legacy debug CI workflows were
replaced by a single regression and firmware-build workflow. The
production `alpha=0.10` Encoder estimator, hardware drivers, Flash
storage map APIs and optional diagnostic firmware profiles were not
modified. Historical commands under `tools/` no longer execute in
the active branch; retrieve them from the archive as needed.


### Phục hồi pipeline map Flash theo yêu cầu người dùng (2026-10-24)

`tools/flash_map_pipeline.py` đã được lấy lại từ `archive/debug-tools-precleanup-20261024` về `giomuadongbac`, còn giao diện BLE vẫn ở `web/ble_debug_console.html`, test chạy ở `tests/regression/flash_map_protocol_test.py`. Pipeline mặc định chỉ preview RAM/đọc Flash; ghi thử yêu cầu `--write-size`, `--allow-flash-write` và `--backup-confirmed`. Việc kiểm thử UART/BLE/Flash trên MCU thực tế vẫn chưa nghiệm thu. Các tool debug khác vẫn ở archive.

### 2026-10-24 — Full debug tool suite restored at user request

- 33/33 host files under `tools/` are present again on `giomuadongbac`, from immutable `archive/debug-tools-precleanup-20261024`. The initial two Flash-map-related files were preserved, and the remaining 31 were restored by PR #8. The previous cleanup entries above are historical; the statement that only Flash-map is restored is superseded.
- Available pipelines include `tools/ble_full_pipeline.py`, `tools/system_uart_pipeline.py`, `tools/speed_pid_final_pipeline.py`, `tools/flash_map_pipeline.py`, encoder/IR benchmarks and host tests. Production firmware, IR OLED changes, current WebBLE viewer, newer regression tests, CI and historical results have not been reverted.
- This is a source restoration; **no real BLE/STM32 qualification** has been performed by the restoration itself. Follow each benchmark's hardware safety preconditions.

## Bàn giao OLED / dọn tool trùng lặp — 24/10/2026

- **Chốt nghiệm thu phạm vi đã đo:** OLED ↔ MPU6050 trên một I2C1, motor **LOCKED**, OFF 15 s + ON 15 s, `results/oled_mpu_20261024_082616/`: pipeline verdict PASS 17/17 checks; TIM11 đúng 1000 Hz, gyro -0,73% (1000 → 992,73 Hz), OLED 1056/1056 DMA TX thành công, 73 frames, max TX 161 µs; không có lỗi bus, DMA, MPU, mất chu kỳ hay stale gyro. Giữ nguyên cả log FAIL các lần trước để truy vết.
- **Chưa nghiệm thu ngoài phạm vi:** độ đúng trực quan ký tự/pixel SSD1306, nhiễu motor đang chạy, chạy bền nhiều giờ, dạng sóng điện, UI API/menu và OLED production (vẫn disabled); không gộp các hạng mục này vào PASS.
- **Dọn thừa theo yêu cầu:** xóa tám bản sao trùng/đã thay thế (`tools/ble_debug_console.html`, `tools/test_ble_map_web.js`, và sáu native test có sẵn bản chuẩn ở `tests/regression/`). Chuyển đường dẫn gọi ở hai pipeline cũ sang `tests/regression/`; cập nhật docs/workflow. Còn **26 file** trong `tools/` tính cả `README.md`. Các pipeline riêng cho OLED, Flash, BLE, motor, encoder, PID, IR, UART và toàn bộ `results/` giữ nguyên; bản đầy đủ có ở nhánh `archive/debug-tools-precleanup-20261024`.

## 2026-10-26 — OLED System Test (source + CI ready; hardware PENDING)

- Opt-in profile `system_test_96m`: `src/system_test.c` and `include/system_test.h`,
  reused shared-I2C OLED, MPU6050, 1kHz TIM11, RC3 Motor, encoder.
- Three PA0 menu modes: turn +90°, +180° (IMU yaw PID, encoder not used as angle feedback)
  and straight 200 mm (IMU heading PID plus encoder distance P feedback).
- **Test-only geometry** radius=34mm and 1430 decoded counts/rev; expected
  encoder mean 1338.79 counts for 200mm. Production `WHEEL_DIAMETER_MM=35`
  remains unchanged. Physically confirm wheel radius and decoded CPR before tests.
- Two separate long-press-release actions required: MENU -> READY -> RUN;
  any PA0 press while running stops from TIM11; UART 'S' stops only, cannot start.
  Motor outputs locked between tests; 26% maximum commanded PWM; gyro
  freshness check <=3ms, test timeout and fault locking.
- Uses `OLED_SharedService` outside TIM11; no blocking OLED rendering during
  sensor acquisition. If OLED unavailable / bus below 300kHz, arming denied.
- Software build **PASS** in GitHub Actions run 40347581084 (commit `0efdf92c`);
  a subsequent safety check guards IMU NaN and one-wheel encoder stall.
  Physical motor-on, OLED coexistence, yaw sign, position accuracy, stop latency,
  loaded VM/duty and repeatability **not yet measured**.
- Documentation: [System Test OLED](docs/system_debug/SYSTEM_TEST_OLED_20261026.md).

## 2026-10-26 — Headless PA0 System Test (replaces OLED mode for `system_test_96m`)

- User hardware reported repeated `SSD1306_NOT_FOUND` / `IR_OLED oled=0`.
  The opt-in system test no longer initializes OLED or requires a display.
- Single >=40ms PA0 press-release triggers one run after MPU calibration and
  fresh gyro snapshot. Success advances 90° -> 180° -> 200mm; fault/STOP retries
  the same mode. Any PA0 press during RUN locks motor in 1kHz TIM11; UART1 `S`
  stops but never starts motors. Boot-held / pre-calibration buttons cannot arm.
- UART1 PA9/PA10 115200 sends `ST,CAL`, `ST,READY`, `ST,START`,
  ~20Hz `ST_DATA`, `ST,RESULT`, and `ST,READY_NEXT`.
- IMU sample freshness <=3ms and successful nonblocking calibration are required.
  Earlier user logs displayed `cal=0` even after 40 seconds: this needs separate
  hardware inspection before attempting any live motor test.
- Wheel radius=34mm, 1430 decoded counts/rev only in test module.
  PID gains, real motion, stop behavior and geometric accuracy **UNVERIFIED**.
- See [headless PA0 protocol](docs/system_debug/SYSTEM_TEST_OLED_20261026.md).

## 2026-10-26 — Isolated Turn PID pipeline (bench/floor acceptance PENDING)

- New `turn_pid_diag_96m` firmware: gyro-only 90°/180° turn, without OLED,
  selectable BASE/FAST initial profiles via UART (selection never starts motor),
  physical PA0-only start and 1kHz PA0 stop; requires fresh calibrated MPU.
- Rate trajectory (outer angle P, bounded angular velocity + braking envelope,
  gyro rate PI Kp with Ki=0 initial, feedforward), hard 260/300 permille caps.
  During stop, motors lock; 800ms inertial coast trace is logged.
- `tools/turn_pid_pipeline.py` captures 50Hz production-timestamped
  `TP,D` records, raw UART, per-run CSV/plots, summaries for overshoot,
  10-90 rise, settlement, peak rate, coast drift and telemetry drops.
  Synthetic host tests `tools/test_turn_pid_pipeline.py`.
- No physical motor-on acceptance yet. Hardware caution: prior logs showed
  MPU `cal=0` persisting; `TP,CAL_OK` is required or PA0 cannot start.

- Turn logger follow-up: hardware-stop fallback allows result emission
  if MPU loses samples during 800ms post-stop coast. Session ZIP contains
  all raw CSV, plots, per-run summary and group mean/SD. Isolated firmware
  remains physically UNTESTED.

## 2026-10-27 — MPU gyro actual fresh-publish timing in turn qualification

Added MCU-windowed 1Hz reporting of distinct published gyro snapshots and
completion-time gaps during PA0 90/180-turn tests. This measures observed
post-I2C publication frequency, **not the MPU internal nominal 1kHz rate**.
Tracks gyro sequence gaps, DATA_RDY not-ready, I2C errors, last-sample age,
>1.5/>2ms timing gaps; no controller gain or motor cap changes.
Host serial parser appends `imu_windows.csv` to each run folder and
frequency, timing jitter and age statistics to summary/ZIP, backwards
compatible with old turn traces.
