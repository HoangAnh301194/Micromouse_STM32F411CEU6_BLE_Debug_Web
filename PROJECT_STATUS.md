# PROJECT_STATUS — Micromouse STM32F411CEU6

**Ngày cập nhật:** 19/10/2026 (UTC+7). **Nhánh làm việc:** `giomuadongbac`. **Mã nguồn đã rà soát:** `2fca0d946f751ee4af2f356d9120eaee05c13922`. Đây là hồ sơ ngữ cảnh dự án để agent/chat mới tiếp tục mà không cần đọc toàn bộ lịch sử.

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
| **BLE USART2 PA2/PA3** | Đã có TXE IRQ + 4096B buffer, RX polling | Chưa đo thông lượng, RX overflow, mất bản tin, khi TX queue đầy hàm có thể bỏ phần đuôi mà không có thống kê |
| **OLED** | Có địa chỉ định nghĩa 0x3C, **chưa có driver OLED trong core đã rà soát** | Cần phân xử I2C1 với MPU, làm mới hạn chế và truyền không chặn |
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

MPU: kiểm tra góc quay thật ±90/180/360 độ, drift theo nhiệt độ có thử lặp, so sánh DLPF 42/98 Hz cả ở tĩnh và động. OLED: sau khi ưu tiên IR/BLE, chia lịch sử dụng bus I2C1 với MPU, refresh thấp, không giao dịch đồng bộ trong ngắt. Nguồn 2S, watchdog, an toàn motor và EMI có kế hoạch thử riêng.

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
