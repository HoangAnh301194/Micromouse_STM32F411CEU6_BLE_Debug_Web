# Nghiệm thu Bluetooth Classic SPP realtime — TIM11 + IMU + IR 1 kHz (28/10/2026)

**Nhánh chuẩn:** `giomuadongbac`  
**MCU:** STM32F411CEU6 96 MHz, TIM11 1 kHz, IR sáu kênh ADC1/DMA, MPU6050 I2C1 asynchronous, JDY-33 USART2 PA2/PA3 115200 8N1.  
**Mục tiêu:** Đo giới hạn telemetry SPP khi hệ cảm biến và vòng điều khiển vẫn đang chạy, không nhầm tốc độ UART với khả năng xử lý realtime trên MCU.

## Tình trạng nghiệm thu

- Firmware và Python benchmark: triển khai nguồn; **chưa có log phần cứng cho profile này**.
- Liên kết Classic SPP COM16 trước đây: nhận **1.955 byte trong 40 s** ở tải `PID,READY`, chỉ chứng minh kết nối hoạt động, không xác nhận 1 kHz khi telemetry tải cao.
- Trường hợp này cố ý **khóa motor**. Nhận mẫu PWM trong khung nhị phân nhưng các giá trị áp dụng phải bằng 0; **không phải phép đo lúc robot đang chạy**.
- Giá trị 40% dung lượng 8N1 là giới hạn đề xuất kỹ thuật để sàng lọc, **không phải mức đã nghiệm thu**.

## 1. Các file

| File | Vai trò |
|---|---|
| `src/ble_benchmark.c` | Tái sử dụng firmware đo 1 kHz; thêm tự động chạy bộ test SPP trên PA0; motor-OFF |
| `platformio.ini` | Profile `classic_spp_rate_diag_96m` |
| `tools/classic_spp_rate_pipeline.py` | Nhận dữ liệu binary CRC16 qua COM SPP, xuất CSV/JSON, đánh giá độ trễ vòng điều khiển và tần số IMU/IR |
| `tools/ble_full_pipeline.py` | Bộ giải mã giao thức binary `B5 62` đã có; được tool SPP tái sử dụng |
| `tests/regression/classic_spp_rate_pipeline_test.py` | Test offline chấm điểm, parser phân mảnh và các ca lỗi |
| `.github/workflows/classic-spp-regression.yml` | Host regression |
| `.github/workflows/firmware-regression.yml` | Build firmware và chạy regression không cần board |

## 2. Nguyên tắc an toàn

1. **Ngắt nguồn VM motor vật lý** trước khi nạp/test lần đầu; firmware cũng khóa đầu ra motor. Không chạy motor, không điều khiển từ PC.
2. Đặt robot cố định để MPU6050 có thể hiệu chuẩn gyro; cấp nguồn ổn định cho board và JDY-33.
3. Sau khi nạp firmware, **rút ST-Link nếu muốn**; công cụ đo chỉ cần Bluetooth SPP, không cần USB–TTL.
4. Tắt Web BLE và Dashboard trước khi chạy pipeline vì COM cần độc quyền.
5. Không gửi lệnh SPP vào board trong bài test: firmware thử tải **tắt USART2 RX**; chỉ PA0 vật lý kích hoạt thu phát. Phương án này tránh phụ thuộc lỗi chiều xuống JDY-33 chưa nghiệm thu.

## 3. Nạp firmware benchmark (ST-Link cần một lần)

PowerShell trong thư mục gốc repo:

```powershell
cd D:\MICROMOUSE_26\code_mau
git switch giomuadongbac
git pull --ff-only origin giomuadongbac
python -m pip install platformio pyserial
python -m platformio run -e classic_spp_rate_diag_96m
python -m platformio run -e classic_spp_rate_diag_96m -t upload
```

Chỉ nạp khi ST-Link kết nối chính xác và nguồn VM motor đã ngắt. Nạp profile này **thay thế firmware đang chạy**, không ảnh hưởng code ở nhánh phụ.

Nếu muốn quay về firmware production sau khi kiểm tra, nạp lại đúng profile trước đó bằng ST-Link khi đã đảm bảo an toàn, không giả định một chế độ motor có cùng safety gates.

## 4. Thu dữ liệu không dây trên COM16

Windows phải được Pair với **`Nezumi_Chan_SPP`** và COM16 phải là **Outgoing**. Số COM có thể thay đổi; dùng `bthprops.cpl` kiểm tra.

```powershell
python tools/classic_spp_link_diag.py --list
python tools/classic_spp_rate_pipeline.py --port COM16 --seconds 220
```

Quy trình:

1. Bật nguồn robot khi đang đứng yên; chờ quá trình hiệu chuẩn MPU6050.
2. Kết nối pipeline Python, chờ console hiện **`!Q,state=READY,ready=1,locked=1`**.
3. **Nhấn rồi thả PA0 một lần**. Không nhấn trong lúc hiệu chuẩn.
4. Firmware tự chạy **20 giai đoạn**: baseline không có gói binary, các độ dài 20, 32, 64, 96, 128 byte và nhiều mức tần số 20–300 Hz.
5. Mỗi giai đoạn đo **4 giây**, có khoảng nghỉ **2 giây**, chờ pipeline nhận `!Q,state=DONE`.
6. Đọc báo cáo tại `results/classic_spp_rate_YYYYMMDD_HHMMSS/`.

**Quan trọng:** Không nhấn PA0 cho tới khi Python đã mở cổng COM; nếu khởi động trước, có thể thiếu các bản tin đầu khiến kết quả `INCOMPLETE`. Khi làm lại, reset board rồi kết nối và đợi trạng thái READY.

Có thể chọn thư mục riêng:

```powershell
python tools/classic_spp_rate_pipeline.py --port COM16 --seconds 220 --output results\classic_spp_rate_trial_01
```

## 5. Các cột đánh giá

- `ctrl_Hz`: chênh lệch `Control_DebugStats.ticks` giữa hai mốc MCU trong ca đo / thời gian thực của ca; mục tiêu 1000 Hz.
- `control_late`: số tick lệch khỏi chu kỳ mục tiêu hơn ±50 µs theo thống kê TIM11; so với baseline.
- `control_gap`: số khoảng cách giữa các lần TIM11 >1500 µs; **phải bằng 0** trong ca ứng viên.
- `irq_gt48_per_k`: sự kiện thời gian chạy TIM11 ISR >48 µs trên mỗi 1000 tick, từ histogram hai snapshot; so với baseline.
- `imu_fresh_Hz`: delta số bản ghi gyro mới từ MPU6050, **không phải** tần số gửi telemetry.
- `ir_Hz`: delta số lượt quét IR thành công.
- `ir_fresh_Hz`: delta số lần control nhận IR mới.
- `ir_fail`: lỗi quét IR trong ca, yêu cầu 0.
- `host_received`: số gói binary CRC16 hợp lệ nhận về Python.
- `tx_rejected`, `uart_drop`, `scheduler_skipped`: gói mất ở phía STM32/hàng đợi hoặc lịch gửi.
- `host_gap_p50_ms/p95_ms/p99_ms`: khoảng cách **lúc tới PC**, không phải độ trễ truyền một chiều.

Dữ liệu đầu ra: `raw.bin`, `frames.csv`, `events.json`, `cases.csv`, `summary.json`. `summary.json` nêu mức Hz ứng viên cao nhất theo từng độ dài gói. Các ca có `CANDIDATE` **chỉ đạt ngưỡng sơ bộ khi motor bị khóa**.

## 6. Ma trận tải và đề xuất ban đầu

Giới hạn UART 115200 baud, 8N1: **11.520 B/s** (`115200 / 10`); chưa tính phần bản tin ASCII trạng thái và overhead radio.

| Frame tổng (B) | Mức khảo sát tiêu biểu (Hz) | Mức ứng viên trước khi đo | Tải ở ứng viên |
|---:|---|---:|---:|
| 20 | 100, 200, 300 | **200 Hz** | 4.000 B/s = 34,7% |
| 32 | 50, 100, 120, 180 | **120 Hz** | 3.840 B/s = 33,3% |
| 64 | 20, 40, 60, 80, 120 | **60 Hz** | 3.840 B/s = 33,3% |
| 96 | 25, 40, 50 | **40 Hz** | 3.840 B/s = 33,3% |
| 128 | 20, 30, 40, 80 | **30 Hz** | 3.840 B/s = 33,3% |

Các ca >40% dùng để tìm **giới hạn**, không tự động đưa vào cấu hình điều khiển. Không tối ưu trên mỗi giá trị bảng mà không có log thực tế.

**Khuyến nghị tạm thời:** 64 B × 50–60 Hz cho telemetry hỗn hợp gồm encoder, IMU, IR sáu kênh và PWM; 20–32 B × 100–200 Hz cho giám sát ngắn ít trường. TIM11 và lịch cảm biến vẫn mục tiêu 1 kHz; **không gửi 1000 gói/giây qua UART 115200**.

## 7. Tiêu chuẩn nghiệm thu tạm thời

Chỉ coi là ứng viên khi đồng thời thỏa mãn:

- Có đủ marker đầu/cuối của ca đo, snapshot hệ thống và CRC16, không bị thiếu file.
- `ctrl_Hz` trong vùng 990–1010; `control_gap=0`; jitter không tăng đáng kể so với baseline.
- `imu_fresh_Hz`, `ir_Hz`, `ir_fresh_Hz` đều tối thiểu 980 Hz và không giảm >2% so với baseline.
- Không phát sinh lỗi IR hoặc UART drop; gói về PC so với số gói MCU đã nhận gửi đạt ≥99%.
- Tải binary không vượt 40% lý thuyết của UART; UART queue không vượt 75% bộ đệm.

Đây là các **ngưỡng đánh giá thử nghiệm**, không phải đặc tính phần cứng được đảm bảo. Nếu baseline không đạt 1 kHz, không được đổ lỗi cho Bluetooth chỉ vì test tải cao bị fail.

## 8. Kiểm thử lúc robot thực sự dò map

Bài test hiện tại **không mô phỏng tải cơ học, nhiễu động cơ hoặc quỹ đạo dò map**. PWM trong payload bằng 0 do motor locked. Trước khi áp dụng cho dò map tốc độ cao, cần làm thêm bài A/B trên cùng firmware navigation, cùng đường đi và cùng vận tốc:

- telemetry tắt, telemetry 32 B/100 Hz, telemetry 64 B/50–60 Hz;
- đo thêm kiểm soát vận tốc, thời gian xử lý motion/map, lỗi ADC/I2C, sụt áp VM, tỷ lệ gói lỗi và thời gian lấy mẫu cảm biến;
- ghi SRAM các mẫu ở 1 kHz, chỉ stream bản tin đã giảm mẫu trong main context; truyền toàn bộ trace sau khi robot dừng;
- giữ phương án dừng robot độc lập khi kênh SPP mất kết nối.

**Không dùng Bluetooth làm vòng phản hồi điều khiển 1 kHz.** PID, encoder, IMU, IR và an toàn motor phải chạy cục bộ trên STM32.

## 9. Kiểm thử phần mềm (không cần board)

```powershell
python tests/regression/classic_spp_rate_pipeline_test.py
python tests/regression/classic_spp_protocol_test.py
```

Các test này chỉ xác minh parser và thuật toán đánh giá; cần dữ liệu phần cứng mới có kết luận tốc độ tối đa trên robot.
