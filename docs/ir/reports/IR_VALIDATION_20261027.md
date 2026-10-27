# Báo cáo kiểm thử và nghiệm thu IR 6 kênh — 2026-10-27

**Trạng thái tổng thể:** `ACQUISITION_ACCEPTED`
**Mức cao nhất được nghiệm thu:** `ACQUISITION_ACCEPTED`
**Quyết định bộ lọc:** Chưa chọn bộ lọc cho production hoặc Wall PID; các kết quả lọc dưới đây chỉ là đánh giá tĩnh thăm dò.

## 1. Provenance, phần mềm và an toàn

| Hạng mục | Kết quả |
|---|---|
| Repository / nhánh | `HoangAnh301194/Micromouse_STM32F411CEU6_BLE_Debug_Web` / `giomuadongbac` |
| HEAD nền khi bắt đầu vòng đo | `d90d5c10def6ff2a7c4f98174f738e233741cef7`; working tree có các thay đổi IR diagnostic/tooling được kiểm thử và lưu trong evidence bundle |
| Firmware | PlatformIO `ir_filter_diag_96m`; motor outputs khóa trong firmware |
| SHA-256 firmware ELF | `9C2316BFA7B4456182570416DD9AF33A7CDCDB932B19D53709C56A5B50B55228` |
| Build / nạp | Build và nạp ST-Link thành công; không thay đổi production `IR_GetLatest()`, Motion, PID hoặc motor driver |
| Host / tool | Windows; Python 3.14.3; PlatformIO Core 6.2.0; pyserial 3.5; matplotlib 3.10.8; Node v24.14.0 |
| UART | USART1 PA9/PA10, 115200 baud; USB-SERIAL CH340 COM6 và ST-Link được Windows nhận diện |
| An toàn vật lý | Người vận hành xác nhận VM motor đã ngắt vật lý; không gửi lệnh motor; mọi IRH trong capture hợp lệ báo `locked=1` |
| Firmware HELLO | `IR_FILTER,HELLO,ir_filter_v1_motor_locked`, phát lúc khởi động và lặp 1 Hz để host có thể mở COM sau khi firmware chạy |

### Kiểm tra phần mềm

Đã chạy lại tập kiểm tra tương đương workflow `firmware-regression.yml` trên working tree hiện tại:

- Native/host: encoder, IR sensor, motor, system sweep, flash record/map, BLE RX, BLE web, IR filter/pipeline, motion primitives, OLED/MPU parser, turn-PID parser và kiểm tra bố cục handover — **PASS**.
- PlatformIO: `genericSTM32F411CE`, `ir_filter_diag_96m`, `ir_oled_test_96m`, `oled_mpu_diag_96m`, `flash_map_diag_96m`, `ble_rx_diag_96m`, `system_test_96m`, `turn_pid_diag_96m` — **PASS**.
- GitHub Actions gần nhất quan sát được là run 149 trên commit cũ `4c320200d1a036073993fc8f8e76b8290b756753`; không coi đó là CI của working tree này. Kết quả nghiệm thu phần mềm ở đây là các lệnh local nêu trên.

Các thay đổi trong phạm vi task giới hạn ở `src/ir_filter_diag.c`, collector/test IR, protocol và evidence. Diagnostic gọi `Motor_LockOutputs()` rồi kiểm tra `Motor_OutputsLocked()` trước khi thu dữ liệu. Không nạp firmware khi VM còn cắm; không chạy motor trong các phép đo.

## 2. Capture và evidence gốc

Collector `valid_capture` kiểm tra tính toàn vẹn vận chuyển/telemetry (HELLO, coverage, khóa motor, counter, scan rate), không tự chứng minh fixture đứng yên. Tất cả file raw được giữ nguyên; lượt có dịch chuyển được ghi lại nhưng loại khỏi phân tích tĩnh.

| Capture | Tình huống / điều kiện | Kết quả | Evidence |
|---|---|---|---|
| `193255`, `193540` | No-wall, thiếu HELLO | Invalid capture; không tính vào acceptance metrics | [193255](../../../results/ir_filter_20261027_193255/summary.json), [193540](../../../results/ir_filter_20261027_193540/summary.json) |
| `193858` | No-wall, room light, 30 s | Hợp lệ; IRF/IRB/IRH = 1198/149/29; scan 1210.50 Hz | [raw UART](../../../results/ir_filter_20261027_193858/raw_uart.log), [summary](../../../results/ir_filter_20261027_193858/summary.json), [plots](../../../results/ir_filter_20261027_193858/) |
| `194913` | Hai vách bên L0/R0, trắng mờ, 40 mm, uncertainty ghi 5 mm | Thiếu HELLO; invalid capture, không tính metrics | [194913](../../../results/ir_filter_20261027_194913/summary.json) |
| `195440` | Hai vách bên L0/R0, trắng mờ, room light, 40 mm ±5 mm, 20 s | Hợp lệ; 800/100/20; một lượt, độc lập chưa xác nhận | [raw UART](../../../results/ir_filter_20261027_195440/raw_uart.log), [summary](../../../results/ir_filter_20261027_195440/summary.json), [plots](../../../results/ir_filter_20261027_195440/) |
| `200844` | Cùng fixture 40 mm, 30 s | Hợp lệ; 1199/150/30; lặp cùng gá, không độc lập | [raw UART](../../../results/ir_filter_20261027_200844/raw_uart.log), [summary](../../../results/ir_filter_20261027_200844/summary.json), [plots](../../../results/ir_filter_20261027_200844/) |
| `201307` | 50 mm, hai vách bên L0/R0, trắng mờ, room light, 20 s | Hợp lệ; người vận hành ghi independent reseat, uncertainty ±5 mm | [raw UART](../../../results/ir_filter_20261027_201307/raw_uart.log), [summary](../../../results/ir_filter_20261027_201307/summary.json), [plots](../../../results/ir_filter_20261027_201307/) |
| `201812` | 50 mm, 20 s | Transport hợp lệ nhưng người vận hành xác nhận đã dịch chuyển robot/vách trong capture; **loại khỏi thống kê tĩnh** | [raw UART](../../../results/ir_filter_20261027_201812/raw_uart.log), [summary](../../../results/ir_filter_20261027_201812/summary.json), [L45](../../../results/ir_filter_20261027_201812/filter_L45.png), [R45](../../../results/ir_filter_20261027_201812/filter_R45.png) |
| `202026` | 50 mm, 20 s, không dịch chuyển trong capture | Transport/tĩnh hợp lệ; independence chưa xác nhận. Ghi ±5 mm trong collector nhưng người vận hành chưa xác nhận lúc đó; không dùng làm trial độc lập hoặc hiệu chuẩn | [raw UART](../../../results/ir_filter_20261027_202026/raw_uart.log), [summary](../../../results/ir_filter_20261027_202026/summary.json), [plots](../../../results/ir_filter_20261027_202026/) |
| `202455` | 50 mm, hai vách bên L0/R0, trắng mờ, room light, 20 s | Hợp lệ; independent reseat, uncertainty ±5 mm được người vận hành xác nhận; góc vách chưa đo | [raw UART](../../../results/ir_filter_20261027_202455/raw_uart.log), [summary](../../../results/ir_filter_20261027_202455/summary.json), [plots](../../../results/ir_filter_20261027_202455/) |
| `203335` | 50 mm, hai vách bên L0/R0, trắng mờ, room light, 20 s | Hợp lệ; lượt independent thứ ba được xác nhận, uncertainty ±5 mm; góc vách chưa đo | [raw UART](../../../results/ir_filter_20261027_203335/raw_uart.log), [summary](../../../results/ir_filter_20261027_203335/summary.json), [plots](../../../results/ir_filter_20261027_203335/) |

Các CSV tổng hợp có liên kết trực tiếp tới capture gốc: [manifest.csv](../../../results/ir_qualification_20261027/manifest.csv) và [metrics_by_channel.csv](../../../results/ir_qualification_20261027/metrics_by_channel.csv). Metrics có 240 dòng (8 capture transport-valid × 6 kênh × 5 ứng viên lọc); dòng của `201812` được đánh dấu `EXCLUDED_MOVEMENT_DURING_CAPTURE`, không dùng để xếp hạng bộ lọc tĩnh.

P0 no-wall đã chạy; P0 vách trắng 20 mm chưa chạy vì setup không đặt được ở 20 mm. Dữ liệu 40 mm không được thay thế cho 20 mm. P1 mới khảo sát cấu hình hai vách bên tại 40/50 mm; chưa có ma trận khoảng cách và hình học từng kênh.

## 3. Tính toàn vẹn thu nhận

Trong tám capture có transport hợp lệ:

- Mỗi IRF chứa đủ sáu kênh. Capture 20 s có IRF 799–800, IRB 100, IRH 20; coverage tối thiểu 99.85% / 99.88% / 99.88%. Capture 30 s hợp lệ có coverage IRF/IRB/IRH tối thiểu 99.82% / 99.32% / 96.65%.
- Scan rate đo từ IRH là **1209.69–1210.50 Hz**, không giả định bằng đúng 1 kHz.
- Malformed frame = 0; UART drops, DMA errors/timeouts, ADC overruns và scan failures đều có delta = 0; mọi IRH báo motor locked.
- Không có OFF/ON clipping ở 4095 trong các capture hợp lệ đã kiểm tra; các giá trị ON-OFF cũng không bão hòa.
- Các lượt không có HELLO bị collector đánh dấu invalid dù số frame/counter nhìn qua có vẻ đầy đủ; chúng được giữ lại để chứng minh lỗi và không đưa vào acceptance metrics.

Median OFF/ON từ IRB và median ambient-subtracted `d` từ IRF cho từng kênh ở no-wall và trial độc lập 50 mm thứ ba:

| Kênh | No-wall OFF/ON | No-wall d | 50 mm OFF/ON | 50 mm d |
|---|---:|---:|---:|---:|
| L90 | 46 / 462 | 416 | 27 / 36 | 9 |
| L45 | 17 / 28 | 11 | 59 / 213 | 154 |
| L0 | 33 / 41 | 9 | 354 / 920 | 566 |
| R0 | 35 / 57 | 21 | 280 / 1207 | 928 |
| R45 | 26 / 116 | 91 | 37 / 204 | 167 |
| R90 | 40 / 366 | 327 | 16 / 29 | 13 |

OFF/ON và `d` lấy từ các luồng IRB/IRF riêng (40 Hz và 5 Hz); chúng không phải mẫu đồng thời. Dữ liệu gốc nằm trong [IRB ON/OFF CSV](../../../results/ir_filter_20261027_203335/ir_on_off.csv) và [IRF trace CSV](../../../results/ir_filter_20261027_203335/ir_filter_trace.csv).

## 4. Định lượng bộ lọc tĩnh

Thống kê dưới đây dùng sáu capture tĩnh không có dịch chuyển được quan sát (hai lượt 40 mm và bốn capture 50 mm; trong đó independence của `202026` chưa xác nhận). Ba capture 50 mm được xác nhận reseat độc lập. Đây là dữ liệu thăm dò trên một vật liệu/ánh sáng và một bố trí hai vách; không phải kết quả cho toàn dải sử dụng. SNR dùng công thức protocol, ghép mỗi capture tĩnh với capture no-wall `193858`; do baseline chỉ có một lượt và khác thời điểm, SNR chỉ có giá trị tham khảo.

| Kênh | SNR đo được, khoảng | Median3 giảm STD | EMA α=0.10 giảm STD | EMA α=0.25 giảm STD | Median3 + EMA α=0.25 giảm STD | Bias median của hybrid |
|---|---:|---:|---:|---:|---:|---:|
| L90 | 299.5–303.9 | 31.7% | 48.4% | 43.5% | 44.4% | +0.028 |
| L45 | 98.1–751.6 | 36.3% | 71.5% | 53.3% | 55.9% | −0.032 |
| L0 | 228.1–663.6 | 36.8% | 55.4% | 46.5% | 48.8% | −0.005 |
| R0 | 297.6–823.9 | 36.2% | 56.9% | 48.3% | 52.2% | +0.043 |
| R45 | 43.0–563.3 | 30.0% | 60.1% | 48.8% | 48.7% | +0.030 |
| R90 | 162.6–176.7 | 37.3% | 53.5% | 43.3% | 47.0% | −0.041 |

Bias tính theo ADC counts. Ở các capture tĩnh đã chọn, các filter giữ gần nguyên mean wall/no-wall trong khi giảm STD; EMA α=0.10 thường giảm STD nhiều nhất. Kết quả **không** đo được trễ cạnh. Không chọn production filter: EMA α=0.10 có đánh đổi latency chưa đo, còn hybrid chưa chứng minh ưu thế cho đáp ứng tường động. Phân bố từng lượt gồm mean, STD, MAD, P95/P99, spike count/rate, bias và saturation ở CSV.

Ba lượt độc lập 50 mm (`201307`, `202455`, `203335`) cho L45 khoảng 120–154 counts và R45 khoảng 167–211 counts. Độ lệch này có thể do sai số gá/góc/vị trí; góc vách chưa được đo. L90/R90 cũng đổi mạnh giữa no-wall và hai vách bên. Không thể phân biệt nhiễu chéo điện/quang với hình học nhìn thấy vách nếu chưa thử từng vách riêng.

## 5. Đáp ứng động, robustness và LUT

- **P3:** chưa chạy bài step có nhãn. Capture IRF là 40 Hz (chu kỳ 25 ms); không dùng để chứng minh t10/t50/t90 hoặc độ trễ 5–10 ms. Cần MCU timestamp/event capture tốc độ cao và tối thiểu 10 cạnh lên/xuống mỗi vùng.
- **P4:** chưa chạy ma trận ánh sáng, vật liệu, góc và vị trí; chưa có confusion matrix/định lượng false positive/negative. Các capture hiện tại chỉ dùng vách trắng mờ, ánh sáng phòng, hai vách bên.
- **P5:** chưa fit LUT và không có holdout độc lập; không suy khoảng cách từ ADC và không ngoại suy.

## 6. Gate nghiệm thu G0–G6

| Gate | Trạng thái | Ngưỡng | Quan sát | Evidence / nguyên nhân và thử lại |
|---|---|---|---|---|
| G0 — SAFETY | **PASS** | VM tháo vật lý, mọi IRH `locked=1`, không lệnh motor | Người vận hành xác nhận VM ngắt; diagnostic kiểm tra output lock; mọi capture hợp lệ có `locked=1`; không có lệnh motor | Các [summary capture](../../../results/ir_filter_20261027_203335/summary.json). Giữ VM ngắt cho mọi lượt sensor tiếp theo. |
| G1 — DATA INTEGRITY | **PASS** | Native/build PASS; HELLO; ≥95% IRF/IRB/IRH; đủ 6 kênh; scan rate đo được; malformed/drop/DMA/timeout/overrun/failure delta = 0 | Local CI-equivalent PASS; các capture hợp lệ có coverage ≥95%, 0 malformed/error/drop và scan 1209.69–1210.50 Hz. Capture thiếu HELLO bị từ chối đúng | [summary no-wall](../../../results/ir_filter_20261027_193858/summary.json), [summary trial 5](../../../results/ir_filter_20261027_203335/summary.json), [metrics CSV](../../../results/ir_qualification_20261027/metrics_by_channel.csv). |
| G2 — COVERAGE | **INCONCLUSIVE** | 3 lượt độc lập 20 s/cự ly/kênh cần hiệu chuẩn; đủ cự ly/góc/bề mặt/ánh sáng/uncertainty | Cấu hình hai vách bên ở 50 mm có 3 lượt independent được xác nhận; 202026 chưa xác nhận độc lập. 40 mm lặp cùng gá, chưa đủ 3 lượt độc lập; P0 20 mm và các cự ly khác chưa đo; góc vách mới chưa đo | [manifest](../../../results/ir_qualification_20261027/manifest.csv). Tiếp tục P0 20 mm nếu gá được, hoàn thành coverage còn lại và ghi góc/kênh. |
| G3 — STATIC FILTER | **INCONCLUSIVE** | SNR ≥5; filter được chọn giảm STD ≥25% ở phần lớn vùng đo mà không mất biến thiên tường | Trên dữ liệu tĩnh hiện có, SNR thăm dò >5 và hybrid giảm STD 44.4–55.9% theo median từng kênh; chưa đủ vùng/khoảng cách/hình học và chưa đánh giá đáp ứng tường động | [metrics CSV](../../../results/ir_qualification_20261027/metrics_by_channel.csv). Mở rộng coverage; không tuyên bố bộ lọc dùng cho Wall PID chỉ từ dữ liệu tĩnh. |
| G4 — DYNAMIC | **INCONCLUSIVE** | ≥10 cạnh lên/xuống mỗi vùng; timestamp đủ độ phân giải và `v_target × latency ≤ 5 mm` | Không có event test. IRF 40 Hz không phân giải được mục tiêu mili-giây | [protocol](../IR_AGENT_QUALIFICATION_PROTOCOL.md). Bổ sung phép đo timestamp tốc độ cao motor-locked; không suy latency từ log UART 40 Hz. |
| G5 — ROBUSTNESS | **INCONCLUSIVE** | Định lượng false positive/negative và cross-talk theo ánh sáng/vật liệu/góc/vị trí; không có nhầm tường nguy hiểm | Chưa có ma trận robust hoặc một-vách-một-lần; độ nhạy L45/R45 thay đổi đáng kể qua reseat | [manifest](../../../results/ir_qualification_20261027/manifest.csv). Dùng gá đo góc/khoảng cách chính xác hơn và tách từng vách. |
| G6 — DISTANCE LUT | **INCONCLUSIVE** | LUT đơn điệu từng kênh; holdout độc lập MAE ≤2 mm, P95 abs error ≤5 mm; không ngoại suy | Chưa có dải đơn điệu được xác lập, LUT hoặc holdout | [manifest](../../../results/ir_qualification_20261027/manifest.csv). Chỉ thực hiện sau P0–P4 và đánh giá holdout độc lập. |

**Kết luận:** `ACQUISITION_ACCEPTED` — chỉ G0/G1 PASS. Không đạt `STATIC_FILTER_ACCEPTED`, `FILTER_DYNAMIC_ACCEPTED` hay `IR_CALIBRATION_ACCEPTED`; chưa đủ bằng chứng để tích hợp bộ lọc vào production hoặc khẳng định Wall PID tốc độ cao sẵn sàng.

## 7. Việc còn lại

1. Hoàn thành P0 vách trắng 20 mm nếu gá cho phép; không dùng số liệu 40 mm thay thế.
2. Với P1, hoàn tất 3 lần reseat độc lập ở từng cự ly hợp lệ 10/15/20/25/30/40/50/60/80 mm, theo đúng hướng từng sensor; 50 mm đã đủ 3 trial độc lập, 40 mm chưa đủ.
3. Đo chính xác góc và uncertainty cho L0/R0, L45/R45, L90/R90; thử từng vách riêng để phân biệt hình học và cross-talk.
4. Chạy P3 có timestamp MCU độ phân giải cao, P4 ánh sáng/vật liệu/vị trí, rồi mới đánh giá lựa chọn filter và LUT holdout.
