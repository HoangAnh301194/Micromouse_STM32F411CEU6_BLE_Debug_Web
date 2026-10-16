# Checklist Motor Driver tốc độ cao — 16/10/2026

## Đã tối ưu, đã kiểm tra

- [x] Bỏ ghi lại hai CCR và GPIO BSRR khi PWM đã ổn định, không có thay đổi deadband.
- [x] Vẫn chạy bộ đếm nghỉ đảo chiều mỗi tick; giữ preload guard, ramp 20/30 permille/ms và STOP tức thời.
- [x] Buộc cập nhật lại đầu ra sau thay đổi deadband và sau tick preload đầu tiên, kể cả lệnh nhỏ hơn slew rate.
- [x] Native C: 6.000 duty có dấu, 10.000 thay đổi lệnh, STOP/brake/reverse/lock; thêm tình huống cache/deadband và high-speed encoder watchdog.
- [x] Thử thật từng bánh 20/40/60/80/100% PWM; hai bánh 40/60/80/100%, không đảo chiều hoặc phanh cưỡng bức đang quay.
- [x] Xác nhận hai bánh 100% hai lần sau bản sửa cuối, xung 600 ms, nghỉ tối thiểu 900 ms và encoder yên 300 ms.
- [x] IMU Policy B, IR ADC/DMA/TIM10, encoder TIM3/TIM4, UART DMA và USART2 TX cùng hoạt động trong bench.
- [x] Không có late/gap TIM11, drop UART hoặc lỗi RX trong các trial hoàn tất; IMU error/timeout bằng 0.
- [x] Build default, MPU RC2 và motor bench tương thích; không sửa thuật toán Motion/PID, encoder, IMU, IR hoặc BLE.
- [x] Sửa snapshot telemetry: state, applied duty, CCR, GPIO và thời gian được chụp trong cùng critical section.
- [x] STOP được host xác nhận; đầu ra khóa, không tự quay sau boot/reset.

## Kết quả trước–sau

So đúng cùng điều kiện hai bánh 100% PWM, 500 lần gọi đo/xung, hai xung mỗi bản:

| Đại lượng | Trước, driver revision 2 | Sau, driver revision 3 |
| --- | --- | --- |
| Motor_Update1ms trung bình | 259,018 cycles | 141,014 cycles |
| Tốc độ encoder trái, plateau | 8.894,3 count/s | 8.856,1 count/s |
| Tốc độ encoder phải, plateau | 9.380,5 count/s | 9.385,4 count/s |
| CV tốc độ trong plateau, lớn nhất | Trái 0,164%; phải 0,840% | Trái 0,179%; phải 0,872% |

Giảm khoảng **45,6% CPU cho driver ở trạng thái này**, tương đương khoảng 1,23 µs/lần ở 96 MHz danh định.
Ở tần suất gọi 1 kHz, tổng tải CPU giảm khoảng **0,123 điểm phần trăm**, không phải 45,6% tổng CPU.
Không phải tăng 45,6% tốc độ động cơ hoặc hiệu suất điện. Khác biệt tốc độ nhỏ không được quy cho tối ưu phần mềm.
CV tính từ delta encoder trên các khoảng telemetry 40 ms trong nửa cuối xung; không phải số đo tốc độ cơ liên tục.

## Chạy lại 08:05 ngày 16/10/2026, giờ Việt Nam

- 6 xung hoàn tất; hai xung hai bánh 100% PWM đạt, mỗi xung 600 ms.
- Tốc độ plateau trung bình ở 100%: trái **8.365,7 count/s**, phải **8.640,7 count/s**.
- CV plateau lớn nhất: trái **0,199%**, phải **1,488%**; cả hai bánh đạt 2/2 lần xác nhận.
- Driver trung bình **141,010 cycles/lần**; `late=0`, `gaps=0`, `tx=0`, `rx=0`.
- IMU `error_reads=0`, `timeout_reads=0`, `age_max=1.968 µs`; STOP được xác nhận.
- Tốc độ thấp hơn lượt trước; chưa đo VM thực tế nên không quy nguyên nhân cho pin hoặc firmware.

## Quy trình tự động rút gọn

```powershell
python tools/motor_startup_pipeline.py --run --upload --high-speed --port COM6 --vm-volts 8.4 --confirm-wheels-elevated
```

Một lệnh: native C + parser regression, build/upload một lần, kiểm tra profile/watchdog, chờ cảm biến sẵn sàng,
quét 15 xung, xác nhận plateau hai bánh 100%, STOP, lưu log/JSON/CSV.
Không chạy lại các batch audit dài hoặc upload firmware default sau khi bench kết thúc.
`--confirm-only` rút còn 6 xung để xác nhận lại sau khi sweep đã hoàn tất; vẫn warmup, kiểm tra cảm biến và giữ mọi chốt an toàn.
Mất heartbeat 400 ms, thiếu 5 count sau 100 ms hoặc không tiến triển encoder 80 ms khi đang chạy sẽ khóa đầu ra.
Watchdog phần cứng danh định 500 ms; không thay thế bảo vệ quá dòng/nhiệt.

## Còn giới hạn khi tích hợp

- [ ] Chạy có tải mặt sàn, vận tốc m/s hoặc RPM: chưa thực hiện, chưa có counts/vòng bánh được xác minh trong phép đo này.
- [ ] Dòng/nhiệt/áp sụt/hiệu suất điện và chạy 100% liên tục: không đo được; không chứng nhận từ encoder.
- [ ] IMU freshness theo yêu cầu Motion/PID: quan sát `age_max` khoảng **2 ms**; chưa đủ để cam kết mọi mẫu luôn dưới 1 hoặc 1,5 ms.
- [ ] IR production: `IR_Init` chưa cấu hình ADC common prescaler. Bench đặt riêng /4, tức 24 MHz danh định; cấu hình ADC của firmware tích hợp phải được rà riêng, không coi bench đã sửa module IR.
- [ ] BLE radio/ứng dụng từ xa: chỉ tạo tải USART2 TX, không kiểm chứng đường truyền radio end-to-end.
- [ ] PWM/coast transient, dòng phanh hoặc đảo chiều ở tốc độ cao: chưa đo dạng sóng; các khóa P0 của bench cũ giữ nguyên.
- [ ] TIM2 và PB12–PB15 phải thuộc riêng driver; cache không có trách nhiệm phục hồi khi module khác tự sửa thanh ghi này.

## Dữ liệu

- Baseline: `motor_logs/startup_speed_20261016_013358_380418/` — 15 xung PASS.
- Trung gian: `motor_logs/startup_speed_20261016_013716_629478/` — 13 xung hoàn tất; dừng ở BEGIN xung 14 vì snapshot applied/CCR lệch tick. Giữ nguyên log FAIL, không bỏ qua kiểm tra.
- Bản cuối: `motor_logs/startup_speed_20261016_014006_799985/` — 6 xung PASS, hai lần hai bánh 100%, STOP xác nhận.
- Chạy lại: `motor_logs/startup_speed_20261016_080553_270935/` — 6 xung PASS, hai lần hai bánh 100%, STOP xác nhận; mã nguồn và raw UART khớp SHA-256 trong báo cáo.
- So sánh máy đọc được: `motor_logs/startup_speed_20261016_014006_799985/comparison.json`.
