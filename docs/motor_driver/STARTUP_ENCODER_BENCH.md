# Đo PWM khởi động qua encoder

## Phạm vi

- Chỉ bench Motor Driver; không thay thuật toán Motion/PID, IMU, IR hoặc BLE.
- N20 12 V, TB6612FNG VM từ pin 2S 8,4 V do người vận hành báo; bánh nâng khỏi mặt đất.
- Dấu encoder đã được chủ dự án xác nhận. Không khảo sát lại logic 5 V/3,3 V.
- Không có cảm biến dòng, nhiệt, sụt áp; encoder không thay thế các phép đo này.
- Đây là đo khởi động không tải, không phải chứng nhận chạy tốc độ cao hoặc PWM 100%.

## Giới hạn firmware

- Profile `motor_startup_96m_trim25`, handshake phiên bản 2; PWM 30 kHz, TIM11 1 kHz.
- Không quay lúc boot/reset: cần `STARTUP ARM`, tiếp theo `STARTUP PULSE` hợp lệ.
- Từng bánh, chiều tiến; tăng 0,5% đến tối đa 20%, không đảo chiều, không short-brake.
- Mỗi xung 250 ms; nghỉ tối thiểu 750 ms, encoder yên tối thiểu 300 ms trước xung tiếp.
- Deadband bench bằng 0; slew tăng 20, giảm 30 permille/ms. Không tự lưu vào driver.
- Tối đa 128 xung/phiên, thời hạn 180 giây, mất heartbeat 400 ms thì khóa đầu ra.
- IWDG prescaler 128 (`PR=5`), reload 124; chu kỳ danh định 500 ms với LSI 32 kHz, không phải thời gian đo chính xác.
- Sai bánh >2 count, đếm ngược <-2, gap TIM11, lỗi UART, STOP: khóa vĩnh viễn đến reset.

## Quy trình

1. Người vận hành xác nhận bánh nâng an toàn, VM không quá 8,4 V; sẵn sàng ngắt pin nếu bất thường.
2. Chạy native C, self-test Python, build, upload qua ST-Link.
3. Xác thực profile và đầu ra neutral qua COM6, 115200 baud.
4. Yêu cầu MCU ngừng refresh watchdog khi disarmed; xác nhận reset mới bằng cờ IWDGRSTF.
5. ARM, quét từng bánh từ 0,5%, bước 0,5%; heartbeat mỗi 80 ms.
6. Phân biệt có dịch chuyển với quay duy trì: ít nhất 5 count, 3 khoảng tăng count, 2 count trong khoảng cuối xung.
7. Xác nhận ứng viên bằng 10 lần khởi động độc lập; nếu không đạt đủ, tiếp tục quét trong giới hạn.
8. STOP, xác nhận neutral và fault latch; lưu UART thô, lệnh, JSON, CSV, Markdown, hash nguồn/firmware.

```powershell
python tools/motor_startup_pipeline.py --self-test
python tools/motor_startup_pipeline.py --run --upload --port COM6 --vm-volts 8.4 --confirm-wheels-elevated
```

`motor_logs/startup_<timestamp>/` giữ cả lần FAIL. Không ghi đè dữ liệu cũ.
Không có tín hiệu đến 20% được ghi `NOT_FOUND_WITHIN_20_PERCENT`; không tự tăng trần để ép quay.
UART 20 ms chỉ cho độ phân giải thời gian quan sát khoảng 20 ms, không phải thời gian bắt đầu quay chính xác 1 ms.
10/10 lần thành công là kết quả mẫu hữu hạn, không đảm bảo khởi động tuyệt đối trong điều kiện khác.

## Lỗi boot đã xác định ngày 16/10/2026

ST-Link dừng startup bench tại `PC=0x0800206c`, trong vòng chờ `IWDG->SR`;
firmware chưa thực hiện khóa bật `0xCCCC`, nên UART chưa xuất profile và motor chưa nhận xung.
Không phải lỗi COM6 hoặc kết luận lỗi clock: bench cũ cùng clock trả lời PING bình thường.

Đã sửa theo thứ tự trong mã nguồn HAL chính thức của STMicroelectronics,
repository `stm32f4xx-hal-driver`, file `Src/stm32f4xx_hal_iwdg.c`, hàm `HAL_IWDG_Init`:
bật IWDG, mở ghi, cấu hình prescaler/reload, chờ cờ cập nhật có timeout, reload counter.
Không thêm phụ thuộc HAL; bench vẫn dùng CMSIS trực tiếp.
Đổi `PR=4` sang `PR=5` để khớp mục tiêu danh định 500 ms; `PR=4` chỉ khoảng 250 ms tại LSI 32 kHz.
Xóa cờ reset cũ ngay trước probe, tránh coi cờ sticky từ lần reset trước là bằng chứng mới.

Regression Python kiểm tra thứ tự mã nguồn và timeout; không mô phỏng phần cứng IWDG.
Probe UART trên MCU là bằng chứng reset thực tế; không chứng minh bảo vệ dòng/nhiệt.

## Kết quả thực nghiệm 16/10/2026

Thư mục: `motor_logs/startup_20261016_010521_884597/`.
Trạng thái `MEASURED_STARTUP_CANDIDATES`, 66 xung hoàn tất, STOP được xác nhận.

| Bánh | Dịch chuyển đầu tiên | Ứng viên khởi động lặp lại | Xác nhận | Count trung bình/250 ms | CV count |
| --- | --- | --- | --- | --- | --- |
| Trái | 6,5% | 8,5% | 10/10 | 124,8 | 4,53% |
| Phải | 7,0% | 9,5% | 10/10 | 134,9 | 3,25% |

- Bánh phải từng quay duy trì ở 7,5% một lần khi quét, nhưng không lần nào trong 10 lần xác nhận đạt tiêu chí. Không chọn 7,5% làm ngưỡng tin cậy.
- Count xác nhận tại ngưỡng: trái 119–134, phải 129–139 trong 250 ms.
- Thời điểm quan sát đạt 5 count: trung bình trái 47,0 ms, phải 48,9 ms; có lượng tử hóa telemetry.
- Toàn bộ frame hợp lệ: `late=0`, `gaps=0`, `tx=0`, `rx=0`.
- Probe IWDG gây reset thật khi disarmed; cờ sau reset `603979778`, có IWDGRSTF.
- Snapshot STOP: `state=4`, `fault=5` (HOST_STOP), `al=ar=0`, `cl=cr=3200`, `odr=0`.
- Đây là ngưỡng ứng viên trong lần đo hiện tại; không tự thay cấu hình driver sản phẩm.

## Giới hạn kết luận

Không tự đặt deadband bằng PWM khởi động; PWM giữ quay có thể khác PWM khởi động.
Chưa đo tải mặt sàn, tốc độ bánh theo m/s hoặc RPM khi chưa biết counts/vòng,
khả năng chịu dòng/nhiệt, tuổi thọ hay hoạt động PWM 100%.
Các khóa P0 ở bench cũ giữ nguyên; startup bench là ứng dụng độc lập có giới hạn riêng.
