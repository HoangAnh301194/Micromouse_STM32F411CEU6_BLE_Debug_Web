# Pipeline debug Motor Driver

Điểm vào: `tools/motor_driver_debug_pipeline.py`.
Phạm vi: driver/bench/telemetry. Không sửa Motion/PID, encoder, IMU hoặc IR.
Dấu encoder được chủ dự án xác nhận; không kiểm tra lại. Mức logic 5 V/3,3 V không phải gate.

## Chạy theo thứ tự

1. Chạy regression host, không truy cập phần cứng:

```powershell
python tools/motor_driver_debug_pipeline.py --self-test
```

2. Chạy native C/watchdog, 9 gate offline, build năm profile motor và thu neutral COM6:

```powershell
python tools/motor_driver_debug_pipeline.py --run --build --port COM6
```

Không upload, không gửi START, không quay motor. `--build` chỉ biên dịch các profile bench đã có; khóa P0 giữ nguyên. Bỏ `--port` nếu chỉ cần offline; bỏ `--build` nếu chỉ cần kiểm tra mã và telemetry.

3. Chỉ khi VM đã ngắt vật lý và MCU/UART vẫn có nguồn, mới thu thêm trace 512 mẫu:

```powershell
python tools/motor_driver_debug_pipeline.py --run --port COM6 --capture-trace --confirm-vm-disconnected
```

`--confirm-vm-disconnected` là xác nhận thao tác vật lý của người dùng, không phải lệnh ngắt nguồn. Script từ chối thiếu xác nhận trước mọi build/UART.

4. Phân tích log chạy motor đã lưu, không chạy lại thử nghiệm:

```powershell
python tools/motor_driver_debug_pipeline.py --run --motor-log motor_logs/auto_20261013_101254/raw_serial.log --motor-log motor_logs/auto_20261013_105353/raw_serial.log
```

Hai log ví dụ là dữ liệu lịch sử, không xác nhận đáp ứng của firmware đang nạp. Có thể lặp `--motor-log` cho các phiên khác nhau; không truyền lại cùng một capture để tăng số lần thử. Hash các frame không gồm timestamp host được dùng để phát hiện sao chép log.

## Gate và dữ liệu

| Checklist | Pipeline thực hiện | Khi chưa đủ dữ liệu |
| --- | --- | --- |
| A1 | Commit/branch, hash nguồn host, Python, tham số phân tích; hash BIN/ELF sau build thành công | Hash image đang chạy vẫn null nếu chưa được chứng minh |
| A2–A3 | Compile/chạy `motor.c` + `motor_test.c` thật với register mocks; test metrics/UART; batch 9 gate | Lỗi kiểm thử là FAIL, không cho truy cập UART tiếp |
| A4 | `MOTOR PING` + `MOTOR DRIVER`, 12 snapshot mặc định | Không có COM6 thì NOT_MEASURED, không giả định PASS |
| A5 | Import trace hoặc thu khi VM ngắt; min/max/P95/P99 jitter từ từng timestamp MCU | Thiếu trace là NOT_MEASURED; không suy P99 từ ISR max |
| A6 | Ghi `USER_CONFIRMED_NOT_RETESTED` cho dấu encoder | Counts/vòng không chặn phân tích counts/s; chưa báo RPM/m/s |
| B1 | Startup heuristic từng bánh/mức duty từ log; không tự ghi deadband | Holding threshold chưa có trong protocol hiện tại |
| B2 | Mean/std/CV, tỷ lệ khởi động, thời gian quan sát đạt 5 counts; gộp các capture cùng cấu hình protocol | Dưới 10 lần/nhóm là LOW_REPEATS; điều kiện pin/tải/image chưa xác minh |
| B3 | Tốc độ từ count/timestamp thô; chỉ chọn cửa sổ applied cố định liên tục đủ dài, kiểm tra drift và độ phân giải | INSUFFICIENT_PLATEAU/NONSTATIONARY_WINDOW/LOW_COUNT_RESOLUTION, CV steady để null |
| B4 | So sánh rise slow/fast, counts và stop tail | Chưa có fall-rate sweep trong protocol; giữ NOT_MEASURED |
| B5 | Kiểm tra CCR neutral và counts ròng/tổng đường đi quan sát sau STOP; watchdog host đúng 1500 ms | Native watchdog PASS không thay thế powered heartbeat-loss trace |
| B6 | So sánh trái/phải khi cùng chạy; giữ telemetry IMU/IR có sẵn từ log tích hợp | Chênh lệch hai bánh không tự quy thành lỗi driver hoặc tự thêm PID |
| C1 | Có thể nạp bộ CSV dạng sóng vào 9-gate batch | Dạng sóng không bắt buộc cho phần chức năng; thiếu thì NOT_MEASURED |
| C2 | Không suy dòng/nhiệt/sụt áp từ encoder | NOT_MEASURED, không dùng để chặn phân tích phần mềm |
| C3–C4 | Báo chưa tự động thử energized brake/reverse/stall hoặc so tần số PWM | Không gửi lệnh hoặc thay đổi tần số để lấp dữ liệu thiếu |

Pipeline không gọi entrypoint chạy/nạp của `motor_auto_pipeline.py` hay `motor_full_qualification.py`; chỉ dùng parser offline. Các cảnh báo điện áp logic từ công cụ cũ không trở thành gate mới.
Protocol chạy motor v14/v16/v17/v18 hiện còn khóa P0; log tự nhận khóa nhưng có START sẽ bị từ chối vì mâu thuẫn. Muốn có dữ liệu chạy mới cần một phiên bản bench được review riêng, không phải thêm cờ vào pipeline này. Protocol nhập hiện hỗ trợ các phiên v4/v6/v7/v8 đã lưu; không giả vờ đã có log cho hold/fall/watchdog mới.

## Ngưỡng phân tích

```text
--min-repeats 10
--plateau-min-ms 300
--plateau-start-ms 0
--plateau-max-drift-pct 5
--max-sample-gap-ms 40
--min-counts-per-interval 10
```

Đây là cấu hình sàng lọc đề xuất, không phải tiêu chuẩn micromouse hoặc giới hạn an toàn. Xung 200 ms hiện tại không tự đạt cửa sổ plateau 300 ms. Không tự tăng duty/thời gian chạy để đạt ngưỡng phân tích.

- CV dùng độ lệch chuẩn mẫu và trị tuyệt đối trung bình; mean 0 hoặc dưới hai mẫu thì CV null, không đổi thành 0%.
- Candidate plateau chỉ là heuristic theo cửa sổ, không chứng minh hoàn toàn hết quá độ hay xác định nguyên nhân nhiễu.
- CSV tốc độ chỉ chứa cửa sổ applied cố định liên tục được chọn; CSV motor và raw log giữ toàn bộ mẫu gốc.
- Protocol motor cũ chỉ có timestamp ms; jitter microsecond chỉ lấy từ trace TIM5. Có kiểm tra timestamp wrap uint32, mẫu trùng/mất và elapsed bất nhất.
- Repeat groups tách phiên bản protocol, bánh, test, hai lệnh, rise và độ dài xung. Fall/deadband/load/battery không được log đầy đủ nên `setup_verified=false`; không dùng CV này để tự chốt hiệu chuẩn.

## Báo cáo và mã thoát

Thư mục mới: `motor_logs/driver_debug_<timestamp>/`. `--output` chỉ nhận thư mục mới/rỗng, không ghi đè bằng chứng có sẵn.

- `report.json`, `report.md`: trạng thái từng gate, provenance, cấu hình và mục còn thiếu.
- `offline.json`, các `*.log`: kết quả nguồn/model/native/build/self-test.
- `uart/`: UART raw và JSON locked-idle nếu có COM6.
- `functional_trace_raw.log`: trace nếu đã import/thu.
- `motor_input_*.log`: bản sao byte-for-byte các đầu vào được yêu cầu.
- `motor_samples.csv`, `speed_samples.csv`: dữ liệu chuẩn hóa nếu có log motor hợp lệ.

`motor_logs/.gitattributes` giữ nguyên byte cho các thư mục `driver_checklist_*` và `driver_debug_*`, kể cả CRLF và khoảng trắng trong log gốc. Không chuẩn hóa các file bằng chứng trước khi commit.

Mã thoát: `0` nếu pipeline không lỗi nhưng có thể còn REVIEW; `1` nếu test/build/capture/input FAIL; `2` khi thiếu bằng chứng với `--require-complete`, hoặc CLI không hợp lệ.
`MEASURED_SAVED_LOG` không đồng nghĩa PASS trên firmware hiện tại. Thiếu hold/fall/watchdog-live hoặc chưa xác minh image thì `complete_checklist=false`; không tạo nhãn “tối ưu hoàn toàn”.

CI chạy host regression trên Python 3.11/3.14 và tạo báo cáo offline. Git/push không được nhúng trong pipeline; thao tác đó thực hiện riêng sau khi kiểm thử và review diff.

Nguồn nghiên cứu, công thức và giới hạn kết luận: `docs/motor_driver/DEBUG_DATA_CHECKLIST_20261015.md`, mục Nguồn đối chiếu.
