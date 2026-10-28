# README — các tool kiểm thử Micromouse

Chạy tại **thư mục gốc repo**. Tool mới chỉ phân tích offline, **không tự nạp firmware/khởi động motor**. Thay `COM6` theo cổng thực tế.

| Mục tiêu | Lệnh |
|---|---|
| Kiểm tra phần mềm trước test | `python tools/maze_readiness_suite.py preflight` |
| Kèm build các firmware liên quan (không nạp) | `python tools/maze_readiness_suite.py preflight --build` |
| Firmware Wall PID 4 cell, BLE PA2/PA3 | `python -m platformio run -e wall_pid_4cell_96m -t upload` |
| Phân tích một lượt BLE đã Export | `python tools/wall_pid_ble_report.py --input run1.txt` |
| **Tổng hợp nhiều lượt Wall PID (mới)** | `python tools/maze_readiness_suite.py motion --logs run1.txt run2.txt` |
| Tạo phiên IR (VM motor OFF, UART1) | `python tools/ir_full_qualification.py init --session ir_wall_01` |
| Tổng kết IR đã thu | `python tools/ir_full_qualification.py report --session ir_wall_01` |
| **Đánh giá phát hiện tường 6 kênh (mới)** | `python tools/maze_readiness_suite.py wall-events --train train.csv --test test.csv` |
| **Một lệnh chạy toàn bộ offline (mới)** | `python tools/maze_readiness_suite.py all --logs run1.txt run2.txt --train train.csv --test test.csv` |

Các tool khác đã có: `python tools/turn_pid_pipeline.py --analyze results/turn_pid_YYYYmmdd_HHMMSS` (quay 90/180 offline), `python tools/ble_full_pipeline.py --scan` (quét BLE), `python tools/flash_map_pipeline.py --help` (Flash map). Test IR tĩnh/động và lệnh `capture` đầy đủ: [IR runbook](../docs/ir/IR_FULL_QUALIFICATION_RUNBOOK.md). Lệnh cũ cần UART1 PA9/PA10; **Wall PID mode hiện tại debug qua BLE PA2/PA3**. `preflight` cần GCC cho native tests; `--build` cần PlatformIO.

## Chuẩn bị dữ liệu cho bộ mới

- BLE: Export terminal sau mỗi lượt từ [Web BLE console](../web/ble_debug_console.html) thành `run1.txt`, `run2.txt`. Bộ tổng hợp tạo `motion_runs.csv`, `motion_summary.json`.
- Phát hiện tường: hai file **khác nhau** `train.csv` và `test.csv`, cùng 13 cột: `ms,L90,L45,L0,R0,R45,R90,wall_L90,wall_L45,wall_L0,wall_R0,wall_R45,wall_R90`.
- Giá trị ADC 0–4095 sau EMA 0,10; `wall_*` là nhãn 0/1 gán bằng **quan sát/đo độc lập**. Mỗi kênh phải có cả nhãn 0 và 1 trong tập train. Không sử dụng chính ADC để gán nhãn rồi tự đánh giá.
- Output mặc định nằm tại `results/maze_readiness/`. Báo cáo gồm FP, FN, precision, recall, F1, độ trễ phát hiện p95 và số lượt Wall PID hoàn tất. **Tất cả vẫn là diagnostic, không tự cấp nghiệm thu chạy thực tế.**

## Cần bổ sung trước Maze Solve

| Ưu tiên | Tool chưa có | Lý do |
|---|---|---|
| **P0** | Logger đồng bộ 6 IR + encoder/yaw + vị trí cell, gửi BLE sau khi dừng | Bộ phát hiện tường hiện cần dữ liệu và nhãn đo thực tế |
| **P0** | Tool gán nhãn mép tường, mất tường, tường trước từ encoder + mẫu chuẩn | Đo false positive/false negative và phát hiện sớm/muộn |
| **P0** | Test **phanh/dừng theo quãng đường** và trôi sau ngắt motor | Lượt đầu từng trôi thêm 80,2 mm |
| **P1** | Tool hợp nhất tín hiệu trái/phải/trước, trạng thái `UNKNOWN` | Không suy luận “không có tường” khi chưa quan sát |
| **P1** | Map consistency + flood-fill replay | Kiểm tra tường hai ô liền kề và cập nhật bản đồ |

**Lưu ý:** ngưỡng học được trong `wall-events` chỉ phục vụ **detect tường offline**, không áp dụng làm ngưỡng dừng ADC của Wall PID. L90/R90 nằm phía sau nhưng nhìn **thẳng về trước**. Không nghiệm thu Maze Solve khi chưa có kiểm thử hiện trường và số đo độc lập về giữ tâm/dừng.
