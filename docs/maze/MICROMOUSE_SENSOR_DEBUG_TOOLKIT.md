# High-speed Micromouse sensor test / debug toolkit

**Branch:** `giomuadongbac` · **Status:** triển khai firmware + tool phân tích; **chưa nghiệm thu thu/phát dữ liệu trên robot thực tế**. Không sửa các mode motion 90/180, Wall PID 720 mm hoặc production. Tất cả phân tích bên dưới đều là **offline**; mọi giới hạn ADC dùng trong phân tích wall-state **không nối vào điều kiện dừng motor**.

## 1. Khảo sát thực tế và quyết định kỹ thuật

1. [Peter Harrison — Calibrating Reflective Sensors](https://micromouseonline.com/2010/07/07/calibrating-reflective-sensors/): LED ON−OFF tách nền ánh sáng nhưng ADC còn phụ thuộc góc và độ phản xạ. Mỗi cảm biến cần hiệu chuẩn riêng trên nhiều khoảng cách.
2. [Peter Harrison — Easier Wall Sensor Calibration](https://micromouseonline.com/2011/05/07/easier-wall-sensor-calibration/): ưu tiên phương pháp chuẩn hóa/phân loại có thể tái lập; dùng đường đặc tính thực thay vì nội suy từ hai điểm rồi tuyên bố mm chính xác.
3. [Micromouse USA — Calibration Strategy Part 2](https://micromouseusa.com/?p=828): thời điểm chuyển **wall→opening** qua cảm biến chéo giúp sửa sai số vị trí dọc khi đi thẳng nhiều cell. Phải đo sai số sớm/muộn theo **encoder mm**, không chỉ đồng hồ.
4. [Micromouse Online — Sensor Alignment](https://micromouseonline.com/2011/05/29/wall-sensor-alignment-error/): hướng chiếu 45° ảnh hưởng mạnh vị trí phát hiện mép, nhất là khi chuẩn bị cua.
5. [UKMARS steering](https://github-wiki-see.page/m/ukmars/ukmarsbot-mazerunner/wiki/steering) và [sensors](https://github-wiki-see.page/m/ukmars/ukmarsbot-mazerunner/wiki/sensors): chuẩn hóa cảm biến, phân biệt lệch ngang/yaw; tránh dùng tín hiệu mất tường như một lỗi vị trí ngang.
6. [Micromouse Online — Sensors](https://micromouseonline.com/micromouse-book/sensors/): **vị trí đọc tường để cập nhật map** khác vị trí cảm biến được dùng cho lái; front wall còn liên quan stopping distance.
7. [Wang (2022), A Design of Micromouse Control System](https://alife-robotics.co.jp/LP/2022/OS30-1.htm): ví dụ STM32 với 6 nhóm IR, PWM và điều hướng maze; bài hội nghị mô tả cấu trúc hệ thống, không cung cấp cơ sở để mặc định gain/ngưỡng của robot này.

**Bố trí thực tế:** `L0,R0` đo tường ngang; `L45,R45` quan sát chéo phía trước và mép tường; `L90,R90` **được lắp ở phía sau thân nhưng nhìn thẳng về trước** để quan sát tường đầu ô tiếp theo. Tọa độ điểm đặt LED và góc tia chiếu phải đo trên robot, không được mặc định chúng cùng vị trí.

## 2. Tool đã bổ sung

| Tool | Chạy ở đâu | Vai trò |
|---|---|---|
| `maze_sensor_capture_96m` | STM32, **motor-locked** | Thu 6× EMA0.10 + encoder L/R + yaw/gyro + timestamp/scan, 200 Hz, tối đa 512 mẫu (2,56 s). PA0 START/STOP; dump BLE sau capture; không dùng UART1, không ghi Flash. |
| `maze_sensor_capture_report.py` | PC offline | Giải mã `MC` từ Web BLE export; kiểm tra thứ tự, độ dài, FNV32, age, drop; tạo `trace.csv`, `capture_qc.json`, `viewer.html` tương tác. |
| `ir_calibration_lab.py` | PC offline | Fit LUT phi tuyến **từng kênh + vật liệu + ánh sáng** trên các khoảng cách đo thật, isotonic monotonic, holdout; không extrapolate. |
| `maze_wall_observer.py edges` | PC offline | Phát hiện ứng viên cạnh tường xuất hiện/mất; đo sai số vị trí mép với nhãn ground truth tính theo mm encoder. |
| `maze_wall_observer.py fusion` | PC offline | Wall state `WALL/OPEN/UNKNOWN`, hai kênh mâu thuẫn → `UNKNOWN`; tạo ứng viên dữ liệu cell kế tiếp khi có hình học được đo. |
| `maze_readiness_suite.py` (đã có) | PC offline | Tiền kiểm, so sánh nhiều Wall PID run, confusion matrix theo nhãn TRAIN/TEST của 6 kênh. |

## 3. Quy trình thu log lần đầu (không cho motor chạy)

Ngắt **nguồn VM motor vật lý**, cấp 3,3 V logic, kiểm tra PA0 / PC13 / JDY-33. Đặt robot trên đoạn maze có cấu hình tường được đo sẵn. Dùng tay dịch chuyển robot *cùng bánh trên mặt sàn* nếu muốn encoder phản ánh quãng đường; **không cố ép motor/gearbox bằng lực lớn**. Nếu bánh không xoay hoặc trượt thì dữ liệu encoder không là ground truth quãng đường.

```powershell
git pull --ff-only origin giomuadongbac
python -m platformio run -e maze_sensor_capture_96m
python -m platformio run -e maze_sensor_capture_96m -t upload
```

- Kết nối Web BLE console `web/ble_debug_console.html`, đọc `MC,READY`.
- **PA0 nhấn–thả** → START RAM capture. LED PC13 nháy nhanh.
- Dịch robot qua 1–4 mép tường (trong **≤2,56 giây**); **nhấn–thả PA0 lần hai** để ngừng, hoặc tự đầy 512 mẫu.
- LED sáng khi dump; BLE TX chia **20 byte /125ms**, rate danh định 160 B/s. 512 bản ghi ASCII có thể cần **vài phút**, đừng ngắt nguồn hoặc đóng kết nối trước `MC,END`.
- **Export terminal** sau khi có `MC,END`; giữ nguyên từng dòng `MC,...`, không cắt bỏ phần giữa. Tool kiểm FNV32 và số thứ tự, thiếu/đổi một dòng sẽ FAIL. Tốc độ BLE danh định chưa phải thông lượng OTA được nghiệm thu. Bài thu này chỉ đo trên **robot đứng/move bằng tay**, không đo được IR ở tốc độ 0,9 m/s thực khi motor chạy.

```powershell
python tools/maze_sensor_capture_report.py --input maze_01_ble.txt
```

Mở `maze_01_ble_maze_capture/viewer.html` trên Chrome. Đồ thị gồm **6 ADC EMA**, encoder distance, gyro yaw/rate, tuổi mẫu IR; có con trỏ theo frame, trục thời gian/quãng đường và đường chia cell 180 mm. Nếu `capture_qc.json:complete=false`, **không dùng log để hiệu chuẩn tốc độ cao**.

## 4. Thí nghiệm calibration phi tuyến

Để phù hợp đặc tính đo cậu quan sát (khoảng 20 mm ADC ~3000 và 70 mm ADC ~500–600), đo từng kênh ở **nhiều khoảng cách**, từ 20,25,30,35,40,45,50,60,70,80 mm hoặc các mốc gá cho phép. Đặt **lại robot/tường mỗi trial**, đo bằng thước từ mặt cảm biến đến mặt tường, kiểm tra góc chiếu; mỗi mốc tối thiểu 2–3 trial và tách tối thiểu 2 trial TEST không tham gia fit.

Chuyển từ log IR tĩnh đã có sang manifest (chỉ tự suy L0/R0 từ distance_left/right; cảm biến chéo/trước cần đo riêng):

```powershell
python tools/ir_calibration_lab.py manifest --session results/ir_full_qualification/ir_wall_01 --output calib_manifest.csv
```

Thêm hàng cho các lần đo khác với cột
`trace_path,channel,distance_mm,trial_id,surface,light,split`. Một channel/mốc/cấu hình = một lần đặt độc lập. Mỗi `trace_path` trỏ tới `ir_filter_trace.csv`; `split=train` hoặc `test` theo lần đặt vật lý.

```powershell
python tools/ir_calibration_lab.py fit --manifest calib_manifest.csv --output results/ir_lut_lab
```

Mở `results/ir_lut_lab/calibration_viewer.html`, xem LUT gợi ý / phạm vi / độ lỗi holdout. **Không dùng `lut_candidates.csv` để điều khiển** nếu thiếu mốc/góc/ánh sáng và chưa nghiệm thu trên robot.

## 5. Wall lost, next-cell và sensor fusion

Phát hiện các **ứng viên biến thiên nhanh**, *không tự kết luận tường mất* khi chỉ thấy ADC thấp:

```powershell
python tools/maze_wall_observer.py edges --trace maze_01_ble_maze_capture/trace.csv --out results/maze_edge_lab
```

Nếu cậu có tọa độ thật của cạnh tường (đo mm trên track), tạo `edge_truth.csv`:

```csv
channel,type,distance_mm
L45,WALL_LOST,360.0
R45,WALL_APPEAR,540.0
```

Sau đó thêm `--truth edge_truth.csv`. Tool tính `offset_mm`, `matched/missed`; **các event chưa gán nhãn chỉ là candidate**.

Để replay trạng thái WALL/OPEN/UNKNOWN, dùng **mô hình nhãn TRAIN/TEST** tạo bởi:

```powershell
python tools/maze_readiness_suite.py wall-events --train train_wall.csv --test test_wall.csv
python tools/maze_wall_observer.py fusion --trace maze_01_ble_maze_capture/trace.csv --model results/maze_readiness/wall_events/wall_detection_report.json
```

Không có hình học, tool vẫn xuất timeline của từng kênh và ba hướng nhưng **không gán cell cho tường**. Để gán ô dự kiến, cần tự đo 6 khoảng chiếu trước của cảm biến và tạo `geometry.json`:

```json
{
  "measured_and_reviewed": true,
  "forward_projection_mm": {
    "L90": 0, "L45": 0, "L0": 0,
    "R0": 0, "R45": 0, "R90": 0
  }
}
```

**Các số 0 phía trên chỉ là placeholder của cấu trúc, không phải thông số hình học hợp lệ để đem đo.** Điền 6 giá trị thật rồi thêm `--geometry geometry.json`. Công cụ mới dùng phép dịch chiếu dọc x như **phép gán nhãn ứng viên ban đầu**; chưa phải ray casting 2D hay phương pháp xác định đúng tường trước trong mọi góc quay. Muốn triển khai thực, phải đo vị trí/góc LED và kiểm chứng từng edge/cell trên track có ground truth.

## 6. Những bộ cần phát triển tiếp

**P0 trước chạy maze liên tục tốc độ cao:** (1) logger 6 IR trong chính firmware motor-on `wall_pid_4cell_96m` với RAM snapshot 200 Hz, một mode *opt-in* riêng và giám sát dừng độc lập; (2) đo braking/coast stopping distance trên mặt sàn với mốc thật — lượt cũ từng trôi **80,2 mm** sau motor lock; (3) bộ gán nhãn tường theo tọa độ cell và vị trí sensor-ray chuẩn. **Chưa triển khai các bộ này trên phần cứng motor-on.**

**P1:** vị trí/hướng hợp nhất encoder+gyro + wall correction khi cặp tường tin cậy, bộ so sánh yaw-gyro với yaw suy từ encoder cần đo wheelbase; wall-edge re-localization; confidence & hysteresis theo tốc độ. Trạng thái `UNKNOWN` phải ngăn cập nhật map sai, không nhất thiết bắt robot dừng.

**P2:** map 16×16 consistency/flood-fill replay, điều kiện `wall_observation_at_cell_center`, next-cell confidence và multi-cell slalom path continuity; kiểm tra map trong khi robot đi liên tục.

**Chưa nghiệm thu phần cứng của các mode mới.** PASS ở CI chỉ chứng minh parser, thuật toán cơ bản trên dữ liệu giả lập và firmware build được; không chứng minh latency BLE, chất lượng phát hiện tường hay khả năng đi đủ bốn cell tốc độ cao.
