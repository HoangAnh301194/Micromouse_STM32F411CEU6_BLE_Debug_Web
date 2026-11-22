# Safe 7-mode menu and passive System Test over Classic SPP

**Branch:** `giomuadongbacv2`  
**Build:** `mode_menu_96m` (default in `platformio.ini`).

## Chế độ điều khiển

- KEY PA0 vật lý: nhấn ngắn để chuyển, giữ ít nhất 400 ms để chọn; debounce 30 ms.
- Hai nút SPP **Chuyển (KEY ngắn)** và **Chọn (KEY giữ)** gửi một byte ASCII `N` hoặc `K` đến STM32. Mỗi cú nhấn gửi một sự kiện logic, không giữ phím điện tử 400 ms.
- Sau khi chọn cổng COM Bluetooth **Outgoing**, dashboard gửi một byte `?` để kiểm tra liên kết. Chỉ khi nhận `SPP CONNECTED`, hai nút mới được bật.
- Mọi thao tác đọc/ghi serial được thực hiện trong cùng worker thread; Tkinter chỉ gửi lệnh vào queue.

## Cấu trúc menu

```text
Main menu:
1. Calibration
2. Run Slow
3. Run Faster
4. A* Run
5. Speedrun
6. Reset Map
7. System Test   --> 1. IR Test
                     2. Encoder Test
                     3. MPU Test
                     4. Back
```

Ở menu chính, chọn các mode 1–6 **chỉ phát trạng thái**, không thực hiện hành động nào.
Chọn Mode 7 để vào menu con (ban đầu chưa focus mục nào).
Nhấn Chuyển lần đầu focus IR, lần lượt Encoder, MPU, Back và quay vòng.
Nhấn Chọn để kích hoạt luồng đo thụ động; nhấn Chuyển sẽ **dừng luồng trước** đồng thời chuyển focus sang mục tiếp theo. Chọn Back để trở lại menu chính.

OLED và SPP báo cùng trạng thái. Status log bên phải chỉ nhận mỗi sự kiện một lần, không lặp heartbeat.

## Dạng bản tin mẫu

Ví dụ sau chỉ minh họa giao thức, **không phải số liệu thực nghiệm**.

Status log bên phải:

```text
SPP CONNECTED
Mode 7: System Test
Selected mode 7: System Test
System Test: choose IR / Encoder / MPU / Back
System Test 1: IR Test
System Test 2: Encoder Test
Selected test 2: Encoder Test
System Test 3: MPU Test
System Test 4: Back
System Test: back to main menu
```

Data log bên trái phát một bản tin mỗi **50 ms (20 Hz)** cho bài test đang chạy:

```csv
IR,ms=1050,valid=1,age_us=530,scan=1200,L0=8,L45=9,L90=10,R0=11,R45=12,R90=13
ENC,ms=1100,dt_ms=50,left=21,right=-19,delta_l=5,delta_r=-3,pps_l=101,pps_r=-58,window_l=100,window_r=-60
MPU,ms=1150,valid=1,age_us=280,yaw_cd=120,gz_cd=-35
```

### Encoder

Encoder dùng TIM4 (bánh trái, PB6/PB7) và TIM3 (bánh phải, PB4/PB5) ở chế độ quadrature.
`Encoder_Update()` xử lý bộ đếm mỗi 1 ms trong TIM11. Hai hằng hướng đang bằng `-1` để chuẩn hóa chiều tiến dương.

| Trường | Định nghĩa |
|---|---|
| `left`, `right` | Tích lũy xung quadrature có dấu kể từ khởi động / reset encoder |
| `delta_l`, `delta_r` | Chênh lệch đếm từ bản tin trước |
| `dt_ms` | Thời gian thực tế giữa hai bản tin encoder |
| `window_l`, `window_r` | Tốc độ trung bình trên cửa sổ, xung/giây: `delta * 1000 / dt_ms` |
| `pps_l`, `pps_r` | Tốc độ qua EMA alpha=0.10 đang có ở TIM11, xung/giây |

Đây là công cụ **đo và so sánh** hai cách tính tốc độ, **chưa đổi alpha hay PPR** vì cần dữ liệu thử bánh thật.
Giá trị `ENCODER_PPR=1430` trong `pinout.h` vẫn chỉ là giả định cấu hình cần đối chiếu quãng đường thực tế.

### IR và MPU

- IR gửi đúng thứ tự vật lý L0/L45/L90/R0/R45/R90, dữ liệu raw sau trừ ánh sáng nền, không phải khoảng cách mm; đọc snapshot được bảo vệ ngắt của TIM11. `valid=0` khi chưa có dữ liệu hoặc tuổi mẫu lớn hơn 5000 µs.
- MPU dùng snapshot đã hiệu chuẩn của MPU6050: `yaw_cd` = 100 × góc yaw (độ), `gz_cd` = 100 × vận tốc góc quanh Z (độ/s). `valid=0` khi dữ liệu không sẵn sàng hoặc tuổi lớn hơn 3000 µs.
- Các giá trị `ms` là uptime STM32; file CSV tự bổ sung `host_time`. File `raw` giữ nguyên bản tin gốc.

Log lưu tự động trong `logs/spp_runs/connections/` với hai file: `*_status.log` và `*_data.csv`.

## Build / flash / test

```powershell
git switch giomuadongbacv2
git pull --ff-only origin giomuadongbacv2
pio run -e mode_menu_96m
pio run -e mode_menu_96m -t upload
py tools/classic_spp_dashboard.py --port COM16
```

Thay COM16 bằng cổng COM Outgoing thực tế.

Hồi quy offline:

```powershell
py tests/regression/mode_menu_native_test.py
py tests/regression/spp_dual_log_test.py
```

Cấu hình `mode_menu_96m` **luôn gọi `Motor_LockOutputs()`** trước khi khởi tạo menu. Không có lệnh motor/PID/Flash ở Mode 7 hoặc trên hai nút từ xa. Khi thử encoder, tách VM 2S và quay bánh bằng tay, cố định thân xe; sau đó kiểm tra dấu tiến dương, số xung/vòng và tương quan tốc độ giữa hai phía. Kết quả kiểm thử native hoặc build không thay cho phép đo trên phần cứng thật.

**Lưu ý:** đây là menu test mới trong profile `mode_menu_96m`, không phải profile `system_test_96m` cũ vốn có bài thử motor 90°/180°/200 mm. Không nạp nhầm profile motor-capable.
