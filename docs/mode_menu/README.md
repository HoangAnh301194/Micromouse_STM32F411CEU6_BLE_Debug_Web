# Safe 7-mode menu and passive System Test over Classic SPP

**Branch:** `giomuadongbacv2`  
**Build:** `mode_menu_96m` (default in `platformio.ini`).

## Chế độ điều khiển

- KEY PA0 vật lý: nhấn ngắn để chuyển, giữ ít nhất 400 ms để chọn; debounce 30 ms.
- Hai nút SPP **Chuyển (KEY ngắn)** và **Chọn (KEY giữ)** gửi một byte ASCII `N` hoặc `K` đến STM32. Mỗi cú nhấn gửi một sự kiện logic, không giữ phím điện tử 400 ms.
- **Stop test** gửi `P` để tạm dừng truyền mẫu của bài test đang chọn; **Start test** gửi `G` để tiếp tục. Cả hai lệnh không tác động motor và chỉ được firmware chấp nhận khi đang chọn IR/Encoder/MPU Test. Dashboard chỉ cập nhật trạng thái nút khi nhận ACK từ STM32.
- Sau khi chọn cổng COM Bluetooth **Outgoing**, dashboard gửi byte `?` để kiểm tra hai chiều. Nếu không thấy `SPP CONNECTED`, desktop tự gửi lại mỗi 0,8 giây trong sáu lần đầu rồi giảm xuống mỗi 3 giây. Chỉ khi nhận **đúng** bản tin này, hai nút mới được bật. Mở được COM không có nghĩa là MCU đã sẵn sàng nhận lệnh.
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
Nhấn Chọn để kích hoạt và bắt đầu truyền luồng đo thụ động; nhấn Chuyển sẽ **dừng luồng trước** đồng thời chuyển focus sang mục tiếp theo. Chọn Back để trở lại menu chính.
Trong lúc bài test đang chạy, **Stop test** chỉ tạm dừng truyền bản tin đo; encoder và các timer phần cứng vẫn tiếp tục đếm/cập nhật. **Start test** sẽ tiếp tục đọc cùng bộ đếm tích lũy, không reset về 0. Cả IR, Encoder và MPU đều dùng chung hai nút này. Khung đo còn trong hàng đợi Bluetooth có thể đến muộn sau khi nhận thao tác Stop.

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
TEST STREAM STOPPED: Encoder Test
TEST STREAM STARTED: Encoder Test
System Test 3: MPU Test
System Test 4: Back
System Test: back to main menu
```

Data log bên trái phát **20 mẫu/giây (50 ms/mẫu)** **chỉ cho bài test đang được chọn**. Mỗi lần chọn bài test mới, dashboard tự xóa bảng dữ liệu cũ và đổi tiêu đề CSV cho đúng loại cảm biến.

**Encoder Test — đúng ba cột CSV (`timestamp,ENC_L,ENC_R`):**

Firmware gửi trên UART/SPP (có tiền tố `ENC` để phân loại bản tin):
```text
ENC,21,-19
ENC,27,-22
ENC,32,-26
```

Dashboard **hiển thị và xuất file CSV** (bỏ tiền tố định tuyến `ENC` và tự thêm timestamp từ đồng hồ Windows tại thời điểm Python nhận bản tin):
```csv
timestamp,ENC_L,ENC_R
2026-11-23T14:56:41.241,21,-19
2026-11-23T14:56:41.291,27,-22
2026-11-23T14:56:41.341,32,-26
```

Đây là **thời gian nhận trên máy tính (ISO 8601, giờ địa phương)**, không phải thời điểm MCU chốt xung. Bản tin firmware mới chỉ gửi `ENC,<left>,<right>`; timestamp được Python gắn vào để phục vụ phân tích log.

- `ENC_L`: số xung quadrature có dấu tích lũy của encoder bánh trái TIM4, PB6/PB7.
- `ENC_R`: số xung quadrature có dấu tích lũy của encoder bánh phải TIM3, PB4/PB5.
- `Encoder_Update()` vẫn được gọi mỗi 1 ms trong TIM11; không đổi cấu hình encoder, PPR, bộ lọc hay hướng.
- **Không gửi** `ms`, `dt_ms`, `delta_l`, `delta_r`, `pps_l`, `pps_r`, `window_l`, `window_r`, IR hoặc MPU trong khung Encoder Test.
- Đây là **count tích lũy**, không phải xung/giây. Nếu cần đo tốc độ sau này, kết hợp thời gian lấy mẫu hoặc dùng công cụ đo riêng. `ENCODER_PPR=1430` trong `pinout.h` vẫn cần kiểm chứng trên thực tế.

**IR Test — chỉ bảng IR**, không có cột encoder hay MPU:

```csv
valid,L0,L45,L90,R0,R45,R90
1,8,9,10,11,12,13
```

Firmware vẫn lấy snapshot sáu cảm biến IR, `valid=0` khi chưa có scan hoặc mẫu cũ hơn 5000 µs.

**MPU Test — chỉ bảng MPU**, không có cột encoder hay IR:

```csv
valid,yaw_cd,gz_cd
1,120,-35
0,,
```

`yaw_cd` = góc yaw × 100 (đơn vị 0,01 độ), `gz_cd` = vận tốc quay trục Z × 100 (đơn vị 0,01 độ/s); `valid=0` thì các ô đo để trống.

Mỗi phiên COM tự lưu trong `logs/spp_runs/connections/`:

- `*_status.log`: sự kiện mode, SPP và cảnh báo.
- `*_encoder.csv`: **chỉ `timestamp,ENC_L,ENC_R`**, tự tạo khi nhận Encoder Test. Cả bản tin encoder cũ (`ENC,ms=...,left=...,right=...,delta_l=...,...`) và khung mới (`ENC,2620,4230`) đều được chuẩn hóa thành đúng ba cột, không mang theo `pps`, `delta` hoặc `raw`.
- `*_ir.csv`: chỉ các cột IR và `valid`.
- `*_mpu.csv`: chỉ các cột MPU và `valid`.
- `*_data.csv`: file tương thích với những dòng telemetry khác và giao thức cũ ngoài ba bài test. **Khi phân tích Encoder Test, mở `*_encoder.csv`, không mở `*_data.csv`**. File CSV cũ đã tạo trước khi cập nhật sẽ không tự thay đổi.

Ví dụ là dữ liệu minh họa, **không phải kết quả thực nghiệm**. Các file cảm biến sẽ chỉ xuất hiện khi có mẫu hợp lệ theo định dạng của từng test. Muốn Stop/Start **ngừng phát từ MCU** cần nạp firmware `mode_menu_96m` mới; cập nhật mỗi Python chỉ đủ để chuẩn hóa CSV cũ.

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
py tests/regression/spp_handshake_test.py
```

Cấu hình `mode_menu_96m` **luôn gọi `Motor_LockOutputs()`** trước khi khởi tạo menu. Không có lệnh motor/PID/Flash ở Mode 7 hoặc trên hai nút từ xa. Khi thử encoder, tách VM 2S và quay bánh bằng tay, cố định thân xe; sau đó kiểm tra dấu tiến dương, số xung/vòng và tương quan tốc độ giữa hai phía. Kết quả kiểm thử native hoặc build không thay cho phép đo trên phần cứng thật.

**Lưu ý:** đây là menu test mới trong profile `mode_menu_96m`, không phải profile `system_test_96m` cũ vốn có bài thử motor 90°/180°/200 mm. Không nạp nhầm profile motor-capable.

## Kết nối SPP và nút điều khiển bị mờ (23/11/2026)

Trong code hiện tại, hai nút Chuyển/Chọn khởi tạo `state="disabled"`
và được bật chỉ sau khi nhận đúng dòng `SPP CONNECTED`.
Đây là khóa bảo vệ giao diện, không phải một nút bị lỗi.

- `Opening COMx...` hoặc `COMx COM open`: Windows đã mở cổng COM, **chưa chứng minh** link SPP hai chiều hoạt động.
- `SPP RX data seen; waiting for firmware handshake`: có ít nhất một byte RX, **chưa chứng minh** firmware menu đã phản hồi `?`.
- `SPP CONNECTED`: vòng truyền `? -> STM32 -> SPP CONNECTED` được xác nhận; hai nút được bật.
- Sau sáu lần thử không có ACK, ứng dụng hiện cảnh báo hướng dẫn chọn COM **Outgoing** và nạp đúng `mode_menu_96m`; vẫn thử lại 3 giây/lần và giữ nút vô hiệu hóa.
- COM chưa mở hoặc bị chiếm/đứt RFCOMM: ưu tiên kiểm tra Windows Bluetooth hơn là thay đổi firmware.

Kiểm tra COM trên Windows:

```powershell
py tools/classic_spp_link_diag.py --list
```

Trong **More Bluetooth settings > COM Ports**, xác nhận đúng thiết bị JDY-33,
cột **Direction = Outgoing**. Dashboard **không còn mặc định COM37**: nó
chỉ tự điền COM khi chỉ có một thiết bị JDY-33 xác định rõ hoặc duy nhất
một cổng Bluetooth Outgoing. Nếu có nhiều thiết bị, tự chọn COM sau khi
xem `--list`. Lệnh `--port COM8` vẫn được ưu tiên khi nhập tường minh. Không mở dashboard đồng thời với terminal hoặc script
chẩn đoán trên cùng cổng.

Nếu cần kiểm tra riêng tầng Windows/Bluetooth, **đóng dashboard trước** rồi chạy:

```powershell
py tools/classic_spp_link_diag.py --port COM8 --seconds 20
```

Thay COM8 bằng đúng cổng Outgoing. Script này chỉ kiểm tra COM/RX, không gửi
`?`, do đó **không** thể chứng minh firmware đã hiểu menu khi kết quả chỉ
`no_serial_error`. Để kiểm tra toàn bộ đường handshake, mở lại dashboard và
quan sát status `SPP CONNECTED`.

**Quan trọng:** firmware tại `src/main_runtime.c` hiện vẫn dừng vô hạn sau
`MPU6050_Init()` thất bại. Một boot failure có thể khiến COM mở nhưng không
phản hồi `?`. Bản sửa retry chỉ khắc phục trường hợp mất probe trong giai
đoạn khởi tạo link; nó không sửa lỗi MPU hay lỗi ghép đôi Windows.

Kiểm thử no-radio: `py tests/regression/spp_handshake_test.py`.
