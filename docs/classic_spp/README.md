# Nezumi Chan — Bluetooth Classic SPP (Windows) — README

**Nhánh:** giomuadongbac | **Ngày cập nhật:** 28/10/2026 | **Nền tảng đã thử:** Windows 11 + Python 3.12 + JDY-33.

## 1. Kết nối hoạt động như thế nào?

Micromouse chỉ sử dụng **một module JDY-33**:

- **Classic SPP:** tên Nezumi_Chan_SPP. Ứng dụng desktop Python kết nối bằng **cổng COM Bluetooth ảo của Windows**.
- **BLE:** tên Nezumi_Chan. Web Bluetooth sử dụng dịch vụ FFE0 / characteristic FFE1.
- Cả hai dùng USART2 của STM32: **PA2 TX -> JDY-33 RXD; PA3 RX <- JDY-33 TXD**; UART **115200 baud, 8N1**.
- Công cụ desktop hiện **receive-only**: nhận telemetry/bản đồ, không gửi motion command, không ghi Flash, không thay đổi cấu hình STM32.

Để chạy SPP **không cần cắm dây ST-Link hoặc USB–TTL vào PC**. Cấp nguồn cho Micromouse, bật Bluetooth laptop và mở đúng COM Outgoing. ST-Link chỉ để nạp/debug firmware qua SWD; USB–TTL chỉ để cấu hình AT hoặc debug UART qua dây.

~~~text
STM32 (src/bt_debug.c)
  USART2 PA2 (TX) -> JDY-33 RXD
                           |
                     Bluetooth SPP
                           |
                     Windows Outgoing COM
                           |
           +---------------+---------------+
           |                               |
 tools/classic_spp_link_diag.py  tools/classic_spp_dashboard.py
  (đo kết nối và lỗi)                (giao diện)
~~~

## 2. Danh sách module và file

| Thành phần | Đường dẫn trong repo | Công dụng |
|---|---|---|
| Driver UART Bluetooth trên STM32 | src/bt_debug.c, include/bt_debug.h | Truyền/nhận dữ liệu USART2 và bộ đệm TX/RX |
| Entrypoint firmware | src/main.c | Chọn chế độ runtime/diagnostic theo build flag |
| Chẩn đoán bản đồ (tùy chọn) | src/flash_map_diag.c, include/flash_map_diag.h | Gửi MAPCFG/MAPROW/MAPEND qua USART2, motor locked |
| Sinh/kiểm tra map | src/flash_map_protocol.c, include/flash_map_protocol.h | Định dạng map và xác thực payload |
| Cấu hình build | platformio.ini | Môi trường firmware mặc định và flash_map_diag_96m |
| **Giao diện desktop chính** | **tools/classic_spp_dashboard.py** | Tkinter: Terminal, Maze, telemetry/PID plot, ghi/replay log |
| **Test kết nối SPP** | **tools/classic_spp_link_diag.py** | COM scan, thời gian mở COM, số byte nhận và lỗi hệ thống |
| Phân loại COM Windows | tools/classic_spp_ports.py | Heuristic phân loại Incoming/Outgoing từ BTHENUM |
| Bộ giải mã giao thức | tools/classic_spp_protocol.py | Line framing, MAP CRC32, parse telemetry |
| Debug Windows | tools/windows_bluetooth_link_debug.ps1 | Event Viewer + PnP khi điều tra pairing/disconnect |
| Unit tests | tests/regression/classic_spp_protocol_test.py | Kiểm thử parser và nhận diện COM offline |
| Web BLE cũ | web/ble_debug_console.html | Web Bluetooth độc lập, không thay đổi |
| Tài liệu mở rộng | docs/system_debug/JDY33_CLASSIC_SPP_DESKTOP_20261028.md | Mô tả tính năng và giới hạn |
| Debug kết nối nâng cao | docs/system_debug/WINDOWS_CLASSIC_SPP_CONNECTION_DEBUG_20261028.md | Windows Events, PnP và ETW |

**Không có module Bluetooth thứ hai; không cần sửa PCB/USART2.** Tính năng Classic SPP desktop được bổ sung phía PC, dùng lại driver STM32 có sẵn.

## 3. Pull code và cài Python trên Windows

Mở PowerShell ở máy đang làm việc:

~~~powershell
cd D:\MICROMOUSE_26\code_mau
git switch giomuadongbac
git pull --ff-only origin giomuadongbac
python --version
python -m pip install pyserial
~~~

Python 3.12 đã thử thành công. Dashboard dùng Tkinter có trong bộ cài Python thông thường trên Windows.

**Chưa cần nạp firmware hoặc cài PlatformIO** nếu STM32 đang phát telemetry qua JDY-33. Nạp lại sẽ thay thế firmware đang chạy.

## 4. Pair với JDY-33 và xác định COM

1. Bật nguồn Micromouse để cấp nguồn cho JDY-33.
2. Đóng Web BLE Nezumi_Chan khi thử Classic SPP lần đầu.
3. Windows > **Settings > Bluetooth & devices > Add device > Bluetooth** > chọn **Nezumi_Chan_SPP** để ghép đôi.
4. Nhấn **Win+R**, nhập **bthprops.cpl** > tab **COM Ports**.
5. Chọn dòng **Direction = Outgoing**, **Name = Nezumi_Chan_SPP** (hoặc tên trước đây JDY-33-SPP); ghi số COM.
6. Nếu chưa có cổng Outgoing, chọn **Add... > Outgoing**, thiết bị Nezumi_Chan_SPP, dịch vụ Port.
7. Windows có thể hiển thị **Paired / Not connected** khi chương trình chưa mở COM; không cần kết nối thủ công trước.

Xem các COM có trên máy:

~~~powershell
python tools/classic_spp_link_diag.py --list
~~~

Lưu ý: phân loại Incoming/Outgoing từ Hardware ID là **heuristic**; thông tin Direction + Name trên Windows Bluetooth COM Ports là bước xác minh cuối cùng.

### Kết quả nghiệm thu kết nối thực tế 28/10/2026

- JDY-33 Classic SPP đã được ánh xạ vào **COM16 Outgoing** trên máy Windows được thử.
- Remote Bluetooth address tại thời điểm thử: **78:97:0D:11:D4:B4**.
- Baudrate: **115200**; Python **3.12.10**; pyserial **3.5**.
- Python mở COM sau khoảng **5,8 giây**, sau đó nhận bản tin định kỳ khoảng **1,5 giây/lần**.
- **40 giây, 1.955 byte nhận được, không có SerialException**.
- Bản tin điển hình:

~~~text
PID,READY,state=IDLE,locked=1,calibrated=1,rxbytes=12,ore=0,fe=126,ne=3,rxoverflow=0
~~~

Điều này xác nhận **STM32 -> JDY-33 -> Bluetooth SPP -> Windows -> Python** hoạt động. Không có bằng chứng ở đây để khẳng định chiều ngược lại, hoạt động đồng thời BLE/SPP, hay truyền đủ một map 16x16 đã nghiệm thu.

**COM16 không cố định**: trên cùng máy, COM16 từng là Incoming và chỉ sau khi tạo lại đã trở thành Outgoing. Mỗi lần Windows ghép đôi/tạo cổng lại, hãy kiểm tra.

## 5. Kiểm tra đường truyền SPP không cần GUI

Đảm bảo **không có chương trình nào khác đang mở cùng COM** (miniterm, dashboard, script khác).

~~~powershell
cd D:\MICROMOUSE_26\code_mau
python tools/classic_spp_link_diag.py --port COM16 --seconds 40
~~~

Thay COM16 nếu Windows cấp số khác. Các sự kiện quan trọng:

- **opening**: chuẩn bị mở cổng.
- **com_open**: Windows mở COM thành công; chưa đảm bảo có byte trên đường radio.
- **rx**: **đã nhận byte thực từ JDY-33**.
- **alive**: COM còn mở và thống kê lưu lượng.
- **serial_error**: lỗi đọc hoặc mất kết nối.
- **completed**: kết thúc phép thử bình thường.

Log JSONL: **results/classic_spp/spp_link_YYYYMMDD_HHMMSS.jsonl**.

## 6. Khởi chạy Python Dashboard

Chỉ sau khi công cụ test ở bước 5 kết thúc để giải phóng COM:

~~~powershell
python tools/classic_spp_dashboard.py --port COM16
~~~

Trong ứng dụng:

1. Chọn **COM16** và baud **115200**; bấm **Connect**.
2. Tab **Terminal**: xem các dòng PID,READY,... theo thời gian thực.
3. Tab **Telemetry & PID**: xem các trường số/đồ thị khi có bản tin tương ứng.
4. Tab **Maze**: chỉ xuất hiện bản đồ khi firmware phát MAPCFG/MAPROW/MAPEND hoặc định dạng được parser hỗ trợ.
5. Có thể ghi file log hoặc replay file đã lưu.

Firmware hiện đã ghi nhận bản tin **PID,READY**, chưa thấy MAPCFG/MAPROW trong phép thử vừa rồi. **Maze trống không phải lỗi kết nối SPP**.

## 7. Test bản đồ 16x16 (tùy chọn, không bắt buộc cho kết nối)

Firmware diagnostic **flash_map_diag_96m** có thể broadcast map giả lập qua USART2, sử dụng UART1 PA9/PA10 cho lệnh điều khiển map và **khóa motor**.

**Chỉ nạp khi thật sự muốn chuyển sang chế độ test map** và đã kết nối ST-Link:

~~~powershell
python -m platformio run -e flash_map_diag_96m -t upload
~~~

Nếu Windows Python báo No module named platformio, dùng CLI trong môi trường PlatformIO hoặc cài riêng PlatformIO Core; **không cần sửa hoặc nạp firmware để test SPP đang hoạt động**.

## 8. Xử lý sự cố

| Hiện tượng | Kiểm tra |
|---|---|
| Windows Paired nhưng Not connected | Bình thường khi chưa mở COM Outgoing |
| Mở COM thành công nhưng rx_bytes=0 | Đúng COM Outgoing chưa? JDY-33 có nguồn không? Firmware có phát USART2? |
| Access is denied | Đóng miniterm, Dashboard và script khác đang giữ COM |
| COM Incoming | Đừng dùng; tìm hoặc tạo đúng Outgoing cho Nezumi_Chan_SPP |
| GUI không có Maze nhưng có PID,READY | Cần firmware phát bản tin map, không phải vấn đề Bluetooth |
| serial_error hoặc mất kết nối | Thu log Python và Windows đồng thời để đối chiếu thời điểm |
| fe / ne khác 0 trong PID,READY | Counter lỗi UART RX của STM32; kiểm tra **chênh lệch/tốc độ tăng** qua thời gian, không kết luận do Bluetooth từ một giá trị đơn lẻ |
| BLE mất kết nối khi chạy SPP | Dual-mode không có nghĩa đã nghiệm thu hai link đồng thời; test lần lượt |

Theo dõi sự kiện Windows (PowerShell Administrator), chạy **đồng thời** với script COM ở cửa sổ PowerShell khác:

~~~powershell
powershell -NoProfile -ExecutionPolicy Bypass -File .\tools\windows_bluetooth_link_debug.ps1 -DurationSeconds 90
~~~

Log có thể chứa tên thiết bị và Bluetooth address; tránh đăng nguyên bản công khai.

## 9A. Kiểm thử giới hạn SPP với TIM11/IR/IMU 1 kHz

Có bộ **`classic_spp_rate_diag_96m`** và **`tools/classic_spp_rate_pipeline.py`** để đo tốc độ telemetry 64 mức tải 20–128 B (bao gồm quá tải 100–156% UART), tự chọn mức khuyến nghị từ điểm suy giảm đầu tiên cùng 20% dự phòng, so sánh dữ liệu PC với bộ đếm thời gian thực trên MCU, sinh `cases.csv`/`summary.json`. Chế độ benchmark khóa motor và khởi động bằng PA0; **không tự đánh giá được tải motor/điều hướng thực tế**.

[Chi tiết cách nạp, chạy và ngưỡng nghiệm thu](../system_debug/CLASSIC_SPP_REALTIME_RATE_20261028.md).

## 9. Kiểm thử offline

~~~powershell
python tests/regression/classic_spp_protocol_test.py
python -m py_compile tools/classic_spp_ports.py tools/classic_spp_protocol.py tools/classic_spp_link_diag.py tools/classic_spp_dashboard.py
~~~

Xem thêm:

- [JDY-33 desktop console](../system_debug/JDY33_CLASSIC_SPP_DESKTOP_20261028.md)
- [Windows Bluetooth connection debug](../system_debug/WINDOWS_CLASSIC_SPP_CONNECTION_DEBUG_20261028.md)
- [Flash Map/Web BLE pipeline](../system_debug/FLASH_MAP_WEB_PIPELINE.md)
- [Web BLE viewer](../../web/ble_debug_console.html)

**Không cần đổi lại tên Bluetooth, PIN, baudrate, hoặc firmware khi kết nối SPP đã hoạt động tốt.**
