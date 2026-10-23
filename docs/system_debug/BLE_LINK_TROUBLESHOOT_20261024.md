# BLE PA2/PA3 — chẩn đoán kết nối trước pipeline T01–T18 (2026-10-24)

## Ghi nhận từ WebBLE log ngày 2026-10-24

- `web/ble_debug_console.html` và bản sao cũ `tools/ble_debug_console.html`
  đã bị nối đuôi khoảng 10 KB mã JavaScript **sau** `</html>`.
  Trình duyệt hiển thị mã này thành văn bản. Bản sửa cắt bỏ đúng phần
  nối thừa, không đụng parser bản đồ 5×5/16×16.
- Phân loại log `u.includes('ERR')` khiến `!S` bị tô ERR
  chỉ vì tên trường `i2cerr=0` / `rxerr=0`.
  Bản sửa chỉ coi sự kiện mở đầu `!ERR,` là lỗi.
- Web Maze Monitor gọi `requestCurrentMap()` và gửi **byte ASCII 0x3F**
  (`?`) ngay sau connect, dù firmware đang là BLE benchmark chứ không
  phải map firmware. `?` không có newline và có thể ghép với
  `HELLO\n` thành `?HELLO` rồi gây `UNKNOWN_COMMAND`.
  Bản sửa firmware benchmark bỏ qua `?` đứng riêng đầu lệnh.
  Giao thức Flash Map firmware giữ nguyên.
- Python runner trước đó ưu tiên GATT Write With Response khi FFE1 có
  cả hai thuộc tính; browser dùng Write Without Response trước.
  Bản sửa đồng nhất ưu tiên Write Without Response, ghi lỗi GATT dễ hiểu.
  **Không** suy ra rằng lỗi `Unreachable` hoàn toàn do kiểu write;
  cùng lúc mở Web và Python cũng có thể gây tranh kết nối.

Từ log 6000–21000 ms: TIM11 chạy thêm 15000 tick / 15 s = 1 kHz,
IR thêm 15000 lượt quét, `late=gap=stale=i2cerr=i2cto=txdrop=0`,
`locked=1`. `maxcy=1040` chu kỳ ≈ 10.83 µs ở 96 MHz danh định.
Đây chỉ là profile status ≈1 Hz, **chưa nghiệm thu 100–400 Hz BLE**
hay motor ON.

**Không chạy Web BLE và Python benchmark song song.** Muốn xem status đơn
thuần thì dùng Web (mở `http://localhost:8000/ble_debug_console.html`
khi server chạy `python -m http.server 8000 --directory web`).
Muốn đo end-to-end nhị phân thì disconnect Web, đóng tab, cấp lại nguồn
JDY-33 nếu cần, dùng `--probe` rồi `ble_full_pipeline.py` từ Python.

## Hiện tượng

- `BLE device not found: Nezumi_Chan`: Windows BLE scanner không thấy
  quảng bá đúng tên. Nó **không** chứng minh chip JDY-33 đã hỏng; tên quảng bá
  có thể khác, bị ẩn hoặc module đang kết nối với ứng dụng khác.
- `No !H within 6s`: đường truyền GATT FFE1 đã mở nhưng `HELLO\n`
  không có phản hồi `!H`. Phải tách PC->BLE->JDY UART RXD/PA3 và
  PA2/JDY UART TXD->BLE->PC.
- `!ERR,reason=UNKNOWN_COMMAND`: firmware đã đọc một dòng UART không
  nằm trong bộ lệnh; firmware mới báo thêm `rxhex` và `rxlen` để biết
  chính xác nội dung byte nhận được, không suy đoán nguyên nhân.

## Đúng trình tự — không bật motor

Tắt mọi tab WebBLE đang kết nối, ứng dụng điện thoại Bluetooth terminal,
và chương trình Python khác giữ JDY-33. Đảm bảo PC bật Bluetooth và JDY-33
có nguồn 3.3V hợp lệ; TX/RX **chéo**, PA2->RXD, PA3<-TXD, nối mass chung.
Motor VM có thể tắt độc lập.

1. Quét thiết bị quảng bá, **không flash**, không chạy test:

   ```powershell
   python tools/ble_full_pipeline.py --scan
   ```

   Kết quả gồm `name`, `address`, `services`, `rssi`.
   Một địa chỉ ngẫu nhiên không tự chứng minh đó là JDY-33; cách nhận dạng
   là tắt/cấp lại nguồn JDY, đối chiếu thiết bị xuất hiện biến mất,
   hoặc xác minh tên từ thiết bị BLE/ứng dụng đáng tin cậy. Không chọn bừa
   thiết bị `NCTH69OIG` hay những tên khác chỉ vì xuất hiện trong danh sách.

2. **Probe chỉ đường truyền**, nạp motor-OFF bằng ST-Link nếu chưa chắc
   firmware nào đang chạy; gõ đúng tên thực tế thay `Nezumi_Chan`:

   ```powershell
   python tools/ble_full_pipeline.py --probe --name Nezumi_Chan
   ```

   Hoặc sau khi nhận diện module đúng từ lần quét:

   ```powershell
   python tools/ble_full_pipeline.py --probe --address "XX:XX:XX:XX:XX:XX"
   ```

   `--probe` chỉ gửi `HELLO`, `PING`, `STATUS`, `HELLO` để
   kiểm tra chính xác byte ghi/nhận. **Không gửi ARM, RUN hoặc yêu cầu
   motor ON**. Nếu board đã chắc chắn chạy
   `ble_profile_motor_off_115200_96m`, thêm `--no-flash` để bỏ ST-Link.

3. Chỉ sau khi `PROBE PASS` thì mới chạy pipeline đo đầy đủ ở motor OFF:

   ```powershell
   python tools/ble_full_pipeline.py --name Nezumi_Chan
   ```

4. Lưu và gửi các file trong `results/ble_one_shot_.../` của lần probe:
   `ble_probe_raw.bin`, `ble_probe_events.json`,
   `ble_probe_summary.json`, kèm `flash_*.log` khi sử dụng ST-Link.
   Nếu có lỗi từ full pipeline, gửi thêm `ble_summary.json` và các
   `*_events.json`, `*_raw.bin`.

## Cách đọc raw log

- **RX không byte nào:** kiểm tra GATT service FFE0, characteristic FFE1
  có notify, firmware boot, PA2->JDY RXD, module có nguồn và cùng baudrate.
  Trong trường hợp này lỗi có thể ở chiều phát của MCU, không kết luận BLE
  mất gói.
- **Có !BOOT / !S nhưng không !H / !P:** MCU phát được, nhưng lệnh từ
  BLE xuống USART2 PA3 không nguyên vẹn. Kiểm tra GATT write properties,
  chiều PC->BLE, dây JDY TXD->PA3, UART RXNE/ORE/FE/NE.
- **Có !ERR UNKNOWN_COMMAND, rxhex=...:** so sánh hex với lệnh Python:
  `HELLO\n` = `48 45 4C 4C 4F 0A`; trong `rxhex` sẽ không có
  byte LF kết thúc. Nếu khác, là bằng chứng byte tới firmware bị sai,
  bị tiền tố/ký tự thừa hoặc đến từ nguồn khác; nếu đúng nhưng firmware
  vẫn trả UNKNOWN, phải kiểm tra bộ parser/bản build.
- **Có !H và !P:** liên kết control cơ bản hoạt động, tiếp tục
  đo sequence/CRC và goodput của T01–T18; không suy ra từ đây rằng
  BLE 200 Hz đã PASS.

## Kiểm tra điện và firmware nếu probe không PASS

- Đảm bảo cấp cho JDY-33 đúng mức điện áp của **board module đang dùng**,
  logic UART tương thích 3.3V với STM32, TXD/RXD được nối chéo và GND chung.
- Xác minh PC thực sự kết nối tới JDY-33 chứ không phải BLE khác có tên lạ.
- USART2 PA2/PA3 = 115200 baud với profile OFF 115200. JDY-33 UART
  cũng phải cấu hình cùng baud; đổi PIO xuống 38400 mà không đổi baud
  module sẽ làm hỏng đường UART.
- Dùng logic analyzer hoặc USB-UART 3.3V **chỉ để nghe thụ động** nếu cần:
  quan sát PA2 có chuỗi `!BOOT`/`!S` và PA3 có
  `HELLO\n`. Không nối đồng thời TX của hai bộ điều khiển lên cùng
  PA3 khi chưa thiết kế ghép tín hiệu.
- Không chạy chế độ `--motor-on` trước khi kiểm tra xong; khi thử
  motor thật phải nâng/cố định bánh và có cắt VM vật lý độc lập.

Tham khảo: `docs/system_debug/BLE_ONE_SHOT_PIPELINE_20261024.md`.
