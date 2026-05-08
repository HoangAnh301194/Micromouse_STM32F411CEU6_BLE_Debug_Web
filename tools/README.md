# Tools

## BLE Debug Console

File `ble_debug_console.html` là công cụ host-side để:

- kết nối robot qua Web Bluetooth
- xem log UART/BLE
- hiển thị maze, flood map, path và pose robot

### Cách dùng

1. Mở file HTML bằng Chrome hoặc Edge.
2. Bật nguồn robot và module BLE.
3. Nhấn `Connect BLE`.
4. Chọn thiết bị quảng bá service `0xFFE0`.

### Ghi chú

- Console hiện đồng bộ với firmware cấu hình `maze 5x5`.
- Firmware phát `CELL:` để cập nhật từng bước và `MAZE:` để dump đầy đủ.
- Nếu bạn đổi kích thước maze trong firmware, hãy sửa hằng số tương ứng trong file HTML này.
