# System Test — OLED + MPU6050 + Encoder (26/10/2026)

## Mục đích
Firmware test độc lập opt-in `system_test_96m`, không thay đổi firmware production.
Menu OLED áp dụng nút PA0 như bản `main` cũ, nhưng dùng các driver RC2 hiện tại.

## Chế độ
| Mode | Phản hồi | Điều kiện dừng |
|---|---|---|
| TURN +90 | PID góc từ yaw gyro MPU6050; hai bánh quay ngược chiều | sai số <=2° và tốc độ góc <=12°/s liên tục 120ms |
| TURN +180 | tương tự; yaw không wrap qua ±180° | cùng tiêu chuẩn |
| STRAIGHT 20CM | encoder hai bánh đo quãng đường + PID giữ hướng từ IMU, bộ P khoảng cách | đạt encoder-estimated 200 mm |

Thông số riêng trong `src/system_test.c`: bánh **bán kính 34 mm**, 1430 xung/vòng.
Quãng đường/xung = 2π34/1430 = 0.14940 mm/xung; 200mm = **1338.79 xung trung bình**.
`include/pinout.h` production vẫn giữ đường kính cũ 35 mm, không bị sửa.
Lưu ý 1430 phải là số đếm quadrature thực của TIM3/4; nếu là PPR một kênh phải hiệu chỉnh.

## Menu
- MENU: nhấn ngắn PA0 để chọn 1/2/3, nhấn giữ >=650ms rồi nhả để sang READY.
- READY: nhấn giữ >=650ms rồi nhả **lần thứ hai** để chạy; nhấn ngắn về MENU.
- RUN: nhấn PA0 bất kỳ để dừng trong TIM11; UART1 `S` cũng dừng. UART không thể START.
- RESULT: nhấn ngắn về MENU; nhấn giữ về READY; không tự chạy tiếp.

## An toàn và giới hạn
- Mặc định motor locked; chỉ profile `system_test_96m` mới có API mở khóa sau xác nhận PA0.
- PWM giới hạn 26% (260/1000); motor watchdog 300ms, mất dữ liệu MPU >3ms sẽ khóa đầu ra.
- Mỗi bài có timeout lần lượt 4.5 / 7.5 / 4.5 giây; lỗi chiều encoder, góc quá giới hạn,
  LCD/I2C không ổn định sẽ khóa động cơ. OLED_Shared được phục vụ ngoài TIM11.
- OLED không hiện hoặc chưa có frame / MPU chưa calibration sẽ không cho RUN.
- Không có tránh vật cản IR. **Bắt buộc thử bánh nâng hoặc robot được cố định, ngắt được nguồn VM 2S độc lập**.
- PID gain/PWM min là giá trị khởi tạo, chưa hiệu chỉnh và chưa chứng minh sai số thực tế.
- Ban đầu kiểm tra cực tính gyro/yaw, chiều motor và encoder trước khi thử trên sàn.

## Build & nạp
```powershell
python -m platformio run -e system_test_96m
python -m platformio run -e system_test_96m -t upload
```
UART1 115200 8N1 có các khung `ST,BOOT`, `ST,START`, `ST,ms=` mỗi 250ms.

## Nghiệm thu tiếp theo
- Build CI chỉ chứng minh biên dịch. Cần đo thật yaw +90/+180, quãng đường 200mm (10 lần),
  độ lặp lại, overshoot, drift, độ trễ STOP và tình trạng OLED/I2C khi chạy motor.
- Nếu động cơ không thắng ma sát ở mức PWM an toàn hiện tại, không tự tăng trước khi
  đo dòng, áp, nhiễu và kiểm tra điều kiện gá thử.

Additional runtime guards reject implausible gyro/NaN, >15 degree heading deviation
in the straight test, severe encoder disagreement, or one wheel not responding.
