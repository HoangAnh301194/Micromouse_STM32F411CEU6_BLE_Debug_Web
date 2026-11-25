# v3 – Kiểm soát 7 hạng mục (25/11/2026)

1. **CI:** Thay GitHub Actions bị xung đột YAML bằng workflow cho `giomuadongbacv3`. Build runtime mặc định, build profile OLED opt-in, chạy native & Python regression.
2. **Build/test:** CI chạy các lệnh trong workflow, nhưng **không thay thế nghiệm thu STM32**. Nếu Actions thất bại, không nạp firmware motor-capable.
3. **OneBox thực tế:** giữ nguyên dừng an toàn `HOST_LOST`, `STALL`, `ENCODER`, `TIMEOUT`; yêu cầu `ONEBOX,RESULT` và `locked=1` sau chạy.
4. **PID:** không tự đổi Kp/Ki/Kd. Phân tích `REF_L/R_PPS`, `SPEED_L/R_PPS`, `PWM_L/R`, MAE/RMSE và tỉ lệ PWM bão hòa với `tools/onebox_audit.py`.
5. **CSV:** bổ sung `REL_L,REL_R` count kể từ START và mẫu kết thúc dùng thời gian/count chính xác từ OneBox core. Giữ trường absolute cũ để tương thích. Ghi log khoảng 50 Hz KHÔNG chứng minh 1 kHz control.
6. **OLED:** profile mặc định `mode_menu_96m` không đổi; chỉ profile `mode_menu_oled_diag_96m` bật OLED probe. Ngắt VM motor và kiểm tra I2C trước khi flash opt-in profile.
7. **Heading:** bộ phân tích shadow **chỉ offline**, tính `yaw_proxy_deg` theo chênh lệch quãng đường hai bánh và wheelbase đo thực. Không gửi PWM hoặc thay lệnh motor. Để có điều khiển hướng closed-loop, cần nghiệm thu gyro/IR đồng bộ và điều kiện an toàn riêng.

Chạy:
```powershell
py tests/regression/spp_dual_log_test.py
py tests/regression/encoder_onebox_native_test.py
py tools/test_onebox_audit.py
pio run -e mode_menu_96m
py tools/onebox_audit.py "logs/spp_runs/mode 7 - encoder test - 1 box forward/<file>.csv" --wheelbase-mm 74
```
Tham số 74 chỉ là **ví dụ cách nhập**; thay bằng wheelbase đo trên Micromouse. Không lấy số ví dụ làm cấu hình điều khiển.
