# P0 — Wall PID 720 mm + full-rate 6-IR logger (BLE)

**Branch** `giomuadongbac`. **Firmware:** `wall_pid_4cell_96m` (không tạo mode mới).
**Status:** native C tests & PlatformIO build (CI); **chưa đo hardware logger**.

## Nguyên tắc

- Điều khiển như cũ: 4×180 = **720 mm**, bánh **đường kính 34 mm**, **1430 count/vòng**, EMA IR α=0,10, Wall PID 200 Hz, mục tiêu 900 mm/s, PWM cap 80%.
- Chỉ `L0/R0` tạo sai số lái. Ghi thêm `L90,L45,R45,R90` để phân tích wall-edge / next-cell, **không đưa bốn kênh này vào PID hoặc điều kiện dừng**.
- 1 kHz TIM11 + 200 Hz logger trong RAM: ghi snapshot **chung tick sau WP_Step** (IR EMA 6 kênh, scan ID/age, encoder từng bánh, yaw/rate, sai số lái không đơn vị, tốc độ, mục tiêu, PWM, trạng thái RUN/DONE/FAULT).
- Giới hạn **1004 frame ~5,02 giây** × tối đa khoảng 48 byte/frame; không ghi Flash, không động chạm sector 6/7. 5 s là timeout controller hiện tại. Nếu đầy buffer `dropped>0` thì **QC FAIL**, không dùng để tune.
- **Không có UART1 PA9/PA10.** BLE JDY-33 PA2/PA3; các mẫu 200 Hz được giữ hoàn toàn trong RAM, **không truyền BLE trong ISR**.
- Sau khi khóa motor, xuất log `W,END/W,POST/W,TRACE_END` cũ trước; sau đó xuất `MW,BEGIN`, 200 Hz `MW,<data>`, `MW,END` kèm `n`, `dropped`, **FNV32**. Bộ phát gửi tối đa **20 byte/125 ms** (~160 B/s UART→BLE giả định, không phải goodput đã chứng minh). Do đó 1 lượt 720mm có thể cần **vài phút** để dump toàn bộ. Giữ điện, BLE terminal và trình duyệt kết nối đến đúng `MW,END`.
- **PA0** bấm–thả để START; **nhấn PA0 trong khi chạy** dừng khẩn; reset mới được bắt đầu lại. Dừng vì lỗi, encoder, IMU, IR timestamp quá cũ, timeout không thay đổi. Không có điều kiện dừng vì ADC cao/thấp hoặc thay đổi nhanh.

## Lệnh kiểm tra trước buổi test

```powershell
git pull --ff-only origin giomuadongbac
python tests/regression/wall_pid_6ir_report_test.py
python -m platformio run -e wall_pid_4cell_96m
```

**Chạy robot trên sàn chỉ sau khi kiểm tra cơ cấu và PA0 trên gá nâng bánh.** Lượt chạy trước từng ghi nhận robot trôi thêm **80,2 mm** sau khóa motor, và chưa có phép nghiệm thu phanh thực tế. Không cấp VM khi chưa có công tắc cắt độc lập, vùng chạy 720mm + khoảng hãm dư, không có tường chắn sát cuối đường. PWM cap 80% không có nghĩa robot an toàn ở tốc độ cao. Không tăng tốc/giới hạn từ bài log này.

Sau khi kiểm tra cơ khí và vùng thử:

```powershell
python -m platformio run -e wall_pid_4cell_96m -t upload
```

Kết nối [Web BLE terminal](../../web/ble_debug_console.html) **trước khi bấm PA0**; đợi `W,READY`. Đặt cân giữa hai tường, PA0 press-release, kết thúc cần **cả `W,TRACE_END` và `MW,END`**. Nút Export terminal cho file `wall_4cell_run01.txt`.

## Phân tích và trực quan hóa

```powershell
python tools/wall_pid_ble_report.py --input wall_4cell_run01.txt
python tools/wall_pid_6ir_report.py --input wall_4cell_run01.txt
```

Tool hai tạo:

```text
wall_4cell_run01_wall_6ir/
├── full_trace_200hz.csv
├── full_trace_qc.json
└── viewer.html
```

`viewer.html` có đồ thị tương tác: **6 IR EMA, encoder/quãng đường theo cell 180mm, gyro, tuổi mẫu IR, tốc độ thực/tốc độ đặt, PWM trái/phải, sai số Wall PID không đơn vị** và con trỏ xem mẫu. Nếu `combined_complete=false` hoặc `complete=false`: xem `faults`, lỗi đứt gói và `MW,END`; không dùng CSV không đầy đủ để tuning.

Tool edge / fusion / pose cũ **nhận được CSV 200 Hz mới**:

```powershell
python tools/maze_wall_observer.py edges --trace wall_4cell_run01_wall_6ir/full_trace_200hz.csv
```

Fusion cần model huấn luyện trên dữ liệu gán nhãn **độc lập**; không tự suy ra đã thấy tường từ một biên độ ADC. Thông số sensor-to-robot/cell cần đo riêng để quy chiếu L45/R45 và L90/R90 vào ô.

## Định dạng frame `MW`

```text
MW,BEGIN,n=300,hz=200,locked=1,motor_run=1,dropped=0,format=v1
MW,0,5,scan,encL,encR,yaw_cdeg,gyro_cdps,ir_age_us,valid,
  L90,L45,L0,R0,R45,R90,error_norm_1000,speed_mm_s,target_mm_s,
  pwmL_permille,pwmR_permille,state
...
MW,END,n=300,fnv32=12345678,dropped=0,locked=1
```

Ví dụ n/hash trên chỉ là **minh họa**, không phải log thực. **Không ngắt dòng một frame thành nhiều dòng** khi copy; BLE notify có thể chia packet tùy ý nhưng phải ghép thành dòng hoàn chỉnh trong terminal.

**Giới hạn:** dù kiểm tra sequence/FNV32 có thể phát hiện mất/sai dữ liệu, vẫn phải xác minh truyền BLE thực tế với JDY-33 và khớp quãng đường với thước bên ngoài. Một số lỗi vô tuyến có thể dẫn đến `MW,END` mất; tool sẽ báo QC không đạt, không tự chấp nhận log thiếu.
