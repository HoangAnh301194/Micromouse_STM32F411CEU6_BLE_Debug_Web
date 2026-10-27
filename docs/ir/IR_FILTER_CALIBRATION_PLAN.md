# IR 6 kênh — thu dữ liệu, chọn bộ lọc và nghiệm thu sơ bộ

**Phiên bản chuẩn bị:** `ir_filter_diag_96m` (IR_FILTER_DIAG), motor locked
bằng firmware; **ngắt VM vật lý** khi đo. Dữ liệu thực tế mới
là bằng chứng quyết định bộ lọc nào thắng. Module
`src/ir_sensor.c` gốc được giữ nguyên định dạng dữ liệu đầu ra.

## 0. Kiến trúc đã triển khai

```text
TIM10 + ADC1/DMA
 └── OFF ambient ADC (6 kênh)
 └── PAIR1, PAIR2, PAIR3 LED ON ADC
 └── Diff_i = max(ON_i - OFF_i, 0)  [0..4095]
          ├── RAW (không lọc)
          ├── Median-3
          ├── EMA alpha=0.10
          ├── EMA alpha=0.25
          └── Median-3 → EMA alpha=0.25
                  |
          tools/ir_filter_pipeline.py
          ├── IRF: 40Hz, RAW + 4 candidate output frames
          ├── IRB:  5Hz, OFF/ON paired at same scan
          └── IRH:  1Hz, lỗi DMA/UART, motor lock
```

**Quan trọng:** các bộ lọc được cập nhật ở mỗi *scan IR mới*
(trên board, xấp xỉ 1kHz), trong khi UART chỉ **lấy mẫu
xuất log ở 40Hz**. Không nhầm dữ liệu log xuống mẫu với tốc độ
điều khiển; tốc độ scan thực phải tính từ `IRH.scans` /
thời gian đo. Filter mặc định dùng EMA 0.10 hoặc 0.25
là *phương án nghiên cứu*, chưa phải kết luận đã chọn.

Thứ tự mảng 6 kênh: `L90,L45,L0,R0,R45,R90`.
Trong module phần cứng hiện tại:
- `L0,R0` nằm về phía trước, nhìn sang tường trái/phải:
  ưu tiên sai lệch tâm hành lang `e_wall`.
- `L45,R45` nhìn chéo: dự đoán góc vào tường, góc cua,
  phát hiện khoảng trống, cạnh vách.
- `L90,R90` nằm phía sau, nhìn **về phía trước** theo
  bố trí thực tế của robot: xác định tường trước và giới
  hạn an toàn; **không giả sử** chúng nhìn ngang.
  Khoảng cách phải đo từ mặt cảm biến, không đo từ mũi xe.

## 1. Kiểm tra pipeline an toàn (P0)

Không cắm nguồn VM trong buổi đo. Kiểm tra UART PA9/PA10 TTL3.3V
chung GND, ST-Link nạp firmware, nguồn 3.3V hợp lệ.
Không khởi tạo OLED, BLE và MPU trong `IR_FILTER_DIAG`.
Chạy:

```powershell
git pull --ff-only origin giomuadongbac
python tests/regression/ir_filter_native_test.py
python tools/test_ir_filter_pipeline.py
python -m platformio run -e ir_filter_diag_96m -t upload
```

Firmware cần in:
`IR_FILTER,HELLO,ir_filter_v1_motor_locked,...`.

Bắt đầu thu:
```powershell
python tools/ir_filter_pipeline.py --port COM7 --seconds 20 --scenario static --distance-mm 20 --surface white_matte --light room --trial 1 --plot
```

Kết quả tự động ở `results/ir_filter_YYYYMMDD_HHMMSS/`:
- `raw_uart.log`: UART đầy đủ;
- `ir_filter_trace.csv`: mỗi dòng một kênh ở cùng bản ghi IRF
  (các cột raw, median3, EMA10, EMA25, hybrid);
- `ir_on_off.csv`: OFF/ON trong **cùng** scan IRB, không
  trộn tùy tiện với frame IRF khác thời điểm;
- `summary.json`: tiêu chí dữ liệu, mean/std/MAD/range;
- `filter_L90.png` v.v. (6 hình nếu có matplotlib).

Không ghi dữ liệu nếu frame lỗi, CSV/JSON ghi rõ chất lượng.
Ví dụ replay không cần board:
```powershell
python tools/ir_filter_pipeline.py --input results/ir_filter_YYYYMMDD_HHMMSS/raw_uart.log --scenario static --trial 1 --plot
```
`--input` tạo thư mục phân tích mới, không ghi đè.

## 2. Thí nghiệm tĩnh theo khoảng cách (P1)

Giữ robot bất động, đứng yên ít nhất 2 giây sau khi thay
vị trí tường rồi mới ghi. Khoảng cách đo từ **cửa sổ
thu-phát** tới mặt chuẩn trên từng trục quang, dùng thước
hoặc gá cố định, không suy khoảng cách từ ADC.

Quét đề xuất: 10, 15, 20, 25, 30, 40, 50, 60, 80 mm
(nếu kênh đã mất tín hiệu từ sớm, ngừng tăng và đánh dấu
`out-of-range`). Mỗi cự ly đo 3 lần độc lập × 20 giây,
đổi vị trí rồi đặt lại để đo độ lặp giữa lần.

Đối với L0/R0: giữ vách vuông góc hướng nhìn sang ngang
và thay khoảng cách hai bên riêng biệt. Đối với L45/R45:
dùng đúng bề mặt/vị trí vách ở hình học chéo thực tế,
ghi hướng vách và cự ly vuông góc; không áp một LUT ngang
cho tia chéo. Đối với L90/R90: đặt vách trước ở khoảng cách
đo từ vị trí các cảm biến phía sau.

Đo đủ bối cảnh: không có tường; vách trắng mờ (chuẩn),
vách đen hoặc hấp thụ, vật liệu bóng; ánh sáng phòng/bật
đèn mạnh. Nếu có ánh sáng mặt trời trực tiếp, đo riêng và
đánh dấu bối cảnh — giá trị ADC chạm 4095 là bão hòa,
không dùng nội suy cự ly.

Nên đặt tên metadata có ý nghĩa; mỗi `--port` chạy tạo
một thư mục riêng, không ghi đè kết quả cũ.

## 3. Kiểm chứng nhiễu lọc (P2)

Ở từng kênh và từng cự ly, so sánh:
- `std_adc`: độ lệch chuẩn trong 20s tĩnh;
- `mad_adc`: trung vị độ lệch tuyệt đối, bền hơn với đột biến;
- `range_adc`: dao động đỉnh-đỉnh;
- trung bình ON, OFF, và DIFF để xác định ảnh hưởng đèn nền;
- thứ tự tăng/giảm ADC theo khoảng cách, vùng nhạy, độ bão hòa;
- mức độ thay đổi kết quả giữa 3 lần đo và bề mặt khác nhau.

**Khuyến nghị làm baseline (chưa nghiệm thu):**
`median3 → EMA(alpha=0.25)`, vì median3 xử lý spike đơn mẫu,
EMA giảm nhiễu nền. Giữ `EMA(alpha=0.25)` làm đối chứng
ít trễ hơn median+EMA, `EMA(alpha=0.10)` là đối chứng
mượt nhưng có nguy cơ trễ lớn khi robot chạy nhanh.
Đừng gộp mặc định 6 kênh vào một hệ số nếu số liệu
chứng minh độ nhiễu/đáp ứng khác nhau.

## 4. Thí nghiệm động và độ trễ (P3)

Đặt vách cố định 20–40mm, nhanh chóng dịch khỏi chùm tia
rồi trả lại, tối thiểu 10 lần; ghi `--scenario step`.
Lặp cho sự kiện mất vách và có vách, từng kênh.
Đo thời gian đạt 50% và 90% thay đổi, overshoot, trễ
dò cạnh và số lần phát hiện giả. Chỉ đọc dấu biến thiên
trên đồ thị `40Hz` như **ước lượng thô** (độ phân giải 25ms).
Nếu cần tối ưu cho >0.5–1m/s và độ trễ ~2–10ms,
phải bổ sung một bài log tốc độ cao/bộ bắt mốc sự kiện
trực tiếp trên MCU; không khẳng định từ đồ thị 40Hz.

Với EMA mỗi scan, trễ đáp ứng xấp xỉ
`tau = -Tscan / ln(1-alpha)`. Ở `Tscan=1ms`,
alpha=0.10 cho tau~9.5ms; alpha=0.25 cho tau~3.5ms.
Median3 có thêm độ trễ phát hiện bước cỡ 1 scan.
Đây là tính toán mô hình, không phải độ trễ đo bằng máy.

## 5. Đánh giá chéo và kiểm chuẩn hình học (P4)

- Cho một vách trái/phải: kênh đối diện không được tăng
  bất thường quá mức nhiễu nền đã đo.
- So sánh L0/R0 khi robot lệch trái/phải 5mm:
  dấu sai số `e_wall` phải đúng; test gương trái/phải.
- Ở 45°, xoay thân robot ±5° và ±10°: ghi đặc trưng
  thay đổi do góc tới của tia; không nhầm thành khoảng cách.
- Cặp L90/R90: kiểm tra độ lệch ước lượng khoảng cách do
  vị trí lắp đặt và góc vách trước.
- Chọn LUT riêng cho từng kênh dựa trên cặp
  `(distance_mm, filtered_adc)`; nội suy ngược chỉ trong
  đoạn đơn điệu có SNR tốt, không ngoại suy và không
  giả định đường cong vốn tuyến tính.

## 6. Tiêu chí nghiệm thu trước Wall PID

1. Dữ liệu đủ 6 kênh, 3 lượt mỗi cự ly; metadata rõ ràng.
2. `IRH.locked=1` ở mọi snapshot; lỗi ADC/DMA=0;
   UART không mất bản ghi, scan rate không suy giảm quá
   mức dự kiến; 40Hz log đủ cho kiểm tra tĩnh.
3. Không lựa chọn bộ lọc chỉ vì STD nhỏ; phải chứng minh
   độ trễ phát hiện mép tường là chấp nhận được với
   vận tốc mục tiêu.
4. Khoảng đo hiệu dụng và LUT phân kênh, kiểm tra
   vật liệu, góc vách và nền ánh sáng.
5. **Chưa** cho bộ lọc mới đi vào `IR_GetLatest()`
   trong production trước khi kiểm chứng, tránh thay đổi
   giao diện PID wall và motion đang nghiệm thu.
