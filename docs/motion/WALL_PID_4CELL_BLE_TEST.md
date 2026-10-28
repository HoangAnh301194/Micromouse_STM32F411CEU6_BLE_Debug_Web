# Test Wall PID giữa hai tường — 4 cell (720 mm)

**Mode duy nhất:** `wall_pid_4cell_96m`  
**Trạng thái:** đã lập trình, native-test và build MCU; **chưa chạy nghiệm thu trên robot thật**.

## 1. Điều kiện thử nghiệm

- Chiều dài: **4 × 180 mm = 720 mm**, chạy một mạch không dừng mỗi cell.
- Đường kính bánh xe: **34 mm**. Encoder: **1430 xung/vòng bánh** (sau giải mã quadrature).
- Chu vi bánh: π × 34 ≈ 106,814 mm. Mỗi xung ≈ **0,0747 mm**.
  Kết thúc 720 mm tương ứng khoảng **9640 xung trung bình của hai bánh**.
- Chỉ **L0 và R0** tạo sai số ngang. Dữ liệu sau LED ON−OFF và EMA(α=0,10), cập nhật trong TIM11.
- Mẫu IR đầu vào: `IR_WallSnapshot` lấy tại 1 kHz, PID lái được thực thi **200 Hz**. Vòng tốc độ dùng encoder hai bánh. **Gyro không trộn vào điều khiển lái**, chỉ phát hiện quay lệch bất thường hoặc lỗi MPU.
- Setpoint vận tốc đỉnh: **900 mm/s**; tăng tốc giới hạn **1700 mm/s²**, giảm tốc mục tiêu **2000 mm/s²**; **PWM bị chặn ≤800‰ (80%)**. Đây là tham số *chưa tune*, không bảo đảm động cơ đạt 900 mm/s hay robot dừng chính xác.
- Servo động cơ tiến cùng chiều; không quay tại chỗ, không reverse và không bật motion 90/180.
- Cần đường thẳng giữa hai vách ít nhất **720 mm cộng khoảng hãm/trôi dự phòng**; phía cuối không có tường chắn gần. Không dùng đoạn đua có ngã rẽ hoặc khe hở.
- Đặt robot **đúng giữa hai vách bằng thước** trước khi bắt đầu. Mỗi cảm biến L0/R0 phải nhìn thấy vách bên. Dữ liệu thử 50–60 mm đều là trên vách trắng mờ, ánh sáng phòng. Nếu bề mặt/vị trí khác, cần hiệu chuẩn lại thay vì ép chạy.

### Vì sao không dùng L0−R0 thô?

Các cảm biến có đáp ứng quang học phi tuyến: ADC có thể tăng từ
vài trăm lên trên 3000 khi khoảng cách giảm từ khoảng 70 mm về 20 mm.
Hai điểm 50–60 mm không đủ để suy ra khoảng cách theo hàm tuyến tính.

**Công thức firmware hiện tại (2026-10-28):**

```text
uL = (L0_EMA - L0_start) / (L0_EMA + L0_start + 32)
uR = (R0_EMA - R0_start) / (R0_EMA + R0_start + 32)
error_norm = clamp(uL - uR, -1, +1)
steer_PWM = clamp(Kp*error_norm + Ki*I + Kd*D, -200, +200)
```

**`error_norm` không có đơn vị mm**; `32` chỉ giúp mẫu số luôn
dương, **không phải ngưỡng ADC dùng để dừng robot**. Dải
`-1..+1` là giới hạn đầu ra PID, không phải ngưỡng bảo vệ cảm biến.

Cách chuẩn hóa này giảm phụ thuộc vào mức ADC riêng của L0/R0 nhưng
**không tuyến tính hóa quang học** và chưa bảo đảm sai số tương ứng
độ lệch tâm thực tính theo mm. Nếu robot khởi động lệch tâm, nó vẫn có
thể cố duy trì vị trí lệch đó.

**Dấu điều khiển:** lệch sang trái → error dương → PWM bánh trái tăng, PWM bánh phải giảm → robot quay về phải.

## 2. Nạp firmware

Trước khi cho motor chạy, cần kiểm tra VM cutoff độc lập, chân STBY của TB6612, chiều encoder và chiều quay. Đầu tiên dựng robot trên gá **hai bánh không chạm đất**, có nút ngắt VM dễ với tới. **Không thử ngay 80% PWM trên mặt sàn** khi bộ PID chưa đo kiểm.

```powershell
git pull --ff-only origin giomuadongbac
python tests/regression/wall_pid_4cell_native_test.py
python -m platformio run -e wall_pid_4cell_96m
python -m platformio run -e wall_pid_4cell_96m -t upload
```

Giữ robot cố định, đợi IMU hoàn tất calibration; PC13 báo sẵn sàng. Firmware chỉ có **một lượt chạy sau mỗi lần reset**; không tự lặp và không nhận lệnh bắt đầu từ BLE RX.

**Nên tiến hành tuần tự:** gá nâng bánh (đảo chiều/counter/motor-lock) → thử trên mặt sàn với trần PWM giảm và khu vực trống → chạy hành lang 720 mm. Nếu không có cutoff VM độc lập hoặc không thể bố trí đường thoát an toàn, dừng ở test trên gá. Tham số build hiện tại vẫn là cap 80%; muốn giảm PWM cho lượt đầu, sửa `WP_PWM_CAP_PM` trong `include/wall_pid_test_core.h`, build/nạp lại và xác minh giới hạn trước khi cấp nguồn motor. Không tăng PWM cap khi chưa có số liệu.

## 3. Điều khiển PA0 và LED PC13

| Khi nào | PA0 | Phản hồi |
|---|---|---|
| Khởi động, đang calibration | Không bấm | Đợi `W,READY` trong BLE terminal |
| Đủ cảm biến/IMU, robot đặt cân giữa 2 tường | **Bấm rồi thả ≥40 ms** | START, chạy liên tục đến 720 mm theo encoder |
| **Đang chạy** | **Nhấn PA0 ngay** | ISR TIM11 khóa motor, không cần chờ thả |
| Sau DONE/FAULT | Không khởi động lại | Đọc toàn bộ log BLE; reset STM32 để bắt đầu lượt mới |

PC13 nháy chậm khi ready, nháy nhanh khi chạy, sáng liên tục sau khi dừng. Nếu không có `W,READY`, kiểm tra IMU calibration, IR EMA warmup, MPU timestamp và pin PA0; không bỏ qua điều kiện an toàn.

## 4. Log qua BLE USART2 PA2/PA3

Chỉ dùng JDY-33 BLE TX; **không dùng PA9/PA10 (UART1)**. Mở terminal Web Bluetooth đang có của repo hoặc BLE terminal nhận notify, ghép các đoạn notification theo thứ tự trước khi giải mã thành dòng text. Kết nối terminal **trước khi bấm PA0**.

Firmware phát HELLO/READY/START. Khi chạy, lưu khoảng **20 mẫu/s vào RAM** để không làm nghẽn BLE và không can thiệp vào TIM11.

Sau khi dừng, firmware phát `W,END` và bắt đầu phát lại các dòng `W,<csv>` với nhịp giới hạn. Sau 800 ms, firmware phát thêm `W,POST` đo độ trôi (**có thể xen giữa những dòng trace**); cuối cùng phát `W,TRACE_END`. Mỗi chunk **tối đa 20 byte, cách 125 ms**, tức 160 byte/s từ STM32 sang USART2. Tốc độ này là giới hạn thử nghiệm bảo thủ, **không phải BLE goodput đã được xác nhận**.

Ví dụ dạng log (giá trị chỉ minh họa, không phải số đo thực):

```text
W,READY,l=460,r=664,PA0=PRESS_RELEASE
W,START,mm=720,diam=34,cpr=1430,pwmcap=800,error=REL_NORM,l=460,r=664
W,END,DONE,ms=1400,d10=7203,avgen1000=35,peakn1000=100,vpk=912,pkpwm=690,locked=1,trace=28
W,POST,dist10=7260,drift10=57,locked=1
W,50,8,461,660,6,31,85,117,117
...
W,TRACE_END
```

Một dòng `W,` số gồm `elapsed_ms,distance_0.1mm,L0_ADC,R0_ADC,error_norm_0.001,speed_mm_s,target_mm_s,PWM_L_permille,PWM_R_permille`. Trường `error_norm_0.001` là sai số quang học không đơn vị được nhân 1000, **không phải mm**. Dòng `W,END` nêu kết quả tổng hợp, `W,POST` là quãng đường thực theo encoder sau 800 ms trôi (không có độc lập đo bằng thước).

**Không coi log qua BLE thành công chỉ vì TX IRQ không báo lỗi.** Nếu mất dòng, thiếu `W,TRACE_END`, sai thứ tự hoặc `trace` trong END khác số dòng thực, log không đạt điều kiện dùng để tune.

## 5. Tự dừng và fault guards

- PA0 STOP: ưu tiên cao nhất, thăm dò 1 kHz trong TIM11.
- Lỗi IR: thiếu EMA hợp lệ hoặc mẫu quá 3 ms. **Không có ngưỡng cắt vì biên độ L0/R0.**
- Lỗi gyro: mẫu không hợp lệ/quá 3 ms, góc lệch khởi điểm quá 15°.
- Lỗi encoder: đếm lùi quá giới hạn, mất di chuyển kéo dài, chênh lệch hai bánh quá lớn.
- Tốc độ encoder > 1500 mm/s, timeout 5 giây.
- Hoàn thành 720 mm theo encoder: khóa motor tức thì; bắt buộc xem thêm quãng đường trôi trong `W,POST`.

`Motor_LockOutputs()` là lệnh dừng điện của driver, **không bảo đảm phanh cơ học lập tức**. Không hướng robot về một bức tường cuối hành lang gần 720 mm.

## 6. Các chỉ số bắt buộc trước khi tăng tốc/tune

Chụp đủ log và đánh giá ít nhất 3 lượt trên hành lang an toàn: `reason=DONE`, chiều motor/counter đúng, trung bình và P95 trị tuyệt đối sai số ngang, lệch đỉnh, vận tốc đạt, PWM hai bánh, dao động lặp lại, trôi sau dừng, và dữ liệu BLE đầy đủ.

Giai đoạn này **chưa phải nghiệm thu high-speed Wall PID**. EMA0.10 vẫn thiếu đo độ trễ cạnh thực; hệ số đổi ADC→mm là ước lượng cục bộ; khi đạt tốc độ lớn, độ trễ cảm biến có thể gây lắc hoặc va chạm. Không chỉnh gain từ một lần chạy hoặc coi `DONE` là thành công về độ chính xác.

### Chuyển BLE terminal log thành báo cáo

Dùng nút **Export** trên `web/ble_debug_console.html`, lưu các dòng
`W,...` thành file văn bản, ví dụ `wall_4cell_ble.txt`. Dòng
nhận BLE phải đã được ghép theo ký tự xuống dòng; không dùng text
rời của từng notification.

```powershell
python tools/wall_pid_ble_report.py --input wall_4cell_ble.txt
```

Tool tạo `wall_4cell_ble_report/trace.csv` và `summary.json`.
Nếu `data_complete=false`, không dùng trace đó để tuning PID.
Hệ số EMA 0,10 và hệ số nhạy L0/R0 phải được giữ nguyên giữa các
lượt để so sánh đúng.

## Phân tích thực nghiệm 28/10/2026 — dừng bất thường sau ~1 cell

Ảnh terminal do người vận hành cung cấp:

```text
W,START,mm=720,diam=34,cpr=1430,pwmcap=800,l=669,r=750
W,END,IR_EXCURSION,ms=440,d10=705,avgerr100=49,peakerr100=573,vpk=319,pkpwm=638,locked=1,trace=8
W,POST,dist10=1508,drift10=802,locked=1
W,TRACE_END
```

**Kết luận được chứng minh:** firmware chưa hoàn tất 4 cell;
thời điểm dừng chỉ 70,5 mm (đếm encoder); sau 800 ms tăng
đến 150,8 mm, tức **80,2 mm** tiếp tục di chuyển. Tốc độ đỉnh
319 mm/s, thấp hơn tốc độ đặt 900 mm/s. Việc ngắt là
`IR_EXCURSION`, không phải `DONE`. Độ chính xác mm theo encoder
vẫn tùy thuộc đường kính thực tế, trượt bánh, độ phân giải giải mã.

**Chưa xác định nguyên nhân ngắt IR.** Trong 8 khung đã thấy,
L0 ≈650–669, R0 ≈729–761; không có khung đúng tại ms440.
Điều kiện ngắt trước đó là `|L0-L0_start|>165 ADC` hoặc
`|R0-R0_start|>510 ADC`. Đối với lần này, L0_start=669,
R0_start=750, nên các mốc là **L0 < 504 hoặc >834;
R0 <240 hoặc >1260**. Một khung nhiễu, tường bị mất mép,
căn chỉnh hình học và hiện tượng nhiễu điện đều có thể giải thích;
không có căn cứ để chọn một nguyên nhân duy nhất.

**Sửa phần mềm sau phân tích:**

1. `src/wall_pid_test.c` gửi thêm `W,FAULT` sau khi dừng,
   chứa **EMA + RAW L0/R0 ở đúng scan fault**, hai giá trị tham chiếu,
   tuổi mẫu và số thứ tự scan. Việc log vẫn diễn ra từ main, không
   gửi BLE trong TIM11.
2. `src/wall_pid_test_core.c` bỏ cách **luôn dành sẵn 200‰**
   cho lái, làm PWM nền bị chặn 600‰ dù lái=0. Giờ
   **chỉ dành biên PWM bằng |steer| đang yêu cầu**,
   tổng PWM vẫn chặn 800‰, điều khiển lái đối xứng.
3. **Không thay đổi ngưỡng ngắt IR**, không thay đổi mục tiêu
   tốc độ 900 mm/s, không giảm kiểm tra motor/gyro/encoder,
   không tự động thử lại khi lỗi.
4. Thêm kiểm thử regression rằng không còn chặn 600‰ khi sai
   số lái=0, chụp đúng khung fault và nhận dạng cảm biến lỗi
   khi phân tích `tools/wall_pid_ble_report.py`.

**Quan trọng về an toàn:** do firmware mới có thể sử dụng
PWM nền cao hơn trước, phải xác minh trên gá nâng bánh, với
VM cutoff độc lập, trước mọi lượt chạy trên mặt sàn.
Không cho phép robot đâm vào tường cuối hành lang:
`Motor_LockOutputs` sử dụng coast, tức dừng cấp lực chứ
**không phanh chủ động**. Quãng trôi 80,2 mm là dữ liệu
ở khoảng 319 mm/s, **không ngoại suy tuyến tính** thành quãng
trôi ở 900 mm/s. Cần đo riêng mới thiết kế dừng dự đoán/phanh.

### Kế hoạch thử lại có kiểm soát

- **Bước A:** cắt VM, kiểm tra giá trị L0/R0 tĩnh trước/sau di chuyển
  tường; giá trị khởi điểm hiện tại (L0=669 R0=750) không khớp
  với phép đo hai bên 60 mm trước đó (khoảng 460/664 ADC);
  không suy luận rằng robot đã cân giữa hai tường chỉ từ ADC.
- **Bước B:** nâng bánh trên giá, nạp firmware, xác minh chiều
  encoder, giới hạn PWM, dừng PA0 và `W,FAULT`.
- **Bước C:** thử có giám sát trong hành lang dài hơn 720 mm
  cộng vùng thoát/dừng đủ rộng, ngắt VM độc lập; không có vật
  cản cuối hành lang. Không nhảy thẳng sang tốc độ cao
  khi 80,2 mm trôi chưa được làm rõ.
- **Bước D:** cung cấp `W,END`, `W,FAULT`, `W,POST`,
  toàn bộ trace sau `W,TRACE_END` và quãng đường đo bằng thước.
  Xác định L0/R0 nào vượt ngưỡng trước khi quyết định lọc
  glitch, nới ngưỡng, hoặc hiệu chuẩn lại.

## Thay đổi 28/10: Bỏ ngưỡng biên độ IR theo yêu cầu

Đã loại khỏi `WP_Start()` và `WP_Step()` tất cả các điều kiện
giới hạn ADC tùy ý, bao gồm:

- `L0 < 290 / > 750`, `R0 < 390 / > 1300` lúc khởi động.
- `L0 < 145 / > 1500`, `R0 < 200 / > 2100` (`WALL_LOST`).
- `abs(L0 - L0_start) > 165`,
  `abs(R0 - R0_start) > 510` (`IR_EXCURSION`).

Các nhãn lỗi cũ vẫn có thể xuất hiện trong **báo cáo lịch sử**, nhưng
không còn nhánh code chủ động dừng vì ADC cao/thấp hoặc biến thiên lớn
trong controller mới.

**Giữ:** dừng khẩn PA0, giới hạn góc IMU, dữ liệu IR/MPU quá hạn,
bảo vệ encoder/tốc độ, timeout, PWM cap và khóa motor sau khi hoàn thành.
Khi mất tường thật, firmware hiện tại **không thể bảo đảm phát hiện mất tường chỉ từ biên độ ADC**.
Đừng thử trên hành lang có khe hở/ngã rẽ/đoạn cuối chắn gần; phải có
ngắt VM độc lập. 80,2 mm trôi sau khóa motor ở lượt chạy trước
là một rủi ro vật lý chưa được giải quyết bằng sửa bộ lọc.

Bộ gain mới sử dụng `error_norm` không đơn vị và **chưa được hiệu chỉnh
trên robot thật**. Những báo cáo cũ dùng `avgerr100`, các báo cáo
mới dùng `avgen1000` và `peakn1000`; không so sánh trực tiếp hai loại.
`tools/wall_pid_ble_report.py` tự nhận dạng bản log cũ/mới.
