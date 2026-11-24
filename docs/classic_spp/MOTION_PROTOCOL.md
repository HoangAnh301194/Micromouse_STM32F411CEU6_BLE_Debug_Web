# Giao thức Bluetooth Classic SPP cho Motion Control Test (v2)

Áp dụng cho **STM32F411CEU6 (`USART2 PA2/PA3` @ 115200 8N1 via JDY-33)** và các công cụ host (`tools/classic_spp_dashboard.py`, `tools/motion_test_runner.py`).

## 1. Quy ước chung
- **Baudrate:** `115200 8N1`.
- **Phân tách gói:** Theo dòng kết thúc bằng `\n` (chấp nhận `\r\n` hoặc `\n`).
- **Tương thích ngược (lệnh 1 ký tự không có tiền tố `$`):**
  - `F` / `f`: Chạy thẳng 1 ô mê cung (`165.0 mm`).
  - `T` / `t`: Chạy thẳng `2500.0 mm`.
  - `Q` / `q`: Chạy `Quick Step Test` mức `LOW`.
  - `L` / `l`: Quay trái `+90.0°`.
  - `R` / `r`: Quay phải `-90.0°`.
  - `S` / `s` / `!` / `\x18`: **Dừng khẩn cấp (Emergency STOP)** — ưu tiên cao nhất, cắt motor ngay lập tức.
  - `Z` / `z`: Reset Yaw MPU6050 (chỉ khi IDLE).
  - `?`: Truy vấn trạng thái nhanh (`ACK,0,PING,...`).

## 2. Lệnh có cấu trúc (Host → STM32)
Định dạng:
```text
$<cmd_id>,<VERB>[,<arg1>,<arg2>,...]\n
```
- `<cmd_id>`: Số nguyên tăng dần từ host (`1, 2, 3, ...`) để khớp phản hồi `ACK`/`NACK`.
- Tất cả tham số số thực được kiểm tra `isfinite()` (từ chối `NaN`/`Inf`) và giới hạn biên ở phía firmware.
- Các lệnh khởi chạy chuyển động (`WTEST`, `STRAIGHT`, `TURN`, `VEL`, `QSTEP`) bị từ chối với `NACK,<id>,<VERB>,BUSY` nếu `Motion_IsDone() == 0`, hoặc `NOT_READY` nếu MPU chưa hiệu chuẩn/stale, hoặc `LOCKED` nếu motor đang bị khóa.

### Bảng lệnh
| Lệnh | Tham số | Phản hồi | Ghi chú |
|---|---|---|---|
| `PING` | *(không)* | `ACK,<id>,PING,READY=<0\|1>,STATE=<0..5>,CAL=<0..4>,LOCK=<0\|1>,RES=<0..4>` | Kiểm tra sẵn sàng |
| `STOP` | *(không)* | `ACK,<id>,STOP,OK` | Dừng tức thì, kết quả `ABORTED` |
| `HB` | *(không)* | `ACK,<id>,HB,OK` | Làm mới Host Watchdog (timeout 2.0 s) |
| `RESET_ENC` | *(không)* | `ACK,<id>,RESET_ENC,OK` / `NACK,...,BUSY` | Reset encoder về 0 |
| `RESET_YAW` | *(không)* | `ACK,<id>,RESET_YAW,OK` / `NACK,...,BUSY\|NOT_READY` | Reset yaw MPU về 0 |
| `GET_GAINS` | *(không)* | `GAINS,L,...`, `GAINS,R,...`, `HGAINS,...`, `ACK,<id>,GET_GAINS,OK` | Đọc cấu hình thực tế trên RAM |
| `SET_GAINS` | `<L\|R\|B>,<kp>,<ki>,<ks>,<kv>,<ka>,<ilim>,<pwm_max>` | `GAINS,L,...`, `GAINS,R,...`, `HGAINS,...`, `ACK,<id>,SET_GAINS,OK` | Chỉ lưu RAM, kiểm tra `[0..1000]` |
| `SET_HGAINS` | `<kp>,<ki>,<kd>,<max_w>` | `GAINS,...`, `HGAINS,...`, `ACK,<id>,SET_HGAINS,OK` | Cập nhật Heading PID |
| `RESET_INT` | *(không)* | `ACK,<id>,RESET_INT,OK` | Xóa tích phân PID |
| `RESTORE_GAINS` | *(không)* | `GAINS,...`, `HGAINS,...`, `ACK,<id>,RESTORE_GAINS,OK` | Khôi phục mặc định |
| `WTEST` | `<l_pps>,<r_pps>,<dur_s>,<max_pm>,<mode(0\|1)>` | `ACK,<id>,WTEST,STARTED` | `mode`: 0=Step, 1=Ramp; bật Host Watchdog |
| `STRAIGHT` | `<dist_mm>,<max_v>,<min_v>,<end_v>,<acc>,<dec>,<max_w>,<hold(0\|1)>` | `ACK,<id>,STRAIGHT,STARTED` | `hold=0` cho bench test không tải |
| `TURN` | `<ang_deg>,<max_w>,<min_w>,<acc_w>,<dec_w>,<ang_kp>,<rate_kp>,<ang_tol>,<rate_tol>,<timeout>` | `ACK,<id>,TURN,STARTED` | Quay vòng kín Gyro + `WheelSpeedPid` |
| `VEL` | `<v_mm_s>,<w_dps>,<hold(0\|1)>,<target_yaw>,<dur_s>` | `ACK,<id>,VEL,STARTED` | Chạy có giới hạn thời gian + Watchdog |
| `QSTEP` | `<level(0\|1\|2)>` | `ACK,<id>,QSTEP,STARTED` | `0=LOW`, `1=MEDIUM`, `2=HIGH` |
| `SMOOTH` | `<kind(0..7)>,<dir(-1\|1)>[,v,max_w,alpha,entry,exit,rate_kp]` | `ACK,<id>,SMOOTH,STARTED` | Cua mượt 5 pha (`SS90E..DD90`) |

## 3. Sự kiện & Telemetry (STM32 → Host)
1. **Sự kiện kết thúc Motion:**
   ```text
   EVENT,DONE,RES=<SUCCESS|TIMEOUT|ABORTED|FAULT>,DIST=<mm>,ERR=<deg>,T=<ms>
   ```
2. **Telemetry chính (50 Hz):**
   ```text
   t=<ms> state=<u> ph=<u> encL=<d> encR=<d> vL=<d> vR=<d> vRefL=<d> vRefR=<d> pL=<d> pR=<d> aL=<d> aR=<d> yaw=<f> gz=<f> d=<f> e=<f> sat=<u>
   ```
   - `pL`, `pR`: PWM yêu cầu từ bộ điều khiển PID (‰).
   - `aL`, `aR`: PWM thực tế sau ramp/deadband của `motor.c` (‰).
   - `sat`: Bitmask bão hòa (`bit 0`: Trái, `bit 1`: Phải).
3. **Telemetry mở rộng (10 Hz):**
   ```text
   EXT,t=<ms>,vR=<f>,wR=<f>,vM=<f>,wM=<f>,iL=<f>,iR=<f>,res=<u>
   ```
   - `iL`, `iR`: Giá trị bộ tích phân `WheelSpeedPid` trái/phải.
   - `vR`, `wR`, `vM`, `wM`: Vận tốc đặt và đo của chassis (`mm/s`, `deg/s`).

## 4. An toàn & Watchdog
- Các lệnh chạy từ PC (`WTEST`, `STRAIGHT`, `TURN`, `VEL`, `QSTEP`) kích hoạt **Host Watchdog (2.0 s)**. Nếu mất kết nối hoặc host ngừng gửi `HB`, firmware tự dừng motor và kết thúc với `EVENT,DONE,RES=ABORTED`.
- Khi kết nối lại, firmware giữ nguyên trạng thái `MOTION_IDLE`, không tự khởi động lại chuyển động cũ.
