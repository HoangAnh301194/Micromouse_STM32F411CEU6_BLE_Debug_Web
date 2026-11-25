# Safe 7-mode menu and passive System Test over Classic SPP

**Branch:** `giomuadongbacv2`  
**Firmware thống nhất (24/11/2026):** `mode_menu_96m` là bản Build/Upload
mặc định trong VS Code. Một firmware chứa toàn bộ menu và driver encoder,
MPU, IR, motor. Các tài nguyên được cấp quyền theo chế độ **ở runtime**.
Motor **bị khóa lúc khởi động**, chỉ `Mode 7 → Encoder Test →
1 Box Forward → Start test` mới mở khóa để chạy thử có giới hạn.
`encoder_1box_96m` là alias tương thích có cùng chức năng.

## Chế độ điều khiển

- KEY PA0 vật lý: nhấn ngắn để chuyển, giữ ít nhất 400 ms để chọn; debounce 30 ms.
- Hai nút SPP **Chuyển (KEY ngắn)** và **Chọn (KEY giữ)** gửi một byte ASCII `N` hoặc `K` đến STM32. Mỗi cú nhấn gửi một sự kiện logic, không giữ phím điện tử 400 ms.
- **Start test** gửi `G`, **Stop test** gửi `P`. Trong Log Receive, hai lệnh điều khiển các phiên ghi encoder; trong 1 Box Forward, `G` khởi động **một lượt đi thẳng 165 mm**, `P` dừng. Với IR/MPU/All, chúng điều khiển luồng cảm biến. Dashboard cập nhật trạng thái theo ACK.
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
                     2. Encoder Test --> 1. Log Receive        [IDLE until START]
                                         2. 1 Box Forward       [START -> 165 mm]
                                         3. 5 Box Forward       [NO MOTION]
                                         4. Pivot 90 Left       [NO MOTION]
                                         5. Pivot 90 Right      [NO MOTION]
                                         6. Pivot 180 Left      [NO MOTION]
                                         7. Pivot 180 Right     [NO MOTION]
                                         8. PID                 [RAM tuning, MOTOR LOCKED]
                                         9. Back
                     3. MPU Test
                     4. All Sensors Test [IR+MPU+ENC; motor initialized, locked]
                     5. Back
```

Ở menu chính, chọn các mode 1–6 **chỉ phát trạng thái**, không thực hiện hành động nào.
Chọn Mode 7 để vào menu con (ban đầu chưa focus mục nào).
Nhấn Chuyển lần đầu focus IR, lần lượt Encoder, MPU, Back và quay vòng.
Nhấn Chọn để kích hoạt IR/MPU và bắt đầu truyền luồng đo thụ động. **Encoder Test bây giờ chỉ mở thêm một cấp menu**, không phát `ENC` và không tạo file CSV ngay khi chọn. Trong Encoder Test, nhấn Chuyển để duyệt `Log Receive`, sáu bài thử di chuyển và `Back`; nhấn Chọn tại `Log Receive` **chỉ vào trạng thái chờ**. Phải nhấn nút **Start test** trên giao diện SPP mới phát encoder 20 Hz và tạo một file CSV mới. Nhấn **Stop test** sẽ ngừng phát, flush và đóng file; Start lần kế tiếp tạo **file mới**. Duyệt sang mục khác sẽ ngừng truyền. `Back` của Encoder Test trở lại System Test; `Back` của System Test trở lại menu chính.

**Quan trọng:** `1 Box Forward` chạy trong firmware thống nhất `mode_menu_96m` **nhưng chỉ sau Start test**. Năm mục `5 Box Forward` và `Pivot 90/180 Left/Right` còn lại chưa điều khiển motor. `All Sensors Test` đo các cảm biến nhưng không tự chạy xe.
Trong **Encoder → Log Receive**, Stop đóng dứt điểm CSV hiện tại, Start lần sau tạo CSV khác. Stop/Start không đặt lại số đếm encoder: các timer phần cứng vẫn đếm bình thường. IR/MPU giữ cách Stop/Start trước đó (tạm dừng/tiếp tục cùng file). Khung đo còn trong hàng đợi Bluetooth có thể đến muộn trước ACK Stop; chỉ ACK `TEST STREAM STOPPED` xác nhận kết thúc ghi.

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
Encoder Test: choose Log Receive / motion tests / Back
Encoder Test option 1: Log Receive
Selected encoder option 1: Log Receive
TEST STREAM STARTED: Encoder Test
TEST STREAM STOPPED: Encoder Test
TEST STREAM STARTED: Encoder Test
TEST STREAM STOPPED: Encoder Test
Encoder Test option 2: 1 Box Forward
Selected encoder option 2: 1 Box Forward (ARMED; press Start)
Encoder Test: back to System Test
System Test 3: MPU Test
System Test 4: Back
System Test: back to main menu
```

Data log bên trái phát **20 mẫu/giây (50 ms/mẫu)** **chỉ cho bài test đang được chọn**. Mỗi lần chọn bài test mới, dashboard tự xóa bảng dữ liệu cũ và đổi tiêu đề CSV cho đúng loại cảm biến.

**Encoder Test → Log Receive — đúng ba cột CSV (`timestamp,ENC_L,ENC_R`):**

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

### Tổ chức log — một file cho mỗi lần Start/Stop (23/11/2026)

**Chỉ riêng Mode 7 → Encoder Test → Log Receive:** một thư mục cố định, mỗi lần nhấn **Start test** mở một file CSV **mới**, mỗi lần nhận ACK **Stop test** flush và đóng file. Tên file lấy từ timestamp tại thời điểm Start, không đặt timestamp vào tên thư mục:

```text
logs/spp_runs/
└── mode 7 - encoder test - log receiver/
    ├── 20261123_153340_852000_COM16.csv
    ├── 20261123_153405_123000_COM16.csv
    └── 20261123_153420_123000_COM16.csv
```

Ví dụ số liệu chỉ minh họa. Các file có đúng:

```csv
timestamp,ENC_L,ENC_R
2026-11-23T15:33:40.852,0,-5
2026-11-23T15:33:40.945,12,16
```

Quy tắc ghi log:

- **Chọn Log Receive:** chuyển sang trạng thái `IDLE/READY`. **Không có** khung `ENC`, không tạo CSV, Start test bật và Stop test tắt.
- **Start test:** gửi `G`; sau ACK `TEST STREAM STARTED: Encoder Test`, phần mềm tạo một CSV mới có tiêu đề ngay cả khi chưa có mẫu nào. Mẫu tiếp theo ghi vào file này.
- **Stop test:** gửi `P`; sau ACK `TEST STREAM STOPPED: Encoder Test`, file được flush, đóng và lưu. Dữ liệu ngoài phiên thu bị bỏ qua. Stop trước Start không tạo file.
- **Start lần tiếp theo:** tạo file khác **trong cùng thư mục**, tuyệt đối không append vào file trước. Nếu timestamp trùng, thêm hậu tố `_01`, `_02`… để không ghi đè.
- **Chuyển mode, Back hoặc Disconnect:** đóng file đang mở để giữ dữ liệu đã thu; lần chọn/Start mới cần mở phiên mới.
- **Dữ liệu encoder:** chỉ các cột `timestamp,ENC_L,ENC_R`, giữ nguyên giá trị đếm tích lũy (không tự reset). Bản tin encoder cũ dạng `left=...` vẫn chuẩn hóa được.
- **Status bên phải:** chỉ hiển thị trong GUI, không ghi file.
- **IR/MPU Test:** vẫn theo logic tự phát khi chọn, lưu `continuous_data.csv` trong thư mục riêng có timestamp; Stop/Start chỉ pause/resume trong cùng file, không đổi cấu trúc của hai bài test này.
- **Mở COM hoặc duyệt Forward/Pivot placeholder:** không tạo log. Khi reconnect, phải vào Log Receive và nhấn Start; không tự suy ra quyền thu từ khung `ENC` lẻ.

Nút **Open logs folder** mở thư mục lưu; **Replay CSV/log** đọc file đã ghi mà không tạo phiên mới. Log lịch sử dưới `logs/spp_runs/connections/` hoặc thư mục kiểu cũ được giữ nguyên, không xóa và không tự di chuyển.

Lần này sửa cả `src/mode_menu.c`, nên **cần flash lại `mode_menu_96m`** cùng với cập nhật Python. Không dùng `spp_smoke_96m` khi kiểm tra menu vì đó chỉ là firmware chẩn đoán UART.

## 1 Box Forward: opt-in closed-loop 165 mm trial (23/11/2026)

### Encoder evidence and target count

Nine new `Mode 7 / Encoder Test / Log Receive` CSV files were uploaded
in commit `7420d12`. The last file contains a mid-capture encoder reset;
its final `0,0` is **not** a valid one-cell displacement. The other
eight captures, described by the user as manual center-to-center (165 mm)
traversals, yield:

| Measurement | Left | Right |
|---|---:|---:|
| Mean count, 8 runs | 2400.375 | 2389.625 |
| Median count | 2398 | 2394.5 |
| Sample SD | 29.7 | 33.7 |
| Empirical mm/count | 0.068739 | 0.069048 |

Target counts for the initial one-shot controller:
`ONEBOX_TARGET_L=2400`, `ONEBOX_TARGET_R=2390`
(`include/encoder_onebox_core.h`). They reflect **manual roll data**,
not proof of physically accurate autonomous stopping. Verify on the
actual floor and adjust for wheel slip, braking and mechanical backlash.

### Unified runtime resource selection (one firmware)

Default Build/Upload: **`mode_menu_96m`**, which includes the encoder
one-box motion controller and all test drivers. The old
`encoder_1box_96m` environment is retained only as an alias.

| Test state | IR acquisition | MPU acquisition | Encoder | Motor |
|---|---|---|---|---|
| Idle/menu | Off | Off | Hardware counts only | Locked |
| IR Test | On | Off | Hardware counts only | Locked |
| MPU Test | Off | On; async gyro calibration starts on first use | Hardware counts only | Locked |
| Encoder → Log Receive | Off | Off | Log after Start | Locked |
| Encoder → 1 Box Forward | Off | Off | Closed-loop motion after Start | Conditional unlock only |
| Other Forward/Pivot | Off | Off | Hardware counts only | Locked |
| All Sensors Test | On | On | Combined readings | Initialized but locked |

Driver initialization is shared, but acquisition and motor permissions
are selected at runtime. Encoder timer counting and motor command
watchdog continue independently for safety. **No sensor power rails are
switched by software.** All Sensors Test emits 20-Hz combined `ALL`
CSV (wheel counts, IR, yaw/rate and validity flags) in its own folder.
All mode does not run a motor task; motor stays locked until a future
explicitly supervised controller is implemented.

Encoder one-box is now **encoder-feedback only**. No MPU or IR samples
are used during motion. Remaining safety guards are signed-count
direction, wheel divergence, stall, 6.5s deadline, PA0/Stop and 350ms
host heartbeat. Without IMU supervision there is no independent yaw
measurement or obstacle avoidance. Test with wheels lifted first.

### Continuous CSV for Encoder → 1 Box Forward

The one-firmware System Test streams **each implemented sensor test**
through the left Continuous Data pane:

- IR Test → `IR` CSV (six IR channels)
- MPU Test → `MPU` CSV (yaw and gyro-Z)
- Encoder → Log Receive → `ENC` CSV (signed total counts)
- Encoder → **1 Box Forward** → `ONEBOX_CSV` CSV (encoder + applied PWM)
- All Sensors Test → `ALL` CSV (IR, MPU, and encoder)

The five other Forward/Pivot menu options remain **unimplemented**:
they only issue one-shot status messages and do not pretend to measure
an unexecuted motor experiment.

For **1 Box Forward**, pressing Start only in the armed submenu unlocks
the motor and returns a status ACK `ONEBOX,START,...`. The MCU then sends
20-Hz, main-context telemetry (no USART inside the 1-kHz control ISR):

```text
ONEBOX_CSV,ms=0,left=100,right=150,pwm_l=0,pwm_r=0
ONEBOX_CSV,ms=50,left=146,right=195,pwm_l=150,pwm_r=148
ONEBOX_CSV,ms=100,left=210,right=260,pwm_l=185,pwm_r=180
ONEBOX_CSV,ms=...,left=...,right=...,pwm_l=0,pwm_r=0
ONEBOX,RESULT,reason=DONE,L=...,R=...,ms=...,locked=1
```

Example numbers above are **illustrative**, not measured on the robot.
The Python application converts the sensor frames into:

```csv
timestamp,t_ms,ENC_L,ENC_R,PWM_L,PWM_R
2026-11-24T14:01:02.000,0,100,150,0,0
2026-11-24T14:01:02.050,50,146,195,150,148
2026-11-24T14:01:02.100,100,210,260,185,180
```

- `timestamp`: Windows host-receive ISO 8601 time.
- `t_ms`: MCU elapsed milliseconds since successful Start.
- `ENC_L`, `ENC_R`: **absolute signed cumulative encoder counts**,
  same semantics as Encoder Log Receive; they do **not** reset at Start.
  For travel counts, subtract the first value in each run.
- `PWM_L`, `PWM_R`: **actually applied signed motor PWM**, units
  per mille of full scale (`200` = 20% duty), read from the driver.
  These are not PID targets and may lag commands due to slew/deadband.
- A final zero-PWM sample is emitted **before** `ONEBOX,RESULT`, so
  normal completion, Stop, timeout and encoder faults all close a file.
- The first `ONEBOX,START` creates a new timestamp-named file.
  `ONEBOX,RESULT` closes/flushed it. Every subsequent Start makes a
  **separate file** in the stable directory:

```text
logs/spp_runs/
└── mode 7 - encoder test - 1 box forward/
    ├── 20261124_140102_000000_COM16.csv
    └── 20261124_140213_000000_COM16.csv
```

The `ONEBOX,START/RESULT/REJECT` status messages remain visible only
in the right terminal; they are **not** appended to continuous-data CSV.
Other test CSV formats and the separate Encoder Log Receive folder are
preserved. No new motor control behavior or new PWM setting is introduced
by this logging change.

### Start/Stop and safeguards

1. Enter `1 Box Forward`. Status says `ARMED; press Start`.
   **No motor command is issued by selection.**
2. In Encoder → 1 Box Forward press **Start test**. STM32 checks selected
   mode, clock, TIM11 scheduler, PA0 and motor watchdog; **MPU is not required**.
3. STM32 emits `ONEBOX,START,target_l=2400,target_r=2390,pwm_max=200`
   and performs one encoder feedback-controlled forward trial.
   Motor PWM is capped at **200/1000 = 20%**, with tapered PWM and
   a wheel-count imbalance correction (not full heading PID).
4. GUI sends one-byte `H` heartbeats at about 100ms intervals only
   while onebox motion is requested. **350 ms without the lease**, a
   physical PA0 press, GUI Stop (`P`), wrong-sign encoder feedback,
   wheel divergence, no progress for 900ms, or a run lasting >6.5 s forces a motor stop.
   A severe fault locks the driver until MCU reset.
5. At the encoder target, the MCU brakes for 100ms then coasts.
   Result is a status-only event, e.g.
   `ONEBOX,RESULT,reason=DONE,L=2400,R=2390,ms=...,locked=0`.
   This event is **not** written into the Log Receive CSV.
6. Stop test interrupts motion immediately; if the USB/Bluetooth link
   fails, the lease timeout is an independent fallback. When the MCU
   itself fails completely, the motor watchdog may also fail: keep a
   **physical power cutoff** available.

**Acceptance sequence:** start with the wheels physically raised and a
safe 2S/VM cutoff, and verify forward sign, wheel directions, a STOP
command, PA0 emergency stop and HOST_LOST fallback. Then test on an
unobstructed floor at low risk; measure the actual stopping distance
and final position relative to the cell center. Only after that can
the count target and PWM/braking profile be refined. No autonomous
accuracy or obstacle avoidance is claimed from the software build.

```powershell
git fetch origin
git switch giomuadongbacv2
git pull --ff-only origin giomuadongbacv2

# One firmware for all tests; default VS Code Upload:
pio run
pio run -t upload
py tools/classic_spp_dashboard.py --port COM16
```

Test coverage: `tests/regression/encoder_onebox_native_test.py`
checks done/timeout, reverse counts, wheel imbalance, no-progress and
host-lease expiry in pure C. CI also builds `encoder_1box_96m`.
Physical safety/accuracy still requires a supervised bench test.

## SPP boot recovery and sensor reset controls (23/11/2026)

### What changed for the recurring COM-open / 0-lines fault

Previous `mode_menu_96m` returned from `MPU6050_Init` on failure and
entered an infinite loop **before** the main-context SPP handshake reader.
Consequently Windows could open its Bluetooth COM while Python never got
`SPP CONNECTED`. This is an identified code-level failure mode, not proof
that MPU failure caused every observed COM16 outage.

The motor-locked menu firmware now continues into its passive diagnostic
loop when MPU initialization/calibration startup fails:
- `?` always responds with `SPP CONNECTED`, then `BOOT,READY,MPU=OK`
  or `BOOT,DEGRADED,MPU_STATUS=...,STEP=...,I2C=...`.
- The Encoder/IR test scheduler remains operational without a valid MPU;
  MPU Test reports `valid=0` and yaw reset is refused.
- The gyro main-context I2C update is skipped when the MPU did not
  initialize. Motion outputs stay latched locked; the passive TIM11
  sensor ISR **does not call motion-update/fault handlers** in this profile.
- OLED boot-time I2C probing is **off by default** in `mode_menu_96m`.
  This is intentional because that optional peripheral has not been
  validated on the robot and must not interfere with the SPP/gyro path.
  An explicitly opted-in build with `MODE_MENU_ENABLE_OLED` restores
  the OLED path once hardware/bus reliability has been established.
- Python restores `link.flush()` on handshake probes, shows **RX bytes**
  separately from parsed lines, and provides **Reconnect COM** to reopen
  the SPP COM without restarting the dashboard. This is a user-triggered
  recovery; repeated automatic COM reopen is deliberately avoided.
- **Limit:** a board clock fault occurring before `BT_Init()`, a stuck
  I2C call that never returns, broken USART wiring, or an incorrect
  Windows COM cannot be repaired by this protocol. Confirm with hardware
  status and the new RX-byte counter.

### Sensor reset buttons

The following commands are one-byte ASCII on USART2; they do not
control motors and are accepted only for the currently selected test:

| Button | Byte | Guard | Effect | Status acknowledgment |
|---|---|---|---|---|
| **Reset Encoder** | `E` | Encoder Test selected | Atomic `Encoder_Reset()`, zero cumulative left/right counts and speed EMA; quadrature counting continues | `ENCODER RESET OK` |
| **Zero MPU yaw** | `Y` | MPU Test selected **and MPU boot OK** | Existing `MPU6050_ResetYaw()` zeros integrated yaw only, not gyro offset/calibration | `MPU YAW RESET OK` |
| **Restart IR scan** | `I` | IR Test selected | `IR_Stop()` and `IR_StartScan()`; restart scan state, **not an IR calibration** | `IR SCAN RESTARTED` |

The previous **Start test** (`G`) and **Stop test** (`P`) still
control **sensor telemetry streaming only**, not motor motion. When a
test is selected, streaming begins automatically. Stopping does not
zero the sensor values or stop the encoder timers. All control buttons
are disabled until SPP receives `SPP CONNECTED`; reset controls are
disabled if their matching sensor test is not active.

### Hardware acceptance

1. Pull and flash **both** firmware and Python from `giomuadongbacv2`:
   `pio run -e mode_menu_96m -t upload`.
2. Select the Windows Bluetooth SPP **Outgoing** COM; open the dashboard.
   Confirm `SPP CONNECTED`, `BOOT,READY` and an RX-byte count above 0.
3. In Encoder Test, turn one wheel by hand while motor power is isolated.
   Select **Log Receive** and verify it stays idle. Press **Start test**,
   turn a wheel by hand, then **Stop test**. Confirm one timestamped CSV
   in `mode 7 - encoder test - log receiver/`. Start/Stop again and verify
   a second CSV without appending to the first. Reset Encoder if needed.
4. In MPU Test, rotate the chassis manually, stop it, press Zero MPU yaw
   and verify yaw returns near zero before drifting/integrating again.
5. In IR Test, press Restart IR scan, then verify subsequent fresh IR
   samples. Observe that the IR reset does **not** create a zero baseline.
6. If COM opens but there are no responses, compare **RX 0 bytes** with
   **RX nonzero bytes but 0 lines**. Check LED, outgoing port and loaded
   firmware; try **Reconnect COM** once. If a `BOOT,DEGRADED` message
   appears, investigate the reported MPU boot step/I2C status; encoder
   and IR can continue but MPU measurements are not valid.

Older firmware does not implement the `E/Y/I` commands or degraded
startup, so **updating Python alone will not resolve the MCU boot path**.

## Triaging COM open / RX 0 bytes: isolated SPP smoke image

**Why this test exists:** even after the firmware was modified to tolerate a
returned MPU startup error, the Windows dashboard still reported exactly
`RX 0 bytes`. That result is not enough to distinguish the physical UART link,
Windows Outgoing COM, MCU clock/boot or a sensor startup stall. Source review
also found a few unbounded DMA-disable waits in `src/i2c.c`; the degraded
error handler cannot run if an earlier I2C wait never returns. Neither
observation proves the actual cause on this user's hardware.

**Isolation image `spp_smoke_96m`:**

- Explicit opt-in firmware on the **same branch** and same 96 MHz clock,
  same `BT_Init()`, PA2/PA3 USART2 and 115200 baud as `mode_menu_96m`.
- Calls `Motor_Init()` and `Motor_LockOutputs()`; motor is locked until reset.
- Does **not** initialize MPU6050, I2C, OLED, six IR channels, encoder or TIM11.
- Toggles PC13 status LED every second and emits
  `SPP_SMOKE,HB,ms=...,rx=...,fe=...,ne=...,ore=...,drop=...`
  **once per second**, whether or not the PC transmits anything.
- Replies to each ASCII `?` with `SPP CONNECTED` and
  `SPP_SMOKE,PONG,ms=...,rx=...,fe=...,ne=...,ore=...,drop=...`.
  Hardware counters identify UART framing/noise/overrun, not RF quality.
- It contains **no** mode, motor, Flash, reset-sensor or movement commands.

To run it, first **disconnect VM/motor power, support the chassis**, close
the dashboard and any serial terminal, connect ST-Link:

```powershell
git fetch origin
git switch giomuadongbacv2
git pull --ff-only origin giomuadongbacv2
pio run -e spp_smoke_96m
pio run -e spp_smoke_96m -t upload
py tools/classic_spp_smoke_probe.py --port COM16 --seconds 20
```

COM16 must be the **actual Outgoing** port; verify Windows
Bluetooth settings or `py tools/classic_spp_link_diag.py --list`.
Close Web BLE during Classic SPP isolation. The probe transmits only `?`.
In addition to printed bytes/lines/heartbeat/PONG counts, visually observe
whether PC13 blinks once per second.

| Probe observation | Interpretation / next step |
|---|---|
| Heartbeats **and** PONG | Both UART directions and COM transport pass in isolation. If normal firmware still has `RX 0`, suspect a startup stall, sensor/I2C interaction or code specific to normal runtime. |
| Heartbeats but **no** PONG | STM32 TX -> PC works. Investigate STM32 PA3 RX, JDY TXD and `fe/ne/ore` counter trends. |
| PC13 blinks, `RX 0 bytes` | Firmware executes; investigate STM32 PA2 TX -> JDY RXD, shared ground, JDY UART baud, Windows Outgoing COM and concurrent BLE. |
| PC13 stays still, `RX 0 bytes` | Investigate ST-Link flashing, MCU boot/clock and independent power. This is not proof of Windows Bluetooth fault. |
| Partial/corrupt RX | Check baud/framing/clock and JDY/STM32 UART wiring. |
| Cannot open COM | Confirm COM Outgoing and that no other process has it open. |

After the diagnosis, **restore the normal menu image** and only then use
the dashboard Encoder/IR/MPU menus:

```powershell
pio run -e mode_menu_96m -t upload
py tools/classic_spp_dashboard.py --port COM16
```

The smoke test is a **hardware-isolation aid, not a proven fix**. Preserve
the printed results, especially heartbeat count, PONG count and PC13
status before making any further firmware or baud changes. Do not
change the JDY-33 AT baudrate as an unverified workaround.

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

Cấu hình `mode_menu_96m` vẫn khóa motor ngay khi boot, nhưng **mở khóa có điều kiện** khi Start đúng Encoder → 1 Box Forward. Các mode khác không kích hoạt chuyển động. Thử motor lần đầu phải nâng bánh khỏi sàn, có bộ ngắt nguồn VM vật lý, xác nhận chiều bánh và Stop/PA0; CI không thay thế phép thử thực tế.

**Lưu ý:** `mode_menu_96m` là firmware menu thống nhất, khác `system_test_96m` cũ. Không nạp các profile chẩn đoán cũ khi kiểm thử menu.

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

## PID vận tốc bánh (24/11/2026)
Chọn PID để GUI hiện sáu ô (P_L/I_L/D_L, P_R/I_R/D_R); bấm Áp dụng PID và chờ PID,APPLIED từ STM32. Chỉ ghi RAM khi motor khóa và đúng mode; reset MCU khôi phục mặc định. Chuyển sang 1 Box Forward, Start để chạy với setpoint cố định 1100 PPS mỗi bánh, PWM 0..200, dừng/phanh theo encoder. PWM thay đổi để bám vận tốc; không còn giảm setpoint gần đích. Tối đa bốn chữ số thập phân, P 0..1, I 0..2, D 0..0.1. Kiểm tra build và thử nâng bánh, có ngắt VM độc lập trước thử trên sàn. Chưa nghiệm thu thực địa.

CSV 1 Box Forward giữ sáu cột cũ và thêm REF_L_PPS, REF_R_PPS, SPEED_L_PPS, SPEED_R_PPS; firmware cũ để trống bốn cột mới. Tần số SPP 20 Hz, không thay đổi PID 100 Hz.

### Live wheel plots
Encoder Log Receive / 1 Box Forward divides the left terminal into top CSV and two lower wheel plots. PPS (reference, measured, error), PWM permille and relative count use separate scales; missing signals are not fabricated. OneBox uses MCU elapsed ms, passive ENC uses host receive timestamps. Start clears previous plots, Stop preserves them. CSV replay also plots without writing files. No firmware change.

### Baseline alignment (25/11/2026)
OneBox adopts P=.09 I=.25 D=0, ks=80 kv=.098, integral limit 3000 PPS*s, PID 1 kHz and 50 Hz target telemetry from giomuadongbac. Keep forward-only PWM cap 200, 1100 PPS, empirical targets, RAM tuning and existing guards. Not a full Motion merge; verify timing and hardware before acceptance.

At ONEBOX RESULT, dashboard computes source-derived per-wheel metrics from the captured samples and writes a sibling .metrics.json. PWM saturation is based on applied PWM, not raw controller output. No data means N/A; 50 Hz observations cannot verify 1 kHz timing.

### Equal one-cell targets (25/11/2026)
Both wheel targets are 2400 counts. Each wheel receives zero reference when it first reaches its target; the other continues at 1100 PPS until it reaches its own target. DONE requires both arrival flags; no average-based early completion. Driver ramp/coast can still cause overshoot, and all fault guards remain active. Hardware accuracy unverified.

### Updated encoder calibration candidate (25/11/2026)
Ten passive Log Receive CSVs 14:54:43–15:02:54, commit 3cd0777: final-minus-first mean L=2413.9, R=2427.4 counts. Shared one-cell target updated to 2414 (rounded L mean) per prior equal-target requirement, conditional on captures covering exactly 165 mm. Independent arrival, gains, PWM limit and guards unchanged. Passive data cannot establish powered stopping overshoot or speed PID performance.

### Manual MPU angle test — 25/11/2026
Mode 7 > MPU Test is the angle-only passive test, no additional submenu. Motor remains locked; selection arms and triggers calibration if required, READY status after calibrated, Zero MPU yaw then Start/Stop for independent CSV takes. CSV timestamp,mcu_ms,valid,yaw_deg,age_us; invalid samples leave yaw blank. Existing centidegree firmware frame retained for compatibility. No changes to gyro filters or PID.

### MPU activation fix (25/11/2026)
Unified firmware starts MPU calibration at boot and services its asynchronous acquisition continuously; TIM11 always supplies sample ticks. MPU Test logging remains opt-in; Start is rejected until calibrated, Start or Zero retries a failed calibration, and 1 Hz status exposes state, sample count, retries and I2C health. Encoder OneBox still uses encoder feedback only. Keep stationary during calibration. Hardware validation pending.
