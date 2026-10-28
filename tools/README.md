# Host diagnostic and qualification tools (restored)

> **Mới — JDY-33 Bluetooth Classic SPP / Windows:** [Hướng dẫn kết nối đã nghiệm thu trên COM16](../docs/classic_spp/README.md), gồm `classic_spp_dashboard.py`, `classic_spp_link_diag.py`, `classic_spp_ports.py`, `classic_spp_protocol.py`, và `windows_bluetooth_link_debug.ps1`.


## Bluetooth Classic SPP / JDY-33 — Hướng dẫn kết nối trực tiếp (Windows)

**Đọc README đầy đủ:** [../docs/classic_spp/README.md](../docs/classic_spp/README.md).
Đây là bộ công cụ SPP trên **cùng module JDY-33** của Micromouse STM32F411CEU6,
không phải module phần cứng mới. Cấu hình đã thử trên Windows 11 ngày 28/10/2026:
**Nezumi_Chan_SPP**, **COM16 Outgoing**, **115200 baud**, **40 giây RX = 1.955 byte**,
không phát sinh lỗi nối tiếp. **COM16 không cố định ở máy khác.**

### A. Sơ đồ hệ thống và module

```text
STM32F411CEU6 -- USART2 PA2(TX)/PA3(RX) -- JDY-33
                                             |
                                             | Bluetooth Classic SPP (không dây)
                                             v
                                 Windows COM Outgoing
                                             |
                     +-----------------------+-----------------------+
                     |                                               |
      classic_spp_link_diag.py                         classic_spp_dashboard.py
       (thu log, test link)                            (Terminal / Maze / PID)
```

| File (tính từ gốc repository) | Vai trò |
|---|---|
| `src/bt_debug.c`, `include/bt_debug.h` | Driver UART USART2 115200 8N1 của STM32, giao tiếp JDY-33 |
| `src/main.c`, `platformio.ini` | Firmware đang chạy và các cấu hình build |
| `src/flash_map_diag.c`, `src/flash_map_protocol.c` | Phát map thử nghiệm `MAPCFG/MAPROW/MAPEND` khi nạp chế độ `flash_map_diag_96m` |
| **`tools/classic_spp_dashboard.py`** | **Giao diện Python chính**: Terminal, bản đồ Maze, telemetry/PID, lưu và xem lại log |
| `tools/classic_spp_link_diag.py` | Liệt kê COM; ghi thời gian mở kết nối, dữ liệu RX, các lỗi serial vào JSONL |
| `tools/classic_spp_ports.py` | Hỗ trợ phân biệt cổng Windows Incoming/Outgoing; chỉ là heuristic |
| `tools/classic_spp_protocol.py` | Xử lý các dòng UART, parser telemetry và map CRC32 |
| `tools/windows_bluetooth_link_debug.ps1` | Theo dõi sự kiện Windows Bluetooth và PnP khi gặp lỗi |
| `tests/regression/classic_spp_protocol_test.py` | Kiểm thử tự động cho parser và nhận diện COM |
| `web/ble_debug_console.html` | Web BLE cũ (tên `Nezumi_Chan`), tách biệt Classic SPP |

**Cảnh báo:** GUI hiện **chỉ nhận dữ liệu (RX-only)**; việc có thể thu telemetry từ
STM32 không xác nhận đã điều khiển MCU qua SPP hay chạy đồng thời BLE + Classic.

### B. Pull code, chuẩn bị phần cứng

Trong PowerShell từ **thư mục gốc repo** (ví dụ `D:\\MICROMOUSE_26\\code_mau`):

```powershell
cd D:\MICROMOUSE_26\code_mau
git switch giomuadongbac
git pull --ff-only origin giomuadongbac
python -m pip install pyserial
```

1. Cấp nguồn cho Micromouse để JDY-33 hoạt động; bật Bluetooth trên Windows.
2. **Không cần cắm ST-Link, USB–TTL hoặc dây USB vào PC** cho kết nối SPP.
   ST-Link chỉ cần khi nạp firmware; USB–TTL dùng khi gửi lệnh AT/debug UART bằng dây.
3. Tạm ngắt Web BLE `Nezumi_Chan`; chỉ test Classic SPP.
4. Pair `Nezumi_Chan_SPP` trong **Settings > Bluetooth & devices**.
   Windows hiện **Paired / Not connected** khi chưa mở cổng serial là bình thường.
5. Nhấn **Win+R** > `bthprops.cpl` > **COM Ports**; tìm dòng
   **Direction: Outgoing; Name: Nezumi_Chan_SPP**.
   Nếu chưa có, **Add > Outgoing > Nezumi_Chan_SPP > Port**.
   Ghi nhớ số COM thực tế; không chọn cổng Incoming.

### C. Lệnh kiểm tra rồi chạy Dashboard

Kiểm tra các cổng (không mở kết nối):

```powershell
python tools/classic_spp_link_diag.py --list
```

Chạy test SPP trên cổng Outgoing đã xác nhận (ví dụ máy hiện tại là COM16):

```powershell
python tools/classic_spp_link_diag.py --port COM16 --seconds 40
```

Trong log, `com_open` nghĩa là Windows đã mở COM;
**`rx` và `rx_bytes > 0`** mới chứng minh có dữ liệu qua JDY-33.
Ví dụ máy đã thử nhận:

```text
PID,READY,state=IDLE,locked=1,calibrated=1,rxbytes=12,ore=0,fe=126,ne=3,rxoverflow=0
```

Đóng tool chẩn đoán trước khi mở Dashboard: mỗi COM chỉ nên có **một ứng dụng** mở.

```powershell
python tools/classic_spp_dashboard.py --port COM16
```

Trong GUI: **Connect** > tab **Terminal** để thấy bản tin `PID,READY`;
tab **Maze** chỉ có bản đồ khi firmware phát `MAPCFG/MAPROW/MAPEND`.
Không cần nạp `flash_map_diag_96m` chỉ để xem Terminal.

### D. Nếu gặp lỗi

- **COM không đúng / 0 byte:** kiểm tra Outgoing và Name trong `bthprops.cpl`;
  COM16 từng là Incoming trước khi Windows tái tạo thành Outgoing trên máy test.
- **Access is denied:** đóng miniterm, Dashboard hoặc chương trình khác đang dùng cùng COM.
- **Paired nhưng Not connected:** chạy tool hoặc Dashboard trên **Outgoing COM** để Windows tự kết nối.
- **SerialException / mất link:** thu log Windows và log SPP **cùng thời điểm**
  bằng hai PowerShell khác nhau:

  ```powershell
  powershell -NoProfile -ExecutionPolicy Bypass -File .\tools\windows_bluetooth_link_debug.ps1 -DurationSeconds 90
  ```

  ```powershell
  python tools/classic_spp_link_diag.py --port COM16 --seconds 40
  ```

- **Có PID nhưng Maze trống:** firmware hiện tại chưa phát khung map, không phải lỗi SPP.

Chi tiết thêm: [Windows connection debug](../docs/system_debug/WINDOWS_CLASSIC_SPP_CONNECTION_DEBUG_20261028.md),
[JDY-33 desktop docs](../docs/system_debug/JDY33_CLASSIC_SPP_DESKTOP_20261028.md).

---

**Quickstart hiện tại:** [README_TESTING.md](README_TESTING.md) — lệnh chạy Wall PID, IR, BLE và bộ kiểm thử Maze Readiness tất cả trong một.

The 33 historical `tools/` files were restored from `archive/debug-tools-precleanup-20261024`; on 24/10/2026, eight redundant copies were retired. Additional Classic SPP tools were added subsequently, so the historical count of 26 files is **not** the current directory size. Canonical regression tests live in `tests/regression/` and `tests/web/`, and the browser console lives in `web/ble_debug_console.html`. The archive remains unchanged.

Keep the guarded Flash-map runner, BLE one-shot qualification, OLED/MPU hardware acceptance (`tools/oled_mpu_pipeline.py`), UART, IR, encoder, PID pipelines and nonduplicate host regressions; removing them would compromise future requalification. **Never delete or rewrite real `results/` logs**. Software tests cannot substitute for hardware acceptance.

- Web UI: `web/ble_debug_console.html`
- Native contract: `tests/regression/flash_map_protocol_test.py`
- How to run safely: `docs/system_debug/FLASH_MAP_WEB_PIPELINE.md`
- Firmware profile: `flash_map_diag_96m`

Run read-only preview by default. Flash writing requires the dedicated,
motor-locked diagnostic firmware and explicit write/backup flags. Do not
mass-erase sectors 6/7 on upload, and do not confuse a CI PASS with a
successful physical Flash readback/reboot test.

Main BLE one-shot entrypoint: `python tools/ble_full_pipeline.py --name Nezumi_Chan` (motor OFF by default). For the full test matrix, safety gates and evidence paths see `docs/system_debug/BLE_ONE_SHOT_PIPELINE_20261024.md`.


### JDY-33 RX A/B/C qualification — before considering PID tuning

Run `python tools/ble_rx_qualification.py --port COM7 --name Nezumi_Chan`
with ST-Link and COM7 USB-TTL connected, Web BLE disconnected, motor power
safely isolated. A/B isolate receiver enable under identical autonomous
TX load; C writes 16-byte nonce frames via GATT and requires matching
MCU `RXACK` **on UART1** as independent proof of PA3 reception.
Default one-run orchestration restores motor-locked `flash_map_diag_96m`
on exit. Reports live under `results/ble_rx_ab_*`.
See `docs/system_debug/BLE_RX_AB_QUALIFICATION_20261024.md`.
This does not operate motors or change PID gains.

### PA0 gyro turn optimization 90° / 180° (new)

Run the **isolated** `turn_pid_diag_96m` firmware (never normal
`system_test_96m` for this campaign), then:
`python tools/turn_pid_pipeline.py --port COM7 --mode 90 --profile base --runs 3`.
The capture saves 50Hz gyro/rate/PWM/encoder CSV, plots, raw serial and summary
under `results/turn_pid_*`. UART commands select idle mode/profile but **cannot
start motors**; only physical PA0. After accepted BASE tests, compare FAST
(30% maximum PWM). See `docs/system_debug/TURN_PID_PIPELINE_20261026.md`.
Offline host tests: `python tools/test_turn_pid_pipeline.py`.

### Phase 1 motor sweep and Phase 2 tuned pivot

`python tools/turn_pid_pipeline.py --port COM7 --mode motor --profile base --runs 3`
performs **only PA0-initiated** short PWM characterization (six <=26% stages).
After interpreting the logs, use `--mode 90 --profile tuned --runs 1`
for a cautious first 35% TUNED turn. Captures include applied PWM,
encoder speed (PPS), `motor_steps.csv`, gyro timing and normal turn
response metrics. `docs/system_debug/TURN_PHASE1_PHASE2_20261027.md`.

### Jolt-free pivot trial v4

Use `--profile smooth` (never `tuned`) for experimental smooth terminal
approach without a 15% stall kick or near-target reverse PWM.
Capture 1 pilot turn before collecting 3 repeats; see
`docs/system_debug/TURN_PHASE1_PHASE2_20261027.md`. Motor sweep on-time
is now 100ms per stage to stay within the prior +65deg safety envelope.
