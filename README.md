# Micromouse STM32F411CEU6 - Minimal Core

> **Bàn giao OLED và dọn tools trùng lặp (24/10/2026):** Bộ 33 file lịch sử được khôi phục từ [nhánh archive](https://github.com/HoangAnh301194/Micromouse_STM32F411CEU6_BLE_Debug_Web/tree/archive/debug-tools-precleanup-20261024). Sau yêu cầu dọn của người dùng, loại 8 bản sao không cần thiết, còn **26 file trong `tools/`** (gồm `tools/README.md`); regression chuẩn ở `tests/`, WebBLE ở [`web/ble_debug_console.html`](web/ble_debug_console.html). **Toàn bộ log `results/`, pipeline nghiệm thu OLED và firmware gốc được giữ nguyên.**


> **Tiến độ mới nhất và ngữ cảnh bàn giao đa agent:** [PROJECT_STATUS.md](PROJECT_STATUS.md). Agent/chat mới bắt đầu từ [AGENTS.md](AGENTS.md); kết quả nghiệm thu I2C–MPU6050 nằm tại [docs/mpu_i2c_rc2/README.md](docs/mpu_i2c_rc2/README.md).

This branch (`giomuadongbac`) is the STM32F411CEU6 RC2 baseline with opt-in 96 MHz system integration and UART PA9/PA10 diagnostic profiles; the default production build does not run a stress benchmark.

The PCB pin mapping is intentionally unchanged. The goal is to keep the real-time path small, explicit and testable before maze solving, OLED activation, calibration workflows and advanced motion features are enabled.

## Active runtime

```text
main.c
├── board.c
├── control.c
├── motion.c
├── pid.c
├── motor.c
├── encoder.c
├── ir_sensor.c
├── mpu6050.c
├── i2c.c
├── system_timer.c
└── bt_debug.c
```

Module roles:

```text
main.c       = initialization + application commands + low-rate telemetry
control.c    = TIM11 owner + 1 kHz scheduler/ISR
motion.c     = motion state machine + controller
pid.c        = reusable PID mathematics
drivers      = peripheral-specific register configuration
```

Flash persistence now has an app-only 256 KiB linker region and versioned,
CRC32-verified, append-only records in reserved sectors 6/7. Its 5x5 legacy
layout is readable, and the new map blob API can hold up to 1000 bytes.
It remains **inactive** in the runtime: no startup writes/erases, and board
write/readback/power-loss qualification is still outstanding. See
[Flash storage safety and integration](docs/flash_storage.md).

The SSD1306 driver and font sources are present in `src/ssd1306.c`,
`src/fonts.c`, `include/ssd1306.h`, and `include/fonts.h`, but are not
initialized by the active runtime. I2C1 remains owned by `i2c.c`; the OLED
must be integrated as a non-blocking shared-bus client with MPU priority.

The real-time path is:

```text
TIM11 @ 1 kHz
      |
      v
control.c
├── Encoder_Update()
├── Motor_Update1ms() (PWM ramp and motor command lease)
├── MPU6050_NotifyControlTick() (timing only; I2C state machine outside TIM11)
├── consume/restart IR scan
└── Motion_Update()
        └── PID_Compute()
              └── Motor_SetPair()
```

BLE and USART1 debug formatting / telemetry run in main context, not in the TIM11 ISR. MPU6050 async I2C work is also progressed outside TIM11.

## Peripheral ownership

- TIM2 CH1/CH2 -> `motor.c`
- TIM3/TIM4 -> `encoder.c`
- TIM5 -> `system_timer.c`
- TIM10 -> `ir_sensor.c`
- TIM11 -> `control.c`
- ADC1 + DMA2 Stream0 -> `ir_sensor.c`
- I2C1 -> `i2c.c`
- MPU6050 logic -> `mpu6050.c`
- USART2/JDY-33 -> `bt_debug.c`

`include/pinout.h` remains the fixed PCB wiring definition and is not changed by this refactor.

TIM10 and TIM11 derive their prescalers from the current RCC clock configuration rather than assuming a fixed 96 MHz timer clock.

## Minimal motion scope

Only three motion states are active:

- `MOTION_IDLE`
- `MOTION_STRAIGHT`
- `MOTION_TURN`

BLE commands:

```text
F  straight 180 mm
L  turn left 90 deg
R  turn right 90 deg
S  immediate stop
Z  zero gyro yaw (idle only)
```

## OLED + MPU6050 shared-I2C diagnostic (motor locked)

The default production build intentionally keeps OLED disabled. The new
`oled_mpu_diag_96m` build is an **opt-in, motor-locked hardware qualification**
profile that retains the same 1 kHz TIM11, IR and MPU acquisition workload.

The cooperative renderer `src/oled_shared.c` sends only changed pixels with
short DMA transactions through the single I2C1 owner in `src/i2c.c`.
MPU gyro always has priority. A bus below nominal 300 kHz suppresses OLED
refresh rather than risking sensor freshness. Do not use legacy blocking
`SSD1306_UpdateScreen()` or full-page DMA after MPU sampling begins.

```powershell
python -m platformio run -e oled_mpu_diag_96m -t upload
python -m pip install pyserial
python tools/oled_mpu_pipeline.py --port COM5 --baseline-seconds 15 --oled-seconds 15
```

The pipeline compares OLED OFF/ON using USART1 PA9/PA10 and generates raw
UART, CSV and JSON evidence under `results/oled_mpu_YYYYMMDD_HHMMSS`.
`--self-test` verifies host parsing only; no build or synthetic test is
equivalent to MCU/hardware acceptance. For measured criteria and wiring,
see [OLED/MPU shared-I2C acceptance](docs/system_debug/OLED_MPU_SHARED_20261024.md).

**Handover (2026-10-24):** Board A/B acceptance PASS (15 s OFF + 15 s ON): TIM11 1 kHz, gyro 1000 → 992.73 Hz (-0.73%), OLED 1056/1056 DMA TX completed, 73 frames, zero OLED/MPU bus faults; see [`report.json`](results/oled_mpu_20261024_082616/report.json). **Scope:** OLED transport with motor outputs locked; production OLED/menu/API are not enabled, visual pixel verification, motor noise and long soak remain unqualified. The acceptance pipeline remains available for future regressions.

## IR sensor + OLED 128x64 bench mode (motor locked)

Six IR channels can be displayed using existing TIM10/ADC1 DMA and SSD1306
drivers without activating normal motion, MPU6050 or TIM11 control.
The isolated test gives the OLED exclusive PB8/PB9 I2C1 access;
it does **not** validate OLED+MPU coexistence.

```powershell
python -m platformio run -e ir_oled_test_96m -t upload
```

Screen rows show L90/R90, L45/R45, L0/R0 as ambient-subtracted
ADC counts, refreshed at most 5Hz. Optional PA9/PA10 UART frames use
`IR6,...` at 5Hz. Motor outputs are permanently locked.
[Wiring, caveats and restoring normal firmware](docs/system_debug/IR_OLED_VIEWER_20261024.md).

## IR filter qualification (motor-locked)

The separate `ir_filter_diag_96m` profile streams six-channel RAW,
Median-3, EMA and hybrid candidates over USART1 at 115200 baud. Before
upload or capture, physically disconnect motor VM; the firmware lock is an
additional safeguard, not a replacement for the physical disconnect.

The latest evidence reaches **ACQUISITION_ACCEPTED only**. No filter has
been selected for production or Wall PID. See the [IR validation report](docs/ir/reports/IR_VALIDATION_20261027.md),
[acceptance JSON](results/ir_qualification_20261027/acceptance.json),
[capture manifest](results/ir_qualification_20261027/manifest.csv) and
[per-channel metrics](results/ir_qualification_20261027/metrics_by_channel.csv).
The capture pipeline preserves raw UART, ON/OFF samples, per-channel traces,
summary JSON and six plots under `results/ir_filter_*`.

Example motor-locked captures (measure and record actual fixture geometry):

```powershell
python tools/ir_filter_pipeline.py --port COM6 --seconds 30 --scenario no-wall --surface none --light room --trial 1 --plot
python tools/ir_filter_pipeline.py --port COM6 --seconds 20 --scenario static --distance-mm 50 --surface "white matte" --light room --trial 1 --notes "record angle and fixture uncertainty" --plot
```

## Build

```powershell
git fetch origin
git switch giomuadongbac
git pull origin giomuadongbac

python -m platformio run -t clean
python -m platformio run
```

Upload with ST-Link:

```powershell
python -m platformio run -t upload
```

## Bring-up order

1. Verify MPU6050/I2C and yaw output.
2. Verify encoder sign/count.
3. Verify motor direction and PWM with wheels lifted.
4. Verify TIM11 control-loop timing.
5. Verify IR scanning.
6. Test straight motion at low speed.
7. Test left/right turns.
8. Reintroduce wall control and navigation only after the baseline is stable.

See `docs/architecture.md` for ownership rules.

## UART debug PA9/PA10: tested frequency and task-specific configuration

**Hardware configuration:** 96 MHz nominal HSI trim25; USART1 **115200 baud, 8N1**. PA9 TX uses DMA2 Stream7 Channel 4 and a 1024-byte circular queue; PA10 RX uses RXNE interrupt. TIM11 runs separately at **1 kHz** with IRQ priority 0; DMA TX IRQ priority 7, RX IRQ priority 8. UART formatting/enqueue runs in main, not the TIM11 ISR. This link is separate from USART2/JDY-33 BLE.

**Real-board evidence (2026-10-19):** three independent 5-second-per-rate hardware runs, motor outputs locked but TIM11, I2C/MPU and IR active. Every run passed **80 Hz** as its highest tested valid benchmark rate and recommended **60 Hz** to preserve headroom. A PA10 PING/ACK test ran at 10 Hz throughout each rate. These rates apply to the test's approximately 75-byte `U_PROBE` messages plus system status traffic; a different message size or concurrent workload needs its own validation.

| Rate | 104312 received/generated | 104529 received/generated | 105726 received/generated | Peak TX queue and measured UART CPU, run 104529 | Result |
|---|---:|---:|---:|---:|---|
| 20 Hz | 100/100 | 100/100 | 100/100 | 483 B / 0.342% | PASS |
| 40 Hz | 200/200 | 200/200 | 201/201 | 542 B / 0.484% | PASS |
| **60 Hz** | 300/300 | 300/300 | 301/301 | 575 B / 0.595% | **Recommended** |
| 80 Hz | 400/400 | 401/401 | 401/401 | 696 B / 0.728% | PASS, less reserve |
| 100 Hz | 501/501 | 501/501 | 501/501 | 978 B / 0.828% | LIMIT: >768 B queue threshold |
| 120 Hz | 601/601 | 601/601 | 601/601 | 865 B / 0.954% | LIMIT: >768 B queue threshold |
| 140 Hz | 678/701 | 684/701 | 681/701 | 1023 B / 1.058% | FAIL: dropped messages |

PASS criteria: **zero** dropped TX records, missing host sequences, RX overflow/errors, DMA errors, missed PING acknowledgments, TIM11 lateness/gaps or MPU/I2C errors/timeouts; TX queue peak <=768 B (75% of 1024), UART work <=3% of nominal CPU cycles, and probe rate >=95% requested. The displayed CPU percentage measures UART formatting/enqueue + DMA TX and USART RX interrupt time **only**; it is **not overall CPU utilization**. Saturation is determined mainly by the **wire bitrate / DMA buffer**, not TIM11 CPU contention.

### Select the log rate for the application

| Debug task | Recommended setting at 115200 8N1 | Why |
|---|---|---|
| Fault, STOP, reset and exceptional I2C/MPU alerts | Event-driven; health counter 1–5 Hz | Preserve control/diagnostic messages and avoid repetitive spam |
| General system health: clock, TIM11, I2C, IR, MPU | 5–10 Hz | Small diagnostic load, adequate for changing counters |
| PWM / encoder / controller debugging | 20–40 Hz | Observe trends without consuming all link bandwidth |
| Continuous compact telemetry | **60 Hz** | Conservatively selected in all three measured motor-locked runs |
| Short high-resolution captures | 80 Hz | Maximum tested PASS for this payload; monitor queue/drops |
| 100–120 Hz sustained | **Not default** | No lost probes in short test, but exceeded the queue headroom threshold |
| >=140 Hz sustained or 1 kHz ASCII text logs | **Do not use** | Real dropped records start at 140 Hz; TIM11 1 kHz is an ISR rate, not a UART log rate |

**Calculation:** 115200 baud x (1 byte / 10 serial bits) = **11,520 bytes/s** ideal link limit. For an encoded record of `N` bytes, `f_max <= 115200 / (10 * N)` before accounting for other records, acknowledgments and headroom. Around 75 bytes corresponds to **~153 records/s theoretical**, never an assured application rate. The recommended 60 Hz is therefore a *measured, packet-specific engineering setting*, not a hard MCU clock limit.

**Reproducible evidence:**
- [104312 UART benchmark](results/system_uart_20261019_104312/uart/uart_rate_summary.json)
- [104529 UART benchmark](results/system_uart_20261019_104529/uart/uart_rate_summary.json)
- [105726 UART benchmark](results/system_uart_20261019_105726/uart/uart_rate_summary.json)
- [104529 system Motor OFF/ON summary](results/system_uart_20261019_104529/summary.json) and [sample-level CSV](results/system_uart_20261019_104529/measurements.csv)
- [Detailed UART benchmark methodology](docs/system_debug/UART_PA9_PA10_RATE_PROFILE_20261017.md) and [extended motor sweep](docs/system_debug/SERIAL_AUTOMATED_96MHZ_20261017.md)

On the tested Windows machine the USB-UART adapter was `COM7`. Use the actual port on each machine:

```powershell
# Benchmark UART with motor outputs locked
python tools/system_uart_pipeline.py --port COM7 --flash --uart-only

# Unified multi-profile diagnostic; this WILL request motor PWM in the ON phase
python tools/system_uart_pipeline.py --port COM7 --flash --motor-on --with-uart
```

### Distinguish Motor OFF shadow, UART benchmark and physical Motor ON

- `integration_diag_uart_96m`: motor outputs locked, TIM11/MPU/IR active, PC can request UART probe sweep. **No physical motor drive**.
- `integration_diag_motor_off_96m`: motor outputs locked, runs the same 33 stage timing schedule without motor drive. **No physical motor drive**.
- `integration_diag_motor_on_96m`: firmware calls `Motor_SetPairPermille()` to request **real TIM2 PWM** (left/right/both 0..100%). It is **not merely TIM11 simulating motor rotation**. Commanded PWM alone does not prove electrical motor output.

During real-board run `104529`, the Motor-ON log recorded applied PWM and nonzero encoder counts, including both wheels at 80%: **25,982 / 26,095 counts in 3.5 seconds**. Motor OFF remained at zero. This is strong evidence of encoder movement but **not direct measurement of motor supply voltage, output PWM waveform, driver current, temperature or visual rotation**. If the operator sees no movement, verify VM power and actual mechanics rather than assuming software PASS certifies the drive. The later run `105726` has an empty `results` array and no complete Motor-ON log, so does not qualify motor motion.

The 104529 Motor-ON endurance phase passed 33 software stages with zero TIM11 gap/late, gyro stale, UART drops or DMA errors, and ~**0.155% measured UART CPU** for normal telemetry. Its Motor-OFF phase was **UNQUALIFIED** only for an I2C nominal clock check: MPU boot speed probing selected **~296,296 Hz** rather than 400,000 Hz; the operational OFF I2C errors/timeouts remained zero. The reason the 400 kHz boot probe failed is not yet captured. **Important:** the 5–400 Hz UART stress benchmark was conducted in separate motor-locked firmware *before* the motor endurance phases. 80 Hz under concurrent energized motor load has **not** been established.

**Handover status:** PA9 TX DMA and PA10 RX command/PING are **qualified for controlled debug use with 115200 8N1, 60 Hz recommended**, packet-dependent and with queue/error monitoring. **Not yet qualified:** combined motor-power electrical behavior / EMI, stress UART at 80 Hz with motor energized, all packet sizes, lifetime endurance, or safety-critical communication. Keep these scopes distinct.

## MPU6050 / I2C1 module — handed over

The MPU6050 gyro, asynchronous temperature measurement and I2C1 driver are maintained as one logical module without relocating validated source files. Public include: [`mpu6050_i2c_module.h`](include/mpu6050_i2c_module.h). The **[module README](docs/mpu_i2c_rc2/README.md)** records integration, measured acceptance (STILL, SHADOW and 80–100% motor LIVE), constraints and future optimization. Raw evidence in `results/` is retained unchanged. The completed MPU 180-second test pipeline has been retired; other modules' motor/UART/clock tools remain available.

### Flash map WebBLE diagnostic

The WebBLE 5×5/16×16 map viewer is retained at [web/ble_debug_console.html](web/ble_debug_console.html). Flash-map bench automation is available at [`tools/flash_map_pipeline.py`](tools/flash_map_pipeline.py); see the [operator guide](docs/system_debug/FLASH_MAP_WEB_PIPELINE.md). Production does not save maps automatically.

### One-command JDY-33 BLE PA2/PA3 qualification (new, hardware pending)

For motor-locked Bluetooth testing without a PA9/PA10 USB-TTL adapter:
```powershell
python tools/ble_full_pipeline.py --name Nezumi_Chan
```
The command auto-flashes ST-Link builds, uses the Windows BLE adapter and Bleak,
runs 18 sizes/rates at 115200 and six at 38400, writes loss/CRC/TIM11/IR/MPU
metrics and restores the motor-locked image. An additional guarded, raised-wheel
real speed-PID motor-ON phase can be enabled by:
```powershell
python tools/ble_full_pipeline.py --name Nezumi_Chan --motor-on --fixture-ready --independent-vm-cutoff --allow-30-percent
```
See [BLE one-shot documentation](docs/system_debug/BLE_ONE_SHOT_PIPELINE_20261024.md).
No physical BLE benchmark PASS exists yet. Do not open the legacy browser Web
Console simultaneously; the benchmark uses BLE GATT directly from Python.

## System Test PA0 + UART1 (no OLED, 26/10/2026)

Build `python -m platformio run -e system_test_96m`. This profile does **not**
detect or update the OLED. After MPU calibration completes, **press and release
PA0 once** to run one test. The sequence after successful runs is +90° IMU-only
pivot, +180° IMU-only pivot, then straight 200 mm using gyro heading PID and
encoder distance control. A subsequent PA0 press while moving stops and locks
motor immediately; failed runs retry the same mode on a new press. UART1 PA9/PA10
at 115200 logs readiness, calibration, results and 20 Hz samples; UART1 `S`
stops but cannot start motion. Test-only geometry: **R=34 mm, CPR=1430**.
PWM capped at 26%; first validate with lifted/secured wheels and an independent
VM power cutoff. See [PA0 System-test protocol](docs/system_debug/SYSTEM_TEST_OLED_20261026.md).
**Physical motion and distance accuracy are not yet accepted.**

## 90° / 180° gyro turn PID diagnostics (2026-10-26)

An opt-in `turn_pid_diag_96m` profile replaces OLED with PA0 and UART for
repeatable yaw turn tuning. A one-shot physical PA0 press starts rotation;
UART1 COM7 selects mode 90/180 and bounded BASE/FAST profiles (no remote start).
A TIM11-timestamped 50Hz trace includes yaw, gyro rate, rate reference,
PID/feedforward, PWM, encoders and post-stop inertial drift. Start with
`python tools/turn_pid_pipeline.py --port COM7 --mode 90 --profile base --runs 3`.
Logs are preserved under `results/turn_pid_*`.
Read [turn tuning pipeline](docs/system_debug/TURN_PID_PIPELINE_20261026.md).

### Turn tuning stage 1 / stage 2 (2026-10-27)

Added bounded open-loop PWM motor response mode (USART1 idle command `m`,
physical PA0 start only) and new experimental `TUNED` pivot profile
(idle command `t`, ±2° finish target). Log applies actual motor PWM and
encoder counts/s; retain historical BASE/FAST. See
[Phase 1 + Phase 2 test procedure](docs/system_debug/TURN_PHASE1_PHASE2_20261027.md).
