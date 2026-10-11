# MPU6050 diagnostics on STM32F411CEU6 (100 MHz)

This branch intentionally runs **isolated MPU6050 / I2C / TIM11 diagnostics**. The old motor test remains under `#if 0` in `src/main.c`; motors are not initialized/commanded by the active `main`.

## Clock and hardware assumptions

- **STM32F411CEU6**: 100 MHz HCLK from HSI 16 MHz through PLL (M=16, N=200, P=2), Voltage Scale 1, 3 Flash wait states.
- APB1 50 MHz, APB2 100 MHz, TIM5 1 MHz tick, TIM11 1 kHz tick.
- **3.3 V power supply** assumed. Flash wait states configured for **VDD >= 2.7 V**.
- USB OTG FS is **not available** with the current PLLQ=5 (40 MHz), which does not provide the required 48 MHz USB clock. External USB–TTL UART and ST-Link do not need the USB peripheral clock.
- I2C1 PB8=SCL, PB9=SDA, MPU6050 7-bit address 0x68. USART1 TX=PA9, 115200 baud.
- No MPU6050 INT pin wiring needed: firmware reads `INT_STATUS(0x3A)` via interrupt-driven 1-byte I2C transaction, then reads gyro Z via interrupt-driven 2-byte transaction.
- Full-scale gyro remains **±2000 °/s**; `SMPLRT_DIV=0` (1 kHz) with DLPF enabled.

## Build and flash

```powershell
git switch feature/motor-debug-console
git pull --ff-only origin feature/motor-debug-console
python -m platformio run
python -m platformio run --target upload
```

After reset, first verify `CLOCK,sysclk=100000000 hsi_pll100=1 apb1=50000000 apb2=100000000` and `[IMU_READY]`. If initialization fails, inspect `[IMU_INIT_FAIL]`, hardware power and I2C lines. This configuration has not been tested on physical hardware by the assistant; verify the serial log before relying on it.

## Four experiments

### (1) Sample freshness and intersample timing

Collect 60–120 s of `IMU`, `FRESH`, and `NOISE` while keeping the device stationary.

`FRESH` reports total `status`, total `ready`, per-window `not_ready`, per-window `fresh`, and total `stale`. `ready` is MPU6050 `INT_STATUS.DATA_RDY=1`; `fresh` counts gyro reads preceded by that indication. `same_code` in `NOISE` counts identical quantized gyro codes; identical codes are **not** proof of a repeated physical sample. `sample_min_us/sample_max_us` are spacing between successful **I2C data read completions**, not the sensor's internal data-ready timestamps.

Each reading now requires **two I2C transactions**. It is normal for the measured throughput to be lower than the previous single-transaction baseline. Check `err/timeout` as well. Do not assume the two I2C transactions are atomically synchronized to a 1 kHz sensor sample; reading `INT_STATUS` merely establishes data-ready evidence at polling time.

### (2) Drift without deadzone

Keep the device absolutely still during the new **5-second gyro warmup + 4000 DATA_RDY-confirmed calibration samples** and for at least 120 s after `DRIFT_START`. `FRESH` prints:
- `yaw_deadzone_mdeg`: current LPF + 0.3°/s software deadzone.
- `yaw_lpf_no_deadzone_mdeg`: LPF + calibrated bias but **no deadzone**.
- `yaw_raw_no_deadzone_mdeg`: bias-corrected gyro integrated without software LPF/deadzone.

Degrees = millidegrees / 1000. Compare slope (degrees per minute), not only the last line. Measurements are relative yaw estimates, not absolute heading.

### (3) Independent angular reference: manual visual turns are NOT accurate enough

**Do not treat a hand-aligned ±90° or ±180° turn as ground truth**:
a ±5° visual reference uncertainty exceeds the desired IMU error.
The KEY button is **no longer** used to score angle accuracy.
Instead it requests stationary re-calibration (allow time to release the
button before the 5-second warmup is over).

For precision rotation experiments, use a calibrated motorized turntable
with an independently measured encoder angle, or a top-down calibrated
camera with a tracked fiducial/AprilTag and quantified angular uncertainty.
Motion controller performance and gyro angle accuracy are different tests.
Without an independent reference, only report drift, repeatability and
unreferenced closed-loop rotation observations, not absolute angle error.

### (4) DLPF 42 Hz vs 98 Hz, with dynamic response

Run the same known rotation profile twice, once with

```c
#define MPU6050_DLPF_CFG 3U  // gyro 42 Hz, nominal delay ~4.8 ms
```

and once with

```c
#define MPU6050_DLPF_CFG 2U  // gyro 98 Hz, nominal delay ~2.8 ms
```

in `include/mpu6050.h`. Rebuild/upload after changing the macro. Calibrate with the robot stationary each time. During motion (>10°/s), the firmware prints `D,<time_us>,<raw_bias_corrected_mdps>,<software_lpf_mdps>` near 100 Hz until 1.5 seconds after the last motion threshold crossing. Compare noise (`NOISE`), transient response and known-angle error between runs.

**Limitation:** `D` compares MPU sensor output and the software LPF. It cannot independently measure the MPU hardware DLPF group delay: an externally timed motion reference (turntable encoder/photogate) would be needed for that.

## TIM11 ISR timing

`TIMING` logs min/max observed period, counts of intervals outside ±50 µs, intervals longer than 1500 µs, and **observed** maximum ISR CPU cycles. It is *not* a formal WCET bound; active main currently sets `Control_SetDiagnosticOnly(1)`. Re-test with motor/encoder/IR logic enabled later, under maximum expected load.

## Debug output and interpretations

- `IMU`: gyro throughput/errors, I2C transfer timing, current filtered gyro, TIM11 average frequency.
- `FRESH`: DRDY evidence, stale reads, three independent yaw integrals.
- `NOISE`: statistics over all valid gyro register reads in each one-second window, before software LPF and deadzone.
- `TIMING`: observed TIM11 timing/Jitter.
- `ANGLE_START` / `ANGLE_DONE`: KEY reference-turn measurements.
- `D`: 100 Hz trace on moving gyro.
- `DRIFT_START`: automatic zero of three yaw integrators just after initial calibration.

The log may still contain replacement characters if USB-UART transport/receive settings are problematic; `uart_drop=0` and `uart_dma_err=0` only cover the firmware's own transmit-buffer/DMA counters.

## Boot failure diagnostics (clock/IMU recovery)

A previous revision waited for `PWR_CSR.VOSRDY` **before** starting PLL; on
STM32F411 the requested voltage scale is activated **after PLL is enabled**.
This caused the boot code to return early and run at 16MHz. Clock startup
now waits for `PLLRDY`, **then** `VOSRDY`, and finally switches SYSCLK to PLL.

On reset inspect:
- `CLOCK,sysclk=100000000 hsi_pll100=1 apb1=50000000 apb2=100000000`
- `CLOCK_DIAG,fault=0` and RCC/PWR/FLASH register snapshots.
- `[IMU_READY]` if all MPU writes and readbacks succeed.

If still failing, do **not** assume there is a hardware wiring problem. Read
`BOOT_FAIL` for the exact `clock_fault`, `imu_status`, `init_step`,
`reg`, `expect`, `got` and `i2c_status`.
`I2C_DIAG` also prints I2C1 SR1/SR2/CR1/CR2.
The LED blinks and `BOOT_FAIL_ALIVE` repeats about once per second in
failure mode. Motors remain disabled.

Clock fault stages: 0=success; 1=HSI ready failed; 2=Flash latency did
not latch; 3=HSI SYSCLK switch failed; 4=PLL failed to turn off;
5=PLL did not lock; 6=voltage scale not ready after PLL lock;
7=SYSCLK switch to PLL failed; 8=CMSIS clock calculation mismatch.

MPU startup stages: 1=address probe, 2=WHO_AM_I, 3=reset,
4=wake, 5=GYRO_CONFIG write, 6=CONFIG/DLPF write, 7=SMPLRT_DIV write,
8=GYRO_CONFIG readback, 9=DLPF readback, 10=INT_PIN_CFG write,
11=INT_ENABLE write. Values are hexadecimal except step/status.

No ARM toolchain or board was available for build/bench testing in the
assistant's environment. Confirm the startup log first before longer IMU tests.

## MPU address NACK immediately after 100MHz PLL

Observed log: `BOOT_FAIL_ALIVE,clock_fault=0 imu_step=1 i2c_status=4`.
According to `include/i2c.h`, status 4 = `I2C_NACK`:
the probe sent address 0x68 but the slave did not acknowledge.
`clock_fault=0` means the clock sequence did not report failure, but
verify actual `CLOCK,sysclk=100000000` after opening serial and resetting
the board.

The startup driver now **auto-tests** 400, 300, 200, and 100 kHz,
requiring BOTH address ACK and a correct WHO_AM_I (0x68).
`I2C_PROBE` prints for each rate:
- `400_a` / `300_a` / etc: address status, 0=OK, 4=NACK.
- `400_w` / etc: WHO_AM_I I2C status. 255 = skipped when probe failed.
- `400_id` / etc: WHO_AM_I value. 0xFF = not tried.
- `selected`: index 0=400, 1=300, 2=200, 3=100, 255=none.
- `selected_scl`: selected actual nominal SCL from configured CCR.
`I2C_LINES` reports PB8 SCL, PB9 SDA digital input levels sampled at
the end of boot. In an **idle** I2C bus both pins should normally read 1.
If `I2C_PROBE` fails at **every** speed, inspect MPU VDD, common ground,
AD0 address strap (0x68 vs 0x69), PB8/PB9 continuity, and external pull-ups,
rather than lowering CPU frequency.

The I2C driver retains the selected bus speed across its recovery path.
If only 100 or 200 kHz works, the new two-transaction DATA_RDY+gyro
diagnostic may read substantially fewer than 1000 gyro samples/s.
The exact sample rate must be established from measured `read_hz`, not
assumed from `SMPLRT_DIV=0`.

To capture boot output on COM7: open `python -m platformio device monitor
-p COM7 -b 115200`, **then press RESET** on the MCU board.

## New calibration comparison — 5-second warmup + 4-second collection

This section supersedes the original 200-sample startup calibration.
At boot, `MPU6050_Calibrate` starts a **nonblocking** zero-rate sequence:

1. State `CAL,state=1`: 5000 ms warm-up / settle, with I2C gyro polling still active. The gyro does not output a valid yaw during calibration.
2. State `CAL,state=2`: 4000 DATA_RDY-confirmed gyro samples (8 blocks × 500). Each block must have standard deviation ≤0.40°/s and span ≤2.0°/s; across accepted block means, max-min must remain ≤0.12°/s. Those limits reject **obvious** movement, not perfectly constant slow rotation or linear acceleration.
3. If a block fails, discard all accumulated calibration samples, return to warmup, and retry. Three rejections maximum: `CAL,state=4` (failed), no valid gyro yaw. `fail_reason=1` for noisy/spiky block, `2` for inconsistent block means.
4. On success, `CAL,state=3`, set the mean zero-rate bias and zero the three yaw integrals. `CAL_DONE` prints accepted samples, block count, retries, previous/new bias, max block standard deviation, and block mean spread.
5. While the robot remains **physically stationary**, `CAL_CHECK` prints measured raw integrated yaw drift and rate after 30 and 120 seconds. Raw yaw is deliberately free of software deadzone, so nonzero drift is visible. `NOISE` continues to report per-second raw gyro noise and mean corrected rate.

Press **KEY once only while robot is at rest** to request calibration again.
A previous completed bias is recorded for comparison. There is no runtime
bias adaptation, because slow physical rotation cannot be distinguished
reliably from a gyro offset with gyro Z measurements alone.

**Suggested reproducible comparison:** reset MCU; keep the board still
through the 9-second initialization and subsequent 120-second observation;
record `CAL_DONE`, both `CAL_CHECK` records, and `NOISE`. Repeat once from
a cold power-up and once after the device has been operating for ~3–5 minutes:
press KEY **only while stationary**, let it finish and repeat the 120-second
observation. Temperature stabilization may take longer than 5 seconds; this
experiment distinguishes short-window averaging from slow temperature bias
drift. Do not infer a precision grade from a visual turn.

`CAL_CHECK,raw_mdeg_per_min` is calculated as integrated raw-deadzone-free
yaw divided by actual elapsed time. A better value than the prior ~0.6°/min
is **not guaranteed** until measured under comparable conditions.

## COM7 capture and automatic CSV/plot export

`tools/imu_capture_plot.py` captures UART telemetry, stores
`raw_serial.log`, separates `IMU/FRESH/NOISE/TIMING/CAL/CAL_DONE/CAL_CHECK/D`
into per-kind CSV files, and generates PNG charts after recording.
It also works offline with a previously saved terminal log.

```powershell
python -m pip install pyserial matplotlib
# Stop PlatformIO serial monitor before opening COM7 in Python.
python tools/imu_capture_plot.py --port COM7 --baud 115200 --seconds 180
# Optional live yaw display
python tools/imu_capture_plot.py --port COM7 --seconds 0 --live
# Offline from a text capture
python tools/imu_capture_plot.py --input "imu_log.txt"
```

Output is written under `imu_logs/<timestamp>/`. The firmware's
`NOISE` and `FRESH` records remain 1Hz aggregated statistics rather
than the original 1000 individual samples per second. Binary/raw streaming
would require a separate firmware transport change.

## Fixes after the 60-second KEY recalibration experiment

The **D** diagnostic burst observed after MCU time `2147483648us`
(~35m47s uptime) was a firmware bug: signed deadline subtraction
unexpectedly became positive at the `2^31` boundary, generating
100Hz `D` records while gyro rates were only ~0.17 deg/s.
`src/main.c` now uses an explicit active flag and unsigned elapsed
microseconds since a **real motion trigger** (>10 deg/s) to keep
dynamic traces valid across timer wrap.

`tools/imu_capture_plot.py` now:
- Preserves timestamps when reprocessing its own `raw_serial.log` using
  `--input`; also accepts unprefixed serial text.
- Excludes the pre-KEY yaw session and calibration zero rows if a
  `DRIFT_START` record exists.
- Drops `NOISE,n=0` warmup/collect placeholders instead of reporting
  zero noise.
- Removes the previous completed 4000-sample plateau from plots of
  a new KEY recalibration.
- Suppresses `D` plots when the entire recorded trace is below 1 deg/s
  and prints an informational message about possible old firmware traces.

Example offline reanalysis:
```powershell
python tools/imu_capture_plot.py --input "imu_logs/<session>/raw_serial.log"
```

For rigorous comparisons, capture **at least 130 seconds AFTER
`CAL_DONE`** to include the 120-second `CAL_CHECK`; a 60-second
record beginning before a manual KEY press does not contain it.
The TIM11 period of 1000us is derived from the MCU clock and is not
independent proof of physical 1000Hz; compare to an external timing
reference (logic analyzer/scope) before final timing validation.


## PC vs STM32 clock-rate verification (no oscilloscope required)

**Purpose:** Investigate the observed ~4.5% difference between 1 s of nominal
TIM5 time and the Windows serial-timestamp interval. This is a long-term
frequency comparison, not a precision measurement of TIM11 ISR jitter.
The USART/USB converter and Windows scheduler introduce receive-time latency.

Current diagnostic firmware emits once per nominal MCU second, **before**
the bulk diagnostic output:

`CLK,seq=N,us=TIM5_CNT,ticks=TIM11_COUNT,tx_drop=N`

No UART code runs inside TIM11 ISR; the IMU and motor-off safety settings
are unchanged. USART1 USB–TTL on PA9/PA10 at 115200 baud; TIM5 count wraps
at 2^32 us (~71.6 nominal minutes). Python separates RESET sessions and
unwraps TIM5 rollover.

### Procedure on Windows / PowerShell

1. Update the correct Git branch and build/upload the new MCU firmware:

```powershell
git switch feature/motor-debug-console
git pull --ff-only origin feature/motor-debug-console
python -m platformio run
python -m platformio run --target upload
python -m pip install pyserial matplotlib
```

2. Close PlatformIO Serial Monitor and any other COM7 user.
3. Leave the robot still and motors OFF. Start the recorder **before**
   pressing the hardware RESET button once:

```powershell
python tools/clock_serial_test.py --port COM7 --baud 115200 --seconds 240
```

Do not press KEY or RESET a second time during the measurement. Keep the
USB cable connected, avoid CPU-heavy work on the PC and wait for capture
to finish. The recorder prints one progress message for every 20 CLK
frames. Four minutes is recommended; >=30 usable ticks are the minimum.

4. See the new timestamped folder under `clock_logs/`:

- `clock_samples.csv` — PC perf_counter time + STM32 TIM5/TIM11 timestamps
- `clock_summary.txt` — regression, 30/60s robust checks, jitter
- `01_clock_drift.png` — accumulated PC-minus-MCU time error
- `02_serial_timestamp_jitter.png` — UART/USB host timing residual
- `raw_serial.log` — all received serial lines (not just CLK)

Repeat offline without opening COM7:

```powershell
python tools/clock_serial_test.py --input "clock_logs\SESSION\clock_samples.csv"
```

Built-in synthetic self-check without a board:

```powershell
python tools/clock_serial_test.py --self-test
```

### Interpretation and limitations

- `pc_seconds_per_mcu_second` is the physical time on PC per
  1 nominal TIM5 second. Example 1.045 means a 4.5% longer period.
- `effective_tim5_hz` is TIM5's approximate physical tick rate
  inferred from Windows' monotonic clock. If the ratio is 1.045,
  expected rate is about 956.9 kHz versus nominal 1 MHz.
- `mcu_clock_rate_error_pct` uses frequency as the denominator.
  A +4.5% *period* deviation corresponds to about -4.3% *frequency* deviation;
  do not confuse the two.
- `effective_tim11_hz` gives physical TIM11 interrupts/s;
  ~956–958Hz alongside ~957kHz TIM5 would indicate a shared clock-rate issue.
- `fit_residual_p95_ms` quantifies UART/USB/OS timestamp jitter,
  **not** genuine TIM11 ISR timing.
- A single short UART gap, USB buffering or missed serial records must not
  be interpreted as MCU frequency drift. Use a 240s least-squares slope,
  30s and 60s cross-checks, repeated runs and `tx_drop` checks.
- The PC's monotonic clock is independently sourced but itself imperfect,
  and this is **not metrology-grade**. Confirm with an external oscillator,
  logic analyzer or scope before adjusting hardware clock calibration.
- For safety, do **not** change the STM32 PLL, TIM5 prescaler or PID timing
  merely because this script flags a frequency difference. Diagnose first.

For analysis, send **`clock_samples.csv` and
`clock_summary.txt`** from the same session. If a reset,
UART loss or malformed records are suspected, also send `raw_serial.log`.


## HSI trim A/B experiment (stage 1: factory vs HSITRIM=17)

Baseline: nominal TIM5=1 MHz, TIM11=1 kHz, but two independent PC/serial
measurements of the original HSITRIM=16 configuration found approximately
-4.34% and -4.40% frequency errors. These are **physical frequency
estimates** relative to the PC's monotonic clock, not directly measured by
the STM32 itself.

Per RM0383, RCC_CR.HSITRIM is **bits 7:3** and RCC_CR.HSICAL is **bits 15:8**.
Do not confuse them or write the HSICAL bits; the latter are calibrated by ST.
Default trim is normally 16. The trim response is chip-dependent; never
guess the offset needed for 100 MHz or abruptly change several steps.

**Safety implementation:**
- Standard PlatformIO environment `genericSTM32F411CE` is **unchanged**:
  no HSITRIM override and no attempt to tune normal firmware automatically.
- Opt-in `clock_hsi_trim17` uses `-DCLOCK_HSI_TRIM=17`.
  The trim is applied while **HSI is selected and PLL is disabled**, before
  PLL/100 MHz setup; no live clock switching or motor operation.
- Firmware prints on boot:
  `CLOCK_TRIM,mode=override requested=17 active=17 hsical=<chip_cal> ...`.
  `CLOCK_TRIM,mode=default requested=16 active=16 ...` is the
  expected baseline (always verify the actual field).
- Power/reset alone resets the **hardware trim register**, but the *flashed
  trim17 firmware reapplies it on every boot*. To restore the original
  configuration, **reflash `genericSTM32F411CE`**.
- STM32F411 SYSCLK must not exceed 100 MHz. Stop increasing trim if the PC
  estimate reaches 100 MHz. A host-referenced measurement is approximate,
  so leave margin and do not deploy a trimmed firmware as production.

### One conservative comparison test

1. Fetch and build the optional, motor-OFF trim17 variant with ST-Link:
   
```powershell
git switch feature/motor-debug-console
git pull --ff-only origin feature/motor-debug-console
python -m platformio run -e clock_hsi_trim17
python -m platformio run -e clock_hsi_trim17 --target upload
```

2. Close PlatformIO Serial Monitor; capture one 180-second run:

```powershell
python tools/clock_serial_test.py --port COM7 --baud 115200 --seconds 180
```

3. Once COM7 is open, press RESET exactly once (not KEY).
   **Do not touch the robot** for the rest of the measurement. Check
   `raw_serial.log` for the `CLOCK_TRIM,mode=override requested=17 active=17`
   banner. Absence of this marker means the test did not boot trim17 correctly;
   do **not** compare that run as a new HSI trim measurement.

4. Compare original baseline with newly captured candidate:

```powershell
python tools/clock_trim_compare.py --baseline "clock_logs\20261011_202835\clock_summary.txt" --candidate "clock_logs\NEW_SESSION\clock_summary.txt"
```

Expected sign if HSI trim behaves normally: HSITRIM 17 makes oscillator faster,
so `pc_seconds_per_mcu_second` moves **towards 1.0** and
`effective_tim5_hz` **increases**. Check UART drops and host
fit residual as well. Do not extrapolate to multiple trims before evaluating
this first run.

5. Revert to the original, non-trim build:

```powershell
python -m platformio run -e genericSTM32F411CE --target upload
```

Upload/push only the new `clock_logs` session (CSV, summary and
optionally raw log). Then decide from the **measured** response whether a
second trim step is justified. Do not modify PLLM/PLLN/PLLP, prescalers, PID
period or gyro `dt` before this test.

**Note:** `CLOCK_TRIM` is diagnostic text, so existing
`clock_serial_test.py` still records it in `raw_serial.log`
but it is not one of the numerical `CLK` regression samples.


## Stage 2: HSITRIM=18 after repeatable +1 trim improvement

2026-10-11 measured with PC monotonic/USART, all without missing CLK sequences
or firmware UART drops:

| Nominal HSI trim | Session | TIM5 estimated Hz | TIM11 estimated Hz | MCU clock freq error |
| --- | --- | ---: | ---: | ---: |
| 16 | 20261011_202835 | 956028.315 | 956.047 | -4.3972% |
| 17 (as flashed by test operator; check boot log) | 20261011_211801 | 960808.939 | 960.747 | -3.9191% |
| 17 (as flashed by test operator; check boot log) | 20261011_212343 | 960871.921 | 960.855 | -3.9128% |

Trim17 was repeatable within 0.0066% frequency between runs; both improve
TIM5 by ~0.5% relative to trim16. There is no evidence yet that the response
is linear for larger steps, and the PC estimate is not a traceable
oscilloscope/frequency-counter reference. The original logs did not commit
`raw_serial.log`, so the HSITRIM=17 attribution is based on the
operator's flashing procedure rather than checked `CLOCK_TRIM` boot text.

### Procedure (one extra step, no automatic sweep)

```powershell
git switch feature/motor-debug-console
git pull --ff-only origin feature/motor-debug-console
python -m platformio run -e clock_hsi_trim18
python -m platformio run -e clock_hsi_trim18 --target upload
python tools/clock_serial_test.py --port COM7 --baud 115200 --seconds 180
```

After the COM port opens, press RESET once. Confirm the newest session's
`raw_serial.log` contains
`CLOCK_TRIM,mode=override requested=18 active=18`.
No movement or KEY presses; motors remain disabled.

Compare against trim17 run (baseline here specifically means trim17):

```powershell
python tools/clock_trim_compare.py --baseline "clock_logs\20261011_212343\clock_summary.txt" --candidate "clock_logs\NEW_SESSION\clock_summary.txt"
```

Expected if response continues at the previous +0.5% rate: ~966Hz TIM11 and
~96.6MHz SYSCLK, but this is a **hypothesis, not measured trim18 output**.
Always inspect the candidate's effective SYSCLK and stop any increase if it
approaches or exceeds the STM32F411 100MHz maximum. UART baud also derives
from HSI/PLL; inspect for framing glitches and missed samples.

Revert after test:

```powershell
python -m platformio run -e genericSTM32F411CE --target upload
```

Upload `clock_samples.csv`, `clock_summary.txt` and, for boot
verification, `raw_serial.log` or the single `CLOCK_TRIM` line.
