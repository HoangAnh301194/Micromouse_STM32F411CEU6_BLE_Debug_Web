# IMU/I2C 180-second yaw drift and integrated load benchmark

**Scope:** STM32F411CEU6, nominal 96MHz HSI trim25, TIM11 1kHz, MPU6050 I2C1, six IR ADC channels and USART1 PA9/PA10 115200 baud.

**Status:** source and all 3 firmware images compile in CI; synthetic host analysis passes. Actual 180s STILL and SHADOW hardware measurements have now completed on 2026-10-19; see [the measured performance and actionable analysis](IMU_I2C_180S_RESULTS_20261019_115313.md). **LIVE physical motor phase remains untested.** PASS indicates telemetry/scheduling checks, not intrinsic absence of gyro bias or yaw drift.

## Three modes and how they differ

| Mode | Firmware environment | Duration after MPU calibration | Motor outputs | Actual computation |
|---|---|---:|---|---|
| STILL | `integration_diag_imu_soak_96m` | 180s | Hardware locked | Live MPU/I2C, IR, encoder, TIM11, UART telemetry |
| SHADOW | `integration_diag_full_shadow_96m` | 180s | Hardware locked | STILL plus PID_Compute with synthetic reference/plant at 1kHz |
| LIVE, explicit optional | `integration_diag_full_live_96m` | 180s | Real TIM2 outputs | SHADOW plus limited motor pulses on left/right/both, capped 35%, on at most 2s per 5s, coast at least 3s |

Shadow PID **is only simulated controlled-plant feedback**; its CPU work is real inside TIM11. The MPU and IR sensors are real in all phases. LIVE is actual PWM, **not a real physical speed-run PID trajectory**, and does not validate closed-loop turning. During LIVE, any yaw change may include physical motion and vibration and cannot be interpreted as stationary gyro drift.

## Safe one-command pipeline (Windows / COM7 example)

```powershell
cd D:\Micromouse_STM32F411CEU6_BLE_Debug_Web
git fetch origin
git switch giomuadongbac
git pull --ff-only origin giomuadongbac
python -m pip install platformio pyserial

# Default two safe 180s phases: STILL, then synthetic PID SHADOW.
python tools/imu_soak_pipeline.py --port COM7 --flash

# Additional third 180s phase: real wheel motor pulses, optional.
# Lift and secure the wheels, separate motor VM cutoff, supervise.
python tools/imu_soak_pipeline.py --port COM7 --flash --motor-live

# Review collected CSV without MCU, COM, motor or flash:
python tools/imu_soak_pipeline.py --review results/imu_soak_YYYYMMDD_HHMMSS/measurements.csv
```

The firmware requires successful MPU calibration before SOAK START; yaw is zero-referenced once after calibration. LIVE requires a separate motor-capable firmware image, the explicit SOAK ARM LIVE command, SOAK START, and continuous PING (750ms timeout). The 180s phase ends automatically and locks motor output. STOP and the '!' byte abort at any time, but an external VM cutoff is necessary for MCU lockup or driver failure. Keep the actual robot rigidly stationary for STILL and SHADOW; suspend the wheels for LIVE.

## What will be logged and analyzed

| UART record | Period | Data |
|---|---:|---|
| IM6 | 60Hz | elapsed ms, gyro sample sequence, integrated filtered yaw, yaw without deadzone, raw-minus-bias integrated yaw (millidegrees), filtered gyro Z (milli-deg/s), snapshot age (us) |
| IM1 | 1s | number of successful gyro reads, stale control ticks, I2C errors and timeouts, DATA_RDY-not-ready, max I2C transaction duration, IR scan counter, TIM11 late/gap/max cycles, TX queue and drops, UART RX errors |
| IMN | 1s | bias-corrected raw gyro mean and standard deviation, calibration bias, raw extremes, min/max fresh sample spacing and repeated equal raw samples |
| IMIR | 1s | six latest actual ADC IR channels + scan count; no extra scan or emitter toggling |
| IMPID | 1s | synthetic PID ticks and max CPU cycles, simulated tracking error/command; applied real PWM and encoder count |
| I2CP | start | startup 400/300/200/100k I2C probing: address ACK, WHO_AM_I read result, selected fallback |
| SYS_T/M/U/UQ | 1s | existing system MPU/UART/TIM11/IR health |
| SOAK_BEGIN/DONE | event | start, duration, mode, completion/abort reason |

**60Hz is a transmission rate, NOT a sensor sample rate.** The 1kHz gyro scheduler remains active. The STM32 aggregates all 1ms events and exports counters, maxima and sample-throughput data. The CSV contains individual 60Hz samples, and summary.json computes drift (degrees, degrees/min) for all three integrals, raw gyro noise, worst spacing/age, I2C speed fallback, IR/TIM11/UART/PID health and missed telemetry records.

Software PASS only indicates no specified timing/data faults. Yaw drift needs its own engineering acceptance threshold based on the target speed-run heading tolerance; no arbitrary drift threshold has been imposed.

## Relevant Micromouse prior work and concrete debug lessons

1. **Peter Harrison, Setting up a gyro on the micromouse** [2011](https://micromouseonline.com/2011/05/08/setting-up-a-gyro-on-the-micromouse/): characterize rate sensitivity using measured samples and fast rotation up to very high yaw rates. Here: log raw/corrected gyro, bias, sampling and drift; next compare 90/180/360-degree reference turns.
2. **Micromouse USA, How to calibrate your gyro** [2013](https://micromouseusa.com/?p=541): the residual zero-rate offset accumulates into yaw error, while filtering trades noise against responsiveness. Here: simultaneous yaw integrations with deadzone/no deadzone/raw and 1s gyro noise statistics.
3. **Peter Harrison, Characterising the drive system** [2011](https://micromouseonline.com/2011/05/15/characterising-the-drive-system-on-the-micromouse/): drive PWM step sweeps and compare measured gyro angular-rate transients/gain. Here: bounded motor pulse windows; later a controlled wheel-on-ground turn test with reference angle.
4. **Peter Harrison, Speed affects smooth turns** [2013](https://micromouseonline.com/2013/03/04/speed-affects-micromouse-smooth-turn/): increased speed changes the transition into turns even with similar geometric radius. Here: monitor timing/jitter and gyro delay; later evaluate real turn profiles and cornering.
5. **Ahmed Ben Salah et al. Micromouse 2026** [project](https://github.com/Ahmedd-Ben-Salah/micromouse-2026): debugging included mistaken yaw sensor axis, accidentally subtracting gyro bias twice, wall-sensor cross-talk and stale/implausible sensor values. Here: expose bias, raw gyro and six IR channels independently, rather than relying only on corrected yaw.
6. **NeuroMouse26, AAMC 2026** [project](https://github.com/enkhbold470/neuromouse26): independent IMU/wall/motor tests, heading hold, encoder correction and multi-cell speed profiles. Here: compare full scheduler and sensor readiness. Real trajectory control remains a separate qualification.

## Recommended next diagnostics after the first 180s measurements

- **Gyro filter A/B:** DLPF 42Hz (current setting) versus 98Hz; repeat 180s STILL on the same fixture, then compare actual response lag on externally measured turns. Avoid judging dynamic suitability by low stationary noise alone.
- **Temperature drift:** sample the MPU6050's *actual temperature register* and track bias after a cold boot and different warm-up times. The current Z-axis async I2C path does not yet collect MPU temperature.
- **Real yaw scale tests:** 90, 180, 360-degree signed turns with optical/fixture reference, repeated across speeds; quantify systematic scale error, slip, overshoot and final yaw drift.
- **IR at speed:** ambient-versus-emitter light, sensor saturation/crosstalk, wall threshold hysteresis and missed wall-edge events. IMIR samples all six final ADC readings but does not yet send full raw ambient/on phases.
- **Supply and motor EMI:** monitor battery/VM sag, driver current and temperature independently; none are established by these UART logs.
- **Repeatability:** several 180s runs with varied battery voltage, temperature and load; compare mean, quantiles and worst-case faults rather than a single PASS.

The 60Hz UART benchmark was previously established only for approximately 75-byte probe records. If IM6 plus existing 1Hz telemetry approaches the TX-ring limit, the PC report must flag queue/drops; do not assume 60Hz is universally safe for larger messages.


## Troubleshooting: apparent hang after successful ST-Link upload

ST-Link `Programming Finished / Verified OK` confirms FLASH, **not** MPU calibration, USART1 reception, or the subsequent 180s experiment.

With the updated `tools/imu_soak_pipeline.py`:
- The PC prints that COM7 is open at 115200 8N1, then once-per-second `SYS_M` fields: `cal_state`, calibration sample count, successful gyro reads, I2C errors, timeout and gyro sequence.
- Normal first calibration is about 5 seconds warm-up + 4 seconds for 4000 gyro samples at 1kHz (~9s), plus boot overhead. Retries may extend this.
- If the PC sees **no SYS_M in 15s**, it stops with recent raw UART lines. This distinguishes a missing UART / early MCU boot path from ordinary gyro warm-up.
- `cal=4` means firmware calibration rejected all attempts; the host exits immediately instead of waiting for 60 seconds.
- The calibration timeout remains **60s** for slow sampling/retries; if this fires, inspect `still_uart.log` for `SYS_CAL` and `SYS_M`.
- If MPU register probing fails **before** TIM11 starts, firmware now emits `SOAK_BOOT_FAIL` on PA9, including step/register/I2C status (the previous code only sent a BLE failure).
- After `SOAK_ACK`, the program prints progress every ~10s until `180/180s`. Then the SHADOW phase flashes and performs its own calibration and 180s test. This means an entire two-phase run lasts >360s plus uploads/calibration.

**On Windows if nothing changes:** press Ctrl+C once, inspect the newest directory under `results/imu_soak_*` and its `still_uart.log`. If it contains no `SYS_M` frames, check COM number, PA9 to USB-UART RX, PA10 to USB-UART TX, common GND, 3.3V TTL, and whether the program stops during I2C initialization. If it does have SYS_M frames, inspect `cal`, `n`, `i2cerr` and `to`. Never assume MCU hardware is frozen just because console output is quiet.


## UPDATE — MPU die temperature on existing I2C and 80–100% motor EMI experiment

**Temperature needs NO extra MCU pin.** The on-chip MPU6050 temperature registers are `TEMP_OUT_H=0x41` and `TEMP_OUT_L=0x42`; datasheet conversion: `temperature_C = signed_raw / 340 + 36.53`. Firmware now obtains this via a **single extra two-byte asynchronous I2C read about once per second**, using existing I2C1 after a successful gyro read. It does **not** read in TIM11 or replace the 1kHz gyro acquisition. This is **MPU silicon temperature**, not motor, H-bridge or battery temperature. Temperature readings are tagged with their own valid/read/error/timeout/age fields and reported on `IMN`:

```text
IMN,...,tc=28210,tvalid=1,tok=17,terr=0,tto=0,tage=148
```

Here `tc` is **milli-degrees Celsius**, e.g. 28210 = 28.210°C. The host's `temperature_c` summary includes start/end/min/max/delta and Pearson correlation between die temperature and each 1s bias-corrected gyro mean. Correlation from a single boot is **not causation** and does not independently prove a temperature compensation model.

**USART1 queue correction:** During active soak, simultaneous redundant `SYS_T/M/U/UQ/E` 1Hz bursts are suppressed. The `IM1/IMN/IMIR/IMPID` metrics remain, with an added `qp` peak queue indicator reset at SOAK start. New pipeline checks treat `qp >768 B` as insufficient TX headroom even if no packet is lost.

**Motor state:** Earlier default command exercised **STILL and SHADOW only**, which *intentionally lock motor outputs* and therefore provide no powered-motor EMI comparison. The optional `--motor-live` uses a separate real motor firmware with pulses ≤35% duty; to explicitly test higher motor demand, an additional dedicated image is now available:

```powershell
# Safe default: temperature + 180s gyro stability, no motor energy
python tools/imu_soak_pipeline.py --port COM7 --flash

# Includes both safe profiles then separate low-duty LIVE (max 35%)
python tools/imu_soak_pipeline.py --port COM7 --flash --motor-live

# Includes both safe profiles then separate HIGH-PWM LIVE (80, 90, 100%)
# Never run on the maze. Secure wheels clear of all surfaces and use
# an independent manual switch to disconnect 2S VM immediately.
python tools/imu_soak_pipeline.py --port COM7 --flash --motor-live --high-pwm
```

**HIGH-PWM stage profile (one 30s cycle, repeated within 180s):**

| 5s slot | Motor command | Max energized duration | Minimum coast/rest |
|---|---|---:|---:|
| 0 | OFF both | 0s | 5.0s |
| 1 | LEFT 80% | 1.0s | 4.0s |
| 2 | RIGHT 80% | 1.0s | 4.0s |
| 3 | BOTH 80% | 1.0s | 4.0s |
| 4 | BOTH 90% | 0.75s | 4.25s |
| 5 | BOTH 100% | 0.50s | 4.50s |

The motor rate is ramp-limited by existing Motor Driver software; the sequence is open-loop **actual TIM2 PWM**, while synthetic PID computations run separately and do not control that PWM. The experiment does **not** run motor at 100% continuously for 3 minutes.

The MCU sends `IMDRV` every ~250ms, recording requested duty, **applied duty**, encoder counts and stage, with immediate abort (`SOAK_DONE ok=0`, reason 5) if a driven wheel fails to advance ≥10 encoder counts within ~300ms. Missing PA10 PING >750ms, I2C read error, stale gyro/TIM11 fault, motor latch or host STOP also aborts and locks outputs. **These software safeguards cannot detect short-circuit current, driver overheating, wiring failure or a stalled motor faster than ~300ms.** A 2S VM supply is not proof of available measured motor current or that the actual wheel rotates. Supervise the fixture, no fingers near the wheels, and keep an **independent physical VM cutoff**; high duty must not be tested against a loaded/blocked wheel.

Acceptance requires high levels (800/900/1000 permille) to appear as **applied PWM** in the UART trace, plus encoder movement; a PWM command or CI PASS alone is not sufficient evidence of actual rotation. Hardware validation is pending. The 180s STILL yaw drift metric must never be compared as "stationary bias" against LIVE if the robot body can rotate or vibrate.

**Grounding:** [TDK/InvenSense MPU6050 register map](https://invensense.tdk.com/wp-content/uploads/2015/02/MPU-6000-Register-Map1.pdf), section 4.18, temperature registers and °C formula.
