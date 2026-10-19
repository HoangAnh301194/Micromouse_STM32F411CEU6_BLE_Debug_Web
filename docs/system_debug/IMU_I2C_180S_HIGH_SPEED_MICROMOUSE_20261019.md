# IMU/I2C 180-second yaw drift and integrated load benchmark

**Scope:** STM32F411CEU6, nominal 96MHz HSI trim25, TIM11 1kHz, MPU6050 I2C1, six IR ADC channels and USART1 PA9/PA10 115200 baud.

**Status:** source and all 3 firmware images compile in CI; synthetic host analysis passes. No new 180-second physical result has been recorded; do not claim gyro drift is acceptable until the robot is actually measured.

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
