# Micromouse STM32F411CEU6 - Minimal Core

> **Tiến độ mới nhất và ngữ cảnh bàn giao đa agent:** [PROJECT_STATUS.md](PROJECT_STATUS.md). Agent/chat mới bắt đầu từ [AGENTS.md](AGENTS.md); kết quả nghiệm thu I2C–MPU6050 nằm tại [docs/mpu_i2c_rc2/README.md](docs/mpu_i2c_rc2/README.md).

This branch (`giomuadongbac`) is the STM32F411CEU6 RC2 baseline with opt-in 96 MHz system integration and UART PA9/PA10 diagnostic profiles; the default production build does not run a stress benchmark.

The PCB pin mapping is intentionally unchanged. The goal is to keep the real-time path small, explicit and testable before maze solving, OLED, calibration workflows and advanced motion features are added back.

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
