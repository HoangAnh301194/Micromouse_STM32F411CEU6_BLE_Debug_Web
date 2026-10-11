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

Keep the device absolutely still during the **200 DATA_RDY-confirmed calibration samples** and for at least 120 s after `DRIFT_START`. `FRESH` prints:
- `yaw_deadzone_mdeg`: current LPF + 0.3°/s software deadzone.
- `yaw_lpf_no_deadzone_mdeg`: LPF + calibrated bias but **no deadzone**.
- `yaw_raw_no_deadzone_mdeg`: bias-corrected gyro integrated without software LPF/deadzone.

Degrees = millidegrees / 1000. Compare slope (degrees per minute), not only the last line. Measurements are relative yaw estimates, not absolute heading.

### (3) Known reference turns using KEY

Hold the robot fixed in the same orientation at the start and keep motors disconnected/disabled.

- Press KEY once: `ANGLE_START,target_mdeg=90000`; gyro yaw integrators reset to zero.
- Manually rotate the robot +90° against a **physical** protractor/reference mark.
- Press KEY again: `ANGLE_DONE` prints gyro-integrated angles and errors.
- Next runs cycle through **−90°, +180°, −180°**, then repeat.

Pressing the KEY can mechanically disturb the measurement. Ideally mount the mouse on a marked turntable and use a second operator. Repeat each angle at least 5–10 times; compare signed error, mean absolute error and repeatability. Sign convention should first be confirmed with a small, known-positive rotation. The experiment does not drive any motors.

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
