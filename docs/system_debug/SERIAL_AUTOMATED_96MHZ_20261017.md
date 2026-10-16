# STM32F411 96MHz combined-system UART diagnostic — KEY not required

Firmware branch: `integration/system-debug-96m-20261017`; source baseline: `giomuadongbac` at `fcab7519d01fe1a0760a947aa41e82b8babed0de`. No changes to the frozen Motor Driver Rev3 source.

## Serial connection

Use STM32 USART1 PA9 (TX) to USB-UART RX, PA10 (RX) to USB-UART TX, and GND to GND. **3.3 V TTL**, not RS-232 signal levels. `115200 8N1`. ST-Link is used for firmware upload; USB-UART captures telemetry.

## Single-command autonomous pipeline (Windows PowerShell)

Run from the repository root:

```powershell
git fetch origin
git switch integration/system-debug-96m-20261017
git pull --ff-only origin integration/system-debug-96m-20261017

python -m pip install platformio pyserial

# Default: motor power outputs permanently locked at boot; auto-flash and capture.
python tools/system_uart_pipeline.py --port COM6 --flash

# EXPLICIT motor-on opt-in: automatically flash/run OFF baseline, then
# flash/run ON sweep, capture, compare, and save CSV/JSON and raw UART logs.
python tools/system_uart_pipeline.py --port COM6 --flash --motor-on
```

Replace COM6 with the actual Windows COM port. `--flash` requires a connected ST-Link. The script does not run asynchronously; it executes while the PC remains connected. It does not touch hardware in GitHub CI.

**Before issuing `--motor-on`**: secure the wheels off the ground, provide a separate readily accessible cut-off for the motor VM supply, confirm wiring and driver voltage/current suitability, remove obstructions, and arrange supervision of the bench. Serial STOP/heartbeat are software mitigations; they cannot protect against a completely hung MCU or physical driver failure. These tests do not establish electrical qualification at 100% duty.

## MCU profiles

| Profile | PWM output | 96 MHz profile | MPU, I2C, IR, UART, encoders, TIM11 |
|---|---|---|---|
| `genericSTM32F411CE` | Normal firmware | HSI trim25 | Normal |
| `integration_diag_motor_off_96m` | Hard-locked at boot | HSI trim25 | Fully instrumented |
| `integration_diag_motor_on_96m` | 0..100% when explicitly started via UART | HSI trim25 | Same instrumentation |

OFF and ON follow the same 33 commanded stages: LEFT 0..100%, RIGHT 0..100%, BOTH 0..100% in 10% steps; each stage drives 250ms and coasts 150ms. OFF shadows the exact timing without motor demand. No high-level PID runs in the diagnostic (Motion remains idle). UART logs contain TIM11 tick and timing counters, gyro freshness, I2C errors, Motor lock/watchdog, IR scans, encoder count deltas, RX/TX errors.

## UART command protocol (ASCII newline terminated)

`HELP`, `STATUS`, `MODE OFF`, `MODE ON`, `START`, `PING`, `STOP`. Single byte `!` also aborts immediately, no newline needed. **No KEY button press**. The host only uses `MODE ON` and `START` after the operator explicitly passes `--motor-on` to the script, never on connection. `MODE ON` is refused by motor-off firmware. `START` is refused when MPU calibration has not completed, gyro is stale, clock is in fault, or motor outputs are locked. The full sequence is one-shot per flash/reset.

The host sends `PING` every 150ms during the live sweep. If PING stops for >750ms while the main loop operates, or MPU is stale, I2C times out, TIM11 deadlines slip, or watchdog latches, the firmware stops and locks motor outputs. `STOP` is irreversible for the current session even before START. After 33 steps the output is locked until reset. If a whole-system stall prevents both host commands and TIM11/firmware watchdog execution, only an external supply cutoff can be relied on.

## Expected outputs

`results/system_uart_<timestamp>/` with `off_uart.log`, `on_uart.log` when enabled, `measurements.csv`, and `summary.json`. The host flags: 33 distinct stages, clock trim25, I2C 400k nominal, no UART loss/DMA errors, no gyro/I2C errors or TIM11 gaps, no output watchdog latch, and clean DONE/locked state. A PASS is **software/telemetry acceptance, not an oscilloscope-verified signal/physical safety certificate**.

TIM2 generates hardware PWM, TIM3/TIM4 count quadrature encoder edges in hardware. ISR stress is measured as the TIM11 `Motor_Update1ms`, `Encoder_Update` and full control ISR cycle usage rather than imaginary PWM/encoder interrupts.

GitHub Actions builds all three profiles and runs static, negative-mutation and native Motor Driver regression tests. **CI cannot activate the real motor, measure current or validate live EMI.** Do not merge hardware-qualification claims into the main branch without real captured UART logs.
