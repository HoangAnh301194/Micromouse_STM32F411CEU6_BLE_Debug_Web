# USART1 PA9/PA10 + MPU6050/I2C RC2 — scoped integration into giomuadongbac

Date: 2026-10-17. This package is a **new integration revision**. It does NOT rewrite either frozen release.

## References and transfer scope

- Target branch baseline before integration: `giomuadongbac`, `291fee927a60c85d0b6ae077965c1ed0653d28cd`.
- MPU/I2C RC2 pinned tested source: `5219e04d3fe7b8fdb5a092100154e6d9f0317e09`; release branch `release/mpu6050-i2c-rc2-tim11-20261013`.
- USART1 donor branch: `feature/motor-driver-optimization`, `4ed2816f8c9b2dbc0f4adbebbe88dcd1b5dde41c`; 1/2-way UART baseline includes DMA TX, conditionally-enabled RX IRQ.
- Motor Driver Rev3 remains pinned to `b40228e85382e54f2487d8145fa3b0558802065b`.
- **Exact unmodified MPU/I2C RC2 donor blobs:** `src/i2c.c`, `include/i2c.h`, `src/mpu6050.c`, `include/mpu6050.h`, `include/mpu_policy.h`, `src/system_timer.c`, `include/system_timer.h`. The legacy TIM5-only implementation is replaced by RC2's timer-aware 1MHz timestamp + nonblocking TIM5 compare-wake.
- **Modified USART1 donor:** `src/uart.c`, `include/uart.h` are carried forward with **one scope adaptation**: RXNE IRQ ring is also enabled by `UART_DEBUG_RX` for the normal firmware. No motor-auto bench flags are enabled.
- **Original-target integration glue:** `src/control.c`, `src/main.c`, `platformio.ini`; **unchanged** `src/motor.c`, `src/motion.c`, `src/board.c`, `src/ir_sensor.c`, `src/bt_debug.c`, `src/encoder.c`, `include/pinout.h`.

## Actual default firmware behavior

Default build explicitly defines `MPU_SAMPLING_POLICY_B=1`, `MPU_TIM11_TICK_SCHED=1`, `UART_DEBUG_RX=1`. This differs from RC2's motor-OFF A/B test application; default firmware is an **untested motor-enabled integration**.

- UART1 PA9 TX 115200 uses **DMA2 Stream7/Channel4**, ring buffer, drop-full-message policy. PA10 RX uses the USART1 IRQ ring buffer, bounded to **8 bytes/main iteration**.
- UART1 accepts **read-only debug commands** `?` for status, `H` for help. In particular, it does not accept BLE's `F/L/R` commands and does not invoke `Motor_SetPair`.
- UART1 prints 100ms `DBG` with encoder counts, gyro/yaw scaled to integer, gyro-snapshot validity/age and UART drop count. `?` includes I2C timeout/error and RX overrun counters. This does not prove waveform fidelity on PA9, which needs a COM6 hardware readback.
- USART2 PA2/PA3 JDY-33 BLE remains the pre-existing command/debug path. Motor motion commands stay restricted to BLE `F/L/R` and only after calibration and valid MPU snapshot; `S` always stops.
- The RC2 `MPU6050_Calibrate()` call **starts** a nonblocking stationary warmup (5s) and acquisition of **4000 confirmed fresh samples**, not an immediate completed calibration. `MPU6050_Update()` in **main** advances INT_STATUS → gyro acquisition by I2C1 EV/ER IRQs; it does not block TIM11.
- TIM11 ISR `MPU6050_NotifyControlTick()` records a timestamp/sequence, still calls `Motor_Update1ms()` once, consumes coherent `MPU6050_GetSnapshot()` only when valid and <=3ms old, otherwise `Motion_Stop()`. No I2C commands/printf occur in TIM11. The 300ms driver lease and 30ms reversal gap are preserved.
- `I2C_DMA_Init(I2C_1)` is intentionally removed from target main: RC2's normal MPU path is **interrupt-driven, not DMA_RX**.
- TIM5 compare wake for a bounded +220us DATA_RDY retry is supported by exact RC2 `system_timer`. The main loop can `__WFI()` when no immediate gyro poll is needed; high-priority TIM11 wakes it.

## Acceptance and limitations

Prior independent RC2 motor-OFF evidence: 2× reversed 240s A/B pairs, gyro sample reuse around 0.20%→0.10%, no observed I2C errors/timeouts. See donor `docs/mpu_i2c_rc2/README.md`; these captures are **NOT copied into target production branch**, and do not validate combined Motor+MPU noise/race/jitter.

Software gates:

```powershell
python tools/uart_mpu_rc2_integration_gate.py --self-test
python tools/uart_mpu_rc2_integration_gate.py --audit
python tools/motor_giomuadongbac_gate.py --audit
python tools/motor_driver_native_test.py
python -m platformio run -e genericSTM32F411CE
```

Expected `PASS_SELECTIVE_UART1_MPU_RC2_INTEGRATION`, `PASS_MOTOR_DRIVER_GIOMUADONGBAC_INTEGRATION`, and compiled STM32 firmware. CI checks this in both Python 3.11 and 3.14. The test includes 8 negative mutants and verifies donor blob SHA-1 identities.

**Not claimed:** physical PA9/PA10 bidirectional USART1 testing on new runtime firmware; STM32 TIM11 jitter with simultaneous physical motor, IR, I2C and UART traffic; reliable gyro yaw under motor electrical noise; live BLE closed-loop motor behavior; brake/current/waveform safety. These require a distinct hardware qualification before energizing motors solely on the basis of software CI.

Do not full-merge either source branch, replace source C blob without updating/retaining provenance, nor flash unreviewed firmware on a live machine. Use a separate controlled bench test with motors disconnected for the next stage.
