# UART1 PA9/PA10 — throughput, CPU budget and TIM11 contention benchmark

Date: 2026-10-17. Main branch: `giomuadongbac`.
**This is a future hardware test; CI does NOT prove the optimal measured rate.**
All 96MHz/HSI trim25 numbers are PLL-derived nominal until independently measured.

## Current code and theoretical ceiling

UART1 PA9 is **DMA2 Stream7 Channel4 TX** with a **1024-byte** nonblocking TX ring. PA10 receives via USART1 **RXNE IRQ priority 8**, and DMA IRQ has priority 7. TIM11 is priority 0 and runs at 1kHz; main performs printf formatting and enqueue, with **no formatting or blocking UART calls inside TIM11**.

At UART **115200 baud, 8N1**, every byte uses 1 start + 8 data + 1 stop = **10 serial bits**. The raw line ceiling is **11520 B/s**, NOT 115200 B/s. The fixed test `U_PROBE` packet is **75 bytes**, yielding ~**153 packets/s absolute wire-only upper bound** even before existing 10Hz UART motion diagnostics, periodic MPU/TIM11/IR health status, command ACKs and guard margin. A different message length changes this maximum:

```
frequency_ceiling_hz = 115200 / (10 * encoded_message_bytes)
```

**Only the actual benchmark report can justify a tested sustainable frequency on this board.** Avoid changing UART baud or modifying interrupt priorities as part of this sweep; characterize the current 115200 implementation first.

## Firmware and instrumentation

The new opt-in PlatformIO environment is `integration_diag_uart_96m`:
- Sets `CLOCK_VALIDATE_96M=1`, `CLOCK_HSI_TRIM=25`, MPU Policy B, TIM11 scheduler.
- Sets `INTEGRATION_DIAG_MOTOR_OFF=1` — **motor outputs physically commanded locked in firmware at boot**, and UART `MODE ON` / `START` are rejected.
- Adds `UART_PROFILE_DIAG=1`, absent from normal production and motor-on firmware. The TX rate generator does NOTHING at boot and only runs after the PC sends `TXBENCH <Hz>`.
- Each `U_PROBE` frame carries a monotonically increasing sequence to confirm receipt at the PC and detect missing lines or out-of-order data.
- `U_TX` reports attempted probe count, skipped deadlines, bytes enqueued, bytes completed by DMA, TX queue occupancy/high-water mark, dropped messages, formatting CPU cycles, TX DMA IRQ cycles, RX IRQ cycles and DMA errors.
- `U_SYS` reports TIM11 late/gap counts, gyro staleness, I2C errors/timeouts, PA10 received-byte counts, RX overflows/errors and worst-case formatting/IRQ service cycles.
- Only one main-context probe is generated when due, **no catch-up bursts**. MPU/I2C/TIM11 are still active. Health checks which briefly mask interrupts run at most once every **20 ms**, not once per main-loop spin.
- Host additionally sends `PING` via PA10 at **10Hz** during each rate, recording acknowledgments, RX errors, and host-to-MCU-to-host round-trip delay (includes OS/USB latency).

## Commands

```powershell
cd D:\Micromouse_STM32F411CEU6_BLE_Debug_Web
git status
git fetch origin
git switch giomuadongbac
git pull --ff-only origin giomuadongbac

python -m pip install platformio pyserial

# Only UART profiling; NO motor motion test, NO KEY button.
python tools/system_uart_pipeline.py --port COM6 --flash --uart-only

# Optional: direct UART profiler with 5 seconds at each frequency.
python tools/uart_rate_benchmark.py --port COM6 --flash --duration 5

# Later: one combined pipeline; UART safety check first,
# then motor OFF/ON 0..100% endurance. This WILL energize motors.
python tools/system_uart_pipeline.py --port COM6 --flash --motor-on --with-uart
```

`--flash` requires ST-Link. Replace COM6 with the correct USB-UART adapter. STM32 PA9→adapter RX, PA10←adapter TX, common GND; 3.3V TTL, NOT RS-232 voltage. The connected PC must remain present throughout the test; neither GitHub Actions nor the model has access to the Windows COM port.

The default TX rate sequence is **5, 10, 20, 40, 60, 80, 100, 120, 140, 160, 180, 200, 240, 300, 400 Hz** (each 3 seconds, plus UART drain). This exceeds physical capacity on purpose to detect the queue-overflow knee. On-device commands: `TXBENCH 0` stops, `TXBENCH 40` starts 40Hz, `TXSTAT` requests an immediate snapshot; `STOP` or `!` aborts.

## How to interpret

Results are written to `results/system_uart_<timestamp>/uart/` when called through the existing unified pipeline, or `results/uart_rate_<timestamp>/` via the standalone tool:
- `uart_profile_raw.log` — every UART line plus relative PC receive timestamps.
- `uart_rate_rows.csv` — observed frequency, bytes/sec, lost sequence frames, PA10 ACKs, RX errors, TX DMA backlog, CPU occupancy and TIM11 sensor health for each tested rate.
- `uart_rate_summary.json` — pass/fail conditions and the highest **contiguous set** of passing rates, with a conservative tested operating point including ~20% headroom.

A rate is marked PASS only when all these hold:
1. No dropped TX messages / host sequence gaps / missed generator deadlines.
2. PA10 has no RX error/overflow and all **10Hz PING** commands are acknowledged.
3. UART TX ring high-water ≤768 bytes (75% of 1024).
4. CPU time spent in UART printf + DMA TX IRQ + USART RX IRQ ≤**3%** of nominal 96MHz time budget. This is a measured *subset* of CPU work, NOT total instantaneous processor utilization.
5. Zero additional TIM11 gaps/late>50us events, zero additional gyro stale/I2C errors/timeouts.
6. Generated probe frequency ≥95% of the requested rate.

**There is no single universal “max Hz”**; it depends on UART packet size, print formatting, existing load, clock accuracy, host buffering and concurrent motor activity. The benchmark is motor-OFF; verify the selected rate again in the motor-ON endurance pipeline before accepting it under real motion load. Host-side RTT includes Windows scheduler and USB-UART buffering, not just MCU ISR latency.

If the model cannot measure actual PA9 waveform, USART BRR register values and clock are still *nominal*. Tests involving driver heating, battery sag, motor EMI and physical stop behavior are separate.

## Integrated Motor OFF/ON UART diagnostic

The `UART_ProfileStats_t` counters are also compiled into both `INTEGRATION_DIAG` Motor OFF/ON builds, without enabling a TX stress generator. During the longer motor sweeps, `SYS_UQ` emits once per second: queue depth/high-water, bytes accepted, DMA-completed bytes, UART Printf cycles, DMA and RX interrupt cycles, and successful PA10 RX bytes. The host stores this in `summary.json` under `uart_dma_cpu_profile` and `uart_cpu_fraction_of_nominal_sysclk`. These values let you compare log cost **with** motor-load ISR activity versus the motor-locked UART sweep. The production `genericSTM32F411CE` has no additional UART profiling work.

The PC computes a conservative maximum **contiguous** passing tested rate, plus a lower tested rate with ~20% rate margin. Actual CPU budget counts only `UART_Printf` formatting/enqueue and DMA/USART interrupts; periodic parser/poll overhead is not included. A host PA10 PING RTT is end-to-end through Windows/USB-UART and not a pure STM32 IRQ latency measurement.
