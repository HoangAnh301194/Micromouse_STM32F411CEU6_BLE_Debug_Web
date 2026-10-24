# BLE PA2/PA3 one-shot qualification — JDY-33

**Status:** Source + automated host parser + offline CI/build implementation; **physical
JDY-33/STM32 bench results are not yet recorded**. Do not call this hardware
qualified until real CSV/JSON has been uploaded. This is not the PA9/PA10 USB-TTL
benchmark and must not be confused with `tools/uart_rate_benchmark.py`.

## One command, no USB-TTL cable following the car

Windows PowerShell, after pulling `giomuadongbac`:

```powershell
cd D:\Micromouse_STM32F411CEU6_BLE_Debug_Web
python tools/ble_full_pipeline.py --name Nezumi_Chan
```

The Python program installs missing `bleak` and `platformio` if necessary,
builds/flashes with **ST-Link**, scans BLE advertising, connects to JDY-33
GATT FFE0/FFE1, subscribes to notifications and sends commands, runs all
motor-OFF cases, writes raw records, CSV and JSON, and re-flashes the
motor-**locked** OFF image in a `finally` block. The host requires a working
Bluetooth adapter; close the Web BLE console before starting because two
applications competing for the same GATT connection can cause failures.

The motor-ON **guarded** variant is also one Python invocation:

```powershell
python tools/ble_full_pipeline.py --name Nezumi_Chan --motor-on --fixture-ready --independent-vm-cutoff --allow-30-percent
```

It performs the OFF tests **first** and only flashes the motor-ON profile
if baseline transport/control/MPU/IR checks pass. The operator must secure
both drive wheels in the air, supervise the setup, and provide a physically
independent VM cutoff. With this command the host additionally arms the
existing 12-stage speed-PID integration sweep (0–30% PWM, 10 ms PID update),
continually sends BLE `PING` each 150 ms to renew the existing 750 ms
firmware BLE lease, and checks for a successful locked-DONE outcome. The
motor driver's independent 300-tick command timeout remains enabled. There is
no BLE-controlled manual high-PWM override and no 100%-PWM motor run.

ST-Link must remain available to the PC through all automatic flashing
phases; **this is a supervised lifted-wheel bench**, not a free-roaming
floor-driving experiment. The BLE data collection itself does not use
a USB-UART cable. When automatic re-flash fails, remove VM power physically
and restore the motor-locked image manually before any further testing.

Use `--address <BLE adapter identifier>` if the advertised name differs;
`--seconds 5` is the default test duration (2–10 seconds allowed). The
`--no-flash` option is intended only to measure an already loaded
motor-OFF 115200 firmware; it cannot launch the 38400/motor-ON suite.

## Matrix: automatic, no individual TXBENCH commands

- Motor OFF 115200: BASE (no packets) + 18 tests T01–T18; sizes
  16, 20, 32, 64, 128 encoded bytes, 10–400 Hz, including deliberate
  overload above 4 KB/s.
- Motor OFF 38400: B01–B06 (six sizes/rates under this UART's nominal ceiling).
- Optional motor ON 115200: M01–M06 over the existing real, supervised
  speed-PID 12-stage wheel sweep. The full-scale encoder, IR and MPU acquisition
  machinery is active in both OFF and ON diagnostics.
- A deliberate overload result is **LIMIT**, and is reported as
  `COMPLETE_WITH_LIMITS`, not a false end-to-end PASS.

This runner currently compares 115200 vs 38400 with interrupt-based TXE.
**DMA TX A/B is not implemented** because USART2 DMA1 Stream6 conflicts
with I2C1_TX ownership in the original source. The pipeline does not silently
claim it has tested DMA. Do not reassign active I2C DMA without a separate
driver, bus-contention and interrupt verification.

## Frame and timestamps

Binary wire frame (little endian): `B5 62 | version(1) | type(1) |
length(1) | session(1) | seq(4) | TIM5_us(4) | payload |
CRC16-CCITT(2)`. Length 16–128 is the **whole UART frame** including CRC.
No assumption that one ATT notification equals one application frame.

For frames of at least 20 B, payload starts with measured encoder left/right
PPS as signed int16. For 32 B, it also encodes gyro Z and yaw at 0.1-unit
resolution, applied motor pair permille and truncated encoder counts.
For 64/128 B, IR six channels, IR scan sequence/completion timestamp and
MPU completion timestamp/sequence are included. Remaining bytes are a
deterministic sequence test pattern. Four bytes of payload cannot encode all
six IR sensors; do not misinterpret a 20-byte transport probe as a complete
sensor snapshot.

`!H`, `!S`, `!D`, `!T`, `!U`, `!B`, `!E`, `!P` are newline-separated
control/health events. `!T` reports a TIM11 ISR runtime histogram with
nominal 4/8/12/16/24/32/48 microsecond buckets; these give **bounds**, not
exact P95/P99. `!U` separately reports UART accepted bytes and TX register
write counts; writing USART2 DR does **not** prove BLE over-air delivery.
TIM5 provides 1 us MCU timestamps, while host uses `perf_counter_ns()`
when notifications arrive. Host arrival-gap percentiles and PING RTT are
reported; **true one-way BLE latency is not measured**, because MCU and
Windows clocks are not synchronized.

## Acceptance/limitations

A rate PASS requires accepted frames arrive with correct CRC, no TX
drops/sequence gaps or generator skips, TX queue peak below 75%, BLE
encoding plus USART ISR measured CPU subset below 3% nominal 96 MHz,
no new TIM11 late>50 us, gap>1500 us, gyro stale/I2C faults, and IR
failures, and evidence that TIM11/IR actually ran. The measured CPU
subset is not a whole-core profiler. Frames deliberately above radio
capacity can fail expected throughput checks; the pipeline identifies
the knee rather than claiming every test passed.

Hardware data must be saved at `results/ble_one_shot_YYYYMMDD_HHMMSS/`:
- `ble_test_matrix.csv`: offered B/s, received frames, lost sequence,
  CPU subset, TIM11 timing, queue backlog and PASS/LIMIT.
- `*_raw.bin`: **exact GATT notification payload bytes** (concatenated).
- `*_frames.csv`: decoded packets and MCU/host monotonic timestamps.
- `*_events.json`: all firmware health, setup and safety events.
- `ble_summary.json`, `ble_qualification.md`: summary for sharing.

The current browser `web/ble_debug_console.html` remains the legacy
text/maze viewer, **not** a decoder for binary benchmark frames. During
this one-shot benchmark use the Python Bleak receiver, not Web Bluetooth.

Offline parser regressions: `python tools/test_ble_full_pipeline.py`.
Firmware build gates are automated by
`.github/workflows/ble-one-shot-validation.yml`; CI cannot exercise
real JDY-33 radio timing, physical motor current, encoder wheel movement,
or independent power cutoff. No model may claim hardware PASS without
actual captured artifacts.
