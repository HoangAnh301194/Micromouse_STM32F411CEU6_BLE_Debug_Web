# v5 High-PWM UART handshake: confirmed root cause and v6 fix — 2026-10-13

**Source session:** `motor_logs/auto_20261013_103631/` (pushed to `feature/motor-driver-optimization`).

## Actual result

- `motor_auto_high_96m_trim25` successfully compiled/flashed.
- Host received **repeated STM32 UART output** (~once every 0.4s after host PING), so the physical COM6 UART **TX from MCU was functioning**.
- Each received line was malformed: `MOTOR_AUTO_PROFILE,version=5,...stby_fixed_MOTOR_AUTO_READY,version=5,...`
- Exactly **0/87** motor starts, **0** encoder/driver telemetry samples, and **0** host `MOTOR KEEP` because the host never sent `MOTOR START HIGH 600`. This is a safe refusal of malformed UART handshake; it contains **no motor electrical-performance evidence**.
- The prior v4 bench had successfully received 192/192 heartbeats on the same hardware. The v5 source/log behavior explains the failure; do not change PCB wiring or diagnose PA10 from this trace.

## Proven software root cause

In `src/uart.c`, `UART_Printf` used a local `char buffer[256]` and ignored the return value of `vsnprintf`. In v5, `ReplyReady()` in `src/motor_auto_bench.c` introduced additional `high_pulse_ms` and `high_post_ms` fields and its `MOTOR_AUTO_PROFILE` frame **exceeded the UART Printf capacity**. `vsnprintf` silently truncated the tail, removing `\r\n`. The next `MOTOR_AUTO_READY` frame therefore appeared concatenated into the same host `readline()` result. The host's field parser categorized the combined line as PROFILE rather than READY and kept sending PING until handshake timed out.

This is a **TX log framing/formatting bug**, not an observed STM32 RX overrun or physical loss of UART wiring.

## Patch details

1. **C firmware v6 (high profile only)**: `ReplyReady()` outputs a compact complete `MOTOR_AUTO_PROFILE` record (actual populated frame about 220 bytes), fitting the 255-byte maximum `UART_Printf` payload; valid CRLF preserved; READY is separately framed. `AUTO_VERSION` is 6 for high mode; baseline v4 still has 21×3 PWM stages and remains version 4.
2. **USART1 output robustness**: `UART_Printf` now checks `vsnprintf` result. If the formatted string is ≥256 bytes or the formatter fails, it **drops the entire frame and increments `tx_dropped_messages`** rather than emitting an unterminated or truncated record. This prevents silent frame-merging failure across future revisions.
3. **Host v6 verification**: `tools/motor_auto_pipeline.py` requires a complete matching PROFILE **before** READY and verifies version, stage count, caps, RX mode, and high-pulse durations. Explicit error when `MOTOR_AUTO_READY` is embedded inside another frame; no START is sent on malformed/incomplete fields.
4. **CI host test**: `test_firmware_uart_profile_frame()` reads the exact C source format string, renders the expected v4/v6 message, checks CRLF and enforces byte count <256. Self-tests also reject a synthetic merged/truncated v5 handshake.
5. All existing software motor safety controls remain (boot neutral, explicit host arm, 200ms base/120ms high pulse cutoff, host heartbeat, TIM11 wall-time and stall guards, encoder movement/settle abort on high stages). **Current/thermal stall protection is still not present**.

## Verification and next action

Confirm Github Actions green on the patch commit. CI proves source, parser and firmware build pass; **the v6 UART handshake and high-PWM movement are not yet hardware-verified**.

After updating branch, one-command host pipeline rebuild/flash (do not set `--skip-flash`):

```powershell
git fetch origin
git switch feature/motor-driver-optimization
git pull --ff-only origin feature/motor-driver-optimization
python tools/motor_auto_pipeline.py --run --high-pwm --confirm-wheels-elevated --port COM6 --vm-state powered --meter-vm-volts 8.2
```

`--meter-vm-volts 8.2` is only declared hand-meter data, not MCU-measured; replace the number if the supply measurement differs. Do not energize the high profile until elevated wheels, free-spinning shafts, controlled power and immediate **physical VM disconnect** are assured. **A 60% duty command is not an H-bridge current limit**, and unknown stall/short current can exceed TB6612 limits before encoder watchdogs act.

Expected initial UART log:

```text
MOTOR_AUTO_PROFILE,version=6,...high_pulse_ms=120,high_post_ms=800
MOTOR_AUTO_READY,version=6,...,state=0
MOTOR_AUTO_BEGIN,rounds=3,steps=29,...
```

Follow with source-grounded 87 pulse statistics only if the experiment completes. If it aborts due to a high-stage guard or current/thermal concern, **do not blindly rerun or override the gate**.
