# Motor Debug v3 — driver improvements and BLE test console

Branch: `feature/motor-debug-console`; PR #2 targets `giomuadongbac`, **not merged**.

## Important: RUN TEST disabled / no response

The original Web page required `MTREADY,2` before enabling RUN, but connecting
BLE while the Motor tab was already open did **not automatically send MT PING**.
That made RUN appear permanently disabled.

The updated `tools/motor_debug.js` automatically sends `MT PING` after BLE
connect, displays a 2.2-second no-response diagnostic, and enables RUN after
connection rather than requiring a successful handshake. Firmware v3 responds
`MTREADY,3`; firmware v2 responds `MTREADY,2` and can still run basic
integer-percent tests. If the connection is real but there is no MTREADY,
check the serial RX wiring (JDY-33 TX -> STM32 USART2 PA3), that the latest
branch was successfully **flashed** (GitHub push alone does not flash the
board), and the BLE Log tab. A RUN without a firmware acknowledgment reports
a timeout; it does **not** confirm that a test started.

Use Chrome/Edge Web Bluetooth in a supported secure-origin environment.
Refresh the page after switching branches; the script URL includes a
cache-busting v3 query parameter.

## Firmware changes

- `Motor_SetPairPermille(-1000..1000)`: 0.1% command resolution.
  `Motor_SetPair(-100..100)` is retained.
- `Motor_SetDeadband(left,right)`: optional per-wheel deadband 0..500
  permille; default **zero**. The applied command and final output duty
  are different when deadband is nonzero.
- `Motor_SetSlewRates(rise,fall)`: configurable 1..1000 permille/ms,
  default rise=20, fall=30. `Motor_Update1ms()` runs in TIM11.
- Reversals must ramp command duty to zero and remain neutral at least
  **30 ms** before the opposite command is applied. This is **not**
  a guarantee the rotating shaft has stopped at high speed: an encoder-based
  speed interlock remains a future safety improvement.
- `Motor_Stop()` coasts immediately (overrides slew), `Motor_Brake()`
  short-brakes. `MT BRAKE` pulses brake for **50 ms** then coasts.
  `Control_Stop()` also terminates Motor Test.
- `HardFault_Handler()` now disables TIM2 PWM channel outputs and clears
  direction GPIOs. Without confirmed STBY wiring, this is not a hardware-safe
  guarantee under all fault conditions.
- Encoder speed PPS estimation now uses a rolling **10 ms** window at
  the existing 1 kHz update rate, plus its existing low-pass filter.

## Commands (new firmware)

```text
MT PING
MT CFG <deadLpermille> <deadRpermille> <risePermillePerMs> <fallPermillePerMs>
MT RUNP <leftPermille> <rightPermille> <durationMs> <logMs>
MT SETP <leftPermille> <rightPermille>
MT BRAKE
MT TRACE
MT HB
MT STOP
```

Legacy `MT RUN <leftPercent> <rightPercent> <durationMs> <logMs>`
is retained. Duration 0 means run continuously until STOP or heartbeat
loss (1.5s). Nonzero duration is in milliseconds and has no 1000ms cap.
Normal telemetry is 1–10 Hz, default 5 Hz. Main loop formats and sends
logs; ISR never prints.

Example: wheel left 15.5%, right stopped, 3 s:
```text
MT CFG 0 0 20 30
MT RUNP 155 0 3000 200
```

Example update both target duties during a running test:
```text
MT SETP 300 300
```

The driver applies its slew and reversal interlock to this command.

## Telemetry & high-frequency recording

```text
MTREADY,3
MTACK,CFG,...
MTACK,RUNP,...
MTACK,SETP,...
MTACK,BRAKE
MTDATA3,<elapsedMs>,<cmdL>,<cmdR>,<slewedL>,<slewedR>,<encL>,<encR>,<ppsL>,<ppsR>,<dtMs>
MTEND,TIMEOUT|STOP|HEARTBEAT_LOST|BRAKE
```

`slewedL/R` represents the duty command after slew, **before**
optional deadband mapping. It is not a measured electrical PWM waveform.
Encoder RPM conversion is performed in the browser using user-entered
counts per wheel revolution (default 1430 is unverified).

While Motor Test runs, a fixed-size **512-sample ring in RAM** captures
1 kHz snapshots. No UART I/O is done inside TIM11. After test completion,
press **GET 1kHz TRACE** (sends `MT TRACE`) to transmit one saved row every
20 ms via BLE; then press **Export CSV** to download that trace.

```text
MTTRACE,<sampleIndex>,<cmdL>,<cmdR>,<slewedL>,<slewedR>,<encL>,<encR>
MTTRACEEND
```

Only the **latest 512 ms** remain after a longer test; earlier samples are
overwritten. Trace requests are rejected while a test is running.

## Testing and remaining gaps

1. Build from this branch with `python -m platformio run` and flash with
   `python -m platformio run --target upload`. Confirm you see
   `MTREADY,3`; otherwise the new firmware has not been verified on-board.
2. Lift both wheels clear. Verify forward mapping and encoder sign at
   low PWM before testing high PWM. With a 2S battery (up to 8.4 V),
   motor rated voltage/current is still unknown; 100% PWM may be unsafe.
3. Test left/right separately, then both; watch requested and slewed duty.
4. Test update step and a low-PWM reverse. Avoid high-speed reversals
   until mechanical braking and encoder sign are validated.
5. Test BRAKE and view final encoder response; release should occur
   after 50 ms.
6. Run a 1–3s test, retrieve the 1 kHz RAM trace and check overshoot,
   ramp time, and wheel asymmetry.

**Still not implemented:** independent per-wheel feedforward+velocity PID,
verified voltage/current/stall limits, hardware independent watchdog
(IWDG), STBY control (PCB wiring not verified), encoder-based confirmation
that the wheel has stopped before reversing, and calibrated deadband
values. No motor performance or safe rated duty can be asserted without
physical tests.

**Validation:** code pushed to GitHub; syntax of the standalone browser JS
has been checked, but PlatformIO compilation and real STM32 hardware
tests have **not** yet been confirmed.
