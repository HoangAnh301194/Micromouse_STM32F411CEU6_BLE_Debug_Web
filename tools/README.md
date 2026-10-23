# Host Tools

## IR Sensor Timing Capture

`ir_sensor_timing_capture.py` captures motor-off diagnostic telemetry from
USART1 PA9/PA10 and saves the raw UART stream plus a JSON timing summary:

```powershell
python tools/ir_sensor_timing_capture.py --port COM7 --seconds 30
```

Use the actual USB-UART port and physically disconnect motor VM before the IR
test. See [IR timing validation](../docs/system_debug/IR_SENSOR_TIMING_20261019.md)
for the diagnostic record definitions and acceptance scope. The capture
script requires pyserial (`python -m pip install pyserial`).

## BLE Debug Console

[`ble_debug_console.html`](./ble_debug_console.html) is a standalone, receive-focused
host monitor for the robot's JDY-33 BLE link. Its primary view keeps the live 16×16
maze matrix and terminal log visible together. The matrix retains Flood, Path, and
Explored display toggles, robot pose/progress, and optional maze statistics. Drag
the divider between the matrix and terminal to resize either panel; on narrow
screens, the divider resizes the panels vertically. The terminal supports clearing
and exporting the latest 5,000 lines.

### Getting Started

1. Open `ble_debug_console.html` using a Chromium-based browser (Google Chrome or Microsoft Edge).
2. Power on the robot and the JDY-33 BLE module.
3. Click **Connect** and select the robot. After a link drop, use **Reconnect** to
   reconnect to the previously selected device; use **Disconnect** to close the link.

### Notes

- The browser UI currently displays a 16×16 matrix and listens for `CELL:`, `MAZE:`,
  and chunked `MZ0:`–`MZ7:` frames. Keep its matrix size and packet parsers aligned
  with the firmware protocol.
- The terminal is receive-only; arbitrary BLE command transmission is intentionally
  not part of this simplified monitor.
- Browser interaction and packet rendering can be checked without hardware, but
  those checks do not validate BLE throughput, packet loss, or reconnect behavior
  with the physical JDY-33 module.


## Encoder unified qualification (2026-10-20)

Run a complete source/native/build audit, synthetic speed-estimator A/B,
and archived motor-count analysis **without touching hardware**:

    python tools/encoder_qualification_pipeline.py --mode offline

To compile, flash only the dedicated motor-locked encoder firmware,
capture all USART1 diagnostics on COM7 for two minutes, and generate
a single report (requires physically disconnected motor VM):

    python tools/encoder_qualification_pipeline.py --mode full --port COM7 --seconds 120 --confirm-vm-disconnected --flash-motor-off

Outputs are saved in a new results/encoder_qual_* folder, including
REPORT.md, summary.json, raw UART, 1Hz SYS_E CSV, opt-in ~50Hz SYS_EN CSV,
host regression logs, simulated estimator study, and archived PWM review.

The real 33-stage OFF/ON sweep reaches **100% PWM**. It is a separately
gated opt-in, never part of default qualification. Follow the
[Encoder pipeline safety protocol](../docs/encoder/QUALIFICATION_PIPELINE_20261020.md).

### Encoder motor-driven sweep (no hand turning)

With all wheels securely lifted/guarded, an independent VM cutoff available and
an operator present, the following single command runs source/native/build checks,
ST-Link flash, the historical 33-stage OFF and ON sweeps up to 100% PWM,
50-Hz encoder capture and automatic report/plots:

    python tools/encoder_qualification_pipeline.py --mode full --port COM7 --motor-on-sweep --fixture-ready

The `--motor-on-sweep` option includes 100% PWM; there is no separate
high-PWM acknowledgment. Keep the physical motor supply cutoff accessible.
See docs/encoder/QUALIFICATION_PIPELINE_20261020.md for test limitations.

## Diagnostic-only alpha sweep + I2C BUSY evidence

`encoder_qualification_pipeline.py` now includes the `shadow_ab_regression`
gate and a 1-kHz diagnostic-only bank of EMA alpha values
`0.05, 0.08, 0.10, 0.12, 0.15, 0.20, 0.30, 0.40, 0.60, 0.80` plus 5/10/20-ms boxcars
on both wheels. The PWM/motor and STOP safety controls are unchanged.
Production speed EMA `alpha=0.30` and PID inputs are untouched.

When `--motor-on-sweep --fixture-ready` is used on a correctly secured
test rig, the existing 33-stage OFF/ON sweep also records `SYS_AB` at
5 Hz, per-stage 1-kHz `SYS_AM` aggregates, `SYS_IPB` MPU boot-speed
status, and `SYS_IB` I2C BUSY/SR1/SR2/PB8/PB9 evidence.
`encoder_shadow_ab.py` writes `encoder_alpha_ab.json`,
`encoder_alpha_stage_metrics.csv` and
`encoder_alpha_samples_10hz.csv`. A shortlist is **heuristic**, because
the 20-ms count-window reference is derived from the same Encoder signal.

Host-only syntax before any physical test: `python
tools/encoder_qualification_pipeline.py --mode offline`.
Single full physical pipeline:

```powershell
python tools/encoder_qualification_pipeline.py --mode full --port COM7 --motor-on-sweep --fixture-ready
```

Requires lifted/secured wheels, an independent motor VM cutoff and
attending operator. PWM sweeps to 100%. Keep both `off_uart.log` and
`on_uart.log` when pushing results: Git may ignore raw `.log` files.
Do not use `--skip-build` for a first firmware run.

Detailed test/acceptance contract:
`docs/encoder/ALPHA_AB_I2C_MULTIDIAG_20261020.md`.

The ten-alpha follow-up test retains the existing 1-kHz shadow metrics,
but reduces visualization telemetry to 5 Hz, schedules static I2C boot
probes every 10 seconds, and counts USART RX ORE/FE/NE flags separately
in `SYS_UR`. All UART acceptance gates remain strict. The host decoder
supports both the previous seven-alpha data and the refined ten-alpha
data. Do not infer a final production filter from one test run.

## P0→P3 single-command gated qualification (2026-10-20)

`tools/encoder_p0_p3_pipeline.py` now coordinates the entire process:
**software preflight + P0 real Motor-OFF timing/UART/FIFO gate → P1
compile-limited 0–30% 12-stage Motor-ON gate → P2 full 0–100% 33 OFF
+ 33 ON stages → P3 ten-alpha comparison and provisional PID alpha candidate**.
Every stage is fail-closed; firmware production alpha and PID remain unchanged.

Software and capped-profile build only (no MCU/COM/motor):
```powershell
python tools/encoder_p0_p3_pipeline.py --offline-only
```

Actual physical motor run with secured raised wheels, operator present
and a hardware-independent power cutoff:
```powershell
python tools/encoder_p0_p3_pipeline.py --port COM7 --fixture-ready --independent-vm-cutoff --allow-100-percent
```

The last flag explicitly authorizes the **full 100% PWM sweep after P1
passes**. The script verifies the firmware-advertised maximum PWM/stage
count before START, captures original UART logs and restores locked
OFF firmware on exit when flashing remains possible.
Inspect `results/encoder_p0_p3_*/summary.json` and
`alpha_pid_decision.json`; any P3 alpha is only a **candidate** pending
repeatability and closed-loop PID response tests.

See `docs/encoder/ENCODER_P0_P3_SEQUENTIAL_20261020.md` for complete
strict safety/acceptance conditions.
