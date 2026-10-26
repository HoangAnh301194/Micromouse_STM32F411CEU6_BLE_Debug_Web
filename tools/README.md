# Host diagnostic and qualification tools (restored)

The 33 historical `tools/` files were restored from `archive/debug-tools-precleanup-20261024`. At explicit user request on 24/10/2026, eight redundant copies are retired; **26 live files remain** (including this README and the OLED A/B qualification runner). Canonical regression tests live in `tests/regression/` and `tests/web/`, and the unique browser console lives in `web/ble_debug_console.html`. The archive remains unchanged.

Keep the guarded Flash-map runner, BLE one-shot qualification, OLED/MPU hardware acceptance (`tools/oled_mpu_pipeline.py`), UART, IR, encoder, PID pipelines and nonduplicate host regressions; removing them would compromise future requalification. **Never delete or rewrite real `results/` logs**. Software tests cannot substitute for hardware acceptance.

- Web UI: `web/ble_debug_console.html`
- Native contract: `tests/regression/flash_map_protocol_test.py`
- How to run safely: `docs/system_debug/FLASH_MAP_WEB_PIPELINE.md`
- Firmware profile: `flash_map_diag_96m`

Run read-only preview by default. Flash writing requires the dedicated,
motor-locked diagnostic firmware and explicit write/backup flags. Do not
mass-erase sectors 6/7 on upload, and do not confuse a CI PASS with a
successful physical Flash readback/reboot test.

Main BLE one-shot entrypoint: `python tools/ble_full_pipeline.py --name Nezumi_Chan` (motor OFF by default). For the full test matrix, safety gates and evidence paths see `docs/system_debug/BLE_ONE_SHOT_PIPELINE_20261024.md`.


### JDY-33 RX A/B/C qualification — before considering PID tuning

Run `python tools/ble_rx_qualification.py --port COM7 --name Nezumi_Chan`
with ST-Link and COM7 USB-TTL connected, Web BLE disconnected, motor power
safely isolated. A/B isolate receiver enable under identical autonomous
TX load; C writes 16-byte nonce frames via GATT and requires matching
MCU `RXACK` **on UART1** as independent proof of PA3 reception.
Default one-run orchestration restores motor-locked `flash_map_diag_96m`
on exit. Reports live under `results/ble_rx_ab_*`.
See `docs/system_debug/BLE_RX_AB_QUALIFICATION_20261024.md`.
This does not operate motors or change PID gains.

### PA0 gyro turn optimization 90° / 180° (new)

Run the **isolated** `turn_pid_diag_96m` firmware (never normal
`system_test_96m` for this campaign), then:
`python tools/turn_pid_pipeline.py --port COM7 --mode 90 --profile base --runs 3`.
The capture saves 50Hz gyro/rate/PWM/encoder CSV, plots, raw serial and summary
under `results/turn_pid_*`. UART commands select idle mode/profile but **cannot
start motors**; only physical PA0. After accepted BASE tests, compare FAST
(30% maximum PWM). See `docs/system_debug/TURN_PID_PIPELINE_20261026.md`.
Offline host tests: `python tools/test_turn_pid_pipeline.py`.

### Phase 1 motor sweep and Phase 2 tuned pivot

`python tools/turn_pid_pipeline.py --port COM7 --mode motor --profile base --runs 3`
performs **only PA0-initiated** short PWM characterization (six <=26% stages).
After interpreting the logs, use `--mode 90 --profile tuned --runs 1`
for a cautious first 35% TUNED turn. Captures include applied PWM,
encoder speed (PPS), `motor_steps.csv`, gyro timing and normal turn
response metrics. `docs/system_debug/TURN_PHASE1_PHASE2_20261027.md`.
