# Host diagnostic and qualification tools (restored)

All **33 historical files under `tools/`** have been restored to the active
`giomuadongbac` branch from `archive/debug-tools-precleanup-20261024`.
This includes the guarded Flash-map runner, BLE one-shot qualification,
UART, IR, encoder and PID pipelines and host regressions. The archive
remains available unchanged; restoring scripts does not establish hardware
qualification.

- Web UI: `web/ble_debug_console.html`
- Native contract: `tests/regression/flash_map_protocol_test.py`
- How to run safely: `docs/system_debug/FLASH_MAP_WEB_PIPELINE.md`
- Firmware profile: `flash_map_diag_96m`

Run read-only preview by default. Flash writing requires the dedicated,
motor-locked diagnostic firmware and explicit write/backup flags. Do not
mass-erase sectors 6/7 on upload, and do not confuse a CI PASS with a
successful physical Flash readback/reboot test.

Main BLE one-shot entrypoint: `python tools/ble_full_pipeline.py --name Nezumi_Chan` (motor OFF by default). For the full test matrix, safety gates and evidence paths see `docs/system_debug/BLE_ONE_SHOT_PIPELINE_20261024.md`.
