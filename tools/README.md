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
