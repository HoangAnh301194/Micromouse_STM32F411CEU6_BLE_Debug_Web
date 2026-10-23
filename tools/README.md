# Flash map pipeline (active exception)

This directory intentionally restores **only** `flash_map_pipeline.py` from
`archive/debug-tools-precleanup-20261024` for on-board map persistence
qualification. All other historical host debug utilities remain archived.

- Web UI: `web/ble_debug_console.html`
- Native contract: `tests/regression/flash_map_protocol_test.py`
- How to run safely: `docs/system_debug/FLASH_MAP_WEB_PIPELINE.md`
- Firmware profile: `flash_map_diag_96m`

Run read-only preview by default. Flash writing requires the dedicated,
motor-locked diagnostic firmware and explicit write/backup flags. Do not
mass-erase sectors 6/7 on upload, and do not confuse a CI PASS with a
successful physical Flash readback/reboot test.
