# Auto-captured Classic SPP logs

The firmware gates periodic MPU and six-IR telemetry on confirmed MPU6050 calibration success. After `CALIB OK`, the MCU emits one MPU and one IR line every 1000 ms (best effort). Before this, only calibration progress events are periodic. The seven-mode PA0 selector is also enabled after calibration; selecting a mode never starts the motors.

Launch: `py tools/classic_spp_dashboard.py --port COM8` (replace COM8 with outgoing SPP COM).

The dashboard automatically creates `logs/spp_runs/connections/` for every COM connection, containing all received lines, and `logs/spp_runs/runs/` for explicit `W,START` to `W,END` runs. An incomplete run is marked `# DISCONNECTED`. The current menu-only firmware has no run start action; therefore only connection logs appear until a real run action is enabled. Existing manual Start log remains optional. Logs are created by the PC, not written to STM32 Flash.
