# PlatformIO test environment cleanup — 2026-10-27

Branch: `giomuadongbac`.

## Decision

Move previously used, now-retired qualification environments out of the
**default** `platformio.ini` to an explicit archived configuration
`platformio_legacy.ini`. No source modules, diagnostic evidence, Flash
sectors, hardware calibration or historical logs were deleted.

The default config now lists **12 active environments**, down from 21.
**9 historical environments** are available only via explicit `-c
platformio_legacy.ini`. The original build flags for *all twelve retained
environments* are preserved; `ble_profile_motor_on_pid_115200_96m` now
inherits the same flags directly from `integration_diag_motor_on_96m`
rather than indirectly from the archived `speed_pid_alpha_030_96m`.
Regression `tests/regression/platformio_profile_cleanup_test.py`
asserts parity after substitution.

## Current active `platformio.ini`

| Profile | Why retained |
|---|---|
| `genericSTM32F411CE` | Production firmware (not a diagnostic; no automatic motor commands) |
| `ir_filter_diag_96m` | Static IR validation/40 Hz logger retained until full acceptance |
| `ir_burst_diag_96m` | PA0/PC13 high-rate IR dynamic capture, dynamic step experiments pending |
| `system_test_96m` | PA0 integrated motion trials (not fully hardware-qualified) |
| `turn_pid_diag_96m` | 90/180-degree motion retest and fault investigations |
| `flash_map_diag_96m` | Safe read/qualification of calibration/maze Flash layout |
| `ble_rx_diag_96m` | BLE RX integrity still unverified |
| `ble_profile_motor_off_115200_96m` | BLE wireless throughput/integration A/B incomplete |
| `ble_profile_motor_off_38400_96m` | BLE baud-rate A/B incomplete |
| `ble_profile_motor_on_pid_115200_96m` | Guarded experimental BLE PID link; not approved for unrestrained travel |
| `integration_diag_motor_off_96m` | Parent/safe fixture for BLE motor-off |
| `integration_diag_motor_on_96m` | Parent of BLE opt-in motor-on diagnostic |

The motor-capable BLE profiles remain **strictly opt-in**. Their retention
is not a statement that live BLE PID tuning or floor tests have passed.

## Archived historical profiles

| Historical mode | Original purpose |
|---|---|
| `integration_diag_uart_96m` | UART1 rate/stress qualification |
| `integration_diag_encoder_motor_off_96m` | Encoder motor-off baseline |
| `integration_diag_encoder_motor_on_96m` | Encoder motor-on sweep |
| `integration_diag_encoder_motor_on_limited_30_96m` | 30% PWM safety gate |
| `speed_pid_alpha_008_96m` | Encoder speed EMA=0.08 A/B |
| `speed_pid_alpha_010_96m` | Encoder speed EMA=0.10 A/B (selected/default in source) |
| `speed_pid_alpha_030_96m` | Encoder speed EMA=0.30 A/B |
| `ir_oled_test_96m` | OLED/IR isolated viewer |
| `oled_mpu_diag_96m` | Shared-I2C MPU/OLED A/B test |

**Historical replay, motor power disconnected unless an independent
supervised raised-wheel test explicitly permits it:**

```powershell
python -m platformio run -c platformio_legacy.ini -e oled_mpu_diag_96m
python -m platformio run -c platformio_legacy.ini -e speed_pid_alpha_010_96m
```

Add `-t upload` only when the physical setup and safety interlocks are
verified. Flash uploads must preserve sectors 6 and 7 and any critical data.

Historical experiment scripts invoking archived images were updated
to select `platformio_legacy.ini` explicitly:

- `tools/encoder_qualification_pipeline.py`
- `tools/encoder_p0_p3_pipeline.py`
- `tools/system_uart_pipeline.py` (encoder telemetry profiles only)
- `tools/speed_pid_final_pipeline.py`
- `tools/oled_mpu_pipeline.py`
- `tools/system_integration_gate.py` (historical source audit)

Historical `docs/**/*.md` lines containing short
`platformio run -e <retired>` examples must explicitly use
`-c platformio_legacy.ini` to be run on the new branch.
Do not treat a historical diagnostic example as a recommendation to
flash that image instead of the current firmware.

## Verification

```powershell
git pull --ff-only origin giomuadongbac
python tests/regression/platformio_profile_cleanup_test.py
python -m platformio run -e genericSTM32F411CE
python -m platformio run -e ir_filter_diag_96m
python -m platformio run -e ir_burst_diag_96m
```

CI workflow `.github/workflows/firmware-regression.yml`
no longer builds retired OLED firmware as part of the default
regression, but retains existing native filter, driver, motion, Flash,
and main firmware checks. The old OLED host parser self-test may still
be run independently without flashing.

### Limitations

* Archive compiles **current source** under the old flags; historical
  results remain in Git and are not recreated just by rebuilding.
* This is configuration cleanup only. No new Wall PID code, BLE
  logger, Flash IR persistence or hardware test was introduced.
* Selected **IR filter EMA alpha=0.10** remains in
  `src/ir_wall_signal.c`. Encoder speed EMA=0.10 is controlled by
  the runtime encoder source; removing historical speed-PID A/B modes
  does not change the selected runtime alpha.
