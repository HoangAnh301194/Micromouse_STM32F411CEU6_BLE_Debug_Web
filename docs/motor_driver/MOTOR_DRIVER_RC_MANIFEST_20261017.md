# Motor Driver — Software Release Candidate Manifest (2026-10-17)

**Repository:** `HoangAnh301194/Micromouse_STM32F411CEU6_BLE_Debug_Web`
**Working branch:** `feature/motor-driver-optimization`
**Code baseline:** `b40228e85382e54f2487d8145fa3b0558802065b`
**MCU evidence commit:** `a5b67de451702f2ce92613affdf28bd6ea348eaa`
**Driver revision:** `MOTOR_DRIVER_REVISION=3`
**Candidate state:** **SOFTWARE_RC / MCU_IDLE_ATTESTED**
**Full electrical, loaded, continuous running release:** **BLOCKED — NOT QUALIFIED**.

## Scope and freeze

Software handoff freezes Motor Driver P0 changes, bench-local v4 UART
protocol and no-power diagnostics as of the code baseline above.
The evidence commit adds measured logs, but does not modify firmware source.
This manifest is not a claim that `main` contains the candidate; do not
merge, tag a production release or flash normal production firmware by
implicitly treating this documentation as sign-off.

**In scope**: `src/motor.c`, `include/motor.h`, explicit motor test/startup
bench interfaces; TIM2 PWM/direction ownership; 1 kHz TIM11 service
contract; 300 ms command lease; STOP/preload/reverse/brake source behavior;
host-only test tooling and UART protocol checks.

**Out of scope**: Motion/PID tuning, encoder calibration, IR accuracy,
IMU performance, BLE radio link, battery/power measurement, thermals,
board STBY current cutoff, general application readiness and research
high-speed physical qualification.

## Frozen configuration (do not conflate build environments)

| Property | Software contract |
| --- | --- |
| MCU | STM32F411CEU6 |
| H-bridge | TB6612FNG |
| PWM | TIM2 channels 1 and 2; active-high PWM mode 1; target 30 kHz |
| Update scheduler | TIM11 nominal 1 kHz; `Motor_Update1ms()` exactly once per serviced tick |
| Right PWM/direction | PA15/TIM2_CH1; PB15/PB14 |
| Left PWM/direction | PB3/TIM2_CH2; PB13/PB12 |
| API | Signed permille `[-1000,1000]`, legacy percent `[-100,100]` |
| Runtime timeout | `MOTOR_COMMAND_TIMEOUT_MS=300`, latched software STOP while TIM11 runs |
| Ramping defaults | Rise 20, fall 30 permille/tick; opposite-sign reverse gap >=30 ms |
| Bench host UART | USART1 PA9/PA10, 115200 baud; protocol v4 for high speed |
| `motor_speed_96m_trim25` | Nominal 96 MHz, `ARR=3199`, `CCR=3200` neutral, experimental & unqualified |
| `genericSTM32F411CE` | Default **separate** build, 100 MHz nominal profile, not v4 benchmark image |

**Important:** `CCR=3200` is the observed neutral for the isolated
96 MHz bench (`ARR+1`). Production/default images must derive neutral
from their configured TIM2 ARR rather than hard-code 3200.

## Evidence and provenance

### 1. No-hardware software gates — PASS

- GitHub Actions workflow `Motor PWM Diagnostic Build and Host Gate`,
  [run 38944406079](https://github.com/HoangAnh301194/Micromouse_STM32F411CEU6_BLE_Debug_Web/actions/runs/38944406079):
  **48/48 steps successful**, on the source baseline `b40228e`.
  Includes native C register mocks, source-level STOP transitions, MCU
  build environments, startup UART parser, safety guards, Python 3.14.
- User Windows host: `motor_driver_native_test.py` PASS and
  `motor_startup_pipeline.py --self-test` **33/33** tests PASS
  after previous P0 fixes.
- Following read-only host parser fix: user Windows
  `motor_startup_readonly_check.py --self-test` **11/11** PASS.
- This is **software-level evidence**. Register mocks cannot measure
  actual PWM edge transitions, bridge current or temperature.

### 2. Physical MCU, VM physically disconnected — PASS / idle only

Recorded raw evidence (versioned, do not overwrite):

`motor_logs/startup_readonly_20261017_010627_618666/`

Files: `commands.log`, `raw_uart.log`, `report.json`.
Commit: `a5b67de451702f2ce92613affdf28bd6ea348eaa`.

- `status=PASS_INSPECTION_ONLY_IDLE_V4`, `firmware_version=4`
- `experimental=true`, `electrical_qualified=false`
- Exactly **one** `STARTUP_STATUS` responding to **one** `STARTUP PING`
- **Six** disarmed, neutral telemetry samples over **200 ms**
- TIM11 tick trace: **444800 to 445000** (200 ticks), no software
  timing gap/late flags
- UART counters: `tx=0,rx=0`; state `0` (DISARMED), fault `0`
- Directions `al=0,ar=0,odr=0`, PWM compare registers
  `cl=3200,cr=3200` (for this 96 MHz bench)
- Zero encoder changes in inspected samples.
- Report says motor VM was **user-confirmed physically disconnected**.
  Host cannot independently verify VM wiring.

Raw UART SHA-256:
`c3608449a62ec264827d7a5792dc67dba785ffb77655a57b3280ee3e395ebde1`.

**Interpretation**: Physical STM32 executed the v4 idle UART profile and
registered-neutral output commands while TIM11 advanced. No motor was
commanded to run by this check. Real H-bridge output voltages/current
remain unmeasured, so the result is not a full electrical qualification.

### 3. Historical elevated-wheel high-speed evidence — research only

Older v3 captures on 2026-10-16 measured bounded ~600 ms high-speed
pulses with elevated wheels, as documented in
`HIGH_SPEED_CHECKLIST.md` and original
`motor_logs/startup_speed_20261016_080553_270935/`. These captures
predate the v4 safety ACK changes and **must never be relabeled as
v4-validated**, production load, continuous operation or electrical
qualification. The operator suspected lower 2S battery voltage during
the last speed test; synchronized VM measurement was not recorded.

## Release gates

| Gate | Current disposition |
| --- | --- |
| P0 command watchdog software / sticky lock | PASS_NATIVE; **MCU timeout test pending** |
| STOP/preload/reverse/brake source regression | PASS_NATIVE; **electrical transient test pending** |
| High-speed v4 explicit operator + firmware ACK | PASS_HOST_CI; **powered v4 test not requested** |
| Physical MCU disarmed UART/clock/neutral | **PASS_IDLE_MCU_200MS** |
| Timed 600 ms bench after P0 integration | Simulated/CI tests only; v4 powered characterization **not done** |
| Full Motion/PID + motor runtime integration on robot | NOT_VERIFIED_AFTER_P0 |
| Real output waveform, VM sag, running/stall current, thermal, loaded stop | NOT_MEASURED |
| Independent power cutoff / STBY interlock | NOT_VERIFIED |

**Disposition:** Accept software RC for integration planning and
VM-disconnected validation. **Do not approve powered deployment or merge
as production-safe based solely on the above.** Preserve a separate
explicit electrical and system integration review before any such
approval.

See [API contract](MOTOR_DRIVER_API_CONTRACT.md) and
[handoff checklist](MOTOR_DRIVER_HANDOFF_CHECKLIST_20261017.md).
