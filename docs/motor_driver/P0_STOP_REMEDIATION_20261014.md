# P0 STOP/Coast/Brake source remediation — 2026-10-14

## Scope
This is an **unpowered, source-level correction** after v7 PASS_BENCH. The historical v7 traces remain valid for their original LOW/LOW/CCR=0 firmware and **must not** be reinterpreted as proof of a physical high-impedance coast.

## Change
- `Motor_Stop()` now requests IN1=IN2=LOW and PWM HIGH (`CCR=ARR+1`) for both TB6612 channels, matching Toshiba's **settled** Hi-Z Stop truth-table entry.
- `Motor_Init()` initializes non-inverted PWM Mode 1 and deterministic high CCR before connecting pins to TIM2 alternate function.
- `Motor_SetPairPermille()` returns from brake toward defined STOP; `Motor_Update1ms()` keeps direction pins LOW for the first 1-ms tick after an applied zero while the first drive CCR propagates through preload. The reverse gap remains 30 ms.
- Source-only audit now checks STOP_HIZ, explicit timer polarity, initialization, brake, reverse gap and preload guard.
- The existing host v7/v8 parser still expects zero CCR at STOP. Accordingly **new powered motor runs are blocked** at host entry; `--folder`, `--self-test` and offline qualification remain supported. These changes do not constitute an approved new motor bench protocol.

## UART STOP/Coast compatibility (source-only, no powered run)

The existing v4/v6/v7/v8 recorded runs are **archived** as zero-CCR neutral. New Motor Driver firmware uses **v14/v16/v17/v18** respectively, with a settled `CCR=ARR+1=3200` Hi-Z STOP input command (96 MHz TIM2, 30 kHz), not 3200 permille of drive. `MOTOR_AUTO_PROFILE.version` identifies these two different source contracts. The off-line `--folder` parser reads the saved wire version and rejects missing/mixed/unknown versions instead of silently reinterpreting old logs.

Host validation now separates the register contract:
- If `applied=0`, expect the version-specific **STOP CCR** (0 only in archived versions; 3200 in new versions).
- If `applied!=0`, enforce the capped **drive CCR** (up to 640 for 20% base, 1920 for 60% high).
- Static PING snapshot, drive samples with one uncommanded wheel, post-neutral samples, stage END/POST_END and final DONE all use the version-specific STOP CCR.
- Self-tests include a stored 63-stage v7 log, and intentionally mismatched neutral snapshots must fail.
- The standalone diagnostic tool and top-level coordinator still reject new powered motor runs. Firmware build and synthetic parser success do not qualify electrical terminals, current, brake, reverse or PWM preload timing.

## MCU UART P0 arming interlock

A host-side `--run` block cannot prevent somebody sending `MOTOR START` directly to USART1. The new diagnostic firmware therefore hard-locks the **firmware** command dispatcher with `MOTOR_AUTO_P0_ARM_LOCKED=1`: both `MOTOR START` and `MOTOR START HIGH 600` are rejected with `MOTOR_AUTO_ARM_BLOCKED,reason=P0_ELECTRICAL_UNQUALIFIED`. It calls `Motor_Stop()`, never enters `AUTO_SETTLE`, and does not reach `MotorTest_StartPermille()`. The unreachable arm code is behind a compile-time conditional and cannot be activated by a PlatformIO build flag. Host protocol v14/v16/v17/v18 advertises `p0_locked=1` in `PROFILE` and `READY`, which the parser must validate; archived v4/v6/v7/v8 logs remain exempt.

This does **not** reflash a currently deployed board. Previously flashed firmware (including historical v7) can still accept START until replaced or power is disconnected. Nor does this interlock cover other independent firmware entrypoints such as the KEY-armed diagnostic or custom application builds. Maintain a physical VM disconnect. The interlock is not an electrical/thermal/current measurement or an MCU-independent emergency stop.

## Known hazards / outstanding validation
1. With GPIO BSRR taking effect immediately and CCR being preloaded, the brief LOW/LOW/PWM-low transient as drive output is neutralized is not specified by the cited Toshiba truth table. The new startup delay prevents an unintended full-duty PWM pulse before the first limited CCR transfer but is not a substitute for captured waveforms.
2. The actual TB6612 output voltage, short-brake current, driver temperature, motor current and exact 3.3V-to-VCC HIGH-level margin remain unmeasured.
3. The v7/v8 MCU `MOTOR_AUTO_STATIC` and `MOTOR_AUTO_END` telemetry will report `CCR=ARR+1` after STOP, while the old host parser expects zero. A new coordinated protocol version, strict host handshake, self-tests and fresh evidence are required before powered runs can be re-enabled.
4. `Motor_Brake()` is still a physical short-brake command without current qualification; no new brake/reverse experiment is authorised.
5. `HardFault_Handler()` disables TIM2 channel outputs instead of relying on a delayed CCR write. On this board STBY is not an independent MCU-controlled emergency kill.

## TIM2 PWM register-source consistency

A stale `MOTOR_PWM_PERIOD=4999` (from an earlier configuration) was present in `include/pinout.h`. No current Motor Driver source references it. The obsolete period/prescaler macros have been removed to prevent future callers treating 4999 as the actual `ARR`: `Motor_Init()` owns `PSC=0` and derives `ARR=Board_GetAPB1TimerClockHz()/MOTOR_PWM_FREQUENCY - 1`. With the verified 96 MHz timer clock and 30 kHz PWM, `ARR=3199` and settled STOP is `CCR=3200`. An unpowered regression now checks that single-source-of-truth relationship.

## Additional no-power transition regression

`tools/motor_stop_transition_regression.py` validates source operation order and an independent 1-ms wheel/preload model for coast-to-drive, immediate stop, 30-ms opposite-sign reversal, and brake release. GitHub Actions runs it on Python 3.11 and 3.14. This is a host simulation and cannot establish electrical safety or eliminate the unspecified LOW/LOW/PWM-low transient.

Both the direct powered runner (`motor_auto_pipeline.py --run`) and the top-level powered coordinator (`motor_full_qualification.py --run-integrated`) now reject invocation before flashing or driving. Historical v7 PASS_BENCH remains valid ONLY for its older firmware/protocol, not the revised STOP_HIZ source.

## Non-powered on-device UART check

`python tools/motor_auto_pipeline.py --check-link --sensor-integration --port COM6` is now a strictly read-only UART protocol check: it **does not build/upload firmware or send START**, even when `--skip-flash` is absent. With the motor supply VM physically disconnected, the test can check an **already-flashed** diagnostic image. A v17 firmware image must report `p0_locked=1` in BOTH `MOTOR_AUTO_PROFILE` and `MOTOR_AUTO_READY`, and `state=0` in READY. Missing lock, old v7 protocol, or an armed state is rejected; none of those failures justify enabling VM. The source-level regression mocks serial RX/TX, checks valid and invalid handshakes, and fails if `check_link()` attempts an upload. This does not itself flash the updated firmware to the board or validate drive/brake transitions.

## Safe checks
```powershell
python tools/motor_stop_truth_table_audit.py --self-test
python tools/motor_stop_transition_regression.py
python tools/motor_stop_truth_table_audit.py
python tools/motor_full_qualification.py --self-test
python tools/motor_full_qualification.py --audit
```

## Qualification decision
- P0-STOP **source contract: REVIEW** (settled input condition corrected; transitions and physical outputs not validated).
- P0-ELECTRICAL **BLOCKED**.
- P1-INTEGRATION **PASS_BENCH on historical v7 only**; no new powered test. CI passing is source/build/simulation evidence, not measured transients.
- No Motion, PID, IMU accuracy or IR calibration scope changes.

Manufacturer: https://toshiba.semicon-storage.com/info/TB6612FNG_datasheet_en_20141001.pdf?did=10660&prodName=TB6612FNG
