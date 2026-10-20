# Encoder qualification P0–P3 — single supervised pipeline

**Date:** 2026-10-20  
**Branch:** `giomuadongbac`  
**Implementation:** `tools/encoder_p0_p3_pipeline.py`  
**Status:** Source/CI verification only. No new hardware measurements are claimed.

## Rationale

Last verified `system_uart_20261020_092523` (MOTOR-OFF) measured TIM11
Encoder max 517 cycles at nominal 96 MHz vs baseline 7,211 cycles, zero
USART ORE, zero FIFO losses, 907/907 PING acknowledgements, and full
33/33 motor-off stages. This proves the 1-kHz shadow EMA offload is
effective in the motor-locked configuration; it does **not** prove
motor-on UART reliability or PID tuning.

## One command, four strictly gated tasks

| Step | Physical/test profile | Pass requirement | Failure behavior |
|---|---|---|---|
| Software preflight | Existing native/regression tests and build OFF/ON, plus the new 30%-limited build | Every gate PASS, no MCU flash yet | No physical motor test |
| **P0** | Compile-locked Motor-OFF, 33 logical stages | All 33 checks; Encoder ISR max **less than 2,000 cycles**, TIM11 max less than 3,500 cycles at nominal 96 MHz; UART RX/TX errors zero; FIFO loss/edge zero; PING healthy | **Cannot flash motor-ON** |
| **P1** | Dedicated `integration_diag_encoder_motor_on_limited_30_96m`, **12** actual stages: LEFT/RIGHT/BOTH × PWM 0/10/20/30% | All limited-mode checks PASS; PWM physical duty never above 30%; both wheels respond to 20/30%; UART errors/heartbeat/watchdog/IMU/TIM11/FIFO all healthy | **No full-range run** |
| **P2** | Original full OFF 33 + ON 33 profile, duty PWM 0–100% | Both phases PASS with full safety/Encoder/MPU/UART/queue criteria | No alpha recommendation |
| **P3** | Offline post-processing of the full P2 raw `.log` evidence | Exactly 792/792 A/B metric frames, 10 configured alphas, no duplicate/malformed frames; wheel-level shortlist agreement | Mark P3 unresolved; never change firmware |

**Safety controls preserved:**

- The 30% ceiling is a **compile-time firmware stage table**, not a
  script-only PWM clamp. It cannot issue PWM beyond 30% in P1.
- Before arming P1 or P2, the host reads `CMD_STATUS` with
  `max_pwm` and `total`, rejects mismatched firmware, and does not
  send `START`.
- The existing MCU 750 ms PING lease, one-shot START, sticky
  STOP/FAULT and motor watchdog remain unchanged.
- The host additionally rejects any P1 telemetry indicating PWM >30%.
- Software/firmware cannot replace a mechanically secure stand, an
  independent VM power disconnect and continuous human supervision.
- On all normal/error exits, the wrapper **attempts** a motor-locked
  firmware upload. If restoration fails, it marks
  `RESTORE_FAILED_DISCONNECT_VM`; physical VM must remain disconnected
  until the operator verifies safe firmware.
- No automatic changes to PID gains or production Encoder EMA
  `SPEED_LPF_ALPHA=0.30` are performed.

## Usage

On Windows PowerShell, update the checked-out clean branch first:

```powershell
git status -sb
git fetch origin
git pull --ff-only origin giomuadongbac
```

Check preflight and the dedicated limited profile build **without touching
motors**, using:

```powershell
python tools/encoder_p0_p3_pipeline.py --offline-only
```

For the complete supervised run, only with the robot securely lifted and
guarded, operator present, and a physically independent motor VM cutoff
ready:

```powershell
python tools/encoder_p0_p3_pipeline.py --port COM7 --fixture-ready --independent-vm-cutoff --allow-100-percent
```

This **does** authorize the original full 0–100% PWM profile after P1
passes, including a brief 100%-PWM interval. Never run on a free-moving
robot. Do not run if battery/current/mechanical safeguards are uncertain.
There is no manual confirmation between the 30%-limited pass and the
full-range test; disable the independent VM supply to stop immediately.

## Result layout

New directory: `results/encoder_p0_p3_YYYYMMDD_HHMMSS/`

- `summary.json`, `REPORT.md`: top-level strict gate status, stage
  health, fault origin and motor-locked firmware restoration result.
- `preflight/`: software audits/native regression/build results.
- `P0_P1_limited/`: 33-stage locked OFF and 12-stage limited ON
  `summary.json`, `measurements.csv`, `off_uart.log`, `on_uart.log`,
  `encoder_shadow_off.csv`, `encoder_shadow_on.csv`.
- `P2_full/`: full 33 OFF + 33 ON evidence in equivalent format.
- `encoder_alpha_ab.json`, `encoder_alpha_stage_metrics.csv`,
  `encoder_alpha_samples_10hz.csv`: P2-derived ten-alpha analyses.
- `alpha_pid_decision.json`: provisional shortlist based on shared
  Encoder's 20-ms boxcar proxy; **not independent speed accuracy**.
- `restore_motor_off.log`: record of attempted safe firmware restoration.

Git may ignore UART `.log` files; push them with `git add -f` to
preserve the complete fault and heartbeat trace.

## Interpretation of P3

The rank is a *candidate* for a future **closed-loop** speed-control
trial. Open-loop PWM sweeps cannot establish PID settling time,
overshoot, true RPM error, wheel-ground slip, or controller stability.
Therefore P3 returns
`PROVISIONAL_FOR_PID_FOLLOWUP` or
`WHEEL_RANKINGS_DISAGREE`, never automatic production deployment.
A final PID alpha choice requires repeated runs with matched battery
conditions and closed-loop step tracking. This limitation is intentional,
not a failed software implementation.

## Gate regression

Host-only tests: `python tools/test_encoder_p0_p3_pipeline.py`; original
Encoder/PWM/UART native and parser suites remain active. A separate
`INTEGRATION_DIAG_LIMITED_PWM_30` native build verifies that the real
C state machine cannot emit >30% and completes exactly 12 stages.
GitHub Actions builds the limited and production profiles.
