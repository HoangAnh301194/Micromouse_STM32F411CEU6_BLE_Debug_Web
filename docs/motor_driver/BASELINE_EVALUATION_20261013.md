# Motor Driver PWM/Encoder Baseline Analysis — 2026-10-13

**Evidence commit:** `5b81396ed61b741506e2244469ddd2e771d9ef56`
**Session:** `motor_logs/pwm_20261013_091405/`
**Status:** `TRACE_COMPLETE_NOT_ELECTRICAL_APPROVAL` (host report); physical motion additionally reported by operator.
**Firmware:** `motor_pwm_key_bench_96m_trim25`, `MOTOR_PWM_DIAG`, nominal SYSCLK 96MHz, TIM2 PWM register-derived 30kHz, 1kHz TIM11.
**Supply:** reported powered VM; DC voltmeter 8.2V. N20 motors 12V rated; current/stall unknown, TB6612 VCC 5V/STBY tied 5V, MCU GPIO 3.3V. No electrical-current or oscilloscope trace.

## Experiment record

Two earlier sessions also committed, but they are **not** physical motor baselines:
- `pwm_20261013_090244`: VM disconnected, no complete motor pulse sequence.
- `pwm_20261013_090858`: VM powered, but no `MOTOR_START` records. No arm/KEY testing completed.
The third `pwm_20261013_091405` is the first complete 5-pulse motor response capture.

| Stage | Command (left/right) | Pulse duration | End L encoder | End R encoder | Observed CCR L/R while active | Response |
| --- | --- | --- | ---: | ---: | --- | --- |
| 0 | +8% / 0 | 200ms | +71 | -1 | 256 / 0 | Left rotates; inactive right has -1 count |
| 1 | 0 / +8% | 200ms | 0 | +1 | 0 / 256 | Insufficient evidence of **sustained** right motion |
| 2 | +12% / 0 | 200ms | +138 | 0 | 384 / 0 | Left rotates clearly |
| 3 | 0 / +12% | 200ms | -1 | +125 | 0 / 384 | Right rotates clearly; left has -1 count |
| 4 | +12% / +12% | 200ms | +135 | +136 | 384 / 384 | Both respond almost equally |

All five pulses have `MOTOR_START`, exactly 200ms end with `reason=0`, and end in software-reported `CCR1=CCR2=0`. Samples logged every ~20ms plus a final 200ms stop state, 11 samples per stage (55 total). No `MOTOR_FAULT`, TIM11 `gap_over_1500us`, TIM11 observed `late_over_50us` or UART TX dropped messages occurred in logged session.

Observed TIM11 max ISR DWT cycles = **1618**, or ~16.85µs at **nominal** 96MHz CPU core clock. This is an *observed maximum*, not a formal WCET. PWM CCR readback only validates timer register programming; no oscilloscope was available to independently measure the physical waveform and output MOSFET switching.

### Significant conclusions

1. **Positive:** driver PWM command→TIM2 CCR→observed encoder motion works at 12% for both wheels with no recorded timing faults; in simultaneous 12% pulse L=135, R=136, only 1 count difference.
2. **Nonlinear low-end:** the left wheel produced 71 counts at 8%; the right wheel produced only 1 count. **Do not claim both motors start reliably at 8%, and do not fix right deadband to a guessed constant.** One right-wheel count might be noise, a rotor twitch or very slow motion; it does not show sustained rotation.
3. **Inactive wheel small counts:** -1 encoder count on noncommanded wheel in stages 0 and 3, plausibly electrical pickup, quantization/glitch or passive movement. Data do not identify the cause. Avoid calling this a motor cross-drive fault without further tests.
4. **First capture limitation:** no repeated trials, no reverse rotation, no after-stop encoder trajectory, no stall/coil current measurement, no supply transient waveform; cannot infer safe motor current or full control robustness.
5. **Measurement logic caveat:** `motor_response_observed=true` and per-stage `any_encoder_count_changed=true` in old host analyzer are broad flags, **not per-commanded-wheel robust rotation certifications**. Future parser should distinguish commanded-wheel count threshold and repeatability from any nonzero count on either wheel.

## Next low-risk measurement gate (same fixed PCB)

**Do not start by increasing PWM, running 100% or changing the driver mapping.**

- **R0 repeatability:** repeat isolated left and right 8% and 12% 200ms pulses at least three times each, with wheels elevated, same battery state and manually recorded VM DC. Change order L/R in alternating sets to minimize drift/initial rotor-position effects. Existing one-shot harness is intended for five stages per reset; extending it to a new explicit R0 profile should be a separate, reviewed change.
- **R1 threshold discrimination:** if safe electrical behavior and R0 are satisfactory, interpolate at e.g. 10% for both wheels, still short and explicitly armed. Derive repeatable start/no-start threshold and delayed first encoder tick distribution. Duty is NOT current-limiting.
- **R2 zero-state verification:** add 250–500ms *encoder-only post-stop sampling*, PWM kept neutral; quantify wheel inertia/coasting and false tick pulses. This does not require reverse drive or electrical brake testing.
- **R3 fault-check:** after R0/R2, test KEY early release and watchdog stop (initially VM disconnected); fail run report intentionally on early stop while recording evidence. Do not use fault-injection that stalls/shorts an energized motor.
- **R4 subsequent motor+IMU coexistence:** perform integrated motor load vs RC2 MPU freshness timing only after standalone motor characterization.

Measure actual **right 8% response and inactive-wheel encoder -1 count** before any PID, per-wheel deadband mapping or high-speed Motion tuning.

## No hardware modification

PCB cannot be rewired. A known risk remains: at TB6612 VCC=5V, `VIH(min)=0.7VCC=3.5V` is above nominal 3.3V STM32 HIGH. Successful motor movement on current specimen documents practical operation under these conditions but does not make the level electrically guaranteed. STBY is held HIGH so firmware cannot assert a separate standby hardware cutoff.

**Source links**:
- [Raw UART](../../motor_logs/pwm_20261013_091405/raw_serial.log)
- [Per-stage samples CSV](../../motor_logs/pwm_20261013_091405/motor_samples.csv)
- [Machine report](../../motor_logs/pwm_20261013_091405/motor_pwm_report.json)
- [Test run metadata](../../motor_logs/pwm_20261013_091405/session_meta.json)
