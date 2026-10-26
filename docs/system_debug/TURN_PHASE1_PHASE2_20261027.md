# Micromouse Pivot — Phase 1 + Phase 2 tuning (2026-10-27)

**Code branch:** `giomuadongbac`, opt-in PlatformIO `turn_pid_diag_96m`.
**No autonomous operation**. USART1/COM7 selects mode, then **only a physical
PA0 press-release** energizes motors. A second PA0 press hard-stops.
External VM cutoff required. Keep robot free of walls and people; clear
floor for physical yaw; lift wheels only for initial motor-direction safety.
Stop if wheels, wiring, gearbox or motor driver heat abnormally.
Do not tune on a rigid fixture; the IMU cannot measure actual chassis yaw.

## Source-data assessment (latest 2026-10-27 00:51:04)

`results/turn_pid_20261027_005104` yielded three BASE 90-deg DONE
stops at 1.650/1.670/1.660s, mean stop 1.660s, peak gyro
90.2/90.1/93.8 dps, mean final error 1.793 degrees, actual new
MPU publication mean 999.728 Hz. This meets the **allowed ±2deg
heading tolerance** but does NOT meet the desired high-speed motion
performance. Gyro/I2C sampling is not the main bottleneck.

## Phase 1 — bounded motor response / PWM-vs-angular-rate

Select `--mode motor` (sends idle-only UART character `m`).
The 200Hz control tick sends six differential motor PWM pulses separated
by neutral intervals; one physical PA0 press-release runs the **whole
bounded short sweep**:

| Stage | Commanded PWM | Pulse | Neutral |
| --- | ---: | ---: | ---: |
| 0 | 70/1000 (7%) | 150ms | 100ms |
| 1 | 95/1000 (9.5%) | 150ms | 100ms |
| 2 | 120/1000 (12%) | 150ms | 100ms |
| 3 | 155/1000 (15.5%) | 150ms | 100ms |
| 4 | 200/1000 (20%) | 150ms | 100ms |
| 5 | 260/1000 (26%) | 150ms | 100ms |

**These are not automatic PWM gains**: pulses estimate starting motion,
rise/decay response, actual driver ramp, gyro angular rate and wheel
encoder asymmetry. The brief neutral does not ensure the chassis is at
rest before the next pulse, so true breakaway friction must be verified
with isolated longer pulses if necessary. No fabricated RPM/torque estimates.
Commanded PWM != electrical voltage at the motor terminals. PWM stage is
limited to 26% in this first diagnostic, independent of the later optional
80% hardware design goal.

Safety: the sweep locks motors if yaw exceeds +65deg, if gyro magnitude
exceeds 220dps, if timeouts/stale IMU/PA0 STOP occur, or if direction is
opposite. The 0.8s post-stop gyro coast is still logged. `ANGLE_LIMIT`,
`TIMEOUT`, `PA0_STOP`, `WRONG_DIRECTION`, `IMU_STALE` or `STALLED_NEAR_TARGET`
must be investigated; they are **not accepted** runs.

Run after visually checking polarity (with wheels lifted, motor supply
independently switchable) and placing the robot on a clear flat surface:
```powershell
git pull origin giomuadongbac
python -m platformio run -e turn_pid_diag_96m -t upload
python tools/turn_pid_pipeline.py --port COM7 --mode motor --profile base --runs 3
```
Press/release PA0 once per sweep, do not keep holding the button.
Ensure the first trace shows the intended turn direction. For the motor
sweep, do not require a final yaw of 90deg: this is an open-loop
characterization, not a 90deg-position control trial.

### Outputs / interpretation

Each `run_NNN` now has `samples.csv` (50Hz gyro yaw/rate, requested PWM,
applied left/right PWM after motor's 1kHz slew, encoder ticks and derived
signed encoder counts/sec) and `motor_steps.csv` / `motor_steps.json`
(stage PWM, measured gyro response, applied duty, wheel delta/imbalance).
`imu_windows.csv`, `raw_uart.log`, summary files and plot remain.
The last `stage=6` is reserved for an optional TUNED stall-recovery pulse,
never for the motor sweep, whose stages are 0–5.

Use these measurements to estimate wheel PWM-to-speed calibration and
a *provisional* breakaway band. Use measured physical battery voltage
and encoders if absolute angular acceleration/torque models are needed.

## Phase 2 — TUNED pivot angle control (BASE and FAST unchanged)

Select `--profile tuned` (idle UART `t`). `90` and `180` modes
remain available. New phase 2 pilot uses:
- outer angle proportional gain: `kp_angle=6.5` (deg/s per degree)
- inner angular-rate P: `kp_rate=1.75`; `ki_rate=0`, `kd_rate=0`
- rate feedforward: `kff_rate=0.95`; continuous static feedforward up to
  48 permille, ramped to avoid sudden sign flips
- measured angular-rate look-ahead: `predict_s=0.045` sec
- nominal angular speed ceiling: `200deg/s`
- angular acceleration/deceleration reference: `850deg/s²`
- PWM cap: **350/1000 (35%)**, absolute diagnostic guard **400/1000 (40%)**
- accuracy acceptance: existing `±2deg`, stable |gyro| <=12deg/s
  for 150ms (same as BASE); final true heading requires external reference

Near-target stall recovery is ONLY enabled for `TUNED`: after 160ms
of |gyro|<3deg/s while 2deg<|error|<12deg, issue at most three separate
80ms / 15% duty correction pulses. These are explicitly logged with
`stage=6`. If motion still stalls after three pulses, report
`STALLED_NEAR_TARGET` rather than driving indefinitely.
A tuned turn is locked for overshoot greater than 8deg beyond target
or angular rate above 450deg/s. Older BASE/FAST timing/gains remain
unchanged for apples-to-apples benchmark comparisons.

**Important:** This profile is an engineered *first test*, not a measured
optimum. Its 35% cap must not be raised to 80% just because the MCU
supports a larger command. Derive high-speed speed/braking limits only
after examining phase-1 motor response and a safe TUNED trial. Controlled
backward correction of a few degrees is not the same as slalom cornering.

### Accepted test order

1. Test electrical motor polarity and physical PA0 STOP on lifted wheels.
2. Run 3 separate MOTOR sweeps on the ground, review `motor_steps.csv`,
   encoder deltas and driver applied duty. Reject abnormal oscillation.
3. Run **one** TUNED 90deg trial on a clear floor:
   ```powershell
   python tools/turn_pid_pipeline.py --port COM7 --mode 90 --profile tuned --runs 1
   ```
4. If yaw is correct and no unsafe reversal, excessive overshoot, stall
   fault or translation, capture 3 repeated TUNED 90deg trials.
5. Compare with historic BASE 90deg (1.660s average at ±2deg). Change
   one gain at a time based on the measured ratio speed-reference/gyro,
   applied duty, angular slowing and coast drift; only afterwards run
   180deg in the same order.
6. High-speed slalom is a **separate future module** using nonzero
   linear speed and curved path geometry. A high-speed pivot cannot
   by itself qualify a slalom trajectory.

Example logs: `TP,HELLO,turn_pid_diag_v3_pivot`,
`TP,START,...trace_rev=3`, `TP,D,...`, `TP,IMU,...`, `TP,RESULT,...`.
Prior rev1/v2 17-field UART traces remain parseable. No ZIP is generated
by default; commit textual `results/turn_pid_*` logs to GitHub directly.
The Python host excludes non-DONE turns from repeated timing averages
and keeps raw aborted traces for debugging.
