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

## 2026-10-27 — Real-log diagnosis and SMOOTH v4 pilot

Measured source logs: `results/turn_pid_20261027_011355` (three motor
sweeps), `results/turn_pid_20261027_011451` (one TUNED 90deg), and
`results/turn_pid_20261027_011550` (three repeated TUNED 90deg).
The motor sweep finished `ANGLE_LIMIT` in each trial after yaw exceeded
+65deg in the 6th stage (this sweep is NOT meant to turn 90deg).
TUNED mean stop time in the repeated trial was 1.382s, peak gyro about
205deg/s, final angle within +/-2deg; one UART trace had corrupted
frames and a 220ms gap despite firmware UART loss counters of zero.

The observed distinct physical jerk close to 90deg is supported by
`stage=6` (bounded 15% / 80ms kick) and shortly afterward negative
`cmd_pm` around -130..-142 at +88..+90deg. At +86.5..+87.3deg,
positive duty ~66..87/1000 had been applied but gyro stayed at 0deg/s;
a short kick caused a sharp forward velocity and then a reverse braking
command. That is not acceptable smooth terminal control even if final
angle happened to be accurate.

### New `SMOOTH` profile — distinct from historical TUNED

Choose `--profile smooth`, UART `v` only while idle.
`TP,HELLO` reports `turn_pid_diag_v4_smooth`.
The old BASE, FAST, TUNED remain UNMODIFIED for reproducibility. New
profile is first unverified experimental improvement:

| Parameter | TUNED previous | SMOOTH v4 pilot |
| --- | ---: | ---: |
| max nominal angular rate | 200deg/s | 230deg/s |
| accel/decel bound | 850deg/s2 | 950deg/s2 |
| Kp outer angle | 6.5 | 7.0 |
| Kp inner gyro rate | 1.75 | 1.90 |
| Ki/Kd inner | 0 / 0 | 0 / 0 |
| gyro-rate FF | 0.95 | 1.08 |
| smooth static FF | 48permille | 42permille |
| angle prediction | 45ms | 60ms |
| motor PWM cap | 35% | 35% |
| settle angle tolerance | +/-2deg | +/-2deg |

**SMOOTH does not use the 15% timed stiction kick or reverse PWM** near
the goal. From +82 to +88deg it clamps terminal command to 0..125permille,
and if gyro slows below 18deg/s gently increases command toward a
provisional forward duty of 108permille, limited to +9permille per 5ms
until motion recovers. When yaw first enters +/-2deg of target, the
controller sends Motor_Stop (coast) and waits for gyro |rate|<=12deg/s
and 150ms stability. Any excess >+2deg (or stall for 420ms in terminal
zone) terminates as `ANGLE_LIMIT` / `STALLED_NEAR_TARGET` rather than
issuing a violent reverse command. An overshoot abort is a FAILED run
for analysis; it must not be reported as accepted 90deg accuracy.

Terminal samples: `stage=7` means smooth final approach,
`stage=8` means coasting within +/-2deg. The tool reports
`terminal_80_to_stop_ms`, `kick_samples`, `smooth_approach_samples`
and `reverse_pwm_samples` for comparisons. No ZIP by default.

**Motor sweep fix:** stage on-time reduced from 150ms to 100ms;
neutral-time increased from 100ms to 150ms. Same 250ms/stage, six
7–26% duty levels and +65deg safety abort. This reduces cumulative yaw
while keeping encoder/duty instrumentation. These shortened pulses are
not a complete no-load motor dynamic model; externally verify results.

### Safe trial sequence

```powershell
git pull origin giomuadongbac
python -m platformio run -e turn_pid_diag_96m -t upload
python tools/turn_pid_pipeline.py --port COM7 --mode motor --profile base --runs 1
```

First check motors on a safe elevated fixture for direction/STOP behavior;
perform actual motion trials on a clear unobstructed floor with external
motor power cutoff. A rigidly fixed chassis cannot validate gyro yaw.
Review `motor_steps.csv` and require `reason=DONE` before progressing.
Then test only ONE SMOOTH 90deg run:

```powershell
python tools/turn_pid_pipeline.py --port COM7 --mode 90 --profile smooth --runs 1
```

If there is no jolt, excessive coast overshoot, odd direction, overheating
or appreciable translation, repeat 3 SMOOTH 90deg trials. Do NOT use
`--profile tuned` expecting the jerk to disappear; that profile is
intentionally preserved as the known baseline.

The 35% duty cap is a controlled pilot, not the mechanical limit. Do not
raise to 80% until the brake/coast dynamics and 90deg physical path are
validated. PID wall straightening is supplementary and cannot guarantee
safe stopping or accurate yaw where side walls are absent.

## 2026-10-27 — FLOW v6 high-speed monotonic braking (first hardware trial)

**Reason:** Actual RAPID results in `results/turn_pid_20261027_014527`
show four completed 90-degree turns (stop_ms=785,1350,1350,975),
and one `STALLED_NEAR_TARGET` (2075ms). 1kHz gyro peaks reached
454.1–470deg/s. A repeated 800permille forward command occurred shortly
AFTER negative braking: for run 2, cmd=-176 at t=160ms, +800 at t=220ms,
and -65 at t=300ms. The motor-driver's 30ms reverse-neutral protection
makes these direction changes especially undesirable. Long forward-only
approach phases explain the stop-time scatter (as long as 930ms from
80deg to stop). In aborted run 4, yaw changed by -11deg during the
motor-locked post-stop interval with a left encoder delta while the
right encoder did not change; investigate external movement, friction,
wheel slip, floor and hardware before treating it as closed-loop control.

**FLOW is opt-in, preserves all previous control profiles**: `--profile
flow` maps to idle UART 'w', index 5, and is limited to 90deg. PWM
ceiling remains 80%, angular-rate demand becomes 520deg/s, reference
acceleration 3200deg/s2. No motor-to-body inertial model is claimed.
A latched four-stage controller replaces RAPID's reactive sign-changing
rate loop: `stage=13` high-power forward acceleration, `stage=14`
one bounded reverse braking interval (maximum 28% reverse and subject
to driver 30ms reversal gap), `stage=15` zero-duty coasting, and
`stage=16` forward-only gentle terminal alignment (up to 17%).
`stage=17` is final neutral/stability check. Once it leaves stage 13,
it can NEVER return to high-power propulsion within that run. It
uses measured gyro speed to initiate braking when the rate-based
stopping-distance estimate plus 10deg becomes comparable to the
remaining error. Its deceleration estimate is a first hypothesis,
NOT measured from the new chassis. ±2deg / |gyro|<=12deg/s for
150ms remains the acceptance tolerance, overshoot >2deg is a fault.

The prior rapid profile remains available for A/B comparison.
Log parser now tracks the five new stages and flags all
high-power reacceleration events after the first braking phase
(`rapid_or_flow_brake_reaccelerations`). FLOW should report ZERO
for this metric. Historical RAPID run 2 would report a nonzero value.

**Important:** RUN 4 post-stop physical yaw discrepancy is unresolved.
Before higher-power floor trials, check the free-wheel, motor polarity,
loose hardware/IMU mounting, and whether the chassis was picked
up or nudged during the 800ms post-stop window. Operate on a clear
floor with a safe motor supply cutoff; hold no obstacle/hand near
the robot. A full-speed 90deg test should begin with ONE run.
Neither successful CI nor 1kHz gyro logging proves a smooth,
safe turn. Do not repeat a trial after unexpected motion.

Commands (PowerShell):
```powershell
git pull origin giomuadongbac
python -m platformio run -e turn_pid_diag_96m -t upload
python tools/turn_pid_pipeline.py --port COM7 --mode 90 --profile flow --runs 1
```

Physical PA0 press-release is still the ONLY start action; PA0 press
during motion stops; UART `S` also stops. If the first floor run is
smooth and `reason=DONE`, then three repeats are useful to compare
mean stop time, variance, peak angular rate, unexpected reverse
movement and final heading against RAPID.

## 2026-10-27 — GLIDE v7, FLOW jitter and high-speed brake diagnosis

SOURCE LOGS from `results/turn_pid_20261027_020005` and
`results/turn_pid_20261027_020022`: 3 FLOW DONE trials, 875/905/930ms
stop times; peak gyro from the MCU's 1kHz log 508.7/486.8/486.0deg/s;
heading errors 1.48/1.60/1.25deg.

FLOW entered negative braking at yaw ~31–32deg and reached almost
zero yaw rate already at yaw 60–63deg. Therefore it crawled about
25–28 more degrees with repeated motor command cutoffs. Run2 near-tail
motor PWM had +116/0/+48/+96/+137... toggling as desired angular
rate shifted across the measured rate. The time from 80deg to completed
stop was 350–365ms; yaw-rate and UART data do not support claiming
faster final turns from increased PWM alone.

GLIDE is a separate 90deg-only experimental profile (index 6, UART
idle command `g`, host `--profile glide`). Base FLOW stays unmodified
for comparison. It uses the same 80%-MAX PWM and 520deg/s nominal
angular reference, but delays the SINGLE brake trigger according to a
revised estimated stopping distance (`gyro_rate²/(2*3600) + 18deg`).
That figure is an engineering trial based on the previous observed
braking interval, not a measured physical invariant. Request slew limits
(95permille per 5ms accelerating, 160permille per 5ms braking) soften
large torque-command jumps. Existing motor driver PWM slew and the
30ms direction-reversal interlock remain enabled. A delayed brake may
increase overshoot: the first floor trial must be conducted in clear
space with external motor-power cutoff ready.

GLIDE phases: 18 initial forward acceleration, 19 one bounded
reverse-torque braking interval, 20 neutral coast, 21 continuously
controlled gentle final approach, 22 motor neutral/stable heading.
GLIDE does not use the binary `if(rate>wanted_rate+18)u=0` rule that
caused repeated cutoffs in FLOW. Instead a low-speed desired rate
and smooth forward duty update are used with symmetric +/-16 permille
per 5ms slew and 0..165permille cap. No terminal hard reversal or
15% kick. Brake stage cannot switch to coast solely because
error<=21deg while actual yaw-rate remains very high.

The host adds `brake_first_yaw_deg`, `max_cmd_delta_20ms` (a proxy,
not physical jerk), GLIDE stage counts and return-to-acceleration
counts. Judge speed via stop_ms with reason DONE, full trace coverage,
1kHz peak gyro, overshoot, remaining terminal angle time, and
observed motion smoothness on the floor.

Powerful physical motion still requires caution: keep the chassis
free of loose cables and ensure that the 2deg overshoot guard does not
trigger. Do not automatically repeat after unexpected reverse motion
or translation. This is NOT a validated slalom controller, and
firmware compilation/CI alone cannot qualify dynamic braking.

Test sequence:
```powershell
git pull origin giomuadongbac
python -m platformio run -e turn_pid_diag_96m -t upload
python tools/turn_pid_pipeline.py --port COM7 --mode 90 --profile glide --runs 1
```
Only after the single trial is physically smooth, no unexpected
post-stop movement, and reason DONE: repeat with `--runs 3`. No ZIP
is created; push plain results on GitHub as before.

## 2026-10-27 — SPRINT v8 full 100%-PWM ceiling and 180deg test

Verified newest source logs: `results/turn_pid_20261027_021854`,
`results/turn_pid_20261027_021939`.
Six GLIDE DONE rows recorded stop times 715,660,725,695,720,740ms
(709ms mean), 1kHz gyro peak around 542..564deg/s, and measured
applied wheel PWM typically ~710..717/1000. A DONE-labelled run drifted
to 85.40deg after the motor was locked (4.6deg error), showing that
the pre-stop verdict was insufficient. Other two traces lack terminal
RESULT and are PARTIAL; do not count them in accepted-run averages.

SPRINT v8 is a separate opt-in profile, index 7, UART idle select
character `p`, and host `--profile sprint`. It supports `--mode 90`
AND `--mode 180`, *not* motor sweep. It permits command PWM up to
**1000/1000 (100%)** and keeps the hardware PWM slew limiter and
existing 30ms direction-reversal neutral gap. This is a duty **ceiling**;
requested PWM and measured effective gyro speed are different values.
The controller uses a variable yaw-rate reference ceiling:
620deg/s for a 90deg pivot, 720deg/s for a 180deg pivot,
and angular-acceleration reference up to 4000deg/s2. A one-way
rate-based braking transition accounts for remaining angular distance,
under a single bounded 31% reverse torque episode. Final PWM 0..16%
tapers starting within 6deg of the endpoint, attempting to avoid the
visible final motor stop jolt. The motor may not reach 100% PWM or
either reference velocity on 90deg; no measured performance is claimed.
Rate abort guard is 850deg/s; yaw > target+2deg aborts, as does
detected reverse motion. Physical PA0 remains the only START and a
separate motor supply cutoff is required for high-energy tests.

SPRINT stages: 23 forward high-power acceleration, 24 single braking
interval, 25 neutral coast, 26 forward-only final approach, 27
final angle holding with motors neutral. Existing log collection
reports `sprint_accel_samples`, `sprint_brake_samples`, etc. and
`terminal_last10_to_stop_ms` for **both** 90 and 180deg.

**New post-stop qualification:** After any `DONE` 90/180 turn,
the 800ms motor-locked observation checks that yaw stays inside
+/-2deg. A violation changes the terminal result to
`POST_STOP_DRIFT`, and it is rejected from accepted aggregate means.
It can be caused by slippage, external handling or other motion;
it does not prove a controller defect by itself. No motor output is
automatically reenabled during this observation.

One-pass physical test procedure:
```powershell
git pull origin giomuadongbac
python -m platformio run -e turn_pid_diag_96m -t upload
python tools/turn_pid_pipeline.py --port COM7 --mode 90 --profile sprint --runs 1
# ONLY if the previous trial safely returns DONE with no jolt/drift:
python tools/turn_pid_pipeline.py --port COM7 --mode 180 --profile sprint --runs 1
# Additional repeatability trials, only after safe single trials:
python tools/turn_pid_pipeline.py --port COM7 --mode 90 --profile sprint --runs 3
python tools/turn_pid_pipeline.py --port COM7 --mode 180 --profile sprint --runs 3
```

The first 180deg spin covers twice the angular distance and can sustain
high PWM for longer. Use a clear floor, no wires or obstacles, and
verify motor power cut and PA0 STOP before running. A first 180deg run
is not automatic; a physical PA0 press-release is needed after READY.
If any `ANGLE_LIMIT`, `POST_STOP_DRIFT`,
`STALLED_NEAR_TARGET`, `WRONG_DIRECTION`, abnormal mechanical
translation or excessive heat occurs, stop the experimental series
and review the log rather than immediately repeating at full power.
This diagnostic is a stationary pivot, NOT a forward-moving slalom.

## 2026-10-27 — TRACK v9 continuous velocity trajectory: eliminate midturn halt

User-reported issue: more physical SPRINT 90deg floor logs pushed under
`results/turn_pid_20261027_024310` and
`results/turn_pid_20261027_024355`. On complete 50Hz UART samples,
SPRINT repeatedly initiated reverse braking at about 32deg with
560–575deg/s angular speed. At 200ms it remained 260–350deg/s
around yaw 70–72deg, but around 220–260ms the motor braking had driven
gyro speed down to +20 or even -22deg/s around yaw 72–76deg, still
14–18deg short of the requested setpoint. Afterwards commanded
+70..160 permille gradually re-accelerated the robot to about
+70deg/s, causing a visible two-stage turn. This is physically
documented OVERBRAKING, not just missing PWM headroom.

SPRINT software limits and phase transition remain untouched as a
comparative baseline. **New opt-in TRACK v9** (profile=8, UART 'k',
host `--profile track`) supports 90 and 180deg independently, with
1000/1000 PWM ceiling, 620/720deg/s nominal maximum angular rate.
It replaces the distinct accelerate/reverse brake/creep stage
machine with a continuously updated angle-to-rate envelope:
`rate_brake=sqrt(2*3500*max(0,error-1deg-0.022s*max(gyro,0)))`
and
`rate_target=min(620/720,14*error,rate_brake)`.
The rate target ramps upward no faster than +4000deg/s2 and down no
faster than -6000deg/s2; inner velocity feedback and acceleration
feedforward compute bounded PWM. Output changes are limited to
105permille per 5ms away from the target; inside 12deg, PWM steps
are limited to 35permille per 5ms when already low, with a rapid
ramp-down allowed if duty remains above 180permille.

Avoiding a fixed braking-to-zero phase does NOT itself guarantee
no reverse-torque requests: TRACK can command bounded -320permille
before final 12deg and no more than -130permille in the last 12deg.
The hardware still imposes its 30ms reverse-polarity neutral gap.
Close to target (<4deg, measured |gyro|<65deg/s) negative PWM is
prohibited, and firmware uses neutral stop in the ±2deg band.
An unexpected early stop, -40deg/s reverse, yaw beyond target+2deg
or 800ms post-stop yaw drift beyond ±2deg is a FAILED run.
**Continuous trajectories are a hypothesis that requires floor
verification**, not a passed motion benchmark.

TRACK debug stage mapping: 28=accelerate/positive control outside
12deg, 29=bounded reverse braking outside 12deg, 30=inside final
12deg under continuous gyro-rate regulation, 31=stop/settling.
Host additionally exports `midturn_low_speed_samples` (samples
with |gyro|<15deg/s, yaw>0.3*target and yaw<target-10deg),
`midturn_reverse_samples` (gyro<-3deg/s in the same region),
and `min_midturn_gyro_dps` to directly compare the problem.
The SPRINT logs typically show these unwanted events.

**Trial order**: test on a clear floor with the independently
operable motor supply cutoff and PA0 STOP validated. Verify firmware
banner `TP,HELLO,turn_pid_diag_v9_track`, and run **one** 90deg
TRACK trial first. Do not immediately run multiple high-power trials
or 180deg before that run is physically acceptable.

```powershell
git pull --ff-only origin giomuadongbac
python -m platformio run -e turn_pid_diag_96m -t upload
python tools/turn_pid_pipeline.py --port COM7 --mode 90 --profile track --runs 1
# Only if 90deg is smooth, stays inside +/-2deg after motor stop:
python tools/turn_pid_pipeline.py --port COM7 --mode 180 --profile track --runs 1
```

In the two raw sessions, several runs had large UART telemetry gaps
despite firmware-logged qdrop=0 and uartdrop=0, so their derived metrics
need the `meets_trace_thresholds` flag checked. The user should not
interpret a partial result, malformed UART frame or reconstructed
stop time as proof of motion quality.

## 2026-10-27 — GUARD physical data and PACE v11 qualification

Four newest pushed sessions: `101542`, `101625`, `101700` and
`101751`. GUARD v10 provided **zero DONE** completed observations:
six 90deg result records were STALLED_NEAR_TARGET with headings
85.06–86.95deg after 965–1075ms. Three 180deg records also
reported STALLED_NEAR_TARGET (1155–1295ms) with end headings
177.06,176.73,147.20deg. Other traces were PARTIAL, not completed
tests. Peak 1kHz yaw rate was ~434–441deg/s on 90 and
~542–548deg/s on 180. Near-goal gyro becomes exactly zero; in
90deg run2, UART indicated yaw=85.86deg and cmd=60permille unchanged
for >250ms. GUARD previously used
`max(0,error-4deg-0.055s*gyro_positive)`, which zeroed the
velocity reference BEFORE reaching ±2deg. Its 0.8*rate command
then failed to overcome motor stiction. This is a controller bug,
not evidence that the hardware cannot run fast.

PACE v11 is a new optional 90/180 profile (UART `j`, index 10,
CLI `--profile pace`). GUARD remains unchanged for A/B.
Speed-reference caps are 520deg/s (90) / 620deg/s (180).
PWM ceilings are 850/1000 (90) and 920/1000 (180), not
always-on duty. The continuous braking envelope now uses
0.7deg reserve and 48ms speed-based prediction. Rather than a
fixed high-power terminal kick, positive PWM is supplemented
smoothly by up to 62permille of low-speed friction compensation,
decaying to zero by 45deg/s gyro speed. Close to the target
the PWM request is capped at 155permille when the measured
speed is low; reverse commands stay bounded, and motor driver's
30ms direction interlock is retained. The ±2deg AND <=12deg/s
dwell for 150ms and the 800ms motor-locked post-stop yaw
check remain mandatory. No claimed hardware validation yet.
Previous high-power TRACK caused significant overshoot; the
PACE maximum still needs physical qualification.

Commands in PowerShell, first trial ONLY:
```powershell
git pull --ff-only origin giomuadongbac
python -m platformio run -e turn_pid_diag_96m -t upload
python tools/turn_pid_pipeline.py --port COM7 --mode 90 --profile pace --runs 1
```
Confirm `turn_pid_diag_v11_pace`, calibration and READY. Press
and release PA0 only after READY, with independent motor-power
cutoff available. Stop the sequence on jerk, reverse rotation,
overshoot, or STALLED/ANGLE_LIMIT/POST_STOP_DRIFT reasons.
Only a physically smooth, safely finished 90deg run qualifies
for a single 180deg trial:
```powershell
python tools/turn_pid_pipeline.py --port COM7 --mode 180 --profile pace --runs 1
```
Do not infer a safer/faster robot from CI success. Compare
`stop_ms`, `final_abs_error_deg`, `midturn_low_speed_samples`,
`midturn_reverse_samples`, `max_applied_pwm_pm`, and
`in_target_band_high_rate_samples` using complete UART traces.

## 2026-10-27 — COAST v12: remove short PWM polarity chatter

Analyzed physical `PACE` UART logs in sessions
`103237`, `103309`, `103340`, `103408`, `103453`.
Accepted 90deg DONE turns took 625–815ms and reached gyro peaks
467–479deg/s; accepted 180deg turns took 835–905ms, peaks
~600–608deg/s. PACE still emitted several *short* negative PWM
requests, including run4 session103309 requests -79 at 69.17deg,
+17 at 75.20deg, -62 at 81.81deg, then +113 at 86.94deg.
Actual applied PWM stayed zero during many of these intervals.
The driver implements a >=30ms neutral wait after motor direction
reversal. Because the negative commands may expire before the gap,
their net effect is loss of drive, not reliable physical reverse
torque. Repeated crossings thus make the turn feel discontinuous.

Session103340 run12 also alternated motor OFF/ON near yaw 88deg
(`0,+68,0,+64,0`) because settling was not latched when sensor
noise briefly crossed the ±2deg boundary; this increased turn time.

**COAST v12** adds isolated profile 11, UART idle command `c`,
host `--profile coast`; ALL old profiles remain unchanged. For
90deg it uses 555deg/s maximum reference and up to 89% PWM,
for 180deg 655deg/s and up to 94% PWM. It is **positive or zero
PWM only** (left wheel negative, right wheel positive for the
usual counterclockwise yaw); negative requests are rejected in the
host parser. This removes repeated commanded sign swaps and the
driver's polarity interlock but also REMOVES active reverse braking.
A conservative coast stopping envelope, more advance notice and
smoothed PWM command are used instead. The stronger endgame
friction correction ramps output towards ~110permille when the
robot is almost stationary outside the ±2deg band; it remains
bounded and is not a fixed-time stall kick. The first time the
angle is within ±2deg AND gyro magnitude <=12deg/s, motor neutral
is latched for 150ms. If it escapes the ±2deg band, report drift,
do not issue a second motor pulse to recapture the angle.
The post-stop 800ms yaw check stays enabled. Stages 42=forward
motor drive, 43=commanded neutral coast, 44=low-speed final,
45=latched stable, 46=within angle band but too fast.

**Risk:** Removing reverse torque could make an aggressive moving
robot overshoot, especially if floor friction, battery voltage or
payload differ from the recorded trials. CI only demonstrates
compilation and parser tests; it does NOT establish motion safety,
successful turnaround, smoother motion or improved lap time.
A large previous 90deg post-stop yaw change (-46.64deg) remains
unexplained and must be checked for chassis movement or IMU
mounting issues independently.

FIRST TEST only:
```powershell
git pull --ff-only origin giomuadongbac
python -m platformio run -e turn_pid_diag_96m -t upload
python tools/turn_pid_pipeline.py --port COM7 --mode 90 --profile coast --runs 1
```
Observe `turn_pid_diag_v12_coast` banner, PA0 press-release START,
and independent supply cutoff on a clear floor. Any overshoot,
unstable physical turn, `ANGLE_LIMIT`, `WRONG_DIRECTION`,
`STALLED_NEAR_TARGET`, or `POST_STOP_DRIFT` means DO NOT
continue with 180deg, higher speed, or repeated tests. Inspect
original UART logs and 50Hz trace before revising.

The goal for the next log is zero `coast_negative_command_samples`,
zero midturn reversals or stalls, a genuinely settled angle within
±2deg, and lower time-to-complete. The 1kHz peak gyro and actual
applied PWM must be measured rather than inferred from setpoints.

## 2026-10-27 — COAST v13 faster test: 700/800deg/s ref, 100% PWM limit

The user prioritized greater turning speed. The opt-in `coast`
profile (index=11, UART 'c') has been increased from nominal
555/655deg/s to **700deg/s for 90deg** and **800deg/s for 180deg**.
Both PWM ceilings become **1000permille (100%)**, rather than
890/940. Angular acceleration reference is increased from
3750 to 4900deg/s2, and rate feedback/feedforward
(1.95/0.83 -> 2.20/1.03) can request higher acceleration
torque. Request slew is still active (105pm per 5ms at
higher speed); the motor driver also limits slew at 1kHz.
COAST remains positive torque or neutral ONLY, with no
brief reversing torque or 30ms reversal gaps.

The 78ms speed-prediction lead (was 60ms) and an extra 3.5deg
reserve (was 2deg) make the calculated coasting envelope
more conservative in response to larger acceleration.
This is a **speed experiment, NOT a guaranteed measured
700/800deg/s physical turn**. Because COAST provides no
active reverse braking, it may overshoot substantially on
a low-friction/high-inertia robot; abort logic alone
cannot undo physical angular momentum. 90deg/180deg
must NOT be tested as a 5-run unattended batch until
individual motion is judged safe.

The speed ceiling and hardware motor torque/voltage constraints
are not identical. Already observed PACE physical applied
PWM peaked roughly ~60% (90) / ~75% (180); raising only
the PWM cap would not have changed those runs. The stronger
initial acceleration/rate feedforward aim to produce an
observable change. Existing `command_polarity_flips`,
`applied_pwm_max_step_20ms`, and
`neutral_actual_high_gyro_samples` metrics remain.

Run only ONE 90deg:
```powershell
git pull --ff-only origin giomuadongbac
python -m platformio run -e turn_pid_diag_96m -t upload
python tools/turn_pid_pipeline.py --port COM7 --mode 90 --profile coast --runs 1
```

Expected banner: `TP,HELLO,turn_pid_diag_v13_coast_fast`.
Ensure PA0 emergency stop and independent motor power
cutoff, clear floor and secured wires. If any unexpected
turn, overshoot, impact, wobble or
`ANGLE_LIMIT`/`WRONG_DIRECTION`/`POST_STOP_DRIFT`/
`STALLED_NEAR_TARGET`, stop tests and inspect logs.
Run ONE 180deg only after 90deg is physically safe;
then push raw UART, summary, metrics to GitHub.

## 2026-10-27 — COAST v13 physical rejection and SINGLEBRAKE v14

Newest pushed physical COAST v13 logs: `turn_pid_20261027_111007`,
`turn_pid_20261027_111044`, `turn_pid_20261027_111107`.

90deg: four hardware trials, two DONE (730 and 1025ms)
with measured 1kHz peak gyro 477–482deg/s. One ANGLE_LIMIT
exceeded **98.36deg** (8.36deg peak overshoot), and one
POST_STOP_DRIFT ended 87.99deg (2.01deg error).
Measured applied PWM max only ~62–67% despite **100% cap**.
180deg: one POST_STOP_DRIFT at final 177.94deg, measured
gyro peak **699.3deg/s**, effective applied PWM **94.6%**.
No motion set is accepted for speed/precision simultaneously.

Important repeated 90deg oscillation:
111044 run5 at t100ms gyro439deg/s, cmd561;
t120ms gyro484, reference325, cmd141;
t140ms gyro427, cmd19; t160ms gyro256, cmd439;
t180ms gyro212, cmd551; t200ms gyro374, cmd131;
t220ms gyro327, ref9, cmd0. At t360ms yaw89.18
while gyro **116.6deg/s**, no reverse brake available,
then yaw reached **98.36deg**. COAST phase is
positive-only and has no means to shed this angular
momentum quickly.
In 180deg log, rate reached699deg/s then PWM requested
0→327→747→482→0 across t260–360ms. Repeated large
positive/zero PWM transitions explain non-smoothness
even without direction reversal.

A direct speed ceiling change (555→700deg/s or 655→800)
did not increase 90deg physical peak above~485 and did
not solve stop time. Angular-rate reference couples raw
gyro to lookahead distance and to the PI controller
without smoothing; this can make duty oscillate.
Significant 20ms applied-PWM jumps and neutral motor
samples while gyro>100dps have been recorded.

**SINGLEBRAKE v14** is an opt-in, separate experimental
profile (index12, UART idle char `n`, Python CLI
`--profile singlebrake`) for 90/180, leaving COAST
and ALL older profiles unchanged.
Targets: 650deg/s, max90% PWM for 90; 780deg/s,
max100% for 180 (setpoints, NOT achieved/validated).
Acceleration reference up to 5500deg/s2. Unlike COAST,
it explicitly uses a single reverse braking stage at most
30% PWM, which keeps the preexisting 30ms polarity
reversal interlock ON. A calibrated predictive brake
trigger includes gyro-rate²/(2*5500), rate-dependent
latency compensation and 8deg reserve. The controller
can then coast, perform at most one low-speed positive
correction phase (≤15.5% PWM) and latch a neutral
stop for 150ms when BOTH angle±2deg and gyro≤12deg/s
pass. Never return to high-power acceleration or reverse
braking after the single episode; abnormal overshoot,
wrong direction and 800ms post-stop drift still FAIL.

Potential flaw that must be measured: the trigger model is
not an identified physics model. The old SPRINT stage began
braking at ~32deg with rate~560 and stopped ~72–77deg
in 90deg trials, while for 180 it began near74deg with
rate~805 and stopped near136–139deg. The single-brake
trigger has been calibrated to act somewhat later, while
maintaining margin for the motor driver's 30ms gap.
Result might still be undershoot, overshoot or an
unpleasant ONE-TIME braking jolt. This is not yet a
proven smooth, fast racing turn or an accepted motion.

First physical trial ONLY (never 5 unattended runs):
```powershell
git pull --ff-only origin giomuadongbac
python -m platformio run -e turn_pid_diag_96m -t upload
python tools/turn_pid_pipeline.py --port COM7 --mode 90 --profile singlebrake --runs 1
```

Confirm firmware HELLO version `turn_pid_diag_v14_singlebrake`
and TP CONFIG profile=12, check PA0 physical emergency
stop, physically independent power cutoff, clear and
non-slipping test area. If there is overshoot, unexpected
wheel translation, strong jolt, reverse yaw, ANGLE_LIMIT,
WRONG_DIRECTION or drift, STOP further tests. Only
after physically smooth and verified DONE 90deg may a
single 180deg trial be attempted. Log and compare
`brake_first_yaw_deg`, `single_brake_samples`,
`command_polarity_flips`, applied motor PWM and actual
gyro rate. No claim of better speed or smoothness can
be made from CI compilation alone.
