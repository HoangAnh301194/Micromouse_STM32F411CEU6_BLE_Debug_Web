# Final Encoder speed-estimator alpha: guarded closed-loop trial

**Status: diagnostic implementation only. No new physical data captured by GitHub Actions.**
Branch: `giomuadongbac`. Run `tools/speed_pid_final_pipeline.py`.

## Important architectural correction

Existing `src/motion.c` does NOT implement a wheel-speed PID:
its `PID_Compute` callers control straight-line heading and turn yaw.
The prior P0-P3 motor sweep was **open-loop PWM**, while the 10-alpha bank
was an observational estimator. Therefore the next experiment **must create
a separate wheel-speed closed-loop controller** rather than relabeling the
open-loop PWM results as PID tracking.

The new `src/speed_pid_diag.c` does exactly that in an **isolated
diagnostic-only image**. Both wheels use the actual `src/pid.c`
`PID_Compute` and fixed **exploratory**, NOT production-tuned gains:
Kp=0.050, Ki=0.080, Kd=0.000; sample period 10 ms in main, target
PPS steps 0, 800, 1600, 2400, fixed feedforward commands 0,
100, 200, 300 permille and PID correction clamp +/-100 permille.
Final command is physically capped by *software demand* at **0..300
permille (30%)**, while the Motor Driver retains its ramp and watchdog.

Only `SPEED_LPF_ALPHA` in **diagnostic** builds changes between 0.08,
0.10 and 0.30, filtered at 1 kHz by TIM11. All production builds retain
alpha 0.30 and existing heading/turn PID parameters unchanged. Each
diagnostic uses the original raised-wheel 12-stage LEFT/RIGHT/BOTH
schedule with 0/10/20/30% **stage reference labels**; actual PID PWM
may differ from the nominal stage reference, but is limited to <=30%.

## Physical safety — read BEFORE ANY execution

- This is a **LIVE MOTOR-ON** diagnostic. Wheels MUST be securely
  elevated and guarded; do not run on an unrestrained mobile robot.
- An independent, tested physical motor-VM power cutoff and an attentive
  operator are mandatory. Never rely on USB/UART/software STOP alone.
- Every one of 9 test runs flashes a new motor-enabled image. The
  pipeline explicitly requests **VM_OFF** before EACH flash and
  **MOTOR_READY** after flashing. It requests VM_OFF again after DONE.
  If a run crashes, physically disconnect VM before handling board
  or ST-Link; the host attempts motor-OFF restoration in `finally`.
- Firmware still requires MODE ON + START with fresh MPU calibration,
  750-ms PING lease, 300-ms Motor Driver command lease, TIM11/IMU/I2C
  watchdogs, permanent STOP/FAULT lock, and 180s session deadline.
  The host checks the **firmware-reported alpha and exact fixed PID
  gains**, target step, PWM hard ceiling and 12-stage count before START.
- Production PID/alpha will **not** be auto-modified by this pipeline.

## Commands

Power off motor VM before flashing. First synchronize the clean checkout:

```powershell
git status -sb
git fetch origin
git pull --ff-only origin giomuadongbac
```

Run ALL native host regressions and compile the three alpha firmware
images without opening serial COM ports, flashing or moving motors:

```powershell
python tools/speed_pid_final_pipeline.py --offline-only
```

For the nine supervised motor-enabled trials (three repeats each,
counterbalanced order, **never more than 30%**):

```powershell
python tools/speed_pid_final_pipeline.py --port COM7 --fixture-ready --independent-vm-cutoff --allow-30-percent
```

This command first runs a fresh real 33-stage locked Motor-OFF sanity
check. Each run then uses a distinct firmware environment:

- `speed_pid_alpha_008_96m`: alpha 0.08.
- `speed_pid_alpha_010_96m`: alpha 0.10.
- `speed_pid_alpha_030_96m`: alpha 0.30 (production baseline).

Counterbalanced order: (0.30,0.08,0.10), (0.10,0.30,0.08),
(0.08,0.10,0.30). The 9 serial logs, stage readings,
host PING timing and restoration proof are saved under
`results/speed_pid_final_YYYYMMDD_HHMMSS`.

### What is measured

`SYS_PID` at 20 Hz contains target PPS, EMA feedback for each wheel,
independent **50ms encoder-count difference** reference PPS, both
controller command PWM and applied (ramped) PWM. This interval-count
reference is *temporally distinct* from the 1-kHz EMA feedback but is
still derived from the SAME quadrature encoder — not an independent
external tachometer. It does not prove actual shaft speed, ground-slip
or absolute metrology.

The host evaluates normalized mean absolute tracking error (55%),
overshoot (15%), 50ms reference-speed variation (15%) and settling
time (15%). Metrics are fixed before collection, computed per
active wheel for 9 nonzero stages and compared across 3 repetitions.
Missing stage, missing samples, firmware FAULT, UART/clock/IMU error,
unreliable PING or command above 30% all **block selection**.

The winner must beat the runner-up by >=5% on weighted score and win
>=2 of 3 counterbalanced repeats. Otherwise result
`INCONCLUSIVE` means **do not change production alpha**.
If clear, status `CLOSED_LOOP_CANDIDATE` is a recommendation
for final review, not an automatic firmware deployment.

## Remaining qualification boundary

This trial can qualify the **count-derived speed estimation loop on
a raised-wheel fixture** and distinguish which EMA gives better
tracking for a FIXED, exploratory speed PID. It cannot prove that the
existing production *heading/turn* PID is optimal with that EMA,
nor wheel-ground traction, absolute distance measurement or
under-load performance. Final production approval should additionally
check motion regression with the chosen EMA on the actual robot.

## Files

- `summary.json`: all passes/faults, 9 trial results, ranked
  candidate or explicit inconclusive status; motor-off restore result.
- `motor_off/summary.json`: P0 prereq, original UART evidence.
- `repeat_N_alpha_XXX/on_uart.log`: actual raw serial frames.
- `repeat_N_alpha_XXX/metrics.json`: per-stage/wheel speed metrics.
- `restore_motor_off.log`: upload result after test.

If `.log` files are ignored by Git, use `git add -f` on the
result directory's UART logs when publishing evidence.
