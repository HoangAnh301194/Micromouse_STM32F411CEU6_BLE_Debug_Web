# TURN PID -- 90° / 180° qualified trace pipeline (NO OLED)

## Scope and safety
Opt-in PlatformIO \`turn_pid_diag_96m\`: **only turns**, no straight mode.
USES the existing USART1 debug PA9 (TX)/PA10 (RX), ST-Link, MPU6050 and
TIM11 (1kHz). Motor outputs LOCKED at boot, calibration, idle and after stop.
**Only a physical PA0 press-release** starts movement. No remote START command
or autonomous sequence. A new PA0 press stops in TIM11; UART \`S\` stops via
main (not a substitute for physically cutting VM supply).

Test first with wheels mechanically raised/secured, VM separately switchable,
clear of obstructions, operator present. Check left/right direction and gyro
sign. An inverted gyro/motor mapping or motor drive misbehavior can damage
hardware despite software guards. Motor cap is 260/1000 BASE or 300/1000 FAST.
Do not start FAST or floor testing until BASE direction and stop are accepted.

## Install and run (Windows PowerShell)
\`\`\`powershell
git switch giomuadongbac
git pull origin giomuadongbac
python -m platformio run -e turn_pid_diag_96m -t upload
python -m pip install pyserial matplotlib
python tools/turn_pid_pipeline.py --port COM7 --mode 90 --profile base --runs 3
\`\`\`
Do **not** run miniterm concurrently with the Python capture: only one
program should own COM7. Open capture, reset STM32 if necessary,
wait for \`TP,CAL_OK\`, \`TP,CONFIG,mode=90,profile=0\` and
\`TP,READY\`, then press/release PA0 **once per trial**. PC saves each
trial immediately after a \`TP,RESULT\` frame.
No motor start before gyro calibration / fresh snapshot (<=3ms).
If calibration remains stuck (\`TP,CAL ok=0\`), diagnose MPU first;
the script intentionally does not override calibration.

After accepted BASE 90° on fixture, run:
\`\`\`powershell
python tools/turn_pid_pipeline.py --port COM7 --mode 90 --profile fast --runs 3
python tools/turn_pid_pipeline.py --port COM7 --mode 180 --profile base --runs 3
python tools/turn_pid_pipeline.py --port COM7 --mode 180 --profile fast --runs 3
\`\`\`
The user must verify spacing between runs and press PA0 each time.
To recover a serial capture after interruption:
\`\`\`powershell
python tools/turn_pid_pipeline.py --analyze results/turn_pid_YYYYmmdd_HHMMSS
\`\`\`

## Controller design and what is tuned
- Outer loop: angle error -> gyro angular velocity reference [deg/s].
- Rate limit: \`min(kp_angle*abs(error), max_rate, sqrt(2*accel*abs(error)))\`.
- Angular speed command slew is limited by max angular acceleration.
- Inner loop: gyro rate error -> PWM PI controller + rate feedforward.
  Rate Ki starts at zero to avoid integrator windup until physical baseline.
- Position/speed accepted together: angle band ±2°, |gyro| <=12 deg/s
  continuously for 150 ms. Timeout 4.0s for 90°, 6.5s for 180°.
- STOP locks motor, then captures **0.8s coasting** without PWM so peak
  overshoot and post-stop drift are not masked.
- Wrong yaw polarity and excessive positive overshoot abort/lock the run.

Profiles (initial exploratory values, NOT yet physically optimized):

| Profile | max PWM | max rate | accel | kp-angle | kp-rate | kff-rate |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| BASE (0) | 260 | 150°/s | 650°/s² | 4.8 | 1.15 | 0.35 |
| FAST (1) | 300 | 230°/s | 950°/s² | 6.0 | 1.35 | 0.40 |

FAST is not a guaranteed 230°/s physical angular rate, merely a commanded
upper bound; measured gyro dynamics determine real attainable rate.

## Serial protocol
- Input while idle: \`1\` selects 90°, \`2\` selects 180°, \`b\` BASE,
  \`f\` FAST, \`?\` status. These **never** cause motor drive.
- \`S\` (anytime) stops; PA0 physical STOP is sampled in TIM11.
- On boot \`TP,HELLO\`, \`TP,BOOT\`, \`TP,CAL\` each 1 second while calibrating.
- After calibration \`TP,CAL_OK\`, \`TP,READY\`.
- Physical PA0 triggers \`TP,START\`, then \`TP,D,...\` at 50Hz, then
  \`TP,RESULT\` after 0.8s post-stop sampling.

\`TP,D\` is a positional CSV with 17 columns:
\`\`\`text
TP,D,id,t_ms,mode,profile,phase,yaw_cd,error_cd,gyro_dps10,ref_dps10,pid_pm,ff_pm,cmd_pm,enc_left,enc_right,imu_age_us
\`\`\`
\`phase=1\` means RUN, \`phase=2\` means coast/locked. Angular centidegrees
divide by 100; gyro tenths of degrees/second divide by 10. \`cmd_pm\` is
**requested PWM**, not measured voltage or RPM. Time \`t_ms\` is generated
from TIM11 ticks, NOT UART arrival time. Raw log includes all status frames,
and \`qdrop\` / \`uartdrop\` expose telemetry losses.

## Outputs
For each capture session under \`results/turn_pid_YYYYmmdd_HHMMSS/\`:
- \`raw_uart.log\` unmodified received lines, saved immediately.
- \`run_001/samples.csv\` with decoded 50Hz time series.
- \`run_001/summary.json\` with settle/rise/overshoot/coast drift.
- \`run_001/trace.png\` four aligned charts (when matplotlib installed).
- \`summary.csv\`, \`summary.json\` compares all completed runs.
- \`session.json\` capture metadata.

**Metrics definitions:** Rise is 10% to 90% of target using interpolated
first crossings. Overshoot is max(0,peak-yaw-target). Settling time is the
first sample after which all subsequent yaw remains within ±2°, requiring
>=150 ms trace to the end. Coast drift is final yaw minus yaw nearest
\`stop_ms\`. The result reason must be \`DONE\` and ring/serial drops zero
before comparing speed. If the robot continues rotating after an emergency
stop, physical VM cutoff must take priority.

## Analysis sequence
1. Three BASE 90° attempts on a secured fixture. Verify direction/STOP,
   gyro rate, angle, and telemetry integrity.
2. Three FAST 90° attempts only if BASE valid; compare mean/SD for response,
   overshoot and settling (not just fastest single time).
3. Repeat BASE then FAST at 180°; greater rotation needs separate braking data.
4. Next parameter revisions should change one variable at a time (speed
   limit, braking accel, rate Kp, then optional Ki), retain raw evidence,
   and rebuild the isolated profile for reproducibility. Real-floor testing
   only after safe fixture acceptance.
