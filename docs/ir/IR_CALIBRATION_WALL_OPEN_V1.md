# Mode 1 — Stationary two-condition IR calibration (wall/open)

This replaces the former four-pivot 360-degree procedure on branch
\`giomuadongbacv3\`. Mode 1 does not command the motors and does not use
encoder or MPU measurements to calibrate IR.

## Physical procedure

### Stage 1 — WALL

1. Put the robot on a flat, stationary surface.
2. Place walls in front, on the left and on the right of the robot.
3. Leave the rear side open (no wall behind the robot).
4. Keep the robot still while firmware captures 150 fresh samples per IR
   channel at approximately 50 samples/s (about 3 seconds).

The six channels are recorded in this order:
\`L0,L45,L90,R0,R45,R90\`.

### Stage 2 — OPEN

1. Wait for the firmware message \`CAL,WAIT,stage=OPEN\`.
2. Move the robot to an open area with no walls around it.
3. Put it down on a flat surface and keep it still.
4. Press the dashboard button **Confirm open-space capture (O)**, send SPP
   command \`O\`, or press the physical KEY once.
5. Firmware captures another 150 fresh samples per channel.

The KEY during either capture stage aborts the calibration. SPP \`P\` or
\`N\` also aborts. The robot never pivots or drives itself.

## Threshold estimation

For each channel \(i\), firmware estimates robust distribution quantiles:

- \(W_{10,i}\): 10th percentile of the WALL samples.
- \(N_{90,i}\): 90th percentile of the OPEN samples.
- \(N_{95,i}\): 95th percentile of the OPEN samples, stored as the open
  baseline in Flash.

A channel passes only when
\[
G_i = W_{10,i} - N_{90,i} \ge 96\ \text{ADC counts}.
\]

The hysteresis thresholds are placed inside the measured separation:

\[
T_{off,i}=N_{90,i}+\frac{G_i}{3}, \qquad
T_{on,i}=N_{90,i}+\frac{2G_i}{3}.
\]

All six channels must pass. If one or more channels fail, firmware reports
\`quality=INSUFFICIENT\` and does not write Flash. This is intentional: the
algorithm must not invent a threshold when the two labelled distributions
overlap or have the wrong polarity.

The histogram uses 64 bins of 64 ADC counts. The quantiles are therefore
robust approximations, not exact order statistics. The minimum separation
of 96 counts is a conservative first acceptance gate and should be tuned
against real robot captures.

## SPP protocol

- \`CAL,START,mode=1,stages=2,samples=150\`: new session.
- \`CAL,STAGE,WALL,...\`: first capture begins.
- \`CAL_IR,stage=0,ms=...,L0=...,...\`: WALL data row.
- \`CAL,STAGE_DONE,stage=WALL,...\`: WALL capture completed.
- \`CAL,WAIT,stage=OPEN,...\`: operator must reposition the robot.
- \`O\`: confirms open-space placement.
- \`CAL,STAGE,OPEN,...\`: second capture begins.
- \`CAL_IR,stage=1,ms=...,L0=...,...\`: OPEN data row.
- \`CAL,THRESH,...\`: per-channel quantiles and candidate thresholds.
- \`CAL,REVIEW,...\`, \`CAL,SAVED,...\`, \`CAL,END,...\`: quality and save result.

The SPP CSV schema is
\`t_ms,stage,L0,L45,L90,R0,R45,R90\`, with stage 0 = WALL and stage 1 = OPEN.
The host stores both phases in one timestamped run under
\`logs/spp_runs/mode 1 - ir calibration wall-open/\`. The adjacent report
contains separate WALL/OPEN statistics and an SVG plot with solid/dashed
traces for the two conditions.

## Flash and safety semantics

On a valid six-channel result, firmware attempts to save the legacy
\`IR_Simple_Calib_t\` record using the checked Flash API. It sets
\`has_open_calib=1\`, stores the OPEN p95 baselines, and writes per-channel
\`wall_on\` / \`wall_off\` thresholds. It leaves center-reference fields
unset because these two placements do not measure a geometrically centred
cell pose.

The candidate is **not auto-applied** to the motion controller
(\`auto_apply=0\`). A successful Flash save does not prove that the thresholds
are suitable for wall following at every distance, surface, angle, or ambient
light condition. Follow with a physical wall-detection test at the actual
maze wall material and intended sensor distances.

If Flash saving fails, the candidate remains available for a manual retry
using \`V\`. If the two distributions do not separate, no Flash write occurs.
