# v2 three-placement IR calibration ported to giomuadongbacv3

## Motivation
The pre-existing Mode 1 in v3 uses autonomous 4×90° pivots and an unlabelled
P10/P90 ADC distribution. The v2 calibration is **not equivalent**: it uses
three operator-labelled placements, 200 fresh samples per position, and checks
whether open and wall observations are separable.

This integration adds an **opt-in static method** without replacing v3's
360° path or weakening its pivot / heartbeat safeguards.

## Select and capture (SPP on COM Outgoing)
1. Start with motors stopped and physically remove motor power for
   static measurements; ensure the sensors are powered and scanning.
2. Focus on menu **Mode 1** (do not select the default 360° method).
3. In the SPP dashboard press **Calib 3 vị trí (C)**. Command `C` selects
   Mode 1 internally and starts the static workflow; there is no pivot.
4. Put the robot centred in a cell with a **front wall and both side walls**.
   Press **Đo vị trí (G)**, or press KEY briefly. Do not move during sampling.
5. Place the robot between **two side walls, no front wall**; capture again.
6. Place the robot in a **clear/open area**; capture once more.
7. Inspect `CAL_STATIC,RESULT` for all six sensors and
   `CAL_STATIC,REVIEW,quality=STATIC_CANDIDATE`.
   Press **Review → Save Flash (V)** only after explicit inspection.
8. `P` aborts the current capture, `K` exits the static method.

The static method has no motor commands. The default KEY long-select of
Mode 1 still chooses the *v3 360°* calibration. To enter static mode,
use `C` over SPP first; subsequent capture/exit can use KEY.

## Algorithm carried over from v2
Each placement: settle 250ms, then read 200 fresh 6-channel IR samples
at >=10ms intervals, no ISR logging. A capture times out at 5 seconds.
The per-channel median is the average of sorted observations 99 and 100
(0-indexed). Approximate 5th/95th percentile boundaries use sorted
samples 10 and 189.

For side-facing L0/R0: open=OPEN_SPACE; wall=minimum of the two wall
placements. For front/diagonal sensors: open=maximum of NO_FRONT and
OPEN_SPACE; wall=THREE_WALLS. Require wall-open >=50 raw ADC counts,
non-overlapping high-open/low-wall reference bands, and no saturation
(>=4090). Rejections restart the whole three-placement sequence.

For accepted references, `ON=open+0.6*(wall-open)` and
`OFF=open+0.3*(wall-open)`. The full v2 IR_Simple_Calib_t mapping,
centre references and relative right-side gains are populated.

**Safety change from v2:** successful capture enters REVIEW only.
There is no automatic Flash write. `V` writes while motors are locked,
control/IR stopped temporarily; acquisition is restarted afterwards.
The static calibration is a threshold candidate, not a validated
distance-versus-ADC lookup table.

## Verification
```powershell
python tests/regression/ir_calibration_static_port_test.py
python tests/regression/ir_calibration_spp_test.py
python -m py_compile tools/classic_spp_dashboard.py
pio run -e mode_menu_96m
```
Source-level tests cannot prove C compilation or timing. Check the
physical signal before accepting Flash. In particular, six channels near
zero (as seen in the latest 2026-11-26 captures) must be rejected, not
saved.

**Log protocol:** `CAL_STATIC,ENTER/WAIT/START/STAGE_OK/RESULT/REVIEW/
REJECT/SAVED/EXIT` reports go to the status terminal. This implementation
does not add a separate raw CSV for the static captures; raw IR
visualization remains available in the System Test / IR test.
