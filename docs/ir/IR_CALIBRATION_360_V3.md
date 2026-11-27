# Mode 1 — IR 360-degree calibration (v3 opt-in)
Mode 1 select by physical KEY long-press or SPP 'K' after focus (SPP 'N').
Selecting Mode 1 immediately arms a 360-degree session; no three-placement
operator input is required. The app displays START only upon firmware
CAL,START,mode=1,segments=4 and END only upon firmware CAL,END.
Motors begin movement at PIVOT_START, after all guards pass.
Selection arms the run; motors unlock ONLY once KEY released, fresh MPU gyro
stable, IR fresh, and the existing one-box motor guards accept a pivot.

Sequence: four +90-degree MPU/encoder pivots, each with the existing 100ms
motor braking phase and at least 120ms dwell. A new segment starts only after
the gyro is within +/-2 deg/s for 50ms. No fixed millisecond sleep in TIM11.
Physical KEY during motion, SPP 'P', bad encoder/IMU, and watchdog faults abort.

Firmware produces CAL,START; CAL,PIVOT_START; CAL_IR,ms=...,L0=...,...;
CAL,PIVOT_DONE; CAL,REVIEW; CAL,THRESH; CAL,SAVED; CAL,END.
Faults emit CAL,FAULT,reason=... followed by CAL,END,result=FAILED.
Successful completed-and-saved session emits CAL,END,result=SUCCESS.
Rejected low-signal sessions emit CAL,END,result=INSUFFICIENT;
Flash failures emit CAL,END,result=SAVE_FAILED.
MCU packet format is exactly:
CAL_IR,ms=123,L0=120,L45=230,L90=340,R0=450,R45=560,R90=670
The dedicated host CSV has t_ms,L0,L45,L90,R0,R45,R90 columns.
At 20Hz log rate this is NOT a full-rate raw IR scan. MPU/encoder diagnostics
remain in the existing PIVOT_CSV stream, NOT inside the CAL_IR CSV file.

**Critical calibration limitation**: current v3 estimates candidate hysteresis
thresholds from per-channel 10th and 90th percentile approximations using
32 histogram bins (128 ADC per bin). These samples are *not labelled WALL/OPEN*
by geometric ray-casting, so
candidate thresholds have not been physically validated. CAL,REVIEW reports
quality=CANDIDATE_ONLY and auto-apply is always disabled. The user should
review the saved plots; if all six spans pass the quality gate, firmware
persists the candidate automatically to the legacy
IR_Simple_Calib_t sector (never while motor/control are running). This is
NOT a validated mm-distance LUT, nor an automatic Wall PID calibration.
No Flash write takes place during turning. A failed run never saves.

Start with wheels off the floor and an independent motor power cutoff.
The old 90-degree primitive was not validated as a continuous four-turn
maneuver; if any pivot overshoots or cannot settle, STOP and inspect logs.
The yaw-wrap fix applies to individual 90-degree turns at +/-180 degrees.
NOTE: this calibration does not compensate for sensor geometry or sample
latency; geometry-labelled wall/open classification remains follow-on work.

## Host, review, and recorded artifacts
A physical KEY selection is autonomous and uses a local lease renewer, while a
selection sent through SPP must receive the dashboard's 100ms host ping; losing
the host stops the pivot within the existing 350ms lease. After each pivot,
the gyro must settle, and a newly added check rejects segment errors >8deg
or a 360deg round-trip error >8deg. 100ms driver brake plus at least 120ms
dwell means a few-ms break is **not safe or guaranteed**.

SPP dashboard Mode 1 shows six ADC bars: blue means the current value
crosses that channel's ON threshold; grey means it does not. During capture,
before firmware CAL,THRESH is available, the UI derives a provisional
p10/p90-based hysteresis threshold from the current session samples. This
estimate is display-only; the final firmware CAL,THRESH replaces it.
The left upper terminal is locked to CAL_IR and shows
t_ms,L0,L45,L90,R0,R45,R90. The right terminal shows only menu focus and
selection steps (Mode / Selected mode / selected test submenu); it does not
duplicate CAL, pivot, boot, or test runtime logs.
After the motor stops, CAL_LIVE,valid=...,L0=...,... is sent every 100ms
for *display only*. CAL_LIVE is not appended to the completed per-run CSV.
Invalid scans show an explicit warning rather than meaning NO WALL.
A short physical KEY press after a completed or failed Mode 1 run arms a
new 360-degree calibration, clears the previous in-flight capture, and
creates a fresh host CSV. During motion, KEY still aborts for safety.
After END, the SPP dashboard's Calibrate Again button sends the dedicated
single-byte command R. Firmware accepts R only when Mode 1 remains selected,
the previous run is in REVIEW/SAVED/FAULT, and no pivot/motor motion is active;
otherwise it replies CAL,RESTART_REJECTED. A successful request replies
CAL,RESTART_ACCEPTED and begins a new CAL,START session.
The Tkinter dashboard rejects CAL,WAIT,stage=... (legacy static method)
and CAL_IR,valid=... as an incompatible v3 protocol, rather than
mistaking these packets for 360-degree acquisition.
The calibration pivot speed reference is 800 PPS; ordinary pivot remains
1100 PPS. This is a wheel encoder speed reference, not a guaranteed angular
velocity or direct PWM target.
On successful review, the session CSV is closed in the stable folder
`logs/spp_runs/mode 1 - ir calibration 360/`, with a new timestamp filename
on each run. It contains exactly `t_ms,L0,L45,L90,R0,R45,R90`.
When all six threshold messages arrive, Python also writes an adjacent
`*_summary.json` (min/max/mean/std/p10/p90/thresholds) and
`*_ir_plot.svg` (six traces over MCU elapsed time). These artifacts need
the desktop SPP session; a KEY-only physical run without a PC still executes
but cannot save a CSV until a host captures the serial stream.

After CAL,REVIEW quality=CANDIDATE_ONLY, firmware attempts the safe
Flash save automatically and emits CAL,SAVED,flash=OK,auto_apply=0.
If quality is INSUFFICIENT, Flash is not written. If Flash fails,
the candidate stays in REVIEW and the manual V retry is available. Flash save uses the legacy struct with
`has_open_calib=0`; no production Wall PID automatically imports the saved
candidate. STOP/P aborts motion and never commits Flash.

Run before flashing from Windows PowerShell:
```powershell
python tests/regression/ir_calibration_spp_test.py
python tests/regression/ir_calibration_report_test.py
python -m py_compile tools/classic_spp_dashboard.py
pio run -e mode_menu_96m
```
Start all motor experiments with wheels on a stand and a separate power
cutoff. Then check a clear 165mm cell with the entire body swept envelope.
