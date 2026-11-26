# Mode 1 — IR 360-degree calibration (v3 opt-in)
Mode 1 select by a physical KEY long-press or SPP 'K' after focus (SPP 'N').
Selection arms the run; motors unlock ONLY once KEY released, fresh MPU gyro
stable, IR fresh, and the existing one-box motor guards accept a pivot.

Sequence: four +90-degree MPU/encoder pivots, each with the existing 100ms
motor braking phase and at least 120ms dwell. A new segment starts only after
the gyro is within +/-2 deg/s for 50ms. No fixed millisecond sleep in TIM11.
Physical KEY during motion, SPP 'P', bad encoder/IMU, and watchdog faults abort.

Firmware produces CAL,START; CAL,PIVOT_START; CAL_IR,ms=...,L0=...,...;
CAL,PIVOT_DONE; CAL,REVIEW; CAL,THRESH; CAL,SAVED or CAL,FAULT.
CAL_IR carries exactly t_ms,L0,L45,L90,R0,R45,R90 in its dedicated host CSV.
At 20Hz log rate this is NOT a full-rate raw IR scan. MPU/encoder diagnostics
remain in the existing PIVOT_CSV stream, NOT inside the CAL_IR CSV file.

**Critical calibration limitation**: current v3 estimates candidate hysteresis
thresholds from the per-channel observed minimum and maximum during the sweep.
These extrema are *not labelled WALL/OPEN* by geometric ray-casting, so
candidate thresholds have not been physically validated. CAL,REVIEW reports
quality=CANDIDATE_ONLY and auto-apply is always disabled. The user must review
plots and explicitly send 'V' to persist the candidate to the legacy
IR_Simple_Calib_t sector (never while motor/control are running). This is
NOT a validated mm-distance LUT, nor an automatic Wall PID calibration.
No Flash write takes place during turning. A failed run never saves.

Start with wheels off the floor and an independent motor power cutoff.
The old 90-degree primitive was not validated as a continuous four-turn
maneuver; if any pivot overshoots or cannot settle, STOP and inspect logs.
The yaw-wrap fix applies to individual 90-degree turns at +/-180 degrees.
NOTE: this calibration does not compensate for sensor geometry or sample
latency; geometric wall labeling and quantile thresholds are follow-on work.
