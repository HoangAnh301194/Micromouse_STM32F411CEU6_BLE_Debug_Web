# IR full qualification — PA0 / PC13 / six channels

Status: **IMPLEMENTED FOR GUIDED MOTOR-LOCKED MEASUREMENTS**, not hardware accepted. No experiment was physically run as part of authoring this firmware/tool. Existing production firmware, turn PID and Wall PID are unchanged.

## 0. Mandatory wiring and safety checklist

- [ ] ST-Link connected and recognized by Windows.
- [ ] USB-TTL: MCU PA9 TX → adapter RX, common GND, 3.3V levels; choose actual COM using `python -m serial.tools.list_ports`.
- [ ] Motor VM physically unplugged (not merely PWM=0). Do not reconnect for this entire campaign.
- [ ] MCU 3.3V supplied and stable; battery 2S motor VM disconnected.
- [ ] PA0 button can be pressed and released with robot stationary; PC13 onboard LED visible.
- [ ] Fix robot and measure fixture from **optical face of each sensor**.
- [ ] Do not manipulate a wall so abruptly that the robot or wired PC connections are struck.
- [ ] Keep this process isolated from motor test, turn PID, BLE and OLED.
- [ ] Read the host report and confirm `locked=1` before accepting a burst.

## 1. Light and button contract

| PC13 LED | Meaning |
|---|---|
| One short ON pulse per second | READY: circular buffer continuously recording |
| Steady ON | PA0 press-release accepted, IR scan stopped, UART transmitting frozen samples |
| Fast 5Hz blinking | IR scan stale or motor lock fault; stop and investigate |

Press-and-release PA0 after the intended movement has happened. The record contains the **preceding** 1024 IR scans, about 0.85 seconds at 1210Hz. It does not contain the entire motor travel, and PA0 marks the **end of capture**, not the physical time of the wall edge.

The trace consists of raw ON-OFF ADC, Median3, EMA0.10, EMA0.25 and Median3→EMA0.25 updated per IR scan, with MCU `us` timestamp and scan counter. Each press transmits `IB,BEGIN`, 1024 `IBD` records and `IB,END`. Transmission pauses sampling for roughly 25-30 seconds at 115200 baud; do not move again until PC13 pulses slowly and UART prints READY. The collector detects missing/damaged frames. There is no UART start or motor enable command.

## 2. Setup map and ordered checklist

Use top-view coordinates: **forward = +Y, left = -X, right = +X**. Both side walls parallel with Y. L0 points -X and R0 +X. L45/R45 diagonal forward. L90/R90 are physically in rear positions but **point forward**, not backwards. All distances to be measured from each optical sensor face along its viewing ray; never assign distance from the maze cell center to every sensor.

- [ ] **A: No-wall baseline.** Clear all six viewing directions, 30s, ordinary room light; inspect L90/R90 for background reflections.
- [ ] **B: Two-side-wall static.** Robot centered and secured. Record L0→left wall and R0→right wall separately. White matte walls, angle ≈90° relative to lateral beams. Three **independent reseats** per distance. Suggested 20, 25, 30, 35, 40, 45, 50, 55, 60, 70 and 80mm; stop at out-of-range. 20–30s per run. Optional coarse screening can start at 40/50mm.
- [ ] **C: Single-edge crossing.** With VM unplugged, manually slide a single wall card across L0's beam (then R0), or translate *secured, unpowered* robot past a visible wall ending. Press PA0 immediately at end of edge. Collect at least 10 wall-appearance and 10 wall-disappearance bursts for each sensor in near/mid/far range.
- [ ] **D: Yaw sweep.** With walls unchanged, manually rotate the unpowered robot on a marked jig by +5°, -5°, +10°, -10°. Capture L0/R0/L45/R45 before and after. Record angle with external protractor; do **not** label angle as from an IMU—the burst firmware only captures IR.
- [ ] **E: Left only / right only.** Remove exactly one wall and repeat static/edge at the remaining wall. Helps identify geometric visibility vs optical/electrical cross-talk.
- [ ] **F: Diagonal and front.** Place isolated matte walls in L45 and R45 directions, then in the forward beams of rear-mounted L90/R90. Measure angle/distance separately.
- [ ] **G: Ambient/material changes.** Repeat representative no-wall, static and edges in normal and stronger light, matte white/black and glossy target. Record source, angle and fixture uncertainty.
- [ ] **H: Calibration and independent holdout.** Only fit six individual LUTs after sufficient repeatable ADC-distance coverage; set out-of-range limits and retain untouched placements/distance values for holdout.

IMPORTANT: If the fixture uncertainty is ±5mm, do not claim a 2mm MAE has been validated. Improve with sliding measurement jig (~±1mm or better if available).

## 3. One-time preparation (PowerShell at repo root)

```powershell
git pull --ff-only origin giomuadongbac
python -m pip install pyserial matplotlib
python tools/ir_full_qualification.py plan
python tools/ir_full_qualification.py preflight --build
python tools/ir_full_qualification.py init --session ir_wall_01
```

### A/B: Long static acquisition using existing 40Hz motor-locked firmware

Physically disconnect VM before uploading:

```powershell
python -m platformio run -e ir_filter_diag_96m -t upload
python tools/ir_full_qualification.py capture --session ir_wall_01 --kind static --port COM6 --case no-wall --vm-off --seconds 30 --surface none --light room --trial 1 --notes "no target wall visible"
python tools/ir_full_qualification.py capture --session ir_wall_01 --kind static --port COM6 --case static --vm-off --seconds 30 --distance-left 50 --distance-right 50 --surface white_matte --light room --trial 1 --notes "both lateral beams measured individually, uncertainty +/-1mm"
```

Use **actual COM and actual measured distances**. Running the identical fixture repeatedly does not count as independent reseat. Reposition / remeasure after each run.

### C/D/E/F/G: High-rate dynamic/yaw acquisition

Upload the separate burst firmware with motor VM still physically unplugged:

```powershell
python -m platformio run -e ir_burst_diag_96m -t upload
python tools/ir_full_qualification.py capture --session ir_wall_01 --kind burst --port COM6 --case side_edge_L0 --channel L0 --direction rising --vm-off --trial 1 --notes "manually insert LEFT wall edge, then release PA0"
```

When PC says READY and PC13 pulses slowly:

1. Make the intended wall appear/disappear, or perform a manual yaw.
2. Press and release PA0 immediately after the event (inside the preceding ~0.85s ring window).
3. Watch PC13 steady ON for UART dump, allow all 1024 frames to transfer without moving.
4. Check `summary.json` for valid trace and any `edge` classification. If event not captured, retry as a new trial.
5. Repeat with `--direction falling`, `--trial 2`, etc. The 10+ rising/falling event requirement is per target channel and range.

Example yaw (manual, with external angle marking):

```powershell
python tools/ir_full_qualification.py capture --session ir_wall_01 --kind burst --port COM6 --case side_yaw_left_5 --channel L45 --direction yaw_left --vm-off --trial 1 --notes "MANUAL yaw about center, protractor +5deg, walls static"
```

### Analysis and checkpoint

```powershell
python tools/ir_full_qualification.py status --session ir_wall_01
python tools/ir_full_qualification.py report --session ir_wall_01
```

All outputs are in `results/ir_full_qualification/ir_wall_01/runs/`; no raw log is erased or overwritten. Pausing is safe: rerun `capture` to continue the same session, using a unique timestamp. Report `acceptance.json` deliberately stays INCONCLUSIVE until actual independent timing and full test coverage are available.

## 4. Acceptance criteria

G0: VM disconnected confirmed, motor outputs locked; G1: error-free, scan-sequential raw burst. G2: per-sensor independent geometry/distance coverage; G3: static noise reduction + repeatability; G4: real wall edge detection latency and missed edges; G5: alternative illumination/material and one-wall cross-talk; G6: independent LUT accuracy.

**No false motor-on acceptance.** This campaign tests sensor behavior while a human moves the wall or unpowered robot. It does not measure a powered robot's actual acceleration, yaw IMU angle, wall PID tracking, vibrations or servo stability. Powered straight/slalom/turn testing must be a *different* opt-in firmware and safety qualification with operator approval, VM cutoff, speed/PWM caps and deadman supervision.

**Do not use RAW crossing as a physical absolute event clock.** The burst parser only reports relative filter lag against RAW. For a 5ms absolute event latency requirement, add a synchronized external marker (photogate/GPIO or event-triggering actuator) and rerun dynamic tests before claiming G4 PASS.
