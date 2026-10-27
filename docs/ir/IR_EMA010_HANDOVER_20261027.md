# IR EMA 0.10 — handover to two-wall PID
Date: 2026-10-27. Branch: `giomuadongbac`.

## Scope, decision and qualification level

**Operator-selected filter: EMA(alpha=0.10) on ambient-subtracted IR RAW.**
The selection is a pragmatic STATIC handover decision, **not** a claim that a
high-speed motion/dynamic IR filter has passed validation.

* **Accepted: G0 safety and G1 acquisition/transport** from the existing
  motor-locked physical capture set; the 60 mm capture also has
  `valid_capture=true`, 1,199 IRF frames / 30.016 s, 99.86% 40 Hz frame
  coverage, 1,210.31 Hz physical scan rate in the isolated diagnostic,
  150 ON/OFF records, 30 IRH records, and zero reported
  DMA/ADC/scan/UART failures; `locked=1`.
* **Chosen for development: EMA alpha 0.10** on all six channels,
  because it reduced static standard deviation in the measured
  50/60 mm two-wall setup. This is a parameter selection, not proof of
  100% stable Wall PID.
* **Not accepted:** full geometry/range coverage (G2), filter robustness
  across the entire operating envelope (G3), high-speed edge latency
  and missed wall events (G4), other lighting/materials/cross-talk (G5),
  and ADC-to-mm LUT error on independent trials (G6).
* **No autonomous high-speed Wall PID has been enabled or tuned by this
  handover.** Existing motion controllers remain unchanged.

Physical source:
`results/ir_full_qualification/ir_wall_01/runs/20261027_223245_890374_static_static/`
with `setup.json`, `summary.json`, and `data/ir_filter_trace.csv`.

| Channel | 60 mm RAW STD | 60 mm EMA10 STD | STD reduction |
|---|---:|---:|---:|
| L90 | 1.072 | 0.326 | 69.63% |
| L45 | 1.087 | 0.243 | 77.66% |
| L0  | 1.626 | 0.554 | 65.93% |
| R0  | 2.005 | 1.163 | 41.99% |
| R45 | 1.204 | 0.621 | 48.44% |
| R90 | 1.148 | 0.476 | 58.59% |

At 50 mm the raw mean L0/R0 was 558.285/980.175; at 60 mm
460.474/663.600. Different responses **cannot be interpreted as
different distances**. Mechanical alignment, optical characteristics
and reflectivity have not been separated by calibrated reference trials.

## Production-ready interface (signal transport only)

Files:

- `include/ir_wall_signal.h`: pure C, portable value/status contract;
- `src/ir_wall_signal.c`: six simultaneous EMA 0.10 banks, no IO;
- `include/control.h`: `Control_GetWallIRSnapshot` exposed;
- `src/control.c`: feeds EMA once for every newly consumed IR scan in TIM11;
- `tests/regression/ir_wall_signal_native_test.py`: parity with the
  existing diagnostic `IR_FilterStep(...).ema10`, warmup, stale,
  duplicate/invalid data, missing scan resets and 32-bit counter wrap.

**Physical path:**

```
6 IR Tx/Rx -> TIM10 + ADC1 DMA
   -> LED OFF / paired LED ON
   -> max(ON-OFF,0), raw 0..4095
   -> IR_GetLatest() (consumed by TIM11, up to one completed scan / tick)
       |-> existing ir_latest RAW [UNCHANGED]
       |-> IR_WallSignal_Update() -> 6x EMA alpha 0.10
                -> Control_GetWallIRSnapshot() [NEW]
                -> planned Wall PID [NOT YET ACTIVATED]
```

The diagnostic scan rate of ~1,210 Hz is NOT the production filter
update rate: the TIM11 consumer runs at 1 kHz. All filter time
constants must be measured again in the integrated controller.

**Fixed sensor order:**

```
IR_WALL_L90=0, IR_WALL_L45=1, IR_WALL_L0=2,
IR_WALL_R0=3, IR_WALL_R45=4, IR_WALL_R90=5
```

The rear-mounted L90/R90 **face forward**, not toward the back.

**Snapshot sample:**

```c
#include "control.h"

IR_WallSnapshot ir;
if (Control_GetWallIRSnapshot(&ir) == IR_WALL_VALID) {
    uint16_t left_adc  = ir.ema10_adc[IR_WALL_L0];
    uint16_t right_adc = ir.ema10_adc[IR_WALL_R0];
    /* Only ADC values; do NOT subtract left_adc-right_adc
       as a metric centre error. Apply each sensor's own calibrated
       ADC->distance LUT first, then compute wall error in mm. */
    (void)left_adc;
    (void)right_adc;
} else {
    /* NOT READY: warmup, no sample or >3 ms-old sample.
       Wall PID must inhibit corrections and enter its safe fallback.
       No changes to existing legacy motion behaviour are made here. */
}
```

Contract:

* EMA float recursion `y += 0.10f * (raw - y)` per **new**
  successfully acquired scan. Rounded to ADC counts for PID client,
  matching the diagnostic module.
* Wait **32 consecutive valid scans** after startup or a scan gap
  (warmup). A missing scan or timestamp gap >3000 microseconds
  reinitializes the EMA to the latest raw sample to avoid silently
  carrying a stale value into a control correction.
* `status=IR_WALL_VALID` only when warmup is complete and sample age
  is <=3000 microseconds, computed wrap-safely using MCU `micros()`.
* Reject ADC outside 0..4095, repeated/regressing sequence numbers,
  and regressing timestamps. Expose `skipped_scans`,
  `reset_count`, `rejected_frames`; no unbounded staleness.
* Caller must read the snapshot, not take direct pointers to a
  live ISR-owned array. `Control_GetWallIRSnapshot` performs an
  IRQ-guarded copy for main-loop readers; in TIM11 use the existing
  control-context state (do not re-enable IRQs unnecessarily).
* No wall presence, speed selection, turns, motor output, flash
  writes, BLE traffic, or threshold decisions in this module.
* `Control_GetIRSnapshot` remains RAW and unchanged for existing
  diagnostic use. Do not silently change the 90/180-degree motion
  primitive behaviour.

## Wall PID next-stage prerequisites

1. Independently measure left/right distance from optical sensor face
   to wall, establish channel-specific `distance_mm(adc)` curves and
   reference geometry (wall spacing, sensor offset, attitude).
2. Label usable distance interval and wall confidence for L0/R0.
   Detect no-wall/edge and produce clear `LEFT_ONLY`, `RIGHT_ONLY`,
   `BOTH`, `NONE` states before permitting normal wall correction.
3. Compute physically meaningful centre error (mm) from separately
   calibrated left and right distances. Do not combine the ADC counts.
4. Add controlled Wall PID in the motion module, initially wheels
   off floor / low speed; use explicit motion opt-in and independent VM
   cutoff. Define stale-IR and sensor-loss motor fallback.
5. Run dynamic edge tests at high-rate with a synchronized independent
   edge timestamp, check p95 delays and false wall detection. At target
   speed v, displacement due to latency is `v * latency`;
   use that to decide whether EMA 0.10 is fast enough.
6. Benchmark integrated TIM11 ISR duration and 1 kHz deadline, then
   advance through increasing speed/straight/turn tests with logging
   and objective pass criteria. Re-open filter choice if dynamic lag
   exceeds the target.

## Build and tests

```powershell
git pull --ff-only origin giomuadongbac
python tests/regression/ir_wall_signal_native_test.py
python tests/regression/ir_filter_native_test.py
python -m platformio run -e genericSTM32F411CE
```

These software/build tests do not replace flashing and physical
Wall PID acceptance. Running `genericSTM32F411CE` with VM connected
can permit legacy motion commands. Do not execute movement tests
from this handover procedure; keep motor power disconnected until
the separate supervised Wall PID campaign.
