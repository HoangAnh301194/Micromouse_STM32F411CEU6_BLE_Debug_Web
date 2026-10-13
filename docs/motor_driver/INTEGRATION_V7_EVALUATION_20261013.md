# Motor Driver v7 + MPU6050/I2C + IR integration review — 2026-10-13

## Evidence and test boundary

- User-pushed actual hardware session: `motor_logs/auto_20261013_113713/`, including UART raw log, CSV, 63-stage report and metadata.
- Qualification report: `motor_logs/qualification_20261013_113830/qualification_report.json`.
- Firmware `motor_auto_sensor_96m_trim25`, protocol v7, 3 rounds x 21 motor stages, 200ms PWM and 400ms software-neutral after each; 8–20% single-wheel, both 16% maximum.
- **VM 8.0V** from user-reported handheld measurement; voltage is not sampled by MCU.
- Hardware: elevated/free-spinning wheels; this is an interoperability/timing bench, NOT motor current/thermal qualification or MPU/IR physical accuracy certification.

## Measured bench integrity

| Signal / gate | Result | Interpretation |
| --- | ---: | --- |
| START / END / POST_END | 63/63/63 | All intended 63 motor stages completed |
| Motor encoder/UART samples | 1,890 | 30 per stage, drive+neutral windows |
| UART host KEEP sent / MCU accepted | 192/192 | Bidirectional host command path worked |
| UART RX overrun / framing / ring drop; TX drop | 0/0/0; 0 | No UART errors recorded |
| TIM11 ISR late (>50us threshold) / gap (>1.5ms) | 0 / 0 | No timing warning recorded at 1kHz |
| Highest observed TIM11 ISR execution | 1749 DWT cycles = about **18.22us @ nominal 96MHz** | Recorded maximum only, not worst-case execution proof |
| I2C errors / timeouts | 0 / 0 | No failures recorded |
| Fresh gyro sample increase | +53,031 | MPU acquisition progresses during motor test |
| IR complete scan count increase | +73,965 | IR state machine/ADC scan progresses during motor test |
| SENSOR drive/post telemetry records | 126 = 2 x 63 | Expected phase count |
| GPIO/CCR read-only neutral snapshots | 23 | CCR=0, all four IN pins LOW, OC1PE/OC2PE enabled and ARR=3199; **register values do not prove bridge terminal mode** |
| Host analyzer status | `TRACE_COMPLETE_NOT_ELECTRICAL_APPROVAL` | Valid trace, not electrical safety approval |
| Qualification P1-INTEGRATION | `PASS_BENCH` | Supported within this precise low-duty, elevated-wheel experiment |

## Reconcile the alarming `missing=9000`: **pre-calibration**, not new missing samples during motor run

In `src/motor_auto_bench.c` the integrated program calls `Control_Init(&integration_mpu)` and `Control_Start()` **before** MCU has calibrated MPU6050 and signaled UART READY. Therefore TIM11 begins ticking while the MPU snapshot is still absent. In `src/control.c` the `MPU_INTEGRATION_BENCH` TIM11 ISR increments `integ_missing` when `MPU6050_GetSnapshot()` has no valid sample (or sample age >3000us). This is expected before calibration is ready and has no direct bearing on motor-stage reliability. `MOTOR_AUTO_WAIT_SENSORS,cal=1` appears repeatedly before START.

- First actual motor-stage sensor record: `missing=9000` (round 0 stage 0 drive).
- Final motor-stage sensor record: `missing=9000` (round 2 stage 20 post).
- **Missing/expired delta during observed motor-stage intervals: 0**.
- `reused`: 0 → 8, or 8 / 53,031 fresh-sample increments = **0.015%**.
- `aged` (>1.5ms): 0 → 1, or 1 / 53,031 = **0.0019%**.
- I2C `err` and `timeout`: 0 → 0.

The raw `missing=9000` field should NOT be treated as 9000 missing samples while the motor drives, nor silently ignored in reporting. Host pipeline now emits `missing_before_motor_phase`, `missing_delta`, `reused_delta`, `aged_over1500_delta` and `reused_per_1000_fresh`, with a regression test for nonzero initial missing but zero new missing. These new fields can be computed by **offline re-analysis** without MCU firmware update/reflash. The full qualification report now checks the missing delta before granting `PASS_BENCH`.

## Motor response cross-check at 8.0V

Encoder counts mean across three 200ms trials, single-wheel:

| Duty | Left ticks | Right ticks | Left advantage relative to right |
| --- | ---: | ---: | ---: |
| 12% | 144.00 | 123.67 | +16.44% |
| 16% | 196.67 | 184.33 | +6.69% |
| 20% | 251.00 | 236.00 | +6.36% |

Together at 16%, left `191/189/190`, right `190/189/190`, almost identical. Startup: lowest duty with 3/3 sustained encoder movement in this 3-round bench is left 10%, right 9%; values are only tentative and not robust loaded deadband settings. The 12% wheel asymmetry differs from earlier motor-only v6, but **VM changed from 8.2V to 8.0V, test order and motor conditions differ**; therefore no causal conclusion about MPU/IR interference can be drawn from a non-paired comparison.

## Gyro / IR module boundary

Gyro 20ms window absolute angular-rate proxies averaged **47.78 mdps while PWM on** and **44.94 mdps in software-neutral stage windows**; the maximum in a drive window was **286 mdps**. These numbers are a diagnostic proxy, not gyroscope precision, calibrated drift, isolated electromagnetic interference, or a vibration spectrum. IR complete scan count advanced, but individual distance accuracy/linearity/sensor thresholds were not validated here.

This Motor Driver integration profile deliberately runs `Control_SetDiagnosticOnly(1)` and skips Motion/PID control tasks. It verifies **sensor acquisition progress + motor short-pulse generation + TIM11 scheduling + UART**, NOT simultaneous full-motion wall-following workload, closed-loop path accuracy or loaded race performance.

## Remaining gates after this successful session

| Gate | Status | Decision |
| --- | --- | --- |
| PWM and integrated sensor acquisition at <=20% | **PASS_BENCH** | Accept this result for one off-ground short-pulse run; no identical rerun necessary |
| STOP/Coast/Brake | REVIEW | MCU GPIO/CCR snapshot works; electrical output state and current need independent measurement; don't energize short-brake/reverse blindly |
| Motor current/voltage ripple/thermal | BLOCKED | External instruments/data needed; encoder and UART cannot substitute |
| Startup PWM robustness | REVIEW | Retry independent/reordered low-duty experiments only if the motor driver's scope needs stronger deadband statistics |
| Combined 30–60% motor + sensor | Not run in v7, only motor-alone v6 | Keep gated by external power-stage measurements; v7 did not exercise high PWM |
| PWM 80–100%, long sustained, loaded | BLOCKED | Do not infer high-speed current/thermal performance from short elevated-wheel stages |

## Tool changes based on log (analysis only)

- `tools/motor_auto_pipeline.py`: record baseline and deltas for missing/reused/age in sensor integration JSON; regression tests.
- `tools/motor_full_qualification.py`: `--audit` auto-selects latest v7/v8 sensor session in `motor_logs/auto_*`, includes it in qualification instead of displaying `NOT_RUN` after a successful push; `--integrated-log` supports explicit choice. Normalize old reports' cumulative counters and downgrade integration from PASS if `missing_delta >0`.
- **No MCU PWM, electrical brake or sensor-calibration firmware changes**. The complete v7 measurement can be re-audited offline.

## Run locally after Git pull, no motors and no flash

```powershell
git fetch origin
git switch feature/motor-driver-optimization
git pull --ff-only origin feature/motor-driver-optimization

python tools/motor_auto_pipeline.py --folder motor_logs/auto_20261013_113713
python tools/motor_full_qualification.py --audit
```

The first command only reparses saved raw UART, updating the existing run's host-generated JSON and text summaries. The second creates an updated `motor_logs/qualification_...` result with `P1-INTEGRATION=PASS_BENCH` and explicit missing/reused/age deltas (if data are still complete). **It does not flash, energize or run the motor**.
