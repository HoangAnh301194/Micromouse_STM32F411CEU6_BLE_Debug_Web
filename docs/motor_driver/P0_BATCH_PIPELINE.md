# P0 STOP/Coast/Brake — one-shot batch qualification

Branch: `feature/motor-driver-optimization`

Entry point: `tools/motor_p0_batch_pipeline.py`

## Purpose and limits

Use one command to execute the entire Motor Driver P0 **offline** suite rather than repeating individual scripts. It does **not** reflash the STM32, open a serial port, send UART START, or energize a motor. It does not change Motion/PID or run an active-brake experiment.

The batch runs these eight existing test gates: Toshiba truth table, source audit, state transition/preload and locked-UART regression, v14 63-stage parser, v16 87-stage parser, v17 integrated 63-stage parser with archived v7 replay, v18 integrated 87-stage parser, and full qualification self-test. It runs all checks even if one fails and writes failure stdout/stderr for diagnosis.

```powershell
git fetch origin
git pull --ff-only origin feature/motor-driver-optimization
python tools/motor_p0_batch_pipeline.py --run
```

A single timestamped `motor_logs/p0_batch_*/` directory will contain:

- `p0_batch_report.json`: machine-readable result and per-gate evidence.
- `p0_batch_summary.txt`: human-readable matrix and remaining unmeasured tests.

The expected **offline** result is `PASS_OFFLINE` for all eight gates and `REVIEW_AWAITING_PIN_CAPTURE` overall. A successful process exit code does **not** mean that the TB6612 has been electrically qualified.

For pipeline code self-test (synthetic waveforms deliberately not treated as measurement):

```powershell
python tools/motor_p0_batch_pipeline.py --self-test
```

GitHub Actions runs the same aggregate self-test with Python 3.11 and 3.14, runs the full one-shot report, and uploads `motor-p0-batch-no-hardware` as a workflow artifact.

## Twelve-case oscilloscope import (optional, future measurements)

Once scope captures from an independently reviewed, instrumented and deliberately controlled test are available, put **12 actual captured** CSV files in one directory. The pipeline does not create, flash, or execute the test that generates these signals.

| Case | Left-wheel filename | Right-wheel filename | Checks |
|---|---|---|---|
| DRIVE → COAST | `drive_to_coast_left.csv` | `drive_to_coast_right.csv` | Direction initially active, coast settled with IN1=IN2=0 and PWM HIGH |
| COAST → DRIVE | `coast_to_drive_left.csv` | `coast_to_drive_right.csv` | Coast first, then forward drive with PWM chopping, no full-high initial period >34 μs |
| DRIVE → BRAKE | `drive_to_brake_left.csv` | `drive_to_brake_right.csv` | Direction initially active; IN1=IN2=HIGH at end |
| BRAKE → COAST | `brake_to_coast_left.csv` | `brake_to_coast_right.csv` | Brake first, settled Coast at end |
| FORWARD → REVERSE | `forward_to_reverse_left.csv` | `forward_to_reverse_right.csv` | Both directions observed, neutral gap at least 30 ms |
| REVERSE → FORWARD | `reverse_to_forward_left.csv` | `reverse_to_forward_right.csv` | Both directions observed, neutral gap at least 30 ms |

Each CSV **must** have a header exactly containing at least `t_us,in1,in2,pwm`; entries are sampled digital 0 or 1, and `t_us` is monotonically increasing in microseconds (can start at any origin). Example of the *format only*:

```csv
t_us,in1,in2,pwm
0,1,0,1
1,1,0,0
2,1,0,0
```

Acquire four channels per motor (IN1, IN2, PWM and time). The maximum sampling interval must be **≤2 μs** (at least 500 kS/s with no gaps); traces must include ≥1 ms of clear pre- and post-transition states, and span at least 4 ms. For reversal, include more than 30 ms of neutral time in the captured window. These requirements are intended to avoid misclassifying a few samples as a valid transition; they do **not** guarantee detection of narrow physical glitches.

When the **real** measurements exist:

```powershell
python tools/motor_p0_batch_pipeline.py --run --waveforms .\motor_logs\p0_scope_capture
```

All 12 captures are processed in one run; the report lists each trace, first/last stable states, observed reversal interval, LOW/LOW/PWM-low samples and sampling cadence. With all pin traces accepted, the gate is `PIN_SEQUENCE_REVIEW` and overall `REVIEW_PIN_LEVEL_ONLY`. It is **never** an automatic electrical PASS: PWM/IN pins are not H-bridge OUT1/OUT2, current or temperature. Malformed/incomplete captures produce nonzero exit status and `FAIL_PIN_CAPTURE`.

## P0 acceptance matrix

| Gate | Evidence | Qualification |
|---|---|---|
| Source, truth table, RCC/TIM2 init, deadtime, UART interlock | Eight offline gates | `PASS_OFFLINE`, only for the checked source/model |
| GPIO/PWM pin sequence on two wheels | All 12 measured CSV traces | `PIN_SEQUENCE_REVIEW`, subject to engineer review |
| TB6612 output terminals, short-brake current, thermal rise, loaded behavior | External electrical instrumentation, independent safety review | `BLOCKED` until supplied |

The existing v7 result `PASS_BENCH` at VM=8.0 V validates that historical firmware in its tested conditions; it does not validate changed STOP_HIZ transitions. The PCB still reports v7 unless a new firmware image is intentionally flashed. This batch deliberately does not flash or arm one.

Use the report to decide whether to proceed to supervised electrical validation; do not enable an automatic Brake/Reverse drive path as a shortcut.
