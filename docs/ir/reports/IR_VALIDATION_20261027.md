# IR six-channel validation — 2026-10-27

**Status:** `BLOCKED_HARDWARE`
**Qualification:** `NOT_QUALIFIED`
**Decision:** no filter selected for production or Wall PID. No physical sensor data was collected.

## 1. Provenance, software and safety

| Item | Result |
|---|---|
| Repository / branch | `HoangAnh301194/Micromouse_STM32F411CEU6_BLE_Debug_Web` / `giomuadongbac` |
| HEAD used | `df5daa24cd2c1cb0ae7f50c7cfc304770850ecac` |
| Worktree | Pre-existing user change in `.vscode/settings.json` preserved; task changes are limited to IR capture tooling and this report/evidence |
| Firmware profile | `ir_filter_diag_96m`, built successfully; **not uploaded** |
| Production firmware | `genericSTM32F411CE`, build successful |
| Host | Windows, Python 3.14.3, PlatformIO 6.2.0, pyserial 3.5, matplotlib 3.10.8 |
| UART configuration | Source config is USART1 PA9/PA10, 115200 baud; COM6 was reported connected by the operator but was not opened |
| Native / host tests | Complete local workflow test set passed after the capture-pipeline change (see §2) |
| GitHub Actions | [Run 149](https://github.com/HoangAnh301194/Micromouse_STM32F411CEU6_BLE_Debug_Web/actions/runs/40495050813) succeeded on `4c320200d1a036073993fc8f8e76b8290b756753`, which predates current HEAD; local checks below were rerun on current HEAD plus working changes |
| Operator-reported hardware state | UART/ST-Link connected and robot secured; **motor VM remains physically connected and powered** |
| Safety decision | G0 is blocked. No firmware upload, serial capture, motor command, or physical measurement was attempted. |

The `IR_FILTER_DIAG` source calls `Motor_LockOutputs()` and checks `Motor_OutputsLocked()` before starting acquisition. This is a firmware safeguard, not a substitute for the required physical VM disconnect. The operator explicitly reported that VM is connected and on, so the hardware gate cannot be waived.

## 2. Source audit and software verification

Reviewed [the qualification protocol](../IR_AGENT_QUALIFICATION_PROTOCOL.md), [the calibration plan](../IR_FILTER_CALIBRATION_PLAN.md), [the acceptance template](../IR_AGENT_ACCEPTANCE_REPORT_TEMPLATE.md), and the actual `src/ir_sensor.c`, `src/ir_filter.c`, `src/ir_filter_diag.c`, `tools/ir_filter_pipeline.py`, `platformio.ini`, and CI workflow.

Source observations:

- The IR driver uses TIM10-driven states and ADC1/DMA2 Stream0 to acquire ambient plus three paired emitter groups. The six channel values are `max(ON-OFF, 0)`. This is source-level behavior; ADC, emitter operation, scan rate, and DMA health have not been observed on this board in this task.
- The diagnostic profile locks motor outputs in firmware, logs IRF at a nominal 40 Hz, paired IRB at 5 Hz, and IRH at 1 Hz. The filter candidates are updated when each completed scan is consumed. The UART-decimated IRF stream cannot certify millisecond dynamic response.
- Capture validation previously accepted as few as 60 IRF rows and did not require the diagnostic HELLO, IRB coverage, a measured scan rate, or zero ADC overrun / DMA timeout deltas. That was too permissive for an acceptance gate.
- The capture parser now requires the motor-locked HELLO and IRH scan counter, checks 95% stream coverage against the requested/observed capture duration, checks IRB and IRH coverage, motor-lock snapshots, errors, drops, and a positive measured scan rate. Per-channel filter metrics now include P95/P99, noise-reduction percentage, bias, isolated one-sample spike count/rate, and ADC saturation counts/rates. Isolated-spike classification is a robust diagnostic (`6 × 1.4826 × MAD`, minimum 1 ADC count), not an acceptance threshold.
- No source change was made to IR acquisition, filter coefficients, Motion, PID, motor driver, or production `IR_GetLatest()` behavior.

Validation run on the current checkout:

| Check | Result |
|---|---|
| `python tests/regression/ir_sensor_native_test.py` | PASS — asynchronous state machine, snapshots, failure/timeout/restart behavior under register mocks |
| `python tests/regression/ir_filter_native_test.py` | PASS |
| `python tools/test_ir_filter_pipeline.py` | PASS — parser, coverage guard, HELLO rejection, synthetic metric/step tests; synthetic only |
| Full workflow host/native commands: encoder, IR, motor, system sweep, flash, web, BLE, motion primitives, turn-PID parser, OLED/MPU parser | PASS |
| PlatformIO builds: production, `ir_filter_diag_96m`, `ir_oled_test_96m`, `oled_mpu_diag_96m`, `flash_map_diag_96m`, `ble_rx_diag_96m`, `system_test_96m`, `turn_pid_diag_96m` | PASS |
| Firmware upload / USART1 hardware capture | NOT RUN — blocked by G0 |

Commands used for the focused acceptance firmware and production build:

```powershell
python tests/regression/ir_sensor_native_test.py
python tests/regression/ir_filter_native_test.py
python tools/test_ir_filter_pipeline.py
python -m platformio run -e genericSTM32F411CE
python -m platformio run -e ir_filter_diag_96m
```

The native/synthetic tests and compile results are `TEST_SOFTWARE` evidence only, not `HARDWARE_MEASUREMENT`.

## 3. Experiment manifest and raw evidence

The planned test matrix and per-channel output table are in [manifest.csv](../../../results/ir_qualification_20261027/manifest.csv) and [metrics_by_channel.csv](../../../results/ir_qualification_20261027/metrics_by_channel.csv). Rows are explicitly marked `BLOCKED_NOT_RUN`; they are not completed captures.

No `raw_uart.log`, `ir_filter_trace.csv`, `ir_on_off.csv`, physical summary, or sensor plot was created: COM6 was not opened and no firmware was uploaded. No existing results were changed. The `results/ir_qualification_20261027/` directory contains only the plan/status manifest, empty measurement slots, and machine-readable verdict. As a result:

- P0 acquisition and ON/OFF behavior: not measured.
- P1 distances 10, 15, 20, 25, 30, 40, 50, 60, and 80 mm: zero of three independent trials per point.
- P2 measured filter comparison, SNR, bias, saturation, and cross-trial repeatability: no physical data; no filter ranking.
- P3: no wall edges captured. 40 Hz logs cannot resolve the proposed 5 ms latency budget; a timestamped MCU event capture is still required.
- P4: no material, lighting, angle, position, or cross-talk matrix measured.
- P5: no monotonic calibration interval or independent holdout; no LUT or distance error metrics.

The requested six plots are intentionally absent rather than generated from synthetic or invented values.

## 4. Filter comparison and engineering decision

There are no measured channel distributions. Consequently mean, STD, MAD, P95/P99, noise reduction, bias, isolated spikes, saturation, and SNR remain blank in the per-channel CSV. SNR requires paired no-wall and wall measurements; neither is available. The filter candidates (RAW, Median-3, EMA 0.10, EMA 0.25, Median-3 + EMA 0.25) remain experimental. The calibration plan's hybrid preference is not treated as an acceptance result.

**Recommendation:** retain current production behavior; do not select or integrate any candidate filter for any channel until P0–P4 produce repeatable physical evidence. The 1 kHz scan-rate statement is an expectation only until computed from consecutive IRH snapshots on hardware.

## 5. Acceptance gates G0–G6

Thresholds below are frozen from the protocol before measurement. “Observed” distinguishes software checks from unavailable physical evidence.

| Gate | Status | Requirement / threshold | Observed | Evidence and retry |
|---|---|---|---|---|
| G0 — SAFETY | **BLOCKED** | Operator confirms motor VM physically disconnected; firmware reports `locked=1` in every IRH record | Operator reported VM connected and powered. Diagnostic firmware was not uploaded; no IRH records exist. | This report. Physically disconnect VM, keep robot secured, then reconfirm before upload. |
| G1 — DATA INTEGRITY | **BLOCKED** | Native/build pass; valid HELLO; ≥95% expected IRF/IRB/IRH; all six channels; scan rate measured; malformed/drop/DMA/timeout/overrun/failure deltas zero | Software/native/build checks PASS. No MCU HELLO, stream counts, scan rate, or hardware error counters were observed. | [Acceptance JSON](../../../results/ir_qualification_20261027/acceptance.json). Retry P0 no-wall and white-matte 20 mm, 30 s each, after G0. |
| G2 — COVERAGE | **BLOCKED** | Three independent 20 s trials at each in-range channel/distance, with distance, angle, surface, light, and fixture uncertainty recorded | 0 physical trials. | [Manifest](../../../results/ir_qualification_20261027/manifest.csv). Run P1 only after P0 integrity passes. |
| G3 — STATIC FILTER | **INCONCLUSIVE** | SNR ≥5 in intended use region; selected filter reduces STD by ≥25% at most tested distances without losing wall variation | No wall/no-wall samples; no SNR or filter statistics. | [Per-channel metrics](../../../results/ir_qualification_20261027/metrics_by_channel.csv). Complete P0–P2; do not choose a filter from plots alone. |
| G4 — DYNAMIC | **INCONCLUSIVE** | ≥10 rising and falling events per important region; latency measured at adequate resolution and within `v_target × latency ≤ 5 mm` | No event capture. Existing IRF is 40 Hz (25 ms sample interval), insufficient to certify a 5 ms budget. | Add/enable a motor-locked MCU timestamp/event capture; do not infer sub-10 ms response from IRF. |
| G5 — ROBUSTNESS | **INCONCLUSIVE** | Quantified false-positive/negative and cross-talk tests across light, material, angle, and ±5 mm position, with no safety-relevant wall misclassification | No physical robustness tests. | Complete P4 and retain source observations for each setup. |
| G6 — DISTANCE LUT | **INCONCLUSIVE** | Per-channel monotonic region; independent holdout MAE ≤2 mm and P95 absolute error ≤5 mm; no extrapolation | No calibration or holdout data. | Complete P1/P2, fit only a justified monotonic interval, then evaluate an independent holdout. |

**Highest supported acceptance level:** none (`NOT_QUALIFIED`). `ACQUISITION_ACCEPTED`, `STATIC_FILTER_ACCEPTED`, `FILTER_DYNAMIC_ACCEPTED`, and `IR_CALIBRATION_ACCEPTED` are all withheld. Wall PID readiness is not established.

## 6. Safe next step

1. Physically remove motor VM and keep the robot secured. Do not rely on the firmware lock alone.
2. Verify the USB-TTL is 3.3 V logic, shares ground, uses PA9/PA10, and COM6 is available.
3. Reconfirm those physical conditions, then upload the already-built motor-locked profile:

   ```powershell
   python -m platformio run -e ir_filter_diag_96m -t upload
   python tools/ir_filter_pipeline.py --port COM6 --seconds 30 --scenario no-wall --surface none --light room --trial 1 --plot
   python tools/ir_filter_pipeline.py --port COM6 --seconds 30 --scenario static --distance-mm 20 --surface white_matte --light room --trial 1 --notes "angle=0deg; fixture uncertainty to be measured" --plot
   ```

4. Stop immediately if HELLO is absent, any `locked` value is not 1, any counter rises, or capture quality is invalid. Preserve each generated run directory and return the physical fixture metadata before proceeding to P1.
