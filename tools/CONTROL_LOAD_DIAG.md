# Motor-OFF TIM11 Load Benchmark (Experimental)

Profile: `control_load_diag_96m_trim25`. Retains nominal 96MHz trim25, MPU6050 policy B, and 1kHz TIM11.

**Motor safety:** `Motor_Init()` is NOT called in active `main.c`. No TIM2 PWM setup, no motor command, and PID outputs are discarded. Do not connect motor power for this diagnostic. Existing old +100% KEY motor harness remains disabled under `#if 0`. Keep wheels off floor if encoder inputs are handled manually.

## Stages in one recording

Following 5s warmup and 4000 accepted gyro samples for calibration:
- Stage 0, first 60s: IMU freshness + TIM11 interrupt instrumentation
- Stage 1, next 60s: add `Encoder_Update(0.001f)` on every TIM11 interrupt
- Stage 2, remaining capture time: add two `PID_Compute()` calls using measured wheel speed and a zero speed reference. Outputs are **not** applied to motors.

All transitions occur automatically from `main`, with `LOAD_SWITCH` UART records. `LOAD` reports per-stage ticks, observed maximum CPU cycles, cycles exceeding nominal 800us and 1000us budgets, and discarded PID output. The baseline ISR measurements are cumulative; compare stage-local `LOAD` instead.

**Important limitations:** This estimates runtime cost of encoder reading + PID arithmetic with stationary wheels; it does NOT exercise motor drive, encoder edge load during high speed, IR DMA, or real control closed-loop behavior. Motor power must stay disconnected. The observed DWT counter is not certified WCET. Physical clock uncertainty also affects conversion from cycles into microseconds.

## Commands: PowerShell

```powershell
git switch feature/motor-debug-console
git fetch origin
git merge --ff-only origin/feature/motor-debug-console
python -m platformio run -e control_load_diag_96m_trim25
python -m platformio run -e control_load_diag_96m_trim25 -t upload

# Close all COM7 users; press physical RESET once AFTER opening COM7.
python tools/diagnostic_pipeline.py --port COM7 --seconds 240

# Replace with the newly printed pipeline_ timestamp.
python tools/control_load_analyze.py --folder ".\\clock_logs\\pipeline_YYYYMMDD_HHMMSS"

# Restore the default diagnostic firmware.
python -m platformio run -e genericSTM32F411CE -t upload
```

The pipeline's original analyzer still grades the **IMU motor-off diagnostic** across all stages. The additional `load_stage_report.json` grades the staged workload separately. Both need independent review. Do not label the system validated for motion solely from these PASS states.

### Review before flashing

Confirm `Motor_Init` is not called in the active main path, `CONTROL_LOAD_DIAG` is defined only for the opt-in PlatformIO profile, and the console header contains `LOAD_PROFILE,mode=staged,...motor_off=1,pwm_initialized=0`. Confirm physical motor power is disconnected.

### Stop conditions

Stop and inspect if `LOAD_PROFILE` is missing, I2C errors/timeouts or UART drops occur, TIM11 gaps occur, ISR workload exceeds 800us, or the MCU fails gyro calibration. Check `LOAD_SWITCH` and all three stages before interpreting any aggregate.

## 2026-10-12 first staged capture: pipeline_20261012_094736

Review of the uploaded raw UART + reports:

| Stage | Observed ISR max at nominal 96MHz | Per-stage gyro reuse from within-stage SYNC_CTRL deltas |
| --- | ---: | ---: |
| 0, IMU | 2.03 us | 103 / 59,001 = 0.17457% |
| 1, IMU + encoder | 8.24 us | 121 / 59,000 = 0.20508% |
| 2, IMU + encoder + PID | 22.81 us | 219 / 108,000 = 0.20278% |

All three stages had 0 nominal 800us or 1000us ISR budget exceedances, and no TIM11 >1.5ms gap. I2C errors and timeouts were zero.

However, **the full pipeline run is NOT PASS**: 2 UART dropped messages occurred, one shortly after CAL_DONE and one around the Stage 2 boundary. Overall gyro reuse was 448 / 228001 = 0.1965%, compared with 0.014% in the previous lighter-telemetry motor-off run. There was no obvious large Stage 1->2 jump in reuse, so this observation does not establish PID overload.

The old `control_load_analyze.py` incorrectly reported "Stage 0: ticks decreased" because it included pre-calibration LOAD frames and the firmware resets LOAD counters at CAL_DONE. The patched analyzer now skips pre-calibration frames and reports gyro reuse by stage, but **the UART-drop failure remains genuine**. Reprocess the existing capture (no flash required):

```powershell
git fetch origin
git merge --ff-only origin/feature/motor-debug-console
python tools/diagnostic_pipeline.py --folder ".\\clock_logs\\pipeline_20261012_094736"
```

### A/B test of UART burst interference

The candidate issue is the 1Hz batch of UART formatting/queueing in the main loop, *not proven* from the current run. A separate opt-in build moves **only the LOAD telemetry** about 200ms after the other 1Hz diagnostics, keeping IMU B, CPU clock, encoder/PID stages and baudrate unchanged. It does **not** add motor PWM.

```powershell
python -m platformio run -e control_load_staggered_96m_trim25
python -m platformio run -e control_load_staggered_96m_trim25 -t upload
python tools/diagnostic_pipeline.py --port COM7 --seconds 240
```

Disconnect motor power. Press hardware RESET once after opening COM7. Compare the two profiles' per-stage `gyro_reused_pct`, `gyro_age_gt1500_ticks`, and UART `tx_drop_delta`. The A/B is suggestive only with matching stationary conditions and verified boot markers. If the effect persists, investigate 1Hz formatting cost / UART DMA queue rather than attributing it to the PID. No change is justified to MPU ODR or motor controls from this test alone.

## 2026-10-12 second capture: pipeline_20261012_101822

The staged, staggered-UART control-load firmware compiled successfully on the user's STM32 PlatformIO environment. The uploaded log measured:
- 0 UART queue dropped messages, 0 CLK sequence gaps and no I2C errors.
- Estimated clock mismatch -0.030876%.
- Overall gyro reuse 136 / 228001 = 0.0596% (previous unstaggered staged run: 448 / 228001 = 0.1965%).
- Stage 0: 68 / 59000 = 0.11525%; observed ISR maximum ~1.97us.
- Stage 1: 55 / 59001 = 0.09322%; ISR ~8.27us.
- Stage 2: 10 / 108000 = 0.00926%; ISR ~22.87us.
- Zero observed budget overflows at 800us and 1000us in all stages.

**This capture has not passed full diagnostic attestation.** It includes an on-wire **malformed startup UART record** combining two formerly separate lines:
```text
SYNC_POLICY,policy=B,skip_on_nr=1,r�I2C_PROBE,400_a=0 ...
```
The raw line is incomplete, so neither a clean `SYNC_POLICY.motor_off=1` nor a standalone `I2C_PROBE` can be read. The MCU `uart_tx_drop=0` counter measures firmware queue admission only, not every possible serial-line byte corruption. Treat these as *missing evidence*, not proof that the sensor or motor configuration failed. Do not relax the existing analyzer to infer missing fields.

A firmware update now replays `SYNC_POLICY` at +10s and +40s post-calibration and `I2C_PROBE` at +13s and +43s, outside the initial startup UART burst. The analyzer validates all original criteria and accepts fully observed, separately transmitted replay records, including the actual selected I2C speed and WHO_AM_I value. The pipeline now reports suspect/malformed UART lines explicitly.

Inspect the already recorded capture without flashing (it will still FAIL startup attestation, as expected):
```powershell
git fetch origin
git merge --ff-only origin/feature/motor-debug-console
python tools/imu_freshness_analyze.py --self-test
python tools/diagnostic_pipeline.py --folder ".\\clock_logs\\pipeline_20261012_101822"
```

To verify the corrected UART diagnostic firmware with a NEW hardware capture, disconnect motor power, close other serial monitors, build/upload **only after build SUCCESS**, and press RESET after the port opens:
```powershell
python -m platformio run -e control_load_staggered_96m_trim25
python -m platformio run -e control_load_staggered_96m_trim25 -t upload
python tools/diagnostic_pipeline.py --port COM6 --seconds 240
```

Do not infer that PID was responsible for the prior freshness losses: the MCU ISR stayed well below the period, and reused-gyro rates changed substantially with elapsed experiment phase. A repeated clean boot, valid replay, zero UART corruption and a stable stage breakdown must precede actuator/PWM authorization. The serial framing root cause is still unresolved.
