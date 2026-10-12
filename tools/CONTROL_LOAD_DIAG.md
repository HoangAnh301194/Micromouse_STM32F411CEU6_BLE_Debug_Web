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
