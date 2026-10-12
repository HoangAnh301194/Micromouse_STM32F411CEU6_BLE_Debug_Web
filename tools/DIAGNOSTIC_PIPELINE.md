# One-session diagnostic pipeline (motor OFF)

Added in `feature/motor-debug-console` after the A/B MPU6050 freshness experiment.

## Purpose

Run ONE USART1/COM7 capture and extract, sequentially:
1. Boot integrity, firmware identity and motor-off evidence.
2. MCU/host clock regression and missing UART sequences.
3. MPU6050 calibration, I2C errors and data freshness.
4. TIM11 period/gaps and gyro-age trends.
5. Approximately 30-frame windows of repeated gyro samples.

This is a **measurement tool**, not a motor controller. It does not flash the MCU, send motor commands or enable outputs. It does not prove motor+encoder+PID timing.

## Preparation

Use the existing motor-OFF policy B firmware (or policy A for control experiment). Connect USART1 to the USB-TTL adapter (currently COM7), and close all serial monitors. The robot must remain stationary; motors must not be powered for this test.

```powershell
git switch feature/motor-debug-console
git fetch origin
git merge --ff-only origin/feature/motor-debug-console
python -m pip install pyserial matplotlib
python tools/imu_freshness_analyze.py --self-test
python tools/clock_serial_test.py --self-test

# Flash selected B experiment BEFORE starting the pipeline:
python -m platformio run -e imu_freshness_ab_b_96m_trim25
python -m platformio run -e imu_freshness_ab_b_96m_trim25 -t upload

# Start capture; press physical RESET ONCE after COM7 opens.
python tools/diagnostic_pipeline.py --port COM7 --seconds 240
```

Outputs: `clock_logs/pipeline_<timestamp>/raw_serial.log`, `clock_samples.csv`, `clock_summary.txt`, `imu_freshness_report.json`, `pipeline_report.json` and clock plots. The script invokes the existing analyzers, preserving their validation rules. A missing boot marker or calibration never silently becomes PASS.

Offline pipeline on a previously recorded run:

```powershell
python tools/diagnostic_pipeline.py --folder clock_logs/20261012_084346
```

To restore the default motor-OFF diagnostic image after the experiment:

```powershell
python -m platformio run -e genericSTM32F411CE -t upload
```

**Caution:** The "approximately 30-frame" trend windows are only 30 seconds if `SYNC_CTRL` is emitted once per second. Do not treat those rows as precision timestamps. Jitter in the host-received serial timing is *not* TIM11 ISR jitter. The current firmware's DWT-based ISR duration is an observed maximum, not WCET. Compare A/B only between profiles with matching clock/temperature/setup. A clean motor-OFF PASS does not authorize switching on motor power.

## Next implementation gate

Before enabling motor tests, add an explicitly opt-in firmware profile with boot-default motors disabled, a physical operator-trigger/arming requirement, overcurrent or battery guards where supported, independent timeout/STOP, and an evidence-backed encoder/PWM direction test with the wheels off the floor. Only then run progressively heavier TIM11 loads (encoder only, encoder+PID calculation, finally motor driver), logging phase-specific ISR duration and MPU freshness. Do not re-enable the old KEY +100% motor routine unchanged.
