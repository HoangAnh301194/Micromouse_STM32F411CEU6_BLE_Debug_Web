# Gyro freshness — paired IRQ-completion vs publication experiment

**Branch:** `feature/mpu-freshness-phase-ab`
**Baseline preserved:** `release/mpu6050-i2c-rc1-20261012`
**Integration reference:** `feature/mpu-i2c-integration-rc1`
**Scope:** observation ONLY; motors disconnected, no MPU policy changes, no DMA.

## Question

Why did the isolated integration run
`mpu_integration_logs/session_20261012_235159`
report 449 reused gyro samples / 224000 TIM11 ticks (0.20045%),
vs the RC1 staged diagnostic capture at 0.0441% overall and
0.10508% for stage 0?

Two distinguishable mechanisms:

- The MPU request schedule drifts relative to the 1ms TIM11 clock, or
  I2C completion sometimes arrives after the control tick.
- The I2C transfer has already completed but main has not yet processed
  and **published** the gyro/yaw snapshot when TIM11 runs.

Do not attribute the old discrepancy to either explanation before the
new paired measurements. Original captures use different UART traffic,
ISR work and sensor publication markers.

## Experimental design — A/B within the SAME TIM11 tick

Rather than running different firmware workloads, the opt-in build captures
both markers on every 1kHz TIM11 tick:

- **Arm A — IRQ-complete:** the I2C1 event IRQ has completed a successful
  2-byte Gyro Z transfer. An IRQ-owned, single-attempt seqlock publishes
  the completion timestamp and sequence without I2C or UART in TIM11.
- **Arm B — published:** main has completed Gyro Z filtering/yaw integration
  and atomically published a coherent `MPU6050_Snapshot_t`.
  Each snapshot also identifies the exact source I2C IRQ sequence.

The same sampling policy, same TIM11 ticks, clock, UART timing, process
and calibration serve both arms. The trace changes **only** instrumentation;
no intentional acquisition/actuation behavior is changed. IRQ trace
contention is counted rather than blocking/spinning inside priority-0 TIM11.

Four timestamps (TIM5 microseconds) are recorded for a completed sample:

1. `t_request`: actual I2C register-status transaction setup time;
2. `t_complete`: Gyro Z register read completion in I2C1 IRQ;
3. `t_main`: main observes the successful completion returned by Poll();
4. `t_publish`: gyro/yaw snapshot just before atomic publication commit.

`t_request → t_complete` includes status query, any Policy B retry and
subsequent 2-byte Gyro Z transfer. They do **not** measure the sensor's
internal gyroscope acquisition time. Numeric delta statistics:
request interval; request→IRQ; IRQ→main; main→publish; IRQ→publish.
The report also records an ordered timestamp quartet for one recent
completed sample at each 5s phase report.

## Firmware flags and interfaces

Opt-in PlatformIO environment: `mpu_phase_ab_96m_trim25`.

Source changes are confined to gyro I2C tracing, MPU snapshot publication,
TIM11 diagnostic counters, and host scripts. All additional firmware
instrumentation uses `#if defined(MPU_PHASE_AB)`. The existing default,
legacy RC1 and integration profiles do not set that flag.

- I2C bus: IRQ-driven 400kHz, address 0x68; DATA_RDY+Policy B.
- SYSCLK: nominal 96MHz, HSI trim25.
- Calibration: 5s warmup + 4000 stable samples.
- TIM11: 1kHz, motor OFF, no blocking I2C, no UART, no DMA.
- Main logs existing `MINT` at 1Hz, plus `PHASE_REQ`,
  `PHASE_LAT` and `PHASE_PAIR` every 5s after calibration.
- `PHASE_PAIR.irq_reuse`: reused IRQ-complete sequence.
- `PHASE_PAIR.pub_reuse`: reused published-snapshot sequence.
- `PHASE_PAIR.irq_ahead`: completed Gyro Z IRQ for a newer sample
  than the source IRQ recorded in the published snapshot.
- `PHASE_PAIR.contention`: the high-priority TIM11 tried to read
  the trace while I2C IRQ's seqlock was mid-publication; skipped,
  not spun upon.

If `pub_reuse > irq_reuse`, some TIM11 ticks saw a completed I2C sample
that was not yet published. Analyze IRQ→main and main→publish separately.
If `pub_reuse ≈ irq_reuse`, pay attention to request gap/phase,
data-ready polling and I2C completion timing. A single run suggests
a mechanism; repeated controlled runs establish confidence.

The phase analyzer reports `PASS_PAIRED_TRACE_FOR_DIAGNOSIS` only if the
**evidence is complete**, not that gyro freshness regression is resolved.
It also embeds the old integration regression gate separately, which may
remain `REVIEW_FRESHNESS_REGRESSION`.

## Instructions on Windows (motors OFF)

```powershell
git fetch origin
git switch feature/mpu-freshness-phase-ab

python tools/mpu_phase_ab_analyze.py --self-test
python tools/mpu_integration_regression.py --self-test
python tools/imu_freshness_analyze.py --self-test
python tools/mpu_integration_regression.py --source-check

python -m platformio run -e control_load_staggered_96m_trim25
python -m platformio run -e mpu_integration_policy_b_96m_trim25
python -m platformio run -e mpu_phase_ab_96m_trim25
```

**Do not flash until compilation succeeds. Disconnect VM/motor supply;
only STM32 logic and MPU should be powered**. If all checks/builds pass:

```powershell
python -m platformio run -e mpu_phase_ab_96m_trim25 -t upload
python tools/mpu_phase_ab_analyze.py --port COM6 --seconds 240
```

After COM6 opens, press STM32 RESET **once**. Leave the robot stationary
for the 5s warmup, calibration and entire capture. The host creates
`mpu_phase_ab_logs/session_YYYYMMDD_HHMMSS/raw_serial.log` and
`phase_ab_report.json`.

To re-analyze an existing run without the board:

```powershell
python tools/mpu_phase_ab_analyze.py --folder "mpu_phase_ab_logs/session_YYYYMMDD_HHMMSS"
```

Preserve raw logs; upload the new session folder if further code review
is needed. The branch's GitHub Actions workflow also attempts builds
for legacy RC1, integration and phase A/B, and the phase analyzer self-test.

## Acceptance and interpretation limits

A usable paired trace must contain **one** motor-OFF/Policy-B boot profile,
400kHz I2C WHO_AM_I=0x68, calibration=4000 samples/8 blocks, zero I2C
errors/timeouts and missed TIM11 scheduling, at least 150 postcal valid
1Hz MINT frames, at least 30 complete 5-second phase groups and a
monotonic/continuous trace. A bad trace is not a negative algorithm result.

This experiment does **not** itself constitute a final MPU/I2C release.
It tests only stationary, motor-disconnected sampling under added trace
instrumentation. IRQ tracing adds a small amount of processing to the
I2C completion and TIM11 ISR, so absolute reuse rates should not be
compared blindly with previous uninstrumented captures. Both A and B
nevertheless experience the same added overhead in this new run.

Before releasing integration, compare paired channels, investigate any
causal gap, perform a confirmed build and motor-OFF repeat, and then
separately validate under real motor electrical loads.
