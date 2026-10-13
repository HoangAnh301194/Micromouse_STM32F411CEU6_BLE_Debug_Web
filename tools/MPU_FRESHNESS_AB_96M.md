# MPU6050 DATA_RDY A/B experiment (motor-OFF)

## Aim

The previous 240s test at clock_logs/20261011_235107 found about 0.50% of
TIM11 cycles had no newly accepted gyro sample. This experiment tests
whether skipping gyro I2C reads on DATA_RDY=0 and retrying the status once
improves that observation. It does not modify the production clock, PID,
filter settings, ODR or motor code.

## Exact A/B policy

| Setting | A (control) | B (experimental) |
| --- | --- | --- |
| PlatformIO | imu_freshness_96m_trim25 | imu_freshness_ab_b_96m_trim25 |
| Clock | PLLN192, HSITRIM25, 96MHz nominal | identical |
| TIM11 | 1kHz diagnostic-only | identical |
| Motor/encoder/PID | disabled | disabled |
| DLPF / ODR | DLPF3 / 1kHz | identical |
| DATA_RDY=0 | still reads gyro; marks stale | skips gyro read |
| Retry | none | at most one 220us-delayed status read |
| Deadline | normal 1ms schedule | abandon late retry at 850us of slot |
| CPU idle | WFI | WFI plus one-shot TIM5 CC1 interrupt |

B only adds a TIM5 compare interrupt which disables its own one-shot
interrupt and wakes main. No I2C or sensor formatting runs inside ISR.
TIM5's interrupt priority is 14, lower than TIM11 priority 0.

The DATA_RDY=0 counts of B include *additional status rechecks* and
must not be compared directly with A. Use initial_nr (first-poll
not-ready), TIM11 repeat percentage, gyro completion ages, and total
estimated I2C transactions per 1000 control ticks.

## Counters and interpretation

- SYNC_POLICY: ensures firmware is definitely A or B.
- SYNC_AB: initial_nr, gyro_started, gyro_skipped, retry_started,
  retry_ready, retry_nr, retry_expired, retry_busy.
- SYNC_SENSOR: status ready/notready, gyro reads accepted as fresh,
  stale reads, and accepted-sample completion gaps.
- SYNC_CTRL: TIM11 sample reuse, max age, age>1000us, age>1500us,
  approximate age p95/p99 histogram thresholds.
- CLK, IMU, TIMING, CAL_DONE: clock and diagnostic integrity.

Age refers to I2C read completion, not MPU's actual sensor sample time.
The p95/p99 histogram uses 250us bins, and 1750us denotes the
unbounded top bin (age >=1750us), NOT an exact percentile.

## PowerShell runbook

Keep motors UNPOWERED and robot completely stationary. Close every
other serial monitor. Never use reset --hard to update this branch.

1. Update and run offline checks:

    git switch feature/motor-debug-console
    git status -sb
    git fetch origin
    git merge --ff-only origin/feature/motor-debug-console
    python tools/imu_freshness_analyze.py --self-test
    python tools/imu_freshness_ab_compare.py --self-test

Stop if these fail or if git cannot fast-forward.

2. Profile A, 240s:

    python -m platformio run -e imu_freshness_96m_trim25
    python -m platformio run -e imu_freshness_96m_trim25 --target upload
    python tools/clock_serial_test.py --port COM7 --baud 115200 --seconds 240

Once Python has opened COM7, press hardware RESET once. Note the new
clock_logs session directory, called A_DIR in examples. Then restore
the default firmware before analysis:

    python -m platformio run -e genericSTM32F411CE --target upload
    python tools/imu_freshness_analyze.py --folder ".\clock_logs\A_DIR"

3. Profile B, 240s:

    python -m platformio run -e imu_freshness_ab_b_96m_trim25
    python -m platformio run -e imu_freshness_ab_b_96m_trim25 --target upload
    python tools/clock_serial_test.py --port COM7 --baud 115200 --seconds 240

Again press RESET after Python opens COM7, remain stationary, record
the new folder B_DIR, restore default before analysis:

    python -m platformio run -e genericSTM32F411CE --target upload
    python tools/imu_freshness_analyze.py --folder ".\clock_logs\B_DIR"

4. Compare (replace A_DIR and B_DIR with the actual session names):

    python tools/imu_freshness_ab_compare.py --a ".\clock_logs\A_DIR\imu_freshness_report.json" --b ".\clock_logs\B_DIR\imu_freshness_report.json"

5. Push BOTH A and B capture folders, with raw_serial.log,
clock_summary.txt and imu_freshness_report.json.

## Acceptance and limitations

Both reports need PASS_DIAGNOSTIC, matching 96MHz trim25 motor-off
signatures, no missing CLK, TX drops, I2C errors or TIM11 gaps.
B must stay inside its single-retry guard. The PRIMARY outcomes are
TIM11 repeated gyro ticks and ticks with gyro age above 1.5ms; other
metrics include fresh completion gaps and total I2C transfer rate.

If B appears slightly better, repeat A/B/A in similar temperatures
and supply voltage to separate policy effect from oscillator drift.
A failure or no benefit means we keep A while considering further
alternatives. The experiment does not authorize production deployment.


## Recovery: 2026-10-12 submitted A/B logs (no new hardware capture needed for preliminary comparison)

Existing runs:
- A: `clock_logs/20261012_082345/` contains full firmware boot headers,
  but hardware RESET interrupted a UART line: the `[BOOT]` marker occurs
  inside `SYNC_CTRL,ticks=0,re[BOOT]...`. New analyzer extracts the marker
  suffix and uses only the post-RESET session.
- B: `clock_logs/20261012_082926/` begins at `CLK seq=23`, roughly
  23 seconds after boot, so it lacks `[BOOT]`, `CLOCK_PROFILE`,
  `SYNC_POLICY` and `CAL_DONE` lines. Its full per-second `CLK`,
  `IMU`, `SYNC_SENSOR`, `SYNC_CTRL`, `SYNC_AB`, and `TIMING`
  telemetry is still usable to estimate steady-state performance.
  Since the startup was NOT captured, B must remain
  `PARTIAL_DIAGNOSTIC` (NOT a validated full PASS).

Refresh from the feature branch, then rerun analysis **offline** on the
existing folders. It does not require another flash/reset:

```powershell
git switch feature/motor-debug-console
git status -sb
git fetch origin
git merge --ff-only origin/feature/motor-debug-console

python tools/imu_freshness_analyze.py --self-test
python tools/imu_freshness_ab_compare.py --self-test

python tools/imu_freshness_analyze.py --folder ".\clock_logs\20261012_082345"
python tools/imu_freshness_analyze.py --folder ".\clock_logs\20261012_082926"

python tools/imu_freshness_ab_compare.py `
  --a ".\clock_logs\20261012_082345\imu_freshness_report.json" `
  --b ".\clock_logs\20261012_082926\imu_freshness_report.json" `
  --allow-partial
```

Expected analyzer status: A `PASS_DIAGNOSTIC`, B `PARTIAL_DIAGNOSTIC`.
The `--allow-partial` comparator option explicitly produces a
**PROVISIONAL comparison**, never an authorization to enable motors or
adopt sampling policy B in normal firmware.

Offline numbers computed directly from the raw cumulative counters:

| Measurement | A | B |
| --- | ---: | ---: |
| Compared control cycles | 227000 | 238001 |
| TIM11 repeated-sample cycles | 1243 (0.5476%) | 10 (0.0042%) |
| TIM11 age>1.5ms cycles | 1236 | 9 |
| Gyro completion gaps >2ms | 3 | 2 |
| First-poll not-ready per 1000 TIM11 ticks | 2.718 | 10.756 |
| Estimated I2C transfers per 1000 TIM11 ticks | 1994.52 | 2010.69 |
| B retries started / ready (within captured interval) | 0 / 0 | 2559 / 2559 |

The improvement in accepted gyro freshness is substantial, although
B requires ~0.81% more estimated I2C transactions in this comparison.
A and B do NOT share the same exact start time, and B lacks boot and
calibration metadata; review this as experimental evidence only. If B
is later considered for permanent integration, repeat it with a clean
boot capture and preferably an A/B/A comparison in consistent thermal
conditions.
