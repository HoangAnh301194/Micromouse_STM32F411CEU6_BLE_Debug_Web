# MPU6050 DATA_RDY / TIM11 1 kHz freshness diagnostics

**State:** opt-in diagnostic only. The board/robot motors must remain powered off
and stationary. `genericSTM32F411CE` (default) is UNCHANGED.
No HSI auto-tuning or PID/motion changes are introduced here.

## Motivation from previous 96 MHz measurement

The 240-second motor-OFF test at `clock_logs/20261011_232050/` found:
- HSI trim25 / PLLN192: estimated physical SYSCLK 95.817 MHz
- TIM11 physical rate about 998.2 Hz
- 0 I2C errors, 0 timeouts, 0 UART TX drops
- `FRESH,status=234999 ready=233906 stale=1093` (~0.465% status=0)

**Important:** Existing firmware reads MPU6050 `INT_STATUS` (DATA_RDY),
then reads two gyro bytes **even when status is 0**. Therefore its
`stale` counter means a gyro transaction associated with a
not-ready status. It does *not* prove the sensor dropped a sample, nor does
a repeated raw gyro value alone prove duplicate acquisition.

The MPU6050's sample clock and STM32 TIM11 are free-running. Until
time correlation is measured, do not change ODR, DLPF, PID `dt`,
or switch to a hardware DATA_RDY interrupt.

## Instrumentation

PlatformIO environment: `imu_freshness_96m_trim25`.

- Based on the tested **96 MHz nominal** profile: `PLLN=192`,
  `HSITRIM=25`, motors OFF.
- Existing TIM11 runs **counter-only** diagnostic mode; the new optional
  1kHz ISR instrumentation measures *age of the latest accepted gyro read*
  and ticks on which the sequence is unchanged. It never launches an I2C
  transfer or executes motor/PID updates.
- MPU main loop records the gap between consecutive **DATA_RDY=1 accepted
  gyro-read completions**, count of gaps exceeding 1.5/2.0 ms, max spacing
  between status checks, and longest consecutive run of status=0.
- `SYNC_SENSOR` and `SYNC_CTRL` lines are printed
  **once per second from main**, replacing the old `FRESH` line
  *only in this opt-in profile*. All counts are cumulative from boot, so
  the host analyzer uses post-calibration differences.
- The host compares MCU TIM5 to Windows perf_counter as in past trials.

### Log interpretation

| Label | Meaning |
| --- | --- |
| `SYNC_SENSOR.notready` | INT_STATUS sampled as 0: sample not flagged ready *at time polled* |
| `SYNC_SENSOR.stale` | gyro read completed following INT_STATUS=0 |
| `SYNC_SENSOR.fgap_max` | maximum MCU microseconds between **accepted read completions** |
| `SYNC_SENSOR.poll_gap_max` | maximum MCU microseconds between completed status polls |
| `SYNC_SENSOR.nr_run_max` | longest uninterrupted series of INT_STATUS=0 |
| `SYNC_CTRL.repeat` | TIM11 cycles with no increment in gyro-fresh sequence since previous TIM11 tick |
| `SYNC_CTRL.age_max` | maximum age of last accepted gyro I2C completion seen at TIM11 entry |
| `SYNC_CTRL.age_gt1500` | TIM11 ticks when last accepted gyro read completed over 1.5ms earlier |

`age_max` is **not** the MPU6050 internal sampling timestamp.
Fresh-sequence age measured with TIM5 lets us identify main-loop / I2C
timing gaps, not directly measure the MPU internal DMP/ADC clock.

## Steps on Windows PowerShell

Update the existing branch while retaining any local work (STOP if `--ff-only`
cannot merge):

```powershell
cd D:\Micromouse_STM32F411CEU6_BLE_Debug_Web
git switch feature/motor-debug-console
git status -sb
git fetch origin
git merge --ff-only origin/feature/motor-debug-console

python tools/imu_freshness_analyze.py --self-test
```

If self-test passes:

```powershell
python -m platformio run -e imu_freshness_96m_trim25
python -m platformio run -e imu_freshness_96m_trim25 --target upload
python tools/clock_serial_test.py --port COM7 --baud 115200 --seconds 240
```

When Python opens COM7, press **STM32 RESET once**. Keep the robot stationary.
Close all other serial monitors. Do not press KEY, drive motors, or manually
change clock during the experiment.

**Restore the original diagnostic/default firmware after capture:**

```powershell
python -m platformio run -e genericSTM32F411CE --target upload
```

Analyze the specific folder printed by `clock_serial_test.py`.
For example:

```powershell
python tools/imu_freshness_analyze.py --folder ".\clock_logs\YYYYMMDD_HHMMSS"
```

It writes `imu_freshness_report.json` beside `raw_serial.log`.
Commit / push the log folder and analysis JSON. Leave all previous logs and
documentation intact.

## After this test

Evaluate three distinct quantities:
1. **DATA_RDY not-ready rate** (polling phase and/or true ODR mismatch).
2. **Fresh-read gap distribution** (main-loop and I2C service latency).
3. **TIM11 sample-reuse rate and age** (what a control ISR actually sees).

These quantities need not be identical. A small stable difference between two
free-running nominal 1kHz oscillators can produce occasional status=0 polls
without an I2C error. Long (>2 ms) fresh-read gaps or control sample ages
are higher-priority issues for motion safety.

For a later hardware experiment, an MPU6050 DATA_RDY pin wired to an STM32
EXTI input could provide sensor-update edge timestamps. **Do not assume
this pin is connected on the existing PCB**; verify wiring first.

Do not make the 96 MHz profile permanent based on this test alone; clock
stability over temperature/voltage remains a separate validation item.
