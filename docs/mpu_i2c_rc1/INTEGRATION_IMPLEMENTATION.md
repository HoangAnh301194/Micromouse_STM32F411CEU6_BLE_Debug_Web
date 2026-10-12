# MPU6050/I2C1 production-policy integration candidate

**Branch:** `feature/mpu-i2c-integration-rc1`
**Frozen baseline:** `release/mpu6050-i2c-rc1-20261012`
**Scope:** software integration + motor-OFF regression candidate. **Not production-accepted or motor tested.**

## What changed

1. `include/mpu_policy.h` introduces the production flag
   `MPU_SAMPLING_POLICY_B`, independently from `MPU_FRESHNESS_DIAG`.
   The legacy benchmark flag `MPU_FRESHNESS_AB_B` transparently aliases
   the new flag so the frozen comparator still runs the same skip/retry path.
2. Production sample policy: on an INT_STATUS not-ready result, skip Gyro Z,
   allow a single TIM5 compare wake +220us if the initial status returns
   by 600us within the requested 1ms slot; abandon if elapsed reaches
   850us. Do not wait inside TIM11.
3. The retry state (`retry_count`, `retry_due_us`) is now in the
   MPU handle whenever production Policy B is enabled, **not** inside the
   diagnostic-only counter structure. TIM5 compare API uses the same
   production flag; A/default builds do not instantiate the retry IRQ.
4. `MPU6050_GetSnapshot` returns a coherent **completed, calibrated**
   snapshot: `yaw_deg, gyro_z_dps, completed_us, sequence, valid`.
   Publication is protected by a brief IRQ-masked critical section
   in main; TIM11 reads without I2C. Timestamp is **read completion**
   on TIM5, not the MPU's internal acquisition time.
5. In the ordinary motion consumer, invalid or over-3ms-old snapshots
   cause `Motion_Stop()` instead of passing uninitialized/old yaw into
   `Motion_Update()`. Actual motion integration and the stop behavior
   under real load remain untested.
6. Added a new **motor-OFF and diagnostic-OFF** application to prove the
   sampling policy is not conditionally removed without debug flags.
   This application tests a 1kHz TIM11 ISR snapshot consumer and reports
   reuse/age/timing via `MINT` records.
7. Added an offline serial regression analyzer and CI build workflow.

**Not changed:** RC1 numerical gyro parameters (+/-2000 dps, DLPF=3),
calibration limits, target 1kHz, 400kHz I2C, motor control logic or
default build profile. No DMA was added.

## Build/validate on Windows, motors disconnected

```powershell
git fetch origin
git switch feature/mpu-i2c-integration-rc1
python tools/mpu_integration_regression.py --source-check
python tools/mpu_integration_regression.py --self-test
python tools/imu_freshness_analyze.py --self-test
python -m platformio run -e control_load_staggered_96m_trim25
python -m platformio run -e mpu_integration_policy_b_96m_trim25
```

Builds from the integration branch are **not** yet confirmed in the
authoring environment (no PlatformIO ARM toolchain available there).
The GitHub Actions workflow runs the same builds and self-tests;
check its run status before calling compile/regression gates complete.

Only after successful build, if a motor-OFF run is desired, flash
the *integration* profile with motor power **disconnected**:

```powershell
python -m platformio run -e mpu_integration_policy_b_96m_trim25 -t upload
python tools/mpu_integration_regression.py --port COM6 --seconds 240
```

After COM6 opens, press STM32 RESET once. The runner saves
`mpu_integration_logs/session_.../raw_serial.log` and
`mpu_integration_report.json`. The two output statuses are:
`PASS_INTEGRATION_MOTOR_OFF` (qualified stationary gate), or `FAIL`.
Do NOT feed this run into `diagnostic_pipeline.py` because the new
integration image purposely does not emit `CLK` or `SYNC_*` records.

## Acceptance gates

- Startup signature: Policy B, no diagnostic flags, 96MHz nominal,
  trim25; correct I2C 400kHz, address/WHO_AM_I 0x68.
- 5s gyro warmup + 4000 stationary samples, 8 blocks, zero retries.
- At least 150 consecutive post-calibration MINT records with
  `cal=1,valid=1` and monotonic control/sample counters.
- Zero I2C error, timeout, stale gyro reads, UART TX drop, timing late
  or TIM11 gap in the post-calibration window.
- Record gyro reuse, age>1.5ms, max sample age, missing or expired>3ms
  consumer snapshots. Compare quantitatively to frozen RC1:
  100/227001=0.0441% reuse, 99 aged>1.5ms ticks. These are reference
  observations, **not absolute pass/fail thresholds**.
- Test the sensor under actual motor/electrical load in a separate
  integration phase before declaring production-ready.
- Fault injection (I2C NACK/bus-stuck/timeout), multi-calibration repeat,
  moving-yaw drift and the production Motion_Stop guard are still pending.

## Architecture and integration contract

```text
Board_Init / SystemTimer_Init
    |
    +--> MPU6050_Init() -- I2C1 400kHz, WHO_AM_I validation
    +--> MPU6050_Calibrate() -- nonblocking, stationarity check
    |
main loop (frequently)                      TIM11 ISR (1 kHz)
    |                                       |
MPU6050_Update()                      MPU6050_GetSnapshot()
    |                                       |
I2C EV/ER IRQs (status + 2 gyro bytes)   valid, age <= 3ms?
    |                                       |
DATA_RDY=0 -> policy B retry              yes -> Motion_Update()
    |                                     no  -> Motion_Stop()
completed Gyro Z / yaw ------------------+
    |
atomic snapshot publish
```

Do not call blocking I2C init/recovery or print UART within TIM11.
Do not call the new image a live navigation controller: the integration
bench intentionally sets `Control_SetDiagnosticOnly(1)` to keep actuators
off while exercising the real Policy B driver and TIM11 snapshot reader.
The default `genericSTM32F411CE` is untouched and does not
automatically inherit production Policy B.

## Evidence / traceability

Baseline documentation is in
`docs/mpu_i2c_rc1/README.md` and `docs/mpu_i2c_rc1/evidence/`.
Baseline source blob identities are pinned in
`docs/mpu_i2c_rc1/MANIFEST.json`.
The new branch's `.github/workflows/mpu-i2c-integration.yml` executes
host checks and two independent PlatformIO builds.

No new physical capture from the integrated image has yet been supplied.
