# RC2 MPU6050/I2C — integration & acceptance checklist

**Qualified baseline:** `release/mpu6050-i2c-rc2-tim11-20261013`.
**Validated source SHA:** `5219e04d3fe7b8fdb5a092100154e6d9f0317e09`.
**Current gate:** stationary motor-OFF, reverse-order A/B success.
**Do NOT use this checklist to claim production/motor-on acceptance.**

## A. Released / frozen at RC2

- [x] Independent `MPU_SAMPLING_POLICY_B` (no dependency on
      `MPU_FRESHNESS_DIAG` to skip stale DATA_RDY=0 gyro reads).
- [x] I2C1 event/error IRQ 400kHz; no MPU RX DMA in the validated path.
- [x] Policy B +220us TIM5 retry; bounded acquisition state transitions;
      no I2C in TIM11.
- [x] TIM11 1kHz **latest-tick-opportunity** scheduling on opt-in
      `MPU_TIM11_TICK_SCHED`; no queued catch-up reads.
- [x] Coherent, completed, calibrated gyro/yaw snapshot in normal
      consumer, with valid/age/sequence fields.
- [x] Separate RC1 snapshot maintained for comparisons.
- [x] Two motor-OFF 240s controlled A/B pairs, opposite order, each
      with real raw serial evidence and host phase/reuse analysis.
- [x] Both B scheduler traces passed motor-OFF integration checks;
      no I2C errors/timeouts, TIM11 scheduling faults or UART drops.
- [x] GitHub Actions run `38290813533` succeeded at pinned source SHA:
      host tests and ARM firmware builds for legacy + experimental
      profiles.
- [x] `docs/mpu_i2c_rc2/MANIFEST.json` pins source files, CI and
      raw evidence by Git blob SHA.

## B. API contract / integration sequence

1. Use a **new integration branch** from pinned RC2 source. Do not
   develop in the RC2 release branch.
2. Choose the proper clock/init order:
   `Board_Init(); SystemTimer_Init(); UART_Init(); MPU6050_Init();`
   then calibrate with the robot stationary. Never run motion if
   init/calibration fails.
3. In a new firmware build, explicitly set:
   `MPU_SAMPLING_POLICY_B=1`,
   `MPU_TIM11_TICK_SCHED=1`, nominal 96MHz/trim25 only if this
   board's timing measurements still justify it. **Do not** assume
   `genericSTM32F411CE` already defines RC2.
4. TIM11 ISR: call `MPU6050_NotifyControlTick(handle, entry_us)`
   once per real control tick. Keep ISR free of I2C, UART and
   blocking sensor operations.
5. Main loop calls `MPU6050_Update()` frequently; it owns the
   async status→gyro transactions, optional status retry, filter,
   calibration state and snapshot publication. Non-blocking I2C IRQs
   handle transfer events.
6. Motion consumer reads `MPU6050_GetSnapshot()`; check `valid`,
   `completed_us` and age (candidate guard >3000us), and do not
   silently use stale or uncalibrated gyro/yaw.
7. `Control_SetDiagnosticOnly(1)` means **no motor activity** and
   is used in the validated bench. Removing it changes the tested
   workload and requires explicit new evidence.
8. Hardware reports `DATA_RDY` freshness at MCU polling time.
   There is no hardware timestamp for the MPU's internal conversion.
   A gyro sample reused across TIM11 ticks is expected occasionally:
   B paired reuse was 0.09911% / 0.10000% in two motor-OFF runs.

## C. Not yet validated — must be completed before motor-on release

- [ ] Build an explicitly named uninstrumented motor-enabled
      configuration; verify both RC2 policy and tick scheduler
      flags survive when `MPU_PHASE_AB` /
      `MPU_INTEGRATION_BENCH` are removed.
- [ ] Check startup, calibration failure and recovery without PWM
      enabled: missing MPU/WHO_AM_I, bus-stuck, I2C NACK and timeout.
- [ ] Verify invalid/expired gyro snapshot forces a physically safe
      motor output; review `Motion_Stop()` and driver integration.
- [ ] With motors operating at representative electrical loads,
      verify data freshness, I2C error/timeouts, IRQ deadlines and
      UART drops; establish an EMI/stability baseline. This is a
      separate stage from Motor Driver software optimization.
- [ ] Verify gyro bias after warming, dynamics/yaw drift and motion
      control at actual Micromouse speeds. Gyro Z only is not
      6-axis attitude estimation.
- [ ] Run a realistic combined sensor/encoder/IR/control workload to
      bound observed ISR scheduling and CPU margin, rather than
      extrapolating from the isolated bench.
- [ ] Confirm clock and timing on each board; HSI trim25 is
      board/measurement dependent; nominal 96MHz is not precise
      hardware metrology.
- [ ] Define final fault counters, telemetry exposure and
      start/stop/recalibration behavior for the real application.
- [ ] Add a final real-device acceptance record, then create a **new**
      revision if code changes; never rewrite pinned RC2 source.

## D. Handoff to next module: Motor Driver

RC2 is a sensor/acquisition scheduler reference. The **Motor Driver**
scope should remain separate:

- Electrical actuation abstraction: PWM frequency/duty,
  direction, TB6612FNG control and enable/disable semantics,
  saturation/command clipping, deterministic stop and fail-safe
  outputs, timer and interrupt interactions.
- Motor tests should measure PWM timer behavior, duty update jitter,
  reversal and safe-stop transitions **without** introducing
  motion planning, velocity PID, trajectory control or odometry
  feedback into the driver acceptance.
- A future **system-integration** test should run Motor Driver
  alongside RC2 to evaluate I2C/IMU stability under motor load.
  This test is not evidence that either separate module is already
  production qualified.

## E. Regression commands (Windows)

```powershell
git fetch origin
git switch release/mpu6050-i2c-rc2-tim11-20261013
python tools/mpu_phase_ab_analyze.py --self-test
python tools/mpu_tick_scheduler_ab_compare.py --self-test
python tools/mpu_integration_regression.py --self-test
python tools/mpu_integration_regression.py --source-check

python -m platformio run -e control_load_staggered_96m_trim25
python -m platformio run -e mpu_integration_policy_b_96m_trim25
python -m platformio run -e mpu_phase_ab_96m_trim25
python -m platformio run -e mpu_phase_tick_96m_trim25
```

Compare pinned raw A/B runs directly, independently of saved JSON:

```powershell
python tools/mpu_tick_scheduler_ab_compare.py `
  --legacy "mpu_phase_ab_logs/session_20261013_005503" `
  --tick "mpu_phase_ab_logs/session_20261013_010213"

python tools/mpu_tick_scheduler_ab_compare.py `
  --legacy "mpu_phase_ab_logs/session_20261013_011839" `
  --tick "mpu_phase_ab_logs/session_20261013_011420"
```

Expected comparison result for both pairs:
`IMPROVED_REQUIRES_REPLICATION` (the analyzer is intentionally
conservative; this RC2 acceptance separately documents that the
reverse-order replication has already occurred).

**Freeze distinction:** the source commit and blob manifest are pinned,
but a branch is not technically immutable unless GitHub rules prevent
writes/force-pushes. Treat direct edits as policy violations and use
a new revision for any firmware change.
