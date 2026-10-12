# MPU6050/I2C RC1 → Micromouse application: integration gate

This document distinguishes the **accepted motor-OFF diagnostic** from a
**production candidate**. Avoid silently treating diagnostic flags as a final
controller interface. Do not merge RC1 into normal firmware as "fully tuned"
without completing the engineering gates.

## Stage A — Freeze and audit (completed as diagnostic RC1)

- [x] Isolate verified source at source commit `7e71d611bfedcc173f7d2798ba51fd6db49b59d9`.
- [x] Record clock / I2C / sampling / calibration parameters.
- [x] Archive PASS diagnostic + load reports and clock summary.
- [x] Preserve reproducible PlatformIO diagnostic build environment.
- [x] Keep the subsequent, unvalidated motor PWM bench OUT of this release.
- [ ] Rebuild frozen branch on the user's PlatformIO installation; source was
      already measured on hardware, but the release packaging itself has not
      been recompiled in this conversation.

## Stage B — Separate policy from diagnostics (REQUIRED, NOT DONE)

1. Expose explicit production configuration such as
   `MPU_SAMPLING_POLICY_B` that selects the same verified skip/one-retry
   algorithm **without** requiring `MPU_FRESHNESS_DIAG` telemetry.
   Do NOT rename only the macro: move production timing and retry state
   out of the debug-only `#if MPU_FRESHNESS_DIAG` fields.
2. Preserve current nonblocking `MPU6050_Update` in main. Continue using
   I2C1 event/error IRQs for async register reads. Keep blocking register
   probes and GPIO SCL bus-recovery routines outside TIM11.
3. Retain TIM5 +220us one-shot wake and slot-expiration rules, with
   no timing work, waits, float formatting or UART in any ISR.
4. Introduce a stable sensor snapshot API with clear validity:
   calibrated, last update timestamp, fresh sequence/counter,
   gyro Z value/yaw and I2C fault state. Establish atomic/coherent
   handoff to the TIM11 consumer.
5. Change the application initialization and scheduling explicitly:
   `MPU6050_Init` → `MPU6050_Calibrate`; main calls
   `MPU6050_Update` frequently; TIM11 consumes the last complete
   snapshot. Only allow actual motion after sensor ready and calibration
   passes. Calibration failure must not start the motors.
6. Confirm that I2C init/recovery does not destructively reinitialize
   GPIO shared with encoder, IR, BLE, PWM or other onboard devices.
7. Keep a separate `mpu_i2c_integrated` opt-in build config. The
   `genericSTM32F411CE` default remains untouched until verified.

## Stage C — Software and fault-injection validation (NOT DONE)

- [ ] New integrated build compiles and passes static/compiler tests.
- [ ] Policy B behavior identical to RC1 on recorded deterministic traces.
- [ ] I2C NACK startup, bus stuck SDA low, I2C timeout, and recovery are
      individually tested; no infinite busy waits and no unexpected
      changes to I2C selected SCL.
- [ ] Clock configuration and TIM5/TIM11 timing checked with production
      interrupt priorities and all required peripheral loads.
- [ ] Safe reset/recalibration; no vehicle motion during calibration.
- [ ] UART stress test: no blocking print in TIM11, bounded low-priority
      telemetry, no untracked serial frame corruption.
- [ ] Sensor fault flags propagate to motion's safe-stop interface.
- [ ] Regression comparison (RC1 vs integration) for stale/reused gyro,
      max sensor age, I2C error, 1kHz ISR load and watchdog behavior.

## Stage D — Physical integration (NOT DONE)

- [ ] Start with motors disconnected; exercise final MPU + other sensors
      concurrently, logging runtime faults.
- [ ] Perform unloaded, current-limited and time-bounded motor pulses
      (separate hardware safety gate) while logging I2C & TIM11. Check
      for supply noise, brownouts and I2C freshness regressions.
- [ ] Static yaw drift across several calibration cycles and temperatures.
- [ ] Known-angle turns/encoder reference to quantify yaw error during
      robot movement; revisit deadzone, LPF and bias logic as needed.
- [ ] Test stall/wheel jam and power loss without treating absence of
      an onboard current sensor as an internal current-limit guarantee.

## Quantitative gate to carry forward

Do not erase thresholds or results. A new integration run must at minimum
have complete startup/attestation, no timing-missed-deadline, zero I2C error/
timeout, zero UART TX drop and no corrupted header rows. In addition report
the same freshness metrics and stage loads relative to the observed RC1
baseline: **100 reused ticks / 227001 (0.0441%)**, two >2ms completed
gyro-read gaps and 99 >1.5ms gyro completion-age ticks. Any regression
requires investigation. The single-run reference numbers are NOT universal
hard limits; set qualified limits with repeated, representative tests.

**Exit rule:** Label `production-integrated` only after stages B–D have
test evidence. Until then RC1 is a useful, reproducible, motor-off driver
baseline and can be used as the reference for integration changes.
