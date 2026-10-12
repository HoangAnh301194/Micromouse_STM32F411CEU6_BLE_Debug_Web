# MPU6050 / I2C1 — RC2 TIM11 scheduler freeze (13 October 2026)

> **RELEASE STATUS: MOTOR-OFF INTEGRATION QUALIFIED.**
> Source code frozen to an identified tested commit. Not certified for
> motor-on electrical noise, dynamic yaw accuracy, or production motion.
> Do not confuse **RC2 release candidate** with a finished robot.

## Immutable reference and provenance

- Validated source commit (no functional edits after A/B tests):
  `5219e04d3fe7b8fdb5a092100154e6d9f0317e09`.
- Release branch: `release/mpu6050-i2c-rc2-tim11-20261013`.
- Experimental source branch: `feature/mpu-tim11-tick-scheduler-ab`.
- Prior baseline: `release/mpu6050-i2c-rc1-20261012`, unchanged.
- Every release source file's Git blob SHA is recorded in
  `docs/mpu_i2c_rc2/MANIFEST.json`. This is a checksum-backed source
  **snapshot**, not a separately duplicated implementation.
- Source code and test logs are inherited intact from the source commit.
  This release branch adds **documentation and inventory only**.
- Git refs are movable unless branch protection is configured.
  **Use the full validated commit SHA or verify the manifest**, rather than
  assuming the release branch name alone provides cryptographic immutability.

## Module contract

**Hardware:** STM32F411CEU6, MPU6050 WHO_AM_I=0x68, I2C1 at 400kHz,
PB8/PB9; nominal SYSCLK 96MHz with HSI trim25 (not a precise reference
clock measurement).

**MPU startup and data:** Gyro Z ±2000dps, DLPF_CFG=3, SMPLRT_DIV=0;
initial 5-second stationary warmup, then 4000 valid calibration gyro
samples in 8 stability blocks. Sampling target is one fresh gyro sample
per 1kHz TIM11 control period; this is a target, **not a guaranteed
sample at every tick**.

**Policy B:** `MPU_SAMPLING_POLICY_B` independent of
`MPU_FRESHNESS_DIAG`. Check INT_STATUS.DATA_RDY before reading Gyro Z;
on not-ready, skip stale data, attempt at most one nonblocking +220us
TIM5 compare retry if inside the original 600us initiation / 850us
expiry limits. Normal Gyro Z acquisition uses I2C1 event/error IRQs,
**not DMA RX**. Recovery/timeouts run outside the hard real-time ISR.

**TIM11 scheduler:** When `MPU_TIM11_TICK_SCHED=1`, each 1kHz TIM11
IRQ calls `MPU6050_NotifyControlTick(..., entry_us)`, which only
increments a sequence/timestamp. Main's `MPU6050_Update()` starts at
most one new I2C status transaction per **latest unconsumed tick
opportunity**, only if the acquisition state machine is idle.
Missed opportunities are **dropped**, never queued or replayed.
No I2C, no busy waiting and no UART printing inside TIM11.

**Consumer:** On successful post-calibration gyro acquisition, main
atomically publishes `MPU6050_GetSnapshot()` fields:
`yaw_deg, gyro_z_dps, completed_us, sequence, valid`. The published
completion timestamp represents MCU main observing completed transfer,
not the MPU's internal sensor sampling instant. The ordinary
`Control` motion path rejects invalid/>3000us-old snapshots via
`Motion_Stop()` (motor-enabled behavior remains **untested**).

## Validated firmware build

The **actually tested RC2 artifact profile** is:

```ini
[env:mpu_phase_tick_96m_trim25]
build_flags =
    -DCLOCK_VALIDATE_96M=1
    -DCLOCK_HSI_TRIM=25
    -DMPU_SAMPLING_POLICY_B=1
    -DMPU_INTEGRATION_BENCH=1
    -DMPU_PHASE_AB=1
    -DMPU_TIM11_TICK_SCHED=1
```

This is an **instrumented, motor-OFF test application**. The main entry
uses `Control_SetDiagnosticOnly(1)`; the default
`genericSTM32F411CE` profile does **not** automatically enable
Policy B + tick synchronization. Simply flashing the default env does
not deploy the RC2 validated behavior.

For later integration into motor-control firmware, both
`MPU_SAMPLING_POLICY_B=1` and `MPU_TIM11_TICK_SCHED=1` must be
defined together and the Control/TIM11 hook kept. An
instrumentation-free motor-enabled configuration has **not** been
validated; enabling it is a separate qualification step.

## A/B hardware validation (240 seconds per capture, motor OFF)

There were two completed order-reversed comparisons, in the same
motor-OFF stationary configuration.

| Test | First pair A→B | Repeat pair B→A |
| --- | ---: | ---: |
| Legacy elapsed-time: published reuse /225000 | 446 = 0.19822% | 449 = 0.19956% |
| TIM11-tick: published reuse /225000 | 223 = 0.09911% | 225 = 0.10000% |
| Legacy long request gaps >1500us | 223 | 225 |
| TIM11 long request gaps >1500us | 1 | 0 |
| Integration motor-OFF test for TIM11 | PASS | PASS |
| I2C errors / timeouts, TIM11 timing violations | 0 | 0 |
| UART drops, >3ms expired snapshots post-cal | 0 | 0 |

**Result:** ~50% fewer reused gyro samples and approximately complete
elimination of >1500us request gaps, replicated in reverse run order.
The new scheduler still reuses ~0.10% of gyro samples at TIM11 ticks;
it **does not guarantee perfect 1kHz freshness**, and its residual
5 samples / 5000 ticks should not be concealed.

RC1 diagnostic overall reuse = 0.0441%, stage-0 reuse = 0.10508%.
Different ISR/UART workloads make those comparisons non-equivalent.
The only controlled scheduler decision is the two same-branch A/B pairs
above. Raw captures and reports are pinned in the manifest.

CI verified: GitHub Actions run
`38290813533` succeeded for the tested source commit, including RC1,
the no-diagnostic Policy B integration, paired phase instrumented builds,
TIM11 scheduler trial firmware, and host synthetic self-tests. CI
build success is independent from the subsequent physical motor-OFF
captures.

## Reproduce / use release snapshot

```powershell
git fetch origin
git switch release/mpu6050-i2c-rc2-tim11-20261013

python tools/mpu_phase_ab_analyze.py --self-test
python tools/mpu_tick_scheduler_ab_compare.py --self-test
python tools/mpu_integration_regression.py --self-test
python tools/mpu_integration_regression.py --source-check

python -m platformio run -e mpu_phase_tick_96m_trim25
```

To reproduce a motor-OFF 240s capture only, after successful build,
with actuator power safely disabled:

```powershell
python -m platformio run -e mpu_phase_tick_96m_trim25 -t upload
python tools/mpu_phase_ab_analyze.py --port COM6 --seconds 240
```

Press RESET **once after the COM port opens**. This is not a motor
validation program.

See `INTEGRATION_CHECKLIST.md` for remaining production gates;
`MANIFEST.json` for file/evidence checksums and full source provenance.

## Freeze policy

- Do **not** modify driver code, ISR code, configuration, or pinned
  raw evidence directly on this RC2 branch.
- For motor integration, create a new `feature/...` branch from the
  pinned commit/release branch; compare before any merge.
- If a bug requires firmware changes, define a new RC revision and
  rerun relevant regressions. Do not silently call it the same RC2.
- The release branch is a documentation-backed reference and **not
  server-enforced read-only protection** unless GitHub branch rules
  are separately configured.

## Remaining limits

Not measured/qualified: under motor electrical switching noise;
long-term/high-speed motion yaw drift; fault-injection of NACK,
bus-stuck and timeout with normal control output; all integrated
motion and recovery responses; formal ISR worst-case timing under
maximum combined workload; MPU full IMU six-axis orientation.
RC2 does not imply optimized Motor Driver.
