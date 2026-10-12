# MPU6050/I2C Policy B — controlled TIM11 tick scheduler A/B

**Experiment branch:** `feature/mpu-tim11-tick-scheduler-ab`
**No production release yet. No motor changes. DMA is not used for MPU gyro.**

## Why this branch exists

The previous `PHASE_AB` stationary 240-second run
`mpu_phase_ab_logs/session_20261013_002704` established:

- **A (I2C IRQ completion): 451 / 225000 = 0.20044% reuse.**
- **B (published gyro snapshot): 452 / 225000 = 0.20089% reuse.**
- Only 1 additional reuse between IRQ completion and main publication.
- Request gaps over 1500us: 228; over 1050us: 453;
  maximum 2000us. IRQ→main average 2us, main→publish average 13us.
- Recurring 5-second pattern: 4995 requests vs 5000 TIM11 ticks.
- Zero I2C timeout/error, missed TIM11 timing, post-calibration
  expired >3ms snapshots or UART drops.

This narrows the likely cause to **acquisition request scheduling relative
to TIM11**, not filtering or publication overhead. However, the exact
causality has NOT been proven before this controlled change.

## Only one functional change in the experimental B build

**A (legacy):** `mpu_phase_ab_96m_trim25`

```c
if (sample_phase == 0 &&
    (uint32_t)(now - last_request_us) >= 1000) {
    I2C_ReadRegAsync_Start(...);
}
```

**B (tick experiment):** `mpu_phase_tick_96m_trim25`

```c
// TIM11 ISR: update tick sequence/timestamp only, NO I2C
MPU6050_NotifyControlTick(control_mpu, entry_us);

// Main loop, when I2C sample state machine is idle:
if (sample_phase == 0 &&
    control_sample_tick_seq != 0 &&
    control_sample_tick_seq != control_last_started_tick_seq) {
    // Consume the LATEST tick opportunity once on a successful START.
    I2C_ReadRegAsync_Start(...);
}
```

A tick is a **sampling opportunity, not a backlog**. If main is still
finishing a previous transaction, old ticks are never replayed. Both
profiles use the same pre-calibration initialization, I2C event/error
IRQs, hardware gyro settings and **production Policy B** status/retry
(+220us once, 600us retry start gate, 850us slot expiry).

TIM11 never performs I2C reads, busy-waits, UART printing, or DMA setup.
The old default / production environments do NOT define
`MPU_TIM11_TICK_SCHED`. This option must NOT be enabled by default
before A/B evidence is accepted.

## A/B experimental controls

Perform **both** runs on the **same PCB, supply and stationary placement**.
Do not change gyro DLPF, I2C speed, CPU clock, code revisions,
BLE/IR/encoder load, logging or calibration between A and B.
Both are motor-OFF with identical `MPU_PHASE_AB` counters:
`PHASE_REQ`, `PHASE_LAT`, `PHASE_PAIR` every 5s and `MINT` every 1s.

`PHASE_PROFILE` explicitly reports:
- `scheduler=legacy_elapsed` for A.
- `scheduler=tim11_tick` for B.

The analyzer reads each raw log and enforces those labels; existing older
unlabeled captures are NOT accepted as a new matched A/B pair.

### 1. Fetch and software checks

```powershell
git fetch origin
git switch feature/mpu-tim11-tick-scheduler-ab

python tools/mpu_phase_ab_analyze.py --self-test
python tools/mpu_tick_scheduler_ab_compare.py --self-test
python tools/mpu_integration_regression.py --self-test
python tools/mpu_integration_regression.py --source-check

python -m platformio run -e control_load_staggered_96m_trim25
python -m platformio run -e mpu_phase_ab_96m_trim25
python -m platformio run -e mpu_phase_tick_96m_trim25
```

If compilation or self-tests fail, **do not flash**; share the failure log.
Build success does not demonstrate sensor timing.

### 2. Capture A — legacy schedule

Ensure **motor supply VM is disconnected** and robot is stationary.
Only power MCU and MPU logic from suitable supply. Use the correct serial
port (example COM6). A record beginning before RESET is harmless as the
analyzer checks for one complete tagged boot following RESET.

```powershell
python -m platformio run -e mpu_phase_ab_96m_trim25 -t upload
python tools/mpu_phase_ab_analyze.py --port COM6 --seconds 240
```

Press STM32 RESET **once** immediately after the port opens. Rename or
retain the generated session folder; do not overwrite.

### 3. Capture B — TIM11 tick schedule

With hardware unchanged:

```powershell
python -m platformio run -e mpu_phase_tick_96m_trim25 -t upload
python tools/mpu_phase_ab_analyze.py --port COM6 --seconds 240
```

Again press RESET once after the serial port opens. Allow the full
5s warmup + ~4s gyro calibration before steady-state comparison.
The serial output should show `scheduler=tim11_tick`.

### 4. Compare BOTH raw captures

Replace placeholders with **actual folder names** printed by the
capture tool.

```powershell
python tools/mpu_tick_scheduler_ab_compare.py `
  --legacy "mpu_phase_ab_logs/session_A" `
  --tick "mpu_phase_ab_logs/session_B" `
  --output "mpu_phase_ab_logs/tim11_scheduler_comparison.json"
```

The comparison tool reads `raw_serial.log` for each run (never trusts
the edited JSON alone), checks the motor-OFF firmware identity,
postcalibration completeness, monotonic counters and contention. Output
includes the relative change in:

- IRQ-completion reuse percentage;
- Published-snapshot reuse percentage;
- Request gaps >1050us and >1500us;
- Max request interval and IRQ→publication latency.

The result is `IMPROVED_REQUIRES_REPLICATION`,
`NO_CONCLUSIVE_IMPROVEMENT`, `REGRESSED_INVESTIGATE`,
or `INCOMPLETE_OR_INVALID`.

The prior 0.15% freshness reuse level is a **provisional review trigger**,
not a validated universal MPU requirement. Even a clean run must be
repeated (preferably reverse B→A as well) before accepting the new
scheduler in the main motion code.

## What to send back / finish gate

Commit or push **both entire session folders** (especially
`raw_serial.log`) and `tim11_scheduler_comparison.json`, plus
build/self-test output. Do not merge experiment into baseline RC1.

If the tick scheduler reduces request gaps and IRQ reuse on repeated
matched runs with zero I2C errors/timeouts and no missed TIM11 deadlines,
we can freeze the sampling scheduler with an explicit API contract.
Only then run a separate motor-electrical-noise integration test.
If both schedules still show ~0.20% reuse, inspect WFI wake races,
interrupt timing and pending I2C state, rather than tuning gyro filter,
I2C speed or PWM prematurely.

**Verification note:** This authoring action reviewed source and added
CI builds/tests but did not execute an ARM PlatformIO build or a
hardware capture. Do not claim integration PASS until evidence exists.
