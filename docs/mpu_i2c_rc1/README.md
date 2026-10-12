# MPU6050 / I2C1 — RC1 freeze (12 Oct 2026)

> **STATUS: DIAGNOSTIC-VALIDATED / INTEGRATION NOT YET VALIDATED.**
> This is a pinned **motor-off** source snapshot plus measured acceptance evidence,
> NOT a production-ready Micromouse inertial module.

## Provenance and reproducibility

- Hardware: STM32F411CEU6, MPU6050 at 7-bit address `0x68` over I2C1 PB8/PB9.
- Source baseline commit: `7e71d611bfedcc173f7d2798ba51fd6db49b59d9`
  (the firmware source used for the 22:26:40 validation run).
- Release branch: `release/mpu6050-i2c-rc1-20261012`.
- Validation capture: `clock_logs/pipeline_20261012_222640` on
  `feature/motor-debug-console`, and copied summary evidence in this folder.
- The snapshot preserves validated source/headers; no motor PWM tests
  or unverified subsequent motor-bench modifications were imported.

## Frozen source inventory

| File | Purpose | Notes |
| --- | --- | --- |
| `src/i2c.c`, `include/i2c.h` | I2C1 bus, async 1/2-byte register receiver, IRQ, timeout and recovery | Event/error IRQ driven for gyro reads; **not DMA RX** |
| `src/mpu6050.c`, `include/mpu6050.h` | MPU startup, 1kHz nonblocking acquisition, calibration, filtering, yaw | Gyro Z only; no proven 6-axis fused orientation |
| `src/system_timer.c`, `include/system_timer.h` | TIM5 timestamps; one-shot retry wake at +220us | Retry wake compiled only when Policy B enabled |
| `src/control.c`, `include/control.h` | TIM11 1kHz diagnostics/sample age/load measurement | Motor output remains disabled in validated profile |
| `src/board.c`, `include/board.h` | Nominal 96MHz HSI/PLL trim25 clock profile | Nominal != calibrated traceable frequency |
| `src/main.c` | Isolated motor-OFF acquisition, calibration and telemetry | NOT the final robot main/application |
| `platformio.ini` | Build flags for reproducible diagnostic firmware | Default env is **not** Policy B |

No source copy is intentionally duplicated in this release directory; the
branch itself is the source snapshot. Avoid divergent copies of drivers.

## Verified configuration — do not silently substitute

```ini
[env:control_load_staggered_96m_trim25]
build_flags =
    -DCLOCK_VALIDATE_96M=1
    -DCLOCK_HSI_TRIM=25
    -DMPU_FRESHNESS_DIAG=1
    -DMPU_FRESHNESS_AB_B=1
    -DCONTROL_LOAD_DIAG=1
    -DCONTROL_LOAD_LOG_STAGGER=1
```

- Board SYSCLK: nominal 96 MHz, PLLN=192, HSI trim=25.
- I2C1 selected 400 kHz; address `0x68`, WHO_AM_I `0x68`.
- MPU gyro range: +/-2000 dps, DLPF_CFG=3, SMPLRT_DIV=0;
  goal one completed Gyro Z read per 1ms control tick.
- Acquisition: async `INT_STATUS` read then `GYRO_ZOUT_H/L` read,
  driven from **main**; interrupts complete I2C transfers.
- Policy B: if status says not ready, do NOT read gyro; schedule at most one
  nonblocking 220us status retry, staying within the current sample slot.
- Calibration: 5s warmup + 4000 accepted gyro samples; 500-sample stability
  blocks and retry/reject limits in `include/mpu6050.h`.
- TIM11 never blocks waiting for I2C nor prints UART. UART LOAD logs
  staggered from the 1Hz bulk diagnostics.

**Critical integration trap:** in this source revision, the Policy B
skip/retry logic is controlled by **diagnostic build flags**, and
`main.c` always starts `Control_SetDiagnosticOnly(1)` for this
experimental application. Building the ordinary default environment
does **not** inherit the validated sampling policy. Do not call it
production-integrated until this coupling is removed and revalidated.

## Hardware acceptance: `pipeline_20261012_222640`

All figures below are observed over **one stationary, motor-off 240s capture**,
not a mathematical worst-case or evidence of operation during moving loads.

| Gate / measurement | Result |
| --- | ---: |
| IMU / freshness report | `PASS_DIAGNOSTIC` |
| staged scheduler load | `PASS_LOAD_DIAGNOSTIC` |
| pipeline mandatory checks | 9 / 9 true |
| startup frame corruption | 0 detected |
| TX drops and CLK missing sequences | 0 and 0 |
| calibration | 4000 samples; 0 retries |
| I2C errors/timeouts | 0 / 0 |
| MPU DATA_RDY status not ready | 1,736 over post-calibration window |
| gyro samples reused by TIM11 | 100 / 227,001 = 0.0441% |
| accepted gyro sample gap > 2 ms | 2 occurrences |
| post-cal gyro completion age > 1.5 ms | 99 ticks |
| measured ISR maxima stages 0/1/2 | 1.969 / 8.167 / 22.844 us nominal |
| 800us and 1000us budget overruns | 0 in all three stages |
| host-estimated SYSCLK | ~95.7401 MHz (−0.27073% from nominal) |

The clock estimate compares TIM5 to a Windows monotonic host clock over
USB/UART; it is **not** reference oscillator or oscilloscope calibration.
Different captures yielded varying clock estimates, so do not quote a
guaranteed physical MHz. `fresh` refers to sensor status at MCU polling time,
not the hidden internal acquisition timestamp. An unchanged gyro sample used
on two control ticks is **not** proof of duplicate raw codes.

Evidence copied into `docs/mpu_i2c_rc1/evidence/`:
- `imu_freshness_report.json`
- `load_stage_report.json`
- `pipeline_report.json`
- `clock_summary.txt`

Full raw capture and clock samples are in the source repository under
`clock_logs/pipeline_20261012_222640/` on the feature branch. Those logs
are **not** included in the lightweight report-only release folder.

## Rebuild reproducibly — MOTOR OFF

```powershell
git fetch origin
git switch release/mpu6050-i2c-rc1-20261012
python tools/imu_freshness_analyze.py --self-test
python -m platformio run -e control_load_staggered_96m_trim25
```

Compiling is not a motor test. To repeat hardware validation, power the board
with **motor supply disconnected**, upload the same environment, then collect:

```powershell
python -m platformio run -e control_load_staggered_96m_trim25 -t upload
python tools/diagnostic_pipeline.py --port COM6 --seconds 240
```

Press STM32 RESET after COM opens. Verify complete `SYNC_POLICY`, `I2C_PROBE`,
calibration, clock, UART and all stage reports before accepting the new run.

## Release criteria and scope

**Frozen/accepted for its measured scope:** I2C1 400kHz startup, Gyro Z
async sampling strategy B, stationary calibration, TIM11 1kHz timing,
retry/wake path and motor-OFF staggered UART scheduling.

**Not accepted yet:** MPU + IR + BLE + motor simultaneously, release behavior
under supply dips and EMI, forced bus-stuck/timeout recovery, moving yaw drift,
accurate absolute clock frequency, reliable fault/stall current protection
or PID performance. A separate integration gate is required; see
[`INTEGRATION_CHECKLIST.md`](INTEGRATION_CHECKLIST.md).

**Do not call this "globally optimal"**: it is one measured configuration
which met the current diagnostic acceptance tests.
