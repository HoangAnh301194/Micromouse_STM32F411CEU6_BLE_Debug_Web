# Encoder — unified qualification pipeline (2026-10-20)

## Purpose

One host-side workflow to audit the **actual** `src/encoder.c` logic, build the locked diagnostic image, run existing integration regressions, compare several PPS estimators in a clearly labeled *synthetic* A/B study, capture UART health and two wheel counters, and review archived/current motor sweep results.

**Important:** this is a diagnostic pipeline, **not** automatic modification of production firmware or PID gains. A 1-Hz `SYS_E` stream does not support claims about 1-kHz noise, precise edge loss, or dynamic PID quality. No independent mechanical reference has been supplied.

Source head when implemented: `giomuadongbac`, after `0bb5caa`. Main entrypoint: `tools/encoder_qualification_pipeline.py`.

## Hardware / safety contract

- STM32F411CEU6, ST-Link, PA9 TX / PA10 RX at 115200 8N1 with **3.3 V TTL** and common ground, external 2S motor VM.
- Native and synthetic benchmarks require **no board**, no motor voltage and no flash.
- In the normal capture mode, motor VM must be **physically disconnected**, and the firmware must be the dedicated `integration_diag_encoder_motor_off_96m` profile (or the older motor-off profile with only 1-Hz SYS_E). A runtime SYS_M `locked=1` is verified before accepting the capture, plus SYS_E applied PWM must remain zero.
- An optional diagnostic ST-Link upload compiles and flashes **only** `integration_diag_encoder_motor_off_96m`, which inherits the sticky motor-OFF lock and adds an opt-in ~50-Hz SYS_EN packet. Never use a production/motor-capable image in this phase.
- Motor-on is **not enabled by default**. The separately gated existing `system_uart_pipeline.py` performs 33 levels, left/right/both, up to **100% physical PWM**, after its 750-ms serial heartbeat and MPU/calibration checks. It is not a safe low-speed-only encoder test. Operator must confirm securely lifted/guarded wheels, independent immediate VM disconnect, understanding of 100% PWM, and direct attendance. At end the wrapper attempts to restore a motor-OFF image; if restore fails, independently disconnect VM and verify the firmware, not merely a successful host command. Do not rely on software STOP as the sole emergency stop.
- Existing regression logs and motor driver/MPU/I2C/IR source are not modified.

## One-command workflows (PowerShell, repository root)

Start with working tree verification, preserving any local changes:

```powershell
git branch --show-current
git rev-parse --short HEAD
git status --short
git fetch origin
# Pull with --ff-only ONLY after checking local work
git pull --ff-only origin giomuadongbac
```

**A. Comprehensive software audit, build, estimator A/B, historical PWM evidence (no physical actions):**

```powershell
python tools/encoder_qualification_pipeline.py --mode offline
```

**B. One unattended, safe motor-OFF session; rebuild and upload only locked firmware, then 120-s capture.** Before this command physically disconnect motor VM and verify UART/ST-Link:

```powershell
python tools/encoder_qualification_pipeline.py --mode full --port COM7 --seconds 120 --confirm-vm-disconnected --flash-motor-off
```

If the correct motor-locked diagnostic image is **already installed**, omit `--flash-motor-off` to keep ST-Link inactive. The host does not write anything over UART in this mode.

**C. Explicitly authorized whole-platform sweep with motor power, not a default follow-up.** This is provided for after separately confirming the mechanical/electrical test fixture and accepting 0..100% PWM. **Do not execute casually**:

```powershell
python tools/encoder_qualification_pipeline.py --mode full --port COM7 --motor-on-sweep --fixture-ready
```

This runs static/native tests, a safe OFF phase, and then the previously qualified diagnostic ON stages through `system_uart_pipeline.py`. It needs a connected ST-Link and a human operator present throughout. No source-level auto-tuning takes place.

## What is tested

| Stage | Evidence | Status meaning |
| --- | --- | --- |
| Encoder register audit | `SMCR.SMS=3`, 16-bit ARR, TIM11 1-kHz call, motor-off build flag | Source contract check only |
| C-native encoder regression | Compiles actual `src/encoder.c` against independent STM32 register stubs: both wheels, direction baseline, rollover, reset, EMA, 10,000 ticks | Host-executed regression; not EMI qualification |
| Existing integration gates | Integrations, negative cases, frozen Motor Driver, sweeping state machine, IR regression | Preservation checks only |
| Firmware compile | PlatformIO `integration_diag_motor_off_96m` | Compile, not flash unless opt-in |
| Estimator A/B | Synthetic quantized 1-ms pulses vs 1/5/10/20-ms windows; outputs metrics and time series | Simulation only — do not auto-apply winner |
| UART locked capture | Stores raw UART; extracts `SYS_EN` (~50 Hz), `SYS_E` (1 Hz), `SYS_M`, `SYS_T`, `SYS_U` | Counter and whole-system health at telemetry sampling rate |
| Archived/new full PWM sweep | `SYS_STEP` and coast data summarized from existing `summary.json` | Commanded PWM vs encoder-count response, not electrical/ground-truth validation |

Test IDs in `summary.json` show PASS/FAIL individually; critical failure prevents proceeding to hardware. Outputs are stored under a **new** `results/encoder_qual_YYYYMMDD_HHMMSS/` directory (or an explicitly designated new path). The pipeline never overwrites historical files.

Typical outputs:

- `REPORT.md` — qualification report and remaining limitations.
- `summary.json` — machine-readable results, observed MCU error counters, stage metrics.
- `encoder_native.log`, `build_locked.log`, individual gate logs.
- `estimator_metrics.csv`, `estimator_samples.csv`, `estimator_report.json` — **synthetic** comparison.
- `uart_raw_motor_off.log`, `locked_capture.csv` — raw capture and 1-Hz counters.
- `encoder_50hz.csv` — 20-ms count deltas, estimated window-PPS and existing firmware EMA PPS when the opt-in encoder profile is installed.
- `sweep_encoder_stages_historical.csv` / `sweep_encoder_stages_motor_on.csv` where evidence exists.
- `motor_on_automated.log`, `system_uart_hardware/...`, `restore_motor_off.log` only for an explicitly authorized motor-on sequence.

## Before claiming Encoder module completion

1. Investigate hardware IC1F/IC2F filtering and verify maximum valid quadrature edge rate, avoiding filter-induced loss.
2. The opt-in motor-off encoder profile now emits 50-Hz SYS_EN; compare 20-ms PPS with firmware EMA first. Higher-rate timestamps and an external reference are still needed to assess missed edges confidently.
3. Check missed edges against an external revolution/distance reference, rather than interpreting a software counter alone as proof.
4. Choose a wheel-speed estimator by real step-response and repeatability, then tune wheel-speed PI separately with a dedicated bounded motor safety and stop plan.
5. Maintain clock/TIM11, MPU/I2C, IR and Motor Driver nonregressions, and record firmware hash plus raw log for each test.

## Dependencies

- Python 3 (host), gcc for C native regressions, PlatformIO for build/flash, pyserial for UART.
- Matplotlib is **not required** by this script. CSV can be plotted offline later.
- Unit-only quick check: `python tools/encoder_qualification_pipeline.py --self-test`.

## Motor-driven Encoder benchmark (no manual turning)

One command after wheel fixture setup: upload encoder OFF, run 33 shadow stages; upload encoder ON, run the same 33 stages at 0,10,...,100% PWM by LEFT / RIGHT / BOTH; record 50-Hz encoder data during both; analyze health and stage response; then attempt restoring motor-locked OFF firmware.

Use `--motor-on-sweep --fixture-ready` (command above). The former already authorizes up to 100% PWM. No separate high-PWM confirmation and no manual rotation or timing required. The fixture-ready flag means the wheels are securely lifted/guarded, an independent physical VM cutoff exists, and the operator remains present.

Hardware telemetry during the ON sweep is collected via `integration_diag_encoder_motor_on_96m`. This profile inherits the existing motor-on serial/MPU heartbeat/fault-latch state machine and adds decimated `SYS_EN` samples (~50 Hz). The equivalent encoder OFF profile also emits `SYS_EN`.

Additional outputs: `encoder_motor_off_50hz.csv`, `encoder_motor_on_50hz.csv`, and (with matplotlib) `encoder_motor_off_speed.png`, `encoder_motor_on_speed.png`. These store window-PPS, existing EMA PPS, stage/group/PWM labels and system-health summaries. CSV/logs are written to new result paths. No PID/autotuning changes are made.

The existing hard deadline, serial STOP and 750-ms heartbeat are retained. A software STOP or post-test flashing is never a substitute for an independent physical VM cutoff.

Remaining open: true missed-pulse rate, external RPM accuracy, PWM electrical waveform, current, thermal limits, PID speed closed-loop qualification. None can be inferred solely from MCU timer counts.

## I2C rate fallback observed in 2026-10-20 motor-on log

- Historical diagnostic ON at `results/encoder_qual_20261020_014805/` recorded `iclk=296296` in `SYS_U` throughout; OFF was `400000`. `iclk` is configured nominal STM32 I2C timing, not a physical SCL measurement.
- `MPU6050_Init()` probes 400, 300, 200, 100 kHz; it requires both address ACK and WHO_AM_I=0x68. The 296296 value is 48 MHz / (3 * 54), the integer-divider result for the 300-kHz fallback choice.
- The old 400-kHz host acceptance check is intentionally retained. Successful MPU calibration and zero I2C read errors at ~300 kHz do not prove 400-kHz stability.
- The encoder-only ON/OFF diagnostic profiles now emit `SYS_IPB` (~1 Hz) with `idx`, `mode`, and each trial's `aN` address status, `wN` WHO read status, and `idN` WHO value (N=0 for 400k, N=1 for 300k). The next sweep will show which 400-kHz initialization probe failed; it is not assumed to be motor EMI because boot/probing precedes the PWM drive.
- When all 33-stage OFF/ON checks and 50-Hz Encoder records complete but the only system failure is `i2c_400k_nominal`, the wrapper reports `SWEEP_COMPLETE_I2C_RATE_UNQUALIFIED` with encoder measurements retained and an I2C warning. It still returns a nonzero exit status, preserving strict system qualification.
- Do not force I2C to 400 kHz, change pull-ups or weaken MPU bootstrap recovery solely from this single fallback run. Analyze `SYS_IPB` first.
