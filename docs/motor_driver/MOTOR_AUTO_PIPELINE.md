# Motor Driver — one-command multi-phase diagnostic v3

**Branch:** `feature/motor-driver-optimization`
**Entry point:** `tools/motor_auto_pipeline.py`
**Opt-in PlatformIO profile:** `motor_auto_pipeline_96m_trim25`
**Protocol:** `MOTOR_AUTO_PROFILE,version=3`
**Hardware limits:** fixed PCB; no hardware modifications; TB6612 VCC=5V, STBY hard-wired 5V; STM32 I/O nominal 3.3V; N20 rated 12V; VM from 2S battery, current/driver temperature not measured.

## Objective — one flash, one capture, one report

All **four low-level diagnostic families** execute sequentially in one
firmware session after a single host START. Each of the **21 stages runs
three times** (63 total short pulses). The script builds and uploads the
firmware once, connects USART1, verifies the firmware profile and
bidirectional PING, sends START, maintains KEEP heartbeat, captures every
UART sample, and writes CSV/JSON/text reports. No KEY or manual RESET.

| Family | Stage indices (within each round) | Left / right commanded PWM | Slew rise rate | What it measures |
| --- | --- | --- | ---: | --- |
| Startup | 0–9 | single-wheel 8%, 9%, 10%, 11%, 12%, left then right at each duty | 10 permille/ms | repeatable startup threshold and first encoder counts |
| Response | 10–16 | left/right at 12%,16%,20% each; both together 16% | 10 permille/ms | encoder counts vs command, asymmetry and simultaneous operation |
| Slew slow | 17–18 | left then right at 16% | 5 permille/ms | impact of gradual rise |
| Slew fast | 19–20 | left then right at 16% | 20 permille/ms | comparison with faster rise |

For every stage: **200ms active PWM**, then **400ms at software-zero
PWM** with encoder telemetry, then 250ms stage-to-stage idle interval.
Initial neutral settling is 1000ms. Sampling every ~20ms. The
maximum per-wheel command is **20%**, the simultaneous-wheel limit is
**16%**, and no reverse, short-brake, deliberate stall or PID motion
controller is exercised. This is **not current limiting**.

## Single command on Windows (PowerShell)

Update the branch once:

```powershell
git fetch origin
git switch feature/motor-driver-optimization
git pull --ff-only origin feature/motor-driver-optimization
```

For a complete experiment with both wheels **elevated and free to spin**
and an immediate physical VM disconnect available:

```powershell
python tools/motor_auto_pipeline.py --run --port COM6 --vm-state powered --meter-vm-volts 8.2
```

The 8.2 value is a **manual voltmeter observation**, never measured by
the STM32; replace or omit it as appropriate. The script takes care of
build, STLINK flashing, serial capture and analysis. It now permits a
longer default UART capture timeout (150s) for the extended suite.
It ends on `MOTOR_AUTO_DONE` without using all 150 seconds if the
firmware completes sooner.

**Optional on subsequent experiments with exactly the same compiled
firmware**: add `--skip-flash` to avoid a second upload. This does
not skip the strict firmware version/profile handshake. Since the
firmware allows only one run per MCU boot, the operator must RESET it
before attempting to run again using `--skip-flash`; the normal
one-command run includes STLINK reset/upload.

## Boundaries and protections

- No automatic startup on MCU reset, no KEY handling. No PWM until the
  host opens a **bidirectional** USART1 link and sends `MOTOR START`.
- Existing USB-UART TX must be physically connected to STM32 USART1 RX
  PA10, in addition to USART1 TX PA9 and shared ground. The last
  `auto_20261013_093808` experiment successfully completed this
  handshake.
- The host checks protocol v4 with `rx_mode=irq256`, 3 rounds × 21 stages, 20% command cap,
  16% simultaneous-wheel cap, 200ms drive, 400ms neutral window and
  idle firmware state before START. Old v2/v3 firmware will be rejected, including with `--skip-flash`.
- `MOTOR KEEP` heartbeat about every 350ms; on >1600ms host silence,
  a monitored TIM11 >1.5ms gap or unexpected pulse end, firmware
  latches fault, stops generating PWM commands and refuses further
  stages. The MotorTest tick independently ends each pulse at 200ms.
- STOP and MCU fault handling only provide a **software** neutral
  output. STBY is wired permanently HIGH and no external current
  limiting has been verified. Immediately remove VM on unexpected
  motion, smoke, smell or heating; never lock the wheels.

## Outputs: single session directory

`motor_logs/auto_YYYYMMDD_HHMMSS/`

1. `raw_serial.log`: untouched timestamped UART.
2. `motor_auto_samples.csv`: round, stage, test, phase, rise, applied
   duty, TIM2 CCR, encoder counts, TIM11 timing and UART drop counters.
3. `motor_auto_report.json`: 63 expected stage records, including
   `NOT_TESTED` for skipped stages (never fabricated as 0 count),
   `PARTIAL` for incomplete three-run observations, and
   `MOTOR_AUTO_LINK` decoded-heartbeat/RX overrun diagnostics, 3-repetition statistics
   (mean, sample standard deviation, heuristic first-five-count time,
   sustained encoder count, motion after STOP), and strict pass/fail
   of protocol completeness, timing, no motor-PWM after stop, command
   ceiling, expected stage family and slew configuration.
4. `motor_auto_summary.txt`: per-condition condensed findings.
5. `session_meta.json`: declared VM state, optional hand-measured DC
   voltage and number of host `MOTOR KEEP` transmissions.

**HOST_TIMEOUT fix:** v3 used polling for USART1 RX while main was
printing a dense telemetry stream; RX bytes could be overrun unnoticed.
v4 uses an interrupt-fed 256-byte RX ring, decoded KEEP count, explicit
USART RX error counters and 250ms host KEEP cadence. The timeout remains
1600ms and still causes a latched software stop. The actual root cause
of the October 13 v3 failure is a plausible RX timing hypothesis, not
proven in the pre-fix log. See
[`HOST_TIMEOUT_RCA_20261013.md`](HOST_TIMEOUT_RCA_20261013.md).

`TRACE_COMPLETE_NOT_ELECTRICAL_APPROVAL` indicates **trace validity
only**, never electrical safety, current, winding temperature or
real PWM waveform verification.

Afterward, push the result once:

```powershell
git add motor_logs/
git commit -m "test(motor): capture v3 integrated 3x21 motor baseline"
git push origin feature/motor-driver-optimization
```

## What this suite deliberately excludes

Active electrical short-braking, reversing at speed, high duty/high
load, deliberately stalled wheels, measured motor current, oscilloscope
validation of PWM waveforms and **MPU/I2C mixed-load** operation.
Those require independent safety gates or a later integration profile
and cannot be inferred from encoder-only traces. The 8–12% startup
boundary is a statistical observation in an elevated-wheel test,
not a universally calibrated deadband under load. Encoder
counts-per-revolution have not been independently calibrated.

Toshiba TB6612 datasheet documents VIH minimum = 0.7×VCC, so logic
VCC=5V implies a guaranteed HIGH threshold 3.5V: above nominal
STM32 3.3V output. This fixed-configuration margin risk persists
despite previous successful motor rotation.
