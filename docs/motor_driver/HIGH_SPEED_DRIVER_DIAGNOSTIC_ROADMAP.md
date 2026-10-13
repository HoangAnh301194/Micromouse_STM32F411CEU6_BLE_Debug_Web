# High-speed Micromouse — Motor Driver remaining tests and integrated diagnostic (2026-10-13)

**Repo branch**: `feature/motor-driver-optimization`
**Evidence reviewed**: `motor_logs/auto_20261013_101254` (valid 63/63 motor-off-load pulses; 192/192 host/Microcontroller heartbeat; 0 observed timing gaps).
**Firmware opt-in high profile**: `motor_auto_high_96m_trim25` (`MOTOR_AUTO_BENCH + MOTOR_AUTO_HIGH_TEST`)
**One Python entrypoint**: `tools/motor_auto_pipeline.py`.
**Parent MPU RC2** is frozen and not modified by this work.

## What the 63-pulse v4 session establishes, and does not

- Established in this specimen/condition: TIM2 command/CCR→encoder response up to 20%, USART1 IRQ RX→192/192 host heartbeats, TIM11 maximum observed 1625 cycles at nominal 96 MHz, reliable encoder counts at 10% on both wheels for 3 short free-spin tests, and extended movement after software PWM returns 0.
- Not established: terminal switching waveform, 30 kHz effective output verified on scope, motor current/stall/driver temperature, motor rated speed, true Hi-Z coast/short brake states, reverse/reversal interlock, load-bearing traction, simultaneous motor + MPU/I2C/IR timing/EMI, performance at duty >20%, or a genuine PWM→motor speed transfer function under constant speed.
- The `Motor_Stop()` implementation sets **IN1=IN2=LOW and CCR=0**; Toshiba TB6612FNG datasheet specifically documents high-impedance Stop as **IN1=IN2=LOW, PWM=HIGH**. Never claim the implemented stop is proven true coast. TIM2 CCR uses preload and GPIO direction changes take effect immediately; direction/duty output transitions need instrumented verification.
- `Motor_Brake()` sets both IN inputs HIGH and CCR=ARR+1, a documented short-brake mode, but *electrical current is unmeasured*. Do not automatically test live short-brake or sudden direction reversal at speed.
- `pinout.h` descriptive header calls right IN1=PB14 IN2=PB15, but compiled macros specify right IN1=PB15 IN2=PB14. Firmware logically drives and encoder counts worked; confirm actual forward polarity from motor wiring and sign (not from comments).
- `pinout.h` legacy `MOTOR_PWM_PERIOD 4999`, and `include/motor.h` historical 20 kHz comments, conflict with active 30 kHz computed TIM2 ARR. These are documentation/consistency issues, not a measured frequency failure.
- Encoder counts per physical wheel revolution and N20 gear ratio, no-load and stall current, driver VM transient, driver temperature are NOT measured. The 1 kHz encoder ticker reads TIM3/TIM4 hardware quadrature counters, so no wheel RPM should be reported without calibration.

## Test inventory, scope and gating

| Gate | Specific test | Automated now? | Decision and rationale |
| --- | --- | --- | --- |
| D0 | Boot neutral, serial version, heartbeat fail-stop | v4 verified, retained v5 | Software guard. Still not independent hardware kill because STBY fixed HIGH |
| D1 | 8/9/10/11/12% start response, 3 repeats per wheel | Yes | 10% left / 9% right lowest observed 3/3, not universal calibrated deadband |
| D2 | 12/16/20% open-loop, both 16%, 3 repeats | Yes | Base motor-response comparison |
| D3 | Rise 5 vs 20 permille/ms at 16%, 3 repeats | Yes | Slew affects 200ms accumulated count |
| D4 | At every stage, software stop and 400ms encoder sampling | Yes | CCR neutral and inertia/noise, not physical Hi-Z |
| D5 | Extended 30/40/50/60% single-wheel forward, 3 repeats | **Opt-in only** | 120ms pulse, 800ms neutral window, encoder motion guard at 100ms and late-settle gate at 800ms; physical current still unknown |
| D6 | Extended high stop/settling/left-vs-right and timing | Opt-in with D5 | Max timer CCR and 1kHz ISR telemetry, steady-speed inference not claimed |
| D7 | TIM2 pin effective waveform / PWM phase | Not possible with available voltmeter | Requires oscilloscope / logic analyzer; register CCR not enough |
| D8 | Exact Hi-Z coast/brake electrical mode, CCR preload transition | Source audit yes; energized-state qualification **not** | Requires scope, current-safe test setup; do not auto short-brake without current data |
| D9 | Running-wheel reverse/interlock (30ms neutral gate) | Not under unknown current | Under load high-speed reversal potentially regenerative/transient; review code/static bench first |
| D10 | Stall, short, mechanical blockage, TB6612 thermal margin | **Not supported** | Current shunt/sensor and power/temperature instrumentation required. No PWM % safely substitutes for current protection |
| D11 | 70/80/90/100% PWM and prolonged speed plateau | **Deliberately excluded for now** | Unknown N20 stall current and TB6612 current/thermal envelope on fixed PCB, not enough to qualify from 20% free-spin evidence |
| D12 | Motor on + MPU6050/I2C/IR workload and timing/EMI | Separate safe integration configuration required | Current motor auto bench explicitly excludes MPU and IR; cannot claim scheduling or sensor reliability under motor switching |
| D13 | Loaded drivetrain, straight-line, cornering, wall sensing, PID speed | **Outside Motor Driver** | Speed-loop, Motion and navigation own these validation goals |

## New opt-in one-command upper-mid/high research profile (NOT 100%)

Run once, with freely spinning, **elevated wheels**, hands and wiring clear of shafts, and an immediate way to physically disconnect motor VM:

```powershell
git fetch origin
git switch feature/motor-driver-optimization
git pull --ff-only origin feature/motor-driver-optimization
python tools/motor_auto_pipeline.py --run --high-pwm --confirm-wheels-elevated --port COM6 --vm-state powered --meter-vm-volts 8.2
```

This selects **exactly one** PlatformIO high-test image and flashes it **once**, uses the verified USART1 RX IRQ + host heartbeat, automatically executes **3 rounds × 29 stages = 87 pulses**:

- Existing **21 stages**: startup 8–12%, response 12–20%, slew 5 vs 20 permille/ms, each with 200ms drive + 400ms software-neutral window.
- **Eight new stages** per round: L/R at **30/40/50/60%** individually (not simultaneous) with 120ms drive and 800ms software-neutral observation.
- The hardware must remain fully supported and wheel-free; this is NOT a declaration that 60% is electrically safe. No current/thermal sensing available. Stage guard aborts if after ~100ms energized motor has <5 counts; high-stage post-neutral guard aborts if count changes by >5 in last 100ms. These guards **cannot** prevent peak startup/stall current or protect against a failed MCU/bridge.
- The firmware **does not accept** ordinary `MOTOR START` in high profile. Only host script after `--high-pwm` strict version 5 handshake sends `MOTOR START HIGH 600`.
- The base v4 environment remains available with the old command (no `--high-pwm`), no high stages.
- A failed stage latches software-neutral state and reports a fault. **Do not repeatedly rerun** a failed high duty, and do not touch stall/brake/reverse when driver electrical limits are unverified.

One output set in `motor_logs/auto_YYYYMMDD_HHMMSS/`: `raw_serial.log`, `motor_auto_samples.csv`, `motor_auto_report.json`, `motor_auto_summary.txt`, `session_meta.json`. Host analyzes expected stage counts and high durations separately. First high run should never use `--skip-flash`.

## Before any genuinely high-speed 80–100% characterization

The fixed PCB cannot be modified, but tests can still establish external limits: identify exact N20 motor and gearbox rating, measure motor no-load and stall currents separately *if safe and properly equipped*, verify TB6612 VM/current/temperature behavior with scope and suitable current probe/shunt, ensure electrical brake and reverse transitions, and establish whether the driver can sustain the expected duty/time under load. Toshiba specifies output-current ratings; neither a 200ms pulse nor a percentage PWM alone constrains the instantaneous current to that rating. The datasheet also requires VIH >=0.7×VCC; at VCC=5V the 3.3V MCU HIGH is below the guaranteed threshold, even if this specific board currently responds correctly.

Only after those gates should a **separate explicit high-risk 80–100% profile** be considered. Do not hide 100% inside an automated 3-round bench on an unmeasured power stage.

## Sources

- Toshiba TB6612FNG datasheet, H-SW control modes, current and logic input margins:
  https://toshiba.semicon-storage.com/info/TB6612FNG_datasheet_en_20141001.pdf?did=10660&prodName=TB6612FNG
- Micromouse Online — drive system characterization and PWM/time response:
  https://micromouseonline.com/2011/05/15/characterising-the-drive-system-on-the-micromouse/
- Micromouse Online — motor controller and acceleration saturation:
  https://micromouseonline.com/2011/05/15/designing-the-motor-controller/
