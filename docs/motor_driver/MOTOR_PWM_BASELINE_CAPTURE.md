# Motor Driver baseline capture — Manual KEY, TIM2 PWM, N20 encoders

**Branch:** `feature/motor-driver-optimization`
**Firmware environment:** `motor_pwm_key_bench_96m_trim25`
**App mode:** `MOTOR_PWM_DIAG` (only when explicitly built)
**State:** implemented and awaiting actual STM32 motor-bench capture; NOT motor-current-qualified.

## Hardware facts, cannot be changed on this PCB

- STM32F411CEU6 3.3V GPIO drives TB6612FNG, TB6612 `VCC=5V`
  and `STBY` permanently wired to 5V. At VCC=5V, guaranteed VIH
  ≥3.5V; actual 3.3V GPIO HIGH is below this guaranteed threshold.
  Existing successful driving demonstrates operation on one board
  but does not eliminate the undefined noise/threshold margin.
- `VM` is a 2S pack (nominal 7.4V, up to 8.4V), motor N20 rated
  12V, gearbox speed and stall current unknown; no verified motor
  current sensor, regulator current limit or oscilloscope.
- One capacitor was reported at VM, capacitance/placement unknown.
- A voltage-only multimeter does **not** measure PWM frequency,
  transistor switching glitches, peak coil current, EMI or true
  instantaneous VM ripple.

**Safety limitation:** This firmware prevents automatic drive and limits
commanded *average PWM percentage and time*. It does **not** provide
hardware current limiting, external bridge shutdown, independent
watchdog or a guarantee against high peak/stall current. It is not a
proof of safety under all fault conditions. Be able to disconnect
physical motor power immediately. Never intentionally stall a wheel.

## What changed in this branch

Earlier `feature/motor-debug-console` created `motor_pwm_diag.c` and
`tools/motor_pwm_capture.py`, but its only pushed capture
`motor_logs/pwm_20261012_225510` was **INCOMPLETE**:
the raw log actually contains `[BOOT] Isolated MPU6050/I2C test
(motors disabled)` and no `MOTOR_*` samples. Also the old
`Control_SetDiagnosticOnly(1)` TIM11 branch returned before
`Motor_Update1ms()` and `MotorTest_Tick1ms()`.

The new source implements an opt-in `MOTOR_PWM_DIAG` branch inside the
TIM11 ISR which invokes:

```c
Encoder_Update(CONTROL_DT_S);
Motor_Update1ms();
MotorTest_Tick1ms();
```

This deliberately does not call MPU/I2C, IR, BLE, Motion or velocity PID.
The `main.c` build selector chooses the motor bench only when the
`MOTOR_PWM_DIAG` macro is defined. **RC2 MPU profile and default
build selection are unchanged.**

## Capture matrix: no automatic driving

At reset: TIM2 PWM starts with compare 0 and motor target 0. The motor
diagnostic has **five forward-only** stages, each operator-triggered:

| Stage | Left permille | Right permille | Max pulse |
| --- | ---: | ---: | ---: |
| 0 | +80 (8%) | 0 | 200ms |
| 1 | 0 | +80 (8%) | 200ms |
| 2 | +120 (12%) | 0 | 200ms |
| 3 | 0 | +120 (12%) | 200ms |
| 4 | +120 (12%) | +120 (12%) | 200ms |

Software deadband=0; rise=10 permille/ms; fall=20 permille/ms.
These **are diagnostic parameters only**, not calibrated motor
settings. PWM is nominal **30kHz** via TIM2; command service occurs
at TIM11 1kHz.

For EACH stage, follow the physical KEY sequence:

1. Press and **hold KEY for at least 1.5s**, then release; the log
   must say `MOTOR_ARMED`. The arming token expires after 10s.
2. **Press KEY again and KEEP IT HELD** for the short 200ms
   pulse. Release KEY to command an immediate stop (shorter than the
   200ms programmed pulse). To capture a *complete comparable stage*,
   hold through the `MOTOR_END` message.
3. Let go and repeat step 1 for the next stage. All five stages
   execute only once per boot, sequentially, without automatic
   restarts. Check `MOTOR_DONE` after stage 4.
4. If a TIM11 1.5ms gap is detected, `MOTOR_FAULT` latches the
   output at neutral and **only RESET can clear this state**.

No BLE/keyboard command can start the test. The old `#if 0`
automatic KEY 100% harness remains disabled.

## First bring-up: validate without motor VM power

1. Physically **disconnect motor VM**, leaving only MCU/UART supply.
   Keep battery and motor leads clear of shorts; perform every wiring
   change with power disconnected.
2. Verify MCU GPIO HIGH (~3.3V) and TB6612 VCC/STBY (~5V)
   using the voltage meter, if accessible. Do not attempt a hardware
   voltage-level modification on this fixed PCB.
3. Build and run host self-tests first:

```powershell
git fetch origin
git switch feature/motor-driver-optimization
python tools/motor_pwm_capture.py --self-test
python -m platformio run -e motor_pwm_key_bench_96m_trim25
python -m platformio run -e mpu_phase_tick_96m_trim25
```

4. Flash only the named bench image, then open serial capture:

```powershell
python -m platformio run -e motor_pwm_key_bench_96m_trim25 -t upload
python tools/motor_pwm_capture.py --port COM6 --seconds 120 --vm-state disconnected
```

Press STM32 RESET **once after COM6 opens**. Confirm:

```text
[BOOT] KEY-armed motor PWM bench; MOTOR OFF
MOTOR_PROFILE,version=2,nominal_hz=96000000,pwm_hz=30000,...,hard_cap_permille=120,...,motor_off_at_boot=1
MOTOR_WARNING,...
```

With VM still disconnected, optionally exercise the KEY sequence and
check `MOTOR_START`, `MOTOR_SAMPLE`, `MOTOR_END` records and host
report. Encoder movement will be zero, which is expected without VM.

## Only if prepared for a cautious motor-powered trial

Wheel off the ground, away from people and surfaces, with an immediate
physical way to remove VM. Do not use any of these measurements to
infer current safety. Stop if there is unexpected motion, driver heat,
burning smell, unstable output or changed behavior. The preferred
approach is a borrowed current-limited DC supply and ammeter; no
safe stall current limit can be derived from GPIO duty alone.

After safely preparing the elevated robot and performing all wiring
changes while power is off, open a **new capture** that explicitly
labels the motor supply as powered:

```powershell
python tools/motor_pwm_capture.py --port COM6 --seconds 120 --vm-state powered --meter-vm-volts 7.8
```

**Replace 7.8 with the actual DC VM voltage displayed on the meter.**
The voltage option is optional; omit it if no reliable reading is
available. It is never a current/peak-voltage measurement.

Arm each stage individually.
**Do not** manually increase duty or duration because an 8% pulse
does not overcome deadband: that is valid baseline evidence. Avoid
high-speed reverse, braking, blocked wheels, and any arbitrary 100%
experiment.

The capture script records each received UART frame with host time in
`motor_logs/pwm_YYYYMMDD_HHMMSS/raw_serial.log` and generates:

- `motor_samples.csv`: actual TIM2 CCR register reads
  (`ccr_l`, `ccr_r`), signed commanded/applied duty, encoder
  counts, TIM11 jitter/duration and UART drop counters.
- `session_meta.json`: operator-declared VM state and optional
  hand-measured DC VM voltage. The firmware does **not** know whether
  the power rail is physically connected.
- `motor_pwm_report.json`: five-stage startup/END completeness,
  ≤120-permille duty, correct phase order, bounded length, no
  TIM11 gaps, no missing output, no UART drops; each stage reports
  whether encoder counts changed.

**Important:** a CCR register read is **not a waveform measurement**.
`TRACE_COMPLETE_NOT_ELECTRICAL_APPROVAL` means the UART data passed
completeness gates; it is **not an electrical or thermal approval**.
An early KEY release produces `INCOMPLETE` intentionally, and
requires a separate new reboot/capture for a five-stage full test.
No hidden I2C, IMU or motion-control workload is exercised here.

To reprocess an existing capture offline without reflashing:

```powershell
python tools/motor_pwm_capture.py --folder "motor_logs/pwm_ACTUAL_SESSION"
```

After completion:

```powershell
git add motor_logs/
git commit -m "test(motor): capture five-stage KEY PWM/encoder baseline"
git push origin feature/motor-driver-optimization
```

## Follow-up analysis once logs exist

1. Evaluate if the firmware actually produced nonzero PWM and whether
   encoder counts changed for each wheel, along with sign.
2. Check max TIM11 ISR cycles and gaps, UART drops, whether left/right
   responses differ, and stage-to-stage repeatability. The encoder
   PPR constant is **not confirmed**, so do not quote RPM from an
   unknown calibration.
3. Scope/test the precise TB6612 stop/coast and preload race later when
   instrumentation is available. No claim that `Motor_Stop()` is
   verified Hi-Z is made by these logs.
4. Decide software improvements (output-state truth table, reversal
   interlock, command watchdog and stage-specific deadband) **one
   change at a time**, on this feature branch.
5. Run a separate motor+MPU RC2 mixed-load test before a motor release.

References:
- Toshiba TB6612FNG datasheet:
  https://toshiba.semicon-storage.com/info/TB6612FNG_datasheet_en_20141001.pdf?did=10660&prodName=TB6612FNG
- ST AN4013 PWM/timer details:
  https://www.st.com/resource/en/application_note/an4013-stm32-crossseries-timer-overview-stmicroelectronics.pdf
