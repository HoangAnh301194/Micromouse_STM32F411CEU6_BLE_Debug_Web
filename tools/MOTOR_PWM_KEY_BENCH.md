# Opt-in motor PWM bench — electrical characterization first

PlatformIO environment: `motor_pwm_key_bench_96m_trim25`.

**This is NOT a validated motor-current-limited controller and is NOT PID.**
It is an optional isolated, low-duty, physically armed pulse test to collect
real PWM/encoder/timer data, with conservative duty and time limits.

## Hardware facts and critical unknowns

- The user's 2S pack supplies TB6612 `VM` at approximately 7.4–8.4V.
- The user's STM32 board is supplied with 5V via its board power input,
  but its GPIO outputs are nominal 3.3V.
- **TB6612 `VCC` logic supply remains unverified.** Datasheet high threshold:
  `VIH(min)=0.7*VCC`. If VCC=5V, nominal 3.3V MCU outputs do NOT meet
  guaranteed 3.5V HIGH threshold; if TB6612 VCC=3.3V, nominal 3.3V HIGH
  is compatible. Verify actual VCC, all common GNDs, and STBY wiring first.
- **N20 motor rated voltage, coil resistance, rated/stall current unknown.**
  Direct 2S operation may be unsafe if N20 variant is a 3V or 6V motor.
- TB6612 max operating 1A/ch in datasheet conditions; absolute maximum
  1.2A average / 3.2A 10ms single pulse must not be a design target.
- Without current measurement, neither duty limit nor thermal shutdown
  guarantees safety against a jammed rotor or large startup current.
- Full software hardening (IWDG, validated external STBY interlock, voltage/
  current feedback) is NOT done. Physical power disconnect is mandatory.

## How to measure without an onboard current sensor

For the first powered experiment, prefer an external DC bench supply with a
current-limit dial, using a voltage suited to the actual N20 motor rating
(confirmed from part number / datasheet). Set a current limit that also
protects the PCB and TB6612; do not assume 1.2A/ch is a safe target. Measure
`VM` directly between driver VM and GND while pulsing. A digital multimeter
set to DC amperes can be connected IN SERIES with the motor VM supply to
measure its average draw, but not individual PWM peak/coil current, and
must never be connected across VM/GND in current-measuring mode. For transient
current, use an appropriate shunt and safely grounded/differential scope.
Never deliberately hold/stall the motor at full battery voltage.

Power down before inserting current probes or rearranging wiring. Check meter
fuse/range and battery polarity.

## Firmware architecture

This opt-in profile keeps the ordinary motor-OFF IMU diagnostic untouched.
No MPU, BLE, IR or velocity PID. `TIM11` 1kHz ISR performs encoder update,
`Motor_Update1ms` and `MotorTest_Tick1ms`. UART logs are outside ISR.
`TIM2` PWM is nominal 30kHz; motor starts at neutral duty=0.

Five manually requested, forward-only pulses, **200ms each**, in permille:
- Left +80, right 0
- Left 0, right +80
- Left +120, right 0
- Left 0, right +120
- Left +120, right +120

Each pulse requires the operator to **hold KEY >=1.5s and release to ARM**,
then **press KEY again and keep it down** to request the pulse. Releasing
KEY immediately calls MotorTest_Stop in the main loop. The 1kHz motor test
stops the pulse after 200ms even if held. Each test requires a new arm
sequence. The sequence cannot restart without resetting the MCU.

The motor driver is configured at deadband=0, rise=10 permille/ms,
fall=20 permille/ms; these are test parameters, **not tuned values**. PWM
duty in permille is NOT a safe motor voltage/current estimate.

## Run only after electrical prerequisites confirmed

First build without connecting motor power:
```powershell
git fetch origin
git merge --ff-only origin/feature/motor-debug-console
python -m platformio run -e motor_pwm_key_bench_96m_trim25
```

If the build succeeds, if TB6612 VCC/GPIO level compatibility and motor-rated
voltage/current have been checked, and a current-limited motor supply and
physical disconnect are available, the test may be considered for a lifted,
unloaded robot:

```powershell
python -m platformio run -e motor_pwm_key_bench_96m_trim25 -t upload
python tools/motor_pwm_capture.py --port COM6 --seconds 90
```

After the COM port opens, press the STM32 RESET once and follow the printed
KEY instructions; **do not press KEY until physically prepared**. Stop
the test / disconnect motor power immediately for unexpected movement,
heat, smoke, excessive supply current or voltage sag. Do not run from the
unrestricted 2S battery on a first unknown-rating motor test.

The capture tool produces `motor_logs/pwm_.../raw_serial.log`,
`motor_samples.csv` and `motor_pwm_report.json`.
`TRACE_COMPLETE_NOT_ELECTRICAL_APPROVAL` means log completeness only; it is
NOT a declaration of safe current, voltage, driver thermal margin, or
closed-loop speed control. A short/noisy encoder count is NOT a reason to
increase PWM without electrical verification.

**Do not run `diagnostic_pipeline.py` with this profile**; that pipeline
expects IMU and CLK records that are intentionally absent from this bench.

## Next engineering steps

Use actual measured motor voltage/current and unloaded PWM-speed curves to
define bounded PWM and acceptable startup/stall conditions. Then implement
and characterize encoder-based velocity PID, feedforward, anti-windup,
acceleration profiles, motor fault/stall handling and closed-loop control.
Do not treat the existing zero-setpoint computational PID benchmark as motor
tuning. No measured load-step performance exists yet.
