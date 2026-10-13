# Motor Driver Research & Optimization Plan — STM32F411 / TB6612FNG / N20

**Research date:** 2026-10-13
**Branch:** `feature/motor-driver-optimization`
**Parent baseline:** `release/mpu6050-i2c-rc2-tim11-20261013`
**Status:** source audit + literature review only; no motor implementation, hardware test, PCB validation or benchmarking was performed for this new branch.

## Scope boundary

**Motor Driver owns:** TIM2 hardware PWM, H-bridge direction pins, legal output states, brake/coast, duty limiting, output slew/ramp policy, reversal interlock, deterministic disable/enable, watchdog and electrical fault handling, telemetry reflecting *actual CCR/GPIO*.

**Does NOT own:** velocity PI/PID, trajectory tracking, route planning, wheel odometry, encoder estimation algorithms, yaw fusion. Motor Driver can consume a bounded safety permission from a supervisor, but should not quietly subsume Motion.

## Git/source audit (original source, unchanged)

- Local branch SHA is identical to parent release at time of review: `387a532811736ed60b0de4cf60fb34794a5b70d9`.
- `src/motor.c`: TIM2 PWM outputs PA15 CH1 right, PB3 CH2 left; direction GPIO PB12–PB15, 30kHz by `MOTOR_PWM_FREQUENCY` (`include/pinout.h`), 1kHz `Motor_Update1ms`.
- API `Motor_SetPairPermille(-1000..1000)`; nominal rise 20 permille/ms, fall 30 permille/ms; reversal gap 30ms; configurable per-wheel deadband; stop and brake.
- TIM2 CCR preloaded (`OC1PE`, `OC2PE`): duty commits on next timer update event, while direction BSRR is immediate. Measure this boundary and ensure no unsafe polarity+stale PWM transient. Do not assume GPIO and CCR take effect simultaneously.
- `Motor_Stop()` writes both direction pins LOW and CCR=0. **Toshiba TB6612FNG truth table only explicitly labels `IN1=IN2=L, PWM=H, STBY=H` as Hi-Z Stop; `PWM=L` is not this documented case.** Verify output scope/logic and revise sequence only after IC pinout and STBY wiring are confirmed.
- `Motor_Brake()` sets both direction pins HIGH and CCR=ARR+1 (forced 100% PWM), consistent with TB6612 short brake `H,H,H/L` if STBY HIGH; brake current still unknown.
- `pinout.h` header says motor A (right) IN1=PB14, IN2=PB15, but macro definitions swap them (IN1=PB15, IN2=PB14); physical polarity must be checked.
- `include/motor.h`, `src/motor.c` comments and `tools/MOTOR_DEBUG.md` mention 20kHz PWM; active `pinout.h` is 30kHz. `MOTOR_PWM_PERIOD 4999` is a stale constant; implementation computes ARR from clock. At 96MHz TIM2 clock and PSC=0, 30kHz => ARR=3199, ~3200 counter steps; command interface 1000 permille steps is the coarser quantization.
- `MotorTest_Heartbeat` only protects Motor Test (1500ms); ordinary actuator commands lack a proven independent command-age timeout.
- `MotorTest_BrakePulse` tests 50ms BRAKE only at limited command/speed; it does **not** sense brake current.
- The historical button-triggered +100% test inside `src/main.c` is presently inside `#if 0`; not active firmware. Current active `main.c` instead runs isolated motor-OFF IMU diagnostics, so a NEW opt-in motor-specific bench profile is needed.
- `MotorTrace` stores 512 recent 1kHz samples (0.512s), including requested/applied duty and encoder counts; it does not measure physical PWM waveform, VM, motor current, voltage or driver temperature.
- `src/board.c` HardFault code disables TIM2 CC outputs and clears direction GPIO. This is an MCU reaction, not guaranteed external H-bridge cutoff without measured STBY/pulldowns.
- Two-phase LiPo supply is stated as 7.4V nominal / 8.4V full directly to VM; STM32 board supply 5V. **TB6612 logic VCC and STBY wiring are not yet verified.** If TB6612 VCC=5V, 3.3V STM32 outputs do NOT meet guaranteed VIH ≥0.7×VCC=3.5V on Toshiba datasheet. Verify regulator and pin voltage, not just battery.
- Motor N20 actual rated voltage, winding/gear ratio, no-load/stall current and safe current are unknown. `ENCODER_PPR=1430` is a firmware constant, not validated wheel counts/revolution.

## Gate P0 — electrical truth and safety (must pass BEFORE spinning motors)

- [ ] Obtain PCB schematic and board photos, identify TB6612 VCC, STBY (tied HIGH vs MCU), VM decoupling and bulk caps, MCU GPIO level, GND layout, any current shunt and battery overvoltage protection.
- [ ] Obtain N20 model/label: rated voltage, nominal/no-load/stall current, gear ratio. Do not use arbitrary 100% PWM on unknown 6V N20 with 8.4V VM.
- [ ] Verify PWM frequency and exact TIM2_CH1/CH2 mapping using oscilloscope/logic analyzer at 10/50/90% command **with motors disconnected or outputs safely disabled**.
- [ ] Validate TB6612 truth-table for zero/drive/brake/standby at the actual PCB pins; explicitly determine effect of current `IN1=IN2=LOW, PWM=LOW` before naming it coast.
- [ ] Scope GPIO/CCR effective transitions during zero-crossing, stop, startup and release from brake; no drive glitch, reverse torque transient or unintended pulse.
- [ ] Document safe commissioning power source: current-limited lab PSU and elevated wheels. If current is not measurable, do NOT claim driver current safety.
- [ ] Define safe fail state when startup fails, calibration fails, TIM11 stalls, firmware HardFault occurs, BLE disconnects, or command source freezes.

## Gate P1 — low-level driver correctness

- [ ] Clean and centralize `MOTOR_PWM_FREQUENCY`; remove contradictory hardcoded period, comments and stale 20kHz claims. Do not raise PWM frequency before measured thermal/EMI tradeoff.
- [ ] Explicitly configure TIM2 PSC, ARR, CCR, CCER, CCMR, CR1, preload and pin idle states; preserve other timer channels only if deliberately used.
- [ ] Treat coast, brake, standby and drive as separate explicit states with verified truth table; implement safe transition ordering around preload boundaries.
- [ ] Verify both left and right forward polarity and encoder sign, then correct mismatch between physical mapping and `pinout.h`.
- [ ] Keep command scaling and `applied` software output distinct from measured CCR; add readback of effective timer compare register (preload phase behavior must be accounted for).
- [ ] Add independently enforceable output enable and command timeout strategy. Test absence of a new command, ISR stop and failures without relying on MotorTest's own heartbeat. If STBY is tied HIGH physically, software cannot guarantee hard hardware disable.
- [ ] Keep 30ms reversal gap as initial constraint; only shorten after shaft-speed evidence. A safety supervisor may veto reversal using encoder speed; do not embed full velocity control in driver.
- [ ] Define bound and reset semantics for rise/fall rate and deadband. Calibrate per wheel/direction, not one guessed number.

## Gate P2 — quantitative driver optimization

- [ ] Motor power source: scope/measured VM at driver pins during start, hold, coast, brake and both-wheel load; check supply pumping/ringing and MPU/I2C interference.
- [ ] Run open-loop isolated left/right/both 1–3s tests at **hardware-safe measured duty**; record duty command vs actual CCR vs encoder count/velocity, bench current and component temperature.
- [ ] Map static starting duty, forward/reverse dead-zone, low-duty jitter, asymmetry and min sustainable duty across at least two safe VM values (if appropriate to motor rating).
- [ ] Controlled frequency comparison only **after** verifying motor current/thermal: compare ~20, 30, 40 kHz (all below TB6612 100kHz max) at equivalent motor/battery condition; measure current ripple/EMI/driver heating and effective motion. 30kHz baseline; do not assume a winner.
- [ ] Confirm ISR worst-observed duration and jitter for TIM11 with motor update and MPU RC2 data path active; no UART in hard-RT ISR.
- [ ] Evaluate slew/reversal settling and brake-vs-coast mechanically, comparing wheel angular speed and current/VM transient; **do not use repeated high-speed short-brake tests without current instrumentation**.

## Suggested trial sequence & acceptance artifacts

| Stage | Experiment (keep isolated) | Expected evidence |
| --- | --- | --- |
| H0 | Hardware/schematic truth | Pin map, driver mode/state truth table, N20 + voltage/current envelope |
| S0 | Source static test, motor physically unpowered | Peripheral registers, PWM period, coercion/clamping, polarity, brake/stop logical states |
| S1 | Dedicated opt-in motor-off firmware | Compiled target, boot attestation, fail-safe state, no auto 100% tests |
| H1 | Scope PWM + logic, no moving motor | freq, phase, duty actual, startup/stop/reversal waveforms |
| H2 | 1-wheel low-current, lifted axle | current, VM sag, encoder sign, static duty threshold, thermal |
| H3 | 2-wheel low-current + safe brake/reversal | state-machine transitions, no direction glitch, timing |
| H4 | Combined system EMI/ISR test with RC2 | zero unacceptable IMU/I2C faults and deadline misses; EMI measured |
| R1 | Freeze Motor Driver revision | new release branch, source SHA, manifest, raw traces, regression report |

**Never assert a numerical safe-duty ceiling or current limit before
motor rating and source/current instrumentation are known.** A low PWM
duty alone is not a guaranteed stall-current limit.

## Relevant research, engineering documentation, and field experience

### Primary manufacturer material (first read)

1. Toshiba TB6612FNG datasheet, modes (p.4), ratings (p.3), logic VIH (p.5):
   https://toshiba.semicon-storage.com/info/TB6612FNG_datasheet_en_20141001.pdf?did=10660&prodName=TB6612FNG
   Pay particular attention to `L,L,H -> Hi-Z Stop`,
   `H,H,H/L -> Short brake`, VCC 2.7–5.5V and fPWM ≤100kHz.
2. ST, RM0383 STM32F411xC/E reference manual (TIM2/TIM11):
   https://www.st.com/en/microcontrollers-microprocessors/stm32f411/documentation.html
3. ST, AN4013 Introduction to timers for STM32 MCUs, preload/shadow and synchronization:
   https://www.st.com/resource/en/application_note/an4013-stm32-crossseries-timer-overview-stmicroelectronics.pdf
4. TI, SLVA321A Current Recirculation and Decay Modes:
   https://www.ti.com/lit/an/slva321a/slva321a.pdf
   Exposes how decay path affects current ripple and mechanical brake behavior.
5. TI, SLVA504A Calculating Power Dissipation for a H-Bridge or Half Bridge Driver:
   https://www.ti.com/lit/an/slva504a/slva504a.pdf
   Electrical/mechanical losses depend on current, on resistance,
   PWM frequency, decay and board thermal behavior.
6. TI, SLVA959B Best Practices for Board Layout of Motor Drivers:
   https://www.ti.com/lit/an/slva959b/slva959b.pdf
   Bypass/bulk capacitor and switch current-return layout for EMI.
7. maxon, PWM power stage: current ripple & motor chokes:
   https://support.maxongroup.com/hc/en-us/articles/360005046213-PWM-power-stage-Current-ripple-Motor-chokes
   Higher PWM frequency lowers current ripple but may raise switching
   loss; motor inductance and topology matter.

### Peer-reviewed research (methodological relevance, not drop-in controller)

8. OpenMCT — open-source DC motor control educational kit (2026):
   https://pmc.ncbi.nlm.nih.gov/articles/PMC13224385/
   Demonstrates static PWM dead-zone sweeps and system-identification
   experiments. **Only the characterization procedure belongs in the
   Motor Driver stage; feedback controller design belongs to Motion/Speed.**
9. PL-TOON — A Low-Cost Experimental Platform for Teaching and Research
   on Decentralized Cooperative Control (2021):
   https://pmc.ncbi.nlm.nih.gov/articles/PMC7998519/
   Motor dead-zone self-calibration illustrates why a one-size-fits-all
   deadband is fragile; applies as an idea, not a safe value for N20.
10. Sensorless Speed Control of Brushed DC Motor Based on New Current
    Ripple Component Signal Processing, Energies 2021, 14(17), 5359:
    https://www.mdpi.com/1996-1073/14/17/5359
    Current ripple can carry commutation information, but our robot has
    encoders **and no current sensor**; not a near-term implementation.
11. Designing a Brushed DC Motor Driver with a Novel Adaptive Learning
    Algorithm, Electronics 2024, 13(22), 4344:
    https://www.mdpi.com/2079-9292/13/22/4344
    Useful general low-inductance and switching tradeoffs, but the paper's
    cascaded converter differs substantially from TB6612FNG + LiPo.

### Specialist/field sources

12. Peter Harrison et al., Micromouse Online, PWM frequency and power
    consumption (2010):
    https://micromouseonline.com/2010/09/05/pwm-frequency-and-power-consumption/
    Practical caution: APB timer clock multiplier can produce unexpected
    PWM frequency and H-bridge heating; **measure the waveform**.
13. TI E2E, PWM off-time brake vs coast, DRV8876 example:
    https://e2e.ti.com/support/motor-drivers-group/motor-drivers/f/motor-drivers-forum/979142/drv8876-pwm-mode-operation
    General physics only: TB6612FNG truth table is device-specific.
14. Pololu example N20 family, 30:1 micro metal gearmotor HP 6V:
    https://www.pololu.com/product/1093
    A **different** 6V N20 version has an extrapolated stall current of
    1.6A at 6V; illustrates why no N20 current/speed/rating can be
    inferred from 12mm gearbox shape alone.

## Input needed from owner to set safe testing and exact priorities

1. Schematic/close-up PCB of TB6612: STBY, VCC, VM, capacitors,
   AO1/AO2/BO1/BO2, soldered configuration. STBY and VCC wiring are
   most urgent, particularly because MCU outputs ~3.3V.
2. Two N20 nameplates/product URLs or measured motor rating: nominal V,
   rated/stall I, gearbox ratio, motor winding resistance; if unknown,
   state unknown rather than guessing.
3. Tools available: oscilloscope/logic analyzer, current-limited
   bench PSU, series ammeter or shunt clamp, IR thermometer. No built-in
   current sensing has been verified.
4. If available: recent motor test logs, actual direction polarity
   per wheel, target wheel RPM and high-speed operating interval.

**No driver firmware was changed as part of this source audit.**
