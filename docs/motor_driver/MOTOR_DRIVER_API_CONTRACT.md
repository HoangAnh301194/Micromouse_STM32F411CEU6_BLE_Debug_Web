# Motor Driver API & Peripheral Ownership Contract — RC 2026-10-17

Code baseline: `b40228e85382e54f2487d8145fa3b0558802065b`.
Normative code: `include/motor.h`, `src/motor.c`, `include/motor_test.h`,
`src/motor_test.c`, `src/control.c`, `include/pinout.h`.

## Boundary (Driver versus Motion/PID)

```text
Motion / PID / higher-level state machine
             |
        signed PWM command
  Motor_SetPair() / Motor_SetPairPermille()
             |
Motor Driver: clamp -> ramp -> neutral/reverse guard
             |             command lease
TIM11 @1kHz: Motor_Update1ms()
             |
      TIM2 CH1/CH2 + GPIOB direction
             |
           TB6612FNG
```

- Driver owns **PWM generation, direction, command ramping, deadband,
  reversal guard, STOP/BRAKE, command freshness and sticky output lock**.
- Motion/PID owns **velocity/pose goals, feedback decisions, motor duty
  calculation, odometry use, trajectory and control gains**.
- Driver must not fetch IR/MPU/encoder data to calculate motion commands
  (except explicit standalone MotorTest/Startup safety wrappers).
- Higher layers must never directly write the Driver's TIM2 or motor
  direction GPIO registers. Hardware ownership does **not** imply Driver
  ownership of encoder TIM3/TIM4 or TIM11 scheduling.

## Hardware ownership

| Device | Owner / role |
| --- | --- |
| TIM2_CH1, PA15 | Right motor PWM |
| TIM2_CH2, PB3 | Left motor PWM |
| PB15 (R IN1), PB14 (R IN2) | Right direction |
| PB13 (L IN1), PB12 (L IN2) | Left direction |
| TIM11 | Shared 1 kHz scheduler in `control.c`, not exclusively Driver-owned |
| TIM3 / TIM4 | Encoder subsystem, not Driver-owned |
| USART1 PA9/PA10 | Motor bench/debug serial, distinct from USART2 BLE |
| TB6612 STBY / independent cutoff | No Driver guarantee for external hard shutdown |

## Public `motor.h` contract

| Function | Preconditions / effects |
| --- | --- |
| `Motor_Init()` | Configures motor pins and TIM2. Starts in software-neutral STOP; **must not clear an existing sticky outputs lock** |
| `Motor_SetPairPermille(left,right)` | Signed permille, individually clamped to [-1000,+1000]. Writes target, resets command age if not locked. Applied duty is ramped by TIM11, not necessarily immediate |
| `Motor_SetPair(l,r)` | Legacy signed integer percent; scales by 10 and clamps to permille limits |
| `Motor_Update1ms()` | Caller services **once per actual TIM11 1 ms tick**. Expires stale nonzero request/applied command after 300 ticks; locks outputs + STOP. Avoid calls from unrelated main-loop cadence |
| `Motor_Stop()` | Immediate software command-neutral. IN1/IN2 LOW; PWM CCR set to ARR+1 (active HIGH). Ramping canceled, lock preserved |
| `Motor_Brake()` | Asserts brake input command if not locked; **current/thermal consequence unmeasured**. Not for uncontrolled emergency stop |
| `Motor_StopWithMode(mode)` | Dispatches COAST or BRAKE; current limitations remain |
| `Motor_LockOutputs()` | Sticky software-neutral output lock. Subsequent SetPair ignored. No public `Motor_UnlockOutputs()` |
| `Motor_OutputsLocked()` | Returns sticky lock; do not treat unlocked as electrical safety certification |
| `Motor_GetDiagnostics(&snap)` | IRQ-protected register+internal-state snapshot. `command_timeout_latched`, `command_age_ms`, applied/target, lock and CCR are diagnostic fields |
| `Motor_GetAppliedPairPermille(l,r)` | Actual *software ramp state*, not measured terminal voltage or speed |
| `Motor_SetDeadband(l,r)` | Clamps each to [0,500]; zero default; any change to physical minimum power requires measured calibration |
| `Motor_SetSlewRates(rise,fall)` | Each clamped to [1,1000] permille/tick, zero request becomes 1; default rise 20/fall 30 |
| `Motor_KeepAlive()` | Reserved for bounded `MotorTest_Tick1ms()` only. Normal Motion must renew via fresh `Motor_SetPair...` commands; a standalone tick calling KeepAlive can mask dead control logic |

**Units and outputs**:
- `+1000` = nominal 100% command in selected direction; `-1000`
  reversed; this is **not** a request for measured current or wheel speed.
- `0` targets software STOP. On the 96 MHz benchmark `ARR=3199`,
  neutral `CCR=3200`; other clock profiles compute a different ARR.
- Direction `IN1/IN2=LOW/LOW` and PWM HIGH is the intended settled
  TB6612 stop truth-table combination. Transient electrical behavior
  during timer preload has **not** been observed with a scope.
- Driver defaults to a 30 ms neutral gap on direction reversal;
  first drive after STOP has a one-TIM11-tick preload guard.
- `Motor_Update1ms` command watchdog requires TIM11 to run.
  A halted MCU/ISR is **not** detected by this software timer.
- **Sticky lock recovery** is intentionally not exposed by API:
  diagnosis/reboot with verified neutral conditions before resuming;
  never work around it by direct pin writes.

## Explicit test interfaces, not production control

`motor_test.h`:
- `MotorTest_StartPermille` requires nonzero signed commands within
  [-1000,+1000], log interval 100..1000 ms and IDLE state.
- `MotorTest_Tick1ms` enforces configured duration, 1500 ms heartbeat,
  and renews Driver's shorter command lease only while a valid test is
  running.
- `MotorTest_BrakePulse` is gated by software speed/duty checks;
  no assumption about measured brake current or safe use under load.
- `MotorTest_AcknowledgeEnd` resets bench calibration defaults.
- This wrapper is only for isolated diagnostics. Do not substitute
  bench APIs for real Motion/PID control.

`motor_startup.h` high-speed bench:
- `MotorStartup_ApproveExperimental()` is compiled only for
  `MOTOR_STARTUP_HIGH_SPEED` and requires disarmed, quiet, unlocked state.
- `MotorStartup_Arm` additionally requires experimental approval.
- `MotorStartup_Pulse` is bounded by sequence, time, wheel selection,
  duty and host heartbeat. Firmware v4 `qualified=0`.
- It must never be used as evidence of electrical/loaded approval.

## Integration responsibilities

1. Call `Motor_Init`, initialize sensor/timer subsystems and start
   Control in the intended order. Do not energize until a separate
   hardware and physical safety review passes.
2. Main control loop issues fresh `Motor_SetPair...` commands for
   active Motion states; `Motion_Update` already calls
   `Motor_SetPair` in current source. Avoid bypassing the lock.
3. Preserve call ordering in `control.c` and do not call motor APIs
   from competing interrupt/main contexts without reviewing ownership.
4. `Control_Stop()` currently calls `MotorTest_Stop()` and
   `Motion_Stop()` before TIM11 stops; preserve this neutralization.
5. Do not edit `TIM2->CCR1/CCR2`, PWM modes, prescaler, DIR GPIO
   `PB12..PB15`, or repurpose PA15/PB3 from non-Driver modules.
6. Maintain timeout, stop/reverse and UART protocol regression tests
   when modifying the public contract.
7. Treat physical `PWM/Hi-Z` states and thermal/current safety as
   separate hardware qualification tasks.

See [release manifest](MOTOR_DRIVER_RC_MANIFEST_20261017.md) for the
verified evidence and [handoff checklist](MOTOR_DRIVER_HANDOFF_CHECKLIST_20261017.md).
