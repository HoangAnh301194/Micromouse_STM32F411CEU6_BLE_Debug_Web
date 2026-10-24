# Motor Driver Revision 3 — selective integration into `giomuadongbac`

## Provenance and scope

- Source branch: `feature/motor-driver-optimization`
- Pinned source commit: `4ed2816f8c9b2dbc0f4adbebbe88dcd1b5dde41c`
- Frozen Motor Driver code baseline: `b40228e85382e54f2487d8145fa3b0558802065b`
- Target baseline before merge: `524e6205d822a4653a2eaec5e3fd4ff6b5758e47`
- Original Motor Driver CI: `38949415872`, 52/52 PASS.
- Original physical startup v4 idle-only report: `a5b67de451702f2ce92613affdf28bd6ea348eaa` (motor supply physically disconnected by operator).
- Transfer mechanism: **selective source transplant plus integration compatibility glue**, NOT full Git merge of 482 unrelated commits.

## Exactly what is integrated

**Unmodified donor blobs**:
`src/motor.c`, `include/motor.h`, `src/motor_test.c`,
`include/motor_test.h`, `src/motor_startup.c`,
`include/motor_startup.h`, plus host native regression harness.
The Motor Driver is revision 3. Timed/experimental MotorTest/Startup
state machines are included as library code, but the separate experimental
high-speed UART bench application, its sensor load benchmarks and its
many logs are **not** copied to the production branch.

**Target-local minimal glue**:
- `src/control.c`: adds one `Motor_Update1ms()` in its original TIM11 1kHz
  interrupt path, after `Encoder_Update` and before MPU/IR processing.
  No unconditional `Motor_KeepAlive()` is permitted. No Motion/PID gains
  or IMU/IR algorithms are replaced.
- `include/pinout.h`: 30kHz TIM2 PWM; removes obsolete fixed
  `MOTOR_PWM_PERIOD` / `MOTOR_PWM_PRESCALER`; Timer2 ARR is derived from
  actual APB1 timer clock by `Motor_Init`.
- `src/board.c`: HardFault best-effort neutralization disables
  TIM2 compare outputs and drives motor direction GPIO low. No changes to
  clocking, UART, I2C, IMU or other peripheral setup.
- `src/motion.c`, `include/motion.h`, `src/main.c`, `platformio.ini`
  remain **unchanged** on the destination branch.

## Driver/API contract

- `Motor_SetPair(left,right)` remains compatible with existing Motion,
  signed percent `[-100,100]`, each converted to signed permille.
- `Motor_SetPairPermille(left,right)` is the new native low-level API.
  Higher levels do not directly touch `TIM2` or direction GPIO.
- `Motor_Update1ms()` must execute exactly once per TIM11 1ms
  while `Control_Start()` is active, to service slew and 300ms command
  freshness. Motion's active update must issue real fresh commands.
- Software-neutral STOP uses IN1=IN2=LOW and CCR=ARR+1 (PWM high in
  the settled TB6612 truth-table state). Software cannot prove physical
  terminal Hi-Z and transient current.
- Reverse neutral gap 30ms; rise/fall slew defaults 20/30 permille/ms.
  `Motor_LockOutputs()` remains sticky after command expiry.
- The 300ms lease cannot detect a fully stalled TIM11 or MCU hang.
  The existing `Control_Stop()` calls `Motion_Stop()` before disabling TIM11.

## Quality gates

```powershell
python tools/motor_giomuadongbac_gate.py --self-test
python tools/motor_giomuadongbac_gate.py --audit
python tests/regression/motor_driver_native_test.py
python -m platformio run -e genericSTM32F411CE
```

The GitHub Actions workflow `motor-driver-giomuadongbac.yml` runs the
source/bridge negative tests under Python 3.11 and 3.14, the compiled
native C state machine regression and a **build only** of the original
`genericSTM32F411CE` firmware.

## Acceptance distinctions / no physical claims

- These software gates require **no repeated motor speed/encoder counts**.
  Supply variations can alter RPM while leaving the PWM driver logic correct.
- The original donor MCU read-only capture demonstrates neutral/clock/UART
  for its **separate 96MHz v4 bench**; it does not validate the destination's
  original 100MHz runtime clock, MPU/IR scheduler or physical 300ms watchdog.
- Hardware current, PWM switching, brake/stop transients, loaded driving
  and independent emergency power cutoff remain `NOT_MEASURED`.
- **Do not flash or drive motors solely because CI passes.** Commission
  the integrated branch under an independent motor power/safety plan.
- The original donor `motor_release_gate.py` pins 18 files outside the
  scoped integration; it is intentionally NOT copied because those unrelated
  sensor/timer files remain original to `giomuadongbac`.
