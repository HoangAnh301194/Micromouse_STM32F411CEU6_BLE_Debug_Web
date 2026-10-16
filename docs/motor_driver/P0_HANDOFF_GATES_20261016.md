# P0 Motor Driver handoff gates — 2026-10-16

Scope: isolated Motor Driver and motor benches; Motion/PID, encoder algorithm, IR and MPU remain unchanged.

## P0-1: 300 ms actuator command lease

- `Motor_SetPairPermille()`/`Motor_SetPair()` refresh a 300-TIM11-tick command lease.
- If a requested or applied command is nonzero and no valid refresh arrives, `Motor_Update1ms()` records a timeout, latches `Motor_LockOutputs()` and commands software-neutral STOP. `Motor_Init()` does not clear an output lock.
- `Motor_KeepAlive()` is reserved for the isolated MotorTest state machine, which retains its explicit duration/1500ms heartbeat timeout. Normal Motion should renew its lease through actual repeated SetPair commands, not by a timer that can hide a stalled controller.
- This watchdog operates **only while TIM11 executes**. It is not a fail-safe against CPU hangs, faulty H-bridge silicon, physical shorts, or loss of MCU clock. STBY is not an independently controlled hardware cutoff.
- Motor diagnostics expose `command_age_ms` and `command_timeout_latched`. No automatic software re-arm is permitted after expiry.
- 300ms is a provisional software safety bound, not a measured physical stopping distance or a complete robot safety case. The integration owner must validate tighter latency if required.

## P0-2: STOP/preload guard and unmeasured electrical states

- Settled STOP command remains IN1=IN2=LOW, TIM2 PWM HIGH (CCR=ARR+1), aligned to Toshiba TB6612FNG's **input truth table**.
- Regression checks interruption of the first-tick PWM preload with STOP, same-direction restart, preloaded-CCR direction guard, reverse neutral gap and command-age expiry on the actual driver with register mocks.
- Physical LOW/LOW/PWM-LOW transient, external Hi-Z, brake current, switching waveform, overshoot, temperature and loaded stop distance remain `NOT_MEASURED` without instrumentation; software test PASS must never be used as electrical certification.

## P0-3: experimental high-speed bench is not qualification

- `motor_speed_96m_trim25` uses **high-speed UART protocol v4** with `experimental=1,qualified=0` (archived v3 raw captures remain unchanged).
- Host rejects high-speed `--run` unless the operator explicitly supplies `--ack-experimental-unqualified` in addition to previously required raised-wheel and VM metadata.
- MCU independently refuses `MotorStartup_Arm()` without prior `MotorStartup_ApproveExperimental()`; the bench command `STARTUP ACK EXPERIMENT` produces a strictly checked `STARTUP_EXPERIMENT_ACK,accepted=1`. This occurs only after read-only watchdog reset verification and sensor readiness.
- Operator acknowledgement is deliberate selection of an unqualified, elevated-wheel research test; it is **not** evidence that high-current/thermal/waveform safety checks passed.
- Reports record `qualification_status=NOT_QUALIFIED_ELECTRICAL_OR_LOADED`. A `MEASURED_HIGH_SPEED` result means measured encoder response under the specified bounded pulse conditions, NOT loaded/full-power qualification.
- Older `motor_auto` P0 locks remain in place. Do not change `MOTOR_AUTO_P0_ARM_LOCKED` or classify host CI as an electrical PASS.

## Software-only checks (no upload or motor movement)

```powershell
python tools/motor_driver_native_test.py
python tools/motor_startup_pipeline.py --self-test
python tools/motor_stop_transition_regression.py
python tools/motor_driver_debug_pipeline.py --self-test
python -m platformio run -e motor_startup_96m_trim25 -e motor_speed_96m_trim25
python -m platformio run -e genericSTM32F411CE
```

No 100% PWM test is needed merely for code freeze. Bench v4 has not been physically validated by this software change; explicitly keep power, thermal and transition verification open. Operator reported a slight 2S battery voltage decrease during the last speed measurement, which is plausible as a contributing cause, but it is not a synchronized VM reading.

### Handoff criteria

1. CI, native driver state-machine and host parser tests PASS; default/bench compile PASS.
2. Normal command timeout is independently tested; no stale re-arm after timeout. STOP/preload and experimental ACK negative tests pass.
3. Historical physical logs retain original version and stated test constraints.
4. Electrical/loaded/thermal evidence and independent hard power cutoff are explicitly NOT_MEASURED/NOT_QUALIFIED, not silently waived.
5. Recheck integration on the physical robot with suitable safe power constraints before claiming continuous high-speed motion.

## Inspection-only physical MCU gate (no motor VM)

A separate opt-in check reads an already-flashed high-speed firmware v4 **without** arming motor outputs. The host transmits only `STARTUP PING`; it does not flash, send experimental ACK, ARM, KEEP, PULSE, STOP or WATCHDOG. It fails closed on unknown firmware, active state, non-neutral CCR/direction, encoder movement, unexpected reset, TIM11 timestamp divergence or UART errors.

1. **Physically disconnect TB6612 VM motor supply** before powering the MCU/UART. Keeping the USB ST-Link and logic 3.3V powered does **not** imply VM is disconnected; verify wiring. Do not connect or reconnect VM during these checks.
2. Build the experimental firmware and, **only with VM still disconnected**, intentionally upload the new v4 image via ST-Link:

```powershell
python -m platformio run -e motor_speed_96m_trim25
python -m platformio run -e motor_speed_96m_trim25 -t upload
```

3. Keep VM disconnected and run the inspection-only host:

```powershell
python tools/motor_startup_readonly_check.py --self-test
python tools/motor_startup_readonly_check.py --run --port COM6 --confirm-vm-disconnected
```

The report is saved to a new `motor_logs/startup_readonly_*/report.json` plus UART command/raw captures. Expected status: `PASS_INSPECTION_ONLY_IDLE_V4`. This proves only the protocol and commanded neutral registers, not physical TB6612 output states, motor current, waveform quality or safe running. A report of FAIL/unknown firmware must **not** be bypassed by issuing ARM manually.

Testing a normal-runtime command timeout **on a live MCU** additionally requires a purpose-built motor-off diagnostic; this UART profile cannot prove that particular runtime fault path. Until such a test exists, keep that gate labeled `PASS_NATIVE_ONLY`.


## 2026-10-17 UART PROFILE overflow fixed after first real v4 MCU inspection

The first real read-only run `startup_readonly_20261017_005104_056967` failed safely:
`STARTUP_STATUS state=4,fault=7,tx=2,rx=0,late=0,gaps=0,cl=3200,cr=3200,odr=0`.

**Confirmed source root cause:** `STARTUP_PROFILE` grew from v3 to v4 by adding
`experimental=1,qualified=0`. The fully formatted line is about 261 bytes.
`UART_Printf()` used a 256-byte temporary buffer and intentionally dropped
an overlong record. Two PROFILE attempts (boot + PING) produced exactly the
two TX drops in the MCU log and latched `MOTOR_STARTUP_UART`.

**Fix:** Startup high-speed diagnostic uses a 320-byte `UART_Printf` staging
buffer. Ordinary firmware and other profiles keep 256 bytes. The 1024-byte
USART1 TX DMA ring, DMA semantics and **drop=>FAULT** safety policy are not
changed. Regression calculates the worst-case formatted line and ensures it
fits including its NUL terminator. No evidence of an electrically unsafe PWM
state in the captured UART register snapshot; physical bridge outputs remain
unmeasured.

**Verification on MCU:** disconnect TB6612 VM physically; do not connect it
during the check. Update the branch, rebuild and upload
`motor_speed_96m_trim25`, then rerun
`python tools/motor_startup_readonly_check.py --run --port COM6 --confirm-vm-disconnected`.
The previously flashed binary necessarily still has the defect until reflashed.
Do not reinterpret the failed historical capture as a PASS.

## 2026-10-17 second physical UART run: safe firmware, strict host checker mismatch

Evidence: `motor_logs/startup_readonly_20261017_005904_227767`, committed as
`3aa45a575c7b`. UART shows a complete firmware v4 PROFILE and a
`STARTUP_STATUS,id=0,state=0,fault=0,al=0,ar=0,cl=3200,cr=3200,odr=0,late=0,gaps=0,tx=0,rx=0`.
The earlier UART `tx=2` formatting defect is fixed.

The inspection-only checker rejected `STARTUP_STATUS` as an unknown
frame, despite the firmware's `Command()` contract deliberately replying
to `STARTUP PING` with `Profile()` **and** `Status("STARTUP_STATUS")`.
This was a host-parser false FAIL, not a new firmware fault. The PC checker
now requires and validates exactly one neutral `STARTUP_STATUS`, plus at
least 5 fresh, monotonic neutral `STARTUP_SAMPLE` records; missing,
duplicate, armed or faulted STATUS fails closed. Firmware, UART DMA,
watchdog, motion driver, PWM and safety interlocks were NOT modified.

The captured run ended after the first STATUS due to the old checker; it
is NOT full duration/tick attestation. The owner should keep VM physically
disconnected and re-run **only** the updated Python read-only checker on
COM6 (no MCU rebuild/reflash):

```powershell
git fetch origin
git pull --ff-only origin feature/motor-driver-optimization
python tools/motor_startup_readonly_check.py --self-test
python tools/motor_startup_readonly_check.py --run --port COM6 --confirm-vm-disconnected
```

## 2026-10-17 final read-only MCU PASS and software RC handoff

The subsequent physical MCU inspection passed, archived in
`motor_logs/startup_readonly_20261017_010627_618666` at
commit `a5b67de451702f2ce92613affdf28bd6ea348eaa`.
Firmware v4; one neutral `STARTUP_STATUS`, 6 idle samples/200 ms;
TIM11 ticks 444800..445000; `late=gaps=tx=rx=0`;
`state=fault=0`; `al=ar=odr=0`; `cl=cr=3200`.
The operator confirmed physically disconnected motor VM.
The final PC read-only self-test passed 11/11. This validates
physical MCU **idle UART/clock/command registers only**, not powered
output safety, the physical 300ms lease, or electrical transients.

Handoff artifacts:
[MOTOR_DRIVER_RC_MANIFEST_20261017.md](MOTOR_DRIVER_RC_MANIFEST_20261017.md),
[MOTOR_DRIVER_API_CONTRACT.md](MOTOR_DRIVER_API_CONTRACT.md),
[MOTOR_DRIVER_HANDOFF_CHECKLIST_20261017.md](MOTOR_DRIVER_HANDOFF_CHECKLIST_20261017.md).
