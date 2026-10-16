# Motor Driver Handoff Checklist — 2026-10-17

**Decision:** Freeze current **software release candidate**, not powered
robot release. Refer to
[MOTOR_DRIVER_RC_MANIFEST_20261017.md](MOTOR_DRIVER_RC_MANIFEST_20261017.md)
for source SHA / evidence provenance and
[MOTOR_DRIVER_API_CONTRACT.md](MOTOR_DRIVER_API_CONTRACT.md) for module
ownership.

## A. Signed-off for software handoff

- [x] Source baseline pinned: `b40228e85382e54f2487d8145fa3b0558802065b`.
- [x] Hardware read-only evidence preserved at commit `a5b67de451702f2ce92613affdf28bd6ea348eaa`.
- [x] CI `38944406079`: 48 steps passed; driver native, startup
      parser, negative safety cases, firmware compiles.
- [x] Windows host run: startup host tests 33/33 PASS.
- [x] Windows host read-only test: 11/11 PASS.
- [x] Physical STM32 high-speed v4 idle UART: 6 samples / 200 ms,
      TIM11 200 ticks, neutral registers, no TX/RX/late/gap errors,
      exactly one validated PING status. VM physically disconnected
      per operator confirmation; **no motor actuation performed**.
- [x] Motor command lease 300 ticks; native sticky-timeout regression.
- [x] STOP/preload/reverse/brake and clock/profile regressions PASS
      in native C/host; bench v4 requires explicit ACK.
- [x] Motion/PID source and hardware pin ownership left unchanged;
      contract documented, independent integration sign-off pending.
- [x] Failed historical captures preserved with original failure reason
      (v4 PROFILE overflow, host parser PING STATUS mismatch).
- [x] Last 2S speed variation recorded only as possible voltage factor;
      no inferred RPM/efficiency improvement.

## B. Mandatory open gates before claiming a powered safe release

- [ ] Confirm latest source and platform lockfile/SDK versions for
      production build (separate from bench image). CI build is not
      a robot acceptance test.
- [ ] Independent physical Motor Driver enable/disable and TB6612
      `STBY` / emergency cutoff strategy reviewed and tested.
- [ ] Verify physical low/high waveform of STOP/coast/preload/reverse
      and transient output states with appropriate instrumentation
      or a documented alternative external hardware qualification.
      Without available logic analyzer/oscilloscope, mark **NOT MEASURED**;
      do not fabricate equivalent certainty from register mocks.
- [ ] Measure or otherwise independently establish acceptable VM
      range, peak/running current, motor/bridge thermal envelope,
      battery supply sag, wire ratings and driver protection.
- [ ] Test the command timeout latch on **physical MCU** under a
      dedicated VM-disconnected diagnostic (not this startup PING check).
- [ ] Verify Motion/PID/Motor command freshness end-to-end on the
      intended real firmware/robot, including sensor stale stop,
      MCU reset, host communication loss and main-loop stall handling.
- [ ] Loaded and dynamic driving/braking/reverse stress tests only
      after electrical safety gates; separate explicit authorization
      and current-limited setup required.
- [ ] Confirm BLE USART2 and IR/IMU runtime interoperability
      independently; earlier motor startup bench is not equivalent.
- [ ] Review and approve production merge/release only after unresolved
      safety gates have a named owner, actual measurements and evidence.

## C. No-power verification commands (Windows PowerShell)

```powershell
git fetch origin
git switch feature/motor-driver-optimization
git pull --ff-only origin feature/motor-driver-optimization
python tools/motor_driver_native_test.py
python tools/motor_startup_pipeline.py --self-test
python tools/motor_startup_readonly_check.py --self-test
python tools/motor_stop_transition_regression.py
python -m platformio run -e genericSTM32F411CE -e motor_speed_96m_trim25
```

**Physical inspection-only replay** (only after physically verifying
TB6612 VM has been disconnected and kept disconnected):

```powershell
python tools/motor_startup_readonly_check.py --run --port COM6 --confirm-vm-disconnected
```

This command does **not** flash, ARM or pulse motors. It only sends a
`STARTUP PING`. Do not rerun if the physical VM isolation is uncertain.
If a test FAILs, preserve `raw_uart.log`, `commands.log`,
`report.json`; do not convert FAIL to PASS in documentation.

## D. Acceptance boundary

Software API is frozen for handoff at the pinned baseline;
later modifications, even only to `Motor_Init`, UART diagnostic
or Control's TIM11 path, invalidate the matching release code
baseline and require re-review. Physical MCU idle check is accepted.
The module remains **NOT QUALIFIED FOR POWERED/LOADED OPERATION**.
