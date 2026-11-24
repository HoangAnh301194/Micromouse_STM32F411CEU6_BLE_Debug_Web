# Unified runtime — Phase 1
Base: f71c7d20248167d0ca1d75cb15585747f3ff1f8a.

## Build
`pio run` builds the sole operating environment micromouse_96m.
`pio run -t upload` uploads it. Previous isolated diagnostic profiles remain in platformio_v5_diagnostics.ini (explicit -c option); they are not part of the operating build.

## Runtime scope
One Main_Runtime and TIM11 scheduler, existing Mode 9 menu, passive IR/encoder/MPU streams, and guarded 165 mm OneBox. KEY and SPP select; G explicitly starts OneBox. H every 100 ms, P stops. APP,STATE reports runtime state. OneBox is the only motion owner in this phase; the legacy compile guards are retained internally to exclude all other motor writers. No other motion runner is enabled. Other menu entries remain NOT IMPLEMENTED (main modes 1–6 are selection only). This is an incremental migration, not complete maze-running firmware.

Boot locked; passive tests locked; OneBox only unlocks after readiness checks and explicit Start. Normal finish brakes 100 ms then locks before reporting RESULT; severe faults lock immediately and require reset. Existing PWM/count/lease/timeout limits remain. OLED remains optional and off pending hardware validation. Flash layout and clocks unchanged.

## Verification required
Static source review is not a firmware build or hardware validation. Run pio run, inspect flash/RAM, run native regression tests, and verify SPP handshake, passive streams and CSV sessions. With VM cutoff accessible, lift wheels before OneBox Start; verify both directions, Stop, PA0, heartbeat loss and locked=1 after completion. Do not run on floor until bench tests pass. Next phases: independent motion runners, general motor ownership, PID, calibration/map and navigation.
