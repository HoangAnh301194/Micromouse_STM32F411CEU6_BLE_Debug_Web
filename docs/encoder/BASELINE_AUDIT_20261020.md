# Encoder baseline audit — 2026-10-20

**Branch:** `giomuadongbac`  
**Read-only audit base:** `b37336b3325431673e40bcc818ff710859d2156b`  
**Scope:** TIM3/TIM4 quadrature encoder acquisition, velocity estimation, calibration protocol. No motor test or firmware flash is authorized or performed by this audit.

## Handover state

- `PROJECT_STATUS.md` is authoritative for module acceptance. Prior real-board motor-on tests demonstrated encoder pulses for both wheels and a positive forward sign. This does **not** establish wheel counts/revolution, ground-truth travel, missed edges or wheel-speed control qualification.
- TIM11 calls `Encoder_Update(CONTROL_DT_S)` at 1 kHz. The prior IR motor-off capture and UART/MPU integration passes do not qualify encoder movement under physical load.
- The last encoder-relevant motor-on result in `results/system_uart_20261019_104529/summary.json` has software acceptance but `physically_qualified=false`. These historical motor-on measurements are NOT an authorization to run another motor test.

## Code mapping

| Element | Current implementation |
| --- | --- |
| Right wheel | `TIM3` / PB4(A), PB5(B), AF2 |
| Left wheel | `TIM4` / PB6(A), PB7(B), AF2 |
| Timer encoder mode | `SMCR.SMS=3`: quadrature x4 (both TI inputs) |
| Counter width | 16 bits (`ARR=0xFFFF`) |
| Signed accumulator | `total_left` / `total_right`, 32-bit signed |
| Direction convention | `ENCODER_LEFT_DIRECTION=-1`, `ENCODER_RIGHT_DIRECTION=-1` |
| Speed estimate | `delta_count / dt_s`, then first-order LPF with alpha=0.30 each 1-ms tick |
| Constants used in motion | `ENCODER_PPR=1430`, `WHEEL_DIAMETER_MM=35`, `WHEEL_BASE_MM=84` in `include/pinout.h` |
| Existing diagnostics | `SYS_E` at about 1 Hz: `encL`, `encR`, `ppsL`, `ppsR`, `appL`, `appR` |

Sources: `src/encoder.c`, `include/encoder.h`, `src/control.c`, `src/motion.c`, `src/main.c`, `include/pinout.h`, `platformio.ini`.

## Static audit: noteworthy limits, not verified defects

1. **Wheel conversion is uncalibrated.** `1430` is used as *decoded timer counts per wheel revolution*, not independently validated. Timer SMS=3 performs x4 decoding; do not apply an additional x4 unless physical measurement justifies it. One revolution is assumed to travel `pi * 35 = 109.956 mm`; nominal `mm/count = 0.076892`. These are configuration-derived estimates, not metrology results.
2. **1-ms velocity quantization.** At 1 kHz, a one-count difference means 1,000 counts/s, equivalent to ~76.9 mm/s if the currently unverified conversion is used. LPF alpha 0.30 smooths these steps but does not add precision. For wheel-speed PID, benchmark a longer estimator window (e.g. 10–20 ms) or edge-timing estimation, and measure delay/noise before selecting a controller rate.
3. **16-bit timer rollover.** `Encoder_Update` reads `CNT`, casts to signed 16-bit and differences successive snapshots. The intended wrap-safe signed delta assumes the true change magnitude remains below 32768 timer counts per sample and typical two's-complement conversion behavior. Add host/native rollover tests before claiming qualification.
4. **Input filtering and electrical robustness.** `Encoder_TimerInit` configures AF pull-ups and TI1/TI2 inputs without explicit digital filter bits. This is not proof of noise, but glitches, contact/wiring noise and EMI under motor-on/load are not qualified.
5. **No wheel-speed PI loop yet.** The current `motion.c` issues PWM feedforward/ramp and heading/turn PID; encoder counts support travel estimation, but no dedicated closed-loop left/right wheel-speed PID is shown. Do not call this an accepted speed controller.

## Safe Phase E0 — hand-turn encoder reception (NO motor power)

Prerequisites:
- Use the already-installed `integration_diag_motor_off_96m` firmware **only if verified**. Do not upload a production/motor-on firmware to perform this test.
- Keep motor VM physically disconnected; independently verify encoder VCC/GND remain powered (some boards may share the motor supply).
- No commands `F/L/R`, `START`, `MODE ON`, `--motor-on`, or automatic PWM sweep. This test needs only slow manual wheel rotation.
- Use a 3.3-V TTL USB-UART adapter, MCU PA9 TX to adapter RX, common ground, read-only serial at 115200 baud. Do not open the same port from multiple applications.

Procedure:
1. Read at least two `SYS_E` frames while both wheels are stationary; ensure `appL=appR=0`, count stationary jitter, and note initial `encL/encR`.
2. Mark one wheel and turn **only that wheel** forwards exactly 5 full output-shaft/wheel revolutions manually, slowly enough not to miss UART snapshots. Record final `encL`, `encR`; repeat once.
3. Repeat the opposite wheel, then reverse both individually. The sign should invert; nonmoving wheel should stay near its baseline.
4. Effective wheel counts/rev = `abs(end_count - start_count) / 5`. Repeat with 10 revolutions if counts are borderline or uncertain.
5. Save the unchanged raw UART log, date/time, firmware SHA/profile, motor power state, wheel mark and revolution count; never overwrite historical `results/` files.

**Pass gates (proposed, not measured):** both channels respond under hand turn, signs agree with defined forward direction, repeat counts/revolution close within a measured tolerance, idle counts remain stable, and the nonrotating wheel stays quiet. Select tolerances only after observing actual data and repeatability.

The existing `SYS_E` once-per-second telemetry is useful for calibration totals and basic signs; it is **not** sufficient for a reliable 1-kHz missed-pulse/noise claim.

## Later phases

- **E1:** static/native tests for count direction, rollover, reset, speed filtering and edge loss; introduce counters/diagnostics only when justified.
- **E2:** wheel circumference and counts/revolution by an external reference, and speed measurement repeatability under safely authorized motor-on conditions.
- **E3:** design and qualify separate wheel-speed PI/PID loops after E0–E2 evidence, with sample period, anti-windup, stop/lease safety and timed response metrics.
- **E4:** integrate straight-distance, yaw-hold, stop and 90/180-degree turn verification under the separate physical-motion safety plan.

## Editing and acceptance discipline

Do not modify the fixed `include/pinout.h`, frozen Motor Driver Rev3 or MPU/I2C implementation without a specific verified reason. Do not classify build success as physical encoder qualification. Before editing from any local clone, inspect branch/HEAD/`git status --short` and preserve pending local work. GitHub-only audit cannot reveal local uncommitted changes.
