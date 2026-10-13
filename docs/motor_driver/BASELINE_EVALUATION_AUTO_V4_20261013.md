# Motor Driver v4 — Powered 63-pulse integrated diagnostic review (2026-10-13)

**Source evidence:** `motor_logs/auto_20261013_101254/`, uploaded in `9656fb905d117ffcea8bb082ad21db12954ea881`.
**Firmware:** `motor_auto_pipeline_96m_trim25`, `MOTOR_AUTO_PROFILE version=4`, USART1 `rx_mode=irq256`.
**Conditions:** VM `powered` as declared by operator, DC voltmeter 8.2 V, elevated-wheel short PWM test. Motor N20 nominally rated 12V; TB6612 VCC 5V/STBY permanently high; STM32 GPIO ~3.3V. No motor current/temperature or oscilloscope data collected.
**Reported status:** `TRACE_COMPLETE_NOT_ELECTRICAL_APPROVAL`, `errors=[]`. This is **not** proof of electrical safety or full-speed/loaded performance.

## A. Pipeline / UART heartbeat fault resolution

| Integrity gate | Measured |
| --- | ---: |
| MOTOR_AUTO_START | 63 of 63 |
| MOTOR_AUTO_END | 63 of 63; all intended 200ms |
| MOTOR_AUTO_POST_END | 63 of 63; all 400ms |
| MOTOR_AUTO_SAMPLE | 1,890 (30 per stage: 10 drive + 20 neutral) |
| MOTOR_AUTO_LINK | 63 |
| Host MOTOR KEEP sent | 192 |
| MCU successfully decoded KEEP | 192 |
| UART RX ORE / FE / noise / ring overflows | 0 / 0 / 0 / 0 |
| UART bad decoded commands | 0 |
| TIM11 observed late >50us / gaps >1.5ms | 0 / 0 |
| UART TX dropped messages | 0 |
| Maximum observed TIM11 ISR DWT cycles | 1,625 (~16.93us @ nominal 96MHz), **not formal WCET** |
| Host age in stage-level MCU link records | ~159–180ms |
| Firmware fault | None |

**Interpretation:** v4's interrupt USART1 RX ring and more frequent host heartbeat **passed this powered bench session**. The previous v3 session `auto_20261013_095735` aborted with `HOST_TIMEOUT` after 2 starts; with v4, the 192 host-sent and 192 MCU-decoded KEEP messages match. This supports a successful fix in this run, though a single complete session does not prove fault-free operation under all system workloads. Unlike an older release, v4 excludes entirely unexecuted motor stages from count averages.

## B. Startup: per-wheel short-pulse 200ms, three trials

| Command duty | Left 3 measured encoder counts | Left sustained | Right 3 counts | Right sustained |
| --- | --- | --- | --- | --- |
| 8% | 0, 2, 78 | 1/3 | 1, 77, 77 | 2/3 |
| 9% | 1, 98, 94 | 2/3 | 75, 90, 81 | 3/3 |
| 10% | 107, 107, 108 | 3/3 | 92, 100, 100 | 3/3 |
| 11% | 123, 127, 120 | 3/3 | 112, 122, 111 | 3/3 |
| 12% | 139, 139, 139 | 3/3 | 127, 137, 136 | 3/3 |

The **lowest tested duty with 3/3 encoder-supported startup** is **10% left** and **9% right**, under these bench conditions. This is a *three-trial heuristic*, not a certified minimum startup threshold: outcomes at 8–9% vary sharply, and the robot was not under floor/wall load. Do not immediately hardcode 100/90 permille as production deadband compensation.

## C. Isolated PWM response, each trial 200ms

| PWM | Left mean encoder count | Right mean encoder count | Left-to-right delta relative to right | L mean post-neutral additional counts | R mean post-neutral additional counts |
| --- | ---: | ---: | ---: | ---: | ---: |
| 12% | 139.33 | 131.33 | +6.09% | +56.67 | +47.67 |
| 16% | 196.67 | 185.00 | +6.31% | +112.00 | +93.33 |
| 20% | 253.33 | 244.33 | +3.68% | +180.00 | +151.00 |

At 16% **simultaneous** duty, left and right were much closer: `L 196/193/190` (mean 193), `R 196/193/191` (mean 193.33). This is not a validated straight-line trajectory under load. Per-wheel differences could involve motor, encoder or mechanical properties; no correction factor should be transferred directly into the Motion controller.

## D. Response after PWM→0 (400ms observation)

PWM applied and TIM2 CCR were reported as 0 in every post-neutral sample. The encoders nevertheless continued accumulating counts and then flattened, with occasional 1–3 count bounce. In the isolated 20% trials, mean additional counts were:

- Left: **+180** over 400ms, ~71% of the 200ms powered count.
- Right: **+151** over 400ms, ~62% of the 200ms powered count.
- For the 20% isolated trials, each wheel's final 100ms of the neutral observation showed **zero net count change** (does not prove the motor's physical angular velocity is exactly zero or rule out count noise).
- Time when post-stop encoder count had reached approximately 95% of its *net* 400ms change: left ~180ms, right ~160ms (20ms sampling resolution). This is an empirical threshold for these pulses, **not** exact mechanical stop time.

**Important:** `Motor_Stop()` writes both direction pins LOW and TIM2 CCR=0, with PWM compare preload enabled. The register log demonstrates neutral software state, not independently measured bridge output current or whether the motor terminals actually enter the TB6612's Hi-Z coast vs short-brake state. Do not assume `Motor_Stop()` is physically braking the wheel, and do not introduce active electrical braking tests without current observation.

## E. Rise-slew sensitivity at a fixed 16% commanded duty

| Motor | Rise 5 permille/ms: mean counts | Rise 20 permille/ms: mean counts | Difference |
| --- | ---: | ---: | ---: |
| Left | 185.00 | 206.67 | +21.67 (+11.7% relative to slow) |
| Right | 174.00 | 193.00 | +19.00 (+10.9%) |

Both outcomes are based on three short pulses each. Faster programmed rise reaches the 16% target earlier during a fixed 200ms window, making the total powered PWM trajectory different. This **does not by itself establish greater steady-state motor speed**, superior control performance or acceptable acceleration of a loaded Micromouse.

## F. Review / next engineering gate

1. **Accept v4 UART/host pipeline as hardware-supported under this one elevated-wheel session:** no heartbeat loss or real-time deadline misses observed. Keep the 1600ms MCU heartbeat timeout and independent 200ms motor-test cutoff; do not bypass safety interlocks to improve completion rate.
2. **Do not mark Motor Driver fully optimized or release-ready.** The 63-pulse suite excludes load-bearing travel, high-duty operation, stall current, motor temperature, reverse-drive transition, commanded versus measured stop state, and motor+MPU/IR concurrency.
3. **P0 static review of driver STOP/BRK and CCR preload race**, including the TB6612 truth table, GPIO transitions, reverse gap and the lack of an MCU-controlled STBY pin. Separate firmware state correctness from actual electrical terminal measurements.
4. **P1 improve start-threshold robustness** with reordered, independent cold/rest-start trials at 8–12%, voltage metadata and a larger number of trials; the current lowest-tested 3/3 levels are a *candidate* range only. Keep motor-off or conservative freely spinning-wheel conditions.
5. **P1 quantify coast/deceleration** under the existing neutral-stop command; because the short bench already shows many encoder counts after stop, motion-layer distance planning must eventually account for braking/deceleration, but that planning is **outside Motor Driver**.
6. **P2 integration**: a separate motor-energized MPU/I2C RC2 and real-time timing/EMI benchmark after driver command-state review; pure MCU compile is not an integrated hardware qualification.

**No firmware behavior was changed by this review.** Sources:
- `../../motor_logs/auto_20261013_101254/motor_auto_report.json`
- `../../motor_logs/auto_20261013_101254/motor_auto_samples.csv`
- `../../motor_logs/auto_20261013_101254/raw_serial.log`
- `../../motor_logs/auto_20261013_101254/session_meta.json`.
