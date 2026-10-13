# Motor Driver v2 Baseline Review — 2026-10-13

**Session:** `motor_logs/auto_20261013_093808/`
**Evidence commit:** `d7e50bfc2d437e1c9c689fd91ab16297f58514a1` (`update`).
**Baseline firmware:** `motor_auto_pipeline_96m_trim25`, serial HOST START, 3 rounds × 9 stages, 200ms PWM followed by 300ms output-neutral encoder observation.
**Supply:** operator-declared TB6612 VM powered, DC meter 8.2V; N20 nominal motor rating 12V. No current sensor, current limit measurements, oscilloscope or thermal data. STM32 3.3V GPIO drives TB6612 VCC=5V, STBY tied 5V, outside datasheet guaranteed VIH margin.

## Trace integrity

- Host report: `TRACE_COMPLETE_NOT_ELECTRICAL_APPROVAL`, `errors=[]`.
- `MOTOR_AUTO_START`: **27**; `MOTOR_AUTO_END`: **27**, 200ms each, reason=0; `MOTOR_AUTO_POST_END`: **27**, 300ms each; **675** telemetry samples.
- TIM11 `tim11_late`=0 and `tim11_gaps`=0 for all 675 samples; UART `tx_drop`=0. Largest observed TIM11 DWT count **1623 cycles**, ~**16.91us at nominal 96MHz**; **not formal WCET**.
- TIM2 CCR measured in peripheral registers: 8%=256, 12%=384, 16%=512, 20%=640 (nominal 3200 period). All records in post-neutral interval show CCR=0 and applied duty=0.
- USB-UART RX to STM32 was functional in this session, because the host serial START/KEEP pipeline completed all stages.
- Last remote commit changed **only five experimental log files**, no new firmware changes. No independent proof of PWM-pin electrical waveform, motor current or bridge temperature.

## Each condition, three 200ms encoder counts

| PWM | Motor | Round 1 | Round 2 | Round 3 | Mean | Sample standard deviation | Sustained heuristic |
| --- | --- | ---: | ---: | ---: | ---: | ---: | --- |
| 8% | L | 79 | 1 | 1 | 27.00 | 45.03 | 1/3 |
| 8% | R | 69 | 1 | 1 | 23.67 | 39.26 | 1/3 |
| 12% | L | 142 | 138 | 141 | 140.33 | 2.08 | 3/3 |
| 12% | R | 132 | 132 | 130 | 131.33 | 1.15 | 3/3 |
| 16% | L | 199 | 199 | 204 | 200.67 | 2.89 | 3/3 |
| 16% | R | 186 | 186 | 190 | 187.33 | 2.31 | 3/3 |
| 20% | L | 256 | 255 | 256 | 255.67 | 0.58 | 3/3 |
| 20% | R | 242 | 242 | 242 | 242.00 | 0.00 | 3/3 |
| 16% both | L | 196 | 196 | 195 | 195.67 | 0.58 | 3/3 |
| 16% both | R | 196 | 196 | 197 | 196.33 | 0.58 | 3/3 |

**Source-grounded interpretation:**
1. 8% fails sustained encoder movement on repeat rounds for **both** motors. Previous KEY test showed only the right motor low-end weak; a single prior sample cannot establish a permanent right-only deadband.
2. 12% is the **lowest tested duty with sustained encoder counts in all three trials**, not the certified minimum startup duty under load. True dead-zone boundary lies somewhere between 8% and 12% under current conditions, but cannot be pinpointed because 8% sometimes succeeds. Avoid hardcoding a calibrated deadband or compensating for only the right wheel yet.
3. When each motor is driven alone, left count > right count at 12/16/20% (about 6.9%, 7.1%, 5.6% using right count as denominator). This is **open-loop directional asymmetry** over 200ms, not PID tracking error. Potential causes include motor parameters, friction, driver voltage, encoder scaling, actual load.
4. When both wheels drive 16% simultaneously, mean outputs 195.67 L / 196.33 R, nearly equal; don't generalize this to sustained loaded straight-line tracking based on a single stage.

## After-stop motion, 300ms output-neutral window

The firmware returns TIM2 CCR to zero at each 200ms cutoff; encoder counts continue increasing, then roughly plateau, sometimes fluctuating by 1–3 counts. These measurements establish **post-command motion/encoder activity**, not independently verified physical high-impedance coast or active short-brake state.

| Motor + PWM alone | Additional encoder counts in post 300ms (3 rounds) | Mean post/drive ratio |
| --- | --- | ---: |
| L 8% | +17, -1, -1 | Not meaningful at 8% because two drive windows had only one count |
| R 8% | +15, -1, -1 | Not meaningful at 8% |
| L 12% | +54, +57, +58 | ~40% |
| R 12% | +45, +46, +48 | ~35% |
| L 16% | +112, +112, +109 | ~55% |
| R 16% | +94, +94, +89 | ~49% |
| L 20% | +181, +184, +185 | ~72% |
| R 20% | +154, +154, +155 | ~64% |
| Both 16%, L | +95, +95, +101 | ~50% |
| Both 16%, R | +99, +99, +97 | ~50% |

At 20% alone, output-neutral position continues to change for a material interval; in round 1, left advances by 30 encoder counts in the first 20ms after cutoff and by 181 net counts over the 300ms period. At 20% right, counts rise ~29 over the first 20ms and 154 net over 300ms. **Avoid inferring the wheel physically stops at the instant of Motor_Stop()**. Likewise, measuring CCR=0 does not establish TB6612 coast vs brake truth-table state. Some count near plateau bounces ±1–3; encoder jitter or mechanical recoil remain unverified.

## Risk/quality status

| Subsystem or claim | Result |
| --- | --- |
| ISR/PWM/encoder command path at 12–20% | Baseline pass under elevated-wheel bench conditions |
| Host-controlled 3-round 27-pulse capture and interlock | Baseline pass, no errors during this one run |
| Repeated reliable 8% startup | **Not demonstrated**, sustained only 1/3 for both motors |
| Left/right per-wheel 12–20% equal open-loop response | **Not demonstrated**, single-wheel asymmetry 5.6–7.1% |
| Both-wheel 16% short benchmark near equality | Observed, 195.7 vs 196.3 counts |
| Immediate physical stop at PWM=0 | **Not demonstrated**, substantial post-zero encoder travel |
| True hi-Z coast / short brake state | Not measured |
| Safe full-speed, stall current, under-load motion | **Not evaluated** |
| MPU/I2C performance while motors energized | **Not evaluated**, this app excludes MPU and IR |

## Recommended next stage (do not fold into Velocity PID / Motion)

1. **STOP output-state correctness before optimizing high-speed driver:** static code review of `Motor_Stop`, TB6612 IN1/IN2/PWM truth table, CCR preload and stop-transition ordering. Current VCC=5V, STBY fixed HIGH must be treated as immutable. Output-neutral motion is quantified, but coast/brake mode needs electrical evidence. Do **not** immediately introduce high-current short-brake pulses without current measurement.
2. **Startup repeatability calibration:** safe short, elevated-wheel randomized/reordered duty set covering 8/9/10/11/12%, at least three independent repeats with same measured DC VM and low electrical stress; ensure 8% anomalous first-round effect is not sequence-dependent. A host-verified 3-round count criterion is useful; a true calibrated deadband needs load/voltage mapping.
3. **STOP dynamics and encoder quality:** record speed/count for at least 300ms after zero, including initial speed and approximate settle time; exclude 1–3 count quantization/recoil oscillations and distinguish post-motor torque from inertia. Consider neutral-current/noise tests with VM disconnected to bound false encoder counts.
4. **Command timeout and inhibit logic:** profile-isolated watchdog and fault injection **without VM** first; ensure no stale PWM remains on host loss/reset/TIM11 issue. Note STBY cannot electrically disable bridge.
5. **Repetition protocol:** the 3 rounds in one session do **not** prove repeatability across separate power cycles, battery voltages, temperatures or surface loads. Keep running on the fixed PCB.
6. **Integration:** only after driver correctness, test motor energized vs RC2 MPU/I2C freshness and RT scheduler telemetry, to detect shared power/EMI interactions.

**Do not mark Motor Driver release-ready yet.** Source: `motor_logs/auto_20261013_093808/raw_serial.log`, `motor_auto_samples.csv`, `motor_auto_report.json`, `session_meta.json`.
