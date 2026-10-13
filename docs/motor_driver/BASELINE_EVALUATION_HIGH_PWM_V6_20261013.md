# High-PWM Motor Driver v6 — 87/87 elevated-wheel test review (2026-10-13)

**Source**: `motor_logs/auto_20261013_105353/` (raw UART, `motor_auto_samples.csv`, `motor_auto_report.json`, `session_meta.json`).
**Power**: TB6612 VM manually reported 8.2 V; not measured continuously by MCU. Motor N20 12V nominal. PCB TB6612 VCC 5 V, STM32 GPIO nominal 3.3 V, STBY fixed high. **No current, winding/driver temperature, VM ripple, real scope PWM waveform, measured shaft RPM, load or traction recorded.**
**Scope**: 3 rounds of 29 stages = 87 active pulses (63 base + 24 opt-in 30–60% single-wheel high), PWM 30 kHz firmware timer register configuration, base 200ms/400ms neutral, high 120ms/800ms neutral. All counts are *encoder tick totals*, not RPM, angular degrees, meters or stopping distance.

## 1. Experimental validity

| Test condition | Result |
| --- | ---: |
| STATUS | `TRACE_COMPLETE_NOT_ELECTRICAL_APPROVAL`, errors=[] |
| START/END/POST_END | 87/87/87; no missing stages |
| Samples | 2,994 = 63×30 base + 24×46 high |
| UART host KEEP sent vs STM32 decoded | 301/301 |
| RX overrun / framing / RX ring overflows | 0/0/0 |
| UART TX drops | 0 |
| TIM11 late / >1.5ms gaps | 0/0 |
| Highest observed TIM11 ISR execution cycles | 1654 cycles = ~17.23us at 96 MHz nominal (not formal worst-case) |
| Faults | 0 |
| High stages TIM2 CCR peak at 30/40/50/60% | 960/1280/1600/1920 of nominal 3200 |

**Verdict:** v6 bench command, recording and host watchdog worked for all 87 stages on the elevated-wheel setup. No overload or electrical-limit qualification is implied.

## 2. High-duty PWM response, three 120ms pulses each

| Duty | L encoder counts (3 rounds) | R encoder counts (3 rounds) | L mean / R mean | L vs R difference relative R |
| --- | --- | --- | --- | ---: |
| 30% | 187,182,189 | 175,175,174 | 186.0 / 174.67 | +6.49% |
| 40% | 236,233,231 | 230,225,227 | 233.33 / 227.33 | +2.64% |
| 50% | 274,276,278 | 265,270,265 | 276.0 / 266.67 | +3.50% |
| 60% | 310,306,308 | 298,299,304 | 308.0 / 300.33 | +2.55% |

All these stages reported sustained encoder counts in 3/3 runs and have small within-condition count variance. Net ticks in a **120ms PWM startup** window increase monotonically with duty, but they are not steady-state motor-speed transfer curves. At 60% with 10 permille/ms slew, the target duty is first reached after ~60ms, leaving only ~60ms at full commanded duty before stop.

## 3. Massive encoder response during the 800ms *software-neutral* window

| Duty | L additional counts (mean) | R additional counts (mean) | L/R 95% response time after zero (mean) | L/R 99% response time | Final 100ms net count |
| --- | ---: | ---: | --- | --- | --- |
| 30% | +381.33 | +321.00 | 260/220ms | 300/260ms | 0 in all 3 trials for both |
| 40% | +643.67 | +558.00 | 340/300ms | 380–400/340ms | 0 in all 3 trials for both |
| 50% | +947.33 | +827.00 | 400/340–360ms | 460/400ms | 0 in all 3 trials for both |
| 60% | **+1265.00** | **+1113.33** | **440/400ms** | **520/460ms** | 0 in all 3 trials for both |

Direct inspection of 20ms CSV samples shows all commanded PWM CCR and applied duty return to zero in the post-neutral phase. At 60% L, approximately +431/+766/+1007/+1166/+1249 net counts appear at 100/200/300/400/500ms post-stop, with count nearly constant at 600–800ms. At 60% R the analogous measurements are +420/+734/+947/+1071/+1115; nearly constant by 500–800ms.

**Critical distinction:** time to reach 95% or 99% of the net 800ms position change is an **encoder-defined position response milestone**, not certified zero velocity, electrical short-braking, motor stop time, stopping distance of a loaded mouse, or coasting mode. A net-zero final 100ms does not exclude ±1 count oscillation or encoder noise. The stage's MCU `HIGH_NOT_SETTLED` gate checked only a near-end count difference; passing it does NOT mean immediately stopping on PWM=0. Mechanical inertia and electrical output state remain entangled.

At 60%, position increments during neutral are roughly 4.1× (L) and 3.7× (R) those recorded in the preceding 120ms powered window, illustrating why `Motor_Stop()` cannot be treated as a physical instant stop.

## 4. Startup threshold *nonmonotonicity* — do not hardcode 8%

| PWM | L counts (three runs) | L sustained trials | R counts | R sustained trials |
| --- | --- | --- | --- | --- |
| 8% | 70,75,82 | 3/3 | 1,1,2 | 0/3 |
| 9% | 1,1,2 | 0/3 | 71,75,81 | 3/3 |
| 10% | 110,108,109 | 3/3 | 91,98,107 | 3/3 |
| 11% | 131,122,127 | 3/3 | 108,120,115 | 3/3 |
| 12% | 138,139,138 | 3/3 | 124,128,127 | 3/3 |

The existing v6 analyzer printed `lowest_tested_3of3_permille=80` for L, despite L **failing at a HIGHER duty of 90** in every repeated trial. That "lowest" result is a mathematical minimum but **an invalid calibrated startup-deadband inference**. Potential test-order/rotor-position/friction or hardware encoder factors cannot be distinguished with this evidence. A more conservative *observed monotonic suffix candidate* is **L 10%, R 9%**, and neither is yet a robust under-load threshold. Previous v4 session exhibited different behavior at 8%; don't transfer fixed deadband parameters to Motion from these 3-test bench runs.

**Host parser corrected:** `tools/motor_auto_pipeline.py` now outputs both the naive minimum and a `lowest_monotonic_suffix_3of3_permille`, plus a list of decreases in success rate as duty increases. A self-test models 8%-success/9%-failure/10%-success. No firmware PWM output code changed. The already captured session can be reanalyzed offline with `--folder motor_logs/auto_20261013_105353`, without re-flashing, to refresh JSON/summary locally.

## 5. Motor Driver engineering decisions (not Motion)

- **Evidence-backed pass**: UART v6 profile/heartbeat, bounded short pulses, fixed TIM2 duty command traces through 60%, 1kHz TIM11 service under motor-only workload, three-trial consistency for high PWM, MCU high-step 800ms wait/settle gate for this run.
- **Open technical correctness**: exact TB6612 truth-table output mode of `Motor_Stop()` with current pin and preload order; reproduce signed encoder/noise glitches, direction interlock and reverse-safe transitions in a separately qualified setup, GPIO 3.3V against 5V VIH margin, and current/thermal protection. Do **not** automatically short-brake/reverse/force full PWM.
- **Hardware limits**: Toshiba TB6612FNG current limits (1.2A average / 3.2A peak absolute maximum as listed on Toshiba's product page) and actual 12V N20 motor currents must be reconciled; the present 8.2V VM and successful 60% short free-spin tests alone cannot qualify current safety, thermal headroom, battery loaded-voltage transient or PCB traces.
- **System qualification**: energize motor while exercising MPU6050/I2C and infrared/scheduler, after safe electrical boundaries are established, then evaluate EMI/power stability. Motion-layer path trajectory, cornering and closed-loop wheel-speed gains remain out-of-scope here.
- **High-speed work cannot jump directly to 80/100%**: 60% is the present measured envelope for **120ms off-load individual motors** only. First measure power-stage current, waveform, heat, loaded voltage and deceleration; protect with physical VM disconnect. A software encoder guard cannot prevent an instantaneous stall-current event.

**DO NOT schedule a new powered repetition of the exact 87 pulse suite; its main questions already have three consistent measurements.** Prioritize analysis, static state-transition audit and instrumentation gates.

Source: `motor_logs/auto_20261013_105353/motor_auto_report.json`, `motor_auto_samples.csv`, `raw_serial.log`, `session_meta.json`. Driver datasheet: https://toshiba.semicon-storage.com/info/datasheet_en_20141001.pdf?did=10660; manufacturer product facts: https://toshiba.semicon-storage.com/ap-en/semiconductor/product/motor-driver-ics/brushed-dc-motor-driver-ics/detail.TB6612FNG.html
