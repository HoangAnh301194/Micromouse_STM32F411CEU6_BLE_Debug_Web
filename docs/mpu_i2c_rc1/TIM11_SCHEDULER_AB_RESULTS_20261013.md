# TIM11 scheduler A/B results — 13 Oct 2026

**Decision:** `IMPROVED_REQUIRES_REPLICATION`. Keep motor off. Do **not**
merge experimental tick scheduler into production yet.

Source captures (both from this branch, as pushed by operator):
- Legacy A: `mpu_phase_ab_logs/session_20261013_005503/raw_serial.log`
- TIM11 tick B: `mpu_phase_ab_logs/session_20261013_010213/raw_serial.log`
- Comparator: `mpu_phase_ab_logs/tim11_scheduler_comparison.json`

## Verified result from captured data

| Metric | A — legacy elapsed | B — TIM11 tick |
| --- | ---: | ---: |
| Diagnostics trace status | PASS | PASS |
| Integration gate | REVIEW_FRESHNESS_REGRESSION | PASS_INTEGRATION_MOTOR_OFF |
| Paired TIM11 ticks | 225000 | 225000 |
| I2C IRQ-completion reuse | 446 (0.19822%) | 223 (0.09911%) |
| Published-snapshot reuse | 446 (0.19822%) | 223 (0.09911%) |
| Additional publication reuse | 0 | 0 |
| Registered sampling requests | 224777 | 224999 |
| Request gaps >1050us | 442 | 222 |
| Request gaps >1500us | 223 | 1 |
| Longest request gap | 2000us | 2000us |
| IRQ→main, avg/max | 2/3us | 2/3us |
| Main→publication, avg/max | 13/15us | 13/16us |
| IRQ→publication, avg/max | 15/17us | 15/18us |
| I2C errors/timeouts | 0/0 | 0/0 |
| UART dropped records | 0 | 0 |
| TIM11 late/gap errors | 0/0 | 0/0 |
| Postcal invalid/expired (>3ms) | 0 | 0 |

Per-window patterns were independently checked against the **raw UART**,
not inferred merely from summary JSON:

| 5s window pattern | A | B |
| --- | ---: | ---: |
| 5000 TIM11 ticks + 4995 requests | 43/45 | 0/45 |
| 5000 TIM11 ticks + 4996 requests | 2/45 | 0/45 |
| 5000 TIM11 ticks + 5000 requests | 0/45 | 44/45 |
| 5000 TIM11 ticks + 4999 requests | 0/45 | 1/45 |
| 10 IRQ/snapshot reused ticks | 43/45 | 0/45 |
| 5 IRQ/snapshot reused ticks | 0/45 | 41/45 |
| No request gap >1500us | 0/45 | 44/45 |

**Interpretation:** A scheduler synchronized to TIM11 removed
~99.6% of long (>1500us) request gaps and cut reuse approximately in half.
This supports the hypothesis that the floating `now-last_request_us>=1000`
condition sometimes skips a 1ms wake opportunity. It does **not** prove
every reused sample had the same cause. Trace A vs B within each firmware
showed identical reuse counters, reinforcing that filter/gyro publication
was not the dominant cause.

**Residual 0.09911% reuse:** The tick scheduler registers almost every
1kHz request but still sometimes presents an unchanged completed sample
at an exact TIM11 consumer instant. This may be scheduling/phase jitter
or incomplete samples by the consumer deadline, rather than lost requests.
The available aggregate 5s logs do not demonstrate a unique root cause.
The single >1500us request gap and 2000us max also deserve attention,
but no I2C timeout/error or >3ms invalid snapshot was observed.

**Reference caution:** Previous RC1 diagnostic was 0.0441% aggregate and
0.10508% stage-0. Those different ISR/UART workloads are **not** strictly
matched to this experiment; do not claim the new scheduler beat or lost
against full RC1 production behavior solely from those numbers.

## Follow-up before freezing MPU/I2C

1. Run the reversed order **B → A**, with the same motor-off stationary
   setup, same software commit, clock/MPU settings, 240s each.
2. Check that both are labeled correctly in `PHASE_PROFILE.scheduler`.
   Analyze with `tools/mpu_tick_scheduler_ab_compare.py`. Save and push
   raw logs, original reports, command outputs and resulting compare JSON.
3. If possible add another B replicate. Seek comparable order-independent
   reduction with no clock, I2C, calibration, TIM11 or UART regressions.
4. If repeat confirms improvement, freeze TIM11 sampler as a new module
   revision while retaining RC1 untouched; document snapshot validity,
   missed-tick semantics and failed-calibration safe handling.
5. Perform a subsequent *separate* realistic electrical-load/EMI test
   before calling it production validated. This A/B deliberately kept
   the motor supply disconnected.

Neither a host static check nor a 240s stationary trace establishes
formal interrupt WCET or long-term yaw drift accuracy.

No algorithm or firmware adjustment is necessary for the next replicate:
it is a pure confirmation of existing experimental builds.
