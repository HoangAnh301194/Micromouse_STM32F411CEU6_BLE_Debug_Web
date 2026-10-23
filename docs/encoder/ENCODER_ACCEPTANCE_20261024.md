# Encoder module acceptance record — 2026-10-24

**Acceptance decision:** PASS for the incremental quadrature-encoder driver and
1-kHz encoder-count / PPS-estimator path **on the supervised lifted-wheel
test fixture**. Select **production EMA alpha = 0.10** as an explicit
engineering tradeoff, not a scientifically established unique optimum.

## Evidence and scope

- Repository: `HoangAnh301194/Micromouse_STM32F411CEU6_BLE_Debug_Web`,
  branch `giomuadongbac`.
- Primary physical run:
  `results/speed_pid_final_20261024_003343/summary.json` (immutable
  machine report, including the historical `INCONCLUSIVE` winner rule).
- Fixture Motor-OFF gate: 33/33 stages PASS, zero TIM11 lateness/gaps,
  zero dropped/edge-misaligned Encoder shadow samples; Motor-OFF
  `in=out=162846, lost=0, edge=0`.
- Closed-loop diagnostic: 9/9 live motor-on runs PASS, 12/12 stages each
  (108 total), three repetitions each of EMA alpha 0.08, 0.10, 0.30.
  All had UART TX drop=0, UART RX error=0, DMA error=0, TIM11 late=0
  and gap=0, motor fault=0, I2C timeout/error=0.
- Worst observed motor-on TIM11 IRQ 1899 cycles (~19.8us at nominal
  96MHz) and encoder callback 965 cycles (~10.1us). This measures
  interrupt execution time, **not total CPU consumption**.
- Largest recorded host PING transmit interval 150.74ms vs original
  750ms firmware safety deadline; nine sessions completed without
  UART_LINK_LOST. Production watchdogs, capped 30% test and STOP
  behavior remain unchanged.
- Motor-OFF diagnostic firmware was reflashed successfully after trials.
  This does NOT mean the updated production alpha was physically
  tested after the change below.

## Controlled 3-alpha closed-loop comparison

Fixed diagnostic per-wheel PID: Kp=0.05, Ki=0.08, Kd=0.0,
at 100Hz; 1kHz Encoder EMA; target steps 800,1600,2400 PPS; 0..30%
software-command duty limit. The raw 50ms Encoder count-rate (not
external tachometer ground truth) was the scoring reference.

| Alpha | Weighted score (lower better) | Speed MAE | Overshoot | Variation |
|---|---:|---:|---:|---:|
| **0.10** | **0.03224** | 3.136% | **4.942%** | 0.766% |
| 0.08 | 0.03270 | 3.138% | 5.324% | **0.687%** |
| 0.30 (old production) | 0.03383 | 3.225% | 5.347% | 1.092% |

Alpha 0.10 won 2 of 3 repetitions, but its weighted advantage over
0.08 was only **1.41%**, below the **pre-registered 5%** selection
margin. Thus the raw `summary.json` properly says
`status=INCONCLUSIVE; candidate_alpha=null`. **Do not edit or
reinterpret that experiment report as PASS for statistical superiority.**
We nevertheless explicitly choose 0.10 as the production engineering
parameter because its measured mean score and overshoot are lowest,
and both 0.08 and 0.10 consistently improve on the original 0.30
baseline in this test context. This choice is a decision under
uncertainty, not proof of 0.10 superiority or a formally tuned
production speed PID.

## Production implementation

- `src/encoder.c`: default/production `SPEED_LPF_ALPHA=0.10f` for
  both wheels; unchanged 1kHz update logic, quadrature counter mapping,
  direction signs, ISR/FIFO and other modules.
- Dedicated `ENCODER_SPEED_PID_DIAG` builds retain their compile-time
  isolated 0.08 / 0.10 / 0.30 overrides for future A/B comparison.
- `src/pid.c` and production Motion heading/turn PID parameters
  are **not changed**; closed-loop test PID was diagnostic-only.
- Firmware compile/native regression must pass on the production
  `genericSTM32F411CE` profile. Physical test evidence predates
  this production-only selection and is not a post-deployment test.

## Acceptance boundaries and follow-up

**Accepted:** Encoder count acquisition and signs in the measured
forward test, update cadence, per-wheel speed estimation, under-30%
raised-wheel speed-PID integration, clock/UART/MPU co-existence in the
qualifying run, and host/firmware safety handling within that scope.

**Not accepted by this report:** wheel-ground slip, exact ground
distance, wheel odometry scale calibration, real in-maze performance,
100% PWM closed-loop speed control, tuning Kp/Ki/Kd of an actual
production wheel-speed controller (which has not been implemented as
such), or long-run loaded motor behavior. Before deploying as a
finished motion controller, perform a production firmware smoke test
with wheel speed, heading and stopping on the actual intended robot.
If later behavior regresses, alpha 0.08 and the original 0.30 remain
available as measured comparator options.

**Decision:** `ENCODER_BENCH_ACCEPTED; PRODUCTION_ALPHA_ENGINEERING_CHOICE=0.10;`
`UNIQUE_ALPHA_OPTIMUM_UNPROVEN; MOTION_CONTROL_VALIDATION_PENDING`.
