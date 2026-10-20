# Encoder + I2C multi-diagnostic A/B pipeline (20/10/2026)

## Scope and preservation

Branch: `giomuadongbac`. This extends the already installed
`tools/encoder_qualification_pipeline.py`; no new command is required.
The existing 33-stage LEFT / RIGHT / BOTH, PWM 0–100% sweep, motor lock,
ST-Link restoration, operator-presence requirement, independent VM cutoff,
MPU calibration check, UART lease and watchdog remain the safety baseline.

**Status: implemented in source; NOT YET compiled or hardware-qualified in this
revision.** Previous `encoder_qual_20261020_022526` data remain a baseline,
not an acceptance result for the new shadow-bank workload.

The production firmware is not built with `ENCODER_PROFILE_DIAG` or
`ENCODER_ALPHA_AB_DIAG`. In both opt-in diagnostic profiles, the original
PPS path `SPEED_LPF_ALPHA = 0.30f`, quadrature x4 counting, encoder
polarity/PPR and PID input are unchanged. Shadow values do not command motors.

## Measurement collection

`src/encoder.c` updates **seven additional EMA instances PER WHEEL on every
TIM11 1-ms tick** using the same signed counter delta as the production
estimator:

- EMA `alpha = 0.05, 0.10, 0.20, 0.30, 0.40, 0.60, 0.80`
- Trailing-count boxcars 5 / 10 / 20 ms, updated each 1-ms tick
- Raw encoder counters and existing production EMA

The 20-ms boxcar is a **dependent diagnostic speed proxy**, NOT independent
actual RPM. The A/B scoring can therefore select an alpha close to that
chosen smoother; that preference cannot establish physical velocity accuracy.

1-kHz cumulative measurements are reset at each DRIVE/COAST stage transition,
without resetting EMA histories. `SYS_AM` reports six metric records per
interval (two wheels × three metrics) over several main-loop iterations:

- `m=0`: aggregate absolute deviation from trailing 20-ms count rate after
  the first 500 ms of that interval.
- `m=1`: aggregate deviation over the first 500 ms (transition proxy).
- `m=2`: aggregate absolute 1-ms EMA-to-EMA difference after 500 ms
  (variation/smoothness proxy; not independent measurement noise).
- `n`: number of 1-ms samples included in that statistic; `x0..x6`
  correspond, in order, to the seven alpha candidates. Divide by `n`.

`SYS_AB` exports instantaneous seven-alpha and three-window PPS snapshots
at 10 Hz, separately per wheel (`w=0` left, `w=1` right), suitable for
viewing longer trends but **not** resolving 1–20-ms response latency.
`SYS_EN` remains approximately 50 Hz. UART text is emitted only in main,
not in TIM11; the host gates fail on UART dropped messages or TIM11 timing faults.

## I2C startup and BUSY evidence

Prior completed run `encoder_qual_20261020_022526` recorded:

- OFF: 400 kHz succeeded, `WHO_AM_I=0x68`.
- ON: 400-kHz address probe returned `I2C_BUSY=2`; the next ~300-kHz
  configuration succeeded at a nominal 296,296 Hz with zero later runtime
  I2C errors and timeouts.
- A Motor-ON firmware image initializes and probes the MPU **before** the
  motor sweep begins. BUSY during boot does not alone establish motor PWM EMI.

In addition to `SYS_IPB` (the selected I2C speed/probe status),
diagnostic-only `SYS_IB` reports:

- `n`: count of wait-for-bus-free timeouts; `rec`: recovery count
- `sr1/sr2`: registers sampled at the last BUSY timeout
- `pin`: PB8(SCL) bit 0 and PB9(SDA) bit 1 BEFORE recovery
- `after`: sampled PB8/PB9 levels after recovery

This is a GPIO status snapshot, not a waveform or proof of cause.
No I2C timing/timeout, bus recovery algorithm, or pull-up setting has been
changed by the A/B instrumentation. Do not force 400 kHz without evidence.

## Host results and decision rule

One pipeline creates a new `results/encoder_qual_YYYYMMDD_HHMMSS/` directory.
The existing report/check gates remain, plus:

- `system_uart_hardware/off_uart.log` and `on_uart.log`: full raw UART
- `encoder_motor_off_50hz.csv`, `encoder_motor_on_50hz.csv`
- `encoder_alpha_samples_10hz.csv`
- `encoder_alpha_stage_metrics.csv`
- `encoder_alpha_ab.json` — weighted per-metric summaries, scores,
  per-wheel/per-PWM comparison and separate wheel rankings, **provisional alpha top two**
- `encoder_alpha_overlay_60pct_L.png`,
  `encoder_alpha_overlay_60pct_R.png`, `encoder_alpha_ranking.png`
  when matplotlib is available
- `summary.json` and `REPORT.md` including I2C BUSY and general health

Scoring compares candidate metric values to the **existing alpha=0.30**,
using a predeclared normalized heuristic:

`score = 0.50 × steady deviation ratio + 0.35 × first-500ms deviation
ratio + 0.15 × steady variation ratio`.

Only active wheel intervals with PWM 20–80% participate in the alpha shortlist:
steady-state deviation and variation are taken from **DRIVE only**;
first-500ms transition deviation includes both DRIVE onset and COAST onset.
Short 90–100% intervals and stationary channels are excluded. Detailed per-wheel
and per-PWM tables remain available. The ranking is **not** an automatic
firmware configuration choice. Require repeated tests at comparable battery
voltage/load, PWM step-response and, ideally, an independent tachometer or
external reference before using any new alpha in control.

If any of the 792 expected `SYS_AM` metric records is missing, duplicated,
or malformed, or fewer than 500 snapshots are received, the alpha A/B
test is **INCOMPLETE**, not silently accepted. The original system
I2C 400-kHz criterion remains strict and independent of Encoder A/B status.
Restoration of the locked motor-off diagnostic is attempted regardless
of a reported failure; a physical VM cutoff remains necessary.

## Running the diagnostic on Windows (operator present)

After pulling `giomuadongbac` with a clean working tree:

```powershell
python tools/encoder_qualification_pipeline.py --mode full --port COM7 --motor-on-sweep --fixture-ready
```

This automatically executes host/native/self-tests, both PlatformIO builds,
OFF 33 stages, ON 33 stages, per-stage A/B analysis, I2C bus instrumentation,
plots/reports, and attempts locked-OFF firmware restoration.

**Physical prerequisite:** wheels lifted and firmly secured, no obstacles,
independent motor power disconnect, operator present throughout, and
awareness that the existing sweep includes 100% PWM. Never run the ON
configuration with an unsecured robot.

## Evidence limitations

- Source audits and synthetic regression gates are not real-hardware qualification.
- An estimator can look smooth but lag actual speed. Metrics are proxies.
- The CPU cost of seven shadow EMAs and rolling windows must be validated
  on physical STM32 TIM11 timing. Reject any nonzero `late`, `gap`,
  motor watchdog fault or UART drop.
- 100% PWM lasts 750 ms in the current sweep; do not infer long-duration
  motor/encoder stability from this short segment.
- Zero I2C errors after successful 300-kHz fallback is not evidence of
  dependable 400-kHz startup.
- Raw logs (including `.log` ignored by `.gitignore`) must be retained
  and intentionally pushed if needed for publication-grade audit.

## First real multi-diagnostic run: 20261020_030513 (provisional)

Source data: `results/encoder_qual_20261020_030513/`, executed at
`697d9dd` and later uploaded. This run completed OFF and ON 33-stage
sweeps, passed all software/build gates, and restored motor-off firmware.

- OFF: `tx_drop=0`, `rx_err=0`, `TIM11 late=gap=0`.
- ON: `tx_drop=1`, `rx_err=24`, `TIM11 late=gap=0`.
  Hence both `uart_no_tx_drop` and `uart_no_rx_error` failed.
- MPU: 400-kHz configured I2C in **both phases**; `i2cerr=0`,
  `to=0`. No fallback to 296 kHz this run.
- Max TIM11 cycles (ON): 8,965 vs 1,905 baseline `022526`.
  Max Encoder update cycles: 8,235 vs 971 baseline. These are
  maxima, not CPU utilization or representative duration.
- Physical motor PWM response was recorded and encoder consistency gates
  passed; this does **not** prove physical wheel-speed precision.

**Alpha scores from this run are NOT qualified or reconstructed from the
regular `measurements.csv`.** The old host pipeline evaluated the 50-Hz
health before running the alpha analyzer, so `alpha_ab` is `null`
in `030513/summary.json`. Moreover the standard `measurements.csv`
fixed schema discarded the `SYS_AB/SYS_AM` fields. The entire original
`off_uart.log` and `on_uart.log` was not present in the uploaded
GitHub result folder as of this review.

**Salvage without another hardware run**, if raw logs still exist locally:

```powershell
python tools/encoder_shadow_ab.py --run-dir results/encoder_qual_20261020_030513/system_uart_hardware --output-dir results/encoder_qual_20261020_030513
```

The host analyzer preserves any real alpha measurements it can parse,
including partial data while labelling missing or duplicate metric frames.
Then push its JSON/CSV output and the ignored original raw UART logs. If
the raw logs have been deleted, the earlier fixed-schema
`measurements.csv` cannot reconstruct the 1-kHz alpha metrics.

**Subsequent code correction:** full pipeline no longer suppresses alpha
analysis solely because UART health is unqualified; a completed sweep
can be recorded as `SWEEP_COMPLETE_HEALTH_UNQUALIFIED` with strict
exit status. A new per-phase `encoder_shadow_off.csv` /
`encoder_shadow_on.csv` preserves parsed full SYS_AM/SYS_AB/SYS_IPB/
SYS_IB fields in addition to the original `.log` files, making
future uploaded results independently analyzable.

No production alpha or motor/PID parameters were changed based on
this run.

## 2026-10-20 follow-up: UART throttling and refined alpha A/B (source implementation)

**Motivation and measured baseline:** On the first seven-alpha physical sweep,
`encoder_qual_20261020_030513`, the 2,048-byte diagnostic TX queue reached
2,010 bytes (ON peak); `SYS_U` reported 24 USART RX errors and one rejected
TX message. The on-board 1-ms Encoder ISR completed with no missed TIM11 tick,
but the Encoder max execution duration increased significantly vs no shadow bank.
This evidence suggests profiling/telemetry pressure, **not proof** that TX
backpressure caused USART RX error flags or encoder miscounts.

**New ten-candidate shadow bank**, with original production `alpha=0.30`
unchanged:

`0.05, 0.08, 0.10, 0.12, 0.15, 0.20, 0.30, 0.40, 0.60, 0.80`.

The previous sweep recommended 0.10 **only according to its dependent
20-ms window proxy score**, therefore the new sweep densely samples 0.08,
0.10, 0.12 and 0.15. These extra candidates run at 1 kHz on the same raw
counter deltas; they do not affect PID inputs or motor commands.

**UART workload protections** only when `ENCODER_ALPHA_AB_DIAG` is enabled:

- `SYS_AB` trace emission reduced from 10 Hz to **5 Hz**; long-view
  overlay plots retain all ten alphas, 5/10/20-ms boxcars.
- `SYS_IPB` and `SYS_IB` repeat every **10 seconds**, instead of every
  second. These are retained boot-probe / BUSY metrics and are not needed
  for alpha scoring; the host receives them in raw logs.
- The optional `SYS_AB` trace and mandatory per-stage `SYS_AM` records
  are queued only if the current DMA TX queue contains **fewer than 768
  bytes**. Six `SYS_AM` metric frames per interval are retained and
  gradually emitted; should a frame be missing, the alpha A/B acceptance
  remains `INSUFFICIENT_SHADOW_DATA`, not silently PASS.
- `SYS_EN` remains approximately 50 Hz and all safety, motor, MPU, TIM11
  and other system diagnostics retain their normal health checks.

**USART RX error classification:** `SYS_UR` reports cumulative
`err` (invalid USART RX IRQ events), `ore` (overrun),
`fe` (framing error), `ne` (noise error), and `ov` (software RX
ring overflow). Individual USART SR flags can occur together, so their
sum need not equal `err`. The original `SYS_U.rx_err` remains the
authoritative strict acceptance counter. No UART baud, priority, DMA
driver, RX discard policy, I2C speed or motor driver is changed.

**Data schema and replay:** `SYS_AM` now uses `x0..x9`;
`SYS_AB` includes `a08`, `a12` and `a15`. The analysis script
`tools/encoder_shadow_ab.py` distinguishes the historical 7-alpha
format from the new 10-alpha format. `system_uart_hardware/
encoder_shadow_{off,on}.csv` and original UART logs preserve the
additional measurements, including `SYS_UR`. Historical raw logs
are not rewritten.

**Acceptance criteria for next physical run:** all 14 software/build
gates PASS; 33 OFF and 33 ON stages completed; 792/792 alpha metric
frames, zero malformed/duplicate entries; TIM11 late/gap 0;
MPU I2C runtime errors and timeouts 0; UART TX drops/DMA faults/RX
errors/RX overflow 0; motor watchdog faults 0; OFF locked restore
successful. Any failure remains in the report and blocks whole-system
qualification. Ranking/alpha selection is **provisional** until
repeatability and actual closed-loop response have been assessed.

**State:** Code and CI are independent of physical hardware. No new
physical result is claimed by this document. Refer to the Actions
workflow for latest software test state.

## Run 20261020_083902 — motor-on UART lease safety abort

Source: `results/encoder_qual_20261020_083902/` (captured on
`0ccea850`), uploaded to `giomuadongbac`. The 14 software/native/build
gates passed. The OFF sweep finished 33 stages, **ON was interrupted** at
zero-based stage 13 (RIGHT wheel PWM 20% DRIVE); this is not a passing
hardware run. Reported `SYS_STEP.state=4,fault=9` corresponds to
`SWEEP_UART_LINK_LOST` in `include/integration_sweep.h` and occurs when
the current motor-on diagnostic session receives no valid parsed `PING`
for more than 750 ms. Firmware called sticky motor output lock, and
host subsequently reported locked motor-off upload `pass=true`.

Observed counters before termination:

- OFF `SYS_IPB`: 400-kHz probe I2C_BUSY=2 and successful
  296,296-Hz fallback; PB8 high / PB9 low at BUSY timeout, both high
  after recovery; one UART ORE error.
- ON `SYS_IPB`: nominal 400 kHz accepted; 18 UART RX errors, all
  ORE (FE=NE=0). No TIM11 late/gap or MPU I2C read-time errors in
  saved phase 50-Hz summary.
- Encoder A/B `INSUFFICIENT_SHADOW_DATA`: 552/792 metric frames.
  The first 396 belong to OFF and only 156 to early ON stages.
  Any ranking, including leading 0.08 in the partial report, is
  exploratory and **NOT** a full two-wheel 0–100% A/B finding.

**Root cause statement:** confirmed lost UART command-lease;
not yet confirmed why valid `PING` stopped reaching the MCU.
USART hardware ORE can corrupt command reception, but frequency
and causal timing must be established using paired host PING
and CMD_PING acknowledgement logs. The 750-ms MCU timeout must
not be enlarged or disabled as a workaround.

**Independent host defects found and corrected after this capture:**

1. `render_plots()` used non-existent `stage` field instead of
   `id` in `encoder_alpha_samples_10hz.csv`, triggering
   `KeyError('stage')`; optional plot failures are now nonfatal.
2. Interrupted ON session formerly dropped its phase-level
   `evaluate()` result from `system_uart_hardware/summary.json`,
   leaving the parent report displaying only OFF health failures.
   Interrupted phase now retains partial health, safety fault and
   number of stages explicitly as **UNQUALIFIED**.
3. Host PING instrumentation now saves attempted PING count,
   CMD_PING acknowledgements, largest host inter-PING send interval,
   and maximum UART write duration to allow later correlation.
   Motor control/heartbeat watchdog policy is unchanged.

**Next action:** no immediate motor-on rerun. Preserve
`system_uart_hardware/off_uart.log` and `on_uart.log`
(if still present locally and Git-ignored), inspect PING/ACK temporal
gaps and the UART ORE evolution. Prefer a motor-locked serial
diagnostic for validating telemetry path first. Only when the
command-lease path has been verified safe should the supervised
motor sweep be repeated. Neither I2C nor EMA production parameters
were changed by the recovery patches.

## 2026-10-20: Shadow A/B offloaded from TIM11 IRQ

### Why this optimization exists

Measured baseline before this patch, motor-ON run `083902`:
TIM11 maximum `11,741` CPU cycles at nominal 96 MHz (~122.3 us),
of which Encoder consumed `10,999` cycles (~114.6 us). At USART1
115,200 8N1, an incoming byte takes ~86.8 us. TIM11 uses NVIC
priority 0; RXNE uses priority 8, so a lengthy TIM11 interrupts UART
byte servicing. Raw logs confirmed repeated RX overrun (`ORE`),
corrupted/unknown commands and finally `fault=9`
(`SWEEP_UART_LINK_LOST`). This is a strong timing hypothesis,
**not proof** that all ORE events originated from TIM11.

### Implemented mechanism

- The original quadrature TIM3/TIM4 x4 counter read and
  **production** speed EMA `0.30` remain in the high-priority 1-ms
  TIM11 ISR, unchanged.
- The diagnostic-only path no longer computes ten shadow EMAs, 5/10/20
  ms boxcars or score aggregates inside TIM11. It only writes the
  signed left/right counts for the 1-ms tick to a static 256-entry
  single-producer/single-consumer (SPSC) circular FIFO. FIFO uses one
  reserved slot, so effective capacity is **255 ticks**.
- Main first polls the bounded USART command parser (PING/STOP), then
  executes the original unchanged 750-ms sweep heartbeat safety
  check, then consumes **up to 12 samples** from the FIFO. At the
  transition snapshot, at most 32 additional samples are drained,
  then the interval's metrics are emitted in paced background UART
  messages. This sequencing does not allow the high-priority ISR to
  block UART per-character reception for the entire EMA calculation.
- The consumer replays the same two signed counter deltas in FIFO
  order, calculates raw PPS as `signed_counts * 1000`, and updates
  the ten existing floating-point EMA and three boxcar estimates.
  There is no thinning of the 1-kHz measurement timeline;
  **overflows are counted and NEVER accepted as valid A/B data**.
- The diagnostic `SYS_AQ` record at 1 Hz includes:
  `in` (FIFO accepted ticks), `out` (ticks consumed),
  `lost` (FIFO overflow), `edge` (stage transition backlog
  greater than two ticks), `q` (current pending), and
  `peak` (highest pending). Host qualification requires
  `lost=0` and `edge=0`. The margin of two samples accounts for
  concurrency near transition sampling; it does not guarantee exact
  physical transition alignment.
- No USART DMA RX, NVIC priority, motor PWM/STOP, MPU I2C parameters
  or heartbeat timeout has been modified by this optimization.

The FIFO makes IRQ runtime short, but **does not reduce total estimator
CPU work**: it moves that work into a preemptible main context.
The 12-sample processing budget is deliberately bounded to prevent
unbounded starvation of motor lease and UART command processing.
A saturated main loop can still result in `lost>0` or `edge>0`;
such a run is unqualified rather than silently producing biased data.

### Software validation and acceptance workflow

The existing `tools/encoder_native_test.py` now compiles/runs actual
`src/encoder.c` in both production and diagnostic configurations.
The diagnostic native harness checks:

- Producer does not update shadow EMA until main consumes.
- Two-channel signed 1-ms deltas are consumed in FIFO order.
- Service respects its budget and tracks stage-boundary backlog.
- A full FIFO reports exactly how many samples were lost.
- Normal 10,000-tick quadrature/wrap test still passes when serviced
  by the consumer.

GitHub Actions executes the host/native suite and both PlatformIO
diagnostic builds plus production build (CI is not physical acceptance).

**Next physical test MUST be motor-off only first.** Disconnect the
motor VM independently (do not rely only on firmware locking), keep
the serial adapter on COM7 and run:

```powershell
python tools/system_uart_pipeline.py --port COM7 --flash --encoder-telemetry
```

This flashes the explicit motor-OFF locked diagnostic, exercises all
33 logical OFF sweep steps while transmitting PING and the normal UART
telemetry, and records raw UART plus `SYS_AQ/SYS_UR/SYS_T`.
Do not add `--motor-on` for this first validation.

**Acceptance before motor-on:** `SYS_AQ.lost=edge=0`, all
33 OFF steps, no `fault=9`, `SYS_U.rx_err=0` and
`tx_drop=dma_err=0`, zero `SYS_T.late/gap`, and a clearly
lower measured `SYS_T.encmaxcy` versus the 10-alpha 083902
baseline. If USART RX ORE persists, investigate ISR priority,
main-loop command framing, and RX DMA in a separate firmware
branch rather than weakening the motor watchdog.

Do not claim a new alpha optimum from this OFF-only run, since
motor speed/PWM response remains unobserved.
