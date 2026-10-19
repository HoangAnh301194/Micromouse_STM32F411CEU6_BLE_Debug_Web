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
  per-wheel/per-PWM comparison, **provisional alpha top two**
- `encoder_alpha_overlay_60pct_L.png`,
  `encoder_alpha_overlay_60pct_R.png`, `encoder_alpha_ranking.png`
  when matplotlib is available
- `summary.json` and `REPORT.md` including I2C BUSY and general health

Scoring compares candidate metric values to the **existing alpha=0.30**,
using a predeclared normalized heuristic:

`score = 0.50 × steady deviation ratio + 0.35 × first-500ms deviation
ratio + 0.15 × steady variation ratio`.

Only active wheel intervals with PWM 20–80% participate in the alpha shortlist,
avoiding short 90–100% intervals and stationary channels. Detailed per-wheel
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
