# Full high-speed Micromouse Motor Driver qualification — one Python entrypoint

**Branch:** `feature/motor-driver-optimization`

**Python orchestrator:** `tools/motor_full_qualification.py`

**Existing motor runner reused:** `tools/motor_auto_pipeline.py`

**New one-flash sensor+motor firmware:**

- `motor_auto_sensor_96m_trim25`: protocol **v7**, 3×21 = 63 short motor stages, at most 20% one wheel and 16% both; MPU6050 Policy B RC2 + IR sensors run concurrently. No HIGH-PWM stage.
- `motor_auto_sensor_high_96m_trim25`: protocol **v8**, 3×29 = 87 motor stages, includes isolated 30/40/50/60% single-wheel 120ms pulses, plus MPU/I2C/IR. **External electrical evidence gate required**. This is NOT safe merely because v6 completed 87 motor-only pulses.

The existing standalone motor **v4/v6**, frozen standalone MPU RC2 and default firmware remain available and unchanged in scope. The new test is opt-in.

## Scope: all five requested groups are in ONE coordinator — not five unsafe auto-experiments

| Gate | Automated now | What can really be proven |
| --- | --- | --- |
| P0 STOP/Coast/Brake | Source audit + VM-independent MCU ODR/CCR/OC1PE/OC2PE register snapshot on each PING | **Software neutral state only**. Electrical Hi-Z, short-brake current and output terminal voltages still require external oscilloscope/current equipment |
| P0 Electrical limits | Evidence JSON intake + conservative numeric and completeness filters | Specific N20 motor model, load/no-load/stall current, VM, TB6612 temperature, actual GPIO HIGH and PWM output need **external instrumentation**. No ADC motor-current channel currently present |
| P1 Encoder/startup | Re-analyse v6 `auto_20261013_105353` (no reflash), flag nonmonotonic 8–9% responses, three-repeat statistics | Current fixed-order data are **not sufficient** to identify rotor-position dependence or loaded deadband. Mark REVIEW |
| P1 Motor + MPU6050/I2C + IR | **New single-flash v7** (or gated v8): MCU calibrates MPU while motor disabled, only then emits UART READY; TIM11 services short motor stages, MPU RC2 samples in main, IR TIM10/ADC performs scans; emits two `MOTOR_AUTO_SENSORS` counters per stage | Actual observed gyro fresh/dropped/error/reuse, IR scan count and 1kHz scheduler timing under motor PWM, once run on PCB |
| P2 High PWM 80–100% and loaded runs | 30–60% short free-spinning v6 result re-used; optional externally gated v8 <=60%; **80–100% and loaded are blocked** | 87/87 v6 pulses show *commands* and encoder motion only, not allowable current/thermal/sustained speed or loaded wall-following |

### Why the high-power gate is intentionally strict

The Toshiba TB6612FNG official documentation lists IOUT ~1.2A average and a 3.2A *single pulse of 10ms* absolute maximum; **3.2A does NOT apply to 120ms, stall or repeated high-duty pulses**. The chip's control HIGH threshold is `0.7×VCC`. At logic VCC=5V, guaranteed HIGH threshold is 3.5V, versus the STM32 nominal 3.3V. Fixed STBY=5V is not an independent firmware kill. A short encoder test cannot detect unsafe current spikes, heat or incorrect output waveforms.

Neither stall nor active brake nor reversal will be triggered automatically. **Do not deliberately stall a powered motor without an independently qualified current-limited measurement setup.** Stall current may be estimated from manufacturer data or separately characterized with a properly limited power supply; record method, do not invent a value.

## First command: audit everything already measured, no flashing/motion

From project root after updating Git:

```powershell
python tools/motor_full_qualification.py --audit
```

Reads `src/motor.c`, `include/pinout.h` and existing
`motor_logs/auto_20261013_105353/motor_auto_report.json`.
Writes exactly one new `motor_logs/qualification_YYYYMMDD_HHMMSS/`
with `qualification_report.json` and `qualification_summary.txt`.

Results use **PASS_BENCH**, **REVIEW**, **BLOCKED**, **NOT_RUN**, or
**INCOMPLETE**. Neither blocked nor unmeasured entries become PASS.

## One command for a new fully integrated powered test (<=20% only)

With robot stationary, wheels elevated/free, physical VM cutoff accessible:

```powershell
python tools/motor_full_qualification.py --run-integrated --port COM6 --vm-state powered --meter-vm-volts 8.2 --confirm-wheels-elevated
```

Replace `8.2` with the *actual hand-measured* VM or omit if unknown.
The orchestrator reuses existing high-PWM measurements, calls the
v7 motor runner, flashes **one** integrated image, waits for **IMU
calibration and IR scan readiness** before allowing UART START,
executes 63 guarded stages with 3 repetitions, and analyses all sensor
and motor telemetry. It then emits a combined qualification report.
Never press a KEY or flash separate test apps inside this run.

If MPU fails initialization/calibration, IR doesn't scan, TIM11 stalls,
UART heartbeat fails or commanded PWM exceeds profile constraints,
the firmware must not proceed through subsequent motor stages. No
hardware-cutoff guarantee exists; an actual electrical fault still
requires external VM disconnect.

**Software-only nonmotor UART link precheck** for the v7 firmware:
```powershell
python tools/motor_auto_pipeline.py --check-link --sensor-integration --port COM6
```
This flashes v7 and can take up to a minute for calibration.
Do not use `--skip-flash` unless that exact v7/v8 firmware was
already flashed and the host profile handshake confirms it.

## Electrical evidence: template, not fabricated telemetry

Copy `docs/motor_driver/electrical_evidence.template.json` and fill it
from **real external observations**, including motor exact part number,
separately assessed no-load/running/stall currents for both channels,
external IOUT peaks, VM, driver case temperature, actual logic HIGH,
scope waveform, external hardware current cutoff and measurement method.

To include that evidence in an audit:

```powershell
python tools/motor_full_qualification.py --audit --electrical-evidence docs/motor_driver/electrical_evidence.measured.json
```

A numeric gate passing is classified **REVIEW**, not third-party
certification or guaranteed safety. Only once all independent electrical
conditions and a physical cutoff are verified, the explicit combined
v8 mode can be requested:

```powershell
python tools/motor_full_qualification.py --run-integrated --with-high --electrical-evidence docs/motor_driver/electrical_evidence.measured.json --port COM6 --vm-state powered --meter-vm-volts 8.2 --confirm-wheels-elevated
```

This still stops at **60%**, never commands 80–100%. It does not make
a 60% duty electrically safe simply by generating data. For higher
duty and loaded running, a **separate instrumented mechanical/electrical
qualification and Motion-controlled test** must be designed.

## Implementation and protocol integrity

Integrated v7/v8 uses separate firmware identifiers; Python rejects
incorrect version, duty cap, stage count, sensor field or missing READY.
The MCU replays a read-only `MOTOR_AUTO_STATIC` snapshot during PING
with left/right CCR, direction GPIO ODR, OC1PE/OC2PE preload and ARR,
but this register snapshot is NOT an oscilloscope reading. The MCU also
emits `MOTOR_AUTO_SENSOR_READY` after calibration and
`MOTOR_AUTO_SENSORS` in both drive and post-neutral phases of every stage.
Every report validates monotonic sensor counters, gyro absolute-rate
samples taken at 20ms intervals during drive and neutral observation,
real freshness/IR scan progress, 1kHz timing, host heartbeat, and fails
closed for missing telemetry. Differences between the gyro-on and
gyro-neutral proxies can include vibration or genuine rotation: the
pipeline cannot identify electrical EMI with certainty. USART1 UART RX IRQ ring and per-pulse software watchdog
remain enabled. MCU STBY remains fixed HIGH, so UART STOP is not an
emergency hardware kill.

## Existing v6 results and limitations

- v6 `auto_20261013_105353`: 87/87 starts/ends/post-ends, 2994 samples,
  301/301 host heartbeats, no TIM11 gap/late and no UART RX/TX drops.
- 30/40/50/60% short pulses yielded increasing encoder counts, but
  high-duty **post-neutral** encoder count continued for hundreds of
  milliseconds; especially L +1265/R +1113 at 60%.
- Startup nonmonotonic: left 8% 3/3 success but left 9% 0/3.
  Do NOT calibrate startup deadband to 8% on the basis of minimum of
  successful levels; genuine rotor-position/order testing is still open.

**Last point:** this is a qualification pipeline, not a high-speed Motion
controller. PID gains, turning control, physical stop distances and
maze competition operation are outside this Motor Driver harness.

Sources: Toshiba datasheet
https://toshiba.semicon-storage.com/info/TB6612FNG_datasheet_en_20141001.pdf?did=10660&prodName=TB6612FNG
and product maximum ratings
https://toshiba.semicon-storage.com/ap-en/semiconductor/product/motor-driver-ics/brushed-dc-motor-driver-ics/detail.TB6612FNG.html
