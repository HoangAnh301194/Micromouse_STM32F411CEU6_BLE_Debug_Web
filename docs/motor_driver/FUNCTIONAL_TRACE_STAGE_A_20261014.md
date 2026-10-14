# Motor Driver qualification — Stage A: 1 kHz SRAM trace, no oscilloscope

Branch: `feature/motor-driver-optimization`

## Scope

This stage adds **read-only STM32 register/encoder telemetry** to the existing
`MOTOR_AUTO_BENCH` firmware variants 14/16/17/18. It does not change
`Motor_Init`, `Motor_Stop`, `Motor_Brake`, motor slew, watchdog,
Motion or PID. The firmware P0 START lock remains enabled.

Hardware available: STM32F411CEU6, TB6612FNG, both quadrature encoders,
USB-UART/USART1 PA9 TX, PA10 RX. **No logic analyzer or oscilloscope.**
No 12-file scope evidence is required for this Stage A.

The trace records 512 samples at 1 kHz (512 ms), using ~16 KiB SRAM,
and stores:

- TIM5 `micros()` TIM11-entry timestamp and observed period
- target left/right and applied signed duty (permille)
- CPU-visible TIM2 CCR2/CCR1 preload register values
- PB13/PB12 and PB15/PB14 motor-direction GPIO bitfield
- left and right total encoder counts
- Motor Test state/reason, including watchdog transitions

**Important:** TIM2 CCR register reads cannot establish the timer's active
preload latch output, TB6612 terminal voltage, PWM electrical edges,
braking current, or physical Hi-Z operation. Encoder deltas are functional
evidence only when a supervised motor test is performed, which this revision
does NOT arm.

## One-command offline pipeline (no firmware upload)

```powershell
git fetch origin
git pull --ff-only origin feature/motor-driver-optimization
python tools/motor_p0_batch_pipeline.py --run
```

The pipeline now covers **nine offline source/model gates**, including the
new 512-sample UART trace source/parser negative tests. An optional
oscilloscope CSV folder is no longer an expected prerequisite; when absent
it is reported `NOT_MEASURED`.

Reports:
`motor_logs/p0_batch_<timestamp>/p0_batch_report.json` and
`motor_logs/p0_batch_<timestamp>/p0_batch_summary.txt`.

Without new live capture, expected overall status is
`REVIEW_AWAITING_FUNCTIONAL_EVIDENCE`.
It is a review, **not a failed offline test**.

## Read-only on-device capture via COM6

The currently flashed v7 firmware does **not** implement `MOTOR TRACE`.
A git push **does not** change flashed firmware. If deliberately preparing
the new version for a motor-off test, **physically disconnect VM (motor supply)
and keep it disconnected throughout the experiment**. Build and upload the
already P0-locked firmware using the project's normal ST-Link connection:

```powershell
python -m platformio run -e motor_auto_sensor_96m_trim25
python -m platformio run -e motor_auto_sensor_96m_trim25 -t upload
```

No UART START or motor enable is needed. This firmware stays locked:
`MOTOR START` and `MOTOR START HIGH 600` are rejected by STM32.

After STM32 initialization and sensor calibration, perform **one entire
source+live-idle trace run**:

```powershell
python tools/motor_p0_batch_pipeline.py --run --uart-port COM6 --confirm-vm-disconnected
```

The pipeline only writes `MOTOR PING` and `MOTOR TRACE`; it never uploads
firmware, sends START or issues BRAKE/REVERSE. It waits for a current firmware
PROFILE and READY with `p0_locked=1`, `state=0`.
The onboard capture takes 512 ms, then UART emits one row every 25 ms.
The host checks for duplicate/missing rows, timestamps, 30kHz CCR neutrality,
GPIO direction, buffer drops and unexpected non-zero commands.

The output includes `motor_functional_trace_raw.log`, the full P0 JSON,
and a readable P0 summary. The expected qualified sub-result is
`PASS_IDLE_REGISTERS`; overall remains
`REVIEW_IDLE_REGISTER_TRACE_ONLY` because *no motor was started*.

If using a previously recorded UART trace instead of a port:

```powershell
python tools/motor_p0_batch_pipeline.py --run --uart-trace .\motor_logs\capture\raw_serial.log
```

The standalone parser can be exercised without the other eight tests:

```powershell
python tools/motor_functional_trace_pipeline.py --self-test
```

## Results interpretation

| Gate | Stage A outcome | Remaining work |
| --- | --- | --- |
| Source/ISR trace hook and buffer | Host regression and ARM firmware builds | Measure ISR overhead in live telemetry |
| 512-row capture integrity | Synthetic negative tests; real capture if supplied | Real board, new firmware |
| STOP settled GPIO/CCR | Register-level idle check | Actual terminal switching still unknown |
| Encoder response to DRIVE/COAST | NOT_RUN in P0-locked firmware | Future controlled low-duty qualification |
| Reversal/active brake current | NOT_RUN | Requires reviewed powered-test limits |
| Electrical PWM transient and motor current | UNVERIFIED_ELECTRICAL | Cannot infer from SRAM trace |

Stage B/C motor-powered plans are independent and **cannot be unlocked just
by a host flag or an expected self-test result**. No Motion/PID work is in scope.
