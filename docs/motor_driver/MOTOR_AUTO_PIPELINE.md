# Motor Debug v2 — one Python command, zero KEY presses, 3 independent rounds

**Firmware profile**: \`motor_auto_pipeline_96m_trim25\`.
**One host script**: \`tools/motor_auto_pipeline.py\`.
**Branch**: \`feature/motor-driver-optimization\`.

## Purpose

The 2026-10-13 five-stage motor log proved the left and right N20 respond to 12% PWM, but the right wheel recorded only 1 encoder count with 8% PWM. This new experiment quantifies repeatability with **three independent rounds** at 8%, 10%, and 12%, plus 300ms encoder measurement **after PWM is set to neutral**. Exactly 21 short pulses are emitted in a successful session.

## No KEY and no accidental motor startup

- MCU initializes TIM2 PWM neutral; after every RESET it is **disarmed**.
- Host opens USB-UART serial, sends \`MOTOR PING\`; STM32 answers \`MOTOR_AUTO_PROFILE\` and \`MOTOR_AUTO_READY\`.
- The Python script checks expected version, duty ceiling, number of rounds and that firmware state is idle, then sends **exactly one** \`MOTOR START\`.
- MCU waits 1000ms neutral, then executes a fixed matrix; no KEY button use.
- Host sends \`MOTOR KEEP\` about every 350ms. If none for 1600ms, firmware latches \`MOTOR_AUTO_FAULT\` and forces software PWM neutral. Each active pulse is hard stopped in the existing TIM11 1kHz motor-test watchdog after 200ms; there are no indefinite run commands.
- TIM11 gap faults also latch, prevent further pulses until RESET. Host \`MOTOR STOP\` aborts a run. STOP is only a firmware command and cannot substitute for physical VM power removal if firmware is unresponsive.
- Firmware automatically ends after exactly 3 rounds; no further START will be accepted until RESET.

**Required UART direction:** Prior bench only used USART1 TX PA9 -> USB-UART RX. This protocol additionally requires the same USB-UART adapter's TX -> STM32 **PA10 USART1 RX**, with common GND. Firmware can enable PA10 RX in software, but it **cannot create a physical return wire**. If the existing USB-UART hookup is one-way and PCB wiring cannot be changed, \`--check-link\` / \`--run\` will fail closed, with no PWM motion. There is intentionally no dangerous automatic-on-reset fallback.

## Software sequence (all in one invocation)

One-time local checkout/update:

\`\`\`powershell
git fetch origin
git switch feature/motor-driver-optimization
git pull --ff-only origin feature/motor-driver-optimization
\`\`\`

Then **only one Python command** is needed for build, STLINK upload, host handshake, all three measurement rounds and analysis:

\`\`\`powershell
python tools/motor_auto_pipeline.py --run --port COM6 --vm-state powered --meter-vm-volts 8.2
\`\`\`

Replace 8.2 with the voltage actually measured across VM/GND, or omit \`--meter-vm-volts\` if not measured. \`--vm-state powered\` is a required *operator declaration*, not a measurement. Be prepared to physically disconnect VM, put the chassis upside-down or safely suspend both wheels, and prevent contact with wheels/shaft. **Do not hold/stall the wheels or try braking/reversing in this test.** This firmware only operates at low short duty and does **not** sense/limit motor current.

The script does:
\`platformio build + STLINK upload -> COM6 PING/READY -> START -> KEEP heartbeat -> 21 isolated drive/post-stage cycles -> raw log -> CSV -> JSON + text summary\`.

The host needs Python, PlatformIO and pyserial:
\`\`\`powershell
python -m pip install platformio pyserial
\`\`\`

These are dependencies, **not extra pipeline scripts** to run for each experiment. A missing pyserial module causes the pipeline to fail before sending START. Both legacy KEY-bench and MPU RC2 remain untouched. Previous capture Python stays available but is **not called** by the new pipeline.

Optional no-motion UART check if unsure whether PA10 is wired:
\`\`\`powershell
python tools/motor_auto_pipeline.py --check-link --port COM6
\`\`\`
This command uploads the new firmware then validates host-to-STM32 communication; **never sends START**. Only needed if troubleshooting; normal \`--run\` already performs the same handshake.

## Firmware fixed matrix — executed three times automatically

| Step | Left duty | Right duty | Drive | Neutral encoder observation |
| --- | ---: | ---: | ---: | ---: |
| 0 | 8% | 0 | 200ms | 300ms |
| 1 | 0 | 8% | 200ms | 300ms |
| 2 | 10% | 0 | 200ms | 300ms |
| 3 | 0 | 10% | 200ms | 300ms |
| 4 | 12% | 0 | 200ms | 300ms |
| 5 | 0 | 12% | 200ms | 300ms |
| 6 | 12% | 12% | 200ms | 300ms |

Each pulse uses deadband=0 and slew rates rise=10 / fall=20 permille per ms, as in the previous baseline. There is a fixed 250ms gap between stages and 1000ms neutral before stage 0. No PID, IMU or IR runs in this opt-in build; TIM11 handles motor/encoder updates only.

Because the robot is not equipped with a verified current sensor, duty is **not a current limit**. 12V rated N20 running on 2S and successful historical movement still do not establish safe stall current.

## Outputs

The program creates \`motor_logs/auto_YYYYMMDD_HHMMSS/\`:

- \`raw_serial.log\`: every UART record with host monotonic-relative time.
- \`motor_auto_samples.csv\`: PWM applied, CCR registers, motor encoder counts and TIM11 status, every 20ms in drive and post-neutral phase.
- \`motor_auto_report.json\`: completeness gate and detailed 3-round condition statistics (mean / sample std, count differences after STOP, first time 5 encoder counts observed, heuristic sustained count signal).
- \`motor_auto_summary.txt\`: condensed per-condition findings.
- \`session_meta.json\`: operator-declared powered VM state and hand-measured DC voltage (if supplied).

An unpowered or non-moving wheel is **not automatically a failed experiment** if the requested PWM was issued: that is relevant deadband evidence. The report instead fails for missing/unexpected pulse commands, malformed/missing telemetry, unsafe PWM duty, incomplete post-stop observation, timeout and UART/TIM11 faults.

\`TRACE_COMPLETE_NOT_ELECTRICAL_APPROVAL\` means a COMPLETE measured trace, **not** electrical current/temperature/voltage approval. 5 encoder counts and last-100ms encoder movement are only heuristics to distinguish a twitch/single count from sustained movement; these are not speed control or calibrated deadband values. Encoder counts-per-wheel-revolution remain unverified.

After a successful run:

\`\`\`powershell
git add motor_logs/
git commit -m "test(motor): three-round autonomous host-armed baseline"
git push origin feature/motor-driver-optimization
\`\`\`

This is the only user action required to upload the evidence. Review \`motor_auto_report.json\` before proceeding with driver tuning.

## Hardware limitation

Existing fixed PCB: TB6612 VCC=5V and STBY tied to 5V, STM32 GPIO nominal 3.3V, one reported VM capacitor, no verified current sensing. Datasheet guarantees HIGH for VIH>=3.5V at 5V VCC, so this board works in practice *outside guaranteed HIGH-level margin*. The device cannot be put into STBY by firmware. Power and heat during motor load have **not** been characterized. External physical disconnect is required if unexpected motion continues.
