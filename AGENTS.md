# Agent entrypoint for the Micromouse firmware repository

**First read [PROJECT_STATUS.md](PROJECT_STATUS.md).** It is the maintained Vietnamese handoff and acceptance matrix: current modules, evidence, unresolved risks, test artifacts and the next priority. After that, consult [README.md](README.md) and only the sources/docs for the task at hand. Do not re-audit every historical commit unless evidence conflicts.

- **Branch target:** `giomuadongbac`. Confirm Git HEAD, working tree and `platformio.ini` before editing. Prefer minimal scoped changes; do not rewrite frozen Motor Driver or the handed-over MPU/I2C module without a specific finding.
- **Default next module:** six IR sensors (`src/ir_sensor.c`, `include/ir_sensor.h`, `src/control.c`, `include/pinout.h`). Audit the blocking ADC/DMA wait inside TIM10 ISR, then calibrate real wall response and validate coexistence with TIM11 1 kHz and MPU/I2C ~1 kHz.
- **Hardware invariants:** `include/pinout.h` defines the existing PCB, TIM11 priority 0 handles control, USART1 PA9/PA10 is the accepted debug path, I2C1 PB8/PB9 services MPU, TIM10 owns IR, TIM2 drives motor PWM. Never create a second owner for I2C1.
- **Handover evidence:** `docs/mpu_i2c_rc2/README.md` for accepted MPU/I2C RC2; source of truth for actual observations is immutable `results/` CSV/JSON, not workflow PASS alone.
- **Safety:** sensor experiments keep motor physically disabled by default. Motor tests require explicit opt-in, lifted/secured wheels and an independent way to remove 2S VM power. A firmware command/encoder count is not an oscilloscope or external ground truth.
- **Acceptance discipline:** separate compile/CI, simulated workload, MCU telemetry, measured physical movement, electrical verification and closed-loop trajectory performance. Document these as separate statuses.
- **Cleanup discipline:** Retired host debug programs remain archived at `archive/debug-tools-precleanup-20261024`. **Exception restored at user request:** `tools/flash_map_pipeline.py` is active for the guarded Flash map 5x5/16x16 UART1/JDY-33 workflow, paired with `tests/regression/flash_map_protocol_test.py` and `web/ble_debug_console.html`. Do not reintroduce or recommend other retired debug tools unless explicitly requested. Preserve historical `results/` logs and avoid force-push.
- **After a finished workstream:** update `PROJECT_STATUS.md`, the affected module README, links to actual test results, acceptance status and remaining optimizations.

Communication: professional, evidence-based Vietnamese; explain technical terminology when needed. When user requests one automated pipeline, prefer a comprehensive bounded test over repeated individual micro-tests.
