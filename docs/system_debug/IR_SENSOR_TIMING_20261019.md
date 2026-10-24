# IR six-channel asynchronous acquisition and timing validation

## Scope

This workstream validates that all six ADC channels are acquired through
ADC1/DMA2 Stream0, each paired-emitter phase advances, complete scans reach
the TIM11 1 kHz consumer, and acquisition/interrupt load is bounded.
Distance accuracy, wall detection, calibration, noise, and cross-talk are
outside this phase.

## Runtime flow

- TIM10 advances the acquisition state machine every nominal 100 us.
- ADC1 scans six channels in sequence. DMA transfer completion advances the
  state machine; neither the TIM10 ISR nor the DMA ISR waits for conversion.
- The sensor sequence includes one ambient read and three emitter-pair reads.
- A scan is published only when all four DMA transfers complete. An ADC
  overrun, DMA error, or 500 us DMA deadline miss aborts the partial scan,
  turns all emitters off, and returns to a retryable idle state.
- TIM11 consumes a completed snapshot, records its age, and requests another
  scan. It also retries if a failed scan left the driver idle.
- ADC clock prescaler is configured for PCLK2/4 (nominally 24 MHz with this
  branch's nominal 96 MHz PCLK2). This is a configuration value, not an
  externally measured clock.

## Diagnostic records

Only `INTEGRATION_DIAG` firmware emits the additional records, from main
context:

- `SYS_IR`: cumulative TIM11 control ticks that consumed a new snapshot or
  had no new scan, and latest/max sample age.
- `SYS_IRD`: scan and DMA counts/errors/timeouts/ADC overruns, cumulative and
  maximum scan/DMA durations.
- `SYS_IRC`: TIM10/DMA ISR cycle counts and maxima, plus nominal ADC clock.
- `SYS_IRV`: latest complete scan's ambient ADC values, emitter-on ADC values,
  published values, and sample age, logged once per second. Raw arrays are
  published together and sequence-checked to avoid mixing different scans.

The diagnostic build suppresses the legacy 10 Hz UART debug line and uses a
2 KiB UART TX queue to preserve complete telemetry bursts; production UART
buffer sizing and debug output are unchanged.

Cycle counters are 32-bit cumulative counters and wrap. Compare interval
deltas using unsigned 32-bit subtraction. `ir_isr_cpu_percent_of_nominal_sysclk`
in the host summary counts only measured TIM10 and DMA ISR cycles divided by
nominal 96 MHz capacity; it is not total MCU utilization.

## Build and board test

From the repository root:

```powershell
python tests/regression/ir_sensor_native_test.py
python tools/test_ir_sensor_timing_capture.py
pio run -e genericSTM32F411CE -e integration_diag_motor_off_96m -e integration_diag_uart_96m
pio run -e integration_diag_motor_off_96m -t upload
python tools/ir_sensor_timing_capture.py --port COM7 --seconds 30
```

Use the actual USB-UART port. PA9 connects to adapter RX, PA10 to adapter TX,
and grounds are common; use 3.3 V TTL. Disconnect the motor VM supply
physically during IR testing. The selected profile also locks motor outputs
in firmware. ST-Link performs upload; USART1 is only for telemetry capture.

The capture tool writes `uart.log` and `summary.json` below
`results/ir_timing_<timestamp>/`. A valid timing run must include multiple
complete `SYS_IRD`, `SYS_IRC`, `SYS_IRV`, and `SYS_U` records. It rejects
truncated telemetry records and reports the capture invalid if UART dropped
messages or reported UART errors. Review scan-rate/fresh-tick deltas,
DMA starts vs completions, error counters, sample ages, TIM11 late/gap
counters, MPU/I2C health, and UART drops together. Software/native tests do
not substitute for this board capture.

## Board result: 2026-10-19

Profile `integration_diag_motor_off_96m`, uploaded and verified by ST-Link;
USART1 captured on COM7 at 115200 baud for 30.068 s. The board was run with
motor outputs locked and motor VM physically disconnected. Firmware source:
commit `9c7862ab9cc092295c14cab707bb366582db9433`.

- IR native regression and capture parser unit tests passed; the production,
  motor-off diagnostic, and UART diagnostic PlatformIO profiles all built.
- 29,000 scans completed in the 29,000 ms counter interval (1,000 Hz); all
  29,000 control ticks consumed a fresh sample. Maximum sample age: 293 us.
- Mean scan duration: 723.71 us; maximum: 724 us. All 116,000 DMA transfers
  completed (four per scan), with mean 22 us and maximum 23 us.
- No DMA errors, DMA timeouts, ADC overruns, TIM11 late/gap events, MPU/I2C
  errors/timeouts, or UART drops. Measured IR ISR cycles were 1.5965% of the
  nominal 96 MHz cycle budget; TIM10 and DMA ISR maxima were 162 and 299
  cycles respectively.
- The configured ADC clock was 24 MHz nominal (PCLK2/4). UART TX queue peak
  was 1,049 of 2,047 usable bytes. Thirty complete six-channel raw-value
  records were logged. Motor lock was asserted and all recorded applied
  motor outputs were zero.

Raw log: [uart.log](../../results/ir_timing_20261019_173447/uart.log);
computed values: [summary.json](../../results/ir_timing_20261019_173447/summary.json).
This is an acquisition/timing smoke test only; it is not an accuracy,
distance, noise, or wall-detection qualification.
