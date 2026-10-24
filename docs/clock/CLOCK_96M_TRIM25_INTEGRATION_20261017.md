# Clock 96 MHz / HSI trim25 — selected integrated firmware profile

Date: 2026-10-17.
Target: `giomuadongbac` (starting SHA `7a266791f1466cabe05ae1a70377923a10a488b7`).
Clock code provenance: `release/mpu6050-i2c-rc2-tim11-20261013` (RC2 tested-source SHA `5219e04d3fe7b8fdb5a092100154e6d9f0317e09`).
Do not merge donor diagnostic main, motor, clock-log archives, or other experimental firmware.
All values in the table below are **nominal RCC-register-derived**, not independent measurements.

## Choice and engineering rationale

On the user's STM32F411 board, prior stationary motor-OFF RC2 A/B trials and HSI characterization favored the nominal 96 MHz, trim25 build for MPU gyro-sampling stability. This is the selected configuration **within the previously tested candidates**, not a proof of universal optimum on other chips/temperatures/voltages.

The firmware default `genericSTM32F411CE` now includes:

```ini
-DCLOCK_VALIDATE_96M=1
-DCLOCK_HSI_TRIM=25
-DMPU_SAMPLING_POLICY_B=1
-DMPU_TIM11_TICK_SCHED=1
```

`src/board.c` includes the RC2 bounded RCC startup, prior to any peripheral
initialization. After switching safely to HSI, clearing PLL, setting trim25,
and waiting for readiness, `PLLM=16, PLLN=192, PLLP=2` selects nominal 96 MHz.
AHB /1, APB1 /2 and APB2 /1. Flash latency set conservatively to 3WS.
`SystemCoreClockUpdate()` and `Board_GetClockFault()` check the internal
configuration at startup. They do **not** measure actual oscillator frequency.

## Peripherals computed from selected clock

| Peripheral | Nominal derived clock | Divider / setting | Target |
|---|---:|---|---|
| SYSCLK / AHB | 96 MHz | PLLM16, PLLN192, PLLP2 | 96 MHz |
| APB1 | 48 MHz | PPRE1 /2 | 48 MHz |
| APB2 | 96 MHz | PPRE2 /1 | 96 MHz |
| TIM2 Motor PWM | 96 MHz | PSC=0, ARR=3199 | 30 kHz |
| TIM5 timestamp | 96 MHz | PSC=95 | 1 MHz |
| TIM11 control loop | 96 MHz | PSC=95, ARR=999 | 1 kHz |
| I2C1 Fast | 48 MHz | duty2 CCR=40 | 400 kHz nominal |
| USART1 debug | 96 MHz | BRR=833 | 115200 baud nominal |
| USART2 BLE | 48 MHz | BRR based on APB1 | 115200 baud nominal |

PWM ARR, TIM5 PSC, TIM11 PSC, UART BRR and I2C1 CCR are **calculated dynamically**
from APB registers. Their nominal values were checked by the offline gate,
not by an oscilloscope. TIM5's +220us compare wake for Policy B remains
lower-priority than TIM11; actual ISR response under motor load remains unmeasured.

## Startup fault containment

- `Board_Init()` configures RCC profile and records a nonzero fault for
  bounded HSI/PLL/voltage/readiness/mismatch failures.
- `SystemTimer_Init()` and `UART_Init()` allow reporting a failure.
- `main()` checks `Board_GetClockFault()` **before calling `Motor_Init()`**.
  On failure it turns the status LED on, prints `BOOT_FAIL,CLOCK,...`, and
  never initializes motor outputs.
- On success `CLOCK_PROFILE,nominal_hz=...,trim=...,hsical=...,pclk1=...,
  pclk2=...,tim2clk=...,tim11clk=...` is output through USART1 PA9.
- Original Motor Driver Rev3 blobs, STOP, 300ms software watchdog and
  HardFault output-disable+GPIO-neutralization remain unchanged.

## Test gate

```powershell
python tools/clock96_integration_gate.py --self-test
python tools/clock96_integration_gate.py --audit
python tools/motor_giomuadongbac_gate.py --self-test
python tools/motor_giomuadongbac_gate.py --audit
python tools/uart_mpu_integration_gate.py
python tests/regression/motor_driver_native_test.py
python -m platformio run -e genericSTM32F411CE
```

The GitHub Actions workflow `clock96-giomuadongbac.yml` combines these
tests. The clock contract has 10 negative mutation tests; it verifies the
frozen Motor source file Git hashes and flags. CI does not flash STM32.

## Hardware acceptance that is still open

1. First confirm the PC UART receives `CLOCK_PROFILE` after reset with the
   **motor supply physically disconnected**, then check the profile fields.
   A nominal `96000000` print is *not* a measured frequency.
2. Independently verify the clock on the target PCB where feasible (the
   prior host-referenced 96MHz/trim25 characterization is specific to that
   tested board/temperature).
3. Confirm gyro warmup/calibration, I2C error counters, and 1kHz TIM11
   freshness on the combined *default* firmware. The RC2 experiments were
   separate motor-OFF bench binaries.
4. Any motor-on EMI/load/interrupt timing and safe-stop electrical checks
   remain separate qualification; a build/host regression is not evidence
   of real-world safety.
5. PLLQ=5 produces 38.4MHz at nominal 96MHz SYSCLK, not 48MHz USB FS.
   Do not reuse this configuration for USB without separate clock design.

**Disposition:** software selected/compiled; live integrated clock on the
actual board not yet confirmed.
