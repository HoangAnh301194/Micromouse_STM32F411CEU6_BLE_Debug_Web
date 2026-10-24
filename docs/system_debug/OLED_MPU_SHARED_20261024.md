# OLED + MPU6050 on shared I2C1 — acceptance protocol (2026-10-24)

## Scope and safety

- Firmware profile: `oled_mpu_diag_96m` only; production OLED remains disabled.
- Motor driver is hard-locked at boot. No motion/BLE controls are accepted.
- TIM11 priority 0 continues at 1 kHz; MPU6050 runs main + I2C1 EV/ER IRQ, same as the previous Policy B.
- OLED `0x3C` and MPU6050 `0x68` share I2C1 PB8/PB9; OLED startup blocking writes occur only **before** MPU/TIM11 startup.
- The isolated `ir_oled_test_96m` deliberately does not test MPU coexistence.

## Transport design

1. `i2c.c` owns I2C1 and arbitrates gyro versus OLED using a software owner flag; no independent second bus driver.
2. OLED async commands: 4 bytes for page/column position. Data payload: control byte + up to 8 pixels. DMA1 Stream6 TX; I2C1 EV/ER IRQ starts TX and finishes after final BTF. The ISR contains no wait loops.
3. `oled_shared.c` compares 1024-byte framebuffer snapshots against last successfully transferred pixels and sends dirty runs only. New snapshots requested at most once per 200 ms; actual frame-completion rate depends on spare I2C bandwidth.
4. OLED runs only after `MPU6050_Update()` in main, only when gyro transaction is complete, when the current TIM11 sample opportunity has started, and with conservative remaining time until the next 1 ms tick.
5. If MPU probes I2C at below a nominal 300 kHz, OLED never schedules transfers. This is an intentional fail-safe, not a successful OLED test.
6. Avoid blocking `SSD1306_UpdateScreen()` or old `SSD1306_Init_DMA()` after MPU starts. Only `OLED_SharedService()` is approved in shared mode.
7. A timeout/error marks an OLED frame incomplete; the pixel is resent at a future opportunity. Gyro failure recovery remains in main.

## Flash and capture on Windows

```powershell
git checkout giomuadongbac
git pull --ff-only origin giomuadongbac
python -m pip install platformio pyserial
python -m platformio run -e oled_mpu_diag_96m -t upload
python tools/oled_mpu_pipeline.py --port COM5 --baseline-seconds 15 --oled-seconds 15
```

Use the actual USB-to-UART COM port (STM32 PA9 TX -> USB-TTL RX, PA10 RX -> USB-TTL TX, common GND, 3.3 V logic). `--flash` is optional on the pipeline and flashes before capture. Disconnect independent motor VM supply as an additional safety measure.

Host workflow waits for gyro calibration, commands OLED OFF for baseline, then OLED ON while keeping TIM11, IR and gyro active. It receives four 1-Hz frames with matching `seq`: `OLED_C` (control), `OLED_G` (gyro), `OLED_O` (OLED/DMA progress) and `OLED_E` (I2C NACK/BERR/ARLO/OVR and DMA FE/DME/TE failure evidence); missing or malformed frame invalidates final acceptance.

Artifacts: `results/oled_mpu_YYYYMMDD_HHMMSS/uart_raw.log`, `telemetry.csv`, `report.json`.

## Acceptance checks

| Metric | Initial acceptance limit |
|---|---|
| Motor lock | 100% of samples `locked=1` |
| TIM11 | 970–1030 ticks/s both phases; `gap` increment = 0 |
| Gyro freshness | `invalid` increment = 0; sample age <= 3000 us |
| MPU bus | `err` and `to` do not increase |
| MPU sampling | At least 850 successful reads/s in both phases, ON rate >= 97% OFF rate |
| IRQ jitter | OLED ON `late/ticks` <= OFF ratio + 0.005 |
| OLED progress | `chunks`, `frames`, `bytes` and DMA completions increase when enabled |
| OLED bus/DMA faults | `oled_err`, `tx_err`, `tx_to`, `af`, `berr`, `arlo`, `ovr`, `fe`, `dme`, `te` increments = 0 |
| OLED burst | MCU-recorded maximum < 900 us |
| TIM11 ISR worst case | `maxirq` < 48000 core cycles (< 500 us at nominal 96 MHz) |
| Shared I2C recovery | `recoveries` increment = 0 throughout both phases |
| UART integrity | `uart_drop` increment = 0 and no missing/corrupt 3-part sample |

These are conservative *initial* gates, not proof that PWM/IMU latency on a moving robot is safe. Physical 400 kHz SCL waveform, clock rise time, actual OLED FPS, worst-case latency under display churn, sustained long-duration errors and power/noise still require direct hardware investigation. Timeouts without an actual ST-Link + UART + OLED + MPU test are **NOT PASSED**.

## Host-only regression (not hardware evidence)

```powershell
python tools/oled_mpu_pipeline.py --self-test
python -m platformio run -e oled_mpu_diag_96m
```

Build validates toolchain compatibility. Synthetic self-test validates parser and PASS/FAIL logic. Only serial evidence captured from the STM32 may be classified as hardware PASS. Visually confirm OLED characters are correct and stable during the ON phase; a TX completion counter does not prove pixel correctness.

## First physical A/B upload (24/10/2026)

- `results/oled_mpu_20261024_081542`: `INCOMPLETE` because OLED boot `ready=0` (initialization failed; not proof device physically absent).
- `results/oled_mpu_20261024_081626`: `FAIL`: 54 OLED TX started, zero complete, 54 error, zero data chunks/frames. TIM11 = 1000 Hz in both phases; MPU gyro = 1000 Hz OFF versus 992.07 Hz ON, without MPU timeouts, bus recovery, stale samples or tick gaps.
- The old counters do not distinguish I2C AF/NACK from DMA FIFO/direct/transfer errors. New `OLED_E` telemetry provides failure origin; no claim of a fix or hardware PASS without a new measurement. Rebuild/flash the diagnostic before rerunning the updated four-part pipeline.
