# Six-channel IR sensor live OLED viewer — 2026-10-24

## Purpose
An opt-in, **motor-locked** firmware profile for quick physical checks of
six infrared wall sensors using the existing driver in `src/ir_sensor.c`
and the existing SSD1306 128x64 OLED + Font_7x10 driver.

The production firmware remains unchanged. This is **not** acceptance of
IR calibration, ADC linearity, wall-distance measurement or sensor
coexistence with an active MPU6050 on the shared I2C1 bus.

## Flash from PowerShell
```powershell
git switch feature/3v3-i2c-oled-debug
git pull --ff-only origin feature/3v3-i2c-oled-debug
python -m platformio run -c platformio_legacy.ini -e ir_oled_test_96m
python -m platformio run -c platformio_legacy.ini -e ir_oled_test_96m -t upload
```

Use ST-Link; the IR/OLED test profile's `main` will be linked by
`-DIR_OLED_TEST=1`. To restore normal robot firmware:
```powershell
python -m platformio run -e genericSTM32F411CE -t upload
```

## Electrical/peripheral paths
- STM32F411 CEU6 @ nominal 96MHz clock, HSI trim25.
- **IR receivers:** PB1=L90, PB0=L45, PA6=L0, PA5=R0, PA4=R45, PA1=R90.
- **IR emitters:** PA7=L90, PB10=L45, PB2=L0, PA8=R0, PA11=R45, PA12=R90.
- **Sampling:** existing TIM10 sequencer + ADC1/DMA2_Stream0. For each scan
  the code first captures ambient-light ADC values with emitters OFF,
  then lights emitter pairs and computes
  `value = max(emitter_ADC - ambient_ADC, 0)`.
- **OLED:** existing SSD1306 library on PB8/PB9 via I2C1, 128x64 pixels.
  Diagnostic probes both 7-bit addresses 0x3C and 0x3D at nominal 400kHz;
  if neither ACKs, it reinitializes I2C1 at 100kHz and probes both again.
  The detected address is selected before SSD1306 initialization. Full-frame
  refresh is blocking and limited to 5Hz. **Do not call MPU6050_Init** in this
  isolated profile; OLED is its sole I2C1 client, and this does not qualify
  fully shared-bus operation.
- **Motor:** `Motor_Init` then permanent `Motor_LockOutputs`. There
  is no `Control_Start`, speed PID, Motion commands, USART2 BLE control
  or motor-on state machine. Prefer independent VM power disconnect
  during sensor experimentation.
- **UART1:** text stream at 115200 baud, PA9 TX / PA10 RX. Boot diagnostics:
  `OLED_PROBE,400K,3C=...,3D=...`, optional `OLED_PROBE,100K,...`, then
  `OLED_BOOT,status=...,code=...,ready=...,addr=...,scl_hz=...,retry_100k=...,pb8=...,pb9=...`.
  I2C status names are OK, ERROR, BUSY, TIMEOUT and NACK. SSD1306 status is
  OK, ERROR or NOT_FOUND. `pb8`/`pb9` are sampled idle pin levels (normally
  both high). After boot, `IR6,...` sensor telemetry is emitted at 5Hz.
  No Python host scripts are required.

## OLED screen layout
```text
IR 6CH / RAW ADC
90 L:1234 R:1234
45 L:1234 R:1234
00 L:1234 R:1234
N:12345 age:10ms
```
Values are ADC counts **after ambient subtraction**, nominally
0–4095. They are not millimetres. The screen may show
`WAIT IR SCAN` before any successful result or after the last
snapshot becomes older than 500ms. The boot screen first displays `OLED LINK OK`, the selected address and
configured SCL rate, then switches to the live IR view. If neither address
ACKs at either rate, `OLED_BOOT` reports `NOT_FOUND`; inspect 3.3V, GND,
SCL/SDA, idle pin levels and module address. If an address ACKs but init
returns `ERROR`, investigate command/data transfer integrity or panel
compatibility. IR sampling and UART logging continue when OLED is absent.

## Qualifying a sensor
Place a known target at repeatable distances and angles in front of
each IR emitter/receiver. Compare sensor channel changes and check
repeatability and ambient/background sensitivity. Don't convert ADC
counts to wall distance without collecting a channel-specific
calibration curve and hysteresis/noise characterization.

**CI:** Production build, `ir_oled_test_96m` build and existing
native sensor regressions confirm source compatibility only.
Actual OLED and sensor observations require a hardware run.
