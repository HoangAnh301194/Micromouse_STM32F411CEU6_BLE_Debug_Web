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
git pull --rebase origin giomuadongbac
python -m platformio run -e ir_oled_test_96m -t upload
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
- **OLED:** existing SSD1306 library on PB8/PB9 via I2C1, address 0x3C,
  128x64 pixels. Blocks during full-frame refresh; refresh at 5Hz maximum.
  **Do not call MPU6050_Init** in this isolated profile; OLED is its
  sole I2C1 client, and it does not exercise a fully shared I2C bus.
- **Motor:** `Motor_Init` then permanent `Motor_LockOutputs`. There
  is no `Control_Start`, speed PID, Motion commands, USART2 BLE control
  or motor-on state machine. Prefer independent VM power disconnect
  during sensor experimentation.
- **UART1:** optional text stream at 115200 baud, PA9 TX / PA10 RX:
  `IR6,ms=<ms>,n=<scan>,l90=...,l45=...,l0=...,r0=...,r45=...,r90=...`
  emitted at most 5Hz. No Python host scripts required.

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
snapshot becomes older than 500ms. If the OLED does not acknowledge
at 0x3C, UART shows `IR_OLED,BOOT,...oled=0`; sampling and UART
logging continue. Check 3.3V, GND, SCL/SDA and address rather than
mistaking an OLED wiring problem for an IR failure.

## Qualifying a sensor
Place a known target at repeatable distances and angles in front of
each IR emitter/receiver. Compare sensor channel changes and check
repeatability and ambient/background sensitivity. Don't convert ADC
counts to wall distance without collecting a channel-specific
calibration curve and hysteresis/noise characterization.

**CI:** Production build, `ir_oled_test_96m` build and existing
native sensor regressions confirm source compatibility only.
Actual OLED and sensor observations require a hardware run.
