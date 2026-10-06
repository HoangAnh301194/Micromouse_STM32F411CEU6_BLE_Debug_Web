# Micromouse STM32F411CEU6 - Minimal Core

This branch (`giomuadongbac`) is a clean baseline rebuilt from the original firmware.

The PCB pin mapping is intentionally unchanged. The goal is to keep the real-time path small, explicit and testable before maze solving, OLED, calibration workflows and advanced motion features are added back.

## Active runtime

```text
main.c
├── board.c
├── control.c
├── motion.c
├── pid.c
├── motor.c
├── encoder.c
├── ir_sensor.c
├── mpu6050.c
├── i2c.c
├── system_timer.c
└── bt_debug.c
```

Module roles:

```text
main.c       = initialization + application commands + low-rate telemetry
control.c    = TIM11 owner + 1 kHz scheduler/ISR
motion.c     = motion state machine + controller
pid.c        = reusable PID mathematics
drivers      = peripheral-specific register configuration
```

The real-time path is:

```text
TIM11 @ 1 kHz
      |
      v
control.c
├── Encoder_Update()
├── MPU6050_Update()
├── consume/restart IR scan
└── Motion_Update()
        └── PID_Compute()
              └── Motor_SetPair()
```

BLE formatting and telemetry execute in the main loop at 10 Hz, never in the TIM11 ISR.

## Peripheral ownership

- TIM2 CH1/CH2 -> `motor.c`
- TIM3/TIM4 -> `encoder.c`
- TIM5 -> `system_timer.c`
- TIM10 -> `ir_sensor.c`
- TIM11 -> `control.c`
- ADC1 + DMA2 Stream0 -> `ir_sensor.c`
- I2C1 -> `i2c.c`
- MPU6050 logic -> `mpu6050.c`
- USART2/JDY-33 -> `bt_debug.c`

`include/pinout.h` remains the fixed PCB wiring definition and is not changed by this refactor.

TIM10 and TIM11 derive their prescalers from the current RCC clock configuration rather than assuming a fixed 96 MHz timer clock.

## Minimal motion scope

Only three motion states are active:

- `MOTION_IDLE`
- `MOTION_STRAIGHT`
- `MOTION_TURN`

BLE commands:

```text
F  straight 180 mm
L  turn left 90 deg
R  turn right 90 deg
S  immediate stop
Z  zero gyro yaw (idle only)
```

## Build

```powershell
git fetch origin
git switch giomuadongbac
git pull origin giomuadongbac

python -m platformio run -t clean
python -m platformio run
```

Upload with ST-Link:

```powershell
python -m platformio run -t upload
```

## Bring-up order

1. Verify MPU6050/I2C and yaw output.
2. Verify encoder sign/count.
3. Verify motor direction and PWM with wheels lifted.
4. Verify TIM11 control-loop timing.
5. Verify IR scanning.
6. Test straight motion at low speed.
7. Test left/right turns.
8. Reintroduce wall control and navigation only after the baseline is stable.

See `docs/architecture.md` for ownership rules.
