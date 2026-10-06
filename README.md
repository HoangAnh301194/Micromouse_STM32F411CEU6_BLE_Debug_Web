# Micromouse STM32F411CEU6 - Minimal Core

This branch (`giomuadongbac`) is a clean baseline rebuilt from the original firmware.

The PCB pin mapping is intentionally unchanged. The goal of this branch is to make the real-time path small enough to audit and test before maze solving, OLED, calibration workflows and advanced motion features are added back.

## Active runtime

```text
main.c
├── board.c
├── motor.c
├── encoder.c
├── ir_sensor.c
├── i2c.c
├── mpu6050.c
├── pid.c
├── motion.c
├── system_timer.c
└── bt_debug.c
```

The control path is:

```text
TIM11 @ 1 kHz
    ├── Encoder_Update()
    ├── MPU6050_Update()
    ├── collect completed IR scan
    └── Motion_Update()
            └── Motor_SetPair()
```

BLE formatting and telemetry are executed in the main loop at 10 Hz, not in the control ISR.

## Fixed PCB resources

`include/pinout.h` is preserved from `main`.

- TIM2 CH1/CH2: right/left motor PWM
- TIM3: right quadrature encoder
- TIM4: left quadrature encoder
- TIM5: microsecond timebase
- TIM10: IR scan state machine
- TIM11: 1 kHz control loop
- I2C1 PB8/PB9: MPU6050
- USART2 PA2/PA3: JDY-33 BLE

TIM10 and TIM11 now derive their prescalers from the current RCC clock configuration instead of assuming a fixed 96 MHz timer clock.

## Minimal motion scope

Only three states are active:

- `MOTION_IDLE`
- `MOTION_STRAIGHT`
- `MOTION_TURN`

Advanced features are intentionally excluded until this baseline is verified on the real PCB.

BLE commands:

```text
F  straight 180 mm
L  turn left 90 deg
R  turn right 90 deg
S  immediate stop
Z  zero gyro yaw (idle only)
```

## Legacy code

Previous high-level modules are retained under `legacy/` and are not compiled by PlatformIO.

Examples:

- maze/flood-fill/A*
- OLED/fonts
- IR calibration workflow
- flash persistence
- system test menu
- back alignment
- previous motion controller
- previous sensor fusion
- generic hardware wrapper
- USART1 wrapper
- old timer abstraction

The `main` branch remains the complete reference implementation.

## Build

```bash
git fetch origin
git switch giomuadongbac

pio run
```

Upload with ST-Link:

```bash
pio run -t upload
```

Clean rebuild:

```bash
pio run -t clean
pio run
```

## Bring-up order

1. Power the PCB with wheels lifted.
2. Confirm boot message over JDY-33.
3. Send `S` and confirm both PWM channels are zero.
4. Verify encoder signs by rotating both wheels forward by hand.
5. Verify MPU yaw/gyro telemetry while rotating the robot.
6. Verify all six IR values with a wall/object.
7. Test `F` with wheels lifted, then on the floor at low speed.
8. Test `L` and `R`.
9. Only after these tests pass, reintroduce wall control and navigation.

See `docs/architecture.md` for module ownership rules.
