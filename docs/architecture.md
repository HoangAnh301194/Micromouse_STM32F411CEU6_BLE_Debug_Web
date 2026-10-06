# Minimal Core Architecture

## Objective

The `giomuadongbac` branch intentionally reduces the firmware to the smallest useful real-time core. Pin assignments remain fixed because they correspond to the existing PCB.

## Ownership

```text
Application/main
    |
    +-- sends motion commands
    +-- BLE debug at low rate
    |
    v
Motion
    |
    +-- owns motion state and PID state
    +-- produces motor command
    |
    v
Drivers
    +-- motor
    +-- encoder
    +-- IR
    +-- MPU6050/I2C
    +-- BLE
    |
    v
Board/registers
```

### Rule 1: one writer for encoder state

Only `Encoder_Update()` advances encoder totals and speed. All encoder getters are read-only.

### Rule 2: motion commands are atomic

`Motion_CommandStraight()`, `Motion_CommandTurn()` and `Motion_Stop()` protect state changes from TIM11. Motion state is published only after all parameters are initialized.

### Rule 3: the ISR does not format logs

TIM11 performs acquisition and control only. `BT_Printf()` runs from the main loop at 10 Hz.

### Rule 4: no OLED in the baseline runtime

MPU6050 is the only active I2C client while moving. OLED code is retained in `legacy/` until an explicit bus scheduling policy is introduced.

### Rule 5: no hard-coded 96 MHz control timers

TIM10 and TIM11 use the RCC-derived APB timer clocks from `board.c`.

## Active modules

| Module | Responsibility |
| --- | --- |
| board | PCB-level GPIO initialization and clock queries |
| motor | TIM2 PWM and TB6612 direction signals |
| encoder | TIM3/TIM4 quadrature counters and speed snapshots |
| ir_sensor | ADC1 + DMA2 Stream0 + TIM10 paired IR scan |
| i2c | I2C1 transaction layer |
| mpu6050 | gyro acquisition, filtering and yaw integration |
| pid | one reusable PID implementation |
| motion | straight/turn state machine and motor control |
| system_timer | TIM5 micros/millis timebase |
| bt_debug | low-rate BLE diagnostics |
| main | initialization, TIM11 scheduler and operator commands |

## Reintroduction order

Do not restore all legacy modules at once. Recommended order:

1. wall steering
2. calibration
3. binary RAM logger / external SPI logger
4. maze map + flood-fill
5. persistent storage
6. OLED
7. advanced turns/alignment
8. A* speedrun

Each feature should be reintroduced only after the previous baseline still passes timing and motion tests.
