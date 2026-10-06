# Minimal Core Architecture

## Objective

The `giomuadongbac` branch keeps the firmware small enough to audit at register level while preserving the fixed PCB pin mapping.

## Layering

```text
main.c
  |
  | commands / initialization
  v
control.c
  |
  | deterministic 1 kHz scheduling
  v
motion.c
  |
  | control decision
  v
pid.c + motor.c
```

Sensors feed the control loop through their own drivers:

```text
encoder.c -----\
mpu6050.c ------> control.c -> motion.c
ir_sensor.c ----/
```

## Ownership rules

### Rule 1: one peripheral owner

Each hardware peripheral has one software owner.

| Peripheral | Owner |
| --- | --- |
| TIM2 | motor |
| TIM3/TIM4 | encoder |
| TIM5 | system_timer |
| TIM10 | ir_sensor |
| TIM11 | control |
| ADC1 + DMA2 Stream0 | ir_sensor |
| I2C1 | i2c |
| USART2 | bt_debug |

`board.c` does not reconfigure those peripherals. It provides board-wide services such as PCB LED/button initialization, RCC clock queries and HardFault motor shutdown.

### Rule 2: control owns timing, not control law

`control.c` answers **when** periodic work runs. It owns TIM11 and its ISR.

It does not contain straight/turn PID logic.

### Rule 3: motion owns behavior

`motion.c` answers **what the robot should do**. It owns motion state, command transitions and the use of PID outputs to generate motor commands.

### Rule 4: PID is hardware independent

`pid.c` only implements PID mathematics. It receives `dt`; it does not know about TIM11, STM32 registers or interrupts.

### Rule 5: one writer for encoder state

Only `Encoder_Update()` advances encoder totals and speed. All getters are read-only.

### Rule 6: motion commands are atomic

`Motion_CommandStraight()`, `Motion_CommandTurn()` and `Motion_Stop()` protect state transitions against the TIM11 ISR.

### Rule 7: no formatted logging in hard real-time code

TIM11 performs acquisition and control only. `BT_Printf()` runs from the main loop at 10 Hz.

### Rule 8: fixed PCB mapping stays in pinout.h

Drivers read their port/pin/timer assignments from `pinout.h`. The refactor does not remap PCB pins.

## Current execution path

```text
TIM11 IRQ @ 1 kHz
    |
    +-- Encoder_Update(dt)
    +-- MPU6050_Update()
    +-- IR_GetLatest() / IR_StartScan()
    |
    +-- Motion_Update(...)
            |
            +-- PID_Compute(...)
            |
            +-- Motor_SetPair(...)
```

## Reintroduction order

After the baseline is verified on hardware:

1. wall steering
2. calibration
3. binary logger
4. maze/flood-fill
5. persistent storage
6. OLED
7. advanced turns/alignment
8. A* speedrun
