# UART1 / MPU6050 RC2 selectively integrated into giomuadongbac

Target baseline: `291fee927a60c85d0b6ae077965c1ed0653d28cd`.
MPU source: `release/mpu6050-i2c-rc2-tim11-20261013`, validated original source commit `5219e04d3fe7b8fdb5a092100154e6d9f0317e09`.

Copied exact UART1 TX DMA driver (PA9), I2C1 (PB8/PB9) IRQ backend, MPU6050 Policy B, TIM5 retry and their headers. PA10 pin is configured as AF7, but the donor UART implementation is **TX-only**: no RX parser, UART RX interrupt or USART1 commands are available. BLE USART2 remains the command channel.

Target-local changes: main initializes USART1 and calls MPU6050_Update outside TIM11; Control IRQ retains Motor_Update1ms and only signals MPU tick, then reads coherent snapshot; motor commands fail closed on missing or >3ms-old gyro data; default build flags enable Policy B and tick synchronization. Clock selection remains the target's original configuration rather than the previously validated instrumented 96 MHz trim25 profile.

**Hardware motor-on / EMI / waveform / scheduling WCET validation is pending**. RC2 was qualified only in an isolated motor-OFF bench; passing the integration build alone does not qualify powered motors. No pin-state or current verification is implied by this documentation.
