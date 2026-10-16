#ifndef INTEGRATION_SERIAL_H
#define INTEGRATION_SERIAL_H
#if defined(INTEGRATION_DIAG)
#include <stdint.h>
#include "mpu6050.h"
/* Main-only bounded ASCII command protocol over USART1 PA9/PA10 115200 8N1.
 * Never called from TIM11, I2C IRQ, or USART RX ISR. */
void IntegrationSerial_Poll(const MPU6050_Handle_t *mpu, uint32_t now_ms);
#endif
#endif
