#ifndef CONTROL_H
#define CONTROL_H

#include "mpu6050.h"
#include <stdint.h>

#define CONTROL_FREQUENCY_HZ  1000UL
#define CONTROL_DT_S          (1.0f / (float)CONTROL_FREQUENCY_HZ)

void Control_Init(MPU6050_Handle_t *mpu_handle);
void Control_Start(void);
void Control_Stop(void);
uint8_t Control_IsRunning(void);

/* Total number of serviced TIM11 control cycles, sampled from main loop. */
uint32_t Control_GetTickCount(void);

#endif
