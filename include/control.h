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
#if defined(INTEGRATION_DIAG)
typedef struct {
    uint32_t ticks;
    uint32_t late_gt50us;
    uint32_t gaps_gt1500us;
    uint32_t max_irq_cycles;
    uint32_t max_encoder_cycles;
    uint32_t max_motor_service_cycles;
    uint32_t motor_service_ticks;
    uint32_t gyro_valid_ticks;
    uint32_t gyro_invalid_after_first_valid;
    uint32_t ir_scan_count;
} Control_DebugStats_t;
void Control_GetDebugStats(Control_DebugStats_t *out);
#endif

#endif
