#ifndef INTEGRATION_SWEEP_H
#define INTEGRATION_SWEEP_H
#if defined(INTEGRATION_DIAG)
#include <stdint.h>
#include "mpu6050.h"
typedef enum {
    SWEEP_DISARMED = 0, SWEEP_DRIVE = 1, SWEEP_COAST = 2,
    SWEEP_DONE = 3, SWEEP_FAULT = 4
} SweepState;
typedef enum {
    SWEEP_OK = 0, SWEEP_STOP_REQUEST = 1, SWEEP_IMU_STALE = 2,
    SWEEP_TIM11_DEADLINE = 3, SWEEP_I2C_TIMEOUT = 4,
    SWEEP_MOTOR_WATCHDOG = 5, SWEEP_SESSION_TIMEOUT = 6,
    SWEEP_CLOCK_FAULT = 7, SWEEP_BUTTON_RELEASED = 8
} SweepFault;
typedef struct {
    uint8_t state, fault, group, step;
    uint8_t armed_once, motor_enabled;
    uint16_t duty_permille;
    uint32_t elapsed_ms, stage_id, deadline_ms;
} SweepSnapshot;
void Sweep_Init(void);
uint8_t Sweep_SelectMode(uint8_t motor_on); /* Only while DISARMED; 0=shadow,1=live */

uint8_t Sweep_Arm(uint32_t now_ms, const MPU6050_Handle_t *mpu,
                  uint8_t button_held);
void Sweep_Update(uint32_t now_ms, const MPU6050_Handle_t *mpu);
void Sweep_Abort(uint8_t reason);
void Sweep_ImuFaultFromISR(void);
void Sweep_GetSnapshot(SweepSnapshot *out);
uint8_t Sweep_IsActive(void);
#endif
#endif
