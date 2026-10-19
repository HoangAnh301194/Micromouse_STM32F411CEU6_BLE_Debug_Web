#ifndef SOAK_DIAG_H
#define SOAK_DIAG_H
#include <stdint.h>
#include "mpu6050.h"
#if defined(DIAG_SOAK_180S)
/* Main-owned 180s diagnostic: 60Hz compact telemetry, 1Hz aggregated stats.
 * PID stress executes from TIM11 only in DIAG_SOAK_PID builds.
 * Live motor mode is separate and requires explicit UART arm+start. */
void Soak_Init(void);
uint8_t Soak_ArmLive(const MPU6050_Handle_t *mpu);
uint8_t Soak_Start(MPU6050_Handle_t *mpu, uint32_t now_ms);
void Soak_Heartbeat(uint32_t now_ms);
void Soak_Stop(uint8_t reason);
void Soak_Poll(MPU6050_Handle_t *mpu, uint32_t now_ms);
void Soak_PidTick(void);
uint8_t Soak_IsActive(void);
uint8_t Soak_HasStarted(void);
#endif
#endif
