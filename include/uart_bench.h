#ifndef UART_BENCH_H
#define UART_BENCH_H
#if defined(UART_PROFILE_DIAG)
#include <stdint.h>
#include "mpu6050.h"
/* Compile-time motor-disabled diagnostic only, never in production.
 * UART_Bench_SetRate(0) cancels the source. All APIs main-context only. */
void UART_Bench_Init(void);
uint8_t UART_Bench_SetRate(uint16_t hz, uint32_t now_ms,
                           const MPU6050_Handle_t *mpu);
void UART_Bench_Poll(uint32_t now_ms, const MPU6050_Handle_t *mpu);
void UART_Bench_PrintStatus(uint32_t now_ms, const MPU6050_Handle_t *mpu);
#endif
#endif
