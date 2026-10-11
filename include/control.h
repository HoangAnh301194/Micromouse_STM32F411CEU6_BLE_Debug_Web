#ifndef CONTROL_H
#define CONTROL_H

#include "mpu6050.h"
#include <stdint.h>

#define CONTROL_FREQUENCY_HZ  1000UL
#define CONTROL_DT_S          (1.0f / (float)CONTROL_FREQUENCY_HZ)

void Control_Init(MPU6050_Handle_t *mpu_handle);
/* Diagnostics: timer ticks only, skip encoder/motor/IR/motion processing. */
void Control_SetDiagnosticOnly(uint8_t enable);
void Control_Start(void);
void Control_Stop(void);
uint8_t Control_IsRunning(void);

/* Total number of serviced TIM11 control cycles, sampled from main loop. */
uint32_t Control_GetTickCount(void);

/* Observed TIM11 timing (not a formal worst-case execution-time proof).
 * Period timestamps use TIM5 micros(), including WFI sleep time.
 * Handler duration uses ARM DWT CYCCNT and excludes other-priority interrupts.
 */
typedef struct {
    uint32_t periods;
    uint32_t period_min_us;
    uint32_t period_max_us;
    uint32_t late_over_50us;
    uint32_t gap_over_1500us;
    uint32_t exec_max_cycles;
    uint32_t core_hz;
} Control_Timing_t;

void Control_GetTiming(Control_Timing_t *out);


#endif
