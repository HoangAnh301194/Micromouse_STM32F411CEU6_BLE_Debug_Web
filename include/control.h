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

#if defined(MPU_FRESHNESS_DIAG)
/* Diagnostic-only: count TIM11 cycles with the same accepted gyro sample.
 * age_us is measured since MCU completed I2C gyro read, not sensor acquisition.
 * No motor/encoder/IR code is enabled by this statistics interface. */
typedef struct {
    uint32_t control_ticks;
    uint32_t reused_sample_ticks;
    uint32_t age_over1000_ticks;
    uint32_t age_over1500_ticks;
    uint32_t age_max_us;
    /* Approximate percentile upper bounds based on 250us histogram bins.
     * Last bin represents >=1750us, so returns 1750us as a lower bound. */
    uint32_t age_p95_bin_us;
    uint32_t age_p99_bin_us;
} Control_ImuSync_t;
void Control_GetImuSync(Control_ImuSync_t *out);
#endif


#endif
