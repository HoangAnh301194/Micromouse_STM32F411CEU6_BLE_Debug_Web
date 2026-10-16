#include "control.h"

#include "board.h"
#include "encoder.h"
#include "ir_sensor.h"
#include "motion.h"
#include "motor.h"
#include "stm32f4xx.h"
#include "system_timer.h"
#if defined(INTEGRATION_DIAG)
#include "integration_sweep.h"
#endif

static MPU6050_Handle_t *control_mpu = 0;
static volatile uint8_t control_running = 0;
static volatile IR_Data_t ir_latest;
#if defined(INTEGRATION_DIAG)
static volatile Control_DebugStats_t di_stats;
static volatile uint32_t di_prev_irq_us;
static volatile uint8_t di_seen_valid;
void Control_GetDebugStats(Control_DebugStats_t *out)
{
    uint32_t primask;
    if (!out) return;
    primask = __get_PRIMASK();
    __disable_irq();
    out->ticks = di_stats.ticks;
    out->late_gt50us = di_stats.late_gt50us;
    out->gaps_gt1500us = di_stats.gaps_gt1500us;
    out->max_irq_cycles = di_stats.max_irq_cycles;
    out->max_encoder_cycles = di_stats.max_encoder_cycles;
    out->max_motor_service_cycles = di_stats.max_motor_service_cycles;
    out->motor_service_ticks = di_stats.motor_service_ticks;
    out->gyro_valid_ticks = di_stats.gyro_valid_ticks;
    out->gyro_invalid_after_first_valid = di_stats.gyro_invalid_after_first_valid;
    out->ir_scan_count = di_stats.ir_scan_count;
    if (!primask) __enable_irq();
}
#endif

void Control_Init(MPU6050_Handle_t *mpu_handle)
{
    uint32_t timer_clock;
    uint32_t prescaler;
    uint32_t ticks_per_period;

    control_mpu = mpu_handle;
    control_running = 0;
#if defined(INTEGRATION_DIAG)
    di_stats = (Control_DebugStats_t){0};
    di_prev_irq_us = 0U;
    di_seen_valid = 0U;
    SystemCoreClockUpdate();
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0U;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
#endif

    timer_clock = Board_GetAPB2TimerClockHz();

    /* Generate a 1 MHz timer base, then divide to the requested control rate. */
    prescaler = timer_clock / 1000000UL;
    if (prescaler == 0U) {
        prescaler = 1U;
    }

    ticks_per_period = 1000000UL / CONTROL_FREQUENCY_HZ;
    if (ticks_per_period == 0U) {
        ticks_per_period = 1U;
    }

    RCC->APB2ENR |= RCC_APB2ENR_TIM11EN;

    TIM11->CR1 = 0;
    TIM11->PSC = prescaler - 1U;
    TIM11->ARR = ticks_per_period - 1U;
    TIM11->CNT = 0;
    TIM11->EGR = TIM_EGR_UG;
    TIM11->SR = 0;
    TIM11->DIER = TIM_DIER_UIE;

    NVIC_SetPriority(TIM1_TRG_COM_TIM11_IRQn, 0);
    NVIC_EnableIRQ(TIM1_TRG_COM_TIM11_IRQn);
}

void Control_Start(void)
{
    if (control_mpu == 0) {
        return;
    }

    TIM11->CNT = 0;
    TIM11->SR &= ~TIM_SR_UIF;
    control_running = 1U;
    TIM11->CR1 |= TIM_CR1_CEN;
}

void Control_Stop(void)
{
    /*
     * Stopping the scheduler while a motion command is active must also
     * force the motor command to zero; otherwise the last PWM value could
     * remain latched after TIM11 stops.
     */
    Motion_Stop();
    TIM11->CR1 &= ~TIM_CR1_CEN;
    control_running = 0U;
}

uint8_t Control_IsRunning(void)
{
    return control_running;
}

void TIM1_TRG_COM_TIM11_IRQHandler(void)
{
    IR_Data_t sample;
#if defined(INTEGRATION_DIAG)
    uint32_t entry_us, start_cycles, period, section_cycles;
#endif

    if ((TIM11->SR & TIM_SR_UIF) == 0U) {
        return;
    }

    TIM11->SR &= ~TIM_SR_UIF;

    if (!control_running || control_mpu == 0) {
        return;
    }

#if defined(INTEGRATION_DIAG)
    start_cycles = DWT->CYCCNT;
    entry_us = micros();
    di_stats.ticks++;
    if (di_prev_irq_us) {
        period = entry_us - di_prev_irq_us;
        if (period < 950U || period > 1050U) di_stats.late_gt50us++;
        if (period > 1500U) di_stats.gaps_gt1500us++;
    }
    di_prev_irq_us = entry_us;
#endif
    /*
     * Hard real-time path:
     * 1. Update encoder snapshot.
     * 2. Service the Motor Driver's 300-tick command lease and PWM ramp.
     * 3. Advance MPU6050 DMA/update state.
     * 4. Consume a completed IR scan and immediately schedule the next scan.
     * 5. Execute the motion controller using one coherent sensor snapshot.
     */
#if defined(INTEGRATION_DIAG)
    section_cycles = DWT->CYCCNT;
#endif
    Encoder_Update(CONTROL_DT_S);
#if defined(INTEGRATION_DIAG)
    {
        uint32_t cycles = DWT->CYCCNT - section_cycles;
        if (cycles > di_stats.max_encoder_cycles)
            di_stats.max_encoder_cycles = cycles;
    }
    section_cycles = DWT->CYCCNT;
#endif
    /* Motor RC: service every TIM11 1ms tick even if MPU/IR data is stale.
     * The fresh Motion_SetPair on valid feedback renews the 300-tick lease.
     * Do NOT use Motor_KeepAlive here; a stalled Motion must time out. */
    Motor_Update1ms();
#if defined(INTEGRATION_DIAG)
    {
        uint32_t cycles = DWT->CYCCNT - section_cycles;
        if (cycles > di_stats.max_motor_service_cycles)
            di_stats.max_motor_service_cycles = cycles;
        di_stats.motor_service_ticks++;
    }
#endif
    /* Policy B: only notify tick here; never run I2C in TIM11. */
#if defined(MPU_TIM11_TICK_SCHED)
    MPU6050_NotifyControlTick(control_mpu, micros());
#endif

    if (IR_GetLatest(&sample)) {
        uint8_t i;

        for (i = 0; i < IR_SENSOR_COUNT; i++) {
            ir_latest.value[i] = sample.value[i];
        }
        ir_latest.scan_count = sample.scan_count;
#if defined(INTEGRATION_DIAG)
        di_stats.ir_scan_count = sample.scan_count;
#endif

        IR_StartScan();
    }

    {
        MPU6050_Snapshot_t gyro;
        if (MPU6050_GetSnapshot(control_mpu, &gyro) &&
            (uint32_t)(micros() - gyro.completed_us) <= 3000U) {
#if defined(INTEGRATION_DIAG)
            di_seen_valid = 1U;
            di_stats.gyro_valid_ticks++;
#endif
            Motion_Update(CONTROL_DT_S, Encoder_GetLeftCount(),
                          Encoder_GetRightCount(), gyro.yaw_deg, gyro.gyro_z_dps);
        } else {
#if defined(INTEGRATION_DIAG)
            if (di_seen_valid) di_stats.gyro_invalid_after_first_valid++;
#if defined(INTEGRATION_DIAG_MOTOR_ON)
            Sweep_ImuFaultFromISR();
#endif
#endif
            Motion_Stop(); /* Invalid, uncalibrated or stale gyro. */
        }
    }
#if defined(INTEGRATION_DIAG)
    {
        uint32_t cycles = DWT->CYCCNT - start_cycles;
        if (cycles > di_stats.max_irq_cycles) di_stats.max_irq_cycles = cycles;
    }
#endif
}
