#include "control.h"

#include "board.h"
#include "encoder.h"
#include "ir_sensor.h"
#include "motion.h"
#include "motor.h"
#include "system_timer.h"
#include "stm32f4xx.h"

static MPU6050_Handle_t *control_mpu = 0;
static volatile uint8_t control_running = 0;
static volatile IR_Data_t ir_latest;

void Control_Init(MPU6050_Handle_t *mpu_handle)
{
    uint32_t timer_clock;
    uint32_t prescaler;
    uint32_t ticks_per_period;

    control_mpu = mpu_handle;
    control_running = 0;

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

    if ((TIM11->SR & TIM_SR_UIF) == 0U) {
        return;
    }

    TIM11->SR &= ~TIM_SR_UIF;

    if (!control_running || control_mpu == 0) {
        return;
    }

#if defined(MPU_TIM11_TICK_SCHED)
    /* RC2: publish only a sample-opportunity sequence/timestamp.
     * No I2C, busy-wait, UART formatting or retry in the TIM11 ISR. */
    MPU6050_NotifyControlTick(control_mpu, micros());
#endif

    /*
     * Hard real-time path:
     * 1. Update encoder snapshot.
     * 2. Service the Motor Driver's 300-tick command lease and PWM ramp.
     * 3. Notify RC2 acquisition opportunity above (I2C remains in main).
     * 4. Consume a completed IR scan and schedule the next.
     * 5. Execute Motion only with a fresh, valid MPU snapshot.
     */
    Encoder_Update(CONTROL_DT_S);
    /* Motor RC: service every TIM11 1ms tick even if MPU/IR data is stale.
     * The fresh Motion_SetPair on valid feedback renews the 300-tick lease.
     * Do NOT use Motor_KeepAlive here; a stalled Motion must time out. */
    Motor_Update1ms();

    if (IR_GetLatest(&sample)) {
        uint8_t i;

        for (i = 0; i < IR_SENSOR_COUNT; i++) {
            ir_latest.value[i] = sample.value[i];
        }
        ir_latest.scan_count = sample.scan_count;

        IR_StartScan();
    }

    {
        MPU6050_Snapshot_t imu;
        /* Fail closed: an invalid/uncalibrated/>3ms-old snapshot must
         * not renew the Motor Driver's 300ms command lease. */
        if (MPU6050_GetSnapshot(control_mpu, &imu) &&
            (uint32_t)(micros() - imu.completed_us) <= 3000U) {
            Motion_Update(CONTROL_DT_S,
                          Encoder_GetLeftCount(),
                          Encoder_GetRightCount(),
                          imu.yaw_deg,
                          imu.gyro_z_dps);
        } else {
            Motion_Stop();
        }
    }
}
