#include "control.h"

#include "board.h"
#include "encoder.h"
#include "ir_sensor.h"
#include "motion.h"
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

    /*
     * Hard real-time path:
     * 1. Update encoder snapshot.
     * 2. Advance MPU6050 DMA/update state.
     * 3. Consume a completed IR scan and immediately schedule the next scan.
     * 4. Execute the motion controller using one coherent sensor snapshot.
     */
    Encoder_Update(CONTROL_DT_S);
    MPU6050_Update(control_mpu);

    if (IR_GetLatest(&sample)) {
        uint8_t i;

        for (i = 0; i < IR_SENSOR_COUNT; i++) {
            ir_latest.value[i] = sample.value[i];
        }
        ir_latest.scan_count = sample.scan_count;

        IR_StartScan();
    }

    Motion_Update(CONTROL_DT_S,
                  Encoder_GetLeftCount(),
                  Encoder_GetRightCount(),
                  MPU6050_GetYaw(control_mpu),
                  MPU6050_GetGyroZ(control_mpu));
}
