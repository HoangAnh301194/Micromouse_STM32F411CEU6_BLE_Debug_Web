/* TIM5 1MHz timestamp clock, derived from actual APB1 timer clock.
 * Avoid hardcoded HSE values and duplicate SYSCLK/PLL math after 100MHz init.
 */
#include "system_timer.h"
#include "board.h"
#include "stm32f4xx.h"

void SystemTimer_Init(void)
{
    uint32_t timer_hz = Board_GetAPB1TimerClockHz();
    uint32_t divider = timer_hz / 1000000UL;
    if (divider == 0U) divider = 1U;

    RCC->APB1ENR |= RCC_APB1ENR_TIM5EN;
    TIM5->CR1 &= ~TIM_CR1_CEN;
    TIM5->PSC = divider - 1U;
    TIM5->ARR = 0xFFFFFFFFUL;
    TIM5->CNT = 0U;
    TIM5->EGR = TIM_EGR_UG;
    TIM5->SR = 0U;
    TIM5->CR1 |= TIM_CR1_CEN;
}

uint32_t micros(void) { return TIM5->CNT; }
uint32_t millis(void) { return TIM5->CNT / 1000UL; }

void delay_us_blocking(uint32_t us)
{
    uint32_t start = micros();
    while ((uint32_t)(micros() - start) < us) {}
}

void delay_ms_blocking(uint32_t ms)
{
    while (ms--) delay_us_blocking(1000UL);
}

#if defined(MPU_SAMPLING_POLICY_B)
/* TIM5 remains the 32-bit free-running 1MHz timestamp source.
 * This channel-1 compare interrupt is a ONE-SHOT WFI wake for main. It never
 * reads I2C or modifies the 1kHz TIM11 scheduler. Priority 14 is below
 * TIM11 (0) and I2C event IRQs. The experiment runs motor OFF only. */
void SystemTimer_ArmWakeAfterUs(uint32_t delay_us)
{
    if (delay_us < 100U || delay_us > 500U) return;
    TIM5->DIER &= ~TIM_DIER_CC1IE;
    TIM5->SR &= ~TIM_SR_CC1IF;
    TIM5->CCR1 = TIM5->CNT + delay_us;
    NVIC_ClearPendingIRQ(TIM5_IRQn);
    NVIC_SetPriority(TIM5_IRQn, 14U);
    NVIC_EnableIRQ(TIM5_IRQn);
    TIM5->DIER |= TIM_DIER_CC1IE;
}

void SystemTimer_CancelWake(void)
{
    TIM5->DIER &= ~TIM_DIER_CC1IE;
    TIM5->SR &= ~TIM_SR_CC1IF;
}

void TIM5_IRQHandler(void)
{
    if (TIM5->SR & TIM_SR_CC1IF) {
        TIM5->DIER &= ~TIM_DIER_CC1IE;
        TIM5->SR &= ~TIM_SR_CC1IF;
    }
}
#endif /* MPU_SAMPLING_POLICY_B */
