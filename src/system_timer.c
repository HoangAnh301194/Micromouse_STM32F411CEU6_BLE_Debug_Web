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
