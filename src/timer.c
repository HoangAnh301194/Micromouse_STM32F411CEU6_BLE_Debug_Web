/**
 * @file timer.c
 * @brief Timer module implementation for STM32F411CEU6 Black Pill - Register Level
 * @author Your Name
 * @date 2025
 */

#include "timer.h"

/**
 * @brief Enable clock for timer
 * @param TIMx: Timer pointer
 */
void Timer_ClockEnable(TIM_TypeDef *TIMx)
{
    if (TIMx == TIM1) {
        RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;
    }
    else if (TIMx == TIM2) {
        RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
    }
    else if (TIMx == TIM3) {
        RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;
    }
    else if (TIMx == TIM4) {
        RCC->APB1ENR |= RCC_APB1ENR_TIM4EN;
    }
    else if (TIMx == TIM5) {
        RCC->APB1ENR |= RCC_APB1ENR_TIM5EN;
    }
    else if (TIMx == TIM9) {
        RCC->APB2ENR |= RCC_APB2ENR_TIM9EN;
    }
    else if (TIMx == TIM10) {
        RCC->APB2ENR |= RCC_APB2ENR_TIM10EN;
    }
    else if (TIMx == TIM11) {
        RCC->APB2ENR |= RCC_APB2ENR_TIM11EN;
    }
}

/**
 * @brief Initialize timer in basic mode
 * @param TIMx: Timer pointer
 * @param prescaler: Prescaler value (0-65535)
 * @param period: Auto-reload value
 */
void Timer_Init(TIM_TypeDef *TIMx, uint16_t prescaler, uint32_t period)
{
    /* Enable clock for timer */
    Timer_ClockEnable(TIMx);
    
    /* Disable counter */
    TIMx->CR1 &= ~TIM_CR1_CEN;
    
    /* Set prescaler */
    TIMx->PSC = prescaler;
    
    /* Set auto-reload value */
    TIMx->ARR = period;
    
    /* Configure CR1: */
    /* - Up counting mode (DIR = 0) */
    /* - Edge-aligned mode (CMS = 00) */
    /* - Auto-reload preload enable */
    TIMx->CR1 &= ~(TIM_CR1_DIR | TIM_CR1_CMS_0 | TIM_CR1_CMS_1);
    TIMx->CR1 |= TIM_CR1_ARPE;
    
    /* Generate update event to load prescaler and ARR values */
    TIMx->EGR |= (1 << 0);  /* UG bit */
    
    /* Clear update flag */
    TIMx->SR &= ~TIM_SR_UIF;
}

/**
 * @brief Start timer
 * @param TIMx: Timer pointer
 */
void Timer_Start(TIM_TypeDef *TIMx)
{
    TIMx->CR1 |= TIM_CR1_CEN;
}

/**
 * @brief Stop timer
 * @param TIMx: Timer pointer
 */
void Timer_Stop(TIM_TypeDef *TIMx)
{
    TIMx->CR1 &= ~TIM_CR1_CEN;
}

/**
 * @brief Enable timer update interrupt
 * @param TIMx: Timer pointer
 */
void Timer_EnableInterrupt(TIM_TypeDef *TIMx)
{
    TIMx->DIER |= TIM_DIER_UIE;
}

/**
 * @brief Disable timer update interrupt
 * @param TIMx: Timer pointer
 */
void Timer_DisableInterrupt(TIM_TypeDef *TIMx)
{
    TIMx->DIER &= ~TIM_DIER_UIE;
}

/**
 * @brief Clear timer update interrupt flag
 * @param TIMx: Timer pointer
 */
void Timer_ClearFlag(TIM_TypeDef *TIMx)
{
    TIMx->SR &= ~TIM_SR_UIF;
}

/**
 * @brief Initialize PWM mode
 * @param TIMx: Timer pointer
 * @param channel: PWM channel (1-4)
 * @param prescaler: Prescaler value
 * @param period: PWM period
 * @param pulse: PWM pulse width
 */
void Timer_PWM_Init(TIM_TypeDef *TIMx, uint8_t channel, uint16_t prescaler, uint32_t period, uint32_t pulse)
{
    /* Enable clock for timer */
    Timer_ClockEnable(TIMx);
    
    /* Disable counter */
    TIMx->CR1 &= ~TIM_CR1_CEN;
    
    /* Set prescaler */
    TIMx->PSC = prescaler;
    
    /* Set period (ARR) */
    TIMx->ARR = period;
    
    /* Configure CR1 */
    TIMx->CR1 |= TIM_CR1_ARPE;  /* Auto-reload preload enable */
    
    /* Configure PWM mode for selected channel */
    switch(channel) {
        case 1:
            /* Channel 1: PWM mode 1, Output compare preload enable */
            TIMx->CCMR1 &= ~(0xFF << 0);  /* Clear CC1S and OC1M bits */
            TIMx->CCMR1 |= TIM_CCMR_OC1M_PWM1 | TIM_CCMR_OC1PE;
            
            /* Enable channel 1, set polarity to active high */
            TIMx->CCER |= TIM_CCER_CC1E;
            TIMx->CCER &= ~TIM_CCER_CC1P;
            
            /* Set pulse width */
            TIMx->CCR1 = pulse;
            break;
            
        case 2:
            /* Channel 2: PWM mode 1, Output compare preload enable */
            TIMx->CCMR1 &= ~(0xFF << 8);
            TIMx->CCMR1 |= TIM_CCMR_OC2M_PWM1 | TIM_CCMR_OC2PE;
            
            /* Enable channel 2, set polarity to active high */
            TIMx->CCER |= TIM_CCER_CC2E;
            TIMx->CCER &= ~TIM_CCER_CC2P;
            
            /* Set pulse width */
            TIMx->CCR2 = pulse;
            break;
            
        case 3:
            /* Channel 3: PWM mode 1, Output compare preload enable */
            TIMx->CCMR2 &= ~(0xFF << 0);
            TIMx->CCMR2 |= (0x6 << 4) | (1 << 3);  /* OC3M = PWM1, OC3PE = 1 */
            
            /* Enable channel 3, set polarity to active high */
            TIMx->CCER |= TIM_CCER_CC3E;
            TIMx->CCER &= ~TIM_CCER_CC3P;
            
            /* Set pulse width */
            TIMx->CCR3 = pulse;
            break;
            
        case 4:
            /* Channel 4: PWM mode 1, Output compare preload enable */
            TIMx->CCMR2 &= ~(0xFF << 8);
            TIMx->CCMR2 |= (0x6 << 12) | (1 << 11);  /* OC4M = PWM1, OC4PE = 1 */
            
            /* Enable channel 4, set polarity to active high */
            TIMx->CCER |= TIM_CCER_CC4E;
            TIMx->CCER &= ~TIM_CCER_CC4P;
            
            /* Set pulse width */
            TIMx->CCR4 = pulse;
            break;
    }
    
    /* For TIM1 and TIM8, enable main output */
    if (TIMx == TIM1) {
        TIMx->BDTR |= (1 << 15);  /* MOE bit */
    }
    
    /* Generate update event */
    TIMx->EGR |= (1 << 0);
    
    /* Clear flags */
    TIMx->SR = 0;
    
    /* Start timer */
    TIMx->CR1 |= TIM_CR1_CEN;
}

/**
 * @brief Set PWM duty cycle
 * @param TIMx: Timer pointer
 * @param channel: PWM channel (1-4)
 * @param pulse: PWM pulse width
 */
void Timer_PWM_SetDuty(TIM_TypeDef *TIMx, uint8_t channel, uint32_t pulse)
{
    switch(channel) {
        case 1:
            TIMx->CCR1 = pulse;
            break;
        case 2:
            TIMx->CCR2 = pulse;
            break;
        case 3:
            TIMx->CCR3 = pulse;
            break;
        case 4:
            TIMx->CCR4 = pulse;
            break;
    }
}

/**
 * @brief Set timer counter value
 * @param TIMx: Timer pointer
 * @param value: Counter value
 */
void Timer_SetCounter(TIM_TypeDef *TIMx, uint32_t value)
{
    TIMx->CNT = value;
}

/**
 * @brief Get timer counter value
 * @param TIMx: Timer pointer
 * @return Current counter value
 */
uint32_t Timer_GetCounter(TIM_TypeDef *TIMx)
{
    return TIMx->CNT;
}

/**
 * @brief Create delay using timer (blocking)
 * @param TIMx: Timer pointer
 * @param ms: Delay time in milliseconds
 * 
 * Note: Assumes system clock configured appropriately
 * For 1ms tick with 100MHz clock: prescaler = 10000-1, period = 10-1
 */
void Timer_Delay_ms(TIM_TypeDef *TIMx, uint32_t ms)
{
    uint32_t i;
    
    /* Configure timer for 1ms tick */
    /* Adjust prescaler based on your system clock */
    Timer_Init(TIMx, 9999, 9);  /* Example for 100MHz: (9999+1)*(9+1)/100MHz = 1ms */
    
    for (i = 0; i < ms; i++) {
        TIMx->CNT = 0;
        Timer_Start(TIMx);
        
        /* Wait for update flag */
        while (!(TIMx->SR & TIM_SR_UIF));
        
        /* Clear flag */
        TIMx->SR &= ~TIM_SR_UIF;
    }
    
    Timer_Stop(TIMx);
}
