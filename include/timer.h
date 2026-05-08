/**
 * @file timer.h
 * @brief Timer module for STM32F411CEU6 Black Pill - Register Level
 * @author HoangAnh
 * @date 2025
 */

#ifndef TIMER_H
#define TIMER_H

#include <stdint.h>

/* Timer Base Addresses */
#define TIM1_BASE   0x40010000UL
#define TIM2_BASE   0x40000000UL
#define TIM3_BASE   0x40000400UL
#define TIM4_BASE   0x40000800UL
#define TIM5_BASE   0x40000C00UL
#define TIM9_BASE   0x40014000UL
#define TIM10_BASE  0x40014400UL
#define TIM11_BASE  0x40014800UL

/* RCC Base Address */
#define RCC_BASE    0x40023800UL

/* Timer Register Structure */
typedef struct {
    volatile uint32_t CR1;      // Control register 1
    volatile uint32_t CR2;      // Control register 2
    volatile uint32_t SMCR;     // Slave mode control register
    volatile uint32_t DIER;     // DMA/Interrupt enable register
    volatile uint32_t SR;       // Status register
    volatile uint32_t EGR;      // Event generation register
    volatile uint32_t CCMR1;    // Capture/Compare mode register 1
    volatile uint32_t CCMR2;    // Capture/Compare mode register 2
    volatile uint32_t CCER;     // Capture/Compare enable register
    volatile uint32_t CNT;      // Counter
    volatile uint32_t PSC;      // Prescaler
    volatile uint32_t ARR;      // Auto-reload register
    volatile uint32_t RCR;      // Repetition counter register
    volatile uint32_t CCR1;     // Capture/Compare register 1
    volatile uint32_t CCR2;     // Capture/Compare register 2
    volatile uint32_t CCR3;     // Capture/Compare register 3
    volatile uint32_t CCR4;     // Capture/Compare register 4
    volatile uint32_t BDTR;     // Break and dead-time register
    volatile uint32_t DCR;      // DMA control register
    volatile uint32_t DMAR;     // DMA address for full transfer
    volatile uint32_t OR;       // Option register
} TIM_TypeDef;

/* RCC Register Structure */
typedef struct {
    volatile uint32_t CR;
    volatile uint32_t PLLCFGR;
    volatile uint32_t CFGR;
    volatile uint32_t CIR;
    volatile uint32_t AHB1RSTR;
    volatile uint32_t AHB2RSTR;
    uint32_t RESERVED0[2];
    volatile uint32_t APB1RSTR;
    volatile uint32_t APB2RSTR;
    uint32_t RESERVED1[2];
    volatile uint32_t AHB1ENR;
    volatile uint32_t AHB2ENR;
    uint32_t RESERVED2[2];
    volatile uint32_t APB1ENR;
    volatile uint32_t APB2ENR;
} RCC_TypeDef;

/* Timer Pointers */
#define TIM1    ((TIM_TypeDef *) TIM1_BASE)
#define TIM2    ((TIM_TypeDef *) TIM2_BASE)
#define TIM3    ((TIM_TypeDef *) TIM3_BASE)
#define TIM4    ((TIM_TypeDef *) TIM4_BASE)
#define TIM5    ((TIM_TypeDef *) TIM5_BASE)
#define TIM9    ((TIM_TypeDef *) TIM9_BASE)
#define TIM10   ((TIM_TypeDef *) TIM10_BASE)
#define TIM11   ((TIM_TypeDef *) TIM11_BASE)
#define RCC     ((RCC_TypeDef *) RCC_BASE)

/* CR1 Register Bits */
#define TIM_CR1_CEN     (1 << 0)    // Counter enable
#define TIM_CR1_UDIS    (1 << 1)    // Update disable
#define TIM_CR1_URS     (1 << 2)    // Update request source
#define TIM_CR1_OPM     (1 << 3)    // One pulse mode
#define TIM_CR1_DIR     (1 << 4)    // Direction (0=up, 1=down)
#define TIM_CR1_CMS_0   (1 << 5)    // Center-aligned mode selection
#define TIM_CR1_CMS_1   (1 << 6)
#define TIM_CR1_ARPE    (1 << 7)    // Auto-reload preload enable

/* DIER Register Bits */
#define TIM_DIER_UIE    (1 << 0)    // Update interrupt enable
#define TIM_DIER_CC1IE  (1 << 1)    // Capture/Compare 1 interrupt enable
#define TIM_DIER_CC2IE  (1 << 2)    // Capture/Compare 2 interrupt enable
#define TIM_DIER_CC3IE  (1 << 3)    // Capture/Compare 3 interrupt enable
#define TIM_DIER_CC4IE  (1 << 4)    // Capture/Compare 4 interrupt enable

/* SR Register Bits */
#define TIM_SR_UIF      (1 << 0)    // Update interrupt flag
#define TIM_SR_CC1IF    (1 << 1)    // Capture/Compare 1 interrupt flag
#define TIM_SR_CC2IF    (1 << 2)    // Capture/Compare 2 interrupt flag
#define TIM_SR_CC3IF    (1 << 3)    // Capture/Compare 3 interrupt flag
#define TIM_SR_CC4IF    (1 << 4)    // Capture/Compare 4 interrupt flag

/* CCMR Register Bits (Output Compare Mode) */
#define TIM_CCMR_OC1M_PWM1  (0x6 << 4)  // PWM mode 1
#define TIM_CCMR_OC1M_PWM2  (0x7 << 4)  // PWM mode 2
#define TIM_CCMR_OC1PE      (1 << 3)     // Output compare preload enable
#define TIM_CCMR_OC2M_PWM1  (0x6 << 12)
#define TIM_CCMR_OC2M_PWM2  (0x7 << 12)
#define TIM_CCMR_OC2PE      (1 << 11)

/* CCER Register Bits */
#define TIM_CCER_CC1E   (1 << 0)    // Capture/Compare 1 output enable
#define TIM_CCER_CC1P   (1 << 1)    // Capture/Compare 1 output polarity
#define TIM_CCER_CC2E   (1 << 4)    // Capture/Compare 2 output enable
#define TIM_CCER_CC2P   (1 << 5)    // Capture/Compare 2 output polarity
#define TIM_CCER_CC3E   (1 << 8)    // Capture/Compare 3 output enable
#define TIM_CCER_CC3P   (1 << 9)    // Capture/Compare 3 output polarity
#define TIM_CCER_CC4E   (1 << 12)   // Capture/Compare 4 output enable
#define TIM_CCER_CC4P   (1 << 13)   // Capture/Compare 4 output polarity

/* RCC APB1ENR Bits */
#define RCC_APB1ENR_TIM2EN  (1 << 0)
#define RCC_APB1ENR_TIM3EN  (1 << 1)
#define RCC_APB1ENR_TIM4EN  (1 << 2)
#define RCC_APB1ENR_TIM5EN  (1 << 3)

/* RCC APB2ENR Bits */
#define RCC_APB2ENR_TIM1EN  (1 << 0)
#define RCC_APB2ENR_TIM9EN  (1 << 16)
#define RCC_APB2ENR_TIM10EN (1 << 17)
#define RCC_APB2ENR_TIM11EN (1 << 18)

/* Function Prototypes */

/**
 * @brief Initialize timer in basic mode
 * @param TIMx: Timer pointer (TIM1, TIM2, etc.)
 * @param prescaler: Prescaler value (0-65535)
 * @param period: Auto-reload value
 */
void Timer_Init(TIM_TypeDef *TIMx, uint16_t prescaler, uint32_t period);

/**
 * @brief Enable timer clock
 * @param TIMx: Timer pointer
 */
void Timer_ClockEnable(TIM_TypeDef *TIMx);

/**
 * @brief Start timer
 * @param TIMx: Timer pointer
 */
void Timer_Start(TIM_TypeDef *TIMx);

/**
 * @brief Stop timer
 * @param TIMx: Timer pointer
 */
void Timer_Stop(TIM_TypeDef *TIMx);

/**
 * @brief Enable timer update interrupt
 * @param TIMx: Timer pointer
 */
void Timer_EnableInterrupt(TIM_TypeDef *TIMx);

/**
 * @brief Disable timer update interrupt
 * @param TIMx: Timer pointer
 */
void Timer_DisableInterrupt(TIM_TypeDef *TIMx);

/**
 * @brief Clear timer update interrupt flag
 * @param TIMx: Timer pointer
 */
void Timer_ClearFlag(TIM_TypeDef *TIMx);

/**
 * @brief Initialize PWM mode
 * @param TIMx: Timer pointer
 * @param channel: PWM channel (1-4)
 * @param prescaler: Prescaler value
 * @param period: PWM period
 * @param pulse: PWM pulse width
 */
void Timer_PWM_Init(TIM_TypeDef *TIMx, uint8_t channel, uint16_t prescaler, uint32_t period, uint32_t pulse);

/**
 * @brief Set PWM duty cycle
 * @param TIMx: Timer pointer
 * @param channel: PWM channel (1-4)
 * @param pulse: PWM pulse width
 */
void Timer_PWM_SetDuty(TIM_TypeDef *TIMx, uint8_t channel, uint32_t pulse);

/**
 * @brief Set timer counter value
 * @param TIMx: Timer pointer
 * @param value: Counter value
 */
void Timer_SetCounter(TIM_TypeDef *TIMx, uint32_t value);

/**
 * @brief Get timer counter value
 * @param TIMx: Timer pointer
 * @return Current counter value
 */
uint32_t Timer_GetCounter(TIM_TypeDef *TIMx);

/**
 * @brief Create delay using timer (blocking)
 * @param TIMx: Timer pointer
 * @param ms: Delay time in milliseconds
 */
void Timer_Delay_ms(TIM_TypeDef *TIMx, uint32_t ms);

#endif /* TIMER_H */
