#ifndef MOTOR_TEST_STM32F4XX_H
#define MOTOR_TEST_STM32F4XX_H

#include <stdint.h>

typedef struct {
    volatile uint32_t MODER, OTYPER, OSPEEDR, PUPDR, IDR, ODR, BSRR;
    volatile uint32_t AFR[2];
} GPIO_TypeDef;

typedef struct {
    volatile uint32_t CR1, CR2, SMCR, DIER, SR, EGR, CCMR1, CCMR2;
    volatile uint32_t CCER, CNT, PSC, ARR, CCR1, CCR2, CCR3, CCR4;
} TIM_TypeDef;

typedef struct {
    volatile uint32_t AHB1ENR, APB1ENR;
} RCC_TypeDef;

extern GPIO_TypeDef test_gpio_a, test_gpio_b;
extern TIM_TypeDef test_timer;
extern RCC_TypeDef test_rcc;

#define GPIOA (&test_gpio_a)
#define GPIOB (&test_gpio_b)
#define TIM2 (&test_timer)
#define RCC (&test_rcc)
#define RCC_AHB1ENR_GPIOAEN (1UL << 0)
#define RCC_AHB1ENR_GPIOBEN (1UL << 1)
#define RCC_APB1ENR_TIM2EN (1UL << 0)
#define TIM_CR1_CEN (1UL << 0)
#define TIM_CR1_ARPE (1UL << 7)
#define TIM_CCMR1_OC1PE (1UL << 3)
#define TIM_CCMR1_OC2PE (1UL << 11)
#define TIM_CCER_CC1E (1UL << 0)
#define TIM_CCER_CC2E (1UL << 4)
#define TIM_EGR_UG (1UL << 0)

uint32_t __get_PRIMASK(void);
void __disable_irq(void);
void __enable_irq(void);

#endif
