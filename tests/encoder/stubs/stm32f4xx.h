#ifndef ENCODER_STM32_STUB_H
#define ENCODER_STM32_STUB_H
#include <stdint.h>
typedef struct {
    volatile uint32_t MODER, OTYPER, OSPEEDR, PUPDR, IDR, ODR, BSRR, LCKR, AFR[2];
} GPIO_TypeDef;
typedef struct {
    volatile uint32_t CR1, CR2, SMCR, DIER, SR, EGR, CCMR1, CCMR2, CCER, CNT, PSC, ARR;
} TIM_TypeDef;
typedef struct {
    volatile uint32_t CR, PLLCFGR, CFGR, CIR, AHB1RSTR, AHB2RSTR, AHB3RSTR;
    volatile uint32_t reserved, APB1RSTR, APB2RSTR, res2[2], AHB1ENR, AHB2ENR;
    volatile uint32_t AHB3ENR, res3, APB1ENR, APB2ENR;
} RCC_TypeDef;
extern GPIO_TypeDef gpioa_mock, gpiob_mock, gpioc_mock;
extern TIM_TypeDef tim2_mock, tim3_mock, tim4_mock, tim5_mock;
extern RCC_TypeDef rcc_mock;
#define GPIOA (&gpioa_mock)
#define GPIOB (&gpiob_mock)
#define GPIOC (&gpioc_mock)
#define TIM2 (&tim2_mock)
#define TIM3 (&tim3_mock)
#define TIM4 (&tim4_mock)
#define TIM5 (&tim5_mock)
#define RCC (&rcc_mock)
#define RCC_AHB1ENR_GPIOBEN (1U<<1)
#define RCC_APB1ENR_TIM3EN (1U<<1)
#define RCC_APB1ENR_TIM4EN (1U<<2)
#define TIM_CR1_CEN 1U
#define TIM_SMCR_SMS 7U
#define TIM_CCER_CC1E 1U
#define TIM_CCER_CC2E (1U<<4)
#define TIM_EGR_UG 1U
static inline uint32_t __get_PRIMASK(void) {return 0U;}
static inline void __disable_irq(void) {}
static inline void __enable_irq(void) {}
#endif
