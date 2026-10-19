#ifndef IR_TEST_STM32F4XX_H
#define IR_TEST_STM32F4XX_H

#include <stdint.h>

typedef struct {
    volatile uint32_t MODER, OTYPER, PUPDR, BSRR;
} GPIO_TypeDef;

typedef struct {
    volatile uint32_t CR1, SR, CNT, PSC, ARR, EGR, DIER;
} TIM_TypeDef;

typedef struct {
    volatile uint32_t AHB1ENR, APB2ENR;
} RCC_TypeDef;

typedef struct {
    volatile uint32_t CR1, CR2, SMPR1, SMPR2, SQR1, SQR3, DR, SR;
} ADC_TypeDef;

typedef struct {
    volatile uint32_t CSR, CCR, CDR;
} ADC_Common_TypeDef;

typedef struct {
    volatile uint32_t CR, NDTR, PAR, M0AR, FCR;
} DMA_Stream_TypeDef;

typedef struct {
    volatile uint32_t LISR, LIFCR;
} DMA_TypeDef;

typedef struct {
    volatile uint32_t CYCCNT;
} DWT_TypeDef;

extern GPIO_TypeDef test_gpio_a, test_gpio_b;
extern TIM_TypeDef test_tim10;
extern RCC_TypeDef test_rcc;
extern ADC_TypeDef test_adc1;
extern ADC_Common_TypeDef test_adc_common;
extern DMA_Stream_TypeDef test_dma_stream0;
extern DMA_TypeDef test_dma2;
extern DWT_TypeDef test_dwt;

#define GPIOA (&test_gpio_a)
#define GPIOB (&test_gpio_b)
#define TIM10 (&test_tim10)
#define RCC (&test_rcc)
#define ADC1 (&test_adc1)
#define ADC1_COMMON (&test_adc_common)
#define DMA2_Stream0 (&test_dma_stream0)
#define DMA2 (&test_dma2)
#define DWT (&test_dwt)

#define RCC_AHB1ENR_GPIOAEN (1UL << 0)
#define RCC_AHB1ENR_GPIOBEN (1UL << 1)
#define RCC_AHB1ENR_DMA2EN  (1UL << 22)
#define RCC_APB2ENR_ADC1EN  (1UL << 8)
#define RCC_APB2ENR_TIM10EN (1UL << 17)

#define ADC_CR1_SCAN (1UL << 8)
#define ADC_CR2_DMA  (1UL << 14)
#define ADC_CR2_DDS  (1UL << 9)
#define ADC_CR2_ADON (1UL << 0)
#define ADC_CR2_SWSTART (1UL << 30)
#define ADC_SR_OVR (1UL << 5)
#define ADC_CCR_ADCPRE (3UL << 16)
#define ADC_CCR_ADCPRE_0 (1UL << 16)

#define DMA_SxCR_EN    (1UL << 0)
#define DMA_SxCR_DMEIE (1UL << 1)
#define DMA_SxCR_TEIE  (1UL << 2)
#define DMA_SxCR_TCIE  (1UL << 4)
#define DMA_SxCR_MINC  (1UL << 10)
#define DMA_SxCR_PSIZE_0 (1UL << 11)
#define DMA_SxCR_MSIZE_0 (1UL << 13)
#define DMA_SxFCR_FEIE (1UL << 7)
#define DMA2_Stream0_IRQn 56
#define TIM1_UP_TIM10_IRQn 25

#define TIM_CR1_CEN (1UL << 0)
#define TIM_EGR_UG (1UL << 0)
#define TIM_SR_UIF (1UL << 0)
#define TIM_DIER_UIE (1UL << 0)

void NVIC_SetPriority(int32_t irq, uint32_t priority);
void NVIC_EnableIRQ(int32_t irq);
uint32_t __get_PRIMASK(void);
void __disable_irq(void);
void __enable_irq(void);
void __NOP(void);

#endif
