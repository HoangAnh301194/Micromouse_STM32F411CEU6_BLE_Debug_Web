#include "ir_sensor.h"
#include "board.h"
#include "stm32f4xx.h"

typedef enum {
    IR_IDLE = 0,
    IR_AMBIENT,
    IR_PAIR1_ON,
    IR_PAIR1_READ,
    IR_PAIR2_ON,
    IR_PAIR2_READ,
    IR_PAIR3_ON,
    IR_PAIR3_READ,
    IR_READY
} IR_State_t;

static volatile IR_State_t state = IR_IDLE;
static volatile uint16_t dma_buffer[IR_SENSOR_COUNT];
static volatile uint16_t ambient[IR_SENSOR_COUNT];
static volatile uint16_t result[IR_SENSOR_COUNT];
static volatile uint32_t scan_count = 0;

static const uint8_t adc_channels[IR_SENSOR_COUNT] = {
    IR_RX_L90_ADC_CHANNEL,
    IR_RX_L45_ADC_CHANNEL,
    IR_RX_L0_ADC_CHANNEL,
    IR_RX_R0_ADC_CHANNEL,
    IR_RX_R45_ADC_CHANNEL,
    IR_RX_R90_ADC_CHANNEL
};

static void GPIO_Output(GPIO_TypeDef *port, uint8_t pin)
{
    port->MODER &= ~(3UL << (pin * 2U));
    port->MODER |=  (1UL << (pin * 2U));
    port->OTYPER &= ~(1UL << pin);
    port->PUPDR &= ~(3UL << (pin * 2U));
    port->BSRR = (1UL << (pin + 16U));
}

static void GPIO_Analog(GPIO_TypeDef *port, uint8_t pin)
{
    port->MODER |= (3UL << (pin * 2U));
    port->PUPDR &= ~(3UL << (pin * 2U));
}

static void AllEmittersOff(void)
{
    IR_LED_OFF(IR_LED_L90_PORT, IR_LED_L90_PIN);
    IR_LED_OFF(IR_LED_L45_PORT, IR_LED_L45_PIN);
    IR_LED_OFF(IR_LED_L0_PORT, IR_LED_L0_PIN);
    IR_LED_OFF(IR_LED_R0_PORT, IR_LED_R0_PIN);
    IR_LED_OFF(IR_LED_R45_PORT, IR_LED_R45_PIN);
    IR_LED_OFF(IR_LED_R90_PORT, IR_LED_R90_PIN);
}

static uint8_t ADC_TriggerAndWait(void)
{
    uint32_t timeout = 2000U;

    DMA2_Stream0->CR &= ~DMA_SxCR_EN;
    while ((DMA2_Stream0->CR & DMA_SxCR_EN) && timeout) {
        timeout--;
    }
    if (timeout == 0U) return 0U;

    DMA2->LIFCR = 0x3DU;
    DMA2_Stream0->NDTR = IR_SENSOR_COUNT;
    DMA2_Stream0->CR |= DMA_SxCR_EN;
    ADC1->CR2 |= ADC_CR2_SWSTART;

    timeout = 4000U;
    while (((DMA2->LISR & (1UL << 5)) == 0U) && timeout) {
        timeout--;
    }

    DMA2->LIFCR = 0x3DU;
    return (timeout != 0U) ? 1U : 0U;
}

static void ADC_DMA_Init(void)
{
    uint32_t wait = 10000U;

    RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;
    RCC->AHB1ENR |= RCC_AHB1ENR_DMA2EN;

    ADC1->CR1 = ADC_CR1_SCAN;
    ADC1->CR2 = ADC_CR2_DMA | ADC_CR2_DDS;
    ADC1->SMPR2 = 0x04924924UL;
    ADC1->SMPR1 = 0x04924924UL;
    ADC1->SQR1 = ((IR_SENSOR_COUNT - 1U) << 20);
    ADC1->SQR3 = ((uint32_t)adc_channels[0] << 0) |
                 ((uint32_t)adc_channels[1] << 5) |
                 ((uint32_t)adc_channels[2] << 10) |
                 ((uint32_t)adc_channels[3] << 15) |
                 ((uint32_t)adc_channels[4] << 20) |
                 ((uint32_t)adc_channels[5] << 25);

    ADC1->CR2 |= ADC_CR2_ADON;
    while (wait--) {
        __NOP();
    }

    DMA2_Stream0->CR &= ~DMA_SxCR_EN;
    while (DMA2_Stream0->CR & DMA_SxCR_EN) {
    }

    DMA2->LIFCR = 0x3DU;
    DMA2_Stream0->CR = 0;
    DMA2_Stream0->CR |= (1UL << 13) | (1UL << 11) | DMA_SxCR_MINC;
    DMA2_Stream0->PAR = (uint32_t)&ADC1->DR;
    DMA2_Stream0->M0AR = (uint32_t)dma_buffer;
}

static void TIM10_Init(void)
{
    uint32_t timer_clock = Board_GetAPB2TimerClockHz();
    uint32_t prescaler = timer_clock / 1000000UL;

    if (prescaler == 0U) prescaler = 1U;

    RCC->APB2ENR |= RCC_APB2ENR_TIM10EN;

    TIM10->CR1 = 0;
    TIM10->PSC = prescaler - 1U;
    TIM10->ARR = 100U - 1U;
    TIM10->EGR = TIM_EGR_UG;
    TIM10->SR = 0;
    TIM10->DIER = TIM_DIER_UIE;

    NVIC_SetPriority(TIM1_UP_TIM10_IRQn, 1);
    NVIC_EnableIRQ(TIM1_UP_TIM10_IRQn);
}

void IR_Init(void)
{
    uint8_t i;

    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOBEN;

    GPIO_Output(IR_LED_L90_PORT, IR_LED_L90_PIN);
    GPIO_Output(IR_LED_L45_PORT, IR_LED_L45_PIN);
    GPIO_Output(IR_LED_L0_PORT, IR_LED_L0_PIN);
    GPIO_Output(IR_LED_R0_PORT, IR_LED_R0_PIN);
    GPIO_Output(IR_LED_R45_PORT, IR_LED_R45_PIN);
    GPIO_Output(IR_LED_R90_PORT, IR_LED_R90_PIN);

    GPIO_Analog(IR_RX_L90_PORT, IR_RX_L90_PIN);
    GPIO_Analog(IR_RX_L45_PORT, IR_RX_L45_PIN);
    GPIO_Analog(IR_RX_L0_PORT, IR_RX_L0_PIN);
    GPIO_Analog(IR_RX_R0_PORT, IR_RX_R0_PIN);
    GPIO_Analog(IR_RX_R45_PORT, IR_RX_R45_PIN);
    GPIO_Analog(IR_RX_R90_PORT, IR_RX_R90_PIN);

    AllEmittersOff();

    for (i = 0; i < IR_SENSOR_COUNT; i++) {
        ambient[i] = 0;
        result[i] = 0;
        dma_buffer[i] = 0;
    }

    ADC_DMA_Init();
    TIM10_Init();
    state = IR_IDLE;
}

void IR_StartScan(void)
{
    if (state == IR_IDLE) {
        state = IR_AMBIENT;
        TIM10->CNT = 0;
        TIM10->SR &= ~TIM_SR_UIF;
        TIM10->CR1 |= TIM_CR1_CEN;
    }
}

void IR_Stop(void)
{
    TIM10->CR1 &= ~TIM_CR1_CEN;
    AllEmittersOff();
    state = IR_IDLE;
}

uint8_t IR_IsReady(void)
{
    return (state == IR_READY) ? 1U : 0U;
}

uint8_t IR_GetLatest(IR_Data_t *out)
{
    uint8_t i;

    if (out == 0 || state != IR_READY) return 0U;

    for (i = 0; i < IR_SENSOR_COUNT; i++) {
        out->value[i] = result[i];
    }
    out->scan_count = scan_count;
    state = IR_IDLE;
    return 1U;
}

void TIM1_UP_TIM10_IRQHandler(void)
{
    uint8_t i;

    if ((TIM10->SR & TIM_SR_UIF) == 0U) return;
    TIM10->SR &= ~TIM_SR_UIF;

    switch (state) {
    case IR_AMBIENT:
        AllEmittersOff();
        if (ADC_TriggerAndWait()) {
            for (i = 0; i < IR_SENSOR_COUNT; i++) ambient[i] = dma_buffer[i];
            state = IR_PAIR1_ON;
        } else {
            IR_Stop();
        }
        break;

    case IR_PAIR1_ON:
        IR_LED_ON(IR_LED_L90_PORT, IR_LED_L90_PIN);
        IR_LED_ON(IR_LED_R45_PORT, IR_LED_R45_PIN);
        state = IR_PAIR1_READ;
        break;

    case IR_PAIR1_READ:
        if (ADC_TriggerAndWait()) {
            result[0] = (dma_buffer[0] > ambient[0]) ? (dma_buffer[0] - ambient[0]) : 0U;
            result[4] = (dma_buffer[4] > ambient[4]) ? (dma_buffer[4] - ambient[4]) : 0U;
            IR_LED_OFF(IR_LED_L90_PORT, IR_LED_L90_PIN);
            IR_LED_OFF(IR_LED_R45_PORT, IR_LED_R45_PIN);
            state = IR_PAIR2_ON;
        } else {
            IR_Stop();
        }
        break;

    case IR_PAIR2_ON:
        IR_LED_ON(IR_LED_L45_PORT, IR_LED_L45_PIN);
        IR_LED_ON(IR_LED_R90_PORT, IR_LED_R90_PIN);
        state = IR_PAIR2_READ;
        break;

    case IR_PAIR2_READ:
        if (ADC_TriggerAndWait()) {
            result[1] = (dma_buffer[1] > ambient[1]) ? (dma_buffer[1] - ambient[1]) : 0U;
            result[5] = (dma_buffer[5] > ambient[5]) ? (dma_buffer[5] - ambient[5]) : 0U;
            IR_LED_OFF(IR_LED_L45_PORT, IR_LED_L45_PIN);
            IR_LED_OFF(IR_LED_R90_PORT, IR_LED_R90_PIN);
            state = IR_PAIR3_ON;
        } else {
            IR_Stop();
        }
        break;

    case IR_PAIR3_ON:
        IR_LED_ON(IR_LED_L0_PORT, IR_LED_L0_PIN);
        IR_LED_ON(IR_LED_R0_PORT, IR_LED_R0_PIN);
        state = IR_PAIR3_READ;
        break;

    case IR_PAIR3_READ:
        if (ADC_TriggerAndWait()) {
            result[2] = (dma_buffer[2] > ambient[2]) ? (dma_buffer[2] - ambient[2]) : 0U;
            result[3] = (dma_buffer[3] > ambient[3]) ? (dma_buffer[3] - ambient[3]) : 0U;
            IR_LED_OFF(IR_LED_L0_PORT, IR_LED_L0_PIN);
            IR_LED_OFF(IR_LED_R0_PORT, IR_LED_R0_PIN);
            scan_count++;
            state = IR_READY;
            TIM10->CR1 &= ~TIM_CR1_CEN;
        } else {
            IR_Stop();
        }
        break;

    default:
        TIM10->CR1 &= ~TIM_CR1_CEN;
        break;
    }
}
