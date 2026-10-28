#include "ir_sensor.h"
#include "board.h"
#include "system_timer.h"
#include "stm32f4xx.h"

typedef enum {
    IR_IDLE = 0,
    IR_ADC_WAKE,
    IR_AMBIENT_START,
    IR_AMBIENT_DMA,
    IR_PAIR1_ON,
    IR_PAIR1_START,
    IR_PAIR1_DMA,
    IR_PAIR2_ON,
    IR_PAIR2_START,
    IR_PAIR2_DMA,
    IR_PAIR3_ON,
    IR_PAIR3_START,
    IR_PAIR3_DMA,
    IR_READY
} IR_State_t;

#define IR_DMA_TIMEOUT_US 500U
#define DMA2_STREAM0_FLAGS 0x3DU
#define DMA2_STREAM0_TCIF  (1UL << 5)
#define DMA2_STREAM0_ERROR ((1UL << 0) | (1UL << 2) | (1UL << 3))

typedef enum {
    IR_FAILURE_DMA = 0,
    IR_FAILURE_TIMEOUT,
    IR_FAILURE_ADC_OVERRUN
} IR_Failure_t;

static volatile IR_State_t state = IR_IDLE;
static volatile uint16_t dma_buffer[IR_SENSOR_COUNT];
static volatile uint16_t ambient[IR_SENSOR_COUNT];
static volatile uint16_t result[IR_SENSOR_COUNT];
static volatile uint32_t scan_count = 0;
static volatile uint32_t completed_us = 0U;
static volatile uint32_t scan_started_us = 0U;
static volatile uint32_t dma_started_us = 0U;
#if defined(INTEGRATION_DIAG) || defined(IR_FILTER_DIAG)
static uint16_t ambient_work[IR_SENSOR_COUNT];
static uint16_t emitter_work[IR_SENSOR_COUNT];
#endif

#if defined(INTEGRATION_DIAG) || defined(IR_FILTER_DIAG)
static volatile IR_Diagnostics_t diagnostics;
#endif

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

static uint8_t ADC_StartDma(void)
{
    if ((DMA2_Stream0->CR & DMA_SxCR_EN) != 0U) {
        return 0U;
    }

    DMA2->LIFCR = DMA2_STREAM0_FLAGS;
    DMA2_Stream0->NDTR = IR_SENSOR_COUNT;
    DMA2_Stream0->CR |= DMA_SxCR_EN;
    ADC1->SR = 0U;
    dma_started_us = micros();
#if defined(INTEGRATION_DIAG) || defined(IR_FILTER_DIAG)
    diagnostics.dma_started++;
#endif
    ADC1->CR2 |= ADC_CR2_SWSTART;
    return 1U;
}

static void IR_Fail(IR_Failure_t reason)
{
    TIM10->CR1 &= ~TIM_CR1_CEN;
    ADC1->CR2 &= ~(ADC_CR2_ADON | ADC_CR2_SWSTART);
    DMA2_Stream0->CR &= ~DMA_SxCR_EN;
    DMA2->LIFCR = DMA2_STREAM0_FLAGS;
    AllEmittersOff();
    state = IR_IDLE;
#if defined(INTEGRATION_DIAG) || defined(IR_FILTER_DIAG)
    diagnostics.scans_failed++;
    if (reason == IR_FAILURE_TIMEOUT) diagnostics.dma_timeouts++;
    else if (reason == IR_FAILURE_DMA) diagnostics.dma_errors++;
#else
    (void)reason;
#endif
}

static void IR_CompleteDma(void)
{
    uint8_t i;
    uint32_t now = micros();

#if defined(INTEGRATION_DIAG) || defined(IR_FILTER_DIAG)
    uint32_t dma_duration = now - dma_started_us;
    diagnostics.dma_completed++;
    diagnostics.dma_total_us += dma_duration;
    if (dma_duration > diagnostics.dma_max_us)
        diagnostics.dma_max_us = dma_duration;
#endif

    switch (state) {
    case IR_AMBIENT_DMA:
        for (i = 0U; i < IR_SENSOR_COUNT; i++) {
            ambient[i] = dma_buffer[i];
#if defined(INTEGRATION_DIAG) || defined(IR_FILTER_DIAG)
            ambient_work[i] = dma_buffer[i];
#endif
        }
#if defined(INTEGRATION_DIAG) || defined(IR_FILTER_DIAG)
        diagnostics.ambient_reads++;
#endif
        state = IR_PAIR1_ON;
        break;

    case IR_PAIR1_DMA:
#if defined(INTEGRATION_DIAG) || defined(IR_FILTER_DIAG)
        emitter_work[0] = dma_buffer[0];
        emitter_work[4] = dma_buffer[4];
#endif
        result[0] = (dma_buffer[0] > ambient[0]) ?
                    ((uint32_t)dma_buffer[0] - ambient[0]) : 0U;
        result[4] = (dma_buffer[4] > ambient[4]) ?
                    ((uint32_t)dma_buffer[4] - ambient[4]) : 0U;
        IR_LED_OFF(IR_LED_L90_PORT, IR_LED_L90_PIN);
        IR_LED_OFF(IR_LED_R45_PORT, IR_LED_R45_PIN);
#if defined(INTEGRATION_DIAG) || defined(IR_FILTER_DIAG)
        diagnostics.pair1_reads++;
#endif
        state = IR_PAIR2_ON;
        break;

    case IR_PAIR2_DMA:
#if defined(INTEGRATION_DIAG) || defined(IR_FILTER_DIAG)
        emitter_work[1] = dma_buffer[1];
        emitter_work[5] = dma_buffer[5];
#endif
        result[1] = (dma_buffer[1] > ambient[1]) ?
                    ((uint32_t)dma_buffer[1] - ambient[1]) : 0U;
        result[5] = (dma_buffer[5] > ambient[5]) ?
                    ((uint32_t)dma_buffer[5] - ambient[5]) : 0U;
        IR_LED_OFF(IR_LED_L45_PORT, IR_LED_L45_PIN);
        IR_LED_OFF(IR_LED_R90_PORT, IR_LED_R90_PIN);
#if defined(INTEGRATION_DIAG) || defined(IR_FILTER_DIAG)
        diagnostics.pair2_reads++;
#endif
        state = IR_PAIR3_ON;
        break;

    case IR_PAIR3_DMA:
#if defined(INTEGRATION_DIAG) || defined(IR_FILTER_DIAG)
        emitter_work[2] = dma_buffer[2];
        emitter_work[3] = dma_buffer[3];
#endif
        result[2] = (dma_buffer[2] > ambient[2]) ?
                    ((uint32_t)dma_buffer[2] - ambient[2]) : 0U;
        result[3] = (dma_buffer[3] > ambient[3]) ?
                    ((uint32_t)dma_buffer[3] - ambient[3]) : 0U;
        IR_LED_OFF(IR_LED_L0_PORT, IR_LED_L0_PIN);
        IR_LED_OFF(IR_LED_R0_PORT, IR_LED_R0_PIN);
#if defined(INTEGRATION_DIAG) || defined(IR_FILTER_DIAG)
        diagnostics.pair3_reads++;
#endif
        scan_count++;
        completed_us = now;
        state = IR_READY;
        TIM10->CR1 &= ~TIM_CR1_CEN;
#if defined(INTEGRATION_DIAG) || defined(IR_FILTER_DIAG)
        {
            for (i = 0U; i < IR_SENSOR_COUNT; i++) {
                diagnostics.ambient_raw[i] = ambient_work[i];
                diagnostics.emitter_raw[i] = emitter_work[i];
            }
            diagnostics.raw_scan_count = scan_count;
            uint32_t scan_duration = now - scan_started_us;
            diagnostics.scans_completed++;
            diagnostics.scan_total_us += scan_duration;
            if (scan_duration > diagnostics.scan_max_us)
                diagnostics.scan_max_us = scan_duration;
        }
#endif
        break;

    default:
        IR_Fail(IR_FAILURE_DMA);
        break;
    }
}

static void ADC_DMA_Init(void)
{
    uint32_t wait = 10000U;

    RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;
    RCC->AHB1ENR |= RCC_AHB1ENR_DMA2EN;

    /* PCLK2 is nominally 96MHz in the selected profile; /4 keeps ADC clock
     * within the STM32F411 limit while retaining adequate scan throughput. */
    ADC1_COMMON->CCR = (ADC1_COMMON->CCR & ~ADC_CCR_ADCPRE) |
                       ADC_CCR_ADCPRE_0;

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
    DMA2->LIFCR = DMA2_STREAM0_FLAGS;
    DMA2_Stream0->CR = 0;
    DMA2_Stream0->CR = DMA_SxCR_MSIZE_0 | DMA_SxCR_PSIZE_0 |
                       DMA_SxCR_MINC | DMA_SxCR_TCIE |
                       DMA_SxCR_TEIE | DMA_SxCR_DMEIE;
    DMA2_Stream0->PAR = (uint32_t)(uintptr_t)&ADC1->DR;
    DMA2_Stream0->M0AR = (uint32_t)(uintptr_t)dma_buffer;

    DMA2_Stream0->FCR = DMA_SxFCR_FEIE;
    NVIC_SetPriority(DMA2_Stream0_IRQn, 2U);
    NVIC_EnableIRQ(DMA2_Stream0_IRQn);
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
    scan_count = 0U;
    completed_us = 0U;
#if defined(INTEGRATION_DIAG) || defined(IR_FILTER_DIAG)
    diagnostics = (IR_Diagnostics_t){0};
#endif

    ADC_DMA_Init();
    TIM10_Init();
    state = IR_IDLE;
}

void IR_StartScan(void)
{
    if (state == IR_IDLE) {
        if ((DMA2_Stream0->CR & DMA_SxCR_EN) != 0U) {
#if defined(INTEGRATION_DIAG) || defined(IR_FILTER_DIAG)
            diagnostics.start_busy++;
#endif
            return;
        }
        scan_started_us = micros();
        if ((ADC1->CR2 & ADC_CR2_ADON) == 0U) {
            ADC1->CR2 |= ADC_CR2_ADON;
            state = IR_ADC_WAKE;
        } else {
            state = IR_AMBIENT_START;
        }
#if defined(INTEGRATION_DIAG) || defined(IR_FILTER_DIAG)
        diagnostics.scans_started++;
#endif
        TIM10->CNT = 0U;
        TIM10->SR &= ~TIM_SR_UIF;
        TIM10->CR1 |= TIM_CR1_CEN;
    } else {
#if defined(INTEGRATION_DIAG) || defined(IR_FILTER_DIAG)
        diagnostics.start_busy++;
#endif
    }
}

void IR_Stop(void)
{
    TIM10->CR1 &= ~TIM_CR1_CEN;
    ADC1->CR2 &= ~ADC_CR2_ADON;
    DMA2_Stream0->CR &= ~DMA_SxCR_EN;
    DMA2->LIFCR = DMA2_STREAM0_FLAGS;
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
    out->completed_us = completed_us;
    state = IR_IDLE;
    return 1U;
}

#if defined(INTEGRATION_DIAG) || defined(IR_FILTER_DIAG)
void IR_GetDiagnostics(IR_Diagnostics_t *out)
{
    uint32_t primask;
    if (out == 0) return;
    primask = __get_PRIMASK();
    __disable_irq();
    *out = diagnostics;
    if (!primask) __enable_irq();
}
#endif

void TIM1_UP_TIM10_IRQHandler(void)
{
#if defined(INTEGRATION_DIAG) || defined(IR_FILTER_DIAG)
    uint32_t irq_start = DWT->CYCCNT;
#endif

    if ((TIM10->SR & TIM_SR_UIF) == 0U) return;
    TIM10->SR &= ~TIM_SR_UIF;
#if defined(INTEGRATION_DIAG) || defined(IR_FILTER_DIAG)
    diagnostics.tim10_irq_count++;
#endif

    switch (state) {
    case IR_AMBIENT_DMA:
    case IR_PAIR1_DMA:
    case IR_PAIR2_DMA:
    case IR_PAIR3_DMA:
        if ((uint32_t)(micros() - dma_started_us) >= IR_DMA_TIMEOUT_US)
            IR_Fail(IR_FAILURE_TIMEOUT);
        break;

    case IR_AMBIENT_START:
        AllEmittersOff();
        if (!ADC_StartDma()) IR_Fail(IR_FAILURE_DMA);
        else state = IR_AMBIENT_DMA;
        break;

    case IR_ADC_WAKE:
        state = IR_AMBIENT_START;
        break;

    case IR_PAIR1_ON:
        IR_LED_ON(IR_LED_L90_PORT, IR_LED_L90_PIN);
        IR_LED_ON(IR_LED_R45_PORT, IR_LED_R45_PIN);
        state = IR_PAIR1_START;
        break;

    case IR_PAIR1_START:
        if (!ADC_StartDma()) IR_Fail(IR_FAILURE_DMA);
        else state = IR_PAIR1_DMA;
        break;

    case IR_PAIR2_ON:
        IR_LED_ON(IR_LED_L45_PORT, IR_LED_L45_PIN);
        IR_LED_ON(IR_LED_R90_PORT, IR_LED_R90_PIN);
        state = IR_PAIR2_START;
        break;

    case IR_PAIR2_START:
        if (!ADC_StartDma()) IR_Fail(IR_FAILURE_DMA);
        else state = IR_PAIR2_DMA;
        break;

    case IR_PAIR3_ON:
        IR_LED_ON(IR_LED_L0_PORT, IR_LED_L0_PIN);
        IR_LED_ON(IR_LED_R0_PORT, IR_LED_R0_PIN);
        state = IR_PAIR3_START;
        break;

    case IR_PAIR3_START:
        if (!ADC_StartDma()) IR_Fail(IR_FAILURE_DMA);
        else state = IR_PAIR3_DMA;
        break;

    default:
        if (state != IR_READY && state != IR_IDLE)
            IR_Fail(IR_FAILURE_DMA);
        break;
    }

#if defined(INTEGRATION_DIAG) || defined(IR_FILTER_DIAG)
    {
        uint32_t cycles = DWT->CYCCNT - irq_start;
        diagnostics.tim10_irq_total_cycles += cycles;
        if (cycles > diagnostics.tim10_irq_max_cycles)
            diagnostics.tim10_irq_max_cycles = cycles;
    }
#endif
}

void DMA2_Stream0_IRQHandler(void)
{
    uint32_t status = DMA2->LISR;
    uint8_t adc_overrun = (ADC1->SR & ADC_SR_OVR) ? 1U : 0U;
#if defined(INTEGRATION_DIAG) || defined(IR_FILTER_DIAG)
    uint32_t irq_start = DWT->CYCCNT;
#endif

    if (adc_overrun) {
#if defined(INTEGRATION_DIAG) || defined(IR_FILTER_DIAG)
        diagnostics.adc_overruns++;
#endif
        ADC1->SR &= ~ADC_SR_OVR;
        DMA2->LIFCR = DMA2_STREAM0_FLAGS;
        IR_Fail(IR_FAILURE_ADC_OVERRUN);
    } else if ((status & DMA2_STREAM0_ERROR) != 0U ||
        ((status & DMA2_STREAM0_TCIF) == 0U)) {
        DMA2->LIFCR = DMA2_STREAM0_FLAGS;
        IR_Fail(IR_FAILURE_DMA);
    } else if ((uint32_t)(micros() - dma_started_us) >= IR_DMA_TIMEOUT_US) {
        DMA2->LIFCR = DMA2_STREAM0_FLAGS;
        IR_Fail(IR_FAILURE_TIMEOUT);
    } else {
        DMA2->LIFCR = DMA2_STREAM0_FLAGS;
        IR_CompleteDma();
    }

#if defined(INTEGRATION_DIAG) || defined(IR_FILTER_DIAG)
    {
        diagnostics.dma_irq_count++;
        uint32_t cycles = DWT->CYCCNT - irq_start;
        diagnostics.dma_irq_total_cycles += cycles;
        if (cycles > diagnostics.dma_irq_max_cycles)
            diagnostics.dma_irq_max_cycles = cycles;
    }
#endif
}
