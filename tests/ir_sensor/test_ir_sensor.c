#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "ir_sensor.h"
#include "stm32f4xx.h"

void TIM1_UP_TIM10_IRQHandler(void);
void DMA2_Stream0_IRQHandler(void);

GPIO_TypeDef test_gpio_a, test_gpio_b;
TIM_TypeDef test_tim10;
RCC_TypeDef test_rcc;
ADC_TypeDef test_adc1;
ADC_Common_TypeDef test_adc_common;
DMA_Stream_TypeDef test_dma_stream0;
DMA_TypeDef test_dma2;
DWT_TypeDef test_dwt;

static uint32_t fake_us;
static uint32_t enabled_irqs[64];
static uint32_t priorities[64];

uint32_t micros(void) { return fake_us; }
uint32_t Board_GetAPB2TimerClockHz(void) { return 96000000U; }
void NVIC_SetPriority(int32_t irq, uint32_t priority)
{
    assert(irq >= 0 && irq < 64);
    priorities[irq] = priority;
}
void NVIC_EnableIRQ(int32_t irq)
{
    assert(irq >= 0 && irq < 64);
    enabled_irqs[irq] = 1U;
}
uint32_t __get_PRIMASK(void) { return 0U; }
void __disable_irq(void) {}
void __enable_irq(void) {}
void __NOP(void) {}

static void tim10_tick(void)
{
    fake_us += 100U;
    test_tim10.SR |= TIM_SR_UIF;
    TIM1_UP_TIM10_IRQHandler();
}

static void dma_complete(void)
{
    fake_us += 20U;
    test_dma_stream0.CR &= ~DMA_SxCR_EN;
    test_dma2.LISR = (1UL << 5);
    DMA2_Stream0_IRQHandler();
    test_dma2.LISR = 0U;
}

static void dma_error(void)
{
    test_dma_stream0.CR &= ~DMA_SxCR_EN;
    test_dma2.LISR = (1UL << 3);
    DMA2_Stream0_IRQHandler();
    test_dma2.LISR = 0U;
}

static void adc_overrun(void)
{
    test_dma_stream0.CR &= ~DMA_SxCR_EN;
    test_adc1.SR |= ADC_SR_OVR;
    test_dma2.LISR = (1UL << 5);
    DMA2_Stream0_IRQHandler();
    test_dma2.LISR = 0U;
}

static void test_complete_scan(void)
{
    IR_Data_t sample;
    IR_Diagnostics_t stats;

    memset(&test_gpio_a, 0, sizeof(test_gpio_a));
    memset(&test_gpio_b, 0, sizeof(test_gpio_b));
    memset(&test_tim10, 0, sizeof(test_tim10));
    memset(&test_rcc, 0, sizeof(test_rcc));
    memset(&test_adc1, 0, sizeof(test_adc1));
    memset(&test_adc_common, 0, sizeof(test_adc_common));
    memset(&test_dma_stream0, 0, sizeof(test_dma_stream0));
    memset(&test_dma2, 0, sizeof(test_dma2));
    memset(&test_dwt, 0, sizeof(test_dwt));
    memset(enabled_irqs, 0, sizeof(enabled_irqs));
    memset(priorities, 0, sizeof(priorities));
    fake_us = 1000U;

    IR_Init();
    assert((test_adc_common.CCR & ADC_CCR_ADCPRE) == ADC_CCR_ADCPRE_0);
    assert(enabled_irqs[DMA2_Stream0_IRQn] == 1U);
    assert(priorities[DMA2_Stream0_IRQn] == 2U);

    IR_StartScan();
    IR_StartScan(); /* Duplicate request must not launch a second conversion. */
    tim10_tick(); /* Start ambient DMA. */
    assert((test_dma_stream0.CR & DMA_SxCR_EN) != 0U);
    assert(test_dma_stream0.NDTR == IR_SENSOR_COUNT);
    assert(IR_IsReady() == 0U);

    tim10_tick(); /* DMA remains pending; TIM10 must not wait for it. */
    assert((test_dma_stream0.CR & DMA_SxCR_EN) != 0U);
    dma_complete();
    tim10_tick(); /* Pair 1 emitters on. */
    tim10_tick(); /* Start pair 1 DMA. */
    assert((test_dma_stream0.CR & DMA_SxCR_EN) != 0U);
    dma_complete();
    tim10_tick(); /* Pair 2 emitters on. */
    tim10_tick(); /* Start pair 2 DMA. */
    dma_complete();
    tim10_tick(); /* Pair 3 emitters on. */
    tim10_tick(); /* Start pair 3 DMA. */
    assert(IR_IsReady() == 0U);
    dma_complete();

    assert(IR_IsReady() == 1U);
    assert(IR_GetLatest(&sample) == 1U);
    assert(sample.scan_count == 1U);
    assert(sample.completed_us == fake_us);
    assert(IR_GetLatest(&sample) == 0U);

    IR_GetDiagnostics(&stats);
    assert(stats.scans_started == 1U);
    assert(stats.scans_completed == 1U);
    assert(stats.scans_failed == 0U);
    assert(stats.dma_started == 4U);
    assert(stats.dma_completed == 4U);
    assert(stats.ambient_reads == 1U);
    assert(stats.pair1_reads == 1U);
    assert(stats.pair2_reads == 1U);
    assert(stats.pair3_reads == 1U);
    assert(stats.scan_max_us > 0U);
    assert(stats.dma_max_us == 120U);
}

static void test_error_timeout_and_recovery(void)
{
    IR_Diagnostics_t stats;

    IR_StartScan();
    tim10_tick(); /* ADC wake delay after recovering from the DMA error. */
    tim10_tick();
    dma_error();
    assert(IR_IsReady() == 0U);
    IR_StartScan();
    tim10_tick(); /* ADC wake delay after timeout reset. */
    tim10_tick();
    fake_us += 600U;
    tim10_tick();
    assert(IR_IsReady() == 0U);

    IR_StartScan();
    tim10_tick();
    tim10_tick();
    adc_overrun();
    IR_StartScan();
    tim10_tick(); /* ADC wake delay after overrun reset. */
    tim10_tick();
    dma_complete();
    IR_GetDiagnostics(&stats);
    assert(stats.scans_started == 5U);
    assert(stats.scans_completed == 1U);
    assert(stats.scans_failed == 3U);
    assert(stats.dma_errors == 1U);
    assert(stats.dma_timeouts == 1U);
    assert(stats.adc_overruns == 1U);
    assert(stats.ambient_reads == 2U);
}

int main(void)
{
    test_complete_scan();
    test_error_timeout_and_recovery();
    puts("PASS asynchronous IR DMA state machine, complete snapshot, error, timeout and restart");
    return 0;
}
