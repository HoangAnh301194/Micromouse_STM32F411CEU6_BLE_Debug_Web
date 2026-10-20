#include <assert.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "stm32f4xx.h"
#include "encoder.h"
#include "pinout.h"

GPIO_TypeDef gpioa_mock, gpiob_mock, gpioc_mock;
TIM_TypeDef tim2_mock, tim3_mock, tim4_mock, tim5_mock;
RCC_TypeDef rcc_mock;

static void near(float actual, float expected) {
    assert(fabsf(actual-expected)<0.05f);
}

int main(void) {
    Encoder_Init();
    assert((RCC->APB1ENR & (RCC_APB1ENR_TIM3EN|RCC_APB1ENR_TIM4EN)) ==
           (RCC_APB1ENR_TIM3EN|RCC_APB1ENR_TIM4EN));
    assert((TIM3->SMCR & TIM_SMCR_SMS)==3U);
    assert((TIM4->SMCR & TIM_SMCR_SMS)==3U);
    assert(TIM3->ARR==65535U && TIM4->ARR==65535U);
    assert((TIM3->CCER & (TIM_CCER_CC1E|TIM_CCER_CC2E)) ==
           (TIM_CCER_CC1E|TIM_CCER_CC2E));
    assert((TIM4->CCER & (TIM_CCER_CC1E|TIM_CCER_CC2E)) ==
           (TIM_CCER_CC1E|TIM_CCER_CC2E));
    assert(((TIM3->CCMR1 & 3U)==1U) && (((TIM3->CCMR1>>8)&3U)==1U));
    assert((GPIOB->MODER & (3U<<(4*2)))==(2U<<(4*2)));
    assert(Encoder_GetLeftCount()==0 && Encoder_GetRightCount()==0);

    /* Forward counting already established on real board: raw timer decrements. */
    TIM4->CNT=65526U; TIM3->CNT=65516U;
    Encoder_Update(0.001f);
    assert(Encoder_GetLeftCount()==10 && Encoder_GetRightCount()==20);
    near(Encoder_GetLeftSpeedPPS(),3000.0f);
    near(Encoder_GetRightSpeedPPS(),6000.0f);

    /* No edges; EMA should decay. */
    Encoder_Update(0.001f);
    assert(Encoder_GetLeftCount()==10 && Encoder_GetRightCount()==20);
    near(Encoder_GetLeftSpeedPPS(),2100.0f);
    near(Encoder_GetRightSpeedPPS(),4200.0f);

    /* Wrap 0xffff->0 when delta is positive; in current wiring this means reverse. */
    TIM4->CNT=65530U; Encoder_Reset();
    TIM4->CNT=5U;
    Encoder_Update(0.001f);
    assert(Encoder_GetLeftCount()==-11);

    /* Wrap 0->0xffff when delta negative; in current wiring this means forward. */
    TIM4->CNT=0U; Encoder_Reset();
    TIM4->CNT=65530U;
    Encoder_Update(0.001f);
    assert(Encoder_GetLeftCount()==6);
    near(Encoder_GetLeftSpeedPPS(),1800.0f);

    /* Bad dt is ignored; reset eliminates accumulated speed & counts. */
    Encoder_Update(0.0f);
    assert(Encoder_GetLeftCount()==6);
    Encoder_Reset();
    assert(Encoder_GetLeftCount()==0 && Encoder_GetRightCount()==0);
    near(Encoder_GetLeftSpeedPPS(),0.0f);
    near(Encoder_GetRightSpeedPPS(),0.0f);

#if defined(ENCODER_ALPHA_AB_DIAG)
    /* The 1-ms IRQ producer must NOT advance the shadow EMA directly. */
    {
        Encoder_ShadowSnapshot_t before, after;
        Encoder_ShadowQueueStats_t q;
        Encoder_Reset();
        TIM4->CNT = (uint16_t)(TIM4->CNT - 2U);
        TIM3->CNT = (uint16_t)(TIM3->CNT - 3U);
        Encoder_Update(0.001f);
        Encoder_GetShadowQueueStats(&q);
        assert(q.produced == 1U && q.consumed == 0U);
        assert(q.pending == 1U && q.overflow == 0U);
        Encoder_GetShadowSnapshot(&before);
        assert(before.ticks == 0U && before.alpha_left[0] == 0);
        assert(Encoder_ShadowService(1U) == 1U);
        Encoder_GetShadowSnapshot(&after);
        assert(after.ticks == 1U && after.alpha_left[0] > 0);
        Encoder_GetShadowQueueStats(&q);
        assert(q.pending == 0U && q.consumed == 1U);

        /* Bounded service does not overrun its caller budget. */
        for (int i = 0; i < 5; ++i) {
            TIM4->CNT = (uint16_t)(TIM4->CNT - 2U);
            Encoder_Update(0.001f);
        }
        assert(Encoder_ShadowService(2U) == 2U);
        Encoder_GetShadowQueueStats(&q);
        assert(q.pending == 3U);
        Encoder_ShadowMarkStageBoundary();
        Encoder_GetShadowQueueStats(&q);
        assert(q.stage_boundary_lag == 1U);
        assert(Encoder_ShadowService(16U) == 3U);

        /* Never silently overwrite unconsumed encoder samples. */
        Encoder_Reset();
        for (int i = 0; i < ENCODER_SHADOW_QUEUE_CAPACITY + 8; ++i) {
            TIM4->CNT = (uint16_t)(TIM4->CNT - 1U);
            Encoder_Update(0.001f);
        }
        Encoder_GetShadowQueueStats(&q);
        assert(q.pending == ENCODER_SHADOW_QUEUE_CAPACITY - 1U);
        assert(q.overflow == 9U);
        assert(q.produced == ENCODER_SHADOW_QUEUE_CAPACITY - 1U);
        assert(Encoder_ShadowService(1000U) ==
               ENCODER_SHADOW_QUEUE_CAPACITY - 1U);
        Encoder_GetShadowQueueStats(&q);
        assert(q.pending == 0U);
        Encoder_Reset();
    }
#endif

    /* Many wraps with small real per-tick deltas must accumulate correctly. */
    {
        uint16_t ctr=(uint16_t)TIM4->CNT;
        for (int i=0;i<10000;i++) {
            ctr=(uint16_t)(ctr-100U);
            TIM4->CNT=ctr;
            Encoder_Update(0.001f);
#if defined(ENCODER_ALPHA_AB_DIAG)
            /* Simulates bounded main service after each IRQ tick. */
            assert(Encoder_ShadowService(1U)==1U);
#endif
        }
        assert(Encoder_GetLeftCount()==1000000);
    }
    printf("PASS: actual encoder.c register config, x4, dual wheel counts, "
           "EMA, reset, 16-bit wrap and 10k-tick accumulation\n");
    return 0;
}
