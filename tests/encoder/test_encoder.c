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

    /* Many wraps with small real per-tick deltas must accumulate correctly. */
    {
        uint16_t ctr=(uint16_t)TIM4->CNT;
        for (int i=0;i<10000;i++) {
            ctr=(uint16_t)(ctr-100U);
            TIM4->CNT=ctr;
            Encoder_Update(0.001f);
        }
        assert(Encoder_GetLeftCount()==1000000);
    }
    printf("PASS: actual encoder.c register config, x4, dual wheel counts, "
           "EMA, reset, 16-bit wrap and 10k-tick accumulation\n");
    return 0;
}
