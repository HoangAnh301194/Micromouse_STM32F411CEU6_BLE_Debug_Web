#include "board.h"
#include "pinout.h"

#define HSI_HZ 16000000UL
#define HSE_HZ 25000000UL

static uint32_t Board_GetSystemClockHz(void)
{
    uint32_t sws = RCC->CFGR & RCC_CFGR_SWS;

    if (sws == RCC_CFGR_SWS_HSI) {
        return HSI_HZ;
    }

    if (sws == RCC_CFGR_SWS_HSE) {
        return HSE_HZ;
    }

    if (sws == RCC_CFGR_SWS_PLL) {
        uint32_t pllm = RCC->PLLCFGR & RCC_PLLCFGR_PLLM;
        uint32_t plln = (RCC->PLLCFGR & RCC_PLLCFGR_PLLN) >> 6;
        uint32_t pllp = (((RCC->PLLCFGR & RCC_PLLCFGR_PLLP) >> 16) + 1U) * 2U;
        uint32_t src = (RCC->PLLCFGR & RCC_PLLCFGR_PLLSRC) ? HSE_HZ : HSI_HZ;

        if (pllm != 0U && pllp != 0U) {
            return (src / pllm) * plln / pllp;
        }
    }

    return HSI_HZ;
}

static uint32_t Board_GetHCLKHz(void)
{
    static const uint16_t div_table[16] = {
        1, 1, 1, 1, 1, 1, 1, 1,
        2, 4, 8, 16, 64, 128, 256, 512
    };
    uint32_t idx = (RCC->CFGR >> 4) & 0x0FU;
    return Board_GetSystemClockHz() / div_table[idx];
}

static uint32_t Board_GetAPBClockHz(uint8_t apb2)
{
    static const uint8_t div_table[8] = {1, 1, 1, 1, 2, 4, 8, 16};
    uint32_t shift = apb2 ? 13U : 10U;
    uint32_t idx = (RCC->CFGR >> shift) & 0x07U;
    return Board_GetHCLKHz() / div_table[idx];
}

static uint32_t Board_GetAPBTimerClockHz(uint8_t apb2)
{
    static const uint8_t div_table[8] = {1, 1, 1, 1, 2, 4, 8, 16};
    uint32_t shift = apb2 ? 13U : 10U;
    uint32_t idx = (RCC->CFGR >> shift) & 0x07U;
    uint32_t pclk = Board_GetAPBClockHz(apb2);
    return (div_table[idx] == 1U) ? pclk : (pclk * 2U);
}

uint32_t Board_GetAPB1ClockHz(void) { return Board_GetAPBClockHz(0); }
uint32_t Board_GetAPB2ClockHz(void) { return Board_GetAPBClockHz(1); }
uint32_t Board_GetAPB1TimerClockHz(void) { return Board_GetAPBTimerClockHz(0); }
uint32_t Board_GetAPB2TimerClockHz(void) { return Board_GetAPBTimerClockHz(1); }

void Board_Init(void)
{
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN |
                    RCC_AHB1ENR_GPIOBEN |
                    RCC_AHB1ENR_GPIOCEN;
    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;

    BTN_KEY_PORT->MODER &= ~(3UL << (BTN_KEY_PIN * 2U));
    BTN_KEY_PORT->PUPDR &= ~(3UL << (BTN_KEY_PIN * 2U));
    BTN_KEY_PORT->PUPDR |=  (1UL << (BTN_KEY_PIN * 2U));

    LED_STATUS_PORT->MODER &= ~(3UL << (LED_STATUS_PIN * 2U));
    LED_STATUS_PORT->MODER |=  (1UL << (LED_STATUS_PIN * 2U));
    LED_STATUS_PORT->OTYPER &= ~(1UL << LED_STATUS_PIN);
    LED_STATUS_PORT->BSRR = (1UL << LED_STATUS_PIN);
}

uint8_t Board_ButtonPressed(void)
{
    return ((BTN_KEY_PORT->IDR & (1UL << BTN_KEY_PIN)) == 0U) ? 1U : 0U;
}

void Board_StatusLed(uint8_t on)
{
    if (on) {
        LED_STATUS_PORT->BSRR = (1UL << (LED_STATUS_PIN + 16U));
    } else {
        LED_STATUS_PORT->BSRR = (1UL << LED_STATUS_PIN);
    }
}

void HardFault_Handler(void)
{
    /* Do not rely on CCR preload alone: disabling CC outputs is immediate.
     * Force direction pins low (coast) and stop the 1 kHz scheduler. */
    TIM2->CCER &= ~(TIM_CCER_CC1E | TIM_CCER_CC2E);
    TIM2->CCR1 = 0;
    TIM2->CCR2 = 0;
    GPIOB->BSRR = (1UL << (MOTOR_L_IN1_PIN + 16U)) |
                  (1UL << (MOTOR_L_IN2_PIN + 16U)) |
                  (1UL << (MOTOR_R_IN1_PIN + 16U)) |
                  (1UL << (MOTOR_R_IN2_PIN + 16U));
    TIM11->CR1 &= ~TIM_CR1_CEN;
    TIM10->CR1 &= ~TIM_CR1_CEN;

    while (1) {
    }
}
