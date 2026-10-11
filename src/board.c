#include "board.h"
#include "pinout.h"

#define HSI_HZ 16000000UL
#define HSE_HZ 25000000UL
/* Autotune scans HSI trim on a LOWER clock profile to provide headroom.
 * PLLN=160 => nominal 80 MHz. Production and previous trimXX builds remain
 * PLLN=200 => nominal 100 MHz. Never run motor control in autotune firmware. */
#if defined(CLOCK_AUTOTUNE)
#ifndef CLOCK_HSI_TRIM
#error "CLOCK_AUTOTUNE requires a compile-time CLOCK_HSI_TRIM"
#endif
#define BOARD_PLLN 160UL
#define BOARD_EXPECTED_SYSCLK 80000000UL
#else
#define BOARD_PLLN 200UL
#define BOARD_EXPECTED_SYSCLK 100000000UL
#endif


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

/* STM32F411 maximum HCLK is 100MHz, PCLK1<=50MHz, PCLK2<=100MHz.
 * HSI 16MHz -> PLLM=16, PLLN=200, PLLP=2 -> 100MHz.
 * PLLQ=5 gives 40MHz: USB FS 48MHz is NOT available in this profile.
 * Intended for regulated 3.3V Micromouse board, VDD >= 2.7V.
 * Uses bounded readiness waits and retains HSI 16MHz on PLL failure.
 */
/* 0: clock switch completed. Nonzero: last failed initialization stage. */
static volatile uint32_t board_clock_fault = 0U;
uint32_t Board_GetClockFault(void) { return board_clock_fault; }

/* RM0383 RCC_CR: HSITRIM[4:0] = bits 7:3, HSICAL = bits 15:8.
 * Read back for logs; these values do not measure actual frequency. */
uint32_t Board_GetHsiTrim(void) { return (RCC->CR >> 3) & 0x1FUL; }
uint32_t Board_GetHsiCal(void) { return (RCC->CR >> 8) & 0xFFUL; }

#if defined(CLOCK_HSI_TRIM)
#if (CLOCK_HSI_TRIM < 0) || (CLOCK_HSI_TRIM > 31)
#error "CLOCK_HSI_TRIM must be in the STM32F411 HSITRIM range 0..31"
#endif
#endif

static void Board_SetMaxClock100MHz(void)
{
    uint32_t timeout;
    board_clock_fault = 0U;
    RCC->APB1ENR |= RCC_APB1ENR_PWREN;
    (void)RCC->APB1ENR;
    PWR->CR = (PWR->CR & ~PWR_CR_VOS) | PWR_CR_VOS;
    /* Do NOT wait for VOSRDY with PLL off: STM32F411 applies newly
     * selected voltage scale only after enabling PLL (RM0383). */
    /* 3 flash wait states at 2.7V-3.6V for 100MHz, enable caches. */
    FLASH->ACR = (FLASH->ACR & ~FLASH_ACR_LATENCY) |
                 FLASH_ACR_LATENCY_3WS |
                 FLASH_ACR_ICEN | FLASH_ACR_DCEN | FLASH_ACR_PRFTEN;
    timeout = 1000000UL;
    while (((FLASH->ACR & FLASH_ACR_LATENCY) != FLASH_ACR_LATENCY_3WS)
           && --timeout) {}
    if (!timeout) { board_clock_fault = 2U; return; }

    /* Safest prescaler order: APB1 /2 before raising SYSCLK. */
    RCC->CFGR = (RCC->CFGR &
                ~(RCC_CFGR_HPRE | RCC_CFGR_PPRE1 | RCC_CFGR_PPRE2)) |
                RCC_CFGR_PPRE1_DIV2;

    /* Core starts from HSI after reset. Never reconfigure an active PLL. */
    RCC->CR |= RCC_CR_HSION;
    timeout = 1000000UL;
    while (!(RCC->CR & RCC_CR_HSIRDY) && --timeout) {}
    if (!timeout) { board_clock_fault = 1U; return; }
    if ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_HSI) {
        RCC->CFGR &= ~RCC_CFGR_SW;
        timeout = 1000000UL;
        while (((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_HSI)
               && --timeout) {}
        if (!timeout) { board_clock_fault = 3U; return; }
    }
    RCC->CR &= ~RCC_CR_PLLON;
    timeout = 1000000UL;
    while ((RCC->CR & RCC_CR_PLLRDY) && --timeout) {}
    if (!timeout) { board_clock_fault = 4U; return; }

    /* Optional debug-only, build-time HSI trim experiment.
     * We are executing on HSI with PLL OFF at this point. The default
     * production build does not touch HSITRIM. A full hardware RESET
     * restores the factory boot trim (normally 16), and flashing the
     * default PlatformIO environment restores original firmware logic.
     * Never auto-sweep while running PLL/SYSCLK/USART or motor control. */
#if defined(CLOCK_HSI_TRIM)
    RCC->CR = (RCC->CR & ~(0x1FUL << 3)) |
              (((uint32_t)CLOCK_HSI_TRIM & 0x1FUL) << 3);
    /* Allow HSI to settle before PLL startup; CPU is still on HSI. */
    for (volatile uint32_t settle = 0U; settle < 50000U; ++settle) {
        __NOP();
    }
#endif

    RCC->PLLCFGR = (16UL << 0) | (BOARD_PLLN << 6) |
                   (0UL << 16) | (5UL << 24); /* PLLSRC=HSI */
    RCC->CR |= RCC_CR_PLLON;
    timeout = 1000000UL;
    while (!(RCC->CR & RCC_CR_PLLRDY) && --timeout) {}
    if (!timeout) { board_clock_fault = 5U; return; }
#ifdef PWR_CSR_VOSRDY
    timeout = 1000000UL;
    while (!(PWR->CSR & PWR_CSR_VOSRDY) && --timeout) {}
    if (!timeout) { board_clock_fault = 6U; return; }
#endif
    RCC->CFGR = (RCC->CFGR & ~RCC_CFGR_SW) | RCC_CFGR_SW_PLL;
    timeout = 1000000UL;
    while (((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL)
           && --timeout) {}
    if (!timeout) { board_clock_fault = 7U; return; }
    SystemCoreClockUpdate();
    if (SystemCoreClock != BOARD_EXPECTED_SYSCLK)
        board_clock_fault = 8U;
}

void Board_Init(void)
{
    Board_SetMaxClock100MHz();
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
