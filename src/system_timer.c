/**
 * @file system_timer.c
 * @brief System Timer Implementation with Auto Clock Detection (TIM5)
 * @author HELLO12312E & Gemini
 * @date 2025-01-29
 */

#include "system_timer.h"
#include "stm32f4xx.h"

/* --- [FIX] define constant if missing in old library --- */
#ifndef RCC_PLLCFGR_PLLN_Pos
#define RCC_PLLCFGR_PLLN_Pos 6
#endif

#ifndef RCC_PLLCFGR_PLLP_Pos
#define RCC_PLLCFGR_PLLP_Pos 16
#endif
/* ------------------------------------------------------------- */

/* ========================================================================== */
/* PRIVATE HELPER FUNCTIONS                                                   */
/* ========================================================================== */

/**
 * @brief Get System Core Clock (SYSCLK)
 * Reads RCC registers to determine actual running frequency.
 */
static uint32_t GetSystemClock(void) {
    uint32_t sysclk = 16000000; /* Default HSI */
    uint32_t pllm, plln, pllp, pllsrc;
    uint32_t temp_sws;
    
    /* Read SWS bits from CFGR register to check current clock source */
    temp_sws = (RCC->CFGR & RCC_CFGR_SWS);
    
    switch (temp_sws) {
        case RCC_CFGR_SWS_HSI:
            sysclk = 16000000;
            break;
            
        case RCC_CFGR_SWS_HSE:
            sysclk = 25000000; /* Assuming 25MHz crystal */
            break;
            
        case RCC_CFGR_SWS_PLL:
            /* Get PLLM */
            pllm = (RCC->PLLCFGR & RCC_PLLCFGR_PLLM);
            
            /* Get PLLN */
            plln = (RCC->PLLCFGR & RCC_PLLCFGR_PLLN) >> RCC_PLLCFGR_PLLN_Pos;
            
            /* Get PLLP */
            pllp = ((((RCC->PLLCFGR & RCC_PLLCFGR_PLLP) >> RCC_PLLCFGR_PLLP_Pos) + 1) * 2);
            
            /* Get PLL Source */
            if (RCC->PLLCFGR & RCC_PLLCFGR_PLLSRC_HSE) {
                pllsrc = 25000000; 
            } else {
                pllsrc = 16000000; 
            }
            
            /* Calculate SYSCLK */
            if (pllm > 0 && pllp > 0) {
                sysclk = (pllsrc / pllm) * plln / pllp;
            }
            break;
            
        default:
            sysclk = 16000000;
            break;
    }
    
    return sysclk;
}

/**
 * @brief Get APB1 Timer Clock Frequency
 */
static uint32_t GetAPB1TimerClock(void) {
    uint32_t sysclk;
    uint32_t hpre_bits, ppre1_bits;
    uint32_t ahb_prescaler = 1;
    uint32_t apb1_prescaler = 1;
    uint32_t apb1_clock;
    
    sysclk = GetSystemClock();
    
    /* AHB Prescaler */
    hpre_bits = (RCC->CFGR & RCC_CFGR_HPRE);
    switch (hpre_bits) {
        case RCC_CFGR_HPRE_DIV1:   ahb_prescaler = 1; break;
        case RCC_CFGR_HPRE_DIV2:   ahb_prescaler = 2; break;
        case RCC_CFGR_HPRE_DIV4:   ahb_prescaler = 4; break;
        case RCC_CFGR_HPRE_DIV8:   ahb_prescaler = 8; break;
        case RCC_CFGR_HPRE_DIV16:  ahb_prescaler = 16; break;
        case RCC_CFGR_HPRE_DIV64:  ahb_prescaler = 64; break;
        case RCC_CFGR_HPRE_DIV128: ahb_prescaler = 128; break;
        case RCC_CFGR_HPRE_DIV256: ahb_prescaler = 256; break;
        case RCC_CFGR_HPRE_DIV512: ahb_prescaler = 512; break;
        default: ahb_prescaler = 1; break;
    }
    
    /* APB1 Prescaler */
    ppre1_bits = (RCC->CFGR & RCC_CFGR_PPRE1);
    switch (ppre1_bits) {
        case RCC_CFGR_PPRE1_DIV1:  apb1_prescaler = 1; break;
        case RCC_CFGR_PPRE1_DIV2:  apb1_prescaler = 2; break;
        case RCC_CFGR_PPRE1_DIV4:  apb1_prescaler = 4; break;
        case RCC_CFGR_PPRE1_DIV8:  apb1_prescaler = 8; break;
        case RCC_CFGR_PPRE1_DIV16: apb1_prescaler = 16; break;
        default: apb1_prescaler = 1; break;
    }
    
    apb1_clock = (sysclk / ahb_prescaler) / apb1_prescaler;
    
    if (apb1_prescaler != 1) {
        return apb1_clock * 2;
    }
    
    return apb1_clock;
}

/* ========================================================================== */
/* PUBLIC FUNCTIONS                                                           */
/* ========================================================================== */

void SystemTimer_Init(void) {
    uint32_t tim_clock;
    uint32_t prescaler;
    
    RCC->APB1ENR |= RCC_APB1ENR_TIM5EN;
    
    tim_clock = GetAPB1TimerClock();
    prescaler = (tim_clock / 1000000UL) - 1;
    
    TIM5->CR1 &= ~TIM_CR1_CEN;
    TIM5->PSC = prescaler;
    TIM5->ARR = 0xFFFFFFFF;
    TIM5->CNT = 0;
    TIM5->EGR |= TIM_EGR_UG;
    TIM5->SR = 0;
    TIM5->CR1 |= TIM_CR1_CEN;
}

uint32_t micros(void) {
    return TIM5->CNT;
}

uint32_t millis(void) {
    return TIM5->CNT / 1000UL;
}

void delay_us_blocking(uint32_t us) {
    uint32_t start = micros();
    while ((micros() - start) < us);
}

void delay_ms_blocking(uint32_t ms) {
    while (ms--) {
        delay_us_blocking(1000);
    }
}
