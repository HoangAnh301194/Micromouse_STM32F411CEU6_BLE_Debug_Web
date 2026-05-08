/**
 * @file encoder.c
 * @brief Encoder Driver Implementation (Fixed C89 & Logic)
 */

#include "encoder.h"
#include "system_timer.h"

/* ============================================================================ */
/* INTERNAL FUNCTIONS                                                           */
/* ============================================================================ */

/**
 * @brief Configure GPIO for encoder input
 */
static void Encoder_GPIO_Init(Encoder_Select_t encoder) {
    if (encoder == ENCODER_RIGHT) {
        /* Enable GPIOB clock */
        RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
        
        /* PB4, PB5 - Alternate Function mode */
        GPIOB->MODER &= ~((3UL << 8) | (3UL << 10));
        GPIOB->MODER |= (2UL << 8) | (2UL << 10);
        
        /* High speed */
        GPIOB->OSPEEDR |= (3UL << 8) | (3UL << 10);
        
        /* Pull-up */
        GPIOB->PUPDR &= ~((3UL << 8) | (3UL << 10));
        GPIOB->PUPDR |= (1UL << 8) | (1UL << 10);
        
        /* AF2 = TIM3 */
        GPIOB->AFR[0] &= ~((0xFUL << 16) | (0xFUL << 20));
        GPIOB->AFR[0] |= (2UL << 16) | (2UL << 20);
    }
    else {
        /* Enable GPIOB clock */
        RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
        
        /* PB6, PB7 - Alternate Function mode */
        GPIOB->MODER &= ~((3UL << 12) | (3UL << 14));
        GPIOB->MODER |= (2UL << 12) | (2UL << 14);
        
        /* High speed */
        GPIOB->OSPEEDR |= (3UL << 12) | (3UL << 14);
        
        /* Pull-up */
        GPIOB->PUPDR &= ~((3UL << 12) | (3UL << 14));
        GPIOB->PUPDR |= (1UL << 12) | (1UL << 14);
        
        /* AF2 = TIM4 */
        GPIOB->AFR[0] &= ~((0xFUL << 24) | (0xFUL << 28));
        GPIOB->AFR[0] |= (2UL << 24) | (2UL << 28);
    }
}

/* ============================================================================ */
/* PUBLIC FUNCTIONS                                                             */
/* ============================================================================ */

void Encoder_Init(Encoder_Handle_t *handle, Encoder_Select_t encoder, uint16_t resolution) {
    /* Set parameters */
    handle->resolution = resolution;
    handle->last_count = 0;
    handle->prev_speed_total = 0;
    handle->last_update_time = micros();
    handle->speed_pps = 0.0f;
    
    /* Select timer */
    if (encoder == ENCODER_RIGHT) {
        handle->timer = TIM3;
        RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;
    } else {
        handle->timer = TIM4;
        RCC->APB1ENR |= RCC_APB1ENR_TIM4EN;
    }
    
    /* Configure GPIO */
    Encoder_GPIO_Init(encoder);
    
    /* Disable timer */
    handle->timer->CR1 &= ~(1 << 0);
    
    /* Set timer to Encoder Mode 3 */
    handle->timer->SMCR &= ~(7UL << 0);
    handle->timer->SMCR |= (3UL << 0);
    
    /* Configure input capture */
    handle->timer->CCMR1 &= ~((3UL << 0) | (3UL << 8));
    handle->timer->CCMR1 |= (1UL << 0) | (1UL << 8);
    
    /* No input filter */
    handle->timer->CCMR1 &= ~((0xFUL << 4) | (0xFUL << 12));
    
    /* Enable capture */
    handle->timer->CCER |= (1UL << 0) | (1UL << 1) | (1UL << 4);
    
    /* Set ARR to max */
    handle->timer->ARR = 0xFFFF;
    handle->timer->CNT = 0;
    handle->timer->EGR = (1 << 0);
    handle->timer->SR = 0;
    
    /* Enable timer */
    handle->timer->CR1 |= (1 << 0);
}

int16_t Encoder_GetCount(Encoder_Handle_t *handle) {
    return (int16_t)(handle->timer->CNT);
}

int32_t Encoder_GetTotalCount(Encoder_Handle_t *handle) {
    /* [FIX C89] Declare variables at top of block */
    int16_t current_count;
    int16_t delta;
    
    current_count = Encoder_GetCount(handle);
    /* Tnh delta d?a trn b? d?m 16-bit raw */
    delta = current_count - (int16_t)handle->last_count;
    
    handle->total_count += delta;
    handle->last_count = current_count;
    
    return handle->total_count;
}

void Encoder_Reset(Encoder_Handle_t *handle) {
    /* Do NOT reset CNT - let hardware counter run freely.
     * Snapshot current position as new zero reference.
     * Unsigned->signed subtraction handles 16-bit wrap correctly. */
    handle->last_count = (int16_t)(handle->timer->CNT);
    handle->total_count = 0;
    handle->prev_speed_total = 0;
    handle->speed_pps = 0;
}

/**
 * @brief Update speed calculation (Fixed C89 & Logic)
 */
void Encoder_UpdateSpeed(Encoder_Handle_t *handle) {
    uint32_t current_time;
    uint32_t dt_us;
    int32_t current_total;
    int32_t delta_ticks;
    float raw_pps;
    float smooth_pps;

    current_time = micros();
    dt_us = current_time - handle->last_update_time;
    
    if (dt_us == 0) return; 

    current_total = Encoder_GetTotalCount(handle);
    
    delta_ticks = current_total - handle->prev_speed_total;
    handle->prev_speed_total = current_total;
    
    raw_pps = (float)delta_ticks * 1000000.0f / (float)dt_us;
    
    smooth_pps = (LOW_PASS_ALPHA * raw_pps) + ((1.0f - LOW_PASS_ALPHA) * handle->speed_pps); 
    
    handle->speed_pps = smooth_pps; 
    handle->last_update_time = current_time;
}


float Encoder_GetSpeedPPS(Encoder_Handle_t *handle) {
    return handle->speed_pps; 
}
