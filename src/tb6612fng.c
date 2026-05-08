/**
 * @file tb6612fng.c - CRITICAL FIX
 * @brief TB6612FNG Dual Motor Driver - PWM INIT FIXED
 * @date 2025
 * 
 * FIXES:
 * - Remove timer enable in PWM_Init (was causing glitches)
 * - Timer only starts when first SetMotor is called
 */

#include "tb6612fng.h"

static void GPIO_ClockEnable(GPIO_TypeDef *GPIOx)
{
    if (GPIOx == GPIOA) {
        RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    }
    else if (GPIOx == GPIOB) {
        RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
    }
    else if (GPIOx == GPIOC) {
        RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;
    }
}

static void GPIO_ConfigOutput(GPIO_TypeDef *GPIOx, uint8_t pin)
{
    GPIOx->BSRR = (1UL << (pin + 16));
    
    GPIOx->MODER &= ~(3UL << (pin * 2));
    GPIOx->MODER |= (1UL << (pin * 2));
    
    GPIOx->OTYPER &= ~(1UL << pin);
    
    GPIOx->OSPEEDR &= ~(3UL << (pin * 2));
    GPIOx->OSPEEDR |= (2UL << (pin * 2));
    
    GPIOx->PUPDR &= ~(3UL << (pin * 2));
}

static void GPIO_ConfigAF(GPIO_TypeDef *GPIOx, uint8_t pin, uint8_t af)
{
    GPIOx->MODER &= ~(3UL << (pin * 2));
    GPIOx->MODER |= (2UL << (pin * 2));
    
    GPIOx->OTYPER &= ~(1UL << pin);
    
    GPIOx->OSPEEDR &= ~(3UL << (pin * 2));
    GPIOx->OSPEEDR |= (2UL << (pin * 2));
    
    GPIOx->PUPDR &= ~(3UL << (pin * 2));
    
    if (pin < 8) {
        GPIOx->AFR[0] &= ~(0xFUL << (pin * 4));
        GPIOx->AFR[0] |= (af << (pin * 4));
    } else {
        GPIOx->AFR[1] &= ~(0xFUL << ((pin - 8) * 4));
        GPIOx->AFR[1] |= (af << ((pin - 8) * 4));
    }
}

static void GPIO_WritePin(GPIO_TypeDef *GPIOx, uint8_t pin, uint8_t state)
{
    if (state) {
        GPIOx->BSRR = (1UL << pin);
    } else {
        GPIOx->BSRR = (1UL << (pin + 16));
    }
}

static void Timer_ClockEnable(TIM_TypeDef *TIMx)
{
    if (TIMx == TIM1) {
        RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;
    }
    else if (TIMx == TIM2) {
        RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
    }
    else if (TIMx == TIM3) {
        RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;
    }
    else if (TIMx == TIM4) {
        RCC->APB1ENR |= RCC_APB1ENR_TIM4EN;
    }
    else if (TIMx == TIM5) {
        RCC->APB1ENR |= RCC_APB1ENR_TIM5EN;
    }
}

static void PWM_Init(TIM_TypeDef *TIMx, uint8_t channel, uint16_t prescaler, uint32_t period)
{
    Timer_ClockEnable(TIMx);
    
    TIMx->CR1 &= ~(1 << 0);
    
    TIMx->PSC = prescaler;
    TIMx->ARR = period;
    
    TIMx->CR1 |= (1 << 7);
    
    switch(channel) {
        case 1:
            TIMx->CCR1 = 0;
            
            TIMx->CCMR1 &= ~(0xFFUL << 0);
            TIMx->CCMR1 |= (0x6UL << 4) | (1UL << 3);
            
            TIMx->CCER |= (1UL << 0);
            TIMx->CCER &= ~(1UL << 1);
            break;
            
        case 2:
            TIMx->CCR2 = 0;
            TIMx->CCMR1 &= ~(0xFFUL << 8);
            TIMx->CCMR1 |= (0x6UL << 12) | (1UL << 11);
            TIMx->CCER |= (1UL << 4);
            TIMx->CCER &= ~(1UL << 5);
            break;
            
        case 3:
            TIMx->CCR3 = 0;
            TIMx->CCMR2 &= ~(0xFFUL << 0);
            TIMx->CCMR2 |= (0x6UL << 4) | (1UL << 3);
            TIMx->CCER |= (1UL << 8);
            TIMx->CCER &= ~(1UL << 9);
            break;
            
        case 4:
            TIMx->CCR4 = 0;
            TIMx->CCMR2 &= ~(0xFFUL << 8);
            TIMx->CCMR2 |= (0x6UL << 12) | (1UL << 11);
            TIMx->CCER |= (1UL << 12);
            TIMx->CCER &= ~(1UL << 13);
            break;
    }
    
    if (TIMx == TIM1) {
        TIMx->BDTR |= (1UL << 15);
    }
    
    TIMx->EGR |= (1 << 0);
    
    __NOP(); __NOP(); __NOP();

    TIMx->SR = 0;
    
    /* CRITICAL FIX: DO NOT enable timer here! */
    /* Timer starts on first PWM_SetDuty() call */
}

static void PWM_SetDuty(TIM_TypeDef *TIMx, uint8_t channel, uint32_t duty)
{
    switch(channel) {
        case 1: TIMx->CCR1 = duty; break;
        case 2: TIMx->CCR2 = duty; break;
        case 3: TIMx->CCR3 = duty; break;
        case 4: TIMx->CCR4 = duty; break;
    }
    
    if (!(TIMx->CR1 & (1 << 0))) {
        TIMx->CR1 |= (1 << 0);
    }
}

void TB6612_Init(TB6612_Handle_t *handle)
{
    GPIO_ClockEnable(handle->motor_a.gpio_port);
    GPIO_ClockEnable(handle->motor_a.pwm_port);
    GPIO_ClockEnable(handle->motor_b.gpio_port);
    GPIO_ClockEnable(handle->motor_b.pwm_port);
    
    GPIO_ConfigOutput(handle->motor_a.gpio_port, handle->motor_a.in1_pin);
    GPIO_ConfigOutput(handle->motor_a.gpio_port, handle->motor_a.in2_pin);
    GPIO_ConfigOutput(handle->motor_b.gpio_port, handle->motor_b.in1_pin);
    GPIO_ConfigOutput(handle->motor_b.gpio_port, handle->motor_b.in2_pin);
    
    GPIO_WritePin(handle->motor_a.gpio_port, handle->motor_a.in1_pin, 0);
    GPIO_WritePin(handle->motor_a.gpio_port, handle->motor_a.in2_pin, 0);
    GPIO_WritePin(handle->motor_b.gpio_port, handle->motor_b.in1_pin, 0);
    GPIO_WritePin(handle->motor_b.gpio_port, handle->motor_b.in2_pin, 0);
    
    GPIO_ConfigAF(handle->motor_a.pwm_port, handle->motor_a.pwm_pin, handle->motor_a.pwm_af);
    GPIO_ConfigAF(handle->motor_b.pwm_port, handle->motor_b.pwm_pin, handle->motor_b.pwm_af);
    
    PWM_Init(handle->motor_a.pwm_timer, handle->motor_a.pwm_channel, 
             handle->pwm_prescaler, handle->pwm_period);
    
    PWM_Init(handle->motor_b.pwm_timer, handle->motor_b.pwm_channel,
             handle->pwm_prescaler, handle->pwm_period);
    
    handle->speed_a = 0;
    handle->speed_b = 0;
    
    TB6612_StopAll(handle);
}

void TB6612_SetMotor(TB6612_Handle_t *handle, Motor_Select_t motor, 
                     Motor_Direction_t direction, uint8_t speed)
{
    Motor_Config_t *motor_cfg;
    uint32_t pwm_value;
    
    if (speed > 100) speed = 100;
    
    if (motor == MOTOR_A) {
        motor_cfg = &handle->motor_a;
        handle->speed_a = speed;
    } else {
        motor_cfg = &handle->motor_b;
        handle->speed_b = speed;
    }
    
    pwm_value = (uint32_t)((speed * handle->pwm_period) / 100UL);
    
    switch(direction) {
        case MOTOR_FORWARD:
            GPIO_WritePin(motor_cfg->gpio_port, motor_cfg->in1_pin, 1);
            GPIO_WritePin(motor_cfg->gpio_port, motor_cfg->in2_pin, 0);
            PWM_SetDuty(motor_cfg->pwm_timer, motor_cfg->pwm_channel, pwm_value);
            break;
            
        case MOTOR_BACKWARD:
            GPIO_WritePin(motor_cfg->gpio_port, motor_cfg->in1_pin, 0);
            GPIO_WritePin(motor_cfg->gpio_port, motor_cfg->in2_pin, 1);
            PWM_SetDuty(motor_cfg->pwm_timer, motor_cfg->pwm_channel, pwm_value);
            break;
            
        case MOTOR_BRAKE:
            GPIO_WritePin(motor_cfg->gpio_port, motor_cfg->in1_pin, 1);
            GPIO_WritePin(motor_cfg->gpio_port, motor_cfg->in2_pin, 1);
            PWM_SetDuty(motor_cfg->pwm_timer, motor_cfg->pwm_channel, handle->pwm_period);
            break;
            
        case MOTOR_STOP:
            GPIO_WritePin(motor_cfg->gpio_port, motor_cfg->in1_pin, 0);
            GPIO_WritePin(motor_cfg->gpio_port, motor_cfg->in2_pin, 0);
            PWM_SetDuty(motor_cfg->pwm_timer, motor_cfg->pwm_channel, 0);
            break;
    }
}

void TB6612_StopMotor(TB6612_Handle_t *handle, Motor_Select_t motor)
{
    TB6612_SetMotor(handle, motor, MOTOR_STOP, 0);
}

void TB6612_BrakeMotor(TB6612_Handle_t *handle, Motor_Select_t motor)
{
    TB6612_SetMotor(handle, motor, MOTOR_BRAKE, 0);
}

void TB6612_StopAll(TB6612_Handle_t *handle)
{
    TB6612_StopMotor(handle, MOTOR_A);
    TB6612_StopMotor(handle, MOTOR_B);
}

void TB6612_BrakeAll(TB6612_Handle_t *handle)
{
    TB6612_BrakeMotor(handle, MOTOR_A);
    TB6612_BrakeMotor(handle, MOTOR_B);
}

uint8_t TB6612_GetSpeed(TB6612_Handle_t *handle, Motor_Select_t motor)
{
    if (motor == MOTOR_A) {
        return handle->speed_a;
    } else {
        return handle->speed_b;
    }
}
