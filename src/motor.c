#include "motor.h"
#include "board.h"
#include "pinout.h"
#include "stm32f4xx.h"

static uint32_t pwm_period = 0;

static int16_t ClampPwm(int16_t value)
{
    if (value > 100) return 100;
    if (value < -100) return -100;
    return value;
}

static void GPIO_Output(GPIO_TypeDef *port, uint8_t pin)
{
    port->MODER &= ~(3UL << (pin * 2U));
    port->MODER |=  (1UL << (pin * 2U));
    port->OTYPER &= ~(1UL << pin);
    port->OSPEEDR |= (3UL << (pin * 2U));
    port->PUPDR &= ~(3UL << (pin * 2U));
}

static void GPIO_AF(GPIO_TypeDef *port, uint8_t pin, uint8_t af)
{
    uint32_t shift;

    port->MODER &= ~(3UL << (pin * 2U));
    port->MODER |=  (2UL << (pin * 2U));
    port->OTYPER &= ~(1UL << pin);
    port->OSPEEDR |= (3UL << (pin * 2U));
    port->PUPDR &= ~(3UL << (pin * 2U));

    if (pin < 8U) {
        shift = pin * 4U;
        port->AFR[0] &= ~(0xFUL << shift);
        port->AFR[0] |= ((uint32_t)af << shift);
    } else {
        shift = (pin - 8U) * 4U;
        port->AFR[1] &= ~(0xFUL << shift);
        port->AFR[1] |= ((uint32_t)af << shift);
    }
}

static void AddPinLevel(uint32_t *bsrr, uint8_t pin, uint8_t high)
{
    if (high) {
        *bsrr |= (1UL << pin);
    } else {
        *bsrr |= (1UL << (pin + 16U));
    }
}

static void AddDirection(uint32_t *bsrr, uint8_t in1, uint8_t in2, int16_t pwm)
{
    if (pwm > 0) {
        AddPinLevel(bsrr, in1, 1);
        AddPinLevel(bsrr, in2, 0);
    } else if (pwm < 0) {
        AddPinLevel(bsrr, in1, 0);
        AddPinLevel(bsrr, in2, 1);
    } else {
        AddPinLevel(bsrr, in1, 0);
        AddPinLevel(bsrr, in2, 0);
    }
}

static uint32_t DutyFromPercent(int16_t pwm)
{
    uint32_t magnitude = (uint32_t)((pwm < 0) ? -pwm : pwm);
    return (magnitude * (pwm_period + 1U)) / 100U;
}

void Motor_Init(void)
{
    uint32_t timer_clock = Board_GetAPB1TimerClockHz();
    uint32_t arr;

    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOBEN;
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;

    GPIO_Output(MOTOR_R_IN1_PORT, MOTOR_R_IN1_PIN);
    GPIO_Output(MOTOR_R_IN2_PORT, MOTOR_R_IN2_PIN);
    GPIO_Output(MOTOR_L_IN1_PORT, MOTOR_L_IN1_PIN);
    GPIO_Output(MOTOR_L_IN2_PORT, MOTOR_L_IN2_PIN);

    GPIO_AF(MOTOR_R_PWM_PORT, MOTOR_R_PWM_PIN, MOTOR_R_PWM_AF);
    GPIO_AF(MOTOR_L_PWM_PORT, MOTOR_L_PWM_PIN, MOTOR_L_PWM_AF);

    arr = timer_clock / MOTOR_PWM_FREQUENCY;
    if (arr == 0U) arr = 1U;
    pwm_period = arr - 1U;

    TIM2->CR1 = 0;
    TIM2->PSC = 0;
    TIM2->ARR = pwm_period;
    TIM2->CCR1 = 0;
    TIM2->CCR2 = 0;

    TIM2->CCMR1 &= ~((7UL << 4) | (7UL << 12));
    TIM2->CCMR1 |=  (6UL << 4) | TIM_CCMR1_OC1PE |
                    (6UL << 12) | TIM_CCMR1_OC2PE;
    TIM2->CCER |= TIM_CCER_CC1E | TIM_CCER_CC2E;
    TIM2->CR1 |= TIM_CR1_ARPE;
    TIM2->EGR = TIM_EGR_UG;
    TIM2->SR = 0;
    TIM2->CR1 |= TIM_CR1_CEN;

    Motor_Stop();
}

void Motor_SetPair(int16_t left_pwm, int16_t right_pwm)
{
    uint32_t bsrr = 0;

    left_pwm = ClampPwm(left_pwm);
    right_pwm = ClampPwm(right_pwm);

    AddDirection(&bsrr, MOTOR_L_IN1_PIN, MOTOR_L_IN2_PIN, left_pwm);
    AddDirection(&bsrr, MOTOR_R_IN1_PIN, MOTOR_R_IN2_PIN, right_pwm);

    /* All four direction signals are on GPIOB, so commit direction atomically. */
    GPIOB->BSRR = bsrr;

    /* CCR preload makes the two duties take effect together on the next update. */
    TIM2->CCR2 = DutyFromPercent(left_pwm);
    TIM2->CCR1 = DutyFromPercent(right_pwm);
}

void Motor_Stop(void)
{
    Motor_SetPair(0, 0);
}

void Motor_Brake(void)
{
    uint32_t bsrr = 0;

    AddPinLevel(&bsrr, MOTOR_L_IN1_PIN, 1);
    AddPinLevel(&bsrr, MOTOR_L_IN2_PIN, 1);
    AddPinLevel(&bsrr, MOTOR_R_IN1_PIN, 1);
    AddPinLevel(&bsrr, MOTOR_R_IN2_PIN, 1);
    GPIOB->BSRR = bsrr;

    TIM2->CCR1 = pwm_period + 1U;
    TIM2->CCR2 = pwm_period + 1U;
}
