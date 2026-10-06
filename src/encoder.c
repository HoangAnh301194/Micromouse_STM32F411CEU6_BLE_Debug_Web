#include "encoder.h"
#include "pinout.h"
#include "stm32f4xx.h"

#define SPEED_LPF_ALPHA 0.30f

static volatile int32_t total_left = 0;
static volatile int32_t total_right = 0;
static volatile float speed_left_pps = 0.0f;
static volatile float speed_right_pps = 0.0f;

static int16_t last_left_raw = 0;
static int16_t last_right_raw = 0;

static void GPIO_EncoderAF(GPIO_TypeDef *port, uint8_t pin, uint8_t af)
{
    uint32_t shift;

    port->MODER &= ~(3UL << (pin * 2U));
    port->MODER |=  (2UL << (pin * 2U));
    port->OSPEEDR |= (3UL << (pin * 2U));
    port->PUPDR &= ~(3UL << (pin * 2U));
    port->PUPDR |=  (1UL << (pin * 2U));

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

static void Encoder_TimerInit(TIM_TypeDef *tim)
{
    tim->CR1 &= ~TIM_CR1_CEN;
    tim->SMCR = (tim->SMCR & ~TIM_SMCR_SMS) | (3UL << 0);
    tim->CCMR1 = (tim->CCMR1 & ~((3UL << 0) | (3UL << 8))) |
                 (1UL << 0) | (1UL << 8);
    tim->CCER |= TIM_CCER_CC1E | TIM_CCER_CC2E;
    tim->ARR = 0xFFFFU;
    tim->CNT = 0;
    tim->EGR = TIM_EGR_UG;
    tim->SR = 0;
    tim->CR1 |= TIM_CR1_CEN;
}

void Encoder_Init(void)
{
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
    RCC->APB1ENR |= RCC_APB1ENR_TIM3EN | RCC_APB1ENR_TIM4EN;

    GPIO_EncoderAF(ENCODER_R_CHA_PORT, ENCODER_R_CHA_PIN, ENCODER_R_CHA_AF);
    GPIO_EncoderAF(ENCODER_R_CHB_PORT, ENCODER_R_CHB_PIN, ENCODER_R_CHB_AF);
    GPIO_EncoderAF(ENCODER_L_CHA_PORT, ENCODER_L_CHA_PIN, ENCODER_L_CHA_AF);
    GPIO_EncoderAF(ENCODER_L_CHB_PORT, ENCODER_L_CHB_PIN, ENCODER_L_CHB_AF);

    Encoder_TimerInit(ENCODER_R_TIMER);
    Encoder_TimerInit(ENCODER_L_TIMER);

    last_right_raw = (int16_t)ENCODER_R_TIMER->CNT;
    last_left_raw = (int16_t)ENCODER_L_TIMER->CNT;
    Encoder_Reset();
}

void Encoder_Update(float dt_s)
{
    int16_t raw_left;
    int16_t raw_right;
    int16_t delta_left;
    int16_t delta_right;
    float raw_left_pps;
    float raw_right_pps;

    if (dt_s <= 0.0f) return;

    raw_left = (int16_t)ENCODER_L_TIMER->CNT;
    raw_right = (int16_t)ENCODER_R_TIMER->CNT;

    delta_left = (int16_t)(raw_left - last_left_raw);
    delta_right = (int16_t)(raw_right - last_right_raw);

    last_left_raw = raw_left;
    last_right_raw = raw_right;

    total_left += ((int32_t)delta_left * ENCODER_LEFT_DIRECTION);
    total_right += ((int32_t)delta_right * ENCODER_RIGHT_DIRECTION);

    raw_left_pps = ((float)delta_left * ENCODER_LEFT_DIRECTION) / dt_s;
    raw_right_pps = ((float)delta_right * ENCODER_RIGHT_DIRECTION) / dt_s;

    speed_left_pps += SPEED_LPF_ALPHA * (raw_left_pps - speed_left_pps);
    speed_right_pps += SPEED_LPF_ALPHA * (raw_right_pps - speed_right_pps);
}

void Encoder_Reset(void)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();

    last_left_raw = (int16_t)ENCODER_L_TIMER->CNT;
    last_right_raw = (int16_t)ENCODER_R_TIMER->CNT;
    total_left = 0;
    total_right = 0;
    speed_left_pps = 0.0f;
    speed_right_pps = 0.0f;

    if (!primask) __enable_irq();
}

int32_t Encoder_GetLeftCount(void) { return total_left; }
int32_t Encoder_GetRightCount(void) { return total_right; }
float Encoder_GetLeftSpeedPPS(void) { return speed_left_pps; }
float Encoder_GetRightSpeedPPS(void) { return speed_right_pps; }
