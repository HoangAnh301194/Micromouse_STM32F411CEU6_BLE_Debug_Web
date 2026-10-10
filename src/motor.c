#include "motor.h"
#include "board.h"
#include "pinout.h"
#include "stm32f4xx.h"

#define MOTOR_CMD_MAX 1000
#define MOTOR_REVERSE_GAP_MS 30U

typedef struct {
    volatile int16_t target;
    int16_t applied;
    int8_t last_direction;       /* last energized sign: -1, 0, +1 */
    uint16_t reverse_gap_ms;     /* applies only to opposite-sign restart */
    volatile uint16_t deadband;  /* measured physical minimum duty (permille) */
} Wheel_t;

static uint32_t pwm_period = 0;
static Wheel_t wheel_l, wheel_r;
/* 2% per ms up, 3% per ms down; tunable; 1 kHz state update. */
static volatile uint16_t rise_rate = 20U;
static volatile uint16_t fall_rate = 30U;
static volatile uint8_t brake_latched = 0;

static uint32_t LockIRQ(void)
{
    uint32_t x = __get_PRIMASK();
    __disable_irq();
    return x;
}

static void UnlockIRQ(uint32_t x) { if (!x) __enable_irq(); }

static int16_t ClampCmd(int32_t v)
{
    if (v > MOTOR_CMD_MAX) return MOTOR_CMD_MAX;
    if (v < -MOTOR_CMD_MAX) return -MOTOR_CMD_MAX;
    return (int16_t)v;
}
static int8_t Sign(int16_t v) { return v > 0 ? 1 : (v < 0 ? -1 : 0); }

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
    if (high) *bsrr |= (1UL << pin);
    else *bsrr |= (1UL << (pin + 16U));
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
static uint32_t DutyFromCmd(int16_t applied, uint16_t deadband)
{
    uint32_t mag = (uint32_t)(applied < 0 ? -applied : applied);
    uint32_t duty;
    if (mag == 0U) return 0U;
    /* deadband defaults to zero. No deadband value should be assumed
     * safe before a measured motor response. */
    duty = deadband + ((uint32_t)(1000U - deadband) * mag) / 1000U;
    return (duty * (pwm_period + 1U)) / 1000U;
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
    TIM2->CCMR1 |= (6UL << 4) | TIM_CCMR1_OC1PE |
                   (6UL << 12) | TIM_CCMR1_OC2PE;
    TIM2->CCER |= TIM_CCER_CC1E | TIM_CCER_CC2E;
    TIM2->CR1 |= TIM_CR1_ARPE;
    TIM2->EGR = TIM_EGR_UG;
    TIM2->SR = 0;
    TIM2->CR1 |= TIM_CR1_CEN;
    wheel_l.target = wheel_r.target = 0;
    wheel_l.applied = wheel_r.applied = 0;
    wheel_l.last_direction = wheel_r.last_direction = 0;
    wheel_l.reverse_gap_ms = wheel_r.reverse_gap_ms = 0;
    wheel_l.deadband = wheel_r.deadband = 0;
    Motor_Stop();
}

void Motor_SetPairPermille(int16_t left, int16_t right)
{
    uint32_t x = LockIRQ();
    if (brake_latched) {
        uint32_t neutral = 0;
        /* Brake duty was 100%. Clear preload and neutralize the bridge
         * before any new direction can be enabled on the next 1 kHz tick. */
        TIM2->CCR1 = TIM2->CCR2 = 0;
        AddDirection(&neutral, MOTOR_L_IN1_PIN, MOTOR_L_IN2_PIN, 0);
        AddDirection(&neutral, MOTOR_R_IN1_PIN, MOTOR_R_IN2_PIN, 0);
        GPIOB->BSRR = neutral;
        wheel_l.reverse_gap_ms = MOTOR_REVERSE_GAP_MS;
        wheel_r.reverse_gap_ms = MOTOR_REVERSE_GAP_MS;
    }
    brake_latched = 0;
    wheel_l.target = ClampCmd(left);
    wheel_r.target = ClampCmd(right);
    UnlockIRQ(x);
}
void Motor_SetPair(int16_t l, int16_t r)
{
    Motor_SetPairPermille(ClampCmd((int32_t)l * 10L),
                         ClampCmd((int32_t)r * 10L));
}
void Motor_SetDeadband(uint16_t left, uint16_t right)
{
    uint32_t x = LockIRQ();
    wheel_l.deadband = left > 500U ? 500U : left;
    wheel_r.deadband = right > 500U ? 500U : right;
    UnlockIRQ(x);
}
void Motor_SetSlewRates(uint16_t rise, uint16_t fall)
{
    uint32_t x = LockIRQ();
    rise_rate = rise == 0U ? 1U : (rise > 1000U ? 1000U : rise);
    fall_rate = fall == 0U ? 1U : (fall > 1000U ? 1000U : fall);
    UnlockIRQ(x);
}
static void UpdateWheel(Wheel_t *w)
{
    int16_t desired = w->target;
    int16_t current = w->applied;
    int8_t current_sign = Sign(current);
    int8_t desired_sign = Sign(desired);
    uint16_t rate;
    int32_t delta;

    if (w->reverse_gap_ms > 0U) w->reverse_gap_ms--;

    /* If running and command reverses, decelerate to zero first. */
    if (current_sign != 0 && desired_sign != 0 && desired_sign != current_sign)
        desired = 0;

    /* Remember the current direction. After reaching zero impose a
     * PWM-neutral gap before the opposite drive direction is allowed. */
    rate = (desired == 0 ||
           (current_sign != 0 && Sign(desired) != current_sign) ||
           ((current >= 0 ? current : -current) >
            (desired >= 0 ? desired : -desired))) ? fall_rate : rise_rate;
    delta = (int32_t)desired - current;
    if (delta > (int32_t)rate) delta = rate;
    if (delta < -(int32_t)rate) delta = -(int32_t)rate;
    if (current_sign == 0 && desired_sign != 0 &&
        w->last_direction != 0 && desired_sign != w->last_direction &&
        w->reverse_gap_ms > 0U) {
        delta = 0;
    }
    w->applied = (int16_t)(current + delta);

    if (current_sign != 0 && w->applied == 0)
        w->reverse_gap_ms = MOTOR_REVERSE_GAP_MS;
    if (w->applied != 0)
        w->last_direction = Sign(w->applied);
}

void Motor_Update1ms(void)
{
    uint32_t bsrr = 0;
    if (brake_latched) return;
    UpdateWheel(&wheel_l);
    UpdateWheel(&wheel_r);

    /* Direction GPIO is changed only on the 1 kHz update boundary.
     * Reversal always traverses 0 PWM and a neutral gap >= 30 ms;
     * the CCR preload takes effect at the following 20 kHz PWM update. */
    AddDirection(&bsrr, MOTOR_L_IN1_PIN, MOTOR_L_IN2_PIN, wheel_l.applied);
    AddDirection(&bsrr, MOTOR_R_IN1_PIN, MOTOR_R_IN2_PIN, wheel_r.applied);
    GPIOB->BSRR = bsrr;
    TIM2->CCR2 = DutyFromCmd(wheel_l.applied, wheel_l.deadband);
    TIM2->CCR1 = DutyFromCmd(wheel_r.applied, wheel_r.deadband);
}
void Motor_GetAppliedPairPermille(int16_t *l, int16_t *r)
{
    uint32_t x = LockIRQ();
    if (l) *l = wheel_l.applied;
    if (r) *r = wheel_r.applied;
    UnlockIRQ(x);
}
void Motor_Stop(void)
{
    uint32_t bsrr = 0, x = LockIRQ();
    /* Hard stop takes precedence over a pending ramp. */
    if (wheel_l.applied != 0) {
        wheel_l.last_direction = Sign(wheel_l.applied);
        wheel_l.reverse_gap_ms = MOTOR_REVERSE_GAP_MS;
    }
    if (wheel_r.applied != 0) {
        wheel_r.last_direction = Sign(wheel_r.applied);
        wheel_r.reverse_gap_ms = MOTOR_REVERSE_GAP_MS;
    }
    wheel_l.target = wheel_r.target = 0;
    wheel_l.applied = wheel_r.applied = 0;
    brake_latched = 0;
    AddDirection(&bsrr, MOTOR_L_IN1_PIN, MOTOR_L_IN2_PIN, 0);
    AddDirection(&bsrr, MOTOR_R_IN1_PIN, MOTOR_R_IN2_PIN, 0);
    TIM2->CCR1 = TIM2->CCR2 = 0;
    GPIOB->BSRR = bsrr;
    UnlockIRQ(x);
}
void Motor_Brake(void)
{
    uint32_t bsrr = 0, x = LockIRQ();
    wheel_l.target = wheel_r.target = 0;
    wheel_l.applied = wheel_r.applied = 0;
    wheel_l.reverse_gap_ms = wheel_r.reverse_gap_ms = MOTOR_REVERSE_GAP_MS;
    brake_latched = 1;
    AddPinLevel(&bsrr, MOTOR_L_IN1_PIN, 1);
    AddPinLevel(&bsrr, MOTOR_L_IN2_PIN, 1);
    AddPinLevel(&bsrr, MOTOR_R_IN1_PIN, 1);
    AddPinLevel(&bsrr, MOTOR_R_IN2_PIN, 1);
    GPIOB->BSRR = bsrr;
    TIM2->CCR1 = pwm_period + 1U;
    TIM2->CCR2 = pwm_period + 1U;
    UnlockIRQ(x);
}
void Motor_StopWithMode(MotorStopMode_t mode)
{
    if (mode == MOTOR_STOP_BRAKE) Motor_Brake();
    else Motor_Stop();
}
