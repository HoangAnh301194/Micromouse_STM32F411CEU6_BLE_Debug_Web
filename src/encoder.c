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

#if defined(ENCODER_ALPHA_AB_DIAG)
/* Shadow-only 1kHz estimator bank. No data path to production control. */
#if !defined(ENCODER_PROFILE_DIAG) || !defined(INTEGRATION_DIAG)
#error "Encoder alpha A/B requires diagnostic Encoder integration profile"
#endif
/* Producer TIM11 only: fixed bounded O(1) write of two signed deltas.
 * Consumer main loop performs all EMA, boxcar and error calculations.
 * Producer publishes head only after both delta fields are visible. */
typedef struct {
    int16_t l;
    int16_t r;
} ShadowSample;
static ShadowSample shadow_samples[ENCODER_SHADOW_QUEUE_CAPACITY];
static volatile uint16_t shadow_qhead, shadow_qtail, shadow_peak_pending;
static volatile uint32_t shadow_produced, shadow_consumed;
static volatile uint32_t shadow_overflow, shadow_stage_lag;

static void ShadowUpdate(int16_t dl, int16_t dr, float raw_l, float raw_r);

static void ShadowEnqueue(int16_t dl, int16_t dr)
{
    uint16_t head = shadow_qhead;
    uint16_t next = (uint16_t)((head + 1U) &
                               (ENCODER_SHADOW_QUEUE_CAPACITY - 1U));
    if (next == shadow_qtail) {
        ++shadow_overflow; /* Never overwrite unread real 1ms evidence. */
        return;
    }
    shadow_samples[head].l = dl;
    shadow_samples[head].r = dr;
    __DMB();
    shadow_qhead = next;
    ++shadow_produced;
    {
        uint16_t depth = (uint16_t)((next - shadow_qtail) &
                                    (ENCODER_SHADOW_QUEUE_CAPACITY - 1U));
        if (depth > shadow_peak_pending) shadow_peak_pending = depth;
    }
}

static uint16_t ShadowPending(void)
{
    return (uint16_t)((shadow_qhead - shadow_qtail) &
                      (ENCODER_SHADOW_QUEUE_CAPACITY - 1U));
}

uint16_t Encoder_ShadowService(uint16_t budget)
{
    uint16_t processed = 0U;
    while (processed < budget) {
        uint16_t tail = shadow_qtail;
        ShadowSample sample;
        if (tail == shadow_qhead) break;
        sample = shadow_samples[tail];
        __DMB();
        shadow_qtail = (uint16_t)((tail + 1U) &
                                 (ENCODER_SHADOW_QUEUE_CAPACITY - 1U));
        ++shadow_consumed;
        /* Same signed count changes as the 1-kHz production estimator. */
        ShadowUpdate(sample.l, sample.r,
                     (float)sample.l * 1000.0f,
                     (float)sample.r * 1000.0f);
        ++processed;
    }
    return processed;
}

void Encoder_GetShadowQueueStats(Encoder_ShadowQueueStats_t *out)
{
    uint32_t primask;
    if (!out) return;
    primask = __get_PRIMASK();
    __disable_irq();
    out->produced = shadow_produced;
    out->consumed = shadow_consumed;
    out->overflow = shadow_overflow;
    out->stage_boundary_lag = shadow_stage_lag;
    out->pending = ShadowPending();
    out->peak_pending = shadow_peak_pending;
    if (!primask) __enable_irq();
}

/* Invoked only after an ordinary bounded service pass at a stage edge.
 * If a backlog remains, flag that stage's measurements as unqualified.
 * Never extend the motor heartbeat timeout to hide main-loop congestion. */
void Encoder_ShadowMarkStageBoundary(void)
{
    /* One or two ticks may legitimately arrive during stage snapshots.
     * Larger backlogs imply a meaningful stage-boundary misclassification. */
    if (ShadowPending() > 2U) ++shadow_stage_lag;
}

static const float shadow_alpha[ENCODER_SHADOW_ALPHAS] = {
    0.05f, 0.08f, 0.10f, 0.12f, 0.15f,
    0.20f, 0.30f, 0.40f, 0.60f, 0.80f
};
static float shadow_l[ENCODER_SHADOW_ALPHAS];
static float shadow_r[ENCODER_SHADOW_ALPHAS];
static int16_t shadow_ring_l[20], shadow_ring_r[20];
static uint8_t shadow_pos, shadow_filled;
static int32_t shadow_sum_l[ENCODER_SHADOW_WINDOWS];
static int32_t shadow_sum_r[ENCODER_SHADOW_WINDOWS];
static uint32_t shadow_stage_ticks, shadow_steady_ticks, shadow_early_ticks;
static uint32_t shadow_steady_error_l[ENCODER_SHADOW_ALPHAS];
static uint32_t shadow_steady_error_r[ENCODER_SHADOW_ALPHAS];
static uint32_t shadow_early_error_l[ENCODER_SHADOW_ALPHAS];
static uint32_t shadow_early_error_r[ENCODER_SHADOW_ALPHAS];
static uint32_t shadow_variation_l[ENCODER_SHADOW_ALPHAS];
static uint32_t shadow_variation_r[ENCODER_SHADOW_ALPHAS];

static uint32_t ShadowAbs(float value)
{
    if (value < 0.0f) value = -value;
    return (uint32_t)value;
}
static void ShadowAccum(uint32_t *acc, uint32_t value)
{
    if (*acc > (0xFFFFFFFFUL - value)) *acc = 0xFFFFFFFFUL;
    else *acc += value;
}
static void ShadowResetCounters(void)
{
    uint8_t i;
    shadow_stage_ticks = shadow_steady_ticks = shadow_early_ticks = 0U;
    for (i = 0U; i < ENCODER_SHADOW_ALPHAS; ++i) {
        shadow_steady_error_l[i] = shadow_steady_error_r[i] = 0U;
        shadow_early_error_l[i] = shadow_early_error_r[i] = 0U;
        shadow_variation_l[i] = shadow_variation_r[i] = 0U;
    }
}
static void ShadowUpdate(int16_t dl, int16_t dr, float raw_l, float raw_r)
{
    uint8_t i, p5, p10;
    float reference_l, reference_r;
    p5 = (uint8_t)((shadow_pos + 15U) % 20U);
    p10 = (uint8_t)((shadow_pos + 10U) % 20U);
    shadow_sum_l[0] += (int32_t)dl - shadow_ring_l[p5];
    shadow_sum_l[1] += (int32_t)dl - shadow_ring_l[p10];
    shadow_sum_l[2] += (int32_t)dl - shadow_ring_l[shadow_pos];
    shadow_sum_r[0] += (int32_t)dr - shadow_ring_r[p5];
    shadow_sum_r[1] += (int32_t)dr - shadow_ring_r[p10];
    shadow_sum_r[2] += (int32_t)dr - shadow_ring_r[shadow_pos];
    shadow_ring_l[shadow_pos] = dl;
    shadow_ring_r[shadow_pos] = dr;
    shadow_pos = (uint8_t)((shadow_pos + 1U) % 20U);
    if (shadow_filled < 20U) ++shadow_filled;
    reference_l = (float)(shadow_sum_l[2] * 50);
    reference_r = (float)(shadow_sum_r[2] * 50);
    ++shadow_stage_ticks;
    for (i = 0U; i < ENCODER_SHADOW_ALPHAS; ++i) {
        float prior_l = shadow_l[i], prior_r = shadow_r[i];
        shadow_l[i] += shadow_alpha[i] * (raw_l - shadow_l[i]);
        shadow_r[i] += shadow_alpha[i] * (raw_r - shadow_r[i]);
        if (shadow_filled < 20U) continue;
        if (shadow_stage_ticks <= 500U) {
            ShadowAccum(&shadow_early_error_l[i], ShadowAbs(shadow_l[i] - reference_l));
            ShadowAccum(&shadow_early_error_r[i], ShadowAbs(shadow_r[i] - reference_r));
        } else {
            ShadowAccum(&shadow_steady_error_l[i], ShadowAbs(shadow_l[i] - reference_l));
            ShadowAccum(&shadow_steady_error_r[i], ShadowAbs(shadow_r[i] - reference_r));
            ShadowAccum(&shadow_variation_l[i], ShadowAbs(shadow_l[i] - prior_l));
            ShadowAccum(&shadow_variation_r[i], ShadowAbs(shadow_r[i] - prior_r));
        }
    }
    if (shadow_filled == 20U) {
        if (shadow_stage_ticks <= 500U) ++shadow_early_ticks;
        else ++shadow_steady_ticks;
    }
}
void Encoder_ShadowResetMetrics(void)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    ShadowResetCounters();
    if (!primask) __enable_irq();
}
void Encoder_GetShadowSnapshot(Encoder_ShadowSnapshot_t *out)
{
    uint32_t primask;
    uint8_t i;
    if (!out) return;
    primask = __get_PRIMASK();
    __disable_irq();
    out->ticks = shadow_stage_ticks;
    out->valid_reference_ticks = shadow_early_ticks;
    out->window_ticks = shadow_steady_ticks;
    out->quiet_ticks = 0U;
    for (i = 0U; i < ENCODER_SHADOW_ALPHAS; ++i) {
        out->alpha_left[i] = (int32_t)shadow_l[i];
        out->alpha_right[i] = (int32_t)shadow_r[i];
        out->window_abs_error_left[i] = shadow_steady_error_l[i];
        out->window_abs_error_right[i] = shadow_steady_error_r[i];
        out->transient_abs_error_left[i] = shadow_early_error_l[i];
        out->transient_abs_error_right[i] = shadow_early_error_r[i];
        out->variation_left[i] = shadow_variation_l[i];
        out->variation_right[i] = shadow_variation_r[i];
    }
    for (i = 0U; i < ENCODER_SHADOW_WINDOWS; ++i) {
        out->box_left[i] = shadow_sum_l[i] * (i == 0U ? 200 : (i == 1U ? 100 : 50));
        out->box_right[i] = shadow_sum_r[i] * (i == 0U ? 200 : (i == 1U ? 100 : 50));
    }
    if (!primask) __enable_irq();
}
#endif

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
#if defined(ENCODER_ALPHA_AB_DIAG)
    ShadowEnqueue((int16_t)(delta_left * ENCODER_LEFT_DIRECTION),
                  (int16_t)(delta_right * ENCODER_RIGHT_DIRECTION));
#endif
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
#if defined(ENCODER_ALPHA_AB_DIAG)
    {
        uint8_t i;
        shadow_pos = shadow_filled = 0U;
        shadow_qhead = shadow_qtail = shadow_peak_pending = 0U;
        shadow_produced = shadow_consumed = 0U;
        shadow_overflow = shadow_stage_lag = 0U;
        for (i = 0U; i < 20U; ++i) {
            shadow_ring_l[i] = shadow_ring_r[i] = 0;
        }
        for (i = 0U; i < ENCODER_SHADOW_ALPHAS; ++i) {
            shadow_l[i] = shadow_r[i] = 0.0f;
        }
        for (i = 0U; i < ENCODER_SHADOW_WINDOWS; ++i) {
            shadow_sum_l[i] = shadow_sum_r[i] = 0;
        }
        ShadowResetCounters();
    }
#endif

    if (!primask) __enable_irq();
}

int32_t Encoder_GetLeftCount(void) { return total_left; }
int32_t Encoder_GetRightCount(void) { return total_right; }
float Encoder_GetLeftSpeedPPS(void) { return speed_left_pps; }
float Encoder_GetRightSpeedPPS(void) { return speed_right_pps; }
