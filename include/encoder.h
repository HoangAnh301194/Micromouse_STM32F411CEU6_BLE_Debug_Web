#ifndef ENCODER_H
#define ENCODER_H

#include <stdint.h>

#define ENCODER_LEFT_DIRECTION  -1
#define ENCODER_RIGHT_DIRECTION -1

void Encoder_Init(void);
void Encoder_Update(float dt_s);
void Encoder_Reset(void);

int32_t Encoder_GetLeftCount(void);
int32_t Encoder_GetRightCount(void);
float Encoder_GetLeftSpeedPPS(void);
float Encoder_GetRightSpeedPPS(void);

/* Opt-in 1-kHz shadow estimator bank. Never feeds production motion/PID. */
#if defined(ENCODER_ALPHA_AB_DIAG)
#define ENCODER_SHADOW_ALPHAS 10U
#define ENCODER_SHADOW_WINDOWS 3U
#define ENCODER_SHADOW_QUEUE_CAPACITY 256U
/* SPSC queue: TIM11 only produces; main loop only consumes. */
typedef struct {
    uint32_t produced;
    uint32_t consumed;
    uint32_t overflow;
    uint32_t stage_boundary_lag;
    uint16_t pending;
    uint16_t peak_pending;
} Encoder_ShadowQueueStats_t;
/* Bounded processing; never run the ten shadow filters in TIM11. */
uint16_t Encoder_ShadowService(uint16_t budget);
void Encoder_GetShadowQueueStats(Encoder_ShadowQueueStats_t *out);
void Encoder_ShadowMarkStageBoundary(void);

typedef struct {
    int32_t alpha_left[ENCODER_SHADOW_ALPHAS];
    int32_t alpha_right[ENCODER_SHADOW_ALPHAS];
    int32_t box_left[ENCODER_SHADOW_WINDOWS];
    int32_t box_right[ENCODER_SHADOW_WINDOWS];
    /* 1-kHz cumulative metrics; snapshots reset on each diagnostic stage. */
    uint32_t ticks;
    uint32_t valid_reference_ticks;
    uint32_t quiet_ticks;
    uint32_t window_ticks;
    uint32_t window_abs_error_left[ENCODER_SHADOW_ALPHAS];
    uint32_t window_abs_error_right[ENCODER_SHADOW_ALPHAS];
    uint32_t transient_abs_error_left[ENCODER_SHADOW_ALPHAS];
    uint32_t transient_abs_error_right[ENCODER_SHADOW_ALPHAS];
    uint32_t variation_left[ENCODER_SHADOW_ALPHAS];
    uint32_t variation_right[ENCODER_SHADOW_ALPHAS];
} Encoder_ShadowSnapshot_t;
void Encoder_ShadowResetMetrics(void);
void Encoder_GetShadowSnapshot(Encoder_ShadowSnapshot_t *out);
#endif

#endif
