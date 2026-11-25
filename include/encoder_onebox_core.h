#ifndef ENCODER_ONEBOX_CORE_H
#define ENCODER_ONEBOX_CORE_H

#include <stdint.h>

/* Empirical one-cell (165 mm center-to-center) wheel increments.
 * Eight manual rolls on 2026-11-23:
 * left mean 2400.375, right mean 2389.625 quadrature counts.
 * These are only initial travel targets; motor-driven braking needs tuning.
 */
#define ONEBOX_TARGET_L 2400L
#define ONEBOX_TARGET_R 2390L
/* First motor-on one-cell trial: deliberately limited to 20% duty.
 * Duty is NOT wheel speed; a stall at low duty is detected and aborted.
 * Do not raise PWM without a supervised lifted-wheel test. */
#define ONEBOX_MAX_PWM 200
#define ONEBOX_SPEED_PPS 1100
#define ONEBOX_KS_PM 80.0f
#define ONEBOX_KV_PM_PER_PPS 0.098f
#define ONEBOX_INTEGRAL_LIMIT 3000.0f
#define ONEBOX_PID_DT_S 0.001f
#define ONEBOX_DEFAULT_P 900U
#define ONEBOX_DEFAULT_I 2500U
#define ONEBOX_DEFAULT_D 0U
#define ONEBOX_GAIN_SCALE 10000U
#define ONEBOX_PID_P_MAX 10000U
#define ONEBOX_PID_I_MAX 20000U
#define ONEBOX_PID_D_MAX 1000U

/* Separate state and gains for each wheel; coefficients use PPS and permille. */
typedef struct { uint32_t p, i, d; } OneBoxGains;
typedef struct { float integral, previous_speed; uint8_t initialized; } OneBoxSpeedPid;
uint8_t OneBoxGains_Valid(const OneBoxGains *g);
void OneBoxSpeedPid_Reset(OneBoxSpeedPid *s);
int16_t OneBoxSpeedPid_Step(OneBoxSpeedPid *s, const OneBoxGains *g,
                            float reference, float measured);
#define ONEBOX_MAX_TICKS 6500U      /* hard 6.5 s one-shot limit */
#define ONEBOX_STALL_TICKS 900U      /* no encoder movement */
#define ONEBOX_LEASE_TICKS 350U      /* require host heartbeat <350ms */
#define ONEBOX_DIVERGENCE_COUNT 250L

enum {
    ONEBOX_REASON_NONE = 0,
    ONEBOX_REASON_DONE = 1,
    ONEBOX_REASON_USER_STOP = 2,
    ONEBOX_REASON_TIMEOUT = 3,
    ONEBOX_REASON_STALL = 4,
    ONEBOX_REASON_ENCODER = 5,
    ONEBOX_REASON_IMU = 6,
    ONEBOX_REASON_MOTOR = 7,
    ONEBOX_REASON_HOST_LOST = 8
};

typedef struct {
    int32_t start_l, start_r;
    int32_t last_l, last_r;
    int32_t delta_l, delta_r;
    uint32_t ticks;
    uint16_t stale_ticks;
    uint16_t host_age_ticks;
    uint8_t running;
    uint8_t reason;
} OneBoxCore;

typedef struct {
    int16_t pwm_l, pwm_r;
    uint8_t reason;
} OneBoxOutput;

void OneBoxCore_Start(OneBoxCore *s, int32_t start_l, int32_t start_r);
void OneBoxCore_HostPing(OneBoxCore *s);
OneBoxOutput OneBoxCore_Step(OneBoxCore *s, int32_t left, int32_t right);
void OneBoxCore_Stop(OneBoxCore *s, uint8_t reason);

#endif
