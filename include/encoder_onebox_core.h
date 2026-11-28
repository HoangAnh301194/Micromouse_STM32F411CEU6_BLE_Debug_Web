#ifndef ENCODER_ONEBOX_CORE_H
#define ENCODER_ONEBOX_CORE_H

#include <stdint.h>

/* Empirical one-cell (165 mm center-to-center) wheel increments.
 * Eight manual rolls on 2026-11-23:
 * left mean 2400.375, right mean 2389.625 quadrature counts.
 * Updated 2026-11-25: 10 passive ENC captures, left mean 2413.9,
 * right mean 2427.4 (end minus first sample). Use rounded left mean
 * for both wheels per request; assumes captures span one physical cell.
 * These are only initial travel targets; motor-driven braking needs tuning.
 */
#define ONEBOX_TARGET_L 2414L
#define ONEBOX_TARGET_R ONEBOX_TARGET_L
/* First motor-on one-cell trial: deliberately limited to 20% duty.
 * Duty is NOT wheel speed; a stall at low duty is detected and aborted.
 * Do not raise PWM without a supervised lifted-wheel test. */
#define ONEBOX_MAX_PWM 200
#define ONEBOX_SPEED_PPS 1100
/* Conservative initial tuning, not yet bench validated. PPS, counts, degrees. */
#define MOTION_CRUISE_PPS 850.0f
#define MOTION_MIN_PPS 220.0f
#define MOTION_ACCEL_PPS_S 2200.0f
#define MOTION_DECEL_COUNTS 450.0f
#define MOTION_SYNC_KP 3.0f
#define MOTION_SYNC_MAX_PPS 220.0f
#define MOTION_ANGLE_KP 40.0f
#define MOTION_ANGLE_KD 2.0f
#define MOTION_ANGLE_TOL 1.0f
#define MOTION_GYRO_TOL 3.0f
#define MOTION_COUNT_TOL 20L
#define MOTION_STOP_COUNTS 5L
enum { MOTION_ACC=1, MOTION_CRUISE=2, MOTION_DEC=3, MOTION_SETTLE=4, MOTION_END=5 };
/* RAM-only tuning: integer wire units, gains in hundredths. */
typedef struct {
    uint32_t cruise, minimum, acceleration, decel_counts, sync_p_x100,
             sync_max, angle_p_x100, angle_d_x100;
} MotionConfig;
extern MotionConfig motion_config;
uint8_t MotionConfig_Valid(const MotionConfig *c);
float Motion_Base(float previous, float desired, float limit);
float Motion_DistanceDesired(float remaining);
float Motion_AngleDesired(float remaining, float gyro);
void Motion_Sync(float base, int32_t error, float *left, float *right);
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
    ONEBOX_REASON_HOST_LOST = 8,
    ONEBOX_REASON_TOLERANCE = 9
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
    uint8_t reached_l, reached_r;
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
