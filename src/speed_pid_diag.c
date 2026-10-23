/* Diagnostic ONLY: a real per-wheel closed-loop PPS regulator on raised wheels.
 * No production motion code or PID gains are altered.
 * Runs in main every 10 ms; TIM11 owns only the 1-kHz encoder estimator.
 */
#include "speed_pid_diag.h"
#if defined(ENCODER_SPEED_PID_DIAG)
#if !defined(INTEGRATION_DIAG) || !defined(INTEGRATION_DIAG_MOTOR_ON) || \
    !defined(INTEGRATION_DIAG_LIMITED_PWM_30) || defined(ENCODER_ALPHA_AB_DIAG)
#error "Speed PID diagnostic MUST be isolated from production and shadow AB"
#endif
#include "encoder.h"
#include "motor.h"
#include "pid.h"

#define PID_TICK_MS 10U
#define MAX_CMD 300
/* Fixed across A/B builds. These are CONSERVATIVE exploratory gains,
 * not a previously tuned production speed controller. Units:
 * command permille per encoder PPS and per integrated PPS*seconds. */
#define PID_KP 0.050f
#define PID_KI 0.080f
#define PID_KD 0.000f
#define PID_CORRECTION_LIMIT 100.0f
#define PID_INTEGRAL_LIMIT 1250.0f

static PID_Handle_t left_pid, right_pid;
static SpeedPidDiagSnapshot state;
static uint8_t running;
static uint8_t last_group, last_step;

static uint16_t BoundedCommand(float correction, uint16_t ff)
{
    float cmd = (float)ff + correction;
    if (cmd <= 0.0f) return 0U;
    if (cmd >= (float)MAX_CMD) return MAX_CMD;
    return (uint16_t)(cmd + 0.5f);
}

void SpeedPidDiag_Init(void)
{
    PID_Init(&left_pid, PID_KP, PID_KI, PID_KD,
             PID_CORRECTION_LIMIT, PID_INTEGRAL_LIMIT);
    PID_Init(&right_pid, PID_KP, PID_KI, PID_KD,
             PID_CORRECTION_LIMIT, PID_INTEGRAL_LIMIT);
    state = (SpeedPidDiagSnapshot){0};
    last_group = last_step = 255U;
    running = 0U;
}

void SpeedPidDiag_Stop(void)
{
    running = 0U;
    state.target_pps = 0U;
    state.last_pwm_left = state.last_pwm_right = 0U;
    Motor_Stop();
}

void SpeedPidDiag_Run(uint32_t now_ms, uint8_t group, uint8_t step)
{
    uint16_t target, feedforward, out_l = 0U, out_r = 0U;
    if (group > 2U || step > 3U) { SpeedPidDiag_Stop(); return; }
    if (group != last_group || step != last_step) {
        PID_Reset(&left_pid);
        PID_Reset(&right_pid);
        last_group = group; last_step = step;
        running = 1U;
        /* First command starts immediately, others every 10ms. */
        state.last_update_ms = now_ms - PID_TICK_MS;
    }
    if ((uint32_t)(now_ms-state.last_update_ms) < PID_TICK_MS) return;
    state.last_update_ms = now_ms;
    target = (uint16_t)step * 800U; /* 0, 800, 1600, 2400 PPS; PWM headroom */
    feedforward = (uint16_t)step * 100U; /* 0..30% baseline */
    state.target_pps = target;
    state.feedforward_permille = feedforward;
    state.max_pwm_permille = MAX_CMD;
    state.group = group;
    state.step = step;
    if (!running || step == 0U) {
        Motor_Stop();
        state.last_pwm_left = state.last_pwm_right = 0U;
        return;
    }
    if (group != 1U) {
        out_l = BoundedCommand(PID_Compute(&left_pid, (float)target,
                        Encoder_GetLeftSpeedPPS(), 0.010f), feedforward);
    }
    if (group != 0U) {
        out_r = BoundedCommand(PID_Compute(&right_pid, (float)target,
                        Encoder_GetRightSpeedPPS(), 0.010f), feedforward);
    }
    Motor_SetPairPermille((int16_t)out_l, (int16_t)out_r);
    state.last_pwm_left = out_l;
    state.last_pwm_right = out_r;
}

void SpeedPidDiag_Get(SpeedPidDiagSnapshot *out)
{
    if (out) *out = state; /* both sides only accessed in main */
}
#endif
