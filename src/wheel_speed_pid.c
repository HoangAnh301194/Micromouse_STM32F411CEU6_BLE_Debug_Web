#include "wheel_speed_pid.h"
#include "pinout.h"

#include <math.h>
#include <stddef.h>

#define WSP_PI_F             3.14159265f
#define WSP_DEG_TO_RAD_F     (WSP_PI_F / 180.0f)
#define WSP_RAD_TO_DEG_F     (180.0f / WSP_PI_F)
#define WSP_CIRCUMFERENCE_MM (WSP_PI_F * WHEEL_DIAMETER_MM)
#define WSP_PULSES_PER_MM    ((float)ENCODER_PPR / WSP_CIRCUMFERENCE_MM)
#define WSP_MM_PER_PULSE     (WSP_CIRCUMFERENCE_MM / (float)ENCODER_PPR)
#define WSP_ABS_MAX_PM       1000.0f
#define WSP_REF_ZERO_EPS     1e-3f
/* Battery compensation disabled: no measured battery voltage. */
// #define WSP_BAT_SCALE_MIN    0.75f
// #define WSP_BAT_SCALE_MAX    1.35f

static float ClampF(float value, float min_val, float max_val)
{
    if (value < min_val) return min_val;
    if (value > max_val) return max_val;
    return value;
}

static float SanitizeNonNeg(float value)
{
    if (!isfinite(value) || value < 0.0f) return 0.0f;
    return value;
}

static int16_t RoundPermille(float value)
{
    float clamped = ClampF(value, -WSP_ABS_MAX_PM, WSP_ABS_MAX_PM);
    if (clamped >= 0.0f) {
        return (int16_t)(clamped + 0.5f);
    }
    return (int16_t)(clamped - 0.5f);
}

static void ApplyLoopConfig(WheelSpeedLoop_t *loop,
                            const WheelSpeedConfig_t *cfg,
                            uint8_t reset_state)
{
    loop->kp = SanitizeNonNeg(cfg->kp);
    loop->ki = SanitizeNonNeg(cfg->ki);
    loop->ks_permille = SanitizeNonNeg(cfg->ks_permille);
    loop->kv_permille_per_pps = SanitizeNonNeg(cfg->kv_permille_per_pps);
    loop->ka_permille_per_pps2 = SanitizeNonNeg(cfg->ka_permille_per_pps2);
    loop->max_output_pm = ClampF(SanitizeNonNeg(cfg->max_output_pm), 0.0f, WSP_ABS_MAX_PM);
    loop->integral_limit = SanitizeNonNeg(cfg->integral_limit);

    if (reset_state) {
        loop->integral = 0.0f;
        loop->last_ref_pps = 0.0f;
        loop->last_meas_pps = 0.0f;
        loop->last_output_pm = 0.0f;
    } else {
        loop->integral = ClampF(loop->integral, -loop->integral_limit, loop->integral_limit);
    }
}

static void ReadLoopConfig(const WheelSpeedLoop_t *loop, WheelSpeedConfig_t *out)
{
    out->kp = loop->kp;
    out->ki = loop->ki;
    out->ks_permille = loop->ks_permille;
    out->kv_permille_per_pps = loop->kv_permille_per_pps;
    out->ka_permille_per_pps2 = loop->ka_permille_per_pps2;
    out->max_output_pm = loop->max_output_pm;
    out->integral_limit = loop->integral_limit;
}

static float ComputeSingleWheel(WheelSpeedLoop_t *loop,
                                float ref_pps,
                                float accel_pps2,
                                float meas_pps,
                                float dt_s,
                                float max_pm)
{
    float error;
    float ff = 0.0f;
    float p_term;
    float i_candidate;
    float u_unsat;
    float u_final;

    if (!isfinite(ref_pps)) ref_pps = 0.0f;
    if (!isfinite(accel_pps2)) accel_pps2 = 0.0f;
    if (!isfinite(meas_pps)) meas_pps = 0.0f;

    error = ref_pps - meas_pps;

    if (fabsf(ref_pps) > WSP_REF_ZERO_EPS) {
        float sign = (ref_pps > 0.0f) ? 1.0f : -1.0f;
        ff = sign * loop->ks_permille + loop->kv_permille_per_pps * ref_pps;
    }
    ff += loop->ka_permille_per_pps2 * accel_pps2;

    p_term = loop->kp * error;
    i_candidate = ClampF(loop->integral + error * dt_s,
                         -loop->integral_limit,
                         loop->integral_limit);

    // Battery compensation disabled: previously multiplied by bat_scale.
    u_unsat = ff + p_term + loop->ki * i_candidate;

    /* Conditional integration anti-windup: freeze integral if saturated and
     * error pushes further into saturation. */
    if ((u_unsat <= max_pm && u_unsat >= -max_pm) ||
        (u_unsat > max_pm && error < 0.0f) ||
        (u_unsat < -max_pm && error > 0.0f)) {
        loop->integral = i_candidate;
    }

    u_final = ff + p_term + loop->ki * loop->integral;
    loop->last_ref_pps = ref_pps;
    loop->last_meas_pps = meas_pps;
    loop->last_output_pm = ClampF(u_final, -max_pm, max_pm);
    return u_final;
}

static void DesaturateSteeringPriority(float raw_left,
                                       float raw_right,
                                       float max_pm,
                                       float *out_left,
                                       float *out_right)
{
    float abs_l = fabsf(raw_left);
    float abs_r = fabsf(raw_right);

    if (abs_l <= max_pm && abs_r <= max_pm) {
        *out_left = raw_left;
        *out_right = raw_right;
        return;
    }

    {
        float avg = 0.5f * (raw_left + raw_right);
        float half_diff = 0.5f * (raw_right - raw_left);
        float max_avg;

        half_diff = ClampF(half_diff, -max_pm, max_pm);
        max_avg = max_pm - fabsf(half_diff);
        avg = ClampF(avg, -max_avg, max_avg);

        *out_left = avg - half_diff;
        *out_right = avg + half_diff;
    }
}

void WheelSpeedPid_GetDefaultConfig(WheelSpeedConfig_t *out)
{
    if (!out) return;
    out->kp = 0.090f;
    out->ki = 0.250f;
    out->ks_permille = 80.0f;
    out->kv_permille_per_pps = 0.098f;
    out->ka_permille_per_pps2 = 0.0f;
    out->max_output_pm = WSP_ABS_MAX_PM;
    out->integral_limit = 3000.0f;
}

void WheelSpeedPid_Init(WheelSpeedController_t *ctrl,
                        float kp,
                        float ki,
                        float ks_permille,
                        float kv_permille_per_pps,
                        float max_output_pm,
                        float integral_limit)
{
    WheelSpeedConfig_t cfg;
    if (!ctrl) return;
    cfg.kp = kp;
    cfg.ki = ki;
    cfg.ks_permille = ks_permille;
    cfg.kv_permille_per_pps = kv_permille_per_pps;
    cfg.ka_permille_per_pps2 = 0.0f;
    cfg.max_output_pm = max_output_pm;
    cfg.integral_limit = integral_limit;
    WheelSpeedPid_InitConfig(ctrl, &cfg, NULL);
}

void WheelSpeedPid_InitConfig(WheelSpeedController_t *ctrl,
                              const WheelSpeedConfig_t *left_cfg,
                              const WheelSpeedConfig_t *right_cfg)
{
    const WheelSpeedConfig_t *r_cfg;
    float clamped_max;
    if (!ctrl || !left_cfg) return;
    r_cfg = right_cfg ? right_cfg : left_cfg;

    ApplyLoopConfig(&ctrl->left, left_cfg, 1U);
    ApplyLoopConfig(&ctrl->right, r_cfg, 1U);

    clamped_max = (ctrl->left.max_output_pm < ctrl->right.max_output_pm)
                      ? ctrl->left.max_output_pm
                      : ctrl->right.max_output_pm;
    ctrl->max_output_pm = clamped_max;
    // ctrl->battery_scale = 1.0f;  /* Battery compensation disabled. */
}

void WheelSpeedPid_SetGains(WheelSpeedController_t *ctrl,
                            const WheelSpeedConfig_t *left_cfg,
                            const WheelSpeedConfig_t *right_cfg)
{
    const WheelSpeedConfig_t *r_cfg;
    float clamped_max;
    if (!ctrl || !left_cfg) return;
    r_cfg = right_cfg ? right_cfg : left_cfg;

    ApplyLoopConfig(&ctrl->left, left_cfg, 0U);
    ApplyLoopConfig(&ctrl->right, r_cfg, 0U);

    clamped_max = (ctrl->left.max_output_pm < ctrl->right.max_output_pm)
                      ? ctrl->left.max_output_pm
                      : ctrl->right.max_output_pm;
    ctrl->max_output_pm = clamped_max;
}

void WheelSpeedPid_GetGains(const WheelSpeedController_t *ctrl,
                            WheelSpeedConfig_t *left_cfg,
                            WheelSpeedConfig_t *right_cfg)
{
    if (!ctrl) return;
    if (left_cfg) ReadLoopConfig(&ctrl->left, left_cfg);
    if (right_cfg) ReadLoopConfig(&ctrl->right, right_cfg);
}

void WheelSpeedPid_Reset(WheelSpeedController_t *ctrl)
{
    if (!ctrl) return;
    ctrl->left.integral = 0.0f;
    ctrl->left.last_ref_pps = 0.0f;
    ctrl->left.last_meas_pps = 0.0f;
    ctrl->left.last_output_pm = 0.0f;
    ctrl->right.integral = 0.0f;
    ctrl->right.last_ref_pps = 0.0f;
    ctrl->right.last_meas_pps = 0.0f;
    ctrl->right.last_output_pm = 0.0f;
}

void WheelSpeedPid_SetMaxOutput(WheelSpeedController_t *ctrl, float max_output_pm)
{
    float clamped_max;
    if (!ctrl) return;
    clamped_max = ClampF(SanitizeNonNeg(max_output_pm), 0.0f, WSP_ABS_MAX_PM);
    ctrl->max_output_pm = clamped_max;
    ctrl->left.max_output_pm = clamped_max;
    ctrl->right.max_output_pm = clamped_max;
}

#if 0 /* Battery compensation disabled: no measured battery voltage. */
void WheelSpeedPid_SetBatteryScale(WheelSpeedController_t *ctrl,
                                   float nominal_v,
                                   float measured_v)
{
    if (!ctrl) return;
    if (!isfinite(nominal_v) || !isfinite(measured_v) ||
        nominal_v <= 0.1f || measured_v <= 0.1f) {
        ctrl->battery_scale = 1.0f;
        return;
    }
    ctrl->battery_scale = ClampF(nominal_v / measured_v,
                                 WSP_BAT_SCALE_MIN,
                                 WSP_BAT_SCALE_MAX);
}
#endif

void WheelSpeedPid_CmdVelToWheelPps(float v_ref_mm_s,
                                    float w_ref_dps,
                                    float *left_ref_pps,
                                    float *right_ref_pps)
{
    float v_diff_mm_s;

    if (!isfinite(v_ref_mm_s)) v_ref_mm_s = 0.0f;
    if (!isfinite(w_ref_dps)) w_ref_dps = 0.0f;

    /* +yaw / +w_ref_dps = left turn (right wheel faster than left wheel). */
    v_diff_mm_s = 0.5f * WHEEL_BASE_MM * (w_ref_dps * WSP_DEG_TO_RAD_F);

    if (left_ref_pps) {
        *left_ref_pps = (v_ref_mm_s - v_diff_mm_s) * WSP_PULSES_PER_MM;
    }
    if (right_ref_pps) {
        *right_ref_pps = (v_ref_mm_s + v_diff_mm_s) * WSP_PULSES_PER_MM;
    }
}

void WheelSpeedPid_WheelPpsToCmdVel(float left_pps,
                                    float right_pps,
                                    float *v_mm_s,
                                    float *w_dps)
{
    float vl_mm_s;
    float vr_mm_s;

    if (!isfinite(left_pps)) left_pps = 0.0f;
    if (!isfinite(right_pps)) right_pps = 0.0f;

    vl_mm_s = left_pps * WSP_MM_PER_PULSE;
    vr_mm_s = right_pps * WSP_MM_PER_PULSE;

    if (v_mm_s) {
        *v_mm_s = 0.5f * (vl_mm_s + vr_mm_s);
    }
    if (w_dps) {
        *w_dps = ((vr_mm_s - vl_mm_s) / WHEEL_BASE_MM) * WSP_RAD_TO_DEG_F;
    }
}

void WheelSpeedPid_UpdateWheelPpsWithAccel(WheelSpeedController_t *ctrl,
                                           float left_ref_pps,
                                           float right_ref_pps,
                                           float left_accel_pps2,
                                           float right_accel_pps2,
                                           float left_meas_pps,
                                           float right_meas_pps,
                                           float dt_s,
                                           WheelSpeedOutput_t *out)
{
    float raw_l, raw_r, out_l, out_r;

    if (!ctrl || !out) return;
    if (dt_s <= 0.0f || dt_s > 0.05f) {
        out->left_ref_pps = left_ref_pps;
        out->right_ref_pps = right_ref_pps;
        out->left_meas_pps = left_meas_pps;
        out->right_meas_pps = right_meas_pps;
        out->left_pm = 0;
        out->right_pm = 0;
        return;
    }

    // Battery compensation disabled; PWM is computed without voltage scaling.
    raw_l = ComputeSingleWheel(&ctrl->left, left_ref_pps, left_accel_pps2,
                               left_meas_pps, dt_s, ctrl->max_output_pm);
    raw_r = ComputeSingleWheel(&ctrl->right, right_ref_pps, right_accel_pps2,
                               right_meas_pps, dt_s, ctrl->max_output_pm);

    DesaturateSteeringPriority(raw_l, raw_r, ctrl->max_output_pm,
                               &out_l, &out_r);

    ctrl->left.last_output_pm = out_l;
    ctrl->right.last_output_pm = out_r;

    out->left_ref_pps = left_ref_pps;
    out->right_ref_pps = right_ref_pps;
    out->left_meas_pps = left_meas_pps;
    out->right_meas_pps = right_meas_pps;
    out->left_pm = RoundPermille(out_l);
    out->right_pm = RoundPermille(out_r);
}

void WheelSpeedPid_UpdateWheelPps(WheelSpeedController_t *ctrl,
                                  float left_ref_pps,
                                  float right_ref_pps,
                                  float left_meas_pps,
                                  float right_meas_pps,
                                  float dt_s,
                                  WheelSpeedOutput_t *out)
{
    WheelSpeedPid_UpdateWheelPpsWithAccel(ctrl,
                                          left_ref_pps, right_ref_pps,
                                          0.0f, 0.0f,
                                          left_meas_pps, right_meas_pps,
                                          dt_s, out);
}

void WheelSpeedPid_UpdateCmdVelWithAccel(WheelSpeedController_t *ctrl,
                                         float v_ref_mm_s,
                                         float w_ref_dps,
                                         float a_ref_mm_s2,
                                         float alpha_ref_dps2,
                                         float left_meas_pps,
                                         float right_meas_pps,
                                         float dt_s,
                                         WheelSpeedOutput_t *out)
{
    float left_ref_pps = 0.0f;
    float right_ref_pps = 0.0f;
    float left_acc_pps2 = 0.0f;
    float right_acc_pps2 = 0.0f;

    WheelSpeedPid_CmdVelToWheelPps(v_ref_mm_s, w_ref_dps,
                                   &left_ref_pps, &right_ref_pps);
    WheelSpeedPid_CmdVelToWheelPps(a_ref_mm_s2, alpha_ref_dps2,
                                   &left_acc_pps2, &right_acc_pps2);
    WheelSpeedPid_UpdateWheelPpsWithAccel(ctrl,
                                          left_ref_pps, right_ref_pps,
                                          left_acc_pps2, right_acc_pps2,
                                          left_meas_pps, right_meas_pps,
                                          dt_s, out);
}

void WheelSpeedPid_UpdateCmdVel(WheelSpeedController_t *ctrl,
                                float v_ref_mm_s,
                                float w_ref_dps,
                                float left_meas_pps,
                                float right_meas_pps,
                                float dt_s,
                                WheelSpeedOutput_t *out)
{
    WheelSpeedPid_UpdateCmdVelWithAccel(ctrl,
                                        v_ref_mm_s, w_ref_dps,
                                        0.0f, 0.0f,
                                        left_meas_pps, right_meas_pps,
                                        dt_s, out);
}
