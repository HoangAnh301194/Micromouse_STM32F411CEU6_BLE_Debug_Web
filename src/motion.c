#include "motion.h"
#include "encoder.h"
#include "motor.h"
#include "pid.h"
#include "pinout.h"
#include "stm32f4xx.h"
#include "wheel_speed_pid.h"

#include <math.h>
#include <stddef.h>

#define PI_F                        3.14159265f
#define MM_PER_PULSE                (PI_F * WHEEL_DIAMETER_MM / (float)ENCODER_PPR)
#define STRAIGHT_SPEED_DEFAULT_MM_S 250.0f
#define STRAIGHT_SPEED_MIN_MM_S     60.0f
#define STRAIGHT_SPEED_ABS_MAX_MM_S 1200.0f
#define STRAIGHT_ACCEL_MM_S2        600.0f
#define STRAIGHT_DECEL_MM_S2        600.0f
#define STRAIGHT_PWM_MAX_PM         1000.0f
#define STRAIGHT_W_MAX_DPS          120.0f
#define VELOCITY_W_ABS_MAX_DPS      900.0f
#define STEERING_BIAS_ABS_MAX_DPS   360.0f
#define TURN_PWM_MAX                50.0f
#define TURN_PWM_MIN                20.0f
#define TURN_TOLERANCE_DEG          3.0f
#define TURN_RATE_TOL_DPS           10.0f
#define TURN_TIMEOUT_S              1.5f

#define STRAIGHT_TIMEOUT_MAX_S      20.0f
#define WTEST_ABS_MAX_PPS           12000.0f
#define WTEST_MAX_DURATION_S        15.0f
#define QSTEP_PHASE_DURATION_S      0.8f
#define QSTEP_NUM_PHASES            6U

typedef struct {
    volatile Motion_State_t state;
    volatile uint8_t done;
    volatile Motion_Result_t last_result;
    volatile uint8_t phase;

    float target_distance_mm;
    float target_angle_deg;
    float start_yaw_deg;

    volatile float distance_mm;
    volatile float angle_error_deg;

    float current_pwm;
    float current_speed_mm_s;
    float current_w_dps;
    float elapsed_s;
    float straight_timeout_s;

    Motion_StraightProfile_t straight_profile;
    Motion_TurnProfile_t turn_profile;

    float cmd_v_mm_s;
    float cmd_w_dps;
    float cmd_duration_s;
    uint8_t cmd_hold_heading;
    float steering_bias_dps;

    /* Independent wheel speed test state */
    float wtest_left_pps;
    float wtest_right_pps;
    float wtest_duration_s;
    float wtest_max_pwm_pm;
    Motion_WheelTestMode_t wtest_mode;
    uint8_t max_pm_overridden;
    float saved_left_max_pm;
    float saved_right_max_pm;

    /* Quick step test level */
    Motion_QuickStepLevel_t qstep_level;

    /* Host connection watchdog for PC-controlled tests */
    uint8_t host_watchdog_enabled;
    float host_watchdog_timeout_s;
    float host_watchdog_elapsed_s;

    uint8_t sat_flags;
    float v_meas_mm_s;
    float w_meas_dps;

    PID_Handle_t heading_pid;
    PID_Handle_t turn_pid;
    WheelSpeedController_t speed_ctrl;
    Motion_SpeedDebug_t speed_debug;
} Motion_Context_t;

static Motion_Context_t motion;

static float NormalizeAngle(float angle)
{
    if (!isfinite(angle)) return 0.0f;
    angle = fmodf(angle + 180.0f, 360.0f);
    if (angle < 0.0f) angle += 360.0f;
    return angle - 180.0f;
}

static float ClampF(float value, float min_value, float max_value)
{
    if (value < min_value) return min_value;
    if (value > max_value) return max_value;
    return value;
}

static void ClearSpeedDebug(void)
{
    motion.speed_debug.left_ref_pps = 0;
    motion.speed_debug.right_ref_pps = 0;
    motion.speed_debug.left_meas_pps = 0;
    motion.speed_debug.right_meas_pps = 0;
    motion.speed_debug.left_pm = 0;
    motion.speed_debug.right_pm = 0;
    motion.sat_flags = 0U;
    motion.v_meas_mm_s = 0.0f;
    motion.w_meas_dps = 0.0f;
}

static void SanitizeStraightProfile(Motion_StraightProfile_t *p)
{
    Motion_StraightProfile_t def;
    Motion_GetDefaultStraightProfile(&def);

    if (!isfinite(p->max_speed_mm_s) || p->max_speed_mm_s <= 0.0f)
        p->max_speed_mm_s = def.max_speed_mm_s;
    p->max_speed_mm_s = ClampF(p->max_speed_mm_s, 10.0f, STRAIGHT_SPEED_ABS_MAX_MM_S);

    if (!isfinite(p->min_speed_mm_s) || p->min_speed_mm_s <= 0.0f)
        p->min_speed_mm_s = def.min_speed_mm_s;
    p->min_speed_mm_s = ClampF(p->min_speed_mm_s, 10.0f, p->max_speed_mm_s);

    if (!isfinite(p->end_speed_mm_s) || p->end_speed_mm_s < 0.0f)
        p->end_speed_mm_s = 0.0f;
    p->end_speed_mm_s = ClampF(p->end_speed_mm_s, 0.0f, p->max_speed_mm_s);

    if (!isfinite(p->accel_mm_s2) || p->accel_mm_s2 <= 0.0f)
        p->accel_mm_s2 = def.accel_mm_s2;
    if (!isfinite(p->decel_mm_s2) || p->decel_mm_s2 <= 0.0f)
        p->decel_mm_s2 = def.decel_mm_s2;

    if (!isfinite(p->max_w_dps) || p->max_w_dps <= 0.0f)
        p->max_w_dps = def.max_w_dps;
    p->max_w_dps = ClampF(p->max_w_dps, 10.0f, VELOCITY_W_ABS_MAX_DPS);
}

static void SanitizeTurnProfile(Motion_TurnProfile_t *p)
{
    Motion_TurnProfile_t def;
    Motion_GetDefaultTurnProfile(&def);

    if (!isfinite(p->max_w_dps) || p->max_w_dps <= 0.0f) p->max_w_dps = def.max_w_dps;
    p->max_w_dps = ClampF(p->max_w_dps, 30.0f, VELOCITY_W_ABS_MAX_DPS);

    if (!isfinite(p->min_w_dps) || p->min_w_dps <= 0.0f) p->min_w_dps = def.min_w_dps;
    p->min_w_dps = ClampF(p->min_w_dps, 10.0f, p->max_w_dps);

    if (!isfinite(p->accel_dps2) || p->accel_dps2 <= 0.0f) p->accel_dps2 = def.accel_dps2;
    if (!isfinite(p->decel_dps2) || p->decel_dps2 <= 0.0f) p->decel_dps2 = def.decel_dps2;

    if (!isfinite(p->angle_kp) || p->angle_kp <= 0.0f) p->angle_kp = def.angle_kp;
    if (!isfinite(p->gyro_rate_kp) || p->gyro_rate_kp < 0.0f) p->gyro_rate_kp = def.gyro_rate_kp;
    p->gyro_rate_kp = ClampF(p->gyro_rate_kp, 0.0f, 1.5f);

    if (!isfinite(p->tolerance_deg) || p->tolerance_deg <= 0.0f) p->tolerance_deg = def.tolerance_deg;
    if (!isfinite(p->rate_tol_dps) || p->rate_tol_dps <= 0.0f) p->rate_tol_dps = def.rate_tol_dps;
    if (!isfinite(p->timeout_s) || p->timeout_s <= 0.0f) p->timeout_s = def.timeout_s;
}

static void PublishWheelSpeedOutput(const WheelSpeedOutput_t *ws_out)
{
    int16_t max_l = (int16_t)motion.speed_ctrl.left.max_output_pm;
    int16_t max_r = (int16_t)motion.speed_ctrl.right.max_output_pm;
    uint8_t sat = 0U;

    motion.speed_debug.left_ref_pps = (int32_t)ws_out->left_ref_pps;
    motion.speed_debug.right_ref_pps = (int32_t)ws_out->right_ref_pps;
    motion.speed_debug.left_meas_pps = (int32_t)ws_out->left_meas_pps;
    motion.speed_debug.right_meas_pps = (int32_t)ws_out->right_meas_pps;
    motion.speed_debug.left_pm = ws_out->left_pm;
    motion.speed_debug.right_pm = ws_out->right_pm;

    if (ws_out->left_pm >= max_l || ws_out->left_pm <= -max_l) sat |= 0x01U;
    if (ws_out->right_pm >= max_r || ws_out->right_pm <= -max_r) sat |= 0x02U;
    motion.sat_flags = sat;

    WheelSpeedPid_WheelPpsToCmdVel(ws_out->left_meas_pps, ws_out->right_meas_pps,
                                   &motion.v_meas_mm_s, &motion.w_meas_dps);
    motion.current_pwm = 0.05f * (float)(ws_out->left_pm + ws_out->right_pm);
    Motor_SetPairPermille(ws_out->left_pm, ws_out->right_pm);
}

static void ApplyCmdVel(float v_mm_s,
                        float w_dps,
                        float a_mm_s2,
                        float alpha_dps2,
                        float dt_s)
{
    WheelSpeedOutput_t ws_out;

    WheelSpeedPid_UpdateCmdVelWithAccel(&motion.speed_ctrl,
                                        v_mm_s, w_dps,
                                        a_mm_s2, alpha_dps2,
                                        Encoder_GetLeftSpeedPPS(),
                                        Encoder_GetRightSpeedPPS(),
                                        dt_s, &ws_out);
    PublishWheelSpeedOutput(&ws_out);
}

static void ApplyWheelSpeed(float left_pps, float right_pps, float dt_s)
{
    WheelSpeedOutput_t ws_out;

    WheelSpeedPid_UpdateWheelPps(&motion.speed_ctrl,
                                 left_pps, right_pps,
                                 Encoder_GetLeftSpeedPPS(),
                                 Encoder_GetRightSpeedPPS(),
                                 dt_s, &ws_out);
    WheelSpeedPid_WheelPpsToCmdVel(left_pps, right_pps,
                                   &motion.current_speed_mm_s,
                                   &motion.current_w_dps);
    PublishWheelSpeedOutput(&ws_out);
}

static void FinishMotionWithResult(Motion_Result_t res)
{
    if (motion.max_pm_overridden) {
        motion.speed_ctrl.left.max_output_pm = motion.saved_left_max_pm;
        motion.speed_ctrl.right.max_output_pm = motion.saved_right_max_pm;
        motion.speed_ctrl.max_output_pm =
            (motion.saved_left_max_pm < motion.saved_right_max_pm)
                ? motion.saved_left_max_pm
                : motion.saved_right_max_pm;
        motion.max_pm_overridden = 0U;
    }

    motion.state = MOTION_IDLE;
    motion.done = 1U;
    motion.last_result = res;
    motion.phase = 0U;
    motion.current_pwm = 0.0f;
    motion.current_speed_mm_s = 0.0f;
    motion.current_w_dps = 0.0f;
    motion.cmd_v_mm_s = 0.0f;
    motion.cmd_w_dps = 0.0f;
    motion.cmd_duration_s = 0.0f;
    motion.cmd_hold_heading = 0U;
    motion.steering_bias_dps = 0.0f;
    motion.host_watchdog_enabled = 0U;
    motion.host_watchdog_elapsed_s = 0.0f;
    WheelSpeedPid_Reset(&motion.speed_ctrl);
    ClearSpeedDebug();
    Motor_Stop();
}

void Motion_GetDefaultStraightProfile(Motion_StraightProfile_t *out)
{
    if (!out) return;
    out->max_speed_mm_s = STRAIGHT_SPEED_DEFAULT_MM_S;
    out->min_speed_mm_s = STRAIGHT_SPEED_MIN_MM_S;
    out->end_speed_mm_s = 0.0f;
    out->accel_mm_s2 = STRAIGHT_ACCEL_MM_S2;
    out->decel_mm_s2 = STRAIGHT_DECEL_MM_S2;
    out->max_w_dps = STRAIGHT_W_MAX_DPS;
    out->disable_heading_hold = 0U;
}

void Motion_GetDefaultTurnProfile(Motion_TurnProfile_t *out)
{
    if (!out) return;
    out->max_w_dps = 360.0f;
    out->min_w_dps = 40.0f;
    out->accel_dps2 = 1200.0f;
    out->decel_dps2 = 1200.0f;
    out->angle_kp = 6.0f;
    out->gyro_rate_kp = 0.35f;
    out->tolerance_deg = TURN_TOLERANCE_DEG;
    out->rate_tol_dps = TURN_RATE_TOL_DPS;
    out->timeout_s = TURN_TIMEOUT_S;
}

void Motion_Init(void)
{
    WheelSpeedConfig_t ws_cfg;

    motion.state = MOTION_IDLE;
    motion.done = 1U;
    motion.last_result = MOTION_RES_NONE;
    motion.phase = 0U;
    motion.target_distance_mm = 0.0f;
    motion.target_angle_deg = 0.0f;
    motion.start_yaw_deg = 0.0f;
    motion.distance_mm = 0.0f;
    motion.angle_error_deg = 0.0f;
    motion.current_pwm = 0.0f;
    motion.current_speed_mm_s = 0.0f;
    motion.current_w_dps = 0.0f;
    motion.elapsed_s = 0.0f;
    motion.straight_timeout_s = STRAIGHT_TIMEOUT_MAX_S;
    motion.cmd_v_mm_s = 0.0f;
    motion.cmd_w_dps = 0.0f;
    motion.cmd_duration_s = 0.0f;
    motion.cmd_hold_heading = 0U;
    motion.steering_bias_dps = 0.0f;
    motion.wtest_left_pps = 0.0f;
    motion.wtest_right_pps = 0.0f;
    motion.wtest_duration_s = 0.0f;
    motion.wtest_max_pwm_pm = 300.0f;
    motion.wtest_mode = MOTION_WTEST_STEP;
    motion.max_pm_overridden = 0U;
    motion.saved_left_max_pm = STRAIGHT_PWM_MAX_PM;
    motion.saved_right_max_pm = STRAIGHT_PWM_MAX_PM;
    motion.qstep_level = MOTION_QSTEP_LOW;
    motion.host_watchdog_enabled = 0U;
    motion.host_watchdog_timeout_s = 0.0f;
    motion.host_watchdog_elapsed_s = 0.0f;
    Motion_GetDefaultStraightProfile(&motion.straight_profile);
    Motion_GetDefaultTurnProfile(&motion.turn_profile);
    ClearSpeedDebug();

    /* Outer heading PID outputs angular velocity cmd (deg/s) for cmd_vel. */
    PID_Init(&motion.heading_pid, 4.0f, 0.0f, 0.05f, STRAIGHT_W_MAX_DPS, 40.0f);
    PID_Init(&motion.turn_pid, 1.1f, 0.01f, 0.001f, TURN_PWM_MAX, 50.0f);

    /* Inner wheel speed PI + feedforward outputs permille (capped at 100%). */
    WheelSpeedPid_GetDefaultConfig(&ws_cfg);
    ws_cfg.max_output_pm = STRAIGHT_PWM_MAX_PM;
    WheelSpeedPid_InitConfig(&motion.speed_ctrl, &ws_cfg, NULL);

    Motor_Stop();
}

void Motion_CommandStraightProfile(float distance_mm,
                                   float current_yaw_deg,
                                   const Motion_StraightProfile_t *profile)
{
    uint32_t primask;
    Motion_StraightProfile_t cfg;
    uint8_t chaining;
    float initial_speed;

    if (!isfinite(distance_mm) || distance_mm <= 0.0f) return;

    if (profile) cfg = *profile;
    else Motion_GetDefaultStraightProfile(&cfg);
    SanitizeStraightProfile(&cfg);

    primask = __get_PRIMASK();
    __disable_irq();

    chaining = (motion.state == MOTION_STRAIGHT || motion.state == MOTION_VELOCITY) &&
               !motion.done;
    initial_speed = chaining
                        ? ClampF(motion.current_speed_mm_s,
                                 cfg.min_speed_mm_s, cfg.max_speed_mm_s)
                        : cfg.min_speed_mm_s;

    Encoder_Reset();
    if (!chaining) {
        PID_Reset(&motion.heading_pid);
        WheelSpeedPid_Reset(&motion.speed_ctrl);
        ClearSpeedDebug();
        motion.steering_bias_dps = 0.0f;
    }

    motion.straight_profile = cfg;
    motion.heading_pid.limit_output = cfg.max_w_dps;
    motion.target_distance_mm = distance_mm;
    motion.start_yaw_deg = current_yaw_deg;
    motion.distance_mm = 0.0f;
    motion.angle_error_deg = 0.0f;
    motion.current_pwm = 0.0f;
    motion.current_speed_mm_s = initial_speed;
    motion.current_w_dps = 0.0f;
    motion.elapsed_s = 0.0f;
    motion.straight_timeout_s =
        ClampF((distance_mm / cfg.min_speed_mm_s) * 2.5f + 2.0f, 3.0f, STRAIGHT_TIMEOUT_MAX_S);
    motion.phase = 0U;
    motion.last_result = MOTION_RES_NONE;

    /* Publish state last so the ISR never sees a half-initialized command. */
    motion.done = 0U;
    motion.state = MOTION_STRAIGHT;

    if (!primask) __enable_irq();
}

void Motion_CommandStraight(float distance_mm, float current_yaw_deg)
{
    Motion_CommandStraightProfile(distance_mm, current_yaw_deg, NULL);
}

void Motion_CommandTurn(float angle_deg, float current_yaw_deg)
{
    Motion_CommandTurnProfile(angle_deg, current_yaw_deg, NULL);
}

void Motion_CommandTurnProfile(float angle_deg,
                               float current_yaw_deg,
                               const Motion_TurnProfile_t *profile)
{
    uint32_t primask;
    Motion_TurnProfile_t cfg;

    if (!isfinite(angle_deg) || fabsf(angle_deg) < 1.0f) return;

    if (profile) cfg = *profile;
    else Motion_GetDefaultTurnProfile(&cfg);
    SanitizeTurnProfile(&cfg);

    primask = __get_PRIMASK();
    __disable_irq();

    Encoder_Reset();
    PID_Reset(&motion.heading_pid);
    WheelSpeedPid_Reset(&motion.speed_ctrl);
    ClearSpeedDebug();

    motion.turn_profile = cfg;
    motion.target_angle_deg = angle_deg;
    motion.start_yaw_deg = current_yaw_deg;
    motion.angle_error_deg = angle_deg;
    motion.distance_mm = 0.0f;
    motion.current_pwm = 0.0f;
    motion.current_speed_mm_s = 0.0f;
    motion.current_w_dps = 0.0f;
    motion.steering_bias_dps = 0.0f;
    motion.elapsed_s = 0.0f;
    motion.phase = 0U;
    motion.last_result = MOTION_RES_NONE;

    motion.done = 0U;
    motion.state = MOTION_TURN_PROFILE;

    if (!primask) __enable_irq();
}

void Motion_CommandVelocityTimed(float v_ref_mm_s,
                                 float w_ref_dps,
                                 uint8_t hold_heading,
                                 float target_yaw_deg,
                                 float duration_s)
{
    uint32_t primask;
    float v_clamped;
    float w_clamped;
    float dur_clamped = 0.0f;

    if (!isfinite(v_ref_mm_s)) v_ref_mm_s = 0.0f;
    if (!isfinite(w_ref_dps)) w_ref_dps = 0.0f;
    if (!isfinite(target_yaw_deg)) target_yaw_deg = 0.0f;
    if (isfinite(duration_s) && duration_s > 0.0f) {
        dur_clamped = ClampF(duration_s, 0.05f, WTEST_MAX_DURATION_S);
    }

    v_clamped = ClampF(v_ref_mm_s, -STRAIGHT_SPEED_ABS_MAX_MM_S,
                       STRAIGHT_SPEED_ABS_MAX_MM_S);
    w_clamped = ClampF(w_ref_dps, -VELOCITY_W_ABS_MAX_DPS,
                       VELOCITY_W_ABS_MAX_DPS);

    primask = __get_PRIMASK();
    __disable_irq();

    if (motion.state != MOTION_VELOCITY || motion.done) {
        Encoder_Reset();
        PID_Reset(&motion.heading_pid);
        WheelSpeedPid_Reset(&motion.speed_ctrl);
        ClearSpeedDebug();
        motion.distance_mm = 0.0f;
        motion.angle_error_deg = 0.0f;
        motion.elapsed_s = 0.0f;
        motion.phase = 0U;
        motion.last_result = MOTION_RES_NONE;
    }

    motion.cmd_v_mm_s = v_clamped;
    motion.cmd_w_dps = w_clamped;
    motion.cmd_duration_s = dur_clamped;
    motion.cmd_hold_heading = hold_heading ? 1U : 0U;
    if (hold_heading) {
        motion.start_yaw_deg = target_yaw_deg;
    }
    motion.current_speed_mm_s = v_clamped;
    motion.current_w_dps = w_clamped;

    motion.done = 0U;
    motion.state = MOTION_VELOCITY;

    if (!primask) __enable_irq();
}

void Motion_CommandVelocity(float v_ref_mm_s,
                            float w_ref_dps,
                            uint8_t hold_heading,
                            float target_yaw_deg)
{
    Motion_CommandVelocityTimed(v_ref_mm_s, w_ref_dps, hold_heading, target_yaw_deg, 0.0f);
}

void Motion_CommandWheelSpeed(float left_pps,
                              float right_pps,
                              float duration_s,
                              float max_pwm_pm,
                              Motion_WheelTestMode_t mode)
{
    uint32_t primask;
    float l_clamped;
    float r_clamped;
    float dur_clamped;
    float max_pm;

    if (!isfinite(left_pps)) left_pps = 0.0f;
    if (!isfinite(right_pps)) right_pps = 0.0f;
    if (!isfinite(duration_s) || duration_s <= 0.0f) duration_s = 1.0f;
    if (!isfinite(max_pwm_pm) || max_pwm_pm <= 0.0f) max_pwm_pm = 300.0f;

    l_clamped = ClampF(left_pps, -WTEST_ABS_MAX_PPS, WTEST_ABS_MAX_PPS);
    r_clamped = ClampF(right_pps, -WTEST_ABS_MAX_PPS, WTEST_ABS_MAX_PPS);
    dur_clamped = ClampF(duration_s, 0.05f, WTEST_MAX_DURATION_S);
    max_pm = ClampF(max_pwm_pm, 0.0f, STRAIGHT_PWM_MAX_PM);

    primask = __get_PRIMASK();
    __disable_irq();

    Encoder_Reset();
    PID_Reset(&motion.heading_pid);
    WheelSpeedPid_Reset(&motion.speed_ctrl);
    ClearSpeedDebug();

    if (!motion.max_pm_overridden) {
        motion.saved_left_max_pm = motion.speed_ctrl.left.max_output_pm;
        motion.saved_right_max_pm = motion.speed_ctrl.right.max_output_pm;
        motion.max_pm_overridden = 1U;
    }
    WheelSpeedPid_SetMaxOutput(&motion.speed_ctrl, max_pm);

    motion.wtest_left_pps = l_clamped;
    motion.wtest_right_pps = r_clamped;
    motion.wtest_duration_s = dur_clamped;
    motion.wtest_max_pwm_pm = max_pm;
    motion.wtest_mode = mode;

    motion.target_distance_mm = 0.0f;
    motion.target_angle_deg = 0.0f;
    motion.distance_mm = 0.0f;
    motion.angle_error_deg = 0.0f;
    motion.current_pwm = 0.0f;
    motion.current_speed_mm_s = 0.0f;
    motion.current_w_dps = 0.0f;
    motion.steering_bias_dps = 0.0f;
    motion.elapsed_s = 0.0f;
    motion.phase = 0U;
    motion.last_result = MOTION_RES_NONE;

    motion.done = 0U;
    motion.state = MOTION_WHEEL_TEST;

    if (!primask) __enable_irq();
}

void Motion_EnableHostWatchdog(float timeout_s)
{
    uint32_t primask = __get_PRIMASK();
    float t = (!isfinite(timeout_s) || timeout_s <= 0.0f) ? 1.5f : ClampF(timeout_s, 0.2f, 10.0f);
    __disable_irq();
    motion.host_watchdog_enabled = 1U;
    motion.host_watchdog_timeout_s = t;
    motion.host_watchdog_elapsed_s = 0.0f;
    if (!primask) __enable_irq();
}

void Motion_RefreshHostHeartbeat(void)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    motion.host_watchdog_elapsed_s = 0.0f;
    if (!primask) __enable_irq();
}

void Motion_SetSteeringBiasDps(float bias_w_dps)
{
    uint32_t primask = __get_PRIMASK();
    if (!isfinite(bias_w_dps)) bias_w_dps = 0.0f;
    bias_w_dps = ClampF(bias_w_dps, -STEERING_BIAS_ABS_MAX_DPS,
                        STEERING_BIAS_ABS_MAX_DPS);
    __disable_irq();
    motion.steering_bias_dps = bias_w_dps;
    if (!primask) __enable_irq();
}

void Motion_SetWheelSpeedGains(const WheelSpeedConfig_t *left_cfg,
                               const WheelSpeedConfig_t *right_cfg)
{
    uint32_t primask;
    if (!left_cfg) return;
    primask = __get_PRIMASK();
    __disable_irq();
    WheelSpeedPid_SetGains(&motion.speed_ctrl, left_cfg, right_cfg);
    if (motion.max_pm_overridden) {
        motion.saved_left_max_pm = motion.speed_ctrl.left.max_output_pm;
        motion.saved_right_max_pm = motion.speed_ctrl.right.max_output_pm;
        WheelSpeedPid_SetMaxOutput(&motion.speed_ctrl, motion.wtest_max_pwm_pm);
    }
    if (!primask) __enable_irq();
}

void Motion_GetWheelSpeedGains(WheelSpeedConfig_t *left_cfg,
                               WheelSpeedConfig_t *right_cfg)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    WheelSpeedPid_GetGains(&motion.speed_ctrl, left_cfg, right_cfg);
    if (motion.max_pm_overridden) {
        if (left_cfg) left_cfg->max_output_pm = motion.saved_left_max_pm;
        if (right_cfg) right_cfg->max_output_pm = motion.saved_right_max_pm;
    }
    if (!primask) __enable_irq();
}

void Motion_SetHeadingPidGains(float kp, float ki, float kd, float max_w_dps)
{
    uint32_t primask;
    if (!isfinite(kp) || kp < 0.0f) kp = 0.0f;
    if (!isfinite(ki) || ki < 0.0f) ki = 0.0f;
    if (!isfinite(kd) || kd < 0.0f) kd = 0.0f;
    if (!isfinite(max_w_dps) || max_w_dps <= 0.0f) max_w_dps = STRAIGHT_W_MAX_DPS;
    max_w_dps = ClampF(max_w_dps, 10.0f, VELOCITY_W_ABS_MAX_DPS);

    primask = __get_PRIMASK();
    __disable_irq();
    motion.heading_pid.Kp = kp;
    motion.heading_pid.Ki = ki;
    motion.heading_pid.Kd = kd;
    motion.heading_pid.limit_output = max_w_dps;
    if (!primask) __enable_irq();
}

void Motion_GetHeadingPidGains(float *kp, float *ki, float *kd, float *max_w_dps)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    if (kp) *kp = motion.heading_pid.Kp;
    if (ki) *ki = motion.heading_pid.Ki;
    if (kd) *kd = motion.heading_pid.Kd;
    if (max_w_dps) *max_w_dps = motion.heading_pid.limit_output;
    if (!primask) __enable_irq();
}

void Motion_ResetIntegrators(void)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    PID_Reset(&motion.heading_pid);
    PID_Reset(&motion.turn_pid);
    WheelSpeedPid_Reset(&motion.speed_ctrl);
    if (!primask) __enable_irq();
}

void Motion_RestoreDefaultGains(void)
{
    WheelSpeedConfig_t ws_cfg;
    uint32_t primask;
    WheelSpeedPid_GetDefaultConfig(&ws_cfg);
    ws_cfg.max_output_pm = STRAIGHT_PWM_MAX_PM;
    primask = __get_PRIMASK();
    __disable_irq();
    WheelSpeedPid_SetGains(&motion.speed_ctrl, &ws_cfg, NULL);
    PID_Init(&motion.heading_pid, 4.0f, 0.0f, 0.05f, STRAIGHT_W_MAX_DPS, 40.0f);
    motion.max_pm_overridden = 0U;
    if (!primask) __enable_irq();
}

#if 0 /* Battery compensation disabled: no measured battery voltage. */
void Motion_SetBatteryVoltage(float nominal_v, float measured_v)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    WheelSpeedPid_SetBatteryScale(&motion.speed_ctrl, nominal_v, measured_v);
    if (!primask) __enable_irq();
}
#endif

void Motion_CommandQuickStepTestLevel(Motion_QuickStepLevel_t level)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();

    Encoder_Reset();
    PID_Reset(&motion.heading_pid);
    PID_Reset(&motion.turn_pid);
    WheelSpeedPid_Reset(&motion.speed_ctrl);
    ClearSpeedDebug();

    motion.target_distance_mm = 0.0f;
    motion.target_angle_deg = 0.0f;
    motion.start_yaw_deg = 0.0f;
    motion.angle_error_deg = 0.0f;
    motion.distance_mm = 0.0f;
    motion.current_pwm = 0.0f;
    motion.current_speed_mm_s = 0.0f;
    motion.current_w_dps = 0.0f;
    motion.steering_bias_dps = 0.0f;
    motion.elapsed_s = 0.0f;
    motion.phase = 0U;
    motion.last_result = MOTION_RES_NONE;
    motion.qstep_level = (level <= MOTION_QSTEP_HIGH) ? level : MOTION_QSTEP_LOW;

    motion.done = 0U;
    motion.state = MOTION_QUICK_STEP;

    if (!primask) __enable_irq();
}

void Motion_CommandQuickStepTest(void)
{
    Motion_CommandQuickStepTestLevel(MOTION_QSTEP_LOW);
}

void Motion_Stop(void)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    if (!motion.done || motion.state != MOTION_IDLE) {
        FinishMotionWithResult(MOTION_RES_ABORTED);
    } else {
        Motor_Stop();
    }
    if (!primask) __enable_irq();
}
void Motion_SensorFault(void)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    if (!motion.done || motion.state != MOTION_IDLE) {
        FinishMotionWithResult(MOTION_RES_SENSOR_FAULT);
    } else {
        Motor_Stop();
    }
    if (!primask) __enable_irq();
}

void Motion_Update(float dt_s,
                   int32_t encoder_left,
                   int32_t encoder_right,
                   float yaw_deg,
                   float gyro_z_dps)
{
    float left_mm;
    float right_mm;

    if (motion.done || motion.state == MOTION_IDLE) return;
    if (dt_s <= 0.0f || dt_s > 0.02f) return;

    motion.elapsed_s += dt_s;

    if (motion.host_watchdog_enabled) {
        motion.host_watchdog_elapsed_s += dt_s;
        if (motion.host_watchdog_elapsed_s >= motion.host_watchdog_timeout_s) {
            FinishMotionWithResult(MOTION_RES_ABORTED);
            return;
        }
    }

    left_mm = (float)encoder_left * MM_PER_PULSE;
    right_mm = (float)encoder_right * MM_PER_PULSE;
    motion.distance_mm = 0.5f * (left_mm + right_mm);

    if (motion.state == MOTION_WHEEL_TEST) {
        float l_ref = motion.wtest_left_pps;
        float r_ref = motion.wtest_right_pps;

        if (motion.elapsed_s >= motion.wtest_duration_s) {
            FinishMotionWithResult(MOTION_RES_SUCCESS);
            return;
        }

        if (motion.wtest_mode == MOTION_WTEST_RAMP) {
            float ramp_s = 0.5f * motion.wtest_duration_s;
            if (ramp_s > 0.5f) ramp_s = 0.5f;
            if (ramp_s > 0.01f && motion.elapsed_s < ramp_s) {
                float frac = motion.elapsed_s / ramp_s;
                l_ref *= frac;
                r_ref *= frac;
            }
        }

        ApplyWheelSpeed(l_ref, r_ref, dt_s);
        return;
    }

    if (motion.state == MOTION_QUICK_STEP) {
        static const float qstep_v[3][QSTEP_NUM_PHASES] = {
            { 180.0f, 160.0f, 160.0f,   0.0f,    0.0f, 120.0f }, /* LOW */
            { 300.0f, 250.0f, 250.0f,   0.0f,    0.0f, 200.0f }, /* MEDIUM */
            { 500.0f, 420.0f, 420.0f,   0.0f,    0.0f, 420.0f }  /* HIGH */
        };
        static const float qstep_w[3][QSTEP_NUM_PHASES] = {
            {   0.0f, 120.0f, -120.0f, 180.0f, -180.0f, 0.0f },  /* LOW */
            {   0.0f, 240.0f, -240.0f, 360.0f, -360.0f, 0.0f },  /* MEDIUM */
            {   0.0f, 420.0f, -420.0f, 720.0f, -720.0f, 0.0f }   /* HIGH */
        };
        uint32_t ph = (uint32_t)(motion.elapsed_s / QSTEP_PHASE_DURATION_S);
        uint32_t lvl = (uint32_t)motion.qstep_level;
        float v_cmd;
        float w_cmd;

        if (lvl > 2U) lvl = 0U;
        if (ph >= QSTEP_NUM_PHASES) {
            FinishMotionWithResult(MOTION_RES_SUCCESS);
            return;
        }
        motion.phase = (uint8_t)ph;
        v_cmd = qstep_v[lvl][ph];
        w_cmd = qstep_w[lvl][ph];
        motion.current_speed_mm_s = v_cmd;
        motion.current_w_dps = w_cmd;
        ApplyCmdVel(v_cmd, w_cmd, 0.0f, 0.0f, dt_s);
        return;
    }

    if (motion.state == MOTION_VELOCITY) {
        float w_ref_dps = motion.cmd_w_dps + motion.steering_bias_dps;

        if (motion.cmd_duration_s > 0.0f && motion.elapsed_s >= motion.cmd_duration_s) {
            FinishMotionWithResult(MOTION_RES_SUCCESS);
            return;
        }

        if (motion.cmd_hold_heading) {
            float heading_error = NormalizeAngle(yaw_deg - motion.start_yaw_deg);
            motion.angle_error_deg = heading_error;
            w_ref_dps += PID_Compute(&motion.heading_pid, -heading_error, 0.0f, dt_s);
        } else {
            motion.angle_error_deg = 0.0f;
        }

        w_ref_dps = ClampF(w_ref_dps, -VELOCITY_W_ABS_MAX_DPS,
                           VELOCITY_W_ABS_MAX_DPS);
        motion.current_speed_mm_s = motion.cmd_v_mm_s;
        motion.current_w_dps = w_ref_dps;
        ApplyCmdVel(motion.cmd_v_mm_s, w_ref_dps, 0.0f, 0.0f, dt_s);
        return;
    }

    if (motion.state == MOTION_STRAIGHT) {
        const Motion_StraightProfile_t *p = &motion.straight_profile;
        float remaining_mm = motion.target_distance_mm - motion.distance_mm;
        float v_prev = motion.current_speed_mm_s;
        float v_floor;
        float v_ramp;
        float v_brake;
        float v_ref_mm_s;
        float a_ref_mm_s2;
        float heading_error;
        float w_ref_dps;

        if (remaining_mm <= 0.0f) {
            FinishMotionWithResult(MOTION_RES_SUCCESS);
            return;
        }
        if (motion.elapsed_s >= motion.straight_timeout_s) {
            FinishMotionWithResult(MOTION_RES_TIMEOUT);
            return;
        }

        v_floor = (p->end_speed_mm_s > p->min_speed_mm_s)
                      ? p->end_speed_mm_s
                      : p->min_speed_mm_s;
        v_ramp = v_prev + p->accel_mm_s2 * dt_s;
        v_brake = sqrtf(v_floor * v_floor + 2.0f * p->decel_mm_s2 * remaining_mm);
        v_ref_mm_s = ClampF((v_ramp < v_brake) ? v_ramp : v_brake,
                            v_floor, p->max_speed_mm_s);
        a_ref_mm_s2 = (v_ref_mm_s - v_prev) / dt_s;
        motion.current_speed_mm_s = v_ref_mm_s;

        if (p->disable_heading_hold) {
            motion.angle_error_deg = 0.0f;
            w_ref_dps = motion.steering_bias_dps;
        } else {
            heading_error = NormalizeAngle(yaw_deg - motion.start_yaw_deg);
            motion.angle_error_deg = heading_error;

            /* +w_ref_dps = left turn; negate heading_error (= start_yaw - yaw)
             * so drifting left (+heading_error) commands right correction (-w). */
            w_ref_dps = PID_Compute(&motion.heading_pid, -heading_error, 0.0f, dt_s) +
                        motion.steering_bias_dps;
        }
        w_ref_dps = ClampF(w_ref_dps, -p->max_w_dps, p->max_w_dps);
        motion.current_w_dps = w_ref_dps;

        ApplyCmdVel(v_ref_mm_s, w_ref_dps, a_ref_mm_s2, 0.0f, dt_s);
        return;
    }

    if (motion.state == MOTION_TURN_PROFILE) {
        const Motion_TurnProfile_t *p = &motion.turn_profile;
        float traveled = NormalizeAngle(yaw_deg - motion.start_yaw_deg);
        float error = NormalizeAngle(motion.target_angle_deg - traveled);
        float w_prev = motion.current_w_dps;
        float w_ramp;
        float w_brake;
        float w_env;
        float w_target;
        float w_cmd;
        float alpha_ref;

        motion.angle_error_deg = error;

        if (fabsf(error) <= p->tolerance_deg &&
            fabsf(gyro_z_dps) <= p->rate_tol_dps) {
            FinishMotionWithResult(MOTION_RES_SUCCESS);
            return;
        }
        if (motion.elapsed_s >= p->timeout_s) {
            FinishMotionWithResult(MOTION_RES_TIMEOUT);
            return;
        }

        w_ramp = fabsf(w_prev) + p->accel_dps2 * dt_s;
        w_brake = sqrtf(p->min_w_dps * p->min_w_dps +
                        2.0f * p->decel_dps2 * fabsf(error));
        w_env = ClampF((w_ramp < w_brake) ? w_ramp : w_brake,
                       p->min_w_dps, p->max_w_dps);

        /* Smooth proportional approach near 0 deg prevents sign-flip chatter. */
        w_target = ClampF(error * p->angle_kp, -w_env, w_env);
        alpha_ref = (w_target - w_prev) / dt_s;
        motion.current_w_dps = w_target;

        /* Inner gyro-rate feedback compensates tire slip on slick surfaces. */
        w_cmd = ClampF(w_target + p->gyro_rate_kp * (w_target - gyro_z_dps),
                       -p->max_w_dps, p->max_w_dps);
        ApplyCmdVel(0.0f, w_cmd, 0.0f, alpha_ref, dt_s);
        return;
    }
}

uint8_t Motion_IsDone(void) { return motion.done; }
Motion_State_t Motion_GetState(void) { return motion.state; }
Motion_Result_t Motion_GetLastResult(void) { return motion.last_result; }
uint8_t Motion_GetPhase(void) { return motion.phase; }
float Motion_GetDistanceMm(void) { return motion.distance_mm; }
float Motion_GetRemainingDistanceMm(void)
{
    float rem;
    if (motion.state != MOTION_STRAIGHT || motion.done) return 0.0f;
    rem = motion.target_distance_mm - motion.distance_mm;
    return (rem > 0.0f) ? rem : 0.0f;
}
float Motion_GetAngleErrorDeg(void) { return motion.angle_error_deg; }

void Motion_GetChassisVelocity(float *v_meas_mm_s, float *w_meas_dps)
{
    uint32_t primask = __get_PRIMASK();
    float l_pps;
    float r_pps;
    __disable_irq();
    l_pps = Encoder_GetLeftSpeedPPS();
    r_pps = Encoder_GetRightSpeedPPS();
    if (!primask) __enable_irq();
    WheelSpeedPid_WheelPpsToCmdVel(l_pps, r_pps, v_meas_mm_s, w_meas_dps);
}

void Motion_GetSpeedDebug(Motion_SpeedDebug_t *out)
{
    uint32_t primask;
    if (!out) return;
    primask = __get_PRIMASK();
    __disable_irq();
    *out = motion.speed_debug;
    if (!primask) __enable_irq();
}

void Motion_GetExtendedDebug(Motion_ExtendedDebug_t *out)
{
    uint32_t primask;
    if (!out) return;
    primask = __get_PRIMASK();
    __disable_irq();
    out->v_ref_mm_s = motion.current_speed_mm_s;
    out->w_ref_dps = motion.current_w_dps;
    out->v_meas_mm_s = motion.v_meas_mm_s;
    out->w_meas_dps = motion.w_meas_dps;
    out->left_integral = motion.speed_ctrl.left.integral;
    out->right_integral = motion.speed_ctrl.right.integral;
    out->elapsed_s = motion.elapsed_s;
    out->sat_flags = motion.sat_flags;
    out->phase = motion.phase;
    out->result = motion.last_result;
    if (!primask) __enable_irq();
}
