#ifndef WHEEL_SPEED_PID_H
#define WHEEL_SPEED_PID_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float kp;
    float ki;
    float ks_permille;
    float kv_permille_per_pps;
    float ka_permille_per_pps2;
    float max_output_pm;
    float integral_limit;
} WheelSpeedConfig_t;

typedef struct {
    float kp;
    float ki;
    float ks_permille;
    float kv_permille_per_pps;
    float ka_permille_per_pps2;
    float max_output_pm;
    float integral_limit;
    float integral;
    float last_ref_pps;
    float last_meas_pps;
    float last_output_pm;
} WheelSpeedLoop_t;

typedef struct {
    WheelSpeedLoop_t left;
    WheelSpeedLoop_t right;
    float max_output_pm;
    // float battery_scale;  /* Disabled: no battery-voltage measurement. */
} WheelSpeedController_t;

typedef struct {
    float left_ref_pps;
    float right_ref_pps;
    float left_meas_pps;
    float right_meas_pps;
    int16_t left_pm;
    int16_t right_pm;
} WheelSpeedOutput_t;

/* Default tuned gains verified on bench/maze (2026-11-24):
 * kp=0.090, ki=0.250, ks=80.0 permille, kv=0.098 permille/PPS, ka=0.0. */
void WheelSpeedPid_GetDefaultConfig(WheelSpeedConfig_t *out);

void WheelSpeedPid_Init(WheelSpeedController_t *ctrl,
                        float kp,
                        float ki,
                        float ks_permille,
                        float kv_permille_per_pps,
                        float max_output_pm,
                        float integral_limit);

/* Initialize or update left/right gains independently.
 * If right_cfg is NULL, left_cfg is applied to both wheels.
 * SetGains preserves live integral state for glitch-free online tuning. */
void WheelSpeedPid_InitConfig(WheelSpeedController_t *ctrl,
                              const WheelSpeedConfig_t *left_cfg,
                              const WheelSpeedConfig_t *right_cfg);

void WheelSpeedPid_SetGains(WheelSpeedController_t *ctrl,
                            const WheelSpeedConfig_t *left_cfg,
                            const WheelSpeedConfig_t *right_cfg);

void WheelSpeedPid_GetGains(const WheelSpeedController_t *ctrl,
                            WheelSpeedConfig_t *left_cfg,
                            WheelSpeedConfig_t *right_cfg);

void WheelSpeedPid_Reset(WheelSpeedController_t *ctrl);

void WheelSpeedPid_SetMaxOutput(WheelSpeedController_t *ctrl, float max_output_pm);

#if 0 /* Battery compensation disabled: no measured battery voltage. */
/* Battery voltage compensation hook: scales PWM command by nominal_v / measured_v
 * (clamped safely to [0.75, 1.35]). Pass invalid/<=0 values to reset scale to 1.0. */
void WheelSpeedPid_SetBatteryScale(WheelSpeedController_t *ctrl,
                                   float nominal_v,
                                   float measured_v);
#endif

/* Forward kinematics: chassis (v_mm_s, w_dps) -> wheel speeds (PPS).
 * +w_ref_dps = left turn (right wheel faster than left wheel). */
void WheelSpeedPid_CmdVelToWheelPps(float v_ref_mm_s,
                                    float w_ref_dps,
                                    float *left_ref_pps,
                                    float *right_ref_pps);

/* Inverse kinematics: wheel speeds (PPS) -> chassis (v_mm_s, w_dps). */
void WheelSpeedPid_WheelPpsToCmdVel(float left_pps,
                                    float right_pps,
                                    float *v_mm_s,
                                    float *w_dps);

void WheelSpeedPid_UpdateWheelPps(WheelSpeedController_t *ctrl,
                                  float left_ref_pps,
                                  float right_ref_pps,
                                  float left_meas_pps,
                                  float right_meas_pps,
                                  float dt_s,
                                  WheelSpeedOutput_t *out);

void WheelSpeedPid_UpdateWheelPpsWithAccel(WheelSpeedController_t *ctrl,
                                           float left_ref_pps,
                                           float right_ref_pps,
                                           float left_accel_pps2,
                                           float right_accel_pps2,
                                           float left_meas_pps,
                                           float right_meas_pps,
                                           float dt_s,
                                           WheelSpeedOutput_t *out);

void WheelSpeedPid_UpdateCmdVel(WheelSpeedController_t *ctrl,
                                float v_ref_mm_s,
                                float w_ref_dps,
                                float left_meas_pps,
                                float right_meas_pps,
                                float dt_s,
                                WheelSpeedOutput_t *out);

void WheelSpeedPid_UpdateCmdVelWithAccel(WheelSpeedController_t *ctrl,
                                         float v_ref_mm_s,
                                         float w_ref_dps,
                                         float a_ref_mm_s2,
                                         float alpha_ref_dps2,
                                         float left_meas_pps,
                                         float right_meas_pps,
                                         float dt_s,
                                         WheelSpeedOutput_t *out);

#ifdef __cplusplus
}
#endif

#endif /* WHEEL_SPEED_PID_H */
