#ifndef MOTION_H
#define MOTION_H

#include <stdint.h>
#include "motion_primitives.h"
#include "wheel_speed_pid.h"

typedef enum {
    MOTION_IDLE = 0,
    MOTION_STRAIGHT,
    MOTION_TURN,
    MOTION_PIVOT_PRIMITIVE,
    MOTION_QUICK_TEST,
    MOTION_VELOCITY,
    MOTION_TURN_PROFILE
} Motion_State_t;

typedef struct {
    float max_speed_mm_s;
    float min_speed_mm_s;
    float end_speed_mm_s;   /* >0 keeps exit velocity for cell chaining; stops safely if not chained */
    float accel_mm_s2;
    float decel_mm_s2;
    float max_w_dps;
} Motion_StraightProfile_t;

typedef struct {
    float max_w_dps;
    float min_w_dps;
    float accel_dps2;
    float decel_dps2;
    float angle_kp;         /* deg/s per deg of heading error */
    float gyro_rate_kp;     /* dimensionless inner gyro-rate damping/slip compensation */
    float tolerance_deg;
    float rate_tol_dps;
    float timeout_s;
} Motion_TurnProfile_t;

void Motion_Init(void);

/* Default profiles for straight and gyro-closed-loop velocity turns. */
void Motion_GetDefaultStraightProfile(Motion_StraightProfile_t *out);
void Motion_GetDefaultTurnProfile(Motion_TurnProfile_t *out);

/* Basic motion commands (backward-compatible). */
void Motion_CommandStraight(float distance_mm, float current_yaw_deg);
void Motion_CommandTurn(float angle_deg, float current_yaw_deg);
void Motion_CommandQuickStepTest(void);

/* Scalable profiled motion & continuous chassis velocity streaming APIs. */
void Motion_CommandStraightProfile(float distance_mm,
                                   float current_yaw_deg,
                                   const Motion_StraightProfile_t *profile);

void Motion_CommandTurnProfile(float angle_deg,
                               float current_yaw_deg,
                               const Motion_TurnProfile_t *profile);

/* Continuous chassis velocity command (v_mm_s, w_dps) for wall-following,
 * smooth clothoid curves, or high-level maze controllers. Calling repeatedly
 * while already in MOTION_VELOCITY updates setpoints without resetting PID.
 * If hold_heading != 0, outer heading_pid stabilizes yaw around target_yaw_deg. */
void Motion_CommandVelocity(float v_ref_mm_s,
                            float w_ref_dps,
                            uint8_t hold_heading,
                            float target_yaw_deg);

/* Inject bounded steering-rate bias (deg/s, e.g. from IR wall controller)
 * into MOTION_STRAIGHT and MOTION_VELOCITY. Reset to 0 on Stop/Finish. */
void Motion_SetSteeringBiasDps(float bias_w_dps);

/* Runtime gain & battery compensation hooks (ISR-safe). */
void Motion_SetWheelSpeedGains(const WheelSpeedConfig_t *left_cfg,
                               const WheelSpeedConfig_t *right_cfg);
void Motion_GetWheelSpeedGains(WheelSpeedConfig_t *left_cfg,
                               WheelSpeedConfig_t *right_cfg);
void Motion_SetHeadingPidGains(float kp, float ki, float kd, float max_w_dps);
void Motion_SetBatteryVoltage(float nominal_v, float measured_v);

/* Opt-in motion-primitive runtime. Existing CommandTurn legacy PID
 * and BLE L/R behavior remain unchanged until separately qualified.
 * +yaw = left, -yaw = right. One active primitive at a time. */
MotionPrimitiveResult Motion_RequestPrimitive(MotionPrimitiveKind kind,
                                               MotionPrimitiveDirection direction,
                                               float current_yaw_deg);
MotionPrimitiveResult Motion_Turn90(MotionPrimitiveDirection direction,
                                     float current_yaw_deg);
MotionPrimitiveResult Motion_Turn180(MotionPrimitiveDirection direction,
                                      float current_yaw_deg);
MotionPrimitiveStatus Motion_GetPrimitiveStatus(void);
MotionPrimitiveResult Motion_GetPrimitiveResult(void);
MotionPivotPhase Motion_GetPrimitivePhase(void);
void Motion_Stop(void);
void Motion_SensorFault(void); /* Fail active primitive with SENSOR_FAULT, force motor neutral. */

void Motion_Update(float dt_s,
                   int32_t encoder_left,
                   int32_t encoder_right,
                   float yaw_deg,
                   float gyro_z_dps);

typedef struct {
    int32_t left_ref_pps;
    int32_t right_ref_pps;
    int32_t left_meas_pps;
    int32_t right_meas_pps;
    int16_t left_pm;
    int16_t right_pm;
} Motion_SpeedDebug_t;

uint8_t Motion_IsDone(void);
Motion_State_t Motion_GetState(void);
float Motion_GetDistanceMm(void);
float Motion_GetRemainingDistanceMm(void);
float Motion_GetAngleErrorDeg(void);
void Motion_GetChassisVelocity(float *v_meas_mm_s, float *w_meas_dps);
void Motion_GetSpeedDebug(Motion_SpeedDebug_t *out);

#endif
