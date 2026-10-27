#ifndef MOTION_H
#define MOTION_H

#include <stdint.h>
#include "motion_primitives.h"

typedef enum {
    MOTION_IDLE = 0,
    MOTION_STRAIGHT,
    MOTION_TURN,
    MOTION_PIVOT_PRIMITIVE
} Motion_State_t;

void Motion_Init(void);

void Motion_CommandStraight(float distance_mm, float current_yaw_deg);
void Motion_CommandTurn(float angle_deg, float current_yaw_deg);

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

uint8_t Motion_IsDone(void);
Motion_State_t Motion_GetState(void);
float Motion_GetDistanceMm(void);
float Motion_GetAngleErrorDeg(void);

#endif
