#ifndef MOTION_H
#define MOTION_H

#include <stdint.h>

typedef enum {
    MOTION_IDLE = 0,
    MOTION_STRAIGHT,
    MOTION_TURN
} Motion_State_t;

void Motion_Init(void);

void Motion_CommandStraight(float distance_mm, float current_yaw_deg);
void Motion_CommandTurn(float angle_deg, float current_yaw_deg);
void Motion_Stop(void);

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
