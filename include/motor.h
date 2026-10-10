#ifndef MOTOR_H
#define MOTOR_H

#include <stdint.h>

/* API command scale: -1000..+1000 (0.1% resolution).
 * The low-level TIM2 PWM remains 20 kHz.
 * All output changes, except immediate stop/brake, are serviced at 1 kHz. */
typedef enum {
    MOTOR_STOP_COAST = 0,
    MOTOR_STOP_BRAKE = 1
} MotorStopMode_t;

void Motor_Init(void);
void Motor_SetPair(int16_t left_percent, int16_t right_percent); /* legacy */
void Motor_SetPairPermille(int16_t left, int16_t right);
void Motor_SetDeadband(uint16_t left_permille, uint16_t right_permille);
void Motor_SetSlewRates(uint16_t rise_permille_per_ms, uint16_t fall_permille_per_ms);
void Motor_Update1ms(void);
void Motor_GetAppliedPairPermille(int16_t *left, int16_t *right);

void Motor_Stop(void);   /* immediate COAST; clears target & pending ramp */
void Motor_Brake(void);  /* immediate short brake, latched until next command */
void Motor_StopWithMode(MotorStopMode_t mode);

#endif
