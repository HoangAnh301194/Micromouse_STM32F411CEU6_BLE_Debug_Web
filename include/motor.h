#ifndef MOTOR_H
#define MOTOR_H

#include <stdint.h>

void Motor_Init(void);
void Motor_SetPair(int16_t left_pwm, int16_t right_pwm);
void Motor_Stop(void);
void Motor_Brake(void);

#endif
