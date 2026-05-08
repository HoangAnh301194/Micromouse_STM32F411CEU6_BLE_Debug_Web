#ifndef PID_H
#define PID_H

#include <stdint.h>

typedef struct {
    float Kp;
    float Ki;
    float Kd;
    
    float limit_output;   
    float limit_integral; 
    
    float prev_error;
    float integral;
    float output;
} PID_Handle_t;

void PID_Init(PID_Handle_t *pid, float p, float i, float d, float limit_out, float limit_int);
void PID_Reset(PID_Handle_t *pid);
float PID_Compute(PID_Handle_t *pid, float setpoint, float input, float dt);

#endif
