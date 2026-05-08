#include "pid.h"

void PID_Init(PID_Handle_t *pid, float p, float i, float d, float limit_out, float limit_int) {
    pid->Kp = p;
    pid->Ki = i;
    pid->Kd = d;
    pid->limit_output = limit_out;
    pid->limit_integral = limit_int;
    PID_Reset(pid);
}

void PID_Reset(PID_Handle_t *pid) {
    pid->prev_error = 0;
    pid->integral = 0;
}

float PID_Compute(PID_Handle_t *pid, float setpoint, float input, float dt) {
    
    float error, output;
    float p_out, i_out, d_out; 
    
    error = setpoint - input;
    
    /* 1. P Term */
    p_out = pid->Kp * error;
    
    /* 2. I Term */
    pid->integral += error * dt;
    
    /* limit integral (Anti-windup) */
    if (pid->integral > pid->limit_integral) pid->integral = pid->limit_integral;
    else if (pid->integral < -pid->limit_integral) pid->integral = -pid->limit_integral;
    
    i_out = pid->Ki * pid->integral;
    
    /* 3. D Term */
    d_out = 0;
    if (dt > 0.0001f) {
        d_out = pid->Kd * (error - pid->prev_error) / dt;
    }
    
    /* sum */
    output = p_out + i_out + d_out;
    
    /* limit output */
    if (output > pid->limit_output) output = pid->limit_output;
    else if (output < -pid->limit_output) output = -pid->limit_output;
    
    pid->prev_error = error;
    pid->output = output;
    
    return output;
}
