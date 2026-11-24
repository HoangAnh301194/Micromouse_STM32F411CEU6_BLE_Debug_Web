#include "pid.h"
#include <math.h>

void PID_Init(PID_Handle_t *pid, float p, float i, float d, float limit_out, float limit_int) {
    if (!pid) return;
    pid->Kp = p;
    pid->Ki = i;
    pid->Kd = d;
    pid->limit_output = limit_out;
    pid->limit_integral = limit_int;
    PID_Reset(pid);
}

void PID_Reset(PID_Handle_t *pid) {
    if (!pid) return;
    pid->prev_error = 0.0f;
    pid->integral = 0.0f;
    pid->output = 0.0f;
}

float PID_Compute(PID_Handle_t *pid, float setpoint, float input, float dt) {
    float error, output;
    float p_out, i_out, d_out;

    if (!pid || !isfinite(setpoint) || !isfinite(input) || !isfinite(dt) || dt <= 0.0f) {
        return 0.0f;
    }

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
    d_out = 0.0f;
    if (dt > 0.0001f) {
        d_out = pid->Kd * (error - pid->prev_error) / dt;
    }

    /* sum */
    output = p_out + i_out + d_out;
    if (!isfinite(output)) {
        PID_Reset(pid);
        return 0.0f;
    }

    /* limit output */
    if (output > pid->limit_output) output = pid->limit_output;
    else if (output < -pid->limit_output) output = -pid->limit_output;

    pid->prev_error = error;
    pid->output = output;

    return output;
}
