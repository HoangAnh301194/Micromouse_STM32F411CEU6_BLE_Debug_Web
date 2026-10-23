#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include "speed_pid_diag.h"
#include "motor.h"
static float fake_left_pps, fake_right_pps;
static int16_t left_cmd, right_cmd;
static int commands,stops;
float Encoder_GetLeftSpeedPPS(void) { return fake_left_pps; }
float Encoder_GetRightSpeedPPS(void) { return fake_right_pps; }
void Motor_SetPairPermille(int16_t left, int16_t right)
{
    assert(left >= 0 && left <= 300);
    assert(right >= 0 && right <= 300);
    left_cmd=left; right_cmd=right;commands++;
}
void Motor_Stop(void) {left_cmd=right_cmd=0; stops++;}
int main(void)
{
    SpeedPidDiagSnapshot s;
    SpeedPidDiag_Init();
    fake_left_pps=0;fake_right_pps=0;
    SpeedPidDiag_Run(1000,0,1);
    SpeedPidDiag_Get(&s);
    assert(s.target_pps==800 && s.feedforward_permille==100);
    assert(left_cmd>0 && right_cmd==0 && commands==1);
    SpeedPidDiag_Run(1005,0,1);
    assert(commands==1); /* never runs PID above 100Hz */
    SpeedPidDiag_Run(1010,0,1);
    assert(commands==2);
    SpeedPidDiag_Run(1020,2,3);
    SpeedPidDiag_Get(&s);
    assert(s.target_pps==2400 && s.max_pwm_permille==300);
    assert(left_cmd<=300 && right_cmd<=300);
    fake_left_pps=12000;fake_right_pps=12000;
    SpeedPidDiag_Run(1030,2,3);
    assert(left_cmd>=0 && right_cmd>=0);
    SpeedPidDiag_Stop();
    SpeedPidDiag_Get(&s);
    assert(s.target_pps==0 && left_cmd==0 && right_cmd==0 && stops>0);
    puts("PASS real PID_Compute with fixed gains, isolated 100Hz sampling and 30% clamp");
    return 0;
}
