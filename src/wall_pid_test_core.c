#include "wall_pid_test_core.h"
#include <math.h>
#include <string.h>

static float Clip(float x,float lo,float hi){return x<lo?lo:(x>hi?hi:x);}
static float Abs(float x){return x<0.0f?-x:x;}

void WP_Init(WP_Controller *p){if(p)memset(p,0,sizeof(*p));}
void WP_Stop(WP_Controller *p,WP_Reason reason)
{
    if(!p || p->state!=WP_RUNNING)return;
    p->reason=reason;
    p->state=reason==WP_FINISHED?WP_DONE:WP_FAULT;
    p->pwm_l=p->pwm_r=0;
}
uint8_t WP_Start(WP_Controller *p,const WP_Input *in)
{
    uint16_t l,r;
    if(!p||!in||p->state!=WP_IDLE||
       in->ir.status!=IR_WALL_VALID||in->ir.age_us>IR_WALL_MAX_AGE_US||
       !(in->yaw_deg>-360.0f&&in->yaw_deg<360.0f)||
       !(in->gyro_dps>-30.0f&&in->gyro_dps<30.0f))return 0U;
    l=in->ir.ema10_adc[IR_WALL_L0];r=in->ir.ema10_adc[IR_WALL_R0];
    /* Both walls required; 50..60mm reference ADC range, white matte,
     * user physically measures and centres robot at starting position.
     * NO case of "right only" or "left only" is supported in this mode. */
    if(l<290U||l>750U||r<390U||r>1300U)return 0U;
    WP_Init(p);
    p->state=WP_RUNNING;p->reason=WP_NONE;
    p->initial_yaw_deg=in->yaw_deg;
    p->ref_l_adc=l;p->ref_r_adc=r;
    p->previous_left=in->left_count;p->previous_right=in->right_count;
    return 1U;
}
uint8_t WP_Step(WP_Controller *p,const WP_Input *in)
{
    uint16_t l,r;
    float dl,dr,abs_error,derivative,integral_next,steer;
    float velocity,target_brake,velocity_error,base;
    int32_t avg_count_delta;
    if(!p||!in||p->state!=WP_RUNNING)return 0U;
    ++p->ticks;
    p->last_l0_adc=in->ir.ema10_adc[IR_WALL_L0];
    p->last_r0_adc=in->ir.ema10_adc[IR_WALL_R0];
    p->last_l0_raw=in->ir.raw_adc[IR_WALL_L0];
    p->last_r0_raw=in->ir.raw_adc[IR_WALL_R0];
    p->last_ir_age_us=in->ir.age_us;
    p->last_ir_scan=in->ir.scan_count;
    if(in->ir.status!=IR_WALL_VALID||in->ir.age_us>IR_WALL_MAX_AGE_US){
        WP_Stop(p,WP_IR_STALE);return 0U;
    }
    if(!(in->yaw_deg>-360.0f&&in->yaw_deg<360.0f)||
       !(in->gyro_dps>-720.0f&&in->gyro_dps<720.0f)){
        WP_Stop(p,WP_IMU_STALE);return 0U;
    }
    l=in->ir.ema10_adc[IR_WALL_L0];r=in->ir.ema10_adc[IR_WALL_R0];
    if(l<145U||r<200U||l>1500U||r>2100U){
        WP_Stop(p,WP_WALL_LOST);return 0U;
    }
    /* Fails closed on physical displacement beyond tested 50..60mm
     * interval plus conservative margin; profile is not a maze racer. */
    if(Abs((float)l-p->ref_l_adc)>165.0f||
       Abs((float)r-p->ref_r_adc)>510.0f){
        WP_Stop(p,WP_EXCURSION);return 0U;
    }
    p->yaw_error_deg=in->yaw_deg-p->initial_yaw_deg;
    if(Abs(p->yaw_error_deg)>15.0f){
        WP_Stop(p,WP_HEADING);return 0U;
    }
    p->distance_mm=0.5f*((float)in->left_count+
                           (float)in->right_count)*WP_MM_PER_COUNT;
    if(in->left_count < -20 || in->right_count < -20 ||
       (p->distance_mm > 120.0f &&
        Abs((float)(in->left_count-in->right_count)) > 850.0f)){
        WP_Stop(p,WP_ENCODER);return 0U;
    }
    if(p->distance_mm>=WP_TARGET_MM){
        WP_Stop(p,WP_FINISHED);return 0U;
    }
    if(p->ticks>=WP_MAX_TICKS){
        WP_Stop(p,WP_TIMEOUT);return 0U;
    }
    if(p->ticks>650U && in->left_count<20 && in->right_count<20){
        WP_Stop(p,WP_ENCODER);return 0U;
    }
    /* Per-side local conversion is REQUIRED. A direct L0-R0 ADC
     * subtraction incorrectly interprets unequal optical responses.
     * dl/dr approximate distance change from the operator-centred
     * starting pose; not absolute wall distance in millimetres. */
    dl=((float)p->ref_l_adc-(float)l)/WP_LEFT_ADC_PER_MM;
    dr=((float)p->ref_r_adc-(float)r)/WP_RIGHT_ADC_PER_MM;
    p->error_mm=Clip((dr-dl)*0.5f,-14.0f,14.0f);
    if((p->ticks%WP_PID_TICKS)!=0U)return 0U;
    avg_count_delta=(in->left_count-p->previous_left)+
                    (in->right_count-p->previous_right);
    if(avg_count_delta< -5){
        WP_Stop(p,WP_ENCODER);return 0U;
    }
    velocity=((float)avg_count_delta*0.5f*WP_MM_PER_COUNT)/0.005f;
    p->speed_mm_s=Clip(0.65f*p->speed_mm_s+
                       0.35f*velocity,0.0f,2000.0f);
    if(p->speed_mm_s>1500.0f){
        WP_Stop(p,WP_OVERSPEED);return 0U;
    }
    if(p->speed_mm_s>p->peak_speed_mm_s)
        p->peak_speed_mm_s=p->speed_mm_s;
    p->previous_left=in->left_count;p->previous_right=in->right_count;

    p->ramp_speed_mm_s=Clip(p->ramp_speed_mm_s+
                            WP_ACCEL_MM_S2*0.005f,0.0f,WP_SPEED_MM_S);
    target_brake=sqrtf(2.0f*WP_DECEL_MM_S2*
                       (WP_TARGET_MM-p->distance_mm));
    p->target_speed_mm_s=Clip(target_brake,0.0f,p->ramp_speed_mm_s);
    velocity_error=p->target_speed_mm_s-p->speed_mm_s;
    p->speed_integral_pm=Clip(p->speed_integral_pm+
                             velocity_error*0.005f*0.14f,
                             -160.0f,160.0f);
    base=Clip(75.0f + p->target_speed_mm_s*0.51f+
              0.32f*velocity_error+p->speed_integral_pm,
              0.0f,(float)WP_PWM_CAP_PM);
    derivative=p->has_previous?
        0.75f*p->derivative_mm_s+
        0.25f*((p->error_mm-p->last_error_mm)/0.005f):0.0f;
    integral_next=Clip(p->integral_error+p->error_mm*0.005f,
                       -20.0f,20.0f);
    /* Wall-centre PID ONLY; gyro is measured as a safety guard,
     * not added into the controller correction. */
    steer=6.0f*p->error_mm+0.6f*integral_next+0.018f*derivative;
    if(Abs(steer)<WP_STEER_CAP_PM)p->integral_error=integral_next;
    steer=Clip(steer,-(float)WP_STEER_CAP_PM,(float)WP_STEER_CAP_PM);
    /* Before: base PWM hard-capped at 600 (800-200) even when
     * steering requested 0. Now reserve only actual abs(steer) so
     * BOTH commands remain <=800; do not make correction asymmetric. */
    base=Clip(base,0.0f,(float)WP_PWM_CAP_PM-Abs(steer));
    p->derivative_mm_s=derivative;p->last_error_mm=p->error_mm;
    p->has_previous=1U;
    p->pwm_l=(int16_t)Clip(base+steer,0.0f,WP_PWM_CAP_PM);
    p->pwm_r=(int16_t)Clip(base-steer,0.0f,WP_PWM_CAP_PM);
    if(p->pwm_l>p->peak_pwm_pm)p->peak_pwm_pm=p->pwm_l;
    if(p->pwm_r>p->peak_pwm_pm)p->peak_pwm_pm=p->pwm_r;
    ++p->pid_updates;
    abs_error=Abs(p->error_mm);
    p->abs_error_sum_mm+=abs_error;
    if(abs_error>p->peak_abs_error_mm)p->peak_abs_error_mm=abs_error;
    return 1U;
}
