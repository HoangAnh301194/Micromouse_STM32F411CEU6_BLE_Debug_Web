#include "wall_pid_test_core.h"
#include <string.h>

/* Test-only local 50->60mm slopes from static physical data.
 * No global LUT and no absolute-distance measurements or PID corrections
 * outside this bracket. Baseline is supplied by an operator-centered robot. */
static float Clamp(float v,float low,float high)
{
    return v<low?low:(v>high?high:v);
}
static float Abs(float v) { return v<0.0f?-v:v; }

void WP_Init(WP_Controller *p)
{
    if(p)memset(p,0,sizeof(*p));
}
void WP_Stop(WP_Controller *p,WP_Reason reason)
{
    if(!p || p->state!=WP_RUNNING)return;
    p->state=(reason==WP_FINISHED)?WP_DONE:WP_FAULT;
    p->reason=reason;
    p->pwm_l=p->pwm_r=0;
}
uint8_t WP_Start(WP_Controller *p,const WP_Input *in)
{
    uint16_t l,r;
    if(!p||!in||p->state!=WP_IDLE||in->ir.status!=IR_WALL_VALID ||
       in->ir.age_us>IR_WALL_MAX_AGE_US ||
       !(in->yaw_deg>-360.0f && in->yaw_deg<360.0f) ||
       !(in->gyro_dps>-30.0f && in->gyro_dps<30.0f))return 0U;
    l=in->ir.ema10_adc[IR_WALL_L0];
    r=in->ir.ema10_adc[IR_WALL_R0];
    /* Tight 60mm physical bench setup gate; not wall detection for racing. */
    if(l<390U||l>540U||r<540U||r>825U)return 0U;
    WP_Init(p);
    p->state=WP_RUNNING;
    p->initial_yaw_deg=in->yaw_deg;
    p->ref_l_adc=l;p->ref_r_adc=r;
    return 1U;
}
uint8_t WP_Step(WP_Controller *p,const WP_Input *in)
{
    float dl,dr,raw_deriv,deriv,corr,integ_next;
    uint16_t l,r;
    if(!p||!in||p->state!=WP_RUNNING)return 0U;
    ++p->ticks;
    if(in->ir.status!=IR_WALL_VALID || in->ir.age_us>IR_WALL_MAX_AGE_US) {
        WP_Stop(p,WP_IR_STALE);return 0U;
    }
    if(!(in->yaw_deg>-360.0f && in->yaw_deg<360.0f) ||
       !(in->gyro_dps>-250.0f && in->gyro_dps<250.0f)) {
        WP_Stop(p,WP_IMU_STALE);return 0U;
    }
    l=in->ir.ema10_adc[IR_WALL_L0];
    r=in->ir.ema10_adc[IR_WALL_R0];
    if(l<90U||r<120U||l>1300U||r>1800U) {
        WP_Stop(p,WP_WALL_LOST);return 0U;
    }
    if(Abs((float)l-p->ref_l_adc)>120.0f ||
       Abs((float)r-p->ref_r_adc)>380.0f) {
        WP_Stop(p,WP_EXCURSION);return 0U;
    }
    p->yaw_error_deg=in->yaw_deg-p->initial_yaw_deg;
    if(Abs(p->yaw_error_deg)>10.0f) {
        WP_Stop(p,WP_HEADING);return 0U;
    }
    p->distance_mm=0.5f*((float)in->left_count+
                           (float)in->right_count)*WP_MM_PER_TICK;
    if(in->left_count< -15||in->right_count< -15||
       (p->distance_mm>35.0f &&
        Abs((float)(in->left_count-in->right_count))>250.0f)) {
        WP_Stop(p,WP_ENCODER);return 0U;
    }
    if(p->distance_mm>=WP_TARGET_MM) {
        WP_Stop(p,WP_FINISHED);return 0U;
    }
    if(p->ticks>=WP_MAX_TICKS) {
        WP_Stop(p,WP_TIMEOUT);return 0U;
    }
    if(p->ticks>=700U && in->left_count<10 && in->right_count<10) {
        WP_Stop(p,WP_ENCODER);return 0U;
    }
    /* Relative displacement from initial 60mm alignment. Right-left
     * geometry is corrected for the independent sensor sensitivities.
     * Positive error means robot closer to LEFT wall -> steer RIGHT,
     * left motor faster, right motor slower. */
    dl=((float)p->ref_l_adc-(float)l)/WP_LEFT_ADC_PER_MM;
    dr=((float)p->ref_r_adc-(float)r)/WP_RIGHT_ADC_PER_MM;
    p->error_mm=Clamp(0.5f*(dr-dl),-12.0f,12.0f);
    if((p->ticks%WP_PID_TICKS)!=0U)return 0U;
    raw_deriv=p->has_previous ?
        (p->error_mm-p->last_error_mm)/0.005f : 0.0f;
    /* Derivative on 200Hz filtered wall error; 1st order smoothing. */
    deriv=p->has_previous ?
        p->derivative_mm_s+0.25f*(raw_deriv-p->derivative_mm_s):0.0f;
    integ_next=Clamp(p->integral_mm_s+p->error_mm*0.005f,-12.0f,12.0f);
    corr=3.0f*p->error_mm + 0.35f*integ_next +
         0.015f*deriv + 1.5f*p->yaw_error_deg +
         0.08f*in->gyro_dps;
    /* Conservative anti-windup, no reverse or spin permitted. */
    if(Abs(corr)<(float)WP_STEER_CAP_PM)p->integral_mm_s=integ_next;
    corr=Clamp(corr,-(float)WP_STEER_CAP_PM,(float)WP_STEER_CAP_PM);
    p->last_error_mm=p->error_mm;
    p->derivative_mm_s=deriv;p->has_previous=1U;
    p->pwm_l=(int16_t)Clamp(WP_PWM_BASE_PM+corr,0.0f,WP_PWM_CAP_PM);
    p->pwm_r=(int16_t)Clamp(WP_PWM_BASE_PM-corr,0.0f,WP_PWM_CAP_PM);
    return 1U;
}
