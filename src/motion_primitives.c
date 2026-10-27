#include "motion_primitives.h"
#include <math.h>
#include <string.h>

/* First reusable high-speed pivot engine. Tuned from SINGLEBRAKE v15
 * diagnostic; does not mutate the existing diagnostic firmware.
 * Rate loop = 200Hz on top of the normal 1kHz sensor/motor scheduler.
 *
 * The 180deg early brake release (330dps) and approach (120dps) have
 * passed synthetic tests ONLY. Both right pivots are mirrored
 * implementations and require directional on-robot qualification.
 */
#define MP_STEP_S                0.005f
#define MP_ANGLE_BAND_DEG        2.0f
#define MP_STOP_RATE_DPS        12.0f
#define MP_SETTLE_S             0.150f
#define MP_TIMEOUT_90_S         4.0f
#define MP_TIMEOUT_180_S        6.5f
#define MP_RATE_ABORT_DPS     875.0f
#define MP_ACCEL_DPS2        5500.0f
#define MP_BRAKE_DEG_S2      5500.0f
#define MP_BRAKE_RESERVE_DEG    8.0f
#define MP_RATE_KP              2.10f
#define MP_RATE_FF              1.20f
#define MP_ACCEL_FF             0.04f
#define MP_PWM_RISE_STEP      105.0f
#define MP_BRAKE_CAP_PM       300.0f
#define MP_BRAKE_PWM_STEP     160.0f
#define MP_BRAKE_MAX_S          0.220f
#define MP_COAST_90_RATE       38.0f
#define MP_COAST_180_RATE     120.0f
#define MP_COAST_90_MIN_S       0.050f
#define MP_COAST_180_MIN_S      0.030f
#define MP_BRAKE_90_END_RATE  125.0f
#define MP_BRAKE_180_END_RATE 330.0f
#define MP_FINAL_PWM_CAP      155.0f
#define MP_FINAL_STEP          12.0f
#define MP_FINAL_STALL_S        0.520f

static float clampf(float x,float lo,float hi)
{
    return x<lo?lo:(x>hi?hi:x);
}

static float angle_diff(float a,float b)
{
    float d=a-b;
    while(d>180.0f)d-=360.0f;
    while(d< -180.0f)d+=360.0f;
    return d;
}

static void fail(MotionPrimitiveController *c,MotionPrimitiveResult why)
{
    c->status=MOTION_PRIMITIVE_FAULT;
    c->result=why;
    c->pwm_requested=0.0f;
    c->ref_rate_dps=0.0f;
}

static void fill_output(const MotionPrimitiveController *c,
                        MotionPrimitiveOutput *out)
{
    float cmd=c->status==MOTION_PRIMITIVE_RUNNING ?
              clampf(c->pwm_requested,-1000.0f,1000.0f):0.0f;
    int16_t signed_pwm=(int16_t)cmd*(int16_t)c->direction;
    out->left_pm=(int16_t)-signed_pwm;
    out->right_pm=signed_pwm;
    out->immediate_neutral=(c->status!=MOTION_PRIMITIVE_RUNNING ||
        c->phase==MOTION_PIVOT_COAST ||
        c->phase==MOTION_PIVOT_SETTLE ||
        (c->phase==MOTION_PIVOT_APPROACH && c->error_deg<=MP_ANGLE_BAND_DEG))?1U:0U;
    out->status=c->status;
    out->result=c->result;
    out->phase=c->phase;
    out->angle_error_deg=c->error_deg;
    out->ref_rate_dps=c->ref_rate_dps;
}

void MotionPrimitive_Init(MotionPrimitiveController *ctx)
{
    if(!ctx)return;
    memset(ctx,0,sizeof(*ctx));
    ctx->status=MOTION_PRIMITIVE_IDLE;
    ctx->direction=MOTION_DIR_LEFT;
    ctx->result=MOTION_PRIMITIVE_OK;
}

MotionPrimitiveResult MotionPrimitive_Start(MotionPrimitiveController *ctx,
                                            MotionPrimitiveKind kind,
                                            MotionPrimitiveDirection dir,
                                            float current_yaw_deg)
{
    if(!ctx || !isfinite(current_yaw_deg) ||
       (dir!=MOTION_DIR_LEFT && dir!=MOTION_DIR_RIGHT))
        return MOTION_PRIMITIVE_BAD_ARGUMENT;
    if(ctx->status==MOTION_PRIMITIVE_RUNNING)
        return MOTION_PRIMITIVE_BUSY;
    if(kind!=MOTION_PRIMITIVE_PIVOT_90 &&
       kind!=MOTION_PRIMITIVE_PIVOT_180)
        return MOTION_PRIMITIVE_UNSUPPORTED;
    MotionPrimitive_Init(ctx);
    ctx->kind=kind;
    ctx->direction=dir;
    ctx->target_deg=kind==MOTION_PRIMITIVE_PIVOT_180?180.0f:90.0f;
    ctx->error_deg=ctx->target_deg;
    ctx->last_yaw_deg=current_yaw_deg;
    ctx->status=MOTION_PRIMITIVE_RUNNING;
    return MOTION_PRIMITIVE_OK;
}

void MotionPrimitive_Abort(MotionPrimitiveController *ctx,
                           MotionPrimitiveResult reason)
{
    if(ctx && ctx->status==MOTION_PRIMITIVE_RUNNING)
        fail(ctx,reason==MOTION_PRIMITIVE_OK ?
                  MOTION_PRIMITIVE_STOP_REQUEST:reason);
}

void MotionPrimitive_Update(MotionPrimitiveController *c,
                            float dt, float yaw_deg, float gyro_z_dps,
                            MotionPrimitiveOutput *out)
{
    float e,w,prior,u=0.0f;
    if(!out)return;
    memset(out,0,sizeof(*out));
    if(!c){out->status=MOTION_PRIMITIVE_FAULT;
           out->result=MOTION_PRIMITIVE_BAD_ARGUMENT;
           out->immediate_neutral=1U;return;}
    if(c->status!=MOTION_PRIMITIVE_RUNNING){fill_output(c,out);return;}
    if(!isfinite(dt) || dt<=0.0f || dt>0.020f ||
       !isfinite(yaw_deg)||!isfinite(gyro_z_dps)){
        fail(c,MOTION_PRIMITIVE_SENSOR_FAULT);
        fill_output(c,out);return;
    }
    /* MPU yaw may wrap at +-180deg. Integrate coherent successive
     * deltas; do not normalize total 180deg travel back to -180deg. */
    c->traveled_deg+=angle_diff(yaw_deg,c->last_yaw_deg)*(float)c->direction;
    c->last_yaw_deg=yaw_deg;
    c->gyro_dps=gyro_z_dps*(float)c->direction;
    c->error_deg=c->target_deg-c->traveled_deg;
    c->elapsed_s+=dt;
    e=c->error_deg;w=c->gyro_dps;
    if(c->elapsed_s>(c->target_deg==90.0f?MP_TIMEOUT_90_S:MP_TIMEOUT_180_S)){
        fail(c,MOTION_PRIMITIVE_TIMEOUT);
    }else if(e< -MP_ANGLE_BAND_DEG){
        fail(c,MOTION_PRIMITIVE_OVERSHOOT);
    }else if(c->elapsed_s>0.070f && w< -45.0f){
        fail(c,MOTION_PRIMITIVE_WRONG_DIRECTION);
    }else if(c->traveled_deg< -3.0f){
        fail(c,MOTION_PRIMITIVE_WRONG_DIRECTION);
    }else if(fabsf(w)>MP_RATE_ABORT_DPS){
        fail(c,MOTION_PRIMITIVE_RATE_LIMIT);
    }
    if(c->status!=MOTION_PRIMITIVE_RUNNING){fill_output(c,out);return;}

    c->control_accum_s+=dt;
    if(c->control_accum_s<MP_STEP_S-0.000001f){
        fill_output(c,out);return;
    }
    /* Tick phase logic at 200Hz; no long ISR blocking or I/O. */
    c->control_accum_s=0.0f;
    prior=c->pwm_requested;
    if(c->phase==MOTION_PIVOT_SETTLE){
        c->pwm_requested=0.0f;c->ref_rate_dps=0.0f;
        if(fabsf(e)>MP_ANGLE_BAND_DEG)
            fail(c,MOTION_PRIMITIVE_SETTLE_DRIFT);
        else{
            c->settle_s+=MP_STEP_S;
            if(c->settle_s>=MP_SETTLE_S)
                c->status=MOTION_PRIMITIVE_DONE;
        }
        fill_output(c,out);return;
    }
    if(fabsf(e)<=MP_ANGLE_BAND_DEG && fabsf(w)<=MP_STOP_RATE_DPS){
        c->phase=MOTION_PIVOT_SETTLE;
        c->pwm_requested=0.0f;c->ref_rate_dps=0.0f;
        c->settle_s=MP_STEP_S;fill_output(c,out);return;
    }
    if(c->phase==MOTION_PIVOT_ACCEL){
        float v=w>0.0f?w:0.0f;
        float lead=c->target_deg==180.0f?0.015f:0.010f;
        float braking_angle=v*v/(2.0f*MP_BRAKE_DEG_S2)+
                            lead*v+MP_BRAKE_RESERVE_DEG;
        if(v>180.0f && e<=braking_angle){
            c->phase=MOTION_PIVOT_BRAKE;
            c->phase_elapsed_s=0.0f;
            c->ref_rate_dps=0.0f;
        }
    }
    if(c->phase==MOTION_PIVOT_ACCEL){
        float vmax=c->target_deg==180.0f?780.0f:650.0f;
        float cap=c->target_deg==180.0f?1000.0f:900.0f;
        float ref=clampf(c->ref_rate_dps+MP_ACCEL_DPS2*MP_STEP_S,0.0f,vmax);
        float pid=MP_RATE_KP*(ref-w);
        float ff=MP_RATE_FF*ref+MP_ACCEL_FF*MP_ACCEL_DPS2;
        c->ref_rate_dps=ref;
        u=clampf(pid+ff,0.0f,cap);
        u=clampf(u,prior-MP_PWM_RISE_STEP,prior+MP_PWM_RISE_STEP);
        u=clampf(u,0.0f,cap);
    }else if(c->phase==MOTION_PIVOT_BRAKE){
        c->phase_elapsed_s+=MP_STEP_S;
        /* v15: the 180deg negative torque must be released at a
         * positive gyro rate. The 90deg thresholds are unchanged. */
        if((c->target_deg==180.0f && w<=MP_BRAKE_180_END_RATE) ||
           (c->target_deg==90.0f &&
              ((w<=MP_BRAKE_90_END_RATE && e>6.0f) ||
               (w<=85.0f && e<=6.0f))) ||
           c->phase_elapsed_s>=MP_BRAKE_MAX_S){
            c->phase=MOTION_PIVOT_COAST;
            c->phase_elapsed_s=0.0f;
            c->ref_rate_dps=0.0f;
            c->pwm_requested=0.0f;
            fill_output(c,out);return;
        }
        u=-clampf(100.0f+0.75f*(w-125.0f),0.0f,MP_BRAKE_CAP_PM);
        u=clampf(u,prior-MP_BRAKE_PWM_STEP,prior+MP_BRAKE_PWM_STEP);
        c->ref_rate_dps=0.0f;
    }else if(c->phase==MOTION_PIVOT_COAST){
        c->phase_elapsed_s+=MP_STEP_S;
        c->ref_rate_dps=0.0f;
        u=0.0f;
        if(e>MP_ANGLE_BAND_DEG &&
           (c->target_deg==180.0f ?
            (w<=MP_COAST_180_RATE && c->phase_elapsed_s>=MP_COAST_180_MIN_S):
            (w<=MP_COAST_90_RATE && c->phase_elapsed_s>=MP_COAST_90_MIN_S))){
            c->phase=MOTION_PIVOT_APPROACH;
            c->phase_elapsed_s=0.0f;
        }
    }else if(c->phase==MOTION_PIVOT_APPROACH){
        float p=prior>0.0f?prior:0.0f;
        float desired_rate=clampf(6.0f*e,15.0f,90.0f);
        float desired=clampf(105.0f+0.8f*(desired_rate-w),
                             0.0f,MP_FINAL_PWM_CAP);
        if(e<4.0f && w>desired_rate)desired=0.0f;
        u=clampf(desired,p-MP_FINAL_STEP,p+MP_FINAL_STEP);
        u=clampf(u,0.0f,MP_FINAL_PWM_CAP);
        c->ref_rate_dps=desired_rate;
        if(fabsf(w)<3.0f && e>MP_ANGLE_BAND_DEG){
            c->stall_s+=MP_STEP_S;
            if(c->stall_s>=MP_FINAL_STALL_S)
                fail(c,MOTION_PRIMITIVE_STALLED);
        }else c->stall_s=0.0f;
    }
    /* When close to target and still moving, stay neutral and
     * let the gyro rate fall. Do not pulse drive while in-band. */
    if(e<=MP_ANGLE_BAND_DEG && w>MP_STOP_RATE_DPS &&
       c->phase>=MOTION_PIVOT_COAST){
        u=0.0f;
        c->ref_rate_dps=0.0f;
    }
    c->pwm_requested=clampf(u,-MP_BRAKE_CAP_PM,1000.0f);
    fill_output(c,out);
}
