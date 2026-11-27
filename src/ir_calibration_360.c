#if defined(MODE_MENU_ONLY)
#include "ir_calibration_360.h"
#include "board.h"
#include "control.h"
#include "encoder.h"
#include "ir_sensor.h"
#include "motor.h"
#include "system_timer.h"
#include "bt_debug.h"
#include "flash_storage.h"
#include <string.h>
#include <math.h>

/* Existing pivot guarantees a 100ms braking interval. The additional dwell
 * is deliberately nonblocking and waits for a stable gyro measurement. */
#define IRC_DWELL_MIN_MS 120U
#define IRC_READY_GYRO_DPS 2.0f
#define IRC_STABLE_MS 50U
#define IRC_SESSION_TIMEOUT_MS 40000UL
#define IRC_SAMPLE_PERIOD_MS 20U
#define IRC_MIN_SAMPLES 100UL
#define IRC_MIN_SPAN_ADC 40U
/* Approximate 10th/90th percentiles from 128-count bins.
 * Unlike min/max, isolated ADC spikes cannot dominate the thresholds. */
static uint16_t IRC_Quantile(const uint16_t hist[32],uint32_t count,uint32_t numerator) {
    uint32_t goal=(count*numerator+99U)/100U, seen=0U;
    uint8_t k;
    for(k=0;k<32U;k++){
        seen+=hist[k];
        if(seen>=goal)return (uint16_t)((uint32_t)k*128U+64U);
    }
    return 4095U;
}
#define IRC_SENSOR_MAX_AGE_US 5000UL

static float IRC_YawDiff(float now,float old) {
    float d=now-old;
    while(d>180.0f)d-=360.0f;
    while(d< -180.0f)d+=360.0f;
    return d;
}
static const char * IRC_StateName(IRC_State s) {
    switch(s) {
        case IRC_ARMED: return "ARMED"; case IRC_PIVOT: return "PIVOT";
        case IRC_DWELL: return "DWELL"; case IRC_REVIEW: return "REVIEW";
        case IRC_SAVED: return "SAVED"; case IRC_FAULT: return "FAULT";
        default: return "IDLE";
    }
}
void IRC_Init(IRC_Context *c) { if(c) memset(c,0,sizeof(*c)); }
uint8_t IRC_IsMotionActive(const IRC_Context *c) {
    return c && (c->state==IRC_ARMED || c->state==IRC_PIVOT || c->state==IRC_DWELL);
}
void IRC_Arm(IRC_Context *c,uint32_t now,uint8_t physical_start) {
    uint8_t i;
    if(!c)return;
    if(IRC_IsMotionActive(c)) return;
    IRC_Init(c);
    c->state=IRC_ARMED; c->started_ms=now;
    c->physical_start=physical_start?1U:0U;
    for(i=0;i<6;i++)c->low[i]=4095U;
    BT_SendString("CAL,START,mode=1,segments=4,dwell_min_ms=120,save=MANUAL\r\n");
}
void IRC_Abort(IRC_Context *c,const char *why) {
    if(!c || !IRC_IsMotionActive(c))return;
    if(EncoderOneBox_IsRunning())EncoderOneBox_Abort();
    c->state=IRC_FAULT;
    strncpy(c->fault,why?why:"ABORT",sizeof(c->fault)-1U);
    c->fault[sizeof(c->fault)-1U]=0;
    BT_Printf("CAL,FAULT,reason=%s\r\n",c->fault);
}
static uint8_t IRC_GyroReady(MPU6050_Handle_t *mpu) {
    MPU6050_Snapshot_t s;
    return mpu && mpu->calibrated && MPU6050_GetSnapshot(mpu,&s) && s.valid &&
           (uint32_t)(micros()-s.completed_us)<=3000U &&
           s.gyro_z_dps>=-IRC_READY_GYRO_DPS &&
           s.gyro_z_dps<=IRC_READY_GYRO_DPS;
}
static void IRC_Sample(IRC_Context *c,uint32_t now) {
    IR_Data_t v;
    uint32_t age;
    uint16_t a[6];
    uint8_t i;
    if((uint32_t)(now-c->last_sample_ms)<IRC_SAMPLE_PERIOD_MS)return;
    c->last_sample_ms=now;
    Control_GetIRSnapshot(&v);
    age=v.scan_count?(uint32_t)(micros()-v.completed_us):0xFFFFFFFFUL;
    if(age>IRC_SENSOR_MAX_AGE_US) { IRC_Abort(c,"IR_STALE");return; }
    a[0]=v.value[IR_L0];a[1]=v.value[IR_L45];a[2]=v.value[IR_L90];
    a[3]=v.value[IR_R0];a[4]=v.value[IR_R45];a[5]=v.value[IR_R90];
    for(i=0;i<6;i++){
        if(a[i]<c->low[i])c->low[i]=a[i];
        if(a[i]>c->high[i])c->high[i]=a[i];
        if(c->histogram[i][a[i]/128U]<65535U)++c->histogram[i][a[i]/128U];
    }
    if(c->samples==0U){c->center_l0=a[0];c->center_r0=a[3];}
    ++c->samples;
    BT_Printf("CAL_IR,ms=%lu,L0=%u,L45=%u,L90=%u,R0=%u,R45=%u,R90=%u\r\n",
        (unsigned long)(now-c->started_ms),(unsigned)a[0],(unsigned)a[1],
        (unsigned)a[2],(unsigned)a[3],(unsigned)a[4],(unsigned)a[5]);
}
void IRC_OnPivotResult(IRC_Context *c,const EncoderOneBoxResult *r,uint32_t now) {
    if(!c || !r || c->state!=IRC_PIVOT)return;
    if(!r->pivot_target || r->reason!=ONEBOX_REASON_DONE || !r->motor_locked){
        IRC_Abort(c,EncoderOneBox_Reason(r->reason));return;
    }
    BT_Printf("CAL,PIVOT_DONE,segment=%u,ms=%lu\r\n",
              (unsigned)c->segment,(unsigned long)(now-c->started_ms));
    c->state=IRC_DWELL;c->dwell_started_ms=now;c->last_ready_ms=0U;
}
void IRC_Update(IRC_Context *c,MPU6050_Handle_t *mpu,uint32_t now) {
    uint8_t i;
    if(!IRC_IsMotionActive(c))return;
    if((uint32_t)(now-c->started_ms)>IRC_SESSION_TIMEOUT_MS){IRC_Abort(c,"TIMEOUT");return;}
    if(c->state!=IRC_ARMED && Board_ButtonPressed()){
        IRC_Abort(c,"KEY_STOP");return;
    }
    /* Use the existing lease guard, but service it locally for autonomous
     * physical-KEY calibration: no mandatory PC heartbeat while turning. */
    if(c->state==IRC_PIVOT && c->physical_start)EncoderOneBox_HostPing();
    /* While key is held during initial long-select, do not release motors. */
    if(c->state==IRC_ARMED && Board_ButtonPressed())return;
    IRC_Sample(c,now);
    if(!IRC_IsMotionActive(c))return;
    if(c->state==IRC_PIVOT)return;
    if(!IRC_GyroReady(mpu)){c->last_ready_ms=0U;return;}
    if(!c->last_ready_ms)c->last_ready_ms=now;
    if((uint32_t)(now-c->last_ready_ms)<IRC_STABLE_MS)return;
    if(c->state==IRC_DWELL &&
       (uint32_t)(now-c->dwell_started_ms)<IRC_DWELL_MIN_MS)return;
    if(c->state==IRC_DWELL){
        MPU6050_Snapshot_t s;
        if(!MPU6050_GetSnapshot(mpu,&s) || !s.valid){
            IRC_Abort(c,"IMU_STALE");return;
        }
        if(fabsf(IRC_YawDiff(s.yaw_deg,c->segment_origin_yaw)-90.0f)>8.0f){
            IRC_Abort(c,"PIVOT_ANGLE");return;
        }
        if(c->segment>=4U &&
           fabsf(IRC_YawDiff(s.yaw_deg,c->session_origin_yaw))>8.0f){
            IRC_Abort(c,"ROUND_TRIP");return;
        }
    }
    if(c->state==IRC_DWELL && c->segment>=4U){
        if(c->samples<IRC_MIN_SAMPLES){IRC_Abort(c,"SAMPLES");return;}
        c->valid=1U;
        for(i=0;i<6U;i++){
            uint16_t p10=IRC_Quantile(c->histogram[i],c->samples,10U);
            uint16_t p90=IRC_Quantile(c->histogram[i],c->samples,90U);
            uint16_t span=p90>p10?(uint16_t)(p90-p10):0U;
            if(span<IRC_MIN_SPAN_ADC)c->valid=0U;
            c->wall_off[i]=(uint16_t)(p10+(uint32_t)span*30U/100U);
            c->wall_on[i]=(uint16_t)(p10+(uint32_t)span*70U/100U);
        }
        c->state=IRC_REVIEW;
        BT_Printf("CAL,REVIEW,samples=%lu,quality=%s,save=V\r\n",
              (unsigned long)c->samples,c->valid?"CANDIDATE_ONLY":"INSUFFICIENT");
        for(i=0;i<6U;i++)
            BT_Printf("CAL,THRESH,index=%u,lo=%u,hi=%u,off=%u,on=%u\r\n",
               (unsigned)i,(unsigned)c->low[i],(unsigned)c->high[i],
               (unsigned)c->wall_off[i],(unsigned)c->wall_on[i]);
        /* Preserve the existing quality gate. Save the completed candidate
         * automatically only if valid and motor locked. Never auto-apply
         * an unlabelled P10/P90 threshold to wall following. */
        if(c->valid && !IRC_Save(c))
            BT_SendString("CAL,AUTO_SAVE_FAILED,review=AVAILABLE\r\n");
        return;
    }
    if(c->state==IRC_ARMED || c->state==IRC_DWELL) {
        /* On completion the previous pivot has emitted its result and the
         * driver has returned to locked-idle before the next Start. */
        if(!Motor_OutputsLocked() || EncoderOneBox_IsRunning())return;
        {
            MPU6050_Snapshot_t s;
            if(!MPU6050_GetSnapshot(mpu,&s) || !s.valid){
                IRC_Abort(c,"IMU_STALE");return;
            }
            c->segment_origin_yaw=s.yaw_deg;
            if(!c->have_origin) {
                c->session_origin_yaw=s.yaw_deg;
                c->have_origin=1U;
            }
        }
        if(EncoderOneBox_StartCalibrationPivot(mpu,90)) {
            ++c->segment;c->state=IRC_PIVOT;c->last_ready_ms=0U;
            BT_Printf("CAL,PIVOT_START,segment=%u,target=90\r\n",(unsigned)c->segment);
        }else{
            /* If the prior driver is settling, retry without a busy loop. */
            c->last_ready_ms=0U;
        }
    }
}
uint8_t IRC_Save(IRC_Context *c) {
    IR_Simple_Calib_t data;
    Flash_Status_t status;
    if(!c || c->state!=IRC_REVIEW || !c->valid ||
       EncoderOneBox_IsRunning() || !Motor_OutputsLocked())return 0U;
    memset(&data,0,sizeof(data));
    data.wall_on_l0=c->wall_on[0];data.wall_off_l0=c->wall_off[0];
    data.wall_on_l45=c->wall_on[1];data.wall_off_l45=c->wall_off[1];
    data.wall_on_l90=c->wall_on[2];data.wall_off_l90=c->wall_off[2];
    data.wall_on_r0=c->wall_on[3];data.wall_off_r0=c->wall_off[3];
    data.wall_on_r45=c->wall_on[4];data.wall_off_r45=c->wall_off[4];
    data.wall_on_r90=c->wall_on[5];data.wall_off_r90=c->wall_off[5];
    data.center_l0=c->center_l0;data.center_r0=c->center_r0;
    /* No geometry-labelled no-wall evidence: do not claim open calibration. */
    data.has_open_calib=0U;
    data.wall_threshold_left=data.wall_on_l90;
    data.wall_threshold_right=data.wall_on_r90;
    data.wall_threshold_front_left=data.wall_on_l0;
    data.wall_threshold_front_right=data.wall_on_r0;
    data.wall_threshold_l45=data.wall_on_l45;
    data.wall_threshold_r45=data.wall_on_r45;
    data.gain_l0=data.gain_r0=1.0f;
    data.gain_l45=data.gain_r45=1.0f;
    data.gain_l90=data.gain_r90=1.0f;
    data.l0_min=c->low[0];data.l0_max=c->high[0];
    data.r0_min=c->low[3];data.r0_max=c->high[3];
    data.l45_min=c->low[1];data.l45_max=c->high[1];
    data.r45_min=c->low[4];data.r45_max=c->high[4];
    data.calib_mode=1U;
    /* Candidate is not automatically applied to wall following.
     * Existing Flash storage refuses writes while Control runs. */
    IR_Stop();Control_Stop();Motor_LockOutputs();
    status=Flash_SaveIRCalibChecked(&data);
    Control_Start();
    IR_StartScan(); /* restart acquisition after temporary Flash-safe stop */
    if(status==FLASH_OK) {c->state=IRC_SAVED;BT_SendString("CAL,SAVED,flash=OK,auto_apply=0\r\n");return 1U;}
    BT_Printf("CAL,SAVE_REJECT,flash_status=%u\r\n",(unsigned)status);
    return 0U;
}
#endif
