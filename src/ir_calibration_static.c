#if defined(MODE_MENU_ONLY)
#include "ir_calibration_static.h"
#include "board.h"
#include "control.h"
#include "ir_sensor.h"
#include "motor.h"
#include "encoder_onebox.h"
#include "system_timer.h"
#include "bt_debug.h"
#include "flash_storage.h"
#include <string.h>

/* Ported from giomuadongbacv2, deliberately opt-in and MOTOR LOCKED.
 * Three independent labelled placements; 200 fresh readings each.
 * A successful measurement is REVIEW-only until explicit 'V' command. */
#define SC_N 200U
#define SC_SETTLE_MS 250U
#define SC_INTERVAL_MS 10U
#define SC_TIMEOUT_MS 5000U
#define SC_MIN_SPAN 50U
static uint8_t active,stage,busy,review;
static uint16_t count;
static uint32_t started,last_ms,last_scan;
static uint16_t samples[IR_SENSOR_COUNT][SC_N];
static uint16_t ref[3][IR_SENSOR_COUNT],lo[3][IR_SENSOR_COUNT],hi[3][IR_SENSOR_COUNT];
static IR_Simple_Calib_t candidate;
static const char *const names[IR_SENSOR_COUNT]={"L90","L45","L0","R0","R45","R90"};
static const char *const places[3]={"FRONT_AND_BOTH_SIDES","BOTH_SIDES_NO_FRONT","OPEN_SPACE"};
static uint16_t maximum(uint16_t a,uint16_t b){return a>b?a:b;}
static uint16_t minimum(uint16_t a,uint16_t b){return a<b?a:b;}
static void sort(uint16_t *v){
    uint16_t i,j,k;
    for(i=1U;i<SC_N;++i){
        k=v[i];j=i;
        while(j && v[j-1U]>k){v[j]=v[j-1U];--j;}
        v[j]=k;
    }
}
void IRStatic_Init(void){active=stage=busy=review=0U;count=0U;}
uint8_t IRStatic_Active(void){return active;}
void IRStatic_Status(void){
    if(!active)return;
    if(review)BT_SendString("CAL_STATIC,REVIEW,quality=VALIDATED_PLACEMENTS,save=V\r\n");
    else BT_Printf("CAL_STATIC,WAIT,stage=%u,placement=%s,action=G_OR_KEY_SHORT\r\n",
        (unsigned)(stage+1U),places[stage]);
}
void IRStatic_Start(void){
    if(!Motor_OutputsLocked() || EncoderOneBox_IsRunning()){
        BT_SendString("CAL_STATIC,REJECT,reason=MOTOR_UNSAFE\r\n");return;
    }
    IRStatic_Init();active=1U;memset(&candidate,0,sizeof(candidate));
    BT_SendString("CAL_STATIC,ENTER,method=THREE_PLACEMENT,motor=LOCKED\r\n");
    IRStatic_Status();
}
void IRStatic_Capture(uint32_t now){
    if(!active || busy || review)return;
    if(!Motor_OutputsLocked() || EncoderOneBox_IsRunning()){
        BT_SendString("CAL_STATIC,REJECT,reason=MOTOR_UNSAFE\r\n");return;
    }
    count=0U;last_scan=0U;started=last_ms=now;busy=1U;
    BT_Printf("CAL_STATIC,START,stage=%u,samples=%u\r\n",
        (unsigned)(stage+1U),(unsigned)SC_N);
}
void IRStatic_Cancel(void){
    if(!active)return;
    if(busy){busy=0U;count=0U;BT_SendString("CAL_STATIC,ABORT,reason=USER_STOP\r\n");}
    IRStatic_Status();
}
void IRStatic_Exit(void){
    if(!active)return;
    active=busy=review=0U;
    BT_SendString("CAL_STATIC,EXIT\r\n");
}
static void validate(void){
    uint16_t base[IR_SENSOR_COUNT],wall[IR_SENSOR_COUNT],on[IR_SENSOR_COUNT],off[IR_SENSOR_COUNT];
    uint8_t i,good=1U;
    memset(&candidate,0,sizeof(candidate));
    for(i=0U;i<IR_SENSOR_COUNT;++i){
        uint8_t side=(i==IR_L0 || i==IR_R0);
        uint16_t neg_hi,pos_lo;
        base[i]=side?ref[2][i]:maximum(ref[1][i],ref[2][i]);
        wall[i]=side?minimum(ref[0][i],ref[1][i]):ref[0][i];
        neg_hi=side?hi[2][i]:maximum(hi[1][i],hi[2][i]);
        pos_lo=side?minimum(lo[0][i],lo[1][i]):lo[0][i];
        if(wall[i]<=base[i] || wall[i]-base[i]<SC_MIN_SPAN){
            good=0U;on[i]=off[i]=0U;
        }else{
            uint32_t span=(uint32_t)wall[i]-base[i];
            on[i]=(uint16_t)(base[i]+6U*span/10U);
            off[i]=(uint16_t)(base[i]+3U*span/10U);
            if(neg_hi>=off[i] || pos_lo<=on[i] || wall[i]>=4090U)good=0U;
        }
        BT_Printf("CAL_STATIC,RESULT,%s,A=%u,B=%u,ON=%u,OFF=%u,P95_OPEN=%u,P05_WALL=%u\r\n",
            names[i],(unsigned)base[i],(unsigned)wall[i],
            (unsigned)on[i],(unsigned)off[i],(unsigned)neg_hi,(unsigned)pos_lo);
        switch(i){
#define SET_SENSOR(id,n) case id: candidate.open_##n=base[i];candidate.wall_on_##n=on[i];candidate.wall_off_##n=off[i];break
            SET_SENSOR(IR_L90,l90);
            SET_SENSOR(IR_L45,l45);
            SET_SENSOR(IR_L0,l0);
            SET_SENSOR(IR_R0,r0);
            SET_SENSOR(IR_R45,r45);
            SET_SENSOR(IR_R90,r90);
#undef SET_SENSOR
            default:break;
        }
    }
    if(!good){
        stage=0U;review=0U;
        BT_SendString("CAL_STATIC,REJECT,reason=LOW_SPAN_OR_OVERLAP_OR_SATURATION,repeat=ALL\r\n");
        IRStatic_Status();return;
    }
    candidate.wall_threshold_left=on[IR_L90];
    candidate.wall_threshold_right=on[IR_R90];
    candidate.wall_threshold_front_left=on[IR_L0];
    candidate.wall_threshold_front_right=on[IR_R0];
    candidate.wall_threshold_l45=on[IR_L45];
    candidate.wall_threshold_r45=on[IR_R45];
    candidate.center_l90=ref[0][IR_L90];candidate.center_r90=ref[0][IR_R90];
    candidate.front_fsum_target=(uint32_t)candidate.center_l90+candidate.center_r90;
    candidate.center_l0=ref[1][IR_L0];candidate.center_r0=ref[1][IR_R0];
    candidate.gain_l0=candidate.gain_l45=candidate.gain_l90=1.0f;
    if(!candidate.center_r0 || !ref[0][IR_R45] || !candidate.center_r90){
        stage=0U;BT_SendString("CAL_STATIC,REJECT,reason=ZERO_REFERENCE\r\n");
        IRStatic_Status();return;
    }
    candidate.gain_r0=(float)candidate.center_l0/candidate.center_r0;
    candidate.gain_r45=(float)ref[0][IR_L45]/ref[0][IR_R45];
    candidate.gain_r90=(float)candidate.center_l90/candidate.center_r90;
    candidate.front_diff_zero=0;
    candidate.has_front_diff=1U;
    candidate.calib_mode=1U;candidate.has_open_calib=1U;
    review=1U;
    IRStatic_Status();
}
void IRStatic_Poll(uint32_t now){
    IR_Data_t sample;
    uint8_t i;
    if(!active || !busy)return;
    if(!Motor_OutputsLocked() || EncoderOneBox_IsRunning() ||
       (uint32_t)(now-started)>=SC_TIMEOUT_MS){
        busy=0U;count=0U;
        BT_SendString("CAL_STATIC,REJECT,reason=TIMEOUT_OR_MOTOR_STATE\r\n");
        IRStatic_Status();return;
    }
    if((uint32_t)(now-started)<SC_SETTLE_MS ||
       (uint32_t)(now-last_ms)<SC_INTERVAL_MS)return;
    Control_GetIRSnapshot(&sample);
    if(!sample.scan_count || sample.scan_count==last_scan ||
       (uint32_t)(micros()-sample.completed_us)>3000U)return;
    last_scan=sample.scan_count;last_ms=now;
    for(i=0U;i<IR_SENSOR_COUNT;++i)samples[i][count]=sample.value[i];
    if(++count<SC_N)return;
    busy=0U;
    for(i=0U;i<IR_SENSOR_COUNT;++i){
        sort(samples[i]);
        ref[stage][i]=(uint16_t)(((uint32_t)samples[i][99]+samples[i][100])/2U);
        lo[stage][i]=samples[i][10];
        hi[stage][i]=samples[i][189];
    }
    BT_Printf("CAL_STATIC,STAGE_OK,stage=%u,L90=%u,L45=%u,L0=%u,R0=%u,R45=%u,R90=%u\r\n",
        (unsigned)(stage+1U),(unsigned)ref[stage][IR_L90],
        (unsigned)ref[stage][IR_L45],(unsigned)ref[stage][IR_L0],
        (unsigned)ref[stage][IR_R0],(unsigned)ref[stage][IR_R45],
        (unsigned)ref[stage][IR_R90]);
    if(++stage==3U)validate();
    else IRStatic_Status();
}
uint8_t IRStatic_Save(void){
    Flash_Status_t status;
    if(!active || !review || busy || !Motor_OutputsLocked() ||
       EncoderOneBox_IsRunning())return 0U;
    /* No writes while IR sampling and control are live. */
    Motor_LockOutputs();Control_Stop();IR_Stop();
    status=Flash_SaveIRCalibChecked(&candidate);
    Control_Start();IR_StartScan();
    if(status==FLASH_OK){
        review=0U;active=0U;
        BT_SendString("CAL_STATIC,SAVED,flash=OK\r\n");
        return 1U;
    }
    BT_Printf("CAL_STATIC,SAVE_REJECT,flash_status=%u\r\n",(unsigned)status);
    return 0U;
}
#endif
