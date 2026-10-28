/* 200Hz six-channel and motion instrumentation for four-cell Wall PID.
 * Every sample is captured inside TIM11 AFTER one coherent WP_Step.
 * BLE formatting and transfer happen ONLY after the motor is locked.
 * Protocol: MW,BEGIN ... MW,<21 values> ... MW,END fnv32=...
 * No arbitrary ADC boundaries and no changes to controller outputs. */
#if defined(WALL_PID_TEST_MODE)
#include "wall_pid_trace.h"
#include "bt_debug.h"
#include "motor.h"
#include <stdio.h>

#define WPT_PERIOD_MS 125U
#define WPT_CHUNK_BYTES 20U
#define WPT_FNV_OFFSET 2166136261UL
#define WPT_FNV_PRIME 16777619UL

typedef struct {
    uint32_t scan, enc_l, enc_r;
    int32_t yaw_cd;
    uint16_t ms, age_us, ir[6];
    int16_t rate_cd, err_norm_1000, speed_mm_s, target_mm_s;
    int16_t pwm_l, pwm_r;
    uint8_t valid, state;
} WPT_Record;
static WPT_Record data[WPT_RECORD_CAPACITY];
static volatile uint16_t captured, dropped;
static uint16_t cursor, offset, formatted_len;
static uint32_t last_tx_ms, checksum;
static uint8_t replay_state;
static char frame[210];

static uint16_t FormatRow(uint16_t index)
{
    const WPT_Record *r=&data[index];
    int n=snprintf(frame,sizeof(frame),
       "MW,%u,%u,%lu,%ld,%ld,%ld,%d,%u,%u,%u,%u,%u,%u,%u,%u,%d,%d,%d,%d,%d,%u\n",
       (unsigned)index,(unsigned)r->ms,
       (unsigned long)r->scan,
       (long)(int32_t)r->enc_l,(long)(int32_t)r->enc_r,
       (long)r->yaw_cd,(int)r->rate_cd,
       (unsigned)r->age_us,(unsigned)r->valid,
       (unsigned)r->ir[0],(unsigned)r->ir[1],
       (unsigned)r->ir[2],(unsigned)r->ir[3],
       (unsigned)r->ir[4],(unsigned)r->ir[5],
       (int)r->err_norm_1000,(int)r->speed_mm_s,
       (int)r->target_mm_s,(int)r->pwm_l,
       (int)r->pwm_r,(unsigned)r->state);
    if(n<=0 || n>=(int)sizeof(frame))return 0U;
    return (uint16_t)n;
}
static uint32_t Hash(uint32_t h,const char *bytes,uint16_t len)
{
    uint16_t i;
    for(i=0;i<len;i++){
        h^=(uint8_t)bytes[i];
        h*=WPT_FNV_PRIME;
    }
    return h;
}
void WPT_Reset(void)
{
    captured=0U;dropped=0U;cursor=0U;offset=0U;formatted_len=0U;
    replay_state=0U;checksum=WPT_FNV_OFFSET;
}
void WPT_Append(uint32_t tick,const WP_Input *input,
                const WP_Controller *control)
{
    uint16_t i;
    WPT_Record *r;
    if(!input || !control || tick==0U || (tick%WPT_PERIOD_TICKS)!=0U)return;
    if(captured>=WPT_RECORD_CAPACITY){if(dropped<65535U)dropped++;return;}
    r=&data[captured]; /* single TIM11 writer; increment count LAST */
    r->ms=(uint16_t)tick;
    r->scan=input->ir.scan_count;
    r->enc_l=(uint32_t)input->left_count;
    r->enc_r=(uint32_t)input->right_count;
    r->yaw_cd=(int32_t)(input->yaw_deg*100.0f);
    r->rate_cd=(int16_t)(input->gyro_dps*100.0f);
    r->age_us=(uint16_t)(input->ir.age_us>65535U?65535U:input->ir.age_us);
    r->valid=(uint8_t)(input->ir.status==IR_WALL_VALID);
    for(i=0U;i<6U;i++)r->ir[i]=input->ir.ema10_adc[i];
    r->err_norm_1000=(int16_t)(control->error_norm*1000.0f);
    r->speed_mm_s=(int16_t)control->speed_mm_s;
    r->target_mm_s=(int16_t)control->target_speed_mm_s;
    r->pwm_l=control->pwm_l;r->pwm_r=control->pwm_r;
    r->state=(uint8_t)control->state;
    captured++;
}
uint16_t WPT_Count(void){return captured;}
uint16_t WPT_Dropped(void){return dropped;}
void WPT_BeginReplay(void)
{
    if(replay_state!=0U || !Motor_OutputsLocked())return;
    cursor=0U;offset=0U;formatted_len=0U;
    checksum=WPT_FNV_OFFSET;
    replay_state=1U;
}
uint8_t WPT_ReplayComplete(void){return replay_state==4U;}
uint8_t WPT_ServiceReplay(uint32_t now_ms)
{
    uint16_t n;
    int printed;
    if(replay_state==0U || replay_state==4U)return WPT_ReplayComplete();
    if(!Motor_OutputsLocked())return 0U;
    if((uint32_t)(now_ms-last_tx_ms)<WPT_PERIOD_MS)return 0U;
    last_tx_ms=now_ms;
    if(replay_state==1U){
        printed=snprintf(frame,sizeof(frame),
            "MW,BEGIN,n=%u,hz=200,locked=1,motor_run=1,dropped=%u,format=v1\n",
            (unsigned)captured,(unsigned)dropped);
        if(printed>0 && printed<(int)sizeof(frame) &&
           BT_TrySendData((const uint8_t*)frame,(uint16_t)printed)){
            replay_state=2U;formatted_len=offset=0U;
        }
        return 0U;
    }
    if(replay_state==2U && cursor>=captured)replay_state=3U;
    if(replay_state==3U){
        printed=snprintf(frame,sizeof(frame),
           "MW,END,n=%u,fnv32=%08lx,dropped=%u,locked=%u\n",
           (unsigned)captured,(unsigned long)checksum,
           (unsigned)dropped,(unsigned)Motor_OutputsLocked());
        if(printed>0 && printed<(int)sizeof(frame) &&
           BT_TrySendData((const uint8_t*)frame,(uint16_t)printed)){
            replay_state=4U;
        }
        return WPT_ReplayComplete();
    }
    if(!formatted_len){
        formatted_len=FormatRow(cursor);
        if(!formatted_len){replay_state=3U;return 0U;}
        checksum=Hash(checksum,frame,formatted_len);
    }
    n=formatted_len-offset;
    if(n>WPT_CHUNK_BYTES)n=WPT_CHUNK_BYTES;
    if(BT_TrySendData((const uint8_t*)frame+offset,n)){
        offset=(uint16_t)(offset+n);
        if(offset==formatted_len){
            cursor++;formatted_len=offset=0U;
        }
    }
    return 0U;
}
#endif
