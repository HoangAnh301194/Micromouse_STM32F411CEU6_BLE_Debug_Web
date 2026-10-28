/* Opt-in MOTOR-LOCKED 6x IR + odometry + MPU capture for map/edge research.
 * PA0 press-release begins 200Hz RAM acquisition. PA0 again freezes trace;
 * otherwise auto-freezes after 512 rows (2.56s). Only THEN transmit
 * loss-detectable CSV over USART2 JDY-33 PA2; PA9/PA10 are unused.
 * No UART/BLE traffic or Flash write from IRQ, NO motor unlock path.
 */
#if defined(MAZE_SENSOR_CAPTURE_MODE)
#include "board.h"
#include "bt_debug.h"
#include "control.h"
#include "encoder.h"
#include "i2c.h"
#include "ir_sensor.h"
#include "motion.h"
#include "motor.h"
#include "mpu6050.h"
#include "system_timer.h"
#include "stm32f4xx.h"
#include <stdio.h>
#include <string.h>

#define MC_CAPACITY 512U
#define MC_PERIOD_MS 5U
#define MC_BLE_PERIOD_MS 125U
#define MC_CHUNK 20U
#define MC_FNV_INIT 2166136261UL
#define MC_FNV_PRIME 16777619UL

typedef struct {
    uint16_t ms, age_us;
    uint16_t ir[6];
    int32_t enc_l, enc_r;
    int16_t yaw_cd, rate_cd;
    uint32_t scan;
    uint8_t valid;
} MC_Sample;
static MC_Sample frames[MC_CAPACITY];
static MPU6050_Handle_t gyro;
static uint16_t count, replay_row, replay_offset, line_len;
static uint8_t phase, button_prev, button_armed, boot_held;
static uint32_t last_ms, press_ms, started_ms, next_tx_ms, hash_value;
static char line[160];

/* All numeric values are serialized once. FNV-1a operates on exact
 * newline-terminated ASCII trace records, checked at receiver.
 * End marker is not included in hash. */
static uint32_t HashLine(uint32_t hash,const char *data,uint16_t length)
{
    uint16_t i;
    for(i=0U;i<length;i++){
        hash ^= (uint8_t)data[i];
        hash *= MC_FNV_PRIME;
    }
    return hash;
}
static uint16_t MakeLine(uint16_t index)
{
    MC_Sample *a=&frames[index];
    int n=snprintf(line,sizeof(line),
        "MC,%u,%u,%lu,%ld,%ld,%d,%d,%u,%u,%u,%u,%u,%u,%u,%u,%u\n",
        (unsigned)index,(unsigned)a->ms,(unsigned long)a->scan,
        (long)a->enc_l,(long)a->enc_r,
        (int)a->yaw_cd,(int)a->rate_cd,(unsigned)a->age_us,
        (unsigned)a->valid,
        (unsigned)a->ir[0],(unsigned)a->ir[1],(unsigned)a->ir[2],
        (unsigned)a->ir[3],(unsigned)a->ir[4],(unsigned)a->ir[5]);
    return (n>0 && (size_t)n<sizeof(line))?(uint16_t)n:0U;
}
static uint8_t PressRelease(uint32_t now)
{
    uint8_t button=Board_ButtonPressed();
    if(button==button_prev)return 0U;
    button_prev=button;
    if(button){press_ms=now;button_armed=1U;return 0U;}
    if(boot_held){boot_held=0U;button_armed=0U;return 0U;}
    if(button_armed && (uint32_t)(now-press_ms)>40U){
        button_armed=0U;return 1U;
    }
    button_armed=0U;return 0U;
}
static void Begin(uint32_t now)
{
    uint32_t irq=__get_PRIMASK();
    __disable_irq();
    Encoder_Reset();
    if(!irq)__enable_irq();
    count=0U;started_ms=now;last_ms=now;phase=1U;
    BT_SendString("MC,ARMED,motor_locked=1,press_pa0_to_stop\n");
}
static void Freeze(void)
{
    Motor_LockOutputs();
    phase=2U;replay_row=0U;replay_offset=0U;
    line_len=0U;hash_value=MC_FNV_INIT;
    next_tx_ms=millis();
    BT_Printf("MC,BEGIN,n=%u,hz=200,diam_mm=34,cpr=1430,locked=%u,fields=seq_ms_scan_encL_encR_yawcd_ratecd_age_valid_L90_L45_L0_R0_R45_R90\n",
              (unsigned)count,(unsigned)Motor_OutputsLocked());
}
static void Collect(uint32_t now)
{
    IR_WallSnapshot ir;
    MPU6050_Snapshot_t g;
    MC_Sample *row;
    uint8_t valid;
    uint32_t irq;
    if((uint32_t)(now-last_ms)<MC_PERIOD_MS)return;
    last_ms=now;
    if(count>=MC_CAPACITY){Freeze();return;}
    row=&frames[count];
    memset(&ir,0,sizeof(ir));
    memset(&g,0,sizeof(g));
    irq=__get_PRIMASK();__disable_irq();
    valid=(uint8_t)(Control_GetWallIRSnapshot(&ir)==IR_WALL_VALID &&
         MPU6050_GetSnapshot(&gyro,&g) &&
         (uint32_t)(micros()-g.completed_us)<=3000U);
    row->enc_l=Encoder_GetLeftCount();
    row->enc_r=Encoder_GetRightCount();
    if(!irq)__enable_irq();
    row->ms=(uint16_t)(now-started_ms);
    row->valid=valid;
    row->age_us=(uint16_t)(ir.age_us>65535U?65535U:ir.age_us);
    row->scan=ir.scan_count;
    if(valid){
        uint8_t i;
        for(i=0U;i<6U;i++)row->ir[i]=ir.ema10_adc[i];
        row->yaw_cd=(int16_t)(g.yaw_deg*100.0f);
        row->rate_cd=(int16_t)(g.gyro_z_dps*100.0f);
    }else{
        uint8_t i;
        for(i=0U;i<6U;i++)row->ir[i]=0U;
        row->yaw_cd=0;row->rate_cd=0;
    }
    count++;
    if(count>=MC_CAPACITY)Freeze();
}
static void Replay(uint32_t now)
{
    uint16_t n;
    if((uint32_t)(now-next_tx_ms)<MC_BLE_PERIOD_MS)return;
    next_tx_ms=now;
    if(replay_row>=count){
        BT_Printf("MC,END,n=%u,fnv32=%08lx,locked=%u\n",
                  (unsigned)count,(unsigned long)hash_value,
                  (unsigned)Motor_OutputsLocked());
        phase=3U;
        return;
    }
    if(line_len==0U){
        line_len=MakeLine(replay_row);
        if(!line_len){
            BT_SendString("MC,ERROR,FORMAT\n");phase=3U;return;
        }
        hash_value=HashLine(hash_value,line,line_len);
    }
    n=(uint16_t)(line_len-replay_offset);
    if(n>MC_CHUNK)n=MC_CHUNK;
    if(BT_TrySendData((const uint8_t*)line+replay_offset,n)){
        replay_offset=(uint16_t)(replay_offset+n);
        if(replay_offset==line_len){
            replay_offset=0U;line_len=0U;replay_row++;
        }
    }
}
int MazeSensorCapture_Main(void)
{
    uint32_t now,last_status=0U;
    MPU6050_Status rc;
    Board_Init();
    SystemTimer_Init();
    BT_Init();
    Motor_Init();Motor_LockOutputs();
    if(Board_GetClockFault()){
        BT_SendString("MC,FATAL,CLOCK\n");for(;;)__WFI();
    }
    Encoder_Init();IR_Init();I2C_Init(I2C_1,1U);
    rc=MPU6050_Init(&gyro);
    if(rc==MPU6050_OK)rc=MPU6050_Calibrate(&gyro);
    if(rc!=MPU6050_OK){
        BT_SendString("MC,FATAL,MPU\n");for(;;)__WFI();
    }
    Motion_Init();IR_StartScan();Control_Init(&gyro);Control_Start();
    phase=0U;
    boot_held=button_prev=Board_ButtonPressed();
    BT_SendString("MC,HELLO,mode=RAM200_BLE_TX,motor_locked=1,vm_off_required=1\n");
    for(;;){
        now=millis();
        MPU6050_Update(&gyro);
        if(Motor_OutputsLocked()==0U){
            Motor_LockOutputs();
            BT_SendString("MC,FATAL,MOTOR_UNLOCK\n");
            phase=3U;
        }
        if(PressRelease(now)){
            if(phase==0U && gyro.calibrated)Begin(now);
            else if(phase==1U)Freeze();
        }
        if(phase==1U)Collect(now);
        if(phase==2U)Replay(now);
        if(phase==0U && (uint32_t)(now-last_status)>1500U){
            IR_WallSnapshot ir;
            last_status=now;
            if(gyro.calibrated && Control_GetWallIRSnapshot(&ir)==IR_WALL_VALID)
                BT_SendString("MC,READY,PA0_START\n");
        }
        if(phase==1U)Board_StatusLed((uint8_t)((now/90U)%2U));
        else if(phase==2U)Board_StatusLed(1U);
        else Board_StatusLed((uint8_t)((now/500U)%2U));
        if(!MPU6050_NeedsImmediatePoll(&gyro))__WFI();
    }
}
#endif
