/* Single-shot 4-cell L0/R0 dual-wall PID; PA0 physical START/STOP.
 * BLE PA2/PA3 TX only. DO NOT initialize USART1 PA9/PA10.
 * Live control TIM11 1kHz; retain existing 20Hz W trace AND
 * add full six-channel 200Hz MW trace in SRAM. All telemetry replay
 * only after motor lock, 20-byte chunks every 125ms.
 * Motor outputs are locked at boot and latched at termination. */
#if defined(WALL_PID_TEST_MODE)
#include "wall_pid_test.h"
#include "mode_menu.h"
#include "wall_pid_trace.h"
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

#define WT_IDLE    0U
#define WT_RUN     1U
#define WT_RESULT  2U
#define WT_TRACE_COUNT 105U
#define WT_TX_PERIOD_MS 125U

typedef struct {
    uint16_t elapsed_ms;
    uint16_t distance_10mm;
    uint16_t l0,r0;
    int16_t error_norm_1000,speed_mm_s,target_mm_s;
    int16_t pwm_l,pwm_r;
} WT_Trace;

static struct {
    volatile uint8_t phase,stop_consumed,started_once,reason;
    volatile uint32_t started_ms,finished_ms;
    volatile uint16_t trace_count;
    WP_Controller ctl;
    WT_Trace trace[WT_TRACE_COUNT];
} st;
static MPU6050_Handle_t mpu;
static ModeMenu mode_menu;
static uint8_t menu_announced;
/* 1Hz Bluetooth-only monitor. No BLE transmissions from TIM11.
 * Unit of yaw_cd and gz_cd: 1/100 degree and 1/100 degree/s.
 * 200Hz SRAM samples are retained internally, never replayed over BLE.
 */
#define WT_SENSOR_PERIOD_MS 1000U
#define WT_CAL_PERIOD_MS 1000U

static void ServiceSensorTelemetry(uint32_t now)
{
    static uint32_t last_ms=0U;
    MPU6050_Snapshot_t gyro;
    IR_WallSnapshot ir;
    uint8_t valid;
    uint32_t age;
    char msg[170];
    int n;
    if(!mpu.calibrated || mpu.calibrating || mpu.cal_state!=3U)return;
    if((uint32_t)(now-last_ms)<WT_SENSOR_PERIOD_MS)return;
    last_ms=now;
    valid=MPU6050_GetSnapshot(&mpu,&gyro)?1U:0U;
    age=valid?(uint32_t)(micros()-gyro.completed_us):0U;
    n=snprintf(msg,sizeof(msg),
        "MPU,ms=%lu,valid=%u,age_us=%lu,yaw_cd=%ld,gz_cd=%ld\n",
        (unsigned long)now,(unsigned)valid,(unsigned long)age,
        valid?(long)(gyro.yaw_deg*100.0f):0L,
        valid?(long)(gyro.gyro_z_dps*100.0f):0L);
    if(n>0&&(size_t)n<sizeof(msg))
        (void)BT_TrySendData((const uint8_t*)msg,(uint16_t)n);
    /* L0/R0 control uses filtered ADC; transmit all six EMA IR channels. */
    (void)Control_GetWallIRSnapshot(&ir);
    n=snprintf(msg,sizeof(msg),
        "IR,ms=%lu,valid=%u,L0=%u,L45=%u,L90=%u,R0=%u,R45=%u,R90=%u\n",
        (unsigned long)now,(unsigned)(ir.status==IR_WALL_VALID),
        (unsigned)ir.ema10_adc[IR_WALL_L0],
        (unsigned)ir.ema10_adc[IR_WALL_L45],
        (unsigned)ir.ema10_adc[IR_WALL_L90],
        (unsigned)ir.ema10_adc[IR_WALL_R0],
        (unsigned)ir.ema10_adc[IR_WALL_R45],
        (unsigned)ir.ema10_adc[IR_WALL_R90]);
    if(n>0&&(size_t)n<sizeof(msg))
        (void)BT_TrySendData((const uint8_t*)msg,(uint16_t)n);
}

/* Only publish CALIB OK once the MPU driver really marks calibration done.
 * Before completion, published MPU yaw/gyro snapshots are invalid by design.
 */
static void ServiceCalibrationStatus(uint32_t now)
{
    static uint8_t previous_state=255U,previous_retries=255U;
    static uint32_t last_ms=0U;
    uint8_t state=mpu.cal_state,retries=mpu.cal_retries;
    if(state==previous_state && retries==previous_retries &&
       (state==3U || state==4U ||
        (uint32_t)(now-last_ms)<WT_CAL_PERIOD_MS))return;
    previous_state=state;
    previous_retries=retries;
    last_ms=now;
    if(state==3U && mpu.calibrated){
        BT_SendString("CALIB OK\n");
    }else if(state==4U){
        BT_Printf("CALIB FAIL,reason=%u,retries=%u\n",
                  (unsigned)mpu.cal_failure_reason,(unsigned)retries);
    }else if(state==1U||state==2U){
        BT_Printf("CALIB...,stage=%u,samples=%u/%u,retries=%u,gyro_good=%lu,gyro_fresh=%lu,gyro_err=%lu\n",
            (unsigned)state,(unsigned)mpu.calibration_count,
            (unsigned)CALIBRATION_SAMPLES,(unsigned)retries,
            (unsigned long)mpu.good_reads,
            (unsigned long)mpu.fresh_gyro_reads,
            (unsigned long)mpu.error_reads);
    }else{
        BT_SendString("CALIB WAIT,mpu_not_ready\n");
    }
}

static const char *Why(WP_Reason w)
{
    switch(w){
    case WP_FINISHED:return "DONE";
    case WP_PA0_STOP:return "PA0_STOP";
    case WP_IR_STALE:return "IR_STALE";
    case WP_IMU_STALE:return "IMU_STALE";
    case WP_WALL_LOST:return "WALL_LOST";
    case WP_EXCURSION:return "IR_EXCURSION";
    case WP_HEADING:return "YAW_LIMIT";
    case WP_ENCODER:return "ENCODER";
    case WP_TIMEOUT:return "TIMEOUT";
    case WP_MOTOR_LOCK:return "MOTOR_LOCK";
    case WP_START_REJECT:return "START_REJECT";
    case WP_OVERSPEED:return "OVERSPEED";
    default:return "UNKNOWN";
    }
}
static void Finish(WP_Reason reason)
{
    if(st.phase!=WT_RUN)return;
    WP_Stop(&st.ctl,reason);
    st.reason=(uint8_t)st.ctl.reason;
    st.finished_ms=millis();
    Motor_LockOutputs();
    st.phase=WT_RESULT;
}
void WallPidTest_Stop(WP_Reason reason)
{
    uint32_t irq=__get_PRIMASK();
    __disable_irq();
    Finish(reason);
    if(!irq)__enable_irq();
}
void WallPidTest_ControlTick(int32_t left,int32_t right,
                             float yaw,float rate,const IR_WallSnapshot *ir)
{
    WP_Input in;
    uint8_t write;
    if(st.phase!=WT_RUN)return;
    if(Board_ButtonPressed()){
        st.stop_consumed=1U;
        Finish(WP_PA0_STOP);
        return;
    }
    if(Motor_OutputsLocked()){Finish(WP_MOTOR_LOCK);return;}
    if(!ir){Finish(WP_IR_STALE);return;}
    in.ir=*ir;
    in.left_count=left;in.right_count=right;
    in.yaw_deg=yaw;in.gyro_dps=rate;
    write=WP_Step(&st.ctl,&in);
    /* Copy six EMA IR channels, both encoders, gyro, PID & PWM at 200Hz.
     * Constant-time SRAM writes only; no BLE, heap or Flash in TIM11. */
    WPT_Append(st.ctl.ticks,&in,&st.ctl);
    if(st.ctl.state!=WP_RUNNING){
        Finish(st.ctl.reason);
        return;
    }
    if(write)Motor_SetPairPermille(st.ctl.pwm_l,st.ctl.pwm_r);
    if(write && st.ctl.ticks%50U==0U && st.trace_count<WT_TRACE_COUNT){
        WT_Trace *t=&st.trace[st.trace_count++];
        t->elapsed_ms=(uint16_t)st.ctl.ticks;
        t->distance_10mm=(uint16_t)(st.ctl.distance_mm*10.0f);
        t->l0=ir->ema10_adc[IR_WALL_L0];
        t->r0=ir->ema10_adc[IR_WALL_R0];
        t->error_norm_1000=(int16_t)(st.ctl.error_norm*1000.0f);
        t->speed_mm_s=(int16_t)st.ctl.speed_mm_s;
        t->target_mm_s=(int16_t)st.ctl.target_speed_mm_s;
        t->pwm_l=st.ctl.pwm_l;
        t->pwm_r=st.ctl.pwm_r;
    }
}
static uint8_t ReadyForStart(void)
{
    MPU6050_Snapshot_t g;
    IR_WallSnapshot ir;
    return Board_GetClockFault()==0U && !st.started_once &&
           st.phase==WT_IDLE && Motor_OutputsLocked() &&
           Control_IsRunning() && mpu.calibrated &&
           MPU6050_GetSnapshot(&mpu,&g) &&
           (uint32_t)(micros()-g.completed_us)<=3000U &&
           Control_GetWallIRSnapshot(&ir)==IR_WALL_VALID;
}
static void StartPhysical(void)
{
    MPU6050_Snapshot_t g;
    IR_WallSnapshot ir;
    WP_Input in;
    uint32_t irq;
    uint8_t success=0U;
    if(!ReadyForStart()){
        BT_SendString("W,REJECT,NOT_READY_OR_USED\n");
        return;
    }
    if(!MPU6050_GetSnapshot(&mpu,&g) ||
       Control_GetWallIRSnapshot(&ir)!=IR_WALL_VALID){
        BT_SendString("W,REJECT,SENSOR_RACE\n");
        return;
    }
    in.ir=ir;in.yaw_deg=g.yaw_deg;in.gyro_dps=g.gyro_z_dps;
    in.left_count=0;in.right_count=0;
    irq=__get_PRIMASK();__disable_irq();
    if(st.phase==WT_IDLE && Motor_OutputsLocked() && mpu.calibrated &&
       (uint32_t)(micros()-g.completed_us)<=3000U &&
       (uint32_t)(micros()-ir.completed_us)<=IR_WALL_MAX_AGE_US){
        Encoder_Reset();
        WP_Init(&st.ctl);
        if(WP_Start(&st.ctl,&in) && Motor_WallPidTestUnlockOutputs()){
            st.trace_count=0U;st.stop_consumed=0U;
            WPT_Reset();
            st.reason=WP_NONE;
            st.started_once=1U;st.started_ms=millis();
            st.phase=WT_RUN;success=1U;
        } else {
            Motor_LockOutputs(); /* Fail closed */
        }
    }
    if(!irq)__enable_irq();
    if(success){
        BT_Printf("W,START,mm=720,diam=34,cpr=1430,pwmcap=800,error=REL_NORM,l=%u,r=%u\n",
                  (unsigned)in.ir.ema10_adc[IR_WALL_L0],
                  (unsigned)in.ir.ema10_adc[IR_WALL_R0]);
    } else BT_SendString("W,REJECT,WALL_ADC_OR_MOTOR\n");
}
/* Exact v2 PA0 event semantics. No selection triggers a motor action.
 * Once Wall PID runs are explicitly mapped in the future, the 1kHz
 * control-tick emergency PA0 check remains independent from the menu.
 */
static void ServiceModeMenu(uint32_t now)
{
    ModeMenuEvent event;
    uint8_t rx;
    while(BT_ReadByte(&rx)){
        if(rx=='?') BT_SendString("SPP CONNECTED\r\n");
    }
    if(st.phase!=WT_IDLE || !mpu.calibrated || mpu.cal_state!=3U ||
       !menu_announced)return;
    event=ModeMenu_Poll(&mode_menu,Board_ButtonPressed(),now);
    if(event==MODE_MENU_EVENT_FOCUS){
        BT_Printf("Mode %u: %s\r\n",
                  (unsigned)(mode_menu.focus+1U),
                  ModeMenu_Name(mode_menu.focus));
    }else if(event==MODE_MENU_EVENT_SELECT){
        BT_Printf("Selected mode %u: %s\r\n",
                  (unsigned)(mode_menu.selected+1U),
                  ModeMenu_Name(mode_menu.selected));
    }
}

int WallPidTest_Main(void)
{
    MPU6050_Status rc;
    uint8_t end_reported=0U;
    Board_Init();
    SystemTimer_Init();
    BT_Init();
    Motor_Init();Motor_LockOutputs();
    BT_SendString("W,HELLO,mode=MENU_7,pa0=short_next_long_select,motor_locked=1\n");
    if(Board_GetClockFault()){
        BT_SendString("W,FATAL,CLOCK\n");for(;;)__WFI();
    }
    Encoder_Init();
    IR_Init();
    I2C_Init(I2C_1,1U);
    rc=MPU6050_Init(&mpu);
    if(rc==MPU6050_OK){
        BT_SendString("READY CAL,MPU_INIT_OK\n");
        rc=MPU6050_Calibrate(&mpu);
        if(rc==MPU6050_OK)BT_SendString("CALIB...,keep_robot_stationary\n");
    }
    if(rc!=MPU6050_OK){
        BT_SendString("W,FATAL,MPU\n");for(;;)__WFI();
    }
    Motion_Init();IR_StartScan();
    Control_Init(&mpu);
    st.phase=WT_IDLE;st.started_once=0U;st.trace_count=0U;
    /* Do not arm button gestures until calibration is actually complete. */
    menu_announced=0U;
    Control_Start();
    BT_SendString("W,WAIT_CAL,menu_locked_until_calib_ok\n");
    for(;;){
        uint32_t now=millis();
        MPU6050_Update(&mpu);
        ServiceModeMenu(now);
        ServiceCalibrationStatus(now);
        if(!menu_announced && mpu.calibrated && mpu.cal_state==3U){
            /* KEY held during calibration is ignored until released. */
            ModeMenu_Init(&mode_menu,Board_ButtonPressed(),now);
            menu_announced=1U;
            BT_SendString("W,MENU_READY,7_modes,no_action_on_select\n");
        }
        ServiceSensorTelemetry(now);
        if(st.phase==WT_RESULT && !end_reported){
            end_reported=1U;
            BT_Printf("W,END,%s,locked=%u\n",
                Why((WP_Reason)st.reason),(unsigned)Motor_OutputsLocked());
        }
        /* LED PC13 active LOW. Fast flash RUN, solid after finished.
         * Slow 1Hz blink READY. No external PC controls movement. */
        if(st.phase==WT_RUN)Board_StatusLed((uint8_t)((now/120U)%2U));
        else if(st.phase==WT_RESULT)Board_StatusLed(1U);
        else Board_StatusLed((uint8_t)((now/500U)%2U));
        if(!MPU6050_NeedsImmediatePoll(&mpu))__WFI();
    }
}
#endif
