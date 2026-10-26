/* Turn PID qualification -- isolated opt-in firmware, NO OLED.
 * Exactly one physical PA0 press-release starts one run.
 * USART1 can ONLY select an idle mode/profile and stop; never start.
 * 1kHz TIM11 receives a coherent MPU snapshot, 200Hz cascaded turn loop,
 * 50Hz timestamped ring-buffer telemetry; never print inside the ISR.
 * All motors are locked outside active rotation. */
#if defined(TURN_PID_DIAG)
#include "turn_pid_diag.h"
#include "board.h"
#include "control.h"
#include "encoder.h"
#include "i2c.h"
#include "ir_sensor.h"
#include "motor.h"
#include "motion.h"
#include "mpu6050.h"
#include "pid.h"
#include "system_timer.h"
#include "uart.h"
#include "stm32f4xx.h"
#include <math.h>
#include <stdint.h>

#define TP_CONTROL_DIV       5U     /* 200Hz cascaded position/rate loop */
#define TP_LOG_DIV           20U    /* 50Hz trace */
#define TP_POST_TICKS        800U   /* 0.8s after hard stop, captures coast */
#define TP_GYRO_MAX_AGE_US   3000U
#define TP_BTN_MIN_MS        40U
#define TP_MAX_PWM_ABS       300    /* hardware bound: 30%, even FAST */
#define TP_RING_SIZE         128U
#define TP_ANGLE_BAND        2.0f
#define TP_RATE_BAND         12.0f
#define TP_SETTLE_TICKS      150U   /* 150ms continuous stable feedback */

enum {TP_IDLE=0,TP_RUNNING=1,TP_COAST=2,TP_RESULT=3};
enum {TP_90=90,TP_180=180};
enum {TP_BASE=0,TP_FAST=1};
enum {TP_DONE=0,TP_BUTTON=1,TP_TIMEOUT=2,TP_IMU=3,
      TP_DIRECTION=4,TP_ANGLE=5,TP_MOTOR=6,TP_UART=7};

typedef struct {
    float kp_angle;       /* desired deg/s per degree error */
    float kp_rate;        /* motor permille per deg/s rate error */
    float ki_rate;
    float kff_rate;       /* motor permille per deg/s feedforward */
    float accel_dps2;     /* max reference acceleration */
    float speed_dps;      /* reference rate ceiling */
    int16_t max_pwm;      /* permille absolute */
    int16_t min_pwm;      /* empirical stiction starting floor */
} TP_Profile;

static const TP_Profile profiles[2]={
    {4.8f,1.15f,0.0f,0.35f,650.0f,150.0f,260,95},
    {6.0f,1.35f,0.0f,0.40f,950.0f,230.0f,300,95}
};

typedef struct {
    uint32_t id,t_ms,age_us;
    int32_t yaw_cd,err_cd,gyro_dps10,ref_dps10;
    int16_t pid_pm,ff_pm,cmd_pm;
    int32_t encoder_l,encoder_r;
    uint8_t phase,mode,profile;
} TP_Sample;

static volatile uint16_t q_head,q_tail;
static TP_Sample q[TP_RING_SIZE];
static volatile uint32_t q_dropped;

typedef struct {
    volatile uint8_t phase, mode, profile, reason;
    volatile uint8_t button_stop_consumed;
    volatile uint32_t id, ticks, post_ticks;
    volatile uint16_t stable_ticks;
    volatile float zero_yaw,yaw,rate,ref_rate,err,peak_yaw,peak_rate;
    volatile int16_t requested_pwm,pid_pm,ff_pm;
    volatile int32_t enc_l,enc_r;
    volatile uint32_t duration_ms,stop_tick;
    PID_Handle_t pid_rate;
} TP_State;
static TP_State tp;
static MPU6050_Handle_t mpu;

static float Limit(float x,float a,float b) {
    return x<a?a:(x>b?b:x);
}
static float Sign(float x) {return x>0.0f?1.0f:(x<0.0f?-1.0f:0.0f);}
static uint32_t RunTimeout(uint8_t mode) {return mode==TP_90?4000U:6500U;}
static const char *WhyName(uint8_t reason) {
    switch(reason) {
        case TP_DONE:return "DONE";
        case TP_BUTTON:return "PA0_STOP";
        case TP_TIMEOUT:return "TIMEOUT";
        case TP_IMU:return "IMU_STALE";
        case TP_DIRECTION:return "WRONG_DIRECTION";
        case TP_ANGLE:return "ANGLE_LIMIT";
        case TP_MOTOR:return "MOTOR_LOCKED";
        default:return "UART_STOP";
    }
}
static void PushSample(uint32_t age_us) {
    uint16_t next=(uint16_t)((q_head+1U)%TP_RING_SIZE);
    TP_Sample *s;
    if(next==q_tail) {++q_dropped;return;}
    s=&q[q_head];
    s->id=tp.id;
    s->t_ms=tp.ticks;
    s->age_us=age_us;
    s->yaw_cd=(int32_t)(tp.yaw*100.0f);
    s->err_cd=(int32_t)(tp.err*100.0f);
    s->gyro_dps10=(int32_t)(tp.rate*10.0f);
    s->ref_dps10=(int32_t)(tp.ref_rate*10.0f);
    s->pid_pm=tp.pid_pm;
    s->ff_pm=tp.ff_pm;
    s->cmd_pm=tp.requested_pwm;
    s->encoder_l=tp.enc_l;
    s->encoder_r=tp.enc_r;
    s->phase=tp.phase;
    s->mode=tp.mode;
    s->profile=tp.profile;
    __DMB();q_head=next; /* SPSC: publish only after sample is complete */
}
static void StopAndCoast(uint8_t why) {
    if(tp.phase!=TP_RUNNING) return;
    Motor_LockOutputs();            /* immediately neutralize bridge */
    tp.reason=why;
    tp.stop_tick=tp.ticks;
    tp.requested_pwm=0;
    tp.pid_pm=0;
    tp.ff_pm=0;
    tp.ref_rate=0.0f;
    tp.post_ticks=0U;
    tp.phase=TP_COAST;
}
void TurnPidDiag_Stop(uint8_t why) {
    uint32_t mask=__get_PRIMASK();__disable_irq();
    StopAndCoast(why);
    if(!mask)__enable_irq();
}
/* Executes at 1kHz (gyro freshness enforced by Control ISR). */
void TurnPidDiag_ControlTick(int32_t left,int32_t right,float yaw_deg,
                             float gyro_dps,uint32_t age_us) {
    const TP_Profile *cfg;
    float error,wanted,limit_brake,rate_error,u,rate_ref;
    if(tp.phase!=TP_RUNNING && tp.phase!=TP_COAST)return;
    ++tp.ticks;
    tp.yaw=yaw_deg-tp.zero_y; /* unwrapped yaw; 180 does not alias -180 */
    tp.rate=gyro_dps;
    tp.err=(float)tp.mode-tp.yaw;
    tp.enc_l=left;tp.enc_r=right;
    if(tp.yaw>tp.peak_yaw)tp.peak_yaw=tp.yaw;
    if(fabsf(tp.rate)>tp.peak_rate)tp.peak_rate=fabsf(tp.rate);
    if(tp.phase==TP_COAST) {
        ++tp.post_ticks;
        if((tp.ticks%TP_LOG_DIV)==0U)PushSample(age_us);
        if(tp.post_ticks>=TP_POST_TICKS)tp.phase=TP_RESULT;
        return;
    }
    if(Board_ButtonPressed()) {
        tp.button_stop_consumed=1U;
        StopAndCoast(TP_BUTTON);
        PushSample(age_us);return;
    }
    if(Motor_OutputsLocked()){StopAndCoast(TP_MOTOR);PushSample(age_us);return;}
    if(!(tp.yaw>-300.0f && tp.yaw<400.0f &&
         tp.rate>-2500.0f && tp.rate<2500.0f)) {
        StopAndCoast(TP_IMU);PushSample(age_us);return;
    }
    if(tp.ticks>=RunTimeout(tp.mode)) {
        StopAndCoast(TP_TIMEOUT);PushSample(age_us);return;
    }
    /* Abort if the physical yaw goes in the opposite direction despite
     * positive-angle command, or exceeds the intended angle too far. */
    if(tp.ticks>220U && (tp.yaw < -6.0f ||
                          (tp.yaw<1.0f && tp.rate < -35.0f))) {
        StopAndCoast(TP_DIRECTION);PushSample(age_us);return;
    }
    if(tp.yaw>(float)tp.mode+22.0f) {
        StopAndCoast(TP_ANGLE);PushSample(age_us);return;
    }
    if((tp.ticks%TP_CONTROL_DIV)==0U) {
        cfg=&profiles[tp.profile];
        error=tp.err;
        if(fabsf(error)<=TP_ANGLE_BAND &&
           fabsf(tp.rate)<=TP_RATE_BAND) {
            Motor_Stop();
            tp.requested_pwm=0;
            tp.pid_pm=0;tp.ff_pm=0;tp.ref_rate=0.0f;
            tp.stable_ticks+=TP_CONTROL_DIV;
            if(tp.stable_ticks>=TP_SETTLE_TICKS)
                StopAndCoast(TP_DONE);
        } else {
            tp.stable_ticks=0U;
            /* Angular rate trajectory with early braking bound.
             * sqrt(2*a*error) enforces speed decrease ahead of target.
             * Position P and gyro-rate PI are a cascaded controller.
             * Ki starts at 0 until an accepted baseline has been measured. */
            limit_brake=sqrtf(2.0f*cfg->accel_dps2*fabsf(error));
            wanted=Sign(error)*Limit(cfg->kp_angle*fabsf(error),0.0f,
                                     Limit(cfg->speed_dps,0.0f,limit_brake));
            rate_ref=tp.ref_rate;
            rate_ref+=Limit(wanted-rate_ref,
                            -cfg->accel_dps2*0.005f,
                             cfg->accel_dps2*0.005f);
            rate_ref=Limit(rate_ref,-cfg->speed_dps,cfg->speed_dps);
            tp.ref_rate=rate_ref;
            rate_error=rate_ref-tp.rate;
            tp.pid_pm=(int16_t)PID_Compute(&tp.pid_rate,rate_ref,tp.rate,0.005f);
            tp.ff_pm=(int16_t)(cfg->kff_rate*rate_ref);
            u=(float)tp.pid_pm+(float)tp.ff_pm;
            u=Limit(u,-(float)cfg->max_pwm,(float)cfg->max_pwm);
            if(fabsf(u)>1.0f && fabsf(u)<cfg->min_pwm)
                u=Sign(u)*(float)cfg->min_pwm;
            u=Limit(u,-(float)TP_MAX_PWM_ABS,(float)TP_MAX_PWM_ABS);
            tp.requested_pwm=(int16_t)u;
            Motor_SetPairPermille((int16_t)-u,(int16_t)u);
        }
    }
    if((tp.ticks%TP_LOG_DIV)==0U)PushSample(age_us);
}
static uint8_t IsReady(void) {
    MPU6050_Snapshot_t snap;
    if(!mpu.calibrated || !Control_IsRunning() ||
       Board_GetClockFault() || !Motor_OutputsLocked() ||
       !MPU6050_GetSnapshot(&mpu,&snap))return 0U;
    return (uint32_t)(micros()-snap.completed_us)<=TP_GYRO_MAX_AGE_US;
}
static void StartByPA0(void) {
    MPU6050_Snapshot_t snap;
    uint32_t mask;
    if(tp.phase!=TP_IDLE || !IsReady()) {
        UART_Printf("TP,REJECT,mode=%u,cal=%u,reason=NOT_READY\r\n",
                    (unsigned)tp.mode,(unsigned)mpu.calibrated);
        return;
    }
    if(!MPU6050_GetSnapshot(&mpu,&snap))return;
    mask=__get_PRIMASK();__disable_irq();
    if(!mpu.calibrated || !Motor_OutputsLocked() ||
       (uint32_t)(micros()-snap.completed_us)>TP_GYRO_MAX_AGE_US) {
        if(!mask)__enable_irq();
        UART_SendString("TP,REJECT,reason=IMU_RACE\r\n");
        return;
    }
    Encoder_Reset();
    Motor_Stop();
    PID_Init(&tp.pid_rate,profiles[tp.profile].kp_rate,
             profiles[tp.profile].ki_rate,0.0f,
             (float)profiles[tp.profile].max_pwm,150.0f);
    ++tp.id;
    tp.ticks=0U;tp.stop_tick=0U;tp.post_ticks=0U;
    tp.stable_ticks=0U;tp.reason=TP_DONE;
    tp.zero_yaw=snap.yaw_deg;
    tp.yaw=tp.rate=tp.ref_rate=0.0f;
    tp.err=(float)tp.mode;tp.peak_rate=0.0f;tp.peak_yaw=0.0f;
    tp.requested_pwm=tp.pid_pm=tp.ff_pm=0;
    tp.enc_l=tp.enc_r=0;
    tp.button_stop_consumed=0U;
    q_head=q_tail=0U;q_dropped=0U;
    if(!Motor_TurnDiagUnlockOutputs()) {
        tp.reason=TP_MOTOR;tp.phase=TP_RESULT;
    } else tp.phase=TP_RUNNING;
    if(!mask)__enable_irq();
    UART_Printf("TP,START,id=%lu,mode=%u,profile=%u,pwm_cap=%d,kp_angle_x100=%d,kp_rate_x100=%d,kff_x100=%d,accel=%u,max_rate=%u,log_hz=50\r\n",
        (unsigned long)tp.id,(unsigned)tp.mode,(unsigned)tp.profile,
        (int)profiles[tp.profile].max_pwm,
        (int)(profiles[tp.profile].kp_angle*100.0f),
        (int)(profiles[tp.profile].kp_rate*100.0f),
        (int)(profiles[tp.profile].kff_rate*100.0f),
        (unsigned)profiles[tp.profile].accel_dps2,
        (unsigned)profiles[tp.profile].speed_dps);
}
static void HandleButton(uint32_t now) {
    static uint8_t initialized,previous,boot_held,allowed;
    static uint32_t press_at;
    uint8_t pressed=Board_ButtonPressed();
    if(!initialized) {
        initialized=1U;previous=pressed;boot_held=pressed;
        press_at=now;return;
    }
    if(pressed==previous)return;
    previous=pressed;
    if(pressed) {
        press_at=now;
        allowed=tp.phase==TP_IDLE && IsReady();
        /* Mark stop even if main polls the button before the 1kHz ISR. */
        if(tp.phase==TP_RUNNING) {
            tp.button_stop_consumed=1U;
            TurnPidDiag_Stop(TP_BUTTON);
        }
        return;
    }
    if(boot_held){boot_held=0U;return;}
    if(tp.button_stop_consumed){tp.button_stop_consumed=0U;return;}
    if((uint32_t)(now-press_at)<TP_BTN_MIN_MS)return;
    if(!allowed) {
        UART_Printf("TP,REJECT,mode=%u,reason=PRESS_BEFORE_READY,cal=%u\r\n",
                    (unsigned)tp.mode,(unsigned)mpu.calibrated);
        return;
    }
    StartByPA0();
}
static void HandleUart(uint8_t c) {
    if(c=='S'||c=='s'){TurnPidDiag_Stop(TP_UART);return;}
    if(c=='?') {
        UART_Printf("TP,STATUS,phase=%u,mode=%u,profile=%u,cal=%u,cal_state=%u,locked=%u,id=%lu,qdrop=%lu,udrop=%lu\r\n",
            (unsigned)tp.phase,(unsigned)tp.mode,(unsigned)tp.profile,
            (unsigned)mpu.calibrated,(unsigned)mpu.cal_state,
            (unsigned)Motor_OutputsLocked(),(unsigned long)tp.id,
            (unsigned long)q_dropped,(unsigned long)UART_GetDroppedMessages());
        return;
    }
    if(c!='1' && c!='2' && c!='b' && c!='f')return;
    if(tp.phase!=TP_IDLE || !Motor_OutputsLocked()) {
        UART_SendString("TP,CONFIG_REJECT,reason=RUNNING\r\n");return;
    }
    if(c=='1')tp.mode=TP_90;
    if(c=='2')tp.mode=TP_180;
    if(c=='b')tp.profile=TP_BASE;
    if(c=='f')tp.profile=TP_FAST;
    UART_Printf("TP,CONFIG,mode=%u,profile=%u,locked=1\r\n",
                (unsigned)tp.mode,(unsigned)tp.profile);
}
static void DrainSamples(void) {
    uint8_t budget=8U;
    while(budget-- && q_tail!=q_head) {
        TP_Sample s=q[q_tail];
        __DMB();q_tail=(uint16_t)((q_tail+1U)%TP_RING_SIZE);
        /* Compact strictly ordered CSV: 17 columns including TP,D.
         * t_ms is generated in TIM11, not delayed host arrival time. */
        UART_Printf("TP,D,%lu,%lu,%u,%u,%u,%ld,%ld,%ld,%ld,%d,%d,%d,%ld,%ld,%lu\r\n",
            (unsigned long)s.id,(unsigned long)s.t_ms,
            (unsigned)s.mode,(unsigned)s.profile,(unsigned)s.phase,
            (long)s.yaw_cd,(long)s.err_cd,(long)s.gyro_dps10,
            (long)s.ref_dps10,(int)s.pid_pm,(int)s.ff_pm,
            (int)s.cmd_pm,(long)s.encoder_l,(long)s.encoder_r,
            (unsigned long)s.age_us);
    }
}
static void EmitCal(uint32_t now) {
    UART_Printf("TP,CAL,ms=%lu,state=%u,ok=%u,retry=%u,fail=%u,blocks=%u,count=%u,good=%lu,err=%lu,locked=%u\r\n",
        (unsigned long)now,(unsigned)mpu.cal_state,
        (unsigned)mpu.calibrated,(unsigned)mpu.cal_retries,
        (unsigned)mpu.cal_failure_reason,(unsigned)mpu.cal_valid_blocks,
        (unsigned)mpu.calibration_count,(unsigned long)mpu.good_reads,
        (unsigned long)mpu.error_reads,
        (unsigned)Motor_OutputsLocked());
}
static void EmitResult(void) {
    uint32_t mask,ticks,stop,id,drop;
    uint8_t reason,mode,profile;
    int32_t yaw,peak,rate;
    mask=__get_PRIMASK();__disable_irq();
    if(tp.phase!=TP_RESULT){if(!mask)__enable_irq();return;}
    ticks=tp.ticks;stop=tp.stop_tick;
    reason=tp.reason;mode=tp.mode;profile=tp.profile;id=tp.id;
    yaw=(int32_t)(tp.yaw*100.0f);
    peak=(int32_t)(tp.peak_yaw*100.0f);
    rate=(int32_t)(tp.peak_rate*10.0f);
    drop=q_dropped;
    tp.phase=TP_IDLE;
    if(!mask)__enable_irq();
    UART_Printf("TP,RESULT,id=%lu,mode=%u,profile=%u,reason=%s,stop_ms=%lu,end_ms=%lu,final_cd=%ld,peak_cd=%ld,peak_dps10=%ld,qdrop=%lu,uartdrop=%lu,locked=%u\r\n",
        (unsigned long)id,(unsigned)mode,(unsigned)profile,
        WhyName(reason),(unsigned long)stop,(unsigned long)ticks,
        (long)yaw,(long)peak,(long)rate,
        (unsigned long)drop,(unsigned long)UART_GetDroppedMessages(),
        (unsigned)Motor_OutputsLocked());
    UART_Printf("TP,READY,mode=%u,profile=%u,PA0=PRESS_RELEASE\r\n",
                (unsigned)tp.mode,(unsigned)tp.profile);
}
int TurnPidDiag_Main(void) {
    MPU6050_Status status;
    uint8_t ch,cal_announced=0U;
    uint32_t last_cal=0U,last_ready=0U;
    Board_Init();SystemTimer_Init();UART_Init();
    Motor_Init();Motor_LockOutputs();
    UART_SendString("TP,HELLO,turn_pid_diag_v1,no_oled=1,motor_locked=1\r\n");
    if(Board_GetClockFault()) {
        UART_Printf("TP,FATAL,CLOCK,%lu\r\n",
                    (unsigned long)Board_GetClockFault());
        for(;;)__WFI();
    }
    Encoder_Init();IR_Init();
    I2C_Init(I2C_1,1U);
    status=MPU6050_Init(&mpu);
    if(status==MPU6050_OK)status=MPU6050_Calibrate(&mpu);
    if(status!=MPU6050_OK) {
        UART_Printf("TP,FATAL,MPU,status=%u,step=%u,i2c=%u\r\n",
            (unsigned)status,(unsigned)mpu.init_step,
            (unsigned)mpu.init_i2c_status);
        for(;;)__WFI();
    }
    Motion_Init();IR_StartScan();
    Control_Init(&mpu);
    tp.phase=TP_IDLE;tp.mode=TP_90;tp.profile=TP_BASE;
    Control_Start();
    UART_SendString("TP,BOOT,mode=90,profile=0,commands=1/2/b/f/?/S,PA0_ONLY_START\r\n");
    for(;;) {
        uint32_t now=millis();
        MPU6050_Update(&mpu);
        HandleButton(now);
        {uint8_t budget=8U;
         while(budget-- && UART_ReadByte(&ch))HandleUart(ch);
        }
        if(mpu.calibrated && !cal_announced) {
            cal_announced=1U;
            UART_Printf("TP,CAL_OK,ms=%lu,good=%lu\r\n",
                        (unsigned long)now,(unsigned long)mpu.good_reads);
            UART_Printf("TP,READY,mode=%u,profile=%u,PA0=PRESS_RELEASE\r\n",
                        (unsigned)tp.mode,(unsigned)tp.profile);
        }
        DrainSamples(); /* MUST precede RESULT to keep host trace ordered. */
        if(tp.phase==TP_RESULT && q_tail==q_head)EmitResult();
        if((uint32_t)(now-last_cal)>=1000U) {
            last_cal=now;
            if(!mpu.calibrated)EmitCal(now);
        }
        if(tp.phase==TP_IDLE && mpu.calibrated &&
           (uint32_t)(now-last_ready)>=5000U) {
            last_ready=now;
            UART_Printf("TP,IDLE,mode=%u,profile=%u,locked=%u\r\n",
                        (unsigned)tp.mode,(unsigned)tp.profile,
                        (unsigned)Motor_OutputsLocked());
        }
        if(tp.phase==TP_RUNNING)
            Board_StatusLed(((now/150U)&1U)!=0U);
        else Board_StatusLed(tp.phase==TP_IDLE && IsReady());
        if(!MPU6050_NeedsImmediatePoll(&mpu))__WFI();
    }
}
#endif
