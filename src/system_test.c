/* PA0-triggered motor system test. No OLED dependency.
 * Three one-shot modes cycle after success: 90deg, 180deg, 200mm.
 * Press and release PA0 to run when gyro is calibrated and fresh.
 * During a run, any PA0 press triggers lock from TIM11 (1kHz).
 * UART1 PA9/PA10 sends results; UART S is STOP only, never START. */
#if defined(SYSTEM_TEST_MODE)
#include "system_test.h"
#include "board.h"
#include "control.h"
#include "encoder.h"
#include "i2c.h"
#include "ir_sensor.h"
#include "motion.h"
#include "motor.h"
#include "mpu6050.h"
#include "pid.h"
#include "system_timer.h"
#include "uart.h"
#include "stm32f4xx.h"
#include <math.h>
#include <stdint.h>

/* Standalone test geometry (NOT production WHEEL_DIAMETER_MM).
 * Wheel RADIUS=34mm; 1430 decoded quadrature counts per full wheel turn.
 * 200mm corresponds to ~1338.79 average encoder counts. */
#define ST_RADIUS_MM           34.0f
#define ST_ENCODER_CPR         1430.0f
#define ST_MM_PER_COUNT        (6.28318530718f * ST_RADIUS_MM / ST_ENCODER_CPR)
#define ST_TARGET_MM           200.0f
#define ST_PWM_LIMIT           260
#define ST_TURN_MIN_PWM        105
#define ST_STRAIGHT_MIN_PWM    80
#define ST_GYRO_MAX_AGE_US     3000UL
#define ST_BUTTON_MIN_MS       40UL
#define ST_PID_PERIOD_TICKS    5UL

enum { ST_IDLE=0, ST_RUN=1, ST_RESULT=2 };
enum { MODE_TURN90=0, MODE_TURN180=1, MODE_STRAIGHT200=2 };
enum { WHY_DONE=0, WHY_STOP=1, WHY_TIMEOUT=2, WHY_IMU=3,
       WHY_ENCODER=4, WHY_MOTOR=5, WHY_ANGLE=7 };

typedef struct {
    volatile uint8_t phase,mode,why;
    volatile uint8_t stop_press_consumed;
    volatile uint32_t ticks;
    volatile uint16_t stable_ticks;
    volatile float yaw_start,yaw_delta,rate_dps,distance_mm;
    volatile int32_t left_counts,right_counts;
} SystemTestState_t;
static SystemTestState_t st;
static MPU6050_Handle_t mpu;
static PID_Handle_t pid_turn,pid_heading,pid_distance;

static float Clamp(float x,float lo,float hi) {
    return x<lo?lo:(x>hi?hi:x);
}
static uint32_t MaxTicks(uint8_t mode) {
    return mode==MODE_TURN90?4500UL:(mode==MODE_TURN180?7500UL:4500UL);
}
static const char *ModeName(uint8_t mode) {
    return mode==MODE_TURN90?"TURN90":mode==MODE_TURN180?"TURN180":"STRAIGHT200";
}
static const char *ReasonName(uint8_t why) {
    switch(why) {
        case WHY_DONE:return "DONE";
        case WHY_STOP:return "STOP";
        case WHY_TIMEOUT:return "TIMEOUT";
        case WHY_IMU:return "IMU_STALE";
        case WHY_ENCODER:return "ENCODER";
        case WHY_MOTOR:return "MOTOR";
        default:return "ANGLE";
    }
}
static void Finish(uint8_t reason) {
    Motor_LockOutputs();
    st.why=reason;
    st.phase=ST_RESULT;
}
/* May be invoked from TIM11; bounded, no UART or I2C operations. */
void SystemTest_Stop(uint8_t reason) {
    uint32_t state=__get_PRIMASK();
    __disable_irq();
    if(st.phase==ST_RUN) Finish(reason);
    if(!state)__enable_irq();
}
void SystemTest_ControlTick(int32_t left,int32_t right,
                             float yaw,float rate) {
    float err,base,corr,u;
    int16_t pwm_l,pwm_r;
    if(st.phase!=ST_RUN)return;
    /* Physical STOP at ISR frequency, independent of main scheduling. */
    if(Board_ButtonPressed()) {
        st.stop_press_consumed=1U;
        Finish(WHY_STOP);
        return;
    }
    if(Motor_OutputsLocked()) {Finish(WHY_MOTOR);return;}
    st.ticks++;
    st.yaw_delta=yaw-st.yaw_start;
    st.rate_dps=rate;
    st.left_counts=left;st.right_counts=right;
    st.distance_mm=0.5f*((float)left+(float)right)*ST_MM_PER_COUNT;
    if(st.ticks>=MaxTicks(st.mode)){Finish(WHY_TIMEOUT);return;}
    if(!(st.yaw_delta>-720.0f && st.yaw_delta<720.0f &&
         rate>-2500.0f && rate<2500.0f)) {
        Finish(WHY_IMU);return; /* catches NaN/inf */
    }
    if(st.ticks%ST_PID_PERIOD_TICKS)return;
    if(st.mode==MODE_STRAIGHT200) {
        if(fabsf(st.yaw_delta)>15.0f){Finish(WHY_ANGLE);return;}
        if(left<-35 || right<-35 || st.distance_mm < -5.0f ||
           (st.ticks>1200UL && left<10 && right<10) ||
           (st.ticks>1600UL && (left<15 || right<15)) ||
           (left+right>600 && (left-right>280 || right-left>280))) {
            Finish(WHY_ENCODER);return;
        }
        /* Encoder is the distance reference, IMU is the heading reference. */
        if(st.distance_mm>=ST_TARGET_MM){Finish(WHY_DONE);return;}
        err=ST_TARGET_MM-st.distance_mm;
        base=Clamp(PID_Compute(&pid_distance,err,0.0f,0.005f),
                   ST_STRAIGHT_MIN_PWM,ST_PWM_LIMIT);
        corr=PID_Compute(&pid_heading,st.yaw_delta,0.0f,0.005f);
        pwm_l=(int16_t)Clamp(base+corr,0.0f,ST_PWM_LIMIT);
        pwm_r=(int16_t)Clamp(base-corr,0.0f,ST_PWM_LIMIT);
        Motor_SetPairPermille(pwm_l,pwm_r);
    } else {
        const float target=st.mode==MODE_TURN90?90.0f:180.0f;
        err=target-st.yaw_delta;
        if(st.yaw_delta < -20.0f ||
           st.yaw_delta>target+25.0f){Finish(WHY_ANGLE);return;}
        if(fabsf(err)<=2.0f) {
            Motor_Stop();
            if(fabsf(rate)<=12.0f)st.stable_ticks+=ST_PID_PERIOD_TICKS;
            else st.stable_ticks=0U;
            if(st.stable_ticks>=120U)Finish(WHY_DONE);
            return;
        }
        st.stable_ticks=0U;
        u=Clamp(fabsf(PID_Compute(&pid_turn,err,0.0f,0.005f)),
                ST_TURN_MIN_PWM,ST_PWM_LIMIT);
        /* Same positive yaw direction as existing Motion_Update(). */
        pwm_l=(int16_t)(err>=0.0f?-u:u);
        pwm_r=(int16_t)-pwm_l;
        Motor_SetPairPermille(pwm_l,pwm_r);
    }
}
static uint8_t ImuReady(uint32_t *age_us) {
    MPU6050_Snapshot_t snap;
    if(age_us)*age_us=0xFFFFFFFFUL;
    if(Board_GetClockFault()!=0U || !Control_IsRunning() ||
       !mpu.calibrated || !Motor_OutputsLocked() ||
       !MPU6050_GetSnapshot(&mpu,&snap))return 0U;
    if(age_us)*age_us=(uint32_t)(micros()-snap.completed_us);
    return (uint32_t)(micros()-snap.completed_us)<=ST_GYRO_MAX_AGE_US;
}
/* No remote-start path. Called only for debounced physical PA0 release. */
static void StartFromButton(void) {
    MPU6050_Snapshot_t snap;
    uint32_t age,irq;
    if(st.phase!=ST_IDLE)return;
    if(!ImuReady(&age)) {
        UART_Printf("ST,REJECT,mode=%u,reason=IMU_NOT_READY,cal=%u,cal_state=%u,age_us=%lu\r\n",
                    (unsigned)st.mode,(unsigned)mpu.calibrated,
                    (unsigned)mpu.cal_state,(unsigned long)age);
        return;
    }
    if(!MPU6050_GetSnapshot(&mpu,&snap))return;
    irq=__get_PRIMASK();__disable_irq();
    if(!mpu.calibrated || !Motor_OutputsLocked() ||
       (uint32_t)(micros()-snap.completed_us)>ST_GYRO_MAX_AGE_US) {
        if(!irq)__enable_irq();
        UART_SendString("ST,REJECT,reason=IMU_RACE\r\n");
        return;
    }
    /* TIM11 cannot race with zeroing counts and PID state here. */
    Motor_Stop();
    Encoder_Reset();
    PID_Reset(&pid_turn);PID_Reset(&pid_heading);PID_Reset(&pid_distance);
    st.ticks=0U;st.stable_ticks=0U;st.why=WHY_DONE;
    st.yaw_start=snap.yaw_deg;st.yaw_delta=0.0f;
    st.rate_dps=0.0f;st.distance_mm=0.0f;
    st.left_counts=st.right_counts=0;
    st.stop_press_consumed=0U;
    if(Motor_SystemTestUnlockOutputs())st.phase=ST_RUN;
    else {st.why=WHY_MOTOR;st.phase=ST_RESULT;}
    if(!irq)__enable_irq();
    UART_Printf("ST,START,mode=%u,name=%s,target=%u,pwm_limit=%u,R_mm=34,CPR=1430\r\n",
        (unsigned)st.mode,ModeName(st.mode),
        st.mode==MODE_STRAIGHT200?200U:(st.mode==MODE_TURN90?90U:180U),
        (unsigned)ST_PWM_LIMIT);
}
/* Return 1 for one button release. Ignore the release associated with
 * an in-motion emergency stop, even if TIM11 beat the foreground poll. */
static uint8_t PollButtonRelease(uint32_t now) {
    static uint8_t initialized=0U,prev_down=0U;
    static uint8_t ignored_boot_release=0U;
    static uint8_t press_was_ready=0U;
    static uint32_t down_at=0U;
    uint8_t down=Board_ButtonPressed();
    uint32_t held_ms;
    if(!initialized) {
        initialized=1U;prev_down=down;
        ignored_boot_release=down;
        down_at=now;
        return 0U;
    }
    if(down==prev_down)return 0U;
    prev_down=down;
    if(down) {
        down_at=now;
        press_was_ready=(st.phase==ST_IDLE && ImuReady(0));
        return 0U;
    }
    held_ms=(uint32_t)(now-down_at);
    if(ignored_boot_release) {
        ignored_boot_release=0U;
        return 0U;
    }
    if(st.stop_press_consumed) {
        st.stop_press_consumed=0U;
        return 0U;
    }
    if(held_ms<ST_BUTTON_MIN_MS)return 0U;
    if(!press_was_ready) {
        UART_Printf("ST,REJECT,reason=BUTTON_WHILE_NOT_READY,mode=%u,cal=%u\r\n",
                    (unsigned)st.mode,(unsigned)mpu.calibrated);
        return 0U;
    }
    return 1U;
}
static void EmitResultAndNext(void) {
    uint8_t mode,why,next;
    int32_t yl,dl,left,right;
    uint32_t ticks,irq;
    irq=__get_PRIMASK();__disable_irq();
    if(st.phase!=ST_RESULT) {
        if(!irq)__enable_irq();
        return;
    }
    mode=st.mode;why=st.why;ticks=st.ticks;
    yl=(int32_t)(st.yaw_delta*100.0f);
    dl=(int32_t)(st.distance_mm*100.0f);
    left=st.left_counts;right=st.right_counts;
    next=why==WHY_DONE?(uint8_t)((mode+1U)%3U):mode;
    st.mode=next;st.phase=ST_IDLE;
    if(!irq)__enable_irq();
    UART_Printf("ST,RESULT,mode=%u,reason=%s,yaw_cdeg=%ld,dist_0p01mm=%ld,L=%ld,R=%ld,ticks=%lu,locked=%u\r\n",
                (unsigned)mode,ReasonName(why),(long)yl,(long)dl,
                (long)left,(long)right,(unsigned long)ticks,
                (unsigned)Motor_OutputsLocked());
    UART_Printf("ST,READY_NEXT,mode=%u,name=%s,action=PRESS_PA0\r\n",
                (unsigned)next,ModeName(next));
}
static void EmitCalibration(uint32_t now) {
    MPU6050_Snapshot_t sn;
    uint8_t valid=MPU6050_GetSnapshot(&mpu,&sn);
    uint32_t age=valid?(uint32_t)(micros()-sn.completed_us):0xFFFFFFFFUL;
    UART_Printf("ST,CAL,ms=%lu,state=%u,ok=%u,retry=%u,reason=%u,count=%u,good=%lu,err=%lu,age_us=%lu,locked=%u\r\n",
        (unsigned long)now,(unsigned)mpu.cal_state,(unsigned)mpu.calibrated,
        (unsigned)mpu.cal_retries,(unsigned)mpu.cal_failure_reason,
        (unsigned)mpu.calibration_count,(unsigned long)mpu.good_reads,
        (unsigned long)mpu.error_reads,(unsigned long)age,
        (unsigned)Motor_OutputsLocked());
}
int SystemTest_Main(void) {
    MPU6050_Status init;
    uint8_t ch,was_calibrated=0U;
    uint32_t last_cal=0U,last_data=0U,last_idle=0U;
    Board_Init();SystemTimer_Init();UART_Init();
    Motor_Init();Motor_LockOutputs();
    UART_SendString("ST,HELLO,profile=PA0_NO_OLED,locked=1\r\n");
    if(Board_GetClockFault()) {
        UART_Printf("ST,FATAL,CLOCK,fault=%lu\r\n",
                    (unsigned long)Board_GetClockFault());
        for(;;)__WFI();
    }
    Encoder_Init();IR_Init();
    I2C_Init(I2C_1,1U); /* MPU6050 only; no OLED probe or draw. */
    init=MPU6050_Init(&mpu);
    if(init==MPU6050_OK)init=MPU6050_Calibrate(&mpu);
    if(init!=MPU6050_OK) {
        UART_Printf("ST,FATAL,MPU,status=%u,step=%u,i2c=%u\r\n",
            (unsigned)init,(unsigned)mpu.init_step,
            (unsigned)mpu.init_i2c_status);
        for(;;)__WFI();
    }
    Motion_Init();IR_StartScan();
    PID_Init(&pid_turn,3.6f,0.0f,0.0f,260.0f,0.0f);
    PID_Init(&pid_heading,5.0f,0.0f,0.0f,90.0f,0.0f);
    PID_Init(&pid_distance,2.4f,0.0f,0.0f,260.0f,0.0f);
    Control_Init(&mpu);
    st.phase=ST_IDLE;st.mode=MODE_TURN90;st.why=WHY_DONE;
    st.stop_press_consumed=0U;
    Control_Start();
    UART_SendString("ST,BOOT,no_oled=1,locked=1,first=TURN90,PA0=PRESS_RELEASE,S=STOP\r\n");
    UART_SendString("ST,WAIT_CAL,keep_robot_still\r\n");
    for(;;) {
        uint32_t now=millis();
        MPU6050_Update(&mpu);
        if(PollButtonRelease(now))StartFromButton();
        {uint8_t budget=8U;
         while(budget-- && UART_ReadByte(&ch)) {
             if(ch=='S'||ch=='s') SystemTest_Stop(WHY_STOP);
             else if(ch=='?')EmitCalibration(now);
         }
        }
        if(mpu.calibrated && !was_calibrated) {
            was_calibrated=1U;
            UART_Printf("ST,CAL_OK,ms=%lu,good=%lu,locked=%u\r\n",
                        (unsigned long)now,(unsigned long)mpu.good_reads,
                        (unsigned)Motor_OutputsLocked());
            UART_Printf("ST,READY,mode=%u,name=%s,action=PRESS_PA0\r\n",
                        (unsigned)st.mode,ModeName(st.mode));
        }
        if(st.phase==ST_RESULT) EmitResultAndNext();
        if(st.phase==ST_RUN && (uint32_t)(now-last_data)>=50U) {
            int16_t out_l=0,out_r=0;
            uint32_t primask=__get_PRIMASK();
            uint8_t mode;
            int32_t yd,rd,dm,l,r;
            __disable_irq();
            mode=st.mode;
            yd=(int32_t)(st.yaw_delta*100.0f);
            rd=(int32_t)(st.rate_dps*10.0f);
            dm=(int32_t)(st.distance_mm*100.0f);
            l=st.left_counts;r=st.right_counts;
            if(!primask)__enable_irq();
            Motor_GetAppliedPairPermille(&out_l,&out_r);
            last_data=now;
            UART_Printf("ST_DATA,ms=%lu,mode=%u,yaw_cdeg=%ld,gz_0p1dps=%ld,dist_0p01mm=%ld,L=%ld,R=%ld,PWM_L=%d,PWM_R=%d\r\n",
                    (unsigned long)now,(unsigned)mode,(long)yd,
                    (long)rd,(long)dm,(long)l,(long)r,
                    (int)out_l,(int)out_r);
        }
        if((uint32_t)(now-last_cal)>=1000U) {
            last_cal=now;
            if(!mpu.calibrated)EmitCalibration(now);
        }
        if(st.phase==ST_IDLE && mpu.calibrated &&
           (uint32_t)(now-last_idle)>=5000U) {
            last_idle=now;
            UART_Printf("ST,IDLE,mode=%u,name=%s,locked=%u,action=PRESS_PA0\r\n",
                        (unsigned)st.mode,ModeName(st.mode),
                        (unsigned)Motor_OutputsLocked());
        }
        /* PC13 steady when ready, blinks only while a test is running. */
        if(st.phase==ST_RUN)
            Board_StatusLed(((now/150U)&1U)!=0U);
        else Board_StatusLed(st.phase==ST_IDLE && ImuReady(0));
        if(!MPU6050_NeedsImmediatePoll(&mpu))__WFI();
    }
}
#endif /* SYSTEM_TEST_MODE */
