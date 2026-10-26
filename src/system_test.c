/* Opt-in STM32F411 system-test: PA0 menu, MPU yaw pivots, 200mm encoder run.
 * Nonblocking OLED_Shared on existing I2C1; no OLED/I2C work in TIM11. */
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
#include "oled_shared.h"
#include "pid.h"
#include "ssd1306.h"
#include "system_timer.h"
#include "uart.h"
#include "stm32f4xx.h"
#include <math.h>
#include <stdio.h>

/* Test-only geometry: 34mm RADIUS, 1430 quadrature counts/wheel-revolution.
 * Original production WHEEL_DIAMETER_MM in pinout.h stays unchanged.
 * 200mm / (2*pi*34/1430) = 1338.79 average counts. */
#define ST_RADIUS_MM          34.0f
#define ST_COUNTS_REV         1430.0f
#define ST_MM_PER_COUNT       (6.28318530718f*ST_RADIUS_MM/ST_COUNTS_REV)
#define ST_TARGET_MM          200.0f
#define ST_PWM_CAP            260
#define ST_PWM_TURN_MIN       105
#define ST_PWM_STRAIGHT_MIN   80
#define ST_GYRO_MAX_US        3000U
#define ST_SCL_MIN_HZ         300000UL
#define ST_LONG_MS           650U

enum {PAGE_MENU=0,PAGE_READY=1,PAGE_RUN=2,PAGE_RESULT=3};
enum {MODE_90=0,MODE_180=1,MODE_200=2};
enum {WHY_DONE=0,WHY_STOP=1,WHY_TIMEOUT=2,WHY_IMU=3,
      WHY_ENCODER=4,WHY_MOTOR=5,WHY_OLED=6,WHY_ANGLE=7};

typedef struct {
    volatile uint8_t page,mode,why;
    volatile uint32_t ticks;
    volatile uint16_t stable;
    volatile float yaw_start,yaw_delta,rate,dist;
    volatile int32_t left,right;
} State;
static State st;
static MPU6050_Handle_t mpu;
static PID_Handle_t pid_turn,pid_heading,pid_dist;

static float limit(float x,float lo,float hi) {
    return x<lo?lo:(x>hi?hi:x);
}
static uint32_t max_ticks(uint8_t mode) {
    return mode==MODE_90?4500U:(mode==MODE_180?7500U:4500U);
}
static const char *mode_label(uint8_t mode) {
    return mode==MODE_90?"TURN +90":mode==MODE_180?"TURN +180":"STRAIGHT 20CM";
}
static const char *why_label(uint8_t why) {
    switch(why) {
        case WHY_DONE:return "DONE";
        case WHY_STOP:return "USER STOP";
        case WHY_TIMEOUT:return "TIMEOUT";
        case WHY_IMU:return "IMU STALE";
        case WHY_ENCODER:return "ENCODER FAULT";
        case WHY_MOTOR:return "MOTOR FAULT";
        case WHY_OLED:return "OLED/I2C FAULT";
        default:return "ANGLE FAULT";
    }
}
static void finish(uint8_t why) {
    Motor_LockOutputs();
    st.why=why;
    st.page=PAGE_RESULT;
}
void SystemTest_Stop(uint8_t why) {
    uint32_t mask=__get_PRIMASK();__disable_irq();
    if(st.page==PAGE_RUN)finish(why);
    if(!mask)__enable_irq();
}
/* Called at TIM11 1kHz; PID and motor commands are updated at 200Hz.
 * All actuation has a hard PWM ceiling. No USART or I2C from ISR. */
void SystemTest_ControlTick(int32_t left,int32_t right,float yaw,float rate) {
    float err,u,base,corr;
    int16_t l,r;
    if(st.page!=PAGE_RUN)return;
    if(Board_ButtonPressed()){finish(WHY_STOP);return;}
    if(Motor_OutputsLocked()){finish(WHY_MOTOR);return;}
    ++st.ticks;
    st.yaw_delta=yaw-st.yaw_start;
    st.rate=rate;
    st.left=left;st.right=right;
    st.dist=0.5f*((float)left+(float)right)*ST_MM_PER_COUNT;
    if(st.ticks>=max_ticks(st.mode)){finish(WHY_TIMEOUT);return;}
    if(st.ticks%5U)return;
    if(st.mode==MODE_200) {
        if(left < -35 || right < -35 || st.dist < -5.0f ||
           (st.ticks>1200U && left<10 && right<10)) {
            finish(WHY_ENCODER);return;
        }
        if(st.dist>=ST_TARGET_MM){finish(WHY_DONE);return;}
        /* P distance controller uses encoder, PID heading uses IMU yaw.
         * Tuning is provisional; braking/physical overshoot must be measured. */
        err=ST_TARGET_MM-st.dist;
        base=limit(PID_Compute(&pid_dist,err,0.0f,0.005f),
                   ST_PWM_STRAIGHT_MIN,ST_PWM_CAP);
        corr=PID_Compute(&pid_heading,st.yaw_delta,0.0f,0.005f);
        l=(int16_t)limit(base+corr,0,ST_PWM_CAP);
        r=(int16_t)limit(base-corr,0,ST_PWM_CAP);
        Motor_SetPairPermille(l,r);
    } else {
        float target=st.mode==MODE_90?90.0f:180.0f;
        err=target-st.yaw_delta; /* UNWRAPPED gyro yaw, including 180° */
        if(st.yaw_delta < -20.0f || st.yaw_delta>target+25.0f) {
            finish(WHY_ANGLE);return;
        }
        if(fabsf(err)<=2.0f) {
            Motor_Stop();
            if(fabsf(rate)<=12.0f)st.stable+=5U;
            else st.stable=0U;
            if(st.stable>=120U)finish(WHY_DONE);
            return;
        }
        st.stable=0U;
        u=limit(fabsf(PID_Compute(&pid_turn,err,0.0f,0.005f)),
                ST_PWM_TURN_MIN,ST_PWM_CAP);
        l=(int16_t)(err>=0.0f?-u:u);
        r=(int16_t)-l; /* matches existing motion.c turning convention */
        Motor_SetPairPermille(l,r);
    }
}
static uint8_t ready_to_start(void) {
    MPU6050_Snapshot_t sn;
    OLED_SharedStats os;
    OLED_SharedGetStats(&os);
    return mpu.calibrated && Board_GetClockFault()==0U &&
        OLED_SharedEnabled() && OLED_SharedReady() &&
        I2C_GetConfiguredSclHz()>=ST_SCL_MIN_HZ &&
        os.frames>0U && os.errors==0U && Motor_OutputsLocked() &&
        MPU6050_GetSnapshot(&mpu,&sn) &&
        (uint32_t)(micros()-sn.completed_us)<=ST_GYRO_MAX_US;
}
static void start_selected(void) {
    MPU6050_Snapshot_t sn;
    uint32_t mask;
    if(!ready_to_start()) {
        st.page=PAGE_RESULT;st.why=WHY_OLED;
        UART_SendString("ST,REJECT,NOT_READY\r\n");return;
    }
    if(!MPU6050_GetSnapshot(&mpu,&sn))return;
    mask=__get_PRIMASK();__disable_irq();
    if((uint32_t)(micros()-sn.completed_us)>ST_GYRO_MAX_US ||
       !Motor_OutputsLocked()){if(!mask)__enable_irq();return;}
    Motor_Stop();
    Encoder_Reset();
    PID_Reset(&pid_turn);PID_Reset(&pid_heading);PID_Reset(&pid_dist);
    st.ticks=0U;st.stable=0U;st.why=WHY_DONE;
    st.yaw_start=sn.yaw_deg;st.yaw_delta=0.0f;
    st.rate=0.0f;st.dist=0.0f;st.left=st.right=0;
    if(Motor_SystemTestUnlockOutputs())st.page=PAGE_RUN;
    else {st.page=PAGE_RESULT;st.why=WHY_MOTOR;}
    if(!mask)__enable_irq();
    UART_Printf("ST,START,mode=%u,radius_mm=34,cpr=1430,pwm_cap=260\r\n",
                (unsigned)st.mode);
}
/* Single PA0 button: short cycle/back, long release select/confirm.
 * Any press while RUN is detected and stopped INSIDE TIM11 immediately. */
static uint8_t button_event(uint32_t now) {
    static uint8_t prev=0U;
    static uint32_t down_at=0U;
    uint8_t down=Board_ButtonPressed();
    uint32_t duration;
    if(down==prev)return 0U;
    prev=down;
    if(down){down_at=now;return 0U;}
    duration=(uint32_t)(now-down_at);
    return duration<35U?0U:(duration>=ST_LONG_MS?2U:1U);
}
static void on_button(uint8_t ev) {
    if(!ev || st.page==PAGE_RUN)return;
    if(st.page==PAGE_MENU) {
        if(ev==1U)st.mode=(uint8_t)((st.mode+1U)%3U);
        else st.page=PAGE_READY;
    } else if(st.page==PAGE_READY) {
        if(ev==1U)st.page=PAGE_MENU;
        else start_selected();
    } else {
        st.page=ev==1U?PAGE_MENU:PAGE_READY;
    }
}
static void textline(uint16_t y,const char *s) {
    SSD1306_SetCursor(0U,y);
    SSD1306_WriteString(s,&Font_7x10,SSD1306_WHITE);
}
static void draw(void) {
    char line[26];
    uint8_t page,mode,why;
    long yaw,dist,rate,l,r;
    uint32_t mask=__get_PRIMASK();__disable_irq();
    page=st.page;mode=st.mode;why=st.why;
    yaw=(long)st.yaw_delta;dist=(long)st.dist;rate=(long)st.rate;
    l=(long)st.left;r=(long)st.right;
    if(!mask)__enable_irq();
    SSD1306_Fill(SSD1306_BLACK);
    if(page==PAGE_MENU) {
        textline(0U,"SYSTEM TEST");
        textline(13U,mode==0U?"> 1.TURN +90":"  1.TURN +90");
        textline(26U,mode==1U?"> 2.TURN +180":"  2.TURN +180");
        textline(39U,mode==2U?"> 3.STRAIGHT20":"  3.STRAIGHT20");
        textline(52U,mpu.calibrated?"TAP:NEXT HOLD:SEL":"CALIBRATING...");
    } else if(page==PAGE_READY) {
        textline(0U,"READY MOTOR LOCKED");
        textline(13U,mode_label(mode));
        textline(26U,mode==MODE_200?"R34 / 1430 CPR":"IMU YAW PID ONLY");
        textline(39U,"CHECK CLEAR AREA");
        textline(52U,"HOLD:RUN TAP:BACK");
    } else if(page==PAGE_RUN) {
        textline(0U,mode_label(mode));
        (void)snprintf(line,sizeof(line),"Yaw:%ld Gz:%ld",yaw,rate);
        textline(13U,line);
        if(mode==MODE_200) {
            (void)snprintf(line,sizeof(line),"Dist:%ld/200mm",dist);
            textline(26U,line);
            (void)snprintf(line,sizeof(line),"L:%ld R:%ld",l,r);
            textline(39U,line);
        } else {
            (void)snprintf(line,sizeof(line),"Target:%u deg",mode==0U?90U:180U);
            textline(26U,line);
        }
        textline(52U,"PRESS PA0 TO STOP");
    } else {
        textline(0U,why==WHY_DONE?"TEST COMPLETE":"TEST ABORTED");
        textline(13U,why_label(why));
        (void)snprintf(line,sizeof(line),"Yaw:%ld deg",yaw);
        textline(26U,line);
        (void)snprintf(line,sizeof(line),"Dist:%ld mm",dist);
        textline(39U,line);
        textline(52U,"TAP:MENU HOLD:ARM");
    }
}
int SystemTest_Main(void) {
    MPU6050_Status init;
    uint8_t found=0U,ch,event,old_page=255U;
    uint32_t last_draw=0U,last_uart=0U;
    Board_Init();SystemTimer_Init();UART_Init();
    Motor_Init();Motor_LockOutputs();
    if(Board_GetClockFault()){
        UART_SendString("ST,FATAL,CLOCK\r\n");
        for(;;)__WFI();
    }
    Encoder_Init();IR_Init();
    /* Only blocking OLED init before the MPU/TIM11 are started. */
    I2C_Init(I2C_1,1U);
    found=SSD1306_Init()==SSD1306_OK;
    init=MPU6050_Init(&mpu);
    if(init==MPU6050_OK)init=MPU6050_Calibrate(&mpu);
    if(init!=MPU6050_OK){
        UART_Printf("ST,FATAL,MPU,%u\r\n",(unsigned)init);
        for(;;)__WFI();
    }
    OLED_SharedInit(found);
    OLED_SharedEnable(found);
    Motion_Init();IR_StartScan();
    Control_Init(&mpu);
    st.page=PAGE_MENU;st.mode=MODE_90;st.why=WHY_DONE;
    Control_Start();
    UART_Printf("ST,BOOT,oled=%u,locked=%u,radius=34,cpr=1430\r\n",
                (unsigned)found,(unsigned)Motor_OutputsLocked());
    for(;;) {
        uint32_t now=millis();
        MPU6050_Update(&mpu);
        event=button_event(now);
        on_button(event);
        {uint8_t budget=8U;
         while(budget-- && UART_ReadByte(&ch))
             if(ch=='S'||ch=='s')SystemTest_Stop(WHY_STOP);
        }
        if(st.page==PAGE_RUN) {
            OLED_SharedStats os;
            OLED_SharedGetStats(&os);
            if(!OLED_SharedReady()||os.errors ||
               I2C_GetConfiguredSclHz()<ST_SCL_MIN_HZ)
                SystemTest_Stop(WHY_OLED);
        }
        if(st.page!=old_page || (uint32_t)(now-last_draw)>=200U) {
            old_page=st.page;last_draw=now;
            if(OLED_SharedEnabled())draw();
        }
        OLED_SharedService(&mpu);
        if((uint32_t)(now-last_uart)>=250U){
            last_uart=now;
            if(st.page==PAGE_RUN || st.page==PAGE_RESULT)
                UART_Printf("ST,ms=%lu,page=%u,mode=%u,why=%u,yaw=%ld,dist=%ld,L=%ld,R=%ld,lock=%u\r\n",
                            (unsigned long)now,(unsigned)st.page,
                            (unsigned)st.mode,(unsigned)st.why,
                            (long)st.yaw_delta,(long)st.dist,
                            (long)st.left,(long)st.right,
                            (unsigned)Motor_OutputsLocked());
        }
        if(!MPU6050_NeedsImmediatePoll(&mpu))__WFI();
    }
}
#endif /* SYSTEM_TEST_MODE */
