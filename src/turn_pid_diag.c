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
#define TP_MAX_PWM_ABS       400    /* absolute opt-in test ceiling: 40% */
#define TP_RING_SIZE         128U
#define TP_ANGLE_BAND        2.0f
#define TP_RATE_BAND         12.0f
#define TP_SETTLE_TICKS      150U   /* 150ms continuous stable feedback */
/* Phase 1 diagnostic, bounded moving-floor PWM probe; NOT motor calibration.
 * 6 independent short pulses, neutral settling between pulses.
 * Only physical PA0 release starts; a second press is emergency STOP.
 * Test in clear floor area, start with fully charged and secured battery. */
#define TP_SWEEP_ON_MS         150U
#define TP_SWEEP_OFF_MS        100U
#define TP_SWEEP_SEG_MS        (TP_SWEEP_ON_MS+TP_SWEEP_OFF_MS)
#define TP_SWEEP_N            6U
#define TP_SWEEP_MAX_YAW      65.0f
#define TP_SWEEP_MAX_DPS      220.0f
static const int16_t tp_sweep_pwm[TP_SWEEP_N]={70,95,120,155,200,260};


enum {TP_IDLE=0,TP_RUNNING=1,TP_COAST=2,TP_RESULT=3};
enum {TP_SWEEP=0,TP_90=90,TP_180=180};
enum {TP_BASE=0,TP_FAST=1,TP_TUNED=2};
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
    int16_t min_pwm;      /* legacy minimum-duty floor (BASE/FAST only) */
    float static_ff;      /* TUNED: smoothed static friction compensation */
    float predict_s;      /* TUNED: measured angular-rate braking lead */
} TP_Profile;

/* BASE and FAST are left bit-for-bit comparable to previous sessions.
 * TUNED is a FIRST, UNVERIFIED controller trial, not an optimized result. */
static const TP_Profile profiles[3]={
    {4.8f,1.15f,0.0f,0.35f,650.0f,150.0f,260,95,0.0f,0.0f},
    {6.0f,1.35f,0.0f,0.40f,950.0f,230.0f,300,95,0.0f,0.0f},
    {6.5f,1.75f,0.0f,0.95f,850.0f,200.0f,350,0,48.0f,0.045f}
};

typedef struct {
    uint32_t id,t_ms,age_us;
    int32_t yaw_cd,err_cd,gyro_dps10,ref_dps10;
    int16_t pid_pm,ff_pm,cmd_pm,applied_l_pm,applied_r_pm;
    int32_t encoder_l,encoder_r;
    uint8_t phase,mode,profile,stage;
} TP_Sample;

static volatile uint16_t q_head,q_tail;
static TP_Sample q[TP_RING_SIZE];
static volatile uint32_t q_dropped;

typedef struct {
    volatile uint8_t phase, mode, profile, reason, stage;
    volatile uint8_t button_stop_consumed;
    volatile uint32_t id, ticks, post_ticks;
    volatile uint16_t stable_ticks;
    volatile float zero_yaw,yaw,rate,ref_rate,err,peak_yaw,peak_rate;
    volatile int16_t requested_pwm,pid_pm,ff_pm;
    volatile int32_t enc_l,enc_r;
    volatile uint32_t duration_ms,stop_tick,coast_wall_started_ms;
    PID_Handle_t pid_rate;
} TP_State;
static TP_State tp;
static MPU6050_Handle_t mpu;

/* Foreground-only diagnostic: measure distinct gyro publications, not
 * repeated reads of the same snapshot or downsampled UART frames. */
typedef struct {
    uint8_t active;
    uint32_t last_seq,last_completed_us,started_ms;
    uint32_t starts_notready,starts_error;
    uint32_t publications,intervals,sum_dt_us,min_dt_us,max_dt_us;
    uint32_t gap_gt1500,gap_gt2000,observer_missed;
} TP_ImuWindow;
static TP_ImuWindow imu_win;

static void ImuWindowReset(const MPU6050_Snapshot_t *sn,uint32_t now) {
    imu_win.active=1U;
    imu_win.last_seq=sn->sequence;
    imu_win.last_completed_us=sn->completed_us;
    imu_win.started_ms=now;
    imu_win.starts_notready=mpu.status_not_ready_reads;
    imu_win.starts_error=mpu.error_reads;
    imu_win.publications=0U;imu_win.intervals=0U;imu_win.sum_dt_us=0U;
    imu_win.min_dt_us=0xFFFFFFFFUL;imu_win.max_dt_us=0U;
    imu_win.gap_gt1500=0U;imu_win.gap_gt2000=0U;
    imu_win.observer_missed=0U;
}
static void ImuObserve(void) {
    MPU6050_Snapshot_t sn;
    uint32_t delta,dt;
    if(!imu_win.active || !MPU6050_GetSnapshot(&mpu,&sn))return;
    delta=(uint32_t)(sn.sequence-imu_win.last_seq);
    if(delta==0U)return;
    if(delta>1U)imu_win.observer_missed+=(delta-1U);
    imu_win.publications+=delta;
    dt=(uint32_t)(sn.completed_us-imu_win.last_completed_us);
    if(delta==1U && imu_win.last_completed_us!=0U) {
        ++imu_win.intervals;
        imu_win.sum_dt_us+=dt;
        if(dt<imu_win.min_dt_us)imu_win.min_dt_us=dt;
        if(dt>imu_win.max_dt_us)imu_win.max_dt_us=dt;
        if(dt>1500U)++imu_win.gap_gt1500;
        if(dt>2000U)++imu_win.gap_gt2000;
    }
    imu_win.last_seq=sn.sequence;
    imu_win.last_completed_us=sn.completed_us;
}
static void ImuEmit(uint32_t now,uint8_t final) {
    MPU6050_Snapshot_t sn;
    uint32_t elapsed,age=0xFFFFFFFFUL,hz10,avg_dt,min_dt;
    if(!imu_win.active)return;
    ImuObserve();
    elapsed=(uint32_t)(now-imu_win.started_ms);
    if(final && elapsed<50U && imu_win.publications==0U) {
        imu_win.active=0U;return;
    }
    if(elapsed==0U)return;
    hz10=(uint32_t)((((uint64_t)imu_win.publications)*10000ULL)/elapsed);
    avg_dt=imu_win.intervals?imu_win.sum_dt_us/imu_win.intervals:0U;
    min_dt=imu_win.intervals?imu_win.min_dt_us:0U;
    if(MPU6050_GetSnapshot(&mpu,&sn))
        age=(uint32_t)(micros()-sn.completed_us);
    /* <256-byte frame fits UART_Printf's fixed buffer.
     * Published completions may be lower than nominal 1000 Hz. */
    UART_Printf("TP,IMU,id=%lu,t=%lu,win=%lu,new=%lu,hz10=%lu,dtavg=%lu,dtmin=%lu,dtmax=%lu,g1500=%lu,g2000=%lu,miss=%lu,nr=%lu,err=%lu,age=%lu,seq=%lu,last=%u\r\n",
        (unsigned long)tp.id,(unsigned long)tp.ticks,
        (unsigned long)elapsed,(unsigned long)imu_win.publications,
        (unsigned long)hz10,(unsigned long)avg_dt,(unsigned long)min_dt,
        (unsigned long)imu_win.max_dt_us,(unsigned long)imu_win.gap_gt1500,
        (unsigned long)imu_win.gap_gt2000,
        (unsigned long)imu_win.observer_missed,
        (unsigned long)(mpu.status_not_ready_reads-imu_win.starts_notready),
        (unsigned long)(mpu.error_reads-imu_win.starts_error),
        (unsigned long)age,(unsigned long)imu_win.last_seq,(unsigned)final);
    if(final){imu_win.active=0U;return;}
    /* Keep last sequence/timestamp across reporting windows so a real
     * inter-window sampling delay is not lost. */
    imu_win.started_ms=now;
    imu_win.starts_notready=mpu.status_not_ready_reads;
    imu_win.starts_error=mpu.error_reads;
    imu_win.publications=0U;imu_win.intervals=0U;imu_win.sum_dt_us=0U;
    imu_win.min_dt_us=0xFFFFFFFFUL;imu_win.max_dt_us=0U;
    imu_win.gap_gt1500=0U;imu_win.gap_gt2000=0U;
    imu_win.observer_missed=0U;
}

static float Limit(float x,float a,float b) {
    return x<a?a:(x>b?b:x);
}
static float Sign(float x) {return x>0.0f?1.0f:(x<0.0f?-1.0f:0.0f);}
static uint32_t RunTimeout(uint8_t mode) {return mode==TP_SWEEP?2200U:(mode==TP_90?4000U:6500U);}
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
    Motor_GetAppliedPairPermille(&s->applied_l_pm,&s->applied_r_pm);
    s->stage=tp.stage;
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
    tp.coast_wall_started_ms=millis();
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
    float error,wanted,limit_brake,u,rate_ref,corrected_error;
    if(tp.phase!=TP_RUNNING && tp.phase!=TP_COAST)return;
    ++tp.ticks;
    tp.yaw=yaw_deg-tp.zero_yaw; /* unwrapped yaw; 180 does not alias -180 */
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
    if(tp.mode==TP_SWEEP) {
        if(tp.yaw>TP_SWEEP_MAX_YAW || fabsf(tp.rate)>TP_SWEEP_MAX_DPS) {
            StopAndCoast(TP_ANGLE);PushSample(age_us);return;
        }
        if(tp.ticks>=TP_SWEEP_N*TP_SWEEP_SEG_MS) {
            StopAndCoast(TP_DONE);PushSample(age_us);return;
        }
        if((tp.ticks%TP_CONTROL_DIV)==0U) {
            uint32_t step=(tp.ticks-1U)/TP_SWEEP_SEG_MS;
            uint32_t phase_t=(tp.ticks-1U)%TP_SWEEP_SEG_MS;
            int16_t req=phase_t<TP_SWEEP_ON_MS?tp_sweep_pwm[step]:0;
            tp.stage=(uint8_t)step;
            tp.requested_pwm=req;tp.pid_pm=0;tp.ff_pm=0;
            tp.ref_rate=0.0f;
            Motor_SetPairPermille(-req,req);
        }
        if((tp.ticks%TP_LOG_DIV)==0U)PushSample(age_us);
        return;
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
            /* Predict short look-ahead angle error for faster TUNED turns.
             * Legacy BASE/FAST have predict_s=0 and remain unchanged. */
            corrected_error=error-cfg->predict_s*tp.rate;
            limit_brake=sqrtf(2.0f*cfg->accel_dps2*fabsf(corrected_error));
            wanted=Sign(corrected_error)*Limit(cfg->kp_angle*fabsf(corrected_error),0.0f,
                                     Limit(cfg->speed_dps,0.0f,limit_brake));
            rate_ref=tp.ref_rate;
            rate_ref+=Limit(wanted-rate_ref,
                            -cfg->accel_dps2*0.005f,
                             cfg->accel_dps2*0.005f);
            rate_ref=Limit(rate_ref,-cfg->speed_dps,cfg->speed_dps);
            tp.ref_rate=rate_ref;
            tp.pid_pm=(int16_t)PID_Compute(&tp.pid_rate,rate_ref,tp.rate,0.005f);
            tp.ff_pm=(int16_t)(cfg->kff_rate*rate_ref);
            u=(float)tp.pid_pm+(float)tp.ff_pm;
            if(tp.profile==TP_TUNED) {
                /* Continuous near-zero compensation avoids the legacy
                 * discontinuous +/-95-permille clamp on PID sign changes.
                 * Actual breakaway must be measured in phase 1 first. */
                float blend=Limit(fabsf(rate_ref)/40.0f,0.0f,1.0f);
                u+=Sign(rate_ref)*cfg->static_ff*blend;
                tp.ff_pm=(int16_t)((float)tp.ff_pm+
                                  Sign(rate_ref)*cfg->static_ff*blend);
            }
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
    tp.stage=0U;
    tp.button_stop_consumed=0U;
    q_head=q_tail=0U;q_dropped=0U;
    if(!Motor_TurnDiagUnlockOutputs()) {
        tp.reason=TP_MOTOR;tp.phase=TP_RESULT;
    } else {
        tp.phase=TP_RUNNING;
        ImuWindowReset(&snap,millis());
    }
    if(!mask)__enable_irq();
    UART_Printf("TP,START,id=%lu,mode=%u,profile=%u,pwm_cap=%d,kp_angle_x100=%d,kp_rate_x100=%d,kff_x100=%d,accel=%u,max_rate=%u,log_hz=50,trace_rev=3,gyro_band_deg=2\r\n",
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
    if(c!='1' && c!='2' && c!='m' && c!='b' && c!='f' && c!='t')return;
    if(tp.phase!=TP_IDLE || !Motor_OutputsLocked()) {
        UART_SendString("TP,CONFIG_REJECT,reason=RUNNING\r\n");return;
    }
    if(c=='1')tp.mode=TP_90;
    if(c=='2')tp.mode=TP_180;
    if(c=='m')tp.mode=TP_SWEEP;
    if(c=='b')tp.profile=TP_BASE;
    if(c=='f')tp.profile=TP_FAST;
    if(c=='t')tp.profile=TP_TUNED;
    UART_Printf("TP,CONFIG,mode=%u,profile=%u,locked=1\r\n",
                (unsigned)tp.mode,(unsigned)tp.profile);
}
static void DrainSamples(void) {
    uint8_t budget=8U;
    while(budget-- && q_tail!=q_head) {
        TP_Sample s=q[q_tail];
        __DMB();q_tail=(uint16_t)((q_tail+1U)%TP_RING_SIZE);
        /* Rev3: add motor driver APPLIED PWM and motor-sweep stage.
         * ISR timestamp, never host arrival clock. */
        UART_Printf("TP,D,%lu,%lu,%u,%u,%u,%ld,%ld,%ld,%ld,%d,%d,%d,%d,%d,%ld,%ld,%lu,%u\r\n",
            (unsigned long)s.id,(unsigned long)s.t_ms,
            (unsigned)s.mode,(unsigned)s.profile,(unsigned)s.phase,
            (long)s.yaw_cd,(long)s.err_cd,(long)s.gyro_dps10,
            (long)s.ref_dps10,(int)s.pid_pm,(int)s.ff_pm,
            (int)s.cmd_pm,(int)s.applied_l_pm,(int)s.applied_r_pm,
            (long)s.encoder_l,(long)s.encoder_r,
            (unsigned long)s.age_us,(unsigned)s.stage);
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
    UART_SendString("TP,HELLO,turn_pid_diag_v3_pivot,no_oled=1,motor_locked=1\r\n");
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
    UART_SendString("TP,BOOT,mode=90,profile=0,commands=1/2/m/b/f/t/?/S,PA0_ONLY_START\r\n");
    for(;;) {
        uint32_t now=millis();
        MPU6050_Update(&mpu);
        HandleButton(now);
        ImuObserve(); /* foreground only, no additional I2C transaction */
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
        /* If the gyro remains stale after a locked stop, TIM11 cannot
         * collect the normal 800ms tail. Still emit a terminal result. */
        if(tp.phase==TP_COAST &&
           (uint32_t)(now-tp.coast_wall_started_ms)>=1100U) {
            uint32_t irq=__get_PRIMASK();__disable_irq();
            if(tp.phase==TP_COAST)tp.phase=TP_RESULT;
            if(!irq)__enable_irq();
        }
        if(imu_win.active && (uint32_t)(now-imu_win.started_ms)>=1000U)
            ImuEmit(now,0U);
        DrainSamples(); /* MUST precede RESULT to keep host trace ordered. */
        if(tp.phase==TP_RESULT && q_tail==q_head) {
            ImuEmit(now,1U); /* final partial window before TP,RESULT */
            EmitResult();
        }
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
