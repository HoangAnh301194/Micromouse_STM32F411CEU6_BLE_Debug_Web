/* 180-second Micromouse IMU/I2C and integrated workload study.
 * Uses real MPU gyro, IR ADC emitter scans, encoder timers and UART DMA.
 * SHADOW: outputs permanently locked. LIVE: deliberately limited <=35%
 * PWM wheel-lift test, explicit host consent and heartbeat. No maze motion.
 * Fake PID follows a synthetic first-order plant: NEVER drives a motor.
 * No UART formatting, I2C, float print, or slow operations in TIM11 ISR. */
#include "soak_diag.h"
#if defined(DIAG_SOAK_180S)
#include "board.h"
#include "control.h"
#include "encoder.h"
#include "i2c.h"
#include "integration_sweep.h"
#include "ir_sensor.h"
#include "motor.h"
#include "pid.h"
#include "system_timer.h"
#include "uart.h"
#include "stm32f4xx.h"

#if defined(DIAG_SOAK_HIGH_PWM) && !defined(DIAG_SOAK_MOTOR_LIVE)
#error "High duty requires explicit LIVE diagnostic profile"
#endif
#if defined(DIAG_SOAK_MOTOR_LIVE) && (!defined(DIAG_SOAK_PID) || !defined(INTEGRATION_DIAG_MOTOR_ON))
#error "LIVE soak needs PID stress and motor-ON diagnostic image"
#endif
#if !defined(INTEGRATION_DIAG)
#error "SOAK requires instrumented firmware"
#endif

#define SOAK_DURATION_MS 180000UL
#define SOAK_RATE_US 16667UL
#define SOAK_HEARTBEAT_TIMEOUT_MS 750UL
#define SOAK_MOTOR_RENEW_MS 40UL
static volatile uint8_t active, once, live_armed, aborted;
#if defined(DIAG_SOAK_MOTOR_LIVE)
static uint32_t drive_stage_index, drive_start_ms, drive_last_report_ms;
static int32_t drive_start_enc_l, drive_start_enc_r;
static uint8_t drive_stage_checked;
static int16_t drive_target_l, drive_target_r;
#endif
static uint32_t began_ms, next_sample_us, last_diag_ms, last_lease_ms, last_drive_ms;
static uint32_t last_heartbeat_ms, start_good, start_seq, start_stale;
static uint32_t base_gap, base_gyro_invalid, base_timeout, base_errors, base_drop;
static volatile uint32_t pid_ticks, pid_maxcy;
static volatile float plant, pid_last_error, pid_last_command;
#if defined(DIAG_SOAK_PID)
static PID_Handle_t fake_pid;
#endif
static int32_t FixedMilli(float x)
{
    if (x > 1000000.0f) return 1000000000L;
    if (x < -1000000.0f) return -1000000000L;
    return (int32_t)(x * 1000.0f + (x >= 0 ? 0.5f : -0.5f));
}
#if defined(DIAG_SOAK_MOTOR_LIVE)
/* Actual PWM drive. Single-sided and both-wheel stages; no reversal.
 * HIGH uses short peaks only and keeps >4s cooling/coasting per slot.
 * At 100% the motor remains energized for at most 500ms in a 30s cycle. */
static void DriveStep(uint32_t elapsed_ms)
{
    uint32_t stage=elapsed_ms/5000U,part=stage%6U,within=elapsed_ms%5000U;
    uint32_t duration=2000U;
    int16_t l=0,r=0,applied_l=0,applied_r=0;
    int32_t dl,dr;
#if defined(DIAG_SOAK_HIGH_PWM)
    if (part>=1U && part<=3U)duration=1000U;
    else if (part==4U)duration=750U;
    else if (part==5U)duration=500U;
#endif
    if (stage!=drive_stage_index) {
        drive_stage_index=stage;
        drive_start_ms=elapsed_ms-within;
        drive_start_enc_l=Encoder_GetLeftCount();
        drive_start_enc_r=Encoder_GetRightCount();
        drive_stage_checked=0U;
    }
    if (within<duration) {
#if defined(DIAG_SOAK_HIGH_PWM)
        if (part==1U)l=800;
        else if (part==2U)r=800;
        else if (part==3U)l=r=800;
        else if (part==4U)l=r=900;
        else if (part==5U)l=r=1000;
#else
        if (part==1U)l=r=200;
        else if (part==2U)l=r=300;
        else if (part==3U)l=250;
        else if (part==4U)r=250;
        else if (part==5U)l=r=350;
#endif
    }
    drive_target_l=l;
    drive_target_r=r;
    if (l||r)Motor_SetPairPermille(l,r);
    else Motor_Stop();

    /* Detect disconnected/locked/stalled encoder response while high PWM
     * remains commanded. At least 10 counts per driven wheel by 300ms.
     * Fail closed immediately; this is NOT a current/thermal monitor. */
    if (!drive_stage_checked && within>=300U && within<duration &&
        (l!=0||r!=0)) {
        dl=Encoder_GetLeftCount()-drive_start_enc_l;
        dr=Encoder_GetRightCount()-drive_start_enc_r;
        if ((l && (dl<10 && dl>-10)) || (r && (dr<10 && dr>-10))) {
            UART_Printf("IMDRV_FAULT,t=%lu,stage=%lu,l=%d,r=%d,dl=%ld,dr=%ld\r\n",
                  (unsigned long)elapsed_ms,(unsigned long)stage,
                  (int)l,(int)r,(long)dl,(long)dr);
            Soak_Stop(5U);
            return;
        }
        drive_stage_checked=1U;
    }
    if ((uint32_t)(elapsed_ms-drive_last_report_ms)>=250U) {
        drive_last_report_ms=elapsed_ms;
        Motor_GetAppliedPairPermille(&applied_l,&applied_r);
        UART_Printf("IMDRV,t=%lu,stage=%lu,part=%lu,setL=%d,setR=%d,appL=%d,appR=%d,encL=%ld,encR=%ld\r\n",
            (unsigned long)elapsed_ms,(unsigned long)stage,
            (unsigned long)part,(int)l,(int)r,(int)applied_l,(int)applied_r,
            (long)Encoder_GetLeftCount(),(long)Encoder_GetRightCount());
    }
}
#endif

void Soak_Init(void)
{
    active=once=live_armed=aborted=0U;
    began_ms=next_sample_us=last_diag_ms=last_lease_ms=last_drive_ms=0U;
    last_heartbeat_ms=start_good=start_seq=start_stale=0U;
    base_gap=base_gyro_invalid=base_timeout=base_errors=base_drop=0U;
    pid_ticks=pid_maxcy=0U;
#if defined(DIAG_SOAK_MOTOR_LIVE)
    drive_stage_index=0xFFFFFFFFUL;
    drive_start_ms=drive_last_report_ms=0U;
    drive_start_enc_l=drive_start_enc_r=0;
    drive_stage_checked=0U;
    drive_target_l=drive_target_r=0;
#endif
    plant=pid_last_error=pid_last_command=0.0f;
#if defined(DIAG_SOAK_PID)
    PID_Init(&fake_pid, 0.9f, 0.3f, 0.001f, 70.0f, 50.0f);
#endif
}
uint8_t Soak_IsActive(void) { return active; }
uint8_t Soak_HasStarted(void) { return once; }
uint8_t Soak_ArmLive(const MPU6050_Handle_t *mpu)
{
#if defined(DIAG_SOAK_MOTOR_LIVE)
    if (once || live_armed || Sweep_IsActive() || !mpu ||
        mpu->cal_state != 3U || Motor_OutputsLocked()) return 0U;
    live_armed=1U;
    return 1U;
#else
    (void)mpu;
    return 0U;
#endif
}
uint8_t Soak_Start(MPU6050_Handle_t *mpu, uint32_t now_ms)
{
    MPU6050_Snapshot_t s;
    Control_DebugStats_t t;
    if (once || active || Sweep_IsActive() || !mpu ||
        mpu->cal_state != 3U || !mpu->calibrated ||
        Board_GetClockFault() || !Control_IsRunning() ||
        !MPU6050_GetSnapshot(mpu,&s) || !s.valid ||
        (uint32_t)(micros()-s.completed_us) > 3000U) return 0U;
#if defined(DIAG_SOAK_MOTOR_LIVE)
    if (!live_armed || Motor_OutputsLocked()) return 0U;
#else
    if (!Motor_OutputsLocked()) return 0U;
#endif
    MPU6050_ResetYaw(mpu);
    Control_GetDebugStats(&t);
    base_gap=t.gaps_gt1500us;
    base_gyro_invalid=t.gyro_invalid_after_first_valid;
    start_stale=t.gyro_invalid_after_first_valid;
    base_timeout=mpu->timeout_reads;
    base_errors=mpu->error_reads;
    base_drop=UART_GetDroppedMessages();
    start_good=mpu->fresh_gyro_reads;
    start_seq=s.sequence;
    began_ms=last_diag_ms=last_drive_ms=last_heartbeat_ms=now_ms;
    UART_ResetProfileQueuePeak();
#if defined(DIAG_SOAK_MOTOR_LIVE)
    drive_stage_index=0xFFFFFFFFUL;
    drive_last_report_ms=0U;
#endif
    next_sample_us=micros()+SOAK_RATE_US;
    last_lease_ms=now_ms;
    once=active=1U;
    aborted=0U;
    UART_Printf("SOAK_BEGIN,t=%lu,dur=180000,rate=60,live=%u,pid=%u,high=%u,iclk=%lu,bias=%ld\r\n",
                (unsigned long)now_ms,
#if defined(DIAG_SOAK_MOTOR_LIVE)
                1U,
#else
                0U,
#endif
#if defined(DIAG_SOAK_PID)
                1U,
#else
                0U,
#endif
#if defined(DIAG_SOAK_HIGH_PWM)
                1U,
#else
                0U,
#endif
                (unsigned long)I2C_GetConfiguredSclHz(),
                (long)FixedMilli(mpu->gyro_z_offset));
    /* Explain I2C startup fallback: 0=400k, 1=300k, 2=200k, 3=100k.
     * 255 in who field means rate was not reached due to address NACK. */
    for (uint32_t i=0U;i<4U;i++)
        UART_Printf("I2CP,id=%lu,a=%u,w=%u,v=%u,selected=%u\r\n",
                    (unsigned long)i,
                    (unsigned)mpu->probe_address_status[i],
                    (unsigned)mpu->probe_who_status[i],
                    (unsigned)mpu->probe_who_value[i],
                    (unsigned)mpu->selected_speed_index);
    return 1U;
}
void Soak_Heartbeat(uint32_t now_ms)
{
    if (active) last_heartbeat_ms=now_ms;
}
void Soak_Stop(uint8_t reason)
{
    uint32_t span;
    if (!once || !active) return;
    active=0U;
    aborted=(uint8_t)(reason!=0U);
    span=millis()-began_ms;
#if defined(DIAG_SOAK_MOTOR_LIVE)
    Motor_LockOutputs();
#endif
    UART_Printf("SOAK_DONE,ok=%u,reason=%u,ms=%lu,seq=%lu,good=%lu\r\n",
                (unsigned)(reason==0U),(unsigned)reason,(unsigned long)span,
                (unsigned long)start_seq,(unsigned long)start_good);
}
void Soak_PidTick(void)
{
#if defined(DIAG_SOAK_PID)
    uint32_t before,cy;
    float setpoint,output;
    if (!active) return;
    before=DWT->CYCCNT;
    setpoint=(((pid_ticks/1000U) % 4U) < 2U) ? 40.0f : -40.0f;
    output=PID_Compute(&fake_pid,setpoint,plant,0.001f);
    plant += (output - plant) * 0.008f; /* synthetic first-order plant */
    pid_last_error=setpoint-plant;
    pid_last_command=output;
    pid_ticks++;
    cy=DWT->CYCCNT-before;
    if (cy>pid_maxcy) pid_maxcy=cy;
#endif
}
void Soak_Poll(MPU6050_Handle_t *mpu, uint32_t now_ms)
{
    uint32_t elapsed, now_us;
    MPU6050_Snapshot_t s;
    if (!active || !mpu) return;
    elapsed=(uint32_t)(now_ms-began_ms);
    if (elapsed >= SOAK_DURATION_MS) {
        Soak_Stop(0U);
        return;
    }
#if defined(DIAG_SOAK_MOTOR_LIVE)
    if ((uint32_t)(now_ms-last_heartbeat_ms)>SOAK_HEARTBEAT_TIMEOUT_MS ||
        Motor_OutputsLocked()) { Soak_Stop(1U); return; }
    if (mpu->timeout_reads!=base_timeout ||
        mpu->error_reads!=base_errors) { Soak_Stop(2U); return; }
    {
        Control_DebugStats_t t;
        Control_GetDebugStats(&t);
        if (t.gaps_gt1500us!=base_gap ||
            t.gyro_invalid_after_first_valid!=base_gyro_invalid) {
            Soak_Stop(3U); return;
        }
    }
    if ((uint32_t)(now_ms-last_lease_ms)>=SOAK_MOTOR_RENEW_MS) {
        last_lease_ms=now_ms;
        DriveStep(elapsed);
        if (!active) return; /* encoder no-response fail-closed */
    }
#endif
    now_us=micros();
    if ((int32_t)(now_us-next_sample_us)>=0) {
        uint32_t age;
        next_sample_us+=SOAK_RATE_US;
        /* No catch-up burst. Skipped samples are detected on PC by time/seq. */
        if ((int32_t)(now_us-next_sample_us)>0) next_sample_us=now_us+SOAK_RATE_US;
        if (MPU6050_GetSnapshot(mpu,&s)) {
            age=(uint32_t)(now_us-s.completed_us);
            UART_Printf("IM6,t=%lu,s=%lu,y=%ld,n=%ld,r=%ld,z=%ld,a=%lu\r\n",
                        (unsigned long)elapsed,(unsigned long)s.sequence,
                        (long)FixedMilli(s.yaw_deg),
                        (long)FixedMilli(mpu->yaw_no_deadzone),
                        (long)FixedMilli(mpu->yaw_raw),
                        (long)FixedMilli(s.gyro_z_dps),
                        (unsigned long)age);
        }
    }
    if ((uint32_t)(now_ms-last_diag_ms)>=1000U) {
        Control_DebugStats_t t;
        UART_ProfileStats_t u;
        MPU6050_NoiseStats_t n;
        IR_Data_t ir;
        uint32_t ps,pc;
        int32_t perr,pout;
        int16_t left=0,right=0;
        last_diag_ms+=1000U;
        Control_GetDebugStats(&t);
        UART_GetProfileStats(&u);
        MPU6050_GetNoiseWindow(mpu,&n);
        Control_GetIRSnapshot(&ir);
        Motor_GetAppliedPairPermille(&left,&right);
        UART_Printf("IM1,t=%lu,good=%lu,seq=%lu,stale=%lu,err=%lu,to=%lu,nr=%lu,xfer=%lu,ir=%lu,late=%lu,gap=%lu,irqcy=%lu,q=%lu,qp=%lu,drop=%lu,rxerr=%lu\r\n",
            (unsigned long)elapsed,
            (unsigned long)(mpu->fresh_gyro_reads-start_good),
            (unsigned long)mpu->published_sequence,
            (unsigned long)(t.gyro_invalid_after_first_valid-start_stale),
            (unsigned long)(mpu->error_reads-base_errors),
            (unsigned long)(mpu->timeout_reads-base_timeout),
            (unsigned long)mpu->status_not_ready_reads,
            (unsigned long)mpu->max_transfer_us,
            (unsigned long)t.ir_scan_count,
            (unsigned long)(t.late_gt50us),
            (unsigned long)(t.gaps_gt1500us-base_gap),
            (unsigned long)t.max_irq_cycles,
            (unsigned long)u.queued_bytes,
            (unsigned long)u.queue_peak,
            (unsigned long)(u.dropped_msgs-base_drop),
            (unsigned long)u.rx_errors);
        UART_Printf("IMN,t=%lu,n=%lu,mean=%ld,std=%ld,bias=%ld,rawmin=%ld,rawmax=%ld,gmin=%lu,gmax=%lu,eq=%lu,tc=%ld,tvalid=%u,tok=%lu,terr=%lu,tto=%lu,tage=%lu\r\n",
            (unsigned long)elapsed,(unsigned long)n.n,
            (long)FixedMilli(n.mean_corrected_dps),
            (long)FixedMilli(n.std_dps),
            (long)FixedMilli(mpu->gyro_z_offset),
            (long)FixedMilli(n.min_dps),
            (long)FixedMilli(n.max_dps),
            (unsigned long)n.interval_min_us,
            (unsigned long)n.interval_max_us,
            (unsigned long)n.equal_consecutive_raw,
            (long)mpu->temperature_mc,
            (unsigned)mpu->temperature_valid,
            (unsigned long)mpu->temperature_reads,
            (unsigned long)mpu->temperature_errors,
            (unsigned long)mpu->temperature_timeouts,
            (unsigned long)(now_ms-mpu->temperature_last_ms));
        UART_Printf("IMIR,t=%lu,c=%lu,s0=%u,s1=%u,s2=%u,s3=%u,s4=%u,s5=%u\r\n",
             (unsigned long)elapsed,(unsigned long)ir.scan_count,
             (unsigned)ir.value[0],(unsigned)ir.value[1],(unsigned)ir.value[2],
             (unsigned)ir.value[3],(unsigned)ir.value[4],(unsigned)ir.value[5]);
        { uint32_t mask=__get_PRIMASK();__disable_irq();
          ps=pid_ticks;pc=pid_maxcy;
          perr=FixedMilli(pid_last_error);pout=FixedMilli(pid_last_command);
          if (!mask) __enable_irq(); }
        UART_Printf("IMPID,t=%lu,n=%lu,maxcy=%lu,e=%ld,u=%ld,cmdL=%d,cmdR=%d,encL=%ld,encR=%ld\r\n",
            (unsigned long)elapsed,(unsigned long)ps,(unsigned long)pc,
            (long)perr,(long)pout,(int)left,(int)right,
            (long)Encoder_GetLeftCount(),(long)Encoder_GetRightCount());
    }
}
#endif
