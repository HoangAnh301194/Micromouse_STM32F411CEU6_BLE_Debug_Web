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
static void DriveStep(uint32_t elapsed_ms)
{
#if defined(DIAG_SOAK_MOTOR_LIVE)
    uint32_t part = (elapsed_ms / 5000U) % 6U;
    /* Each five-second segment drives at most 2s, then COAST >=3s.
     * Includes equal-pair/individual 20-35% steps for EMI/supply comparison.
     * Do not use open-loop commands on the maze or with wheels grounded. */
    int16_t l=0, r=0;
    if ((elapsed_ms % 5000U) < 2000U) {
        if (part == 1U) { l=200; r=200; }
        else if (part == 2U) { l=300; r=300; }
        else if (part == 3U) { l=250; }
        else if (part == 4U) { r=250; }
        else if (part == 5U) { l=350; r=350; }
    }
    if (l || r) Motor_SetPairPermille(l,r);
    else Motor_Stop();
#else
    (void)elapsed_ms;
#endif
}
void Soak_Init(void)
{
    active=once=live_armed=aborted=0U;
    began_ms=next_sample_us=last_diag_ms=last_lease_ms=last_drive_ms=0U;
    last_heartbeat_ms=start_good=start_seq=start_stale=0U;
    base_gap=base_gyro_invalid=base_timeout=base_errors=base_drop=0U;
    pid_ticks=pid_maxcy=0U;
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
    next_sample_us=micros()+SOAK_RATE_US;
    last_lease_ms=now_ms;
    once=active=1U;
    aborted=0U;
    UART_Printf("SOAK_BEGIN,t=%lu,dur=180000,rate=60,live=%u,pid=%u,iclk=%lu,bias=%ld\r\n",
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
        UART_Printf("IM1,t=%lu,good=%lu,seq=%lu,stale=%lu,err=%lu,to=%lu,nr=%lu,xfer=%lu,ir=%lu,late=%lu,gap=%lu,irqcy=%lu,q=%lu,drop=%lu,rxerr=%lu\r\n",
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
            (unsigned long)(u.dropped_msgs-base_drop),
            (unsigned long)u.rx_errors);
        UART_Printf("IMN,t=%lu,n=%lu,mean=%ld,std=%ld,bias=%ld,rawmin=%ld,rawmax=%ld,gmin=%lu,gmax=%lu,eq=%lu\r\n",
            (unsigned long)elapsed,(unsigned long)n.n,
            (long)FixedMilli(n.mean_corrected_dps),
            (long)FixedMilli(n.std_dps),
            (long)FixedMilli(mpu->gyro_z_offset),
            (long)FixedMilli(n.min_dps),
            (long)FixedMilli(n.max_dps),
            (unsigned long)n.interval_min_us,
            (unsigned long)n.interval_max_us,
            (unsigned long)n.equal_consecutive_raw);
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
