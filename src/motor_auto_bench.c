/* Three-round, no-KEY, host-armed PWM/encoder characterization.
 * INVARIANT: never drive on boot/reset; an explicit USART1 "MOTOR START"
 * is required, and each pulse is interrupted by the 1kHz MotorTest watchdog.
 * This does not implement current limiting / TB6612 STBY hardware cutoff.
 */
#include "board.h"
#include "control.h"
#include "encoder.h"
#include "motor.h"
#include "motor_test.h"
#include "pinout.h"
#include "system_timer.h"
#include "uart.h"
#if defined(MOTOR_AUTO_SENSOR_INTEGRATION)
#include "mpu6050.h"
#include "ir_sensor.h"
#endif
#include "stm32f4xx.h"
#include <string.h>

#if defined(MOTOR_AUTO_BENCH)
#if defined(MOTOR_PWM_DIAG) || defined(MPU_FRESHNESS_DIAG)
#error "MOTOR_AUTO_BENCH cannot run KEY/FRESHNESS diagnostic simultaneously"
#endif
#if defined(MOTOR_AUTO_SENSOR_INTEGRATION)
#if !defined(MPU_INTEGRATION_BENCH) || !defined(MPU_SAMPLING_POLICY_B)
#error "Integrated motor bench requires MPU policy B and integration statistics"
#endif
#else
#if defined(MPU_INTEGRATION_BENCH) || defined(MPU_SAMPLING_POLICY_B)
#error "Motor-only bench must not silently include IMU"
#endif
#endif

#define AUTO_ROUNDS          3U
#if defined(MOTOR_AUTO_HIGH_TEST)
/* Deliberately separate opt-in firmware: regular v4 bench is unchanged.
 * 8 more short PWM stages, 30/40/50/60%, single-wheel only. */
#define AUTO_STEPS          29U
#define AUTO_MAX_PERMILLE    600U
#if defined(MOTOR_AUTO_SENSOR_INTEGRATION)
#define AUTO_VERSION         8U
#else
#define AUTO_VERSION         6U
#endif
#define AUTO_FAMILIES        5U
#define AUTO_HIGH_FIRST     21U
#define AUTO_HIGH_PULSE_MS 120U
#define AUTO_HIGH_POST_MS  800U
#else
#define AUTO_STEPS          21U
#define AUTO_MAX_PERMILLE    200U
#if defined(MOTOR_AUTO_SENSOR_INTEGRATION)
#define AUTO_VERSION         7U
#else
#define AUTO_VERSION         4U
#endif
#define AUTO_FAMILIES        4U
#endif
#define AUTO_BOTH_PERMILLE   160U
#define AUTO_PULSE_MS        200U
#define AUTO_POST_MS         400U
#define AUTO_SETTLE_MS      1000U
#define AUTO_STAGE_GAP_MS    250U
#define AUTO_SAMPLE_MS       20U
#define AUTO_HOST_TIMEOUT_MS 1600U

/* Four diagnostics in ONE flash/session:
 * startup threshold at 8-12%, full response at 12/16/20%,
 * simultaneous 16%, and rise-slew sensitivity at 16%.
 * No polarity reversal, motor electrical braking or intentional stall. */
typedef enum {
    FAMILY_STARTUP=0, FAMILY_RESPONSE=1,
    FAMILY_SLEW_SLOW=2, FAMILY_SLEW_FAST=3, FAMILY_HIGH=4
} AutoFamily_t;
typedef struct { int16_t l, r; uint8_t family; uint8_t rise; } AutoPulse_t;
static const char * const family_names[] = {
    "startup", "response", "slew_slow", "slew_fast", "high"
};
static const AutoPulse_t matrix[AUTO_STEPS] = {
    { 80,  0,FAMILY_STARTUP,10}, {  0, 80,FAMILY_STARTUP,10},
    { 90,  0,FAMILY_STARTUP,10}, {  0, 90,FAMILY_STARTUP,10},
    {100,  0,FAMILY_STARTUP,10}, {  0,100,FAMILY_STARTUP,10},
    {110,  0,FAMILY_STARTUP,10}, {  0,110,FAMILY_STARTUP,10},
    {120,  0,FAMILY_STARTUP,10}, {  0,120,FAMILY_STARTUP,10},
    {120,  0,FAMILY_RESPONSE,10},{  0,120,FAMILY_RESPONSE,10},
    {160,  0,FAMILY_RESPONSE,10},{  0,160,FAMILY_RESPONSE,10},
    {200,  0,FAMILY_RESPONSE,10},{  0,200,FAMILY_RESPONSE,10},
    {160,160,FAMILY_RESPONSE,10},
    {160,  0,FAMILY_SLEW_SLOW, 5}, {0,160,FAMILY_SLEW_SLOW, 5},
    {160,  0,FAMILY_SLEW_FAST,20}, {0,160,FAMILY_SLEW_FAST,20}
#if defined(MOTOR_AUTO_HIGH_TEST)
    ,{300,  0,FAMILY_HIGH,10}, {  0,300,FAMILY_HIGH,10}
    ,{400,  0,FAMILY_HIGH,10}, {  0,400,FAMILY_HIGH,10}
    ,{500,  0,FAMILY_HIGH,10}, {  0,500,FAMILY_HIGH,10}
    ,{600,  0,FAMILY_HIGH,10}, {  0,600,FAMILY_HIGH,10}
#endif
};

typedef enum {
    AUTO_WAIT=0, AUTO_SETTLE, AUTO_DRIVE, AUTO_POST,
    AUTO_GAP, AUTO_DONE, AUTO_FAULT
} AutoState_t;

static AutoState_t state = AUTO_WAIT;
static uint8_t round_i = 0, step_i = 0;
static uint32_t state_since_ms = 0;
static uint32_t sample_since_ms = 0;
static uint32_t last_host_ms = 0;
static uint32_t last_tick_gaps = 0;
static uint32_t last_tick_seen = 0U;
static uint32_t last_tick_seen_ms = 0U;
static uint32_t completed_pulses = 0;
static uint32_t keep_ok = 0U;
static uint32_t bad_rx_commands = 0U;
#if defined(MOTOR_AUTO_SENSOR_INTEGRATION)
static MPU6050_Handle_t integration_mpu;
static uint8_t sensors_ready = 0U;
static uint32_t ir_scan_count = 0U;
static uint32_t ir_last_progress_ms = 0U;
static void SensorReport(const char *phase)
{
    Control_IntegrationStats_t stats;
    Control_GetIntegrationStats(&stats);
    UART_Printf("MOTOR_AUTO_SENSORS,round=%u,stage=%u,phase=%s,"
                "good=%lu,fresh=%lu,err=%lu,timeout=%lu,busy=%lu,"
                "reused=%lu,aged=%lu,missing=%lu,ir=%lu\r\n",
                (unsigned)round_i,(unsigned)step_i,phase,
                (unsigned long)integration_mpu.good_reads,
                (unsigned long)integration_mpu.fresh_gyro_reads,
                (unsigned long)integration_mpu.error_reads,
                (unsigned long)integration_mpu.timeout_reads,
                (unsigned long)integration_mpu.busy_retries,
                (unsigned long)stats.reused,
                (unsigned long)stats.age_over1500,
                (unsigned long)stats.missing_or_expired,
                (unsigned long)ir_scan_count);
}
#endif
#if defined(MOTOR_AUTO_HIGH_TEST)
static int32_t high_post_ref_l = 0, high_post_ref_r = 0;
static uint8_t high_post_ref_valid = 0U;
#endif
static uint32_t PulseMs(void)
{
#if defined(MOTOR_AUTO_HIGH_TEST)
    if (step_i >= AUTO_HIGH_FIRST) return AUTO_HIGH_PULSE_MS;
#endif
    return AUTO_PULSE_MS;
}
static uint32_t PostMs(void)
{
#if defined(MOTOR_AUTO_HIGH_TEST)
    if (step_i >= AUTO_HIGH_FIRST) return AUTO_HIGH_POST_MS;
#endif
    return AUTO_POST_MS;
}
static uint8_t rx_count = 0;
static char rx_line[32];

static void EmitSample(uint32_t now, uint32_t elapsed, const char *phase)
{
    Control_Timing_t t;
    int16_t al=0, ar=0;
    Motor_GetAppliedPairPermille(&al,&ar);
    Control_GetTiming(&t);
    const AutoPulse_t *p = &matrix[step_i];
    UART_Printf("MOTOR_AUTO_SAMPLE,round=%u,stage=%u,phase=%s,"
                "test=%s,rise=%u,ms=%lu,elapsed=%lu,"
                "applied_l=%d,applied_r=%d,enc_l=%ld,enc_r=%ld,"
                "ccr_l=%lu,ccr_r=%lu,tim11_late=%lu,"
                "tim11_max_cycles=%lu,tim11_gaps=%lu,tx_drop=%lu\r\n",
                (unsigned)round_i,(unsigned)step_i,phase,
                family_names[p->family],(unsigned)p->rise,
                (unsigned long)now,(unsigned long)elapsed,(int)al,(int)ar,
                (long)Encoder_GetLeftCount(),(long)Encoder_GetRightCount(),
                (unsigned long)TIM2->CCR2,(unsigned long)TIM2->CCR1,
                (unsigned long)t.late_over_50us,
                (unsigned long)t.exec_max_cycles,
                (unsigned long)t.gap_over_1500us,
                (unsigned long)UART_GetDroppedMessages());
}

static void FailSafe(const char *reason)
{
    if (state == AUTO_FAULT || state == AUTO_DONE) return;
    MotorTest_Stop();
    Motor_Stop();
    state = AUTO_FAULT;
    UART_Printf("MOTOR_AUTO_FAULT,reason=%s,round=%u,stage=%u,"
                "completed=%lu,ccr_l=%lu,ccr_r=%lu,"
                "host_age_ms=%lu,keep_ok=%lu,rx_bytes=%lu,"
                "rx_ore=%lu,rx_fe=%lu,rx_ring_drop=%lu\r\n",
                reason,(unsigned)round_i,(unsigned)step_i,
                (unsigned long)completed_pulses,
                (unsigned long)TIM2->CCR2,(unsigned long)TIM2->CCR1,
                (unsigned long)(millis()-last_host_ms),
                (unsigned long)keep_ok,
                (unsigned long)UART_GetRxByteCount(),
                (unsigned long)UART_GetRxOverrunCount(),
                (unsigned long)UART_GetRxFramingCount(),
                (unsigned long)UART_GetRxRingOverflowCount());
}

static void StartStep(uint32_t now)
{
    const AutoPulse_t *p = &matrix[step_i];
    /* Keep cap in MCU independently of the host-side CSV validator. */
    if (p->l < 0 || p->r < 0 ||
        p->l > (int16_t)AUTO_MAX_PERMILLE ||
        p->r > (int16_t)AUTO_MAX_PERMILLE ||
        (p->l > 0 && p->r > 0 &&
         (p->l > (int16_t)AUTO_BOTH_PERMILLE ||
          p->r > (int16_t)AUTO_BOTH_PERMILLE)) ||
        p->family >= AUTO_FAMILIES ||
        p->rise < 5U || p->rise > 20U) {
        FailSafe("DUTY_CAP");
        return;
    }
    Motor_SetDeadband(0U, 0U);
    Motor_SetSlewRates(p->rise, 20U);
    if (!MotorTest_StartPermille(p->l,p->r,PulseMs(),100U)) {
        FailSafe("START_REJECTED");
        return;
    }
    state = AUTO_DRIVE;
    state_since_ms = now;
    sample_since_ms = now;
    UART_Printf("MOTOR_AUTO_START,round=%u,stage=%u,l=%d,r=%d,"
                "test=%s,rise=%u,pulse_ms=%u\r\n",
                (unsigned)round_i,(unsigned)step_i,
                (int)p->l,(int)p->r,
                family_names[p->family],(unsigned)p->rise,
                (unsigned)PulseMs());
}

static void ReplyReady(void)
{
    /* Repeat complete profile with each PING: host may connect after MCU
     * boot/flash and must still capture the exact firmware signature. */
    UART_Printf("MOTOR_AUTO_PROFILE,version=%u,nominal_hz=%lu,"
                "pwm_hz=%lu,rounds=%u,steps=%u,hard_cap_permille=%u,"
                "pulse_ms=200,post_ms=400,boot_neutral=1,"
                "both_cap_permille=160,families=%u,rx_mode=irq256,"
                "high_pulse_ms=%u,high_post_ms=%u,sensor=%u\r\n",
                (unsigned)AUTO_VERSION,(unsigned long)SystemCoreClock,
                (unsigned long)(Board_GetAPB1TimerClockHz()/(TIM2->ARR+1U)),
                (unsigned)AUTO_ROUNDS,(unsigned)AUTO_STEPS,
                (unsigned)AUTO_MAX_PERMILLE,(unsigned)AUTO_FAMILIES,
#if defined(MOTOR_AUTO_HIGH_TEST)
                (unsigned)AUTO_HIGH_PULSE_MS,(unsigned)AUTO_HIGH_POST_MS
#else
                0U,0U
#endif
#if defined(MOTOR_AUTO_SENSOR_INTEGRATION)
                ,1U
#else
                ,0U
#endif
                );
#if defined(MOTOR_AUTO_SENSOR_INTEGRATION)
    if (!sensors_ready) {
        UART_Printf("MOTOR_AUTO_WAIT_SENSORS,cal=%u,state=%u,good=%lu\r\n",
                    (unsigned)integration_mpu.cal_state,(unsigned)state,
                    (unsigned long)integration_mpu.good_reads);
        return;
    }
#endif
#if defined(MOTOR_AUTO_SENSOR_INTEGRATION)
    UART_Printf("MOTOR_AUTO_SENSOR_READY,good=%lu,ir=%lu,bias_mdps=%ld\r\n",
                (unsigned long)integration_mpu.good_reads,
                (unsigned long)ir_scan_count,
                (long)(integration_mpu.gyro_z_offset * 1000.0f));
#endif
    UART_Printf("MOTOR_AUTO_READY,version=%u,rounds=%u,steps=%u,"
                "hard_cap_permille=%u,both_cap_permille=160,pulse_ms=200,post_ms=400,"
                "families=%u,rx_mode=irq256,one_shot=1,boot_neutral=1,"
                "rx=USART1_PA10,state=%u,sensor=%u\r\n",
                (unsigned)AUTO_VERSION,(unsigned)AUTO_ROUNDS,
                (unsigned)AUTO_STEPS,(unsigned)AUTO_MAX_PERMILLE,
                (unsigned)AUTO_FAMILIES,(unsigned)state,
#if defined(MOTOR_AUTO_SENSOR_INTEGRATION)
                1U
#else
                0U
#endif
                );
}

static void ReceiveLine(const char *line, uint32_t now)
{
    if (strcmp(line,"MOTOR PING") == 0) {
        ReplyReady();
    } else if (strcmp(line,"MOTOR KEEP") == 0) {
        if (state >= AUTO_SETTLE && state <= AUTO_GAP) {
            last_host_ms = now;
            ++keep_ok;
        }
    } else if (strcmp(line,"MOTOR STOP") == 0) {
        FailSafe("HOST_STOP");
#if defined(MOTOR_AUTO_HIGH_TEST)
    } else if (strcmp(line,"MOTOR START HIGH 600") == 0 && state == AUTO_WAIT) {
#else
    } else if (strcmp(line,"MOTOR START") == 0 && state == AUTO_WAIT) {
#endif
        last_host_ms = now;
        state = AUTO_SETTLE;
        state_since_ms = now;
        round_i = 0U;
        step_i = 0U;
        completed_pulses = 0U;
        keep_ok = 0U;
        bad_rx_commands = 0U;
        Motor_Stop();
        UART_Printf("MOTOR_AUTO_BEGIN,rounds=%u,steps=%u,"
                    "settle_ms=%u,boot_neutral=1\r\n",
                    (unsigned)AUTO_ROUNDS,(unsigned)AUTO_STEPS,
                    (unsigned)AUTO_SETTLE_MS);
    } else {
        ++bad_rx_commands;
    }
}

static void PollHost(uint32_t now)
{
    uint8_t ch;
    uint8_t budget = 64U;
    while (budget-- && UART_TryReadByte(&ch)) {
        if (ch == '\r') continue;
        if (ch == '\n') {
            rx_line[rx_count] = '\0';
            if (rx_count > 0U) ReceiveLine(rx_line,now);
            rx_count = 0U;
        } else if (ch >= 32U && ch < 127U && rx_count < sizeof(rx_line)-1U) {
            rx_line[rx_count++] = (char)ch;
        } else {
            rx_count = 0U;
        }
    }
}

int MotorAutoBench_Main(void)
{
    Board_Init();
    SystemTimer_Init();
    UART_Init();
    if (Board_GetClockFault() != 0U) {
        UART_Printf("MOTOR_AUTO_FAULT,reason=CLOCK_INIT_%lu\r\n",
                    (unsigned long)Board_GetClockFault());
        while (1) { Board_StatusLed(1U); }
    }

    Motor_Init();
    Motor_Stop();
    Encoder_Init();
#if defined(MOTOR_AUTO_SENSOR_INTEGRATION)
    /* No motor arming until MPU initialized/calibrated + IR scans.
     * Keep RC2 nonblocking acquisition in main, never I2C in TIM11. */
    {
        MPU6050_Status r = MPU6050_Init(&integration_mpu);
        if (r != MPU6050_OK) {
            UART_Printf("MOTOR_AUTO_FAULT,reason=IMU_INIT_%u\r\n",(unsigned)r);
            while (1) { Board_StatusLed(1U); }
        }
        MPU6050_Calibrate(&integration_mpu);
    }
    IR_Init();
    IR_StartScan();
    Control_Init(&integration_mpu);
#else
    Control_Init(0);
#endif
    Control_SetDiagnosticOnly(1U);
    Control_Start();

    UART_Printf("[BOOT] Motor auto baseline: neutral, USART1 host ARM required\r\n");
    /* Boot banner is informational; every PING repeats strict profile. */
    ReplyReady();

    while (1) {
        uint32_t now = millis();
        Control_Timing_t t;
#if defined(MOTOR_AUTO_SENSOR_INTEGRATION)
        {
            IR_Data_t ir;
            MPU6050_Update(&integration_mpu);
            if (IR_GetLatest(&ir)) {
                ir_scan_count = ir.scan_count;
                ir_last_progress_ms = now;
                IR_StartScan();
            }
            if (integration_mpu.cal_state == 4U) {
                FailSafe("IMU_CAL_FAILED");
            }
            if (!sensors_ready && integration_mpu.calibrated &&
                integration_mpu.cal_state == 3U && ir_scan_count >= 10U) {
                sensors_ready = 1U;
                ReplyReady();
            }
            if (sensors_ready && state >= AUTO_SETTLE && state <= AUTO_GAP) {
                if ((uint32_t)(micros()-integration_mpu.last_good_us) > 15000U)
                    FailSafe("IMU_FRESH_STALLED");
                if ((uint32_t)(now-ir_last_progress_ms) > 100U)
                    FailSafe("IR_SCAN_STALLED");
            }
        }
#endif
        PollHost(now);
        Control_GetTiming(&t);
        /* Independent main-loop check: TIM11's own tick watchdog will not
         * progress if TIM11 stalls. SysTick/main may still force neutral. */
        {
            uint32_t ticks = Control_GetTickCount();
            if (ticks != last_tick_seen) {
                last_tick_seen = ticks;
                last_tick_seen_ms = now;
            }
        }
        if (state >= AUTO_SETTLE && state <= AUTO_GAP) {
            if (now - last_tick_seen_ms > 20U) {
                FailSafe("TIM11_STALLED");
                continue;
            }
            if (state == AUTO_DRIVE &&
                now - state_since_ms > PulseMs() + 25U) {
                FailSafe("MAIN_WALL_PULSE_TIMEOUT");
                continue;
            }
            if (now - last_host_ms > AUTO_HOST_TIMEOUT_MS) {
                FailSafe("HOST_TIMEOUT");
                continue;
            }
            if (t.gap_over_1500us != last_tick_gaps) {
                FailSafe("TIM11_GAP");
                continue;
            }
        }
        last_tick_gaps = t.gap_over_1500us;

        if (state == AUTO_SETTLE &&
            now - state_since_ms >= AUTO_SETTLE_MS) {
            StartStep(now);
        } else if (state == AUTO_DRIVE) {
            MotorTest_Snapshot_t snapshot;
            MotorTest_GetSnapshot(&snapshot);
#if defined(MOTOR_AUTO_HIGH_TEST)
            /* Encoder motion gate: useful for jam detection but cannot
             * measure or limit stall current during the first ~100ms. */
            if (step_i >= AUTO_HIGH_FIRST && snapshot.state == MOTOR_TEST_RUNNING &&
                snapshot.elapsed_ms >= 100U) {
                const AutoPulse_t *p = &matrix[step_i];
                if ((p->l > 0 && Encoder_GetLeftCount() < 5) ||
                    (p->r > 0 && Encoder_GetRightCount() < 5)) {
                    FailSafe("HIGH_ENCODER_NO_MOTION");
                    continue;
                }
            }
#endif
            if (snapshot.state == MOTOR_TEST_ENDED) {
                if (snapshot.reason != MOTOR_TEST_END_TIMEOUT ||
                    snapshot.elapsed_ms != PulseMs()) {
                    FailSafe("PULSE_INTERRUPTED");
                    continue;
                }
                Motor_Stop(); /* neutral throughout subsequent observation */
                EmitSample(now,PulseMs(),"drive");
#if defined(MOTOR_AUTO_SENSOR_INTEGRATION)
                SensorReport("drive");
#endif
                UART_Printf("MOTOR_AUTO_END,round=%u,stage=%u,"
                            "elapsed=%lu,reason=%u,enc_l=%ld,enc_r=%ld,"
                            "ccr_l=%lu,ccr_r=%lu\r\n",
                            (unsigned)round_i,(unsigned)step_i,
                            (unsigned long)snapshot.elapsed_ms,
                            (unsigned)snapshot.reason,
                            (long)Encoder_GetLeftCount(),
                            (long)Encoder_GetRightCount(),
                            (unsigned long)TIM2->CCR2,
                            (unsigned long)TIM2->CCR1);
                completed_pulses++;
                state = AUTO_POST;
                state_since_ms = now;
#if defined(MOTOR_AUTO_HIGH_TEST)
                high_post_ref_valid = 0U;
#endif
                sample_since_ms = now;
            } else if (now - sample_since_ms >= AUTO_SAMPLE_MS) {
                sample_since_ms = now;
                EmitSample(now,snapshot.elapsed_ms,"drive");
            }
        } else if (state == AUTO_POST) {
#if defined(MOTOR_AUTO_HIGH_TEST)
            if (step_i >= AUTO_HIGH_FIRST && !high_post_ref_valid &&
                now - state_since_ms >= AUTO_HIGH_POST_MS - 100U) {
                high_post_ref_l = Encoder_GetLeftCount();
                high_post_ref_r = Encoder_GetRightCount();
                high_post_ref_valid = 1U;
            }
#endif
            if (now - sample_since_ms >= AUTO_SAMPLE_MS) {
                sample_since_ms = now;
                EmitSample(now,now-state_since_ms,"post");
            }
            if (now - state_since_ms >= PostMs()) {
#if defined(MOTOR_AUTO_HIGH_TEST)
                /* A valid low-duty baseline is required before this session
                 * reaches 30%+. A missing encoder or motor-power failure
                 * must not lead to blind higher-duty escalation. */
                if (step_i == 14U && Encoder_GetLeftCount() < 50) {
                    FailSafe("BASELINE_20PCT_LEFT_WEAK");
                    continue;
                }
                if (step_i == 15U && Encoder_GetRightCount() < 50) {
                    FailSafe("BASELINE_20PCT_RIGHT_WEAK");
                    continue;
                }
                if (step_i >= AUTO_HIGH_FIRST &&
                    (!high_post_ref_valid ||
                     (Encoder_GetLeftCount()-high_post_ref_l > 5) ||
                     (Encoder_GetLeftCount()-high_post_ref_l < -5) ||
                     (Encoder_GetRightCount()-high_post_ref_r > 5) ||
                     (Encoder_GetRightCount()-high_post_ref_r < -5))) {
                    FailSafe("HIGH_NOT_SETTLED");
                    continue;
                }
#endif
#if defined(MOTOR_AUTO_SENSOR_INTEGRATION)
                SensorReport("post");
#endif
                UART_Printf("MOTOR_AUTO_POST_END,round=%u,stage=%u,"
                            "elapsed=%u,enc_l=%ld,enc_r=%ld,"
                            "ccr_l=%lu,ccr_r=%lu\r\n",
                            (unsigned)round_i,(unsigned)step_i,
                            (unsigned)PostMs(),
                            (long)Encoder_GetLeftCount(),
                            (long)Encoder_GetRightCount(),
                            (unsigned long)TIM2->CCR2,
                            (unsigned long)TIM2->CCR1);
                UART_Printf("MOTOR_AUTO_LINK,round=%u,stage=%u,"
                            "keep_ok=%lu,host_age_ms=%lu,"
                            "rx_bytes=%lu,rx_ore=%lu,rx_fe=%lu,"
                            "rx_noise=%lu,rx_ring_drop=%lu,"
                            "bad_commands=%lu\r\n",
                            (unsigned)round_i,(unsigned)step_i,
                            (unsigned long)keep_ok,
                            (unsigned long)(now-last_host_ms),
                            (unsigned long)UART_GetRxByteCount(),
                            (unsigned long)UART_GetRxOverrunCount(),
                            (unsigned long)UART_GetRxFramingCount(),
                            (unsigned long)UART_GetRxNoiseCount(),
                            (unsigned long)UART_GetRxRingOverflowCount(),
                            (unsigned long)bad_rx_commands);
                MotorTest_AcknowledgeEnd();
                step_i++;
                if (step_i >= AUTO_STEPS) {
                    step_i = 0U;
                    round_i++;
                    UART_Printf("MOTOR_AUTO_ROUND_END,round=%u,"
                                "completed=%lu\r\n",
                                (unsigned)round_i,
                                (unsigned long)completed_pulses);
                }
                if (round_i >= AUTO_ROUNDS) {
                    Motor_Stop();
                    state = AUTO_DONE;
                    UART_Printf("MOTOR_AUTO_DONE,rounds=%u,pulses=%lu,"
                                "ccr_l=%lu,ccr_r=%lu,tim11_gaps=%lu,"
                                "tx_drop=%lu,keep_ok=%lu,"
                                "rx_ore=%lu,rx_ring_drop=%lu\r\n",
                                (unsigned)AUTO_ROUNDS,
                                (unsigned long)completed_pulses,
                                (unsigned long)TIM2->CCR2,
                                (unsigned long)TIM2->CCR1,
                                (unsigned long)t.gap_over_1500us,
                                (unsigned long)UART_GetDroppedMessages(),
                                (unsigned long)keep_ok,
                                (unsigned long)UART_GetRxOverrunCount(),
                                (unsigned long)UART_GetRxRingOverflowCount());
                } else {
                    state = AUTO_GAP;
                    state_since_ms = now;
                }
            }
        } else if (state == AUTO_GAP &&
                   now - state_since_ms >= AUTO_STAGE_GAP_MS) {
            StartStep(now);
        }
    }
}
#endif
