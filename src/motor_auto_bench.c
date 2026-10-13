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
#include "stm32f4xx.h"
#include <string.h>

#if defined(MOTOR_AUTO_BENCH)
#if defined(MOTOR_PWM_DIAG) || defined(MPU_INTEGRATION_BENCH) || \
    defined(MPU_FRESHNESS_DIAG) || defined(MPU_SAMPLING_POLICY_B)
#error "MOTOR_AUTO_BENCH must be compiled independently from motor key/IMU modes"
#endif

#define AUTO_ROUNDS          3U
#define AUTO_STEPS           7U
#define AUTO_PULSE_MS        200U
#define AUTO_POST_MS         300U
#define AUTO_SETTLE_MS      1000U
#define AUTO_STAGE_GAP_MS    250U
#define AUTO_SAMPLE_MS       20U
#define AUTO_HOST_TIMEOUT_MS 1600U

typedef struct { int16_t l, r; } AutoPulse_t;
static const AutoPulse_t matrix[AUTO_STEPS] = {
    {80,0}, {0,80}, {100,0}, {0,100},
    {120,0}, {0,120}, {120,120}
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
static uint32_t completed_pulses = 0;
static uint8_t rx_count = 0;
static char rx_line[32];

static void EmitSample(uint32_t now, uint32_t elapsed, const char *phase)
{
    Control_Timing_t t;
    int16_t al=0, ar=0;
    Motor_GetAppliedPairPermille(&al,&ar);
    Control_GetTiming(&t);
    UART_Printf("MOTOR_AUTO_SAMPLE,round=%u,stage=%u,phase=%s,"
                "ms=%lu,elapsed=%lu,applied_l=%d,applied_r=%d,"
                "enc_l=%ld,enc_r=%ld,ccr_l=%lu,ccr_r=%lu,"
                "tim11_late=%lu,tim11_max_cycles=%lu,"
                "tim11_gaps=%lu,tx_drop=%lu\r\n",
                (unsigned)round_i,(unsigned)step_i,phase,
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
                "completed=%lu,ccr_l=%lu,ccr_r=%lu\r\n",
                reason,(unsigned)round_i,(unsigned)step_i,
                (unsigned long)completed_pulses,
                (unsigned long)TIM2->CCR2,(unsigned long)TIM2->CCR1);
}

static void StartStep(uint32_t now)
{
    const AutoPulse_t *p = &matrix[step_i];
    Motor_SetDeadband(0U, 0U);
    Motor_SetSlewRates(10U, 20U);
    if (!MotorTest_StartPermille(p->l,p->r,AUTO_PULSE_MS,100U)) {
        FailSafe("START_REJECTED");
        return;
    }
    state = AUTO_DRIVE;
    state_since_ms = now;
    sample_since_ms = now;
    UART_Printf("MOTOR_AUTO_START,round=%u,stage=%u,l=%d,r=%d,"
                "pulse_ms=%u\r\n",(unsigned)round_i,(unsigned)step_i,
                (int)p->l,(int)p->r,(unsigned)AUTO_PULSE_MS);
}

static void ReplyReady(void)
{
    /* Repeat complete profile with each PING: host may connect after MCU
     * boot/flash and must still capture the exact firmware signature. */
    UART_Printf("MOTOR_AUTO_PROFILE,version=1,nominal_hz=%lu,"
                "pwm_hz=%lu,rounds=%u,steps=%u,hard_cap_permille=120,"
                "pulse_ms=200,post_ms=300,boot_neutral=1,"
                "no_current_sense=1,stby_fixed_high=1\r\n",
                (unsigned long)SystemCoreClock,
                (unsigned long)(Board_GetAPB1TimerClockHz()/(TIM2->ARR+1U)),
                (unsigned)AUTO_ROUNDS,(unsigned)AUTO_STEPS);
    UART_Printf("MOTOR_AUTO_READY,version=1,rounds=%u,steps=%u,"
                "hard_cap_permille=120,pulse_ms=200,post_ms=300,"
                "one_shot=1,boot_neutral=1,rx=USART1_PA10,state=%u\r\n",
                (unsigned)AUTO_ROUNDS,(unsigned)AUTO_STEPS,(unsigned)state);
}

static void ReceiveLine(const char *line, uint32_t now)
{
    if (strcmp(line,"MOTOR PING") == 0) {
        ReplyReady();
    } else if (strcmp(line,"MOTOR KEEP") == 0) {
        if (state != AUTO_WAIT) last_host_ms = now;
    } else if (strcmp(line,"MOTOR STOP") == 0) {
        FailSafe("HOST_STOP");
    } else if (strcmp(line,"MOTOR START") == 0 && state == AUTO_WAIT) {
        last_host_ms = now;
        state = AUTO_SETTLE;
        state_since_ms = now;
        round_i = 0U;
        step_i = 0U;
        completed_pulses = 0U;
        Motor_Stop();
        UART_Printf("MOTOR_AUTO_BEGIN,rounds=%u,steps=%u,"
                    "settle_ms=%u,boot_neutral=1\r\n",
                    (unsigned)AUTO_ROUNDS,(unsigned)AUTO_STEPS,
                    (unsigned)AUTO_SETTLE_MS);
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
    Control_Init(0);
    Control_SetDiagnosticOnly(1U);
    Control_Start();

    UART_Printf("[BOOT] Motor auto baseline: neutral, USART1 host ARM required\r\n");
    UART_Printf("MOTOR_AUTO_PROFILE,version=1,nominal_hz=%lu,"
                "pwm_hz=%lu,rounds=%u,steps=%u,hard_cap_permille=120,"
                "pulse_ms=200,post_ms=300,boot_neutral=1,"
                "no_current_sense=1,stby_fixed_high=1\r\n",
                (unsigned long)SystemCoreClock,
                (unsigned long)(Board_GetAPB1TimerClockHz()/(TIM2->ARR+1U)),
                (unsigned)AUTO_ROUNDS,(unsigned)AUTO_STEPS);

    while (1) {
        uint32_t now = millis();
        Control_Timing_t t;
        PollHost(now);
        Control_GetTiming(&t);

        if (state >= AUTO_SETTLE && state <= AUTO_GAP) {
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
            if (snapshot.state == MOTOR_TEST_ENDED) {
                if (snapshot.reason != MOTOR_TEST_END_TIMEOUT ||
                    snapshot.elapsed_ms != AUTO_PULSE_MS) {
                    FailSafe("PULSE_INTERRUPTED");
                    continue;
                }
                Motor_Stop(); /* neutral throughout subsequent observation */
                EmitSample(now,AUTO_PULSE_MS,"drive");
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
                sample_since_ms = now;
            } else if (now - sample_since_ms >= AUTO_SAMPLE_MS) {
                sample_since_ms = now;
                EmitSample(now,snapshot.elapsed_ms,"drive");
            }
        } else if (state == AUTO_POST) {
            if (now - sample_since_ms >= AUTO_SAMPLE_MS) {
                sample_since_ms = now;
                EmitSample(now,now-state_since_ms,"post");
            }
            if (now - state_since_ms >= AUTO_POST_MS) {
                UART_Printf("MOTOR_AUTO_POST_END,round=%u,stage=%u,"
                            "elapsed=%u,enc_l=%ld,enc_r=%ld,"
                            "ccr_l=%lu,ccr_r=%lu\r\n",
                            (unsigned)round_i,(unsigned)step_i,
                            (unsigned)AUTO_POST_MS,
                            (long)Encoder_GetLeftCount(),
                            (long)Encoder_GetRightCount(),
                            (unsigned long)TIM2->CCR2,
                            (unsigned long)TIM2->CCR1);
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
                                "tx_drop=%lu\r\n",
                                (unsigned)AUTO_ROUNDS,
                                (unsigned long)completed_pulses,
                                (unsigned long)TIM2->CCR2,
                                (unsigned long)TIM2->CCR1,
                                (unsigned long)t.gap_over_1500us,
                                (unsigned long)UART_GetDroppedMessages());
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
