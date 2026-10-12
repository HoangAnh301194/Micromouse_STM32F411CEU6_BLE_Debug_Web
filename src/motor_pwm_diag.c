/* Opt-in, low-duty, short-pulse motor/encoder electrical diagnostic.
 * NOT a full voltage/current-qualified motor performance test.
 * No BLE, no MPU, no IR, no PID. Motors never start automatically.
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

#if defined(MOTOR_PWM_DIAG)
#if defined(MPU_FRESHNESS_DIAG) || defined(CONTROL_LOAD_DIAG)
#error "MOTOR_PWM_DIAG cannot run with IMU or load diagnostic profiles"
#endif

#define BENCH_ARM_HOLD_MS     1500U
#define BENCH_ARM_TIMEOUT_MS 10000U
#define BENCH_PULSE_MS       200U
#define BENCH_LOG_MS         20U

/* 0.1% command units; 120 = 12%, NOT a current/voltage limit.
 * Check rated motor supply and TB6612 VCC first. Prefer a current-limited
 * laboratory supply instead of an unrestricted 2S pack for first tests. */
typedef struct { int16_t l, r; } BenchPulse;
static const BenchPulse pulses[] = {
    {80, 0}, {0, 80}, {120, 0}, {0, 120}, {120, 120}
};
#define BENCH_PULSE_COUNT (sizeof(pulses)/sizeof(pulses[0]))

static void Report(uint32_t now, uint32_t stage, uint32_t elapsed)
{
    int16_t al = 0, ar = 0;
    Control_Timing_t t;
    Motor_GetAppliedPairPermille(&al, &ar);
    Control_GetTiming(&t);
    UART_Printf(
       "MOTOR_SAMPLE,ms=%lu,stage=%lu,elapsed=%lu,"
       "applied_l=%d,applied_r=%d,enc_l=%ld,enc_r=%ld,"
       "tim11_max_cycles=%lu,tim11_gaps=%lu,tx_drop=%lu\r\n",
       (unsigned long)now, (unsigned long)stage, (unsigned long)elapsed,
       (int)al, (int)ar,
       (long)Encoder_GetLeftCount(), (long)Encoder_GetRightCount(),
       (unsigned long)t.exec_max_cycles,
       (unsigned long)t.gap_over_1500us,
       (unsigned long)UART_GetDroppedMessages());
}

int MotorPwmDiag_Main(void)
{
    uint32_t armed_since = 0U;
    uint32_t press_start_ms = 0U;
    uint32_t last_log_ms = 0U;
    uint32_t last_tick_gaps = 0U;
    uint8_t seen_release = 0U;
    uint8_t armed = 0U, pending_arm = 0U, active = 0U;
    uint8_t prev_pressed = 0U, stage = 0U;

    Board_Init();
    SystemTimer_Init();
    UART_Init();

    if (Board_GetClockFault() != 0U) {
        UART_Printf("MOTOR_BOOT_FAIL,clock_fault=%lu\r\n",
                    (unsigned long)Board_GetClockFault());
        while (1) { Board_StatusLed(1U); }
    }

    /* Motor init starts TIM2 at 0 duty and neutral direction;
     * it cannot establish current limiting without external hardware. */
    Motor_Init();
    Motor_Stop();
    Motor_SetDeadband(0U, 0U);
    Motor_SetSlewRates(10U, 20U);
    Encoder_Init();

    Control_Init(0);
    Control_SetDiagnosticOnly(1U);
    Control_Start();

    UART_Printf("\r\n[BOOT] KEY-armed motor PWM bench; MOTOR OFF\r\n");
    UART_Printf("MOTOR_PROFILE,nominal_hz=%lu,pwm_hz=%lu,hard_cap_permille=120,"
                "pulse_ms=200,key_arm_ms=1500,one_shot=1,motor_off_at_boot=1\r\n",
                (unsigned long)SystemCoreClock,
                (unsigned long)(Board_GetAPB1TimerClockHz() / (TIM2->ARR + 1U)));
    UART_Printf("MOTOR_WARNING,requires_verified_motor_rating_TB6612_VCC_and_external_current_limit\r\n");
    UART_Printf("MOTOR_HELP,hold_KEY_1p5s_then_release_to_arm;press_and_hold_for_one_short_pulse;release_stops\r\n");

    while (1) {
        uint32_t now = millis();
        uint8_t key = Board_ButtonPressed();
        MotorTest_Snapshot_t snapshot;
        Control_Timing_t timing;
        Control_GetTiming(&timing);

        /* Interrupt timing faults latch the test in STOP mode. */
        if (timing.gap_over_1500us != last_tick_gaps) {
            last_tick_gaps = timing.gap_over_1500us;
            MotorTest_Stop();
            Motor_Stop();
            armed = 0U;
            pending_arm = 0U;
            UART_Printf("MOTOR_FAULT,TIM11_GAP,STOP\r\n");
        }

        /* Releasing KEY while a pulse is running always requests a stop,
         * without waiting for debounce or UART logging. */
        if (active && !key) MotorTest_Stop();

        MotorTest_GetSnapshot(&snapshot);
        if (active && snapshot.state == MOTOR_TEST_ENDED) {
            Motor_Stop();
            Report(now, stage, snapshot.elapsed_ms);
            UART_Printf("MOTOR_END,stage=%u,reason=%u,elapsed=%lu,tx_drop=%lu\r\n",
                        (unsigned)stage, (unsigned)snapshot.reason,
                        (unsigned long)snapshot.elapsed_ms,
                        (unsigned long)UART_GetDroppedMessages());
            MotorTest_AcknowledgeEnd();
            active = 0U;
            armed = 0U; /* Explicit 1.5s re-arm for EACH pulse. */
            pending_arm = 0U;
            if (stage < BENCH_PULSE_COUNT) stage++;
            UART_Printf("MOTOR_DISARMED,next_stage=%u\r\n", (unsigned)stage);
        }

        if (!seen_release) {
            if (!key) seen_release = 1U;
        } else if (stage >= BENCH_PULSE_COUNT) {
            /* One complete sequence per firmware boot: no auto-repeat. */
            if (!active && stage == BENCH_PULSE_COUNT) {
                UART_Printf("MOTOR_DONE,all_5_pulses_complete,pwm=0\r\n");
                stage++;
            }
        } else if (!active && !armed) {
            if (key && !prev_pressed) {
                press_start_ms = now;
                pending_arm = 0U;
            }
            if (key && (uint32_t)(now - press_start_ms) >= BENCH_ARM_HOLD_MS)
                pending_arm = 1U;
            if (!key && prev_pressed) {
                if (pending_arm) {
                    armed = 1U;
                    armed_since = now;
                    UART_Printf("MOTOR_ARMED,stage=%u,expires_ms=%lu\r\n",
                                (unsigned)stage,
                                (unsigned long)(now + BENCH_ARM_TIMEOUT_MS));
                }
                pending_arm = 0U;
            }
        } else if (!active && armed) {
            if ((uint32_t)(now - armed_since) >= BENCH_ARM_TIMEOUT_MS) {
                armed = 0U;
                UART_Printf("MOTOR_DISARMED,reason=ARM_TIMEOUT\r\n");
            } else if (key && !prev_pressed) {
                if (MotorTest_StartPermille(
                    pulses[stage].l, pulses[stage].r,
                    BENCH_PULSE_MS, 100U)) {
                    active = 1U;
                    last_log_ms = now;
                    UART_Printf("MOTOR_START,stage=%u,l=%d,r=%d,pulse_ms=%u\r\n",
                                (unsigned)stage, (int)pulses[stage].l,
                                (int)pulses[stage].r, (unsigned)BENCH_PULSE_MS);
                } else {
                    armed = 0U;
                    UART_Printf("MOTOR_REJECTED,stage=%u\r\n", (unsigned)stage);
                }
            }
        }

        if (active && (uint32_t)(now - last_log_ms) >= BENCH_LOG_MS) {
            last_log_ms = now;
            MotorTest_GetSnapshot(&snapshot);
            Report(now, stage, snapshot.elapsed_ms);
        }
        prev_pressed = key;
    }
}
#endif /* MOTOR_PWM_DIAG */
