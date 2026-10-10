#include "motor_test.h"
#include "encoder.h"
#include "motor.h"
#include "stm32f4xx.h"

static volatile MotorTest_State_t state = MOTOR_TEST_IDLE;
static volatile MotorTest_EndReason_t reason = MOTOR_TEST_END_TIMEOUT;
static volatile uint32_t elapsed_ms = 0;
static volatile uint32_t duration_ms = 0;
static volatile uint16_t heartbeat_age_ms = 0;
static volatile uint16_t log_interval_ms = 200;
static volatile int16_t pwm_left = 0;
static volatile int16_t pwm_right = 0;

static uint32_t LockIRQ(void)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    return primask;
}

static void UnlockIRQ(uint32_t primask)
{
    if (!primask) __enable_irq();
}

uint8_t MotorTest_Start(int16_t left, int16_t right,
                        uint32_t duration, uint16_t log_interval)
{
    uint32_t mask;
    if (left < -MOTOR_TEST_MAX_PWM || left > MOTOR_TEST_MAX_PWM ||
        right < -MOTOR_TEST_MAX_PWM || right > MOTOR_TEST_MAX_PWM ||
        (left == 0 && right == 0) ||
        log_interval < MOTOR_TEST_MIN_LOG_MS ||
        log_interval > MOTOR_TEST_MAX_LOG_MS) {
        return 0;
    }

    mask = LockIRQ();
    if (state != MOTOR_TEST_IDLE) {
        UnlockIRQ(mask);
        return 0;
    }

    Encoder_Reset();
    elapsed_ms = 0;
    duration_ms = duration;
    heartbeat_age_ms = 0;
    log_interval_ms = log_interval;
    pwm_left = left;
    pwm_right = right;
    reason = MOTOR_TEST_END_TIMEOUT;
    Motor_SetPair(left, right);
    state = MOTOR_TEST_RUNNING;
    UnlockIRQ(mask);
    return 1;
}

void MotorTest_Stop(void)
{
    uint32_t mask = LockIRQ();
    if (state == MOTOR_TEST_RUNNING) {
        Motor_Stop();
        reason = MOTOR_TEST_END_STOP;
        state = MOTOR_TEST_ENDED;
    }
    UnlockIRQ(mask);
}

void MotorTest_Heartbeat(void)
{
    uint32_t mask = LockIRQ();
    if (state == MOTOR_TEST_RUNNING) heartbeat_age_ms = 0;
    UnlockIRQ(mask);
}

void MotorTest_Tick1ms(void)
{
    if (state != MOTOR_TEST_RUNNING) return;

    /* Constant-time work inside TIM11; no UART, no formatting, no waits.
     * duration 0 disables the test-duration limit, NOT the safety watchdog. */
    elapsed_ms++;
    heartbeat_age_ms++;

    if (heartbeat_age_ms >= MOTOR_TEST_HEARTBEAT_TIMEOUT_MS) {
        Motor_Stop();
        reason = MOTOR_TEST_END_HEARTBEAT;
        state = MOTOR_TEST_ENDED;
    } else if (duration_ms != 0U && elapsed_ms >= duration_ms) {
        Motor_Stop();
        reason = MOTOR_TEST_END_TIMEOUT;
        state = MOTOR_TEST_ENDED;
    }
}

uint8_t MotorTest_IsRunning(void) { return state == MOTOR_TEST_RUNNING; }

void MotorTest_GetSnapshot(MotorTest_Snapshot_t *out)
{
    uint32_t mask;
    if (!out) return;
    mask = LockIRQ();
    out->state = state;
    out->reason = reason;
    out->elapsed_ms = elapsed_ms;
    out->duration_ms = duration_ms;
    out->log_interval_ms = log_interval_ms;
    out->pwm_left = pwm_left;
    out->pwm_right = pwm_right;
    UnlockIRQ(mask);
}

void MotorTest_AcknowledgeEnd(void)
{
    uint32_t mask = LockIRQ();
    if (state == MOTOR_TEST_ENDED) {
        pwm_left = 0;
        pwm_right = 0;
        state = MOTOR_TEST_IDLE;
    }
    UnlockIRQ(mask);
}
