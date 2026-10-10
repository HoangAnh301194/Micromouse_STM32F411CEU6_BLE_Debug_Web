#include "motor_test.h"
#include "encoder.h"
#include "motor.h"
#include "stm32f4xx.h"

static volatile MotorTest_State_t state = MOTOR_TEST_IDLE;
static volatile MotorTest_EndReason_t reason = MOTOR_TEST_END_TIMEOUT;
static volatile uint32_t elapsed_ms = 0;
static volatile uint32_t duration_ms = 0;
static volatile uint16_t heartbeat_age_ms = 0;
static volatile uint16_t brake_age_ms = 0;
static volatile uint16_t log_interval_ms = 200;
static volatile int16_t pwm_left = 0;
static volatile int16_t pwm_right = 0;

static uint32_t LockIRQ(void) {
    uint32_t x = __get_PRIMASK();
    __disable_irq();
    return x;
}
static void UnlockIRQ(uint32_t x) { if (!x) __enable_irq(); }

uint8_t MotorTest_StartPermille(int16_t left, int16_t right,
                               uint32_t duration, uint16_t interval)
{
    uint32_t x;
    if (left < -MOTOR_TEST_MAX_CMD || left > MOTOR_TEST_MAX_CMD ||
        right < -MOTOR_TEST_MAX_CMD || right > MOTOR_TEST_MAX_CMD ||
        (left == 0 && right == 0) ||
        interval < MOTOR_TEST_MIN_LOG_MS ||
        interval > MOTOR_TEST_MAX_LOG_MS) return 0;

    x = LockIRQ();
    if (state != MOTOR_TEST_IDLE) { UnlockIRQ(x); return 0; }
    Encoder_Reset();
    elapsed_ms = 0;
    duration_ms = duration;
    heartbeat_age_ms = 0;
    brake_age_ms = 0;
    log_interval_ms = interval;
    pwm_left = left;
    pwm_right = right;
    reason = MOTOR_TEST_END_TIMEOUT;
    Motor_SetPairPermille(left, right);
    state = MOTOR_TEST_RUNNING;
    UnlockIRQ(x);
    return 1;
}
uint8_t MotorTest_Start(int16_t left, int16_t right,
                        uint32_t duration, uint16_t interval)
{
    if (left < -100 || left > 100 || right < -100 || right > 100) return 0;
    return MotorTest_StartPermille(left * 10, right * 10, duration, interval);
}
uint8_t MotorTest_SetPermille(int16_t left, int16_t right)
{
    uint32_t x;
    if (left < -MOTOR_TEST_MAX_CMD || left > MOTOR_TEST_MAX_CMD ||
        right < -MOTOR_TEST_MAX_CMD || right > MOTOR_TEST_MAX_CMD) return 0;
    x = LockIRQ();
    if (state != MOTOR_TEST_RUNNING) { UnlockIRQ(x); return 0; }
    pwm_left = left;
    pwm_right = right;
    Motor_SetPairPermille(left, right);
    UnlockIRQ(x);
    return 1;
}
void MotorTest_Stop(void)
{
    uint32_t x = LockIRQ();
    if (state == MOTOR_TEST_RUNNING || state == MOTOR_TEST_BRAKING) {
        Motor_Stop();
        pwm_left = pwm_right = 0;
        reason = MOTOR_TEST_END_STOP;
        state = MOTOR_TEST_ENDED;
    }
    UnlockIRQ(x);
}
void MotorTest_BrakePulse(void)
{
    uint32_t x = LockIRQ();
    if (state == MOTOR_TEST_RUNNING) {
        Motor_Brake();
        pwm_left = pwm_right = 0;
        brake_age_ms = 0;
        state = MOTOR_TEST_BRAKING;
    }
    UnlockIRQ(x);
}
void MotorTest_Heartbeat(void)
{
    uint32_t x = LockIRQ();
    if (state == MOTOR_TEST_RUNNING) heartbeat_age_ms = 0;
    UnlockIRQ(x);
}
void MotorTest_Tick1ms(void)
{
    if (state == MOTOR_TEST_BRAKING) {
        elapsed_ms++;
        if (++brake_age_ms >= MOTOR_TEST_BRAKE_PULSE_MS) {
            Motor_Stop(); /* release brake to coast */
            reason = MOTOR_TEST_END_BRAKE;
            state = MOTOR_TEST_ENDED;
        }
        return;
    }
    if (state != MOTOR_TEST_RUNNING) return;
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
uint8_t MotorTest_IsRunning(void) {
    return state == MOTOR_TEST_RUNNING || state == MOTOR_TEST_BRAKING;
}
void MotorTest_GetSnapshot(MotorTest_Snapshot_t *out)
{
    uint32_t x;
    if (!out) return;
    x = LockIRQ();
    out->state = state;
    out->reason = reason;
    out->elapsed_ms = elapsed_ms;
    out->duration_ms = duration_ms;
    out->log_interval_ms = log_interval_ms;
    out->pwm_left = pwm_left;
    out->pwm_right = pwm_right;
    UnlockIRQ(x);
}
void MotorTest_AcknowledgeEnd(void)
{
    uint32_t x = LockIRQ();
    if (state == MOTOR_TEST_ENDED) {
        pwm_left = pwm_right = 0;
        state = MOTOR_TEST_IDLE;
    }
    UnlockIRQ(x);
}
