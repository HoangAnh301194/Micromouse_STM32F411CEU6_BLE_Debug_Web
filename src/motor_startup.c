#include "motor_startup.h"
#include "motor.h"
#include "motor_test.h"
#include "encoder.h"
#include "stm32f4xx.h"

static volatile MotorStartup_Snapshot_t startup;
static uint32_t link_age, session_age;
static int32_t previous_left, previous_right;
#if defined(MOTOR_STARTUP_HIGH_SPEED)
static uint32_t motion_age_left, motion_age_right;
#endif

static uint32_t Lock(void)
{
    uint32_t mask = __get_PRIMASK();
    __disable_irq();
    return mask;
}

static void Unlock(uint32_t mask)
{
    if (!mask) __enable_irq();
}

void MotorStartup_Init(void)
{
    uint32_t mask = Lock();
    Motor_Stop();
    startup = (MotorStartup_Snapshot_t){0};
    previous_left = Encoder_GetLeftCount();
    previous_right = Encoder_GetRightCount();
    link_age = session_age = 0U;
    Unlock(mask);
}

uint8_t MotorStartup_Arm(void)
{
    uint8_t accepted = 0U;
    uint32_t mask = Lock();
    if (startup.state == MOTOR_STARTUP_DISARMED &&
        startup.quiet_ms >= MOTOR_STARTUP_QUIET_MS && !Motor_OutputsLocked()) {
        startup.state = MOTOR_STARTUP_READY;
        link_age = session_age = 0U;
        accepted = 1U;
    }
    Unlock(mask);
    return accepted;
}

uint8_t MotorStartup_Pulse(uint32_t id, uint8_t wheel, uint16_t duty)
{
    uint8_t accepted = 0U;
    uint32_t mask = Lock();
    if (startup.state == MOTOR_STARTUP_READY &&
        startup.quiet_ms >= MOTOR_STARTUP_QUIET_MS &&
        id == startup.id + 1U && id <= MOTOR_STARTUP_MAX_PULSES &&
        wheel <= MOTOR_STARTUP_MAX_WHEEL && duty >= 5U && duty <= MOTOR_STARTUP_MAX_PERMILLE && duty % 5U == 0U) {
        Motor_SetDeadband(0U, 0U);
        Motor_SetSlewRates(20U, 30U);
        if (MotorTest_StartPermille(wheel != 1U ? (int16_t)duty : 0,
                                   wheel != 0U ? (int16_t)duty : 0,
                                   MOTOR_STARTUP_PULSE_MS, 100U)) {
            startup.id = id;
            startup.wheel = wheel;
            startup.duty = duty;
            startup.elapsed_ms = startup.quiet_ms = startup.rest_ms = 0U;
            previous_left = previous_right = 0;
#if defined(MOTOR_STARTUP_HIGH_SPEED)
            motion_age_left = motion_age_right = 0U;
            startup.cycle_calls = startup.cycle_total = startup.cycle_max = 0U;
#endif
            startup.state = MOTOR_STARTUP_DRIVE;
            link_age = 0U;
            accepted = 1U;
        }
    }
    Unlock(mask);
    return accepted;
}

void MotorStartup_Heartbeat(void)
{
    uint32_t mask = Lock();
    link_age = 0U;
    MotorTest_Heartbeat();
    Unlock(mask);
}

void MotorStartup_Abort(MotorStartup_Fault_t reason)
{
    uint32_t mask = Lock();
    if (startup.state != MOTOR_STARTUP_FAULT) {
        MotorTest_Stop();
        Motor_LockOutputs();
        startup.fault = reason;
        startup.state = MOTOR_STARTUP_FAULT;
    }
    Unlock(mask);
}

void MotorStartup_Tick1ms(void)
{
    int32_t left = Encoder_GetLeftCount();
    int32_t right = Encoder_GetRightCount();
    if (left != previous_left || right != previous_right) startup.quiet_ms = 0U;
    else if (startup.quiet_ms < MOTOR_STARTUP_QUIET_MS) startup.quiet_ms++;
#if defined(MOTOR_STARTUP_HIGH_SPEED)
    motion_age_left = left != previous_left ? 0U : motion_age_left + 1U;
    motion_age_right = right != previous_right ? 0U : motion_age_right + 1U;
#endif
    previous_left = left;
    previous_right = right;
    if (startup.state == MOTOR_STARTUP_DISARMED || startup.state == MOTOR_STARTUP_FAULT) return;
    if (++link_age >= MOTOR_STARTUP_LINK_MS) {
        MotorStartup_Abort(MOTOR_STARTUP_LINK_LOST);
        return;
    }
    if (++session_age >= MOTOR_STARTUP_SESSION_MS) {
        MotorStartup_Abort(MOTOR_STARTUP_SESSION_END);
        return;
    }
    if (startup.state == MOTOR_STARTUP_DRIVE) {
        MotorTest_Snapshot_t test;
        int32_t passive = startup.wheel == 0U ? right : left;
        if (startup.wheel < 2U && (passive > 2 || passive < -2)) {
            MotorStartup_Abort(MOTOR_STARTUP_WRONG_WHEEL);
            return;
        }
        if ((startup.wheel != 1U && left < -2) || (startup.wheel != 0U && right < -2)) {
            MotorStartup_Abort(MOTOR_STARTUP_REVERSE_COUNTS);
            return;
        }
        MotorTest_GetSnapshot(&test);
        startup.elapsed_ms = test.elapsed_ms;
#if defined(MOTOR_STARTUP_HIGH_SPEED)
        if (test.state == MOTOR_TEST_RUNNING && test.elapsed_ms >= 100U &&
            ((startup.wheel != 1U && (left < 5 || motion_age_left >= 80U)) ||
             (startup.wheel != 0U && (right < 5 || motion_age_right >= 80U)))) {
            MotorStartup_Abort(MOTOR_STARTUP_NO_PROGRESS);
            return;
        }
#endif
        if (test.state == MOTOR_TEST_ENDED) {
            if (test.reason != MOTOR_TEST_END_TIMEOUT || test.elapsed_ms != MOTOR_STARTUP_PULSE_MS) {
                MotorStartup_Abort(MOTOR_STARTUP_TEST_ENDED);
                return;
            }
            startup.end_left = left;
            startup.end_right = right;
            startup.completed_id = startup.id;
            startup.rest_ms = 0U;
            startup.state = MOTOR_STARTUP_REST;
            MotorTest_AcknowledgeEnd();
        }
    } else if (startup.state == MOTOR_STARTUP_REST) {
        startup.rest_ms++;
        if (startup.rest_ms >= MOTOR_STARTUP_REST_MS &&
            startup.quiet_ms >= MOTOR_STARTUP_QUIET_MS) startup.state = MOTOR_STARTUP_READY;
    }
}

void MotorStartup_GetSnapshot(MotorStartup_Snapshot_t *out)
{
    uint32_t mask;
    if (!out) return;
    mask = Lock();
    *out = startup;
    Unlock(mask);
}

#if defined(MOTOR_STARTUP_HIGH_SPEED)
void MotorStartup_RecordCycles(uint32_t cycles)
{
    if (startup.state != MOTOR_STARTUP_DRIVE || startup.elapsed_ms < 100U) return;
    startup.cycle_calls++;
    startup.cycle_total += cycles;
    if (cycles > startup.cycle_max) startup.cycle_max = cycles;
}
#endif
