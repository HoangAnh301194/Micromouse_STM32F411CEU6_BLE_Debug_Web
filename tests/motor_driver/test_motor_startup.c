#include "motor_startup.h"
#include "motor_test.h"
#include "motor.h"
#include "stm32f4xx.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>

GPIO_TypeDef test_gpio_a, test_gpio_b;
TIM_TypeDef test_timer;
RCC_TypeDef test_rcc;
static uint32_t irq_mask;
static int32_t count_left, count_right;
uint32_t __get_PRIMASK(void) { return irq_mask; }
void __disable_irq(void) { irq_mask = 1U; }
void __enable_irq(void) { irq_mask = 0U; }
uint32_t Board_GetAPB1TimerClockHz(void) { return 96000000U; }
void Encoder_Reset(void) { count_left = count_right = 0; }
int32_t Encoder_GetLeftCount(void) { return count_left; }
int32_t Encoder_GetRightCount(void) { return count_right; }
float Encoder_GetLeftSpeedPPS(void) { return 0.0f; }
float Encoder_GetRightSpeedPPS(void) { return 0.0f; }

static MotorStartup_Snapshot_t Snapshot(void)
{
    MotorStartup_Snapshot_t snapshot;
    MotorStartup_GetSnapshot(&snapshot);
    return snapshot;
}

static void Tick(uint32_t count, uint8_t keep)
{
    uint32_t index;
    for (index = 0U; index < count; index++) {
        if (keep && index % 100U == 0U) MotorStartup_Heartbeat();
        Motor_Update1ms();
        MotorTest_Tick1ms();
        MotorStartup_Tick1ms();
    }
}

static void CheckFault(MotorStartup_Fault_t expected)
{
    assert(Snapshot().state == MOTOR_STARTUP_FAULT);
    assert(Snapshot().fault == expected);
    assert(Motor_OutputsLocked());
    assert(TIM2->CCR1 == 3200U && TIM2->CCR2 == 3200U);
    assert(!MotorStartup_Pulse(2U, 0U, 100U));
    assert(!MotorStartup_Arm());
}

int main(int argc, char **argv)
{
    assert(argc == 2);
    Motor_Init();
    MotorStartup_Init();
    assert(!MotorStartup_Arm());
    assert(!MotorStartup_Pulse(1U, 0U, 100U));
    Tick(299U, 0U);
    assert(!MotorStartup_Arm());
    Tick(1U, 0U);
    assert(MotorStartup_Arm());
    assert(!MotorStartup_Arm());
    assert(!MotorStartup_Pulse(1U, MOTOR_STARTUP_MAX_WHEEL + 1U, 100U));
    assert(!MotorStartup_Pulse(1U, 0U, MOTOR_STARTUP_MAX_PERMILLE + 5U));
    assert(!MotorStartup_Pulse(1U, 0U, 201U));
    assert(!MotorStartup_Pulse(1U, 0U, 0U));
    assert(!MotorStartup_Pulse(1U, 0U, 7U));
    assert(!MotorStartup_Pulse(2U, 0U, 100U));
    assert(MotorStartup_Pulse(1U, 0U, 100U));
    assert(!MotorStartup_Pulse(1U, 0U, 100U));
#if defined(MOTOR_STARTUP_HIGH_SPEED)
    if (strcmp(argv[1], "high_stall") == 0) {
        Tick(100U, 1U);
        CheckFault(MOTOR_STARTUP_NO_PROGRESS);
    } else if (strcmp(argv[1], "high_both") == 0) {
        uint32_t index;
        for (index = 0U; index < MOTOR_STARTUP_PULSE_MS; index++) {
            count_left++;
            Tick(1U, 1U);
        }
        assert(Snapshot().state == MOTOR_STARTUP_REST);
        Tick(MOTOR_STARTUP_REST_MS, 1U);
        assert(MotorStartup_Pulse(2U, 2U, 1000U));
        for (index = 0U; index < MOTOR_STARTUP_PULSE_MS; index++) {
            count_left++;
            count_right++;
            Tick(1U, 1U);
            MotorStartup_RecordCycles(120U);
        }
        assert(Snapshot().state == MOTOR_STARTUP_REST);
        assert(Snapshot().end_left == MOTOR_STARTUP_PULSE_MS);
        assert(Snapshot().end_right == MOTOR_STARTUP_PULSE_MS);
        assert(Snapshot().cycle_calls > 400U);
        assert(Snapshot().cycle_total == Snapshot().cycle_calls * 120U);
        assert(Snapshot().cycle_max == 120U);
        assert(TIM2->CCR1 == 3200U && TIM2->CCR2 == 3200U);
    } else if (strcmp(argv[1], "high_link") == 0) {
        uint32_t index;
        for (index = 0U; index < MOTOR_STARTUP_LINK_MS; index++) {
            count_left++;
            Tick(1U, 0U);
        }
        CheckFault(MOTOR_STARTUP_LINK_LOST);
    } else {
        assert(0);
    }
#else
    if (strcmp(argv[1], "normal") == 0) {
        Tick(249U, 1U);
        assert(Snapshot().state == MOTOR_STARTUP_DRIVE);
        count_left = 10;
        Tick(1U, 1U);
        assert(Snapshot().state == MOTOR_STARTUP_REST);
        assert(Snapshot().completed_id == 1U && Snapshot().end_left == 10);
        assert(TIM2->CCR1 == 3200U && TIM2->CCR2 == 3200U);
        Tick(749U, 1U);
        assert(!MotorStartup_Pulse(2U, 1U, 200U));
        count_left++;
        Tick(1U, 1U);
        assert(!MotorStartup_Pulse(2U, 1U, 200U));
        Tick(300U, 1U);
        assert(MotorStartup_Pulse(2U, 1U, 200U));
        Tick(250U, 1U);
        Tick(750U, 1U);
        assert(!MotorStartup_Pulse(2U, 1U, 200U));
        assert(MotorStartup_Pulse(3U, 0U, 5U));
        Tick(250U, 1U);
    } else if (strcmp(argv[1], "budget") == 0) {
        uint32_t sequence;
        Tick(1000U, 1U);
        for (sequence = 2U; sequence <= MOTOR_STARTUP_MAX_PULSES; sequence++) {
            assert(MotorStartup_Pulse(sequence, 0U, 5U));
            Tick(1000U, 1U);
        }
        assert(!MotorStartup_Pulse(129U, 0U, 5U));
        assert(Snapshot().state == MOTOR_STARTUP_READY);
        assert(TIM2->CCR1 == 3200U && TIM2->CCR2 == 3200U);
    } else if (strcmp(argv[1], "link") == 0) {
        Tick(399U, 0U);
        assert(Snapshot().state == MOTOR_STARTUP_REST);
        Tick(1U, 0U);
        CheckFault(MOTOR_STARTUP_LINK_LOST);
    } else if (strcmp(argv[1], "wrong_wheel") == 0) {
        count_right = 3;
        Tick(1U, 0U);
        CheckFault(MOTOR_STARTUP_WRONG_WHEEL);
    } else if (strcmp(argv[1], "reverse") == 0) {
        count_left = -3;
        Tick(1U, 0U);
        CheckFault(MOTOR_STARTUP_REVERSE_COUNTS);
    } else if (strcmp(argv[1], "session") == 0) {
        Tick(MOTOR_STARTUP_SESSION_MS, 1U);
        CheckFault(MOTOR_STARTUP_SESSION_END);
    } else if (strcmp(argv[1], "stop") == 0) {
        Tick(10U, 1U);
        MotorStartup_Abort(MOTOR_STARTUP_HOST_STOP);
        CheckFault(MOTOR_STARTUP_HOST_STOP);
    } else {
        assert(0);
    }
#endif
    assert(irq_mask == 0U);
    printf("PASS: bounded startup %s (actual C, register mocks only).\n", argv[1]);
    return 0;
}
