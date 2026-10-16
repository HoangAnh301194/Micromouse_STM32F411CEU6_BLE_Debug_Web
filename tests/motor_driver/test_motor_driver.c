#include "motor.h"
#include "motor_test.h"
#include "pinout.h"
#include "stm32f4xx.h"

#include <assert.h>
#include <stdio.h>
#include <string.h>

GPIO_TypeDef test_gpio_a, test_gpio_b;
TIM_TypeDef test_timer;
RCC_TypeDef test_rcc;
static uint32_t irq_mask;
static uint32_t timer_clock_hz = 96000000U;
static uint32_t encoder_resets;
static float encoder_speed_left, encoder_speed_right;

uint32_t __get_PRIMASK(void) { return irq_mask; }
void __disable_irq(void) { irq_mask = 1U; }
void __enable_irq(void) { irq_mask = 0U; }
uint32_t Board_GetAPB1TimerClockHz(void) { return timer_clock_hz; }
void Encoder_Reset(void) { encoder_resets++; }
float Encoder_GetLeftSpeedPPS(void) { return encoder_speed_left; }
float Encoder_GetRightSpeedPPS(void) { return encoder_speed_right; }

static Motor_Diagnostics_t Snapshot(void)
{
    Motor_Diagnostics_t snapshot;
    Motor_GetDiagnostics(&snapshot);
    return snapshot;
}

static void ResetDriver(void)
{
    memset(&test_gpio_a, 0, sizeof(test_gpio_a));
    memset(&test_gpio_b, 0, sizeof(test_gpio_b));
    memset(&test_timer, 0, sizeof(test_timer));
    memset(&test_rcc, 0, sizeof(test_rcc));
    irq_mask = 0U;
    timer_clock_hz = 96000000U;
    Motor_Init();
    Motor_SetSlewRates(20U, 30U);
}

static void Tick(uint32_t count)
{
    uint32_t tick_index;
    for (tick_index = 0U; tick_index < count; ++tick_index) Motor_Update1ms();
}

static void TestInitialization(void)
{
    Motor_Diagnostics_t snapshot;
    ResetDriver();
    snapshot = Snapshot();
    assert(snapshot.timer_clock_hz == 96000000U);
    assert(snapshot.period_counts == 3200U);
    assert(TIM2->PSC == 0U && TIM2->ARR == 3199U);
    assert(TIM2->CCR1 == 3200U && TIM2->CCR2 == 3200U);
    assert(TIM2->CCMR1 == 0x6868U);
    assert(TIM2->CCER == 0x11U);
    assert(snapshot.ramped_left == 0 && snapshot.ramped_right == 0);
    assert(!snapshot.outputs_locked);
    timer_clock_hz = 100000000U;
    Motor_Init();
    assert(TIM2->ARR == 3332U);
    assert(TIM2->CCR1 == 3333U && TIM2->CCR2 == 3333U);
}

static void TestStartAndClamp(void)
{
    Motor_Diagnostics_t snapshot;
    ResetDriver();
    Motor_SetPairPermille(160, -160);
    Tick(1U);
    snapshot = Snapshot();
    assert(snapshot.ramped_left == 20 && snapshot.ramped_right == -20);
    assert((GPIOB->BSRR & 0xF000U) == 0U);
    assert(TIM2->CCR1 == 64U && TIM2->CCR2 == 64U);
    Tick(1U);
    assert((GPIOB->BSRR & (1UL << MOTOR_L_IN1_PIN)) != 0U);
    assert((GPIOB->BSRR & (1UL << MOTOR_R_IN2_PIN)) != 0U);
    Tick(6U);
    snapshot = Snapshot();
    assert(snapshot.ramped_left == 160 && snapshot.ramped_right == -160);
    assert(TIM2->CCR1 == 512U && TIM2->CCR2 == 512U);
    Motor_SetPair(32767, -32768);
    snapshot = Snapshot();
    assert(snapshot.target_left == 1000 && snapshot.target_right == -1000);
    Tick(50U);
    assert(TIM2->CCR1 == 3200U && TIM2->CCR2 == 3200U);
    Motor_Stop();
    assert(Snapshot().ramped_left == 0 && Snapshot().ramped_right == 0);
}

static void TestReverseAndBrake(void)
{
    ResetDriver();
    Motor_SetPairPermille(100, 0);
    Tick(6U);
    Motor_Stop();
    Motor_SetPairPermille(-100, 0);
    Tick(29U);
    assert(Snapshot().ramped_left == 0);
    Tick(1U);
    assert(Snapshot().ramped_left == -20);
    assert((GPIOB->BSRR & 0xF000U) == 0U);
    Tick(1U);
    assert((GPIOB->BSRR & (1UL << MOTOR_L_IN2_PIN)) != 0U);
    Motor_Brake();
    assert(Snapshot().brake_active == 1U);
    assert((GPIOB->BSRR & 0xF000U) == 0xF000U);
    Motor_SetPairPermille(-100, 0);
    Tick(2U);
    assert(Snapshot().ramped_left == 0);
    Tick(1U);
    assert(Snapshot().ramped_left == -20);
    assert((GPIOB->BSRR & 0xF000U) == 0U);
}

static void TestDutyAndConfiguration(void)
{
    uint16_t deadbands[] = {0U, 100U, 500U};
    uint32_t band_index;
    int16_t command;
    ResetDriver();
    Motor_SetSlewRates(0U, 2000U);
    assert(Snapshot().rise_rate == 1U && Snapshot().fall_rate == 1000U);
    Motor_SetDeadband(1000U, 1000U);
    assert(Snapshot().deadband_left == 500U && Snapshot().deadband_right == 500U);
    Motor_SetSlewRates(1000U, 1000U);
    for (band_index = 0U; band_index < 3U; ++band_index) {
        Motor_Stop();
        Motor_SetDeadband(deadbands[band_index], deadbands[band_index]);
        Motor_SetPairPermille(1, 1);
        Tick(32U);
        for (command = 1; command <= 1000; ++command) {
            uint32_t duty = deadbands[band_index] +
                ((1000U - deadbands[band_index]) * (uint32_t)command) / 1000U;
            Motor_SetPairPermille(command, command);
            Tick(2U);
            assert(TIM2->CCR1 == duty * 3200U / 1000U);
            assert(TIM2->CCR2 == duty * 3200U / 1000U);
        }
        Motor_Stop();
        Motor_SetPairPermille(-1, -1);
        Tick(32U);
        for (command = 1; command <= 1000; ++command) {
            uint32_t duty = deadbands[band_index] +
                ((1000U - deadbands[band_index]) * (uint32_t)command) / 1000U;
            Motor_SetPairPermille(-command, -command);
            Tick(2U);
            assert(TIM2->CCR1 == duty * 3200U / 1000U);
            assert(TIM2->CCR2 == duty * 3200U / 1000U);
        }
    }
    Motor_StopWithMode(MOTOR_STOP_COAST);
    assert(TIM2->CCR1 == 3200U && TIM2->CCR2 == 3200U);
    assert(!Snapshot().brake_active);
}

static void TestInterruptState(void)
{
    ResetDriver();
    irq_mask = 1U;
    Motor_SetPairPermille(100, 100);
    Motor_GetDiagnostics(0);
    Motor_GetAppliedPairPermille(0, 0);
    assert(Snapshot().target_left == 100);
    assert(irq_mask == 1U);
    Motor_Stop();
    assert(irq_mask == 1U);
    irq_mask = 0U;
    Motor_Stop();
    assert(irq_mask == 0U);
}

static void TestCleanReinitialization(void)
{
    Motor_Diagnostics_t snapshot;
    ResetDriver();
    Motor_SetSlewRates(100U, 200U);
    TIM2->CR1 = TIM2->CR2 = TIM2->SMCR = TIM2->DIER = 0xFFFFFFFFU;
    TIM2->CCMR1 = TIM2->CCER = TIM2->CNT = 0xFFFFFFFFU;
    Motor_Init();
    snapshot = Snapshot();
    assert(TIM2->CR1 == 129U && TIM2->CR2 == 0U);
    assert(TIM2->SMCR == 0U && TIM2->DIER == 0U && TIM2->CNT == 0U);
    assert(TIM2->CCMR1 == 0x6868U && TIM2->CCER == 0x11U);
    assert(snapshot.rise_rate == 20U && snapshot.fall_rate == 30U);
    assert(TIM2->CCR1 == 3200U && TIM2->CCR2 == 3200U);
}

static void TestIdleAndReverseCountdown(void)
{
    ResetDriver();
    GPIOB->BSRR = 0xA5A5A5A5U;
    Tick(50U);
    assert(GPIOB->BSRR == 0xA5A5A5A5U);
    Motor_SetPairPermille(100, 0);
    Tick(6U);
    Motor_Stop();
    GPIOB->BSRR = 0xA5A5A5A5U;
    Tick(29U);
    assert(Snapshot().reverse_wait_left == 1U);
    assert(GPIOB->BSRR == 0xA5A5A5A5U);
    Tick(1U);
    assert(Snapshot().reverse_wait_left == 0U);
    assert(GPIOB->BSRR == 0xA5A5A5A5U);
    Motor_SetPairPermille(-100, 0);
    Tick(1U);
    assert(Snapshot().ramped_left == -20);
    assert((GPIOB->BSRR & 0xF000U) == 0U);
}

static void TestChangingCommands(void)
{
    uint32_t sequence = 17U;
    uint32_t command_index;
    int16_t previous_left = 0;
    int16_t previous_right = 0;
    ResetDriver();
    for (command_index = 0U; command_index < 10000U; ++command_index) {
        Motor_Diagnostics_t snapshot;
        int16_t left_command;
        int16_t right_command;
        sequence = sequence * 1664525U + 1013904223U;
        left_command = (int16_t)((int32_t)(sequence % 2001U) - 1000);
        sequence = sequence * 1664525U + 1013904223U;
        right_command = (int16_t)((int32_t)(sequence % 2001U) - 1000);
        Motor_SetPairPermille(left_command, right_command);
        Tick(1U);
        snapshot = Snapshot();
        assert(snapshot.target_left == left_command && snapshot.target_right == right_command);
        assert(snapshot.ramped_left >= -1000 && snapshot.ramped_left <= 1000);
        assert(snapshot.ramped_right >= -1000 && snapshot.ramped_right <= 1000);
        assert(snapshot.ramped_left - previous_left >= -30 && snapshot.ramped_left - previous_left <= 30);
        assert(snapshot.ramped_right - previous_right >= -30 && snapshot.ramped_right - previous_right <= 30);
        assert((int32_t)snapshot.ramped_left * previous_left >= 0);
        assert((int32_t)snapshot.ramped_right * previous_right >= 0);
        assert(TIM2->CCR1 <= 3200U && TIM2->CCR2 <= 3200U);
        previous_left = snapshot.ramped_left;
        previous_right = snapshot.ramped_right;
    }
}

static void BenchTick(uint32_t count)
{
    uint32_t tick_index;
    for (tick_index = 0U; tick_index < count; ++tick_index) {
        Motor_Update1ms();
        MotorTest_Tick1ms();
    }
}

static MotorTest_Snapshot_t BenchSnapshot(void)
{
    MotorTest_Snapshot_t snapshot;
    MotorTest_GetSnapshot(&snapshot);
    return snapshot;
}

static void AssertBenchStopped(MotorTest_EndReason_t reason)
{
    assert(BenchSnapshot().state == MOTOR_TEST_ENDED);
    assert(BenchSnapshot().reason == reason);
    assert(Snapshot().target_left == 0 && Snapshot().target_right == 0);
    assert(Snapshot().ramped_left == 0 && Snapshot().ramped_right == 0);
    assert(TIM2->CCR1 == 3200U && TIM2->CCR2 == 3200U);
    MotorTest_AcknowledgeEnd();
    assert(BenchSnapshot().state == MOTOR_TEST_IDLE);
}

static void TestBenchWatchdog(void)
{
    uint32_t initial_resets;
    ResetDriver();
    initial_resets = encoder_resets;
    assert(!MotorTest_StartPermille(0, 0, 25U, 100U));
    assert(!MotorTest_StartPermille(1001, 0, 25U, 100U));
    assert(!MotorTest_StartPermille(100, 0, 25U, 99U));
    assert(encoder_resets == initial_resets);
    assert(MotorTest_StartPermille(100, -100, 25U, 100U));
    assert(encoder_resets == initial_resets + 1U);
    assert(!MotorTest_StartPermille(100, 0, 25U, 100U));
    BenchTick(24U);
    assert(MotorTest_IsRunning());
    BenchTick(1U);
    assert(BenchSnapshot().elapsed_ms == 25U);
    AssertBenchStopped(MOTOR_TEST_END_TIMEOUT);

    assert(MotorTest_StartPermille(100, 0, 0U, 100U));
    BenchTick(MOTOR_TEST_HEARTBEAT_TIMEOUT_MS - 1U);
    assert(MotorTest_IsRunning());
    BenchTick(1U);
    assert(BenchSnapshot().elapsed_ms == MOTOR_TEST_HEARTBEAT_TIMEOUT_MS);
    AssertBenchStopped(MOTOR_TEST_END_HEARTBEAT);

    assert(MotorTest_StartPermille(0, 100, 0U, 100U));
    BenchTick(1000U);
    MotorTest_Heartbeat();
    BenchTick(MOTOR_TEST_HEARTBEAT_TIMEOUT_MS - 1U);
    assert(MotorTest_IsRunning());
    BenchTick(1U);
    assert(BenchSnapshot().elapsed_ms == 1000U + MOTOR_TEST_HEARTBEAT_TIMEOUT_MS);
    AssertBenchStopped(MOTOR_TEST_END_HEARTBEAT);

    assert(MotorTest_StartPermille(100, 0, 0U, 100U));
    BenchTick(10U);
    assert(MotorTest_SetPermille(120, 0));
    Motor_SetDeadband(100U, 100U);
    MotorTest_Stop();
    AssertBenchStopped(MOTOR_TEST_END_STOP);
    assert(Snapshot().deadband_left == 0U && Snapshot().deadband_right == 0U);
    assert(Snapshot().rise_rate == 20U && Snapshot().fall_rate == 30U);

    assert(MotorTest_StartPermille(100, 0, 0U, 100U));
    BenchTick(10U);
    encoder_speed_left = 2000.0f;
    assert(!MotorTest_BrakePulse());
    encoder_speed_left = 0.0f;
    assert(MotorTest_BrakePulse());
    BenchTick(MOTOR_TEST_BRAKE_PULSE_MS - 1U);
    assert(BenchSnapshot().state == MOTOR_TEST_BRAKING);
    BenchTick(1U);
    AssertBenchStopped(MOTOR_TEST_END_BRAKE);
    assert(irq_mask == 0U);
    puts("PASS: actual motor_test.c timeout, 1500ms heartbeat expiry/renewal, explicit STOP, brake pulse and calibration reset (register mocks only).");
}

static void TestSteadyOutputRefresh(void)
{
    ResetDriver();
    Motor_SetPairPermille(10, 0);
    Tick(1U);
    assert((GPIOB->BSRR & 0xF000U) == 0U);
    assert(TIM2->CCR2 == 32U);
    Tick(1U);
    assert((GPIOB->BSRR & (1UL << MOTOR_L_IN1_PIN)) != 0U);
    GPIOB->BSRR = 0xA5A5A5A5U;
    Tick(20U);
    assert(GPIOB->BSRR == 0xA5A5A5A5U && TIM2->CCR2 == 32U);
    Motor_SetDeadband(100U, 0U);
    Tick(1U);
    assert(TIM2->CCR2 == 348U);
    assert((GPIOB->BSRR & (1UL << MOTOR_L_IN1_PIN)) != 0U);
    Motor_SetDeadband(100U, 0U);
    GPIOB->BSRR = 0xA5A5A5A5U;
    Tick(1U);
    assert(GPIOB->BSRR == 0xA5A5A5A5U);
    Motor_SetPairPermille(1000, 1000);
    Tick(50U);
    assert(TIM2->CCR1 == 3200U && TIM2->CCR2 == 3200U);
    GPIOB->BSRR = 0xA5A5A5A5U;
    Tick(50U);
    assert(GPIOB->BSRR == 0xA5A5A5A5U);
    Motor_Stop();
    assert((GPIOB->BSRR & 0xF000U) == 0U);
    Motor_SetPairPermille(10, 0);
    Tick(1U);
    assert((GPIOB->BSRR & 0xF000U) == 0U);
    Tick(1U);
    assert((GPIOB->BSRR & (1UL << MOTOR_L_IN1_PIN)) != 0U);
}

static void TestStickyLock(void)
{
    uint32_t initial_resets;
    ResetDriver();
    Motor_SetPairPermille(100, -100);
    Tick(6U);
    Motor_LockOutputs();
    assert(Motor_OutputsLocked());
    initial_resets = encoder_resets;
    assert(!MotorTest_StartPermille(100, 100, 200U, 100U));
    assert(encoder_resets == initial_resets);
    Motor_SetPairPermille(1000, -1000);
    Motor_SetPair(100, -100);
    Motor_Brake();
    Tick(100U);
    assert(Snapshot().target_left == 0 && Snapshot().target_right == 0);
    assert(Snapshot().ramped_left == 0 && Snapshot().ramped_right == 0);
    assert(Snapshot().brake_active == 0U);
    assert(TIM2->CCR1 == 3200U && TIM2->CCR2 == 3200U);
    Motor_Init();
    assert(Motor_OutputsLocked());
    Motor_SetPairPermille(100, 100);
    Tick(100U);
    assert(Snapshot().ramped_left == 0 && Snapshot().ramped_right == 0);
    assert(irq_mask == 0U);
}

static void TestPreloadStopBoundaries(void)
{
    ResetDriver();
    Motor_SetPairPermille(1, 1);
    Tick(1U);
    assert(TIM2->CCR1 == 3U && TIM2->CCR2 == 3U);
    assert((GPIOB->BSRR & 0xF000U) == 0U);
    Motor_Stop(); /* interrupt first-tick preload */
    assert(TIM2->CCR1 == 3200U && TIM2->CCR2 == 3200U);
    assert((GPIOB->BSRR & 0xF000U) == 0U);
    Motor_SetPairPermille(10, 10); /* same direction; reversal has its own 30ms gate */
    Tick(1U);
    assert((GPIOB->BSRR & 0xF000U) == 0U);
    Tick(1U);
    assert((GPIOB->BSRR & (1UL << MOTOR_L_IN1_PIN)) != 0U);
    assert((GPIOB->BSRR & (1UL << MOTOR_R_IN1_PIN)) != 0U);
    Motor_Stop();
    assert(Snapshot().command_age_ms == 0U);
}

static void TestCommandWatchdog(void)
{
    Motor_Diagnostics_t s;
    ResetDriver();
    Motor_SetPairPermille(200, 0);
    Tick(MOTOR_COMMAND_TIMEOUT_MS - 2U);
    s = Snapshot();
    assert(!s.outputs_locked && !s.command_timeout_latched);
    assert(s.command_age_ms == MOTOR_COMMAND_TIMEOUT_MS - 2U);
    Motor_SetPairPermille(200, 0);
    Tick(MOTOR_COMMAND_TIMEOUT_MS - 1U);
    assert(!Snapshot().outputs_locked);
    Tick(1U);
    s = Snapshot();
    assert(s.outputs_locked && s.command_timeout_latched);
    assert(s.target_left == 0 && s.ramped_left == 0);
    assert(TIM2->CCR1 == 3200U && TIM2->CCR2 == 3200U);
    Motor_SetPairPermille(1000, 1000);
    Motor_KeepAlive();
    Tick(10U);
    assert(Snapshot().outputs_locked && Snapshot().ramped_left == 0);
    Motor_Init();
    assert(Snapshot().outputs_locked && Snapshot().command_timeout_latched);
}

int main(int argc, char **argv)
{
    if (argc == 2 && strcmp(argv[1], "--watchdog") == 0) {
        TestCommandWatchdog();
        puts("PASS: normal command watchdog expires closed; restart rejected.");
        return 0;
    }
    assert(argc == 1);
    TestInitialization();
    TestStartAndClamp();
    TestReverseAndBrake();
    TestDutyAndConfiguration();
    TestInterruptState();
    TestCleanReinitialization();
    TestIdleAndReverseCountdown();
    TestChangingCommands();
    TestBenchWatchdog();
    TestSteadyOutputRefresh();
    TestPreloadStopBoundaries();
    TestStickyLock();
    puts("PASS: compiled motor.c; 6000 signed duty cases, 10000 command changes, init, idle, slew, reverse, brake, IRQ and lock.");
    puts("Register mocks do not reproduce timer preload, physical PWM, current or temperature.");
    return 0;
}
