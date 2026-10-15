#if defined(MOTOR_STARTUP_BENCH)
#if defined(MOTOR_AUTO_BENCH) || defined(MOTOR_PWM_DIAG) || defined(MPU_INTEGRATION_BENCH) || defined(MPU_FRESHNESS_DIAG)
#error "Startup bench must run in isolation"
#endif
#include "board.h"
#include "control.h"
#include "encoder.h"
#include "motor.h"
#include "motor_startup.h"
#include "system_timer.h"
#include "uart.h"
#include "stm32f4xx.h"
#include <stdio.h>
#include <string.h>

static uint8_t watchdog_probe;

static uint8_t Watchdog_Init(void)
{
    uint32_t start = micros();
    IWDG->KR = 0xCCCCU;
    IWDG->KR = 0x5555U;
    IWDG->PR = 5U;
    IWDG->RLR = 124U;
    while (IWDG->SR != 0U) {
        if ((uint32_t)(micros() - start) >= 100000UL) return 0U;
    }
    IWDG->KR = 0xAAAAU;
    return 1U;
}

static void Profile(void)
{
    UART_Printf("STARTUP_PROFILE,version=2,cap=200,pulse_ms=250,rest_ms=750,quiet_ms=300,link_ms=400,max_pulses=128,session_ms=180000,rise=20,fall=30,deadband=0,wdg_ms=500,core_hz=%lu,pwm_hz=%lu,reset_flags=%lu,build=%s %s\r\n",
        (unsigned long)SystemCoreClock,
        (unsigned long)(Board_GetAPB1TimerClockHz() / (TIM2->ARR + 1U)),
        (unsigned long)RCC->CSR, __DATE__, __TIME__);
}

static void Status(const char *name, const MotorStartup_Snapshot_t *snapshot)
{
    int16_t left, right;
    Control_Timing_t timing;
    uint32_t mask = __get_PRIMASK();
    int32_t counts_left, counts_right;
    __disable_irq();
    counts_left = Encoder_GetLeftCount();
    counts_right = Encoder_GetRightCount();
    if (strcmp(name, "STARTUP_END") == 0) {
        counts_left = snapshot->end_left;
        counts_right = snapshot->end_right;
    }
    Motor_GetAppliedPairPermille(&left, &right);
    if (!mask) __enable_irq();
    Control_GetTiming(&timing);
    UART_Printf("%s,id=%lu,state=%u,fault=%u,ms=%lu,t=%lu,rest=%lu,q=%lu,wheel=%u,duty=%u,l=%ld,r=%ld,al=%d,ar=%d,cl=%lu,cr=%lu,odr=%lu,ticks=%lu,late=%lu,gaps=%lu,tx=%lu,rx=%lu\r\n",
        name, (unsigned long)snapshot->id,
        (unsigned)snapshot->state, (unsigned)snapshot->fault,
        (unsigned long)millis(), (unsigned long)snapshot->elapsed_ms,
        (unsigned long)snapshot->rest_ms, (unsigned long)snapshot->quiet_ms,
        (unsigned)snapshot->wheel, (unsigned)snapshot->duty,
        (long)counts_left, (long)counts_right,
        (int)left, (int)right, (unsigned long)TIM2->CCR2, (unsigned long)TIM2->CCR1,
        (unsigned long)(GPIOB->ODR & 0xF000U), (unsigned long)Control_GetTickCount(),
        (unsigned long)timing.late_over_50us, (unsigned long)timing.gap_over_1500us,
        (unsigned long)UART_GetDroppedMessages(),
        (unsigned long)(UART_GetRxOverrunCount() + UART_GetRxFramingCount() +
                        UART_GetRxNoiseCount() + UART_GetRxRingOverflowCount()));
}

static void Command(const char *line)
{
    MotorStartup_Snapshot_t snapshot;
    unsigned long id;
    unsigned int duty;
    char wheel, extra;
    if (watchdog_probe) return;
    if (strcmp(line, "STARTUP WATCHDOG") == 0) {
        MotorStartup_GetSnapshot(&snapshot);
        if (snapshot.state == MOTOR_STARTUP_DISARMED && snapshot.quiet_ms >= MOTOR_STARTUP_QUIET_MS) {
            RCC->CSR |= RCC_CSR_RMVF;
            watchdog_probe = 1U;
            UART_SendString("STARTUP_WATCHDOG,requested=1\r\n");
            return;
        }
        MotorStartup_Abort(MOTOR_STARTUP_UART);
        return;
    }
    if (strcmp(line, "STARTUP PING") == 0) Profile();
    else if (strcmp(line, "STARTUP KEEP") == 0) MotorStartup_Heartbeat();
    else if (strcmp(line, "STARTUP STOP") == 0) MotorStartup_Abort(MOTOR_STARTUP_HOST_STOP);
    else if (strcmp(line, "STARTUP ARM") == 0) {
        if (!MotorStartup_Arm()) UART_SendString("STARTUP_REJECT,reason=arm\r\n");
    } else if (sscanf(line, "STARTUP PULSE %3lu %c %3u %c", &id, &wheel, &duty, &extra) == 3 &&
               (wheel == 'l' || wheel == 'r') && duty <= MOTOR_STARTUP_MAX_PERMILLE) {
        if (MotorStartup_Pulse((uint32_t)id, wheel == 'l' ? 0U : 1U, (uint16_t)duty)) {
            MotorStartup_GetSnapshot(&snapshot);
            Status("STARTUP_BEGIN", &snapshot);
            return;
        }
        UART_SendString("STARTUP_REJECT,reason=pulse\r\n");
    } else {
        MotorStartup_Abort(MOTOR_STARTUP_UART);
        UART_SendString("STARTUP_REJECT,reason=command\r\n");
    }
    if (strcmp(line, "STARTUP KEEP") != 0) {
        MotorStartup_GetSnapshot(&snapshot);
        Status("STARTUP_STATUS", &snapshot);
    }
}

int MotorStartupBench_Main(void)
{
    char line[64];
    uint32_t length = 0U, last_sample = 0U, last_completed = 0U, last_tick = 0U;
    uint8_t fault_reported = 0U;
    Board_Init();
    SystemTimer_Init();
    UART_Init();
    if (Board_GetClockFault()) {
        UART_SendString("STARTUP_BOOT_FAIL,reason=clock\r\n");
        while (1) { }
    }
    Motor_Init();
    Motor_Stop();
    Encoder_Init();
    MotorStartup_Init();
    Control_Init(0);
    Control_SetDiagnosticOnly(1U);
    Control_Start();
    if (!Watchdog_Init()) {
        MotorStartup_Abort(MOTOR_STARTUP_TIMING);
        UART_SendString("STARTUP_BOOT_FAIL,reason=watchdog_init\r\n");
        while (1) { __WFI(); }
    }
    Profile();
    while (1) {
        uint8_t byte;
        MotorStartup_Snapshot_t snapshot;
        Control_Timing_t timing;
        uint32_t now = millis();
        uint32_t tick = Control_GetTickCount();
        if (tick != last_tick && !watchdog_probe) {
            IWDG->KR = 0xAAAAU;
            last_tick = tick;
        }
        while (UART_TryReadByte(&byte)) {
            if (byte == '\r') continue;
            if (byte == '\n') {
                line[length] = '\0';
                if (length) Command(line);
                length = 0U;
            } else if (length + 1U < sizeof(line)) line[length++] = (char)byte;
            else {
                MotorStartup_Abort(MOTOR_STARTUP_UART);
                length = 0U;
            }
        }
        Control_GetTiming(&timing);
        if (timing.gap_over_1500us) MotorStartup_Abort(MOTOR_STARTUP_TIMING);
        if (UART_GetDroppedMessages() || UART_GetRxOverrunCount() || UART_GetRxFramingCount() ||
            UART_GetRxNoiseCount() || UART_GetRxRingOverflowCount() || UART_GetDmaErrors())
            MotorStartup_Abort(MOTOR_STARTUP_UART);
        MotorStartup_GetSnapshot(&snapshot);
        if (snapshot.completed_id != last_completed) {
            Status("STARTUP_END", &snapshot);
            last_completed = snapshot.completed_id;
        }
        if (snapshot.state == MOTOR_STARTUP_FAULT && !fault_reported) {
            Status("STARTUP_FAULT", &snapshot);
            fault_reported = 1U;
        }
        if (now - last_sample >= 20U) {
            last_sample = now;
            Status("STARTUP_SAMPLE", &snapshot);
        }
        __WFI();
    }
}
#endif
