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
#if defined(MOTOR_STARTUP_HIGH_SPEED)
#include "mpu6050.h"
#include "ir_sensor.h"
#include "bt_debug.h"
static MPU6050_Handle_t bench_mpu;
static uint32_t ir_scans, ir_progress_ms, imu_age_max;
static uint8_t sensors_ready;

static void Sensors_Poll(void)
{
    IR_Data_t sample;
    MPU6050_Update(&bench_mpu);
    if (IR_GetLatest(&sample)) {
        ir_scans = sample.scan_count;
        ir_progress_ms = millis();
        IR_StartScan();
    }
    if (bench_mpu.cal_state == 4U) MotorStartup_Abort(MOTOR_STARTUP_SENSOR);
    if (bench_mpu.calibrated && bench_mpu.cal_state == 3U && ir_scans >= 10U)
        sensors_ready = 1U;
    if (sensors_ready) {
        uint32_t age = micros() - bench_mpu.last_good_us;
        if (age > imu_age_max) imu_age_max = age;
        if (age > 15000U || millis() - ir_progress_ms > 100U ||
            bench_mpu.error_reads || bench_mpu.timeout_reads)
            MotorStartup_Abort(MOTOR_STARTUP_SENSOR);
    }
}

static void Health(void)
{
    Control_Timing_t timing;
    Control_GetTiming(&timing);
    UART_Printf("STARTUP_HEALTH,ready=%u,good=%lu,fresh=%lu,err=%lu,timeout=%lu,ir=%lu,age_max=%lu,isr=%lu,adc_div=4\r\n",
        (unsigned)sensors_ready, (unsigned long)bench_mpu.good_reads,
        (unsigned long)bench_mpu.fresh_gyro_reads, (unsigned long)bench_mpu.error_reads,
        (unsigned long)bench_mpu.timeout_reads, (unsigned long)ir_scans,
        (unsigned long)imu_age_max, (unsigned long)timing.exec_max_cycles);
}
#endif

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
#if defined(MOTOR_STARTUP_HIGH_SPEED)
    UART_Printf("STARTUP_PROFILE,version=3,cap=1000,pulse_ms=600,rest_ms=900,quiet_ms=300,link_ms=400,max_pulses=32,session_ms=90000,rise=20,fall=30,deadband=0,wdg_ms=500,core_hz=%lu,pwm_hz=%lu,reset_flags=%lu,build=%s %s\r\n",
#else
    UART_Printf("STARTUP_PROFILE,version=2,cap=200,pulse_ms=250,rest_ms=750,quiet_ms=300,link_ms=400,max_pulses=128,session_ms=180000,rise=20,fall=30,deadband=0,wdg_ms=500,core_hz=%lu,pwm_hz=%lu,reset_flags=%lu,build=%s %s\r\n",
#endif
        (unsigned long)SystemCoreClock,
        (unsigned long)(Board_GetAPB1TimerClockHz() / (TIM2->ARR + 1U)),
        (unsigned long)RCC->CSR, __DATE__, __TIME__);
}

static void Status(const char *name, const MotorStartup_Snapshot_t *snapshot)
{
    int16_t left, right;
    Control_Timing_t timing;
    MotorStartup_Snapshot_t coherent;
    uint32_t mask = __get_PRIMASK();
    uint32_t compare_left, compare_right, direction, sample_ms, sample_ticks;
    int32_t counts_left, counts_right;
    __disable_irq();
    MotorStartup_GetSnapshot(&coherent);
    snapshot = &coherent;
    counts_left = Encoder_GetLeftCount();
    counts_right = Encoder_GetRightCount();
    if (strcmp(name, "STARTUP_END") == 0) {
        counts_left = snapshot->end_left;
        counts_right = snapshot->end_right;
    }
    Motor_GetAppliedPairPermille(&left, &right);
    Control_GetTiming(&timing);
    compare_left = TIM2->CCR2;
    compare_right = TIM2->CCR1;
    direction = GPIOB->ODR & 0xF000U;
    sample_ms = millis();
    sample_ticks = Control_GetTickCount();
    if (!mask) __enable_irq();
    UART_Printf("%s,id=%lu,state=%u,fault=%u,ms=%lu,t=%lu,rest=%lu,q=%lu,wheel=%u,duty=%u,l=%ld,r=%ld,al=%d,ar=%d,cl=%lu,cr=%lu,odr=%lu,ticks=%lu,late=%lu,gaps=%lu,tx=%lu,rx=%lu\r\n",
        name, (unsigned long)snapshot->id,
        (unsigned)snapshot->state, (unsigned)snapshot->fault,
        (unsigned long)sample_ms, (unsigned long)snapshot->elapsed_ms,
        (unsigned long)snapshot->rest_ms, (unsigned long)snapshot->quiet_ms,
        (unsigned)snapshot->wheel, (unsigned)snapshot->duty,
        (long)counts_left, (long)counts_right,
        (int)left, (int)right, (unsigned long)compare_left, (unsigned long)compare_right,
        (unsigned long)direction, (unsigned long)sample_ticks,
        (unsigned long)timing.late_over_50us, (unsigned long)timing.gap_over_1500us,
        (unsigned long)UART_GetDroppedMessages(),
        (unsigned long)(UART_GetRxOverrunCount() + UART_GetRxFramingCount() +
                        UART_GetRxNoiseCount() + UART_GetRxRingOverflowCount()));
#if defined(MOTOR_STARTUP_HIGH_SPEED)
    if (strcmp(name, "STARTUP_END") == 0)
        UART_Printf("STARTUP_PERF,id=%lu,n=%lu,cycles=%lu,max=%lu,rev=%u\r\n",
            (unsigned long)snapshot->id, (unsigned long)snapshot->cycle_calls,
            (unsigned long)snapshot->cycle_total, (unsigned long)snapshot->cycle_max,
            (unsigned)MOTOR_DRIVER_REVISION);
#endif
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
#if defined(MOTOR_STARTUP_HIGH_SPEED)
        if (!sensors_ready) {
            UART_SendString("STARTUP_REJECT,reason=sensors_not_ready\r\n");
            return;
        }
#endif
        if (!MotorStartup_Arm()) UART_SendString("STARTUP_REJECT,reason=arm\r\n");
    } else if (sscanf(line, "STARTUP PULSE %3lu %c %4u %c", &id, &wheel, &duty, &extra) == 3 &&
               (wheel == 'l' || wheel == 'r'
#if defined(MOTOR_STARTUP_HIGH_SPEED)
                || wheel == 'b'
#endif
               ) && duty <= MOTOR_STARTUP_MAX_PERMILLE) {
        if (MotorStartup_Pulse((uint32_t)id, wheel == 'l' ? 0U : wheel == 'r' ? 1U : 2U, (uint16_t)duty)) {
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
#if defined(MOTOR_STARTUP_HIGH_SPEED)
    uint32_t last_health = 0U;
#endif
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
#if defined(MOTOR_STARTUP_HIGH_SPEED)
    if (MPU6050_Init(&bench_mpu) != MPU6050_OK) {
        Motor_LockOutputs();
        UART_SendString("STARTUP_BOOT_FAIL,reason=imu_init\r\n");
        while (1) { __WFI(); }
    }
    MPU6050_Calibrate(&bench_mpu);
    RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;
    (void)RCC->APB2ENR;
    ADC->CCR = (ADC->CCR & ~ADC_CCR_ADCPRE) | (1UL << 16);
    IR_Init();
    IR_StartScan();
    BT_Init();
    Control_Init(&bench_mpu);
#else
    Control_Init(0);
#endif
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
#if defined(MOTOR_STARTUP_HIGH_SPEED)
        Sensors_Poll();
        if (now - last_health >= 500U) {
            last_health = now;
            Health();
        }
#endif
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
#if defined(MOTOR_STARTUP_HIGH_SPEED)
        if (now - last_sample >= 40U) {
            BT_SendString("MOTOR,driver-load\r\n");
#else
        if (now - last_sample >= 20U) {
#endif
            last_sample = now;
            Status("STARTUP_SAMPLE", &snapshot);
        }
        __WFI();
    }
}
#endif
