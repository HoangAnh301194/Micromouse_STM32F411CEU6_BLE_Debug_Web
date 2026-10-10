#include "board.h"
#include "bt_debug.h"
#include "control.h"
#include "encoder.h"
#include "motion.h"
#include "motor.h"
#include "motor_test.h"
#include "motor_trace.h"
#include "uart.h"
#include <stdio.h>
#include <string.h>
#include "system_timer.h"

#include "stm32f4xx.h"

#define DEBUG_PERIOD_MS 100UL
#define CONTROL_FREQ_REPORT_US 1000000UL

static char rx_line[80];
static uint8_t rx_len = 0;
static uint32_t last_motor_report_ms = 0;
static int32_t last_motor_left = 0;
static int32_t last_motor_right = 0;
static uint32_t last_motor_elapsed = 0;
static uint8_t last_key_state = 0U;
static uint8_t auto_motor_test_active = 0U;

/* Test harness output: USART2/JDY-33 and USART1/USB-UART. */
static void LogBoth(const char *message)
{
    BT_SendString(message);
    UART_SendString(message);
}

static void StartAutoMotorDebugTest(void)
{
    if (MotorTest_IsRunning() || !Motion_IsDone()) return;

    /* Keep the current KEY test unchanged: both motors at +100% for 1.5 s. */
    if (MotorTest_StartPermille(1000, 1000, 1500U, 200U)) {
        auto_motor_test_active = 1U;
        last_motor_report_ms = millis();
        last_motor_left = Encoder_GetLeftCount();
        last_motor_right = Encoder_GetRightCount();
        last_motor_elapsed = 0U;
        MotorTrace_Reset();
        LogBoth("AUTO,RUN,1000,1000,1500,200\r\n");
    } else {
        LogBoth("AUTO,REJECTED\r\n");
    }
}

static void ReportMotor(const MotorTest_Snapshot_t *s, uint8_t finished)
{
    int32_t left, right, dleft, dright;
    int32_t speed_left_pps = 0, speed_right_pps = 0;
    int16_t applied_l, applied_r;
    uint32_t delta_ms;
    uint32_t primask;
    char line[192];

    primask = __get_PRIMASK();
    __disable_irq();
    left = Encoder_GetLeftCount();
    right = Encoder_GetRightCount();
    if (!primask) __enable_irq();

    Motor_GetAppliedPairPermille(&applied_l, &applied_r);
    delta_ms = s->elapsed_ms - last_motor_elapsed;
    dleft = left - last_motor_left;
    dright = right - last_motor_right;
    if (delta_ms != 0U) {
        speed_left_pps = (int32_t)(((int64_t)dleft * 1000LL) / delta_ms);
        speed_right_pps = (int32_t)(((int64_t)dright * 1000LL) / delta_ms);
    }

    (void)snprintf(line, sizeof(line),
                   "MTDATA3,%lu,%d,%d,%d,%d,%ld,%ld,%ld,%ld,%lu\r\n",
                   (unsigned long)s->elapsed_ms,
                   (int)s->pwm_left, (int)s->pwm_right,
                   (int)applied_l, (int)applied_r,
                   (long)left, (long)right,
                   (long)speed_left_pps, (long)speed_right_pps,
                   (unsigned long)delta_ms);
    LogBoth(line);

    last_motor_left = left;
    last_motor_right = right;
    last_motor_elapsed = s->elapsed_ms;

    if (finished) {
        const char *why = s->reason == MOTOR_TEST_END_TIMEOUT ? "TIMEOUT" :
                          s->reason == MOTOR_TEST_END_HEARTBEAT ? "HEARTBEAT_LOST" :
                          s->reason == MOTOR_TEST_END_BRAKE ? "BRAKE" : "STOP";
        (void)snprintf(line, sizeof(line), "MTEND,%s\r\n", why);
        LogBoth(line);
        MotorTest_AcknowledgeEnd();
    }
}

static void HandleMotorLine(const char *line)
{
    int left, right;
    unsigned long duration;
    unsigned int interval;
    MotorTest_Snapshot_t s;
    if (strcmp(line, "MT TRACE") == 0) {
        if (MotorTest_IsRunning()) {
            BT_SendString("MTERR,TRACE_BUSY\r\n");
        } else {
            MotorTrace_RequestDump();
            BT_SendString("MTACK,TRACE\r\n");
        }
        return;
    }
    if (strcmp(line, "MT BRAKE") == 0) {
        if (!MotorTest_IsRunning()) {
            BT_SendString("MTERR,NOT_RUNNING\r\n");
        } else if (!MotorTest_BrakePulse()) {
            BT_SendString("MTERR,BRAKE_REQUIRES_LOW_SPEED\r\n");
        } else {
            BT_SendString("MTACK,BRAKE\r\n");
        }
        return;
    }
    if (strcmp(line, "MT STOP") == 0) {
        MotorTest_Stop();
        Motion_Stop();
        BT_SendString("MTACK,STOP\r\n");
        return;
    }
    if (strcmp(line, "MT PING") == 0) {
        BT_SendString("MTREADY,3\r\n");
        return;
    }
    if (strcmp(line, "MT HB") == 0) {
        MotorTest_Heartbeat();
        return;
    }
    if (strncmp(line, "MT CFG ", 7) == 0) {
        unsigned int dl, dr, rise, fall;
        if (MotorTest_IsRunning() || !Motion_IsDone() ||
            sscanf(line, "MT CFG %u %u %u %u", &dl, &dr, &rise, &fall) != 4 ||
            dl > 500U || dr > 500U || rise == 0U || rise > 1000U ||
            fall == 0U || fall > 1000U) {
            BT_SendString("MTERR,CONFIG_INVALID_OR_BUSY\r\n");
        } else {
            Motor_SetDeadband((uint16_t)dl, (uint16_t)dr);
            Motor_SetSlewRates((uint16_t)rise, (uint16_t)fall);
            BT_Printf("MTACK,CFG,%u,%u,%u,%u\r\n", dl, dr, rise, fall);
        }
        return;
    }
    if (strncmp(line, "MT SETP ", 8) == 0) {
        int lc, rc;
        if (sscanf(line, "MT SETP %d %d", &lc, &rc) != 2 ||
            lc < -1000 || lc > 1000 || rc < -1000 || rc > 1000 ||
            !MotorTest_SetPermille((int16_t)lc, (int16_t)rc)) {
            BT_SendString("MTERR,SET_INVALID_OR_NOT_RUNNING\r\n");
        } else {
            BT_Printf("MTACK,SETP,%d,%d\r\n", lc, rc);
        }
        return;
    }
    if (strncmp(line, "MT RUNP ", 8) == 0) {
        int lc, rc;
        unsigned long duration_ms;
        unsigned int log_ms;
        MotorTest_GetSnapshot(&s);
        if (sscanf(line, "MT RUNP %d %d %lu %u", &lc, &rc,
                   &duration_ms, &log_ms) != 4 ||
            !Motion_IsDone() || !Control_IsRunning() ||
            s.state != MOTOR_TEST_IDLE ||
            lc < -1000 || lc > 1000 || rc < -1000 || rc > 1000 ||
            log_ms < 100U || log_ms > 1000U ||
            !MotorTest_StartPermille((int16_t)lc, (int16_t)rc,
                                     (uint32_t)duration_ms, (uint16_t)log_ms)) {
            BT_SendString("MTERR,RUN_INVALID_OR_BUSY\r\n");
        } else {
            MotorTrace_Reset();
            last_motor_report_ms = millis();
            last_motor_left = last_motor_right = 0;
            last_motor_elapsed = 0;
            BT_Printf("MTACK,RUNP,%d,%d,%lu,%u\r\n",
                      lc, rc, duration_ms, log_ms);
        }
        return;
    }
    if (sscanf(line, "MT RUN %d %d %lu %u",
               &left, &right, &duration, &interval) == 4) {
        if (!Motion_IsDone() || MotorTest_IsRunning() ||
            !Control_IsRunning() ||
            left < -100 || left > 100 || right < -100 || right > 100 ||
            interval < 100 || interval > 1000 ||
            (left == 0 && right == 0)) {
            BT_SendString("MTERR,INVALID_OR_BUSY\r\n");
            return;
        }
        MotorTest_GetSnapshot(&s);
        if (s.state != MOTOR_TEST_IDLE) {
            BT_SendString("MTERR,BUSY\r\n");
            return;
        }
        if (!MotorTest_Start((int16_t)left, (int16_t)right,
                             (uint32_t)duration, (uint16_t)interval)) {
            BT_SendString("MTERR,REJECTED\r\n");
            return;
        }
        MotorTrace_Reset();
        last_motor_report_ms = millis();
        last_motor_left = last_motor_right = 0;
        last_motor_elapsed = 0;
        BT_Printf("MTACK,RUN,%d,%d,%lu,%u\r\n",
                  left, right, duration, interval);
        return;
    }
    BT_SendString("MTERR,UNKNOWN\r\n");
}

static void ProcessBleByte(char ch)
{
    if (rx_len == 0 && (ch == 'S' || ch == 's')) {
        /* Emergency stop remains a one-byte command (also handled in legacy UI). */
        MotorTest_Stop();
        Motion_Stop();
        rx_len = 0;
        return;
    }
    if (ch == '\r' || ch == '\n') {
        if (rx_len) {
            rx_line[rx_len] = 0;
            if (strncmp(rx_line, "MT ", 3) == 0) {
                HandleMotorLine(rx_line);
            }
            rx_len = 0;
        }
        return;
    }
    if (rx_len < sizeof(rx_line)-1) rx_line[rx_len++] = ch;
    else rx_len = 0;
}

static void PollMotorTelemetry(uint32_t now)
{
    MotorTest_Snapshot_t s;
    MotorTest_GetSnapshot(&s);
    if (s.state == MOTOR_TEST_IDLE) return;
    if (s.state == MOTOR_TEST_ENDED ||
        (uint32_t)(now - last_motor_report_ms) >= s.log_interval_ms) {
        last_motor_report_ms = now;
        ReportMotor(&s, s.state == MOTOR_TEST_ENDED);
    }
}


int main(void)
{
    uint32_t last_debug_ms = 0;
    uint32_t last_control_report_us;
    uint32_t last_control_tick_count;

    /* Dedicated motor/encoder test: IMU, I2C and IR are intentionally off. */
    Board_Init();
    SystemTimer_Init();
    Motor_Init();
    Encoder_Init();
    BT_Init();       /* USART2 PA2/PA3, JDY-33 */
    UART_Init();     /* USART1 PA9/PA10, USB-UART debug */
    Motion_Init();

    Control_Init(0);  /* TIM11 at 1 kHz, no MPU integration in this test. */
    Control_Start();
    last_control_report_us = micros();
    last_control_tick_count = Control_GetTickCount();

    LogBoth("\r\n[BOOT] Motor/encoder test harness ready\r\n");
    LogBoth("KEY: motors +100% for 1500ms; USART1/USART2 telemetry\r\n");

    while (1) {
        uint32_t now = millis();
        uint8_t key_pressed = Board_ButtonPressed();

        /*
         * Debug observed TIM11 rate with TIM5, without UART in the ISR.
         * Keep the KEY motor test and motor timing exactly as before.
         */
        uint32_t control_now_us = micros();
        uint32_t control_elapsed_us = control_now_us - last_control_report_us;
        if (control_elapsed_us >= CONTROL_FREQ_REPORT_US) {
            uint32_t current_ticks = Control_GetTickCount();
            uint32_t delta_ticks = current_ticks - last_control_tick_count;
            uint32_t measured_hz = (uint32_t)(
                ((uint64_t)delta_ticks * 1000000ULL + control_elapsed_us / 2U)
                / control_elapsed_us
            );

            UART_Printf("[CTRL] target=%lu Hz actual=%lu Hz ticks=%lu window_us=%lu tx_drop=%lu dma_err=%lu\r\n",
                        (unsigned long)CONTROL_FREQUENCY_HZ,
                        (unsigned long)measured_hz,
                        (unsigned long)delta_ticks,
                        (unsigned long)control_elapsed_us,
                        (unsigned long)UART_GetDroppedMessages(),
                        (unsigned long)UART_GetDmaErrors());

            last_control_report_us = control_now_us;
            last_control_tick_count = current_ticks;
        }

        if (key_pressed && !last_key_state) {
            StartAutoMotorDebugTest();
        }
        last_key_state = key_pressed;

        if (auto_motor_test_active) {
            if (MotorTest_IsRunning()) {
                MotorTest_Heartbeat();
            } else {
                auto_motor_test_active = 0U;
            }
        }

        if (BT_Available()) {
            ProcessBleByte(BT_ReceiveChar());
        }
        PollMotorTelemetry(now);
        MotorTrace_Poll(now);


        if (!MotorTest_IsRunning() && (now - last_debug_ms) >= DEBUG_PERIOD_MS) {
            uint32_t primask = __get_PRIMASK();
            int32_t left_count, right_count;
            Motion_State_t state;

            last_debug_ms = now;
            __disable_irq();
            left_count = Encoder_GetLeftCount();
            right_count = Encoder_GetRightCount();
            state = Motion_GetState();
            if (!primask) __enable_irq();

            UART_Printf("t=%lu state=%u encL=%ld encR=%ld\r\n",
                        (unsigned long)now, (unsigned)state,
                        (long)left_count, (long)right_count);
        }
    }
}
