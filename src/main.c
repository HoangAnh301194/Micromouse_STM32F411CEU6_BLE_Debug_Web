#include "board.h"
#include "bt_debug.h"
#include "control.h"
#include "encoder.h"
#include "i2c.h"
#include "ir_sensor.h"
#include "motion.h"
#include "motor.h"
#include "motor_test.h"
#include "motor_trace.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "mpu6050.h"
#include "system_timer.h"

#include "stm32f4xx.h"

#define DEBUG_PERIOD_MS 100UL

static MPU6050_Handle_t mpu;
static char rx_line[80];
static uint8_t rx_len = 0;
static uint32_t last_motor_report_ms = 0;
static int32_t last_motor_left = 0;
static int32_t last_motor_right = 0;
static uint32_t last_motor_elapsed = 0;

static void HandleBleCommand(char c);

static void ReportMotor(const MotorTest_Snapshot_t *s, uint8_t finished)
{
    int32_t left, right;
    int16_t applied_l, applied_r;
    uint32_t delta_ms;
    int32_t dleft, dright;
    uint32_t key = __get_PRIMASK();
    __disable_irq();
    left = Encoder_GetLeftCount();
    right = Encoder_GetRightCount();
    if (!key) __enable_irq();

    Motor_GetAppliedPairPermille(&applied_l, &applied_r);
    delta_ms = s->elapsed_ms - last_motor_elapsed;
    dleft = left - last_motor_left;
    dright = right - last_motor_right;
    BT_Printf("MTDATA3,%lu,%d,%d,%d,%d,%ld,%ld,%ld,%ld,%lu\r\n",
              (unsigned long)s->elapsed_ms,
              (int)s->pwm_left, (int)s->pwm_right,
              (int)applied_l, (int)applied_r,
              (long)left, (long)right,
              (long)(delta_ms ? dleft * 1000L / delta_ms : 0),
              (long)(delta_ms ? dright * 1000L / delta_ms : 0),
              (unsigned long)delta_ms);
    last_motor_left = left;
    last_motor_right = right;
    last_motor_elapsed = s->elapsed_ms;
    if (finished) {
        const char *why = s->reason == MOTOR_TEST_END_TIMEOUT ? "TIMEOUT" :
                          s->reason == MOTOR_TEST_END_HEARTBEAT ? "HEARTBEAT_LOST" :
                          s->reason == MOTOR_TEST_END_BRAKE ? "BRAKE" : "STOP";
        BT_Printf("MTEND,%s\r\n", why);
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
        } else {
            MotorTest_BrakePulse();
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
        if (MotorTest_IsRunning() ||
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
            } else if (rx_len == 1) {
                HandleBleCommand(rx_line[0]);
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


static void HandleBleCommand(char c)
{
    float yaw = MPU6050_GetYaw(&mpu);
    if (MotorTest_IsRunning()) return;

    switch (c) {
    case 'f':
    case 'F':
        if (Motion_IsDone()) Motion_CommandStraight(180.0f, yaw);
        break;

    case 'l':
    case 'L':
        if (Motion_IsDone()) Motion_CommandTurn(90.0f, yaw);
        break;

    case 'r':
    case 'R':
        if (Motion_IsDone()) Motion_CommandTurn(-90.0f, yaw);
        break;

    case 's':
    case 'S':
        Motion_Stop();
        break;

    case 'z':
    case 'Z':
        if (Motion_IsDone()) MPU6050_ResetYaw(&mpu);
        break;

    default:
        break;
    }
}

int main(void)
{
    uint32_t last_debug_ms = 0;
    MPU6050_Status mpu_status;

    Board_Init();
    SystemTimer_Init();

    Motor_Init();
    Encoder_Init();

    I2C_Init(I2C_1, 1);
    I2C_DMA_Init(I2C_1);

    BT_Init();
    IR_Init();

    mpu_status = MPU6050_Init(&mpu);
    if (mpu_status == MPU6050_OK) {
        mpu_status = MPU6050_Calibrate(&mpu);
    }

    Motion_Init();

    if (mpu_status != MPU6050_OK) {
        Motor_Stop();
        Board_StatusLed(1);
        BT_SendString("[BOOT] MPU6050 init/calibration failed\r\n");

        while (1) {
        }
    }

    /*
     * TIM10 belongs to ir_sensor.c.
     * TIM11 belongs to control.c.
     * main.c only performs system orchestration.
     */
    IR_StartScan();
    Control_Init(&mpu);
    Control_Start();

    BT_SendString("\r\n[BOOT] Minimal core ready\r\n");
    BT_SendString("Commands: F=straight 180mm, L=left 90, R=right 90, S=stop, Z=zero yaw\r\n");

    while (1) {
        uint32_t now = millis();

        if (BT_Available()) {
            ProcessBleByte(BT_ReceiveChar());
        }
        PollMotorTelemetry(now);
        MotorTrace_Poll(now);


        if (!MotorTest_IsRunning() && (now - last_debug_ms) >= DEBUG_PERIOD_MS) {
            int32_t left_count;
            int32_t right_count;
            float yaw;
            float gyro;
            float distance;
            float angle_error;
            Motion_State_t state;
            uint32_t primask;

            last_debug_ms = now;

            /* Snapshot shared state quickly; formatting/transmission stays outside ISR. */
            primask = __get_PRIMASK();
            __disable_irq();

            left_count = Encoder_GetLeftCount();
            right_count = Encoder_GetRightCount();
            yaw = MPU6050_GetYaw(&mpu);
            gyro = MPU6050_GetGyroZ(&mpu);
            distance = Motion_GetDistanceMm();
            angle_error = Motion_GetAngleErrorDeg();
            state = Motion_GetState();

            if (!primask) {
                __enable_irq();
            }

            BT_Printf("t=%lu state=%u encL=%ld encR=%ld yaw=%.2f gz=%.2f d=%.1f e=%.2f\r\n",
                      (unsigned long)now,
                      (unsigned int)state,
                      (long)left_count,
                      (long)right_count,
                      yaw,
                      gyro,
                      distance,
                      angle_error);
        }
    }
}
