#include "board.h"
#include "bt_debug.h"
#include "control.h"
#include "encoder.h"
#include "i2c.h"
#include "ir_sensor.h"
#include "motion.h"
#include "motor.h"
#include "motor_test.h"
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
static uint16_t last_motor_elapsed = 0;

static void HandleBleCommand(char c);

static void ReportMotor(const MotorTest_Snapshot_t *s, uint8_t finished)
{
    int32_t left, right;
    uint16_t delta_ms;
    int32_t dleft, dright;
    uint32_t key = __get_PRIMASK();
    __disable_irq();
    left = Encoder_GetLeftCount();
    right = Encoder_GetRightCount();
    if (!key) __enable_irq();

    delta_ms = (uint16_t)(s->elapsed_ms - last_motor_elapsed);
    dleft = left - last_motor_left;
    dright = right - last_motor_right;
    BT_Printf("MTDATA,%u,%d,%d,%ld,%ld,%ld,%ld,%u\r\n",
              (unsigned)s->elapsed_ms, (int)s->pwm_left, (int)s->pwm_right,
              (long)left, (long)right,
              (long)(delta_ms ? dleft * 1000L / delta_ms : 0),
              (long)(delta_ms ? dright * 1000L / delta_ms : 0),
              (unsigned)delta_ms);
    last_motor_left = left;
    last_motor_right = right;
    last_motor_elapsed = s->elapsed_ms;
    if (finished) {
        BT_Printf("MTEND,%s\r\n",
                  s->reason == MOTOR_TEST_END_TIMEOUT ? "TIMEOUT" : "STOP");
        MotorTest_AcknowledgeEnd();
    }
}

static void HandleMotorLine(const char *line)
{
    int left, right, duration, interval;
    MotorTest_Snapshot_t s;
    if (strcmp(line, "MT STOP") == 0) {
        MotorTest_Stop();
        Motion_Stop();
        BT_SendString("MTACK,STOP\r\n");
        return;
    }
    if (strcmp(line, "MT PING") == 0) {
        BT_SendString("MTREADY,1\r\n");
        return;
    }
    if (sscanf(line, "MT RUN %d %d %d %d",
               &left, &right, &duration, &interval) == 4) {
        if (!Motion_IsDone() || MotorTest_IsRunning() ||
            !Control_IsRunning() ||
            left < -30 || left > 30 || right < -30 || right > 30 ||
            duration < 100 || duration > 1000 ||
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
                             (uint16_t)duration, (uint16_t)interval)) {
            BT_SendString("MTERR,REJECTED\r\n");
            return;
        }
        last_motor_report_ms = millis();
        last_motor_left = last_motor_right = 0;
        last_motor_elapsed = 0;
        BT_Printf("MTACK,RUN,%d,%d,%d,%d\r\n",
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
