#ifndef MOTOR_TEST_H
#define MOTOR_TEST_H

#include <stdint.h>

/* Open-loop motor test: signed percent command in [-100, 100].
 * duration_ms=0 means continuous until STOP or heartbeat loss.
 * The 1 kHz scheduler handles timeout/heartbeat; debug is in main loop. */
#define MOTOR_TEST_MAX_PWM              100
#define MOTOR_TEST_MIN_LOG_MS           100U
#define MOTOR_TEST_MAX_LOG_MS           1000U
#define MOTOR_TEST_HEARTBEAT_TIMEOUT_MS 1500U

typedef enum {
    MOTOR_TEST_IDLE = 0,
    MOTOR_TEST_RUNNING,
    MOTOR_TEST_ENDED
} MotorTest_State_t;

typedef enum {
    MOTOR_TEST_END_TIMEOUT = 0,
    MOTOR_TEST_END_STOP,
    MOTOR_TEST_END_HEARTBEAT
} MotorTest_EndReason_t;

typedef struct {
    MotorTest_State_t state;
    MotorTest_EndReason_t reason;
    uint32_t elapsed_ms;
    uint32_t duration_ms;
    uint16_t log_interval_ms;
    int16_t pwm_left;
    int16_t pwm_right;
} MotorTest_Snapshot_t;

uint8_t MotorTest_Start(int16_t left, int16_t right,
                        uint32_t duration_ms, uint16_t log_interval_ms);
void MotorTest_Stop(void);
void MotorTest_Heartbeat(void);
void MotorTest_Tick1ms(void);
uint8_t MotorTest_IsRunning(void);
void MotorTest_GetSnapshot(MotorTest_Snapshot_t *out);
void MotorTest_AcknowledgeEnd(void);

#endif
