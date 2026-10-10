#ifndef MOTOR_TEST_H
#define MOTOR_TEST_H

#include <stdint.h>

/* Dedicated, time-limited open-loop motor test. TIM11 enforces its deadline.
 * Do not print, parse BLE, or perform floating-point formatting in its ISR. */
#define MOTOR_TEST_MAX_PWM         30
#define MOTOR_TEST_MIN_DURATION_MS 100U
#define MOTOR_TEST_MAX_DURATION_MS 1000U
#define MOTOR_TEST_MIN_LOG_MS      100U
#define MOTOR_TEST_MAX_LOG_MS      1000U

typedef enum {
    MOTOR_TEST_IDLE = 0,
    MOTOR_TEST_RUNNING,
    MOTOR_TEST_ENDED
} MotorTest_State_t;

typedef enum {
    MOTOR_TEST_END_TIMEOUT = 0,
    MOTOR_TEST_END_STOP
} MotorTest_EndReason_t;

typedef struct {
    MotorTest_State_t state;
    MotorTest_EndReason_t reason;
    uint16_t elapsed_ms;
    uint16_t duration_ms;
    uint16_t log_interval_ms;
    int16_t pwm_left;
    int16_t pwm_right;
} MotorTest_Snapshot_t;

uint8_t MotorTest_Start(int16_t left, int16_t right,
                        uint16_t duration_ms, uint16_t log_interval_ms);
void MotorTest_Stop(void);
void MotorTest_Tick1ms(void);  /* TIM11 ISR, constant work, no I/O except stop */
uint8_t MotorTest_IsRunning(void);
void MotorTest_GetSnapshot(MotorTest_Snapshot_t *out);
void MotorTest_AcknowledgeEnd(void);

#endif
