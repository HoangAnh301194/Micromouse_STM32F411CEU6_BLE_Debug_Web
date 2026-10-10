#ifndef MOTOR_TEST_H
#define MOTOR_TEST_H
#include <stdint.h>

/* PWM signed permille: -1000..1000; 1000 = 100% duty before calibration.
 * 0 duration means continuous, but heartbeat still mandatory. */
#define MOTOR_TEST_MAX_CMD               1000
#define MOTOR_TEST_MIN_LOG_MS            100U
#define MOTOR_TEST_MAX_LOG_MS            1000U
#define MOTOR_TEST_HEARTBEAT_TIMEOUT_MS  1500U
#define MOTOR_TEST_BRAKE_PULSE_MS        50U

typedef enum {
    MOTOR_TEST_IDLE = 0,
    MOTOR_TEST_RUNNING,
    MOTOR_TEST_BRAKING,
    MOTOR_TEST_ENDED
} MotorTest_State_t;

typedef enum {
    MOTOR_TEST_END_TIMEOUT = 0,
    MOTOR_TEST_END_STOP,
    MOTOR_TEST_END_HEARTBEAT,
    MOTOR_TEST_END_BRAKE
} MotorTest_EndReason_t;

typedef struct {
    MotorTest_State_t state;
    MotorTest_EndReason_t reason;
    uint32_t elapsed_ms;
    uint32_t duration_ms;
    uint16_t log_interval_ms;
    int16_t pwm_left;            /* signed permille */
    int16_t pwm_right;
} MotorTest_Snapshot_t;

/* Legacy integer percent API retained for existing integrations. */
uint8_t MotorTest_Start(int16_t left_percent, int16_t right_percent,
                        uint32_t duration_ms, uint16_t log_interval_ms);
uint8_t MotorTest_StartPermille(int16_t left_cmd, int16_t right_cmd,
                               uint32_t duration_ms, uint16_t log_interval_ms);
uint8_t MotorTest_SetPermille(int16_t left_cmd, int16_t right_cmd);
void MotorTest_Stop(void);
void MotorTest_BrakePulse(void); /* 50 ms BRAKE, then coast */
void MotorTest_Heartbeat(void);
void MotorTest_Tick1ms(void);
uint8_t MotorTest_IsRunning(void);
void MotorTest_GetSnapshot(MotorTest_Snapshot_t *out);
void MotorTest_AcknowledgeEnd(void);
#endif
