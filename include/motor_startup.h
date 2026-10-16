#ifndef MOTOR_STARTUP_H
#define MOTOR_STARTUP_H
#include <stdint.h>

#if defined(MOTOR_STARTUP_HIGH_SPEED)
#define MOTOR_STARTUP_MAX_PERMILLE 1000U
#define MOTOR_STARTUP_PULSE_MS 600U
#define MOTOR_STARTUP_REST_MS 900U
#define MOTOR_STARTUP_SESSION_MS 90000U
#define MOTOR_STARTUP_MAX_PULSES 32U
#define MOTOR_STARTUP_MAX_WHEEL 2U
#else
#define MOTOR_STARTUP_MAX_PERMILLE 200U
#define MOTOR_STARTUP_PULSE_MS 250U
#define MOTOR_STARTUP_REST_MS 750U
#define MOTOR_STARTUP_SESSION_MS 180000U
#define MOTOR_STARTUP_MAX_PULSES 128U
#define MOTOR_STARTUP_MAX_WHEEL 1U
#endif
#define MOTOR_STARTUP_QUIET_MS 300U
#define MOTOR_STARTUP_LINK_MS 400U

typedef enum {
    MOTOR_STARTUP_DISARMED, MOTOR_STARTUP_READY, MOTOR_STARTUP_DRIVE,
    MOTOR_STARTUP_REST, MOTOR_STARTUP_FAULT
} MotorStartup_State_t;

typedef enum {
    MOTOR_STARTUP_OK, MOTOR_STARTUP_LINK_LOST, MOTOR_STARTUP_SESSION_END,
    MOTOR_STARTUP_WRONG_WHEEL, MOTOR_STARTUP_REVERSE_COUNTS,
    MOTOR_STARTUP_HOST_STOP, MOTOR_STARTUP_TIMING, MOTOR_STARTUP_UART,
    MOTOR_STARTUP_TEST_ENDED, MOTOR_STARTUP_NO_PROGRESS, MOTOR_STARTUP_SENSOR
} MotorStartup_Fault_t;

typedef struct {
    MotorStartup_State_t state;
    MotorStartup_Fault_t fault;
    uint32_t id, completed_id, elapsed_ms, quiet_ms, rest_ms;
    int32_t end_left, end_right;
    uint16_t duty;
    uint8_t wheel;
#if defined(MOTOR_STARTUP_HIGH_SPEED)
    uint32_t cycle_calls, cycle_total, cycle_max;
#endif
} MotorStartup_Snapshot_t;

void MotorStartup_Init(void);
uint8_t MotorStartup_Arm(void);
uint8_t MotorStartup_Pulse(uint32_t id, uint8_t wheel, uint16_t duty);
void MotorStartup_Heartbeat(void);
void MotorStartup_Abort(MotorStartup_Fault_t reason);
void MotorStartup_Tick1ms(void);
void MotorStartup_GetSnapshot(MotorStartup_Snapshot_t *out);
#if defined(MOTOR_STARTUP_HIGH_SPEED)
void MotorStartup_RecordCycles(uint32_t cycles);
#endif
#endif
