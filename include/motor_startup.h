#ifndef MOTOR_STARTUP_H
#define MOTOR_STARTUP_H
#include <stdint.h>

#define MOTOR_STARTUP_MAX_PERMILLE 200U
#define MOTOR_STARTUP_PULSE_MS 250U
#define MOTOR_STARTUP_REST_MS 750U
#define MOTOR_STARTUP_QUIET_MS 300U
#define MOTOR_STARTUP_LINK_MS 400U
#define MOTOR_STARTUP_SESSION_MS 180000U
#define MOTOR_STARTUP_MAX_PULSES 128U

typedef enum {
    MOTOR_STARTUP_DISARMED, MOTOR_STARTUP_READY, MOTOR_STARTUP_DRIVE,
    MOTOR_STARTUP_REST, MOTOR_STARTUP_FAULT
} MotorStartup_State_t;

typedef enum {
    MOTOR_STARTUP_OK, MOTOR_STARTUP_LINK_LOST, MOTOR_STARTUP_SESSION_END,
    MOTOR_STARTUP_WRONG_WHEEL, MOTOR_STARTUP_REVERSE_COUNTS,
    MOTOR_STARTUP_HOST_STOP, MOTOR_STARTUP_TIMING, MOTOR_STARTUP_UART,
    MOTOR_STARTUP_TEST_ENDED
} MotorStartup_Fault_t;

typedef struct {
    MotorStartup_State_t state;
    MotorStartup_Fault_t fault;
    uint32_t id, completed_id, elapsed_ms, quiet_ms, rest_ms;
    int32_t end_left, end_right;
    uint16_t duty;
    uint8_t wheel;
} MotorStartup_Snapshot_t;

void MotorStartup_Init(void);
uint8_t MotorStartup_Arm(void);
uint8_t MotorStartup_Pulse(uint32_t id, uint8_t wheel, uint16_t duty);
void MotorStartup_Heartbeat(void);
void MotorStartup_Abort(MotorStartup_Fault_t reason);
void MotorStartup_Tick1ms(void);
void MotorStartup_GetSnapshot(MotorStartup_Snapshot_t *out);
#endif
