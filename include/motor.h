#ifndef MOTOR_H
#define MOTOR_H

#include <stdint.h>

#define MOTOR_DRIVER_REVISION 3U
/* Watchdog for normal actuator commands (300 ticks at 1kHz). */
#define MOTOR_COMMAND_TIMEOUT_MS 300U

typedef struct {
    uint32_t timer_clock_hz;
    uint32_t period_counts;
    uint32_t control_register;
    uint32_t master_register;
    uint32_t slave_register;
    uint32_t interrupt_register;
    uint32_t compare_mode_register;
    uint32_t channel_register;
    uint32_t compare_left;
    uint32_t compare_right;
    uint32_t direction_register;
    int16_t target_left;
    int16_t target_right;
    int16_t ramped_left;
    int16_t ramped_right;
    uint16_t deadband_left;
    uint16_t deadband_right;
    uint16_t rise_rate;
    uint16_t fall_rate;
    uint16_t reverse_wait_left;
    uint16_t reverse_wait_right;
    uint8_t outputs_locked;
    uint8_t brake_active;
    uint16_t command_age_ms;
    uint8_t command_timeout_latched;
} Motor_Diagnostics_t;

/* API command scale: -1000..+1000 (0.1% resolution).
 * The low-level TIM2 PWM is configured for 30 kHz (MOTOR_PWM_FREQUENCY).
 * All output changes, except immediate stop/brake, are serviced at 1 kHz. */
typedef enum {
    MOTOR_STOP_COAST = 0,
    MOTOR_STOP_BRAKE = 1
} MotorStopMode_t;

void Motor_Init(void);
void Motor_LockOutputs(void);
#if defined(SYSTEM_TEST_MODE)
uint8_t Motor_SystemTestUnlockOutputs(void);
#endif
uint8_t Motor_OutputsLocked(void);
void Motor_GetDiagnostics(Motor_Diagnostics_t *snapshot);
/* MotorTest can renew the lease while enforcing its own duration/heartbeat.
 * Normal Motion renews via repeated SetPair. TIM11 stall is NOT protected. */
void Motor_KeepAlive(void);
void Motor_SetPair(int16_t left_percent, int16_t right_percent); /* legacy */
void Motor_SetPairPermille(int16_t left, int16_t right);
void Motor_SetDeadband(uint16_t left_permille, uint16_t right_permille);
void Motor_SetSlewRates(uint16_t rise_permille_per_ms, uint16_t fall_permille_per_ms);
void Motor_Update1ms(void);
void Motor_GetAppliedPairPermille(int16_t *left, int16_t *right);

void Motor_Stop(void);   /* settled TB6612 Stop: IN1=IN2=0, CCR=ARR+1; physical Hi-Z NOT measured */
void Motor_Brake(void);  /* short-brake command per IN truth table; live electrical effects NOT verified */
void Motor_StopWithMode(MotorStopMode_t mode);

#endif
