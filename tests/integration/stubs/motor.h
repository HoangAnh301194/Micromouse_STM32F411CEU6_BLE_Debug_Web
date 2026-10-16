#ifndef MOCK_MOTOR_H
#define MOCK_MOTOR_H
#include <stdint.h>
typedef struct { uint8_t outputs_locked,command_timeout_latched; } Motor_Diagnostics_t;
void Motor_GetDiagnostics(Motor_Diagnostics_t *);
uint8_t Motor_OutputsLocked(void);
void Motor_SetPairPermille(int16_t,int16_t);
void Motor_Stop(void);
void Motor_LockOutputs(void);
#endif
