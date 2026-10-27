#ifndef TURN_PID_DIAG_H
#define TURN_PID_DIAG_H
#include <stdint.h>
#if defined(TURN_PID_DIAG)
int TurnPidDiag_Main(void);
void TurnPidDiag_ControlTick(int32_t left, int32_t right, float yaw_deg,
                             float gyro_dps, uint32_t sample_age_us);
void TurnPidDiag_Stop(uint8_t reason);
#endif
#endif
