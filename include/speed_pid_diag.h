#ifndef SPEED_PID_DIAG_H
#define SPEED_PID_DIAG_H
#include <stdint.h>
#if defined(ENCODER_SPEED_PID_DIAG)
typedef struct {
    uint16_t target_pps;
    uint16_t max_pwm_permille;
    uint16_t feedforward_permille;
    uint16_t last_pwm_left, last_pwm_right;
    uint32_t last_update_ms;
    uint8_t group, step;
} SpeedPidDiagSnapshot;
void SpeedPidDiag_Init(void);
void SpeedPidDiag_Run(uint32_t now_ms, uint8_t group, uint8_t step);
void SpeedPidDiag_Stop(void);
void SpeedPidDiag_Get(SpeedPidDiagSnapshot *out);
#endif
#endif
