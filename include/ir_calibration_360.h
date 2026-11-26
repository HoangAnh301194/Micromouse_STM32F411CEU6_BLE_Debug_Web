#ifndef IR_CALIBRATION_360_H
#define IR_CALIBRATION_360_H
#include <stdint.h>
#include "mpu6050_i2c_module.h"
#include "encoder_onebox.h"

/* Mode 1 only. 4 explicit 90-degree pivots, never a 360-degree target in
 * the existing single-turn driver. Motor safety guards stay in OneBox. */
typedef enum {
    IRC_IDLE=0, IRC_ARMED, IRC_PIVOT, IRC_DWELL, IRC_REVIEW, IRC_SAVED,
    IRC_FAULT
} IRC_State;
typedef struct {
    IRC_State state;
    uint8_t segment, start_seen, valid, sample_count[6];
    uint32_t started_ms, last_sample_ms, dwell_started_ms, last_ready_ms;
    uint16_t low[6], high[6], wall_on[6], wall_off[6];
    uint16_t histogram[6][32]; /* 128-count ADC bins, for robust quantiles */
    uint16_t center_l0, center_r0;
    float session_origin_yaw, segment_origin_yaw;
    uint8_t have_origin;
    uint32_t samples;
    char fault[24];
} IRC_Context;
void IRC_Init(IRC_Context *c);
void IRC_Arm(IRC_Context *c, uint32_t now);
void IRC_Update(IRC_Context *c, MPU6050_Handle_t *mpu, uint32_t now);
void IRC_OnPivotResult(IRC_Context *c, const EncoderOneBoxResult *result, uint32_t now);
void IRC_Abort(IRC_Context *c, const char *reason);
uint8_t IRC_IsMotionActive(const IRC_Context *c);
uint8_t IRC_Save(IRC_Context *c);
#endif
