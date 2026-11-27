#ifndef IR_CALIBRATION_360_H
#define IR_CALIBRATION_360_H

#include <stdint.h>

/* Mode 1: stationary, two-condition IR calibration.
 * Stage WALL: robot remains still with front/left/right walls, no rear wall.
 * Stage OPEN: robot remains still in open space with no surrounding walls.
 * No motor, encoder, or MPU resource is required. */
typedef enum {
    IRC_IDLE = 0,
    IRC_CAPTURE_WALL,
    IRC_WAIT_OPEN,
    IRC_CAPTURE_OPEN,
    IRC_REVIEW,
    IRC_SAVED,
    IRC_FAULT
} IRC_State;

typedef struct {
    IRC_State state;
    uint8_t valid;
    uint8_t physical_start;
    uint8_t failed_channels;
    uint32_t started_ms;
    uint32_t stage_started_ms;
    uint32_t last_sample_ms;
    uint32_t last_scan_count;
    uint16_t wall_min[6], wall_max[6];
    uint16_t open_min[6], open_max[6];
    uint16_t wall_on[6], wall_off[6];
    uint16_t wall_p10[6], wall_p90[6];
    uint16_t open_p90[6], open_p95[6];
    uint16_t wall_histogram[6][64]; /* 64-count ADC bins */
    uint16_t open_histogram[6][64];
    uint32_t wall_samples;
    uint32_t open_samples;
    char fault[24];
} IRC_Context;

void IRC_Init(IRC_Context *c);
void IRC_Arm(IRC_Context *c, uint32_t now, uint8_t physical_start);
void IRC_Update(IRC_Context *c, uint32_t now);
uint8_t IRC_ConfirmOpen(IRC_Context *c, uint32_t now);
void IRC_Abort(IRC_Context *c, const char *reason);
/* Historical name retained for main_runtime compatibility: true while the
 * calibration session is capturing or waiting for the operator. */
uint8_t IRC_IsMotionActive(const IRC_Context *c);
uint8_t IRC_Save(IRC_Context *c);

#endif
