#ifndef IR_WALL_SIGNAL_H
#define IR_WALL_SIGNAL_H

#include <stdint.h>

/* Portable, read-only wall-PID input adapter.
 * Channel order matches ir_sensor.h/pinout.h:
 * L90, L45, L0, R0, R45, R90.
 * ADC is ambient-subtracted LED_ON - LED_OFF (0..4095), NOT millimetres.
 * No wall-presence threshold, PWM, motor actuation, LUT or gyro fusion.
 */
#define IR_WALL_CHANNELS         6U
#define IR_WALL_EMA_ALPHA        0.10f
#define IR_WALL_WARMUP_SAMPLES   32U
#define IR_WALL_MAX_AGE_US       3000UL
#define IR_WALL_RESET_GAP_US     3000UL

typedef enum {
    IR_WALL_L90 = 0,
    IR_WALL_L45 = 1,
    IR_WALL_L0  = 2,
    IR_WALL_R0  = 3,
    IR_WALL_R45 = 4,
    IR_WALL_R90 = 5
} IR_WallChannel;

typedef enum {
    IR_WALL_NO_SAMPLE = 0,
    IR_WALL_WARMUP = 1,
    IR_WALL_VALID = 2,
    IR_WALL_STALE = 3
} IR_WallStatus;

typedef struct {
    uint16_t raw_adc[IR_WALL_CHANNELS];
    uint16_t ema10_adc[IR_WALL_CHANNELS];
    uint32_t scan_count;
    uint32_t completed_us;
    uint32_t age_us;
    uint32_t skipped_scans;
    uint32_t reset_count;
    uint32_t rejected_frames;
    uint8_t warmup_count;
    IR_WallStatus status;
} IR_WallSnapshot;

typedef struct {
    float ema[IR_WALL_CHANNELS];
    uint16_t raw[IR_WALL_CHANNELS];
    uint16_t rounded[IR_WALL_CHANNELS];
    uint32_t scan_count;
    uint32_t completed_us;
    uint32_t skipped_scans;
    uint32_t reset_count;
    uint32_t rejected_frames;
    uint8_t initialized;
    uint8_t warmup_count;
} IR_WallSignal;

/* One writer per completed IR scan; call from the same context consuming
 * IR_GetLatest(). No IO or heap allocations. Do not call from ADC DMA ISR. */
void IR_WallSignal_Init(IR_WallSignal *state);
uint8_t IR_WallSignal_Update(IR_WallSignal *state,
                             const uint16_t raw_adc[IR_WALL_CHANNELS],
                             uint32_t scan_count,
                             uint32_t completed_us);
/* Snapshot is valid ONLY when status==IR_WALL_VALID.
 * Caller supplies the monotonic TIM5 micros() timestamp. */
IR_WallStatus IR_WallSignal_Read(const IR_WallSignal *state,
                                 uint32_t now_us,
                                 IR_WallSnapshot *out);

#endif
