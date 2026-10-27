#ifndef MOTION_PRIMITIVES_H
#define MOTION_PRIMITIVES_H

#include <stdint.h>

/*
 * Platform-independent motion primitive controller.
 * The sign convention is +yaw=LEFT(CCW), -yaw=RIGHT(CW).
 * This file has no STM32/motor/ROS/BLE dependency.
 *
 * 90deg and 180deg are currently implemented as IN-PLACE pivots.
 * Slalom and straight are reserved identifiers, NOT implemented motions.
 */
typedef enum {
    MOTION_PRIMITIVE_PIVOT_90 = 1,
    MOTION_PRIMITIVE_PIVOT_180 = 2,
    MOTION_PRIMITIVE_SLALOM_90 = 3,
    MOTION_PRIMITIVE_STRAIGHT = 4
} MotionPrimitiveKind;

typedef enum {
    MOTION_DIR_RIGHT = -1,
    MOTION_DIR_LEFT = 1
} MotionPrimitiveDirection;

typedef enum {
    MOTION_PRIMITIVE_IDLE = 0,
    MOTION_PRIMITIVE_RUNNING = 1,
    MOTION_PRIMITIVE_DONE = 2,
    MOTION_PRIMITIVE_FAULT = 3
} MotionPrimitiveStatus;

typedef enum {
    MOTION_PRIMITIVE_OK = 0,
    MOTION_PRIMITIVE_BAD_ARGUMENT,
    MOTION_PRIMITIVE_UNSUPPORTED,
    MOTION_PRIMITIVE_BUSY,
    MOTION_PRIMITIVE_STOP_REQUEST,
    MOTION_PRIMITIVE_SENSOR_FAULT,
    MOTION_PRIMITIVE_TIMEOUT,
    MOTION_PRIMITIVE_OVERSHOOT,
    MOTION_PRIMITIVE_WRONG_DIRECTION,
    MOTION_PRIMITIVE_RATE_LIMIT,
    MOTION_PRIMITIVE_STALLED,
    MOTION_PRIMITIVE_SETTLE_DRIFT
} MotionPrimitiveResult;

typedef enum {
    MOTION_PIVOT_ACCEL = 0,
    MOTION_PIVOT_BRAKE = 1,
    MOTION_PIVOT_COAST = 2,
    MOTION_PIVOT_APPROACH = 3,
    MOTION_PIVOT_SETTLE = 4
} MotionPivotPhase;

typedef struct {
    MotionPrimitiveKind kind;
    MotionPrimitiveDirection direction;
    MotionPrimitiveStatus status;
    MotionPrimitiveResult result;
    MotionPivotPhase phase;
    float target_deg;
    float traveled_deg;    /* Signed toward requested direction: 0 -> target */
    float error_deg;
    float gyro_dps;        /* Signed toward requested direction */
    float ref_rate_dps;
    float last_yaw_deg;
    float elapsed_s;
    float phase_elapsed_s;
    float settle_s;
    float stall_s;
    float control_accum_s;
    float pwm_requested;
} MotionPrimitiveController;

typedef struct {
    int16_t left_pm;       /* [-1000,1000] motor API permille */
    int16_t right_pm;
    uint8_t immediate_neutral; /* Apply Motor_Stop now, bypass PWM down-slew. */
    MotionPrimitiveStatus status;
    MotionPrimitiveResult result;
    MotionPivotPhase phase;
    float angle_error_deg;
    float ref_rate_dps;
} MotionPrimitiveOutput;

/* One active primitive per controller. No queue or motor side effects. */
void MotionPrimitive_Init(MotionPrimitiveController *ctx);
MotionPrimitiveResult MotionPrimitive_Start(MotionPrimitiveController *ctx,
                                            MotionPrimitiveKind kind,
                                            MotionPrimitiveDirection direction,
                                            float current_yaw_deg);
void MotionPrimitive_Update(MotionPrimitiveController *ctx,
                            float dt_s, float yaw_deg, float gyro_z_dps,
                            MotionPrimitiveOutput *out);
void MotionPrimitive_Abort(MotionPrimitiveController *ctx,
                           MotionPrimitiveResult reason);

#endif
