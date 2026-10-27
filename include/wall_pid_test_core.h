#ifndef WALL_PID_TEST_CORE_H
#define WALL_PID_TEST_CORE_H
#include "ir_wall_signal.h"
#include <stdint.h>

/* Isolated, bounded PA0 Wall PID BENCH profile. Not production navigation.
 * Geometry measured by operator: both L0 and R0 are 60 mm from side walls
 * at start, robot facing a clear >=300mm straight corridor.
 * NO calibrated ADC->mm curve exists; 50..60mm slope is a local estimate.
 * All motor commands forward and capped. Flash never written. */
#define WP_PWM_BASE_PM       180
#define WP_PWM_CAP_PM        220
#define WP_STEER_CAP_PM       40
#define WP_TARGET_MM        100.0f
#define WP_MAX_TICKS        1800U
#define WP_CPR              1430.0f
#define WP_WHEEL_RADIUS_MM    34.0f
#define WP_MM_PER_TICK       (6.28318530718f * WP_WHEEL_RADIUS_MM / WP_CPR)
#define WP_LEFT_ADC_PER_MM    9.7811f
#define WP_RIGHT_ADC_PER_MM  31.6575f
#define WP_PID_TICKS           5U

typedef enum { WP_IDLE=0, WP_RUNNING=1, WP_DONE=2, WP_FAULT=3 } WP_State;
typedef enum {
    WP_NONE=0, WP_FINISHED=1, WP_PA0_STOP=2, WP_IR_STALE=3,
    WP_IMU_STALE=4, WP_WALL_LOST=5, WP_EXCURSION=6,
    WP_HEADING=7, WP_ENCODER=8, WP_TIMEOUT=9, WP_MOTOR_LOCK=10
} WP_Reason;
typedef struct {
    IR_WallSnapshot ir;
    int32_t left_count, right_count;
    float yaw_deg, gyro_dps;
} WP_Input;
typedef struct {
    WP_State state;
    WP_Reason reason;
    uint32_t ticks;
    uint16_t ref_l_adc, ref_r_adc;
    float initial_yaw_deg, integral_mm_s, derivative_mm_s;
    float last_error_mm, error_mm, distance_mm, yaw_error_deg;
    int16_t pwm_l, pwm_r;
    uint8_t has_previous;
} WP_Controller;

/* Start only when BOTH 60mm side-wall ADCs and gyro are healthy.
 * Caller MUST separately verify calibrated MPU, safe power and PA0,
 * reset encoders, and unlock motors ONLY after a successful Start. */
void WP_Init(WP_Controller *p);
uint8_t WP_Start(WP_Controller *p, const WP_Input *in);
/* Run one 1ms tick with coherent gyro/encoder/IR values.
 * Return 1 only if fresh 200Hz PWM outputs are available. */
uint8_t WP_Step(WP_Controller *p, const WP_Input *in);
void WP_Stop(WP_Controller *p, WP_Reason reason);

#endif
