#ifndef WALL_PID_TEST_CORE_H
#define WALL_PID_TEST_CORE_H
#include "ir_wall_signal.h"
#include <stdint.h>

/* Dedicated 4-cell dual-wall prototype. Start/reference geometry:
 * user places the robot centred, both L0/R0 facing side walls.
 * Only L0/R0 contribute lateral PID error. IMU is FAULT GUARD ONLY.
 * 50..60mm slopes are provisional local scale factors, NOT a distance LUT. */
#define WP_CELLS                 4U
#define WP_CELL_MM             180.0f
#define WP_TARGET_MM          (WP_CELLS * WP_CELL_MM)
#define WP_WHEEL_DIAMETER_MM    34.0f
#define WP_ENCODER_CPR       1430.0f
#define WP_MM_PER_COUNT      (3.14159265359f * WP_WHEEL_DIAMETER_MM / WP_ENCODER_CPR)
#define WP_MAX_TICKS         5000U
#define WP_PID_TICKS            5U
#define WP_SPEED_MM_S        900.0f   /* provisional, measured/tuned with encoder */
#define WP_ACCEL_MM_S2      1700.0f
#define WP_DECEL_MM_S2      2000.0f
#define WP_PWM_CAP_PM         800     /* PWM LIMIT, NOT continuously requested duty */
#define WP_STEER_CAP_PM       200
#define WP_LEFT_ADC_PER_MM     9.7811f
#define WP_RIGHT_ADC_PER_MM   31.6575f

typedef enum { WP_IDLE=0, WP_RUNNING=1, WP_DONE=2, WP_FAULT=3 } WP_State;
typedef enum {
    WP_NONE=0, WP_FINISHED=1, WP_PA0_STOP=2, WP_IR_STALE=3,
    WP_IMU_STALE=4, WP_WALL_LOST=5, WP_EXCURSION=6,
    WP_HEADING=7, WP_ENCODER=8, WP_TIMEOUT=9, WP_MOTOR_LOCK=10,
    WP_START_REJECT=11, WP_OVERSPEED=12
} WP_Reason;
typedef struct {
    IR_WallSnapshot ir;
    int32_t left_count, right_count;
    float yaw_deg, gyro_dps;
} WP_Input;
typedef struct {
    WP_State state;
    WP_Reason reason;
    uint32_t ticks, pid_updates;
    uint16_t ref_l_adc, ref_r_adc;
    float initial_yaw_deg;
    float integral_error, last_error_mm, derivative_mm_s;
    float error_mm, abs_error_sum_mm, peak_abs_error_mm;
    float distance_mm, speed_mm_s, target_speed_mm_s, ramp_speed_mm_s;
    float speed_integral_pm;
    float peak_speed_mm_s;
    int16_t peak_pwm_pm;
    int32_t previous_left, previous_right;
    float yaw_error_deg;
    int16_t pwm_l, pwm_r;
    uint8_t has_previous;
} WP_Controller;

/* Pure C, no IRQ/actuator access. Caller must gate physical PA0,
 * calibration and valid fresh measurements before unlock. */
void WP_Init(WP_Controller *p);
uint8_t WP_Start(WP_Controller *p,const WP_Input *in);
uint8_t WP_Step(WP_Controller *p,const WP_Input *in);
void WP_Stop(WP_Controller *p,WP_Reason reason);
#endif
