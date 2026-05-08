#ifndef MOTION_CONTROLLER_H
#define MOTION_CONTROLLER_H

#include "ir_simple_calib.h"
#include "pinout.h"
#include <stdint.h>

#define CELL_SIZE_MM 180.0f

/* Physical robot parameters */
#define PIVOT_TURN_SPEED                                                       \
  50.0f /* PWM% for the spinning wheel during pivot turn */
#define DEG_TO_RAD 0.01745329f

typedef enum {
  MOTION_IDLE = 0,
  MOTION_STRAIGHT,
  MOTION_STRAIGHT_CONSTANT, /* Straight motion without terminal deceleration */
  MOTION_TURN_LEFT,
  MOTION_TURN_RIGHT,
  MOTION_TURN_180,
  MOTION_TURN_STABILIZING,
  MOTION_SMOOTH_LEFT,  /* Smooth arc turn left */
  MOTION_SMOOTH_RIGHT, /* Smooth arc turn right */
  MOTION_FRONT_ALIGN,  /* Front wall alignment using IR sensors */
  MOTION_T1WT_APPROACH, /* T1WT approach: detect loss points and run to k+k/3 */
  MOTION_T1WT_PIVOT,    /* T1WT one-wheel pivot turn */
  MOTION_T1WT_EXIT      /* T1WT exit straight 1/3 cell (+ optional chain) */
} Motion_State_t;

typedef struct {
  float kp;
  float ki;
  float kd;
  float integral;
  float prev_error;
  float max_integral;
  float output;
} Motion_PID_t;

typedef struct Motion_Controller_t {
  volatile Motion_State_t
      state; /* Written by ISR, read by main loop — MUST be volatile */
  volatile uint8_t
      complete; /* Written by ISR, read by main loop — MUST be volatile */

  float target_distance;
  float target_angle;
  float start_yaw;
  volatile float traveled_distance; /* Written by ISR, read by main loop for
                                       midpoint check */

  float base_speed;
  float turn_speed;
  float max_speed;
  float current_speed;

  float accel_rate;
  float decel_rate;
  float decel_distance;

  /* Closed-loop speed control (A* mode) */
  Motion_PID_t pid_speed;        /* PID: speed error (mm/s) → PWM correction */
  float actual_speed;            /* Measured speed in mm/s (encoder derivative) */
  float desired_speed;           /* Target speed in mm/s (trapezoidal profile) */
  float prev_traveled_distance;  /* Previous distance for velocity calculation */
  float max_speed_mmps;          /* Max cruise speed in mm/s */
  float accel_mmps2;             /* Acceleration in mm/s² */
  float decel_mmps2;             /* Deceleration in mm/s² */
  uint8_t speed_pid_enable;      /* 0=old PWM mode (explore), 1=PID (A*) */
  float speed_ff_gain;           /* Feedforward: mm/s → approx PWM% */

  Motion_PID_t pid_heading; /* PID for straight motion using MPU */
  Motion_PID_t pid_angle;   /* PID for turn motion */
  Motion_PID_t pid_omega;   /* PID for smooth turn angular velocity */

  /* Wall-following steering correction */
  Motion_PID_t pid_wall;     /* Separate PID for wall centering */
  uint8_t wall_steer_enable; /* 1=use wall correction, 0=gyro only */
  float wall_correction;     /* Last wall PID output (for debug) */

  uint32_t stabilize_start_time;
  uint32_t stabilize_duration;
  uint8_t angle_stable_count;
  uint32_t turn_start_time; /* Absolute turn start time for hard timeout */

  /* Pivot turn parameters (one-wheel pivot) */
  float pivot_turn_speed; /* PWM% for spinning wheel during pivot */

  /* T1WT context: reusable non-blocking turn-chain motion */
  struct {
    uint8_t l45_lost;
    uint8_t l0_lost;
    float l45_loss_dist;
    float target_dist;
    float estimated_k;
    float turn_angle;
    uint16_t l0_th;
    uint16_t r0_th;
    uint16_t l0_off_th;
    uint16_t r0_off_th;
    uint8_t repeat_chain;   /* 1=T1WT chain enabled, 0=single-shot (smooth turn) */
    float saved_pivot_speed;
    uint32_t state_start_ms;
  } t1wt;

  /* Front wall alignment parameters (Fsum/Fdiff method) */
  struct {
    float kp_dist;     /* Fsum error -> velocity gain */
    float kp_yaw;      /* Fdiff error -> rotation gain */
    float fsum_target; /* Target Fsum (L90+R90 at desired distance) */
    float
        fdiff_ref;  /* Fdiff when perfectly aligned (sensor asymmetry offset) */
    float th_front; /* Gating: minimum Fsum to activate alignment */
    float tolerance_d;  /* Fsum error tolerance for stable (ADC units) */
    float tolerance_h;  /* Fdiff error tolerance for stable (ADC units) */
    int max_pwm;        /* Max PWM during alignment */
    int min_pwm;        /* Minimum PWM to overcome motor friction */
    int motor_offset_R; /* Extra PWM boost for right motor (compensate friction
                           asymmetry) */
    uint16_t
        grace_ms; /* Startup grace period - no gating check (let EMA settle) */
    uint8_t stable_needed; /* Consecutive stable readings for completion */
    uint32_t timeout_ms;   /* Max alignment time */
    uint32_t start_time;   /* Alignment start timestamp */
    uint8_t stable_count;  /* Current consecutive stable count */
    uint8_t result;        /* 0=in progress, 1=OK, 2=timeout, 3=no wall */
  } align;
} Motion_Controller_t;

void Motion_Init(Motion_Controller_t *mc);
void Motion_Straight(Motion_Controller_t *mc, float distance_mm);
void Motion_StraightConstant(Motion_Controller_t *mc, float distance_mm);
void Motion_Turn(Motion_Controller_t *mc, float angle_deg);
void Motion_SmoothTurn(Motion_Controller_t *mc, float angle_deg);
void Motion_FrontAlign(Motion_Controller_t *mc, IR_Simple_Calib_t *calib);
void Motion_SnapHeading(void);
void Motion_Update(Motion_Controller_t *mc, float dt);
void Motion_Stop(Motion_Controller_t *mc);
uint8_t Motion_IsComplete(Motion_Controller_t *mc);

void Motion_SetSpeedProfile_Explore(Motion_Controller_t *mc);
void Motion_SetSpeedProfile_ExploreFast(Motion_Controller_t *mc);
void Motion_SetSpeedProfile_Optimal(Motion_Controller_t *mc);

/* Start T1WT non-blocking chain motion */
void Motion_Start_T1WT(Motion_Controller_t *mc, uint16_t l0_th, uint16_t r0_th);

void Motion_StartPID(void);
void Motion_StopPID(void);

#endif
