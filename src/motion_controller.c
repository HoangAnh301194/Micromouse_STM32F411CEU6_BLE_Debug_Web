/* ========================================================================== */
/* motion_controller.c - FINAL FIX: Passive Braking in Stabilize            */
/* ROOT CAUSE: Stabilize phase was still applying PWM correction             */
/* SOLUTION: Stop motors immediately, let friction do the work               */
/* ========================================================================== */

#include "motion_controller.h"
#include "hardware.h"
#include "ir_sensor.h"
#include "ir_simple_calib.h"
#include "mpu6050.h"
#include "sensor_fusion.h"
#include "system_timer.h"
#include "uart.h"

#include <math.h>
#include <stdio.h>

#define PI 3.14159265f

#define MM_PER_PULSE (PI * WHEEL_DIAMETER_MM / (float)ENCODER_PPR)

/* Encoder direction signs: set to +1 or -1 so both read POSITIVE when
 * robot drives forward.  Flip sign if encoder counts backward on that side.
 * >>> TUNE ON REAL HARDWARE: push robot forward by hand and check UART log <<<
 */
#define ENCODER_DIR_LEFT 1
#define ENCODER_DIR_RIGHT 1

/* [FIX] Relaxed tolerance - accept 2 error */
#define ANGLE_TOLERANCE 5.0f
#define ANGLE_STABLE_TOLERANCE 2.5f
#define DISTANCE_TOLERANCE 0.0f

/* [NEW] Velocity-based early stabilize threshold */
#define VELOCITY_STABILIZE_THRESHOLD                                           \
  20.0f /* deg/s - must be slow to stabilize */
#define ANGLE_WITH_VELOCITY_THRESHOLD 4.0f /* degrees */

#define PWM_MIN 0
#define PWM_MAX 100

#define T1WT_DEFAULT_K_MM 60.0f
#define T1WT_L45_PRESENT_TH 80
#define T1WT_L45_LOST_TH 40
#define T1WT_FRONT_OPEN_TH 80
#define T1WT_SIDE_OPEN_TH 150
#define T1WT_APPROACH_TIMEOUT_MS 2000
#define T1WT_PIVOT_TIMEOUT_MS 700
#define T1WT_EXIT_TIMEOUT_MS 1200

extern Sensor_Fusion_t sensor_fusion;
extern Motion_Controller_t motion_ctrl;

static float PID_Compute(Motion_PID_t *pid, float error, float dt) {
  float p_term, i_term, d_term;

  p_term = pid->kp * error;

  pid->integral += error * dt;
  if (pid->integral > pid->max_integral)
    pid->integral = pid->max_integral;
  if (pid->integral < -pid->max_integral)
    pid->integral = -pid->max_integral;
  i_term = pid->ki * pid->integral;

  d_term = 0.0f;
  if (dt > 0.0001f) {
    d_term = pid->kd * (error - pid->prev_error) / dt;
  }
  pid->prev_error = error;

  pid->output = p_term + i_term + d_term;
  return pid->output;
}

static void PID_Reset(Motion_PID_t *pid) {
  pid->integral = 0.0f;
  pid->prev_error = 0.0f;
  pid->output = 0.0f;
}

static float NormalizeAngle(float angle) {
  while (angle > 180.0f)
    angle -= 360.0f;
  while (angle < -180.0f)
    angle += 360.0f;
  return angle;
}

static float Clamp(float value, float min_val, float max_val) {
  if (value < min_val)
    return min_val;
  if (value > max_val)
    return max_val;
  return value;
}

static void DriveStraightOpenLoopHeading(Motion_Controller_t *mc, float dt,
                                         float speed) {
  float current_yaw = MPU6050_GetYaw(&sensor_fusion.mpu_handle);
  float heading_error = NormalizeAngle(current_yaw - mc->start_yaw);
  float correction = PID_Compute(&mc->pid_heading, heading_error, dt);
  int pwm_left = (int)Clamp(speed + correction, (float)PWM_MIN, (float)PWM_MAX);
  int pwm_right =
      (int)Clamp(speed - correction, (float)PWM_MIN, (float)PWM_MAX);

  Hardware_SetMotor(1, pwm_left);
  Hardware_SetMotor(0, pwm_right);
}

static float ComputeRampedSpeed(Motion_Controller_t *mc,
                                float distance_traveled, float dt) {
  float target_speed;
  float remaining_distance;
  float speed_change;

  remaining_distance = mc->target_distance - distance_traveled;

  if (remaining_distance < mc->decel_distance) {
    target_speed = mc->base_speed * (remaining_distance / mc->decel_distance);
    if (target_speed < 18.0f)
      target_speed = 18.0f;
  } else {
    target_speed = mc->max_speed;
  }

  if (mc->current_speed < target_speed) {
    speed_change = mc->accel_rate * dt;
    mc->current_speed += speed_change;
    if (mc->current_speed > target_speed) {
      mc->current_speed = target_speed;
    }
  } else if (mc->current_speed > target_speed) {
    speed_change = mc->decel_rate * dt;
    mc->current_speed -= speed_change;
    if (mc->current_speed < target_speed) {
      mc->current_speed = target_speed;
    }
  }

  return mc->current_speed;
}

/* Closed-loop trapezoidal speed profile in mm/s.
 * Returns desired speed based on remaining distance and acceleration limits.
 * Uses kinematic equation: v² = 2 * a * d  →  v = sqrt(2 * a * d) for decel. */
static float ComputeDesiredSpeed_PID(Motion_Controller_t *mc, float dt) {
  float remaining = mc->target_distance - mc->traveled_distance;
  float desired;
  float base_mmps;

  /* Deceleration zone: v = sqrt(2 * a_decel * remaining) */
  float v_decel = sqrtf(2.0f * mc->decel_mmps2 * fabsf(remaining));
  base_mmps = (mc->speed_ff_gain > 0.001f) ? (mc->base_speed / mc->speed_ff_gain)
                                           : 30.0f;
  if (base_mmps < 30.0f)
    base_mmps = 30.0f;
  if (base_mmps > mc->max_speed_mmps)
    base_mmps = mc->max_speed_mmps;

  if (v_decel < mc->max_speed_mmps) {
    /* In deceleration zone */
    desired = v_decel;
    /* Keep cruising at base speed until almost segment end. */
    if (remaining > 2.0f && desired < base_mmps)
      desired = base_mmps;
  } else if (mc->desired_speed < mc->max_speed_mmps) {
    /* Accelerating: ramp up */
    desired = mc->desired_speed + mc->accel_mmps2 * dt;
    if (desired > mc->max_speed_mmps)
      desired = mc->max_speed_mmps;
  } else {
    /* Cruising at max speed */
    desired = mc->max_speed_mmps;
  }

  /* Minimum speed to avoid stalling */
  if (desired < 30.0f && remaining > 5.0f)
    desired = 30.0f;

  return desired;
}

void Motion_Init(Motion_Controller_t *mc) {
  char buf[64];

  if (!mc)
    return;

  /* TIM11 Configuration for 1ms PID Loop (1kHz) */
  /* 1. Enable TIM11 Clock on APB2 (96MHz) */
  RCC->APB2ENR |= (1 << 18);

  /* 2. Configure TIM11 Prescaler and ARR for 1ms */
  /* 96MHz / 96 = 1 MHz -> 1 tick = 1us */
  TIM11->PSC = 96 - 1;
  /* 1000 ticks * 1us = 1000us = 1ms */
  TIM11->ARR = 1000 - 1;

  /* 3. Enable Update Interrupt in TIM11 */
  TIM11->DIER |= (1 << 0);

  /* 4. Configure NVIC for TIM11 */
  /* TIM11 shares vector with TIM1_TRG_COM. Priority 0 (Highest) to preempt IR
   * Sensor TIM10 */
  NVIC_SetPriority(TIM1_TRG_COM_TIM11_IRQn, 0);
  NVIC_EnableIRQ(TIM1_TRG_COM_TIM11_IRQn);

  /* 5. Do NOT Start TIM11 yet. It will be started by Motion_StartPID() */

  mc->state = MOTION_IDLE;
  mc->complete = 1;

  mc->target_distance = 0.0f;
  mc->target_angle = 0.0f;
  mc->start_yaw = 0.0f;
  mc->traveled_distance = 0.0f;

  mc->base_speed = 45.0f;
  mc->turn_speed = 45.0f;
  mc->max_speed = 80.0f;
  mc->current_speed = 0.0f;

  mc->accel_rate = 250.0f;
  mc->decel_rate = 230.0f;
  mc->decel_distance = 30.0f;

  /* Closed-loop speed PID (for A* mode) */
  mc->pid_speed.kp =
      0.05f; /* TUNE: small correction only (feedforward does main work) */
  mc->pid_speed.ki = 0.1f;   /* TUNE: eliminates steady-state speed error */
  mc->pid_speed.kd = 0.001f; /* TUNE: dampens speed oscillation */
  mc->pid_speed.max_integral = 30.0f;
  PID_Reset(&mc->pid_speed);
  mc->actual_speed = 0.0f;
  mc->desired_speed = 0.0f;
  mc->prev_traveled_distance = 0.0f;
  mc->max_speed_mmps = 400.0f; /* Default cruise speed mm/s */
  mc->accel_mmps2 = 800.0f;    /* Default accel mm/s² (gentle ramp) */
  mc->decel_mmps2 = 1200.0f;   /* Default decel mm/s² */
  mc->speed_pid_enable = 0;    /* Off by default (explore uses old PWM mode) */
  mc->speed_ff_gain = 0.18f;   /* Feedforward: mm/s → PWM% (TUNE on hardware) */

  mc->stabilize_start_time = 0;
  mc->stabilize_duration = 250; /* Reduced from 350ms */
  mc->angle_stable_count = 0;
  mc->turn_start_time = 0;

  /* [TUNED] PD - Dampened for 1ms strict interrupt */
  mc->pid_angle.kp = 1.1f; /* Reduced from 2.0 to prevent aggressive shaking */
  mc->pid_angle.ki = 0.01f;
  mc->pid_angle.kd = 0.001f; /* Proper D damping for 1ms dt */
  mc->pid_angle.max_integral = 50.0f;
  PID_Reset(&mc->pid_angle);
  // PID for heading motion
  mc->pid_heading.kp = 2.0f;
  mc->pid_heading.ki = 0.0f; /* Increased from 0.5f to overcome steady-state
                                error on long runs */
  mc->pid_heading.kd = 0.01f;
  mc->pid_heading.max_integral = 40.0f; /* Increased from 12.0f to give the
                                           integrator more room to correction */
  PID_Reset(&mc->pid_heading);

  /* PID for smooth turn angular velocity — UNUSED for pivot, keep for compat */
  mc->pid_omega.kp = 0.10f;
  mc->pid_omega.ki = 0.01f;
  mc->pid_omega.kd = 0.0f;
  mc->pid_omega.max_integral = 50.0f;
  PID_Reset(&mc->pid_omega);

  /* Wall-following steering correction - separate PID */
  mc->pid_wall.kp = 35.0f; /* TUNE: main wall correction strength */
  mc->pid_wall.ki = 0.5f;  /* TUNE: eliminates steady-state offset */
  mc->pid_wall.kd = 0.0f;  /* TUNE: dampens wall oscillation */
  mc->pid_wall.max_integral = 50.0f;
  PID_Reset(&mc->pid_wall);
  mc->wall_steer_enable = 1; /* Enable wall correction by default */
  mc->wall_correction = 0.0f;

  /* Pivot turn defaults */
  mc->pivot_turn_speed = PIVOT_TURN_SPEED;

  mc->t1wt.l45_lost = 0;
  mc->t1wt.l0_lost = 0;
  mc->t1wt.l45_loss_dist = 0.0f;
  mc->t1wt.target_dist = 0.0f;
  mc->t1wt.estimated_k = T1WT_DEFAULT_K_MM;
  mc->t1wt.turn_angle = 0.0f;
  mc->t1wt.l0_th = 0;
  mc->t1wt.r0_th = 0;
  mc->t1wt.l0_off_th = 0;
  mc->t1wt.r0_off_th = 0;
  mc->t1wt.repeat_chain = 0;
  mc->t1wt.saved_pivot_speed = mc->pivot_turn_speed;
  mc->t1wt.state_start_ms = 0;

  /* Front wall alignment defaults (Fsum/Fdiff method) */
  mc->align.kp_dist = 0.04f;    /* Distance gain: ~400 error -> ~16 PWM */
  mc->align.kp_yaw = 0.06f;     /* Yaw gain: ~200 error -> ~14 PWM */
  mc->align.fsum_target = 0.0f; /* Set by calibration */
  mc->align.fdiff_ref = 0.0f;   /* Set by calibration (sensor asymmetry) */
  mc->align.th_front = 0.0f;    /* Set by calibration */
  mc->align.tolerance_d = 5.0f; /* Fsum error tolerance (relaxed for P-only) */
  mc->align.tolerance_h = 5.0f; /* Fdiff error tolerance (relaxed for P-only) */
  mc->align.max_pwm = 45;
  mc->align.min_pwm = 5;        /* Deadzone compensation: minimum PWM to move */
  mc->align.motor_offset_R = 3; /* Right motor friction compensation (+2 PWM) */
  mc->align.grace_ms = 200;     /* Wait 200ms for EMA filter to stabilize */
  mc->align.stable_needed = 3;  /* Need 5 consecutive stable readings */
  mc->align.timeout_ms = 500;
  mc->align.stable_count = 0;
  mc->align.result = 0;

  sprintf(buf, "Motion Init: PD @ 1kHz (Kp=%.1f Kd=%.1f)\r\n", mc->pid_angle.kp,
          mc->pid_angle.kd);

  sprintf(buf, "  Passive braking in stabilize (150ms)\r\n");
}

void Motion_SetSpeedProfile_Explore(Motion_Controller_t *mc) {
  if (!mc)
    return;
  mc->base_speed = 20.0f;
  mc->max_speed = 45.0f;
  mc->turn_speed = 48.0f;
  mc->accel_rate = 250.0f;
  mc->decel_rate = 230.0f;
  mc->decel_distance = 28.0f;
  mc->speed_pid_enable = 0; /* Explore: old open-loop PWM mode */
  /* Wall centering PID: aggressive at low speed */
  mc->pid_wall.kp = 35.0f;
  mc->pid_wall.ki = 0.7f;
  mc->pid_wall.kd = 0.0f;
}

void Motion_SetSpeedProfile_ExploreFast(Motion_Controller_t *mc) {
  if (!mc)
    return;
  mc->base_speed = 35.0f;
  mc->max_speed = 75.0f; /* Faster than Explore (45) */
  mc->turn_speed = 55.0f;
  mc->accel_rate = 250.0f;
  mc->decel_rate = 230.0f;
  mc->decel_distance = 35.0f;
  mc->speed_pid_enable = 0; /* Explore: old open-loop PWM mode */
  /* Wall centering PID: softer at medium speed to reduce oscillation */
  mc->pid_wall.kp = 45.0f;
  mc->pid_wall.ki = 0.7f;
  mc->pid_wall.kd = 0.0f;
}

void Motion_SetSpeedProfile_Optimal(Motion_Controller_t *mc) {
  if (!mc)
    return;
  /* Old PWM values kept as fallback for turn_speed etc. */
  mc->base_speed = 55.0f;
  mc->max_speed = 75.0f;
  mc->turn_speed = 62.0f;
  mc->accel_rate = 300.0f;
  mc->decel_rate = 280.0f;
  mc->decel_distance = 38.0f;
  /* Closed-loop speed profile in mm/s */
  mc->speed_pid_enable = 1;    /* Enable PID speed control */
  mc->max_speed_mmps = 400.0f; /* TUNE: max cruise speed mm/s */
  mc->accel_mmps2 = 800.0f;    /* TUNE: acceleration mm/s² (gentle) */
  mc->decel_mmps2 = 1200.0f;   /* TUNE: deceleration mm/s² */
  mc->speed_ff_gain = 0.18f;   /* TUNE: feedforward mm/s → PWM% */
  /* Wall centering PID: gentle at high speed to prevent oscillation */
  mc->pid_wall.kp = 40.0f;
  mc->pid_wall.ki = 0.7f;
  mc->pid_wall.kd = 0.0f;
}

void Motion_Straight(Motion_Controller_t *mc, float distance_mm) {
  if (!mc)
    return;

  Hardware_ResetEncoder(0);
  Hardware_ResetEncoder(1);

  mc->state = MOTION_STRAIGHT;
  mc->complete = 0;
  mc->target_distance = distance_mm;
  mc->traveled_distance = 0.0f;
  mc->start_yaw = MPU6050_GetYaw(&sensor_fusion.mpu_handle);

  /* [OPTIMIZATION] Do NOT reset current_speed to 0.0f!
   * Inherit speed from previous motion for seamless transition. */
  if (mc->current_speed < 10.0f)
    mc->current_speed = 10.0f;

  /* Speed PID: reset state but keep desired_speed for seamless transitions */
  mc->prev_traveled_distance = 0.0f;
  mc->actual_speed = 0.0f;
  PID_Reset(&mc->pid_speed);

  PID_Reset(&mc->pid_heading);
  PID_Reset(&mc->pid_wall);
  mc->wall_correction = 0.0f;
}

void Motion_StraightConstant(Motion_Controller_t *mc, float distance_mm) {
  if (!mc)
    return;

  Hardware_ResetEncoder(0);
  Hardware_ResetEncoder(1);

  mc->state = MOTION_STRAIGHT_CONSTANT;
  mc->complete = 0;
  mc->target_distance = distance_mm;
  mc->traveled_distance = 0.0f;
  mc->start_yaw = MPU6050_GetYaw(&sensor_fusion.mpu_handle);

  /* Do NOT reset current_speed to 0.0f! Inherit speed from previous motion for
   * seamless transition */
  if (mc->current_speed < 10.0f)
    mc->current_speed = 10.0f;

  /* Speed PID: reset state but keep desired_speed for seamless transitions */
  mc->prev_traveled_distance = 0.0f;
  mc->actual_speed = 0.0f;
  PID_Reset(&mc->pid_speed);

  PID_Reset(&mc->pid_heading);
  PID_Reset(&mc->pid_wall);
  mc->wall_correction = 0.0f;
}

void Motion_SmoothTurn(Motion_Controller_t *mc, float angle_deg) {
  if (!mc)
    return;

  Hardware_ResetEncoder(0);
  Hardware_ResetEncoder(1);

  mc->complete = 0;
  mc->target_angle = angle_deg;
  mc->start_yaw = MPU6050_GetYaw(&sensor_fusion.mpu_handle);
  mc->angle_stable_count = 0;

  /* Record turn start time for hard timeout */
  mc->turn_start_time = millis();

  if (angle_deg > 0) {
    mc->state = MOTION_SMOOTH_LEFT;
  } else {
    mc->state = MOTION_SMOOTH_RIGHT;
  }

  PID_Reset(&mc->pid_angle);
}

void Motion_Start_T1WT(Motion_Controller_t *mc, uint16_t l0_th,
                       uint16_t r0_th) {
  if (!mc)
    return;

  Hardware_ResetEncoder(0);
  Hardware_ResetEncoder(1);

  mc->state = MOTION_T1WT_APPROACH;
  mc->complete = 0;
  mc->traveled_distance = 0.0f;
  mc->prev_traveled_distance = 0.0f;
  mc->start_yaw = MPU6050_GetYaw(&sensor_fusion.mpu_handle);
  mc->turn_start_time = millis();

  mc->t1wt.l45_lost = 0;
  mc->t1wt.l0_lost = 0;
  mc->t1wt.l45_loss_dist = 0.0f;
  mc->t1wt.target_dist = 99999.0f;
  if (mc->t1wt.estimated_k < 15.0f || mc->t1wt.estimated_k > 150.0f) {
    mc->t1wt.estimated_k = T1WT_DEFAULT_K_MM;
  }
  mc->t1wt.turn_angle = 0.0f;
  mc->t1wt.repeat_chain = 1;
  mc->t1wt.saved_pivot_speed = mc->pivot_turn_speed;
  mc->t1wt.state_start_ms = mc->turn_start_time;

  mc->t1wt.l0_th = l0_th;
  mc->t1wt.r0_th = r0_th;
  mc->t1wt.l0_off_th = (uint16_t)(l0_th * 0.40f);
  mc->t1wt.r0_off_th = (uint16_t)(r0_th * 0.40f);

  if (mc->current_speed < 10.0f)
    mc->current_speed = 10.0f;

  PID_Reset(&mc->pid_heading);
  PID_Reset(&mc->pid_angle);
}

void Motion_FrontAlign(Motion_Controller_t *mc, IR_Simple_Calib_t *calib) {
  if (!mc || !calib)
    return;

  Hardware_StopMotors();

  mc->state = MOTION_FRONT_ALIGN;
  mc->complete = 0;

  if (calib->has_front_diff) {
    /* Step 3 data: gain-equalized, more accurate */
    mc->align.fsum_target = (float)calib->front_fsum_target;
    mc->align.fdiff_ref = (float)calib->front_diff_zero;
  } else {
    /* Fallback to Step 1 raw center values */
    mc->align.fsum_target = (float)(calib->center_l90 + calib->center_r90);
    mc->align.fdiff_ref = (float)calib->center_l90 - (float)calib->center_r90;
  }

  mc->align.th_front = mc->align.fsum_target *
                       0.15f; /* 15% threshold: only reject truly absent wall */
  mc->align.start_time = millis();
  mc->align.stable_count = 0;
  mc->align.result = 0; /* 0 = in progress */
}

/**
 * @brief Snap gyro yaw to nearest 90-degree grid line.
 *        Eliminates accumulated drift so subsequent Motion_Straight
 *        uses a clean cardinal reference.
 */
void Motion_SnapHeading(void) {
  float cur = MPU6050_GetYaw(&sensor_fusion.mpu_handle);
  float snapped;
  char buf[48];
  /* roundf to nearest 90 */
  if (cur >= 0.0f)
    snapped = ((int)(cur + 45.0f) / 90) * 90.0f;
  else
    snapped = ((int)(cur - 45.0f) / 90) * 90.0f;
  MPU6050_SetYaw(&sensor_fusion.mpu_handle, snapped);
  sprintf(buf, "SNAP: %.1f -> %.0f\r\n", cur, snapped);
  UART_SendString(buf);
}

void Motion_Turn(Motion_Controller_t *mc, float angle_deg) {
  if (!mc)
    return;

  Hardware_ResetEncoder(0);
  Hardware_ResetEncoder(1);

  mc->complete = 0;
  mc->target_angle = angle_deg;
  mc->start_yaw = MPU6050_GetYaw(&sensor_fusion.mpu_handle);
  mc->angle_stable_count = 0;

  if (fabsf(angle_deg) > 135.0f) {
    mc->state = MOTION_TURN_180;
  } else if (angle_deg > 0) {
    mc->state = MOTION_TURN_LEFT;
  } else {
    mc->state = MOTION_TURN_RIGHT;
  }

  mc->turn_start_time = millis(); /* Hard timeout reference */
  PID_Reset(&mc->pid_angle);
}

void Motion_Update(Motion_Controller_t *mc, float dt) {
  float current_yaw, angle_traveled, angle_error, heading_error;
  float correction, speed, pid_output;
  int32_t enc_left, enc_right;
  float dist_left, dist_right;
  int pwm_left, pwm_right;
  static uint32_t debug_timer = 0;
  char buf[128];
  float gyro_z;
  float abs_gyro_z;

  if (!mc || mc->complete)
    return;

  /* [CRITICAL] Allow dt down to 1ms (1000Hz) */
  if (dt < 0.0005f || dt > 0.1f)
    return;

  enc_left = Hardware_GetEncoderCount(1) * ENCODER_DIR_LEFT;
  enc_right = Hardware_GetEncoderCount(0) * ENCODER_DIR_RIGHT;
  dist_left = (float)enc_left * MM_PER_PULSE;
  dist_right = (float)enc_right * MM_PER_PULSE;
  mc->traveled_distance = (dist_left + dist_right) / 2.0f;

  /* Compute actual speed from encoder derivative (mm/s) with low-pass filter */
  {
    float delta_dist = mc->traveled_distance - mc->prev_traveled_distance;
    float raw_speed = delta_dist / dt; /* mm/s at 1kHz */
    /* Low-pass filter: smooth out encoder quantization noise
     * alpha=0.1 → ~10ms time constant at 1kHz */
    mc->actual_speed = mc->actual_speed * 0.9f + raw_speed * 0.1f;
    mc->prev_traveled_distance = mc->traveled_distance;
  }

  current_yaw = MPU6050_GetYaw(&sensor_fusion.mpu_handle);
  gyro_z = MPU6050_GetGyroZ(&sensor_fusion.mpu_handle);
  abs_gyro_z = fabsf(gyro_z);

  switch (mc->state) {
  case MOTION_STRAIGHT:
    if (mc->traveled_distance >= mc->target_distance - DISTANCE_TOLERANCE) {
      Hardware_StopMotors();
      mc->complete = 1;
      break;
    }

    if (mc->speed_pid_enable) {
      /* Closed-loop: feedforward + PID correction */
      float ff_pwm, pid_corr;
      mc->desired_speed = ComputeDesiredSpeed_PID(mc, dt);
      ff_pwm = mc->speed_ff_gain *
               mc->desired_speed; /* Feedforward: ~72% PWM at 400mm/s */
      pid_corr =
          PID_Compute(&mc->pid_speed, mc->desired_speed - mc->actual_speed, dt);
      speed = ff_pwm + pid_corr;
      speed = Clamp(speed, 0.0f, (float)PWM_MAX);
    } else {
      /* Open-loop PWM ramp (explore mode) */
      speed = ComputeRampedSpeed(mc, mc->traveled_distance, dt);
    }

    heading_error = NormalizeAngle(current_yaw - mc->start_yaw);
    correction = PID_Compute(&mc->pid_heading, heading_error, dt);

    /* Wall-following: separate PID adds directly to correction */
    mc->wall_correction = 0.0f;
    if (mc->wall_steer_enable && sensor_fusion.wall_steer_mode > 0 &&
        mc->traveled_distance > 20.0f) {
      mc->wall_correction =
          PID_Compute(&mc->pid_wall, sensor_fusion.wall_steering_error, dt);
      correction += mc->wall_correction;
    }

    pwm_left = (int)(speed + correction);
    pwm_right = (int)(speed - correction);

    pwm_left = (int)Clamp((float)pwm_left, (float)PWM_MIN, (float)PWM_MAX);
    pwm_right = (int)Clamp((float)pwm_right, (float)PWM_MIN, (float)PWM_MAX);

    Hardware_SetMotor(1, pwm_left);
    Hardware_SetMotor(0, pwm_right);

    if (millis() - debug_timer > 500) {
      if (mc->speed_pid_enable) {
      sprintf(buf, "Str: d=%ld/%ld v=%ld/%ld pwm=%ld\r\n",
        (long)mc->traveled_distance, (long)mc->target_distance,
        (long)mc->actual_speed, (long)mc->desired_speed, (long)speed);
      } else {
        sprintf(buf, "Str: d=%.0f/%.0f yaw=%.1f err=%.1f\r\n",
                mc->traveled_distance, mc->target_distance, current_yaw,
                heading_error);
      }
      UART_SendString(buf);
      debug_timer = millis();
    }
    break;

  case MOTION_STRAIGHT_CONSTANT:
    if (mc->traveled_distance >= mc->target_distance) {
      /* Target Reached. DO NOT STOP MOTORS. DO NOT RESET ENCODERS. Just Mark
       * Complete */
      mc->complete = 1;
      break;
    }

    if (mc->speed_pid_enable) {
      /* Closed-loop: feedforward + PID, no decel (constant) */
      float ff_pwm, pid_corr;
      if (mc->desired_speed < mc->max_speed_mmps) {
        mc->desired_speed += mc->accel_mmps2 * dt;
        if (mc->desired_speed > mc->max_speed_mmps)
          mc->desired_speed = mc->max_speed_mmps;
      }
      ff_pwm = mc->speed_ff_gain * mc->desired_speed;
      pid_corr =
          PID_Compute(&mc->pid_speed, mc->desired_speed - mc->actual_speed, dt);
      speed = ff_pwm + pid_corr;
      speed = Clamp(speed, 0.0f, (float)PWM_MAX);
    } else {
      /* Open-loop: accelerate PWM to max, hold */
      if (mc->current_speed < mc->max_speed) {
        mc->current_speed += mc->accel_rate * dt;
        if (mc->current_speed > mc->max_speed)
          mc->current_speed = mc->max_speed;
      }
      speed = mc->current_speed;
    }

    /* Debug: log every 200ms */
    if (millis() - debug_timer > 200) {
      if (mc->speed_pid_enable) {
      sprintf(buf, "SC: d=%ld/%ld v=%ld/%ld pwm=%ld\r\n",
        (long)mc->traveled_distance, (long)mc->target_distance,
        (long)mc->actual_speed, (long)mc->desired_speed, (long)speed);
      } else {
        sprintf(buf, "SC: L=%ld R=%ld d=%.0f/%.0f spd=%.0f\r\n", enc_left,
                enc_right, mc->traveled_distance, mc->target_distance,
                mc->current_speed);
      }
      UART_SendString(buf);
      debug_timer = millis();
    }

    heading_error = NormalizeAngle(current_yaw - mc->start_yaw);
    correction = PID_Compute(&mc->pid_heading, heading_error, dt);

    /* Wall-following: separate PID adds directly to correction */
    mc->wall_correction = 0.0f;
    if (mc->wall_steer_enable && sensor_fusion.wall_steer_mode > 0) {
      mc->wall_correction =
          PID_Compute(&mc->pid_wall, sensor_fusion.wall_steering_error, dt);
      correction += mc->wall_correction;
    }

    pwm_left = (int)(speed + correction);
    pwm_right = (int)(speed - correction);

    pwm_left = (int)Clamp((float)pwm_left, (float)PWM_MIN, (float)PWM_MAX);
    pwm_right = (int)Clamp((float)pwm_right, (float)PWM_MIN, (float)PWM_MAX);

    Hardware_SetMotor(1, pwm_left);
    Hardware_SetMotor(0, pwm_right);
    break;

  case MOTION_TURN_LEFT:
  case MOTION_TURN_RIGHT:
  case MOTION_TURN_180:
    angle_traveled = NormalizeAngle(current_yaw - mc->start_yaw);
    angle_error = NormalizeAngle(mc->target_angle - angle_traveled);

    /* [FIX] Hard timeout: force-complete if turn takes > 3 seconds total */
    if (millis() - mc->turn_start_time > 500) {
      Hardware_StopMotors();
      mc->complete = 1;
      break;
    }

    /* [FIX] ALWAYS require low velocity to enter stabilize */
    if ((fabsf(angle_error) < ANGLE_TOLERANCE &&
         abs_gyro_z < VELOCITY_STABILIZE_THRESHOLD) ||
        (fabsf(angle_error) < ANGLE_WITH_VELOCITY_THRESHOLD &&
         abs_gyro_z < 10.0f)) {

      /* [CRITICAL] Stop motors immediately on stabilize entry */
      Hardware_StopMotors();

      mc->state = MOTION_TURN_STABILIZING;
      mc->stabilize_start_time = millis();
      mc->angle_stable_count = 0;
      break;
    }

    /* PD Controller - D term handles damping naturally */
    pid_output = PID_Compute(&mc->pid_angle, angle_error, dt);

    correction = Clamp(pid_output, -mc->turn_speed, mc->turn_speed);

    pwm_left = -(int)correction;
    pwm_right = (int)correction;

    pwm_left = (int)Clamp((float)pwm_left, -100.0f, 100.0f);
    pwm_right = (int)Clamp((float)pwm_right, -100.0f, 100.0f);

    Hardware_SetMotor(1, pwm_left);
    Hardware_SetMotor(0, pwm_right);
    break;

  case MOTION_TURN_STABILIZING:
    angle_traveled = NormalizeAngle(current_yaw - mc->start_yaw);
    angle_error = NormalizeAngle(mc->target_angle - angle_traveled);

    /* [CRITICAL] Keep motors stopped - passive braking only */
    Hardware_StopMotors();

    /* [FIX] Hard timeout from TURN START (not stabilize entry) - prevents
     * infinite turn-stabilize loop */
    if (millis() - mc->turn_start_time >= 500) {
      Hardware_StopMotors();
      mc->complete = 1;
      break;
    }

    /* [FIX] Stabilize-phase timeout: force-complete after 1 second in stabilize
     */
    if (millis() - mc->stabilize_start_time >= 500) {
      Hardware_StopMotors();
      mc->complete = 1;
      break;
    }

    /* [FIX] Check for large overshoot - back to active control (but only once)
     */
    if (fabsf(angle_error) > 10.0f && mc->angle_stable_count < 2) {
      mc->angle_stable_count++; /* Track re-entry count to prevent infinite loop
                                 */
      if (angle_error > 0) {
        mc->state = MOTION_TURN_LEFT;
      } else {
        mc->state = MOTION_TURN_RIGHT;
      }
      mc->pid_angle.integral = 0.0f;
      break;
    }

    /* [FIX] Complete when velocity drops and time elapsed */
    if (millis() - mc->stabilize_start_time >= mc->stabilize_duration &&
        abs_gyro_z < 5.0f) {

      Hardware_StopMotors();
      mc->complete = 1;
    }
    break;

  case MOTION_SMOOTH_LEFT:
  case MOTION_SMOOTH_RIGHT: {
    /* Compatibility: route legacy smooth states into T1WT pivot engine. */
    mc->t1wt.repeat_chain = 0;
    mc->t1wt.turn_angle = (mc->state == MOTION_SMOOTH_LEFT) ? 90.0f : -90.0f;
    mc->target_angle = mc->t1wt.turn_angle;
    mc->t1wt.target_dist = CELL_SIZE_MM / 3.0f;
    mc->t1wt.saved_pivot_speed = mc->pivot_turn_speed;
    mc->t1wt.state_start_ms = millis();
    mc->state = MOTION_T1WT_PIVOT;
    PID_Reset(&mc->pid_angle);
    break;
  }

  case MOTION_T1WT_APPROACH: {
    uint16_t l0 = (uint16_t)sensor_fusion.ir_sensors[2];
    uint16_t r0 = (uint16_t)sensor_fusion.ir_sensors[3];
    uint16_t l45 = (uint16_t)sensor_fusion.ir_sensors[1];
    uint16_t r45 = (uint16_t)sensor_fusion.ir_sensors[4];
    uint8_t seen_l0 = (l0 > mc->t1wt.l0_th);
    uint8_t seen_r0 = (r0 > mc->t1wt.r0_th);
    uint8_t seen_l45 = (l45 > T1WT_L45_PRESENT_TH);
    uint8_t seen_r45 = (r45 > T1WT_L45_PRESENT_TH);

    if (millis() - mc->t1wt.state_start_ms > T1WT_APPROACH_TIMEOUT_MS) {
      Hardware_StopMotors();
      mc->state = MOTION_IDLE;
      mc->complete = 1;
      break;
    }

    if (!mc->t1wt.l45_lost) {
      if ((seen_l0 && seen_l45 && l45 < T1WT_L45_LOST_TH) ||
          (seen_r0 && seen_r45 && r45 < T1WT_L45_LOST_TH)) {
        mc->t1wt.l45_lost = 1;
        mc->t1wt.l45_loss_dist = mc->traveled_distance;
      }
    } else if (!mc->t1wt.l0_lost) {
      if (seen_l0 && l0 < mc->t1wt.l0_off_th) {
        float k = mc->traveled_distance - mc->t1wt.l45_loss_dist;
        mc->t1wt.l0_lost = 1;
        if (k > 15.0f && k < 150.0f)
          mc->t1wt.estimated_k = k;
        else
          k = mc->t1wt.estimated_k;
        mc->t1wt.target_dist = mc->traveled_distance + (k / 3.0f);
        mc->t1wt.turn_angle = 90.0f;
      } else if (seen_r0 && r0 < mc->t1wt.r0_off_th) {
        float k = mc->traveled_distance - mc->t1wt.l45_loss_dist;
        mc->t1wt.l0_lost = 1;
        if (k > 15.0f && k < 150.0f)
          mc->t1wt.estimated_k = k;
        else
          k = mc->t1wt.estimated_k;
        mc->t1wt.target_dist = mc->traveled_distance + (k / 3.0f);
        mc->t1wt.turn_angle = -90.0f;
      }
    } else {
      if (mc->traveled_distance >= mc->t1wt.target_dist) {
        Hardware_ResetEncoder(0);
        Hardware_ResetEncoder(1);
        mc->start_yaw = MPU6050_GetYaw(&sensor_fusion.mpu_handle);
        mc->target_angle = mc->t1wt.turn_angle;
        mc->t1wt.saved_pivot_speed = mc->pivot_turn_speed;
        mc->pivot_turn_speed = 100.0f;
        PID_Reset(&mc->pid_angle);

        mc->t1wt.state_start_ms = millis();
        mc->state = MOTION_T1WT_PIVOT;
        break;
      }
    }

    DriveStraightOpenLoopHeading(mc, dt, mc->current_speed);
    break;
  }

  case MOTION_T1WT_PIVOT: {
    float angle_traveled_pv = NormalizeAngle(current_yaw - mc->start_yaw);
    float angle_error_pv = NormalizeAngle(mc->target_angle - angle_traveled_pv);
    int pivot_pwm;

    if (millis() - mc->t1wt.state_start_ms > T1WT_PIVOT_TIMEOUT_MS) {
      Hardware_StopMotors();
      mc->pivot_turn_speed = mc->t1wt.saved_pivot_speed;
      mc->state = MOTION_IDLE;
      mc->complete = 1;
      break;
    }

    if (fabsf(angle_error_pv) < ANGLE_TOLERANCE) {
      Hardware_StopMotors();
      mc->pivot_turn_speed = mc->t1wt.saved_pivot_speed;

      Hardware_ResetEncoder(0);
      Hardware_ResetEncoder(1);
      mc->target_distance = CELL_SIZE_MM / 3.0f;
      mc->start_yaw = MPU6050_GetYaw(&sensor_fusion.mpu_handle);
      mc->t1wt.target_dist = mc->target_distance;
      mc->t1wt.state_start_ms = millis();
      PID_Reset(&mc->pid_heading);
      mc->state = MOTION_T1WT_EXIT;
      break;
    }

    pid_output = PID_Compute(&mc->pid_angle, angle_error_pv, dt);
    pivot_pwm = (int)Clamp(fabsf(pid_output), 20.0f, mc->pivot_turn_speed);

    if (mc->target_angle > 0) {
      if (angle_error_pv > 0) {
        Hardware_SetMotor(1, 0);
        Hardware_SetMotor(0, pivot_pwm);
      } else {
        Hardware_SetMotor(1, pivot_pwm);
        Hardware_SetMotor(0, 0);
      }
    } else {
      if (angle_error_pv < 0) {
        Hardware_SetMotor(1, pivot_pwm);
        Hardware_SetMotor(0, 0);
      } else {
        Hardware_SetMotor(1, 0);
        Hardware_SetMotor(0, pivot_pwm);
      }
    }

    break;
  }

  case MOTION_T1WT_EXIT: {
    if (millis() - mc->t1wt.state_start_ms > T1WT_EXIT_TIMEOUT_MS) {
      Hardware_StopMotors();
      mc->state = MOTION_IDLE;
      mc->complete = 1;
      break;
    }

    if (mc->traveled_distance >= mc->target_distance) {
      uint16_t l90_val = (uint16_t)sensor_fusion.ir_sensors[0];
      uint16_t r90_val = (uint16_t)sensor_fusion.ir_sensors[5];

      Hardware_StopMotors();

      if (mc->t1wt.repeat_chain && l90_val < T1WT_FRONT_OPEN_TH &&
          r90_val < T1WT_FRONT_OPEN_TH) {
        uint16_t l0_val = (uint16_t)sensor_fusion.ir_sensors[2];
        uint16_t r0_val = (uint16_t)sensor_fusion.ir_sensors[3];

        if (l0_val < T1WT_SIDE_OPEN_TH) {
          mc->target_angle = 90.0f;
        } else if (r0_val < T1WT_SIDE_OPEN_TH) {
          mc->target_angle = -90.0f;
        } else {
          mc->state = MOTION_IDLE;
          mc->complete = 1;
          break;
        }

        Hardware_ResetEncoder(0);
        Hardware_ResetEncoder(1);
        mc->start_yaw = MPU6050_GetYaw(&sensor_fusion.mpu_handle);
        mc->t1wt.saved_pivot_speed = mc->pivot_turn_speed;
        mc->pivot_turn_speed = 100.0f;
        mc->t1wt.state_start_ms = millis();
        PID_Reset(&mc->pid_angle);
        mc->state = MOTION_T1WT_PIVOT;
      } else {
        mc->state = MOTION_IDLE;
        mc->complete = 1;
      }
      break;
    }

    DriveStraightOpenLoopHeading(mc, dt, mc->current_speed);
    break;
  }

  default:
    Hardware_StopMotors();
    mc->complete = 1;
    break;

  case MOTION_FRONT_ALIGN: {
    float fsum, fdiff, e_dist, e_yaw;
    float v, w;
    int pl, pr;
    uint32_t elapsed = millis() - mc->align.start_time;

    /* Timeout check */
    if (elapsed > mc->align.timeout_ms) {
      Hardware_StopMotors();
      mc->align.result = 2; /* timeout */
      mc->complete = 1;
      break;
    }

    /* Use EMA-filtered IR from sensor fusion (updated at 1kHz in same ISR) */
    fsum = sensor_fusion.ir_sensors[0] + sensor_fusion.ir_sensors[5];
    fdiff = sensor_fusion.ir_sensors[0] - sensor_fusion.ir_sensors[5];

    /* Grace period: let EMA filter stabilize before checking gating.
     * During grace, just drive forward gently if Fsum is too low. */
    if (elapsed < mc->align.grace_ms) {
      if (fsum < mc->align.th_front) {
        /* During grace: drive forward slowly to approach wall */
        Hardware_SetMotor(1, mc->align.min_pwm);
        Hardware_SetMotor(0, mc->align.min_pwm);
      }
      /* Don't check gating, don't compute errors yet */
      break;
    }

    /* After grace period: check gating */
    if (fsum < mc->align.th_front) {
      Hardware_StopMotors();
      mc->align.result = 3; /* no front wall */
      mc->complete = 1;
      break;
    }

    /* Compute errors
     * e_dist: positive = too far, negative = too close
     * e_yaw:  fdiff - fdiff_ref
     *   When nose points RIGHT: L90 > R90 → fdiff > fdiff_ref → e_yaw > 0
     *   Positive e_yaw → w > 0 → vR > vL → turn LEFT → correct! ✓
     */
    e_dist = mc->align.fsum_target - fsum;
    e_yaw = fdiff - mc->align.fdiff_ref;

    /* Check stability */
    if (fabsf(e_dist) < mc->align.tolerance_d &&
        fabsf(e_yaw) < mc->align.tolerance_h) {
      mc->align.stable_count++;
      if (mc->align.stable_count >= mc->align.stable_needed) {
        Hardware_StopMotors();
        mc->align.result = 1; /* OK */
        mc->complete = 1;
        break;
      }
    } else {
      mc->align.stable_count = 0;
    }

    /* Control law: v = Kd * e_dist, w = Ky * e_yaw */
    v = mc->align.kp_dist * e_dist;
    w = mc->align.kp_yaw * e_yaw;

    /* Mix: vL = v - w, vR = v + w
     * w > 0 → vR bigger → turn LEFT ✓ */
    pl = (int)(v - w);
    pr = (int)(v + w);

    /* Deadzone compensation: boost small non-zero PWM to minimum */
    if (pl > 0 && pl < mc->align.min_pwm)
      pl = mc->align.min_pwm;
    if (pl < 0 && pl > -mc->align.min_pwm)
      pl = -mc->align.min_pwm;
    if (pr > 0 && pr < mc->align.min_pwm)
      pr = mc->align.min_pwm;
    if (pr < 0 && pr > -mc->align.min_pwm)
      pr = -mc->align.min_pwm;

    /* Right motor offset: compensate friction asymmetry */
    if (pr > 0)
      pr += mc->align.motor_offset_R;
    if (pr < 0)
      pr -= mc->align.motor_offset_R;

    /* Both errors very small = aligned, coast to stop */
    if (fabsf(e_dist) < mc->align.tolerance_d * 0.25f &&
        fabsf(e_yaw) < mc->align.tolerance_h * 0.25f) {
      pl = 0;
      pr = 0;
    }

    /* Clamp */
    if (pl > mc->align.max_pwm)
      pl = mc->align.max_pwm;
    if (pl < -mc->align.max_pwm)
      pl = -mc->align.max_pwm;
    if (pr > mc->align.max_pwm)
      pr = mc->align.max_pwm;
    if (pr < -mc->align.max_pwm)
      pr = -mc->align.max_pwm;

    Hardware_SetMotor(1, pl);
    Hardware_SetMotor(0, pr);
    break;
  }
  }
}

void Motion_Stop(Motion_Controller_t *mc) {
  if (!mc)
    return;
  Hardware_StopMotors();
  mc->state = MOTION_IDLE;
  mc->complete = 1;
  mc->current_speed = 0.0f;
}

uint8_t Motion_IsComplete(Motion_Controller_t *mc) {
  return mc ? mc->complete : 1;
}

/* ========================================================================= */
/* PID Hard-Realtime Interrupt (TIM11 - 1kHz/1ms)                            */
/* ========================================================================= */

void TIM1_TRG_COM_TIM11_IRQHandler(void) {
  /* Check if TIM11 caused the interrupt */
  if (TIM11->SR & (1 << 0)) {
    /* Clear the update interrupt flag */
    TIM11->SR &= ~(1 << 0);

    /* 1. Read Sensors (Hardware layer) */
    MPU6050_Update(&sensor_fusion.mpu_handle);
    Hardware_UpdateEncoders();

    /* IR Sensor triggers its own scan independently in TIM10,
       we just make sure it keeps running if it stopped */
    if (!IR_Sensor_IsReady()) {
      IR_Sensor_StartScan();
    }

    /* 2. Update Sensor Fusion (Dt = 0.001f fixed) */
    SensorFusion_Update(&sensor_fusion, 0.001f);

    /* 3. Execute PID & Send PWM to Motors (Dt = 0.001f fixed) */
    Motion_Update(&motion_ctrl, 0.001f);
  }
}

void Motion_StartPID(void) {
  /* Reset counters before starting */
  TIM11->CNT = 0;
  TIM11->SR &= ~(1 << 0);
  TIM11->CR1 |= (1 << 0);
}

void Motion_StopPID(void) { TIM11->CR1 &= ~(1 << 0); }
