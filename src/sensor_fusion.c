/**
 * @file sensor_fusion.c
 * @brief Sensor Fusion with Kalman Filter + Wall-Following Steering
 */

#include "sensor_fusion.h"
#include "hardware.h"
#include "ir_sensor.h"
#include "mpu6050.h"
#include "system_timer.h"


#include "ir_simple_calib.h"
#include <math.h>
#include <stdio.h>
#include <string.h>


/* ========================================================================== */
/* CONSTANTS (from pinout.h)                                                  */
/* ========================================================================== */

#define PI 3.14159265f

/* IR sensor thresholds */
#define IR_WALL_THRESHOLD 500
#define IR_FAR_THRESHOLD 200

/* Dynamic Complementary Filter parameters */
#define CF_ALPHA_BASE 0.98f /* Base trust in Gyro (98%) */
#define CF_ALPHA_MIN                                                           \
  0.85f /* Minimum trust in Gyro during violent turns (85%) */
#define GYRO_VIOLENT_THRESHOLD                                                 \
  150.0f /* deg/s threshold where Gyro drift becomes severe */

/* Collision detection */
#define COLLISION_THRESHOLD 3000

/* ========================================================================== */
/* PRIVATE VARIABLES                                                          */
/* ========================================================================== */

static int32_t last_encoder_left = 0;
static int32_t last_encoder_right = 0;

/* ========================================================================== */
/* PRIVATE FUNCTIONS                                                          */
/* ========================================================================== */

static float PulsesToMM(int32_t pulses) {
  float wheel_circumference = WHEEL_DIAMETER_MM * PI;
  return (float)pulses * wheel_circumference / (float)ENCODER_PPR;
}

static float PulsesToPPS(int32_t pulses, float dt) {
  if (dt < 0.0001f)
    return 0.0f;
  return (float)pulses / dt;
}

static void ReadGyro(Sensor_Fusion_t *sf) {
  MPU6050_Update(&sf->mpu_handle);
  sf->heading = MPU6050_GetYaw(&sf->mpu_handle);
  sf->gyro_z = MPU6050_GetGyroZ(&sf->mpu_handle);
}

static void ReadEncoders(Sensor_Fusion_t *sf, float dt) {
  int32_t enc_left, enc_right;
  int32_t delta_left, delta_right;
  float distance_left, distance_right;
  float delta_distance;

  enc_left = Hardware_GetEncoderCount(1);
  enc_right = Hardware_GetEncoderCount(0);

  delta_left = enc_left - last_encoder_left;
  delta_right = enc_right - last_encoder_right;

  distance_left = PulsesToMM(delta_left);
  distance_right = PulsesToMM(delta_right);

  delta_distance = (distance_left + distance_right) / 2.0f;

  sf->encoder_left = PulsesToPPS(delta_left, dt);
  sf->encoder_right = PulsesToPPS(delta_right, dt);

  sf->pos_x += delta_distance * cosf(sf->heading * PI / 180.0f);
  sf->pos_y += delta_distance * sinf(sf->heading * PI / 180.0f);

  if (dt > 0.0001f) {
    sf->speed = delta_distance / dt;
  }

  last_encoder_left = enc_left;
  last_encoder_right = enc_right;
}

static void ReadIRSensors(Sensor_Fusion_t *sf) {
  uint16_t ir_values[6];
  uint8_t i;

  IR_Sensor_GetResults(ir_values);

  for (i = 0; i < 6; i++) {
    sf->ir_sensors[i] = (float)ir_values[i];
  }

  sf->collision_detected = 0;
  for (i = 0; i < 6; i++) {
    if (sf->ir_sensors[i] > COLLISION_THRESHOLD) {
      sf->collision_detected = 1;
      break;
    }
  }
}

/**
 * @brief Compute wall-following steering error from L0/R0 sensors
 *
 * Error is normalized to [-1.0, +1.0]:
 *   +ve = robot too close to left wall → steer right
 *   -ve = robot too close to right wall → steer left
 *   0   = centered or no walls detected
 */
static void ComputeWallSteering(Sensor_Fusion_t *sf) {
  float l0 = sf->ir_sensors[2]; /* L0: perpendicular left */
  float r0 = sf->ir_sensors[3]; /* R0: perpendicular right */
  float l0_sp, r0_sp;
  uint8_t has_left, has_right;

  /* Apply gain equalization so L0 and R0 have matching response curves */
  l0 *= sf->ir_calib.gain_l0;
  r0 *= sf->ir_calib.gain_r0;

  l0_sp = (float)sf->ir_calib.center_l0 * sf->ir_calib.gain_l0;
  r0_sp = (float)sf->ir_calib.center_r0 * sf->ir_calib.gain_r0;

  /* Not calibrated or invalid setpoints → no wall steering */
  if (!sf->ir_calib.is_calibrated || l0_sp < 100.0f || r0_sp < 100.0f) {
    sf->wall_steering_error = 0.0f;
    sf->wall_steer_mode = 0;
    return;
  }

  /* Detect walls using 50% of calibrated threshold (more sensitive for
   * steering). Thresholds are also gain-equalized to match l0/r0. */
  has_left =
      (l0 > (float)sf->ir_calib.wall_threshold_front_left * sf->ir_calib.gain_l0 * 0.5f) ? 1 : 0;
  has_right =
      (r0 > (float)sf->ir_calib.wall_threshold_front_right * sf->ir_calib.gain_r0 * 0.5f) ? 1 : 0;

  /* --- Normalized steering using min/max (Mode 2, or Mode 1 with open calib) --- */
  if ((sf->ir_calib.calib_mode >= 2 || sf->ir_calib.has_open_calib) &&
      sf->ir_calib.l0_max > sf->ir_calib.l0_min + 30 &&
      sf->ir_calib.r0_max > sf->ir_calib.r0_min + 30) {

    float l0_min_eq = (float)sf->ir_calib.l0_min * sf->ir_calib.gain_l0;
    float l0_max_eq = (float)sf->ir_calib.l0_max * sf->ir_calib.gain_l0;
    float r0_min_eq = (float)sf->ir_calib.r0_min * sf->ir_calib.gain_r0;
    float r0_max_eq = (float)sf->ir_calib.r0_max * sf->ir_calib.gain_r0;
    float l0_range = l0_max_eq - l0_min_eq;
    float r0_range = r0_max_eq - r0_min_eq;
    float l0_norm, r0_norm;

    if (has_left && has_right) {
      sf->wall_steer_mode = 3;
      /* Normalize both sensors to [0.0, 1.0] */
      l0_norm = (l0 - l0_min_eq) / l0_range;
      r0_norm = (r0 - r0_min_eq) / r0_range;
      /* Clamp */
      if (l0_norm < 0.0f) l0_norm = 0.0f;
      if (l0_norm > 1.0f) l0_norm = 1.0f;
      if (r0_norm < 0.0f) r0_norm = 0.0f;
      if (r0_norm > 1.0f) r0_norm = 1.0f;
      /* Error: positive = too close to left -> steer right */
      sf->wall_steering_error = l0_norm - r0_norm;
    } else if (has_left) {
      sf->wall_steer_mode = 1;
      l0_norm = (l0 - l0_min_eq) / l0_range;
      if (l0_norm < 0.0f) l0_norm = 0.0f;
      if (l0_norm > 1.0f) l0_norm = 1.0f;
      /* Center setpoint normalized */
      {
        float center_norm = (l0_sp - l0_min_eq) / l0_range;
        if (center_norm < 0.01f) center_norm = 0.01f;
        sf->wall_steering_error = (l0_norm - center_norm) / center_norm;
      }
    } else if (has_right) {
      sf->wall_steer_mode = 2;
      r0_norm = (r0 - r0_min_eq) / r0_range;
      if (r0_norm < 0.0f) r0_norm = 0.0f;
      if (r0_norm > 1.0f) r0_norm = 1.0f;
      {
        float center_norm = (r0_sp - r0_min_eq) / r0_range;
        if (center_norm < 0.01f) center_norm = 0.01f;
        sf->wall_steering_error = -(r0_norm - center_norm) / center_norm;
      }
    } else {
      sf->wall_steer_mode = 0;
      sf->wall_steering_error = 0.0f;
    }

  } else {
    /* --- Mode 1 (or invalid min/max): original steering logic --- */
    if (has_left && has_right) {
      sf->wall_steer_mode = 3;
      if ((l0 + r0) > 1.0f) {
        sf->wall_steering_error = (l0 - r0) / (l0 + r0);
      } else {
        sf->wall_steering_error = 0.0f;
      }
    } else if (has_left) {
      sf->wall_steer_mode = 1;
      sf->wall_steering_error = (l0 - l0_sp) / l0_sp;
    } else if (has_right) {
      sf->wall_steer_mode = 2;
      sf->wall_steering_error = -(r0 - r0_sp) / r0_sp;
    } else {
      sf->wall_steer_mode = 0;
      sf->wall_steering_error = 0.0f;
    }
  }

  /* Clamp to prevent wild corrections */
  if (sf->wall_steering_error > 1.0f)
    sf->wall_steering_error = 1.0f;
  if (sf->wall_steering_error < -1.0f)
    sf->wall_steering_error = -1.0f;

  /* Rate limiter: prevent sudden jumps when passing posts */
  {
    static float prev_wall_error = 0.0f;
    float delta = sf->wall_steering_error - prev_wall_error;
    if (delta > WALL_STEER_MAX_RATE)
      sf->wall_steering_error = prev_wall_error + WALL_STEER_MAX_RATE;
    else if (delta < -WALL_STEER_MAX_RATE)
      sf->wall_steering_error = prev_wall_error - WALL_STEER_MAX_RATE;
    prev_wall_error = sf->wall_steering_error;
  }
}

static void DynamicComplementaryFilter(Sensor_Fusion_t *sf,
                                       float gyro_heading_rate,
                                       float encoder_heading, float dt) {
  float alpha, abs_gyro_z;
  float innovation;

  /* 1. Calculate dynamic alpha based on angular velocity */
  abs_gyro_z = fabsf(gyro_heading_rate);

  if (abs_gyro_z < 10.0f) {
    /* Robot is going straight or minor corrections: Trust Gyro heavily because
     * it's fast and smooth */
    alpha = CF_ALPHA_BASE;
  } else {
    /* Robot is turning wildly: Gyro integrates large errors (Drift). Shift
     * trust gradually back to Encoders */
    float penalty =
        (abs_gyro_z / GYRO_VIOLENT_THRESHOLD) * (CF_ALPHA_BASE - CF_ALPHA_MIN);
    alpha = CF_ALPHA_BASE - penalty;

    /* Clamp alpha */
    if (alpha < CF_ALPHA_MIN)
      alpha = CF_ALPHA_MIN;
  }

  /* 2. Gyro prediction (High-Pass logic) */
  sf->heading = sf->heading + gyro_heading_rate * dt;

  /* Normalize heading before mixing to prevent 359 vs 1 degree jump disasters
   */
  while (sf->heading > 180.0f)
    sf->heading -= 360.0f;
  while (sf->heading < -180.0f)
    sf->heading += 360.0f;

  /* 3. Calculate gap between Prediction and physical Encoder reality */
  innovation = encoder_heading - sf->heading;

  /* Handle wrap-around for the innovation (e.g., Gyro says 179, Encoder says
   * -179) */
  while (innovation > 180.0f)
    innovation -= 360.0f;
  while (innovation < -180.0f)
    innovation += 360.0f;

  /* 4. Mix (Complementary Logic): Pull heading slightly towards the Encoder
   * reality */
  sf->heading = sf->heading + (1.0f - alpha) * innovation;

  /* Final normalization */
  while (sf->heading > 180.0f)
    sf->heading -= 360.0f;
  while (sf->heading < -180.0f)
    sf->heading += 360.0f;
}

/* ========================================================================== */
/* PUBLIC FUNCTIONS                                                           */
/* ========================================================================== */

void SensorFusion_Init(Sensor_Fusion_t *sf) {
  uint8_t i, j;

  memset(sf, 0, sizeof(Sensor_Fusion_t));

  MPU6050_Init(&sf->mpu_handle);
  delay_ms_blocking(100);
  MPU6050_Calibrate(&sf->mpu_handle);

  IR_Sensor_Init();
  delay_ms_blocking(100);
  IR_Sensor_StartScan();

  /* Initialize IR calibration for wall DETECTION only */
  IR_Simple_Init(&sf->ir_calib);

  sf->pos_x = 0.0f;
  sf->pos_y = 0.0f;
  sf->heading = 0.0f;
  sf->speed = 0.0f;
  sf->collision_detected = 0;

  for (i = 0; i < 4; i++) {
    for (j = 0; j < 4; j++) {
      sf->P[i][j] = (i == j) ? 1.0f : 0.0f;
      sf->Q[i][j] = (i == j) ? 0.01f : 0.0f;
    }
  }

  for (i = 0; i < 2; i++) {
    for (j = 0; j < 2; j++) {
      sf->R[i][j] = (i == j) ? 0.1f : 0.0f;
    }
  }

  last_encoder_left = Hardware_GetEncoderCount(1);
  last_encoder_right = Hardware_GetEncoderCount(0);
}

void SensorFusion_Update(Sensor_Fusion_t *sf, float dt) {
  float gyro_heading_rate;
  float encoder_delta_angle;
  uint8_t i;

  if (IR_Sensor_IsReady()) {
    uint16_t ir_raw[6];
    IR_Sensor_GetResults(ir_raw);

    for (i = 0; i < 6; i++) {
      if (sf->ir_sensors[i] < 1.0f) {
        /* First reading: prime the EMA filter */
        sf->ir_sensors[i] = (float)ir_raw[i];
      } else {
        /* EMA low-pass filter: use slower alpha for L0/R0 (index 2,3)
         * to smooth out post reflection spikes */
        float alpha = (i == 2 || i == 3) ? IR_EMA_ALPHA_WALL : IR_EMA_ALPHA;
        sf->ir_sensors[i] +=
            alpha * ((float)ir_raw[i] - sf->ir_sensors[i]);
      }
    }

    IR_Sensor_StartScan();
  }

  ReadGyro(sf);
  gyro_heading_rate = sf->gyro_z;

  ReadEncoders(sf, dt);

  Hardware_UpdateEncoders();

  encoder_delta_angle = (sf->encoder_right - sf->encoder_left) *
                        WHEEL_DIAMETER_MM * PI / (float)ENCODER_PPR /
                        WHEEL_BASE_MM * 180.0f / PI;

  /* We no longer pass dt to the CF for Gyro integration sinceReadGyro provides
     raw rate, wait, actually ReadGyro doesn't integrate, we need to do it. */
  DynamicComplementaryFilter(sf, gyro_heading_rate, encoder_delta_angle, dt);

  /* Compute wall-following steering error (runs every cycle) */
  ComputeWallSteering(sf);
}

void SensorFusion_ResetOdometry(Sensor_Fusion_t *sf) {
  sf->pos_x = 0.0f;
  sf->pos_y = 0.0f;
  sf->heading = 0.0f;
  sf->speed = 0.0f;

  MPU6050_ResetYaw(&sf->mpu_handle);

  last_encoder_left = Hardware_GetEncoderCount(1);
  last_encoder_right = Hardware_GetEncoderCount(0);
}

void SensorFusion_SetPosition(Sensor_Fusion_t *sf, float x, float y,
                              float heading) {
  sf->pos_x = x;
  sf->pos_y = y;
  sf->heading = heading;

  MPU6050_SetYaw(&sf->mpu_handle, heading);
}

void SensorFusion_GetPosition(Sensor_Fusion_t *sf, float *x, float *y,
                              float *heading) {
  if (x)
    *x = sf->pos_x;
  if (y)
    *y = sf->pos_y;
  if (heading)
    *heading = sf->heading;
}

float SensorFusion_GetSpeed(Sensor_Fusion_t *sf) { return sf->speed; }

uint8_t SensorFusion_IsCollision(Sensor_Fusion_t *sf) {
  return sf->collision_detected;
}

float SensorFusion_GetWallSteeringError(Sensor_Fusion_t *sf) {
  return sf->wall_steering_error;
}

void SensorFusion_DetectWalls(Sensor_Fusion_t *sf, uint8_t *left,
                              uint8_t *front, uint8_t *right) {
  uint16_t ir_raw[6];
  uint8_t i;

  /* Convert float to uint16_t */
  for (i = 0; i < 6; i++) {
    ir_raw[i] = (uint16_t)sf->ir_sensors[i];
  }

  /* Use IR calibration for detection */
  if (left) {
    *left = IR_Simple_DetectLeftWall(&sf->ir_calib, ir_raw);
  }

  if (front) {
    *front = IR_Simple_DetectFrontWall(&sf->ir_calib, ir_raw);
  }

  if (right) {
    *right = IR_Simple_DetectRightWall(&sf->ir_calib, ir_raw);
  }

  /* Check collision */
  sf->collision_detected = 0;
  for (i = 0; i < 6; i++) {
    if (ir_raw[i] > COLLISION_THRESHOLD) {
      sf->collision_detected = 1;
      break;
    }
  }
}

void SensorFusion_PrintStatus(Sensor_Fusion_t *sf) {
  char buf[256];

  sprintf(buf, "Pos: (%.1f, %.1f) mm, Heading: %.1f deg\r\n", sf->pos_x,
          sf->pos_y, sf->heading);

  sprintf(buf, "Speed: %.1f mm/s\r\n", sf->speed);

  sprintf(buf, "IR: L90=%d L45=%d L0=%d R0=%d R45=%d R90=%d\r\n",
          (int)sf->ir_sensors[0], (int)sf->ir_sensors[1],
          (int)sf->ir_sensors[2], (int)sf->ir_sensors[3],
          (int)sf->ir_sensors[4], (int)sf->ir_sensors[5]);

  if (sf->collision_detected) {
  }
}

void SensorFusion_GetIRValues(Sensor_Fusion_t *sf, uint16_t *values) {
  uint8_t i;
  if (values) {
    for (i = 0; i < 6; i++) {
      values[i] = (uint16_t)sf->ir_sensors[i];
    }
  }
}

uint8_t SensorFusion_SelfTest(Sensor_Fusion_t *sf) {
  uint8_t status, ir_ok, i;
  int32_t enc_l, enc_r;
  char buf[64];

  status = 1;

  if (MPU6050_IsReady(&sf->mpu_handle) != MPU6050_OK) {

    status = 0;
  } else {
  }

  enc_l = Hardware_GetEncoderCount(1);
  enc_r = Hardware_GetEncoderCount(0);
  sprintf(buf, "Encoders: L=%ld R=%ld\r\n", enc_l, enc_r);

  ReadIRSensors(sf);
  ir_ok = 1;
  for (i = 0; i < 6; i++) {
    if (sf->ir_sensors[i] < 10.0f || sf->ir_sensors[i] > 4090.0f) {
      ir_ok = 0;
    }
  }

  if (ir_ok) {

  } else {

    status = 0;
  }

  if (status) {

  } else {
  }

  return status;
}

uint8_t SensorFusion_DetectWall_Level(Sensor_Fusion_t *sf,
                                      uint8_t sensor_index) {
  uint16_t value;
  const uint16_t WALL_PRESENT = 400;
  const uint16_t COLLISION_RISK = 2000;

  if (sensor_index >= 6)
    return 0;

  value = (uint16_t)sf->ir_sensors[sensor_index];

  if (value > COLLISION_RISK) {
    return 2;
  } else if (value > WALL_PRESENT) {
    return 1;
  }

  return 0;
}
