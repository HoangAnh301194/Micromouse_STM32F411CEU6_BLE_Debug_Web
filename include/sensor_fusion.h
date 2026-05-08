/**
 * @file sensor_fusion.h
 * @brief Sensor Fusion with Kalman Filter + IR Wall Calibration
 */

#ifndef SENSOR_FUSION_H
#define SENSOR_FUSION_H

#include "ir_simple_calib.h"
#include "mpu6050.h"
#include "pinout.h"
#include <stdint.h>

/* IR EMA Filter Alpha (0.0 = heavy filter, 1.0 = no filter) */
#define IR_EMA_ALPHA 0.8f
/* Slower EMA for L0/R0 wall sensors to smooth out post reflections */
#define IR_EMA_ALPHA_WALL 1.0f
/* Max change in wall_steering_error per cycle (rate limiter) */
#define WALL_STEER_MAX_RATE 0.01f

/* ========================================================================== */
/* TYPES                                                                      */
/* ========================================================================== */

typedef struct {
  /* MPU6050 */
  MPU6050_Handle_t mpu_handle;

  /* Position and Orientation */
  float pos_x;   /* X position in mm */
  float pos_y;   /* Y position in mm */
  float heading; /* Heading angle in degrees */
  float speed;   /* Linear speed in mm/s */

  /* Sensor Readings */
  float ir_sensors[6]; /* IR sensor values (L90, L45, L0, R0, R45, R90) */
  float encoder_left;  /* Left encoder velocity (pulses/sec) */
  float encoder_right; /* Right encoder velocity (pulses/sec) */
  float gyro_z;        /* Gyroscope Z-axis (deg/s) */

  /* Derived Data */
  float wall_error; /* Wall centering error */
  float
      wall_steering_error; /* Normalized lateral error for heading correction */
  uint8_t wall_steer_mode; /* 0=none, 1=left only, 2=right only, 3=both */
  uint8_t collision_detected;

  /* Kalman Filter Matrices */
  float P[4][4]; /* Covariance matrix */
  float Q[4][4]; /* Process noise */
  float R[2][2]; /* Measurement noise */

  /* IR Calibration */
  IR_Simple_Calib_t ir_calib;

  /* NEW: Wall-Following Setpoints */
  float ir_wall_left_setpoint;  /* Target ADC value for left wall */
  float ir_wall_right_setpoint; /* Target ADC value for right wall */
  uint8_t ir_wall_calibrated;   /* 1 if setpoints are valid */

} Sensor_Fusion_t;

/* ========================================================================== */
/* PUBLIC FUNCTIONS                                                           */
/* ========================================================================== */

/**
 * @brief Initialize sensor fusion system
 */
void SensorFusion_Init(Sensor_Fusion_t *sf);

/**
 * @brief Update sensor fusion (call in control loop)
 * @param dt: Delta time in seconds
 */
void SensorFusion_Update(Sensor_Fusion_t *sf, float dt);

/**
 * @brief Reset odometry to zero
 */
void SensorFusion_ResetOdometry(Sensor_Fusion_t *sf);

/**
 * @brief Set robot position manually
 */
void SensorFusion_SetPosition(Sensor_Fusion_t *sf, float x, float y,
                              float heading);

/**
 * @brief Get current position
 */
void SensorFusion_GetPosition(Sensor_Fusion_t *sf, float *x, float *y,
                              float *heading);

/**
 * @brief Get current speed
 */
float SensorFusion_GetSpeed(Sensor_Fusion_t *sf);

/**
 * @brief Get wall centering error
 */
float SensorFusion_GetWallError(Sensor_Fusion_t *sf);

/**
 * @brief Get wall-following steering error (normalized)
 * @return [-1.0, +1.0]: +ve = too close to left, -ve = too close to right
 */
float SensorFusion_GetWallSteeringError(Sensor_Fusion_t *sf);

/**
 * @brief Check for collision
 */
uint8_t SensorFusion_IsCollision(Sensor_Fusion_t *sf);

/**
 * @brief Detect walls in 3 directions (left, front, right)
 */
void SensorFusion_DetectWalls(Sensor_Fusion_t *sf, uint8_t *left,
                              uint8_t *front, uint8_t *right);

/**
 * @brief Print sensor fusion status (debug)
 */
void SensorFusion_PrintStatus(Sensor_Fusion_t *sf);

/**
 * @brief Get raw IR sensor values
 */
void SensorFusion_GetIRValues(Sensor_Fusion_t *sf, uint16_t *values);

/**
 * @brief Self-test all sensors
 */
uint8_t SensorFusion_SelfTest(Sensor_Fusion_t *sf);

/**
 * @brief Detect wall level (0=no wall, 1=wall, 2=collision risk)
 */
uint8_t SensorFusion_DetectWall_Level(Sensor_Fusion_t *sf,
                                      uint8_t sensor_index);

/* ========================================================================== */
/* NEW: WALL-FOLLOWING CALIBRATION FUNCTIONS                                  */
/* ========================================================================== */

/**
 * @brief Auto-calibrate IR wall setpoints at starting position
 * @note Robot MUST be in a cell with LEFT and RIGHT walls
 * @return 1 if success, 0 if failed
 */
uint8_t SensorFusion_CalibrateWallSetpoints(Sensor_Fusion_t *sf);

/**
 * @brief Get wall-following setpoints
 * @param left_sp: Output left wall setpoint (ADC units)
 * @param right_sp: Output right wall setpoint (ADC units)
 */
void SensorFusion_GetWallSetpoints(Sensor_Fusion_t *sf, float *left_sp,
                                   float *right_sp);

/**
 * @brief Check if wall calibration is valid
 * @return 1 if calibrated, 0 if not
 */
uint8_t SensorFusion_IsWallCalibrated(Sensor_Fusion_t *sf);

#endif /* SENSOR_FUSION_H */
