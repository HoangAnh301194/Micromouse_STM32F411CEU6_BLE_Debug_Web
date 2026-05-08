/* ========================================================================== */
/* mpu6050.h - MPU6050 Driver Header - FIXED CONFIGURATION                   */
/* ========================================================================== */

#ifndef MPU6050_H
#define MPU6050_H

#include "i2c.h"
#include <stdint.h>


/* ========================================================================== */
/* CRITICAL CONFIGURATION - ADD THIS SECTION                                  */
/* ========================================================================== */

/**
 * @brief Gyro Full Scale Range Configuration
 *
 * CRITICAL: This value MUST match the scale calculation in gyro_scale!
 *
 * Range    | Value | Scale   | Max Rate  | Best For
 * ---------|-------|---------|-----------|---------------------------
 * 250/s  |   0   | 131.0   | 250/s    | High precision, slow turns
 * 500/s  |   1   | 65.5    | 500/s    | Recommended for micromouse
 * 1000/s |   2   | 32.8    | 1000/s   | Fast turns
 * 2000/s |   3   | 16.4    | 2000/s   | Very fast, lower precision
 */
#define MPU6050_GYRO_RANGE 1 /* Default: 500/s */

/**
 * @brief Gyro Low-Pass Filter Alpha
 * Range: 0.0 (heavily filtered) to 1.0 (no filtering)
 * Higher value = more responsive but noisier
 * Lower value = smoother but slower response
 */
#define GYRO_Z_LPF_ALPHA 0.7f

/**
 * @brief Gyro Deadzone Threshold (deg/s)
 * Values below this threshold are treated as zero to reduce drift
 * Typical: 0.2 - 0.5 deg/s
 */
#define GYRO_Z_DEADZONE 0.3f

/**
 * @brief Moving Average Window Size
 * Number of samples for moving average filter
 * Larger = smoother but slower response
 */
#define GYRO_Z_AVG_SAMPLES 3

/**
 * @brief Calibration Sample Count
 * Number of samples to collect for offset calculation
 * More samples = better accuracy but slower calibration
 *
 * Recommended values:
 * - 100:  Fast test (~0.3s)
 * - 500:  Good balance (~1.5s)
 * - 1000: High accuracy (~3s)
 */
#define CALIBRATION_SAMPLES 200 /* 100 samples - fast startup */

/* ========================================================================== */
/* HARDWARE CONFIGURATION                                                     */
/* ========================================================================== */

#define MPU6050_ADDR 0x68
#define MPU6050_I2C_INSTANCE I2C_1

/* ========================================================================== */
/* DATA STRUCTURES                                                            */
/* ========================================================================== */

typedef enum {
  MPU6050_OK = 0,
  MPU6050_ERROR,
  MPU6050_TIMEOUT,
  MPU6050_NOT_FOUND
} MPU6050_Status;

typedef struct {
  /* I2C Configuration */
  I2C_Instance i2c;
  uint8_t address;

  /* Gyro Configuration */
  float gyro_scale;

  /* Gyro Z-axis Data */
  float gyro_z_offset;                      /* Calibration offset (deg/s) */
  float gyro_z_lpf;                         /* Low-pass filtered value */
  float gyro_z_avg_buf[GYRO_Z_AVG_SAMPLES]; /* Moving average buffer */
  uint8_t gyro_z_avg_idx;                   /* Moving average index */
  float gyro_z_filtered;                    /* Final filtered output (deg/s) */

  /* Yaw Integration */
  float yaw;               /* Integrated yaw angle (degrees) */
  uint32_t last_update_us; /* Last update timestamp (microseconds) */

  /* DMA Variables */
  uint8_t dma_rx_buf[2];        /* Buffer for DMA to write raw gyro data to */
  volatile uint8_t dma_running; /* Flag to indicate DMA transfer is active */
  uint32_t last_dma_req_us;     /* Timestamp of last DMA request */
} MPU6050_Handle_t;

/* ========================================================================== */
/* PUBLIC FUNCTION PROTOTYPES                                                 */
/* ========================================================================== */

/**
 * @brief Initialize MPU6050 sensor
 * @param handle Pointer to MPU6050 handle
 * @return MPU6050_Status
 *
 * This function:
 * - Configures I2C communication
 * - Resets MPU6050
 * - Sets gyro range according to MPU6050_GYRO_RANGE
 * - Configures digital low-pass filter
 * - Initializes DMA for continuous reading
 */
MPU6050_Status MPU6050_Init(MPU6050_Handle_t *handle);

/**
 * @brief Calibrate gyroscope offset
 * @param handle Pointer to MPU6050 handle
 * @return MPU6050_Status
 *
 * IMPORTANT: Robot MUST be stationary during calibration!
 * Collects CALIBRATION_SAMPLES readings to calculate offset
 */
MPU6050_Status MPU6050_Calibrate(MPU6050_Handle_t *handle);

/**
 * @brief Update gyroscope readings and integrate yaw
 * @param handle Pointer to MPU6050 handle
 * @return MPU6050_Status
 *
 * Call this function as fast as possible (1kHz recommended)
 * Uses DMA for non-blocking reads
 */
MPU6050_Status MPU6050_Update(MPU6050_Handle_t *handle);

/**
 * @brief Get current yaw angle
 * @param handle Pointer to MPU6050 handle
 * @return Current yaw in degrees (accumulated, not normalized)
 */
float MPU6050_GetYaw(MPU6050_Handle_t *handle);

/**
 * @brief Get current gyro Z rate
 * @param handle Pointer to MPU6050 handle
 * @return Filtered angular velocity in deg/s
 */
float MPU6050_GetGyroZ(MPU6050_Handle_t *handle);

/**
 * @brief Reset yaw angle to zero
 * @param handle Pointer to MPU6050 handle
 */
void MPU6050_ResetYaw(MPU6050_Handle_t *handle);

/**
 * @brief Set yaw angle to specific value
 * @param handle Pointer to MPU6050 handle
 * @param angle Target angle in degrees
 */
void MPU6050_SetYaw(MPU6050_Handle_t *handle, float angle);

/**
 * @brief Check if MPU6050 is responding
 * @param handle Pointer to MPU6050 handle
 * @return MPU6050_Status
 */
MPU6050_Status MPU6050_IsReady(MPU6050_Handle_t *handle);

#endif /* MPU6050_H */
