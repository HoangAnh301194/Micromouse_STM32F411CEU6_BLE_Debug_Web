/* ========================================================================== */
/* mpu6050.c - MPU6050 Driver - BLOCKING MODE (NO DMA)                       */
/* Simple, reliable, fast enough for 1kHz update rate                        */
/* ========================================================================== */

#include "mpu6050.h"
#include "system_timer.h"
#include <math.h>
#include <string.h>
#include <stdio.h>

/* ========================================================================== */
/* MPU6050 REGISTER MAP                                                       */
/* ========================================================================== */

#define MPU6050_WHO_AM_I        0x75
#define MPU6050_PWR_MGMT_1      0x6B
#define MPU6050_PWR_MGMT_2      0x6C
#define MPU6050_GYRO_CONFIG     0x1B
#define MPU6050_ACCEL_CONFIG    0x1C
#define MPU6050_CONFIG          0x1A
#define MPU6050_SMPLRT_DIV      0x19
#define MPU6050_GYRO_ZOUT_H     0x47

/* ========================================================================== */
/* CONFIGURATION                                                              */
/* ========================================================================== */

/**
 * @brief Gyro Scale Lookup Table
 * Index corresponds to GYRO_CONFIG register FS_SEL bits
 */
static const float GYRO_SCALES[] = {
    131.0f,   /* Range 0: 250/s  */
    65.5f,    /* Range 1: 500/s  */
    32.8f,    /* Range 2: 1000/s */
    16.4f     /* Range 3: 2000/s */
};

/* ========================================================================== */
/* PRIVATE HELPER FUNCTIONS                                                   */
/* ========================================================================== */

/**
 * @brief Read Gyro Z register (blocking mode)
 * @param handle Pointer to MPU6050 handle
 * @return Raw gyro Z value (16-bit signed)
 */
static int16_t MPU6050_ReadGyroZ(MPU6050_Handle_t *handle) {
    uint8_t data[2];
    I2C_Status status;
    
    status = I2C_ReadRegisters(handle->i2c, handle->address, 
                               MPU6050_GYRO_ZOUT_H, data, 2);
    
    if (status == I2C_OK) {
        return (int16_t)((data[0] << 8) | data[1]);
    }
    
    return 0;
}

/* Global pointer for DMA callback access */
static MPU6050_Handle_t *mpu_global_handle = NULL;

/**
 * @brief Callback fired when I2C DMA RX finishes
 */
static void MPU6050_DMARxCallback(void) {
    if (mpu_global_handle) {
        mpu_global_handle->dma_running = 0; /* Clear busy flag */
    }
}

/**
 * @brief Low-pass filter implementation
 */
static float LowPassFilter(float new_val, float old_val, float alpha) {
    return alpha * new_val + (1.0f - alpha) * old_val;
}

/**
 * @brief Deadzone filter to reduce drift
 */
static float DeadzoneFilter(float value, float threshold) {
    if (fabsf(value) < threshold) {
        return 0.0f;
    }
    return value;
}

/* ========================================================================== */
/* PUBLIC FUNCTIONS                                                           */
/* ========================================================================== */

MPU6050_Status MPU6050_Init(MPU6050_Handle_t *handle) {
    uint8_t who_am_i, gyro_cfg;
    uint8_t i;
    char buf[128];
    const char *range_str;
    uint16_t max_rate;

    
    /* Initialize handle */
    handle->i2c = MPU6050_I2C_INSTANCE;
    handle->address = MPU6050_ADDR;
    
    /* [CRITICAL] Set gyro scale based on configured range */
    handle->gyro_scale = GYRO_SCALES[MPU6050_GYRO_RANGE];
    
    /* DMA Variables Initialization */
    handle->dma_running = 0;
    handle->last_dma_req_us = 0;
    mpu_global_handle = handle;
    
    /* Register DMA callback */
    I2C_SetCallback_DMA_RX(MPU6050_DMARxCallback);
    
    /* Initialize variables */
    handle->gyro_z_offset = 0.0f;
    handle->gyro_z_lpf = 0.0f;
    handle->gyro_z_avg_idx = 0;
    handle->gyro_z_filtered = 0.0f;
    handle->yaw = 0.0f;
    handle->last_update_us = 0;
    
    for (i = 0; i < GYRO_Z_AVG_SAMPLES; i++) {
        handle->gyro_z_avg_buf[i] = 0.0f;
    }
    
    /* Check device presence */
    if (I2C_IsDeviceReady(handle->i2c, handle->address) != I2C_OK) {

        return MPU6050_NOT_FOUND;
    }
    
    /* Verify WHO_AM_I register */
    if (I2C_ReadRegister(handle->i2c, handle->address, MPU6050_WHO_AM_I, &who_am_i) != I2C_OK) {
        
        return MPU6050_ERROR;
    }
    
    if (who_am_i != 0x68) {
        sprintf(buf, "ERROR: Wrong WHO_AM_I: 0x%02X (expected 0x68)\r\n", who_am_i);
        
        return MPU6050_ERROR;
    }
    
    sprintf(buf, "MPU6050 detected: WHO_AM_I = 0x%02X ?\r\n", who_am_i);

    /* Reset device */
    I2C_WriteRegister(handle->i2c, handle->address, MPU6050_PWR_MGMT_1, 0x80);
    delay_ms_blocking(100);
    
    /* Wake up and set clock source to PLL with Z-axis gyro */
    I2C_WriteRegister(handle->i2c, handle->address, MPU6050_PWR_MGMT_1, 0x03);
    delay_ms_blocking(50);
    
    /* [CRITICAL] Configure gyro range */
    I2C_WriteRegister(handle->i2c, handle->address, MPU6050_GYRO_CONFIG, 
                      MPU6050_GYRO_RANGE << 3);
    
    /* Read back to verify */
    I2C_ReadRegister(handle->i2c, handle->address, MPU6050_GYRO_CONFIG, &gyro_cfg);
    
    /* Print configuration */
    switch(MPU6050_GYRO_RANGE) {
        case 0: range_str = "250/s";  max_rate = 250;  break;
        case 1: range_str = "500/s";  max_rate = 500;  break;
        case 2: range_str = "1000/s"; max_rate = 1000; break;
        case 3: range_str = "2000/s"; max_rate = 2000; break;
        default: range_str = "INVALID"; max_rate = 0;    break;
    }

    sprintf(buf, "  Range:      %s\r\n", range_str);
    
    sprintf(buf, "  Max Rate:   %d deg/s\r\n", max_rate);
    
    sprintf(buf, "  Scale:      %.1f LSB/(deg/s)\r\n", handle->gyro_scale);
    
    sprintf(buf, "  Register:   0x%02X (expected: 0x%02X)\r\n", 
            gyro_cfg, (MPU6050_GYRO_RANGE << 3));

    /* Verify configuration */
    if (gyro_cfg != (MPU6050_GYRO_RANGE << 3)) {
        
    }
    
    if (fabsf(handle->gyro_scale - GYRO_SCALES[MPU6050_GYRO_RANGE]) > 0.1f) {

    }
    
    /* Configure Digital Low-Pass Filter (DLPF) */
    I2C_WriteRegister(handle->i2c, handle->address, MPU6050_CONFIG, 0x03);
    
    /* Set sample rate: 500Hz = 1kHz / (1+1) */
    I2C_WriteRegister(handle->i2c, handle->address, MPU6050_SMPLRT_DIV, 1);

    sprintf(buf, "  DLPF:       44Hz bandwidth\r\n");
    
    sprintf(buf, "  Sample Rate: 500Hz (internal)\r\n");
    
    sprintf(buf, "  Update Rate: 1000Hz (software)\r\n");
    
    sprintf(buf, "  LPF Alpha:  %.2f\r\n", GYRO_Z_LPF_ALPHA);
    
    sprintf(buf, "  Deadzone:   %.2f deg/s\r\n", GYRO_Z_DEADZONE);
    
    sprintf(buf, "  Avg Samples: %d\r\n", GYRO_Z_AVG_SAMPLES);

    /* Initialize timestamp */
    handle->last_update_us = micros();

    

    return MPU6050_OK;
}

MPU6050_Status MPU6050_Calibrate(MPU6050_Handle_t *handle) {
    int32_t sum = 0;
    uint16_t i;
    uint8_t j;
    char buf[64];
    int16_t gz_raw;
    uint16_t zero_count = 0;
    uint16_t consecutive_zeros = 0;
	uint32_t start_time;
     uint32_t elapsed;

    sprintf(buf, "Collecting %d samples...\r\n", CALIBRATION_SAMPLES);

    /* [FIX] Force I2C reset to clear any stuck state */
    
    I2C_Init(handle->i2c, 1);
    delay_ms_blocking(50);
    
    /* Verify I2C is working */
    if (I2C_IsDeviceReady(handle->i2c, handle->address) != I2C_OK) {

        return MPU6050_ERROR;
    }

    /* Test read first */
    
    gz_raw = MPU6050_ReadGyroZ(handle);
    sprintf(buf, "  Test read result: %d\r\n", gz_raw);

    if (gz_raw == 0) {
        /* Try one more time */
        delay_ms_blocking(10);
        gz_raw = MPU6050_ReadGyroZ(handle);
        sprintf(buf, "  Retry read result: %d\r\n", gz_raw);

        if (gz_raw == 0) {

            return MPU6050_ERROR;
        }
    }

    
    start_time = millis();
    
    /* Collect samples - NO DELAY for speed */
    for (i = 0; i < CALIBRATION_SAMPLES; i++) {
        gz_raw = MPU6050_ReadGyroZ(handle);
        
        /* Check for stuck readings */
        if (gz_raw == 0) {
            zero_count++;
            consecutive_zeros++;
            
            if (consecutive_zeros > 10) {

                sprintf(buf, "       Stopped at sample %d/%d\r\n", i, CALIBRATION_SAMPLES);
                
                return MPU6050_ERROR;
            }
        } else {
            consecutive_zeros = 0;
        }
        
        sum += gz_raw;
        
        /* Print progress every 100 samples */
        if (i % 100 == 0) {
            sprintf(buf, "  Progress: %d/%d (raw: %d)\r\n", 
                    i, CALIBRATION_SAMPLES, gz_raw);
            
        }
    }
    
    elapsed = millis() - start_time;
    sprintf(buf, "Calibration took %lu ms\r\n", elapsed);

    /* Calculate offset */
    handle->gyro_z_offset = (float)sum / CALIBRATION_SAMPLES / handle->gyro_scale;
    
    /* Reset filters */
    handle->gyro_z_lpf = 0.0f;
    handle->gyro_z_filtered = 0.0f;
    handle->gyro_z_avg_idx = 0;
    handle->yaw = 0.0f;
    
    for (j = 0; j < GYRO_Z_AVG_SAMPLES; j++) {
        handle->gyro_z_avg_buf[j] = 0.0f;
    }
    
    /* Reset timestamp */
    handle->last_update_us = micros();
    
    /* Print results */
    sprintf(buf, "\r\n? Calibration complete!\r\n");
    
    sprintf(buf, "  Raw offset:  %ld LSB\r\n", sum / CALIBRATION_SAMPLES);
    
    sprintf(buf, "  Gyro offset: %.3f deg/s\r\n", handle->gyro_z_offset);
    
    sprintf(buf, "  Zero count:  %d/%d\r\n", zero_count, CALIBRATION_SAMPLES);

    /* Validate offset */
    if (fabsf(handle->gyro_z_offset) > 5.0f) {

        
    } else {
        
    }

    
    return MPU6050_OK;
}

MPU6050_Status MPU6050_Update(MPU6050_Handle_t *handle) {
    int16_t gz_raw;
    float gz_dps, gz_corrected, gz_lpf, gz_final;
    uint32_t current_us;
    
    /* Calculate dt FIRST - skip if called too fast */
    float dt_dma;
    
    /* Check if DMA is currently running */
    if (handle->dma_running) {
        current_us = micros();
        /* Timeout protection (e.g., 5ms maximum) */
        if ((current_us - handle->last_dma_req_us) > 5000) {
            handle->dma_running = 0; /* Force clear if stuck */
            /* Re-initialize I2C to recover */
            I2C_Init(handle->i2c, 1);
            I2C_DMA_Init(handle->i2c);
        }
        return MPU6050_OK; /* Wait for next loop */
    }
    
    /* Previous DMA finished! Calculate dt for the data that just arrived */
    current_us = micros();
    dt_dma = (float)(current_us - handle->last_update_us) / 1000000.0f;
    
    /* Process the data from the buffer (only if valid dt) */
    if (dt_dma >= 0.0005f) {
        /* Clamp max dt to avoid integration jumps */
        if (dt_dma > 0.1f) dt_dma = 0.1f;
        
        handle->last_update_us = current_us;
        
        /* Construct 16-bit integer from DMA buffer */
        gz_raw = (int16_t)((handle->dma_rx_buf[0] << 8) | handle->dma_rx_buf[1]);
        
        /* Convert to deg/s */
        gz_dps = (float)gz_raw / handle->gyro_scale;
        
        /* Apply calibration offset */
        gz_corrected = gz_dps - handle->gyro_z_offset;
        
        /* Low-pass filter */
        gz_lpf = LowPassFilter(gz_corrected, handle->gyro_z_lpf, GYRO_Z_LPF_ALPHA);
        handle->gyro_z_lpf = gz_lpf;
        
        /* Deadzone filter */
        gz_final = DeadzoneFilter(gz_lpf, GYRO_Z_DEADZONE);
        handle->gyro_z_filtered = gz_final;
        
        /* Integrate to get yaw */
        handle->yaw += gz_final * dt_dma;
    }
    
    /* Start next DMA requested! (Non-blocking) */
    /* [CRITICAL FIX] Check if I2C bus is busy (e.g. OLED blocking transfer)
     * If busy, skip this cycle to avoid deadlock in ISR context.
     * TIM11 ISR (priority 0) preempts everything - if we block here
     * waiting for bus free, the main loop can never finish its I2C
     * transfer, causing a permanent deadlock. */
    if ((I2C1->SR2 & I2C_SR2_BUSY) || I2C_IsBusy_DMA(I2C_1)) {
        /* Bus busy - skip this reading, try next 1ms cycle */
        return MPU6050_OK;
    }
    
    handle->dma_running = 1;
    handle->last_dma_req_us = micros();
    I2C_ReadReg_DMA(I2C1, handle->address, MPU6050_GYRO_ZOUT_H, handle->dma_rx_buf, 2);
    
    return MPU6050_OK;
}

float MPU6050_GetYaw(MPU6050_Handle_t *handle) {
    return handle->yaw;
}

float MPU6050_GetGyroZ(MPU6050_Handle_t *handle) {
    return handle->gyro_z_filtered;
}

void MPU6050_ResetYaw(MPU6050_Handle_t *handle) {
    handle->yaw = 0.0f;
}

void MPU6050_SetYaw(MPU6050_Handle_t *handle, float angle) {
    handle->yaw = angle;
}

MPU6050_Status MPU6050_IsReady(MPU6050_Handle_t *handle) {
    if (I2C_IsDeviceReady(handle->i2c, handle->address) == I2C_OK) {
        return MPU6050_OK;
    }
    return MPU6050_NOT_FOUND;
}
