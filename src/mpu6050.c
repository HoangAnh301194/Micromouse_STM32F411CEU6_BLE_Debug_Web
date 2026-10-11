/* MPU6050 gyro Z diagnostic driver, STM32F411 I2C1.
 * The 2-byte I2C read runs in event/error IRQs. No I2C waits or recovery
 * from TIM11; main polls for completion and starts the next transaction.
 */
#include "mpu6050.h"
#include "system_timer.h"
#include <math.h>

#define MPU6050_WHO_AM_I      0x75U
#define MPU6050_PWR_MGMT_1    0x6BU
#define MPU6050_GYRO_CONFIG   0x1BU
#define MPU6050_CONFIG        0x1AU
#define MPU6050_SMPLRT_DIV    0x19U
#define MPU6050_GYRO_ZOUT_H   0x47U

static const float gyro_scales[4] = {131.0f, 65.5f, 32.8f, 16.4f};

static void MPU6050_ClearNoiseWindow(MPU6050_Handle_t *h)
{
    h->raw_sum = 0;
    h->raw_sum_squares = 0U;
    h->raw_stat_n = 0U;
    h->raw_stat_min = 32767;
    h->raw_stat_max = -32768;
    h->raw_has_previous = 0U;
    h->raw_equal_count = 0U;
    h->last_raw_timestamp_us = 0U;
    h->raw_interval_min_us = 0xFFFFFFFFUL;
    h->raw_interval_max_us = 0U;
}

static void MPU6050_TrackRaw(MPU6050_Handle_t *h, int16_t raw, uint32_t time_us)
{
    uint32_t spacing;
    if (h->raw_stat_n == 0U) {
        h->raw_stat_min = raw;
        h->raw_stat_max = raw;
    } else {
        if (raw < h->raw_stat_min) h->raw_stat_min = raw;
        if (raw > h->raw_stat_max) h->raw_stat_max = raw;
    }
    if (h->raw_has_previous) {
        if (raw == h->last_raw) h->raw_equal_count++;
        spacing = time_us - h->last_raw_timestamp_us;
        if (spacing < h->raw_interval_min_us) h->raw_interval_min_us = spacing;
        if (spacing > h->raw_interval_max_us) h->raw_interval_max_us = spacing;
    }
    h->last_raw = raw;
    h->last_raw_timestamp_us = time_us;
    h->raw_has_previous = 1U;
    h->raw_sum += raw;
    h->raw_sum_squares += (uint64_t)((int32_t)raw * (int32_t)raw);
    h->raw_stat_n++;
}

void MPU6050_GetNoiseWindow(MPU6050_Handle_t *h, MPU6050_NoiseStats_t *out)
{
    double mean_counts, var_counts;
    if (!h || !out) return;
    out->n = h->raw_stat_n;
    out->equal_consecutive_raw = h->raw_equal_count;
    out->interval_min_us = (h->raw_stat_n > 1U) ? h->raw_interval_min_us : 0U;
    out->interval_max_us = h->raw_interval_max_us;
    if (h->raw_stat_n > 0U) {
        mean_counts = (double)h->raw_sum / (double)h->raw_stat_n;
        var_counts = ((double)h->raw_sum_squares / (double)h->raw_stat_n) -
                     mean_counts * mean_counts;
        if (var_counts < 0.0) var_counts = 0.0;
        out->mean_dps = (float)(mean_counts / h->gyro_scale);
        out->std_dps = (float)(sqrt(var_counts) / h->gyro_scale);
        out->min_dps = (float)h->raw_stat_min / h->gyro_scale;
        out->max_dps = (float)h->raw_stat_max / h->gyro_scale;
        out->mean_corrected_dps = out->mean_dps - h->gyro_z_offset;
    } else {
        out->mean_dps = out->std_dps = out->min_dps =
        out->max_dps = out->mean_corrected_dps = 0.0f;
    }
    MPU6050_ClearNoiseWindow(h);
}


MPU6050_Status MPU6050_Init(MPU6050_Handle_t *h)
{
    uint8_t who = 0U, cfg = 0U;
    if (!h || MPU6050_GYRO_RANGE > 3U) return MPU6050_ERROR;
    h->i2c = MPU6050_I2C_INSTANCE;
    h->address = MPU6050_ADDR;
    h->gyro_scale = gyro_scales[MPU6050_GYRO_RANGE];
    h->gyro_z_offset = 0.0f;
    h->gyro_z_lpf = 0.0f;
    h->gyro_z_filtered = 0.0f;
    h->gyro_z_avg_idx = 0U;
    h->yaw = 0.0f;
    h->last_update_us = 0U;
    h->last_request_us = 0U;
    h->last_good_us = 0U;
    h->last_transfer_us = 0U;
    h->max_transfer_us = 0U;
    h->good_reads = 0U;
    h->error_reads = 0U;
    h->timeout_reads = 0U;
    h->busy_retries = 0U;
    h->last_i2c_error = I2C_OK;
    h->calibration_sum = 0;
    h->calibration_count = 0U;
    h->calibrating = 0U;
    h->calibrated = 0U;
    h->dma_running = 0U;
    h->last_dma_req_us = 0U;
    MPU6050_ClearNoiseWindow(h);

    I2C_Init(h->i2c, 1U);
    if (I2C_IsDeviceReady(h->i2c, h->address) != I2C_OK) return MPU6050_NOT_FOUND;
    if (I2C_ReadRegister(h->i2c, h->address, MPU6050_WHO_AM_I, &who) != I2C_OK ||
        who != 0x68U) return MPU6050_NOT_FOUND;

    if (I2C_WriteRegister(h->i2c, h->address, MPU6050_PWR_MGMT_1, 0x80U) != I2C_OK)
        return MPU6050_ERROR;
    delay_ms_blocking(100U);
    /* Gyro Z PLL clock source; wake MPU6050. */
    if (I2C_WriteRegister(h->i2c, h->address, MPU6050_PWR_MGMT_1, 0x03U) != I2C_OK)
        return MPU6050_ERROR;
    delay_ms_blocking(50U);
    if (I2C_WriteRegister(h->i2c, h->address, MPU6050_GYRO_CONFIG,
                          (uint8_t)(MPU6050_GYRO_RANGE << 3)) != I2C_OK)
        return MPU6050_ERROR;
    if (I2C_WriteRegister(h->i2c, h->address, MPU6050_CONFIG, 0x03U) != I2C_OK)
        return MPU6050_ERROR;
    /* With DLPF enabled the gyro output rate is 1kHz at DIV=0.
     * Deliberately retain the original gyro bandwidth 42 Hz for this test. */
    if (I2C_WriteRegister(h->i2c, h->address, MPU6050_SMPLRT_DIV, 0U) != I2C_OK)
        return MPU6050_ERROR;
    if (I2C_ReadRegister(h->i2c, h->address, MPU6050_GYRO_CONFIG, &cfg) != I2C_OK ||
        cfg != (uint8_t)(MPU6050_GYRO_RANGE << 3))
        return MPU6050_ERROR;

    /* Schedule first request from main without blocking. */
    h->last_request_us = micros() - MPU_SAMPLE_PERIOD_US;
    return MPU6050_OK;
}

MPU6050_Status MPU6050_Calibrate(MPU6050_Handle_t *h)
{
    /* This call STARTS asynchronous calibration, does not busy-wait. */
    if (!h) return MPU6050_ERROR;
    h->calibration_sum = 0;
    h->calibration_count = 0U;
    h->calibrated = 0U;
    h->calibrating = 1U;
    h->gyro_z_offset = 0.0f;
    h->gyro_z_filtered = 0.0f;
    h->gyro_z_lpf = 0.0f;
    h->yaw = 0.0f;
    h->last_update_us = 0U;
    MPU6050_ClearNoiseWindow(h);
    return MPU6050_OK;
}

MPU6050_Status MPU6050_Update(MPU6050_Handle_t *h)
{
    uint8_t bytes[2];
    uint32_t duration = 0U;
    uint32_t now;
    int16_t raw;
    float gz, dt;
    I2C_Status result, start_result;
    if (!h) return MPU6050_ERROR;

    /* Poll never waits: returns BUSY, IDLE, or one terminal result. */
    result = I2C_Read2Async_Poll(bytes, &duration);
    now = micros();
    if (result == I2C_OK) {
        h->good_reads++;
        h->last_good_us = now;
        h->last_transfer_us = duration;
        if (duration > h->max_transfer_us) h->max_transfer_us = duration;
        raw = (int16_t)(((uint16_t)bytes[0] << 8) | (uint16_t)bytes[1]);
        /* Collect raw data BEFORE LPF/deadzone. Calibration records are
         * excluded from this post-calibration noise window. */
        if (!h->calibrating) MPU6050_TrackRaw(h, raw, now);
        if (h->calibrating) {
            h->calibration_sum += (int32_t)raw;
            h->calibration_count++;
            if (h->calibration_count >= CALIBRATION_SAMPLES) {
                h->gyro_z_offset = (float)h->calibration_sum /
                                    ((float)h->calibration_count * h->gyro_scale);
                h->calibrating = 0U;
                h->calibrated = 1U;
                h->gyro_z_lpf = 0.0f;
                h->gyro_z_filtered = 0.0f;
                h->last_update_us = now;
                MPU6050_ClearNoiseWindow(h);
            }
        } else if (h->calibrated) {
            gz = ((float)raw / h->gyro_scale) - h->gyro_z_offset;
            h->gyro_z_lpf += GYRO_Z_LPF_ALPHA * (gz - h->gyro_z_lpf);
            gz = (fabsf(h->gyro_z_lpf) < GYRO_Z_DEADZONE) ?
                 0.0f : h->gyro_z_lpf;
            h->gyro_z_filtered = gz;
            if (h->last_update_us != 0U) {
                dt = (float)(now - h->last_update_us) / 1000000.0f;
                /* Do not integrate a large gap caused by an I2C outage. */
                if (dt > 0.0f && dt <= 0.020f) h->yaw += gz * dt;
            }
            h->last_update_us = now;
        }
    } else if (result != I2C_BUSY && result != I2C_IDLE) {
        h->error_reads++;
        if (result == I2C_TIMEOUT) h->timeout_reads++;
        h->last_i2c_error = result;
    }

    /* Request periodic gyro Z reads from main, never from TIM11.
     * A missed scheduling opportunity is counted, not busy-waited. */
    if ((uint32_t)(now - h->last_request_us) >= MPU_SAMPLE_PERIOD_US &&
        result != I2C_BUSY) {
        start_result = I2C_Read2Async_Start(h->address, MPU6050_GYRO_ZOUT_H);
        h->last_request_us = now;
        if (start_result == I2C_BUSY) h->busy_retries++;
        else if (start_result != I2C_OK) {
            h->error_reads++;
            h->last_i2c_error = start_result;
        }
    }
    return MPU6050_OK;
}

float MPU6050_GetYaw(MPU6050_Handle_t *h) { return h ? h->yaw : 0.0f; }
float MPU6050_GetGyroZ(MPU6050_Handle_t *h) { return h ? h->gyro_z_filtered : 0.0f; }
void MPU6050_ResetYaw(MPU6050_Handle_t *h) { if (h) h->yaw = 0.0f; }
void MPU6050_SetYaw(MPU6050_Handle_t *h, float angle) { if (h) h->yaw = angle; }

MPU6050_Status MPU6050_IsReady(MPU6050_Handle_t *h)
{
    if (!h) return MPU6050_ERROR;
    /* Blocking probe for startup/diagnostic ONLY, not the 1kHz path. */
    return (I2C_IsDeviceReady(h->i2c, h->address) == I2C_OK) ?
           MPU6050_OK : MPU6050_NOT_FOUND;
}
