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
#define MPU6050_INT_STATUS    0x3AU
#define MPU6050_INT_ENABLE    0x38U
#define MPU6050_INT_PIN_CFG   0x37U

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


/* Calibration checks only the gyro Z stream. They detect obvious motion,
 * but cannot distinguish a perfectly constant slow turn from sensor bias.
 * Therefore all calibrations must be performed on a physically fixed robot. */
static void MPU6050_ResetCalSamples(MPU6050_Handle_t *h)
{
    h->calibration_sum = 0;
    h->calibration_count = 0U;
    h->cal_block_samples = 0U;
    h->cal_block_sum = 0;
    h->cal_block_sum_sq = 0U;
    h->cal_block_min = 32767;
    h->cal_block_max = -32768;
    h->cal_mean_min_dps = 0.0f;
    h->cal_mean_max_dps = 0.0f;
    h->cal_last_std_dps = 0.0f;
    h->cal_max_std_dps = 0.0f;
    h->cal_last_span_dps = 0.0f;
    h->cal_last_mean_span_dps = 0.0f;
    h->cal_valid_blocks = 0U;
}

static void MPU6050_CalibrationRetry(MPU6050_Handle_t *h, uint8_t reason,
                                      uint32_t now_ms)
{
    h->cal_failure_reason = reason;
    h->cal_retries++;
    if (h->cal_retries >= MPU_CAL_MAX_RETRIES) {
        h->cal_state = 4U; /* failed: require a deliberate KEY retry */
        h->calibrating = 0U;
        h->calibrated = 0U;
        return;
    }
    h->cal_state = 1U; /* discard suspect samples and warm up again */
    h->cal_started_ms = now_ms;
    MPU6050_ResetCalSamples(h);
}

static void MPU6050_AccumulateCalibration(MPU6050_Handle_t *h,
                                          int16_t raw, uint32_t now_ms)
{
    double mean_counts, var_counts;
    float mean_dps, std_dps, span_dps;
    if (h->cal_state == 1U) {
        if ((uint32_t)(now_ms - h->cal_started_ms) < MPU_CAL_WARMUP_MS)
            return;
        h->cal_state = 2U;
        MPU6050_ResetCalSamples(h);
    }
    if (h->cal_state != 2U) return;

    h->calibration_sum += raw;
    h->calibration_count++;
    h->cal_block_sum += raw;
    h->cal_block_sum_sq += (uint64_t)((int32_t)raw * (int32_t)raw);
    h->cal_block_samples++;
    if (raw < h->cal_block_min) h->cal_block_min = raw;
    if (raw > h->cal_block_max) h->cal_block_max = raw;
    if (h->cal_block_samples != MPU_CAL_BLOCK_SAMPLES) return;

    mean_counts = (double)h->cal_block_sum / MPU_CAL_BLOCK_SAMPLES;
    var_counts = ((double)h->cal_block_sum_sq / MPU_CAL_BLOCK_SAMPLES)
                 - mean_counts * mean_counts;
    if (var_counts < 0.0) var_counts = 0.0;
    mean_dps = (float)(mean_counts / h->gyro_scale);
    std_dps = (float)(sqrt(var_counts) / h->gyro_scale);
    span_dps = (float)((int32_t)h->cal_block_max -
                       (int32_t)h->cal_block_min) / h->gyro_scale;
    h->cal_last_std_dps = std_dps;
    h->cal_last_span_dps = span_dps;
    if (std_dps > h->cal_max_std_dps) h->cal_max_std_dps = std_dps;
    if (std_dps > MPU_CAL_BLOCK_MAX_STD_DPS ||
        span_dps > MPU_CAL_BLOCK_MAX_SPAN_DPS) {
        MPU6050_CalibrationRetry(h, 1U, now_ms);
        return;
    }

    if (h->cal_valid_blocks == 0U) {
        h->cal_mean_min_dps = mean_dps;
        h->cal_mean_max_dps = mean_dps;
    } else {
        if (mean_dps < h->cal_mean_min_dps) h->cal_mean_min_dps = mean_dps;
        if (mean_dps > h->cal_mean_max_dps) h->cal_mean_max_dps = mean_dps;
    }
    h->cal_last_mean_span_dps = h->cal_mean_max_dps -
                                h->cal_mean_min_dps;
    if (h->cal_last_mean_span_dps > MPU_CAL_BLOCK_MEAN_SPAN_DPS) {
        MPU6050_CalibrationRetry(h, 2U, now_ms);
        return;
    }
    h->cal_valid_blocks++;
    h->cal_block_samples = 0U;
    h->cal_block_sum = 0;
    h->cal_block_sum_sq = 0U;
    h->cal_block_min = 32767;
    h->cal_block_max = -32768;

    if (h->calibration_count >= CALIBRATION_SAMPLES) {
        h->gyro_z_offset = (float)h->calibration_sum /
            ((float)h->calibration_count * h->gyro_scale);
        h->calibrating = 0U;
        h->calibrated = 1U;
        h->cal_state = 3U;
#if defined(MPU_FRESHNESS_DIAG)
        h->diag_control_ready = 1U;
#endif
        h->cal_completed_ms = now_ms;
        h->gyro_z_lpf = 0.0f;
        h->gyro_z_filtered = 0.0f;
        h->gyro_z_raw_corrected = 0.0f;
        h->yaw = 0.0f;
        h->yaw_no_deadzone = 0.0f;
        h->yaw_raw = 0.0f;
        h->last_update_us = 0U;
        MPU6050_ClearNoiseWindow(h);
    }
}

MPU6050_Status MPU6050_Init(MPU6050_Handle_t *h)
{
    uint8_t who = 0U, cfg = 0U;
    uint8_t i, found = 0U;
    static const uint8_t speed_modes[4] = {1U, 2U, 3U, 0U};
    if (!h || MPU6050_GYRO_RANGE > 3U) return MPU6050_ERROR;
    h->i2c = MPU6050_I2C_INSTANCE;
    h->address = MPU6050_ADDR;
    h->gyro_scale = gyro_scales[MPU6050_GYRO_RANGE];
    h->gyro_z_offset = 0.0f;
    h->gyro_z_lpf = 0.0f;
    h->gyro_z_filtered = 0.0f;
    h->gyro_z_avg_idx = 0U;
    h->yaw = 0.0f;
    h->yaw_no_deadzone = 0.0f;
    h->yaw_raw = 0.0f;
    h->gyro_z_raw_corrected = 0.0f;
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
    h->cal_state = 0U;
    h->cal_retries = 0U;
    h->cal_failure_reason = 0U;
    h->cal_started_ms = 0U;
    h->cal_completed_ms = 0U;
    h->cal_previous_bias_dps = 0.0f;
    MPU6050_ResetCalSamples(h);
    h->dma_running = 0U;
    h->last_dma_req_us = 0U;
    h->sample_phase = 0U;
    h->last_status_ready = 0U;
    h->status_reads = 0U;
    h->status_ready_reads = 0U;
    h->status_not_ready_reads = 0U;
    h->fresh_gyro_reads = 0U;
    h->stale_gyro_reads = 0U;
    h->data_read_start_us = 0U;
    h->dlpf_cfg = MPU6050_DLPF_CFG;
#if defined(MPU_FRESHNESS_DIAG)
    h->diag_control_ready = 0U;
    h->diag_fresh_sequence = 0U;
    h->diag_last_fresh_us = 0U;
    h->diag_previous_fresh_us = 0U;
    h->diag_fresh_gap_min_us = 0xFFFFFFFFUL;
    h->diag_fresh_gap_max_us = 0U;
    h->diag_fresh_gap_gt1500 = 0U;
    h->diag_fresh_gap_gt2000 = 0U;
    h->diag_previous_status_us = 0U;
    h->diag_status_gap_max_us = 0U;
    h->diag_notready_run = 0U;
    h->diag_notready_run_max = 0U;
    h->diag_ab_initial_notready = 0U;
    h->diag_ab_gyro_started = 0U;
    h->diag_ab_gyro_skipped = 0U;
    h->diag_ab_retry_started = 0U;
    h->diag_ab_retry_ready = 0U;
    h->diag_ab_retry_notready = 0U;
    h->diag_ab_retry_expired = 0U;
    h->diag_ab_retry_busy = 0U;
#endif
#if defined(MPU_SAMPLING_POLICY_B)
    h->retry_count = 0U;
    h->retry_due_us = 0U;
#endif
    h->published_yaw_deg = 0.0f;
    h->published_gyro_z_dps = 0.0f;
    h->published_completed_us = 0U;
    h->published_sequence = 0U;
    h->published_valid = 0U;
#if defined(MPU_TIM11_TICK_SCHED)
    h->control_sample_tick_seq = 0U;
    h->control_sample_tick_us = 0U;
    h->control_last_started_tick_seq = 0U;
#endif
#if defined(MPU_PHASE_AB)
    h->phase_prev_request_us = 0U;
    h->phase_last_request_us = 0U;
    h->phase_request_count = 0U;
    h->phase_request_intervals = 0U;
    h->phase_request_min_us = 0xFFFFFFFFUL;
    h->phase_request_max_us = 0U;
    h->phase_request_gt1050 = 0U;
    h->phase_request_gt1500 = 0U;
    h->phase_request_interval_sum_us = 0ULL;
    h->phase_processed_count = 0U;
    h->phase_irq_to_main_max_us = 0U;
    h->phase_irq_to_publish_max_us = 0U;
    h->phase_main_to_publish_max_us = 0U;
    h->phase_request_to_irq_max_us = 0U;
    h->phase_irq_to_main_sum_us = 0ULL;
    h->phase_irq_to_publish_sum_us = 0ULL;
    h->phase_main_to_publish_sum_us = 0ULL;
    h->phase_request_to_irq_sum_us = 0ULL;
    h->phase_last_sample_request_us = 0U;
    h->phase_last_sample_irq_us = 0U;
    h->phase_last_sample_main_us = 0U;
    h->phase_last_sample_publish_us = 0U;
    h->phase_published_irq_sequence = 0U;
#endif
    MPU6050_ClearNoiseWindow(h);

    h->init_step = 0U;
    h->init_register = 0U;
    h->init_expected = 0U;
    h->init_read_value = 0U;
    h->init_i2c_status = I2C_OK;
    h->selected_speed_mode = 0xFFU;
    h->selected_speed_index = 0xFFU;
    for (i = 0U; i < 4U; i++) {
        h->probe_address_status[i] = 0xFFU;
        h->probe_who_status[i] = 0xFFU;
        h->probe_who_value[i] = 0xFFU;
    }

    /* Keep the exact failing step/register/status for USART diagnostics.
     * 1=address, 2=WHO_AM_I, 3=reset, 4=wake, 5=gyro range,
     * 6=DLPF, 7=sample divider, 8=gyro range verify,
     * 9=DLPF verify, 10=INT pin config, 11=INT enable. */
#define MPU_BOOT_WRITE(step_, reg_, value_) do {                          \
        h->init_step = (step_);                                         \
        h->init_register = (reg_);                                     \
        h->init_expected = (value_);                                   \
        h->init_i2c_status = I2C_WriteRegister(h->i2c, h->address,      \
                                                (reg_), (value_));     \
        if (h->init_i2c_status != I2C_OK) return MPU6050_ERROR;         \
    } while (0)

    /* Probe descending bus rates without lowering STM32 CPU clock:
     * 400k, 300k, 200k, 100k. Accept only an ACK AND correct WHO_AM_I.
     * Failed trials are logged; selected mode is retained on bus recovery. */
    for (i = 0U; i < 4U; i++) {
        I2C_Init(h->i2c, speed_modes[i]);
        delay_ms_blocking(2U);
        h->init_step = 1U;
        h->init_register = 0U;
        h->init_i2c_status = I2C_IsDeviceReady(h->i2c, h->address);
        h->probe_address_status[i] = (uint8_t)h->init_i2c_status;
        if (h->init_i2c_status != I2C_OK) continue;

        h->init_step = 2U;
        h->init_register = MPU6050_WHO_AM_I;
        h->init_expected = 0x68U;
        who = 0U;
        h->init_i2c_status = I2C_ReadRegister(h->i2c, h->address,
                                              MPU6050_WHO_AM_I, &who);
        h->probe_who_status[i] = (uint8_t)h->init_i2c_status;
        h->probe_who_value[i] = who;
        h->init_read_value = who;
        if (h->init_i2c_status == I2C_OK && who == 0x68U) {
            h->selected_speed_mode = speed_modes[i];
            h->selected_speed_index = i;
            found = 1U;
            break;
        }
    }
    if (!found) return MPU6050_NOT_FOUND;

    MPU_BOOT_WRITE(3U, MPU6050_PWR_MGMT_1, 0x80U);
    delay_ms_blocking(100U);
    MPU_BOOT_WRITE(4U, MPU6050_PWR_MGMT_1, 0x03U);
    delay_ms_blocking(50U);
    MPU_BOOT_WRITE(5U, MPU6050_GYRO_CONFIG,
                   (uint8_t)(MPU6050_GYRO_RANGE << 3));
    MPU_BOOT_WRITE(6U, MPU6050_CONFIG, MPU6050_DLPF_CFG);
    MPU_BOOT_WRITE(7U, MPU6050_SMPLRT_DIV, 0U);

    h->init_step = 8U;
    h->init_register = MPU6050_GYRO_CONFIG;
    h->init_expected = (uint8_t)(MPU6050_GYRO_RANGE << 3);
    h->init_i2c_status = I2C_ReadRegister(h->i2c, h->address,
                                          MPU6050_GYRO_CONFIG, &cfg);
    h->init_read_value = cfg;
    if (h->init_i2c_status != I2C_OK || cfg != h->init_expected)
        return MPU6050_ERROR;

    h->init_step = 9U;
    h->init_register = MPU6050_CONFIG;
    h->init_expected = MPU6050_DLPF_CFG;
    h->init_i2c_status = I2C_ReadRegister(h->i2c, h->address,
                                          MPU6050_CONFIG, &cfg);
    h->init_read_value = cfg;
    if (h->init_i2c_status != I2C_OK ||
        (cfg & 0x07U) != MPU6050_DLPF_CFG)
        return MPU6050_ERROR;

    /* Enable DATA_RDY status: no MPU INT wire required on the board. */
    MPU_BOOT_WRITE(10U, MPU6050_INT_PIN_CFG, 0x00U);
    MPU_BOOT_WRITE(11U, MPU6050_INT_ENABLE, 0x01U);
#undef MPU_BOOT_WRITE
    h->init_step = 0U; /* All MPU startup steps passed. */

    /* Schedule first request from main without blocking. */
    h->last_request_us = micros() - MPU_SAMPLE_PERIOD_US;
    return MPU6050_OK;
}

MPU6050_Status MPU6050_Calibrate(MPU6050_Handle_t *h)
{
    /* Nonblocking startup / deliberate user recalibration.
     * Keep previous bias for post-calibration comparison; no bias update
     * occurs during motion because cal=0 disables yaw integration. */
    if (!h) return MPU6050_ERROR;
    /* Invalidate the old publish BEFORE touching calibration, yaw or
     * bias fields: TIM11 must not consume a pre-recalibration snapshot. */
    {
        uint32_t primask = __get_PRIMASK();
        __disable_irq();
        h->published_valid = 0U;
        if (!primask) __enable_irq();
    }
    h->cal_previous_bias_dps = h->gyro_z_offset;
    h->cal_retries = 0U;
    h->cal_failure_reason = 0U;
    h->cal_started_ms = millis();
    h->cal_completed_ms = 0U;
    h->cal_state = 1U;
    h->calibrating = 1U;
    h->calibrated = 0U;
#if defined(MPU_FRESHNESS_DIAG)
    h->diag_control_ready = 0U;
#endif
    MPU6050_ResetCalSamples(h);
#if defined(MPU_SAMPLING_POLICY_B)
    /* A delayed status retry from the previous run must not survive
     * explicit recalibration. Do not cancel an in-flight IRQ transfer. */
    if (h->sample_phase == 4U) {
        SystemTimer_CancelWake();
        h->sample_phase = 0U;
    }
    h->retry_count = 0U;
    h->retry_due_us = 0U;
#endif
    h->gyro_z_offset = 0.0f;
    h->gyro_z_filtered = 0.0f;
    h->gyro_z_lpf = 0.0f;
    h->gyro_z_raw_corrected = 0.0f;
    h->yaw = 0.0f;
    h->yaw_no_deadzone = 0.0f;
    h->yaw_raw = 0.0f;
    h->last_update_us = 0U;
#if defined(MPU_PHASE_AB)
    /* New calibration resets experiment window; do not change I2C policy. */
    h->phase_prev_request_us = 0U;
    h->phase_last_request_us = 0U;
    h->phase_request_count = 0U;
    h->phase_request_intervals = 0U;
    h->phase_request_min_us = 0xFFFFFFFFUL;
    h->phase_request_max_us = 0U;
    h->phase_request_gt1050 = 0U;
    h->phase_request_gt1500 = 0U;
    h->phase_request_interval_sum_us = 0ULL;
    h->phase_processed_count = 0U;
    h->phase_irq_to_main_max_us = 0U;
    h->phase_irq_to_publish_max_us = 0U;
    h->phase_main_to_publish_max_us = 0U;
    h->phase_request_to_irq_max_us = 0U;
    h->phase_irq_to_main_sum_us = 0ULL;
    h->phase_irq_to_publish_sum_us = 0ULL;
    h->phase_main_to_publish_sum_us = 0ULL;
    h->phase_request_to_irq_sum_us = 0ULL;
    h->phase_last_sample_request_us = 0U;
    h->phase_last_sample_irq_us = 0U;
    h->phase_last_sample_main_us = 0U;
    h->phase_last_sample_publish_us = 0U;
    h->phase_published_irq_sequence = 0U;
#endif
    MPU6050_ClearNoiseWindow(h);
    return MPU6050_OK;
}

/* nonblocking acquisition in main, two transactions per 1ms target.
 * INT_STATUS(1 byte) -> GYRO_ZOUT_H/L(2 bytes) using I2C EV/ER IRQs.
 * A repeated raw code is not itself proof of a duplicate sensor sample.
 */
MPU6050_Status MPU6050_Update(MPU6050_Handle_t *h)
{
    uint8_t bytes[2] = {0U, 0U};
    uint32_t duration_us = 0U;
    uint32_t now = micros();
    float dt;
#if defined(MPU_PHASE_AB)
    uint32_t irq_complete_us = 0U;
    uint32_t irq_sequence = 0U;
#endif
    int16_t raw;
    I2C_Status status, start;
    if (!h) return MPU6050_ERROR;

    if (h->sample_phase == 1U || h->sample_phase == 2U) {
        status = I2C_Read2Async_Poll(bytes, &duration_us);
        if (status == I2C_BUSY || status == I2C_IDLE) return MPU6050_OK;
        now = micros();
#if defined(MPU_PHASE_AB)
        if (status == I2C_OK && h->sample_phase == 2U) {
            I2C_GyroCompletionTrace_t irq_trace;
            irq_complete_us = I2C_AsyncGetLastCompletionUs();
            if (I2C_GetGyroCompletionTrace(&irq_trace))
                irq_sequence = irq_trace.sequence;
        }
#endif
        if (status != I2C_OK) {
            h->error_reads++;
            if (status == I2C_TIMEOUT) h->timeout_reads++;
            h->last_i2c_error = status;
            h->sample_phase = 0U;
            return MPU6050_ERROR;
        }

        if (h->sample_phase == 1U) {
            h->status_reads++;
            h->last_status_ready = (bytes[0] & 1U) ? 1U : 0U;
            if (h->last_status_ready) h->status_ready_reads++;
            else h->status_not_ready_reads++;
#if defined(MPU_FRESHNESS_DIAG)
            /* Timestamp when the asynchronous INT_STATUS transaction completes.
             * This polls sensor status at MCU time; not an actual DATA_RDY IRQ. */
            if (h->diag_previous_status_us != 0U) {
                uint32_t gap = now - h->diag_previous_status_us;
                if (gap > h->diag_status_gap_max_us)
                    h->diag_status_gap_max_us = gap;
            }
            h->diag_previous_status_us = now;
            if (h->last_status_ready) {
                h->diag_notready_run = 0U;
            } else {
                ++h->diag_notready_run;
                if (h->diag_notready_run > h->diag_notready_run_max)
                    h->diag_notready_run_max = h->diag_notready_run;
            }
#endif
#if defined(MPU_SAMPLING_POLICY_B)
            if (!h->last_status_ready) {
#if defined(MPU_FRESHNESS_DIAG)
                if (h->retry_count == 0U)
                    ++h->diag_ab_initial_notready;
                ++h->diag_ab_gyro_skipped;
#endif
                /* Production Policy B: never consume stale gyro data.
                 * Allow one delayed, nonblocking status retry per slot. */
                if (h->retry_count == 0U &&
                    (uint32_t)(now - h->last_request_us) <= MPU_POLICY_B_RETRY_START_MAX_US) {
                    h->retry_count = 1U;
                    h->retry_due_us = now + MPU_POLICY_B_RETRY_DELAY_US;
                    h->sample_phase = 4U;
                    SystemTimer_ArmWakeAfterUs(MPU_POLICY_B_RETRY_DELAY_US);
                } else {
                    h->sample_phase = 0U;
#if defined(MPU_FRESHNESS_DIAG)
                    if (h->retry_count != 0U)
                        ++h->diag_ab_retry_notready;
                    else
                        ++h->diag_ab_retry_expired;
#endif
                }
                return MPU6050_OK;
            }
#if defined(MPU_FRESHNESS_DIAG)
            if (h->retry_count != 0U) ++h->diag_ab_retry_ready;
#endif
#else
#if defined(MPU_FRESHNESS_DIAG)
            if (!h->last_status_ready) ++h->diag_ab_initial_notready;
#endif
#endif
            /* Async status transfer finished; gyro transaction starts on
             * a later main-loop pass after the peripheral STOP clears. */
            h->sample_phase = 3U;
            return MPU6050_OK;
        }

        /* Successful gyro transaction. For this experiment record even
         * status=0 (not-new) transfers and explicitly mark them as stale.
         */
        h->good_reads++;
        h->last_good_us = now;
        h->last_transfer_us = duration_us;
        if (duration_us > h->max_transfer_us) h->max_transfer_us = duration_us;
        raw = (int16_t)(((uint16_t)bytes[0] << 8) | bytes[1]);
        if (h->last_status_ready) h->fresh_gyro_reads++;
        else h->stale_gyro_reads++;
#if defined(MPU_FRESHNESS_DIAG)
        if (h->last_status_ready) {
            if (h->diag_previous_fresh_us != 0U) {
                uint32_t gap = now - h->diag_previous_fresh_us;
                if (gap < h->diag_fresh_gap_min_us)
                    h->diag_fresh_gap_min_us = gap;
                if (gap > h->diag_fresh_gap_max_us)
                    h->diag_fresh_gap_max_us = gap;
                if (gap > 1500U) ++h->diag_fresh_gap_gt1500;
                if (gap > 2000U) ++h->diag_fresh_gap_gt2000;
            }
            h->diag_previous_fresh_us = now;
            /* Publish timestamp + sequence atomically against TIM11 IRQ;
             * no I2C waits and only two 32-bit writes in a brief critical section. */
            {
                uint32_t primask = __get_PRIMASK();
                __disable_irq();
                h->diag_last_fresh_us = now;
                ++h->diag_fresh_sequence;
                if (primask == 0U) __enable_irq();
            }
        }
#endif
        h->sample_phase = 0U;

        if (!h->calibrating) MPU6050_TrackRaw(h, raw, now);
        if (h->calibrating) {
            /* Only completed, DATA_RDY-confirmed samples are eligible. */
            if (h->last_status_ready)
                MPU6050_AccumulateCalibration(h, raw, millis());
        } else if (h->calibrated && h->last_status_ready) {
            h->gyro_z_raw_corrected =
                ((float)raw / h->gyro_scale) - h->gyro_z_offset;
            h->gyro_z_lpf += GYRO_Z_LPF_ALPHA *
                              (h->gyro_z_raw_corrected - h->gyro_z_lpf);
            h->gyro_z_filtered =
                (fabsf(h->gyro_z_lpf) < GYRO_Z_DEADZONE) ?
                0.0f : h->gyro_z_lpf;
            if (h->last_update_us != 0U) {
                dt = (float)(now - h->last_update_us) / 1000000.0f;
                if (dt > 0.0f && dt <= 0.020f) {
                    h->yaw += h->gyro_z_filtered * dt;
                    h->yaw_no_deadzone += h->gyro_z_lpf * dt;
                    h->yaw_raw += h->gyro_z_raw_corrected * dt;
                }
            }
            h->last_update_us = now;
            /* Coherent main-to-TIM11 publication: completed sample only. */
            {
                uint32_t primask = __get_PRIMASK();
#if defined(MPU_PHASE_AB)
                uint32_t publish_us;
#endif
                __disable_irq();
                h->published_yaw_deg = h->yaw;
                h->published_gyro_z_dps = h->gyro_z_filtered;
                h->published_completed_us = now;
#if defined(MPU_PHASE_AB)
                /* Same point for both A/B channels: just before commit. */
                publish_us = micros();
                h->phase_published_irq_sequence = irq_sequence;
#endif
                ++h->published_sequence;
                h->published_valid = 1U;
                if (!primask) __enable_irq();
#if defined(MPU_PHASE_AB)
                /* Main-only statistics; no printing/I2C in TIM11. */
                {
                    uint32_t irq_to_main = now - irq_complete_us;
                    uint32_t main_to_publish = publish_us - now;
                    uint32_t irq_to_publish = publish_us - irq_complete_us;
                    uint32_t request_to_irq = irq_complete_us - h->phase_last_request_us;
                    h->phase_processed_count++;
                    h->phase_irq_to_main_sum_us += irq_to_main;
                    h->phase_main_to_publish_sum_us += main_to_publish;
                    h->phase_irq_to_publish_sum_us += irq_to_publish;
                    h->phase_request_to_irq_sum_us += request_to_irq;
                    if (irq_to_main > h->phase_irq_to_main_max_us)
                        h->phase_irq_to_main_max_us = irq_to_main;
                    if (main_to_publish > h->phase_main_to_publish_max_us)
                        h->phase_main_to_publish_max_us = main_to_publish;
                    if (irq_to_publish > h->phase_irq_to_publish_max_us)
                        h->phase_irq_to_publish_max_us = irq_to_publish;
                    if (request_to_irq > h->phase_request_to_irq_max_us)
                        h->phase_request_to_irq_max_us = request_to_irq;
                    h->phase_last_sample_request_us = h->phase_last_request_us;
                    h->phase_last_sample_irq_us = irq_complete_us;
                    h->phase_last_sample_main_us = now;
                    h->phase_last_sample_publish_us = publish_us;
                }
#endif
            }
        }
    }

#if defined(MPU_SAMPLING_POLICY_B)
    if (h->sample_phase == 4U) {
        /* TIM5 CC1 may wake main after 220us. Never spin/busy-wait: if
         * not due, return to WFI; if the slot expires, abandon retry. */
        now = micros();
        if ((uint32_t)(now - h->last_request_us) >= MPU_POLICY_B_RETRY_END_MAX_US) {
 #if defined(MPU_FRESHNESS_DIAG)
            ++h->diag_ab_retry_expired;
 #endif
            h->sample_phase = 0U;
            SystemTimer_CancelWake();
            return MPU6050_OK;
        }
        if ((int32_t)(now - h->retry_due_us) < 0)
            return MPU6050_OK;
        SystemTimer_CancelWake();
        start = I2C_ReadRegAsync_Start(h->address, MPU6050_INT_STATUS, 1U);
        if (start == I2C_OK) {
 #if defined(MPU_FRESHNESS_DIAG)
            ++h->diag_ab_retry_started;
 #endif
            h->sample_phase = 1U;
        } else {
            /* If busy, do NOT spin to steal CPU time from TIM11. */
            if (start == I2C_BUSY) {
     #if defined(MPU_FRESHNESS_DIAG)
                ++h->diag_ab_retry_busy;
     #endif
            } else {
                ++h->error_reads;
                h->last_i2c_error = start;
            }
            h->sample_phase = 0U;
        }
        return MPU6050_OK;
    }
#endif
    if (h->sample_phase == 3U) {
        /* STOP was sent in the prior transaction; retry without waiting. */
        start = I2C_ReadRegAsync_Start(h->address, MPU6050_GYRO_ZOUT_H, 2U);
        if (start == I2C_OK) {
            h->data_read_start_us = micros();
#if defined(MPU_FRESHNESS_DIAG)
            ++h->diag_ab_gyro_started;
#endif
            h->sample_phase = 2U;
        } else if (start != I2C_BUSY) {
            h->error_reads++;
            h->last_i2c_error = start;
            h->sample_phase = 0U;
        }
        return MPU6050_OK;
    }

    now = micros();
#if defined(MPU_TIM11_TICK_SCHED)
    /* A tick is an opportunity, not a queue. Main starts the next
     * transaction when the I2C state machine is IDLE; no floating
     * 999us >=1000us time-window test is used in this experiment. */
    uint32_t control_tick_seq;
    {
        uint32_t primask = __get_PRIMASK();
        __disable_irq();
        control_tick_seq = h->control_sample_tick_seq;
        if (!primask) __enable_irq();
    }
    if (h->sample_phase == 0U &&
        control_tick_seq != 0U &&
        control_tick_seq != h->control_last_started_tick_seq) {
#else
    if (h->sample_phase == 0U &&
        (uint32_t)(now - h->last_request_us) >= MPU_SAMPLE_PERIOD_US) {
#endif
        start = I2C_ReadRegAsync_Start(h->address, MPU6050_INT_STATUS, 1U);
        if (start == I2C_OK) {
#if defined(MPU_TIM11_TICK_SCHED)
            h->control_last_started_tick_seq = control_tick_seq;
#endif
#if defined(MPU_PHASE_AB)
            if (h->calibrated) {
                /* Actual START setup timestamp inside I2C driver. */
                uint32_t stamp = I2C_AsyncGetLastStartUs();
                if (h->phase_prev_request_us) {
                    uint32_t gap_us = stamp - h->phase_prev_request_us;
                    h->phase_request_intervals++;
                    h->phase_request_interval_sum_us += gap_us;
                    if (gap_us < h->phase_request_min_us)
                        h->phase_request_min_us = gap_us;
                    if (gap_us > h->phase_request_max_us)
                        h->phase_request_max_us = gap_us;
                    if (gap_us > 1050U) h->phase_request_gt1050++;
                    if (gap_us > 1500U) h->phase_request_gt1500++;
                }
                h->phase_prev_request_us = stamp;
                h->phase_last_request_us = stamp;
                h->phase_request_count++;
            }
#endif
            h->last_request_us = now;
#if defined(MPU_SAMPLING_POLICY_B)
            h->retry_count = 0U;
#endif
            h->sample_phase = 1U;
        } else if (start == I2C_BUSY) {
            h->busy_retries++;
        } else {
            h->error_reads++;
            h->last_i2c_error = start;
        }
    }
    return MPU6050_OK;
}

#if defined(MPU_TIM11_TICK_SCHED)
void MPU6050_NotifyControlTick(MPU6050_Handle_t *h, uint32_t entry_us)
{
    if (!h) return;
    /* Called from TIM11; coherent main read uses a short IRQ mask. */
    h->control_sample_tick_us = entry_us;
    ++h->control_sample_tick_seq;
}
#endif

uint8_t MPU6050_NeedsImmediatePoll(MPU6050_Handle_t *h)
{
    if (!h) return 0U;
#if defined(MPU_SAMPLING_POLICY_B)
    /* If compare IRQ fired before main checked, prevent sleeping with a
     * retry already due. TIM11 provides fallback wake if the race is lost. */
    if (h->sample_phase == 4U &&
        (int32_t)(micros() - h->retry_due_us) >= 0)
        return 1U;
#endif
    return (h->sample_phase == 3U) ? 1U : 0U;
}

#if defined(MPU_PHASE_AB)
void MPU6050_GetPhaseStats(const MPU6050_Handle_t *h,
                           MPU6050_PhaseStats_t *out)
{
    if (!h || !out) return;
    /* Main context only: 64-bit accumulation is intentionally not atomic
     * against any other main task, but TIM11 never writes these counters. */
    out->request_count = h->phase_request_count;
    out->request_intervals = h->phase_request_intervals;
    out->request_min_us = h->phase_request_intervals ?
                          h->phase_request_min_us : 0U;
    out->request_max_us = h->phase_request_max_us;
    out->request_gt1050 = h->phase_request_gt1050;
    out->request_gt1500 = h->phase_request_gt1500;
    out->request_interval_avg_us = h->phase_request_intervals ?
        (uint32_t)(h->phase_request_interval_sum_us / h->phase_request_intervals) : 0U;
    out->processed_count = h->phase_processed_count;
    out->irq_to_main_avg_us = h->phase_processed_count ?
        (uint32_t)(h->phase_irq_to_main_sum_us / h->phase_processed_count) : 0U;
    out->irq_to_main_max_us = h->phase_irq_to_main_max_us;
    out->irq_to_publish_avg_us = h->phase_processed_count ?
        (uint32_t)(h->phase_irq_to_publish_sum_us / h->phase_processed_count) : 0U;
    out->irq_to_publish_max_us = h->phase_irq_to_publish_max_us;
    out->main_to_publish_avg_us = h->phase_processed_count ?
        (uint32_t)(h->phase_main_to_publish_sum_us / h->phase_processed_count) : 0U;
    out->main_to_publish_max_us = h->phase_main_to_publish_max_us;
    out->request_to_irq_avg_us = h->phase_processed_count ?
        (uint32_t)(h->phase_request_to_irq_sum_us / h->phase_processed_count) : 0U;
    out->request_to_irq_max_us = h->phase_request_to_irq_max_us;
    out->last_request_us = h->phase_last_sample_request_us;
    out->last_irq_us = h->phase_last_sample_irq_us;
    out->last_main_us = h->phase_last_sample_main_us;
    out->last_publish_us = h->phase_last_sample_publish_us;
}
#endif

uint8_t MPU6050_GetSnapshot(const MPU6050_Handle_t *h,
                            MPU6050_Snapshot_t *out)
{
    uint32_t primask;
    if (!out) return 0U;
    out->valid = 0U;
    if (!h) return 0U;
    /* Safe in main and in TIM11: producer runs only in main; producer
     * disables interrupts briefly during publication. */
    primask = __get_PRIMASK();
    __disable_irq();
    out->yaw_deg = h->published_yaw_deg;
    out->gyro_z_dps = h->published_gyro_z_dps;
    out->completed_us = h->published_completed_us;
    out->sequence = h->published_sequence;
    out->valid = h->published_valid;
#if defined(MPU_PHASE_AB)
    out->source_irq_sequence = h->phase_published_irq_sequence;
#endif
    if (!primask) __enable_irq();
    return out->valid;
}

float MPU6050_GetYaw(MPU6050_Handle_t *h) { return h ? h->yaw : 0.0f; }
float MPU6050_GetGyroZ(MPU6050_Handle_t *h) { return h ? h->gyro_z_filtered : 0.0f; }
void MPU6050_ResetYaw(MPU6050_Handle_t *h)
{
    if (!h) return;
    h->yaw = 0.0f;
    h->yaw_no_deadzone = 0.0f;
    h->yaw_raw = 0.0f;
    h->last_update_us = 0U;
    { uint32_t primask = __get_PRIMASK();
      __disable_irq();
      h->published_yaw_deg = 0.0f;
      if (!primask) __enable_irq();
    }
}
void MPU6050_SetYaw(MPU6050_Handle_t *h, float angle)
{
    uint32_t primask;
    if (!h) return;
    primask = __get_PRIMASK();
    __disable_irq();
    h->yaw = angle;
    h->published_yaw_deg = angle;
    if (!primask) __enable_irq();
}

MPU6050_Status MPU6050_IsReady(MPU6050_Handle_t *h)
{
    if (!h) return MPU6050_ERROR;
    /* Blocking probe for startup/diagnostic ONLY, not the 1kHz path. */
    return (I2C_IsDeviceReady(h->i2c, h->address) == I2C_OK) ?
           MPU6050_OK : MPU6050_NOT_FOUND;
}
