/* Motor-OFF integration regression for production MPU_SAMPLING_POLICY_B.
 * No DMA, motor, encoder, IR, BLE or MPU_FRESHNESS_DIAG instrumentation.
 * TIM11 consumes a coherent snapshot at 1kHz, without I2C work in ISR.
 */
#include "board.h"
#include "control.h"
#include "mpu6050.h"
#include "system_timer.h"
#include "uart.h"
#include "stm32f4xx.h"

#if defined(MPU_INTEGRATION_BENCH)
#if !defined(MPU_SAMPLING_POLICY_B)
#error "Integration bench must compile independent Policy B"
#endif
#if defined(MPU_FRESHNESS_DIAG) || defined(MPU_FRESHNESS_AB_B) || defined(CONTROL_LOAD_DIAG)
#error "Integration bench must NOT require diagnostic flags"
#endif
#if !defined(CLOCK_VALIDATE_96M) || !defined(CLOCK_HSI_TRIM) || (CLOCK_HSI_TRIM != 25)
#error "Integration bench must use the verified 96MHz/trim25 profile"
#endif

int MpuIntegrationBench_Main(void)
{
    static MPU6050_Handle_t mpu;
    MPU6050_Status init_status;
    uint32_t last_log_us = 0U;
    uint32_t seq = 0U;
    uint8_t calibration_printed = 0U;

    Board_Init();
    SystemTimer_Init();
    UART_Init();

    UART_Printf("\r\n[MINT_BOOT] MPU production sampling Policy B, motors disabled\r\n");
    UART_Printf("MINT_PROFILE,policy=B,diag=0,ab_diag=0,load_diag=0,"
                "motor_off=1,nominal_clock_hz=%lu,trim=%lu,"
                "retry_us=%u,retry_start_limit_us=%u,retry_end_limit_us=%u\r\n",
                (unsigned long)SystemCoreClock,
                (unsigned long)Board_GetHsiTrim(),
                (unsigned)MPU_POLICY_B_RETRY_DELAY_US,
                (unsigned)MPU_POLICY_B_RETRY_START_MAX_US,
                (unsigned)MPU_POLICY_B_RETRY_END_MAX_US);
    init_status = MPU6050_Init(&mpu);
    UART_Printf("MINT_I2C,whoami=0x%02X,selected=%u,scl_nominal=%lu,init_status=%u\r\n",
                (unsigned)mpu.probe_who_value[0],
                (unsigned)mpu.selected_speed_index,
                (unsigned long)I2C_GetConfiguredSclHz(),
                (unsigned)init_status);

    if (init_status != MPU6050_OK || Board_GetClockFault() != 0U) {
        UART_Printf("MINT_BOOT_FAIL,mpu=%u,clock=%lu,step=%u,i2c=%u\r\n",
                    (unsigned)init_status,(unsigned long)Board_GetClockFault(),
                    (unsigned)mpu.init_step,(unsigned)mpu.init_i2c_status);
        while (1) { Board_StatusLed(1U); delay_ms_blocking(250U); Board_StatusLed(0U); delay_ms_blocking(250U); }
    }

    MPU6050_Calibrate(&mpu);
    Control_Init(&mpu);
    /* Keeps every motor and motion callback OFF; gyro snapshot still
     * consumed by the 1kHz ISR for integration freshness measurements. */
    Control_SetDiagnosticOnly(1U);
    Control_Start();

    last_log_us = micros();
    while (1) {
        uint32_t now;
        MPU6050_Snapshot_t snap;
        Control_IntegrationStats_t stats;
        Control_Timing_t timing;
        MPU6050_Update(&mpu);

        if (mpu.calibrated && !calibration_printed) {
            calibration_printed = 1U;
            MPU6050_ResetYaw(&mpu);
            UART_Printf("MINT_CAL_DONE,n=%u,retries=%u,blocks=%u,bias_mdps=%ld\r\n",
                        (unsigned)mpu.calibration_count,
                        (unsigned)mpu.cal_retries,
                        (unsigned)mpu.cal_valid_blocks,
                        (long)(mpu.gyro_z_offset * 1000.0f));
        }
        if (mpu.cal_state == 4U) {
            UART_Printf("MINT_CAL_FAIL,retries=%u,reason=%u\r\n",
                        (unsigned)mpu.cal_retries, (unsigned)mpu.cal_failure_reason);
            while (1) { __WFI(); } /* Never arm motors */
        }

        now = micros();
        if ((uint32_t)(now - last_log_us) >= 1000000UL) {
            Control_GetIntegrationStats(&stats);
            Control_GetTiming(&timing);
            MPU6050_GetSnapshot(&mpu, &snap);
            ++seq;
            UART_Printf(
               "MINT,seq=%lu,cal=%u,valid=%u,ticks=%lu,reused=%lu,"
               "aged1500=%lu,age_max=%lu,missing=%lu,sample_seq=%lu,"
               "good=%lu,fresh=%lu,stale=%lu,notready=%lu,"
               "i2c_err=%lu,timeout=%lu,busy=%lu,"
               "timing_late=%lu,timing_gap=%lu,timing_max_cycles=%lu,"
               "tx_drop=%lu\r\n",
               (unsigned long)seq,(unsigned)mpu.calibrated,
               (unsigned)snap.valid,(unsigned long)stats.ticks,
               (unsigned long)stats.reused,(unsigned long)stats.age_over1500,
               (unsigned long)stats.age_max_us,(unsigned long)stats.missing_or_expired,
               (unsigned long)snap.sequence,(unsigned long)mpu.good_reads,
               (unsigned long)mpu.fresh_gyro_reads,(unsigned long)mpu.stale_gyro_reads,
               (unsigned long)mpu.status_not_ready_reads,
               (unsigned long)mpu.error_reads,(unsigned long)mpu.timeout_reads,
               (unsigned long)mpu.busy_retries,
               (unsigned long)timing.late_over_50us,
               (unsigned long)timing.gap_over_1500us,
               (unsigned long)timing.exec_max_cycles,
               (unsigned long)UART_GetDroppedMessages());
            last_log_us = now;
        }

        if (!MPU6050_NeedsImmediatePoll(&mpu)) __WFI();
    }
}
#endif
