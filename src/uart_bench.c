/* Compile-time-only UART1 PA9(DMA TX)/PA10(RX IRQ) profiling.
 * No PWM output and no UART traffic generated until TXBENCH hz arrives.
 * Does not change TIM11/MPU/I2C priority or interact with motor ISR.
 *
 * Protocol (main-context):
 * TXBENCH <hz>    (0 cancels, 5..400 enables)
 * TXSTAT          prints two short cumulative status lines
 * U_PROBE         73-ish byte sequenced frames, designed to validate HOST
 *                 receipt and actual TX bitrate separately from enqueued.
 */
#include "uart_bench.h"
#if defined(UART_PROFILE_DIAG)
#if !defined(INTEGRATION_DIAG_MOTOR_OFF)
#error "UART_PROFILE_DIAG requires motor output hard-lock at boot"
#endif
#include "uart.h"
#include "control.h"
#include "motor.h"
#include "system_timer.h"
#include "stm32f4xx.h"

static uint16_t current_hz;
static uint32_t due_us;
static uint32_t next_status_ms;
static uint32_t sequence_num;
static uint32_t skipped_deadlines;
static uint32_t last_health_ms, initial_imu_errors, initial_imu_timeouts;
static uint32_t base_tim11_gap, base_stale;

void UART_Bench_Init(void)
{
    current_hz = 0U;
    due_us = next_status_ms = sequence_num = skipped_deadlines = 0U;
    last_health_ms = initial_imu_errors = initial_imu_timeouts = 0U;
    base_tim11_gap = base_stale = 0U;
}

uint8_t UART_Bench_SetRate(uint16_t hz, uint32_t now_ms,
                           const MPU6050_Handle_t *mpu)
{
    if (hz == 0U) {
        current_hz = 0U;
        return 1U; /* STOP always possible, including during fault. */
    }
    if (hz < 5U || hz > 400U || !Control_IsRunning() ||
        !Motor_OutputsLocked() || !mpu || !mpu->calibrated ||
        mpu->cal_state != 3U)
        return 0U;
    current_hz = hz;
    sequence_num = 0U;
    skipped_deadlines = 0U;
    due_us = micros() + 1000000U / hz;
    next_status_ms = now_ms + 1000U;
    last_health_ms = now_ms;
    initial_imu_errors = mpu->error_reads;
    initial_imu_timeouts = mpu->timeout_reads;
    {
        Control_DebugStats_t init;
        Control_GetDebugStats(&init);
        base_tim11_gap = init.gaps_gt1500us;
        base_stale = init.gyro_invalid_after_first_valid;
    }
    UART_ResetProfileQueuePeak();
    return 1U;
}

void UART_Bench_PrintStatus(uint32_t now_ms, const MPU6050_Handle_t *mpu)
{
    UART_ProfileStats_t uart;
    Control_DebugStats_t timing;
    UART_GetProfileStats(&uart);
    Control_GetDebugStats(&timing);
    /* Separating lines avoids UART_Printf's 256-byte stack buffer limit.
     * Counters are cumulative, host differentiates before/after snapshots. */
    UART_Printf("U_TX,ms=%lu,hz=%u,seq=%lu,skip=%lu,accb=%lu,doneb=%lu,q=%lu,peak=%lu,drops=%lu,rejbytes=%lu,fmtcy=%lu,dmacy=%lu,rxcy=%lu,dmaerr=%lu\r\n",
                (unsigned long)now_ms, (unsigned)current_hz,
                (unsigned long)sequence_num, (unsigned long)skipped_deadlines,
                (unsigned long)uart.accepted_bytes,
                (unsigned long)uart.completed_bytes,
                (unsigned long)uart.queued_bytes,
                (unsigned long)uart.queue_peak,
                (unsigned long)uart.dropped_msgs,
                (unsigned long)uart.rejected_bytes,
                (unsigned long)uart.printf_cycles,
                (unsigned long)uart.tx_dma_isr_cycles,
                (unsigned long)uart.rx_isr_cycles,
                (unsigned long)uart.dma_error_count);
    UART_Printf("U_SYS,ms=%lu,hz=%u,ticks=%lu,late=%lu,gap=%lu,stale=%lu,ir=%lu,i2cerr=%lu,to=%lu,rxb=%lu,rxerr=%lu,rxov=%lu,prnmax=%lu,dmaisrmax=%lu,rxisrmax=%lu\r\n",
                (unsigned long)now_ms, (unsigned)current_hz,
                (unsigned long)timing.ticks,
                (unsigned long)timing.late_gt50us,
                (unsigned long)timing.gaps_gt1500us,
                (unsigned long)timing.gyro_invalid_after_first_valid,
                (unsigned long)timing.ir_scan_count,
                (unsigned long)(mpu ? mpu->error_reads : 0UL),
                (unsigned long)(mpu ? mpu->timeout_reads : 0UL),
                (unsigned long)uart.rx_bytes,
                (unsigned long)uart.rx_errors,
                (unsigned long)uart.rx_overflow,
                (unsigned long)uart.printf_max_cycles,
                (unsigned long)uart.tx_dma_isr_max_cycles,
                (unsigned long)uart.rx_isr_max_cycles);
}

void UART_Bench_Poll(uint32_t now_ms, const MPU6050_Handle_t *mpu)
{
    uint32_t now_us, period;
    Control_DebugStats_t stats;
    if (!current_hz) return;
    /* A health snapshot masks interrupts briefly. Check every 20ms,
     * never on every main-loop spin: measurement must not perturb TIM11. */
    if ((uint32_t)(now_ms - last_health_ms) >= 20U) {
        last_health_ms = now_ms;
        if (!Motor_OutputsLocked() || !Control_IsRunning() ||
            !mpu || !mpu->calibrated || mpu->cal_state != 3U ||
            mpu->timeout_reads != initial_imu_timeouts ||
            mpu->error_reads != initial_imu_errors) {
            current_hz = 0U;
            UART_SendString("U_ABORT,reason=SYSTEM_FAULT\r\n");
            return;
        }
        Control_GetDebugStats(&stats);
        if (stats.gaps_gt1500us != base_tim11_gap ||
            stats.gyro_invalid_after_first_valid != base_stale) {
            current_hz = 0U;
            UART_SendString("U_ABORT,reason=TIM11_OR_GYRO\r\n");
            return;
        }
    }
    now_us = micros();
    period = 1000000U / current_hz;
    if ((int32_t)(now_us - due_us) >= 0) {
        uint32_t behind = (uint32_t)(now_us - due_us);
        if (behind >= period) skipped_deadlines += behind / period;
        due_us = now_us + period; /* Never catch-up burst and stall main. */
        UART_Printf("U_PROBE,seq=%06lu,ms=%010lu,payload=0123456789ABCDEF0123456789ABCDEF\r\n",
                    (unsigned long)sequence_num++, (unsigned long)now_ms);
    }
    if ((int32_t)(now_ms - next_status_ms) >= 0) {
        next_status_ms = now_ms + 1000U;
        UART_Bench_PrintStatus(now_ms, mpu);
    }
}
#endif
