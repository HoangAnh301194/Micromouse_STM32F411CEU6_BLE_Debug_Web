#include "board.h"
#include "bt_debug.h"
#if defined(ENCODER_PROFILE_DIAG) && (!defined(INTEGRATION_DIAG) || \
    !(defined(INTEGRATION_DIAG_MOTOR_OFF) || defined(INTEGRATION_DIAG_MOTOR_ON)))
#error "Encoder telemetry requires an explicit integration diagnostic mode"
#endif

#include "control.h"
#include "encoder.h"
#include "ir_sensor.h"
#include "motion.h"
#include "motor.h"
#include "mpu6050_i2c_module.h"
#include "system_timer.h"
#include "uart.h"
#include "integration_sweep.h"
#if defined(INTEGRATION_DIAG)
#include "integration_serial.h"
#if defined(UART_PROFILE_DIAG)
#include "uart_bench.h"
#endif
#endif

#include "stm32f4xx.h"

#define DEBUG_PERIOD_MS 100UL

static MPU6050_Handle_t mpu;

static void HandleBleCommand(char c)
{
#if defined(INTEGRATION_DIAG)
    /* Diagnostic profiles exclude other BLE motion commands. */
    if (c != 's' && c != 'S') return;
    Sweep_Abort(SWEEP_STOP_REQUEST);
#endif
    MPU6050_Snapshot_t snap;
    uint8_t ready = MPU6050_GetSnapshot(&mpu, &snap) &&
                    (uint32_t)(micros() - snap.completed_us) <= 3000U;
    float yaw = ready ? snap.yaw_deg : 0.0f;

    switch (c) {
    case 'f':
    case 'F':
        if (ready && Motion_IsDone()) Motion_CommandStraight(180.0f, yaw);
        break;

    case 'l':
    case 'L':
        if (ready && Motion_IsDone()) Motion_CommandTurn(90.0f, yaw);
        break;

    case 'r':
    case 'R':
        if (ready && Motion_IsDone()) Motion_CommandTurn(-90.0f, yaw);
        break;

    case 's':
    case 'S':
        Motion_Stop();
        break;

    case 'z':
    case 'Z':
        if (ready && Motion_IsDone()) MPU6050_ResetYaw(&mpu);
        break;

    default:
        break;
    }
}

#if defined(ENCODER_ALPHA_AB_DIAG)
/* Only called in main, never from TIM11. Six short metrics frames per stage,
 * scheduled across subsequent main iterations to protect UART DMA queue. */
static void Encoder_LogShadowMetric(uint32_t now, uint32_t stage,
                                    uint8_t state, uint8_t line,
                                    const Encoder_ShadowSnapshot_t *snap)
{
    const uint32_t *arr;
    uint32_t n;
    uint8_t wheel = line / 3U;
    uint8_t metric = line % 3U;
    if (wheel == 0U) {
        arr = metric == 0U ? snap->window_abs_error_left :
              metric == 1U ? snap->transient_abs_error_left :
                             snap->variation_left;
    } else {
        arr = metric == 0U ? snap->window_abs_error_right :
              metric == 1U ? snap->transient_abs_error_right :
                             snap->variation_right;
    }
    n = metric == 1U ? snap->valid_reference_ticks : snap->window_ticks;
    UART_Printf("SYS_AM,ms=%lu,id=%lu,state=%u,w=%u,m=%u,n=%lu,x0=%lu,x1=%lu,x2=%lu,x3=%lu,x4=%lu,x5=%lu,x6=%lu\r\n",
                (unsigned long)now, (unsigned long)stage,
                (unsigned)state, (unsigned)wheel, (unsigned)metric,
                (unsigned long)n, (unsigned long)arr[0],
                (unsigned long)arr[1], (unsigned long)arr[2],
                (unsigned long)arr[3], (unsigned long)arr[4],
                (unsigned long)arr[5], (unsigned long)arr[6]);
}
static void Encoder_LogShadowInstant(uint32_t now,
                                     const Encoder_ShadowSnapshot_t *snap)
{
    uint8_t w;
    for (w = 0U; w < 2U; ++w) {
        const int32_t *a = w == 0U ? snap->alpha_left : snap->alpha_right;
        const int32_t *box = w == 0U ? snap->box_left : snap->box_right;
        UART_Printf("SYS_AB,ms=%lu,w=%u,a05=%ld,a10=%ld,a20=%ld,a30=%ld,a40=%ld,a60=%ld,a80=%ld,b5=%ld,b10=%ld,b20=%ld\r\n",
                    (unsigned long)now, (unsigned)w,
                    (long)a[0], (long)a[1], (long)a[2],
                    (long)a[3], (long)a[4], (long)a[5], (long)a[6],
                    (long)box[0], (long)box[1], (long)box[2]);
    }
}
#endif

int main(void)
{
    uint32_t last_debug_ms = 0;
#if defined(INTEGRATION_DIAG)
    uint32_t last_system_diag_ms = 0U;
#if defined(ENCODER_PROFILE_DIAG)
    uint32_t last_encoder_diag_ms = 0U;
#if defined(ENCODER_ALPHA_AB_DIAG)
    uint32_t last_shadow_diag_ms = 0U, shadow_last_emit_ms = 0U;
    uint32_t shadow_completed_stage = 0U;
    uint8_t shadow_completed_state = 0U;
    uint8_t shadow_metric_next = 6U;
    Encoder_ShadowSnapshot_t shadow_completed;
#endif
#endif
    uint32_t last_ir_value_ms = 0U;
    uint32_t last_ir_value_scan = 0U;
    uint8_t last_cal_state = 0U;
    uint32_t last_sweep_stage = 0xFFFFFFFFUL;
    uint8_t last_sweep_state = 0xFFU;
    int32_t last_sweep_enc_l = 0, last_sweep_enc_r = 0;
    uint32_t last_sweep_log_ms = 0U;
    uint32_t coast_start_ms = 0U, coast_last_ms = 0U;
    int32_t coast_start_l = 0, coast_start_r = 0;
#endif
    MPU6050_Status mpu_status;

    Board_Init();
    SystemTimer_Init();
    UART_Init(); /* USART1 PA9 DMA TX and PA10 IRQ RX. */

    /* Fail closed BEFORE Motor_Init on HSI/PLL readiness or clock-profile
     * failure. Peripheral baud/timer setup below depends on the selected
     * 96MHz profile having been initialized successfully. */
    if (Board_GetClockFault() != 0U) {
        Board_StatusLed(1U);
        UART_Printf("BOOT_FAIL,CLOCK,fault=%lu,sysclk_nominal=%lu,trim=%lu\r\n",
                    (unsigned long)Board_GetClockFault(),
                    (unsigned long)SystemCoreClock,
                    (unsigned long)Board_GetHsiTrim());
        while (1) { __WFI(); } /* Motors NEVER initialized/enabled. */
    }

    UART_Printf("CLOCK_PROFILE,nominal_hz=%lu,trim=%lu,hsical=%lu,pclk1=%lu,pclk2=%lu,tim2clk=%lu,tim11clk=%lu\r\n",
                (unsigned long)SystemCoreClock,
                (unsigned long)Board_GetHsiTrim(),
                (unsigned long)Board_GetHsiCal(),
                (unsigned long)Board_GetAPB1ClockHz(),
                (unsigned long)Board_GetAPB2ClockHz(),
                (unsigned long)Board_GetAPB1TimerClockHz(),
                (unsigned long)Board_GetAPB2TimerClockHz());

    Motor_Init();
#if defined(INTEGRATION_DIAG_MOTOR_OFF)
    /* Hard latch for motor-OFF sensor/clock smoke test: no rearm until reset. */
    Motor_LockOutputs();
#endif
    Encoder_Init();

    /* MPU6050_Init owns RC2 I2C bus probing and initialization. */
    BT_Init();
    IR_Init();

    mpu_status = MPU6050_Init(&mpu);
    if (mpu_status == MPU6050_OK) {
        mpu_status = MPU6050_Calibrate(&mpu); /* Starts async calibration. */
    }

    Motion_Init();
#if defined(INTEGRATION_DIAG)
    Sweep_Init();
#if defined(UART_PROFILE_DIAG)
    UART_Bench_Init();
#endif
#endif

    if (mpu_status != MPU6050_OK) {
        Motor_Stop();
        Board_StatusLed(1);
        BT_SendString("[BOOT] MPU6050 init/calibration failed\r\n");
#if defined(INTEGRATION_DIAG)
        UART_Printf("MPU_BOOT_FAIL,status=%u,step=%u,reg=%u,i2c=%u,got=%u\r\n",
                    (unsigned)mpu_status, (unsigned)mpu.init_step,
                    (unsigned)mpu.init_register, (unsigned)mpu.init_i2c_status,
                    (unsigned)mpu.init_read_value);
#endif

        while (1) {
        }
    }

    /*
     * TIM10 belongs to ir_sensor.c.
     * TIM11 belongs to control.c.
     * main.c only performs system orchestration.
     */
    IR_StartScan();
    Control_Init(&mpu);
    Control_Start();

    BT_SendString("\r\n[BOOT] MPU RC2 calibration started\r\n");
    UART_SendString("[BOOT] USART1 PA9 DMA TX; MPU RC2 sampling enabled\r\n");
    BT_SendString("Commands: F=straight 180mm, L=left 90, R=right 90, S=stop, Z=zero yaw\r\n");
#if defined(INTEGRATION_DIAG_MOTOR_OFF)
    UART_SendString("INTEGRATION_DIAG,MOTOR_OFF_SHADOW,OUTPUTS_LOCKED\r\n");
#elif defined(INTEGRATION_DIAG_MOTOR_ON)
    UART_SendString("INTEGRATION_DIAG,MOTOR_ON,UART_MODE_ON_AND_START,0_TO_100_PERCENT\r\n");
#endif

    while (1) {
        uint32_t now = millis();
        MPU6050_Update(&mpu); /* Main-context IRQ-driven Policy B. */
#if defined(INTEGRATION_DIAG)
        Sweep_Update(now, &mpu);
#endif

        if (BT_Available()) {
            HandleBleCommand(BT_ReceiveChar());
        }
#if defined(INTEGRATION_DIAG)
        /* RX IRQ only queues bytes; command parsing is budgeted in main. */
        IntegrationSerial_Poll(&mpu, now);
#if defined(UART_PROFILE_DIAG)
        /* Main-only TX generator, never in TIM11 or USART ISR. */
        UART_Bench_Poll(now, &mpu);
#endif
#else
        /* Production UART diagnostic remains STOP/status only. */
        {
            uint8_t ch, budget = 16U;
            while (budget-- && UART_ReadByte(&ch)) {
                if (ch == 'S' || ch == 's') Motion_Stop();
                if (ch == '?')
                    UART_Printf("DBG,rx_overflow=%lu,rx_error=%lu,tx_drop=%lu\r\n",
                                (unsigned long)UART_GetRxOverflow(),
                                (unsigned long)UART_GetRxErrors(),
                                (unsigned long)UART_GetDroppedMessages());
            }
        }
#endif

#if defined(INTEGRATION_DIAG)
        /* One diagnostic bundle per second. UART prints only in main, never
         * from TIM11/I2C/TIM10 interrupts. Host must analyze *deltas* of
         * monotonic counters; clock values remain RCC-derived nominal. */
        {
            SweepSnapshot sw;
            Sweep_GetSnapshot(&sw);
            if (sw.state != SWEEP_DISARMED &&
                (sw.stage_id != last_sweep_stage || sw.state != last_sweep_state)) {
                int32_t left = Encoder_GetLeftCount();
                int32_t right = Encoder_GetRightCount();
                int16_t applied_l = 0, applied_r = 0;
                uint32_t dt_ms = last_sweep_log_ms ?
                    (uint32_t)(now - last_sweep_log_ms) : 0U;
                Motor_GetAppliedPairPermille(&applied_l, &applied_r);
#if defined(ENCODER_ALPHA_AB_DIAG)
                if (last_sweep_stage != 0xFFFFFFFFUL) {
                    Encoder_GetShadowSnapshot(&shadow_completed);
                    shadow_completed_stage = last_sweep_stage;
                    shadow_completed_state = last_sweep_state;
                    shadow_metric_next = 0U;
                    shadow_last_emit_ms = now - 25U;
                }
                Encoder_ShadowResetMetrics();
#endif
                /* state=2 denotes END OF DRIVE; state=1 with id>0
                 * denotes END OF PREVIOUS STAGE'S COAST. Both dl/dr
                 * and elapsed dt_ms refer to the preceding interval. */
                UART_Printf("SYS_STEP,ms=%lu,state=%u,fault=%u,mode=%u,id=%lu,group=%u,step=%u,duty=%u,dt=%lu,dl=%ld,dr=%ld,vL=%ld,vR=%ld,appL=%d,appR=%d\r\n",
                            (unsigned long)now, (unsigned)sw.state,
                            (unsigned)sw.fault, (unsigned)sw.motor_enabled,
                            (unsigned long)sw.stage_id, (unsigned)sw.group,
                            (unsigned)sw.step, (unsigned)sw.duty_permille,
                            (unsigned long)dt_ms,
                            (long)(left - last_sweep_enc_l),
                            (long)(right - last_sweep_enc_r),
                            (long)Encoder_GetLeftSpeedPPS(),
                            (long)Encoder_GetRightSpeedPPS(),
                            (int)applied_l, (int)applied_r);
                last_sweep_stage = sw.stage_id;
                last_sweep_state = sw.state;
                last_sweep_enc_l = left;
                last_sweep_enc_r = right;
                last_sweep_log_ms = now;
#if defined(ENCODER_ALPHA_AB_DIAG)
                /* Completed interval is queued above; first frame is sent
                 * outside this transition bundle at the normal main cadence. */
#endif
                if (sw.state == SWEEP_COAST) {
                    coast_start_ms = coast_last_ms = now;
                    coast_start_l = left;
                    coast_start_r = right;
                }
            }
            /* 500-ms samples during each 2–3s coast interval. These
             * capture transient encoder movement and the filtered speed
             * approaching zero without doing I/O in TIM11. */
            if (sw.state == SWEEP_COAST &&
                (uint32_t)(now - coast_last_ms) >= 500U) {
                coast_last_ms = now;
                UART_Printf("SYS_COAST,ms=%lu,id=%lu,dt=%lu,dl=%ld,dr=%ld,ppsL=%ld,ppsR=%ld\r\n",
                            (unsigned long)now,
                            (unsigned long)sw.stage_id,
                            (unsigned long)(now - coast_start_ms),
                            (long)(Encoder_GetLeftCount() - coast_start_l),
                            (long)(Encoder_GetRightCount() - coast_start_r),
                            (long)Encoder_GetLeftSpeedPPS(),
                            (long)Encoder_GetRightSpeedPPS());
            }
        }
        if ((uint32_t)(now - last_system_diag_ms) >= 1000U) {
            Control_DebugStats_t sd;
            Motor_Diagnostics_t md;
            MPU6050_Snapshot_t snap = {0};
            uint8_t valid = MPU6050_GetSnapshot(&mpu, &snap);
            uint32_t age = valid ?
                (uint32_t)(micros() - snap.completed_us) : 0U;
            last_system_diag_ms = now;
            Control_GetDebugStats(&sd);
            Motor_GetDiagnostics(&md);

            UART_Printf("SYS_T,ms=%lu,ticks=%lu,late=%lu,gap=%lu,maxcy=%lu,encmaxcy=%lu,motmaxcy=%lu,motticks=%lu,valid=%lu,stale=%lu,ir=%lu\r\n",
                        (unsigned long)now, (unsigned long)sd.ticks,
                        (unsigned long)sd.late_gt50us,
                        (unsigned long)sd.gaps_gt1500us,
                        (unsigned long)sd.max_irq_cycles,
                        (unsigned long)sd.max_encoder_cycles,
                        (unsigned long)sd.max_motor_service_cycles,
                        (unsigned long)sd.motor_service_ticks,
                        (unsigned long)sd.gyro_valid_ticks,
                        (unsigned long)sd.gyro_invalid_after_first_valid,
                        (unsigned long)sd.ir_scan_count);
            UART_Printf("SYS_IR,ms=%lu,new=%lu,old=%lu,age_us=%lu,age_max_us=%lu\r\n",
                        (unsigned long)now,
                        (unsigned long)sd.ir_fresh_ticks,
                        (unsigned long)sd.ir_no_new_ticks,
                        (unsigned long)sd.ir_last_age_us,
                        (unsigned long)sd.ir_max_age_us);
            {
                IR_Diagnostics_t ir;
                IR_GetDiagnostics(&ir);
                UART_Printf("SYS_IRD,ms=%lu,start=%lu,done=%lu,fail=%lu,busy=%lu,dma_start=%lu,dma_done=%lu,dma_err=%lu,dma_to=%lu,adc_ovr=%lu,scan_us=%lu,scan_max_us=%lu,dma_us=%lu,dma_max_us=%lu\r\n",
                            (unsigned long)now,
                            (unsigned long)ir.scans_started,
                            (unsigned long)ir.scans_completed,
                            (unsigned long)ir.scans_failed,
                            (unsigned long)ir.start_busy,
                            (unsigned long)ir.dma_started,
                            (unsigned long)ir.dma_completed,
                            (unsigned long)ir.dma_errors,
                            (unsigned long)ir.dma_timeouts,
                            (unsigned long)ir.adc_overruns,
                            (unsigned long)ir.scan_total_us,
                            (unsigned long)ir.scan_max_us,
                            (unsigned long)ir.dma_total_us,
                            (unsigned long)ir.dma_max_us);
                UART_Printf("SYS_IRC,ms=%lu,tim_irq=%lu,tim_cy=%lu,tim_maxcy=%lu,dma_irq=%lu,dma_cy=%lu,dma_maxcy=%lu,adc_clk=%lu\r\n",
                            (unsigned long)now,
                            (unsigned long)ir.tim10_irq_count,
                            (unsigned long)ir.tim10_irq_total_cycles,
                            (unsigned long)ir.tim10_irq_max_cycles,
                            (unsigned long)ir.dma_irq_count,
                            (unsigned long)ir.dma_irq_total_cycles,
                            (unsigned long)ir.dma_irq_max_cycles,
                            (unsigned long)(Board_GetAPB2ClockHz() / 4U));
            }
            UART_Printf("SYS_M,ms=%lu,cal=%u,n=%u,seq=%lu,age=%lu,i2cok=%lu,i2cerr=%lu,to=%lu,nr=%lu,busy=%lu,locked=%u,wd=%u\r\n",
                        (unsigned long)now,
                        (unsigned)mpu.cal_state,
                        (unsigned)mpu.calibration_count,
                        (unsigned long)snap.sequence,
                        (unsigned long)age,
                        (unsigned long)mpu.fresh_gyro_reads,
                        (unsigned long)mpu.error_reads,
                        (unsigned long)mpu.timeout_reads,
                        (unsigned long)mpu.status_not_ready_reads,
                        (unsigned long)mpu.busy_retries,
                        (unsigned)md.outputs_locked,
                        (unsigned)md.command_timeout_latched);
            UART_Printf("SYS_U,ms=%lu,tx_drop=%lu,dma_err=%lu,rx_err=%lu,rx_ov=%lu,trim=%lu,clkfault=%lu,iclk=%lu\r\n",
                        (unsigned long)now,
                        (unsigned long)UART_GetDroppedMessages(),
                        (unsigned long)UART_GetDmaErrors(),
                        (unsigned long)UART_GetRxErrors(),
                        (unsigned long)UART_GetRxOverflow(),
                        (unsigned long)Board_GetHsiTrim(),
                        (unsigned long)Board_GetClockFault(),
                        (unsigned long)I2C_GetConfiguredSclHz());
#if defined(ENCODER_PROFILE_DIAG)
            /* Report retained MPU init probe result once per second.
             * For a 300k fallback, reveals whether 400k failed address ACK,
             * WHO_AM_I read, or WHO_AM_I value. Observation only. */
            UART_Printf("SYS_IPB,ms=%lu,idx=%u,mode=%u,a0=%u,w0=%u,id0=%u,a1=%u,w1=%u,id1=%u,a2=%u,w2=%u,id2=%u,a3=%u,w3=%u,id3=%u\r\n",
                        (unsigned long)now,
                        (unsigned)mpu.selected_speed_index,
                        (unsigned)mpu.selected_speed_mode,
                        (unsigned)mpu.probe_address_status[0],
                        (unsigned)mpu.probe_who_status[0],
                        (unsigned)mpu.probe_who_value[0],
                        (unsigned)mpu.probe_address_status[1],
                        (unsigned)mpu.probe_who_status[1],
                        (unsigned)mpu.probe_who_value[1],
                        (unsigned)mpu.probe_address_status[2],
                        (unsigned)mpu.probe_who_status[2],
                        (unsigned)mpu.probe_who_value[2],
                        (unsigned)mpu.probe_address_status[3],
                        (unsigned)mpu.probe_who_status[3],
                        (unsigned)mpu.probe_who_value[3]);
#endif
            /* Optional UART profiling also runs in Motor OFF/ON diagnostics.
             * The production image has no instrumentation or this log. */
            {
                UART_ProfileStats_t tx;
                UART_GetProfileStats(&tx);
                UART_Printf("SYS_UQ,ms=%lu,q=%lu,peak=%lu,accb=%lu,doneb=%lu,prncy=%lu,dmacy=%lu,rxcy=%lu,rxb=%lu\r\n",
                            (unsigned long)now,
                            (unsigned long)tx.queued_bytes,
                            (unsigned long)tx.queue_peak,
                            (unsigned long)tx.accepted_bytes,
                            (unsigned long)tx.completed_bytes,
                            (unsigned long)tx.printf_cycles,
                            (unsigned long)tx.tx_dma_isr_cycles,
                            (unsigned long)tx.rx_isr_cycles,
                            (unsigned long)tx.rx_bytes);
            }
            {
                int16_t app_l = 0, app_r = 0;
                Motor_GetAppliedPairPermille(&app_l, &app_r);
                UART_Printf("SYS_E,ms=%lu,encL=%ld,encR=%ld,ppsL=%ld,ppsR=%ld,appL=%d,appR=%d\r\n",
                            (unsigned long)now,
                            (long)Encoder_GetLeftCount(),
                            (long)Encoder_GetRightCount(),
                            (long)Encoder_GetLeftSpeedPPS(),
                            (long)Encoder_GetRightSpeedPPS(),
                            (int)app_l, (int)app_r);
            }
            if (mpu.cal_state != last_cal_state) {
                UART_Printf("SYS_CAL,ms=%lu,state=%u,retries=%u,reason=%u\r\n",
                            (unsigned long)now,
                            (unsigned)mpu.cal_state,
                            (unsigned)mpu.cal_retries,
                            (unsigned)mpu.cal_failure_reason);
                last_cal_state = mpu.cal_state;
            }
        }
#if defined(ENCODER_ALPHA_AB_DIAG)
        /* Optional low-rate snapshots. Underlying seven EMA and three
         * boxcar candidates advance at EVERY 1ms TIM11 tick. */
        if ((uint32_t)(now - last_shadow_diag_ms) >= 100U) {
            Encoder_ShadowSnapshot_t snap;
            last_shadow_diag_ms = now;
            Encoder_GetShadowSnapshot(&snap);
            Encoder_LogShadowInstant(now, &snap);
        }
        if (shadow_metric_next < 6U &&
            (uint32_t)(now - shadow_last_emit_ms) >= 25U) {
            Encoder_LogShadowMetric(now, shadow_completed_stage,
                                    shadow_completed_state,
                                    shadow_metric_next, &shadow_completed);
            shadow_last_emit_ms = now;
            ++shadow_metric_next;
        }
#endif
#if defined(ENCODER_PROFILE_DIAG)
        /* Opt-in encoder telemetry at ~50Hz for OFF/ON diagnostic builds.
         * Sample a coherent pair in main,
         * never format UART text inside TIM11 and never touch motor commands.
         * This is decimated observation of TIM11's 1kHz encoder updates. */
        if ((uint32_t)(now - last_encoder_diag_ms) >= 20U) {
            uint32_t primask = __get_PRIMASK();
            int32_t left_count, right_count;
            int32_t left_pps, right_pps;
            last_encoder_diag_ms = now;
            __disable_irq();
            left_count = Encoder_GetLeftCount();
            right_count = Encoder_GetRightCount();
            left_pps = (int32_t)Encoder_GetLeftSpeedPPS();
            right_pps = (int32_t)Encoder_GetRightSpeedPPS();
            if (!primask) __enable_irq();
            UART_Printf("SYS_EN,ms=%lu,l=%ld,r=%ld,vL=%ld,vR=%ld\r\n",
                        (unsigned long)now,
                        (long)left_count, (long)right_count,
                        (long)left_pps, (long)right_pps);
        }
#endif
        if ((uint32_t)(now - last_ir_value_ms) >= 1000U) {
            IR_Data_t ir_snapshot;
            last_ir_value_ms = now;
            Control_GetIRSnapshot(&ir_snapshot);
            if (ir_snapshot.scan_count != 0U &&
                ir_snapshot.scan_count != last_ir_value_scan) {
                IR_Diagnostics_t ir;
                uint32_t age = micros() - ir_snapshot.completed_us;
                IR_GetDiagnostics(&ir);
                if (ir.raw_scan_count == ir_snapshot.scan_count) {
                    UART_Printf("SYS_IRV,ms=%lu,scan=%lu,age_us=%lu,ambient_reads=%lu,pair1_reads=%lu,pair2_reads=%lu,pair3_reads=%lu,a0=%u,a1=%u,a2=%u,a3=%u,a4=%u,a5=%u,l0=%u,l1=%u,l2=%u,l3=%u,l4=%u,l5=%u,d0=%u,d1=%u,d2=%u,d3=%u,d4=%u,d5=%u\r\n",
                                (unsigned long)now,
                                (unsigned long)ir_snapshot.scan_count,
                                (unsigned long)age,
                                (unsigned long)ir.ambient_reads,
                                (unsigned long)ir.pair1_reads,
                                (unsigned long)ir.pair2_reads,
                                (unsigned long)ir.pair3_reads,
                                (unsigned)ir.ambient_raw[0],
                                (unsigned)ir.ambient_raw[1],
                                (unsigned)ir.ambient_raw[2],
                                (unsigned)ir.ambient_raw[3],
                                (unsigned)ir.ambient_raw[4],
                                (unsigned)ir.ambient_raw[5],
                                (unsigned)ir.emitter_raw[0],
                                (unsigned)ir.emitter_raw[1],
                                (unsigned)ir.emitter_raw[2],
                                (unsigned)ir.emitter_raw[3],
                                (unsigned)ir.emitter_raw[4],
                                (unsigned)ir.emitter_raw[5],
                                (unsigned)ir_snapshot.value[0],
                                (unsigned)ir_snapshot.value[1],
                                (unsigned)ir_snapshot.value[2],
                                (unsigned)ir_snapshot.value[3],
                                (unsigned)ir_snapshot.value[4],
                                (unsigned)ir_snapshot.value[5]);
                    last_ir_value_scan = ir_snapshot.scan_count;
                }
            }
        }

#endif

        if ((now - last_debug_ms) >= DEBUG_PERIOD_MS) {
            int32_t left_count;
            int32_t right_count;
            float yaw;
            float gyro;
            float distance;
            float angle_error;
            Motion_State_t state;
            uint32_t primask;

            last_debug_ms = now;

            /* Snapshot shared state quickly; formatting/transmission stays outside ISR. */
            primask = __get_PRIMASK();
            __disable_irq();

            left_count = Encoder_GetLeftCount();
            right_count = Encoder_GetRightCount();
            yaw = MPU6050_GetYaw(&mpu);
            gyro = MPU6050_GetGyroZ(&mpu);
            distance = Motion_GetDistanceMm();
            angle_error = Motion_GetAngleErrorDeg();
            state = Motion_GetState();

            if (!primask) {
                __enable_irq();
            }

            BT_Printf("t=%lu state=%u encL=%ld encR=%ld yaw=%.2f gz=%.2f d=%.1f e=%.2f\r\n",
                      (unsigned long)now,
                      (unsigned int)state,
                      (long)left_count,
                      (long)right_count,
                      yaw,
                      gyro,
                      distance,
                      angle_error);
#if !defined(INTEGRATION_DIAG)
            UART_Printf("t=%lu state=%u encL=%ld encR=%ld yaw=%.2f gz=%.2f dropped=%lu\r\n",
                        (unsigned long)now,(unsigned int)state,(long)left_count,
                        (long)right_count,yaw,gyro,(unsigned long)UART_GetDroppedMessages());
#endif
        }
    }
}
