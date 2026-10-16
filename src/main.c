#include "board.h"
#include "bt_debug.h"
#include "control.h"
#include "encoder.h"
#include "i2c.h"
#include "ir_sensor.h"
#include "motion.h"
#include "motor.h"
#include "mpu6050.h"
#include "system_timer.h"
#include "uart.h"
#include "integration_sweep.h"
#if defined(INTEGRATION_DIAG)
#include "integration_serial.h"
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

int main(void)
{
    uint32_t last_debug_ms = 0;
#if defined(INTEGRATION_DIAG)
    uint32_t last_system_diag_ms = 0U;
    uint8_t last_cal_state = 0U;
    uint32_t last_sweep_stage = 0xFFFFFFFFUL;
    uint8_t last_sweep_state = 0xFFU;
    int32_t last_sweep_enc_l = 0, last_sweep_enc_r = 0;
    uint32_t last_sweep_log_ms = 0U;
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
#endif

    if (mpu_status != MPU6050_OK) {
        Motor_Stop();
        Board_StatusLed(1);
        BT_SendString("[BOOT] MPU6050 init/calibration failed\r\n");

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
            UART_Printf("t=%lu state=%u encL=%ld encR=%ld yaw=%.2f gz=%.2f dropped=%lu\r\n",
                        (unsigned long)now,(unsigned int)state,(long)left_count,
                        (long)right_count,yaw,gyro,(unsigned long)UART_GetDroppedMessages());
        }
    }
}
