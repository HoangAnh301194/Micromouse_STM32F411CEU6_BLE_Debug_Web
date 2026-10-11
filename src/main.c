#include "board.h"
#include "bt_debug.h"
#include "control.h"
#include "encoder.h"
#include "motion.h"
#include "motor.h"
#include "motor_test.h"
#include "motor_trace.h"
#include "uart.h"
#include <stdio.h>
#include <string.h>
#include "system_timer.h"

#include "stm32f4xx.h"

/* Old motor + encoder diagnostic harness is intentionally disabled. */
#if 0
#define DEBUG_PERIOD_MS 100UL
#define CONTROL_FREQ_REPORT_US 1000000UL

static char rx_line[80];
static uint8_t rx_len = 0;
static uint32_t last_motor_report_ms = 0;
static int32_t last_motor_left = 0;
static int32_t last_motor_right = 0;
static uint32_t last_motor_elapsed = 0;
static uint8_t last_key_state = 0U;
static uint8_t auto_motor_test_active = 0U;

/* Test harness output: USART2/JDY-33 and USART1/USB-UART. */
static void LogBoth(const char *message)
{
    BT_SendString(message);
    UART_SendString(message);
}

static void StartAutoMotorDebugTest(void)
{
    if (MotorTest_IsRunning() || !Motion_IsDone()) return;

    /* Keep the current KEY test unchanged: both motors at +100% for 1.5 s. */
    if (MotorTest_StartPermille(1000, 1000, 1500U, 200U)) {
        auto_motor_test_active = 1U;
        last_motor_report_ms = millis();
        last_motor_left = Encoder_GetLeftCount();
        last_motor_right = Encoder_GetRightCount();
        last_motor_elapsed = 0U;
        MotorTrace_Reset();
        LogBoth("AUTO,RUN,1000,1000,1500,200\r\n");
    } else {
        LogBoth("AUTO,REJECTED\r\n");
    }
}

static void ReportMotor(const MotorTest_Snapshot_t *s, uint8_t finished)
{
    int32_t left, right, dleft, dright;
    int32_t speed_left_pps = 0, speed_right_pps = 0;
    int16_t applied_l, applied_r;
    uint32_t delta_ms;
    uint32_t primask;
    char line[192];

    primask = __get_PRIMASK();
    __disable_irq();
    left = Encoder_GetLeftCount();
    right = Encoder_GetRightCount();
    if (!primask) __enable_irq();

    Motor_GetAppliedPairPermille(&applied_l, &applied_r);
    delta_ms = s->elapsed_ms - last_motor_elapsed;
    dleft = left - last_motor_left;
    dright = right - last_motor_right;
    if (delta_ms != 0U) {
        speed_left_pps = (int32_t)(((int64_t)dleft * 1000LL) / delta_ms);
        speed_right_pps = (int32_t)(((int64_t)dright * 1000LL) / delta_ms);
    }

    (void)snprintf(line, sizeof(line),
                   "MTDATA3,%lu,%d,%d,%d,%d,%ld,%ld,%ld,%ld,%lu\r\n",
                   (unsigned long)s->elapsed_ms,
                   (int)s->pwm_left, (int)s->pwm_right,
                   (int)applied_l, (int)applied_r,
                   (long)left, (long)right,
                   (long)speed_left_pps, (long)speed_right_pps,
                   (unsigned long)delta_ms);
    LogBoth(line);

    last_motor_left = left;
    last_motor_right = right;
    last_motor_elapsed = s->elapsed_ms;

    if (finished) {
        const char *why = s->reason == MOTOR_TEST_END_TIMEOUT ? "TIMEOUT" :
                          s->reason == MOTOR_TEST_END_HEARTBEAT ? "HEARTBEAT_LOST" :
                          s->reason == MOTOR_TEST_END_BRAKE ? "BRAKE" : "STOP";
        (void)snprintf(line, sizeof(line), "MTEND,%s\r\n", why);
        LogBoth(line);
        MotorTest_AcknowledgeEnd();
    }
}

static void HandleMotorLine(const char *line)
{
    int left, right;
    unsigned long duration;
    unsigned int interval;
    MotorTest_Snapshot_t s;
    if (strcmp(line, "MT TRACE") == 0) {
        if (MotorTest_IsRunning()) {
            BT_SendString("MTERR,TRACE_BUSY\r\n");
        } else {
            MotorTrace_RequestDump();
            BT_SendString("MTACK,TRACE\r\n");
        }
        return;
    }
    if (strcmp(line, "MT BRAKE") == 0) {
        if (!MotorTest_IsRunning()) {
            BT_SendString("MTERR,NOT_RUNNING\r\n");
        } else if (!MotorTest_BrakePulse()) {
            BT_SendString("MTERR,BRAKE_REQUIRES_LOW_SPEED\r\n");
        } else {
            BT_SendString("MTACK,BRAKE\r\n");
        }
        return;
    }
    if (strcmp(line, "MT STOP") == 0) {
        MotorTest_Stop();
        Motion_Stop();
        BT_SendString("MTACK,STOP\r\n");
        return;
    }
    if (strcmp(line, "MT PING") == 0) {
        BT_SendString("MTREADY,3\r\n");
        return;
    }
    if (strcmp(line, "MT HB") == 0) {
        MotorTest_Heartbeat();
        return;
    }
    if (strncmp(line, "MT CFG ", 7) == 0) {
        unsigned int dl, dr, rise, fall;
        if (MotorTest_IsRunning() || !Motion_IsDone() ||
            sscanf(line, "MT CFG %u %u %u %u", &dl, &dr, &rise, &fall) != 4 ||
            dl > 500U || dr > 500U || rise == 0U || rise > 1000U ||
            fall == 0U || fall > 1000U) {
            BT_SendString("MTERR,CONFIG_INVALID_OR_BUSY\r\n");
        } else {
            Motor_SetDeadband((uint16_t)dl, (uint16_t)dr);
            Motor_SetSlewRates((uint16_t)rise, (uint16_t)fall);
            BT_Printf("MTACK,CFG,%u,%u,%u,%u\r\n", dl, dr, rise, fall);
        }
        return;
    }
    if (strncmp(line, "MT SETP ", 8) == 0) {
        int lc, rc;
        if (sscanf(line, "MT SETP %d %d", &lc, &rc) != 2 ||
            lc < -1000 || lc > 1000 || rc < -1000 || rc > 1000 ||
            !MotorTest_SetPermille((int16_t)lc, (int16_t)rc)) {
            BT_SendString("MTERR,SET_INVALID_OR_NOT_RUNNING\r\n");
        } else {
            BT_Printf("MTACK,SETP,%d,%d\r\n", lc, rc);
        }
        return;
    }
    if (strncmp(line, "MT RUNP ", 8) == 0) {
        int lc, rc;
        unsigned long duration_ms;
        unsigned int log_ms;
        MotorTest_GetSnapshot(&s);
        if (sscanf(line, "MT RUNP %d %d %lu %u", &lc, &rc,
                   &duration_ms, &log_ms) != 4 ||
            !Motion_IsDone() || !Control_IsRunning() ||
            s.state != MOTOR_TEST_IDLE ||
            lc < -1000 || lc > 1000 || rc < -1000 || rc > 1000 ||
            log_ms < 100U || log_ms > 1000U ||
            !MotorTest_StartPermille((int16_t)lc, (int16_t)rc,
                                     (uint32_t)duration_ms, (uint16_t)log_ms)) {
            BT_SendString("MTERR,RUN_INVALID_OR_BUSY\r\n");
        } else {
            MotorTrace_Reset();
            last_motor_report_ms = millis();
            last_motor_left = last_motor_right = 0;
            last_motor_elapsed = 0;
            BT_Printf("MTACK,RUNP,%d,%d,%lu,%u\r\n",
                      lc, rc, duration_ms, log_ms);
        }
        return;
    }
    if (sscanf(line, "MT RUN %d %d %lu %u",
               &left, &right, &duration, &interval) == 4) {
        if (!Motion_IsDone() || MotorTest_IsRunning() ||
            !Control_IsRunning() ||
            left < -100 || left > 100 || right < -100 || right > 100 ||
            interval < 100 || interval > 1000 ||
            (left == 0 && right == 0)) {
            BT_SendString("MTERR,INVALID_OR_BUSY\r\n");
            return;
        }
        MotorTest_GetSnapshot(&s);
        if (s.state != MOTOR_TEST_IDLE) {
            BT_SendString("MTERR,BUSY\r\n");
            return;
        }
        if (!MotorTest_Start((int16_t)left, (int16_t)right,
                             (uint32_t)duration, (uint16_t)interval)) {
            BT_SendString("MTERR,REJECTED\r\n");
            return;
        }
        MotorTrace_Reset();
        last_motor_report_ms = millis();
        last_motor_left = last_motor_right = 0;
        last_motor_elapsed = 0;
        BT_Printf("MTACK,RUN,%d,%d,%lu,%u\r\n",
                  left, right, duration, interval);
        return;
    }
    BT_SendString("MTERR,UNKNOWN\r\n");
}

static void ProcessBleByte(char ch)
{
    if (rx_len == 0 && (ch == 'S' || ch == 's')) {
        /* Emergency stop remains a one-byte command (also handled in legacy UI). */
        MotorTest_Stop();
        Motion_Stop();
        rx_len = 0;
        return;
    }
    if (ch == '\r' || ch == '\n') {
        if (rx_len) {
            rx_line[rx_len] = 0;
            if (strncmp(rx_line, "MT ", 3) == 0) {
                HandleMotorLine(rx_line);
            }
            rx_len = 0;
        }
        return;
    }
    if (rx_len < sizeof(rx_line)-1) rx_line[rx_len++] = ch;
    else rx_len = 0;
}

static void PollMotorTelemetry(uint32_t now)
{
    MotorTest_Snapshot_t s;
    MotorTest_GetSnapshot(&s);
    if (s.state == MOTOR_TEST_IDLE) return;
    if (s.state == MOTOR_TEST_ENDED ||
        (uint32_t)(now - last_motor_report_ms) >= s.log_interval_ms) {
        last_motor_report_ms = now;
        ReportMotor(&s, s.state == MOTOR_TEST_ENDED);
    }
}


int main(void)
{
    uint32_t last_debug_ms = 0;
    uint32_t last_control_report_us;
    uint32_t last_control_tick_count;

    /* Dedicated motor/encoder test: IMU, I2C and IR are intentionally off. */
    Board_Init();
    SystemTimer_Init();
    Motor_Init();
    Encoder_Init();
    BT_Init();       /* USART2 PA2/PA3, JDY-33 */
    UART_Init();     /* USART1 PA9/PA10, USB-UART debug */
    Motion_Init();

    Control_Init(0);  /* TIM11 at 1 kHz, no MPU integration in this test. */
    Control_Start();
    last_control_report_us = micros();
    last_control_tick_count = Control_GetTickCount();

    LogBoth("\r\n[BOOT] Motor/encoder test harness ready\r\n");
    LogBoth("KEY: motors +100% for 1500ms; USART1/USART2 telemetry\r\n");

    while (1) {
        uint32_t now = millis();
        uint8_t key_pressed = Board_ButtonPressed();

        /*
         * Debug observed TIM11 rate with TIM5, without UART in the ISR.
         * Keep the KEY motor test and motor timing exactly as before.
         */
        uint32_t control_now_us = micros();
        uint32_t control_elapsed_us = control_now_us - last_control_report_us;
        if (control_elapsed_us >= CONTROL_FREQ_REPORT_US) {
            uint32_t current_ticks = Control_GetTickCount();
            uint32_t delta_ticks = current_ticks - last_control_tick_count;
            uint32_t measured_hz = (uint32_t)(
                ((uint64_t)delta_ticks * 1000000ULL + control_elapsed_us / 2U)
                / control_elapsed_us
            );

            UART_Printf("[CTRL] target=%lu Hz actual=%lu Hz ticks=%lu window_us=%lu tx_drop=%lu dma_err=%lu fe=%lu te=%lu dme=%lu tc=%lu\r\n",
                        (unsigned long)CONTROL_FREQUENCY_HZ,
                        (unsigned long)measured_hz,
                        (unsigned long)delta_ticks,
                        (unsigned long)control_elapsed_us,
                        (unsigned long)UART_GetDroppedMessages(),
                        (unsigned long)UART_GetDmaErrors(),
                        (unsigned long)UART_GetDmaFifoErrors(),
                        (unsigned long)UART_GetDmaTransferErrors(),
                        (unsigned long)UART_GetDmaDirectErrors(),
                        (unsigned long)UART_GetDmaCompleteCount());

            last_control_report_us = control_now_us;
            last_control_tick_count = current_ticks;
        }

        if (key_pressed && !last_key_state) {
            StartAutoMotorDebugTest();
        }
        last_key_state = key_pressed;

        if (auto_motor_test_active) {
            if (MotorTest_IsRunning()) {
                MotorTest_Heartbeat();
            } else {
                auto_motor_test_active = 0U;
            }
        }

        if (BT_Available()) {
            ProcessBleByte(BT_ReceiveChar());
        }
        PollMotorTelemetry(now);
        MotorTrace_Poll(now);


        if (!MotorTest_IsRunning() && (now - last_debug_ms) >= DEBUG_PERIOD_MS) {
            uint32_t primask = __get_PRIMASK();
            int32_t left_count, right_count;
            Motion_State_t state;

            last_debug_ms = now;
            __disable_irq();
            left_count = Encoder_GetLeftCount();
            right_count = Encoder_GetRightCount();
            state = Motion_GetState();
            if (!primask) __enable_irq();

            UART_Printf("t=%lu state=%u encL=%ld encR=%ld\r\n",
                        (unsigned long)now, (unsigned)state,
                        (long)left_count, (long)right_count);
        }
    }
}

#endif /* old motor diagnostic harness */

#include "mpu6050.h"

/* Isolated MPU6050 + I2C1 throughput/latency/error test.
 * No Motor_Init(), Encoder_Init(), MotorTest or motion execution.
 * TIM11 runs at 1kHz in counter-only diagnostic mode.
 */
static MPU6050_Handle_t imu_test;

/* KEY debouncer for requesting a fresh stationary bias calibration. */
static uint8_t Key_TestPress(void)
{
    static uint8_t last_raw = 0U, stable = 0U;
    static uint32_t changed_ms = 0U;
    uint8_t raw = Board_ButtonPressed();
    uint32_t now_ms = millis();
    if (raw != last_raw) { last_raw = raw; changed_ms = now_ms; }
    if ((uint32_t)(now_ms - changed_ms) >= 35UL && stable != raw) {
        stable = raw;
        if (stable) return 1U; /* one event on the pressed edge */
    }
    return 0U;
}



int main(void)
{
    uint32_t last_report_us, last_ticks, last_good;
    uint32_t clk_seq = 0U; /* 1 Hz host-vs-MCU clock diagnostic; motor OFF. */
    uint32_t last_err, last_busy, last_timeout, last_fresh = 0U;
    uint32_t last_status_not_ready = 0U;
    uint32_t max_update_us = 0U;
    uint32_t last_dynamic_log_us = 0U, last_motion_trigger_us = 0U;
    uint8_t dynamic_trace_active = 0U;
    uint32_t last_dynamic_good = 0U;
    uint8_t zeroed_after_cal = 0U;
    uint32_t calibration_done_us = 0U;
    uint8_t cal_eval30_printed = 0U;
    uint8_t cal_eval120_printed = 0U;
    MPU6050_Status init_status;

    Board_Init();
    SystemTimer_Init();
    UART_Init(); /* USART1 PA9: USB-TTL 115200, no BLE required. */

    UART_Printf("[BOOT] Isolated MPU6050/I2C test (motors disabled)\r\n");
#if defined(CLOCK_HSI_TRIM)
    UART_Printf("CLOCK_TRIM,mode=override requested=%u active=%lu hsical=%lu rcc_cr=0x%08lX\r\n",
                (unsigned)CLOCK_HSI_TRIM,
                (unsigned long)Board_GetHsiTrim(),
                (unsigned long)Board_GetHsiCal(),
                (unsigned long)RCC->CR);
#else
    UART_Printf("CLOCK_TRIM,mode=default requested=16 active=%lu hsical=%lu rcc_cr=0x%08lX\r\n",
                (unsigned long)Board_GetHsiTrim(),
                (unsigned long)Board_GetHsiCal(),
                (unsigned long)RCC->CR);
#endif
    UART_Printf("CLOCK,sysclk=%lu hsi_pll100=%u apb1=%lu apb2=%lu tim5_us_clock=1000000\r\n",
                (unsigned long)SystemCoreClock,
                (unsigned)(SystemCoreClock == 100000000UL),
                (unsigned long)Board_GetAPB1ClockHz(),
                (unsigned long)Board_GetAPB2ClockHz());
    UART_Printf("CLOCK_DIAG,fault=%lu rcc_cr=0x%08lX cfgr=0x%08lX pllcfgr=0x%08lX pwr_cr=0x%08lX pwr_csr=0x%08lX flash_acr=0x%08lX\r\n",
                (unsigned long)Board_GetClockFault(),
                (unsigned long)RCC->CR,
                (unsigned long)RCC->CFGR,
                (unsigned long)RCC->PLLCFGR,
                (unsigned long)PWR->CR,
                (unsigned long)PWR->CSR,
                (unsigned long)FLASH->ACR);

    init_status = MPU6050_Init(&imu_test);
    UART_Printf(
        "I2C_PROBE,400_a=%u 400_w=%u 400_id=0x%02X "
        "300_a=%u 300_w=%u 300_id=0x%02X "
        "200_a=%u 200_w=%u 200_id=0x%02X "
        "100_a=%u 100_w=%u 100_id=0x%02X "
        "selected=%u selected_scl=%lu\r\n",
        (unsigned)imu_test.probe_address_status[0],
        (unsigned)imu_test.probe_who_status[0],
        (unsigned)imu_test.probe_who_value[0],
        (unsigned)imu_test.probe_address_status[1],
        (unsigned)imu_test.probe_who_status[1],
        (unsigned)imu_test.probe_who_value[1],
        (unsigned)imu_test.probe_address_status[2],
        (unsigned)imu_test.probe_who_status[2],
        (unsigned)imu_test.probe_who_value[2],
        (unsigned)imu_test.probe_address_status[3],
        (unsigned)imu_test.probe_who_status[3],
        (unsigned)imu_test.probe_who_value[3],
        (unsigned)imu_test.selected_speed_index,
        (unsigned long)I2C_GetConfiguredSclHz());
    UART_Printf("I2C_LINES,scl_pb8=%u sda_pb9=%u idr=0x%08lX\r\n",
        (unsigned)((GPIOB->IDR >> 8) & 1U),
        (unsigned)((GPIOB->IDR >> 9) & 1U),
        (unsigned long)GPIOB->IDR);
    if (init_status != MPU6050_OK || Board_GetClockFault() != 0U) {
        UART_Printf("BOOT_FAIL,clock_fault=%lu imu_status=%d init_step=%u reg=0x%02X expect=0x%02X got=0x%02X i2c_status=%u scl_nominal=%lu\r\n",
                    (unsigned long)Board_GetClockFault(),
                    (int)init_status,
                    (unsigned)imu_test.init_step,
                    (unsigned)imu_test.init_register,
                    (unsigned)imu_test.init_expected,
                    (unsigned)imu_test.init_read_value,
                    (unsigned)imu_test.init_i2c_status,
                    (unsigned long)I2C_GetConfiguredSclHz());
        UART_Printf("I2C_DIAG,sr1=0x%08lX sr2=0x%08lX cr1=0x%08lX cr2=0x%08lX\r\n",
                    (unsigned long)I2C1->SR1, (unsigned long)I2C1->SR2,
                    (unsigned long)I2C1->CR1, (unsigned long)I2C1->CR2);
        /* Failure heartbeat: never silently sleep forever. Motors disabled. */
        while (1) {
            Board_StatusLed(1U);
            delay_ms_blocking(400U);
            Board_StatusLed(0U);
            delay_ms_blocking(600U);
            UART_Printf("BOOT_FAIL_ALIVE,clock_fault=%lu imu_step=%u i2c_status=%u\r\n",
                        (unsigned long)Board_GetClockFault(),
                        (unsigned)imu_test.init_step,
                        (unsigned)imu_test.init_i2c_status);
        }
    }

    MPU6050_Calibrate(&imu_test);
    Control_Init(0);
    Control_SetDiagnosticOnly(1U);
    Control_Start();

    UART_Printf("[IMU_READY] whoami=0x68 scl_nominal=%luHz gyro_range=2000dps dlpf=%u sensor_odr=1000Hz\r\n",
                (unsigned long)I2C_GetConfiguredSclHz(),
                (unsigned)imu_test.dlpf_cfg);
    UART_Printf("CAL_HELP,do not move robot during warmup+collection; KEY requests fresh calibration while idle; motors OFF\r\n");
    UART_Printf("DYN_HELP,D,<us>,<raw_corrected_mdps>,<lpf_mdps> at 100Hz for 1.5s after |gyro|>10dps\r\n");
    last_report_us = micros();
    last_ticks = Control_GetTickCount();
    last_good = imu_test.good_reads;
    last_err = last_busy = last_timeout = 0U;

    while (1) {
        uint32_t start_us = micros();
        uint32_t now, elapsed, duration, good_diff, ticks_diff, age_us;
        uint32_t update_us;
        MPU6050_NoiseStats_t noise;
        Control_Timing_t timing;
        MPU6050_Update(&imu_test); /* I2C IRQ-driven, never busy-waits. */
        update_us = micros() - start_us;

        if (Key_TestPress() && !imu_test.calibrating) {
            UART_Printf("CAL_RESTART,old_bias_mdps=%ld\r\n",
                        (long)(imu_test.gyro_z_offset * 1000.0f));
            MPU6050_Calibrate(&imu_test);
            zeroed_after_cal = 0U;
            cal_eval30_printed = 0U;
            cal_eval120_printed = 0U;
            calibration_done_us = 0U;
        }

        if (imu_test.calibrated && !zeroed_after_cal) {
            MPU6050_ResetYaw(&imu_test);
            zeroed_after_cal = 1U;
            calibration_done_us = micros();
            UART_Printf(
                "CAL_DONE,n=%u blocks=%u retries=%u "
                "bias_mdps=%ld previous_bias_mdps=%ld delta_bias_mdps=%ld "
                "max_block_std_mdps=%ld block_mean_span_mdps=%ld\r\n",
                (unsigned)imu_test.calibration_count,
                (unsigned)imu_test.cal_valid_blocks,
                (unsigned)imu_test.cal_retries,
                (long)(imu_test.gyro_z_offset * 1000.0f),
                (long)(imu_test.cal_previous_bias_dps * 1000.0f),
                (long)((imu_test.gyro_z_offset -
                        imu_test.cal_previous_bias_dps) * 1000.0f),
                (long)(imu_test.cal_max_std_dps * 1000.0f),
                (long)(imu_test.cal_last_mean_span_dps * 1000.0f));
            UART_Printf("DRIFT_START,bias_mdps=%ld,dlpf=%u\r\n",
                        (long)(imu_test.gyro_z_offset * 1000.0f),
                        (unsigned)imu_test.dlpf_cfg);
        }

        /* Diagnostic yaw drifts assume an independent stationary robot.
         * Report actual elapsed time; never update bias automatically. */
        if (zeroed_after_cal && calibration_done_us != 0U) {
            uint32_t elapsed_ms = (uint32_t)
                ((micros() - calibration_done_us) / 1000UL);
            uint8_t report30 = (!cal_eval30_printed && elapsed_ms >= 30000UL);
            uint8_t report120 = (!cal_eval120_printed && elapsed_ms >= 120000UL);
            if (report30 || report120) {
                int32_t raw_mdeg =
                    (int32_t)(imu_test.yaw_raw * 1000.0f);
                int32_t lpf_mdeg =
                    (int32_t)(imu_test.yaw_no_deadzone * 1000.0f);
                int32_t raw_mdeg_per_min = (elapsed_ms != 0U) ?
                    (int32_t)(((int64_t)raw_mdeg * 60000LL) /
                              (int64_t)elapsed_ms) : 0;
                UART_Printf("CAL_CHECK,elapsed_ms=%lu raw_drift_mdeg=%ld "
                            "lpf_no_deadzone_mdeg=%ld "
                            "raw_mdeg_per_min=%ld current_bias_mdps=%ld\r\n",
                            (unsigned long)elapsed_ms,
                            (long)raw_mdeg,
                            (long)lpf_mdeg,
                            (long)raw_mdeg_per_min,
                            (long)(imu_test.gyro_z_offset * 1000.0f));
                if (report30) cal_eval30_printed = 1U;
                if (report120) cal_eval120_printed = 1U;
            }
        }

        /* Dynamic response trace. Log 100Hz DURING rotation, never per I2C
         * event, avoiding USART saturation. Captures raw-vs-software-LPF. */
        now = micros();
        if (imu_test.calibrated && imu_test.good_reads != last_dynamic_good) {
            last_dynamic_good = imu_test.good_reads;
            if (imu_test.gyro_z_raw_corrected > 10.0f ||
                imu_test.gyro_z_raw_corrected < -10.0f) {
                last_motion_trigger_us = now;
                dynamic_trace_active = 1U;
            }
            /* Unsigned elapsed-time arithmetic is rollover-safe for 1.5s.
             * The former signed deadline comparison incorrectly enabled
             * D traces after micros() crossed 2^31 (~35m47s uptime). */
            if (dynamic_trace_active &&
                (uint32_t)(now - last_motion_trigger_us) > 1500000UL)
                dynamic_trace_active = 0U;
            if (dynamic_trace_active &&
                (uint32_t)(now - last_dynamic_log_us) >= 10000UL) {
                last_dynamic_log_us = now;
                UART_Printf("D,%lu,%ld,%ld\r\n",
                    (unsigned long)now,
                    (long)(imu_test.gyro_z_raw_corrected * 1000.0f),
                    (long)(imu_test.gyro_z_lpf * 1000.0f));
            }
        }
        if (update_us > max_update_us) max_update_us = update_us;

        now = micros();
        elapsed = now - last_report_us;
        if (elapsed < 1000000UL) {
            /* Do not sleep during the few microseconds needed for a STOP
             * to finish between INT_STATUS and gyro reads. */
            if (!MPU6050_NeedsImmediatePoll(&imu_test)) __WFI();
            continue;
        }

        duration = elapsed;
        good_diff = imu_test.good_reads - last_good;
        ticks_diff = Control_GetTickCount() - last_ticks;
        age_us = imu_test.last_good_us ?
                 (uint32_t)(now - imu_test.last_good_us) : 0xFFFFFFFFUL;

        /* Independent host-side clock test. Timestamp is sampled BEFORE
         * enqueueing reports, so serial transfer delay is mostly a constant
         * offset. No UART writes from ISR, no RX commands, motor stays OFF.
         * TIM5 micros() wraps naturally at 2^32; host unwraps it. */
        ++clk_seq;
        UART_Printf("CLK,seq=%lu,us=%lu,ticks=%lu,tx_drop=%lu,trim=%lu,fault=%lu,sysclk_nominal=%lu\r\n",
                    (unsigned long)clk_seq,
                    (unsigned long)now,
                    (unsigned long)Control_GetTickCount(),
                    (unsigned long)UART_GetDroppedMessages(),
                    (unsigned long)Board_GetHsiTrim(),
                    (unsigned long)Board_GetClockFault(),
                    (unsigned long)SystemCoreClock);

        /* Autotune runs the same IMU/control loop but intentionally only
         * transmits CLK telemetry once per second. Normally there are six
         * 1Hz burst messages plus CAL_DONE/CAL_CHECK, which filled the 1024B
         * DMA TX ring in observed trim20 logs at seq=10 and seq=40.
         * Never discard a real missing CLK or overclock detection. */
#if !defined(CLOCK_AUTOTUNE)
        UART_Printf(
          "IMU,read_hz=%lu good=%lu err=%lu busy=%lu timeout=%lu "
          "age_us=%lu i2c_last_us=%lu i2c_max_us=%lu update_max_us=%lu "
          "cal=%u cal_n=%u bias_mdps=%ld gz_mdps=%ld yaw_mdeg=%ld "
          "ctrl_hz=%lu last_i2c_error=%u uart_drop=%lu uart_dma_err=%lu\r\n",
          (unsigned long)((good_diff * 1000000ULL) / duration),
          (unsigned long)imu_test.good_reads,
          (unsigned long)(imu_test.error_reads - last_err),
          (unsigned long)(imu_test.busy_retries - last_busy),
          (unsigned long)(imu_test.timeout_reads - last_timeout),
          (unsigned long)age_us,
          (unsigned long)imu_test.last_transfer_us,
          (unsigned long)imu_test.max_transfer_us,
          (unsigned long)max_update_us,
          (unsigned)imu_test.calibrated,
          (unsigned)imu_test.calibration_count,
          (long)(imu_test.gyro_z_offset * 1000.0f),
          (long)(MPU6050_GetGyroZ(&imu_test) * 1000.0f),
          (long)(MPU6050_GetYaw(&imu_test) * 1000.0f),
          (unsigned long)((ticks_diff * 1000000ULL) / duration),
          (unsigned)imu_test.last_i2c_error,
          (unsigned long)UART_GetDroppedMessages(),
          (unsigned long)UART_GetDmaErrors()
        );

        /* 1Hz summary computed from every successful 1kHz raw gyro read.
         * 0.001 deg/s units are used to avoid printf float formatting.
         * This is NOT a 1Hz downsample: statistics include all valid reads. */
        MPU6050_GetNoiseWindow(&imu_test, &noise);
        Control_GetTiming(&timing);
        UART_Printf(
          "FRESH,status=%lu ready=%lu not_ready=%lu fresh=%lu stale=%lu "
          "yaw_deadzone_mdeg=%ld yaw_lpf_no_deadzone_mdeg=%ld "
          "yaw_raw_no_deadzone_mdeg=%ld\r\n",
          (unsigned long)imu_test.status_reads,
          (unsigned long)imu_test.status_ready_reads,
          (unsigned long)(imu_test.status_not_ready_reads - last_status_not_ready),
          (unsigned long)(imu_test.fresh_gyro_reads - last_fresh),
          (unsigned long)(imu_test.stale_gyro_reads),
          (long)(imu_test.yaw * 1000.0f),
          (long)(imu_test.yaw_no_deadzone * 1000.0f),
          (long)(imu_test.yaw_raw * 1000.0f));
        UART_Printf(
          "NOISE,n=%lu raw_mean_mdps=%ld raw_std_mdps=%ld "
          "raw_min_mdps=%ld raw_max_mdps=%ld corrected_mean_mdps=%ld "
          "same_code=%lu sample_min_us=%lu sample_max_us=%lu\r\n",
          (unsigned long)noise.n,
          (long)(noise.mean_dps * 1000.0f),
          (long)(noise.std_dps * 1000.0f),
          (long)(noise.min_dps * 1000.0f),
          (long)(noise.max_dps * 1000.0f),
          (long)(noise.mean_corrected_dps * 1000.0f),
          (unsigned long)noise.equal_consecutive_raw,
          (unsigned long)noise.interval_min_us,
          (unsigned long)noise.interval_max_us
        );
        UART_Printf(
          "CAL,state=%u retries=%u fail_reason=%u warmup_elapsed_ms=%lu "
          "accepted=%u/%u valid_blocks=%u/%u "
          "last_std_mdps=%ld last_span_mdps=%ld "
          "block_mean_span_mdps=%ld bias_mdps=%ld\r\n",
          (unsigned)imu_test.cal_state,
          (unsigned)imu_test.cal_retries,
          (unsigned)imu_test.cal_failure_reason,
          (unsigned long)(imu_test.cal_started_ms ?
              (uint32_t)(millis() - imu_test.cal_started_ms) : 0U),
          (unsigned)imu_test.calibration_count,
          (unsigned)CALIBRATION_SAMPLES,
          (unsigned)imu_test.cal_valid_blocks,
          (unsigned)(CALIBRATION_SAMPLES / MPU_CAL_BLOCK_SAMPLES),
          (long)(imu_test.cal_last_std_dps * 1000.0f),
          (long)(imu_test.cal_last_span_dps * 1000.0f),
          (long)(imu_test.cal_last_mean_span_dps * 1000.0f),
          (long)(imu_test.gyro_z_offset * 1000.0f));
        UART_Printf(
          "TIMING,period_min_us=%lu period_max_us=%lu "
          "late_50us=%lu gap_over_1500us=%lu "
          "observed_isr_max_us=%lu max_cycles=%lu core_hz=%lu\r\n",
          (unsigned long)timing.period_min_us,
          (unsigned long)timing.period_max_us,
          (unsigned long)timing.late_over_50us,
          (unsigned long)timing.gap_over_1500us,
          (unsigned long)((timing.core_hz != 0U)
            ? (((uint64_t)timing.exec_max_cycles * 1000000ULL) / timing.core_hz)
            : 0ULL),
          (unsigned long)timing.exec_max_cycles,
          (unsigned long)timing.core_hz
        );

#endif /* !CLOCK_AUTOTUNE: restore full MPU diagnostics */

        last_report_us = now;
        last_good = imu_test.good_reads;
        last_fresh = imu_test.fresh_gyro_reads;
        last_status_not_ready = imu_test.status_not_ready_reads;
        last_err = imu_test.error_reads;
        last_busy = imu_test.busy_retries;
        last_timeout = imu_test.timeout_reads;
        last_ticks = Control_GetTickCount();
        max_update_us = 0U;
    }
}
