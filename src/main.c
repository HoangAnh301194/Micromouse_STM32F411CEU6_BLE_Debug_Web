#include "board.h"
#include "bt_debug.h"
#include "control.h"
#include "encoder.h"
#include "ir_sensor.h"
#include "motion.h"
#include "motor.h"
#include "mpu6050.h"
#include "system_timer.h"
#include "uart.h"
#include "stm32f4xx.h"

#define DEBUG_PERIOD_MS 100UL

static MPU6050_Handle_t mpu;

/* No BLE motor commands without a valid, freshly completed post-calibration
 * gyro sample. USART1 remains read-only: ? and H never arm a motor. */
static uint8_t ImuReadyForCommand(float *yaw)
{
    MPU6050_Snapshot_t snapshot;
    if (!mpu.calibrated || mpu.calibrating ||
        !MPU6050_GetSnapshot(&mpu, &snapshot) ||
        (uint32_t)(micros() - snapshot.completed_us) > 3000U) return 0U;
    if (yaw) *yaw = snapshot.yaw_deg;
    return 1U;
}

static void HandleBleCommand(char c)
{
    float yaw = 0.0f;

    if (c == 's' || c == 'S') { Motion_Stop(); return; }
    if (!ImuReadyForCommand(&yaw)) return;

    switch (c) {
    case 'f':
    case 'F':
        if (Motion_IsDone()) Motion_CommandStraight(180.0f, yaw);
        break;
    case 'l':
    case 'L':
        if (Motion_IsDone()) Motion_CommandTurn(90.0f, yaw);
        break;
    case 'r':
    case 'R':
        if (Motion_IsDone()) Motion_CommandTurn(-90.0f, yaw);
        break;
    case 'z':
    case 'Z':
        if (Motion_IsDone()) MPU6050_ResetYaw(&mpu);
        break;
    default: break;
    }
}

#if defined(UART_DEBUG_RX)
static void HandleUartDebug(void)
{
    uint8_t ch;
    uint8_t budget;
    /* Hard RX budget: never starve main-loop I2C completion/retry handling. */
    for (budget = 0U; budget < 8U && UART_TryReadByte(&ch); ++budget) {
        if (ch == 'H' || ch == 'h') {
            UART_SendString("USART1: ?=status H=help. No motor start over debug UART.\r\n");
        } else if (ch == '?') {
            MPU6050_Snapshot_t snap;
            uint8_t valid = MPU6050_GetSnapshot(&mpu, &snap);
            UART_Printf("STATUS,ms=%lu,cal=%u,imu_valid=%u,imu_age_us=%lu,i2c_errors=%lu,i2c_timeouts=%lu,rx_overrun=%lu,uart_drop=%lu\r\n",
                (unsigned long)millis(), (unsigned int)mpu.calibrated,
                (unsigned int)valid,
                (unsigned long)(valid ? micros() - snap.completed_us : 0UL),
                (unsigned long)mpu.error_reads, (unsigned long)mpu.timeout_reads,
                (unsigned long)UART_GetRxOverrunCount(),
                (unsigned long)UART_GetDroppedMessages());
        }
    }
}
#endif

int main(void)
{
    MPU6050_Status mpu_status;
    uint32_t last_debug_ms = 0UL;
    uint8_t cal_ready_reported = 0U;

    Board_Init();
    SystemTimer_Init();
    UART_Init(); /* USART1 PA9 TX, PA10 RX, TX DMA2 Stream7, RX IRQ */
    Motor_Init();
    Encoder_Init();

    /* MPU6050_Init() owns I2C1 speed negotiation, internally starts I2C.
     * Do NOT enable MPU DMA_RX: RC2 uses I2C1 EV/ER interrupts. */
    BT_Init(); /* USART2 PA2/PA3 independent of USART1 */
    IR_Init();

    mpu_status = MPU6050_Init(&mpu);
    Motion_Init(); /* Forced neutral before scheduler starts. */
    if (mpu_status == MPU6050_OK) {
        /* Nonblocking! 5s warmup + 4000 valid gyro samples happen in main. */
        mpu_status = MPU6050_Calibrate(&mpu);
    }
    if (mpu_status != MPU6050_OK) {
        Motion_Stop();
        Board_StatusLed(1U);
        UART_Printf("[BOOT_FAIL] MPU6050 init=%d step=%u reg=%u i2c=%d\r\n",
            (int)mpu_status, (unsigned int)mpu.init_step,
            (unsigned int)mpu.init_register, (int)mpu.init_i2c_status);
        BT_SendString("[BOOT_FAIL] MPU6050 init failed; motors stopped\r\n");
        while (1) { __WFI(); }
    }

    IR_StartScan();
    Control_Init(&mpu);
    Control_Start();

    UART_SendString("[BOOT] USART1 Debug PA9/PA10 ready; MPU6050 RC2 calibration pending; Motor Rev3 unchanged\r\n");
    UART_SendString("USART1 commands: ?=status H=help (read-only; no motor arm)\r\n");
    BT_SendString("\r\n[BOOT] Core ready; wait for MPU calibration\r\n");
    BT_SendString("BLE commands after IMU ready: F=straight 180mm, L/R=turn 90, S=stop, Z=zero yaw\r\n");

    while (1) {
        uint32_t now;
        /* Must run frequently in MAIN, not in TIM11; I2C transfers are
         * nonblocking IRQ-driven and scheduled from TIM11 tick opportunities. */
        (void)MPU6050_Update(&mpu);
        now = millis();

        if (mpu.calibrated && !cal_ready_reported) {
            cal_ready_reported = 1U;
            UART_SendString("[IMU_READY] MPU6050 calibrated, valid snapshots incoming\r\n");
            BT_SendString("[IMU_READY] MPU6050 calibrated\r\n");
        }

#if defined(UART_DEBUG_RX)
        HandleUartDebug();
#endif
        if (BT_Available()) HandleBleCommand(BT_ReceiveChar());

        if ((uint32_t)(now - last_debug_ms) >= DEBUG_PERIOD_MS) {
            int32_t left_count, right_count;
            float yaw, gyro, distance, angle_error;
            Motion_State_t state;
            MPU6050_Snapshot_t imu_snapshot;
            uint8_t imu_valid;
            uint32_t imu_age_us;
            uint32_t primask;
            last_debug_ms = now;

            primask = __get_PRIMASK();
            __disable_irq();
            left_count = Encoder_GetLeftCount();
            right_count = Encoder_GetRightCount();
            yaw = MPU6050_GetYaw(&mpu);
            gyro = MPU6050_GetGyroZ(&mpu);
            distance = Motion_GetDistanceMm();
            angle_error = Motion_GetAngleErrorDeg();
            state = Motion_GetState();
            if (!primask) __enable_irq();

            imu_valid = MPU6050_GetSnapshot(&mpu, &imu_snapshot);
            imu_age_us = imu_valid ?
                (uint32_t)(micros() - imu_snapshot.completed_us) : 0U;

            /* Integer telemetry avoids relying on embedded printf float
             * support. UART loss and I2C freshness remain distinguishable. */
            UART_Printf("DBG,ms=%lu,state=%u,cal=%u,valid=%u,age_us=%lu,encL=%ld,encR=%ld,yaw_mdeg=%ld,gz_mdps=%ld,uart_drop=%lu\r\n",
                (unsigned long)now, (unsigned int)state,
                (unsigned int)mpu.calibrated, (unsigned int)imu_valid,
                (unsigned long)imu_age_us, (long)left_count, (long)right_count,
                (long)(yaw * 1000.0f), (long)(gyro * 1000.0f),
                (unsigned long)UART_GetDroppedMessages());
            BT_Printf("t=%lu state=%u encL=%ld encR=%ld yaw=%.2f gz=%.2f d=%.1f e=%.2f\r\n",
                (unsigned long)now, (unsigned int)state,
                (long)left_count, (long)right_count, yaw, gyro,
                distance, angle_error);
        }

        /* TIM11 and TIM5 compare IRQs wake idle main; no spinning for the
         * delayed DATA_RDY retry, and no UART logging inside the 1kHz ISR. */
        if (!MPU6050_NeedsImmediatePoll(&mpu)) __WFI();
    }
}
