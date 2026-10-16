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

#include "stm32f4xx.h"

#define DEBUG_PERIOD_MS 100UL

static MPU6050_Handle_t mpu;

static void HandleBleCommand(char c)
{
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
    Encoder_Init();

    /* MPU6050_Init owns RC2 I2C bus probing and initialization. */
    BT_Init();
    IR_Init();

    mpu_status = MPU6050_Init(&mpu);
    if (mpu_status == MPU6050_OK) {
        mpu_status = MPU6050_Calibrate(&mpu); /* Starts async calibration. */
    }

    Motion_Init();

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

    while (1) {
        uint32_t now = millis();
        MPU6050_Update(&mpu); /* Main-context IRQ-driven Policy B. */

        if (BT_Available()) {
            HandleBleCommand(BT_ReceiveChar());
        }
        /* USB-UART debug is READ-ONLY by default: '?' requests status,
         * 'S' allows software STOP. Never arm motion via noisy/accidental
         * PA10 terminal input; motion commands remain on USART2 BLE. */
        {
            uint8_t ch, budget = 16U;
            while (budget-- && UART_ReadByte(&ch)) {
                if (ch == 'S' || ch == 's') Motion_Stop();
                if (ch == '?') {
                    UART_Printf("DBG,rx_overflow=%lu,rx_error=%lu,tx_drop=%lu\r\n",
                        (unsigned long)UART_GetRxOverflow(),
                        (unsigned long)UART_GetRxErrors(),
                        (unsigned long)UART_GetDroppedMessages());
                }
            }
        }

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
