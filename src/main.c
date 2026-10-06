#include "board.h"
#include "bt_debug.h"
#include "encoder.h"
#include "i2c.h"
#include "ir_sensor.h"
#include "motion.h"
#include "motor.h"
#include "mpu6050.h"
#include "system_timer.h"

#include "stm32f4xx.h"

#define CONTROL_HZ        1000UL
#define CONTROL_DT_S      0.001f
#define DEBUG_PERIOD_MS   100UL

static MPU6050_Handle_t mpu;
static volatile IR_Data_t ir_latest;

static void ControlTimer_Init(void)
{
    uint32_t timer_clock = Board_GetAPB2TimerClockHz();
    uint32_t prescaler = timer_clock / 1000000UL;

    if (prescaler == 0U) prescaler = 1U;

    RCC->APB2ENR |= RCC_APB2ENR_TIM11EN;

    TIM11->CR1 = 0;
    TIM11->PSC = prescaler - 1U;
    TIM11->ARR = (1000000UL / CONTROL_HZ) - 1U;
    TIM11->EGR = TIM_EGR_UG;
    TIM11->SR = 0;
    TIM11->DIER = TIM_DIER_UIE;

    NVIC_SetPriority(TIM1_TRG_COM_TIM11_IRQn, 0);
    NVIC_EnableIRQ(TIM1_TRG_COM_TIM11_IRQn);
}

static void ControlTimer_Start(void)
{
    TIM11->CNT = 0;
    TIM11->SR &= ~TIM_SR_UIF;
    TIM11->CR1 |= TIM_CR1_CEN;
}

static void HandleBleCommand(char c)
{
    float yaw = MPU6050_GetYaw(&mpu);

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

    case 's':
    case 'S':
        Motion_Stop();
        break;

    case 'z':
    case 'Z':
        if (Motion_IsDone()) MPU6050_ResetYaw(&mpu);
        break;

    default:
        break;
    }
}

void TIM1_TRG_COM_TIM11_IRQHandler(void)
{
    IR_Data_t sample;

    if ((TIM11->SR & TIM_SR_UIF) == 0U) return;
    TIM11->SR &= ~TIM_SR_UIF;

    Encoder_Update(CONTROL_DT_S);

    /* MPU is the only active I2C client in the minimal runtime.
       OLED is intentionally excluded from this baseline. */
    MPU6050_Update(&mpu);

    if (IR_GetLatest(&sample)) {
        uint8_t i;
        for (i = 0; i < IR_SENSOR_COUNT; i++) {
            ir_latest.value[i] = sample.value[i];
        }
        ir_latest.scan_count = sample.scan_count;
        IR_StartScan();
    }

    Motion_Update(CONTROL_DT_S,
                  Encoder_GetLeftCount(),
                  Encoder_GetRightCount(),
                  MPU6050_GetYaw(&mpu),
                  MPU6050_GetGyroZ(&mpu));
}

int main(void)
{
    uint32_t last_debug_ms = 0;
    MPU6050_Status mpu_status;

    Board_Init();
    SystemTimer_Init();
    Motor_Init();
    Encoder_Init();

    I2C_Init(I2C_1, 1);
    I2C_DMA_Init(I2C_1);

    BT_Init();
    IR_Init();

    mpu_status = MPU6050_Init(&mpu);
    if (mpu_status == MPU6050_OK) {
        mpu_status = MPU6050_Calibrate(&mpu);
    }

    Motion_Init();

    if (mpu_status != MPU6050_OK) {
        Motor_Stop();
        Board_StatusLed(1);
        BT_SendString("[BOOT] MPU6050 init/calibration failed\r\n");
        while (1) {
        }
    }

    IR_StartScan();
    ControlTimer_Init();
    ControlTimer_Start();

    BT_SendString("\r\n[BOOT] Minimal core ready\r\n");
    BT_SendString("Commands: F=straight 180mm, L=left 90, R=right 90, S=stop, Z=zero yaw\r\n");

    while (1) {
        uint32_t now = millis();

        if (BT_Available()) {
            HandleBleCommand(BT_ReceiveChar());
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
            if (!primask) __enable_irq();

            BT_Printf("t=%lu state=%u encL=%ld encR=%ld yaw=%.2f gz=%.2f d=%.1f e=%.2f\r\n",
                      (unsigned long)now,
                      (unsigned int)state,
                      (long)left_count,
                      (long)right_count,
                      yaw,
                      gyro,
                      distance,
                      angle_error);
        }
    }
}
