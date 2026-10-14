// #include "board.h"
// #include "bt_debug.h"
// #include "control.h"
// #include "encoder.h"
// #include "i2c.h"
// #include "ir_sensor.h"
// #include "motion.h"
// #include "motor.h"
// #include "mpu6050.h"
// #include "system_timer.h"

// #include "stm32f4xx.h"

// #define DEBUG_PERIOD_MS 100UL

// static MPU6050_Handle_t mpu;

// static void HandleBleCommand(char c)
// {
//     float yaw = MPU6050_GetYaw(&mpu);

//     switch (c) {
//     case 'f':
//     case 'F':
//         if (Motion_IsDone()) Motion_CommandStraight(180.0f, yaw);
//         break;

//     case 'l':
//     case 'L':
//         if (Motion_IsDone()) Motion_CommandTurn(90.0f, yaw);
//         break;

//     case 'r':
//     case 'R':
//         if (Motion_IsDone()) Motion_CommandTurn(-90.0f, yaw);
//         break;

//     case 's':
//     case 'S':
//         Motion_Stop();
//         break;

//     case 'z':
//     case 'Z':
//         if (Motion_IsDone()) MPU6050_ResetYaw(&mpu);
//         break;

//     default:
//         break;
//     }
// }

// int main(void)
// {
//     uint32_t last_debug_ms = 0;
//     MPU6050_Status mpu_status;

//     Board_Init();
//     SystemTimer_Init();

//     Motor_Init();
//     Encoder_Init();

//     I2C_Init(I2C_1, 1);
//     I2C_DMA_Init(I2C_1);

//     BT_Init();
//     IR_Init();

//     mpu_status = MPU6050_Init(&mpu);
//     if (mpu_status == MPU6050_OK) {
//         mpu_status = MPU6050_Calibrate(&mpu);
//     }

//     Motion_Init();

//     if (mpu_status != MPU6050_OK) {
//         Motor_Stop();
//         Board_StatusLed(1);
//         BT_SendString("[BOOT] MPU6050 init/calibration failed\r\n");

//         while (1) {
//         }
//     }

//     /*
//      * TIM10 belongs to ir_sensor.c.
//      * TIM11 belongs to control.c.
//      * main.c only performs system orchestration.
//      */
//     IR_StartScan();
//     Control_Init(&mpu);
//     Control_Start();

//     BT_SendString("\r\n[BOOT] Minimal core ready\r\n");
//     BT_SendString("Commands: F=straight 180mm, L=left 90, R=right 90, S=stop, Z=zero yaw\r\n");

//     while (1) {
//         uint32_t now = millis();

//         if (BT_Available()) {
//             HandleBleCommand(BT_ReceiveChar());
//         }

//         if ((now - last_debug_ms) >= DEBUG_PERIOD_MS) {
//             int32_t left_count;
//             int32_t right_count;
//             float yaw;
//             float gyro;
//             float distance;
//             float angle_error;
//             Motion_State_t state;
//             uint32_t primask;

//             last_debug_ms = now;

//             /* Snapshot shared state quickly; formatting/transmission stays outside ISR. */
//             primask = __get_PRIMASK();
//             __disable_irq();

//             left_count = Encoder_GetLeftCount();
//             right_count = Encoder_GetRightCount();
//             yaw = MPU6050_GetYaw(&mpu);
//             gyro = MPU6050_GetGyroZ(&mpu);
//             distance = Motion_GetDistanceMm();
//             angle_error = Motion_GetAngleErrorDeg();
//             state = Motion_GetState();

//             if (!primask) {
//                 __enable_irq();
//             }

//             BT_Printf("t=%lu state=%u encL=%ld encR=%ld yaw=%.2f gz=%.2f d=%.1f e=%.2f\r\n",
//                       (unsigned long)now,
//                       (unsigned int)state,
//                       (long)left_count,
//                       (long)right_count,
//                       yaw,
//                       gyro,
//                       distance,
//                       angle_error);
//         }
//     }
// }


#include "stm32f4xx.h"
#include "uart.h"

#include <stdint.h>
#include <stdio.h>

/* ========================= CONFIGURATION ========================= */

#define LED_EMITTER_PIN       0U       /* PA0: IR emitter */
#define ADC_PIN               1U       /* PA1: ADC1_IN1 */

#define SAMPLE_COUNT          1000U
#define SAMPLE_PERIOD_US      1000U    /* 1 kHz */
#define SETTLE_US             100U
#define ADC_AVERAGE_COUNT     4U

/*
 * Khoảng nghỉ khi GỬI dữ liệu UART, không ảnh hưởng chu kỳ đo 1 ms.
 * UART driver sử dụng buffer DMA 1024 byte và có thể bỏ dữ liệu
 * nếu buffer đầy.
 */
#define SEND_GAP_US           5000U

/* ========================= DATA STORAGE ========================= */

static uint32_t timestamps[SAMPLE_COUNT];
static uint16_t adc_on[SAMPLE_COUNT];
static uint16_t adc_off[SAMPLE_COUNT];

static uint32_t cycles_per_us;

/* ========================= CLOCK CONFIGURATION ========================= */

/*
 * Dùng HSI 16 MHz và PLL để tạo SYSCLK = 100 MHz:
 * PLLM = 16, PLLN = 200, PLLP = 2.
 * APB1 = 50 MHz, APB2 = 100 MHz.
 */
static void SystemClock_Config(void)
{
    /* Bật HSI 16 MHz */
    RCC->CR |= RCC_CR_HSION;

    while ((RCC->CR & RCC_CR_HSIRDY) == 0U)
    {
    }

    /* Tắt PLL trước khi cấu hình */
    RCC->CR &= ~RCC_CR_PLLON;

    while ((RCC->CR & RCC_CR_PLLRDY) != 0U)
    {
    }

    /*
     * PLL source = HSI
     * PLLM = 16
     * PLLN = 200
     * PLLP = 2 (PLLP encoding = 00)
     * PLLQ = 4
     */
    RCC->PLLCFGR =
        (16U << 0U)  |
        (200U << 6U) |
        (0U << 16U)  |
        (4U << 24U);

    /* Flash latency 3 WS, bật prefetch và cache */
    FLASH->ACR =
        FLASH_ACR_ICEN |
        FLASH_ACR_DCEN |
        FLASH_ACR_PRFTEN |
        3U;

    /* AHB /1, APB1 /2, APB2 /1 */
    RCC->CFGR &= ~(RCC_CFGR_HPRE |
                   RCC_CFGR_PPRE1 |
                   RCC_CFGR_PPRE2);

    RCC->CFGR |= RCC_CFGR_PPRE1_DIV2;

    /* Bật PLL */
    RCC->CR |= RCC_CR_PLLON;

    while ((RCC->CR & RCC_CR_PLLRDY) == 0U)
    {
    }

    /* Chuyển SYSCLK sang PLL */
    RCC->CFGR &= ~RCC_CFGR_SW;
    RCC->CFGR |= RCC_CFGR_SW_PLL;

    while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL)
    {
    }

    SystemCoreClockUpdate();

    cycles_per_us = SystemCoreClock / 1000000U;
}

/* ========================= DWT MICROSECOND TIMER ========================= */

static void DWT_Init(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;

    DWT->CYCCNT = 0U;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

static void Delay_us(uint32_t us)
{
    uint32_t start = DWT->CYCCNT;
    uint32_t cycles = us * cycles_per_us;

    while ((uint32_t)(DWT->CYCCNT - start) < cycles)
    {
    }
}

/* ========================= GPIO CONFIGURATION ========================= */

static void GPIO_Init(void)
{
    /* Bật clock GPIOA */
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    (void)RCC->AHB1ENR;

    /* PA0: output điều khiển LED phát IR */
    GPIOA->MODER &= ~(3UL << (LED_EMITTER_PIN * 2U));
    GPIOA->MODER |=  (1UL << (LED_EMITTER_PIN * 2U));

    /* Output push-pull */
    GPIOA->OTYPER &= ~(1UL << LED_EMITTER_PIN);

    /* Tốc độ output */
    GPIOA->OSPEEDR &= ~(3UL << (LED_EMITTER_PIN * 2U));
    GPIOA->OSPEEDR |=  (2UL << (LED_EMITTER_PIN * 2U));

    /* PA1: chế độ analog */
    GPIOA->MODER &= ~(3UL << (ADC_PIN * 2U));
    GPIOA->MODER |=  (3UL << (ADC_PIN * 2U));

    /* PA1 không dùng pull-up/pull-down */
    GPIOA->PUPDR &= ~(3UL << (ADC_PIN * 2U));

    /* Ban đầu tắt LED phát */
    GPIOA->ODR &= ~(1UL << LED_EMITTER_PIN);
}

/* ========================= ADC CONFIGURATION ========================= */

static void ADC_Init(void)
{
    /* Bật clock ADC1 */
    RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;
    (void)RCC->APB2ENR;

    /* ADC clock = APB2 / 4 = 25 MHz */
    ADC->CCR &= ~(3UL << 16U);
    ADC->CCR |=  (1UL << 16U);

    /* Chỉ dùng một conversion trong mỗi lần đo */
    ADC1->SQR1 = 0U;
    ADC1->SQR3 = 1U;   /* Channel 1 = PA1 */

    /* Channel 1: sampling time 480 cycles */
    ADC1->SMPR2 &= ~(7UL << 3U);
    ADC1->SMPR2 |=  (7UL << 3U);

    /* Độ phân giải mặc định 12-bit */
    ADC1->CR1 = 0U;

    /* Software trigger, bật ADC */
    ADC1->CR2 = ADC_CR2_ADON;

    Delay_us(10U);
}

/* ========================= ADC READ ========================= */

static uint16_t ADC_ReadOnce(void)
{
    ADC1->CR2 |= ADC_CR2_SWSTART;

    while ((ADC1->SR & ADC_SR_EOC) == 0U)
    {
    }

    return (uint16_t)(ADC1->DR & 0x0FFFU);
}

static uint16_t ADC_ReadAverage(void)
{
    uint32_t sum = 0U;

    for (uint32_t i = 0U; i < ADC_AVERAGE_COUNT; i++)
    {
        sum += ADC_ReadOnce();
    }

    return (uint16_t)(sum / ADC_AVERAGE_COUNT);
}

/* ========================= MEASUREMENT ========================= */

static void CaptureSamples(void)
{
    uint32_t measurement_start = DWT->CYCCNT;

    for (uint32_t i = 0U; i < SAMPLE_COUNT; i++)
    {
        /*
         * Mốc thời gian đầu mỗi mẫu.
         * Các mẫu được căn theo chu kỳ 1 ms.
         */
        uint32_t sample_start = DWT->CYCCNT;

        timestamps[i] =
            (uint32_t)((sample_start - measurement_start) /
                       cycles_per_us);

        /* Bật LED phát IR, chờ tín hiệu ổn định */
        GPIOA->ODR |= (1UL << LED_EMITTER_PIN);
        Delay_us(SETTLE_US);

        /* Đo ADC khi LED phát đang bật */
        adc_on[i] = ADC_ReadAverage();

        /* Tắt LED phát IR, chờ tín hiệu ổn định */
        GPIOA->ODR &= ~(1UL << LED_EMITTER_PIN);
        Delay_us(SETTLE_US);

        /* Đo ADC khi LED phát đang tắt */
        adc_off[i] = ADC_ReadAverage();

        /*
         * Chờ đến mốc 1 ms tiếp theo.
         * Không chờ thêm nếu một lần đo vượt quá chu kỳ.
         */
        uint32_t target =
            sample_start + SAMPLE_PERIOD_US * cycles_per_us;

        while ((int32_t)(DWT->CYCCNT - target) < 0)
        {
        }
    }

    /* Đảm bảo LED phát tắt sau khi đo */
    GPIOA->ODR &= ~(1UL << LED_EMITTER_PIN);
}

/* ========================= UART CSV OUTPUT ========================= */

static void SendCSV(void)
{
    char buffer[64];

    UART_SendString_DMA(
        "sample_idx,timestamp_us,adc_on,adc_off,signal\r\n"
    );

    /* Chờ header bắt đầu được truyền */
    Delay_us(10000U);

    for (uint32_t i = 0U; i < SAMPLE_COUNT; i++)
    {
        int32_t signal =
            (int32_t)adc_on[i] - (int32_t)adc_off[i];

        (void)snprintf(
            buffer,
            sizeof(buffer),
            "%lu,%lu,%u,%u,%ld\r\n",
            (unsigned long)i,
            (unsigned long)timestamps[i],
            (unsigned int)adc_on[i],
            (unsigned int)adc_off[i],
            (long)signal
        );

        UART_SendString_DMA(buffer);

        /*
         * Giãn thời gian gửi để hạn chế tràn buffer DMA.
         * Khoảng nghỉ này chỉ tác động đến tốc độ truyền dữ liệu.
         */
        Delay_us(SEND_GAP_US);
    }

    /* Cho DMA thời gian truyền phần dữ liệu còn lại */
    Delay_us(20000U);
}

/* ========================= MAIN ========================= */

int main(void)
{
    SystemClock_Config();
    DWT_Init();

    GPIO_Init();
    ADC_Init();

    /* Khởi tạo USART1 + DMA theo driver của repository */
    UART_Init();

    Delay_us(10000U);

    /* Đo xong toàn bộ mẫu rồi mới gửi dữ liệu CSV */
    CaptureSamples();
    SendCSV();

    /* Dừng tại đây sau khi hoàn tất một lượt đo */
    while (1)
    {
    }
}