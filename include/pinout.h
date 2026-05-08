/**
 * @file pinout.h
 * @brief Micromouse Robot - Pin Configuration (UPDATED 2025)
 * @author HoangAnhNguyenHuu & Gemini
 * * --- MOTOR MAPPING CHANGES ---
 * Motor A (Right): PWM=PA15, IN1=PB14, IN2=PB15
 * Motor B (Left):  PWM=PB3,  IN1=PB13, IN2=PB12
 */

#ifndef PINOUT_H
#define PINOUT_H

#include <stdint.h>
#include "stm32f4xx.h"

/* ============================================================================ */
/* TIMER & MOTOR SYSTEM (TIM2)                                     */
/* ============================================================================ */

/* Motor A (Right) - PA15 + PB14/PB15 */
#define MOTOR_R_PWM_PORT        GPIOA
#define MOTOR_R_PWM_PIN         15          /* PA15 - TIM2_CH1 */
#define MOTOR_R_PWM_AF          1
#define MOTOR_R_PWM_TIMER       TIM2
#define MOTOR_R_PWM_CHANNEL     1

/* AIN1 */
#define MOTOR_R_IN1_PORT        GPIOB
#define MOTOR_R_IN1_PIN         15          /* PB15 */

/* AIN2 */
#define MOTOR_R_IN2_PORT        GPIOB
#define MOTOR_R_IN2_PIN         14         /* PB14 */

/* Motor B (Left) - PB3 + PB13/PB12 */
#define MOTOR_L_PWM_PORT        GPIOB
#define MOTOR_L_PWM_PIN         3           /* PB3 - TIM2_CH2 (JTAG Pin - Handled in hardware.c) */
#define MOTOR_L_PWM_AF          1
#define MOTOR_L_PWM_TIMER       TIM2
#define MOTOR_L_PWM_CHANNEL     2

/* BIN1 */
#define MOTOR_L_IN1_PORT        GPIOB
#define MOTOR_L_IN1_PIN         13          /* PB13 */

/* BIN2 */
#define MOTOR_L_IN2_PORT        GPIOB
#define MOTOR_L_IN2_PIN         12          /* PB12 */

/* PWM Configuration */
#define MOTOR_PWM_FREQUENCY     20000
#define MOTOR_PWM_PRESCALER     0
#define MOTOR_PWM_PERIOD        4999

/* ============================================================================ */
/* ENCODERS - N20 GA12 (Quadrature)                             */
/* ============================================================================ */

/* Encoder Right - TIM3 (PB4/PB5) */
#define ENCODER_R_CHA_PORT      GPIOB       
#define ENCODER_R_CHA_PIN       4           /* PB4 - TIM3_CH1 */
#define ENCODER_R_CHA_AF        2

#define ENCODER_R_CHB_PORT      GPIOB       
#define ENCODER_R_CHB_PIN       5           /* PB5 - TIM3_CH2 */
#define ENCODER_R_CHB_AF        2

#define ENCODER_R_TIMER         TIM3
#define ENCODER_R_RESOLUTION    ENCODER_PPR

/* Encoder Left - TIM4 (PB6/PB7) */
#define ENCODER_L_CHA_PORT      GPIOB
#define ENCODER_L_CHA_PIN       6           /* PB6 - TIM4_CH1 */
#define ENCODER_L_CHA_AF        2

#define ENCODER_L_CHB_PORT      GPIOB
#define ENCODER_L_CHB_PIN       7           /* PB7 - TIM4_CH2 */
#define ENCODER_L_CHB_AF        2

#define ENCODER_L_TIMER         TIM4
#define ENCODER_L_RESOLUTION    ENCODER_PPR

/* ============================================================================ */
/* IR DISTANCE SENSORS (6 RX + 6 TX)                                    */
/* ============================================================================ */

/* --- IR RECEIVERS (ADC1 Inputs) - Left to Right --- */
/* 1. Left 90 */
#define IR_RX_L90_PORT          GPIOB
#define IR_RX_L90_PIN           1           /* PB1 - ADC1_IN9 */
#define IR_RX_L90_ADC_CHANNEL   9

/* 2. Left 45 */
#define IR_RX_L45_PORT          GPIOB
#define IR_RX_L45_PIN           0           /* PB0 - ADC1_IN8 */
#define IR_RX_L45_ADC_CHANNEL   8

/* 3. Left 0 */
#define IR_RX_L0_PORT           GPIOA       
#define IR_RX_L0_PIN            6           /* PA6 - ADC1_IN6 */
#define IR_RX_L0_ADC_CHANNEL    6

/* 4. Right 0 */
#define IR_RX_R0_PORT           GPIOA       
#define IR_RX_R0_PIN            5           /* PA5 - ADC1_IN5 */
#define IR_RX_R0_ADC_CHANNEL    5           

/* 5. Right 45 */
#define IR_RX_R45_PORT          GPIOA       
#define IR_RX_R45_PIN           4           /* PA4 - ADC1_IN4 */
#define IR_RX_R45_ADC_CHANNEL   4           

/* 6. Right 90 */
#define IR_RX_R90_PORT          GPIOA       
#define IR_RX_R90_PIN           1           /* PA1 - ADC1_IN1 */
#define IR_RX_R90_ADC_CHANNEL   1

/* --- IR EMITTERS (GPIO Output) - Left to Right --- */
/* 1. Left 90 TX */
#define IR_LED_L90_PORT         GPIOA
#define IR_LED_L90_PIN          7           /* PA7 */

/* 2. Left 45 TX */
#define IR_LED_L45_PORT         GPIOB
#define IR_LED_L45_PIN          10          /* PB10 */

/* 3. Left 0 TX */
#define IR_LED_L0_PORT          GPIOB
#define IR_LED_L0_PIN           2           /* PB2 (BOOT1) */

/* 4. Right 0 TX */
#define IR_LED_R0_PORT          GPIOA
#define IR_LED_R0_PIN           8           /* PA8 */

/* 5. Right 45 TX */
#define IR_LED_R45_PORT         GPIOA
#define IR_LED_R45_PIN          11          /* PA11 */

/* 6. Right 90 TX */
#define IR_LED_R90_PORT         GPIOA
#define IR_LED_R90_PIN          12          /* PA12 */

typedef enum {
    IR_L90 = 0, IR_L45 = 1, IR_L0  = 2,
    IR_R0  = 3, IR_R45 = 4, IR_R90 = 5
} IR_Sensor_t;

#define IR_SENSOR_COUNT 6

/* ============================================================================ */
/* I2C & PERIPHERALS                                    */
/* ============================================================================ */

#define I2C_SCL_PORT            GPIOB
#define I2C_SCL_PIN             8           /* PB8 */
#define I2C_SCL_AF              4

#define I2C_SDA_PORT            GPIOB
#define I2C_SDA_PIN             9           /* PB9 */
#define I2C_SDA_AF              4

#define MPU6050_I2C_ADDR        0x68
#define OLED_I2C_ADDR           0x3C

#define DEBUG_TX_PORT           GPIOA
#define DEBUG_TX_PIN            9           /* PA9 */
#define DEBUG_TX_AF             7

#define DEBUG_RX_PORT           GPIOA
#define DEBUG_RX_PIN            10          /* PA10 */
#define DEBUG_RX_AF             7

#define DEBUG_BAUDRATE          115200

/* Bluetooth UART2 — JDY-33 Module */
#define BT_TX_PORT              GPIOA
#define BT_TX_PIN               2           /* PA2 - USART2_TX → JDY-33 RXD */
#define BT_TX_AF                7
#define BT_RX_PORT              GPIOA
#define BT_RX_PIN               3           /* PA3 - USART2_RX → JDY-33 TXD */
#define BT_RX_AF                7

#define BTN_KEY_PORT            GPIOA
#define BTN_KEY_PIN             0           /* PA0 */
#define BUTTON_PRESSED          0
#define BUTTON_RELEASED         1

#define LED_STATUS_PORT         GPIOC
#define LED_STATUS_PIN          13          /* PC13 */

/* ============================================================================ */
/* MACROS                                               */
/* ============================================================================ */

#define GPIO_SET_PIN(port, pin)     ((port)->BSRR = (1UL << (pin)))
#define GPIO_RESET_PIN(port, pin)   ((port)->BSRR = (1UL << ((pin) + 16)))
#define GPIO_TOGGLE_PIN(port, pin)  ((port)->ODR ^= (1UL << (pin)))
#define GPIO_READ_PIN(port, pin)    (((port)->IDR & (1UL << (pin))) ? 1 : 0)

#define IR_LED_ON(port, pin)    GPIO_SET_PIN(port, pin)
#define IR_LED_OFF(port, pin)   GPIO_RESET_PIN(port, pin)
#define READ_BTN_KEY()          GPIO_READ_PIN(BTN_KEY_PORT, BTN_KEY_PIN)

#define LED_ON()                GPIO_RESET_PIN(LED_STATUS_PORT, LED_STATUS_PIN) 
#define LED_OFF()               GPIO_SET_PIN(LED_STATUS_PORT, LED_STATUS_PIN)
#define LED_TOGGLE()            GPIO_TOGGLE_PIN(LED_STATUS_PORT, LED_STATUS_PIN)

/* ============================================================================ */
/* ROBOT PHYSICAL CONSTANTS                                                     */
/* ============================================================================ */

#define WHEEL_DIAMETER_MM       35.0f
#define WHEEL_BASE_MM           84.0f
#define ENCODER_PPR             1430

#endif /* PINOUT_H */
