/**
 * @file hardware.c
 * @brief Hardware Init - Encoder returns PPS (Pulses Per Second)
 * @date 2025
 * 
 * Encoder speed unit: PPS (Pulses Per Second)
 * Hardware_SetMotor() limits PWM to 0-100 range
 * Proper type conversion for negative speeds
 */
#include "ir_sensor.h"
#include "hardware.h"
#include "tb6612fng.h"
#include "encoder.h"
#include "system_timer.h"

#include <stdio.h>

#define PI                  3.14159265f

static TB6612_Handle_t motor_driver;
static Encoder_Handle_t encoder_right;
static Encoder_Handle_t encoder_left;

void Hardware_InitClocks(void) {
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;
    
    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;
    
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
    RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;
    RCC->APB1ENR |= RCC_APB1ENR_TIM4EN;
    RCC->APB1ENR |= RCC_APB1ENR_TIM5EN;
}

void Hardware_DisableJTAG(void) {
    volatile uint32_t dummy;
    volatile int i;
    
    dummy = RCC->APB2ENR;
    (void)dummy;
    
    GPIOA->MODER &= ~(3UL << (15 * 2));
    GPIOA->MODER |= (1UL << (15 * 2));
    GPIOA->BSRR = (1UL << (15 + 16));
    
    GPIOB->MODER &= ~(3UL << (3 * 2));
    GPIOB->MODER |= (1UL << (3 * 2));
    GPIOB->BSRR = (1UL << (3 + 16));
    
    GPIOB->MODER &= ~(3UL << (4 * 2));
    GPIOB->MODER |= (1UL << (4 * 2));
    GPIOB->BSRR = (1UL << (4 + 16));
    
    for (i = 0; i < 50000; i++);
}

void GPIO_ConfigOutput(GPIO_TypeDef *port, uint8_t pin) {
    port->BSRR = (1UL << (pin + 16));
    
    port->MODER &= ~(3UL << (pin * 2));
    port->MODER |= (1UL << (pin * 2));
    port->OTYPER &= ~(1UL << pin);
    port->OSPEEDR |= (3UL << (pin * 2));
    port->PUPDR &= ~(3UL << (pin * 2));
    
    port->BSRR = (1UL << (pin + 16));
}

void GPIO_ConfigInput(GPIO_TypeDef *port, uint8_t pin, uint8_t pull) {
    port->MODER &= ~(3UL << (pin * 2));
    port->PUPDR &= ~(3UL << (pin * 2));
    if (pull == 1) {
        port->PUPDR |= (1UL << (pin * 2));
    } else if (pull == 2) {
        port->PUPDR |= (2UL << (pin * 2));
    }
}

void GPIO_ConfigAF(GPIO_TypeDef *port, uint8_t pin, uint8_t af) {
    port->MODER &= ~(3UL << (pin * 2));
    port->MODER |= (2UL << (pin * 2));
    port->OTYPER &= ~(1UL << pin);
    port->OSPEEDR |= (3UL << (pin * 2));
    port->PUPDR &= ~(3UL << (pin * 2));
    
    if (pin < 8) {
        port->AFR[0] &= ~(0xFUL << (pin * 4));
        port->AFR[0] |= (af << (pin * 4));
    } else {
        port->AFR[1] &= ~(0xFUL << ((pin - 8) * 4));
        port->AFR[1] |= (af << ((pin - 8) * 4));
    }
}

void GPIO_ConfigAnalog(GPIO_TypeDef *port, uint8_t pin) {
    port->MODER |= (3UL << (pin * 2));
    port->PUPDR &= ~(3UL << (pin * 2));
}

void Hardware_InitGPIO(void) {
    GPIO_ConfigInput(BTN_KEY_PORT, BTN_KEY_PIN, 1);
    
    GPIO_ConfigOutput(LED_STATUS_PORT, LED_STATUS_PIN);
    LED_OFF();

    GPIO_ConfigOutput(IR_LED_L90_PORT, IR_LED_L90_PIN);
    GPIO_ConfigOutput(IR_LED_L45_PORT, IR_LED_L45_PIN);
    GPIO_ConfigOutput(IR_LED_L0_PORT,  IR_LED_L0_PIN);
    
    GPIO_ConfigOutput(IR_LED_R0_PORT,  IR_LED_R0_PIN);
    GPIO_ConfigOutput(IR_LED_R45_PORT, IR_LED_R45_PIN);
    GPIO_ConfigOutput(IR_LED_R90_PORT, IR_LED_R90_PIN);
    
    GPIO_ConfigAnalog(IR_RX_L90_PORT, IR_RX_L90_PIN);
    GPIO_ConfigAnalog(IR_RX_L45_PORT, IR_RX_L45_PIN);
    GPIO_ConfigAnalog(IR_RX_L0_PORT,  IR_RX_L0_PIN);
    
    GPIO_ConfigAnalog(IR_RX_R0_PORT,  IR_RX_R0_PIN);
    GPIO_ConfigAnalog(IR_RX_R45_PORT, IR_RX_R45_PIN);
    GPIO_ConfigAnalog(IR_RX_R90_PORT, IR_RX_R90_PIN);
}

void Hardware_InitSystemTimer(void) {
    SystemTimer_Init();
}

void Hardware_Init(void) {
    volatile int i;
    
    Hardware_InitClocks();
    Hardware_DisableJTAG();
    
    for (i = 0; i < 50000; i++);
    
    Hardware_InitGPIO();
    Hardware_InitSystemTimer();
}

void Hardware_InitMotors(void) {
    motor_driver.motor_a.gpio_port = MOTOR_R_IN1_PORT;
    motor_driver.motor_a.in1_pin = MOTOR_R_IN1_PIN;
    motor_driver.motor_a.in2_pin = MOTOR_R_IN2_PIN;
    motor_driver.motor_a.pwm_port = MOTOR_R_PWM_PORT;
    motor_driver.motor_a.pwm_pin = MOTOR_R_PWM_PIN;
    motor_driver.motor_a.pwm_af = MOTOR_R_PWM_AF;
    motor_driver.motor_a.pwm_timer = MOTOR_R_PWM_TIMER;
    motor_driver.motor_a.pwm_channel = MOTOR_R_PWM_CHANNEL;
    
    motor_driver.motor_b.gpio_port = MOTOR_L_IN1_PORT;
    motor_driver.motor_b.in1_pin = MOTOR_L_IN1_PIN;
    motor_driver.motor_b.in2_pin = MOTOR_L_IN2_PIN;
    motor_driver.motor_b.pwm_port = MOTOR_L_PWM_PORT;
    motor_driver.motor_b.pwm_pin = MOTOR_L_PWM_PIN;
    motor_driver.motor_b.pwm_af = MOTOR_L_PWM_AF;
    motor_driver.motor_b.pwm_timer = MOTOR_L_PWM_TIMER;
    motor_driver.motor_b.pwm_channel = MOTOR_L_PWM_CHANNEL;
    
    motor_driver.pwm_prescaler = MOTOR_PWM_PRESCALER;
    motor_driver.pwm_period = MOTOR_PWM_PERIOD;
    
    TB6612_Init(&motor_driver);
}

void Hardware_SetMotor(uint8_t motor, int8_t speed) {
    Motor_Select_t m;
    Motor_Direction_t dir;
    uint8_t abs_speed;
    
    m = (motor == 0) ? MOTOR_A : MOTOR_B;
    
    if (speed > 100) speed = 100;
    if (speed < -100) speed = -100;
    
    if (speed > 0) {
        dir = MOTOR_FORWARD;
        abs_speed = (uint8_t)speed;
    } else if (speed < 0) {
        dir = MOTOR_BACKWARD;
        abs_speed = (uint8_t)(-speed);
    } else {
        dir = MOTOR_STOP;
        abs_speed = 0;
    }
    
    TB6612_SetMotor(&motor_driver, m, dir, abs_speed);
}

void Hardware_StopMotors(void) {
    TB6612_StopAll(&motor_driver);
}

void Hardware_InitEncoders(void) {
    Encoder_Init(&encoder_right, ENCODER_RIGHT, ENCODER_R_RESOLUTION);
    Encoder_Init(&encoder_left, ENCODER_LEFT, ENCODER_L_RESOLUTION);
}

int32_t Hardware_GetEncoderCount(uint8_t encoder) {
    if (encoder == 0) return Encoder_GetTotalCount(&encoder_right);
    else return Encoder_GetTotalCount(&encoder_left);
}

float Hardware_GetEncoderSpeed(uint8_t encoder) {
    if (encoder == 0) {
        return Encoder_GetSpeedPPS(&encoder_right);
    } else {
        return Encoder_GetSpeedPPS(&encoder_left);
    }
}

void Hardware_ResetEncoder(uint8_t encoder) {
    if (encoder == 0) Encoder_Reset(&encoder_right);
    else Encoder_Reset(&encoder_left);
}

void Hardware_UpdateEncoders(void) {
    Encoder_UpdateSpeed(&encoder_right);
    Encoder_UpdateSpeed(&encoder_left);
}

void Hardware_InitIRSensors(void) {
    IR_Sensor_Init();
}

void Hardware_IRLedOn(IR_Sensor_t sensor) {
    switch(sensor) {
        case IR_L90: IR_LED_ON(IR_LED_L90_PORT, IR_LED_L90_PIN); break;
        case IR_L45: IR_LED_ON(IR_LED_L45_PORT, IR_LED_L45_PIN); break;
        case IR_L0:  IR_LED_ON(IR_LED_L0_PORT,  IR_LED_L0_PIN);  break;
        case IR_R0:  IR_LED_ON(IR_LED_R0_PORT,  IR_LED_R0_PIN);  break;
        case IR_R45: IR_LED_ON(IR_LED_R45_PORT, IR_LED_R45_PIN); break;
        case IR_R90: IR_LED_ON(IR_LED_R90_PORT, IR_LED_R90_PIN); break;
    }
}

void Hardware_IRLedOff(IR_Sensor_t sensor) {
    switch(sensor) {
        case IR_L90: IR_LED_OFF(IR_LED_L90_PORT, IR_LED_L90_PIN); break;
        case IR_L45: IR_LED_OFF(IR_LED_L45_PORT, IR_LED_L45_PIN); break;
        case IR_L0:  IR_LED_OFF(IR_LED_L0_PORT,  IR_LED_L0_PIN);  break;
        case IR_R0:  IR_LED_OFF(IR_LED_R0_PORT,  IR_LED_R0_PIN);  break;
        case IR_R45: IR_LED_OFF(IR_LED_R45_PORT, IR_LED_R45_PIN); break;
        case IR_R90: IR_LED_OFF(IR_LED_R90_PORT, IR_LED_R90_PIN); break;
    }
}

void Hardware_IRLedAllOff(void) {
    IR_LED_OFF(IR_LED_L90_PORT, IR_LED_L90_PIN);
    IR_LED_OFF(IR_LED_L45_PORT, IR_LED_L45_PIN);
    IR_LED_OFF(IR_LED_L0_PORT,  IR_LED_L0_PIN);
    
    IR_LED_OFF(IR_LED_R0_PORT,  IR_LED_R0_PIN);
    IR_LED_OFF(IR_LED_R45_PORT, IR_LED_R45_PIN);
    IR_LED_OFF(IR_LED_R90_PORT, IR_LED_R90_PIN);
}

uint16_t Hardware_ReadIRSensor(IR_Sensor_t sensor) {
    return IR_Sensor_Read(sensor);
}

void Hardware_InitIRSensorsDMA(void) {
    IR_Sensor_Init();
}

void Hardware_IRSensorStartScan(void) {
    IR_Sensor_StartScan();
}

uint8_t Hardware_IRSensorReady(void) {
    return IR_Sensor_IsReady();
}

void Hardware_IRSensorGetResults(uint16_t *results) {
    IR_Sensor_GetResults(results);
}

void Hardware_InitButtons(void) {
}

uint8_t Hardware_ReadButton(uint8_t button) {
    if (button == 0) {
        return (READ_BTN_KEY() == BUTTON_PRESSED) ? 1 : 0;
    }
    return 0;
}

float Hardware_GetBatteryVoltage(void) {
    return 8.2f;
}

void Hardware_PrintStatus(void) {
    char buffer[100];

    
    sprintf(buffer, "Uptime: %lu ms\r\n", millis());

    sprintf(buffer, "Enc R: %ld, %.1f pps\r\n", Hardware_GetEncoderCount(0), Hardware_GetEncoderSpeed(0));

    sprintf(buffer, "Enc L: %ld, %.1f pps\r\n", Hardware_GetEncoderCount(1), Hardware_GetEncoderSpeed(1));

    
}

/* ========================================================================== */
/* SAFETY: HardFault Handler - stops motors on any crash                      */
/* ========================================================================== */

/**
 * @brief HardFault handler overrides the [WEAK] default in startup .s
 * @note On any MCU crash:
 *   1. Force PWM to 0 (direct register write, no function calls)
 *   2. Disable TIM11 (stop PID ISR)
 *   3. Loop forever (requires manual reset)
 * This prevents motors from running uncontrolled after a crash.
 */
void HardFault_Handler(void) {
    /* Force all motor PWM to zero - direct register writes only! */
    TIM2->CCR1 = 0;   /* Right motor PWM = 0 */
    TIM2->CCR2 = 0;   /* Left motor PWM = 0 */
    
    /* Disable TIM11 ISR (PID loop) so it can't restart motors */
    TIM11->CR1 &= ~(1 << 0);
    
    /* Disable TIM10 ISR (IR scanning) */
    TIM10->CR1 &= ~(1 << 0);
    
    /* Infinite loop — MCU is in fault state, needs reset */
    while (1);
}
