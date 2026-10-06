/**
 * @file tb6612fng.h
 * @brief TB6612FNG Dual Motor Driver Module for STM32F411CEU6
 * @author HELLO12312E
 * @date 2025-01-29
 * 
 * @note STBY pin should be hardwired to VCC (always enabled)
 */

#ifndef TB6612FNG_H
#define TB6612FNG_H

#include <stdint.h>
#include "stm32f4xx.h"  /* Use CMSIS definitions */

/* Motor Selection */
typedef enum {
    MOTOR_A = 0,
    MOTOR_B = 1
} Motor_Select_t;

/* Motor Direction */
typedef enum {
    MOTOR_FORWARD = 0,
    MOTOR_BACKWARD = 1,
    MOTOR_BRAKE = 2,
    MOTOR_STOP = 3
} Motor_Direction_t;

/* Motor Configuration Structure */
typedef struct {
    GPIO_TypeDef *gpio_port;    /* GPIO port for IN1, IN2 */
    uint8_t in1_pin;            /* IN1 pin number (0-15) */
    uint8_t in2_pin;            /* IN2 pin number (0-15) */
    GPIO_TypeDef *pwm_port;     /* GPIO port for PWM pin */
    uint8_t pwm_pin;            /* PWM pin number (0-15) */
    uint8_t pwm_af;             /* Alternate function number for PWM */
    TIM_TypeDef *pwm_timer;     /* Timer for PWM */
    uint8_t pwm_channel;        /* PWM channel (1-4) */
} Motor_Config_t;

/* TB6612FNG Driver Structure */
typedef struct {
    Motor_Config_t motor_a;     /* Motor A configuration */
    Motor_Config_t motor_b;     /* Motor B configuration */
    uint16_t pwm_prescaler;     /* PWM prescaler */
    uint32_t pwm_period;        /* PWM period (ARR value) */
    uint8_t speed_a;            /* Current speed of motor A (0-100%) */
    uint8_t speed_b;            /* Current speed of motor B (0-100%) */
} TB6612_Handle_t;

/* Function Prototypes */

/**
 * @brief Initialize TB6612FNG motor driver
 * @param handle: Pointer to TB6612 handle structure
 * @note STBY pin should be hardwired to VCC on hardware
 */
void TB6612_Init(TB6612_Handle_t *handle);

/**
 * @brief Set motor speed and direction
 * @param handle: Pointer to TB6612 handle
 * @param motor: Motor selection (MOTOR_A or MOTOR_B)
 * @param direction: Motor direction (FORWARD/BACKWARD/BRAKE/STOP)
 * @param speed: Motor speed (0-100%)
 */
void TB6612_SetMotor(TB6612_Handle_t *handle, Motor_Select_t motor, 
                     Motor_Direction_t direction, uint8_t speed);

/**
 * @brief Stop motor (coast mode - IN1=IN2=0)
 * @param handle: Pointer to TB6612 handle
 * @param motor: Motor selection
 */
void TB6612_StopMotor(TB6612_Handle_t *handle, Motor_Select_t motor);

/**
 * @brief Brake motor (short brake - IN1=IN2=1)
 * @param handle: Pointer to TB6612 handle
 * @param motor: Motor selection
 */
void TB6612_BrakeMotor(TB6612_Handle_t *handle, Motor_Select_t motor);

/**
 * @brief Stop all motors
 * @param handle: Pointer to TB6612 handle
 */
void TB6612_StopAll(TB6612_Handle_t *handle);

/**
 * @brief Brake all motors
 * @param handle: Pointer to TB6612 handle
 */
void TB6612_BrakeAll(TB6612_Handle_t *handle);

/**
 * @brief Get current speed of motor
 * @param handle: Pointer to TB6612 handle
 * @param motor: Motor selection
 * @return Current speed (0-100)
 */
uint8_t TB6612_GetSpeed(TB6612_Handle_t *handle, Motor_Select_t motor);

#endif /* TB6612FNG_H */
