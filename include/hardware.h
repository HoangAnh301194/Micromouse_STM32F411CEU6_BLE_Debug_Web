/**
 * @file hardware.h
 * @brief Hardware Abstraction Layer - Updated for PWM Fan/Buzzer
 * @author HoangAnhNguyenHu & Gemini
 * @date 2025-01-30
 * * CHANGES:
 * - Added Hardware_SetAuxPWM() for Fan/Buzzer control.
 * - Updated Button comments (Mode button removed).
 */

#ifndef HARDWARE_H
#define HARDWARE_H

#include <stdint.h>
#include "pinout.h"

/* ============================================================================ */
/* HARDWARE INITIALIZATION                              */
/* ============================================================================ */

void Hardware_InitClocks(void);
void Hardware_InitGPIO(void);
void Hardware_InitSystemTimer(void);

/**
 * @brief Init all hardware (Clocks, GPIO, Timers, Motors, etc.)
 */
void Hardware_Init(void);

/* ============================================================================ */
/* MOTOR CONTROL                                        */
/* ============================================================================ */

void Hardware_InitMotors(void);
void Hardware_SetMotor(uint8_t motor, int8_t speed);
void Hardware_StopMotors(void);

/* ============================================================================ */
/* AUX PWM (FAN / BUZZER) - NEW!                        */
/* ============================================================================ */

/**
 * @brief dieu khien PWM cho chn PB11 (Fan or Buzzer)
 * @param duty: 0 (off) to 4999 (Max speed)
 */
//void Hardware_SetAuxPWM(uint16_t duty);

/* ============================================================================ */
/* ENCODER FUNCTIONS                                    */
/* ============================================================================ */

void Hardware_InitEncoders(void);
int32_t Hardware_GetEncoderCount(uint8_t encoder);
float Hardware_GetEncoderSpeed(uint8_t encoder);
void Hardware_ResetEncoder(uint8_t encoder);
void Hardware_UpdateEncoders(void);

/* ============================================================================ */
/* IR SENSORS                                           */
/* ============================================================================ */

void Hardware_InitIRSensors(void);
void Hardware_IRLedOn(IR_Sensor_t sensor);
void Hardware_IRLedOff(IR_Sensor_t sensor);
void Hardware_IRLedAllOff(void);
uint16_t Hardware_ReadIRSensor(IR_Sensor_t sensor);

/* ============================================================================ */
/* IR SENSORS - DMA VERSION                             */
/* ============================================================================ */

/**
 * @brief Initialize IR sensors v?i DMA support
 * @note G?i sau Hardware_InitIRSensors() (ADC init)
 */
void Hardware_InitIRSensorsDMA(void);

/**
 * @brief Trigger IR scan (non-blocking)
 */
void Hardware_IRSensorStartScan(void);

/**
 * @brief Check if scan completed
 * @return 1 = ready, 0 = scanning
 */
uint8_t Hardware_IRSensorReady(void);

/**
 * @brief Get IR sensor results (compensated)
 * @param results: Array[6] output buffer
 */
void Hardware_IRSensorGetResults(uint16_t *results);

/* ============================================================================ */
/* BUTTONS                                              */
/* ============================================================================ */

void Hardware_InitButtons(void);

/**
 * @brief Read button state
 * @param button: 0=KEY (PA0). (ID 1=MODE is removed)
 * @return 1=pressed, 0=released
 */
uint8_t Hardware_ReadButton(uint8_t button);

/* ============================================================================ */
/* SYSTEM STATUS                                        */
/* ============================================================================ */

float Hardware_GetBatteryVoltage(void);
void Hardware_PrintStatus(void);

#endif /* HARDWARE_H */
