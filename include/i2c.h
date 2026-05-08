/**
 * @file i2c.h
 * @brief I2C Driver Header - COMPLETE FIXED
 */

#ifndef I2C_H
#define I2C_H

#include "stm32f4xx.h"
#include <stdint.h>

/* ========================================================================== */
/* TYPE DEFINITIONS                                                           */
/* ========================================================================== */

/**
 * @brief I2C Status codes
 */
typedef enum {
    I2C_OK = 0,
    I2C_ERROR,
    I2C_BUSY,
    I2C_TIMEOUT,
    I2C_NACK
} I2C_Status;

/**
 * @brief I2C Instance
 */
typedef enum {
    I2C_1 = 0,
    I2C_2,
    I2C_3
} I2C_Instance;

/**
 * @brief DMA Callback Type
 */
typedef void (*I2C_DMA_Callback_t)(void);

/* ========================================================================== */
/* INITIALIZATION FUNCTIONS                                                   */
/* ========================================================================== */

/**
 * @brief Initialize I2C peripheral
 * @param instance I2C instance (I2C_1, I2C_2, I2C_3)
 * @param fast_mode 0 = Standard mode (100kHz), 1 = Fast mode (400kHz)
 */
void I2C_Init(I2C_Instance instance, uint8_t fast_mode);

/**
 * @brief Initialize DMA for I2C1
 * @param instance I2C instance (only I2C_1 supported)
 */
void I2C_DMA_Init(I2C_Instance instance);

/* ========================================================================== */
/* BLOCKING I/O FUNCTIONS                                                     */
/* ========================================================================== */

/**
 * @brief Write data to I2C device (blocking mode)
 * @param instance I2C instance
 * @param slave_addr 7-bit slave address
 * @param data Pointer to data buffer
 * @param len Number of bytes to write
 * @return I2C_Status
 */
I2C_Status I2C_Write(I2C_Instance instance, uint8_t slave_addr, 
                     uint8_t *data, uint16_t len);

/**
 * @brief Read data from I2C device (blocking mode)
 * @param instance I2C instance
 * @param slave_addr 7-bit slave address
 * @param data Pointer to data buffer
 * @param len Number of bytes to read
 * @return I2C_Status
 */
I2C_Status I2C_Read(I2C_Instance instance, uint8_t slave_addr, 
                    uint8_t *data, uint16_t len);

/**
 * @brief Write single register (blocking mode)
 * @param instance I2C instance
 * @param slave_addr 7-bit slave address
 * @param reg Register address
 * @param value Value to write
 * @return I2C_Status
 */
I2C_Status I2C_WriteRegister(I2C_Instance instance, uint8_t slave_addr, 
                             uint8_t reg, uint8_t value);

/**
 * @brief Read single register (blocking mode)
 * @param instance I2C instance
 * @param slave_addr 7-bit slave address
 * @param reg Register address
 * @param value Pointer to store read value
 * @return I2C_Status
 */
I2C_Status I2C_ReadRegister(I2C_Instance instance, uint8_t slave_addr, 
                            uint8_t reg, uint8_t *value);

/**
 * @brief Read multiple registers (blocking mode)
 * @param instance I2C instance
 * @param slave_addr 7-bit slave address
 * @param reg Starting register address
 * @param data Pointer to data buffer
 * @param len Number of bytes to read
 * @return I2C_Status
 */
I2C_Status I2C_ReadRegisters(I2C_Instance instance, uint8_t slave_addr, 
                             uint8_t reg, uint8_t *data, uint16_t len);

/**
 * @brief Check if device is ready on I2C bus
 * @param instance I2C instance
 * @param slave_addr 7-bit slave address
 * @return I2C_OK if device responded, I2C_NACK if not present
 */
I2C_Status I2C_IsDeviceReady(I2C_Instance instance, uint8_t slave_addr);

/* ========================================================================== */
/* DMA (NON-BLOCKING) FUNCTIONS                                               */
/* ========================================================================== */

/**
 * @brief Write data via DMA (non-blocking, for OLED)
 * @param instance I2C instance (only I2C_1 supported)
 * @param slave_addr 7-bit slave address
 * @param data Pointer to data buffer (must remain valid until transfer completes)
 * @param len Number of bytes to write
 * @return I2C_OK if transfer started, I2C_BUSY if DMA busy
 * @note Function returns immediately, data sent in background
 */
I2C_Status I2C_Write_DMA(I2C_Instance instance, uint8_t slave_addr, 
                         uint8_t *data, uint16_t len);

/**
 * @brief Read register via DMA (non-blocking, for MPU6050)
 * @param I2Cx I2C peripheral pointer (e.g., I2C1)
 * @param addr 7-bit slave address
 * @param reg Register address
 * @param buffer Pointer to buffer (must remain valid until transfer completes)
 * @param count Number of bytes to read
 * @note Function returns immediately, data read in background
 */
void I2C_ReadReg_DMA(I2C_TypeDef *I2Cx, uint8_t addr, uint8_t reg, 
                     uint8_t *buffer, uint16_t count);

/**
 * @brief Register callback for TX DMA completion (OLED)
 * @param callback Function to call when TX DMA completes
 */
void I2C_SetCallback_DMA(I2C_DMA_Callback_t callback);

/**
 * @brief Register callback for RX DMA completion (MPU6050)
 * @param callback Function to call when RX DMA completes
 */
void I2C_SetCallback_DMA_RX(I2C_DMA_Callback_t callback);

/**
 * @brief Check if DMA transfer is busy
 * @param instance I2C instance
 * @return 1 if busy, 0 if idle
 */
uint8_t I2C_IsBusy_DMA(I2C_Instance instance);

#endif /* I2C_H */
