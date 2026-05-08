/**
 * @file bt_debug.h
 * @brief Bluetooth Debug UART Driver for JDY-33 via USART2 (PA2=TX, PA3=RX)
 * @details Non-blocking DMA transmission with circular buffer.
 *          Drop-on-full policy to never stall the main loop.
 *          Printf-style formatted output for convenient debug logging.
 */

#ifndef __BT_DEBUG_H
#define __BT_DEBUG_H

#include "stm32f4xx.h"
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <stdarg.h>

/* ============================================================================ */
/* Configuration                                                                */
/* ============================================================================ */

#define BT_BAUDRATE         115200
#define BT_TX_BUF_SIZE      4096      /* Large buffer for maze map printing   */

/* Global enable/disable — set to 0 to compile-out all BT traffic */
#define BT_DEBUG_ENABLED    1

/* ============================================================================ */
/* API                                                                          */
/* ============================================================================ */

/**
 * @brief Initialize USART2 + DMA1 for Bluetooth debug output
 * @note  Call once during system init, after clock setup
 */
void BT_Init(void);

/**
 * @brief Send a null-terminated string via DMA (non-blocking)
 * @param str  String to transmit
 * @note  Drops excess data if buffer is full — never blocks
 */
void BT_SendString(const char *str);

/**
 * @brief Printf-style formatted debug output over Bluetooth
 * @param fmt  Format string (same syntax as printf)
 * @param ...  Variable arguments
 * @note  Output is truncated at 256 chars per call
 *
 * Example usage:
 *   BT_Printf("Cell(%d,%d) walls=%02X\r\n", x, y, walls);
 */
void BT_Printf(const char *fmt, ...);

/**
 * @brief Send a single character (blocking, for low-level debug only)
 */
void BT_SendChar(char c);

/**
 * @brief Send raw bytes via DMA (non-blocking)
 * @param data  Pointer to byte array
 * @param len   Number of bytes to send
 */
void BT_SendData(const uint8_t *data, uint16_t len);

/**
 * @brief Check if data has been received on USART2
 * @return 1 if data available, 0 otherwise
 */
uint8_t BT_Available(void);

/**
 * @brief Read one received byte (blocking)
 * @return The received character
 */
char BT_ReceiveChar(void);

/* ============================================================================ */
/* Convenience macros                                                           */
/* ============================================================================ */

#if BT_DEBUG_ENABLED
  #define BT_LOG(fmt, ...)    BT_Printf(fmt, ##__VA_ARGS__)
#else
  #define BT_LOG(fmt, ...)    ((void)0)
#endif

#endif /* __BT_DEBUG_H */
