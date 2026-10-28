/**
 * @file bt_debug.h
 * @brief Bluetooth Debug UART Driver for JDY-33 via USART2 (PA2=TX, PA3=RX)
 * @details TXE interrupt / RXNE interrupt, frame-atomic nonblocking TX.
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

#ifndef BT_BAUDRATE
#define BT_BAUDRATE         115200
#endif
#define BT_TX_BUF_SIZE      4096      /* Large buffer for maze map printing   */

/* Global enable/disable — set to 0 to compile-out all BT traffic */
#define BT_DEBUG_ENABLED    1

typedef struct {
    uint32_t tx_accepted;
    uint32_t tx_uart_dr_bytes; /* DR writes, NOT confirmed BLE delivery. */
    uint32_t tx_drop;
    uint32_t irq_cycles;
    uint32_t irq_count;
    uint32_t rx_bytes;
    uint32_t rx_ore;
    uint32_t rx_fe;
    uint32_t rx_ne;
    uint32_t rx_overflow;
    uint16_t tx_queue;
    uint16_t tx_queue_peak;
} BT_Profile_t;

/* ============================================================================ */
/* API                                                                          */
/* ============================================================================ */

/**
 * @brief Initialize USART2 IRQ TX/RX for Bluetooth debug output
 * @note  Call once during system init, after clock setup
 */
void BT_Init(void);

/**
 * @brief Send a null-terminated string via TXE IRQ (non-blocking)
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
 * @brief Send raw bytes via TXE IRQ (non-blocking)
 * @param data  Pointer to byte array
 * @param len   Number of bytes to send
 */
void BT_SendData(const uint8_t *data, uint16_t len);
/* One main producer only. Returns 0 when the WHOLE frame was rejected. */
uint8_t BT_TrySendData(const uint8_t *data, uint16_t len);
uint8_t BT_ReadByte(uint8_t *out);
void BT_GetProfile(BT_Profile_t *out);
/* Benchmark-only: restart per-test TX queue watermark at current occupancy. */
void BT_ResetTxQueuePeak(void);

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
