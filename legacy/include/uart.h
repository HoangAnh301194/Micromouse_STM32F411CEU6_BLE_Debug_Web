/**
 * @file uart.h
 * @brief Register-level UART Driver for STM32F411 with DMA support
 */

#ifndef __UART_H
#define __UART_H

#include "stm32f4xx.h"
#include <stdio.h>
#include <string.h>

/* UART Configuration */
#define UART_BAUDRATE 115200
#define UART_TX_BUF_SIZE 1024

/* Function Prototypes */

/**
 * @brief Configure GPIO pins for UART1 (PA9=TX, PA10=RX)
 */
void UART_GPIO_Config(void);

/**
 * @brief Initialize USART1 and DMA for transmission
 */
void UART_Init(void);

/**
 * @brief Initialize DMA controller for UART1 TX
 */
void UART_DMA_Init(void);

/**
 * @brief Send a single character (Blocking)
 * @param c Character to send
 */
void UART_SendChar(char c);

/**
 * @brief Send a string (Uses DMA internally)
 * @param str String to send
 */
void UART_SendString(const char *str);

/**
 * @brief Send a string via DMA (Non-blocking)
 * @param str String to send
 */
void UART_SendString_DMA(const char *str);

/**
 * @brief Send a number as a string (Blocking)
 * @param num Number to send
 */
void UART_SendNumber(uint32_t num);

/**
 * @brief Receive a character (Blocking)
 * @return Received character
 */
char UART_ReceiveChar(void);

/**
 * @brief Check if data is available to be read
 * @return 1 if data is available, 0 otherwise
 */
uint8_t UART_Available(void);

#endif /* __UART_H */
