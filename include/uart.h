#ifndef UART_H
#define UART_H

#include <stdint.h>

#define UART_BAUDRATE 115200U

void UART_Init(void);
void UART_SendString(const char *str);
void UART_SendString_DMA(const char *str);
void UART_Printf(const char *format, ...);

/* Diagnostic counters, main-context reads. A full queue drops whole messages. */
uint32_t UART_GetDroppedMessages(void);
uint32_t UART_GetDmaErrors(void);

#endif