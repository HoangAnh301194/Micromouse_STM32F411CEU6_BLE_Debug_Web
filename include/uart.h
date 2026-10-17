#ifndef UART_H
#define UART_H

#include <stdint.h>

#define UART_BAUDRATE 115200U

void UART_Init(void);
uint8_t UART_ReadByte(uint8_t *out); /* nonblocking main-context RX PA10 */
uint32_t UART_GetRxOverflow(void);
uint32_t UART_GetRxErrors(void);
void UART_SendString(const char *str);
void UART_SendString_DMA(const char *str);
void UART_Printf(const char *format, ...);

/* Diagnostic counters, main-context reads. A full queue drops whole messages. */
uint32_t UART_GetDroppedMessages(void);
uint32_t UART_GetDmaErrors(void);
uint32_t UART_GetDmaFifoErrors(void);
uint32_t UART_GetDmaTransferErrors(void);
uint32_t UART_GetDmaDirectErrors(void);
uint32_t UART_GetDmaCompleteCount(void);

#if defined(UART_PROFILE_DIAG)
/* Diagnostic-only snapshot. No cost in normal production builds.
 * DMA completed bytes means transferred from RAM into USART, NOT proof
 * of corresponding bytes captured correctly by a host serial adapter. */
typedef struct {
    uint32_t accepted_msgs, accepted_bytes, rejected_bytes;
    uint32_t completed_bytes, dropped_msgs;
    uint32_t queued_bytes, queue_peak, rx_bytes, rx_errors, rx_overflow;
    uint32_t printf_calls, printf_cycles, printf_max_cycles;
    uint32_t tx_dma_isr_cycles, tx_dma_isr_max_cycles;
    uint32_t rx_isr_cycles, rx_isr_max_cycles;
    uint32_t dma_error_count, dma_completions;
} UART_ProfileStats_t;
void UART_GetProfileStats(UART_ProfileStats_t *out);
void UART_ResetProfileQueuePeak(void);
#endif

#endif