#include "uart.h"

#include "board.h"
#include "pinout.h"
#include "stm32f4xx.h"
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

/* USART1 TX only: DMA2 Stream7, Channel 4.
 * The producer runs in main context; DMA completion runs in its IRQ.
 * tx_tail is advanced ONLY when DMA has finished reading the source bytes.
 */
#if defined(INTEGRATION_DIAG)
#define UART_TX_BUFFER_SIZE 2048U
#else
#define UART_TX_BUFFER_SIZE 1024U
#endif
#define UART_RX_BUFFER_SIZE 256U
/* DMA2 HISR/HIFCR flags for Stream7 (STM32F411 RM0383). */
#define UART_DMA7_TCIF  (1UL << 27)
#define UART_DMA7_TEIF  (1UL << 25)
#define UART_DMA7_DMEIF (1UL << 24)
#define UART_DMA7_FEIF  (1UL << 22)
#define UART_DMA7_FATAL (UART_DMA7_TEIF | UART_DMA7_DMEIF)
#define UART_DMA7_ERROR (UART_DMA7_FATAL | UART_DMA7_FEIF)
#define UART_DMA7_FLAGS 0x0F400000UL

static uint8_t tx_buffer[UART_TX_BUFFER_SIZE];
static volatile uint8_t rx_buffer[UART_RX_BUFFER_SIZE];
static volatile uint16_t rx_head, rx_tail;
static volatile uint32_t rx_overflow, rx_errors;
static volatile uint16_t tx_head;  /* Next byte to commit (main context). */
static volatile uint16_t tx_tail;  /* Oldest uncompleted byte (DMA ISR). */
static volatile uint16_t dma_length;
static volatile uint8_t dma_busy;
static volatile uint32_t tx_dropped_messages;
static volatile uint32_t dma_errors;
static volatile uint32_t dma_fe_errors;
static volatile uint32_t dma_te_errors;
static volatile uint32_t dma_dme_errors;
static volatile uint32_t dma_complete_count;
#if defined(INTEGRATION_DIAG)
static volatile uint32_t p_accept_msgs, p_accept_bytes, p_rejected_bytes;
static volatile uint32_t p_completed_bytes, p_queue_peak, p_rx_bytes;
static volatile uint32_t p_printf_calls, p_printf_cycles, p_printf_max_cycles;
static volatile uint32_t p_dma_irq_cycles, p_dma_irq_max;
static volatile uint32_t p_rx_irq_cycles, p_rx_irq_max;
#endif

static uint32_t UART_LockIRQ(void)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    return primask;
}

static void UART_UnlockIRQ(uint32_t primask)
{
    if (primask == 0U) __enable_irq();
}

/* Call with interrupts already disabled, or from the DMA IRQ.
 * Never publish a new tx_tail here; the DMA still owns this memory. */
static void UART_DMA_StartNext(void)
{
    uint16_t length;

    if (dma_busy || tx_head == tx_tail) return;

    /* DMA normal mode auto-disables EN at transfer completion. */
    if (DMA2_Stream7->CR & DMA_SxCR_EN) return;

    length = (tx_head > tx_tail)
           ? (uint16_t)(tx_head - tx_tail)
           : (uint16_t)(UART_TX_BUFFER_SIZE - tx_tail);

    DMA2->HIFCR = UART_DMA7_FLAGS;
    DMA2_Stream7->M0AR = (uint32_t)&tx_buffer[tx_tail];
    DMA2_Stream7->NDTR = length;
    dma_length = length;
    dma_busy = 1U;
    /* ST AN4031 §4.3: stream EN must precede the peripheral DMA request.
     * Keeping USART DMAT enabled while no stream is armed can set FEIF7
     * once per message when USART TXE is already asserted. */
    DMA2_Stream7->CR |= DMA_SxCR_EN;
    USART1->CR3 |= USART_CR3_DMAT;
}

void UART_Init(void)
{
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_DMA2EN;
    RCC->APB2ENR |= RCC_APB2ENR_USART1EN;

    DEBUG_TX_PORT->MODER &= ~(3UL << (DEBUG_TX_PIN * 2U));
    DEBUG_TX_PORT->MODER |=  (2UL << (DEBUG_TX_PIN * 2U));
    DEBUG_RX_PORT->MODER &= ~(3UL << (DEBUG_RX_PIN * 2U));
    DEBUG_RX_PORT->MODER |=  (2UL << (DEBUG_RX_PIN * 2U));

    DEBUG_TX_PORT->OTYPER &= ~(1UL << DEBUG_TX_PIN);
    DEBUG_TX_PORT->OSPEEDR |= 3UL << (DEBUG_TX_PIN * 2U);
    DEBUG_RX_PORT->OSPEEDR |= 3UL << (DEBUG_RX_PIN * 2U);
    DEBUG_TX_PORT->PUPDR &= ~(3UL << (DEBUG_TX_PIN * 2U));
    DEBUG_RX_PORT->PUPDR &= ~(3UL << (DEBUG_RX_PIN * 2U));

    DEBUG_TX_PORT->AFR[1] &= ~(0xFUL << ((DEBUG_TX_PIN - 8U) * 4U));
    DEBUG_TX_PORT->AFR[1] |= (uint32_t)DEBUG_TX_AF << ((DEBUG_TX_PIN - 8U) * 4U);
    DEBUG_RX_PORT->AFR[1] &= ~(0xFUL << ((DEBUG_RX_PIN - 8U) * 4U));
    DEBUG_RX_PORT->AFR[1] |= (uint32_t)DEBUG_RX_AF << ((DEBUG_RX_PIN - 8U) * 4U);

    USART1->CR1 = 0;
    USART1->CR2 = 0;
    USART1->CR3 = 0;
    USART1->BRR = (Board_GetAPB2ClockHz() + UART_BAUDRATE / 2U) / UART_BAUDRATE;
    USART1->CR1 = USART_CR1_TE | USART_CR1_RE | USART_CR1_RXNEIE | USART_CR1_UE;
    rx_head = rx_tail = 0U;
    rx_overflow = rx_errors = 0U;
    NVIC_SetPriority(USART1_IRQn, 8U);
    NVIC_EnableIRQ(USART1_IRQn);

    DMA2_Stream7->CR &= ~DMA_SxCR_EN;
    while (DMA2_Stream7->CR & DMA_SxCR_EN) {
        /* Only during startup; never wait in the 1 kHz ISR. */
    }
    DMA2->HIFCR = UART_DMA7_FLAGS;
    DMA2_Stream7->CR = (4UL << 25) | DMA_SxCR_MINC |
                       DMA_SxCR_DIR_0 | DMA_SxCR_TCIE | DMA_SxCR_TEIE |
                       DMA_SxCR_DMEIE;
    DMA2_Stream7->PAR = (uint32_t)&USART1->DR;
    tx_head = tx_tail = dma_length = 0;
    dma_busy = 0U;
    tx_dropped_messages = dma_errors = 0U;
    dma_fe_errors = dma_te_errors = dma_dme_errors = 0U;
    dma_complete_count = 0U;
#if defined(INTEGRATION_DIAG)
    p_accept_msgs = p_accept_bytes = p_rejected_bytes = 0U;
    p_completed_bytes = p_queue_peak = p_rx_bytes = 0U;
    p_printf_calls = p_printf_cycles = p_printf_max_cycles = 0U;
    p_dma_irq_cycles = p_dma_irq_max = 0U;
    p_rx_irq_cycles = p_rx_irq_max = 0U;
#endif

    /* DMAT stays disabled until UART_DMA_StartNext() arms the DMA stream. */
    NVIC_SetPriority(DMA2_Stream7_IRQn, 7U);
    NVIC_EnableIRQ(DMA2_Stream7_IRQn);
}

/* Nonblocking and main-context-only. Enqueue the entire message or drop
 * it as a unit; never transmit a truncated CSV/log record.
 * The head is published only AFTER the bytes have been copied, so the
 * DMA completion IRQ cannot start reading a half-written message. */
void UART_SendString_DMA(const char *str)
{
    size_t length, i;
    uint16_t head, tail, next_head;
    uint32_t primask;

    if (str == 0) return;
    length = strlen(str);
    if (length == 0U) return;

    head = tx_head;
    tail = tx_tail;
    if (length > (size_t)(UART_TX_BUFFER_SIZE - 1U -
            ((head + UART_TX_BUFFER_SIZE - tail) % UART_TX_BUFFER_SIZE))) {
        tx_dropped_messages++;
#if defined(INTEGRATION_DIAG)
        p_rejected_bytes += (uint32_t)length;
#endif
        return;
    }

    next_head = head;
    for (i = 0; i < length; ++i) {
        tx_buffer[next_head] = (uint8_t)str[i];
        next_head = (uint16_t)((next_head + 1U) % UART_TX_BUFFER_SIZE);
    }

    /* One main producer. IRQ may consume but never writes tx_head. */
    __DMB();
    primask = UART_LockIRQ();
    tx_head = next_head;
#if defined(INTEGRATION_DIAG)
    p_accept_msgs++;
    p_accept_bytes += (uint32_t)length;
    {
        uint16_t used = (uint16_t)((tx_head + UART_TX_BUFFER_SIZE - tx_tail)
                                  % UART_TX_BUFFER_SIZE);
        if (used > p_queue_peak) p_queue_peak = used;
    }
#endif
    UART_DMA_StartNext();
    UART_UnlockIRQ(primask);
}

void UART_SendString(const char *str)
{
    UART_SendString_DMA(str);
}

void UART_Printf(const char *format, ...)
{
    char buffer[256];
    va_list args;
#if defined(INTEGRATION_DIAG)
    uint32_t start_cycles = DWT->CYCCNT;
#endif
    va_start(args, format);
    (void)vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);
    UART_SendString_DMA(buffer);
#if defined(INTEGRATION_DIAG)
    {
        uint32_t elapsed = DWT->CYCCNT - start_cycles;
        p_printf_calls++;
        p_printf_cycles += elapsed;
        if (elapsed > p_printf_max_cycles) p_printf_max_cycles = elapsed;
    }
#endif
}

uint32_t UART_GetDroppedMessages(void)
{
    return tx_dropped_messages;
}

uint32_t UART_GetDmaErrors(void)
{
    return dma_errors;
}

uint32_t UART_GetDmaFifoErrors(void) { return dma_fe_errors; }
uint32_t UART_GetDmaTransferErrors(void) { return dma_te_errors; }
uint32_t UART_GetDmaDirectErrors(void) { return dma_dme_errors; }
uint32_t UART_GetDmaCompleteCount(void) { return dma_complete_count; }

void DMA2_Stream7_IRQHandler(void)
{
#if defined(INTEGRATION_DIAG)
    uint32_t irq_start = DWT->CYCCNT;
#endif
    uint32_t flags = DMA2->HISR & UART_DMA7_FLAGS;
    if (flags == 0U) return;

    DMA2->HIFCR = UART_DMA7_FLAGS;

    /* Count each hardware flag separately, including FEIF + TCIF on the
     * same interrupt. A FIFO flag alone must NOT abort an otherwise
     * successful transfer. */
    if (flags & UART_DMA7_FEIF) dma_fe_errors++;
    if (flags & UART_DMA7_TEIF) dma_te_errors++;
    if (flags & UART_DMA7_DMEIF) dma_dme_errors++;
    if (flags & UART_DMA7_ERROR) dma_errors++;
    if (flags & UART_DMA7_TCIF) dma_complete_count++;

    if (!dma_busy) return;

    if ((flags & UART_DMA7_TCIF) || (flags & UART_DMA7_FATAL)) {
        /* Prevent a pending USART TX request with DMA stream disabled. */
        USART1->CR3 &= ~USART_CR3_DMAT;
        if (flags & UART_DMA7_FATAL) {
            /* Discard this incomplete chunk, then resume queued messages. */
            DMA2_Stream7->CR &= ~DMA_SxCR_EN;
        }

        /* DMA owns the source until completion or an explicit abort. */
#if defined(INTEGRATION_DIAG)
        if (flags & UART_DMA7_TCIF) p_completed_bytes += dma_length;
#endif
        tx_tail = (uint16_t)((tx_tail + dma_length) % UART_TX_BUFFER_SIZE);
        dma_length = 0U;
        dma_busy = 0U;

        /* Re-arm first; only then request data from USART1. */
        UART_DMA_StartNext();
    }
#if defined(INTEGRATION_DIAG)
    {
        uint32_t cycles = DWT->CYCCNT - irq_start;
        p_dma_irq_cycles += cycles;
        if (cycles > p_dma_irq_max) p_dma_irq_max = cycles;
    }
#endif
}

/* PA10 USART1 RXNE interrupt, never processes commands in ISR. */
void USART1_IRQHandler(void)
{
#if defined(INTEGRATION_DIAG)
    uint32_t irq_start = DWT->CYCCNT;
#endif
    uint32_t sr = USART1->SR;
    if (sr & (USART_SR_RXNE | USART_SR_ORE | USART_SR_FE | USART_SR_NE)) {
        uint8_t value = (uint8_t)USART1->DR; /* read clears RXNE/error */
        if (sr & (USART_SR_ORE | USART_SR_FE | USART_SR_NE)) {
            rx_errors++;
        } else if (sr & USART_SR_RXNE) {
            uint16_t next = (uint16_t)((rx_head + 1U) % UART_RX_BUFFER_SIZE);
            if (next == rx_tail) {
                rx_overflow++;
            } else {
                rx_buffer[rx_head] = value;
                rx_head = next;
#if defined(INTEGRATION_DIAG)
                p_rx_bytes++;
#endif
            }
        }
    }
#if defined(INTEGRATION_DIAG)
    {
        uint32_t cycles = DWT->CYCCNT - irq_start;
        p_rx_irq_cycles += cycles;
        if (cycles > p_rx_irq_max) p_rx_irq_max = cycles;
    }
#endif
}

/* Single consumer, main only. ISR owns rx_head. */
uint8_t UART_ReadByte(uint8_t *out)
{
    if (!out || rx_head == rx_tail) return 0U;
    *out = rx_buffer[rx_tail];
    rx_tail = (uint16_t)((rx_tail + 1U) % UART_RX_BUFFER_SIZE);
    return 1U;
}
uint32_t UART_GetRxOverflow(void) { return rx_overflow; }
uint32_t UART_GetRxErrors(void) { return rx_errors; }


#if defined(INTEGRATION_DIAG)
/* Short interrupt-masked atomic counters snapshot, diagnostics only.
 * Keep out of the 1kHz TIM11 ISR. */
void UART_GetProfileStats(UART_ProfileStats_t *out)
{
    uint32_t primask;
    if (!out) return;
    primask = UART_LockIRQ();
    out->accepted_msgs = p_accept_msgs;
    out->accepted_bytes = p_accept_bytes;
    out->rejected_bytes = p_rejected_bytes;
    out->completed_bytes = p_completed_bytes;
    out->dropped_msgs = tx_dropped_messages;
    out->queued_bytes = (uint32_t)((tx_head + UART_TX_BUFFER_SIZE - tx_tail)
                                    % UART_TX_BUFFER_SIZE);
    out->queue_peak = p_queue_peak;
    out->rx_bytes = p_rx_bytes;
    out->rx_errors = rx_errors;
    out->rx_overflow = rx_overflow;
    out->printf_calls = p_printf_calls;
    out->printf_cycles = p_printf_cycles;
    out->printf_max_cycles = p_printf_max_cycles;
    out->tx_dma_isr_cycles = p_dma_irq_cycles;
    out->tx_dma_isr_max_cycles = p_dma_irq_max;
    out->rx_isr_cycles = p_rx_irq_cycles;
    out->rx_isr_max_cycles = p_rx_irq_max;
    out->dma_error_count = dma_errors;
    out->dma_completions = dma_complete_count;
    UART_UnlockIRQ(primask);
}
void UART_ResetProfileQueuePeak(void)
{
    uint32_t primask = UART_LockIRQ();
    p_queue_peak = (uint32_t)((tx_head + UART_TX_BUFFER_SIZE - tx_tail)
                              % UART_TX_BUFFER_SIZE);
    UART_UnlockIRQ(primask);
}
#endif
