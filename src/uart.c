#include "uart.h"

#include "board.h"
#include "pinout.h"
#include "stm32f4xx.h"
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

/* USART1 DMA2 Stream7, Channel 4 TX. Optional interrupt RX for motor pipeline.
 * The producer runs in main context; DMA completion runs in its IRQ.
 * tx_tail is advanced ONLY when DMA has finished reading the source bytes.
 */
#define UART_TX_BUFFER_SIZE 1024U
/* DMA2 HISR/HIFCR flags for Stream7 (STM32F411 RM0383). */
#define UART_DMA7_TCIF  (1UL << 27)
#define UART_DMA7_TEIF  (1UL << 25)
#define UART_DMA7_DMEIF (1UL << 24)
#define UART_DMA7_FEIF  (1UL << 22)
#define UART_DMA7_FATAL (UART_DMA7_TEIF | UART_DMA7_DMEIF)
#define UART_DMA7_ERROR (UART_DMA7_FATAL | UART_DMA7_FEIF)
#define UART_DMA7_FLAGS 0x0F400000UL

static uint8_t tx_buffer[UART_TX_BUFFER_SIZE];
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

#if defined(MOTOR_AUTO_BENCH) || defined(MOTOR_STARTUP_BENCH)
/* Only auto-motor profile enables this USART1 RXNE IRQ ring.
 * 256 bytes is ample for repeated 11-byte heartbeat frames.
 * ISR writer / main reader, single producer-consumer, no interrupt locks. */
#define UART_RX_BUFFER_SIZE 256U
static volatile uint8_t rx_buffer[UART_RX_BUFFER_SIZE];
static volatile uint16_t rx_head, rx_tail;
static volatile uint32_t rx_bytes, rx_overrun, rx_framing;
static volatile uint32_t rx_noise, rx_ring_overflows;
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
    USART1->CR1 = USART_CR1_TE | USART_CR1_UE;
#if defined(MOTOR_AUTO_BENCH) || defined(MOTOR_STARTUP_BENCH)
    /* USART1 RXNE IRQ, scoped only to the explicit motor bench.
     * A polled RX can overrun while main formats telemetry for TX DMA. */
    rx_head = rx_tail = 0U;
    rx_bytes = rx_overrun = rx_framing = 0U;
    rx_noise = rx_ring_overflows = 0U;
    NVIC_SetPriority(USART1_IRQn, 6U); /* below TIM11, above DMA TX */
    NVIC_EnableIRQ(USART1_IRQn);
    USART1->CR1 |= USART_CR1_RE | USART_CR1_RXNEIE;
#endif

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
    int formatted_len;

    va_start(args, format);
    formatted_len = vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);
    /* A truncated record may lose CRLF and merge with the next frame,
     * making a host handshake impossible or telemetry ambiguous.
     * Always drop an overlong record atomically and count the error. */
    if (formatted_len < 0 || formatted_len >= (int)sizeof(buffer)) {
        ++tx_dropped_messages;
        return;
    }
    UART_SendString_DMA(buffer);
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
        tx_tail = (uint16_t)((tx_tail + dma_length) % UART_TX_BUFFER_SIZE);
        dma_length = 0U;
        dma_busy = 0U;

        /* Re-arm first; only then request data from USART1. */
        UART_DMA_StartNext();
    }
}

#if defined(MOTOR_AUTO_BENCH) || defined(MOTOR_STARTUP_BENCH)
/* Independent of UART TX DMA and main-context printf execution. */
void USART1_IRQHandler(void)
{
    uint32_t sr = USART1->SR;
    uint8_t ch;
    uint16_t next;
    if (!(sr & (USART_SR_RXNE | USART_SR_ORE | USART_SR_FE | USART_SR_NE))) return;
    /* DR read after SR clears ORE/FE/NE; consume each byte only once. */
    ch = (uint8_t)USART1->DR;
    if (sr & USART_SR_ORE) ++rx_overrun;
    if (sr & USART_SR_FE) ++rx_framing;
    if (sr & USART_SR_NE) ++rx_noise;
    if (!(sr & USART_SR_RXNE) ||
        (sr & (USART_SR_ORE | USART_SR_FE | USART_SR_NE))) return;
    next = (uint16_t)((rx_head + 1U) & (UART_RX_BUFFER_SIZE - 1U));
    if (next == rx_tail) { ++rx_ring_overflows; return; }
    rx_buffer[rx_head] = ch;
    __DMB();
    rx_head = next;
    ++rx_bytes;
}

uint8_t UART_TryReadByte(uint8_t *out)
{
    if (out == 0 || rx_head == rx_tail) return 0U;
    *out = rx_buffer[rx_tail];
    __DMB();
    rx_tail = (uint16_t)((rx_tail + 1U) & (UART_RX_BUFFER_SIZE - 1U));
    return 1U;
}

uint32_t UART_GetRxByteCount(void) { return rx_bytes; }
uint32_t UART_GetRxOverrunCount(void) { return rx_overrun; }
uint32_t UART_GetRxFramingCount(void) { return rx_framing; }
uint32_t UART_GetRxNoiseCount(void) { return rx_noise; }
uint32_t UART_GetRxRingOverflowCount(void) { return rx_ring_overflows; }
#endif
