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
#define UART_TX_BUFFER_SIZE 1024U
#define UART_DMA7_TCIF  (1UL << 27)
#define UART_DMA7_ERROR ((1UL << 25) | (1UL << 24) | (1UL << 22))
#define UART_DMA7_FLAGS 0x0F400000UL

static uint8_t tx_buffer[UART_TX_BUFFER_SIZE];
static volatile uint16_t tx_head;  /* Next byte to commit (main context). */
static volatile uint16_t tx_tail;  /* Oldest uncompleted byte (DMA ISR). */
static volatile uint16_t dma_length;
static volatile uint8_t dma_busy;
static volatile uint32_t tx_dropped_messages;
static volatile uint32_t dma_errors;

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
    DMA2_Stream7->CR |= DMA_SxCR_EN;
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

    USART1->CR3 |= USART_CR3_DMAT;
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

    va_start(args, format);
    (void)vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);
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

void DMA2_Stream7_IRQHandler(void)
{
    uint32_t flags = DMA2->HISR & UART_DMA7_FLAGS;
    if (flags == 0U) return;

    DMA2->HIFCR = UART_DMA7_FLAGS;

    if (dma_busy && ((flags & UART_DMA7_TCIF) ||
                     (flags & UART_DMA7_ERROR))) {
        DMA2_Stream7->CR &= ~DMA_SxCR_EN;

        /* On a DMA error the partial message is discarded rather than
         * replaying an unknown number of already-transmitted bytes. */
        if (flags & UART_DMA7_ERROR) dma_errors++;

        tx_tail = (uint16_t)((tx_tail + dma_length) % UART_TX_BUFFER_SIZE);
        dma_length = 0U;
        dma_busy = 0U;
        UART_DMA_StartNext();
    }
}
