#include "uart.h"

#include "board.h"
#include "pinout.h"
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#define UART_TX_BUFFER_SIZE 1024U

static uint8_t tx_buffer[UART_TX_BUFFER_SIZE];
static volatile uint16_t tx_head;
static volatile uint16_t tx_tail;
static volatile uint8_t dma_busy;

static void UART_DMA_StartNext(void)
{
    uint16_t length;

    if (dma_busy || tx_head == tx_tail) {
        return;
    }

    dma_busy = 1U;
    length = tx_head > tx_tail ? (uint16_t)(tx_head - tx_tail) :
                                 (uint16_t)(UART_TX_BUFFER_SIZE - tx_tail);

    DMA2_Stream7->CR &= ~DMA_SxCR_EN;
    while (DMA2_Stream7->CR & DMA_SxCR_EN) {
    }
    DMA2->HIFCR = 0x0F400000UL;
    DMA2_Stream7->M0AR = (uint32_t)&tx_buffer[tx_tail];
    DMA2_Stream7->NDTR = length;
    tx_tail = (uint16_t)((tx_tail + length) % UART_TX_BUFFER_SIZE);
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
    }
    DMA2->HIFCR = 0x0F400000UL;
    DMA2_Stream7->CR = (4UL << 25) | DMA_SxCR_MINC |
                       DMA_SxCR_DIR_0 | DMA_SxCR_TCIE;
    DMA2_Stream7->PAR = (uint32_t)&USART1->DR;
    USART1->CR3 |= USART_CR3_DMAT;
    NVIC_SetPriority(DMA2_Stream7_IRQn, 7U);
    NVIC_EnableIRQ(DMA2_Stream7_IRQn);
}

void UART_SendString_DMA(const char *str)
{
    uint16_t next_head;

    if (str == 0) {
        return;
    }

    while (*str != '\0') {
        next_head = (uint16_t)((tx_head + 1U) % UART_TX_BUFFER_SIZE);
        if (next_head == tx_tail) {
            UART_DMA_StartNext();
            return;
        }
        tx_buffer[tx_head] = (uint8_t)*str++;
        tx_head = next_head;
    }

    UART_DMA_StartNext();
}

void UART_SendString(const char *str)
{
    UART_SendString_DMA(str);
}

void UART_Printf(const char *format, ...)
{
    char buffer[UART_TX_BUFFER_SIZE];
    va_list args;

    va_start(args, format);
    (void)vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);
    UART_SendString_DMA(buffer);
}

void DMA2_Stream7_IRQHandler(void)
{
    if (DMA2->HISR & (1UL << 27)) {
        DMA2->HIFCR = 1UL << 27;
        DMA2_Stream7->CR &= ~DMA_SxCR_EN;
        dma_busy = 0U;
        UART_DMA_StartNext();
    }
}