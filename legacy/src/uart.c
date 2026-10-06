/**
 * @file uart.c
 * @brief Optimized UART Driver with DMA & Circular Buffer for STM32F411
 * @details This driver uses DMA2 Stream 7 Channel 4 for non-blocking 
 * serial transmission, allowing the CPU to focus on PID and other tasks.
 */

#include "uart.h"
#include "bt_debug.h"
#include <stdarg.h>

/* Circular buffer for TX data */
static uint8_t uart_tx_buffer[UART_TX_BUF_SIZE];
static volatile uint16_t tx_head = 0; /* Index where new data is written */
static volatile uint16_t tx_tail = 0; /* Index from where data is sent by DMA */
static volatile uint8_t  dma_busy = 0; /* Flag indicating an active DMA transfer */

/**
 * @brief Configure GPIO PA9 (TX) and PA10 (RX) for USART1
 */
void UART_GPIO_Config(void) {
    /* Enable Clock for GPIOA */
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    
    /* Set PA9 and PA10 to Alternate Function mode */
    GPIOA->MODER &= ~((3UL << 18) | (3UL << 20));
    GPIOA->MODER |= (2UL << 18) | (2UL << 20);
    
    /* Set high speed for serial pins */
    GPIOA->OSPEEDR |= (3UL << 18) | (3UL << 20);
    
    /* Enable pull-ups for stability */
    GPIOA->PUPDR &= ~((3UL << 18) | (3UL << 20));
    GPIOA->PUPDR |= (1UL << 18) | (1UL << 20);
    
    /* Map pins to Alternate Function 7 (USART1) */
    GPIOA->AFR[1] &= ~((0xFUL << 4) | (0xFUL << 8));
    GPIOA->AFR[1] |= (7UL << 4) | (7UL << 8);
}

/**
 * @brief Get current APB2 Clock frequency (USART1 is on APB2)
 * @return Frequency in Hz
 */
static uint32_t Get_APB2_Clock(void) {
    uint32_t SystemCoreClock;
    uint32_t tmp;
    uint32_t pllm, plln, pllp;
    uint32_t ahbprescaler, apb2prescaler;
    
    /* Check current clock source */
    tmp = RCC->CFGR & RCC_CFGR_SWS;
    
    if (tmp == RCC_CFGR_SWS_HSI) {
        SystemCoreClock = 16000000;
    }
    else if (tmp == RCC_CFGR_SWS_HSE) {
        SystemCoreClock = 25000000; /* Assuming 25MHz crystal */
    }
    else if (tmp == RCC_CFGR_SWS_PLL) {
        pllm = RCC->PLLCFGR & RCC_PLLCFGR_PLLM;
        plln = (RCC->PLLCFGR & RCC_PLLCFGR_PLLN) >> 6;
        pllp = (((RCC->PLLCFGR & RCC_PLLCFGR_PLLP) >> 16) + 1) * 2;
        
        if (RCC->PLLCFGR & RCC_PLLCFGR_PLLSRC) {
            SystemCoreClock = (25000000 / pllm) * plln / pllp;
        } else {
            SystemCoreClock = (16000000 / pllm) * plln / pllp;
        }
    }
    else {
        SystemCoreClock = 16000000;
    }
    
    /* Calculate AHB prescaler */
    tmp = (RCC->CFGR & RCC_CFGR_HPRE) >> 4;
    if (tmp & 0x08) {
        ahbprescaler = 1 << ((tmp & 0x07) + 1);
    } else {
        ahbprescaler = 1;
    }
    
    /* Calculate APB2 prescaler */
    tmp = (RCC->CFGR & RCC_CFGR_PPRE2) >> 13;
    if (tmp & 0x04) {
        apb2prescaler = 1 << ((tmp & 0x03) + 1);
    } else {
        apb2prescaler = 1;
    }
    
    return (SystemCoreClock / ahbprescaler / apb2prescaler);
}

/**
 * @brief Initialize UART DMA controller (DMA2, Stream 7, Channel 4)
 */
void UART_DMA_Init(void) {
    /* 1. Enable Clock for DMA2 */
    RCC->AHB1ENR |= RCC_AHB1ENR_DMA2EN;
    
    /* Ensure Stream is disabled before configuration */
    DMA2_Stream7->CR &= ~DMA_SxCR_EN;
    while (DMA2_Stream7->CR & DMA_SxCR_EN);
    
    /* 2. Configure DMA Stream Parameters */
    DMA2_Stream7->CR = 0;
    DMA2_Stream7->CR |= (4UL << 25);     /* Select Channel 4 for USART1 TX */
    DMA2_Stream7->CR |= (1UL << 10);     /* Memory Increment Mode enabled */
    DMA2_Stream7->CR |= (1UL << 6);      /* Data Direction: Memory to Peripheral */
    DMA2_Stream7->CR |= (1UL << 4);      /* Enable Transfer Complete Interrupt */
    
    /* 3. Set Peripheral Address (USART1 Data Register) */
    DMA2_Stream7->PAR = (uint32_t)&(USART1->DR);
    
    /* 4. Configure Interrupt Priority and Enable it */
    NVIC_SetPriority(DMA2_Stream7_IRQn, 7); /* Medium priority level */
    NVIC_EnableIRQ(DMA2_Stream7_IRQn);
    
    /* 5. Allow USART1 to trigger DMA requests */
    USART1->CR3 |= USART_CR3_DMAT;
}

/**
 * @brief Initialize USART1 Peripheral
 */
void UART_Init(void) {
    uint32_t apb2_clock;
    uint32_t brr_value;

    /* Enable USART1 Clock */
    RCC->APB2ENR |= RCC_APB2ENR_USART1EN;
    
    /* Configure GPIO first */
    UART_GPIO_Config();
    
    /* Disable USART for configuration */
    USART1->CR1 &= ~USART_CR1_UE;
    
    /* Calculate BRR based on actual APB2 clock */
    apb2_clock = Get_APB2_Clock();
    brr_value = apb2_clock / UART_BAUDRATE;
    USART1->BRR = brr_value; 
    
    /* Enable Transmitter and Receiver */
    USART1->CR1 |= USART_CR1_TE | USART_CR1_RE;
    
    /* Enable USART */
    USART1->CR1 |= USART_CR1_UE;
    
    /* Initialize DMA subsystem */
    UART_DMA_Init();
}

/**
 * @brief Internal function to initiate the next DMA segment transfer
 */
static void UART_DMA_StartNext(void) {
    uint16_t length;

    /* Don't start if already busy or no data to send */
    if (dma_busy || (tx_head == tx_tail)) return;
    
    dma_busy = 1;
    
    /* Determine the length of the next contiguous block in the circular buffer */
    if (tx_head > tx_tail) {
        length = tx_head - tx_tail;
    } else {
        /* Circular wrap-around: Send from current tail to end of buffer array first */
        length = UART_TX_BUF_SIZE - tx_tail;
    }
    
    /* Reconfigure and start DMA stream */
    DMA2_Stream7->CR &= ~DMA_SxCR_EN;
    while (DMA2_Stream7->CR & DMA_SxCR_EN);
    
    /* Clear all DMA interrupt flags for this stream */
    DMA2->HIFCR = (1UL << 27) | (1UL << 26) | (1UL << 25) | (1UL << 24) | (1UL << 22);
    
    /* Set memory address and data length */
    DMA2_Stream7->M0AR = (uint32_t)&uart_tx_buffer[tx_tail];
    DMA2_Stream7->NDTR = length;
    
    /* Advance tail pointer (wrap around if necessary) */
    tx_tail = (tx_tail + length) % UART_TX_BUF_SIZE;
    
    /* Start transmission */
    DMA2_Stream7->CR |= DMA_SxCR_EN;
}

/**
 * @brief DMA2 Stream 7 Interrupt Service Routine
 * Called when a DMA transmission block is complete.
 */
void DMA2_Stream7_IRQHandler(void) {
    if (DMA2->HISR & (1UL << 27)) { /* TCIF7 flag set */
        DMA2->HIFCR = (1UL << 27);  /* Clear TCIF7 flag */
        dma_busy = 0;
        
        /* Check if more data is in queue to continue sending */
        UART_DMA_StartNext();
    }
}

/**
 * @brief Transmit a string using non-blocking DMA
 * @param str Null-terminated string to send
 * @note If buffer is full, remaining chars are DROPPED (never blocks)
 */
void UART_SendString_DMA(const char *str) {
    uint16_t next_head;
    
    while (*str) {
        next_head = (tx_head + 1) % UART_TX_BUF_SIZE;
        
        /* If buffer is full, DROP remaining data instead of blocking.
         * Blocking here can freeze the main loop when DMA ISR (priority 7)
         * is starved by high-priority ISRs like TIM11 (priority 0). */
        if (next_head == tx_tail) {
            /* Try to kick DMA if idle */
            if (!dma_busy) {
                UART_DMA_StartNext();
            }
            return;  /* Drop the rest — never block */
        }
        
        uart_tx_buffer[tx_head] = *str++;
        tx_head = next_head;
    }
    
    /* Start DMA if it's currently idle */
    if (!dma_busy) {
        UART_DMA_StartNext();
    }
}

/**
 * @brief Basic character transmit (Legacy blocking mode)
 * @note REDIRECTED to Bluetooth (USART2/JDY-33)
 */
void UART_SendChar(char c) {
    BT_SendChar(c);
}

/**
 * @brief Send string — REDIRECTED to Bluetooth (USART2/JDY-33)
 * @note All existing UART_SendString() calls now go through BLE.
 *       To revert to wired UART1, change back to UART_SendString_DMA(str);
 */
void UART_SendString(const char *str) {
    BT_SendString(str);
}

/**
 * @brief Send a number formatted as text
 * @note REDIRECTED to Bluetooth (USART2/JDY-33)
 */
void UART_SendNumber(uint32_t num) {
    char buffer[12];
    sprintf(buffer, "%lu", num);
    BT_SendString(buffer);
}
