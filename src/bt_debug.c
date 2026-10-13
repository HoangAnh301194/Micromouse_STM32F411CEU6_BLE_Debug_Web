/**
 * @file bt_debug.c
 * @brief Bluetooth Debug UART Driver — USART2 with Interrupt-based TX
 * @details Uses USART2 TXE interrupt + circular buffer for non-blocking TX.
 *          DMA1 Stream6 (the only valid DMA for USART2_TX on STM32F411)
 *          is already used by I2C, so we use interrupt-driven TX instead.
 *
 *  Hardware wiring:
 *      PA2 (USART2_TX, AF7)  ->  JDY-33 RXD
 *      PA3 (USART2_RX, AF7)  ->  JDY-33 TXD
 */

#include "bt_debug.h"

/* ============================================================================ */
/* Private data                                                                 */
/* ============================================================================ */

static volatile uint8_t bt_rx_buffer[BT_RX_BUF_SIZE];
static volatile uint16_t bt_rx_head = 0;
static volatile uint16_t bt_rx_tail = 0;

static uint8_t  bt_tx_buffer[BT_TX_BUF_SIZE];
static volatile uint16_t bt_tx_head = 0;   /* Write index (main context)   */
static volatile uint16_t bt_tx_tail = 0;   /* Read index  (ISR context)    */

/* ============================================================================ */
/* Forward declarations                                                         */
/* ============================================================================ */

static void BT_GPIO_Config(void);
static uint32_t BT_Get_APB1_Clock(void);

/* ============================================================================ */
/* GPIO Configuration — PA2 TX, PA3 RX as AF7 (USART2)                         */
/* ============================================================================ */

static void BT_GPIO_Config(void) {
    /* Enable GPIOA clock */
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;

    /* PA2, PA3 -> Alternate Function mode (MODER = 10) */
    GPIOA->MODER &= ~((3UL << 4) | (3UL << 6));
    GPIOA->MODER |=  ((2UL << 4) | (2UL << 6));

    /* High speed */
    GPIOA->OSPEEDR |= (3UL << 4) | (3UL << 6);

    /* Pull-up for idle-high UART lines */
    GPIOA->PUPDR &= ~((3UL << 4) | (3UL << 6));
    GPIOA->PUPDR |=  ((1UL << 4) | (1UL << 6));

    /* Map PA2, PA3 to AF7 (USART2) via AFR[0] (pins 0-7) */
    GPIOA->AFR[0] &= ~((0xFUL << 8) | (0xFUL << 12));
    GPIOA->AFR[0] |=  ((7UL   << 8) | (7UL   << 12));
}

/* ============================================================================ */
/* APB1 Clock Calculation (USART2 sits on APB1)                                 */
/* ============================================================================ */

static uint32_t BT_Get_APB1_Clock(void) {
    uint32_t sysclk;
    uint32_t tmp;
    uint32_t pllm, plln, pllp;
    uint32_t ahb_pre, apb1_pre;

    tmp = RCC->CFGR & RCC_CFGR_SWS;

    if (tmp == RCC_CFGR_SWS_HSI) {
        sysclk = 16000000;
    } else if (tmp == RCC_CFGR_SWS_HSE) {
        sysclk = 25000000;
    } else if (tmp == RCC_CFGR_SWS_PLL) {
        pllm = RCC->PLLCFGR & RCC_PLLCFGR_PLLM;
        plln = (RCC->PLLCFGR & RCC_PLLCFGR_PLLN) >> 6;
        pllp = (((RCC->PLLCFGR & RCC_PLLCFGR_PLLP) >> 16) + 1) * 2;
        if (RCC->PLLCFGR & RCC_PLLCFGR_PLLSRC) {
            sysclk = (25000000 / pllm) * plln / pllp;
        } else {
            sysclk = (16000000 / pllm) * plln / pllp;
        }
    } else {
        sysclk = 16000000;
    }

    tmp = (RCC->CFGR & RCC_CFGR_HPRE) >> 4;
    ahb_pre = (tmp & 0x08) ? (1 << ((tmp & 0x07) + 1)) : 1;

    tmp = (RCC->CFGR & RCC_CFGR_PPRE1) >> 10;
    apb1_pre = (tmp & 0x04) ? (1 << ((tmp & 0x03) + 1)) : 1;

    return sysclk / ahb_pre / apb1_pre;
}

/* ============================================================================ */
/* Public: Initialise USART2 for Bluetooth debug                                */
/* ============================================================================ */

void BT_Init(void) {
    uint32_t apb1_clock;
    uint32_t brr_value;

    /* Enable USART2 clock (APB1) */
    RCC->APB1ENR |= RCC_APB1ENR_USART2EN;

    /* Configure GPIO */
    BT_GPIO_Config();

    /* Disable USART2 for safe configuration */
    USART2->CR1 &= ~USART_CR1_UE;

    /* Calculate baud-rate register value */
    apb1_clock = BT_Get_APB1_Clock();
    brr_value  = apb1_clock / BT_BAUDRATE;
    USART2->BRR = brr_value;

    /* Enable TX and RX */
    USART2->CR1 |= USART_CR1_TE | USART_CR1_RE | USART_CR1_RXNEIE;

    /* Enable USART2 */
    USART2->CR1 |= USART_CR1_UE;

    /* Configure USART2 interrupt — medium-low priority */
    NVIC_SetPriority(USART2_IRQn, 8);
    NVIC_EnableIRQ(USART2_IRQn);

    /* NOTE: TXE interrupt is NOT enabled here.
     * It will be enabled on-demand when data is queued. */
}

/* ============================================================================ */
/* USART2 ISR — feeds bytes from circular buffer to USART2->DR                  */
/* ============================================================================ */

void USART2_IRQHandler(void) {
    if (USART2->SR & USART_SR_RXNE) {
        uint8_t value = (uint8_t)USART2->DR;
        uint16_t next = (uint16_t)((bt_rx_head + 1U) % BT_RX_BUF_SIZE);
        if (next != bt_rx_tail) {
            bt_rx_buffer[bt_rx_head] = value;
            bt_rx_head = next;
        }
    }
    /* TXE: transmit data register empty — ready for next byte */
    if ((USART2->SR & USART_SR_TXE) && (USART2->CR1 & USART_CR1_TXEIE)) {
        if (bt_tx_head != bt_tx_tail) {
            /* Send next byte from buffer */
            USART2->DR = bt_tx_buffer[bt_tx_tail];
            bt_tx_tail = (bt_tx_tail + 1) % BT_TX_BUF_SIZE;
        } else {
            /* Buffer empty — disable TXE interrupt to stop firing */
            USART2->CR1 &= ~USART_CR1_TXEIE;
        }
    }
}

/* ============================================================================ */
/* Public API                                                                   */
/* ============================================================================ */

void BT_SendString(const char *str) {
    uint16_t next_head;

    while (*str) {
        next_head = (bt_tx_head + 1) % BT_TX_BUF_SIZE;

        if (next_head == bt_tx_tail) {
            /* Buffer full — kick ISR and DROP remainder (never block!) */
            USART2->CR1 |= USART_CR1_TXEIE;
            return;
        }

        bt_tx_buffer[bt_tx_head] = *str++;
        bt_tx_head = next_head;
    }

    /* Kick the TXE interrupt to start sending */
    USART2->CR1 |= USART_CR1_TXEIE;
}

void BT_SendData(const uint8_t *data, uint16_t len) {
    uint16_t next_head;
    uint16_t i;

    for (i = 0; i < len; i++) {
        next_head = (bt_tx_head + 1) % BT_TX_BUF_SIZE;
        if (next_head == bt_tx_tail) {
            USART2->CR1 |= USART_CR1_TXEIE;
            return;
        }
        bt_tx_buffer[bt_tx_head] = data[i];
        bt_tx_head = next_head;
    }

    USART2->CR1 |= USART_CR1_TXEIE;
}

void BT_Printf(const char *fmt, ...) {
    char buf[256];
    va_list args;
    va_start(args, fmt);
    vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);
    BT_SendString(buf);
}

void BT_SendChar(char c) {
    while (!(USART2->SR & USART_SR_TXE));
    USART2->DR = c;
}

uint8_t BT_Available(void) {
    return bt_rx_head != bt_rx_tail;
}

char BT_ReceiveChar(void) {
    char value;
    if (bt_rx_head == bt_rx_tail) return 0;
    value = (char)bt_rx_buffer[bt_rx_tail];
    bt_rx_tail = (uint16_t)((bt_rx_tail + 1U) % BT_RX_BUF_SIZE);
    return value;
}
