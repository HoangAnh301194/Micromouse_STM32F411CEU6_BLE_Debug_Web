/* PA2/PA3 USART2 -> JDY-33. Single main-context TX producer, IRQ consumer.
 * Diagnostic counters are compiled only for BLE_PROFILE_DIAG.
 * Never share DMA1 Stream6 with I2C1_TX. No TX/RX busy-wait in the BLE runner.
 */
#include "bt_debug.h"
#include "board.h"
#include <stdio.h>
#include <string.h>
#include <stdarg.h>

#if (BT_TX_BUF_SIZE & (BT_TX_BUF_SIZE - 1U)) != 0
#error "BT_TX_BUF_SIZE must be a power of two"
#endif
#define TX_MASK (BT_TX_BUF_SIZE - 1U)
#define RX_SIZE 256U
#define RX_MASK (RX_SIZE - 1U)

static uint8_t tx_buf[BT_TX_BUF_SIZE];
static volatile uint16_t tx_head, tx_tail;
static uint8_t rx_buf[RX_SIZE];
static volatile uint16_t rx_head, rx_tail;
static volatile BT_Profile_t profile;

static void BT_GPIO_Config(void)
{
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    GPIOA->MODER = (GPIOA->MODER & ~((3UL<<4)|(3UL<<6))) |
                   (2UL<<4) | (2UL<<6);
    GPIOA->OSPEEDR |= (3UL<<4)|(3UL<<6);
    GPIOA->PUPDR = (GPIOA->PUPDR & ~((3UL<<4)|(3UL<<6))) |
                   (1UL<<4)|(1UL<<6);
    GPIOA->AFR[0] = (GPIOA->AFR[0] & ~((15UL<<8)|(15UL<<12))) |
                     (7UL<<8)|(7UL<<12);
}

void BT_Init(void)
{
    RCC->APB1ENR |= RCC_APB1ENR_USART2EN;
    BT_GPIO_Config();
    USART2->CR1 = 0U;
    USART2->CR2 = 0U;
    USART2->CR3 = 0U;
    USART2->BRR = (Board_GetAPB1ClockHz() + BT_BAUDRATE/2U)/BT_BAUDRATE;
    tx_head=tx_tail=rx_head=rx_tail=0U;
    memset((void *)&profile,0,sizeof(profile));
    USART2->CR1 = USART_CR1_TE | USART_CR1_RE | USART_CR1_RXNEIE | USART_CR1_UE;
    NVIC_SetPriority(USART2_IRQn,8U);
    NVIC_EnableIRQ(USART2_IRQn);
}

uint8_t BT_TrySendData(const uint8_t *data, uint16_t len)
{
    uint16_t used, free_bytes, i, write_head;
    if (!data || !len || len >= BT_TX_BUF_SIZE) {
        profile.tx_drop++;
        return 0U;
    }
    /* One producer (main); ISR only moves tail. Publishing head LAST prevents
     * the interrupt reading half-written packets. No interrupts masked here. */
    used = (uint16_t)((tx_head - tx_tail) & TX_MASK);
    free_bytes = (uint16_t)(TX_MASK - used);
    if (len > free_bytes) {
        profile.tx_drop++;
        return 0U; /* All or nothing, never a partial packet. */
    }
    write_head = tx_head;
    for (i=0U;i<len;i++) {
        tx_buf[write_head]=data[i];
        write_head=(uint16_t)((write_head+1U)&TX_MASK);
    }
    tx_head=write_head;
    profile.tx_accepted+=len;
    if (used+len > profile.tx_queue_peak) profile.tx_queue_peak=used+len;
    USART2->CR1 |= USART_CR1_TXEIE;
    return 1U;
}

void BT_SendData(const uint8_t *data, uint16_t len)
{
    (void)BT_TrySendData(data,len);
}

void BT_SendString(const char *s)
{
    size_t n;
    if (!s) return;
    n=strlen(s);
    if (n > 65535U) { profile.tx_drop++; return; }
    (void)BT_TrySendData((const uint8_t *)s,(uint16_t)n);
}

void BT_Printf(const char *fmt,...)
{
    char buf[256];
    va_list args;
    int n;
    va_start(args,fmt);
    n=vsnprintf(buf,sizeof(buf),fmt,args);
    va_end(args);
    if (n<0 || (size_t)n >= sizeof(buf)) { profile.tx_drop++; return; }
    BT_SendData((const uint8_t *)buf,(uint16_t)n);
}

void USART2_IRQHandler(void)
{
#if defined(BLE_PROFILE_DIAG)
    uint32_t started=DWT->CYCCNT;
#endif
    uint32_t sr=USART2->SR;
    if (sr & (USART_SR_RXNE | USART_SR_ORE | USART_SR_FE | USART_SR_NE)) {
        uint8_t byte=(uint8_t)USART2->DR;
        if (sr & USART_SR_ORE) profile.rx_ore++;
        if (sr & USART_SR_FE) profile.rx_fe++;
        if (sr & USART_SR_NE) profile.rx_ne++;
        if ((sr & USART_SR_RXNE) && !(sr & (USART_SR_ORE|USART_SR_FE|USART_SR_NE))) {
            uint16_t next=(uint16_t)((rx_head+1U)&RX_MASK);
            if (next==rx_tail) profile.rx_overflow++;
            else { rx_buf[rx_head]=byte; rx_head=next; profile.rx_bytes++; }
        }
    }
    if ((sr & USART_SR_TXE) && (USART2->CR1 & USART_CR1_TXEIE)) {
        if (tx_head!=tx_tail) {
            USART2->DR=tx_buf[tx_tail];
            tx_tail=(uint16_t)((tx_tail+1U)&TX_MASK);
            profile.tx_uart_dr_bytes++;
        } else {
            USART2->CR1 &= ~USART_CR1_TXEIE;
        }
    }
#if defined(BLE_PROFILE_DIAG)
    profile.irq_cycles += (uint32_t)(DWT->CYCCNT-started);
    profile.irq_count++;
#endif
}

uint8_t BT_ReadByte(uint8_t *out)
{
    if (!out || rx_head==rx_tail) return 0U;
    *out=rx_buf[rx_tail];
    rx_tail=(uint16_t)((rx_tail+1U)&RX_MASK);
    return 1U;
}

uint8_t BT_Available(void) { return rx_head!=rx_tail; }
/* Legacy APIs retained; do not call from ISR or benchmark path. */
void BT_SendChar(char c) { while (!(USART2->SR & USART_SR_TXE)) {} USART2->DR=c; }
char BT_ReceiveChar(void) { uint8_t c; while (!BT_ReadByte(&c)) {} return (char)c; }

void BT_ResetTxQueuePeak(void)
{
    /* BT_TrySendData has one main-context producer. Snapshot queue
     * atomically relative to USART2 TXE interrupt consuming tx_tail. */
    uint32_t primask=__get_PRIMASK();
    __disable_irq();
    profile.tx_queue_peak=(uint16_t)((tx_head-tx_tail)&TX_MASK);
    if (!primask) __enable_irq();
}

void BT_GetProfile(BT_Profile_t *out)
{
    uint32_t primask;
    if (!out) return;
    primask=__get_PRIMASK();
    __disable_irq();
    *out=profile;
    out->tx_queue=(uint16_t)((tx_head-tx_tail)&TX_MASK);
    if (!primask) __enable_irq();
}
