/**
 * @file i2c.c
 * @brief I2C Driver with DMA Support - COMPLETE FIXED
 * @author HoangAnh
 * @date 2025
 */

#include "i2c.h"
#include "board.h"
#include "system_timer.h"
#include "stm32f4xx.h"
#include <stdint.h>

/* ========================================================================== */
/* HARDWARE REGISTER DEFINITIONS                                              */
/* ========================================================================== */

#ifndef RCC_BASE
#define RCC_BASE    0x40023800UL
#endif

/* RCC registers */
#define RCC_APB1ENR  (*(volatile uint32_t *)(RCC_BASE + 0x40))
#define RCC_AHB1ENR  (*(volatile uint32_t *)(RCC_BASE + 0x30))

/* GPIO registers */
#define GPIOB_MODER   (*(volatile uint32_t *)(GPIOB_BASE + 0x00))
#define GPIOB_OTYPER  (*(volatile uint32_t *)(GPIOB_BASE + 0x04))
#define GPIOB_OSPEEDR (*(volatile uint32_t *)(GPIOB_BASE + 0x08))
#define GPIOB_PUPDR   (*(volatile uint32_t *)(GPIOB_BASE + 0x0C))
#define GPIOB_IDR     (*(volatile uint32_t *)(GPIOB_BASE + 0x10))
#define GPIOB_ODR     (*(volatile uint32_t *)(GPIOB_BASE + 0x14))
#define GPIOB_AFRL    (*(volatile uint32_t *)(GPIOB_BASE + 0x20))
#define GPIOB_AFRH    (*(volatile uint32_t *)(GPIOB_BASE + 0x24))

/* I2C register offsets */
#define I2C_CR1     0x00
#define I2C_CR2     0x04
#define I2C_OAR1    0x08
#define I2C_OAR2    0x0C
#define I2C_DR      0x10
#define I2C_SR1     0x14
#define I2C_SR2     0x18
#define I2C_CCR     0x1C
#define I2C_TRISE   0x20

/* Timeout configuration */
#define I2C_TIMEOUT_MS  50

/* DMA Configuration for I2C1 TX */
#define I2C1_TX_DMA_STREAM   DMA1_Stream6
#define I2C1_TX_DMA_CHANNEL  1

/* ========================================================================== */
/* PRIVATE VARIABLES - FIXED CALLBACK SYSTEM                                  */
/* ========================================================================== */

static volatile uint8_t i2c_dma_busy = 0;
static uint32_t i2c_nominal_scl_hz = 0;

/* [FIX] Separate TX and RX callbacks */
static I2C_DMA_Callback_t i2c_dma_tx_callback = 0;  /* For OLED (Stream 6) */
static I2C_DMA_Callback_t i2c_dma_rx_callback = 0;  /* For MPU6050 (Stream 0) */

static uint32_t i2c_dma_instance_base = 0;

/* ========================================================================== */
/* PRIVATE HELPER FUNCTIONS                                                   */
/* ========================================================================== */

static uint32_t I2C_GetBase(I2C_Instance instance) {
    switch(instance) {
        case I2C_1: return I2C1_BASE;
        case I2C_2: return I2C2_BASE;
        case I2C_3: return I2C3_BASE;
        default: return I2C1_BASE;
    }
}

static void I2C_ClearErrors(uint32_t i2c_base) {
    /* I2C error bits are write-zero-to-clear. Do not erase all SR1 flags. */
    volatile uint32_t *sr1 = (volatile uint32_t *)(i2c_base + I2C_SR1);
    *sr1 &= ~(I2C_SR1_BERR | I2C_SR1_ARLO | I2C_SR1_AF |
              I2C_SR1_OVR | I2C_SR1_TIMEOUT);
}

static void I2C_BusRecovery(void) {
    uint8_t i;
    
    GPIOB_MODER &= ~(3UL << 16);
    GPIOB_MODER |= (1UL << 16);
    GPIOB_OTYPER |= (1 << 8);
    
    for (i = 0; i < 9; i++) {
        GPIOB_ODR &= ~(1 << 8);
        delay_us_blocking(5);
        GPIOB_ODR |= (1 << 8);
        delay_us_blocking(5);
    }
    
    GPIOB_ODR &= ~(1 << 9);
    delay_us_blocking(5);
    GPIOB_ODR |= (1 << 8);
    delay_us_blocking(5);
    GPIOB_ODR |= (1 << 9);
    delay_us_blocking(5);
    
    GPIOB_MODER &= ~(3UL << 16);
    GPIOB_MODER |= (2UL << 16);
}

static I2C_Status I2C_WaitBusFree(uint32_t i2c_base) {
    uint32_t start_time = millis();
    
    while ((*(volatile uint32_t *)(i2c_base + I2C_SR2)) & I2C_SR2_BUSY) {
        if (delay_elapsed_ms(start_time, I2C_TIMEOUT_MS)) {
            I2C_ClearErrors(i2c_base);
            (*(volatile uint32_t *)(i2c_base + I2C_CR1)) |= I2C_CR1_STOP;
            delay_us_blocking(100);
            
            if ((*(volatile uint32_t *)(i2c_base + I2C_SR2)) & I2C_SR2_BUSY) {
                I2C_BusRecovery();
                return I2C_BUSY;
            }
        }
    }
    return I2C_OK;
}

static void I2C_SoftReset(uint32_t i2c_base) {
    (*(volatile uint32_t *)(i2c_base + I2C_CR1)) |= I2C_CR1_SWRST;
    delay_us_blocking(10);
    (*(volatile uint32_t *)(i2c_base + I2C_CR1)) &= ~I2C_CR1_SWRST;
}

static I2C_Status I2C_Start(uint32_t i2c_base) {
    uint32_t start_time = millis();
    
    I2C_ClearErrors(i2c_base);
    (*(volatile uint32_t *)(i2c_base + I2C_CR1)) |= I2C_CR1_START;
    
    while(!((*(volatile uint32_t *)(i2c_base + I2C_SR1)) & I2C_SR1_SB)) {
        if(delay_elapsed_ms(start_time, I2C_TIMEOUT_MS)) {
            return I2C_TIMEOUT;
        }
    }
    return I2C_OK;
}

static void I2C_Stop(uint32_t i2c_base) {
    (*(volatile uint32_t *)(i2c_base + I2C_CR1)) |= I2C_CR1_STOP;
    delay_us_blocking(20);
}

static I2C_Status I2C_SendAddress(uint32_t i2c_base, uint8_t addr) {
    uint32_t start_time = millis();
    
    (*(volatile uint32_t *)(i2c_base + I2C_DR)) = addr;
    
    while(!((*(volatile uint32_t *)(i2c_base + I2C_SR1)) & I2C_SR1_ADDR)) {
        if((*(volatile uint32_t *)(i2c_base + I2C_SR1)) & I2C_SR1_AF) {
            (*(volatile uint32_t *)(i2c_base + I2C_SR1)) &= ~I2C_SR1_AF;
            return I2C_NACK;
        }
        if(delay_elapsed_ms(start_time, I2C_TIMEOUT_MS)) {
            return I2C_TIMEOUT;
        }
    }
    
    (void)(*(volatile uint32_t *)(i2c_base + I2C_SR1));
    (void)(*(volatile uint32_t *)(i2c_base + I2C_SR2));
    return I2C_OK;
}

/* ========================================================================== */
/* PUBLIC INITIALIZATION FUNCTIONS                                            */
/* ========================================================================== */

void I2C_Init(I2C_Instance instance, uint8_t fast_mode) {
    uint32_t pclk_hz, pclk_mhz, ccr, target_hz;
    uint32_t i2c_base = I2C_GetBase(instance);
    if (instance != I2C_1) return;

    RCC_AHB1ENR |= (1UL << 1);
    RCC_APB1ENR |= (1UL << 21);
    GPIOB_MODER = (GPIOB_MODER & ~((3UL << 16) | (3UL << 18))) |
                  (2UL << 16) | (2UL << 18);
    GPIOB_OTYPER |= (1UL << 8) | (1UL << 9);
    GPIOB_OSPEEDR |= (3UL << 16) | (3UL << 18);
    GPIOB_PUPDR = (GPIOB_PUPDR & ~((3UL << 16) | (3UL << 18))) |
                  (1UL << 16) | (1UL << 18);
    GPIOB_AFRH = (GPIOB_AFRH & ~0xFFUL) | 0x44UL;

    /* Timing from actual APB1 clock; do not assume PCLK1=50 MHz. */
    pclk_hz = Board_GetAPB1ClockHz();
    pclk_mhz = pclk_hz / 1000000UL;
    if (pclk_mhz < 2U) pclk_mhz = 2U;
    if (pclk_mhz > 50U) pclk_mhz = 50U;
    I2C_SoftReset(i2c_base);
    (*(volatile uint32_t *)(i2c_base + I2C_CR1)) &= ~I2C_CR1_PE;
    (*(volatile uint32_t *)(i2c_base + I2C_CR2)) =
        ((*(volatile uint32_t *)(i2c_base + I2C_CR2)) & ~I2C_CR2_FREQ) | pclk_mhz;
    if (fast_mode && pclk_hz >= 4000000UL) {
        target_hz = 400000UL;
        /* Fast mode, duty 2: tSCL = 3 * CCR / PCLK1 */
        ccr = (pclk_hz + (3UL * target_hz - 1UL)) / (3UL * target_hz);
        if (ccr < 1U) ccr = 1U;
        (*(volatile uint32_t *)(i2c_base + I2C_CCR)) = I2C_CCR_FS | ccr;
        (*(volatile uint32_t *)(i2c_base + I2C_TRISE)) =
            ((pclk_mhz * 300UL + 999UL) / 1000UL) + 1UL;
        i2c_nominal_scl_hz = pclk_hz / (3UL * ccr);
    } else {
        target_hz = 100000UL;
        ccr = (pclk_hz + (2UL * target_hz - 1UL)) / (2UL * target_hz);
        if (ccr < 4U) ccr = 4U;
        (*(volatile uint32_t *)(i2c_base + I2C_CCR)) = ccr;
        (*(volatile uint32_t *)(i2c_base + I2C_TRISE)) = pclk_mhz + 1UL;
        i2c_nominal_scl_hz = pclk_hz / (2UL * ccr);
    }
    (*(volatile uint32_t *)(i2c_base + I2C_CR1)) = I2C_CR1_PE | I2C_CR1_ACK;
}

uint32_t I2C_GetConfiguredSclHz(void) { return i2c_nominal_scl_hz; }


/* ========================================================================== */
/* DMA FUNCTIONS                                                               */
/* ========================================================================== */

void I2C_DMA_Init(I2C_Instance instance) {
    if (instance != I2C_1) return;
    
    RCC->AHB1ENR |= RCC_AHB1ENR_DMA1EN;
    
    /* TX Stream 6 */
    I2C1_TX_DMA_STREAM->CR &= ~DMA_SxCR_EN;
    while (I2C1_TX_DMA_STREAM->CR & DMA_SxCR_EN);
    DMA1->HIFCR = 0x3F << 16;
    
    I2C1_TX_DMA_STREAM->CR = 
        (I2C1_TX_DMA_CHANNEL << 25) | (2 << 16) | (0 << 13) | (0 << 11) |
        (1 << 10) | (0 << 9) | (1 << 6) | (0 << 5) | (1 << 4);
    I2C1_TX_DMA_STREAM->PAR = (uint32_t)&I2C1->DR;
    
    /* RX Stream 0 */
    DMA1_Stream0->CR &= ~DMA_SxCR_EN;
    while (DMA1_Stream0->CR & DMA_SxCR_EN);
    DMA1->LIFCR = 0x3F;
    
    DMA1_Stream0->CR = 
        (1 << 25) | (2 << 16) | (0 << 13) | (0 << 11) |
        (1 << 10) | (0 << 9) | (0 << 6) | (0 << 5) | (1 << 4);
    DMA1_Stream0->PAR = (uint32_t)&I2C1->DR;
    
    /* Enable interrupts */
    NVIC_EnableIRQ(DMA1_Stream6_IRQn);
    NVIC_SetPriority(DMA1_Stream6_IRQn, 2);
    NVIC_EnableIRQ(DMA1_Stream0_IRQn);
    NVIC_SetPriority(DMA1_Stream0_IRQn, 2);
    
    I2C1->CR2 |= I2C_CR2_DMAEN;
}

/* ========================================================================== */
/* CALLBACK REGISTRATION - FIXED                                              */
/* ========================================================================== */

void I2C_SetCallback_DMA(I2C_DMA_Callback_t callback) {
    i2c_dma_tx_callback = callback;
}

void I2C_SetCallback_DMA_RX(I2C_DMA_Callback_t callback) {
    i2c_dma_rx_callback = callback;
}

uint8_t I2C_IsBusy_DMA(I2C_Instance instance) {
    (void)instance;
    return i2c_dma_busy;
}

/* ========================================================================== */
/* DMA READ FUNCTION - COMPLETE FIXED                                         */
/* ========================================================================== */

void I2C_ReadReg_DMA(I2C_TypeDef *I2Cx, uint8_t addr, uint8_t reg, 
                     uint8_t *buffer, uint16_t count) {
    /* [CRITICAL FIX] All blocking waits now have timeout protection.
     * This function is called from TIM11 ISR (priority 0, highest).
     * Without timeouts, if I2C bus is held by main loop (e.g. OLED write),
     * the ISR blocks forever → system deadlock.
     * Using loop counter (~500us at 96MHz) since millis() won't advance in ISR. */
    volatile uint32_t timeout;
    
    /* Phase 1: Write register address (blocking with timeout) */
    timeout = 10000;
    while (I2Cx->SR2 & I2C_SR2_BUSY) {
        if (--timeout == 0) return;  /* Bus stuck — abort silently */
    }
    
    I2Cx->CR1 |= I2C_CR1_START;
    timeout = 10000;
    while (!(I2Cx->SR1 & I2C_SR1_SB)) {
        if (--timeout == 0) { I2Cx->CR1 |= I2C_CR1_STOP; return; }
    }
    
    I2Cx->DR = (addr << 1);
    timeout = 10000;
    while (!(I2Cx->SR1 & I2C_SR1_ADDR)) {
        if (--timeout == 0) { I2Cx->CR1 |= I2C_CR1_STOP; return; }
    }
    (void)I2Cx->SR2;
    
    timeout = 10000;
    while (!(I2Cx->SR1 & I2C_SR1_TXE)) {
        if (--timeout == 0) { I2Cx->CR1 |= I2C_CR1_STOP; return; }
    }
    I2Cx->DR = reg;
    timeout = 10000;
    while (!(I2Cx->SR1 & I2C_SR1_TXE)) {
        if (--timeout == 0) { I2Cx->CR1 |= I2C_CR1_STOP; return; }
    }
    
    /* Phase 2: Repeated START for read */
    I2Cx->CR1 |= I2C_CR1_START;
    timeout = 10000;
    while (!(I2Cx->SR1 & I2C_SR1_SB)) {
        if (--timeout == 0) { I2Cx->CR1 |= I2C_CR1_STOP; return; }
    }
    
    I2Cx->DR = (addr << 1) | 1;
    timeout = 10000;
    while (!(I2Cx->SR1 & I2C_SR1_ADDR)) {
        if (--timeout == 0) { I2Cx->CR1 |= I2C_CR1_STOP; return; }
    }
    
    /* [CRITICAL] Configure before clearing ADDR */
    I2Cx->CR1 |= I2C_CR1_ACK;
    I2Cx->CR2 |= I2C_CR2_LAST;
    I2Cx->CR2 |= I2C_CR2_DMAEN;
    
    /* Clear ADDR to start reception */
    (void)I2Cx->SR2;
    
    /* Phase 3: Configure DMA */
    DMA1_Stream0->CR &= ~DMA_SxCR_EN;
    timeout = 10000;
    while (DMA1_Stream0->CR & DMA_SxCR_EN) {
        if (--timeout == 0) return;
    }
    
    DMA1->LIFCR = 0x3F;
    
    DMA1_Stream0->PAR = (uint32_t)&(I2Cx->DR);
    DMA1_Stream0->M0AR = (uint32_t)buffer;
    DMA1_Stream0->NDTR = count;
    
    DMA1_Stream0->CR = 
        (1 << 25) | (2 << 16) | (0 << 13) | (0 << 11) |
        DMA_SxCR_MINC | (0 << 6) | DMA_SxCR_TCIE;
    
    /* Start DMA */
    DMA1_Stream0->CR |= DMA_SxCR_EN;
}

/* ========================================================================== */
/* DMA INTERRUPT HANDLERS - FIXED                                             */
/* ========================================================================== */

void DMA1_Stream6_IRQHandler(void) {
    uint32_t counter;
    
    if (DMA1->HISR & (1 << 21)) {
        DMA1->HIFCR = (1 << 21);
        I2C1_TX_DMA_STREAM->CR &= ~DMA_SxCR_EN;
        
        counter = 0;
        while (!((*(volatile uint32_t *)(i2c_dma_instance_base + I2C_SR1)) & I2C_SR1_BTF)) {
            counter++;
            if (counter > 100000) break;
        }
        
        (*(volatile uint32_t *)(i2c_dma_instance_base + I2C_CR1)) |= I2C_CR1_STOP;
        i2c_dma_busy = 0;
        
        /* [FIX] Call TX callback */
        if (i2c_dma_tx_callback) {
            i2c_dma_tx_callback();
        }
    }
}

void DMA1_Stream0_IRQHandler(void) {
    if (DMA1->LISR & (1 << 5)) {
        DMA1->LIFCR = (1 << 5);
        DMA1_Stream0->CR &= ~DMA_SxCR_EN;
        I2C1->CR1 |= I2C_CR1_STOP;
        
        /* [FIX] Call RX callback */
        if (i2c_dma_rx_callback) {
            i2c_dma_rx_callback();
        }
    }
}

/* ========================================================================== */
/* BLOCKING FUNCTIONS (Unchanged)                                             */
/* ========================================================================== */

I2C_Status I2C_Write_DMA(I2C_Instance instance, uint8_t slave_addr, 
                         uint8_t *data, uint16_t len) {
    uint32_t i2c_base = I2C_GetBase(instance);
    uint32_t timeout;
    
    if (instance != I2C_1) return I2C_ERROR;
    if (i2c_dma_busy) return I2C_BUSY;
    if (I2C_WaitBusFree(i2c_base) != I2C_OK) return I2C_BUSY;
    
    i2c_dma_busy = 1;
    i2c_dma_instance_base = i2c_base;
    I2C_ClearErrors(i2c_base);
    
    (*(volatile uint32_t *)(i2c_base + I2C_CR1)) |= I2C_CR1_START;
    
    timeout = millis();
    while (!((*(volatile uint32_t *)(i2c_base + I2C_SR1)) & I2C_SR1_SB)) {
        if (delay_elapsed_ms(timeout, I2C_TIMEOUT_MS)) {
            i2c_dma_busy = 0;
            return I2C_TIMEOUT;
        }
    }
    
    (*(volatile uint32_t *)(i2c_base + I2C_DR)) = (slave_addr << 1);
    
    timeout = millis();
    while (!((*(volatile uint32_t *)(i2c_base + I2C_SR1)) & I2C_SR1_ADDR)) {
        if ((*(volatile uint32_t *)(i2c_base + I2C_SR1)) & I2C_SR1_AF) {
            (*(volatile uint32_t *)(i2c_base + I2C_SR1)) &= ~I2C_SR1_AF;
            (*(volatile uint32_t *)(i2c_base + I2C_CR1)) |= I2C_CR1_STOP;
            i2c_dma_busy = 0;
            return I2C_NACK;
        }
        if (delay_elapsed_ms(timeout, I2C_TIMEOUT_MS)) {
            (*(volatile uint32_t *)(i2c_base + I2C_CR1)) |= I2C_CR1_STOP;
            i2c_dma_busy = 0;
            return I2C_TIMEOUT;
        }
    }
    
    (void)(*(volatile uint32_t *)(i2c_base + I2C_SR1));
    (void)(*(volatile uint32_t *)(i2c_base + I2C_SR2));
    
    I2C1_TX_DMA_STREAM->CR &= ~DMA_SxCR_EN;
    while (I2C1_TX_DMA_STREAM->CR & DMA_SxCR_EN);
    
    DMA1->HIFCR = 0x3F << 16;
    I2C1_TX_DMA_STREAM->M0AR = (uint32_t)data;
    I2C1_TX_DMA_STREAM->NDTR = len;
    I2C1_TX_DMA_STREAM->CR |= DMA_SxCR_EN;
    
    return I2C_OK;
}

I2C_Status I2C_Write(I2C_Instance instance, uint8_t slave_addr, 
                     uint8_t *data, uint16_t len) {
    uint32_t i2c_base = I2C_GetBase(instance);
    uint32_t start_time;
    uint16_t i;
    I2C_Status status;
    
    if (I2C_WaitBusFree(i2c_base) != I2C_OK) {
        I2C_Init(instance, 1);
        delay_us_blocking(100);
        if (I2C_WaitBusFree(i2c_base) != I2C_OK) return I2C_BUSY;
    }
    
    status = I2C_Start(i2c_base);
    if (status != I2C_OK) { I2C_Stop(i2c_base); return status; }
    
    status = I2C_SendAddress(i2c_base, slave_addr << 1);
    if (status != I2C_OK) { I2C_Stop(i2c_base); return status; }
    
    for(i = 0; i < len; i++) {
        start_time = millis();
        while(!((*(volatile uint32_t *)(i2c_base + I2C_SR1)) & I2C_SR1_TXE)) {
            if(delay_elapsed_ms(start_time, I2C_TIMEOUT_MS)) {
                I2C_Stop(i2c_base);
                return I2C_TIMEOUT;
            }
        }
        (*(volatile uint32_t *)(i2c_base + I2C_DR)) = data[i];
    }
    
    start_time = millis();
    while(!((*(volatile uint32_t *)(i2c_base + I2C_SR1)) & I2C_SR1_BTF)) {
        if(delay_elapsed_ms(start_time, I2C_TIMEOUT_MS)) {
            I2C_Stop(i2c_base);
            return I2C_TIMEOUT;
        }
    }
    
    I2C_Stop(i2c_base);
    return I2C_OK;
}

/* Receiver phase of a register-read transaction. Enter with the pointer
 * already written, or with an idle bus for a plain read. Uses repeated START.
 * STM32F4 requires distinct ACK/POS/STOP ordering for 1 and 2 bytes.
 * This blocking path is used at boot only, not from a control ISR.
 */
static I2C_Status I2C_ReadReceiver(uint32_t base, uint8_t addr,
                                   uint8_t *data, uint16_t len) {
    I2C_TypeDef *bus = (I2C_TypeDef *)base;
    I2C_Status s;
    uint32_t start;
    uint16_t remaining = len;
    if (!data || len == 0U) return I2C_ERROR;
    bus->CR1 |= I2C_CR1_ACK;
    bus->CR1 &= ~I2C_CR1_POS;
    s = I2C_Start(base);
    if (s != I2C_OK) goto error;
    bus->DR = ((uint32_t)addr << 1) | 1UL;
    start = millis();
    while (!(bus->SR1 & I2C_SR1_ADDR)) {
        if (bus->SR1 & I2C_SR1_AF) { s = I2C_NACK; goto error; }
        if (delay_elapsed_ms(start, I2C_TIMEOUT_MS)) { s = I2C_TIMEOUT; goto error; }
    }
    if (len == 1U) {
        bus->CR1 &= ~I2C_CR1_ACK;
        (void)bus->SR1; (void)bus->SR2;
        bus->CR1 |= I2C_CR1_STOP;
        start = millis();
        while (!(bus->SR1 & I2C_SR1_RXNE)) {
            if (delay_elapsed_ms(start, I2C_TIMEOUT_MS)) { s = I2C_TIMEOUT; goto error; }
        }
        data[0] = (uint8_t)bus->DR;
    } else if (len == 2U) {
        bus->CR1 |= I2C_CR1_POS;
        bus->CR1 &= ~I2C_CR1_ACK;
        (void)bus->SR1; (void)bus->SR2;
        start = millis();
        while (!(bus->SR1 & I2C_SR1_BTF)) {
            if (delay_elapsed_ms(start, I2C_TIMEOUT_MS)) { s = I2C_TIMEOUT; goto error; }
        }
        bus->CR1 |= I2C_CR1_STOP;
        data[0] = (uint8_t)bus->DR;
        data[1] = (uint8_t)bus->DR;
    } else {
        (void)bus->SR1; (void)bus->SR2;
        while (remaining > 3U) {
            start = millis();
            while (!(bus->SR1 & I2C_SR1_RXNE)) {
                if (delay_elapsed_ms(start, I2C_TIMEOUT_MS)) { s = I2C_TIMEOUT; goto error; }
            }
            *data++ = (uint8_t)bus->DR;
            --remaining;
        }
        start = millis();
        while (!(bus->SR1 & I2C_SR1_BTF)) {
            if (delay_elapsed_ms(start, I2C_TIMEOUT_MS)) { s = I2C_TIMEOUT; goto error; }
        }
        bus->CR1 &= ~I2C_CR1_ACK;
        *data++ = (uint8_t)bus->DR;
        start = millis();
        while (!(bus->SR1 & I2C_SR1_BTF)) {
            if (delay_elapsed_ms(start, I2C_TIMEOUT_MS)) { s = I2C_TIMEOUT; goto error; }
        }
        bus->CR1 |= I2C_CR1_STOP;
        *data++ = (uint8_t)bus->DR;
        *data = (uint8_t)bus->DR;
    }
    bus->CR1 &= ~I2C_CR1_POS;
    bus->CR1 |= I2C_CR1_ACK;
    return I2C_OK;
error:
    bus->CR1 |= I2C_CR1_STOP;
    bus->CR1 &= ~I2C_CR1_POS;
    bus->CR1 |= I2C_CR1_ACK;
    return s;
}

I2C_Status I2C_Read(I2C_Instance instance, uint8_t addr,
                     uint8_t *data, uint16_t len) {
    uint32_t base = I2C_GetBase(instance);
    if (instance != I2C_1) return I2C_ERROR;
    if (I2C_WaitBusFree(base) != I2C_OK) return I2C_BUSY;
    return I2C_ReadReceiver(base, addr, data, len);
}


I2C_Status I2C_WriteRegister(I2C_Instance instance, uint8_t slave_addr, 
                             uint8_t reg, uint8_t value) {
    uint8_t data[2];
    data[0] = reg;
    data[1] = value;
    return I2C_Write(instance, slave_addr, data, 2);
}

I2C_Status I2C_ReadRegister(I2C_Instance instance, uint8_t addr,
                             uint8_t reg, uint8_t *value) {
    return I2C_ReadRegisters(instance, addr, reg, value, 1U);
}

I2C_Status I2C_ReadRegisters(I2C_Instance instance, uint8_t addr,
                              uint8_t reg, uint8_t *data, uint16_t len) {
    uint32_t base = I2C_GetBase(instance);
    I2C_TypeDef *bus = (I2C_TypeDef *)base;
    uint32_t start;
    I2C_Status s;
    if (instance != I2C_1 || !data || len == 0U) return I2C_ERROR;
    if (I2C_WaitBusFree(base) != I2C_OK) return I2C_BUSY;
    s = I2C_Start(base);
    if (s != I2C_OK) goto fail;
    s = I2C_SendAddress(base, addr << 1);
    if (s != I2C_OK) goto fail;
    start = millis();
    while (!(bus->SR1 & I2C_SR1_TXE)) {
        if (delay_elapsed_ms(start, I2C_TIMEOUT_MS)) { s = I2C_TIMEOUT; goto fail; }
    }
    bus->DR = reg;
    start = millis();
    while (!(bus->SR1 & I2C_SR1_BTF)) {
        if (delay_elapsed_ms(start, I2C_TIMEOUT_MS)) { s = I2C_TIMEOUT; goto fail; }
    }
    /* Combined I2C transaction: repeated START, NO STOP between phases. */
    return I2C_ReadReceiver(base, addr, data, len);
fail:
    bus->CR1 |= I2C_CR1_STOP;
    return s;
}


I2C_Status I2C_IsDeviceReady(I2C_Instance instance, uint8_t slave_addr) {
    uint32_t i2c_base = I2C_GetBase(instance);
    I2C_Status status;
    
    I2C_WaitBusFree(i2c_base);
    
    if(I2C_Start(i2c_base) != I2C_OK) return I2C_TIMEOUT;
    status = I2C_SendAddress(i2c_base, slave_addr << 1);
    I2C_Stop(i2c_base);
    
    return status;
}

/* ==========================================================================
 * IRQ-driven I2C1: read exactly TWO bytes from one register (MPU gyro Z).
 * 6 short peripheral events, no software wait loop and no DMA setup cost.
 * Completion is consumed by main; never call from TIM11.
 * ========================================================================== */
typedef enum {
    I2C2_IDLE = 0,
    I2C2_START_W,
    I2C2_ADDR_W,
    I2C2_REG_BTF,
    I2C2_RESTART,
    I2C2_ADDR_R,
    I2C2_RX_BTF,
    I2C2_DONE
} I2C2_Phase;

static volatile I2C2_Phase i2c2_phase = I2C2_IDLE;
static volatile I2C_Status i2c2_result = I2C_IDLE;
static volatile uint8_t i2c2_addr, i2c2_reg;
static volatile uint8_t i2c2_bytes[2];
static volatile uint32_t i2c2_start_us, i2c2_duration_us;
static uint8_t i2c2_fast_mode = 1U;

static void I2C2_Finish(I2C_Status result)
{
    I2C1->CR2 &= ~(I2C_CR2_ITEVTEN | I2C_CR2_ITBUFEN | I2C_CR2_ITERREN);
    if (result != I2C_OK) I2C1->CR1 |= I2C_CR1_STOP;
    i2c2_duration_us = micros() - i2c2_start_us;
    i2c2_result = result;
    i2c2_phase = I2C2_DONE;
}

I2C_Status I2C_Read2Async_Start(uint8_t addr, uint8_t reg)
{
    if (i2c2_phase != I2C2_IDLE) return I2C_BUSY;
    if (i2c_dma_busy || (I2C1->SR2 & I2C_SR2_BUSY)) return I2C_BUSY;
    if (addr >= 0x80U) return I2C_ERROR;

    i2c2_addr = addr;
    i2c2_reg = reg;
    i2c2_duration_us = 0;
    i2c2_start_us = micros();
    i2c2_result = I2C_BUSY;
    i2c2_phase = I2C2_START_W;
    I2C1->CR1 &= ~(I2C_CR1_POS | I2C_CR1_STOP);
    I2C1->CR1 |= I2C_CR1_ACK;
    I2C_ClearErrors(I2C1_BASE);
    NVIC_SetPriority(I2C1_EV_IRQn, 3);
    NVIC_SetPriority(I2C1_ER_IRQn, 3);
    NVIC_EnableIRQ(I2C1_EV_IRQn);
    NVIC_EnableIRQ(I2C1_ER_IRQn);
    I2C1->CR2 |= I2C_CR2_ITEVTEN | I2C_CR2_ITERREN;
    I2C1->CR1 |= I2C_CR1_START;
    return I2C_OK;
}

void I2C1_EV_IRQHandler(void)
{
    uint32_t sr1 = I2C1->SR1;
    switch (i2c2_phase) {
    case I2C2_START_W:
        if (sr1 & I2C_SR1_SB) {
            I2C1->DR = ((uint32_t)i2c2_addr << 1);
            i2c2_phase = I2C2_ADDR_W;
        }
        break;
    case I2C2_ADDR_W:
        if (sr1 & I2C_SR1_ADDR) {
            (void)I2C1->SR2;  /* SR1 was already read; clear ADDR. */
            I2C1->DR = i2c2_reg;
            i2c2_phase = I2C2_REG_BTF;
        }
        break;
    case I2C2_REG_BTF:
        if (sr1 & I2C_SR1_BTF) {
            I2C1->CR1 |= I2C_CR1_START;  /* repeated START, no STOP */
            i2c2_phase = I2C2_RESTART;
        }
        break;
    case I2C2_RESTART:
        if (sr1 & I2C_SR1_SB) {
            I2C1->CR1 |= I2C_CR1_POS | I2C_CR1_ACK;
            I2C1->DR = ((uint32_t)i2c2_addr << 1) | 1UL;
            i2c2_phase = I2C2_ADDR_R;
        }
        break;
    case I2C2_ADDR_R:
        if (sr1 & I2C_SR1_ADDR) {
            /* Two-byte RX, NACK for second byte: POS=1, ACK=0 before ADDR clear. */
            I2C1->CR1 |= I2C_CR1_POS;
            I2C1->CR1 &= ~I2C_CR1_ACK;
            (void)I2C1->SR2;
            i2c2_phase = I2C2_RX_BTF;
        }
        break;
    case I2C2_RX_BTF:
        if (sr1 & I2C_SR1_BTF) {
            I2C1->CR1 |= I2C_CR1_STOP;
            i2c2_bytes[0] = (uint8_t)I2C1->DR;
            i2c2_bytes[1] = (uint8_t)I2C1->DR;
            I2C1->CR1 &= ~I2C_CR1_POS;
            I2C1->CR1 |= I2C_CR1_ACK;
            I2C2_Finish(I2C_OK);
        }
        break;
    default:
        I2C1->CR2 &= ~I2C_CR2_ITEVTEN;
        break;
    }
}

void I2C1_ER_IRQHandler(void)
{
    uint32_t flags = I2C1->SR1 & (I2C_SR1_BERR | I2C_SR1_ARLO |
                                    I2C_SR1_AF | I2C_SR1_OVR | I2C_SR1_TIMEOUT);
    if (flags) {
        /* Only write error bits back as 0; do not clear the event flags. */
        I2C1->SR1 &= ~flags;
        if (i2c2_phase != I2C2_IDLE && i2c2_phase != I2C2_DONE) {
            I2C2_Finish((flags & I2C_SR1_AF) ? I2C_NACK : I2C_ERROR);
        } else {
            I2C1->CR2 &= ~I2C_CR2_ITERREN;
        }
    }
}

I2C_Status I2C_Read2Async_Poll(uint8_t out[2], uint32_t *duration_us)
{
    I2C_Status status;
    if (i2c2_phase == I2C2_IDLE) return I2C_IDLE;
    if (i2c2_phase != I2C2_DONE) {
        if ((uint32_t)(micros() - i2c2_start_us) <= 2000UL) return I2C_BUSY;
        /* Main-context timeout recovery: NEVER reset I2C inside TIM11 ISR. */
        NVIC_DisableIRQ(I2C1_EV_IRQn);
        NVIC_DisableIRQ(I2C1_ER_IRQn);
        I2C1->CR2 &= ~(I2C_CR2_ITEVTEN | I2C_CR2_ITBUFEN | I2C_CR2_ITERREN);
        I2C1->CR1 |= I2C_CR1_STOP;
        I2C_Init(I2C_1, i2c2_fast_mode);
        i2c2_duration_us = micros() - i2c2_start_us;
        i2c2_result = I2C_TIMEOUT;
        i2c2_phase = I2C2_DONE;
        NVIC_ClearPendingIRQ(I2C1_EV_IRQn);
        NVIC_ClearPendingIRQ(I2C1_ER_IRQn);
    }
    status = i2c2_result;
    if (duration_us) *duration_us = i2c2_duration_us;
    if (out && status == I2C_OK) {
        out[0] = i2c2_bytes[0];
        out[1] = i2c2_bytes[1];
    }
    i2c2_result = I2C_IDLE;
    i2c2_phase = I2C2_IDLE;
    return status;
}
