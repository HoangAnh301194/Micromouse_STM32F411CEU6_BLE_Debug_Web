/**
 * @file i2c.c
 * @brief I2C Driver with DMA Support - COMPLETE FIXED
 * @author HoangAnh
 * @date 2025
 */

#include "i2c.h"
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
    volatile uint32_t sr1 = (*(volatile uint32_t *)(i2c_base + I2C_SR1));
    (*(volatile uint32_t *)(i2c_base + I2C_SR1)) = 0;
    (void)sr1;
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
    uint32_t i2c_base = I2C_GetBase(instance);
    
    RCC_AHB1ENR |= (1 << 1);
    
    if(instance == I2C_1) {
        RCC_APB1ENR |= (1 << 21);
        
        GPIOB_MODER &= ~((3UL << 16) | (3UL << 18));
        GPIOB_MODER |= (2UL << 16) | (2UL << 18);
        GPIOB_OTYPER |= (1 << 8) | (1 << 9);
        GPIOB_OSPEEDR |= (3UL << 16) | (3UL << 18);
        GPIOB_PUPDR &= ~((3UL << 16) | (3UL << 18));
        GPIOB_PUPDR |= (1UL << 16) | (1UL << 18);
        GPIOB_AFRH &= ~((0xFUL << 0) | (0xFUL << 4));
        GPIOB_AFRH |= (4UL << 0) | (4UL << 4);
    } else {
        return;
    }
    
    I2C_SoftReset(i2c_base);
    (*(volatile uint32_t *)(i2c_base + I2C_CR2)) = 50;
    
    if(fast_mode) {
        (*(volatile uint32_t *)(i2c_base + I2C_CCR)) = (1 << 15) | 42;
        (*(volatile uint32_t *)(i2c_base + I2C_TRISE)) = 16;
    } else {
        (*(volatile uint32_t *)(i2c_base + I2C_CCR)) = 250;
        (*(volatile uint32_t *)(i2c_base + I2C_TRISE)) = 51;
    }
    
    (*(volatile uint32_t *)(i2c_base + I2C_CR1)) |= I2C_CR1_PE;
}

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

I2C_Status I2C_Read(I2C_Instance instance, uint8_t slave_addr, 
                    uint8_t *data, uint16_t len) {
    uint32_t i2c_base = I2C_GetBase(instance);
    uint32_t start_time;
    uint16_t i;
    
    if (I2C_WaitBusFree(i2c_base) != I2C_OK) {
        I2C_Init(instance, 1);
        delay_us_blocking(100);
    }
    
    if(I2C_Start(i2c_base) != I2C_OK) return I2C_TIMEOUT;
    (*(volatile uint32_t *)(i2c_base + I2C_CR1)) |= I2C_CR1_ACK;
    
    if(I2C_SendAddress(i2c_base, (slave_addr << 1) | 1) != I2C_OK) {
        I2C_Stop(i2c_base);
        return I2C_NACK;
    }
    
    for(i = 0; i < len; i++) {
        if(i == len - 1) {
            (*(volatile uint32_t *)(i2c_base + I2C_CR1)) &= ~I2C_CR1_ACK;
        }
        
        start_time = millis();
        while(!((*(volatile uint32_t *)(i2c_base + I2C_SR1)) & I2C_SR1_RXNE)) {
            if(delay_elapsed_ms(start_time, I2C_TIMEOUT_MS)) {
                I2C_Stop(i2c_base);
                return I2C_TIMEOUT;
            }
        }
        data[i] = (*(volatile uint32_t *)(i2c_base + I2C_DR));
    }
    
    I2C_Stop(i2c_base);
    return I2C_OK;
}

I2C_Status I2C_WriteRegister(I2C_Instance instance, uint8_t slave_addr, 
                             uint8_t reg, uint8_t value) {
    uint8_t data[2];
    data[0] = reg;
    data[1] = value;
    return I2C_Write(instance, slave_addr, data, 2);
}

I2C_Status I2C_ReadRegister(I2C_Instance instance, uint8_t slave_addr, 
                            uint8_t reg, uint8_t *value) {
    uint32_t i2c_base = I2C_GetBase(instance);
    uint32_t start_time;
    
    if (I2C_WaitBusFree(i2c_base) != I2C_OK) I2C_Init(instance, 1);
    if(I2C_Start(i2c_base) != I2C_OK) return I2C_TIMEOUT;
    if(I2C_SendAddress(i2c_base, slave_addr << 1) != I2C_OK) {
        I2C_Stop(i2c_base);
        return I2C_NACK;
    }
    
    start_time = millis();
    while(!((*(volatile uint32_t *)(i2c_base + I2C_SR1)) & I2C_SR1_TXE)) {
        if(delay_elapsed_ms(start_time, I2C_TIMEOUT_MS)) {
            I2C_Stop(i2c_base);
            return I2C_TIMEOUT;
        }
    }
    (*(volatile uint32_t *)(i2c_base + I2C_DR)) = reg;
    
    start_time = millis();
    while(!((*(volatile uint32_t *)(i2c_base + I2C_SR1)) & I2C_SR1_BTF)) {
        if(delay_elapsed_ms(start_time, I2C_TIMEOUT_MS)) {
            I2C_Stop(i2c_base);
            return I2C_TIMEOUT;
        }
    }
    
    return I2C_Read(instance, slave_addr, value, 1);
}

I2C_Status I2C_ReadRegisters(I2C_Instance instance, uint8_t slave_addr, 
                             uint8_t reg, uint8_t *data, uint16_t len) {
    uint32_t i2c_base = I2C_GetBase(instance);
    uint32_t start_time;
    
    if (I2C_WaitBusFree(i2c_base) != I2C_OK) I2C_Init(instance, 1);
    if(I2C_Start(i2c_base) != I2C_OK) return I2C_TIMEOUT;
    if(I2C_SendAddress(i2c_base, slave_addr << 1) != I2C_OK) {
        I2C_Stop(i2c_base);
        return I2C_NACK;
    }
    
    start_time = millis();
    while(!((*(volatile uint32_t *)(i2c_base + I2C_SR1)) & I2C_SR1_TXE)) {
        if(delay_elapsed_ms(start_time, I2C_TIMEOUT_MS)) {
            I2C_Stop(i2c_base);
            return I2C_TIMEOUT;
        }
    }
    (*(volatile uint32_t *)(i2c_base + I2C_DR)) = reg;
    
    start_time = millis();
    while(!((*(volatile uint32_t *)(i2c_base + I2C_SR1)) & I2C_SR1_BTF)) {
        if(delay_elapsed_ms(start_time, I2C_TIMEOUT_MS)) {
            I2C_Stop(i2c_base);
            return I2C_TIMEOUT;
        }
    }
    
    return I2C_Read(instance, slave_addr, data, len);
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
