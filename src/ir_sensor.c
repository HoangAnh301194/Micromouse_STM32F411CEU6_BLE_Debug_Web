/**
 * @file ir_sensor.c
 * @brief IR Sensors Processing with 7-State Paired Pulsing (TIM10)
 *        Pairs: L90+R45, L45+R90, L0+R0 (opposite/staggered = no cross-talk)
 *        Scan time: 7 x 100us = 0.7ms (was 1.3ms)
 */

#include "ir_sensor.h"
#include "hardware.h"
#include <string.h>

static volatile IR_Sensor_Handle_t ir_handle;

static const uint8_t adc_channels[6] = {
    IR_RX_L90_ADC_CHANNEL, IR_RX_L45_ADC_CHANNEL, IR_RX_L0_ADC_CHANNEL, 
    IR_RX_R0_ADC_CHANNEL,  IR_RX_R45_ADC_CHANNEL, IR_RX_R90_ADC_CHANNEL 
};

#ifndef RCC_APB2ENR_TIM10EN
#define RCC_APB2ENR_TIM10EN (1 << 17)
#endif

static void ADC_DMA_Config(void) {
    volatile uint32_t wait = 10000;
    RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;
    RCC->AHB1ENR |= RCC_AHB1ENR_DMA2EN;
    
    ADC1->CR1 = 0; ADC1->CR2 = 0;
    ADC1->CR1 |= ADC_CR1_SCAN;
    ADC1->CR2 |= ADC_CR2_DMA | ADC_CR2_DDS;
    ADC1->SMPR2 = 0x04924924; ADC1->SMPR1 = 0x04924924; 
    ADC1->SQR1 = (6 - 1) << 20;
    ADC1->SQR3 = (adc_channels[0] << 0) | (adc_channels[1] << 5) | (adc_channels[2] << 10) |
                 (adc_channels[3] << 15) | (adc_channels[4] << 20) | (adc_channels[5] << 25);   
    ADC1->CR2 |= ADC_CR2_ADON;
    while(wait--);
    
    DMA2_Stream0->CR &= ~DMA_SxCR_EN;
    while (DMA2_Stream0->CR & DMA_SxCR_EN); 
    DMA2->LIFCR = 0x3D; 
    DMA2_Stream0->CR = 0;
    DMA2_Stream0->CR |= (0 << 25) | (1 << 13) | (1 << 11) | (1 << 10) | DMA_SxCR_MINC; 
    DMA2_Stream0->PAR = (uint32_t)&ADC1->DR;
    DMA2_Stream0->M0AR = (uint32_t)ir_handle.dma_buffer;
}

static void ADC_TriggerAndWait(void) {
    DMA2_Stream0->CR &= ~DMA_SxCR_EN;
    while(DMA2_Stream0->CR & DMA_SxCR_EN); 
    
    DMA2->LIFCR = 0x3D;
    DMA2_Stream0->NDTR = 6;
    DMA2_Stream0->CR |= DMA_SxCR_EN;
    ADC1->CR2 |= ADC_CR2_SWSTART;
    
    /* Synchronous block for DMA transfer completion (takes ~6-10us) */
    while((DMA2->LISR & (1 << 5)) == 0);
    DMA2->LIFCR = 0x3D;
}

static void TIM10_Config(void) {
    RCC->APB2ENR |= RCC_APB2ENR_TIM10EN;

    TIM10->PSC = 96 - 1; /* Clock APB2 = 96MHz => 1MHz (1us/tick) */
    TIM10->ARR = 100 - 1; /* Chu ky 100us */
    TIM10->DIER |= TIM_DIER_UIE; 

    NVIC_SetPriority(TIM1_UP_TIM10_IRQn, 1); 
    NVIC_EnableIRQ(TIM1_UP_TIM10_IRQn);

    TIM10->CR1 |= TIM_CR1_CEN; 
}

void TIM1_UP_TIM10_IRQHandler(void) {
    if (TIM10->SR & TIM_SR_UIF) {
        TIM10->SR &= ~TIM_SR_UIF;

        if (ir_handle.state == IR_STATE_IDLE || ir_handle.state == IR_STATE_READY) {
            return;
        }

        switch (ir_handle.state) {
            case IR_STATE_AMBIENT:
                Hardware_IRLedAllOff();
                ADC_TriggerAndWait(); 
                memcpy((void*)ir_handle.ambient, (void*)ir_handle.dma_buffer, 12);
                ir_handle.state = IR_STATE_PAIR1_ON;
                break;

            /* === Pair 1: L90 (front-left) + R45 (diagonal-right) === */
            case IR_STATE_PAIR1_ON:
                Hardware_IRLedOn(IR_L90);
                Hardware_IRLedOn(IR_R45);
                ir_handle.state = IR_STATE_PAIR1_READ;
                break;

            case IR_STATE_PAIR1_READ:
                ADC_TriggerAndWait();
                if (ir_handle.dma_buffer[0] > ir_handle.ambient[0])
                    ir_handle.result[0] = ir_handle.dma_buffer[0] - ir_handle.ambient[0];
                else ir_handle.result[0] = 0;
                if (ir_handle.dma_buffer[4] > ir_handle.ambient[4])
                    ir_handle.result[4] = ir_handle.dma_buffer[4] - ir_handle.ambient[4];
                else ir_handle.result[4] = 0;
                Hardware_IRLedOff(IR_L90);
                Hardware_IRLedOff(IR_R45);
                ir_handle.state = IR_STATE_PAIR2_ON;
                break;

            /* === Pair 2: L45 (diagonal-left) + R90 (front-right) === */
            case IR_STATE_PAIR2_ON:
                Hardware_IRLedOn(IR_L45);
                Hardware_IRLedOn(IR_R90);
                ir_handle.state = IR_STATE_PAIR2_READ;
                break;

            case IR_STATE_PAIR2_READ:
                ADC_TriggerAndWait();
                if (ir_handle.dma_buffer[1] > ir_handle.ambient[1])
                    ir_handle.result[1] = ir_handle.dma_buffer[1] - ir_handle.ambient[1];
                else ir_handle.result[1] = 0;
                if (ir_handle.dma_buffer[5] > ir_handle.ambient[5])
                    ir_handle.result[5] = ir_handle.dma_buffer[5] - ir_handle.ambient[5];
                else ir_handle.result[5] = 0;
                Hardware_IRLedOff(IR_L45);
                Hardware_IRLedOff(IR_R90);
                ir_handle.state = IR_STATE_PAIR3_ON;
                break;

            /* === Pair 3: L0 (side-left) + R0 (side-right) === */
            case IR_STATE_PAIR3_ON:
                Hardware_IRLedOn(IR_L0);
                Hardware_IRLedOn(IR_R0);
                ir_handle.state = IR_STATE_PAIR3_READ;
                break;

            case IR_STATE_PAIR3_READ:
                ADC_TriggerAndWait();
                if (ir_handle.dma_buffer[2] > ir_handle.ambient[2])
                    ir_handle.result[2] = ir_handle.dma_buffer[2] - ir_handle.ambient[2];
                else ir_handle.result[2] = 0;
                if (ir_handle.dma_buffer[3] > ir_handle.ambient[3])
                    ir_handle.result[3] = ir_handle.dma_buffer[3] - ir_handle.ambient[3];
                else ir_handle.result[3] = 0;
                Hardware_IRLedOff(IR_L0);
                Hardware_IRLedOff(IR_R0);
                ir_handle.scan_count++;
                ir_handle.state = IR_STATE_READY;
                break;

            default: break;
        }
    }
}

void IR_Sensor_Init(void) {
    memset((void*)&ir_handle, 0, sizeof(ir_handle));
    ir_handle.state = IR_STATE_IDLE;
    ADC_DMA_Config();
    TIM10_Config();
}

void IR_Sensor_StartScan(void) {
    if (ir_handle.state == IR_STATE_IDLE || ir_handle.state == IR_STATE_READY) {
        ir_handle.state = IR_STATE_AMBIENT;
    }
}

uint8_t IR_Sensor_IsReady(void) {
    return (ir_handle.state == IR_STATE_READY);
}

void IR_Sensor_GetResults(uint16_t *output) {
    if (output) memcpy(output, (void*)ir_handle.result, 12);
    ir_handle.state = IR_STATE_IDLE;
}

uint16_t IR_Sensor_Read(IR_Sensor_t sensor) {
    if (sensor >= 6) return 0;
    return ir_handle.result[sensor];
}

void IR_Sensor_Reset(void) {
    Hardware_IRLedAllOff();
    ir_handle.state = IR_STATE_IDLE;
}
