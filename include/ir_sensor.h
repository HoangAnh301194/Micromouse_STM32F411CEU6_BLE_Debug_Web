/**
 * @file ir_sensor.h
 * @brief IR Distance Sensors with 13-stage DMA and Ambient Compensation
 */

#ifndef IR_SENSOR_H
#define IR_SENSOR_H

#include <stdint.h>
#include "pinout.h"

/* 13-State Machine for Sequential Pulsing */
typedef enum {
    IR_STATE_IDLE = 0,
    IR_STATE_AMBIENT,       /* 1: Read ambient (all LEDs off) */
    IR_STATE_PAIR1_ON,      /* 2: Turn on L90 + R45 */
    IR_STATE_PAIR1_READ,    /* 3: Read L90 + R45, turn off */
    IR_STATE_PAIR2_ON,      /* 4: Turn on L45 + R90 */
    IR_STATE_PAIR2_READ,    /* 5: Read L45 + R90, turn off */
    IR_STATE_PAIR3_ON,      /* 6: Turn on L0 + R0 */
    IR_STATE_PAIR3_READ,    /* 7: Read L0 + R0, turn off */
    IR_STATE_READY          /* 8: Scan Complete */
} IR_State_t;

typedef struct {
    uint16_t dma_buffer[6];  
    uint16_t ambient[6];     
    uint16_t result[6];      

    volatile IR_State_t state;
    uint32_t scan_count;
} IR_Sensor_Handle_t;

/* Public API */
void IR_Sensor_Init(void);
void IR_Sensor_StartScan(void);
uint8_t IR_Sensor_IsReady(void);
void IR_Sensor_GetResults(uint16_t *output);
uint16_t IR_Sensor_Read(IR_Sensor_t sensor);
void IR_Sensor_Reset(void);

#endif /* IR_SENSOR_H */
