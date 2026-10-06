#ifndef IR_SENSOR_H
#define IR_SENSOR_H

#include "pinout.h"
#include <stdint.h>

typedef struct {
    uint16_t value[IR_SENSOR_COUNT];
    uint32_t scan_count;
} IR_Data_t;

void IR_Init(void);
void IR_StartScan(void);
void IR_Stop(void);
uint8_t IR_IsReady(void);
uint8_t IR_GetLatest(IR_Data_t *out);

#endif
