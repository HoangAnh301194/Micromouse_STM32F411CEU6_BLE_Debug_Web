#ifndef OLED_SHARED_H
#define OLED_SHARED_H
#include "mpu6050.h"
#include <stdint.h>
typedef struct {
    uint32_t commands, chunks, data_bytes, frames;
    uint32_t busy_skips, admission_skips, errors;
} OLED_SharedStats;
void OLED_SharedInit(uint8_t found);
void OLED_SharedEnable(uint8_t on);
uint8_t OLED_SharedEnabled(void);
uint8_t OLED_SharedReady(void);
void OLED_SharedService(const MPU6050_Handle_t *mpu);
void OLED_SharedGetStats(OLED_SharedStats *out);
#endif
