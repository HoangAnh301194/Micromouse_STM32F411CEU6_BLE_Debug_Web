#ifndef IR_SENSOR_H
#define IR_SENSOR_H

#include "pinout.h"
#include <stdint.h>

typedef struct {
    uint16_t value[IR_SENSOR_COUNT];
    uint32_t scan_count;
    uint32_t completed_us;
} IR_Data_t;

#if defined(INTEGRATION_DIAG)
typedef struct {
    uint32_t scans_started;
    uint32_t scans_completed;
    uint32_t scans_failed;
    uint32_t start_busy;
    uint32_t dma_started;
    uint32_t dma_completed;
    uint32_t dma_errors;
    uint32_t dma_timeouts;
    uint32_t adc_overruns;
    uint32_t scan_total_us;
    uint32_t scan_max_us;
    uint32_t dma_total_us;
    uint32_t dma_max_us;
    uint32_t tim10_irq_count;
    uint32_t tim10_irq_total_cycles;
    uint32_t tim10_irq_max_cycles;
    uint32_t dma_irq_count;
    uint32_t dma_irq_total_cycles;
    uint32_t dma_irq_max_cycles;
    uint32_t ambient_reads;
    uint32_t pair1_reads;
    uint32_t pair2_reads;
    uint32_t pair3_reads;
    uint32_t raw_scan_count;
    uint16_t ambient_raw[IR_SENSOR_COUNT];
    uint16_t emitter_raw[IR_SENSOR_COUNT];
} IR_Diagnostics_t;

void IR_GetDiagnostics(IR_Diagnostics_t *out);
#endif

void IR_Init(void);
void IR_StartScan(void);
void IR_Stop(void);
uint8_t IR_IsReady(void);
uint8_t IR_GetLatest(IR_Data_t *out);

#endif
