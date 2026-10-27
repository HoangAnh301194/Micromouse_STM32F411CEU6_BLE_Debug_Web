#ifndef IR_FILTER_H
#define IR_FILTER_H
#include <stdint.h>
#define IR_FILTER_CHANNELS 6U

/* Independent, pure-C candidate filters for six ambient-subtracted
 * infrared ADC channels. The input must be max(LED_ON - LED_OFF, 0),
 * never the uncorrected LED_ON sample.
 * This module never touches the motor, ISR, DMA, UART or LUT.
 * Alpha values are candidates, NOT calibrated choices. */
typedef struct {
    uint8_t initialized;
    uint8_t ring_index;
    uint8_t ring_count;
    uint16_t ring[IR_FILTER_CHANNELS][3];
    float ema10[IR_FILTER_CHANNELS];
    float ema25[IR_FILTER_CHANNELS];
    float median_ema25[IR_FILTER_CHANNELS];
} IR_Filter_t;

typedef struct {
    uint16_t raw[IR_FILTER_CHANNELS];
    uint16_t median3[IR_FILTER_CHANNELS];
    uint16_t ema10[IR_FILTER_CHANNELS];
    uint16_t ema25[IR_FILTER_CHANNELS];
    uint16_t median_ema25[IR_FILTER_CHANNELS];
} IR_FilterFrame_t;

void IR_FilterReset(IR_Filter_t *state);
void IR_FilterStep(IR_Filter_t *state,
                   const uint16_t raw[IR_FILTER_CHANNELS],
                   IR_FilterFrame_t *out);
#endif
