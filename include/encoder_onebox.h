#ifndef ENCODER_ONEBOX_H
#define ENCODER_ONEBOX_H
#include <stdint.h>
#include "encoder_onebox_core.h"

#if defined(MODE_MENU_ONLY)
typedef struct {
    uint8_t reason;
    int32_t count_l, count_r;
    uint32_t elapsed_ms;
    uint8_t motor_locked;
} EncoderOneBoxResult;

void EncoderOneBox_Init(void);
uint8_t EncoderOneBox_Start(void); /* SPP 'G' only, after menu 1 Box Forward */
void EncoderOneBox_HostPing(void);  /* SPP 'H' <=100ms, 350ms safety lease */
void EncoderOneBox_Abort(void);     /* SPP 'P' or menu navigation */
void EncoderOneBox_ControlTick(void); /* called from 1kHz TIM11 */
uint8_t EncoderOneBox_IsRunning(void);
/* After physical PA0 emergency stop, consume the press/release instead
 * of also treating it as a menu Next/Select event. */
uint8_t EncoderOneBox_SuppressMenuButton(void);
uint8_t EncoderOneBox_PopResult(EncoderOneBoxResult *out);
const char *EncoderOneBox_Reason(uint8_t reason);
#endif
#endif
