#ifndef BOARD_H
#define BOARD_H

#include "stm32f4xx.h"
#include <stdint.h>

void Board_Init(void);
/* Zero = clock PLL 100MHz success, nonzero = startup stage failure. */
uint32_t Board_GetClockFault(void);
/* Raw RCC_CR HSI trim/calibration fields; not a frequency measurement. */
uint32_t Board_GetHsiTrim(void);
uint32_t Board_GetHsiCal(void);
uint8_t Board_ButtonPressed(void);
void Board_StatusLed(uint8_t on);

uint32_t Board_GetAPB1ClockHz(void);
uint32_t Board_GetAPB2ClockHz(void);
uint32_t Board_GetAPB1TimerClockHz(void);
uint32_t Board_GetAPB2TimerClockHz(void);

#endif
