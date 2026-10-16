#ifndef BOARD_H
#define BOARD_H

#include "stm32f4xx.h"
#include <stdint.h>

void Board_Init(void);
/* RCC/PLL startup diagnostic, not an independent clock measurement. */
uint32_t Board_GetClockFault(void);
uint32_t Board_GetHsiTrim(void);
uint32_t Board_GetHsiCal(void);
uint8_t Board_ButtonPressed(void);
void Board_StatusLed(uint8_t on);

uint32_t Board_GetAPB1ClockHz(void);
uint32_t Board_GetAPB2ClockHz(void);
uint32_t Board_GetAPB1TimerClockHz(void);
uint32_t Board_GetAPB2TimerClockHz(void);

#endif
