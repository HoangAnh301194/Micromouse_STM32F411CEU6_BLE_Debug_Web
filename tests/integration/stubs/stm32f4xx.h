#ifndef MOCK_STM32F4_H
#define MOCK_STM32F4_H
#include <stdint.h>
uint32_t __get_PRIMASK(void);
void __disable_irq(void);
void __enable_irq(void);
#endif
