#ifndef MOTOR_FUNCTIONAL_TRACE_H
#define MOTOR_FUNCTIONAL_TRACE_H

#include <stdint.h>

#if defined(MOTOR_AUTO_BENCH)
/* SRAM recorder for PA9/PA10 USART1 diagnostic builds only.
 * Capture is strictly observational: no PWM commands, no motor START. */
#define MOTOR_FUNCTIONAL_TRACE_SAMPLES 512U
void MotorFunctionalTrace_Init(void);
uint8_t MotorFunctionalTrace_Begin(void);
void MotorFunctionalTrace_Record1ms(uint32_t entry_us, uint32_t interval_us);
void MotorFunctionalTrace_Poll(uint32_t now_ms);
#endif

#endif
