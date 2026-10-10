#ifndef MOTOR_TRACE_H
#define MOTOR_TRACE_H
#include <stdint.h>

/* 512 samples at 1 kHz = 512 ms rolling window; RAM only, no UART in ISR. */
#define MOTOR_TRACE_CAPACITY 512U

void MotorTrace_Reset(void); /* called from main before starting a test */
void MotorTrace_Record1ms(void); /* called from TIM11 ISR */
void MotorTrace_RequestDump(void); /* main only; call after test ends */
void MotorTrace_Poll(uint32_t now_ms); /* main only; 1 CSV line per 20 ms */
#endif
