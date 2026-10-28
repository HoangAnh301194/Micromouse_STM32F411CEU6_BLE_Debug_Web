#ifndef WALL_PID_TRACE_H
#define WALL_PID_TRACE_H
#include "wall_pid_test_core.h"
#include <stdint.h>

/* Isolated ISR producer/main BLE consumer. Motor mode must already be
 * locked at replay. 200Hz x 5s fits in SRAM; no malloc/Flash/UART1. */
#define WPT_RECORD_CAPACITY 1004U
#define WPT_PERIOD_TICKS 5U
void WPT_Reset(void);
void WPT_Append(uint32_t tick, const WP_Input *input,
                const WP_Controller *control);
uint16_t WPT_Count(void);
uint16_t WPT_Dropped(void);
/* Begin/Service only from foreground after the motor is locked. */
void WPT_BeginReplay(void);
uint8_t WPT_ServiceReplay(uint32_t now_ms);
uint8_t WPT_ReplayComplete(void);
#endif
