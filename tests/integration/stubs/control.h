#ifndef MOCK_CONTROL_H
#define MOCK_CONTROL_H
#include <stdint.h>
typedef struct {uint32_t gaps_gt1500us, max_irq_cycles;} Control_DebugStats_t;
void Control_GetDebugStats(Control_DebugStats_t *out);
uint8_t Control_IsRunning(void);
#endif
