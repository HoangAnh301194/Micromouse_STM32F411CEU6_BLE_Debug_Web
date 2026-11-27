#ifndef IR_CALIBRATION_STATIC_H
#define IR_CALIBRATION_STATIC_H
#include <stdint.h>
/* Opt-in v2 three-placement calibration. The motor never unlocks here. */
void IRStatic_Init(void);
void IRStatic_Start(void);
void IRStatic_Capture(uint32_t now);
void IRStatic_Poll(uint32_t now);
void IRStatic_Cancel(void);
void IRStatic_Exit(void);
uint8_t IRStatic_Active(void);
uint8_t IRStatic_Save(void);
void IRStatic_Status(void);
#endif
