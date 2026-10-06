#ifndef ENCODER_H
#define ENCODER_H

#include <stdint.h>

#define ENCODER_LEFT_DIRECTION   1
#define ENCODER_RIGHT_DIRECTION  1

void Encoder_Init(void);
void Encoder_Update(float dt_s);
void Encoder_Reset(void);

int32_t Encoder_GetLeftCount(void);
int32_t Encoder_GetRightCount(void);
float Encoder_GetLeftSpeedPPS(void);
float Encoder_GetRightSpeedPPS(void);

#endif
