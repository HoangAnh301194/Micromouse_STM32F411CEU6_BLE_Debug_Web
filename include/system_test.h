#ifndef SYSTEM_TEST_H
#define SYSTEM_TEST_H
#include <stdint.h>
#if defined(SYSTEM_TEST_MODE)
int SystemTest_Main(void);
void SystemTest_ControlTick(int32_t left, int32_t right, float yaw, float rate);
void SystemTest_Stop(uint8_t reason);
#endif
#endif
