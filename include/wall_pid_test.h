#ifndef WALL_PID_TEST_H
#define WALL_PID_TEST_H
#include "wall_pid_test_core.h"
#include <stdint.h>
int WallPidTest_Main(void);
/* Called only from TIM11 after encoder, IR EMA10, and MPU snapshot.
 * NEVER send BLE traffic from this ISR. */
void WallPidTest_ControlTick(int32_t left,int32_t right,
                             float yaw,float rate,
                             const IR_WallSnapshot *ir);
void WallPidTest_Stop(WP_Reason reason);
#endif
