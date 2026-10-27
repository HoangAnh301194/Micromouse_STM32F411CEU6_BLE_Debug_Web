/* Central entrypoint only. Real application loops and diagnostic modes
 * reside in their own source files. Keep every exclusive main isolated. */
#if defined(TURN_PID_DIAG)
#include "turn_pid_diag.h"
int main(void) { return TurnPidDiag_Main(); }
#elif defined(SYSTEM_TEST_MODE)
#include "system_test.h"
int main(void) { return SystemTest_Main(); }
#elif defined(BLE_RX_DIAG)
int BleRxDiag_Main(void);
int main(void) { return BleRxDiag_Main(); }
#elif defined(OLED_MPU_DIAG)
int OLED_MPU_DiagMain(void);
int main(void) { return OLED_MPU_DiagMain(); }
#elif defined(IR_OLED_TEST)
int IR_OLED_TestMain(void);
int main(void) { return IR_OLED_TestMain(); }
#elif defined(IR_FILTER_DIAG)
int IR_FilterDiag_Main(void);
int main(void) { return IR_FilterDiag_Main(); }
#elif defined(BLE_PROFILE_DIAG)
int BleBenchmark_Main(void);
int main(void) { return BleBenchmark_Main(); }
#elif defined(FLASH_MAP_DIAG)
#include "flash_map_diag.h"
int main(void) { return FlashMapDiag_Main(); }
#else
int Main_Runtime(void);
int main(void) { return Main_Runtime(); }
#endif
