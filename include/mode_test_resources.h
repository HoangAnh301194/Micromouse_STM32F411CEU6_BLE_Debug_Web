#ifndef MODE_TEST_RESOURCES_H
#define MODE_TEST_RESOURCES_H
#include <stdint.h>
#include "mode_menu.h"

/* Logical capabilities assigned per menu selection (no power rail switching).
 * Encoder hardware counters and motor watchdog always remain initialized;
 * sensor acquisition/telemetry is routed by active mode.
 */
#define TEST_RES_IR       0x01U
#define TEST_RES_MPU      0x02U
#define TEST_RES_ENCODER  0x04U
#define TEST_RES_MOTOR    0x08U

uint8_t ModeTest_Resources(const ModeMenu *menu);
uint8_t ModeTest_OneBoxArmed(const ModeMenu *menu);

#endif
