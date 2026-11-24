#ifndef MODE_MENU_H
#define MODE_MENU_H

#include <stdint.h>

/* Pure PA0 menu state machine: no motor, IR, Flash, or Bluetooth side effects.
 * The seven indices and labels match the old main-branch OLED menu. */
#define MODE_MENU_ITEM_COUNT 7U
#define MODE_MENU_LONG_PRESS_MS 400U
#define MODE_MENU_DEBOUNCE_MS 30U
#define MODE_MENU_NO_SELECTION 0xFFU
#define MODE_MENU_TEST_COUNT 5U
#define MODE_MENU_TEST_IR 0U
#define MODE_MENU_TEST_ENCODER 1U
#define MODE_MENU_TEST_MPU 2U
#define MODE_MENU_TEST_ALL 3U
#define MODE_MENU_TEST_BACK 4U
#define MODE_MENU_PAGE_MAIN 0U
#define MODE_MENU_PAGE_TEST 1U
#define MODE_MENU_PAGE_ENCODER 2U
/* Seven encoder submenu items plus Back; only Log Receive streams data. */
#define MODE_MENU_ENCODER_COUNT 8U
#define MODE_MENU_ENCODER_LOG_RECEIVE 0U
#define MODE_MENU_ENCODER_1BOX 1U
#define MODE_MENU_ENCODER_BACK 7U

typedef enum {
    MODE_MENU_EVENT_NONE = 0,
    MODE_MENU_EVENT_FOCUS = 1,
    MODE_MENU_EVENT_SELECT = 2,
    MODE_MENU_EVENT_TEST_ENTER = 3,
    MODE_MENU_EVENT_TEST_FOCUS = 4,
    MODE_MENU_EVENT_TEST_SELECT = 5,
    MODE_MENU_EVENT_TEST_EXIT = 6,
    MODE_MENU_EVENT_ENCODER_ENTER = 7,
    MODE_MENU_EVENT_ENCODER_FOCUS = 8,
    MODE_MENU_EVENT_ENCODER_SELECT = 9,
    MODE_MENU_EVENT_ENCODER_EXIT = 10
} ModeMenuEvent;

typedef struct {
    uint8_t focus;
    uint8_t selected;
    uint8_t page;
    uint8_t test_focus;
    uint8_t active_test;
    uint8_t encoder_focus;
    uint8_t active_encoder_option;
    uint8_t stream_running; /* test-only telemetry, never motor motion */
    uint8_t raw_pressed;
    uint8_t stable_pressed;
    uint8_t armed;
    uint8_t long_sent;
    uint32_t raw_changed_ms;
    uint32_t pressed_at_ms;
} ModeMenu;

void ModeMenu_Init(ModeMenu *menu, uint8_t button_pressed, uint32_t now_ms);
ModeMenuEvent ModeMenu_Poll(ModeMenu *menu, uint8_t button_pressed, uint32_t now_ms);
/* Shared pure menu events: physical KEY and safe one-byte SPP commands. */
ModeMenuEvent ModeMenu_Next(ModeMenu *menu);
ModeMenuEvent ModeMenu_Select(ModeMenu *menu);
/* Return 1 only on a real state change with a selected System Test. */
uint8_t ModeMenu_PauseTest(ModeMenu *menu);
uint8_t ModeMenu_ResumeTest(ModeMenu *menu);
/* Only the selected IR/Encoder/MPU test may accept its reset command. */
uint8_t ModeMenu_CanResetSensor(const ModeMenu *menu, uint8_t test_index);
const char *ModeMenu_TestName(uint8_t index);
const char *ModeMenu_EncoderName(uint8_t index);
const char *ModeMenu_Label(uint8_t index);
const char *ModeMenu_Code(uint8_t index);
const char *ModeMenu_Name(uint8_t index);

#endif
