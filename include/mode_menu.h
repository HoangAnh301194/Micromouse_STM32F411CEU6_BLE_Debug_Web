#ifndef MODE_MENU_H
#define MODE_MENU_H
#include <stdint.h>
#define MODE_MENU_ITEM_COUNT 7U
#define MODE_MENU_LONG_PRESS_MS 400U
#define MODE_MENU_DEBOUNCE_MS 30U
#define MODE_MENU_NO_SELECTION 0xFFU
#define MODE_MENU_TEST_COUNT 8U
#define MODE_MENU_TEST_BACK 7U
#define MODE_MENU_PAGE_MAIN 0U
#define MODE_MENU_PAGE_TEST 1U
#define MODE_MENU_PAGE_ENCODER 2U
#define MODE_MENU_ENCODER_COUNT 9U
#define MODE_MENU_ENCODER_LOG_RECEIVE 0U
#define MODE_MENU_ENCODER_ONE_BOX_FORWARD 1U
typedef enum {
 MODE_MENU_EVENT_NONE=0, MODE_MENU_EVENT_FOCUS=1, MODE_MENU_EVENT_SELECT=2,
 MODE_MENU_EVENT_TEST_ENTER=3, MODE_MENU_EVENT_TEST_FOCUS=4,
 MODE_MENU_EVENT_TEST_SELECT=5, MODE_MENU_EVENT_TEST_EXIT=6,
 MODE_MENU_EVENT_ENCODER_ENTER=7, MODE_MENU_EVENT_ENCODER_FOCUS=8,
 MODE_MENU_EVENT_ENCODER_SELECT=9, MODE_MENU_EVENT_ENCODER_EXIT=10
} ModeMenuEvent;
typedef struct {
 uint8_t focus,selected,page,test_focus,active_test,encoder_focus;
 uint8_t active_encoder_option,active_group,active_item,stream_running;
 uint8_t raw_pressed,stable_pressed,armed,long_sent;
 uint32_t raw_changed_ms,pressed_at_ms;
} ModeMenu;
void ModeMenu_Init(ModeMenu*,uint8_t,uint32_t);
ModeMenuEvent ModeMenu_Poll(ModeMenu*,uint8_t,uint32_t);
ModeMenuEvent ModeMenu_Next(ModeMenu*);
ModeMenuEvent ModeMenu_Select(ModeMenu*);
uint8_t ModeMenu_PauseTest(ModeMenu*);
uint8_t ModeMenu_ResumeTest(ModeMenu*);
uint8_t ModeMenu_CanResetSensor(const ModeMenu*,uint8_t);
const char *ModeMenu_TestName(uint8_t);
const char *ModeMenu_ItemName(uint8_t,uint8_t);
uint8_t ModeMenu_ItemCount(uint8_t);
const char *ModeMenu_StreamName(uint8_t);
const char *ModeMenu_Label(uint8_t);
const char *ModeMenu_Code(uint8_t);
const char *ModeMenu_Name(uint8_t);
#endif
