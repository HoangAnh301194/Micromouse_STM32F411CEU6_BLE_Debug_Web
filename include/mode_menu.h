#ifndef MODE_MENU_H
#define MODE_MENU_H

#include <stdint.h>

/* Pure PA0 menu state machine: no motor, IR, Flash, or Bluetooth side effects.
 * The seven indices and labels match the old main-branch OLED menu. */
#define MODE_MENU_ITEM_COUNT 7U
#define MODE_MENU_LONG_PRESS_MS 400U
#define MODE_MENU_DEBOUNCE_MS 30U
#define MODE_MENU_NO_SELECTION 0xFFU

typedef enum {
    MODE_MENU_EVENT_NONE = 0,
    MODE_MENU_EVENT_FOCUS = 1,
    MODE_MENU_EVENT_SELECT = 2
} ModeMenuEvent;

typedef struct {
    uint8_t focus;
    uint8_t selected;
    uint8_t raw_pressed;
    uint8_t stable_pressed;
    uint8_t armed;
    uint8_t long_sent;
    uint32_t raw_changed_ms;
    uint32_t pressed_at_ms;
} ModeMenu;

void ModeMenu_Init(ModeMenu *menu, uint8_t button_pressed, uint32_t now_ms);
ModeMenuEvent ModeMenu_Poll(ModeMenu *menu, uint8_t button_pressed, uint32_t now_ms);
const char *ModeMenu_Label(uint8_t index);
const char *ModeMenu_Code(uint8_t index);
const char *ModeMenu_Name(uint8_t index);

#endif
