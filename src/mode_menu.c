/* Menu selection only. Physical actions are intentionally NOT implemented. */
#include "mode_menu.h"

static const char *const menu_labels[MODE_MENU_ITEM_COUNT] = {
    "1.Calibration", "2.Run Slow", "3.Run Faster", "4.A* Run",
    "5.Speedrun", "6.Reset Map", "7.System Test"
};

static const char *const menu_codes[MODE_MENU_ITEM_COUNT] = {
    "CALIBRATION", "RUN_SLOW", "RUN_FASTER", "ASTAR_RUN",
    "SPEEDRUN", "RESET_MAP", "SYSTEM_TEST"
};

const char *ModeMenu_Label(uint8_t index)
{
    return index < MODE_MENU_ITEM_COUNT ? menu_labels[index] : "INVALID";
}

const char *ModeMenu_Code(uint8_t index)
{
    return index < MODE_MENU_ITEM_COUNT ? menu_codes[index] : "INVALID";
}

void ModeMenu_Init(ModeMenu *menu, uint8_t button_pressed, uint32_t now_ms)
{
    if (!menu) return;
    menu->focus = 0U;
    menu->selected = MODE_MENU_NO_SELECTION;
    menu->raw_pressed = button_pressed ? 1U : 0U;
    menu->stable_pressed = menu->raw_pressed;
    /* A key already held at boot must be released before a new action. */
    menu->armed = menu->raw_pressed ? 0U : 1U;
    menu->long_sent = 0U;
    menu->raw_changed_ms = now_ms;
    menu->pressed_at_ms = now_ms;
}

ModeMenuEvent ModeMenu_Poll(ModeMenu *menu, uint8_t button_pressed,
                            uint32_t now_ms)
{
    uint8_t raw;
    if (!menu) return MODE_MENU_EVENT_NONE;
    raw = button_pressed ? 1U : 0U;

    if (raw != menu->raw_pressed) {
        menu->raw_pressed = raw;
        menu->raw_changed_ms = now_ms;
    }

    if (raw != menu->stable_pressed &&
        (uint32_t)(now_ms - menu->raw_changed_ms) >= MODE_MENU_DEBOUNCE_MS) {
        menu->stable_pressed = raw;
        if (raw) {
            menu->pressed_at_ms = now_ms;
            menu->long_sent = 0U;
        } else {
            if (!menu->armed) {
                menu->armed = 1U;
                return MODE_MENU_EVENT_NONE;
            }
            if (!menu->long_sent) {
                if ((uint32_t)(now_ms - menu->pressed_at_ms) >=
                    MODE_MENU_LONG_PRESS_MS) {
                    menu->selected = menu->focus;
                    menu->long_sent = 1U;
                    return MODE_MENU_EVENT_SELECT;
                }
                menu->focus = (uint8_t)((menu->focus + 1U) % MODE_MENU_ITEM_COUNT);
                return MODE_MENU_EVENT_FOCUS;
            }
        }
    }

    if (menu->armed && menu->stable_pressed && !menu->long_sent &&
        (uint32_t)(now_ms - menu->pressed_at_ms) >= MODE_MENU_LONG_PRESS_MS) {
        menu->selected = menu->focus;
        menu->long_sent = 1U;
        return MODE_MENU_EVENT_SELECT;
    }

    return MODE_MENU_EVENT_NONE;
}
