/* Menu selection only. Physical actions are intentionally NOT implemented. */
#include "mode_menu.h"

static const char *const menu_labels[MODE_MENU_ITEM_COUNT] = {
    "1.Calibration", "2.Run Slow", "3.Run Faster", "4.A* Run",
    "5.Speedrun", "6.Reset Map", "7.System Test"
};

static const char *const menu_names[MODE_MENU_ITEM_COUNT] = {
    "Calibration", "Run Slow", "Run Faster", "A* Run",
    "Speedrun", "Reset Map", "System Test"
};

static const char *const test_names[MODE_MENU_TEST_COUNT] = {
    "IR Test", "Encoder Test", "MPU Test", "All Sensors Test", "Back"
};

/* Encoder submenu: navigation and passive Log Receive only.
 * The movement options are inert placeholders in MODE_MENU_ONLY. */
static const char *const encoder_names[MODE_MENU_ENCODER_COUNT] = {
    "Log Receive",
    "1 Box Forward", "5 Box Forward",
    "Pivot 90 Left", "Pivot 90 Right",
    "Pivot 180 Left", "Pivot 180 Right", "PID", "Back"
};

const char *ModeMenu_EncoderName(uint8_t index)
{
    return index < MODE_MENU_ENCODER_COUNT ? encoder_names[index] : "INVALID";
}

const char *ModeMenu_TestName(uint8_t index)
{
    return index < MODE_MENU_TEST_COUNT ? test_names[index] : "INVALID";
}

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

const char *ModeMenu_Name(uint8_t index)
{
    return index < MODE_MENU_ITEM_COUNT ? menu_names[index] : "INVALID";
}

void ModeMenu_Init(ModeMenu *menu, uint8_t button_pressed, uint32_t now_ms)
{
    if (!menu) return;
    /* No mode is focused until the first PA0 short press. */
    menu->focus = MODE_MENU_NO_SELECTION;
    menu->selected = MODE_MENU_NO_SELECTION;
    menu->page = MODE_MENU_PAGE_MAIN;
    menu->test_focus = MODE_MENU_NO_SELECTION;
    menu->active_test = MODE_MENU_NO_SELECTION;
    menu->encoder_focus = MODE_MENU_NO_SELECTION;
    menu->active_encoder_option = MODE_MENU_NO_SELECTION;
    menu->stream_running = 0U;
    menu->raw_pressed = button_pressed ? 1U : 0U;
    menu->stable_pressed = menu->raw_pressed;
    /* A key already held at boot must be released before a new action. */
    menu->armed = menu->raw_pressed ? 0U : 1U;
    menu->long_sent = 0U;
    menu->raw_changed_ms = now_ms;
    menu->pressed_at_ms = now_ms;
}

ModeMenuEvent ModeMenu_Next(ModeMenu *menu)
{
    if (!menu) return MODE_MENU_EVENT_NONE;
    if (menu->page == MODE_MENU_PAGE_ENCODER) {
        /* Any browse action exits Log Receive and stops telemetry. */
        menu->active_test = MODE_MENU_NO_SELECTION;
        menu->active_encoder_option = MODE_MENU_NO_SELECTION;
        menu->stream_running = 0U;
        menu->encoder_focus = menu->encoder_focus == MODE_MENU_NO_SELECTION ? 0U :
            (uint8_t)((menu->encoder_focus + 1U) % MODE_MENU_ENCODER_COUNT);
        return MODE_MENU_EVENT_ENCODER_FOCUS;
    }
    if (menu->page == MODE_MENU_PAGE_TEST) {
        /* Browsing intentionally stops the previous passive stream. */
        menu->active_test = MODE_MENU_NO_SELECTION;
        menu->stream_running = 0U;
        menu->test_focus = menu->test_focus == MODE_MENU_NO_SELECTION ? 0U :
            (uint8_t)((menu->test_focus + 1U) % MODE_MENU_TEST_COUNT);
        return MODE_MENU_EVENT_TEST_FOCUS;
    }
    menu->focus = menu->focus == MODE_MENU_NO_SELECTION ? 0U :
        (uint8_t)((menu->focus + 1U) % MODE_MENU_ITEM_COUNT);
    return MODE_MENU_EVENT_FOCUS;
}

ModeMenuEvent ModeMenu_Select(ModeMenu *menu)
{
    if (!menu) return MODE_MENU_EVENT_NONE;
    if (menu->page == MODE_MENU_PAGE_ENCODER) {
        if (menu->encoder_focus == MODE_MENU_NO_SELECTION)
            return MODE_MENU_EVENT_NONE;
        menu->active_test = MODE_MENU_NO_SELECTION;
        menu->stream_running = 0U;
        menu->active_encoder_option = MODE_MENU_NO_SELECTION;
        if (menu->encoder_focus == MODE_MENU_ENCODER_BACK) {
            menu->page = MODE_MENU_PAGE_TEST;
            menu->encoder_focus = MODE_MENU_NO_SELECTION;
            return MODE_MENU_EVENT_ENCODER_EXIT;
        }
        menu->active_encoder_option = menu->encoder_focus;
        if (menu->encoder_focus == MODE_MENU_ENCODER_LOG_RECEIVE) {
            /* Entering Log Receive only ARMS telemetry. A separate 'G'
             * Start command is mandatory before the first ENC frame.
             * Every 'P' Stop returns to this armed, non-streaming state. */
            menu->active_test = 1U;
            menu->stream_running = 0U;
        }
        /* All six motion-labelled options only report selection.
         * No encoder sampling, no motor commands, no movement. */
        return MODE_MENU_EVENT_ENCODER_SELECT;
    }
    if (menu->page == MODE_MENU_PAGE_TEST) {
        if (menu->test_focus == MODE_MENU_NO_SELECTION)
            return MODE_MENU_EVENT_NONE;
        if (menu->test_focus == MODE_MENU_TEST_BACK) {
            menu->page = MODE_MENU_PAGE_MAIN;
            menu->active_test = MODE_MENU_NO_SELECTION;
            menu->stream_running = 0U;
            menu->test_focus = MODE_MENU_NO_SELECTION;
            return MODE_MENU_EVENT_TEST_EXIT;
        }
        if (menu->test_focus == MODE_MENU_TEST_ENCODER) {
            /* Encoder Test is a navigation page, not an active stream. */
            menu->page = MODE_MENU_PAGE_ENCODER;
            menu->encoder_focus = MODE_MENU_NO_SELECTION;
            menu->active_encoder_option = MODE_MENU_NO_SELECTION;
            menu->active_test = MODE_MENU_NO_SELECTION;
            menu->stream_running = 0U;
            return MODE_MENU_EVENT_ENCODER_ENTER;
        }
        menu->active_test = menu->test_focus;
        menu->stream_running = 1U; /* IR/MPU streams start immediately */
        return MODE_MENU_EVENT_TEST_SELECT;
    }
    if (menu->focus == MODE_MENU_NO_SELECTION) return MODE_MENU_EVENT_NONE;
    menu->selected = menu->focus;
    if (menu->selected == 6U) {
        menu->page = MODE_MENU_PAGE_TEST;
        menu->test_focus = MODE_MENU_NO_SELECTION;
        menu->active_test = MODE_MENU_NO_SELECTION;
        menu->encoder_focus = MODE_MENU_NO_SELECTION;
        menu->active_encoder_option = MODE_MENU_NO_SELECTION;
        menu->stream_running = 0U;
        return MODE_MENU_EVENT_TEST_ENTER;
    }
    return MODE_MENU_EVENT_SELECT;
}

uint8_t ModeMenu_PauseTest(ModeMenu *menu)
{
    if (!menu || (menu->page != MODE_MENU_PAGE_TEST &&
                  menu->page != MODE_MENU_PAGE_ENCODER) ||
        menu->active_test == MODE_MENU_NO_SELECTION ||
        (menu->page == MODE_MENU_PAGE_ENCODER &&
         menu->active_encoder_option != MODE_MENU_ENCODER_LOG_RECEIVE) ||
        !menu->stream_running)
        return 0U;
    menu->stream_running = 0U;
    return 1U;
}

uint8_t ModeMenu_ResumeTest(ModeMenu *menu)
{
    if (!menu || (menu->page != MODE_MENU_PAGE_TEST &&
                  menu->page != MODE_MENU_PAGE_ENCODER) ||
        menu->active_test == MODE_MENU_NO_SELECTION ||
        (menu->page == MODE_MENU_PAGE_ENCODER &&
         menu->active_encoder_option != MODE_MENU_ENCODER_LOG_RECEIVE) ||
        menu->stream_running)
        return 0U;
    menu->stream_running = 1U;
    return 1U;
}

uint8_t ModeMenu_CanResetSensor(const ModeMenu *menu, uint8_t test_index)
{
    if (!menu || test_index >= MODE_MENU_TEST_BACK)
        return 0U;
    if (menu->active_test != test_index) return 0U;
    if (menu->page == MODE_MENU_PAGE_TEST)
        return test_index != 1U; /* IR and MPU only on System Test page */
    return (menu->page == MODE_MENU_PAGE_ENCODER &&
            test_index == 1U &&
            menu->active_encoder_option == MODE_MENU_ENCODER_LOG_RECEIVE) ? 1U : 0U;
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
                    menu->long_sent = 1U;
                    return ModeMenu_Select(menu);
                }
                return ModeMenu_Next(menu);
            }
        }
    }

    if (menu->armed && menu->stable_pressed && !menu->long_sent &&
        (uint32_t)(now_ms - menu->pressed_at_ms) >= MODE_MENU_LONG_PRESS_MS) {
        menu->long_sent = 1U;
        return ModeMenu_Select(menu);
    }

    return MODE_MENU_EVENT_NONE;
}
