#include "mode_menu.h"

/* Mode 9 replaces the old Mode 7 System Test in the same menu slot. */
static const char *const menu_labels[7] = {
    "1.Calibration", "2.Run Slow", "3.Run Faster", "4.A* Run",
    "5.Speedrun", "6.Reset Map", "9.System Test"
};
static const char *const menu_names[7] = {
    "Calibration", "Run Slow", "Run Faster", "A* Run",
    "Speedrun", "Reset Map", "System Test"
};
static const char *const menu_codes[7] = {
    "CALIBRATION", "RUN_SLOW", "RUN_FASTER", "ASTAR_RUN",
    "SPEEDRUN", "RESET_MAP", "SYSTEM_TEST"
};
static const char *const group_names[8] = {
    "IR SENSOR TEST", "ENCODER TEST", "MPU TEST", "MOTOR TEST",
    "PID TEST", "BLUETOOTH TEST", "SYSTEM INTEGRATION", "BACK"
};
static const char *const ir_items[] = {
    "IR Live Data", "IR Filter", "IR Burst Capture", "IR Calibration", "Back"
};
static const char *const encoder_items[] = {
    "Log Receive", "1 Box Forward", "5 Box Forward", "Pivot 90 Left",
    "Pivot 90 Right", "Pivot 180 Left", "Pivot 180 Right", "Encoder Speed", "Back"
};
static const char *const mpu_items[] = {
    "Gyro Data", "Yaw Monitor", "MPU Calibration", "I2C Status", "Back"
};
static const char *const motor_items[] = {
    "Left Motor", "Right Motor", "Both Motors", "PWM Sweep", "Motor Driver", "Back"
};
static const char *const pid_items[] = {
    "Speed PID", "Turn PID", "Wall PID", "Back"
};
static const char *const bluetooth_items[] = {
    "SPP Connection", "BLE RX", "Baudrate Test", "SPP Throughput", "Back"
};
static const char *const integration_items[] = {
    "Clock Validation", "UART Benchmark", "Timer/DMA", "I2C Test", "Flash Map Test", "Back"
};
static const char *const *const group_items[7] = {
    ir_items, encoder_items, mpu_items, motor_items,
    pid_items, bluetooth_items, integration_items
};
static const uint8_t group_item_counts[7] = {5U, 9U, 5U, 6U, 4U, 5U, 6U};

uint8_t ModeMenu_ItemCount(uint8_t group_index)
{
    return group_index < 7U ? group_item_counts[group_index] : 0U;
}

const char *ModeMenu_TestName(uint8_t group_index)
{
    return group_index < 8U ? group_names[group_index] : "INVALID";
}

const char *ModeMenu_ItemName(uint8_t group_index, uint8_t item_index)
{
    if (group_index >= 7U || item_index >= group_item_counts[group_index])
        return "INVALID";
    return group_items[group_index][item_index];
}

const char *ModeMenu_StreamName(uint8_t stream_index)
{
    switch (stream_index) {
    case 0U: return "IR Live Data";
    case 1U: return "Encoder Log Receive";
    case 2U: return "MPU Gyro Data";
    default: return "No live stream";
    }
}

const char *ModeMenu_Label(uint8_t index)
{
    return index < 7U ? menu_labels[index] : "INVALID";
}

const char *ModeMenu_Code(uint8_t index)
{
    return index < 7U ? menu_codes[index] : "INVALID";
}

const char *ModeMenu_Name(uint8_t index)
{
    return index < 7U ? menu_names[index] : "INVALID";
}

void ModeMenu_Init(ModeMenu *menu, uint8_t button_pressed, uint32_t now_ms)
{
    if (!menu) return;

    menu->focus = MODE_MENU_NO_SELECTION;
    menu->selected = MODE_MENU_NO_SELECTION;
    menu->page = MODE_MENU_PAGE_MAIN;
    menu->test_focus = MODE_MENU_NO_SELECTION;
    menu->active_test = MODE_MENU_NO_SELECTION;
    menu->encoder_focus = MODE_MENU_NO_SELECTION;
    menu->active_encoder_option = MODE_MENU_NO_SELECTION;
    menu->active_group = MODE_MENU_NO_SELECTION;
    menu->active_item = MODE_MENU_NO_SELECTION;
    menu->stream_running = 0U;
    menu->raw_pressed = button_pressed ? 1U : 0U;
    menu->stable_pressed = menu->raw_pressed;
    menu->armed = menu->raw_pressed ? 0U : 1U;
    menu->long_sent = 0U;
    menu->raw_changed_ms = now_ms;
    menu->pressed_at_ms = now_ms;
}

ModeMenuEvent ModeMenu_Next(ModeMenu *menu)
{
    uint8_t count;

    if (!menu) return MODE_MENU_EVENT_NONE;

    if (menu->page == MODE_MENU_PAGE_ENCODER) {
        menu->active_test = MODE_MENU_NO_SELECTION;
        menu->active_encoder_option = MODE_MENU_NO_SELECTION;
        menu->active_item = MODE_MENU_NO_SELECTION;
        menu->stream_running = 0U;
        count = ModeMenu_ItemCount(menu->active_group);
        if (count == 0U) return MODE_MENU_EVENT_NONE;
        menu->encoder_focus = menu->encoder_focus == MODE_MENU_NO_SELECTION ? 0U :
            (uint8_t)((menu->encoder_focus + 1U) % count);
        return MODE_MENU_EVENT_ENCODER_FOCUS;
    }

    if (menu->page == MODE_MENU_PAGE_TEST) {
        menu->active_test = MODE_MENU_NO_SELECTION;
        menu->active_group = MODE_MENU_NO_SELECTION;
        menu->active_item = MODE_MENU_NO_SELECTION;
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
    uint8_t count;

    if (!menu) return MODE_MENU_EVENT_NONE;

    if (menu->page == MODE_MENU_PAGE_ENCODER) {
        count = ModeMenu_ItemCount(menu->active_group);
        if (menu->encoder_focus == MODE_MENU_NO_SELECTION || count == 0U)
            return MODE_MENU_EVENT_NONE;

        menu->active_test = MODE_MENU_NO_SELECTION;
        menu->active_encoder_option = MODE_MENU_NO_SELECTION;
        menu->active_item = MODE_MENU_NO_SELECTION;
        menu->stream_running = 0U;

        if (menu->encoder_focus == (uint8_t)(count - 1U)) {
            menu->page = MODE_MENU_PAGE_TEST;
            menu->encoder_focus = MODE_MENU_NO_SELECTION;
            return MODE_MENU_EVENT_ENCODER_EXIT;
        }

        menu->active_item = menu->encoder_focus;
        menu->active_encoder_option = menu->encoder_focus;

        /* Existing passive streams remain available. Other tests are menu
         * entries only until their runtime modules are safely integrated. */
        if (menu->active_group == 0U && menu->active_item == 0U) {
            menu->active_test = 0U;
            menu->stream_running = 1U;
        } else if (menu->active_group == 1U && menu->active_item == 0U) {
            menu->active_test = 1U; /* Start is an explicit host G command. */
        } else if (menu->active_group == 2U &&
                   (menu->active_item == 0U || menu->active_item == 1U)) {
            menu->active_test = 2U;
            menu->stream_running = 1U;
        }
        return MODE_MENU_EVENT_ENCODER_SELECT;
    }

    if (menu->page == MODE_MENU_PAGE_TEST) {
        if (menu->test_focus == MODE_MENU_NO_SELECTION)
            return MODE_MENU_EVENT_NONE;

        if (menu->test_focus == MODE_MENU_TEST_BACK) {
            menu->page = MODE_MENU_PAGE_MAIN;
            menu->active_test = MODE_MENU_NO_SELECTION;
            menu->active_group = MODE_MENU_NO_SELECTION;
            menu->active_item = MODE_MENU_NO_SELECTION;
            menu->stream_running = 0U;
            menu->test_focus = MODE_MENU_NO_SELECTION;
            return MODE_MENU_EVENT_TEST_EXIT;
        }

        menu->active_group = menu->test_focus;
        menu->page = MODE_MENU_PAGE_ENCODER;
        menu->encoder_focus = MODE_MENU_NO_SELECTION;
        menu->active_encoder_option = MODE_MENU_NO_SELECTION;
        menu->active_item = MODE_MENU_NO_SELECTION;
        menu->active_test = MODE_MENU_NO_SELECTION;
        menu->stream_running = 0U;
        return MODE_MENU_EVENT_ENCODER_ENTER;
    }

    if (menu->focus == MODE_MENU_NO_SELECTION)
        return MODE_MENU_EVENT_NONE;

    menu->selected = menu->focus;
    if (menu->selected == 6U) {
        menu->page = MODE_MENU_PAGE_TEST;
        menu->test_focus = MODE_MENU_NO_SELECTION;
        menu->active_test = MODE_MENU_NO_SELECTION;
        menu->active_group = MODE_MENU_NO_SELECTION;
        menu->active_item = MODE_MENU_NO_SELECTION;
        menu->encoder_focus = MODE_MENU_NO_SELECTION;
        menu->active_encoder_option = MODE_MENU_NO_SELECTION;
        menu->stream_running = 0U;
        return MODE_MENU_EVENT_TEST_ENTER;
    }
    return MODE_MENU_EVENT_SELECT;
}

uint8_t ModeMenu_PauseTest(ModeMenu *menu)
{
    if (!menu || menu->page != MODE_MENU_PAGE_ENCODER ||
        menu->active_test == MODE_MENU_NO_SELECTION || !menu->stream_running)
        return 0U;
    menu->stream_running = 0U;
    return 1U;
}

uint8_t ModeMenu_ResumeTest(ModeMenu *menu)
{
    if (!menu || menu->page != MODE_MENU_PAGE_ENCODER ||
        menu->active_test == MODE_MENU_NO_SELECTION || menu->stream_running)
        return 0U;
    menu->stream_running = 1U;
    return 1U;
}

uint8_t ModeMenu_CanResetSensor(const ModeMenu *menu, uint8_t sensor_index)
{
    if (!menu || menu->page != MODE_MENU_PAGE_ENCODER ||
        menu->active_test == MODE_MENU_NO_SELECTION)
        return 0U;

    if (sensor_index == 0U)
        return (menu->active_group == 0U && menu->active_item == 0U) ? 1U : 0U;
    if (sensor_index == 1U)
        return (menu->active_group == 1U && menu->active_item == 0U) ? 1U : 0U;
    if (sensor_index == 2U)
        return (menu->active_group == 2U &&
                (menu->active_item == 0U || menu->active_item == 1U)) ? 1U : 0U;
    return 0U;
}

ModeMenuEvent ModeMenu_Poll(ModeMenu *menu, uint8_t button_pressed, uint32_t now_ms)
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
                if ((uint32_t)(now_ms - menu->pressed_at_ms) >= MODE_MENU_LONG_PRESS_MS) {
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
