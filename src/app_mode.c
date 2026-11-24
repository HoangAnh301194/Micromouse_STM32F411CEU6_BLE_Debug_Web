#include "app_mode.h"
const char *AppMode_StreamProtocolName(uint8_t stream)
{
    switch (stream) {
        case 0U: return "IR Test";
        case 1U: return "Encoder Test";
        case 2U: return "MPU Test";
        default: return "Unknown";
    }
}
#if defined(MICROMOUSE_UNIFIED_RUNTIME)
#include "encoder_onebox.h"
#include "motor.h"
AppState AppMode_GetState(const ModeMenu *menu)
{
    /* Motion state is authoritative, including braking and latched faults.
     * Passive streams never acquire motor ownership. */
    switch (EncoderOneBox_State()) {
        case ONEBOX_STATE_FAULT: return APP_FAULT;
        case ONEBOX_STATE_RUNNING: return APP_RUNNING;
        case ONEBOX_STATE_STOPPING: return APP_STOPPING;
        default: break;
    }
    if (!Motor_OutputsLocked()) return APP_FAULT;
    if (!menu || menu->page != MODE_MENU_PAGE_ENCODER) return APP_IDLE;
    if (menu->active_group == 1U &&
        menu->active_encoder_option == MODE_MENU_ENCODER_ONE_BOX_FORWARD)
        return APP_ARMED;
    if (menu->active_test != MODE_MENU_NO_SELECTION)
        return menu->stream_running ? APP_RUNNING : APP_ARMED;
    return APP_IDLE;
}
const char *AppMode_StateName(AppState state)
{
    switch (state) {
        case APP_BOOT: return "BOOT";
        case APP_IDLE: return "IDLE";
        case APP_ARMED: return "ARMED";
        case APP_RUNNING: return "RUNNING";
        case APP_STOPPING: return "STOPPING";
        case APP_FAULT: return "FAULT";
        default: return "UNKNOWN";
    }
}
#endif
