#ifndef APP_MODE_H
#define APP_MODE_H
#include "mode_menu.h"
typedef enum { APP_BOOT, APP_IDLE, APP_ARMED, APP_RUNNING, APP_STOPPING, APP_FAULT } AppState;
const char *AppMode_StreamProtocolName(uint8_t stream);
#if defined(MICROMOUSE_UNIFIED_RUNTIME)
AppState AppMode_GetState(const ModeMenu *menu);
const char *AppMode_StateName(AppState state);
#endif
#endif
