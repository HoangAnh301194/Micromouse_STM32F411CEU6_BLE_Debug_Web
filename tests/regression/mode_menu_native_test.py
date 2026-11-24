#!/usr/bin/env python3
"""Native regression tests for the Mode 9 System Test menu."""
from __future__ import annotations
import os
from pathlib import Path
import shutil
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[2]
CC = os.environ.get("CC", "gcc")
HARNESS = r"""
#include "mode_menu.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>

int main(void) {
    ModeMenu m;
    unsigned i;

    ModeMenu_Init(&m, 0U, 0U);
    assert(m.focus == MODE_MENU_NO_SELECTION);
    assert(ModeMenu_Next(&m) == MODE_MENU_EVENT_FOCUS);
    for (i = 1U; i < 7U; ++i) assert(ModeMenu_Next(&m) == MODE_MENU_EVENT_FOCUS);
    assert(m.focus == 6U);
    assert(!strcmp(ModeMenu_Label(6U), "9.System Test"));
    assert(!strcmp(ModeMenu_Code(6U), "SYSTEM_TEST"));
    assert(ModeMenu_Select(&m) == MODE_MENU_EVENT_TEST_ENTER);
    assert(m.page == MODE_MENU_PAGE_TEST);

    /* IR group and the existing live IR telemetry path. */
    assert(ModeMenu_Next(&m) == MODE_MENU_EVENT_TEST_FOCUS);
    assert(m.test_focus == 0U);
    assert(!strcmp(ModeMenu_TestName(0U), "IR SENSOR TEST"));
    assert(ModeMenu_Select(&m) == MODE_MENU_EVENT_ENCODER_ENTER);
    assert(m.active_group == 0U && m.page == MODE_MENU_PAGE_ENCODER);
    assert(ModeMenu_ItemCount(0U) == 5U);
    assert(ModeMenu_Next(&m) == MODE_MENU_EVENT_ENCODER_FOCUS);
    assert(!strcmp(ModeMenu_ItemName(0U, 0U), "IR Live Data"));
    assert(ModeMenu_Select(&m) == MODE_MENU_EVENT_ENCODER_SELECT);
    assert(m.active_test == 0U && m.stream_running);
    assert(ModeMenu_CanResetSensor(&m, 0U));
    assert(!ModeMenu_CanResetSensor(&m, 1U));
    assert(ModeMenu_PauseTest(&m) && !m.stream_running);
    assert(ModeMenu_ResumeTest(&m) && m.stream_running);

    /* Browse to IR Filter: it is visible but not falsely reported as active. */
    assert(ModeMenu_Next(&m) == MODE_MENU_EVENT_ENCODER_FOCUS);
    assert(m.encoder_focus == 1U && m.active_test == MODE_MENU_NO_SELECTION);
    assert(ModeMenu_Select(&m) == MODE_MENU_EVENT_ENCODER_SELECT);
    assert(!m.stream_running && m.active_test == MODE_MENU_NO_SELECTION);

    /* Back from the IR submenu. */
    for (i = 2U; i < ModeMenu_ItemCount(0U) - 1U; ++i)
        assert(ModeMenu_Next(&m) == MODE_MENU_EVENT_ENCODER_FOCUS);
    assert(ModeMenu_Next(&m) == MODE_MENU_EVENT_ENCODER_FOCUS);
    assert(m.encoder_focus == ModeMenu_ItemCount(0U) - 1U);
    assert(ModeMenu_Select(&m) == MODE_MENU_EVENT_ENCODER_EXIT);
    assert(m.page == MODE_MENU_PAGE_TEST);

    /* Encoder submenu keeps Log Receive passive until explicit Start. */
    assert(ModeMenu_Next(&m) == MODE_MENU_EVENT_TEST_FOCUS);
    assert(m.test_focus == 1U);
    assert(ModeMenu_Select(&m) == MODE_MENU_EVENT_ENCODER_ENTER);
    assert(ModeMenu_Next(&m) == MODE_MENU_EVENT_ENCODER_FOCUS);
    assert(!strcmp(ModeMenu_ItemName(1U, 0U), "Log Receive"));
    assert(ModeMenu_Select(&m) == MODE_MENU_EVENT_ENCODER_SELECT);
    assert(m.active_test == 1U && !m.stream_running);
    assert(ModeMenu_CanResetSensor(&m, 1U));
    assert(ModeMenu_ResumeTest(&m) && m.stream_running);
    assert(ModeMenu_PauseTest(&m) && !m.stream_running);

    /* 1 Box Forward is a selectable entry but does not run in safe menu. */
    assert(ModeMenu_Next(&m) == MODE_MENU_EVENT_ENCODER_FOCUS);
    assert(m.encoder_focus == 1U);
    assert(ModeMenu_Select(&m) == MODE_MENU_EVENT_ENCODER_SELECT);
    assert(!strcmp(ModeMenu_ItemName(1U, 1U), "1 Box Forward"));
    assert(m.active_test == MODE_MENU_NO_SELECTION && !m.stream_running);

    /* Encoder list includes all proposed items and Back. */
    assert(ModeMenu_ItemCount(1U) == 9U);
    for (i = 2U; i < 9U; ++i) assert(ModeMenu_Next(&m) == MODE_MENU_EVENT_ENCODER_FOCUS);
    assert(m.encoder_focus == 8U);
    assert(ModeMenu_Select(&m) == MODE_MENU_EVENT_ENCODER_EXIT);

    /* MPU Gyro Data reuses the existing passive MPU telemetry. */
    assert(ModeMenu_Next(&m) == MODE_MENU_EVENT_TEST_FOCUS);
    assert(m.test_focus == 2U);
    assert(ModeMenu_Select(&m) == MODE_MENU_EVENT_ENCODER_ENTER);
    assert(ModeMenu_Next(&m) == MODE_MENU_EVENT_ENCODER_FOCUS);
    assert(ModeMenu_Select(&m) == MODE_MENU_EVENT_ENCODER_SELECT);
    assert(m.active_test == 2U && m.stream_running);
    assert(ModeMenu_CanResetSensor(&m, 2U));

    /* Every remaining group has the agreed structure and a Back entry. */
    assert(ModeMenu_ItemCount(2U) == 5U);
    assert(ModeMenu_ItemCount(3U) == 6U);
    assert(ModeMenu_ItemCount(4U) == 4U);
    assert(ModeMenu_ItemCount(5U) == 5U);
    assert(ModeMenu_ItemCount(6U) == 6U);
    assert(!strcmp(ModeMenu_ItemName(2U, 0U), "Gyro Data"));
    assert(!strcmp(ModeMenu_ItemName(3U, 0U), "Left Motor"));
    assert(!strcmp(ModeMenu_ItemName(4U, 0U), "Speed PID"));
    assert(!strcmp(ModeMenu_ItemName(5U, 0U), "SPP Connection"));
    assert(!strcmp(ModeMenu_ItemName(6U, 4U), "Flash Map Test"));
    assert(!strcmp(ModeMenu_TestName(7U), "BACK"));
    assert(!strcmp(ModeMenu_StreamName(0U), "IR Live Data"));
    assert(!strcmp(ModeMenu_StreamName(1U), "Encoder Log Receive"));
    assert(!strcmp(ModeMenu_StreamName(2U), "MPU Gyro Data"));
    assert(!strcmp(ModeMenu_ItemName(7U, 0U), "INVALID"));

    puts("Mode 9 menu native tests: PASS");
    return 0;
}
"""

def main() -> None:
    if not shutil.which(CC):
        raise SystemExit(f"Compiler {CC!r} not available")
    with tempfile.TemporaryDirectory(prefix="mode9-menu-") as tmp:
        source = Path(tmp) / "test_mode9.c"
        binary = Path(tmp) / ("test_mode9.exe" if os.name == "nt" else "test_mode9")
        source.write_text(HARNESS, encoding="utf-8")
        subprocess.run(
            [CC, "-std=c99", "-Wall", "-Wextra", "-Werror",
             "-I", str(ROOT / "include"),
             str(ROOT / "src" / "mode_menu.c"), str(source), "-o", str(binary)],
            check=True,
        )
        subprocess.run([str(binary)], check=True)

if __name__ == "__main__":
    main()
