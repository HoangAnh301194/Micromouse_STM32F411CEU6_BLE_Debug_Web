#!/usr/bin/env python3
"""Compile and test the hardware-independent PA0 mode menu state machine."""
from __future__ import annotations

import os
from pathlib import Path
import shutil
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[2]
CC = os.environ.get("CC", "gcc")

HARNESS = r'''
#include "mode_menu.h"
#include <assert.h>
#include <string.h>
#include <stdio.h>

static ModeMenuEvent at(ModeMenu *m, unsigned ms, unsigned down) {
    return ModeMenu_Poll(m, (uint8_t)down, (uint32_t)ms);
}

static void short_press(ModeMenu *m, unsigned t) {
    assert(at(m, t, 1) == MODE_MENU_EVENT_NONE);
    assert(at(m, t + 31, 1) == MODE_MENU_EVENT_NONE);
    assert(at(m, t + 70, 0) == MODE_MENU_EVENT_NONE);
    assert(at(m, t + 101, 0) == MODE_MENU_EVENT_FOCUS);
}

int main(void) {
    ModeMenu m;
    unsigned i;

    ModeMenu_Init(&m, 0, 0);
    assert(m.focus == MODE_MENU_NO_SELECTION);
    assert(m.selected == MODE_MENU_NO_SELECTION);
    assert(m.stream_running == 0);
    assert(!ModeMenu_PauseTest(&m));
    assert(!ModeMenu_ResumeTest(&m));
    assert(!ModeMenu_PauseTest(NULL));
    assert(!ModeMenu_ResumeTest(NULL));
    assert(!strcmp(ModeMenu_Label(0), "1.Calibration"));
    assert(!strcmp(ModeMenu_Code(6), "SYSTEM_TEST"));
    assert(!strcmp(ModeMenu_Name(0), "Calibration"));
    assert(!strcmp(ModeMenu_Name(1), "Run Slow"));
    assert(!strcmp(ModeMenu_Label(7), "INVALID"));

    /* The first click reports Mode 1, not Mode 2. */
    short_press(&m, 100);
    assert(m.focus == 0 && m.selected == MODE_MENU_NO_SELECTION);
    short_press(&m, 300);
    assert(m.focus == 1 && m.selected == MODE_MENU_NO_SELECTION);

    /* Long press produces exactly one selection event. */
    assert(at(&m, 600, 1) == MODE_MENU_EVENT_NONE);
    assert(at(&m, 631, 1) == MODE_MENU_EVENT_NONE);
    assert(at(&m, 1031, 1) == MODE_MENU_EVENT_SELECT);
    assert(m.focus == 1 && m.selected == 1);
    assert(at(&m, 1060, 1) == MODE_MENU_EVENT_NONE);
    assert(at(&m, 1100, 0) == MODE_MENU_EVENT_NONE);
    assert(at(&m, 1131, 0) == MODE_MENU_EVENT_NONE);

    /* Seven more clicks wrap back to Mode 2. */
    for (i = 0; i < MODE_MENU_ITEM_COUNT; ++i)
        short_press(&m, 1400 + 170 * i);
    assert(m.focus == 1 && m.selected == 1);

    /* Press/release noise shorter than 30 ms yields no event. */
    ModeMenu_Init(&m, 0, 0);
    assert(at(&m, 10, 1) == MODE_MENU_EVENT_NONE);
    assert(at(&m, 20, 0) == MODE_MENU_EVENT_NONE);
    assert(at(&m, 60, 0) == MODE_MENU_EVENT_NONE);
    assert(m.focus == MODE_MENU_NO_SELECTION);

    /* Holding from boot does not select a mode. */
    ModeMenu_Init(&m, 1, 0);
    assert(at(&m, 1000, 1) == MODE_MENU_EVENT_NONE);
    assert(at(&m, 1100, 0) == MODE_MENU_EVENT_NONE);
    assert(at(&m, 1131, 0) == MODE_MENU_EVENT_NONE);
    short_press(&m, 1200);
    assert(m.focus == 0);

    /* Holding without first browsing cannot select a mode. */
    ModeMenu_Init(&m, 0, 0);
    assert(at(&m, 100, 1) == MODE_MENU_EVENT_NONE);
    assert(at(&m, 131, 1) == MODE_MENU_EVENT_NONE);
    assert(at(&m, 531, 1) == MODE_MENU_EVENT_NONE);
    assert(at(&m, 600, 0) == MODE_MENU_EVENT_NONE);
    assert(at(&m, 631, 0) == MODE_MENU_EVENT_NONE);
    assert(m.focus == MODE_MENU_NO_SELECTION);
    assert(m.selected == MODE_MENU_NO_SELECTION);

    /* millis() rollover still allows a selection after Mode 1 focus. */
    ModeMenu_Init(&m, 0, 0xFFFFFE00u);
    short_press(&m, 0xFFFFFE80u);
    assert(m.focus == 0);
    assert(at(&m, 0xFFFFFF10u, 1) == MODE_MENU_EVENT_NONE);
    assert(at(&m, 0xFFFFFF35u, 1) == MODE_MENU_EVENT_NONE);
    assert(at(&m, 0x000000B5u, 1) == MODE_MENU_EVENT_NONE);
    assert(at(&m, 0x000000E0u, 1) == MODE_MENU_EVENT_SELECT);
    assert(m.selected == 0);

    /* PC SPP and PA0 use the same menu actions; streaming is passive. */
    ModeMenu_Init(&m, 0, 0);
    for (i = 0; i < 7; ++i)
        assert(ModeMenu_Next(&m) == MODE_MENU_EVENT_FOCUS);
    assert(m.focus == 6);
    assert(ModeMenu_Select(&m) == MODE_MENU_EVENT_TEST_ENTER);
    assert(m.page == MODE_MENU_PAGE_TEST && m.active_test == MODE_MENU_NO_SELECTION);
    assert(!m.stream_running);
    assert(!ModeMenu_ResumeTest(&m)); /* must select a sensor first */
    assert(ModeMenu_Select(&m) == MODE_MENU_EVENT_NONE);
    assert(ModeMenu_Next(&m) == MODE_MENU_EVENT_TEST_FOCUS);
    assert(m.test_focus == 0);
    assert(!strcmp(ModeMenu_TestName(0), "IR Test"));
    assert(ModeMenu_Select(&m) == MODE_MENU_EVENT_TEST_SELECT);
    assert(m.active_test == 0 && m.stream_running);
    assert(ModeMenu_PauseTest(&m));
    assert(!m.stream_running);
    assert(!ModeMenu_PauseTest(&m)); /* stop twice is a no-op */
    assert(ModeMenu_ResumeTest(&m));
    assert(m.stream_running);
    assert(!ModeMenu_ResumeTest(&m)); /* start twice is a no-op */
    assert(ModeMenu_PauseTest(&m));
    assert(ModeMenu_Next(&m) == MODE_MENU_EVENT_TEST_FOCUS);
    assert(m.test_focus == 1 && m.active_test == MODE_MENU_NO_SELECTION);
    assert(!m.stream_running && !ModeMenu_ResumeTest(&m));
    assert(ModeMenu_Select(&m) == MODE_MENU_EVENT_TEST_SELECT);
    assert(m.active_test == 1 && m.stream_running);
    assert(ModeMenu_PauseTest(&m));
    assert(!m.stream_running);
    assert(ModeMenu_ResumeTest(&m));
    assert(ModeMenu_Next(&m) == MODE_MENU_EVENT_TEST_FOCUS);
    assert(m.test_focus == 2 && m.active_test == MODE_MENU_NO_SELECTION);
    assert(ModeMenu_Select(&m) == MODE_MENU_EVENT_TEST_SELECT);
    assert(m.active_test == 2);
    assert(ModeMenu_Next(&m) == MODE_MENU_EVENT_TEST_FOCUS);
    assert(m.test_focus == MODE_MENU_TEST_BACK && m.active_test == MODE_MENU_NO_SELECTION);
    assert(ModeMenu_Select(&m) == MODE_MENU_EVENT_TEST_EXIT);
    assert(m.page == MODE_MENU_PAGE_MAIN && m.selected == 6);
    assert(!m.stream_running && !ModeMenu_ResumeTest(&m));
    assert(ModeMenu_Next(&m) == MODE_MENU_EVENT_FOCUS);
    assert(m.focus == 0);
    assert(!strcmp(ModeMenu_TestName(4), "INVALID"));

    puts("Mode menu native tests: PASS");
    return 0;
}
'''


def main() -> None:
    if not shutil.which(CC):
        raise SystemExit(f"Compiler {CC!r} not available (install GCC or set CC)")
    with tempfile.TemporaryDirectory(prefix="mode-menu-") as tmp:
        source = Path(tmp) / "test_menu.c"
        binary = Path(tmp) / ("test_menu.exe" if os.name == "nt" else "test_menu")
        source.write_text(HARNESS, encoding="utf-8")
        subprocess.run(
            [CC, "-std=c99", "-Wall", "-Wextra", "-Werror",
             "-I", str(ROOT / "include"),
             str(ROOT / "src" / "mode_menu.c"), str(source),
             "-o", str(binary)],
            check=True,
        )
        subprocess.run([str(binary)], check=True)


if __name__ == "__main__":
    main()
