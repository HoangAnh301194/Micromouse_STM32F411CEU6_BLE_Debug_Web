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

static unsigned short_press(ModeMenu *m, unsigned t) {
    assert(at(m, t, 1) == MODE_MENU_EVENT_NONE);
    assert(at(m, t+31, 1) == MODE_MENU_EVENT_NONE);
    assert(at(m, t+70, 0) == MODE_MENU_EVENT_NONE);
    assert(at(m, t+101, 0) == MODE_MENU_EVENT_FOCUS);
    return t+101;
}

int main(void) {
    ModeMenu m;
    unsigned i;
    ModeMenu_Init(&m, 0, 0);
    assert(m.focus == 0 && m.selected == MODE_MENU_NO_SELECTION);
    assert(!strcmp(ModeMenu_Label(0), "1.Calibration"));
    assert(!strcmp(ModeMenu_Code(6), "SYSTEM_TEST"));
    assert(!strcmp(ModeMenu_Label(7), "INVALID"));

    /* Debounced short click advances focus but does not select. */
    short_press(&m, 100);
    assert(m.focus == 1 && m.selected == MODE_MENU_NO_SELECTION);

    /* A 400-ms hold selects exactly once, does not call any motor API. */
    assert(at(&m, 300, 1) == MODE_MENU_EVENT_NONE);
    assert(at(&m, 331, 1) == MODE_MENU_EVENT_NONE);
    assert(at(&m, 731, 1) == MODE_MENU_EVENT_SELECT);
    assert(m.selected == 1 && m.focus == 1);
    assert(at(&m, 1000, 1) == MODE_MENU_EVENT_NONE);
    assert(at(&m, 1100, 0) == MODE_MENU_EVENT_NONE);
    assert(at(&m, 1131, 0) == MODE_MENU_EVENT_NONE);
    assert(m.focus == 1 && m.selected == 1);

    /* Seven short clicks wrap back to the same menu item. */
    for (i = 0; i < MODE_MENU_ITEM_COUNT; ++i) {
        short_press(&m, 1400 + 170*i);
    }
    assert(m.focus == 1 && m.selected == 1);

    /* Bouncing shorter than 30ms does not create an event. */
    ModeMenu_Init(&m, 0, 0);
    assert(at(&m, 10, 1) == MODE_MENU_EVENT_NONE);
    assert(at(&m, 20, 0) == MODE_MENU_EVENT_NONE);
    assert(at(&m, 60, 0) == MODE_MENU_EVENT_NONE);
    assert(m.focus == 0 && m.selected == MODE_MENU_NO_SELECTION);

    /* Button held at boot is ignored until stable release. */
    ModeMenu_Init(&m, 1, 0);
    assert(at(&m, 1000, 1) == MODE_MENU_EVENT_NONE);
    assert(at(&m, 1100, 0) == MODE_MENU_EVENT_NONE);
    assert(at(&m, 1131, 0) == MODE_MENU_EVENT_NONE);
    assert(m.selected == MODE_MENU_NO_SELECTION);
    short_press(&m, 1200);
    assert(m.focus == 1);

    /* millis() rollover still permits selection. */
    ModeMenu_Init(&m, 0, 0xFFFFFF00u);
    assert(at(&m, 0xFFFFFF10u, 1) == MODE_MENU_EVENT_NONE);
    assert(at(&m, 0xFFFFFF35u, 1) == MODE_MENU_EVENT_NONE);
    assert(at(&m, 0x000000C5u, 1) == MODE_MENU_EVENT_NONE);
    assert(at(&m, 0x000000E0u, 1) == MODE_MENU_EVENT_SELECT);
    assert(m.selected == 0);

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
