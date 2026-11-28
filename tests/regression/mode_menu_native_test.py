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
#include "mode_test_resources.h"
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
    assert(ModeTest_Resources(&m) == 0U);
    assert(!ModeTest_OneBoxArmed(&m));
    assert(!ModeMenu_PauseTest(&m));
    assert(!ModeMenu_ResumeTest(&m));
    assert(!ModeMenu_PauseTest(NULL));
    assert(!ModeMenu_ResumeTest(NULL));
    assert(!ModeMenu_CanResetSensor(NULL, 0));
    assert(!ModeMenu_CanResetSensor(&m, 0));
    assert(!ModeMenu_CanResetSensor(&m, 1));
    assert(!ModeMenu_CanResetSensor(&m, 2));
    assert(!ModeMenu_CanResetSensor(&m, MODE_MENU_TEST_BACK));
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
    assert(ModeTest_Resources(&m) == TEST_RES_IR);
    assert(ModeMenu_CanResetSensor(&m, 0));
    assert(!ModeMenu_CanResetSensor(&m, 1));
    assert(!ModeMenu_CanResetSensor(&m, 2));
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
    /* Encoder Test is a submenu, NOT an immediately active stream. */
    assert(ModeMenu_Select(&m) == MODE_MENU_EVENT_ENCODER_ENTER);
    assert(m.page == MODE_MENU_PAGE_ENCODER);
    assert(m.active_test == MODE_MENU_NO_SELECTION && !m.stream_running);
    assert(!ModeMenu_CanResetSensor(&m, 1));
    assert(!ModeMenu_ResumeTest(&m));
    assert(ModeMenu_Select(&m) == MODE_MENU_EVENT_NONE);
    assert(ModeMenu_Next(&m) == MODE_MENU_EVENT_ENCODER_FOCUS);
    assert(m.encoder_focus == MODE_MENU_ENCODER_LOG_RECEIVE);
    assert(!strcmp(ModeMenu_EncoderName(0), "Log Receive"));
    assert(ModeMenu_Select(&m) == MODE_MENU_EVENT_ENCODER_SELECT);
    assert(m.active_encoder_option == MODE_MENU_ENCODER_LOG_RECEIVE);
    /* Selection is IDLE: absolutely no Encoder telemetry before START. */
    assert(m.active_test == 1 && !m.stream_running);
    assert(ModeTest_Resources(&m) == TEST_RES_ENCODER);
    assert(ModeMenu_CanResetSensor(&m, 1));
    assert(!ModeMenu_CanResetSensor(&m, 0));
    assert(!ModeMenu_CanResetSensor(&m, 2));
    assert(!ModeMenu_PauseTest(&m));  /* Stop before Start is a no-op. */
    assert(ModeMenu_ResumeTest(&m));  /* Explicit Start. */
    assert(m.stream_running);
    assert(!ModeMenu_ResumeTest(&m));
    assert(ModeMenu_PauseTest(&m));   /* Explicit Stop. */
    assert(!m.stream_running);
    assert(!ModeMenu_PauseTest(&m));
    assert(ModeMenu_ResumeTest(&m));  /* New Start, new CSV on desktop. */
    assert(m.stream_running);
    assert(ModeMenu_PauseTest(&m));   /* New Stop; close second CSV. */
    assert(!m.stream_running);

    /* All six motion-labelled options are inert placeholders: no stream. */
    for (i = 1; i < MODE_MENU_ENCODER_BACK; ++i) {
        assert(ModeMenu_Next(&m) == MODE_MENU_EVENT_ENCODER_FOCUS);
        assert(m.encoder_focus == i);
        assert(!m.stream_running);
        assert(m.active_test == MODE_MENU_NO_SELECTION);
        assert(ModeMenu_Select(&m) == MODE_MENU_EVENT_ENCODER_SELECT);
        assert(m.active_encoder_option == i);
        if (i == MODE_MENU_ENCODER_1BOX || i == MODE_MENU_ENCODER_MULTI) {
            assert(ModeTest_OneBoxArmed(&m));
            assert(ModeTest_Resources(&m) == (TEST_RES_ENCODER | TEST_RES_MOTOR));
        } else {
            assert(!ModeTest_OneBoxArmed(&m));
            assert(ModeTest_Resources(&m) == 0U);
        }
        assert(m.active_test == MODE_MENU_NO_SELECTION);
        assert(!m.stream_running && !ModeMenu_ResumeTest(&m));
        assert(!ModeMenu_CanResetSensor(&m, 1));
    }
    assert(!strcmp(ModeMenu_EncoderName(1), "1 Box Forward"));
    assert(!strcmp(ModeMenu_EncoderName(2), "Multi Box Forward"));
    assert(!strcmp(ModeMenu_EncoderName(3), "Pivot 90 Left"));
    assert(!strcmp(ModeMenu_EncoderName(4), "Pivot 90 Right"));
    assert(!strcmp(ModeMenu_EncoderName(5), "Pivot 180 Left"));
    assert(!strcmp(ModeMenu_EncoderName(6), "Pivot 180 Right"));
    assert(!strcmp(ModeMenu_EncoderName(MODE_MENU_ENCODER_PID), "PID"));
    assert(!strcmp(ModeMenu_EncoderName(9), "INVALID"));
    assert(ModeMenu_Next(&m) == MODE_MENU_EVENT_ENCODER_FOCUS);
    assert(m.encoder_focus == MODE_MENU_ENCODER_BACK);
    assert(ModeMenu_Select(&m) == MODE_MENU_EVENT_ENCODER_EXIT);
    assert(m.page == MODE_MENU_PAGE_TEST);
    assert(m.active_test == MODE_MENU_NO_SELECTION && !m.stream_running);
    assert(ModeMenu_Next(&m) == MODE_MENU_EVENT_TEST_FOCUS);
    assert(m.test_focus == 2 && m.active_test == MODE_MENU_NO_SELECTION);
    assert(ModeMenu_Select(&m) == MODE_MENU_EVENT_MPU_ENTER);
    assert(ModeMenu_Next(&m) == MODE_MENU_EVENT_MPU_FOCUS);
    assert(ModeMenu_Select(&m) == MODE_MENU_EVENT_MPU_SELECT);
    assert(m.active_test == 2 && !m.stream_running);
    assert(ModeTest_Resources(&m) == TEST_RES_MPU);
    assert(ModeMenu_ResumeTest(&m) && m.stream_running);
    assert(ModeMenu_PauseTest(&m) && !m.stream_running);
    assert(ModeMenu_CanResetSensor(&m, 2));
    assert(!ModeMenu_CanResetSensor(&m, 1));
    for (int k=1;k<=5;k++) {
        assert(ModeMenu_Next(&m)==MODE_MENU_EVENT_MPU_FOCUS);
        if(k<5) { const int16_t expected[]={0,90,-90,180,-180}; assert(ModeMenu_Select(&m)==MODE_MENU_EVENT_MPU_SELECT); assert(ModeMenu_PivotTarget(&m)==expected[k]); assert(ModeTest_Resources(&m)==(TEST_RES_MPU|TEST_RES_MOTOR)); }
    }
    assert(ModeMenu_Select(&m)==MODE_MENU_EVENT_MPU_EXIT);
    assert(ModeMenu_Next(&m) == MODE_MENU_EVENT_TEST_FOCUS);
    assert(m.test_focus == MODE_MENU_TEST_ALL);
    assert(ModeMenu_Select(&m) == MODE_MENU_EVENT_TEST_SELECT);
    assert(m.active_test == MODE_MENU_TEST_ALL);
    assert(ModeTest_Resources(&m) ==
           (TEST_RES_IR | TEST_RES_MPU | TEST_RES_ENCODER | TEST_RES_MOTOR));
    assert(ModeMenu_Next(&m) == MODE_MENU_EVENT_TEST_FOCUS);
    assert(m.test_focus == MODE_MENU_TEST_BACK && m.active_test == MODE_MENU_NO_SELECTION);
    assert(ModeTest_Resources(&m) == 0U);
    assert(ModeMenu_Select(&m) == MODE_MENU_EVENT_TEST_EXIT);
    assert(m.page == MODE_MENU_PAGE_MAIN && m.selected == 6);
    assert(!m.stream_running && !ModeMenu_ResumeTest(&m));
    assert(!ModeMenu_CanResetSensor(&m, 2));
    assert(ModeMenu_Next(&m) == MODE_MENU_EVENT_FOCUS);
    assert(m.focus == 0);
    assert(!strcmp(ModeMenu_TestName(3), "All Sensors Test"));
    assert(!strcmp(ModeMenu_TestName(5), "INVALID"));

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
             str(ROOT / "src" / "mode_menu.c"),
             str(ROOT / "src" / "mode_test_resources.c"), str(source),
             "-o", str(binary)],
            check=True,
        )
        subprocess.run([str(binary)], check=True)


if __name__ == "__main__":
    main()
