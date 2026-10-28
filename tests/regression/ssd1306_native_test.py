#!/usr/bin/env python3
"""Host-only regression tests for SSD1306 framebuffer numeric rendering."""
from __future__ import annotations

import os
from pathlib import Path
import shutil
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[2]

STM32_STUB = r"""
#ifndef STM32F4XX_H
#define STM32F4XX_H
#include <stdint.h>
typedef struct {
    volatile uint32_t SR2;
} I2C_TypeDef;
extern I2C_TypeDef test_i2c1;
#define I2C1 (&test_i2c1)
#define I2C_SR2_BUSY (1UL << 1)
#endif
"""

TIMER_STUB = r"""
#ifndef SYSTEM_TIMER_H
#define SYSTEM_TIMER_H
#include <stdint.h>
uint32_t millis(void);
void delay_us_blocking(uint32_t us);
void delay_ms_blocking(uint32_t ms);
#endif
"""

HARNESS = r"""
#include "ssd1306.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

I2C_TypeDef test_i2c1;
static uint16_t test_font_data[95];
static FontDef_t test_font = {8U, 1U, test_font_data};

uint32_t millis(void) { return 0U; }
void delay_us_blocking(uint32_t us) { (void)us; }
void delay_ms_blocking(uint32_t ms) { (void)ms; }
I2C_Status I2C_Write(I2C_Instance instance, uint8_t address,
                     uint8_t *data, uint16_t length)
{
    (void)instance; (void)address; (void)data; (void)length;
    return I2C_OK;
}
I2C_Status I2C_IsDeviceReady(I2C_Instance instance, uint8_t address)
{
    (void)instance; (void)address;
    return I2C_OK;
}
void I2C_DMA_Init(I2C_Instance instance) { (void)instance; }
I2C_Status I2C_Write_DMA(I2C_Instance instance, uint8_t address,
                         uint8_t *data, uint16_t length)
{
    (void)instance; (void)address; (void)data; (void)length;
    return I2C_OK;
}
uint8_t I2C_IsBusy_DMA(I2C_Instance instance)
{
    (void)instance;
    return 0U;
}

static void reset_canvas(void)
{
    SSD1306_Fill(SSD1306_BLACK);
    SSD1306_SetCursor(0U, 0U);
}

static void assert_text(const char *expected)
{
    const uint8_t *framebuffer = SSD1306_GetFramebuffer();
    size_t i;
    for (i = 0U; expected[i] != '\0'; ++i) {
        uint8_t decoded = 0U;
        uint8_t x;
        for (x = 0U; x < 8U; ++x) {
            decoded = (uint8_t)((decoded << 1) |
                      (framebuffer[i * 8U + x] & 1U));
        }
        assert(decoded == (uint8_t)expected[i]);
    }
    assert(SSD1306_GetCursorX() == i * 8U);
}

int main(void)
{
    uint16_t ch;
    for (ch = 32U; ch <= 126U; ++ch) {
        test_font_data[ch - 32U] = (uint16_t)(ch << 8);
    }

    reset_canvas();
    SSD1306_WriteInt(INT32_MIN, &test_font, SSD1306_WHITE);
    assert_text("-2147483648");

    reset_canvas();
    SSD1306_WriteInt(INT32_MAX, &test_font, SSD1306_WHITE);
    assert_text("2147483647");

    reset_canvas();
    SSD1306_WriteFloat(-12.5f, 2U, &test_font, SSD1306_WHITE);
    assert_text("-12.50");

    reset_canvas();
    SSD1306_WriteFloat(1.25f, 8U, &test_font, SSD1306_WHITE);
    assert_text("ERR");

    reset_canvas();
    SSD1306_WriteFloat(4294967296.0f, 1U, &test_font, SSD1306_WHITE);
    assert_text("ERR");

    reset_canvas();
    SSD1306_WriteFloat(NAN, 1U, &test_font, SSD1306_WHITE);
    assert_text("ERR");

    puts("SSD1306_NATIVE_PASS (framebuffer formatting; no hardware)");
    return 0;
}
"""


def main() -> int:
    compiler = shutil.which(os.environ.get("CC", "gcc"))
    if compiler is None:
        raise SystemExit("A native C compiler is required (CC or gcc).")

    with tempfile.TemporaryDirectory(prefix="ssd1306-native-") as folder:
        temp = Path(folder)
        (temp / "stm32f4xx.h").write_text(STM32_STUB, encoding="utf-8")
        (temp / "system_timer.h").write_text(TIMER_STUB, encoding="utf-8")
        harness = temp / "test_ssd1306.c"
        harness.write_text(HARNESS, encoding="utf-8")
        executable = temp / ("ssd1306_test.exe" if os.name == "nt" else "ssd1306_test")
        subprocess.run(
            [
                compiler,
                "-std=c11",
                "-O2",
                "-Wall",
                "-Wextra",
                "-Werror",
                "-DOLED_SHARED_I2C=1",
                "-I",
                str(temp),
                "-I",
                str(ROOT / "include"),
                str(ROOT / "src" / "ssd1306.c"),
                str(harness),
                "-lm",
                "-o",
                str(executable),
            ],
            cwd=ROOT,
            check=True,
            timeout=30,
        )
        subprocess.run([str(executable)], cwd=ROOT, check=True, timeout=30)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
