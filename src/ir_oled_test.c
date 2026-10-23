/*
 * Isolated six-channel IR bench viewer for SSD1306 128x64.
 * Standalone main entry under -DIR_OLED_TEST=1 (does not start the
 * MPU/I2C policy-B scheduler or the normal Motion/Encoder control).
 * OLED has exclusive I2C1 ownership in this firmware profile.
 *
 * The displayed value is LED-on ADC minus ambient ADC, clamped to zero.
 * It is NOT a calibrated distance in millimetres.
 */
#include "board.h"
#include "ir_sensor.h"
#include "ssd1306.h"
#include "i2c.h"
#include "motor.h"
#include "system_timer.h"
#include "uart.h"
#include <stdint.h>
#include <stdio.h>

#if defined(IR_OLED_TEST)

#define IR_SCAN_PERIOD_MS    10U
#define OLED_FRAME_PERIOD_MS 200U
#define IR_STALE_PERIOD_MS   500U

static void IR_OLED_DrawRow(uint16_t y, const char *prefix,
                            uint16_t left, uint16_t right)
{
    char line[24];
    (void)snprintf(line, sizeof(line), "%s L:%4u R:%4u",
                   prefix, (unsigned)left, (unsigned)right);
    SSD1306_SetCursor(0U, y);
    (void)SSD1306_WriteString(line, &Font_7x10, SSD1306_WHITE);
}

static void IR_OLED_Draw(const IR_Data_t *sample, uint8_t have_sample,
                         uint32_t sample_age_ms)
{
    char line[24];
    SSD1306_Fill(SSD1306_BLACK);
    SSD1306_SetCursor(0U, 0U);
    (void)SSD1306_WriteString("IR 6CH / RAW ADC", &Font_7x10,
                              SSD1306_WHITE);
    if (!have_sample || sample_age_ms > IR_STALE_PERIOD_MS) {
        SSD1306_SetCursor(0U, 24U);
        (void)SSD1306_WriteString("WAIT IR SCAN", &Font_7x10,
                                  SSD1306_WHITE);
    } else {
        IR_OLED_DrawRow(13U, "90", sample->value[IR_L90],
                        sample->value[IR_R90]);
        IR_OLED_DrawRow(25U, "45", sample->value[IR_L45],
                        sample->value[IR_R45]);
        IR_OLED_DrawRow(37U, "00", sample->value[IR_L0],
                        sample->value[IR_R0]);
    }
    (void)snprintf(line, sizeof(line), "N:%lu  age:%lums",
                   (unsigned long)(have_sample ? sample->scan_count : 0U),
                   (unsigned long)(have_sample ? sample_age_ms : 0U));
    SSD1306_SetCursor(0U, 52U);
    (void)SSD1306_WriteString(line, &Font_7x10, SSD1306_WHITE);
    SSD1306_UpdateScreen(); /* Isolated I2C owner; blocking, 5Hz maximum. */
}

int IR_OLED_TestMain(void)
{
    IR_Data_t sample = {{0}, 0U, 0U};
    uint32_t last_scan_ms = 0U, last_screen_ms = 0U;
    uint32_t last_uart_ms = 0U;
    uint8_t have_sample = 0U;
    uint8_t oled_ready = 0U;

    Board_Init();
    SystemTimer_Init();
    UART_Init(); /* Optional PA9/PA10 record even if OLED is not detected. */

    if (Board_GetClockFault() != 0U) {
        UART_Printf("IR_OLED,CLOCK_FAULT,%lu\r\n",
                    (unsigned long)Board_GetClockFault());
        Board_StatusLed(1U);
        while (1) { /* Fail closed; never start the IR/OLED peripherals. */ }
    }

    /* Motor OFF is latched before initializing any sensors. There is no
       caller of Motor_SetPair in this diagnostic main. */
    Motor_Init();
    Motor_LockOutputs();

    /* MPU6050_Init() must NOT run here: it owns I2C1 in production.
       The OLED is the only I2C1 client in this isolated test image. */
    I2C_Init(I2C_1, 1U);
    IR_Init();
    oled_ready = (SSD1306_Init() == SSD1306_OK) ? 1U : 0U;

    UART_Printf("IR_OLED,BOOT,motor_locked=%u,oled=%u\r\n",
                (unsigned)Motor_OutputsLocked(), (unsigned)oled_ready);
    IR_StartScan();
    last_scan_ms = millis();
    last_screen_ms = millis() - OLED_FRAME_PERIOD_MS;

    while (1) {
        uint32_t now = millis();
        IR_Data_t next;
        if (IR_GetLatest(&next)) {
            sample = next;
            have_sample = 1U;
        }

        /* Rate-limit new acquisition. A failed scan leaves IR idle, so
           the next tick automatically retries without blocking main. */
        if ((uint32_t)(now - last_scan_ms) >= IR_SCAN_PERIOD_MS) {
            last_scan_ms = now;
            if (!IR_IsReady()) IR_StartScan();
        }

        if ((uint32_t)(now - last_screen_ms) >= OLED_FRAME_PERIOD_MS) {
            uint32_t age = have_sample
                ? (uint32_t)(micros() - sample.completed_us) / 1000U
                : 0U;
            last_screen_ms = now;
            if (oled_ready) IR_OLED_Draw(&sample, have_sample, age);
        }

        if ((uint32_t)(now - last_uart_ms) >= 200U) {
            last_uart_ms = now;
            UART_Printf("IR6,ms=%lu,n=%lu,l90=%u,l45=%u,l0=%u,"
                        "r0=%u,r45=%u,r90=%u\r\n",
                        (unsigned long)now,
                        (unsigned long)sample.scan_count,
                        (unsigned)sample.value[IR_L90],
                        (unsigned)sample.value[IR_L45],
                        (unsigned)sample.value[IR_L0],
                        (unsigned)sample.value[IR_R0],
                        (unsigned)sample.value[IR_R45],
                        (unsigned)sample.value[IR_R90]);
        }
    }
}
#endif /* IR_OLED_TEST */
