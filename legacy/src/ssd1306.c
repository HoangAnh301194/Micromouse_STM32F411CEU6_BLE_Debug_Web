/**
 * @file ssd1306.c
 * @brief SSD1306 OLED Driver with DMA Support
 * @author HoangAnh
 * @date 2025
 * 
 * FEATURES:
 * - Blocking mode for initialization and simple use
 * - DMA mode for non-blocking screen updates (massive CPU saving)
 * - Fixed font rendering for 16-bit font data
 * - Complete graphics primitives
 * - C89 compliant with English comments
 */

#include "ssd1306.h"
#include "system_timer.h"

/* ========================================================================== */
/* SSD1306 COMMANDS                                                           */
/* ========================================================================== */

#define SSD1306_CMD_DISPLAY_OFF         0xAE
#define SSD1306_CMD_DISPLAY_ON          0xAF
#define SSD1306_CMD_SET_CONTRAST        0x81
#define SSD1306_CMD_DISPLAY_NORMAL      0xA6
#define SSD1306_CMD_DISPLAY_INVERSE     0xA7
#define SSD1306_CMD_SET_MUX_RATIO       0xA8
#define SSD1306_CMD_SET_DISPLAY_OFFSET  0xD3
#define SSD1306_CMD_SET_START_LINE      0x40
#define SSD1306_CMD_SET_SEG_REMAP       0xA1
#define SSD1306_CMD_SET_COM_SCAN_DEC    0xC8
#define SSD1306_CMD_SET_COM_PINS        0xDA
#define SSD1306_CMD_SET_CLOCK_DIV       0xD5
#define SSD1306_CMD_SET_PRECHARGE       0xD9
#define SSD1306_CMD_SET_VCOMH           0xDB
#define SSD1306_CMD_CHARGE_PUMP         0x8D
#define SSD1306_CMD_MEMORY_MODE         0x20
#define SSD1306_CMD_SET_PAGE_ADDR       0xB0
#define SSD1306_CMD_SET_LOW_COLUMN      0x00
#define SSD1306_CMD_SET_HIGH_COLUMN     0x10

#define SSD1306_CMD_SCROLL_RIGHT        0x26
#define SSD1306_CMD_SCROLL_LEFT         0x27
#define SSD1306_CMD_SCROLL_DIAG_RIGHT   0x29
#define SSD1306_CMD_SCROLL_DIAG_LEFT    0x2A
#define SSD1306_CMD_SCROLL_STOP         0x2E
#define SSD1306_CMD_SCROLL_START        0x2F
#define SSD1306_CMD_SET_SCROLL_AREA     0xA3

/* ========================================================================== */
/* PRIVATE VARIABLES                                                          */
/* ========================================================================== */

static uint8_t ssd1306_buffer[SSD1306_BUFFER_SIZE];
static uint16_t ssd1306_cursor_x = 0;
static uint16_t ssd1306_cursor_y = 0;
static uint8_t ssd1306_inverted = 0;

/* DMA state variables */
static volatile uint8_t ssd1306_dma_enabled = 0;
static volatile uint8_t ssd1306_dma_page = 0;
static volatile uint8_t ssd1306_dma_busy = 0;
static uint32_t ssd1306_dma_start_time = 0;    /* Timestamp when DMA update started */
static uint8_t ssd1306_dma_error_count = 0;    /* Consecutive DMA errors */
static uint8_t ssd1306_dma_chunk_buffer[129];  /* 1 control byte + 128 data */

#define SSD1306_DMA_TIMEOUT_MS   200   /* Max time for full screen DMA update */
#define SSD1306_DMA_MAX_ERRORS   5     /* Force re-init after this many errors */
#define SSD1306_REFRESH_PERIOD   5000  /* Re-send Display ON every 5s as safety net */

#define ABS(x) ((x) > 0 ? (x) : -(x))

/* ========================================================================== */
/* PRIVATE HELPER FUNCTIONS                                                   */
/* ========================================================================== */

/**
 * @brief Write command to SSD1306
 */
static void SSD1306_WriteCommand(uint8_t cmd) {
    uint8_t data[2];
    data[0] = 0x00;  /* Control byte: Co=0, D/C=0 (command) */
    data[1] = cmd;
    I2C_Write(SSD1306_I2C_INSTANCE, SSD1306_I2C_ADDR, data, 2);
}

/**
 * @brief Write data to SSD1306 (blocking mode)
 */
static void SSD1306_WriteData(uint8_t *data, uint16_t len) {
    uint8_t buffer[33];  /* 1 control byte + 32 data bytes */
    uint16_t i, chunk;
    
    while (len > 0) {
        /* Send 32 bytes per transaction */
        chunk = (len > 32) ? 32 : len;
        buffer[0] = 0x40;  /* Control byte: Co=0, D/C=1 (data) */
        
        for (i = 0; i < chunk; i++) {
            buffer[i + 1] = *data++;
        }
        
        /* Send and check error */
        if (I2C_Write(SSD1306_I2C_INSTANCE, SSD1306_I2C_ADDR, buffer, chunk + 1) != I2C_OK) {
            /* If error, short delay and continue */
            delay_us_blocking(50);
        }
        
        len -= chunk;
    }
}

/**
 * @brief DMA completion callback
 */
static void SSD1306_DMA_Callback(void) {
    /* Move to next page */
    ssd1306_dma_page++;
    
    /* If all pages sent, mark as complete */
    if (ssd1306_dma_page >= (SSD1306_HEIGHT / 8)) {
        ssd1306_dma_busy = 0;
        ssd1306_dma_page = 0;
    }
}

/* ========================================================================== */
/* CORE INITIALIZATION FUNCTIONS                                              */
/* ========================================================================== */

/**
 * @brief Initialize SSD1306 OLED (blocking mode)
 */
SSD1306_Status_t SSD1306_Init(void) {
    /* Check if device is connected */
    if (I2C_IsDeviceReady(SSD1306_I2C_INSTANCE, SSD1306_I2C_ADDR) != I2C_OK) {
        return SSD1306_NOT_FOUND;
    }
    
    /* Wait for power stabilization */
    delay_ms_blocking(100);
    
    /* Initialization sequence */
    SSD1306_WriteCommand(SSD1306_CMD_DISPLAY_OFF);
    
    /* Set MUX Ratio */
    SSD1306_WriteCommand(SSD1306_CMD_SET_MUX_RATIO);
    SSD1306_WriteCommand(SSD1306_HEIGHT - 1);
    
    /* Set Display Offset */
    SSD1306_WriteCommand(SSD1306_CMD_SET_DISPLAY_OFFSET);
    SSD1306_WriteCommand(0x00);
    
    /* Set Display Start Line */
    SSD1306_WriteCommand(SSD1306_CMD_SET_START_LINE | 0x00);
    
    /* Set Segment Re-map (A1 = column 127 mapped to SEG0) */
    SSD1306_WriteCommand(SSD1306_CMD_SET_SEG_REMAP);
    
    /* Set COM Output Scan Direction (C8 = remapped mode) */
    SSD1306_WriteCommand(SSD1306_CMD_SET_COM_SCAN_DEC);
    
    /* Set COM Pins Hardware Configuration */
    SSD1306_WriteCommand(SSD1306_CMD_SET_COM_PINS);
#if (SSD1306_HEIGHT == 64)
    SSD1306_WriteCommand(0x12);
#elif (SSD1306_HEIGHT == 32)
    SSD1306_WriteCommand(0x02);
#endif
    
    /* Set Contrast Control */
    SSD1306_WriteCommand(SSD1306_CMD_SET_CONTRAST);
    SSD1306_WriteCommand(0xCF);
    
    /* Disable Entire Display On (A4 = output follows RAM) */
    SSD1306_WriteCommand(0xA4);
    
    /* Set Normal Display */
    SSD1306_WriteCommand(SSD1306_CMD_DISPLAY_NORMAL);
    
    /* Set Display Clock Divide Ratio/Oscillator Frequency */
    SSD1306_WriteCommand(SSD1306_CMD_SET_CLOCK_DIV);
    SSD1306_WriteCommand(0x80);
    
    /* Set Pre-charge Period */
    SSD1306_WriteCommand(SSD1306_CMD_SET_PRECHARGE);
    SSD1306_WriteCommand(0xF1);
    
    /* Set VCOMH Deselect Level */
    SSD1306_WriteCommand(SSD1306_CMD_SET_VCOMH);
    SSD1306_WriteCommand(0x40);
    
    /* Set Memory Addressing Mode (Page addressing mode) */
    SSD1306_WriteCommand(SSD1306_CMD_MEMORY_MODE);
    SSD1306_WriteCommand(0x10);
    
    /* Enable Charge Pump */
    SSD1306_WriteCommand(SSD1306_CMD_CHARGE_PUMP);
    SSD1306_WriteCommand(0x14);
    
    /* Deactivate scroll */
    SSD1306_WriteCommand(SSD1306_CMD_SCROLL_STOP);
    
    /* Clear buffer */
    SSD1306_Fill(SSD1306_BLACK);
    
    /* Update screen */
    SSD1306_UpdateScreen();
    
    /* Turn on display */
    SSD1306_WriteCommand(SSD1306_CMD_DISPLAY_ON);
    
    /* Reset cursor */
    ssd1306_cursor_x = 0;
    ssd1306_cursor_y = 0;
    
    return SSD1306_OK;
}

/**
 * @brief Initialize SSD1306 with DMA support
 */
SSD1306_Status_t SSD1306_Init_DMA(void) {
    SSD1306_Status_t status;
    
    /* First do normal initialization */
    status = SSD1306_Init();
    if (status != SSD1306_OK) {
        return status;
    }
    
    /* Initialize I2C DMA */
    I2C_DMA_Init(SSD1306_I2C_INSTANCE);
    
    /* Register callback */
    I2C_SetCallback_DMA(SSD1306_DMA_Callback);
    
    /* Mark DMA as enabled */
    ssd1306_dma_enabled = 1;
    ssd1306_dma_page = 0;
    ssd1306_dma_busy = 0;
    
    return SSD1306_OK;
}

/* ========================================================================== */
/* SCREEN UPDATE FUNCTIONS                                                    */
/* ========================================================================== */

/**
 * @brief Update screen (blocking mode) - takes ~50-100ms
 */
void SSD1306_UpdateScreen(void) {
    uint8_t page;
    
    for (page = 0; page < (SSD1306_HEIGHT / 8); page++) {
        SSD1306_WriteCommand(SSD1306_CMD_SET_PAGE_ADDR + page);
        SSD1306_WriteCommand(SSD1306_CMD_SET_LOW_COLUMN);
        SSD1306_WriteCommand(SSD1306_CMD_SET_HIGH_COLUMN);
        
        SSD1306_WriteData(&ssd1306_buffer[SSD1306_WIDTH * page], SSD1306_WIDTH);
        
        /* Small delay between pages for I2C stability */
        delay_us_blocking(50);
    }
}

/**
 * @brief Update screen using DMA (non-blocking mode)
 * @return 1 if started successfully, 0 if DMA busy
 * @note Returns IMMEDIATELY (< 1ms), update happens in background
 */
uint8_t SSD1306_UpdateScreen_DMA(void) {
    if (!ssd1306_dma_enabled) {
        return 0;  /* DMA not initialized */
    }
    
    if (ssd1306_dma_busy) {
        /* [FIX] Timeout: if DMA has been busy too long, force-reset it */
        if ((millis() - ssd1306_dma_start_time) > SSD1306_DMA_TIMEOUT_MS) {
            ssd1306_dma_busy = 0;
            ssd1306_dma_page = 0;
            ssd1306_dma_error_count++;
        } else {
            return 0;  /* Still working, be patient */
        }
    }
    
    /* Mark as busy */
    ssd1306_dma_busy = 1;
    ssd1306_dma_page = 0;
    ssd1306_dma_start_time = millis();
    
    /* Start first page transfer */
    SSD1306_Process_DMA();
    
    return 1;
}

/**
 * @brief Process DMA state machine
 * @note MUST BE CALLED IN MAIN LOOP!
 */
void SSD1306_Process_DMA(void) {
    uint16_t i;
    I2C_Status dma_result;
    static uint32_t last_refresh_time = 0;
    
    if (!ssd1306_dma_enabled) {
        return;  /* DMA not initialized */
    }
    
    /* [FIX] Periodic safety: re-send Display ON command every 5s
     * to recover from glitched commands that may turn display off */
    if ((millis() - last_refresh_time) > SSD1306_REFRESH_PERIOD) {
        last_refresh_time = millis();
        /* Only send when bus is idle to avoid collision */
        if (!I2C_IsBusy_DMA(SSD1306_I2C_INSTANCE) && 
            !(I2C1->SR2 & I2C_SR2_BUSY)) {
            SSD1306_WriteCommand(SSD1306_CMD_DISPLAY_ON);
        }
        /* [FIX] If too many consecutive errors, re-init display */
        if (ssd1306_dma_error_count >= SSD1306_DMA_MAX_ERRORS) {
            ssd1306_dma_error_count = 0;
            ssd1306_dma_busy = 0;
            /* Re-send critical init commands */
            SSD1306_WriteCommand(SSD1306_CMD_DISPLAY_ON);
            SSD1306_WriteCommand(SSD1306_CMD_MEMORY_MODE);
            SSD1306_WriteCommand(0x10);  /* Page addressing mode */
            SSD1306_WriteCommand(0xA4);  /* Output follows RAM */
            SSD1306_WriteCommand(SSD1306_CMD_DISPLAY_NORMAL);
        }
    }
    
    if (!ssd1306_dma_busy) {
        return;  /* No pending update */
    }
    
    /* [FIX] Check BOTH software DMA flag AND hardware I2C bus state.
     * This prevents collision with MPU6050 RX DMA on same bus. */
    if (I2C_IsBusy_DMA(SSD1306_I2C_INSTANCE)) {
        return;  /* TX DMA still running */
    }
    if (I2C1->SR2 & I2C_SR2_BUSY) {
        return;  /* Hardware bus busy (e.g. MPU6050 RX DMA) */
    }
    
    /* [FIX] Timeout check: if stuck too long, abort this update */
    if ((millis() - ssd1306_dma_start_time) > SSD1306_DMA_TIMEOUT_MS) {
        ssd1306_dma_busy = 0;
        ssd1306_dma_page = 0;
        ssd1306_dma_error_count++;
        return;
    }
    
    /* Check if all pages sent */
    if (ssd1306_dma_page >= (SSD1306_HEIGHT / 8)) {
        ssd1306_dma_busy = 0;
        ssd1306_dma_error_count = 0;  /* Reset error counter on success */
        return;  /* All done */
    }
    
    /* Set page address (send command using blocking I2C) */
    SSD1306_WriteCommand(SSD1306_CMD_SET_PAGE_ADDR + ssd1306_dma_page);
    SSD1306_WriteCommand(SSD1306_CMD_SET_LOW_COLUMN);
    SSD1306_WriteCommand(SSD1306_CMD_SET_HIGH_COLUMN);
    
    /* Prepare chunk buffer: control byte + 128 data bytes */
    ssd1306_dma_chunk_buffer[0] = 0x40;  /* Control byte: D/C=1 (data) */
    for (i = 0; i < SSD1306_WIDTH; i++) {
        ssd1306_dma_chunk_buffer[i + 1] = ssd1306_buffer[SSD1306_WIDTH * ssd1306_dma_page + i];
    }
    
    /* [FIX] Check return value and handle errors */
    dma_result = I2C_Write_DMA(SSD1306_I2C_INSTANCE, SSD1306_I2C_ADDR, 
                               ssd1306_dma_chunk_buffer, 129);
    if (dma_result != I2C_OK) {
        /* DMA failed to start - skip this page, try next one.
         * This prevents getting stuck retrying the same failed page. */
        ssd1306_dma_page++;
        ssd1306_dma_error_count++;
    }
    /* On success, page counter will be incremented in DMA callback */
}

/**
 * @brief Check if DMA update is complete
 * @return 1 if ready for next update, 0 if busy
 */
uint8_t SSD1306_IsReady_DMA(void) {
    return !ssd1306_dma_busy;
}

/* ========================================================================== */
/* BASIC DISPLAY FUNCTIONS                                                    */
/* ========================================================================== */

void SSD1306_Fill(SSD1306_Color_t color) {
    memset(ssd1306_buffer, (color == SSD1306_BLACK) ? 0x00 : 0xFF, SSD1306_BUFFER_SIZE);
}

void SSD1306_Clear(void) {
    SSD1306_Fill(SSD1306_BLACK);
    SSD1306_UpdateScreen();
}

void SSD1306_DisplayOn(void) {
    SSD1306_WriteCommand(SSD1306_CMD_CHARGE_PUMP);
    SSD1306_WriteCommand(0x14);
    SSD1306_WriteCommand(SSD1306_CMD_DISPLAY_ON);
}

void SSD1306_DisplayOff(void) {
    SSD1306_WriteCommand(SSD1306_CMD_CHARGE_PUMP);
    SSD1306_WriteCommand(0x10);
    SSD1306_WriteCommand(SSD1306_CMD_DISPLAY_OFF);
}

void SSD1306_SetContrast(uint8_t contrast) {
    SSD1306_WriteCommand(SSD1306_CMD_SET_CONTRAST);
    SSD1306_WriteCommand(contrast);
}

void SSD1306_InvertDisplay(uint8_t invert) {
    if (invert) {
        SSD1306_WriteCommand(SSD1306_CMD_DISPLAY_INVERSE);
    } else {
        SSD1306_WriteCommand(SSD1306_CMD_DISPLAY_NORMAL);
    }
}

/* ========================================================================== */
/* PIXEL & CURSOR FUNCTIONS                                                   */
/* ========================================================================== */

void SSD1306_DrawPixel(uint16_t x, uint16_t y, SSD1306_Color_t color) {
    if (x >= SSD1306_WIDTH || y >= SSD1306_HEIGHT) {
        return;
    }
    
    if (ssd1306_inverted) {
        color = (SSD1306_Color_t)!color;
    }
    
    if (color == SSD1306_WHITE) {
        ssd1306_buffer[x + (y / 8) * SSD1306_WIDTH] |= (1 << (y % 8));
    } else {
        ssd1306_buffer[x + (y / 8) * SSD1306_WIDTH] &= ~(1 << (y % 8));
    }
}

void SSD1306_SetCursor(uint16_t x, uint16_t y) {
    ssd1306_cursor_x = x;
    ssd1306_cursor_y = y;
}

uint16_t SSD1306_GetCursorX(void) {
    return ssd1306_cursor_x;
}

uint16_t SSD1306_GetCursorY(void) {
    return ssd1306_cursor_y;
}

/* ========================================================================== */
/* TEXT FUNCTIONS - FIXED for 16-bit font data                               */
/* ========================================================================== */

/**
 * @brief Write single character - FIXED VERSION
 * @note Font data is 16-bit per row, MSB first
 */
char SSD1306_WriteChar(char ch, FontDef_t *font, SSD1306_Color_t color) {
    uint32_t i, j;
    uint16_t b;
    uint16_t char_offset;
    
    /* Check valid character range */
    if (ch < 32 || ch > 126) {
        return 0;
    }
    
    /* Check bounds */
    if (ssd1306_cursor_x + font->FontWidth > SSD1306_WIDTH ||
        ssd1306_cursor_y + font->FontHeight > SSD1306_HEIGHT) {
        return 0;
    }
    
    /* Calculate character offset in font data */
    char_offset = (ch - 32) * font->FontHeight;
    
    /* Draw character row by row */
    for (i = 0; i < font->FontHeight; i++) {
        /* Get font row data (16-bit) */
        b = font->data[char_offset + i];
        
        /* Draw each pixel in the row */
        for (j = 0; j < font->FontWidth; j++) {
            /* Check if bit is set (MSB first, shifted left) */
            if ((b << j) & 0x8000) {
                SSD1306_DrawPixel(ssd1306_cursor_x + j, ssd1306_cursor_y + i, color);
            } else {
                /* Draw background (opposite color) */
                SSD1306_DrawPixel(ssd1306_cursor_x + j, ssd1306_cursor_y + i, 
                                  (SSD1306_Color_t)!color);
            }
        }
    }
    
    /* Move cursor */
    ssd1306_cursor_x += font->FontWidth;
    
    return ch;
}

char SSD1306_WriteString(const char *str, FontDef_t *font, SSD1306_Color_t color) {
    while (*str) {
        if (SSD1306_WriteChar(*str, font, color) != *str) {
            return *str;
        }
        str++;
    }
    return 0;
}

void SSD1306_WriteInt(int32_t num, FontDef_t *font, SSD1306_Color_t color) {
    char buffer[12];
    char *ptr = buffer + 11;
    uint8_t negative = 0;
    
    *ptr = '\0';
    
    if (num == 0) {
        *--ptr = '0';
    } else {
        if (num < 0) {
            negative = 1;
            num = -num;
        }
        
        while (num > 0) {
            *--ptr = '0' + (num % 10);
            num /= 10;
        }
        
        if (negative) {
            *--ptr = '-';
        }
    }
    
    SSD1306_WriteString(ptr, font, color);
}

void SSD1306_WriteFloat(float num, uint8_t decimals, FontDef_t *font, SSD1306_Color_t color) {
    char buffer[20];
    int32_t int_part;
    float frac_part;
    uint8_t i;
    char *ptr = buffer;
    char temp[12];
    char *tp;
    
    if (num < 0) {
        *ptr++ = '-';
        num = -num;
    }
    
    int_part = (int32_t)num;
    frac_part = num - (float)int_part;
    
    tp = temp + 11;
    *tp = '\0';
    
    if (int_part == 0) {
        *--tp = '0';
    } else {
        while (int_part > 0) {
            *--tp = '0' + (int_part % 10);
            int_part /= 10;
        }
    }
    
    while (*tp) {
        *ptr++ = *tp++;
    }
    
    if (decimals > 0) {
        *ptr++ = '.';
        
        for (i = 0; i < decimals; i++) {
            frac_part *= 10;
            *ptr++ = '0' + (int)(frac_part) % 10;
        }
    }
    
    *ptr = '\0';
    SSD1306_WriteString(buffer, font, color);
}

/* ========================================================================== */
/* GRAPHICS PRIMITIVES                                                        */
/* ========================================================================== */

void SSD1306_DrawHLine(uint16_t x, uint16_t y, uint16_t length, SSD1306_Color_t color) {
    uint16_t i;
    for (i = 0; i < length; i++) {
        SSD1306_DrawPixel(x + i, y, color);
    }
}

void SSD1306_DrawVLine(uint16_t x, uint16_t y, uint16_t length, SSD1306_Color_t color) {
    uint16_t i;
    for (i = 0; i < length; i++) {
        SSD1306_DrawPixel(x, y + i, color);
    }
}

void SSD1306_DrawLine(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, SSD1306_Color_t color) {
    int16_t dx, dy, sx, sy, err, e2;
    
    if (x0 >= SSD1306_WIDTH) x0 = SSD1306_WIDTH - 1;
    if (x1 >= SSD1306_WIDTH) x1 = SSD1306_WIDTH - 1;
    if (y0 >= SSD1306_HEIGHT) y0 = SSD1306_HEIGHT - 1;
    if (y1 >= SSD1306_HEIGHT) y1 = SSD1306_HEIGHT - 1;
    
    dx = ABS((int16_t)x1 - (int16_t)x0);
    dy = ABS((int16_t)y1 - (int16_t)y0);
    sx = (x0 < x1) ? 1 : -1;
    sy = (y0 < y1) ? 1 : -1;
    err = dx - dy;
    
    while (1) {
        SSD1306_DrawPixel(x0, y0, color);
        
        if (x0 == x1 && y0 == y1) break;
        
        e2 = 2 * err;
        
        if (e2 > -dy) {
            err -= dy;
            x0 += sx;
        }
        
        if (e2 < dx) {
            err += dx;
            y0 += sy;
        }
    }
}

void SSD1306_DrawRect(uint16_t x, uint16_t y, uint16_t w, uint16_t h, SSD1306_Color_t color) {
    SSD1306_DrawHLine(x, y, w, color);
    SSD1306_DrawHLine(x, y + h - 1, w, color);
    SSD1306_DrawVLine(x, y, h, color);
    SSD1306_DrawVLine(x + w - 1, y, h, color);
}

void SSD1306_FillRect(uint16_t x, uint16_t y, uint16_t w, uint16_t h, SSD1306_Color_t color) {
    uint16_t i;
    for (i = 0; i < h; i++) {
        SSD1306_DrawHLine(x, y + i, w, color);
    }
}

void SSD1306_DrawCircle(int16_t x0, int16_t y0, int16_t r, SSD1306_Color_t color) {
    int16_t f = 1 - r;
    int16_t ddF_x = 1;
    int16_t ddF_y = -2 * r;
    int16_t x = 0;
    int16_t y = r;
    
    SSD1306_DrawPixel(x0, y0 + r, color);
    SSD1306_DrawPixel(x0, y0 - r, color);
    SSD1306_DrawPixel(x0 + r, y0, color);
    SSD1306_DrawPixel(x0 - r, y0, color);
    
    while (x < y) {
        if (f >= 0) {
            y--;
            ddF_y += 2;
            f += ddF_y;
        }
        x++;
        ddF_x += 2;
        f += ddF_x;
        
        SSD1306_DrawPixel(x0 + x, y0 + y, color);
        SSD1306_DrawPixel(x0 - x, y0 + y, color);
        SSD1306_DrawPixel(x0 + x, y0 - y, color);
        SSD1306_DrawPixel(x0 - x, y0 - y, color);
        SSD1306_DrawPixel(x0 + y, y0 + x, color);
        SSD1306_DrawPixel(x0 - y, y0 + x, color);
        SSD1306_DrawPixel(x0 + y, y0 - x, color);
        SSD1306_DrawPixel(x0 - y, y0 - x, color);
    }
}

void SSD1306_FillCircle(int16_t x0, int16_t y0, int16_t r, SSD1306_Color_t color) {
    int16_t f = 1 - r;
    int16_t ddF_x = 1;
    int16_t ddF_y = -2 * r;
    int16_t x = 0;
    int16_t y = r;
    
    SSD1306_DrawVLine(x0, y0 - r, 2 * r + 1, color);
    
    while (x < y) {
        if (f >= 0) {
            y--;
            ddF_y += 2;
            f += ddF_y;
        }
        x++;
        ddF_x += 2;
        f += ddF_x;
        
        SSD1306_DrawVLine(x0 + x, y0 - y, 2 * y + 1, color);
        SSD1306_DrawVLine(x0 - x, y0 - y, 2 * y + 1, color);
        SSD1306_DrawVLine(x0 + y, y0 - x, 2 * x + 1, color);
        SSD1306_DrawVLine(x0 - y, y0 - x, 2 * x + 1, color);
    }
}

void SSD1306_DrawTriangle(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2,
                          uint16_t x3, uint16_t y3, SSD1306_Color_t color) {
    SSD1306_DrawLine(x1, y1, x2, y2, color);
    SSD1306_DrawLine(x2, y2, x3, y3, color);
    SSD1306_DrawLine(x3, y3, x1, y1, color);
}

void SSD1306_FillTriangle(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2,
                          uint16_t x3, uint16_t y3, SSD1306_Color_t color) {
    int16_t deltax, deltay, x, y;
    int16_t xinc1, xinc2, yinc1, yinc2;
    int16_t den, num, numadd, numpixels, curpixel;
    
    deltax = ABS((int16_t)x2 - (int16_t)x1);
    deltay = ABS((int16_t)y2 - (int16_t)y1);
    x = x1;
    y = y1;
    
    xinc1 = (x2 >= x1) ? 1 : -1;
    xinc2 = xinc1;
    yinc1 = (y2 >= y1) ? 1 : -1;
    yinc2 = yinc1;
    
    if (deltax >= deltay) {
        xinc1 = 0;
        yinc2 = 0;
        den = deltax;
        num = deltax / 2;
        numadd = deltay;
        numpixels = deltax;
    } else {
        xinc2 = 0;
        yinc1 = 0;
        den = deltay;
        num = deltay / 2;
        numadd = deltax;
        numpixels = deltay;
    }
    
    for (curpixel = 0; curpixel <= numpixels; curpixel++) {
        SSD1306_DrawLine(x, y, x3, y3, color);
        
        num += numadd;
        if (num >= den) {
            num -= den;
            x += xinc1;
            y += yinc1;
        }
        x += xinc2;
        y += yinc2;
    }
}

/* ========================================================================== */
/* BITMAP FUNCTIONS                                                           */
/* ========================================================================== */

void SSD1306_DrawBitmap(int16_t x, int16_t y, const uint8_t *bitmap,
                        int16_t w, int16_t h, SSD1306_Color_t color) {
    int16_t byteWidth = (w + 7) / 8;
    int16_t i, j;
    uint8_t byte = 0;
    
    for (j = 0; j < h; j++) {
        for (i = 0; i < w; i++) {
            if (i & 7) {
                byte <<= 1;
            } else {
                byte = bitmap[j * byteWidth + i / 8];
            }
            
            if (byte & 0x80) {
                SSD1306_DrawPixel(x + i, y + j, color);
            }
        }
    }
}

/* ========================================================================== */
/* SCROLL FUNCTIONS                                                           */
/* ========================================================================== */

void SSD1306_ScrollRight(uint8_t start_page, uint8_t end_page) {
    SSD1306_WriteCommand(SSD1306_CMD_SCROLL_RIGHT);
    SSD1306_WriteCommand(0x00);
    SSD1306_WriteCommand(start_page);
    SSD1306_WriteCommand(0x00);
    SSD1306_WriteCommand(end_page);
    SSD1306_WriteCommand(0x00);
    SSD1306_WriteCommand(0xFF);
    SSD1306_WriteCommand(SSD1306_CMD_SCROLL_START);
}

void SSD1306_ScrollLeft(uint8_t start_page, uint8_t end_page) {
    SSD1306_WriteCommand(SSD1306_CMD_SCROLL_LEFT);
    SSD1306_WriteCommand(0x00);
    SSD1306_WriteCommand(start_page);
    SSD1306_WriteCommand(0x00);
    SSD1306_WriteCommand(end_page);
    SSD1306_WriteCommand(0x00);
    SSD1306_WriteCommand(0xFF);
    SSD1306_WriteCommand(SSD1306_CMD_SCROLL_START);
}

void SSD1306_ScrollDiagRight(uint8_t start_page, uint8_t end_page) {
    SSD1306_WriteCommand(SSD1306_CMD_SET_SCROLL_AREA);
    SSD1306_WriteCommand(0x00);
    SSD1306_WriteCommand(SSD1306_HEIGHT);
    SSD1306_WriteCommand(SSD1306_CMD_SCROLL_DIAG_RIGHT);
    SSD1306_WriteCommand(0x00);
    SSD1306_WriteCommand(start_page);
    SSD1306_WriteCommand(0x00);
    SSD1306_WriteCommand(end_page);
    SSD1306_WriteCommand(0x01);
    SSD1306_WriteCommand(SSD1306_CMD_SCROLL_START);
}

void SSD1306_ScrollDiagLeft(uint8_t start_page, uint8_t end_page) {
    SSD1306_WriteCommand(SSD1306_CMD_SET_SCROLL_AREA);
    SSD1306_WriteCommand(0x00);
    SSD1306_WriteCommand(SSD1306_HEIGHT);
    SSD1306_WriteCommand(SSD1306_CMD_SCROLL_DIAG_LEFT);
    SSD1306_WriteCommand(0x00);
    SSD1306_WriteCommand(start_page);
    SSD1306_WriteCommand(0x00);
    SSD1306_WriteCommand(end_page);
    SSD1306_WriteCommand(0x01);
    SSD1306_WriteCommand(SSD1306_CMD_SCROLL_START);
}

void SSD1306_ScrollStop(void) {
    SSD1306_WriteCommand(SSD1306_CMD_SCROLL_STOP);
}

/* ========================================================================== */
/* UTILITY FUNCTIONS                                                          */
/* ========================================================================== */

void SSD1306_ToggleInvert(void) {
    uint16_t i;
    
    ssd1306_inverted = !ssd1306_inverted;
    
    for (i = 0; i < SSD1306_BUFFER_SIZE; i++) {
        ssd1306_buffer[i] = ~ssd1306_buffer[i];
    }
}

SSD1306_Status_t SSD1306_IsConnected(void) {
    if (I2C_IsDeviceReady(SSD1306_I2C_INSTANCE, SSD1306_I2C_ADDR) == I2C_OK) {
        return SSD1306_OK;
    }
    return SSD1306_NOT_FOUND;
}
