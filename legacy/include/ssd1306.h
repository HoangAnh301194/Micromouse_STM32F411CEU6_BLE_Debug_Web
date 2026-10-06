/**
 * @file ssd1306.h
 * @brief SSD1306 OLED Driver with DMA Support
 * @author HoangAnh
 * @date 2025
 * 
 * Features:
 * - 128x64 or 128x32 OLED support
 * - Multiple font sizes (7x10, 11x18, 16x26)
 * - Graphics primitives (line, rectangle, circle, triangle)
 * - Bitmap rendering
 * - Scrolling effects
 * - **DMA support for non-blocking screen update**
 */

#ifndef SSD1306_H
#define SSD1306_H

#include <stdint.h>
#include <string.h>
#include "i2c.h"
#include "fonts.h"

/* ============================================================================ */
/* CONFIGURATION                                                                 */
/* ============================================================================ */

/* I2C Instance */
#define SSD1306_I2C_INSTANCE    I2C_1

/* I2C Address (0x3C or 0x3D, 7-bit) */
#define SSD1306_I2C_ADDR        0x3C

/* Screen Size */
#define SSD1306_WIDTH           128
#define SSD1306_HEIGHT          64      /* 64 for 128x64, 32 for 128x32 */

/* Buffer size */
#define SSD1306_BUFFER_SIZE     (SSD1306_WIDTH * SSD1306_HEIGHT / 8)

/* ============================================================================ */
/* DATA TYPES                                                                    */
/* ============================================================================ */

/**
 * @brief SSD1306 Color
 */
typedef enum {
    SSD1306_BLACK = 0,      /* Pixel off */
    SSD1306_WHITE = 1       /* Pixel on */
} SSD1306_Color_t;

/**
 * @brief SSD1306 Status
 */
typedef enum {
    SSD1306_OK = 0,
    SSD1306_ERROR,
    SSD1306_NOT_FOUND
} SSD1306_Status_t;

/* ============================================================================ */
/* CORE FUNCTIONS (BLOCKING MODE)                                               */
/* ============================================================================ */

/**
 * @brief Initialize SSD1306 OLED (blocking mode)
 * @return SSD1306_OK if success, SSD1306_NOT_FOUND if device not detected
 */
SSD1306_Status_t SSD1306_Init(void);

/**
 * @brief Update screen - BLOCKING (50-100ms)
 * @note Call this after drawing to see changes
 */
void SSD1306_UpdateScreen(void);

/**
 * @brief Fill entire screen with color
 * @param color: SSD1306_BLACK or SSD1306_WHITE
 */
void SSD1306_Fill(SSD1306_Color_t color);

/**
 * @brief Clear screen (fill with black) and update
 */
void SSD1306_Clear(void);

/**
 * @brief Turn display ON
 */
void SSD1306_DisplayOn(void);

/**
 * @brief Turn display OFF (sleep mode)
 */
void SSD1306_DisplayOff(void);

/**
 * @brief Set display contrast
 * @param contrast: 0-255
 */
void SSD1306_SetContrast(uint8_t contrast);

/**
 * @brief Invert display colors
 * @param invert: 1 = inverted, 0 = normal
 */
void SSD1306_InvertDisplay(uint8_t invert);

/* ============================================================================ */
/* DMA FUNCTIONS (NON-BLOCKING MODE) - **NEW**                                  */
/* ============================================================================ */

/**
 * @brief Initialize SSD1306 with DMA support
 * @return SSD1306_OK if success
 * @note Ph?i g?i hm ny thay v SSD1306_Init() n?u mu?n dng DMA
 */
SSD1306_Status_t SSD1306_Init_DMA(void);

/**
 * @brief Update screen using DMA (NON-BLOCKING)
 * @return 1 if started successfully, 0 if DMA busy
 * @note Hm tr? v? NGAY (< 1ms), screen update di?n ra background
 * @note Ph?i g?i SSD1306_Process_DMA() trong main loop
 */
uint8_t SSD1306_UpdateScreen_DMA(void);

/**
 * @brief Process DMA state machine
 * @note **PH?I G?I HM NY TRONG MAIN LOOP!**
 * @note Hm ny qu?n l vi?c g?i t?ng page ln OLED
 */
void SSD1306_Process_DMA(void);

/**
 * @brief Check if DMA update is complete
 * @return 1 if ready for next update, 0 if busy
 */
uint8_t SSD1306_IsReady_DMA(void);

/* ============================================================================ */
/* PIXEL & CURSOR FUNCTIONS                                                      */
/* ============================================================================ */

/**
 * @brief Draw single pixel
 * @param x: X coordinate (0 to SSD1306_WIDTH-1)
 * @param y: Y coordinate (0 to SSD1306_HEIGHT-1)
 * @param color: SSD1306_BLACK or SSD1306_WHITE
 */
void SSD1306_DrawPixel(uint16_t x, uint16_t y, SSD1306_Color_t color);

/**
 * @brief Set cursor position for text
 * @param x: X coordinate
 * @param y: Y coordinate
 */
void SSD1306_SetCursor(uint16_t x, uint16_t y);

/**
 * @brief Get current cursor X position
 */
uint16_t SSD1306_GetCursorX(void);

/**
 * @brief Get current cursor Y position
 */
uint16_t SSD1306_GetCursorY(void);

/* ============================================================================ */
/* TEXT FUNCTIONS                                                                */
/* ============================================================================ */

/**
 * @brief Write single character
 * @param ch: Character to write
 * @param font: Pointer to font structure (FontDef_t)
 * @param color: Text color
 * @return Written character, 0 if failed
 */
char SSD1306_WriteChar(char ch, FontDef_t *font, SSD1306_Color_t color);

/**
 * @brief Write string
 * @param str: String to write
 * @param font: Pointer to font structure (FontDef_t)
 * @param color: Text color
 * @return 0 if success, character that failed otherwise
 */
char SSD1306_WriteString(const char *str, FontDef_t *font, SSD1306_Color_t color);

/**
 * @brief Write integer number
 * @param num: Number to write
 * @param font: Pointer to font structure
 * @param color: Text color
 */
void SSD1306_WriteInt(int32_t num, FontDef_t *font, SSD1306_Color_t color);

/**
 * @brief Write float number
 * @param num: Number to write
 * @param decimals: Number of decimal places
 * @param font: Pointer to font structure
 * @param color: Text color
 */
void SSD1306_WriteFloat(float num, uint8_t decimals, FontDef_t *font, SSD1306_Color_t color);

/* ============================================================================ */
/* GRAPHICS PRIMITIVES                                                           */
/* ============================================================================ */

/**
 * @brief Draw line
 * @param x0, y0: Start point
 * @param x1, y1: End point
 * @param color: Line color
 */
void SSD1306_DrawLine(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, SSD1306_Color_t color);

/**
 * @brief Draw horizontal line (optimized)
 */
void SSD1306_DrawHLine(uint16_t x, uint16_t y, uint16_t length, SSD1306_Color_t color);

/**
 * @brief Draw vertical line (optimized)
 */
void SSD1306_DrawVLine(uint16_t x, uint16_t y, uint16_t length, SSD1306_Color_t color);

/**
 * @brief Draw rectangle outline
 * @param x, y: Top-left corner
 * @param w, h: Width and height
 * @param color: Line color
 */
void SSD1306_DrawRect(uint16_t x, uint16_t y, uint16_t w, uint16_t h, SSD1306_Color_t color);

/**
 * @brief Draw filled rectangle
 */
void SSD1306_FillRect(uint16_t x, uint16_t y, uint16_t w, uint16_t h, SSD1306_Color_t color);

/**
 * @brief Draw circle outline
 * @param x0, y0: Center point
 * @param r: Radius
 * @param color: Line color
 */
void SSD1306_DrawCircle(int16_t x0, int16_t y0, int16_t r, SSD1306_Color_t color);

/**
 * @brief Draw filled circle
 */
void SSD1306_FillCircle(int16_t x0, int16_t y0, int16_t r, SSD1306_Color_t color);

/**
 * @brief Draw triangle outline
 */
void SSD1306_DrawTriangle(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, 
                          uint16_t x3, uint16_t y3, SSD1306_Color_t color);

/**
 * @brief Draw filled triangle
 */
void SSD1306_FillTriangle(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2,
                          uint16_t x3, uint16_t y3, SSD1306_Color_t color);

/* ============================================================================ */
/* BITMAP FUNCTIONS                                                              */
/* ============================================================================ */

/**
 * @brief Draw bitmap image
 * @param x, y: Top-left position
 * @param bitmap: Pointer to bitmap data
 * @param w, h: Bitmap width and height
 * @param color: Foreground color
 */
void SSD1306_DrawBitmap(int16_t x, int16_t y, const uint8_t *bitmap, 
                        int16_t w, int16_t h, SSD1306_Color_t color);

/* ============================================================================ */
/* SCROLL FUNCTIONS                                                              */
/* ============================================================================ */

/**
 * @brief Start horizontal scroll right
 * @param start_page: Start page (0-7)
 * @param end_page: End page (0-7)
 */
void SSD1306_ScrollRight(uint8_t start_page, uint8_t end_page);

/**
 * @brief Start horizontal scroll left
 */
void SSD1306_ScrollLeft(uint8_t start_page, uint8_t end_page);

/**
 * @brief Start diagonal scroll right
 */
void SSD1306_ScrollDiagRight(uint8_t start_page, uint8_t end_page);

/**
 * @brief Start diagonal scroll left
 */
void SSD1306_ScrollDiagLeft(uint8_t start_page, uint8_t end_page);

/**
 * @brief Stop scrolling
 */
void SSD1306_ScrollStop(void);

/* ============================================================================ */
/* UTILITY FUNCTIONS                                                             */
/* ============================================================================ */

/**
 * @brief Toggle buffer inversion
 */
void SSD1306_ToggleInvert(void);

/**
 * @brief Check if device is connected
 * @return SSD1306_OK if connected
 */
SSD1306_Status_t SSD1306_IsConnected(void);

#endif /* SSD1306_H */
