/**
 * ============================================================================
 *  st7789.h  --  Minimal ST7789 driver for the GMT147SPI 1.47" 172x320 panel
 *                used by the AAST DSO project (STM32F401 Black Pill).
 *
 *  Pin map (matches master blueprint Phase 2):
 *    SCL -> PA5  (SPI1_SCK)
 *    SDA -> PA7  (SPI1_MOSI)
 *    RES -> PA2  (GPIO out, active low)
 *    DC  -> PA3  (GPIO out, 0=command, 1=data)
 *    CS  -> PA4  (GPIO out, active low)
 *    BL  -> 3V3 (always on, no pin)
 *
 *  This driver pushes pixel frames via blocking + DMA SPI to keep the redraw
 *  loop fast enough for 500 kHz signals.
 * ============================================================================
 */
#ifndef ST7789_H
#define ST7789_H

#include "main.h"
#include <stdint.h>
#include <stdbool.h>

/* ---- Panel geometry ----------------------------------------------------- */
/* The GMT147SPI module is a 172x320 ST7789V2.  We use it in landscape mode
 * (320 wide x 172 tall) so the waveform graticule matches a classic scope. */
#define ST7789_WIDTH        320
#define ST7789_HEIGHT       172
#define ST7789_X_OFFSET     0      /* landscape MADCTL handles offset */
#define ST7789_Y_OFFSET     34     /* 1.47" panel centred in 172/320 RAM */

/* ---- 16-bit RGB565 colours --------------------------------------------- */
#define COLOR_BLACK         0x0000
#define COLOR_WHITE         0xFFFF
#define COLOR_RED           0xF800
#define COLOR_GREEN         0x07E0
#define COLOR_BLUE          0x001F
#define COLOR_YELLOW        0xFFE0
#define COLOR_CYAN          0x07FF
#define COLOR_MAGENTA       0xF81F
#define COLOR_ORANGE        0xFD20
#define COLOR_GRAY          0x8410
#define COLOR_DARKGRAY      0x4208
#define COLOR_LIGHTGRAY     0xC618
#define COLOR_GRID          0x2104  /* very dark grid background */

/* ---- Public API --------------------------------------------------------- */
void ST7789_Init(SPI_HandleTypeDef *hspi);
void ST7789_Reset(void);
void ST7789_Sleep(bool on);

void ST7789_SetWindow(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1);
void ST7789_FillRect(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color);
void ST7789_FillScreen(uint16_t color);
void ST7789_DrawPixel(uint16_t x, uint16_t y, uint16_t color);
void ST7789_DrawHLine(uint16_t x, uint16_t y, uint16_t w, uint16_t color);
void ST7789_DrawVLine(uint16_t x, uint16_t y, uint16_t h, uint16_t color);
void ST7789_DrawLine(int16_t x0, int16_t y0, int16_t x1, int16_t y1, uint16_t color);
void ST7789_DrawRect(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color);

/* Text helpers (5x7 monospace, scalable) */
void ST7789_DrawChar(uint16_t x, uint16_t y, char c, uint16_t fg, uint16_t bg, uint8_t scale);
void ST7789_DrawString(uint16_t x, uint16_t y, const char *s, uint16_t fg, uint16_t bg, uint8_t scale);
void ST7789_PrintF(uint16_t x, uint16_t y, uint16_t fg, uint16_t bg, uint8_t scale, const char *fmt, ...);

/* Bulk pixel push (used by DSO renderer for column erase/draw) */
void ST7789_PushColors(const uint16_t *data, uint32_t len);

#endif /* ST7789_H */
