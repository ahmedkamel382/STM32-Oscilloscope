/**
 * ============================================================================
 *  st7789.c  --  Implementation of the ST7789 driver (see st7789.h).
 *  Tested on the GMT147SPI 1.47" 172x320 module.
 * ============================================================================
 */
#include "st7789.h"
#include "font5x7.h"
#include <string.h>
#include <stdarg.h>
#include <stdio.h>

static SPI_HandleTypeDef *s_hspi = NULL;

/* ---- Pin helpers ------------------------------------------------------- */
static inline void cs_low(void)  { HAL_GPIO_WritePin(TFT_CS_GPIO_Port,  TFT_CS_Pin,  GPIO_PIN_RESET); }
static inline void cs_high(void) { HAL_GPIO_WritePin(TFT_CS_GPIO_Port,  TFT_CS_Pin,  GPIO_PIN_SET);   }
static inline void dc_cmd(void)  { HAL_GPIO_WritePin(TFT_DC_GPIO_Port,  TFT_DC_Pin,  GPIO_PIN_RESET); }
static inline void dc_data(void) { HAL_GPIO_WritePin(TFT_DC_GPIO_Port,  TFT_DC_Pin,  GPIO_PIN_SET);   }

static void write_cmd(uint8_t cmd) {
    dc_cmd();
    cs_low();
    HAL_SPI_Transmit(s_hspi, &cmd, 1, HAL_MAX_DELAY);
    cs_high();
}

static void write_data(const uint8_t *data, uint32_t len) {
    dc_data();
    cs_low();
    HAL_SPI_Transmit(s_hspi, (uint8_t *)data, len, HAL_MAX_DELAY);
    cs_high();
}

static void write_data8(uint8_t b)            { write_data(&b, 1); }
static void write_data16(uint16_t w)          { uint8_t b[2] = { w >> 8, w & 0xFF }; write_data(b, 2); }

/* ---- ST7789 command set we use ---------------------------------------- */
#define ST_NOP      0x00
#define ST_SWRESET  0x01
#define ST_SLPOUT   0x11
#define ST_SLPIN    0x10
#define ST_NORON    0x13
#define ST_INVOFF   0x20
#define ST_INVON    0x21
#define ST_DISPOFF  0x28
#define ST_DISPON   0x29
#define ST_CASET    0x2A
#define ST_RASET    0x2B
#define ST_RAMWR    0x2C
#define ST_MADCTL   0x36
#define ST_COLMOD   0x3A
#define ST_PORCTRL  0xB2
#define ST_GCTRL    0xB7
#define ST_VCOMS    0xBB
#define ST_LCMCTRL  0xC0
#define ST_VDVVRHEN 0xC2
#define ST_VRHS     0xC3
#define ST_VDVS     0xC4
#define ST_FRCTRL2  0xC6
#define ST_PWCTRL1  0xD0
#define ST_PVGAMCTRL 0xE0
#define ST_NVGAMCTRL 0xE1

void ST7789_Reset(void) {
    HAL_GPIO_WritePin(TFT_RST_GPIO_Port, TFT_RST_Pin, GPIO_PIN_SET);
    HAL_Delay(20);
    HAL_GPIO_WritePin(TFT_RST_GPIO_Port, TFT_RST_Pin, GPIO_PIN_RESET);
    HAL_Delay(20);
    HAL_GPIO_WritePin(TFT_RST_GPIO_Port, TFT_RST_Pin, GPIO_PIN_SET);
    HAL_Delay(120);
}

void ST7789_Init(SPI_HandleTypeDef *hspi) {
    s_hspi = hspi;
    cs_high();
    ST7789_Reset();

    write_cmd(ST_SWRESET);  HAL_Delay(150);
    write_cmd(ST_SLPOUT);   HAL_Delay(120);

    /* 65k colour, 16bpp */
    write_cmd(ST_COLMOD);   write_data8(0x55);

    /* MADCTL  -- landscape, RGB order, scan top-left to bottom-right.
       For the GMT147SPI 1.47" panel mounted with the FPC on the right we use
       0x60 (MX=0, MY=1, MV=1, ML=0, RGB).                                 */
    write_cmd(ST_MADCTL);   write_data8(0x60);

    /* Porch, gate, VCOM, gamma -- standard ST7789 init taken from the
     * Sitronix datasheet reference sequence, lightly tuned for 172x320.   */
    write_cmd(ST_PORCTRL);  { uint8_t d[] = {0x0C,0x0C,0x00,0x33,0x33}; write_data(d,5); }
    write_cmd(ST_GCTRL);    write_data8(0x35);
    write_cmd(ST_VCOMS);    write_data8(0x19);
    write_cmd(ST_LCMCTRL);  write_data8(0x2C);
    write_cmd(ST_VDVVRHEN); write_data8(0x01);
    write_cmd(ST_VRHS);     write_data8(0x12);
    write_cmd(ST_VDVS);     write_data8(0x20);
    write_cmd(ST_FRCTRL2);  write_data8(0x0F);  /* ~60 Hz frame rate */
    write_cmd(ST_PWCTRL1);  { uint8_t d[] = {0xA4,0xA1}; write_data(d,2); }
    write_cmd(ST_PVGAMCTRL);{ uint8_t d[]={0xD0,0x04,0x0D,0x11,0x13,0x2B,0x3F,0x54,0x4C,0x18,0x0D,0x0B,0x1F,0x23}; write_data(d,14); }
    write_cmd(ST_NVGAMCTRL);{ uint8_t d[]={0xD0,0x04,0x0C,0x11,0x13,0x2C,0x3F,0x44,0x51,0x2F,0x1F,0x1F,0x20,0x23}; write_data(d,14); }

    write_cmd(ST_INVON);    HAL_Delay(10);
    write_cmd(ST_NORON);    HAL_Delay(10);
    write_cmd(ST_DISPON);   HAL_Delay(120);

    ST7789_FillScreen(COLOR_BLACK);
}

void ST7789_Sleep(bool on) {
    write_cmd(on ? ST_SLPIN : ST_SLPOUT);
    HAL_Delay(120);
}

void ST7789_SetWindow(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1) {
    /* In landscape mode the 1.47" panel is offset; CASET maps to cols. */
    uint16_t xs = x0 + ST7789_X_OFFSET;
    uint16_t xe = x1 + ST7789_X_OFFSET;
    uint16_t ys = y0 + ST7789_Y_OFFSET;
    uint16_t ye = y1 + ST7789_Y_OFFSET;
    uint8_t bx[4] = { xs >> 8, xs & 0xFF, xe >> 8, xe & 0xFF };
    uint8_t by[4] = { ys >> 8, ys & 0xFF, ye >> 8, ye & 0xFF };
    write_cmd(ST_CASET); write_data(bx, 4);
    write_cmd(ST_RASET); write_data(by, 4);
    write_cmd(ST_RAMWR);
}

void ST7789_PushColors(const uint16_t *data, uint32_t len) {
    /* Send 16-bit pixels MSB first (ST7789 expects high byte first). */
    dc_data();
    cs_low();
    /* Use 16-bit SPI by transmitting two bytes per pixel.  HAL_SPI_Transmit
     * is byte-oriented; we feed a temp buffer in 64-pixel chunks to keep
     * stack usage tiny.                                                    */
    uint8_t chunk[128];
    while (len) {
        uint32_t n = len > 64 ? 64 : len;
        for (uint32_t i = 0; i < n; ++i) {
            chunk[2*i  ] = data[i] >> 8;
            chunk[2*i+1] = data[i] & 0xFF;
        }
        HAL_SPI_Transmit(s_hspi, chunk, 2*n, HAL_MAX_DELAY);
        data += n;
        len  -= n;
    }
    cs_high();
}

void ST7789_FillRect(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color) {
    if (x >= ST7789_WIDTH || y >= ST7789_HEIGHT) return;
    if (x + w > ST7789_WIDTH)  w = ST7789_WIDTH  - x;
    if (y + h > ST7789_HEIGHT) h = ST7789_HEIGHT - y;
    ST7789_SetWindow(x, y, x + w - 1, y + h - 1);
    uint8_t hi = color >> 8, lo = color & 0xFF;
    uint8_t buf[64];
    for (int i = 0; i < 32; ++i) { buf[2*i] = hi; buf[2*i+1] = lo; }
    dc_data();
    cs_low();
    uint32_t total = (uint32_t)w * h;
    while (total) {
        uint32_t n = total > 32 ? 32 : total;
        HAL_SPI_Transmit(s_hspi, buf, 2*n, HAL_MAX_DELAY);
        total -= n;
    }
    cs_high();
}

void ST7789_FillScreen(uint16_t color) {
    ST7789_FillRect(0, 0, ST7789_WIDTH, ST7789_HEIGHT, color);
}

void ST7789_DrawPixel(uint16_t x, uint16_t y, uint16_t color) {
    if (x >= ST7789_WIDTH || y >= ST7789_HEIGHT) return;
    ST7789_SetWindow(x, y, x, y);
    write_data16(color);
}

void ST7789_DrawHLine(uint16_t x, uint16_t y, uint16_t w, uint16_t color) {
    ST7789_FillRect(x, y, w, 1, color);
}
void ST7789_DrawVLine(uint16_t x, uint16_t y, uint16_t h, uint16_t color) {
    ST7789_FillRect(x, y, 1, h, color);
}

void ST7789_DrawRect(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color) {
    ST7789_DrawHLine(x, y, w, color);
    ST7789_DrawHLine(x, y + h - 1, w, color);
    ST7789_DrawVLine(x, y, h, color);
    ST7789_DrawVLine(x + w - 1, y, h, color);
}

/* Bresenham line draw -- used by the waveform renderer. */
void ST7789_DrawLine(int16_t x0, int16_t y0, int16_t x1, int16_t y1, uint16_t color) {
    int16_t dx =  (x1 > x0) ? (x1 - x0) : (x0 - x1);
    int16_t dy = -((y1 > y0) ? (y1 - y0) : (y0 - y1));
    int16_t sx =  (x0 < x1) ? 1 : -1;
    int16_t sy =  (y0 < y1) ? 1 : -1;
    int16_t err = dx + dy;
    while (1) {
        ST7789_DrawPixel((uint16_t)x0, (uint16_t)y0, color);
        if (x0 == x1 && y0 == y1) break;
        int16_t e2 = err * 2;
        if (e2 >= dy) { err += dy; x0 += sx; }
        if (e2 <= dx) { err += dx; y0 += sy; }
    }
}

void ST7789_DrawChar(uint16_t x, uint16_t y, char c, uint16_t fg, uint16_t bg, uint8_t scale) {
    if (c < 32 || c > 126) c = '?';
    const uint8_t *g = Font5x7[c - 32];
    for (uint8_t col = 0; col < 5; ++col) {
        uint8_t bits = g[col];
        for (uint8_t row = 0; row < 7; ++row) {
            uint16_t color = (bits & (1 << row)) ? fg : bg;
            ST7789_FillRect(x + col*scale, y + row*scale, scale, scale, color);
        }
    }
    /* one-pixel inter-glyph gap */
    ST7789_FillRect(x + 5*scale, y, scale, 7*scale, bg);
}

void ST7789_DrawString(uint16_t x, uint16_t y, const char *s, uint16_t fg, uint16_t bg, uint8_t scale) {
    while (*s) {
        if (x + 6*scale > ST7789_WIDTH) break;
        ST7789_DrawChar(x, y, *s++, fg, bg, scale);
        x += 6 * scale;
    }
}

void ST7789_PrintF(uint16_t x, uint16_t y, uint16_t fg, uint16_t bg, uint8_t scale, const char *fmt, ...) {
    char buf[64];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(buf, sizeof buf, fmt, ap);
    va_end(ap);
    ST7789_DrawString(x, y, buf, fg, bg, scale);
}
