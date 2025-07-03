/*
 * GLib_ST7735R_1.8_TFT
 * Copyright (C) 2025 Anatoliy Lizanets, Andrew Kushyk, Andriy Honcharenko, ScarsFun
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */
#pragma once

#include "fonts.h"
#include "st7735_cfg.h"
#include <stdbool.h>

extern SPI_HandleTypeDef ST7735_SPI_PORT;

typedef enum
{
    ST7735_MADCTL_MY    = 0x80,
    ST7735_MADCTL_MX    = 0x40,
    ST7735_MADCTL_MV    = 0x20,
    ST7735_MADCTL_RGB   = 0x00,
    ST7735_MADCTL_BGR   = 0x08,

    ST7735_WIDTH        = 128,
    ST7735_HEIGHT       = 160,
    ST7735_XSTART       = 0,
    ST7735_YSTART       = 0,
    ST7735_DATA_ROTATION = (ST7735_MADCTL_MX | ST7735_MADCTL_MY),

    ST7735_NOP          = 0x00,
    ST7735_SWRESET      = 0x01,
    ST7735_RDDID        = 0x04,
    ST7735_RDDST        = 0x09,

    ST7735_SLPIN        = 0x10,
    ST7735_SLPOUT       = 0x11,
    ST7735_PTLON        = 0x12,
    ST7735_NORON        = 0x13,

    ST7735_INVOFF       = 0x20,
    ST7735_INVON        = 0x21,
    ST7735_DISPOFF      = 0x28,
    ST7735_DISPON       = 0x29,
    ST7735_CASET        = 0x2A,
    ST7735_RASET        = 0x2B,
    ST7735_RAMWR        = 0x2C,
    ST7735_RAMRD        = 0x2E,

    ST7735_PTLAR        = 0x30,
    ST7735_COLMOD       = 0x3A,
    ST7735_MADCTL       = 0x36,

    ST7735_FRMCTR1      = 0xB1,
    ST7735_FRMCTR2      = 0xB2,
    ST7735_FRMCTR3      = 0xB3,
    ST7735_INVCTR       = 0xB4,
    ST7735_DISSET5      = 0xB6,

    ST7735_PWCTR1       = 0xC0,
    ST7735_PWCTR2       = 0xC1,
    ST7735_PWCTR3       = 0xC2,
    ST7735_PWCTR4       = 0xC3,
    ST7735_PWCTR5       = 0xC4,
    ST7735_VMCTR1       = 0xC5,

    ST7735_RDID1        = 0xDA,
    ST7735_RDID2        = 0xDB,
    ST7735_RDID3        = 0xDC,
    ST7735_RDID4        = 0xDD,

    ST7735_PWCTR6       = 0xFC,

    ST7735_GMCTRP1      = 0xE0,
    ST7735_GMCTRN1      = 0xE1,

    ST7735_BLACK        = 0x0000,
    ST7735_BLUE         = 0x001F,
    ST7735_RED          = 0xF800,
    ST7735_GREEN        = 0x07E0,
    ST7735_CYAN         = 0x07FF,
    ST7735_MAGENTA      = 0xF81F,
    ST7735_YELLOW       = 0xFFE0,
    ST7735_WHITE        = 0xFFFF

} driver_st_7735_constants;


void ST7735_Backlight_On (void);
void ST7735_Backlight_Off(void);
void ST7735_Init(void);
void ST7735_DrawImage        (uint16_t x, uint16_t y, uint16_t w, uint16_t h, const uint16_t* data);
void ST7735_DrawLine         (int16_t x0, int16_t y0, int16_t x1, int16_t y1, uint16_t color);
void ST7735_DrawFastVLine    (int16_t x, int16_t y, int16_t h, uint16_t color);
void ST7735_DrawFastHLine    (int16_t x, int16_t y, int16_t w, uint16_t color);
void ST7735_SetRotation      (uint8_t m);

#ifdef TESTING

void ST7735_DrawPixel        (uint16_t x, uint16_t y, uint16_t color);
void ST7735_DrawString       (uint16_t x, uint16_t y, const char* str, FontDef font, uint16_t color, uint16_t bgcolor);
void ST7735_FillRectangle    (uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color);
void ST7735_FillScreen       (uint16_t color);
void ST7735_InvertColors     (bool invert);
void ST7735_DrawCircle       (int16_t x0, int16_t y0, int16_t r, uint16_t color);
void ST7735_DrawCircleHelper ( int16_t x0, int16_t y0, int16_t r, uint8_t cornername, uint16_t color);
void ST7735_FillCircle       (int16_t x0, int16_t y0, int16_t r, uint16_t color);
void ST7735_FillCircleHelper (int16_t x0, int16_t y0, int16_t r, uint8_t cornername, int16_t delta, uint16_t color);
void ST7735_DrawEllipse      (int16_t x0, int16_t y0, int16_t rx, int16_t ry, uint16_t color);
void ST7735_FillEllipse      (int16_t x0, int16_t y0, int16_t rx, int16_t ry, uint16_t color);
void ST7735_DrawRect         (int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color);
void ST7735_DrawRoundRect    (int16_t x, int16_t y, int16_t w, int16_t h, int16_t r, uint16_t color);
void ST7735_FillRoundRect    (int16_t x, int16_t y, int16_t w, int16_t h, int16_t r, uint16_t color);
void ST7735_DrawTriangle     (int16_t x0, int16_t y0, int16_t x1, int16_t y1, int16_t x2, int16_t y2, uint16_t color);
void ST7735_FillTriangle     (int16_t x0, int16_t y0, int16_t x1, int16_t y1, int16_t x2, int16_t y2, uint16_t color);

uint8_t ST7735_GetRotation(void);
int16_t ST7735_GetHeight(void);
int16_t ST7735_GetWidth(void);

#endif
