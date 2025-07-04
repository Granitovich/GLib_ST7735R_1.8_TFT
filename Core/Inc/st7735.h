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



typedef enum
{
	DRIVER_ST7735_STATUS_OK = 1,
	DRIVER_ST7735_STATUS_INVALID_PARAMETERS,
	DRIVER_ST7735_STATUS_NOT_INITIALIZED,
	DRIVER_ST7735_STATUS_SEND_ERROR,
	DRIVER_ST7735_STATUS_TIMEOUT,
} driver_st7735_status;

void ST7735_Backlight_On (void);
void ST7735_Backlight_Off(void);
driver_st7735_status ST7735_Init          (SPI_HandleTypeDef *hspi);
driver_st7735_status ST7735_DrawPixel     (uint16_t x, uint16_t y, uint16_t color);
driver_st7735_status ST7735_FillRectangle (uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color);
driver_st7735_status ST7735_DrawImage     (uint16_t x, uint16_t y, uint16_t w, uint16_t h, const uint16_t* data);
driver_st7735_status ST7735_DrawFastVLine (int16_t x, int16_t y, int16_t h, uint16_t color);
driver_st7735_status ST7735_DrawFastHLine (int16_t x, int16_t y, int16_t w, uint16_t color);
driver_st7735_status ST7735_DrawLine      (int16_t x0, int16_t y0, int16_t x1, int16_t y1, uint16_t color);
driver_st7735_status ST7735_SetRotation   (uint8_t rotate);

#ifdef OTHER_FUNCTIONALITY

void ST7735_DrawString       (uint16_t x, uint16_t y, const char* str, FontDef font, uint16_t color, uint16_t bgcolor);
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

uint8_t  ST7735_GetRotation(void);
int16_t  ST7735_GetHeight  (void);
int16_t  ST7735_GetWidth   (void);
uint16_t ST7735_Color565(uint8_t r, uint8_t g, uint8_t b);

#endif
