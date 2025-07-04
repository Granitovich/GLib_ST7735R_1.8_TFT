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

#include "st7735.h"
#include "stdlib.h"
#include "assert.h"

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
    ST7735_WHITE        = 0xFFFF,

	ST7735_DELAY        = 0x80,

} driver_st7735_constants;

typedef struct
{
	SPI_HandleTypeDef *hspi;
	uint8_t data_rotation[4];
	int16_t height;
	int16_t width;
	uint8_t xstart;
	uint8_t ystart;
	uint8_t value_rotation;
} driver_st7735_config;

static driver_st7735_config cfg =
{
		.data_rotation = { ST7735_MADCTL_MX, ST7735_MADCTL_MY, ST7735_MADCTL_MV, ST7735_MADCTL_RGB },
		.height = ST7735_HEIGHT,
		.width  = ST7735_WIDTH,
		.xstart = ST7735_XSTART,
		.ystart = ST7735_YSTART

};


/* based on Adafruit ST7735 library for Arduino */
static const uint8_t
init_cmds1[] = {            		// Init for 7735R, part 1 (red or green tab)
		  15,                       // 15 commands in list:
		  ST7735_SWRESET, ST7735_DELAY,  	//  1: Software reset, 0 args, w/delay
		  150,                    	//     150 ms delay
		  ST7735_SLPOUT, ST7735_DELAY,  	//  2: Out of sleep mode, 0 args, w/delay
		  255,                    	//     500 ms delay
		  ST7735_FRMCTR1, 3,		//  3: Frame rate ctrl - normal mode, 3 args:
		  0x01, 0x2C, 0x2D,       	//     Rate = fosc/(1x2+40) * (LINE+2C+2D)
		  ST7735_FRMCTR2, 3,  		//  4: Frame rate control - idle mode, 3 args:
		  0x01, 0x2C, 0x2D,       	//     Rate = fosc/(1x2+40) * (LINE+2C+2D)
		  ST7735_FRMCTR3, 6,  		//  5: Frame rate ctrl - partial mode, 6 args:
		  0x01, 0x2C, 0x2D,       	//     Dot inversion mode
		  0x01, 0x2C, 0x2D,       	//     Line inversion mode
		  ST7735_INVCTR, 1,  		//  6: Display inversion ctrl, 1 arg, no delay:
		  0x07,                   	//     No inversion
		  ST7735_PWCTR1, 3,  		//  7: Power control, 3 args, no delay:
		  0xA2,
		  0x02,                   	//     -4.6V
		  0x84,                   	//     AUTO mode
		  ST7735_PWCTR2, 1,  		//  8: Power control, 1 arg, no delay:
		  0xC5,                   	//     VGH25 = 2.4C VGSEL = -10 VGH = 3 * AVDD
		  ST7735_PWCTR3, 2,  		//  9: Power control, 2 args, no delay:
		  0x0A,                   	//     Opamp current small
		  0x00,                   	//     Boost frequency
		  ST7735_PWCTR4, 2,  		// 10: Power control, 2 args, no delay:
		  0x8A,                   	//     BCLK/2, Opamp current small & Medium low
		  0x2A,
		  ST7735_PWCTR5, 2,  		// 11: Power control, 2 args, no delay:
		  0x8A, 0xEE,
		  ST7735_VMCTR1, 1,  		// 12: Power control, 1 arg, no delay:
		  0x0E,
		  ST7735_INVOFF, 0,  		// 13: Don't invert display, no args, no delay
		  ST7735_MADCTL, 1,  		// 14: Memory access control (directions), 1 arg:
		  ST7735_DATA_ROTATION,       //     row addr/col addr, bottom to top refresh
		  ST7735_COLMOD, 1,  		// 15: set color mode, 1 arg, no delay:
		  0x05},                 	//     16-bit color

init_cmds2[] = {            // Init for 7735R, part 2 (1.44" display)
		2,                  //  2 commands in list:
		ST7735_CASET, 4,  	//  1: Column addr set, 4 args, no delay:
		0x00, 0x00,         //     XSTART = 0
		0x00, 0x7F,         //     XEND = 127
		ST7735_RASET, 4,  	//  2: Row addr set, 4 args, no delay:
		0x00, 0x00,         //     XSTART = 0
		0x00, 0x7F },       //     XEND = 127

init_cmds3[] = {            		// Init for 7735R, part 3 (red or green tab)
		4,                        	//  4 commands in list:
		ST7735_GMCTRP1, 16, 		//  1: Magical unicorn dust, 16 args, no delay:
		0x02, 0x1c, 0x07, 0x12,
		0x37, 0x32, 0x29, 0x2d,
		0x29, 0x25, 0x2B, 0x39,
		0x00, 0x01, 0x03, 0x10,
		ST7735_GMCTRN1, 16, 		//  2: Sparkles and rainbows, 16 args, no delay:
		0x03, 0x1d, 0x07, 0x06,
		0x2E, 0x2C, 0x29, 0x2D,
		0x2E, 0x2E, 0x37, 0x3F,
		0x00, 0x00, 0x02, 0x10,
		ST7735_NORON, ST7735_DELAY, 		//  3: Normal display on, no args, w/delay
		10,                     	//     10 ms delay
		ST7735_DISPON, ST7735_DELAY, 		//  4: Main screen turn on, no args w/delay
		100 };                  	//     100 ms delay

static driver_st7735_status ST7735_WriteCommand       (uint8_t cmd);
static driver_st7735_status ST7735_WriteData          (uint8_t* buff, size_t buff_size);
static driver_st7735_status ST7735_ExecuteCommandList (const uint8_t *addr);
static driver_st7735_status ST7735_SetAddressWindow   (uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1);

static inline void swap_int16_t(int16_t* a, int16_t* b)
{
    int16_t t = *a;
    *a = *b;
    *b = t;
}

__attribute__((always_inline)) static inline void GPIO_SetPin(GPIO_TypeDef* port, uint16_t pin, bool level)
{
    if (level)
    {
        port->BSRR = pin;
    }
    else
    {
        port->BSRR = (uint32_t)pin << 16;
    }
}


#ifdef OTHER_FUNCTIONALITY

static void ST7735_WriteChar          (uint16_t x, uint16_t y, char ch, FontDef font, uint16_t color, uint16_t bgcolor);

#endif

static void ST7735_Reset()
{
	GPIO_SetPin(ST7735_RES_GPIO_Port, ST7735_RES_Pin, false);
	HAL_Delay(20);
	GPIO_SetPin(ST7735_RES_GPIO_Port, ST7735_RES_Pin, true);
}

static driver_st7735_status ST7735_WriteCommand(uint8_t cmd)
{
	GPIO_SetPin(ST7735_DC_GPIO_Port, ST7735_DC_Pin, false);

	if ( HAL_OK != HAL_SPI_Transmit_DMA(cfg.hspi, &cmd, sizeof(cmd)) )
	{
		return DRIVER_ST7735_STATUS_SEND_ERROR;
	}
	else
	{
		while(cfg.hspi->State == HAL_SPI_STATE_BUSY_TX);
		return DRIVER_ST7735_STATUS_OK;
	}

}

static driver_st7735_status ST7735_WriteData(uint8_t* buff, size_t buff_size)
{
	GPIO_SetPin(ST7735_DC_GPIO_Port, ST7735_DC_Pin, true);

	if ( HAL_OK != HAL_SPI_Transmit_DMA(cfg.hspi, buff, buff_size) )
	{
		return DRIVER_ST7735_STATUS_SEND_ERROR;
	}
	else
	{
		while(cfg.hspi->State == HAL_SPI_STATE_BUSY_TX);
		return DRIVER_ST7735_STATUS_OK;
	}
}

static driver_st7735_status ST7735_ExecuteCommandList(const uint8_t *addr)
{
    uint8_t numCommands, numArgs;
    uint16_t ms;

    numCommands = *addr++;
    while(numCommands--)
    {
    	uint8_t cmd = *addr++;

    	if ( DRIVER_ST7735_STATUS_OK != ST7735_WriteCommand(cmd))
    	{
    		return DRIVER_ST7735_STATUS_SEND_ERROR;
    	}

        numArgs = *addr++;
        /* If high bit set, delay follows args */
        ms = numArgs & ST7735_DELAY;
        numArgs &= ~ST7735_DELAY;
        if(numArgs)
        {
            if (DRIVER_ST7735_STATUS_OK != ST7735_WriteData((uint8_t*)addr, numArgs))
            {
            	return DRIVER_ST7735_STATUS_SEND_ERROR;
            }
            addr += numArgs;
        }
        if(ms)
        {
            ms = *addr++;
            if(ms == 255) ms = 500;
            HAL_Delay(ms);
        }
    }
    return DRIVER_ST7735_STATUS_OK;
}

static driver_st7735_status ST7735_SetAddressWindow(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1)
{
    /* column address set */
	if ( DRIVER_ST7735_STATUS_OK != ST7735_WriteCommand(ST7735_CASET))
	{
		return DRIVER_ST7735_STATUS_SEND_ERROR;
	}
    uint8_t data[] = { 0x00, x0 + cfg.xstart, 0x00, x1 + cfg.xstart };

    if ( DRIVER_ST7735_STATUS_OK != ST7735_WriteData(data, sizeof(data)) )
    {
    	return DRIVER_ST7735_STATUS_SEND_ERROR;
    }
    /* row address set */
    if ( DRIVER_ST7735_STATUS_OK != ST7735_WriteCommand(ST7735_RASET))
    {
    	return DRIVER_ST7735_STATUS_SEND_ERROR;
    }
    data[1] = y0 + cfg.ystart;
    data[3] = y1 + cfg.ystart;

    if ( DRIVER_ST7735_STATUS_OK != ST7735_WriteData(data, sizeof(data)) )
    {
    	return DRIVER_ST7735_STATUS_SEND_ERROR;
    }
    /* write to RAM */
    if ( DRIVER_ST7735_STATUS_OK != ST7735_WriteCommand(ST7735_RAMWR))
    {
    	return DRIVER_ST7735_STATUS_SEND_ERROR;
    }

    return DRIVER_ST7735_STATUS_OK;
}


void ST7735_Backlight_On(void)
{
	GPIO_SetPin(ST7735_BL_GPIO_Port, ST7735_BL_Pin, true);
}

void ST7735_Backlight_Off(void)
{
	GPIO_SetPin(ST7735_BL_GPIO_Port, ST7735_BL_Pin, false);
}

driver_st7735_status ST7735_Init(SPI_HandleTypeDef *hspi)
{
	assert(hspi != NULL);
	cfg.hspi = hspi;

	GPIO_SetPin(ST7735_CS_GPIO_Port, ST7735_CS_Pin, false);
    ST7735_Reset();

    if( DRIVER_ST7735_STATUS_OK != ST7735_ExecuteCommandList (init_cmds1)  ||
    	DRIVER_ST7735_STATUS_OK != ST7735_ExecuteCommandList (init_cmds2)  ||
		DRIVER_ST7735_STATUS_OK != ST7735_ExecuteCommandList (init_cmds3))
    {
    	return DRIVER_ST7735_STATUS_SEND_ERROR;
    }

    GPIO_SetPin(ST7735_CS_GPIO_Port, ST7735_CS_Pin, true);

    return DRIVER_ST7735_STATUS_OK;
}

driver_st7735_status ST7735_DrawPixel(uint16_t x, uint16_t y, uint16_t color)
{
    if ( (x >= cfg.width) || (y >= cfg.height) )
    {
    	return DRIVER_ST7735_STATUS_INVALID_PARAMETERS;
    }

    GPIO_SetPin(ST7735_CS_GPIO_Port, ST7735_CS_Pin, false);

    if( DRIVER_ST7735_STATUS_OK != ST7735_SetAddressWindow(x, y, x+1, y+1))
    {
    	return DRIVER_ST7735_STATUS_SEND_ERROR;
    }
    uint8_t data[] = { color >> 8, color & 0xFF };

    if( DRIVER_ST7735_STATUS_OK != ST7735_WriteData(data, sizeof(data)) )
    {
    	return DRIVER_ST7735_STATUS_SEND_ERROR;
    }

    GPIO_SetPin(ST7735_CS_GPIO_Port, ST7735_CS_Pin, true);

    return DRIVER_ST7735_STATUS_OK;
}

driver_st7735_status ST7735_FillRectangle(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color)
{
    // clipping
    if ( (x >= cfg.width) || (y >= cfg.height) )
    {
    	return DRIVER_ST7735_STATUS_INVALID_PARAMETERS;
    }

    if ((x + w - 1) >= cfg.width)
    {
    	w = cfg.width - x;
    }
    if ((y + h - 1) >= cfg.height)
    {
    	 h = cfg.height - y;
    }

    GPIO_SetPin(ST7735_CS_GPIO_Port, ST7735_CS_Pin, false);
    if( DRIVER_ST7735_STATUS_OK != ST7735_SetAddressWindow(x, y, x + w - 1, y + h - 1) )
    {
    	return DRIVER_ST7735_STATUS_OK;
    }

    const uint8_t data[2] = { color >> 8, color & 0xFF };
    GPIO_SetPin(ST7735_DC_GPIO_Port, ST7735_DC_Pin, true);

    uint8_t tbuf[w*2];
    for (y = h; y > 0; y--)
    {
        for (int x = w * 2; x >= 0; x -= 2)
        {
            tbuf[x] = data[0];
            tbuf[x + 1] = data[1];
        }
        if ( HAL_OK != HAL_SPI_Transmit_DMA(cfg.hspi, tbuf, sizeof(tbuf)) )
        {
        	return DRIVER_ST7735_STATUS_SEND_ERROR;
        }
        else
        {
        while (cfg.hspi->State == HAL_SPI_STATE_BUSY_TX) {};
        }
    }

    GPIO_SetPin(ST7735_CS_GPIO_Port, ST7735_CS_Pin, true);

    return DRIVER_ST7735_STATUS_OK;
}

driver_st7735_status ST7735_DrawImage(uint16_t x, uint16_t y, uint16_t w, uint16_t h, const uint16_t* data)
{
    if ( ((x >= cfg.width) || (y >= cfg.height)) || ( (x + w - 1) >= cfg.width ) || ( (y + h - 1) >= cfg.height ) )
    {
    	return DRIVER_ST7735_STATUS_INVALID_PARAMETERS;
    }

    GPIO_SetPin(ST7735_CS_GPIO_Port, ST7735_CS_Pin, false);

    if( DRIVER_ST7735_STATUS_OK != ST7735_SetAddressWindow(x, y, x+w-1, y+h-1)  ||
        DRIVER_ST7735_STATUS_OK != ST7735_WriteData((uint8_t*)data, sizeof(uint16_t)*w*h) )
    {
    	return DRIVER_ST7735_STATUS_SEND_ERROR;
    }

    GPIO_SetPin(ST7735_CS_GPIO_Port, ST7735_CS_Pin, true);

    return DRIVER_ST7735_STATUS_OK;
}

driver_st7735_status ST7735_DrawLine(int16_t x0, int16_t y0, int16_t x1, int16_t y1, uint16_t color)
{
	int16_t steep = abs(y1 - y0) > abs(x1 - x0);
	if (steep)
	{
		swap_int16_t(&x0, &y0);
		swap_int16_t(&x1, &y1);
	}

	if (x0 > x1)
	{
		swap_int16_t(&x0, &x1);
		swap_int16_t(&y0, &y1);
	}

	int16_t dx, dy;
	dx = x1 - x0;
	dy = abs(y1 - y0);

	int16_t err = dx / 2;
	int16_t ystep;

	if (y0 < y1)
	{
		ystep = 1;
	}
	else
	{
		ystep = -1;
	}

	for (; x0<=x1; x0++)
	{
		if (steep)
		{
			if( DRIVER_ST7735_STATUS_OK != ST7735_DrawPixel(y0, x0, color))
			{
				return DRIVER_ST7735_STATUS_SEND_ERROR;
			}
		}
		else
		{
			if( DRIVER_ST7735_STATUS_OK != ST7735_DrawPixel(x0, y0, color))
			{
				return DRIVER_ST7735_STATUS_SEND_ERROR;
			}
		}
		err -= dy;
		if (err < 0)
		{
			y0 += ystep;
			err += dx;
		}
	}
	return DRIVER_ST7735_STATUS_OK;
}

driver_st7735_status ST7735_DrawFastVLine(int16_t x, int16_t y, int16_t h, uint16_t color)
{

  if ((x >= cfg.width) || (y >= cfg.height))
  {
	  return DRIVER_ST7735_STATUS_INVALID_PARAMETERS;
  }
  if ((y + h - 1) >= cfg.height)
  {
	  h = cfg.height - y;
  }

  if( DRIVER_ST7735_STATUS_OK != ST7735_FillRectangle(x, y, 1, h, color))
  {
	  return DRIVER_ST7735_STATUS_SEND_ERROR;
  }


  return DRIVER_ST7735_STATUS_OK;
}

driver_st7735_status ST7735_DrawFastHLine(int16_t x, int16_t y, int16_t w, uint16_t color)
{

  if ((x >= cfg.width) || (y >= cfg.height))
  {
	  return DRIVER_ST7735_STATUS_INVALID_PARAMETERS;
  }
  if ((x + w - 1) >= cfg.width)
  {
	  w = cfg.width - x;
  }
  if( DRIVER_ST7735_STATUS_OK != ST7735_FillRectangle(x, y, w, 1, color))
  {
	  return DRIVER_ST7735_STATUS_SEND_ERROR;
  }

  return DRIVER_ST7735_STATUS_OK;
}

driver_st7735_status ST7735_SetRotation(uint8_t m)
{
  cfg.value_rotation = m % 4;

  GPIO_SetPin(ST7735_CS_GPIO_Port, ST7735_CS_Pin, false);

  if( DRIVER_ST7735_STATUS_OK != ST7735_WriteCommand(ST7735_MADCTL))
  {
	  return DRIVER_ST7735_STATUS_SEND_ERROR;
  }

  switch (cfg.value_rotation)
  {
    case 0:
    {
    	uint8_t d_r = (cfg.data_rotation[0] | cfg.data_rotation[1] | cfg.data_rotation[3]);
    	if( DRIVER_ST7735_STATUS_OK != ST7735_WriteData(&d_r, sizeof(d_r)))
    	{
    		return DRIVER_ST7735_STATUS_SEND_ERROR;
    	}
    	cfg.width  = ST7735_WIDTH;
    	cfg.height = ST7735_HEIGHT;
    	cfg.xstart = ST7735_XSTART;
    	cfg.ystart = ST7735_YSTART;
    }
     break;
    case 1:
    {
    	uint8_t d_r = (cfg.data_rotation[1] | cfg.data_rotation[2] | cfg.data_rotation[3]);
    	if( DRIVER_ST7735_STATUS_OK != ST7735_WriteData(&d_r, sizeof(d_r)))
    	{
    		return DRIVER_ST7735_STATUS_SEND_ERROR;
    	}
    	cfg.width  = ST7735_HEIGHT;
    	cfg.height = ST7735_WIDTH;
    	cfg.xstart = ST7735_YSTART;
    }
      break;
    case 2:
    {
    	uint8_t d_r = cfg.data_rotation[3];
    	if( DRIVER_ST7735_STATUS_OK != ST7735_WriteData(&d_r, sizeof(d_r)))
    	{
    		return DRIVER_ST7735_STATUS_SEND_ERROR;
    	}
    	cfg.width  = ST7735_WIDTH;
    	cfg.height = ST7735_HEIGHT;
    	cfg.xstart = ST7735_XSTART;
    	cfg.ystart = ST7735_YSTART;
    }
      break;
    case 3:
    {
    	uint8_t d_r = (cfg.data_rotation[0] | cfg.data_rotation[2] | cfg.data_rotation[3]);
    	if( DRIVER_ST7735_STATUS_OK != ST7735_WriteData(&d_r, sizeof(d_r)))
    	{
    		return DRIVER_ST7735_STATUS_SEND_ERROR;
    	}
    	cfg.width  = ST7735_HEIGHT;
    	cfg.height = ST7735_WIDTH;
    	cfg.xstart = ST7735_YSTART;
    	cfg.ystart = ST7735_XSTART;
    }
      break;
  }
  GPIO_SetPin(ST7735_CS_GPIO_Port, ST7735_CS_Pin, true);

  return DRIVER_ST7735_STATUS_OK;
}

#ifdef OTHER_FUNCTIONALITY

void ST7735_DrawString(uint16_t x, uint16_t y, const char* str, FontDef font, uint16_t color, uint16_t bgcolor)
{
	GPIO_SetPin(ST7735_CS_GPIO_Port, ST7735_CS_Pin, false);

    while(*str)
    {
        if(x + font.width >= cfg.width)
        {
            x = 0;
            y += font.height;
            if(y + font.height >= cfg.height)
            {
                break;
            }

            if(*str == ' ')
            {
                /* skip spaces in the beginning of the new line */
                str++;
                continue;
            }
        }

        ST7735_WriteChar(x, y, *str, font, color, bgcolor);
        x += font.width;
        str++;
    }
    GPIO_SetPin(ST7735_CS_GPIO_Port, ST7735_CS_Pin, true);
}

void ST7735_FillScreen(uint16_t color)
{
    ST7735_FillRectangle(0, 0, cfg.width, cfg.height, color);
}



void ST7735_InvertColors(bool invert)
{
	GPIO_SetPin(ST7735_CS_GPIO_Port, ST7735_CS_Pin, false);
    ST7735_WriteCommand(invert ? ST7735_INVON : ST7735_INVOFF);
    GPIO_SetPin(ST7735_CS_GPIO_Port, ST7735_CS_Pin, true);
}

/***************************************************************************************
** Function name:           drawCircle
** Description:             Draw a circle outline
***************************************************************************************/
void ST7735_DrawCircle(int16_t x0, int16_t y0, int16_t r, uint16_t color)
{
  int16_t f = 1 - r;
  int16_t ddF_x = 1;
  int16_t ddF_y = - r - r;
  int16_t x = 0;

  ST7735_DrawPixel(x0 + r, y0  , color);
  ST7735_DrawPixel(x0 - r, y0  , color);
  ST7735_DrawPixel(x0  , y0 - r, color);
  ST7735_DrawPixel(x0  , y0 + r, color);

  while (x < r)
  {
    if (f >= 0)
    {
      r--;
      ddF_y += 2;
      f += ddF_y;
    }
    x++;
    ddF_x += 2;
    f += ddF_x;

    ST7735_DrawPixel(x0 + x, y0 + r, color);
    ST7735_DrawPixel(x0 - x, y0 + r, color);
    ST7735_DrawPixel(x0 - x, y0 - r, color);
    ST7735_DrawPixel(x0 + x, y0 - r, color);

    ST7735_DrawPixel(x0 + r, y0 + x, color);
    ST7735_DrawPixel(x0 - r, y0 + x, color);
    ST7735_DrawPixel(x0 - r, y0 - x, color);
    ST7735_DrawPixel(x0 + r, y0 - x, color);
  }
}

/***************************************************************************************
** Function name:           drawCircleHelper
** Description:             Support function for circle drawing
***************************************************************************************/
void ST7735_DrawCircleHelper( int16_t x0, int16_t y0, int16_t r, uint8_t cornername, uint16_t color)
{
  int16_t f     = 1 - r;
  int16_t ddF_x = 1;
  int16_t ddF_y = -2 * r;
  int16_t x     = 0;

  while (x < r)
  {
    if (f >= 0)
    {
      r--;
      ddF_y += 2;
      f     += ddF_y;
    }
    x++;
    ddF_x += 2;
    f     += ddF_x;
    if (cornername & 0x8)
    {
      ST7735_DrawPixel(x0 - r, y0 + x, color);
      ST7735_DrawPixel(x0 - x, y0 + r, color);
    }
    if (cornername & 0x4)
    {
    	ST7735_DrawPixel(x0 + x, y0 + r, color);
    	ST7735_DrawPixel(x0 + r, y0 + x, color);
    }
    if (cornername & 0x2)
    {
    	ST7735_DrawPixel(x0 + r, y0 - x, color);
    	ST7735_DrawPixel(x0 + x, y0 - r, color);
    }
    if (cornername & 0x1)
    {
    	ST7735_DrawPixel(x0 - x, y0 - r, color);
    	ST7735_DrawPixel(x0 - r, y0 - x, color);
    }

  }
}

/***************************************************************************************
** Function name:           fillCircle
** Description:             draw a filled circle
***************************************************************************************/
void ST7735_FillCircle(int16_t x0, int16_t y0, int16_t r, uint16_t color)
{
	ST7735_DrawFastVLine   (x0, y0 - r, r + r + 1, color);
	ST7735_FillCircleHelper(x0, y0, r, 3, 0, color);
}

/***************************************************************************************
** Function name:           fillCircleHelper
** Description:             Support function for filled circle drawing
***************************************************************************************/
// Used to do circles and roundrects
void ST7735_FillCircleHelper(int16_t x0, int16_t y0, int16_t r, uint8_t cornername, int16_t delta, uint16_t color)
{
  int16_t f     = 1 - r;
  int16_t ddF_x = 1;
  int16_t ddF_y = -r - r;
  int16_t x     = 0;

  delta++;
  while (x < r)
  {
    if (f >= 0)
    {
      r--;
      ddF_y += 2;
      f     += ddF_y;
    }
    x++;
    ddF_x += 2;
    f     += ddF_x;

    if (cornername & 0x1)
    {
    	ST7735_DrawFastVLine(x0 + x, y0 - r, r + r + delta, color);
    	ST7735_DrawFastVLine(x0 + r, y0 - x, x + x + delta, color);
    }
    if (cornername & 0x2)
    {
    	ST7735_DrawFastVLine(x0 - x, y0 - r, r + r + delta, color);
    	ST7735_DrawFastVLine(x0 - r, y0 - x, x + x + delta, color);
    }
  }
}

/***************************************************************************************
** Function name:           drawEllipse
** Description:             Draw a ellipse outline
***************************************************************************************/
void ST7735_DrawEllipse(int16_t x0, int16_t y0, int16_t rx, int16_t ry, uint16_t color)
{
  if (rx < 2) return;
  if (ry < 2) return;
  int16_t x, y;
  int32_t rx2 = rx * rx;
  int32_t ry2 = ry * ry;
  int32_t fx2 = 4 * rx2;
  int32_t fy2 = 4 * ry2;
  int32_t s;

  for (x = 0, y = ry, s = 2 * ry2 + rx2 * (1-2 * ry); ry2 * x <= rx2 * y; x++)
  {
	  ST7735_DrawPixel(x0 + x, y0 + y, color);
	  ST7735_DrawPixel(x0 - x, y0 + y, color);
	  ST7735_DrawPixel(x0 - x, y0 - y, color);
	  ST7735_DrawPixel(x0 + x, y0 - y, color);
    if (s >= 0)
    {
      s += fx2 * (1 - y);
      y--;
    }
    s += ry2 * ((4 * x) + 6);
  }

  for (x = rx, y = 0, s = 2 * rx2 + ry2 * (1 - 2 * rx); rx2 * y <= ry2 * x; y++)
  {
	  ST7735_DrawPixel(x0 + x, y0 + y, color);
	  ST7735_DrawPixel(x0 - x, y0 + y, color);
	  ST7735_DrawPixel(x0 - x, y0 - y, color);
	  ST7735_DrawPixel(x0 + x, y0 - y, color);
	if (s >= 0)
	{
	  s += fy2 * (1 - x);
	  x--;
	}
	s += rx2 * ((4 * y) + 6);
  }
}

/***************************************************************************************
** Function name:           fillEllipse
** Description:             draw a filled ellipse
***************************************************************************************/
void ST7735_FillEllipse(int16_t x0, int16_t y0, int16_t rx, int16_t ry, uint16_t color)
{
  if (rx < 2) return;
  if (ry < 2) return;
  int16_t x, y;
  int32_t rx2 = rx * rx;
  int32_t ry2 = ry * ry;
  int32_t fx2 = 4 * rx2;
  int32_t fy2 = 4 * ry2;
  int32_t s;

  for (x = 0, y = ry, s = 2 * ry2 + rx2 * (1 - 2 * ry); ry2 * x <= rx2 * y; x++)
  {
    ST7735_DrawFastHLine(x0 - x, y0 - y, x + x + 1, color);
    ST7735_DrawFastHLine(x0 - x, y0 + y, x + x + 1, color);

    if (s >= 0)
    {
      s += fx2 * (1 - y);
      y--;
    }
    s += ry2 * ((4 * x) + 6);
  }

  for (x = rx, y = 0, s = 2 * rx2 + ry2 * (1 - 2 * rx); rx2 * y <= ry2 * x; y++)
  {
    ST7735_DrawFastHLine(x0 - x, y0 - y, x + x + 1, color);
    ST7735_DrawFastHLine(x0 - x, y0 + y, x + x + 1, color);

    if (s >= 0)
    {
      s += fy2 * (1 - x);
      x--;
    }
    s += rx2 * ((4 * y) + 6);
  }

}

/***************************************************************************************
** Function name:           drawRect
** Description:             Draw a rectangle outline
***************************************************************************************/
// Draw a rectangle
void ST7735_DrawRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color)
{
  ST7735_DrawFastHLine(x, y, w, color);
  ST7735_DrawFastHLine(x, y + h - 1, w, color);
  ST7735_DrawFastVLine(x, y, h, color);
  ST7735_DrawFastVLine(x + w - 1, y, h, color);
}

/***************************************************************************************
** Function name:           drawRoundRect
** Description:             Draw a rounded corner rectangle outline
***************************************************************************************/
// Draw a rounded rectangle
void ST7735_DrawRoundRect(int16_t x, int16_t y, int16_t w, int16_t h, int16_t r, uint16_t color)
{
  // smarter version
  ST7735_DrawFastHLine(x + r  , y    , w - r - r, color); // Top
  ST7735_DrawFastHLine(x + r  , y + h - 1, w - r - r, color); // Bottom
  ST7735_DrawFastVLine(x    , y + r  , h - r - r, color); // Left
  ST7735_DrawFastVLine(x + w - 1, y + r  , h - r - r, color); // Right
  // draw four corners
  ST7735_DrawCircleHelper(x + r    , y + r    , r, 1, color);
  ST7735_DrawCircleHelper(x + r    , y + h - r - 1, r, 8, color);
  ST7735_DrawCircleHelper(x + w - r - 1, y + r    , r, 2, color);
  ST7735_DrawCircleHelper(x + w - r - 1, y + h - r - 1, r, 4, color);
}

/***************************************************************************************
** Function name:           fillRoundRect
** Description:             Draw a rounded corner filled rectangle
***************************************************************************************/
// Fill a rounded rectangle
void ST7735_FillRoundRect(int16_t x, int16_t y, int16_t w, int16_t h, int16_t r, uint16_t color)
{
  // smarter version
  ST7735_FillRectangle(x + r, y, w - r - r, h, color);

  // draw four corners
  ST7735_FillCircleHelper(x + w - r - 1, y + r, r, 1, h - r - r - 1, color);
  ST7735_FillCircleHelper(x + r    , y + r, r, 2, h - r - r - 1, color);
}

/***************************************************************************************
** Function name:           drawTriangle
** Description:             Draw a triangle outline using 3 arbitrary points
***************************************************************************************/
// Draw a triangle
void ST7735_DrawTriangle(int16_t x0, int16_t y0, int16_t x1, int16_t y1, int16_t x2, int16_t y2, uint16_t color)
{
  ST7735_DrawLine(x0, y0, x1, y1, color);
  ST7735_DrawLine(x1, y1, x2, y2, color);
  ST7735_DrawLine(x2, y2, x0, y0, color);
}

/***************************************************************************************
** Function name:           fillTriangle
** Description:             Draw a filled triangle using 3 arbitrary points
***************************************************************************************/
// Fill a triangle - original Adafruit function works well and code footprint is small
void ST7735_FillTriangle( int16_t x0, int16_t y0, int16_t x1, int16_t y1, int16_t x2, int16_t y2, uint16_t color)
{
  int16_t a, b, y, last;

  // Sort coordinates by Y order (y2 >= y1 >= y0)
  if (y0 > y1)
  {
	  swap_int16_t(&y0, &y1); swap_int16_t(&x0, &x1);
  }

  if (y1 > y2)
  {
	  swap_int16_t(&y2, &y1); swap_int16_t(&x2, &x1);
  }

  if (y0 > y1)
  {
	  swap_int16_t(&y0, &y1); swap_int16_t(&x0, &x1);
  }

  if (y0 == y2)
  { // Handle awkward all-on-same-line case as its own thing
    a = b = x0;
    if (x1 < a)      a = x1;
    else if (x1 > b) b = x1;
    if (x2 < a)      a = x2;
    else if (x2 > b) b = x2;
    ST7735_DrawFastHLine(a, y0, b - a + 1, color);
    return;
  }

  int16_t
  dx01 = x1 - x0,
  dy01 = y1 - y0,
  dx02 = x2 - x0,
  dy02 = y2 - y0,
  dx12 = x2 - x1,
  dy12 = y2 - y1,
  sa   = 0,
  sb   = 0;

/*
   For upper part of triangle, find scanline crossings for segments
   0-1 and 0-2.  If y1=y2 (flat-bottomed triangle), the scanline y1
   is included here (and second loop will be skipped, avoiding a /0
   error there), otherwise scanline y1 is skipped here and handled
   in the second loop...which also avoids a /0 error here if y0=y1
   (flat-topped triangle).
*/

  if (y1 == y2) last = y1;  // Include y1 scanline
  else         last = y1 - 1; // Skip it

  for (y = y0; y <= last; y++)
  {
    a   = x0 + sa / dy01;
    b   = x0 + sb / dy02;
    sa += dx01;
    sb += dx02;

    if (a > b) swap_int16_t(&a, &b);
    ST7735_DrawFastHLine(a, y, b - a + 1, color);
  }
/*
   For lower part of triangle, find scanline crossings for segments
   0-2 and 1-2.  This loop is skipped if y1=y2.
*/
  sa = dx12 * (y - y1);
  sb = dx02 * (y - y0);
  for (; y <= y2; y++)
  {
    a   = x1 + sa / dy12;
    b   = x0 + sb / dy02;
    sa += dx12;
    sb += dx02;

    if (a > b) swap_int16_t(&a, &b);
    ST7735_DrawFastHLine(a, y, b - a + 1, color);
  }
}

uint8_t ST7735_GetRotation(void)
{
  return cfg.value_rotation;
}

int16_t ST7735_GetHeight(void)
{
	return cfg.height;
}

int16_t ST7735_GetWidth(void)
{
	return cfg.width;
}

static void ST7735_WriteChar(uint16_t x, uint16_t y, char ch, FontDef font, uint16_t color, uint16_t bgcolor)
{
    uint32_t i, b, j;

    ST7735_SetAddressWindow(x, y, x+font.width-1, y+font.height-1);

    for(i = 0; i < font.height; i++)
    {
        b = font.data[(ch - 32) * font.height + i];
        for(j = 0; j < font.width; j++)
        {
            if((b << j) & 0x8000)
            {
                uint8_t data[] = { color >> 8, color & 0xFF };
                ST7735_WriteData(data, sizeof(data));
            }
            else
            {
                uint8_t data[] = { bgcolor >> 8, bgcolor & 0xFF };
                ST7735_WriteData(data, sizeof(data));
            }
        }
    }
}

inline uint16_t ST7735_Color565(uint8_t r, uint8_t g, uint8_t b)
{
    return ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | ((b & 0xF8) >> 3);
}

#endif
