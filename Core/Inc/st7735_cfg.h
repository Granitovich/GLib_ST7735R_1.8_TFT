/*
 * st7735_cfg.h
 *
 *  Created on: 16 ���. 2019 �.
 *      Author: Andriy Honcharenko
 *        Blog: https://stm32withoutfear.blogspot.com
 */

#ifndef ST7735_CFG_H_
#define ST7735_CFG_H_

#include "main.h"

#define ST7735_SPI_PORT hspi1	//hspi1, hspi2, hspi3...
#define USE_SPI_DMA			//if used DMA for SPI bus

//Port and pin connected signal 'RES' (reset) ST7735 display
#ifndef ST7735_RES_Pin
#define ST7735_RES_Pin 			GPIO_PIN_12
#endif
#ifndef ST7735_RES_GPIO_Port
#define ST7735_RES_GPIO_Port 	GPIOB
#endif
//Port and pin connected signal 'DC' (data or command) ST7735 display
#ifndef ST7735_DC_Pin
#define ST7735_DC_Pin 			GPIO_PIN_13
#endif
#ifndef ST7735_DC_GPIO_Port
#define ST7735_DC_GPIO_Port 	GPIOB
#endif
//Port and pin connected signal 'CS' (chip select) ST7735 display
#ifndef ST7735_CS_Pin
#define ST7735_CS_Pin 			GPIO_PIN_14
#endif
#ifndef ST7735_CS_GPIO_Port
#define ST7735_CS_GPIO_Port 	GPIOB
#endif
//Port and pin connected signal 'BL' (back light) ST7735 display
#ifndef ST7735_BL_Pin
#define ST7735_BL_Pin			GPIO_PIN_15
#endif
#ifndef ST7735_BL_GPIO_Port
#define ST7735_BL_GPIO_Port 	GPIOB
#endif

#endif /* ST7735_CFG_H_ */
