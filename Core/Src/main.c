/* USER CODE BEGIN Header */
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
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "fatfs.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "st7735.h"
#include "fonts.h"
#include "image.h"

#include <stdio.h>
#include <string.h>
#include <stdarg.h> //for va_list var arg functions
#include "user_diskio.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
void myprintf(const char *fmt, ...);
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
uint8_t r = 3;
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;
DMA_HandleTypeDef hdma_spi1_tx;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
uint16_t image_buffer[160 * 128];

uint16_t (*image_2d)[128] = (uint16_t(*)[128])image_buffer;

#define FRAME_SIZE (160*128*2)  // 40960 байт

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_SPI1_Init(void);
static void MX_SPI2_Init(void);
/* USER CODE BEGIN PFP */
// Глобальний об'єкт файлової системи
static FATFS fs;
// Файловий дескриптор
static FIL fil;

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void myprintf(const char *fmt, ...) {
  static char buffer[256];
  va_list args;
  va_start(args, fmt);
  vsnprintf(buffer, sizeof(buffer), fmt, args);
  va_end(args);

  int len = strlen(buffer);
  HAL_UART_Transmit(&huart2, (uint8_t*)buffer, len, -1);

}

void demoTFT(void)
{
 ST7735_SetRotation(r);

 //ST7735_DrawImage(0, 0, 160, 128, (uint16_t*) test_img_128x128);
 //ST7735_DrawImage(0, 0, 160, 128, (uint16_t*) test_img_128x128_2);
// HAL_Delay(3000);
// ST7735_DrawImage(0, 0, 160, 128, (uint16_t*) test_img_128x128_3);
 HAL_Delay(3000);
 //ST7735_DrawImage(0, 0, 160, 128, (uint16_t*) test_img_128x128_4);
 HAL_Delay(3000);

// r++;
}

void copy_1d_to_2d(const uint16_t* one_d, uint16_t two_d[160][128])
{
    for (int y = 0; y < 160; y++)
    {
        for (int x = 0; x < 128; x++)
        {
            two_d[y][x] = one_d[y * 128 + x];
        }
    }
}
void parse_boot_sector(void)
{
    BYTE sector_buf[512];
    DRESULT dres = disk_read(0, sector_buf, 0, 1); // Читаємо сектор 0

    if (dres != RES_OK) {
        myprintf("Can't read boot sector\r\n");
        return;
    }

    // BPB розпочинається з байта 11
    uint16_t bytes_per_sector = sector_buf[11] | (sector_buf[12] << 8);
    uint8_t sectors_per_cluster = sector_buf[13];
    uint16_t reserved_sectors = sector_buf[14] | (sector_buf[15] << 8);
    uint8_t num_fats = sector_buf[16];
    uint16_t root_entries = sector_buf[17] | (sector_buf[18] << 8);
    uint16_t total_sectors_16 = sector_buf[19] | (sector_buf[20] << 8);
    uint32_t total_sectors_32 = *(uint32_t*)&sector_buf[32];
    uint16_t sectors_per_fat_16 = sector_buf[22] | (sector_buf[23] << 8);
    uint32_t sectors_per_fat_32 = *(uint32_t*)&sector_buf[36];

    myprintf("FAT Info:\r\n");
    myprintf("Bytes per sector:      %u\r\n", bytes_per_sector);
    myprintf("Sectors per cluster:   %u\r\n", sectors_per_cluster);
    myprintf("Reserved sectors:      %u\r\n", reserved_sectors);
    myprintf("Number of FATs:        %u\r\n", num_fats);
    myprintf("Root dir entries:      %u\r\n", root_entries);
    myprintf("Total sectors (16-bit):%u\r\n", total_sectors_16);
    myprintf("Total sectors (32-bit):%lu\r\n", total_sectors_32);
    myprintf("Sectors per FAT (16):  %u\r\n", sectors_per_fat_16);
    myprintf("Sectors per FAT (32):  %lu\r\n", sectors_per_fat_32);

    // Вивести метку тому (Volume Label)
    myprintf("Volume label: %.11s\r\n", &sector_buf[43]);

    // Вивести тип FAT
    myprintf("FAT type string: %.8s\r\n", &sector_buf[54]); // Наприклад, "FAT16   "
}

FRESULT fatfs_read_buff(uint16_t* buff, char* name_of_file )
{
    FRESULT res;
    UINT br;

#define IMAGE_SIZE 20480 // кількість елементів uint16_t (не байтів!)

 res = f_open(&fil, name_of_file, FA_READ);
if (res != FR_OK)
{
    myprintf("f_open failed: %d\r\n", res);
    return res;
}

res = f_read(&fil, buff, IMAGE_SIZE * sizeof(uint16_t), &br);

if (res != FR_OK || br != IMAGE_SIZE * sizeof(uint16_t))
{
    myprintf("f_read failed: %d, read %u bytes\r\n", res, br);
    return res;
}

myprintf("Successfully read %u bytes from file\r\n", br);

    // 7. Закриваємо файл після читання
    f_close(&fil);

    return res;

}
FRESULT fatfs_init()
{
	FRESULT res;
	 // 1. Монтуємо файлову систему
	    res = f_mount(&fs, "", 1);
	    if (res != FR_OK )
	    {
	        myprintf("f_mount failed: %d\r\n", res);
	        return res;
	    }
	    myprintf("File system mounted\r\n");
	    return res;
}
FRESULT fatfs_write(const uint16_t* buff, char* name_of_file )
{
    FRESULT res;
    UINT bw;

    // 2. Створюємо або відкриваємо файл для запису
    res = f_open(&fil, name_of_file, FA_WRITE | FA_CREATE_ALWAYS);
    if (res != FR_OK)
    {
        myprintf("f_open write failed: %d\r\n", res);
        return res;
    }
    myprintf("File opened for writing\r\n");

    // 3. Записуємо буфер у файл

    const uint32_t buf_size = 40960; // Кількість байтів

    res = f_write(&fil, buff, buf_size, &bw);
    if (res != FR_OK || bw != buf_size)
    {
        myprintf("f_write failed: %d\r\n", res);
        f_close(&fil);
        return res;
    }
    HAL_Delay(1000);
    myprintf("Wrote %lu bytes to file\r\n", bw);

    // 4. Закриваємо файл після запису
    f_close(&fil);

    return res;

}

void test_lowlevel_sd(void) {
    DSTATUS status;
    DRESULT res;
    BYTE buffer[512];
    BYTE write_data[512];
    UINT i;

    myprintf("\r\n--- SD Low Level Test Start ---\r\n");

    // 1. Ініціалізація диску
    status = disk_initialize(0);
    if (status != 0) {
        myprintf("disk_initialize failed with status: %d\r\n", status);
        return;
    }
    myprintf("disk_initialize OK\r\n");

    // 2. Перевірка статусу
    status = disk_status(0);
    if (status != 0) {
        myprintf("disk_status reports NOT READY (status=%d)\r\n", status);
        return;
    }
    myprintf("disk_status OK\r\n");

    // 3. Читання 0-го сектора
    memset(buffer, 0, sizeof(buffer));
    res = disk_read(0, buffer, 0, 1);
    if (res != RES_OK) {
        myprintf("disk_read sector 0 failed with result: %d\r\n", res);
        return;
    }
    myprintf("disk_read sector 0 OK, first 16 bytes:\r\n");
    for(i = 0; i < 16; i++) {
        myprintf("%02X ", buffer[i]);
    }
    myprintf("\r\n");

    // 4. Підготовка даних для запису в 1-й сектор (наприклад, запишемо інкрементний патерн)
    for(i = 0; i < 512; i++) {
        write_data[i] = (BYTE)(i & 0xFF);
    }

    // 5. Запис у 1-й сектор
    res = disk_write(0, write_data, 1, 1);
    if (res != RES_OK) {
        myprintf("disk_write sector 1 failed with result: %d\r\n", res);
        return;
    }
    myprintf("disk_write sector 1 OK\r\n");

    // 6. Читання назад 1-го сектора і перевірка
    memset(buffer, 0, sizeof(buffer));
    res = disk_read(0, buffer, 1, 1);
    if (res != RES_OK) {
        myprintf("disk_read sector 1 failed with result: %d\r\n", res);
        return;
    }
    myprintf("disk_read sector 1 OK, first 16 bytes:\r\n");
    for(i = 0; i < 16; i++) {
        myprintf("%02X ", buffer[i]);
    }
    myprintf("\r\n");

    // 7. Перевірка відповідності записаних та прочитаних даних
    if (memcmp(write_data, buffer, 512) == 0) {
        myprintf("Read/Write test PASSED!\r\n");
    } else {
        myprintf("Read/Write test FAILED!\r\n");
    }

    myprintf("--- SD Low Level Test End ---\r\n");
}

#define FILES_COUNT 976
#define MAX_FILENAME_LEN 20

int fill_file_names(char* files[], int count) {
    char buff[MAX_FILENAME_LEN];
    for (int i = 0; i < count; i++) {
        // Формуємо ім'я файлу
        int len = snprintf(buff, MAX_FILENAME_LEN, "i%d.txt", i);
        if (len < 0 || len >= MAX_FILENAME_LEN) {
            // Помилка форматування або довжина більша за буфер
            return -1;
        }
        files[i] = malloc(len + 1); // +1 для '\0'
        if (files[i] == NULL) {
            // Помилка виділення пам'яті — звільняємо вже виділену
            for (int j = 0; j < i; j++) {
                free(files[j]);
            }
            return -2;
        }
        strcpy(files[i], buff);
    }
    return 0; // Успішно
}

void print_error_msg(void)
{
	uint16_t (*ptr_error_image)[128] = (uint16_t(*)[128])error_image;
    ST7735_DrawImage(0, 0, 160, 128, (uint16_t*) ptr_error_image);
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_SPI1_Init();
  MX_SPI2_Init();
  MX_FATFS_Init();
  /* USER CODE BEGIN 2 */
  ST7735_Init();
  ST7735_Backlight_On();
  char* files[1002] =
  {
	  "image_0.txt",
      "image_1.txt",
      "image_2.txt",
      "image_3.txt",
	  "image_4.txt",
	  "image_5.txt",
	  "image_6.txt",
	  "image_7.txt",
	  "image_8.txt",
	  "image_9.txt",
	  "image_10.txt",
  };
  fill_file_names(files, 1001);

  uint16_t (*first_image)[128] = (uint16_t(*)[128])Ektos_image;
//testing
  //test_lowlevel_sd();
  ST7735_SetRotation(r);
  ST7735_DrawImage(0, 0, 160, 128, (uint16_t*) first_image);



  //fatfs_write(image_1, files[1]);

  //parse_boot_sector();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */


  FIL file;
  FRESULT res;
  UINT br;

  fatfs_init();
  HAL_Delay(1000);

  if( FR_OK != f_open(&file, "i.BIN", FA_READ))
  {
	  uint16_t (*ptr_error_image)[128] = (uint16_t(*)[128])error_image;
	  ST7735_DrawImage(0, 0, 160, 128, (uint16_t*) ptr_error_image);
  }

  static uint16_t counter = 0;
  while (1)
  {
	  if (counter == 0 )
	  {
		  if( FR_OK != f_open(&file, "i.BIN", FA_READ))
		  {
			  print_error_msg();
		  }
	  }
	  uint32_t offset = counter * FRAME_SIZE;

	  res = f_lseek(&file, offset);
	  if (res != FR_OK)
	  {
		  print_error_msg();
	  }
	  res = f_read(&file, image_buffer, FRAME_SIZE, &br);
	  if (res != FR_OK || br != FRAME_SIZE)
	  {
		  print_error_msg();
	  }
	  else
	  {
		  // Тепер image_buffer містить кадр, можна відображати:
		  ST7735_DrawImage(0, 0, 160, 128, (uint16_t*)image_2d);
		  counter = (counter +1 ) % 1001;
	  }

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 180;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream3_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(Test_pin_GPIO_Port, Test_pin_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, ST7735_RES_Pin|ST7735_DC_Pin|ST7735_CS_Pin|ST7735_BL_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : Test_pin_Pin */
  GPIO_InitStruct.Pin = Test_pin_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(Test_pin_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : ST7735_RES_Pin ST7735_DC_Pin ST7735_CS_Pin ST7735_BL_Pin */
  GPIO_InitStruct.Pin = ST7735_RES_Pin|ST7735_DC_Pin|ST7735_CS_Pin|ST7735_BL_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : SPI2_CS_Pin */
  GPIO_InitStruct.Pin = SPI2_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SPI2_CS_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
