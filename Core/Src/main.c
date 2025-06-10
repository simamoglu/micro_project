/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "RC522.h"
#include "string.h"
#include "liquidcrystal_i2c.h"
#include "ds1307.h"
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct {
    char name[16];
    struct {
        uint8_t hours;
        uint8_t minutes;
        uint8_t seconds;
        uint8_t date;
        uint8_t month;
    } time;
} FlashEntry;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define BTN_NEXT_PIN GPIO_PIN_0
#define BTN_NEXT_PORT GPIOA
#define FLASH_USER_START_ADDR   ((uint32_t)0x0807F800)  // example for STM32F446RE, last 2KB page
#define FLASH_USER_END_ADDR     ((uint32_t)0x08080000)
#define ENTRY_SIZE              sizeof(FlashEntry)
//#define MAX_ENTRIES             ((FLASH_USER_END_ADDR - FLASH_USER_START_ADDR) / ENTRY_SIZE)
#define MAX_ENTRIES 100
#define NAME_LENGTH 16
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
uint8_t status;
uint8_t str[16];
uint8_t sNum[5];
uint16_t currentIndex = 0;
FlashEntry flashEntries[MAX_ENTRIES];
uint8_t entryCount = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI1_Init(void);
/* USER CODE BEGIN PFP */
void SaveLog(const char* name, RTC_Time* rtc_time) {
    if (entryCount < MAX_ENTRIES) {
        // Copy the name (ensure it's null-terminated)
        strncpy(flashEntries[entryCount].name, name, NAME_LENGTH - 1);
        flashEntries[entryCount].name[NAME_LENGTH - 1] = '\0';

        // Save time fields
        flashEntries[entryCount].time.hours = rtc_time->hours;
        flashEntries[entryCount].time.minutes = rtc_time->minutes;
        flashEntries[entryCount].time.seconds = rtc_time->seconds;
        flashEntries[entryCount].time.date = rtc_time->date;
        flashEntries[entryCount].time.month = rtc_time->month;

        entryCount++;
    }
}

void ShowEntry(void)
{
	HD44780_Clear();
	HD44780_SetCursor(0,0);
	HD44780_PrintStr("Showing entries");
	HAL_Delay(2000);
    if (entryCount == 0) {
        HD44780_Clear();
        HD44780_PrintStr("No entries yet");
        HAL_Delay(2000);
        return;
    }

    uint8_t index = 0;

    while (1) {
        // Show entry
        HD44780_Clear();
        HD44780_SetCursor(0, 0);
        HD44780_PrintStr(flashEntries[index].name);

        HD44780_SetCursor(0, 1);
        char timeBuf[17];
        snprintf(timeBuf, sizeof(timeBuf), "%02d:%02d:%02d %02d/%02d",
                 flashEntries[index].time.hours,
                 flashEntries[index].time.minutes,
                 flashEntries[index].time.seconds,
                 flashEntries[index].time.date,
                 flashEntries[index].time.month);
        HD44780_PrintStr(timeBuf);

        // Wait for button press
        uint32_t timeout = HAL_GetTick() + 5000; // 5 seconds timeout
        while (HAL_GetTick() < timeout) {
            if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == GPIO_PIN_SET) { // Next
                HAL_Delay(200);  // Debounce
                index = (index + 1) % entryCount;
                break;
            }

            if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_1) == GPIO_PIN_SET) { // Back
                HAL_Delay(200);  // Debounce
                if (index == 0) {
                    index = entryCount - 1;
                } else {
                    index--;
                }
                break;
            }
        }

        // Exit loop if no buttons pressed for 10 seconds
        if (HAL_GetTick() >= timeout) {
            HD44780_Clear();
            HD44780_PrintStr("Exiting...");
            HAL_Delay(1000);
            break;
        }
    }
}
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */
  MFRC522_Init();
  HD44780_Init(2);

  RTC_Time rtc_time;
  char timeStr[17];

  if (!DS1307_Init(&hi2c1)) {
      HD44780_Clear();
      HD44780_PrintStr("RTC Not Found");
      HAL_Delay(2000);
  }
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  //Scan for the card ID
	  status = MFRC522_Request(PICC_REQIDL, str);
	  status = MFRC522_Anticoll(str);
	  memcpy(sNum, str, 5);
	  HAL_Delay(200);

	  if (DS1307_GetTime(&hi2c1, &rtc_time)) {
	      snprintf(timeStr, sizeof(timeStr), "%02d:%02d", rtc_time.hours, rtc_time.minutes);
	      HD44780_Clear();
	      HD44780_SetCursor(11, 0);
	      HD44780_PrintStr(timeStr);
	      HD44780_SetCursor(0, 1);
	      HD44780_PrintStr("Please show card");
	  }
	  if ((str[0] == 147) && (str[1] == 32) && (str[2] == 93) && (str[3] == 89) && (str[4] == 93)) {
		  ShowEntry();
		  HAL_Delay(10000);
		  memset(str, 0, sizeof(str));
	  }
	  if ((str[0] == 147) && (str[1] == 32) && (str[2] == 17) && (str[3] == 16) && (str[4] == 43)) {
		  HD44780_Clear();
		  HD44780_SetCursor(0,0);
		  HD44780_PrintStr("Welcome");
		  HD44780_SetCursor(0,1);
		  HD44780_PrintStr("Fazli Kemal");
		  SaveLog("Fazli Kemal", &rtc_time);
		  HAL_Delay(2000);
		  memset(str, 0, sizeof(str));
	  }
	  if ((str[0] == 147) && (str[1] == 32) && (str[2] == 143) && (str[3] == 204) && (str[4] == 61)) {
		  HD44780_Clear();
		  HD44780_SetCursor(0,0);
		  HD44780_PrintStr("Welcome");
		  HD44780_SetCursor(0,1);
		  HD44780_PrintStr("Mevlut Akhan");
		  SaveLog("Mevlut Akhan", &rtc_time);
		  HAL_Delay(2000);
		  memset(str, 0, sizeof(str));
	  }
	  if ((str[0] == 147) && (str[1] == 32) && (str[2] == 106) && (str[3] == 204) && (str[4] == 4)) {
		  HD44780_Clear();
		  HD44780_SetCursor(0,0);
		  HD44780_PrintStr("Welcome");
		  HD44780_SetCursor(0,1);
		  HD44780_PrintStr("Seren Hazal");
		  SaveLog("Seren Hazal", &rtc_time);
		  HAL_Delay(2000);
		  memset(str, 0, sizeof(str));
	  }
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

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
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA8 */
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

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
  __disable_irq();
  while (1)
  {
  }
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
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
