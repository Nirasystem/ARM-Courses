/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "fatfs.h"
#include "sdio.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define LED_MAX_NUM     3

#define DB_KEY          0xABCD1234

typedef enum {
  Led_Mode_Off      = 0,
  Led_Mode_On       = 1,
  Led_Mode_Blink    = 2,
} Led_Mode;

typedef struct {
  uint8_t         Mode;
  uint16_t        BlinkPeriod;
} Led_ModeConfig;

typedef struct {
  Led_ModeConfig  Leds[LED_MAX_NUM];
  uint32_t        Key;
} Database;

typedef struct {
  GPIO_TypeDef*   GPIO;
    uint16_t      Pin;
} Led_PinConfig;

typedef struct {
  const Led_PinConfig*    PinConfig;
  Led_ModeConfig*         Config;
  uint32_t                NextCheck;
} Led_HandleTypeDef;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
static const char DB_PATH[] = "0:/board.db";

static const Led_PinConfig    LED_PIN_CONFIGS[LED_MAX_NUM] = {
  { LED0_GPIO_Port, LED0_Pin },
  { LED1_GPIO_Port, LED1_Pin },
  { LED2_GPIO_Port, LED2_Pin },
};

static const Database DB_DEFAULT = {
  .Leds = {
    { .Mode = Led_Mode_Off, .BlinkPeriod = 1000 },
    { .Mode = Led_Mode_Off, .BlinkPeriod = 1000 },
    { .Mode = Led_Mode_Off, .BlinkPeriod = 1000 },
  },
  .Key = DB_KEY,
};

static Database db = {0};

static Led_HandleTypeDef leds[LED_MAX_NUM] = {0};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void Database_load(Database* db);
void Database_save(Database* db);

void Led_init(Led_HandleTypeDef* led, const Led_PinConfig* pinConfig, Led_ModeConfig* config);
void Led_handle(Led_HandleTypeDef* led);
void Led_initArray(Led_HandleTypeDef* leds, const Led_PinConfig* pinConfig, Led_ModeConfig* config, uint32_t len);
void Led_handleArray(Led_HandleTypeDef* leds, uint32_t len);

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
  MX_SDIO_SD_Init();
  MX_FATFS_Init();
  /* USER CODE BEGIN 2 */
  
  f_mount(&SDFatFS, SDPath, 1);
  
  Database_load(&db);
  if (db.Key != DB_KEY) {
    // Set Factory Settings
    memcpy(&db, &DB_DEFAULT, sizeof(Database));
    Database_save(&db);
  }
  
  Led_initArray(leds, LED_PIN_CONFIGS, db.Leds, LED_MAX_NUM);
  
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    Led_handleArray(leds, LED_MAX_NUM);
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
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void Database_load(Database* db) {
  // Read DB File
  if (f_open(&SDFile, DB_PATH, FA_OPEN_EXISTING | FA_READ) == FR_OK) {
    // Check for validation database
    if (f_size(&SDFile) == sizeof(Database)) {
      uint32_t readBytes;
      f_read(&SDFile, db, sizeof(Database), &readBytes);
      while (readBytes < sizeof(Database) && !f_eof(&SDFile));
    }
    
    f_close(&SDFile);
  }
}
void Database_save(Database* db) {
  // Write DB File
  if (f_open(&SDFile, DB_PATH, FA_CREATE_ALWAYS | FA_WRITE) == FR_OK) {
    uint32_t writeBytes;
    f_write(&SDFile, db, sizeof(Database), &writeBytes);
    while (writeBytes < sizeof(Database));
    
    f_close(&SDFile);
  }
}

void Led_handle(Led_HandleTypeDef* led) {
  if (led->NextCheck <= HAL_GetTick()) {
    switch ((Led_Mode) led->Config->Mode) {
      case Led_Mode_Off:
        HAL_GPIO_WritePin(led->PinConfig->GPIO, led->PinConfig->Pin, GPIO_PIN_SET);
        led->NextCheck = HAL_GetTick() + 1000;
        break;
      case Led_Mode_On:
        HAL_GPIO_WritePin(led->PinConfig->GPIO, led->PinConfig->Pin, GPIO_PIN_RESET);
        led->NextCheck = HAL_GetTick() + 1000;
        break;
      case Led_Mode_Blink:
        HAL_GPIO_TogglePin(led->PinConfig->GPIO, led->PinConfig->Pin);
        led->NextCheck = HAL_GetTick() + led->Config->BlinkPeriod;
        break;
    }
  }  
}

void Led_init(Led_HandleTypeDef* led, const Led_PinConfig* pinConfig, Led_ModeConfig* config) {
  led->PinConfig = pinConfig;
  led->Config = config;
  led->NextCheck = 0;
}

void Led_initArray(Led_HandleTypeDef* leds, const Led_PinConfig* pinConfig, Led_ModeConfig* config, uint32_t len) {
  while (len-- > 0) {
    Led_init(leds++, pinConfig++, config++);
  }
}
void Led_handleArray(Led_HandleTypeDef* leds, uint32_t len) {
  while (len-- > 0) {
    Led_handle(leds++);
  }
}
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
