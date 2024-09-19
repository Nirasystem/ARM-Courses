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
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define LED_CONFIGS_LEN         3
#define DB_KEY                  0xABCD

typedef struct {
  GPIO_TypeDef*         GPIO;
  uint16_t              Pin;
} Led_PinConfig;

typedef enum {
  Led_State_Off,
  Led_State_On,
} Led_State;

typedef struct {
  uint32_t        ResetCounter;
  Led_State       Leds[LED_CONFIGS_LEN];
  uint32_t        Key;
} Database;


/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define FLASH_MAX_TRY           10

#define DB_ADDRESS              0x08060000
#define DB_SECTOR               FLASH_SECTOR_7

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
static const Led_PinConfig LED_CONFIGS[] = {
  { LED0_GPIO_Port, LED0_Pin },
  { LED1_GPIO_Port, LED1_Pin },
  { LED2_GPIO_Port, LED2_Pin },
};

static const Database* db = (const Database*) DB_ADDRESS;



/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void Database_setDefault(void);
void Database_increaseResetCounter(void);
void Database_setLedState(uint8_t num, Led_State state);

uint32_t Flash_getSector(uint32_t addr);
HAL_StatusTypeDef Flash_tryWrite(uint8_t* addr, uint8_t* data, uint32_t len);
HAL_StatusTypeDef Flash_write(uint8_t* addr, uint8_t* data, uint32_t len);
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
  uint8_t i;
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
  /* USER CODE BEGIN 2 */
  // Check Database
  if (db->Key != DB_KEY) {
    // Store default
    Database_setDefault();
  }    
  
  //Database_increaseResetCounter();
  
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    for (i = 0; i < LED_CONFIGS_LEN; i++) {
      HAL_GPIO_WritePin(LED_CONFIGS[i].GPIO, LED_CONFIGS[i].Pin, !db->Leds[i]);
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
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
void Database_increaseResetCounter(void) {
  Database tmp;
  
  memcpy(&tmp, db, sizeof(Database));
  
  tmp.ResetCounter++;
  Flash_write((uint8_t*) db, (uint8_t*) &tmp, sizeof(Database));
}

void Database_setLedState(uint8_t num, Led_State state) {
  Database tmp;
  
  memcpy(&tmp, db, sizeof(Database));
  
  tmp.Leds[num] = state;
  Flash_write((uint8_t*) db, (uint8_t*) &tmp, sizeof(Database));
}

void Database_setDefault(void) {
  static const Database DEFAULT = {
    .Key = DB_KEY,
    .ResetCounter = 0,
    .Leds = {
      Led_State_On,
      Led_State_Off,
      Led_State_On,
    },
  };
  
  Flash_write((uint8_t*) db, (uint8_t*) &DEFAULT, sizeof(Database));
}

uint32_t Flash_getSector(uint32_t addr) {
  if (addr >= 0x080E0000) {
    return FLASH_SECTOR_11;
  }
  else if (addr >= 0x080C0000) {
    return FLASH_SECTOR_10;
  }
  else if (addr >= 0x080A0000) {
    return FLASH_SECTOR_9;
  }
  else if (addr >= 0x08080000) {
    return FLASH_SECTOR_8;
  }
  else if (addr >= 0x08060000) {
    return FLASH_SECTOR_7;
  }
  else if (addr >= 0x08040000) {
    return FLASH_SECTOR_6;
  }
  else if (addr >= 0x08020000) {
    return FLASH_SECTOR_5;
  }
  else if (addr >= 0x08010000) {
    return FLASH_SECTOR_4;
  }
  else if (addr >= 0x0800C000) {
    return FLASH_SECTOR_3;
  }
  else if (addr >= 0x08008000) {
    return FLASH_SECTOR_2;
  }
  else if (addr >= 0x08004000) {
    return FLASH_SECTOR_1;
  }
  else {
    return FLASH_SECTOR_0;
  }
}
HAL_StatusTypeDef Flash_tryWrite(uint8_t* addr, uint8_t* data, uint32_t len) {
  HAL_StatusTypeDef status;
  FLASH_EraseInitTypeDef erase;
  uint32_t endSector;
  uint32_t sectorError;
  // Unlock
  status = HAL_FLASH_Unlock();
  if (status != HAL_OK) {
    return status;
  }
  // Erase
  erase.Banks = FLASH_BANK_1;
  erase.Sector = Flash_getSector((uint32_t) addr);
  endSector = Flash_getSector((uint32_t) addr + len);
  erase.NbSectors = endSector - erase.Sector + 1;
  erase.TypeErase = FLASH_TYPEERASE_SECTORS;
  erase.VoltageRange = FLASH_VOLTAGE_RANGE_2;
  status = HAL_FLASHEx_Erase(&erase, &sectorError);
  if (status != HAL_OK) {
    return status;
  }
  // Clear Cache
  __HAL_FLASH_DATA_CACHE_DISABLE();
  __HAL_FLASH_INSTRUCTION_CACHE_DISABLE();
  
  __HAL_FLASH_DATA_CACHE_RESET();
  __HAL_FLASH_INSTRUCTION_CACHE_RESET();
  
  __HAL_FLASH_DATA_CACHE_ENABLE();
  __HAL_FLASH_INSTRUCTION_CACHE_ENABLE();
  // Program
  while (len-- > 0) {
    status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_BYTE, (uint32_t) addr, *data);
    if (status != HAL_OK) {
      return status;
    }
    addr++;
    data++;
  }
  // Lock
  return HAL_FLASH_Lock();
}
HAL_StatusTypeDef Flash_write(uint8_t* addr, uint8_t* data, uint32_t len) {
  uint8_t tryCount = FLASH_MAX_TRY;
  HAL_StatusTypeDef status = HAL_ERROR;
  
  while (tryCount-- > 0 && status != HAL_OK) {
    status = Flash_tryWrite(addr, data, len);
  }
  
  return status;
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
