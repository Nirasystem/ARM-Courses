/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2023 STMicroelectronics.
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
#include "ALCD.h"
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

#define LEDS_NUM        3

typedef enum {
  Led_Mode_Off,
  Led_Mode_On,
  Led_Mode_Blink,
} Led_Mode;

typedef struct {
  uint16_t      OffTime;
  uint16_t      OnTime;
  Led_Mode      Mode;
} Led_Config;

typedef struct {
  GPIO_TypeDef*       GPIO;
  uint16_t            Pin;
} Led_PinConfig;

typedef struct {
  uint32_t      NextCheck;
  uint8_t       State;
  uint8_t       Change;
} Led_Handle;

typedef struct {
  Led_Config    Leds[LEDS_NUM];
  uint32_t      ResetCount;
  char          Serial[16];
  uint32_t      Key;
} Database;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define DB_KEY          0xABCD
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
static const Led_PinConfig LED_CONFIGS[LEDS_NUM] = {
  { LED0_GPIO_Port, LED0_Pin },
  { LED1_GPIO_Port, LED1_Pin },
  { LED2_GPIO_Port, LED2_Pin },
};


Database* db = (Database*) BKPSRAM_BASE;
Led_Handle leds[LEDS_NUM] = {0};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void Led_handle(void);
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
  /* USER CODE BEGIN 2 */
  
  // Load Database
  if (db->Key != DB_KEY) {
    // Set defaults values
    for (uint8_t i = 0; i < LEDS_NUM; i++) {
      db->Leds[i].Mode = Led_Mode_Off;
      db->Leds[i].OffTime = 500;
      db->Leds[i].OnTime = 500;
    }
    db->ResetCount = 0;
    strcpy(db->Serial, "SN-12345678");
    // Update Key
    db->Key = DB_KEY;
  }
  
  ALCD_init(16, 2);
  
  ALCD_printfXY(0, 0, "Cnt:%u", db->ResetCount++);
  ALCD_putsXY(0, 1, db->Serial);
  
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    Led_handle();
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
void Led_handle(void) {
  for (uint8_t i = 0; i < LEDS_NUM; i++) {
    if (leds[i].NextCheck <= HAL_GetTick() || leds[i].Change) {
      leds[i].Change = 0;
      switch (db->Leds[i].Mode) {
        case Led_Mode_Off:
          HAL_GPIO_WritePin(LED_CONFIGS[i].GPIO, LED_CONFIGS[i].Pin, GPIO_PIN_SET);
          leds[i].NextCheck = 1000 + HAL_GetTick();
          break;
        case Led_Mode_On:
          HAL_GPIO_WritePin(LED_CONFIGS[i].GPIO, LED_CONFIGS[i].Pin, GPIO_PIN_RESET);
          leds[i].NextCheck = 1000 + HAL_GetTick();
          break;
        case Led_Mode_Blink:
          if (LED_CONFIGS[i].GPIO->ODR & LED_CONFIGS[i].Pin) {
            // Pin is High
            // Set it to Low
            HAL_GPIO_WritePin(LED_CONFIGS[i].GPIO, LED_CONFIGS[i].Pin, GPIO_PIN_RESET);
            leds[i].NextCheck = db->Leds[i].OnTime + HAL_GetTick();
          }
          else {
            // Pin is Low
            // Set it to High
            HAL_GPIO_WritePin(LED_CONFIGS[i].GPIO, LED_CONFIGS[i].Pin, GPIO_PIN_SET);
            leds[i].NextCheck = db->Leds[i].OffTime + HAL_GetTick();
          }
          break;
      }
    }
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
