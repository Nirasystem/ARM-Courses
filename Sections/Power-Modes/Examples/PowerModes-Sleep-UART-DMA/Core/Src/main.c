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
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "CircularBuffer.h"
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct {
  GPIO_TypeDef*       GPIO;
  uint16_t            Pin;
} Led_PinConfig;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define LED_CONFIGS_LEN     3
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
static const char STARTUP[] = "PowerModes-Sleep-UART-DMA\r\n";

static const Led_PinConfig LED_CONFIGS[LED_CONFIGS_LEN] = {
  { LED0_GPIO_Port, LED0_Pin },
  { LED1_GPIO_Port, LED1_Pin },
  { LED2_GPIO_Port, LED2_Pin },
};


static UART_CircularBuffer uart1Tx;
static UART_CircularBuffer uart1Rx;
static uint8_t uart1TxBuf[128];
static uint8_t uart1RxBuf[128];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void commandProcess(char* line);

char* Str_ignoreWhitespace(char* str);
uint32_t Str_convertUNum(char* str, int16_t* len);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint32_t HAL_GetTick(void) {
  //return TIM5->CNT;
  return __HAL_TIM_GET_COUNTER(&htim5);
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
  MX_TIM5_Init();
  HAL_TIM_Base_Start(&htim5);
  
  SysTick->CTRL = 0;
  
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART1_UART_Init();
  MX_TIM5_Init();
  /* USER CODE BEGIN 2 */
  UART_CircularBuffer_init(&uart1Tx, &huart1, uart1TxBuf, sizeof(uart1TxBuf));
  UART_CircularBuffer_init(&uart1Rx, &huart1, uart1RxBuf, sizeof(uart1RxBuf));
  
  UART_CircularBuffer_receiveIdle(&uart1Rx);
  
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  
  UART_CircularBuffer_writeBytes(&uart1Tx, (uint8_t*) STARTUP, sizeof(STARTUP) - 1);
  UART_CircularBuffer_transmit(&uart1Tx);
  while (1)
  {
    if (UART_CircularBuffer_available(&uart1Rx) > 0) {
      int16_t len = UART_CircularBuffer_findByte(&uart1Rx, '\n');
      if (len >= 0) {
        len += 1;
        // Check Empty Line
        if (len >= 2) {
          char buf[32];
          UART_CircularBuffer_readBytes(&uart1Rx, (uint8_t*) buf, len);
          buf[len] = '\0';
          // Process Line
          // Command Format:
          // LED = <num>, <state>
          commandProcess(buf);
        }
      }
    }
    
    // Enter Sleep
    HAL_PWR_EnableSleepOnExit();
    HAL_PWR_EnterSLEEPMode(PWR_MAINREGULATOR_ON, PWR_SLEEPENTRY_WFI);
    
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
  RCC_OscInitStruct.PLL.PLLN = 60;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void commandProcess(char* line) {
  if (strncmp(line, "LED", 3) == 0) {
    line += 3;
    line = Str_ignoreWhitespace(line);
    // Check '='
    if (line[0] == '=') {
      line += 1;
      line = Str_ignoreWhitespace(line);
      // Get Num
      int16_t len;
      uint8_t num = Str_convertUNum(line, &len);
      if (len > 0) {
        line += len;
        line = Str_ignoreWhitespace(line);
        // Check ','
        if (line[0] == ',') {
          line += 1;
          line = Str_ignoreWhitespace(line);
          uint8_t state = Str_convertUNum(line, &len);
          if (len > 0) {
            // Check Num
            if (num < LED_CONFIGS_LEN) {
              // Check State
              if (state <= 1) {
                HAL_GPIO_WritePin(LED_CONFIGS[num].GPIO, LED_CONFIGS[num].Pin, !state);
                
                static const char RESULT[] = "Ok\r\n";
                UART_CircularBuffer_writeBytes(&uart1Tx, (uint8_t*) RESULT, sizeof(RESULT) - 1);
                UART_CircularBuffer_transmit(&uart1Tx);
              }
              else {
                static const char RESULT[] = "Wrong State, Valid: 0,1\r\n";
                UART_CircularBuffer_writeBytes(&uart1Tx, (uint8_t*) RESULT, sizeof(RESULT) - 1);
                UART_CircularBuffer_transmit(&uart1Tx);
              }
            }
            else {
              static const char RESULT[] = "Wrong Num, Valid: 0,1,2\r\n";
              UART_CircularBuffer_writeBytes(&uart1Tx, (uint8_t*) RESULT, sizeof(RESULT) - 1);
              UART_CircularBuffer_transmit(&uart1Tx);
            }
          }
        }
      }
    }
    // Wrong Format
    else {
      static const char RESULT[] = "Wrong Format, Expected '='\r\n";
      UART_CircularBuffer_writeBytes(&uart1Tx, (uint8_t*) RESULT, sizeof(RESULT) - 1);
      UART_CircularBuffer_transmit(&uart1Tx);
    }
  }
}


char* Str_ignoreWhitespace(char* str) {
  while (*str != '\0' && *str <= ' ') {
    str++;
  }
  
  return str;
}
uint32_t Str_convertUNum(char* str, int16_t* len) {
  uint32_t z = 0;
  char* pStart = str;
  
  while (*str != 0 && (*str <= '9' && *str >= '0')) {
    z = (z * 10) + (*str++ - '0');
  }
  
  if (len) {
    *len = (int16_t)(str - pStart);
  }    
  
  return z;
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef* huart, uint16_t len) {
  switch ((uint32_t) huart->Instance) {
    case USART1_BASE:
      UART_CircularBuffer_handleRxEvent(&uart1Rx, len);
      break;
  }
  
  HAL_PWR_DisableSleepOnExit();
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef* huart) {
  switch ((uint32_t) huart->Instance) {
    case USART1_BASE:
      UART_CircularBuffer_handleRx(&uart1Rx);
      break;
  }
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef* huart) {
  switch ((uint32_t) huart->Instance) {
    case USART1_BASE:
      UART_CircularBuffer_handleTx(&uart1Tx);
      break;
  }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef* huart) {
  switch ((uint32_t) huart->Instance) {
    case USART1_BASE:
      UART_CircularBuffer_resetIO(&uart1Tx);
      UART_CircularBuffer_resetIO(&uart1Rx);
      UART_CircularBuffer_receive(&uart1Rx);
      break;
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
