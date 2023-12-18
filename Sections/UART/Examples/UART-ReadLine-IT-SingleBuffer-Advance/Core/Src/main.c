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
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define RX_BUF_SIZE     64

typedef struct {
  UART_HandleTypeDef* HUART;
  uint8_t             Buf[RX_BUF_SIZE];
  uint16_t            BufIndex;
  uint8_t             Temp;
  uint8_t             Flag;
} UART_SingleBuffer;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
static UART_SingleBuffer uart1RxBuf;
static UART_SingleBuffer uart2RxBuf;
static UART_SingleBuffer uart3RxBuf;
static UART_SingleBuffer uart4RxBuf;
static UART_SingleBuffer uart5RxBuf;
static UART_SingleBuffer uart6RxBuf;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void UART_SingleBuffer_init(UART_SingleBuffer* buf, UART_HandleTypeDef* huart);
void UART_SingleBuffer_process(UART_SingleBuffer* buf);
void UART_SingleBuffer_reset(UART_SingleBuffer* buf);
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
  MX_USART1_UART_Init();
  MX_UART4_Init();
  MX_UART5_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_USART6_UART_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  UART_SingleBuffer_init(&uart1RxBuf, &huart1);
  UART_SingleBuffer_init(&uart2RxBuf, &huart2);
  UART_SingleBuffer_init(&uart3RxBuf, &huart3);
  UART_SingleBuffer_init(&uart4RxBuf, &huart4);
  UART_SingleBuffer_init(&uart5RxBuf, &huart5);
  UART_SingleBuffer_init(&uart6RxBuf, &huart6);
  while (1)
  {
    if (uart1RxBuf.Flag) {
      // Process
      HAL_UART_Transmit_IT(&huart1, uart1RxBuf.Buf, uart1RxBuf.BufIndex);
      // Reset varialbles
      UART_SingleBuffer_reset(&uart1RxBuf);
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
void HAL_UART_RxCpltCallback(UART_HandleTypeDef* huart) {
  switch ((uint32_t) huart->Instance) {
    case USART1_BASE:
      UART_SingleBuffer_process(&uart1RxBuf);
      break;
    case USART2_BASE:
      UART_SingleBuffer_process(&uart2RxBuf);
      break;
    case USART3_BASE:
      UART_SingleBuffer_process(&uart3RxBuf);
      break;
    case UART4_BASE:
      UART_SingleBuffer_process(&uart4RxBuf);
      break;
    case UART5_BASE:
      UART_SingleBuffer_process(&uart5RxBuf);
      break;
    case USART6_BASE:
      UART_SingleBuffer_process(&uart6RxBuf);
      break;
    default:
      
      break;
  }
}

void UART_SingleBuffer_init(UART_SingleBuffer* buf, UART_HandleTypeDef* huart) {
  buf->HUART = huart;
  buf->BufIndex = 0;
  buf->Flag = 0;
  
  HAL_UART_Receive_IT(buf->HUART, &buf->Temp, sizeof(buf->Temp));
}
void UART_SingleBuffer_process(UART_SingleBuffer* buf) {
  // Store Byte into buffer
  buf->Buf[buf->BufIndex++] = buf->Temp;
  // Check for CRLF = "\r\n" = 0x0D 0x0A
  if (buf->BufIndex >= 2 &&
      buf->Temp == '\n'  &&
      buf->Buf[buf->BufIndex - 2] == '\r'
  ) {
    buf->Flag = 1;
  }
  // Receive Next Byte
  HAL_UART_Receive_IT(buf->HUART, &buf->Temp, sizeof(buf->Temp));
}
void UART_SingleBuffer_reset(UART_SingleBuffer* buf) {
  buf->Flag = 0;
  buf->BufIndex = 0;
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
