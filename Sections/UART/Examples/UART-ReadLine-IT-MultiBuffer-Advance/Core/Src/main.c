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
#define RX_BUF_SIZE         64
#define RX_BUF_LEN          4

typedef struct {
  uint8_t       Buffer[RX_BUF_SIZE];
  uint16_t      Index;
  uint8_t       Flag;
} UART_MultiBuffer_Buf;

typedef struct {
  UART_HandleTypeDef*   HUART;
  UART_MultiBuffer_Buf  Buf[RX_BUF_LEN];
  uint8_t               ActiveBuffer;
  uint8_t               CurrentBuffer;
  uint8_t               Temp;
}UART_MultiBuffer;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
static UART_MultiBuffer uart1RxBuf;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void UART_MultiBuffer_init(UART_MultiBuffer* buf, UART_HandleTypeDef* huart);
void UART_MultiBuffer_process(UART_MultiBuffer* buf);
void UART_MultiBuffer_release(UART_MultiBuffer* buf);

uint8_t UART_MultiBuffer_checkFlag(UART_MultiBuffer* buf);
UART_MultiBuffer_Buf* UART_MultiBuffer_get(UART_MultiBuffer* buf);
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
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  UART_MultiBuffer_init(&uart1RxBuf, &huart1);
  while (1)
  {
    if (UART_MultiBuffer_checkFlag(&uart1RxBuf)) {
      UART_MultiBuffer_Buf* buf = UART_MultiBuffer_get(&uart1RxBuf);
      // Process
      HAL_UART_Transmit(&huart1, buf->Buffer, buf->Index, 1000);
      // Reset and goto next buffer
      UART_MultiBuffer_release(&uart1RxBuf);
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
      UART_MultiBuffer_process(&uart1RxBuf);
      break;
    default:
      
      break;
  }
}
void UART_MultiBuffer_init(UART_MultiBuffer* buf, UART_HandleTypeDef* huart) {
  buf->HUART = huart;
  buf->CurrentBuffer = 0;
  buf->ActiveBuffer = 0;
  for (uint8_t i = 0; i < RX_BUF_LEN; i++) {
    buf->Buf[i].Index = 0;
    buf->Buf[i].Flag = 0;
  }
  
  HAL_UART_Receive_IT(buf->HUART, &buf->Temp, sizeof(buf->Temp));
}
void UART_MultiBuffer_process(UART_MultiBuffer* buf) {
  UART_MultiBuffer_Buf* pBuf = &buf->Buf[buf->ActiveBuffer];
  // Fill buffer
  pBuf->Buffer[pBuf->Index++] = buf->Temp;
  // Check CRLF
  if (pBuf->Index >= 2 &&
      buf->Temp == '\n' &&
      pBuf->Buffer[pBuf->Index - 2] == '\r'
  ) {
    // Set Flag and change to next buffer
    pBuf->Flag = 1;
    if (++buf->ActiveBuffer == RX_BUF_LEN) {
      buf->ActiveBuffer = 0;
    }
  }
  // Receive next byte
  HAL_UART_Receive_IT(buf->HUART, &buf->Temp, sizeof(buf->Temp));
}
void UART_MultiBuffer_release(UART_MultiBuffer* buf) {
  buf->Buf[buf->CurrentBuffer].Flag = 0;
  buf->Buf[buf->CurrentBuffer].Index = 0;
  
  if (++buf->CurrentBuffer == RX_BUF_LEN) {
    buf->CurrentBuffer = 0;
  }
}

uint8_t UART_MultiBuffer_checkFlag(UART_MultiBuffer* buf) {
  return buf->Buf[buf->CurrentBuffer].Flag;
}

UART_MultiBuffer_Buf* UART_MultiBuffer_get(UART_MultiBuffer* buf) {
  return &buf->Buf[buf->CurrentBuffer];
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
