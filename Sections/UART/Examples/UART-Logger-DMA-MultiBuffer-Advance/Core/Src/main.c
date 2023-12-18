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
#include "dma.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

//  _________________
// |  0  |  1  |  2  |
// |_____|_____|_____|

#define TX_BUF_SIZE       16
#define TX_BUF_LEN        3


typedef struct {
  uint8_t               Buf[TX_BUF_SIZE];
  uint16_t              Index;
} UART_MultiBuffer_Buf;

typedef struct {
  UART_HandleTypeDef*   HUART;
  UART_MultiBuffer_Buf  Buf[TX_BUF_LEN];
  uint8_t               ActiveBuffer;
  uint8_t               CurrentBuffer;
} UART_MultiBuffer;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
static UART_MultiBuffer uart1Tx;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void UART_MultiBuffer_init(UART_MultiBuffer* buf, UART_HandleTypeDef* huart);
void UART_MultiBuffer_handle(UART_MultiBuffer* buf);
void UART_MultiBuffer_sendBytes(UART_MultiBuffer* buf, uint8_t* data, uint32_t len);

UART_MultiBuffer_Buf* UART_MultiBuffer_transmit(UART_MultiBuffer* buf);
UART_MultiBuffer_Buf* UART_MultiBuffer_getBuf(UART_MultiBuffer* buf);
UART_MultiBuffer_Buf* UART_MultiBuffer_getBufTx(UART_MultiBuffer* buf);

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
  uint32_t nextSend = 0;
  char str[32];
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
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  UART_MultiBuffer_init(&uart1Tx, &huart1);
  
  UART_MultiBuffer_sendBytes(&uart1Tx, (uint8_t*) "Hello", 5);
  UART_MultiBuffer_sendBytes(&uart1Tx, (uint8_t*) " - ", 3);
  UART_MultiBuffer_sendBytes(&uart1Tx, (uint8_t*) "World", 5);
  UART_MultiBuffer_sendBytes(&uart1Tx, (uint8_t*) "\r\n", 2);
  
  while (1)
  {
    if (nextSend <= HAL_GetTick()) {
      nextSend = HAL_GetTick() + 1000;
      
      uint32_t len = snprintf(str, sizeof(str) - 1, "Send-%u\r\n", HAL_GetTick());
      UART_MultiBuffer_sendBytes(&uart1Tx, (uint8_t*) str, len);
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
void HAL_UART_TxCpltCallback(UART_HandleTypeDef* huart) {
  switch ((uint32_t) huart->Instance) {
    case USART1_BASE:
      UART_MultiBuffer_handle(&uart1Tx);
      break;
    default:
      
      break;
  }
}

void UART_MultiBuffer_init(UART_MultiBuffer* buf, UART_HandleTypeDef* huart) {
  buf->HUART = huart;
  buf->ActiveBuffer = 0;
  buf->CurrentBuffer = 0;
  for (uint8_t i = 0; i < TX_BUF_LEN; i++) {
    buf->Buf[i].Index = 0;
  }
}
void UART_MultiBuffer_handle(UART_MultiBuffer* buf) {
  UART_MultiBuffer_Buf* pBuf = UART_MultiBuffer_getBufTx(buf);
  
  if (++buf->CurrentBuffer >= TX_BUF_LEN) {
    buf->CurrentBuffer = 0;
  }
  pBuf = UART_MultiBuffer_getBufTx(buf);
  
  if (pBuf->Index > 0) {
    UART_MultiBuffer_transmit(buf);
  }
}
UART_MultiBuffer_Buf* UART_MultiBuffer_transmit(UART_MultiBuffer* buf) {
  UART_MultiBuffer_Buf* pBuf = UART_MultiBuffer_getBuf(buf);
  
  if (buf->HUART->gState == HAL_UART_STATE_READY) {  
    HAL_UART_Transmit_DMA(buf->HUART, pBuf->Buf, pBuf->Index);
    pBuf->Index = 0;
    
    if (++buf->ActiveBuffer >= TX_BUF_LEN) {
      buf->ActiveBuffer = 0;
    }
    
    pBuf = UART_MultiBuffer_getBuf(buf);
  }
  
  return pBuf;
}
void UART_MultiBuffer_sendBytes(UART_MultiBuffer* buf, uint8_t* data, uint32_t len) {
  UART_MultiBuffer_Buf* pBuf = UART_MultiBuffer_getBuf(buf);
  uint32_t tmpLen;
  
  while (len > 0) {
    tmpLen = len;
    if (pBuf->Index + tmpLen >= TX_BUF_SIZE) {
      tmpLen = TX_BUF_SIZE - pBuf->Index;
    }
    
    memcpy(&pBuf->Buf[pBuf->Index], data, tmpLen);
    pBuf->Index += tmpLen;
    data += tmpLen;
    len -= tmpLen;
    
    pBuf = UART_MultiBuffer_transmit(buf);
  }
  
}
UART_MultiBuffer_Buf* UART_MultiBuffer_getBuf(UART_MultiBuffer* buf) {
  return &buf->Buf[buf->ActiveBuffer];
}
UART_MultiBuffer_Buf* UART_MultiBuffer_getBufTx(UART_MultiBuffer* buf) {
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
