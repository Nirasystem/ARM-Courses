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
#include <string.h>
#include <stdio.h>
#include <stdarg.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

//                         Circular Buffer (FIFO)
//            Overflow 0                           Overflow 1
//  _______________________________   |  ________________________________
// |_______|_____________|_________|  | |_____|_________________|________|
// 0       R             W         S  | 0     W                 R        S
// 
// R: RPos, W: WPos, S: Size, OVF: Overflow
//
// Formulas: Available (Read)
// OVF  R    W
//  0   R <  W   =>   W - R
//  1   R >  W   =>   S - R + W 
//  0   R == W   =>   0
//  1   R == W   =>   S
//  Available = S * OVF + W - R
//
// Formulas: Space (Write)
// OVF  R    W
//  0   R <  W   =>   S - W + R
//  1   R >  W   =>   R - W
//  0   R == W   =>   S
//  1   R == W   =>   0
//  Space = S * !OVF + R - W

typedef enum {
  UART_CircularBuffer_Result_Ok,
  UART_CircularBuffer_Result_NoSpace,
  UART_CircularBuffer_Result_NoAvailable,
} UART_CircularBuffer_Result;

typedef struct {
  UART_HandleTypeDef*     HUART;
  uint8_t*                Buffer;
  int16_t                 WPos;
  int16_t                 RPos;
  int16_t                 Size;
  int16_t                 PendingBytes;
  uint8_t                 Overflow;
  uint8_t                 InReceive;
  uint8_t                 InTransmit;
} UART_CircularBuffer;


/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
static uint8_t uart1TxBuf[100];
static UART_CircularBuffer uart1Tx;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void UART_CircularBuffer_init(UART_CircularBuffer* buf, UART_HandleTypeDef* huart, uint8_t* dataBuf, int16_t size);

int16_t UART_CircularBuffer_available(UART_CircularBuffer* buf);
int16_t UART_CircularBuffer_space(UART_CircularBuffer* buf);

int16_t UART_CircularBuffer_availableUncheck(UART_CircularBuffer* buf);
int16_t UART_CircularBuffer_spaceUncheck(UART_CircularBuffer* buf);

UART_CircularBuffer_Result UART_CircularBuffer_writeBytes(UART_CircularBuffer* buf, uint8_t* data, uint32_t len);
UART_CircularBuffer_Result UART_CircularBuffer_readBytes(UART_CircularBuffer* buf, uint8_t* data, uint32_t len);

UART_CircularBuffer_Result UART_CircularBuffer_writeFmt(UART_CircularBuffer* buf, const char* fmt, ...);

void UART_CircularBuffer_moveWritePos(UART_CircularBuffer* buf, int16_t step);
void UART_CircularBuffer_moveReadPos(UART_CircularBuffer* buf, int16_t step);

int16_t UART_CircularBuffer_directAvailable(UART_CircularBuffer* buf);
int16_t UART_CircularBuffer_directSpace(UART_CircularBuffer* buf);

uint8_t* UART_CircularBuffer_getReadDataPtr(UART_CircularBuffer* buf);
uint8_t* UART_CircularBuffer_getWriteDataPtr(UART_CircularBuffer* buf);

int16_t UART_CircularBuffer_findByte(UART_CircularBuffer* buf, uint8_t val);

// UART Rx APIs
void UART_CircularBuffer_receive(UART_CircularBuffer* buf);
void UART_CircularBuffer_handleRx(UART_CircularBuffer* buf);
// UART Tx APIS
void UART_CircularBuffer_transmit(UART_CircularBuffer* buf);
void UART_CircularBuffer_handleTx(UART_CircularBuffer* buf);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int fputc(int ch, FILE* f) {
  UART_CircularBuffer_writeBytes(&uart1Tx, (uint8_t*) &ch, 1);
  UART_CircularBuffer_transmit(&uart1Tx);
  
  return ch;
}
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
  UART_CircularBuffer_init(&uart1Tx, &huart1, uart1TxBuf, sizeof(uart1TxBuf));
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  
  while (1)
  {
    if (nextSend <= HAL_GetTick()) {
      nextSend = HAL_GetTick() + 1000;
      
      // Legacy Way
      /*UART_CircularBuffer_writeBytes(&uart1Tx, (uint8_t*) "Send-", 5);
      UART_CircularBuffer_transmit(&uart1Tx);
      
      int16_t len = snprintf(str, sizeof(str) - 1, "%u\r\n", HAL_GetTick());
      UART_CircularBuffer_writeBytes(&uart1Tx, (uint8_t*) str, len);*/
      
      // Write Fmt
      //UART_CircularBuffer_writeFmt(&uart1Tx, "Send-%u\r\n", HAL_GetTick());
      //UART_CircularBuffer_transmit(&uart1Tx);
      
      // Send with printf
      printf("Send-%u\r\n", HAL_GetTick());
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
      UART_CircularBuffer_handleTx(&uart1Tx);
      break;
    default:
      
      break;
  }
}


void UART_CircularBuffer_init(UART_CircularBuffer* buf, UART_HandleTypeDef* huart, uint8_t* dataBuf, int16_t size) {
  buf->HUART = huart;
  buf->Buffer = dataBuf;
  buf->Size = size;
  buf->Overflow = 0;
  buf->RPos = 0;
  buf->WPos = 0;
  buf->InReceive = 0;
  buf->InTransmit = 0;
}

int16_t UART_CircularBuffer_availableUncheck(UART_CircularBuffer* buf) {
  return buf->Size * buf->Overflow + buf->WPos - buf->RPos;
}
int16_t UART_CircularBuffer_spaceUncheck(UART_CircularBuffer* buf) {
  return buf->Size * (!buf->Overflow) + buf->RPos - buf->WPos;
}

int16_t UART_CircularBuffer_available(UART_CircularBuffer* buf) {
  int16_t bytesLen = buf->HUART->hdmarx ? 
                      buf->PendingBytes - __HAL_DMA_GET_COUNTER(buf->HUART->hdmarx) :
                      buf->PendingBytes - buf->HUART->RxXferCount;
  
  if (bytesLen > 0) {
    UART_CircularBuffer_moveWritePos(buf, bytesLen);
    buf->PendingBytes -= bytesLen;
  }
  
  return UART_CircularBuffer_availableUncheck(buf);
}
int16_t UART_CircularBuffer_space(UART_CircularBuffer* buf) {
  int16_t bytesLen = buf->HUART->hdmatx ? 
                      buf->PendingBytes - __HAL_DMA_GET_COUNTER(buf->HUART->hdmatx) :
                      buf->PendingBytes - buf->HUART->TxXferCount;
  
  if (bytesLen > 0) {
    UART_CircularBuffer_moveReadPos(buf, bytesLen);
    buf->PendingBytes -= bytesLen;
  }
  
  return UART_CircularBuffer_spaceUncheck(buf);
}

UART_CircularBuffer_Result UART_CircularBuffer_writeBytes(UART_CircularBuffer* buf, uint8_t* data, uint32_t len) {
  if (UART_CircularBuffer_space(buf) < len) {
    return UART_CircularBuffer_Result_NoSpace;
  }
  
  if (buf->WPos + len >= buf->Size) {
    int16_t tmpLen = buf->Size - buf->WPos;
    
    memcpy(&buf->Buffer[buf->WPos], data, tmpLen);
    data += tmpLen;
    len -= tmpLen;
    // buf->WPos = (buf->WPos + tmpLen) % buf->Size
    buf->WPos = 0;
    buf->Overflow = 1;
  }
  
  if (len > 0) {
    memcpy(&buf->Buffer[buf->WPos], data, len);
    buf->WPos += len;
  }
    
  return UART_CircularBuffer_Result_Ok;
}
UART_CircularBuffer_Result UART_CircularBuffer_readBytes(UART_CircularBuffer* buf, uint8_t* data, uint32_t len) {
  if (UART_CircularBuffer_available(buf) < len) {
    return UART_CircularBuffer_Result_NoAvailable;
  }
  
  if (buf->RPos + len >= buf->Size) {
    int16_t tmpLen = buf->Size - buf->RPos;
    
    memcpy(data, &buf->Buffer[buf->RPos], tmpLen);
    data += tmpLen;
    len -= tmpLen;
    // buf->RPos = (buf->RPos + tmpLen) % buf->Size
    buf->RPos = 0;
    buf->Overflow = 0;
  }
  
  if (len > 0) {
    memcpy(data, &buf->Buffer[buf->RPos], len);
    buf->RPos += len;
  }
    
  return UART_CircularBuffer_Result_Ok;
}

UART_CircularBuffer_Result UART_CircularBuffer_writeFmt(UART_CircularBuffer* buf, const char* fmt, ...) {
  char temp[64];
  
  va_list args;
  va_start(args, fmt);
  int16_t len = vsnprintf(temp, sizeof(temp) - 1, fmt, args);
  va_end(args);
  
  return UART_CircularBuffer_writeBytes(buf, (uint8_t*) temp, len);
}


void UART_CircularBuffer_moveWritePos(UART_CircularBuffer* buf, int16_t step) {
  buf->WPos += step;
  if (buf->WPos >= buf->Size) {
    buf->WPos = 0;
    buf->Overflow = 1;
  }
}

void UART_CircularBuffer_moveReadPos(UART_CircularBuffer* buf, int16_t step) {
  buf->RPos += step;
  if (buf->RPos >= buf->Size) {
    buf->RPos = 0;
    buf->Overflow = 0;
  }
}
int16_t UART_CircularBuffer_directAvailable(UART_CircularBuffer* buf) {
  return buf->Overflow ? buf->Size - buf->RPos :
                         buf->WPos - buf->RPos;
}
int16_t UART_CircularBuffer_directSpace(UART_CircularBuffer* buf) {
  return buf->Overflow ? buf->RPos - buf->WPos :
                         buf->Size - buf->WPos;
}
uint8_t* UART_CircularBuffer_getReadDataPtr(UART_CircularBuffer* buf) {
  return &buf->Buffer[buf->RPos];
}
uint8_t* UART_CircularBuffer_getWriteDataPtr(UART_CircularBuffer* buf) {
  return &buf->Buffer[buf->WPos];
}

int16_t UART_CircularBuffer_findByte(UART_CircularBuffer* buf, uint8_t val) {
  if (UART_CircularBuffer_available(buf) <= 0) {
    return -1;
  }
  
  int16_t offset = 0;
  int16_t len = UART_CircularBuffer_directAvailable(buf);
  uint8_t* base = UART_CircularBuffer_getReadDataPtr(buf);
  uint8_t* p = memchr(base, val, len);
  
  if (p == NULL && buf->Overflow) {
    offset = len;
    base = buf->Buffer;
    len = buf->WPos;
    p = memchr(base, val, len);
  }
  
  return p == NULL ? -1 : (int16_t)(p - base) + offset;
}

// --------------------- UART Rx APIs --------------------------
void UART_CircularBuffer_receive(UART_CircularBuffer* buf) {
  if (!buf->InReceive) {
    buf->PendingBytes = UART_CircularBuffer_directSpace(buf);
    if (buf->PendingBytes > 0) {
      buf->InReceive = 1;
      // Check IT or DMA
      if (buf->HUART->hdmarx) {
        HAL_UART_Receive_DMA(
            buf->HUART, 
            UART_CircularBuffer_getWriteDataPtr(buf), 
            buf->PendingBytes
        );
      }
      else {
        HAL_UART_Receive_IT(
            buf->HUART, 
            UART_CircularBuffer_getWriteDataPtr(buf), 
            buf->PendingBytes
        );
      }
    }
  }
}
void UART_CircularBuffer_handleRx(UART_CircularBuffer* buf) {
  if (buf->InReceive) {
    buf->InReceive = 0;
    UART_CircularBuffer_moveWritePos(buf, buf->PendingBytes);
    UART_CircularBuffer_receive(buf);
  }
}
// --------------------- UART Tx APIs --------------------------
void UART_CircularBuffer_transmit(UART_CircularBuffer* buf) {
  if (!buf->InTransmit) {
    buf->PendingBytes = UART_CircularBuffer_directAvailable(buf);
    if (buf->PendingBytes > 0) {
      buf->InTransmit = 1;
      // Check IT or DMA
      if (buf->HUART->hdmatx) {
        HAL_UART_Transmit_DMA(
            buf->HUART, 
            UART_CircularBuffer_getReadDataPtr(buf), 
            buf->PendingBytes
        );
      }
      else {
        HAL_UART_Transmit_IT(
            buf->HUART, 
            UART_CircularBuffer_getReadDataPtr(buf), 
            buf->PendingBytes
        );
      }
    }
  }
}
void UART_CircularBuffer_handleTx(UART_CircularBuffer* buf) {
  if (buf->InTransmit) {
    buf->InTransmit = 0;
    UART_CircularBuffer_moveReadPos(buf, buf->PendingBytes);
    UART_CircularBuffer_transmit(buf);
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
