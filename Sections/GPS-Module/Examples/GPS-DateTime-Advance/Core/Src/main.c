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
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "ALCD.h"
#include "CircularBuffer.h"
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct {
  uint8_t     Hours;
  uint8_t     Minutes;
  uint8_t     Seconds;
  uint16_t    SubSeconds;
} GPS_UTC_Time;

typedef struct {
  uint8_t     Year;
  uint8_t     Month;
  uint8_t     Day;
} GPS_UTC_Date;


typedef void (*GPS_MessageHandleFn)(char* line);

typedef struct {
  const char*           MessageId;
  GPS_MessageHandleFn   process;
} GPS_MessageHandle;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define GPS_MSG_OFFSET         3
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
static uint8_t uart3RxBuf[1024];
static UART_CircularBuffer uart3Rx;

static GPS_UTC_Time gpsTime = {0};
static GPS_UTC_Date gpsDate = {0};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void GPS_process(char* line);
void GPS_RMC_process(char* line);
void GPS_GGA_process(char* line);
void GPS_GLL_process(char* line);
void GPS_GSV_process(char* line);


char* Str_indexOfAt(char* str, char c, int16_t num);
uint32_t Str_convertUNumFix(char* str, int16_t len);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#define GPS_HANDLE_INIT(T)        { #T, GPS_ ##T ##_process }
#define ARR_LEN(ARR)              sizeof(ARR) / sizeof(ARR[0])

static const GPS_MessageHandle GPS_MESSAGE_HANDLES[] = {
  GPS_HANDLE_INIT(RMC),
  GPS_HANDLE_INIT(GGA),
  GPS_HANDLE_INIT(GLL),
  GPS_HANDLE_INIT(GSV),
};
static const uint16_t GPS_MESSAGE_HANDLES_LEN = ARR_LEN(GPS_MESSAGE_HANDLES);
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
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
  ALCD_init(16, 2);
  ALCD_clear();
  
  
  UART_CircularBuffer_init(&uart3Rx, &huart3, uart3RxBuf, sizeof(uart3RxBuf));
  UART_CircularBuffer_receive(&uart3Rx);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    // Phase 1: Read Line
    if (UART_CircularBuffer_available(&uart3Rx) > 0) {
      int16_t idx = UART_CircularBuffer_findByte(&uart3Rx, '\n');
      if (idx >= 0) {
        char line[128];
        idx++;
        UART_CircularBuffer_readBytes(&uart3Rx, (uint8_t*) line, idx);
        line[idx] = '\0';
        // Process Line
        // Phase 2: Detect Message
        GPS_process(line);
      }
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
void GPS_RMC_process(char* line) {
  char* start;
  char* end;
  // Get Time
  start = Str_indexOfAt(line, ',', 1);
  if (start) {
    start++;
    // hhmmss.sss
    gpsTime.Hours   = Str_convertUNumFix(&start[0], 2);
    gpsTime.Minutes = Str_convertUNumFix(&start[2], 2);
    gpsTime.Seconds = Str_convertUNumFix(&start[4], 2);
    gpsTime.SubSeconds = Str_convertUNumFix(&start[7], 3);
  }
  // Get Date
  start = Str_indexOfAt(line, ',', 9);
  if (start) {
    start++;
    // ddmmyy
    gpsDate.Day     = Str_convertUNumFix(&start[0], 2);
    gpsDate.Month   = Str_convertUNumFix(&start[2], 2);
    gpsDate.Year    = Str_convertUNumFix(&start[4], 2);
  }
  
  // Update time
  ALCD_printfXY(2, 0, "%02u:%02u:%02u.%03u", gpsTime.Hours, gpsTime.Minutes, gpsTime.Seconds, gpsTime.SubSeconds);
  ALCD_printfXY(4, 1, "%02u/%02u/%02u", gpsDate.Year, gpsDate.Month, gpsDate.Day);
}

void GPS_GGA_process(char* line) {
  //ALCD_clear();
  //ALCD_putsXY(0, 1, "GGA");
}
void GPS_GLL_process(char* line) {
  
}
void GPS_GSV_process(char* line) {
  
}
void GPS_process(char* line) {
  // Check '$'
  if (line[0] != '$') {
    return;
  }
  // Find Message
  // Simple Way
  /*for (uint8_t i = 0; i < GPS_MESSAGE_HANDLES_LEN; i++) {
    if (strncmp(&line[GPS_MSG_OFFSET], GPS_MESSAGE_HANDLES[i].MessageId, 3) == 0) {
      GPS_MESSAGE_HANDLES[i].process(line);
      break;
    }
  }*/
  // Faster Way, Linear Search
  const GPS_MessageHandle* pHandle = GPS_MESSAGE_HANDLES;
  uint16_t len = GPS_MESSAGE_HANDLES_LEN;
  while (len-- > 0) {
    if (strncmp(&line[GPS_MSG_OFFSET], pHandle->MessageId, 3) == 0) {
      pHandle->process(line);
      break;
    }
    pHandle++;
  }
}


char* Str_indexOfAt(char* str, char c, int16_t num) {
  if (*str == c) {
    num--;
  }
  
  while (num-- > 0) {
    str = strchr(str + 1, c);
  }
  
  return str;
}

uint32_t Str_convertUNumFix(char* str, int16_t len) {
  uint32_t z = 0;
  
  while (*str != NULL && len-- > 0) {
    if (*str < '0' || *str > '9') {
      break;
    }
    
    z = (z * 10) + (*str - 0x30);
    str++;
  }
  
  return z;
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef* huart) {
  switch ((uint32_t) huart->Instance) {
    case USART3_BASE:
      UART_CircularBuffer_resetIO(&uart3Rx);
      UART_CircularBuffer_receive(&uart3Rx);
      break;
  }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef* huart) {
  switch ((uint32_t) huart->Instance) {
    case USART3_BASE:
      UART_CircularBuffer_handleRx(&uart3Rx);
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
