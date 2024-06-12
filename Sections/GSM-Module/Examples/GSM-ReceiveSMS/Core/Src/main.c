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
#include "CircularBuffer.h"
#include <stdio.h>
#include <string.h>
#include <stdarg.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum {
  GSM_State_Config,
  GSM_State_Idle,
} GSM_State;

typedef enum {
  GSM_ConfigState_AT,
  GSM_ConfigState_ATE0,
  GSM_ConfigState_CMGF,
  GSM_ConfigState_CSMP,
  GSM_ConfigState_CNMI,
} GSM_ConfigState;

typedef struct {
  UART_CircularBuffer*      Rx;
  UART_CircularBuffer*      Tx;
  GSM_State                 State;
  GSM_ConfigState           ConfigState;
  uint8_t                   WaitForSend     : 1;
  uint8_t                   WaitForData     : 1;
  uint8_t                   WaitForResult   : 1;
  uint8_t                   InSendCommand   : 1;
  uint8_t                   Reserved        : 4;
} GSM;

typedef struct {
  GPIO_TypeDef*             GPIO;
  uint16_t                  Pin;
} Led_PinConfig;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define LED_CONFIGS_LEN       3
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
static const Led_PinConfig LED_CONFIGS[LED_CONFIGS_LEN] = {
  { LED0_GPIO_Port, LED0_Pin },
  { LED1_GPIO_Port, LED1_Pin },
  { LED2_GPIO_Port, LED2_Pin },
};

static char CRLF[2]        = "\r\n";
static char AT[]           = "AT";
static char AT_CME_ERROR[] = "+CME ERROR:";
static char AT_CMS_ERROR[] = "+CMS ERROR:";
static char AT_OK[]        = "OK";
static char AT_ERROR[]     = "ERROR";
static char AT_CMT[]       = "+CMT:";

static UART_CircularBuffer uart1Tx = {0};
static uint8_t uart1TxBuf[512];

static UART_CircularBuffer uart3Tx = {0};
static uint8_t uart3TxBuf[512];
static UART_CircularBuffer uart3Rx = {0};
static uint8_t uart3RxBuf[512];

static GSM gsm = {0};

static char phone[17] = {0};
static char message[150] = {0};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void GSM_init(GSM* gsm, UART_CircularBuffer* tx, UART_CircularBuffer* rx);
void GSM_process(GSM* gsm);

void smsProcess(void);

void GSM_sendStr(GSM* gsm, const char* str);
void GSM_sendFmt(GSM* gsm, const char* fmt, ...);

char* Str_ignoreWhitespace(char* str);
char* Str_indexOfAt(char* str, char c, int16_t num);
char* Str_getToken(char* str, char* out, char c, int16_t num);
uint32_t Str_getUNum(char* str, int16_t* len);

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
  MX_DMA_Init();
  MX_USART1_UART_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
  UART_CircularBuffer_init(&uart1Tx, &huart1, uart1TxBuf, sizeof(uart1TxBuf));
  UART_CircularBuffer_init(&uart3Tx, &huart3, uart3TxBuf, sizeof(uart3TxBuf));
  UART_CircularBuffer_init(&uart3Rx, &huart3, uart3RxBuf, sizeof(uart3RxBuf));
  UART_CircularBuffer_receive(&uart3Rx);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  GSM_init(&gsm, &uart3Tx, &uart3Rx);
  
  while (1)
  {
    GSM_process(&gsm);
    smsProcess();
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
void smsProcess(void) {
  if (message[0] != '\0') {
    // Process
    printf("\r\nPhone: %s\r\nMessage: %s\r\n", phone, message);
    // Process LED command
    // LED=<num>,<state>
    char* msg = Str_ignoreWhitespace(message);
    
    if (strncmp(msg, "LED", 3) == 0) {
      msg = msg + 3;
      if (*msg == '=') {
        msg++;
        int16_t len;
        uint8_t num;
        uint8_t state;
        
        num = Str_getUNum(msg, &len);
        msg += len + 1;
        state = Str_getUNum(msg, NULL);
        
        printf("Led Command: %u,%u\r\n", num, state);
        
        if (num < LED_CONFIGS_LEN) {
          HAL_GPIO_WritePin(LED_CONFIGS[num].GPIO, LED_CONFIGS[num].Pin, !state);
        }
      }
    }
    
    
    // End
    message[0] = '\0';
  }
}


void GSM_init(GSM* gsm, UART_CircularBuffer* tx, UART_CircularBuffer* rx) {
  gsm->State = GSM_State_Config;
  gsm->ConfigState = GSM_ConfigState_AT;
  gsm->Tx = tx;
  gsm->Rx = rx;
  gsm->WaitForData = 0;
  gsm->WaitForResult = 0;
  gsm->WaitForSend = 0;
}
void GSM_process(GSM* gsm) {
  if (gsm->State == GSM_State_Config && gsm->InSendCommand == 0) {
    switch (gsm->ConfigState) {
      case GSM_ConfigState_AT:
        GSM_sendStr(gsm, "AT\r\n");
        puts("Send AT\r");
        gsm->InSendCommand = 1;
        gsm->WaitForResult = 1;
        break;
      case GSM_ConfigState_ATE0:
        GSM_sendStr(gsm, "ATE0\r\n");
        puts("Send Echo Off\r");
        gsm->InSendCommand = 1;
        gsm->WaitForResult = 1;
        break;
      case GSM_ConfigState_CMGF:
        GSM_sendStr(gsm, "AT+CMGF=1\r\n");
        puts("Send Text Mode\r");
        gsm->InSendCommand = 1;
        gsm->WaitForResult = 1;
        break;
      case GSM_ConfigState_CSMP:
        GSM_sendStr(gsm, "AT+CSMP=17,167,0,0\r\n");
        puts("Send Text Config\r");
        gsm->InSendCommand = 1;
        gsm->WaitForResult = 1;
        break;
      case GSM_ConfigState_CNMI:
        GSM_sendStr(gsm, "AT+CNMI=2,2\r\n");
        puts("Send Sms Config\r");
        gsm->InSendCommand = 1;
        gsm->WaitForResult = 1;
        break;
    }
  }
  
  int16_t len = UART_CircularBuffer_available(gsm->Rx); 
  if (len > 0) {
    if (gsm->WaitForSend) {
      // Send Data Part
    }
    else {
      // Read Line
      int16_t len = UART_CircularBuffer_findPat(gsm->Rx, (uint8_t*) CRLF, sizeof(CRLF));
      if (len >= 0) {
        char buf[180];
        len += 2;
        UART_CircularBuffer_readBytes(gsm->Rx, (uint8_t*) buf, len);
        buf[len] = '\0';
        if (gsm->WaitForData) {
          // Process Notification with Param, Ex: CMT
          strcpy(message, buf);
          // End 
          gsm->WaitForData = 0;
        }
        else {
          char* line = buf;
          // Ignore Left side of string
          line = Str_ignoreWhitespace(line);
          if (strncmp(line, AT, sizeof(AT) - 1) != 0) {
            // Process Result
            if (gsm->WaitForResult) {
              if (strncmp(line, AT_OK, sizeof(AT_OK) - 1) == 0) {
                // Result Ok
                if (gsm->State == GSM_State_Config) {
                  gsm->InSendCommand = 0;
                  gsm->WaitForResult = 0;
                  switch (gsm->ConfigState) {
                    case GSM_ConfigState_AT:
                      gsm->ConfigState = GSM_ConfigState_ATE0;
                      break;
                    case GSM_ConfigState_ATE0:
                      gsm->ConfigState = GSM_ConfigState_CMGF;
                      break;
                    case GSM_ConfigState_CMGF:
                      gsm->ConfigState = GSM_ConfigState_CSMP;
                      break;
                    case GSM_ConfigState_CSMP:
                      gsm->ConfigState = GSM_ConfigState_CNMI;
                      break;
                    case GSM_ConfigState_CNMI:
                      gsm->ConfigState = GSM_ConfigState_AT;
                      gsm->State = GSM_State_Idle;
                      break;
                  }
                }
                else {
                  
                }
              }
              else if (strncmp(line, AT_ERROR, sizeof(AT_ERROR) - 1) == 0) {
                // Result Error
                if (gsm->State == GSM_State_Config) {
                  gsm->InSendCommand = 0;
                  gsm->WaitForResult = 0;
                }
                else {
                  
                }
              }
              else if (strncmp(line, AT_CME_ERROR, sizeof(AT_CME_ERROR) - 1) == 0) {
                // CME Error
              }
              else if (strncmp(line, AT_CMS_ERROR, sizeof(AT_CMS_ERROR) - 1) == 0) {
                // CMS Error
              }
              else {
                if (line[0] == '+') {
                  // URC
                  
                }
                else {
                  // Fixed Strings
                  
                }
              }
            }
            else {
              // Process URCs
              if (line[0] == '+') {
                // URC
                if (strncmp(line, AT_CMT, sizeof(AT_CMT) - 1) == 0) {
                  puts("CMT Received\n");
                  // Process Phone
                  Str_getToken(line, phone, '"', 1);
                  // Wait for another part
                  gsm->WaitForData = 1;
                }
              }
              else {
                // Fixed Strings
                
              }
            }
          }
          else {
            // Echo On
            
          }
        }       
      }
      else {
        // Empty Line
      }
    }
  }
}

void GSM_sendStr(GSM* gsm, const char* str) {
  UART_CircularBuffer_writeBytes(gsm->Tx, (uint8_t*) str, strlen(str));
  UART_CircularBuffer_transmit(gsm->Tx);
}
void GSM_sendFmt(GSM* gsm, const char* fmt, ...) {
  char buf[256];
  
  va_list args;
  va_start(args, fmt);
  uint32_t len = vsnprintf(buf, sizeof(buf) - 1, fmt, args);
  va_end(args);
  
  UART_CircularBuffer_writeBytes(gsm->Tx, (uint8_t*) buf, len);
  UART_CircularBuffer_transmit(gsm->Tx);
}

char* Str_ignoreWhitespace(char* str) {
  while (*str != '\0' && *str <= ' ') {
    str++;
  }
  return str;
}
char* Str_indexOfAt(char* str, char c, int16_t num) {
  if (*str == c) {
    num--;
  }
  
  while (str != NULL && num-- > 0) {
    str = strchr(str + 1, c);
  }
  
  return str;
}
char* Str_getToken(char* str, char* out, char c, int16_t num) {
  char* start = Str_indexOfAt(str, c, num);
  
  if (start) {
    char* end = strchr(++start, c);
    if (end == NULL) {
      end = memchr(start, '\0', 256);
    }
    int32_t len = (int32_t)(end - start);
    strncpy(out, start, len);
    out[len] = '\0';
    return out;
  }
  
  return NULL;
}

uint32_t Str_getUNum(char* str, int16_t* len) {
  uint32_t z = 0;
  char* start = str;
  
  while (*str != '\0') {
    if (*str > '9' || *str < '0') {
      break;
    }      
    
    z = (z * 10) + (*str++ - '0');
  }
  
  if (len) {
    *len = (int16_t)(str - start);
  }
  
  return z;
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef* huart) {
  switch ((uint32_t) huart->Instance) {
    case USART1_BASE:
      UART_CircularBuffer_handleTx(&uart1Tx);
      break;
    case USART3_BASE:
      UART_CircularBuffer_handleTx(&uart3Tx);
      break;
  }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef* huart) {
  switch ((uint32_t) huart->Instance) {
    case USART1_BASE:
      
      break;
    case USART3_BASE:
      UART_CircularBuffer_handleRx(&uart3Rx);
      break;
  }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef* huart) {
  switch ((uint32_t) huart->Instance) {
    case USART1_BASE:
      UART_CircularBuffer_resetIO(&uart1Tx);
      break;
    case USART3_BASE:
      UART_CircularBuffer_resetIO(&uart3Rx);
      UART_CircularBuffer_resetIO(&uart3Tx);
      UART_CircularBuffer_receive(&uart3Rx);
      break;
  }
}
// Override fputc
int fputc(int c, FILE* f) {
  UART_CircularBuffer_writeBytes(&uart1Tx, (uint8_t*) &c, 1);
  UART_CircularBuffer_transmit(&uart1Tx);
  return 0;
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
