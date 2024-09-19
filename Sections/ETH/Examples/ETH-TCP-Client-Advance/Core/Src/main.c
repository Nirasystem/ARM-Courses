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
#include "dma.h"
#include "lwip.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "lwip/tcp.h"
#include "lwip/ip.h"
#include "lwip/ip_addr.h"

#include "ALCD.h"
#include "CircularBuffer.h"

#include <string.h>
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct {
  GPIO_TypeDef*       GPIO;
  uint16_t            Pin;
} Led_PinConfig;

typedef struct {
  ip_addr_t           IP;
  uint16_t            Port;
} Server_Config;


typedef struct {
  struct tcp_pcb*       Context;
  const Server_Config*  Server;
  const Led_PinConfig*  Led;
  uint32_t              NextConnectionCheck;
} Client;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MAX_CLIENT                3
#define TCP_CONN_RETRY_TIMEOUT    1000
#define TCP_MAX_PACKET            1460
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
static const Server_Config SERVER_CONFIGS[] = {
  { .IP = IPADDR4_INIT_BYTES(192, 168, 100, 100), .Port = 8585 },
  { .IP = IPADDR4_INIT_BYTES(192, 168, 100, 100), .Port = 8080 },
  { .IP = IPADDR4_INIT_BYTES(192, 168, 100, 100), .Port = 9595 },
};
static const uint16_t SERVER_CONFIGS_LEN = sizeof(SERVER_CONFIGS) / sizeof(SERVER_CONFIGS[0]);

static const Led_PinConfig LED_CONFIGS[] = {
  { LED0_GPIO_Port, LED0_Pin },
  { LED1_GPIO_Port, LED1_Pin },
  { LED2_GPIO_Port, LED2_Pin },
};
static const uint16_t LED_CONFIGS_LEN = sizeof(LED_CONFIGS) / sizeof(LED_CONFIGS[0]);

static Client clients[MAX_CLIENT] = {0};

static UART_CircularBuffer txUART1;
static UART_CircularBuffer rxUART1;
static uint8_t             txBuff1[1024];
static uint8_t             rxBuff1[1024];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void Client_setConfig(Client* client, const Server_Config* server, const Led_PinConfig* led);

void Client_init(Client* client);
void Client_handle(Client* client);
void Client_handleArray(Client* clients, uint16_t len);

err_t Client_onConnect(void *arg, struct tcp_pcb *tpcb, err_t err);
err_t Client_onReceive(void *arg, struct tcp_pcb *tpcb, struct pbuf *p, err_t err);
err_t Client_onPoll(void *arg, struct tcp_pcb *tpcb);
void  Client_onError(void *arg, err_t err);

void Client_broadcast(uint8_t* buf, uint16_t len);
void Client_close(Client* client);
uint16_t Client_getIdx(Client* client);
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
  MX_LWIP_Init();
  /* USER CODE BEGIN 2 */
  
  // Initialize ALCD
  ALCD_init(16, 2);
  ALCD_clear();
  
  // Initialize UART Buffer
  UART_CircularBuffer_init(&txUART1, &huart1, txBuff1, sizeof(txBuff1));
  UART_CircularBuffer_init(&rxUART1, &huart1, rxBuff1, sizeof(rxBuff1));
  UART_CircularBuffer_receive(&rxUART1);
  
  // Initialize Clients
  Client_setConfig(&clients[0], &SERVER_CONFIGS[0], &LED_CONFIGS[0]);
  Client_setConfig(&clients[1], &SERVER_CONFIGS[1], &LED_CONFIGS[1]);
  Client_setConfig(&clients[2], &SERVER_CONFIGS[2], &LED_CONFIGS[2]);
  
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    MX_LWIP_Process();
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    Client_handleArray(clients, MAX_CLIENT);
    
    if (UART_CircularBuffer_available(&rxUART1) > 0) {
      uint16_t len = UART_CircularBuffer_available(&rxUART1);
      
      if (len > TCP_MAX_PACKET) {
        len = TCP_MAX_PACKET;
      }        
      
      uint8_t buff[TCP_MAX_PACKET];
      
      UART_CircularBuffer_readBytes(&rxUART1, buff, len);
      Client_broadcast(buff, len);
    }
    
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
void Client_setConfig(Client* client, const Server_Config* server, const Led_PinConfig* led) {
  client->Server = server;
  client->Led = led;
}

void Client_init(Client* client) {
  if (client->Context != NULL) {
    return;
  }
  
  client->Context = tcp_new();
  if (client->Context) {
    tcp_arg(client->Context, client);
    tcp_err(client->Context, Client_onError);
    err_t err = tcp_connect(client->Context , &client->Server->IP, client->Server->Port, Client_onConnect);
    
    if (err != ERR_OK) {
      memp_free(MEMP_TCP_PCB, client->Context);
      client->Context = NULL;
    }      
  }
}
void Client_handle(Client* client) {
  if (client->NextConnectionCheck <= HAL_GetTick() &&
     (client->Context == NULL || 
     (client->Context != NULL && client->Context->state != ESTABLISHED))
  ) {
    client->NextConnectionCheck = HAL_GetTick() + TCP_CONN_RETRY_TIMEOUT;    
    Client_init(client);
  }
}
void Client_handleArray(Client* clients, uint16_t len) {
  while (len-- > 0) {
    Client_handle(clients++);
  }
}

err_t Client_onConnect(void *arg, struct tcp_pcb *tpcb, err_t err) {
  Client* client = (Client*) arg;
  
  if (tpcb && err == ERR_OK) {
    // Connected
    tcp_recv(tpcb, Client_onReceive);
    tcp_poll(tpcb, Client_onPoll, 1);
    // Led On
    HAL_GPIO_WritePin(client->Led->GPIO, client->Led->Pin, GPIO_PIN_RESET);
  }
  else {
    // Not Connected
    Client_close(client);
  }
  
  return ERR_OK;
}
err_t Client_onReceive(void *arg, struct tcp_pcb *tpcb, struct pbuf *p, err_t err) {
  Client* client = (Client*) arg;  
  
  if (tpcb && err == ERR_OK) {
    if (p) {
      // Data Received
      char* pChr = (char*) memchr(p->payload, ' ', p->len);
      if (pChr) {
        pChr++;
        uint32_t len = p->len - (uint32_t) ((uint8_t*) pChr - (uint8_t*) p->payload);
        if (strncmp((char*) p->payload, "alcd", 4) == 0) {
          ALCD_printfXY(0, 0, "Client: %u", Client_getIdx(client));
          uint32_t pad = 16 - len;
          ALCD_goto(0, 1);
          while (len-- > 0) {
            ALCD_putChar(*pChr++);
          }
          while (pad-- > 0) {
            ALCD_putChar(' ');
          }
        }
        else if (strncmp((char*) p->payload, "uart", 4) == 0) {
          UART_CircularBuffer_writeFmt(&txUART1, "Client: %u,", Client_getIdx(client));
          UART_CircularBuffer_writeBytes(&txUART1, (uint8_t*) pChr, len);
          UART_CircularBuffer_transmit(&txUART1);
        }
      }
      
      pbuf_free(p);
    }
    else {
      // Disconnected
      Client_close(client);
    }
  }
  
  return ERR_OK;
}

err_t Client_onPoll(void *arg, struct tcp_pcb *tpcb) {
  Client* client = (Client*) arg;
  
  HAL_GPIO_TogglePin(client->Led->GPIO, client->Led->Pin);
  
  return ERR_OK;
}
void  Client_onError(void *arg, err_t err) {
  Client* client = (Client*) arg;
  
  client->Context = NULL;
}

void Client_broadcast(uint8_t* buf, uint16_t len) {
  for (uint8_t i = 0; i < MAX_CLIENT; i++) {
    if (clients[i].Context != NULL) {
      tcp_write(clients[i].Context, buf, len, TCP_WRITE_FLAG_COPY);
    }
  }
}
void Client_close(Client* client) {
  if (client->Context) {
    tcp_close(client->Context);
    tcp_abort(client->Context);
    memp_free(MEMP_TCP_PCB, client->Context);
    client->Context = NULL;
    // Led off
    HAL_GPIO_WritePin(client->Led->GPIO, client->Led->Pin, GPIO_PIN_SET);
  }    
}

uint16_t Client_getIdx(Client* client) {
  return (uint16_t)(client - clients);
}

// ------------------------------- UART Callbacks ------------------------------

void HAL_UART_TxCpltCallback(UART_HandleTypeDef* huart) {
  switch ((uint32_t) huart->Instance) {
    case USART1_BASE:
      UART_CircularBuffer_handleTx(&txUART1);
      break;
  }
}
void HAL_UART_RxCpltCallback(UART_HandleTypeDef* huart) {
  switch ((uint32_t) huart->Instance) {
    case USART1_BASE:
      UART_CircularBuffer_handleRx(&rxUART1);
      break;
  }
}
void HAL_UART_ErrorCallback(UART_HandleTypeDef* huart) {
  switch ((uint32_t) huart->Instance) {
    case USART1_BASE:
      UART_CircularBuffer_resetIO(&txUART1);  
      UART_CircularBuffer_resetIO(&rxUART1);
      UART_CircularBuffer_receive(&rxUART1);
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
