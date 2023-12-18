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


// ASCII Frame
// <SOF><Data><EOF>
// <Data><EOF>
// <EOF> -> <CR><LF>
//
// Binary Frame
// <SOF><Data><EOF>
// <SOF><Data>

// -------------- My Frame -----------------
//  ____________________________________
// | Sync (4x) | Type (1x) | Params (N) |
// |___________|___________|____________|
// 
// Sync: 0xAABBCCDD (BE) -> { 0xAA, 0xBB, 0xCC, 0xDD }
// 
// Type:
// - None       (0x00)
// - Unknown    (0xFF)
// - Led-Set    (0x01)
// - Led-Get    (0x02)
// - Led-Status (0x03)
//
// Led-Set:
//  _______________________________
// | Led Num (1x) | Led State (1x) |
// |______________|________________|
//
// Led-Get:
//  ______________
// | Led Num (1x) |
// |______________|
//
// Led-Status:
//  _______________________________
// | Led Num (1x) | Led State (1x) |
// |______________|________________|
// 

typedef enum {
  Frame_Type_None       = 0x00,
  Frame_Type_Unknown    = 0xFF,
  Frame_Type_LedSet     = 0x01,
  Frame_Type_LedGet     = 0x02,
  Frame_Type_LedStatus  = 0x03,
} Frame_Type;

typedef struct {
  uint8_t               LedNum;
  uint8_t               LedState;
} Params_LedSet;

typedef struct {
  uint8_t               LedNum;
} Params_LedGet;

typedef struct {
  uint8_t               LedNum;
  uint8_t               LedState;
} Params_LedStatus;

typedef union {
  Params_LedSet         LedSet;
  Params_LedGet         LedGet;
  Params_LedStatus      LedStatus;
} Frame_Params;

typedef struct {
  uint8_t               Type;
  Frame_Params          Params;
} Frame;

typedef enum {
  FrameParser_State_Sync,
  FrameParser_State_Type,
  FrameParser_State_Params,
} FrameParser_State;

typedef struct {
  uint16_t              RemainBytes;
  uint16_t              ParamsSize;
  FrameParser_State     State;
} FrameParser;

typedef struct {
  GPIO_TypeDef*         GPIO;
  uint16_t              Pin;
} Led_PinConfig;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
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
static const uint8_t LED_CONFIGS_LEN = sizeof(LED_CONFIGS) / sizeof(LED_CONFIGS[0]);

static const uint8_t FRAME_SYNC[4] = { 0xAA, 0xBB, 0xCC, 0xDD };


static uint8_t uart1TxBuf[100];
static UART_CircularBuffer uart1Tx;

static uint8_t uart1RxBuf[100];
static UART_CircularBuffer uart1Rx;

static FrameParser frameParser;
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
UART_CircularBuffer_Result UART_CircularBuffer_getBytes(UART_CircularBuffer* buf, uint8_t* data, uint32_t len);

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

// ---------- Parser APIs -------------

uint8_t FrameParser_parse(UART_CircularBuffer* buf, FrameParser* parser, Frame* frame);
int16_t paramsSize(Frame_Type type);

void FrameParser_init(FrameParser* parser);

void sendFrame(UART_CircularBuffer* buf, Frame* frame);

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
  UART_CircularBuffer_init(&uart1Rx, &huart1, uart1RxBuf, sizeof(uart1RxBuf));
  
  FrameParser_init(&frameParser);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  UART_CircularBuffer_receive(&uart1Rx);
  while (1)
  {
    if (UART_CircularBuffer_available(&uart1Rx) > frameParser.RemainBytes) {
      // find frame
      Frame frame;
      Frame txFrame;
      if (FrameParser_parse(&uart1Rx, &frameParser, &frame)) {
        // process frame
        switch ((Frame_Type) frame.Type) {
          case Frame_Type_LedSet:
            if (frame.Params.LedSet.LedNum < LED_CONFIGS_LEN) {
              HAL_GPIO_WritePin(
                LED_CONFIGS[frame.Params.LedSet.LedNum].GPIO, 
                LED_CONFIGS[frame.Params.LedSet.LedNum].Pin, 
                frame.Params.LedSet.LedState
              );
              txFrame.Type = Frame_Type_None;
              sendFrame(&uart1Tx, &txFrame);
            }
            else {
              txFrame.Type = Frame_Type_Unknown;
              sendFrame(&uart1Tx, &txFrame);
            }
          
            /*switch (frame.Params.LedSet.LedNum) {
              case 0:
                HAL_GPIO_WritePin(LED0_GPIO_Port, LED0_Pin, frame.Params.LedSet.LedState);
                break;
              case 1:
                HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, frame.Params.LedSet.LedState);
                break;
              case 2:
                HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, frame.Params.LedSet.LedState);
                break;
            }*/
            break;
          case Frame_Type_LedGet:
            // Read Led Status
            txFrame.Type = Frame_Type_LedStatus;
            txFrame.Params.LedStatus.LedState = (LED_CONFIGS[frame.Params.LedGet.LedNum].GPIO->ODR & LED_CONFIGS[frame.Params.LedGet.LedNum].Pin) != 0;
            sendFrame(&uart1Tx, &txFrame);
            break;
          case Frame_Type_LedStatus:
            // No need process
            txFrame.Type = Frame_Type_Unknown;
            sendFrame(&uart1Tx, &txFrame);
            break;
          case Frame_Type_None:
            txFrame.Type = Frame_Type_Unknown;
            sendFrame(&uart1Tx, &txFrame);
            break;
          case Frame_Type_Unknown:
            txFrame.Type = Frame_Type_Unknown;
            sendFrame(&uart1Tx, &txFrame);
            break;
          default:
            txFrame.Type = Frame_Type_Unknown;
            sendFrame(&uart1Tx, &txFrame);
            break;
        }
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
void sendFrame(UART_CircularBuffer* buf, Frame* frame) {
  // Write SYNC 
  UART_CircularBuffer_writeBytes(buf, (uint8_t*) FRAME_SYNC, sizeof(FRAME_SYNC));
  // Write Type
  UART_CircularBuffer_writeBytes(buf, (uint8_t*) &frame->Type, sizeof(frame->Type));
  // Write Params
  UART_CircularBuffer_writeBytes(buf, (uint8_t*) &frame->Params, paramsSize(frame->Type));
  // Flush
  UART_CircularBuffer_transmit(buf);
}


uint8_t FrameParser_parse(UART_CircularBuffer* buf, FrameParser* parser, Frame* frame) {
  uint8_t temp[sizeof(FRAME_SYNC)] = {0};
  int16_t index;
  
  switch (parser->State) {
    case FrameParser_State_Sync:
      while (UART_CircularBuffer_available(buf) > sizeof(FRAME_SYNC) &&
             (index = UART_CircularBuffer_findByte(buf, FRAME_SYNC[0])) >= 0
      ) {
        // ignore bytes
        if (index > 0) {
          UART_CircularBuffer_moveReadPos(buf, index);
        }
        // read 4 bytes for pattern
        if (UART_CircularBuffer_getBytes(buf, temp, sizeof(temp)) == UART_CircularBuffer_Result_Ok) {
          // check pattern
          if (memcmp(FRAME_SYNC, temp, sizeof(FRAME_SYNC)) == 0) {
            // Ignore pattern bytes
            UART_CircularBuffer_moveReadPos(buf, sizeof(FRAME_SYNC));
            // sync found
            parser->State = FrameParser_State_Type;
            parser->RemainBytes = sizeof(frame->Type) - 1;
            parser->ParamsSize = 0;
            break;
          }
          else {
            // Ignore wrong byte
            UART_CircularBuffer_moveReadPos(buf, 1);
          }
        }
      }
      break;
    case FrameParser_State_Type:
      // Read Type
      UART_CircularBuffer_readBytes(buf, &frame->Type, sizeof(frame->Type));
      // Read Params
      parser->ParamsSize = paramsSize(frame->Type);
      // set parser
      if (parser->ParamsSize > 0) {
        parser->State = FrameParser_State_Params;
        parser->RemainBytes = parser->ParamsSize - 1;
      }
      else {
        parser->State = FrameParser_State_Sync;
        parser->RemainBytes = sizeof(FRAME_SYNC) - 1;
        return 1;
      }
      break;
    case FrameParser_State_Params:
      // Read params
      UART_CircularBuffer_readBytes(buf, (uint8_t*) &frame->Params, parser->ParamsSize);
      // set parser
      parser->State = FrameParser_State_Sync;
      parser->RemainBytes = sizeof(FRAME_SYNC) - 1;
      return 1;
      break;
    default:
      parser->State = FrameParser_State_Sync;
      parser->RemainBytes = sizeof(FRAME_SYNC) - 1;
      break;
  }
  
  return 0;
}

int16_t paramsSize(Frame_Type type) {
  switch (type) {
    case Frame_Type_LedSet:
      return sizeof(Params_LedSet);
    case Frame_Type_LedGet:
      return sizeof(Params_LedGet);
    case Frame_Type_LedStatus:
      return sizeof(Params_LedStatus);
    case Frame_Type_Unknown:
    case Frame_Type_None:
    default:
      return 0;
  }
}

void FrameParser_init(FrameParser* parser) {
  parser->ParamsSize = 0;
  parser->RemainBytes = sizeof(FRAME_SYNC);
  parser->State = FrameParser_State_Sync;
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef* huart) {
  switch ((uint32_t) huart->Instance) {
    case USART1_BASE:
      UART_CircularBuffer_handleTx(&uart1Tx);
      break;
    default:
      
      break;
  }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef* huart) {
  switch ((uint32_t) huart->Instance) {
    case USART1_BASE:
      UART_CircularBuffer_handleRx(&uart1Rx);
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
UART_CircularBuffer_Result UART_CircularBuffer_getBytes(UART_CircularBuffer* buf, uint8_t* data, uint32_t len) {
  if (UART_CircularBuffer_available(buf) < len) {
    return UART_CircularBuffer_Result_NoAvailable;
  }
  
  int16_t rpos = buf->RPos;
  
  if (rpos + len >= buf->Size) {
    int16_t tmpLen = buf->Size - rpos;
    
    memcpy(data, &buf->Buffer[rpos], tmpLen);
    data += tmpLen;
    len -= tmpLen;
    // rpos = (rpos + tmpLen) % buf->Size
    rpos = 0;
    buf->Overflow = 0;
  }
  
  if (len > 0) {
    memcpy(data, &buf->Buffer[rpos], len);
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
