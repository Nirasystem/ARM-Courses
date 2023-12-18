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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stdarg.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define ALCD_RS_GPIO                      GPIOD
#define ALCD_RS_PIN                       GPIO_PIN_11

#define ALCD_RW_GPIO                      GPIOD
#define ALCD_RW_PIN                       GPIO_PIN_10

#define ALCD_EN_GPIO                      GPIOD
#define ALCD_EN_PIN                       GPIO_PIN_7

#define ALCD_D4_GPIO                      GPIOD
#define ALCD_D4_PIN                       GPIO_PIN_15

#define ALCD_D5_GPIO                      GPIOD
#define ALCD_D5_PIN                       GPIO_PIN_14

#define ALCD_D6_GPIO                      GPIOD
#define ALCD_D6_PIN                       GPIO_PIN_13

#define ALCD_D7_GPIO                      GPIOD
#define ALCD_D7_PIN                       GPIO_PIN_12

#define ALCD_CLK_ENABLE()                 __HAL_RCC_GPIOD_CLK_ENABLE()

#define ALCD_Delay_us(US)                 Delay_us((US))
#define ALCD_Delay_ms(MS)                 Delay_ms((MS))

#define ALCD_EN_HIGH_TIME                 20 // us
#define ALCD_EN_LOW_TIME                  20 // us

#define ALCD_PRINTF_BUF_SIZE              40

// ---------------- ALCD Commands Table -------------------

#define ALCD_CMD_CLEAR_DISPLAY            0x01
#define ALCD_CMD_CLEAR_DISPLAY_TIMEOUT    2000  // us

#define ALCD_CMD_RETURN_HOME              0x02
#define ALCD_CMD_RETURN_HOME_TIMEOUT      2000  // us

#define ALCD_CMD_ENTRY_MODE_SET           0x04
#define ALCD_CMD_ENTRY_MODE_SET_TIMEOUT   50    // us

#define ALCD_CMD_DISPLAY_CONTROL          0x08
#define ALCD_CMD_DISPLAY_CONTROL_TIMEOUT  50    // us

#define ALCD_CMD_SHIFT                    0x10
#define ALCD_CMD_SHIFT_TIMEOUT            50    // us

#define ALCD_CMD_FUNCTION_SET             0x20 
#define ALCD_CMD_FUNCTION_SET_TIMEOUT     50    // us

#define ALCD_CMD_SET_CGRAM                0x40
#define ALCD_CMD_SET_CGRAM_TIMEOUT        50    // us

#define ALCD_CMD_SET_DDRAM                0x80
#define ALCD_CMD_SET_DDRAM_TIMEOUT        50    // us

#define ALCD_CMD_WRITE_DATA               50    // us

// -------------------- ALCD Flags ---------------------

#define ALCD_FLAG_ENTRY_MODE_INCREMENT              0x02
#define ALCD_FLAG_ENTRY_MODE_DECREMENT              0x00
#define ALCD_FLAG_ENTRY_MODE_SHIFT                  0x01

#define ALCD_FLAG_DISPLAY_CONTROL_ON                0x04
#define ALCD_FLAG_DISPLAY_CONTROL_OFF               0x00
#define ALCD_FLAG_DISPLAY_CONTROL_CURSOR_ON         0x02
#define ALCD_FLAG_DISPLAY_CONTROL_CURSOR_OFF        0x00
#define ALCD_FLAG_DISPLAY_CONTROL_BLINK_ON          0x01
#define ALCD_FLAG_DISPLAY_CONTROL_BLINK_OFF         0x00

#define ALCD_FLAG_FUNCTION_SET_8BIT                 0x10
#define ALCD_FLAG_FUNCTION_SET_4BIT                 0x00
#define ALCD_FLAG_FUNCTION_SET_2LINE                0x08
#define ALCD_FLAG_FUNCTION_SET_1LINE                0x00
#define ALCD_FLAG_FUNCTION_SET_5_10_DOT             0x04
#define ALCD_FLAG_FUNCTION_SET_5_8_DOT              0x00

typedef enum {
  ALCD_RS_State_Command   = 0,
  ALCD_RS_State_Data      = 1,
} ALCD_RS_State;

typedef union {
  uint8_t           Value;
  struct {
    uint8_t         D0  : 1;
    uint8_t         D1  : 1;
    uint8_t         D2  : 1;
    uint8_t         D3  : 1;
    uint8_t         D4  : 1;
    uint8_t         D5  : 1;
    uint8_t         D6  : 1;
    uint8_t         D7  : 1;
  };
} ALCD_Byte;

typedef struct {
  int8_t            X;
  int8_t            Y;
  int8_t            Rows;
  int8_t            Columns;
} ALCD_Handle;

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
static ALCD_Handle alcdHandle = {0};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void Delay_us(uint32_t us);
void Delay_ms(uint32_t ms);

void ALCD_init(uint8_t col, uint8_t row);
void ALCD_initPins(void);

void ALCD_write(ALCD_RS_State rs, uint8_t val, uint32_t timeout);
void ALCD_write4Bit(ALCD_RS_State rs, uint8_t val);
void ALCD_trigger(void);

void ALCD_clear(void);

void ALCD_goto(uint8_t x, uint8_t y);
void ALCD_putChar(char c);
void ALCD_puts(const char* str);
void ALCD_putNum(int32_t num);
void ALCD_printf(const char* fmt, ...);

void ALCD_putCharXY(uint8_t x, uint8_t y, char c);
void ALCD_putsXY(uint8_t x, uint8_t y, const char* str);
void ALCD_putNumXY(uint8_t x, uint8_t y, int32_t num);
void ALCD_printfXY(uint8_t x, uint8_t y, const char* fmt, ...);

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
  /* USER CODE BEGIN 2 */
  ALCD_init(16, 2);
  /* USER CODE END 2 */
  
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  ALCD_putNumXY(4, 0, 12345);
  ALCD_goto(0, 1);
  ALCD_printfXY(1, 1, "Hello:%d", 111);
  while (1)
  {
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
void ALCD_init(uint8_t col, uint8_t row) {
  ALCD_initPins();
  
  // Wait 40ms for rise up
  ALCD_Delay_ms(40);
  // Send 4Bit command: 8Bit Interface
  ALCD_write4Bit(
    ALCD_RS_State_Command, 
    ALCD_CMD_FUNCTION_SET | ALCD_FLAG_FUNCTION_SET_8BIT
  );
  // Wait 5ms 
  ALCD_Delay_ms(5);
  // Send 4Bit command : 8Bit Interface
  ALCD_write4Bit(
    ALCD_RS_State_Command, 
    ALCD_CMD_FUNCTION_SET | ALCD_FLAG_FUNCTION_SET_8BIT
  );
  // Wait 100us
  ALCD_Delay_us(100);
  // Send 4Bit command : 8Bit Interface
  ALCD_write4Bit(
    ALCD_RS_State_Command, 
    ALCD_CMD_FUNCTION_SET | ALCD_FLAG_FUNCTION_SET_8BIT
  );
  // Send 4Bit command : 4Bit Interface
  ALCD_write4Bit(
    ALCD_RS_State_Command, 
    ALCD_CMD_FUNCTION_SET | ALCD_FLAG_FUNCTION_SET_4BIT
  );
  
  // Send command: 4Bit Interface, 2Line, 5x8 dot 
  ALCD_write(
    ALCD_RS_State_Command, 
    ALCD_CMD_FUNCTION_SET | ALCD_FLAG_FUNCTION_SET_4BIT | 
    ALCD_FLAG_FUNCTION_SET_2LINE | ALCD_FLAG_FUNCTION_SET_5_8_DOT,
    ALCD_CMD_FUNCTION_SET_TIMEOUT
  );
  // Send command: Display off
  ALCD_write(
    ALCD_RS_State_Command, 
    ALCD_CMD_DISPLAY_CONTROL | ALCD_FLAG_DISPLAY_CONTROL_OFF,
    ALCD_CMD_DISPLAY_CONTROL_TIMEOUT
  );
  // Send command: Display clear
  ALCD_write(
    ALCD_RS_State_Command, 
    ALCD_CMD_CLEAR_DISPLAY,
    ALCD_CMD_CLEAR_DISPLAY_TIMEOUT
  );
  
  // Display On
  ALCD_write(
    ALCD_RS_State_Command, 
    ALCD_CMD_DISPLAY_CONTROL | ALCD_FLAG_DISPLAY_CONTROL_ON |
    ALCD_FLAG_DISPLAY_CONTROL_CURSOR_OFF | ALCD_FLAG_DISPLAY_CONTROL_BLINK_OFF,
    ALCD_CMD_DISPLAY_CONTROL_TIMEOUT
  );
  
  alcdHandle.Columns = col;
  alcdHandle.Rows = row;
  alcdHandle.X = 0;
  alcdHandle.Y = 0;
}
void ALCD_initPins(void) {
  GPIO_InitTypeDef init;
  
  // Clock Enable
  ALCD_CLK_ENABLE();
  
  init.Mode = GPIO_MODE_OUTPUT_PP;
  init.Pull = GPIO_NOPULL;
  init.Speed = GPIO_SPEED_HIGH;
  // init RS
  init.Pin = ALCD_RS_PIN;
  HAL_GPIO_Init(ALCD_RS_GPIO, &init);
  // init RW
  init.Pin = ALCD_RW_PIN;
  HAL_GPIO_Init(ALCD_RW_GPIO, &init);
  // init EN
  init.Pin = ALCD_EN_PIN;
  HAL_GPIO_Init(ALCD_EN_GPIO, &init);
  // init D4
  init.Pin = ALCD_D4_PIN;
  HAL_GPIO_Init(ALCD_D4_GPIO, &init);
  // init D5
  init.Pin = ALCD_D5_PIN;
  HAL_GPIO_Init(ALCD_D5_GPIO, &init);
  // init D6
  init.Pin = ALCD_D6_PIN;
  HAL_GPIO_Init(ALCD_D6_GPIO, &init);
  // init D7
  init.Pin = ALCD_D7_PIN;
  HAL_GPIO_Init(ALCD_D7_GPIO, &init);
}

void ALCD_write(ALCD_RS_State rs, uint8_t val, uint32_t timeout) {
  ALCD_Byte byte = { val };
  // RS LOW
  HAL_GPIO_WritePin(ALCD_RS_GPIO, ALCD_RS_PIN, (GPIO_PinState) rs);
  // RW LOW
  HAL_GPIO_WritePin(ALCD_RW_GPIO, ALCD_RW_PIN, GPIO_PIN_RESET);
  // Write High Nibble (4x Bit)
  HAL_GPIO_WritePin(ALCD_D7_GPIO, ALCD_D7_PIN, byte.D7);
  HAL_GPIO_WritePin(ALCD_D6_GPIO, ALCD_D6_PIN, byte.D6);
  HAL_GPIO_WritePin(ALCD_D5_GPIO, ALCD_D5_PIN, byte.D5);
  HAL_GPIO_WritePin(ALCD_D4_GPIO, ALCD_D4_PIN, byte.D4);
  // Trigger
  ALCD_trigger();
  // Write Low Nibble (4x Bit)
  HAL_GPIO_WritePin(ALCD_D7_GPIO, ALCD_D7_PIN, byte.D3);
  HAL_GPIO_WritePin(ALCD_D6_GPIO, ALCD_D6_PIN, byte.D2);
  HAL_GPIO_WritePin(ALCD_D5_GPIO, ALCD_D5_PIN, byte.D1);
  HAL_GPIO_WritePin(ALCD_D4_GPIO, ALCD_D4_PIN, byte.D0);
  // Trigger
  ALCD_trigger();
  // Wait
  ALCD_Delay_us(timeout);
}
void ALCD_write4Bit(ALCD_RS_State rs, uint8_t val) {
  ALCD_Byte byte = { val };
  // RS LOW
  HAL_GPIO_WritePin(ALCD_RS_GPIO, ALCD_RS_PIN, GPIO_PIN_RESET);
  // RW LOW
  HAL_GPIO_WritePin(ALCD_RW_GPIO, ALCD_RW_PIN, GPIO_PIN_RESET);
  // Write High Nibble (4x Bit)
  HAL_GPIO_WritePin(ALCD_D7_GPIO, ALCD_D7_PIN, byte.D7);
  HAL_GPIO_WritePin(ALCD_D6_GPIO, ALCD_D6_PIN, byte.D6);
  HAL_GPIO_WritePin(ALCD_D5_GPIO, ALCD_D5_PIN, byte.D5);
  HAL_GPIO_WritePin(ALCD_D4_GPIO, ALCD_D4_PIN, byte.D4);
  // Trigger
  ALCD_trigger();
}

void ALCD_trigger(void) {
  // Set HIGH
  HAL_GPIO_WritePin(ALCD_EN_GPIO, ALCD_EN_PIN, GPIO_PIN_SET);
  ALCD_Delay_us(ALCD_EN_HIGH_TIME);
  // Set LOW
  HAL_GPIO_WritePin(ALCD_EN_GPIO, ALCD_EN_PIN, GPIO_PIN_RESET);
  ALCD_Delay_us(ALCD_EN_LOW_TIME);
}

void ALCD_clear(void) {
  ALCD_write(
    ALCD_RS_State_Command, 
    ALCD_CMD_CLEAR_DISPLAY,
    ALCD_CMD_CLEAR_DISPLAY_TIMEOUT
  );
}

void ALCD_putChar(char c) {
  ALCD_write(ALCD_RS_State_Data, c, ALCD_CMD_WRITE_DATA);
  if (++alcdHandle.X >= alcdHandle.Columns) {
    alcdHandle.X = 0;
    if (++alcdHandle.Y >= alcdHandle.Rows) {
      alcdHandle.Y = 0;
    }
    ALCD_goto(alcdHandle.X, alcdHandle.Y);
  }
}

void ALCD_puts(const char* str) {
  while (*str != '\0') {
    ALCD_putChar(*str++);
  }
}
void ALCD_goto(uint8_t x, uint8_t y) {
  static const uint8_t LINE_ADDR[4] = {
    0x00, 0x40, 0x20, 0x60,
  };
  
  if (x >= alcdHandle.Columns) {
    x = 0;
    y++;
  }
  
  if (y >= alcdHandle.Rows) {
    y = 0;
  }
  
  ALCD_write(
    ALCD_RS_State_Command, 
    ALCD_CMD_SET_DDRAM | (LINE_ADDR[y] + x),
    ALCD_CMD_SET_DDRAM_TIMEOUT
  );
  
  alcdHandle.X = x;
  alcdHandle.Y = y;
}

void ALCD_putCharXY(uint8_t x, uint8_t y, char c) {
  ALCD_goto(x, y);
  ALCD_putChar(c);
}
void ALCD_putsXY(uint8_t x, uint8_t y, const char* str) {
  ALCD_goto(x, y);
  ALCD_puts(str);
}

void ALCD_putNum(int32_t num) {
  char temp[13];
  snprintf(temp, sizeof(temp) - 1, "%d", num);
  ALCD_puts(temp);
}

void ALCD_printf(const char* fmt, ...) {
  char temp[ALCD_PRINTF_BUF_SIZE];
  
  va_list args;
  
  va_start(args, fmt);
  vsnprintf(temp, sizeof(temp) - 1, fmt, args);
  va_end(args);
  ALCD_puts(temp);
}

void ALCD_putNumXY(uint8_t x, uint8_t y, int32_t num) {
  ALCD_goto(x, y);
  ALCD_putNum(num);
}
void ALCD_printfXY(uint8_t x, uint8_t y, const char* fmt, ...) {
  char temp[ALCD_PRINTF_BUF_SIZE];
  
  va_list args;
  
  va_start(args, fmt);
  vsnprintf(temp, sizeof(temp) - 1, fmt, args);
  va_end(args);
  
  ALCD_goto(x, y);
  ALCD_puts(temp);
}

void Delay_us(uint32_t us) {
  us = SystemCoreClock / 3000000 * us;
  
  // while (--us > 0) {
  //   __NOP();
  // }
  
  // __ARMCC_VERSION Mmmuuxx
#if __ARMCC_VERSION >= 6000000  
  __asm volatile (
    "1:              \n" // Label
    "subs %0, %0, #1 \n" // us--
    "bne 1b          \n" // Back to loop
    : "+r" (us)
    :
    : "cc"
  );
#else
  __asm {
    delay_us_loop:    // Label
    subs us, us, 1    // us--
    bne delay_us_loop // Back to loop
  };
#endif
}

void Delay_ms(uint32_t ms) {
  ms = SystemCoreClock / 3000 * ms;
  
  // while (--ms > 0) {
  //   __NOP();
  // }
  
  // __ARMCC_VERSION Mmmuuxx
#if __ARMCC_VERSION >= 6000000  
  __asm volatile (
    "1:              \n" // Label
    "subs %0, %0, #1 \n" // ms--
    "bne 1b          \n" // Back to loop
    : "+r" (ms)
    :
    : "cc"
  );
#else
  __asm {
    delay_ms_loop:    // Label
    subs ms, ms, 1    // ms--
    bne delay_ms_loop // Back to loop
  };
#endif
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
