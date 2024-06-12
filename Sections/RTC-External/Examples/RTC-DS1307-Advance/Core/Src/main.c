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
#include "i2c.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "ALCD.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct {
  uint8_t       Seconds;
  uint8_t       Minutes;
  uint8_t       Hours;
  uint8_t       WeekDay;
  uint8_t       Day;
  uint8_t       Month;
  uint8_t       Year;
  uint8_t       Control;
  uint8_t       RAM[56];
} DS1307_Map;

typedef struct {
  uint8_t       Seconds;
  uint8_t       Minutes;
  uint8_t       Hours;
} DS1307_Time;

typedef struct {
  uint8_t       WeekDay;
  uint8_t       Day;
  uint8_t       Month;
  uint8_t       Year;
} DS1307_Date;

typedef union {
  // Structure
  struct {
    DS1307_Time   Time;
    DS1307_Time   Date;
  };
  // Separated
  struct {
    uint8_t       Seconds;
    uint8_t       Minutes;
    uint8_t       Hours;
    uint8_t       WeekDay;
    uint8_t       Day;
    uint8_t       Month;
    uint8_t       Year;
  };  
} DS1307_DateTime;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define DS1307_DEV_ADDR           0xD0

//#define DS1307_ADDR_TIME          0
#define DS1307_ADDR_TIME          ((uint32_t) &((DS1307_Map*) 0)->Seconds)
//#define DS1307_ADDR_DATE          3
#define DS1307_ADDR_DATE          ((uint32_t) &((DS1307_Map*) 0)->WeekDay)
//#define DS1307_ADDR_DATE          0
#define DS1307_ADDR_DATE_TIME     ((uint32_t) &((DS1307_Map*) 0)->Seconds)

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

uint8_t RTC_isRunning(void);

HAL_StatusTypeDef RTC_setTime(DS1307_Time* v);
HAL_StatusTypeDef RTC_setDate(DS1307_Time* v);
HAL_StatusTypeDef RTC_setDateTime(DS1307_DateTime* v);

HAL_StatusTypeDef RTC_getTime(DS1307_Time* v);
HAL_StatusTypeDef RTC_getDate(DS1307_Time* v);
HAL_StatusTypeDef RTC_getDateTime(DS1307_DateTime* v);

void bcdToBin(uint8_t* dst, uint8_t* src, uint16_t len);
void binToBcd(uint8_t* dst, uint8_t* src, uint16_t len);

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
  DS1307_DateTime datetime;
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
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
  ALCD_init(16, 2);
  ALCD_clear();
  
  
  if (!RTC_isRunning()) {
    datetime.Year = 23;
    datetime.Month = 12;
    datetime.Day = 11;
    datetime.WeekDay = 3;
    datetime.Hours = 14;
    datetime.Minutes = 22;
    datetime.Seconds = 25;
    RTC_setDateTime(&datetime);
  }
  
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    RTC_getTime(&datetime.Time);
    ALCD_printfXY(4, 0, "%02u:%02u:%02u", datetime.Hours, datetime.Minutes, datetime.Seconds);
    HAL_Delay(1000);
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
uint8_t RTC_isRunning(void) {
  uint8_t buf[1];
  if (HAL_I2C_Mem_Read(&hi2c1, DS1307_DEV_ADDR, 0, I2C_MEMADD_SIZE_8BIT, buf, sizeof(buf), 1000) == HAL_OK) {
    return (buf[0] & 0x80) == 0;
  }
  else {
    return 0;
  }
}

HAL_StatusTypeDef RTC_setTime(DS1307_Time* v) { 
  DS1307_Time buf;
  binToBcd((uint8_t*) &buf, (uint8_t*) v, sizeof(buf));
  return HAL_I2C_Mem_Write(&hi2c1, DS1307_DEV_ADDR, DS1307_ADDR_TIME, I2C_MEMADD_SIZE_8BIT, (uint8_t*) &buf, sizeof(buf), 1000);
}
HAL_StatusTypeDef RTC_setDate(DS1307_Time* v) {
  DS1307_Date buf;
  binToBcd((uint8_t*) &buf, (uint8_t*) v, sizeof(buf));
  return HAL_I2C_Mem_Write(&hi2c1, DS1307_DEV_ADDR, DS1307_ADDR_DATE, I2C_MEMADD_SIZE_8BIT, (uint8_t*) &buf, sizeof(buf), 1000);
}
HAL_StatusTypeDef RTC_setDateTime(DS1307_DateTime* v) {
  DS1307_DateTime buf;
  binToBcd((uint8_t*) &buf, (uint8_t*) v, sizeof(buf));
  return HAL_I2C_Mem_Write(&hi2c1, DS1307_DEV_ADDR, DS1307_ADDR_DATE_TIME, I2C_MEMADD_SIZE_8BIT, (uint8_t*) &buf, sizeof(buf), 1000);
}

HAL_StatusTypeDef RTC_getTime(DS1307_Time* v) {
  HAL_StatusTypeDef result;
  result = HAL_I2C_Mem_Read(&hi2c1, DS1307_DEV_ADDR, DS1307_ADDR_TIME, I2C_MEMADD_SIZE_8BIT, (uint8_t*) v, sizeof(*v), 1000);
  if (result == HAL_OK) {
    bcdToBin((uint8_t*) v, (uint8_t*) v, sizeof(*v));
  }
  return result;
}
HAL_StatusTypeDef RTC_getDate(DS1307_Time* v) {
  HAL_StatusTypeDef result;
  result = HAL_I2C_Mem_Read(&hi2c1, DS1307_DEV_ADDR, DS1307_ADDR_DATE, I2C_MEMADD_SIZE_8BIT, (uint8_t*) v, sizeof(*v), 1000);
  if (result == HAL_OK) {
    bcdToBin((uint8_t*) v, (uint8_t*) v, sizeof(*v));
  }
  return result;
}
HAL_StatusTypeDef RTC_getDateTime(DS1307_DateTime* v) {
  HAL_StatusTypeDef result;
  result = HAL_I2C_Mem_Read(&hi2c1, DS1307_DEV_ADDR, DS1307_ADDR_DATE_TIME, I2C_MEMADD_SIZE_8BIT, (uint8_t*) v, sizeof(*v), 1000);
  if (result == HAL_OK) {
    bcdToBin((uint8_t*) v, (uint8_t*) v, sizeof(*v));
  }
  return result;
}

void bcdToBin(uint8_t* dst, uint8_t* src, uint16_t len) {
  while (len-- > 0) {
    *dst++ = (*src & 0x0F) + (((*src >> 4) & 0x0F) * 10);
    src++;
  }
}
void binToBcd(uint8_t* dst, uint8_t* src, uint16_t len) {
  while (len-- > 0) {
    *dst++ = (*src % 10) | ((*src / 10) << 4);
    src++;
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
