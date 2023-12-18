#ifndef _ALCD_H_
#define _ALCD_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/****************************************************************************/
/*                             Configuration                                */
/****************************************************************************/
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
/**
 * @brief If you want to use putNum function you must enable it first
 */
#define ALCD_SUPPORT_PUT_NUM              1
/**
 * @brief If you want to use printf function you must enable it first
 */
#define ALCD_SUPPORT_PRINTF               1
/**
 * @brief In case for change delay us with custom delay
 */
#define ALCD_Delay_us(US)                 __ALCD_Delay_us((US))
/**
 * @brief In case for change delay ms with custom delay
 */
#define ALCD_Delay_ms(MS)                 __ALCD_Delay_ms((MS))
/**
 * @brief Size of local buffer in printf
 */
#define ALCD_PRINTF_BUF_SIZE              40
/****************************************************************************/

void ALCD_init(uint8_t col, uint8_t row);

void ALCD_clear(void);

void ALCD_goto(uint8_t x, uint8_t y);
void ALCD_putChar(char c);
void ALCD_puts(const char* str);

void ALCD_putCharXY(uint8_t x, uint8_t y, char c);
void ALCD_putsXY(uint8_t x, uint8_t y, const char* str);

#if ALCD_SUPPORT_PUT_NUM 
  void ALCD_putNum(int32_t num);
  void ALCD_putNumXY(uint8_t x, uint8_t y, int32_t num);
#endif

#if ALCD_SUPPORT_PRINTF
  void ALCD_printf(const char* fmt, ...);
  void ALCD_printfXY(uint8_t x, uint8_t y, const char* fmt, ...);
#endif

// built-in delay for alcd

void __ALCD_Delay_us(uint32_t us);
void __ALCD_Delay_ms(uint32_t ms);

#ifdef __cplusplus
};
#endif

#endif // _ALCD_H_
