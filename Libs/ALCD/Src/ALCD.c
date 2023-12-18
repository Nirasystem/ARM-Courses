#include "ALCD.h"
#include "main.h"

#if ALCD_SUPPORT_PUT_NUM || ALCD_SUPPORT_PRINTF
  #include <stdio.h>
#endif
#if ALCD_SUPPORT_PRINTF
  #include <stdarg.h>
#endif

#define ALCD_EN_HIGH_TIME                 20    // us
#define ALCD_EN_LOW_TIME                  20    // us

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
} ALCD_RState;

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


static void ALCD_initPins(void);
static void ALCD_write(ALCD_RState rs, uint8_t val, uint32_t timeout);
static void ALCD_write4Bit(ALCD_RState rs, uint8_t val);
static void ALCD_trigger(void);


static ALCD_Handle alcdHandle = {0};

/**
 * @brief Initialize ALCD 
 * 
 * @param col number of columns, ex: 16, 20
 * @param row number of rows, ex: 1, 2, 4
 */
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
/**
 * @brief Initialize ALCD GPIO pins
 */
static void ALCD_initPins(void) {
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
/**
 * @brief write a byte into ALCD
 * 
 * @param rs determine it's a command or data byte
 * @param val value 
 * @param timeout delay after send out value
 */
static void ALCD_write(ALCD_RState rs, uint8_t val, uint32_t timeout) {
  ALCD_Byte byte = { val };
  // RS LOW
  HAL_GPIO_WritePin(ALCD_RS_GPIO, ALCD_RS_PIN, (GPIO_PinState) rs);
  // RW LOW
  HAL_GPIO_WritePin(ALCD_RW_GPIO, ALCD_RW_PIN, GPIO_PIN_RESET);
  // Write High Nibble (4x Bit)
  HAL_GPIO_WritePin(ALCD_D7_GPIO, ALCD_D7_PIN, (GPIO_PinState) byte.D7);
  HAL_GPIO_WritePin(ALCD_D6_GPIO, ALCD_D6_PIN, (GPIO_PinState) byte.D6);
  HAL_GPIO_WritePin(ALCD_D5_GPIO, ALCD_D5_PIN, (GPIO_PinState) byte.D5);
  HAL_GPIO_WritePin(ALCD_D4_GPIO, ALCD_D4_PIN, (GPIO_PinState) byte.D4);
  // Trigger
  ALCD_trigger();
  // Write Low Nibble (4x Bit)
  HAL_GPIO_WritePin(ALCD_D7_GPIO, ALCD_D7_PIN, (GPIO_PinState) byte.D3);
  HAL_GPIO_WritePin(ALCD_D6_GPIO, ALCD_D6_PIN, (GPIO_PinState) byte.D2);
  HAL_GPIO_WritePin(ALCD_D5_GPIO, ALCD_D5_PIN, (GPIO_PinState) byte.D1);
  HAL_GPIO_WritePin(ALCD_D4_GPIO, ALCD_D4_PIN, (GPIO_PinState) byte.D0);
  // Trigger
  ALCD_trigger();
  // Wait
  ALCD_Delay_us(timeout);
}
/**
 * @brief Write a 4-Bit into ALCD
 * 
 * @param rs determine it's a command or data byte
 * @param val value
 */
static void ALCD_write4Bit(ALCD_RState rs, uint8_t val) {
  ALCD_Byte byte = { val };
  // RS LOW
  HAL_GPIO_WritePin(ALCD_RS_GPIO, ALCD_RS_PIN, GPIO_PIN_RESET);
  // RW LOW
  HAL_GPIO_WritePin(ALCD_RW_GPIO, ALCD_RW_PIN, GPIO_PIN_RESET);
  // Write High Nibble (4x Bit)
  HAL_GPIO_WritePin(ALCD_D7_GPIO, ALCD_D7_PIN, (GPIO_PinState) byte.D7);
  HAL_GPIO_WritePin(ALCD_D6_GPIO, ALCD_D6_PIN, (GPIO_PinState) byte.D6);
  HAL_GPIO_WritePin(ALCD_D5_GPIO, ALCD_D5_PIN, (GPIO_PinState) byte.D5);
  HAL_GPIO_WritePin(ALCD_D4_GPIO, ALCD_D4_PIN, (GPIO_PinState) byte.D4);
  // Trigger
  ALCD_trigger();
}
/**
 * @brief Generate a falling edge on EN pin
 */
static void ALCD_trigger(void) {
  // Set HIGH
  HAL_GPIO_WritePin(ALCD_EN_GPIO, ALCD_EN_PIN, GPIO_PIN_SET);
  ALCD_Delay_us(ALCD_EN_HIGH_TIME);
  // Set LOW
  HAL_GPIO_WritePin(ALCD_EN_GPIO, ALCD_EN_PIN, GPIO_PIN_RESET);
  ALCD_Delay_us(ALCD_EN_LOW_TIME);
}
/**
 * @brief Clear all of ALCD characters
 */
void ALCD_clear(void) {
  ALCD_write(
    ALCD_RS_State_Command, 
    ALCD_CMD_CLEAR_DISPLAY,
    ALCD_CMD_CLEAR_DISPLAY_TIMEOUT
  );
}
/**
 * @brief Print a single character into ALCD
 * 
 * @param c character
 */
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
/**
 * @brief Print string into ALCD
 * 
 * @param str address of string
 */
void ALCD_puts(const char* str) {
  while (*str != '\0') {
    ALCD_putChar(*str++);
  }
}
/**
 * @brief Set cursor into given column and row
 * 
 * @param x index of column
 * @param y index of row
 */
void ALCD_goto(uint8_t x, uint8_t y) {
  static const uint8_t LINE_ADDR[4] = {
    0x00, 0x40, 0x14, 0x54,
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
/**
 * @brief Print a single character into given coordinate
 * 
 * @param x index of column
 * @param y index of row
 * @param c character
 */
void ALCD_putCharXY(uint8_t x, uint8_t y, char c) {
  ALCD_goto(x, y);
  ALCD_putChar(c);
}
/**
 * @brief Print a string into given coordinate
 * 
 * @param x index of column
 * @param y index of row
 * @param str address of string
 */
void ALCD_putsXY(uint8_t x, uint8_t y, const char* str) {
  ALCD_goto(x, y);
  ALCD_puts(str);
}

#if ALCD_SUPPORT_PUT_NUM
/**
 * @brief Print a number in decimal
 * 
 * @param num number
 */
void ALCD_putNum(int32_t num) {
  char temp[13];
  snprintf(temp, sizeof(temp) - 1, "%d", num);
  ALCD_puts(temp);
}
/**
 * @brief Print a number into given coordinate
 * 
 * @param x index of column
 * @param y index of row
 * @param num number
 */
void ALCD_putNumXY(uint8_t x, uint8_t y, int32_t num) {
  ALCD_goto(x, y);
  ALCD_putNum(num);
}
#endif // ALCD_SUPPORT_PUT_NUM
#if ALCD_SUPPORT_PRINTF
/**
 * @brief Print into ALCD with a custom format
 * 
 * @param fmt format
 * @param ... parameters
 */
void ALCD_printf(const char* fmt, ...) {
  char temp[ALCD_PRINTF_BUF_SIZE];
  
  va_list args;
  
  va_start(args, fmt);
  vsnprintf(temp, sizeof(temp) - 1, fmt, args);
  va_end(args);
  ALCD_puts(temp);
}
/**
 * @brief Print into ALCD with custom format at given coordinate
 * 
 * @param x index of column
 * @param y index of row
 * @param fmt format
 * @param ... 
 */
void ALCD_printfXY(uint8_t x, uint8_t y, const char* fmt, ...) {
  char temp[ALCD_PRINTF_BUF_SIZE];
  
  va_list args;
  
  va_start(args, fmt);
  vsnprintf(temp, sizeof(temp) - 1, fmt, args);
  va_end(args);
  
  ALCD_goto(x, y);
  ALCD_puts(temp);
}
#endif // ALCD_SUPPORT_PRINTF
/**
 * @brief ALCD built-in delay microseconds
 * 
 * @param us microseconds
 */
void __ALCD_Delay_us(uint32_t us) {
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
/**
 * @brief ALCD built-in delay miliseconds
 * 
 * @param ms miliseconds
 */
void __ALCD_Delay_ms(uint32_t ms) {
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

