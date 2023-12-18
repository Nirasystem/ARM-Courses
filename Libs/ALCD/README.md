# ALCD Library

The **ALCD** (Alphanumeric LCD) library is a C library designed for use with various microcontroller platforms to control and interact with alphanumeric LCD displays. This library provides functions to initialize the LCD, clear its display, set the cursor position, print characters and strings, and handle numeric and formatted output. It is highly configurable and can be adapted to different hardware setups.

## Table of Contents

- [ALCD Library](#alcd-library)
  - [Table of Contents](#table-of-contents)
  - [Library Features](#library-features)
  - [Getting Started](#getting-started)
    - [Hardware Setup](#hardware-setup)
    - [Library Configuration](#library-configuration)
    - [Initializing the ALCD](#initializing-the-alcd)
  - [Library Functions](#library-functions)
    - [Clearing the Display](#clearing-the-display)
    - [Printing Characters and Strings](#printing-characters-and-strings)
    - [Setting the Cursor Position](#setting-the-cursor-position)
    - [Numeric Output](#numeric-output)
    - [Formatted Output (Printf)](#formatted-output-printf)
  - [Custom Delay Functions](#custom-delay-functions)
  - [Examples](#examples)
  - [License](#license)

## Library Features

- Initialization of alphanumeric LCD displays with configurable column and row settings.
- Functions for clearing the display and setting the cursor position.
- Printing characters and strings to the LCD.
- Numeric output functions for displaying integers.
- Formatted output using a `printf`-like interface.
- Customizable hardware configuration for pins.
- Customizable delay functions for different microcontroller platforms.

## Getting Started

### Hardware Setup

Before using the ALCD library, you need to connect your alphanumeric LCD display to your microcontroller following the pin configuration specified in the library's header file (`ALCD.h`). Make sure to connect the control pins (RS, RW, EN) and data pins (D4, D5, D6, D7) properly.

### Library Configuration

In the `ALCD.h` file, you can configure the library to match your specific hardware setup by modifying the pin assignments and enabling/disabling certain features:

- Set the GPIO pins for RS, RW, EN, D4, D5, D6, and D7.
- Enable or disable support for numeric output with `ALCD_SUPPORT_PUT_NUM`.
- Enable or disable support for formatted output using `printf` with `ALCD_SUPPORT_PRINTF`.
- Adjust delay functions, such as `ALCD_Delay_us` and `ALCD_Delay_ms`, if necessary.
- Customize the size of the local buffer used for formatted output with `ALCD_PRINTF_BUF_SIZE`.

### Initializing the ALCD

To use the ALCD library, follow these steps:

1. Include the `ALCD.h` header file in your project.
2. Initialize your hardware according to your microcontroller platform and pin configuration.
3. Call `ALCD_init(col, row)` to initialize the LCD display, where `col` is the number of columns (e.g., 16, 20) and `row` is the number of rows (e.g., 1, 2, 4).

Example:

```c
#include "ALCD.h"

int main() {
    // Initialize your microcontroller and pins here

    // Initialize the ALCD with 16 columns and 2 rows
    ALCD_init(16, 2);

    // Now you can start using ALCD library functions

    return 0;
}
```

## Library Functions

The ALCD library provides various functions for controlling the LCD display:

### Clearing the Display

- `ALCD_clear()`: Clears the entire display and sets the cursor to the home position.

### Printing Characters and Strings

- `ALCD_putChar(char c)`: Prints a single character to the current cursor position.
- `ALCD_puts(const char* str)`: Prints a null-terminated string to the current cursor position.

### Setting the Cursor Position

- `ALCD_goto(uint8_t x, uint8_t y)`: Sets the cursor position to the specified column (`x`) and row (`y`).

### Numeric Output

- `ALCD_putNum(int32_t num)`: Prints an integer number to the current cursor position.
- `ALCD_putNumXY(uint8_t x, uint8_t y, int32_t num)`: Prints an integer number at the specified column (`x`) and row (`y`).

### Formatted Output (Printf)

- `ALCD_printf(const char* fmt, ...)`: Prints formatted text to the LCD display using a `printf`-like interface.
- `ALCD_printfXY(uint8_t x, uint8_t y, const char* fmt, ...)`: Prints formatted text at the specified column (`x`) and row (`y`) using a `printf`-like interface.

## Custom Delay Functions

The ALCD library provides built-in delay functions for microsecond and millisecond delays, but you can customize these functions to match your specific microcontroller platform if needed. Modify the `ALCD_Delay_us` and `ALCD_Delay_ms` functions in the `ALCD.c` file to implement custom delays.

## Examples

For usage examples and more detailed information on how to use the ALCD library, please refer to the examples provided in your project or documentation.

## License

This ALCD library is released under the [insert license here] license. See the `LICENSE` file for more details.
