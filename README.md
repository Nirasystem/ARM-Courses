# ARM Courses

This repository contains examples and documents of Nirasystem ARM courses.

## ARM Essentials

### Intro

- [x] [Introduction](Sections/Introduction/Docs/Introduction.pdf)
- [x] [Embedded Systems Introduction](Sections/Introduction/Docs/Embedded%20Systems%20Introduction.pdf)
- [x] [ARM IDEs](Sections/Introduction/Docs/ARM%20IDEs.pdf)
- [x] [STM32F4xx Introduction](Sections/Introduction/Docs/STM32F407%20Introduction.pdf)
- [x] [STM32CubeMX](Sections/Introduction/Examples/Template)
- [x] [Hello World Project](Sections/Introduction/Examples/HelloWorld)
- [x] [HAL Project Structure](Sections/Introduction/Examples/Template)

### GPIO (General-Purpose Input/Output)

- [x] [Introduction](Sections/GPIO/Docs/GPIO%20Introduction.pdf)
- [x] [Introduction - Manual](Docs/STM32F4x%20Refrence%20Manual.pdf#page=267)
- **Examples**
  - [x] Blink LED
  - [x] Blink LED - Registry
  - [x] Parallel Blink LED
  - [x] Button - Polling
  - [x] Button - Polling - Registry
  - [x] Button - Non-Blocking
  - [x] Button - Non-Blocking Advance

### Interrupt

- [x] [Introduction](Sections/Interrupt/Docs/Interrupt%20Introduction.pdf)

### EXTI (External Interrupt)

- [x] [Introduction](Sections/EXTI/Docs/EXTI%20Introduction.pdf)
- [X] [Introduction - Manual](Docs/STM32F4x%20Refrence%20Manual.pdf#page=371)
- **Examples**
  - [X] Button-EXTI

### TIM (Timer)

- [x] [Introduction](Sections/TIM/Docs/TIM%20Introduction.pdf)
- [X] [Introduction - Manual](Docs/STM32F4x%20Refrence%20Manual.pdf#page=515)
- **Examples**
  - [X] TIM-LedBlink
  - [X] GPIO-SoftPWM
  - [X] TIM-SoftPWM
  - [X] TIM-PWM
  - [X] TIM-PWM-CenterAligned
  - [X] SoftwareDelay
  - [X] TIM-Delay

### ALCD (Alphanumeric LCD)

- [X] [Introduction](Sections/ALCD/Docs/ALCD%20Introduction.pdf)
- **Examples**
  - [X] ALCD-Example
  - [X] ALCD-Example-Part2
  - [X] ALCD-Library

### DMA (Direct Memory Access)

- [X] [Introduction](Sections/DMA/Docs/DMA%20Introduction.pdf)
- [X] [Introduction - Manual](Docs/STM32F4x%20Refrence%20Manual.pdf#page=302)

### ADC (Analog to Digital Converter)

- [X] [Introduction](Sections/ADC/Docs/ADC%20Introduction.pdf)
- [X] [Introduction - Manual](Docs/STM32F4x%20Refrence%20Manual.pdf#page=388)
- **Examples**
  - [X] ADC-Polling
  - [X] ADC-IT
  - [X] ADC-IT-Overlap
  - [X] ADC-DMA

### Communications

- [X] [Introduction](Sections/Communications/Docs/Communications.pdf)

### UART (Universal Synchronous Receiver/Transmitter)

- [X] [Introduction](Sections/UART/Docs/UART%20Introduction.pdf)
- [X] [Introduction - Manual](Docs/STM32F4x%20Refrence%20Manual.pdf#page=965)
- [X] [RS-232 Introduction](Sections/UART/Docs/RS-232.pdf)
- [X] [RS-485/RS-422 Introduction](Sections/UART/Docs/RS-485.pdf)
- [X] [Buffering](Sections/UART/Docs/Buffering.pdf)
- [X] [Framing](Sections/UART/Docs/Framing.pdf)
- **Examples**
  - [X] UART-Echo
  - [X] UART-Echo-IT
  - [X] UART-Echo-DMA
  - [X] UART-ReadLine-SingleBuffer
  - [X] UART-ReadLine-SingleBuffer-Advance
  - [X] UART-ReadLine-MultiBuffer
  - [X] UART-ReadLine-MultiBuffer-Advance
  - [X] UART-ReadLine-CircularBuffer
  - [X] UART-Logger-MultiBuffer
  - [X] UART-Logger-MultiBuffer-Advance
  - [X] UART-Logger-MultiBuffer-Advance-Fix
  - [X] UART-Logger-CircularBuffer
  - [X] UART-BinaryFrame

---

## ARM Advance

### GPS Module (u-Blox Neo-6)

- [X] [Introduction](Sections/GPS%20Module/Docs/GPS%20Module%20Introduction.pdf)
- [X] [Introduction-Manual](Sections/GPS%20Module/Docs/NEO-6_DataSheet_(GPS.G6-HW-09005)_2.pdf)
- [X] [NMEA Introduction](Sections/GPS%20Module/Docs/NMEA%20Reference%20Manual-Rev2.1-Dec07.pdf)
- [X] [Data Overview](Sections/GPS%20Module/Docs/GPS%20Sample.txt)
- **Examples**
  - [X] [GPS-DateTime](Sections/GPS%20Module/Examples/GPS-DateTime/)
  - [X] [GPS-DateTime-Advance](Sections/GPS%20Module/Examples/GPS-DateTime-Advance/)

### GSM Module (SIM800)

- [X] [Introduction](Sections/GSM%20Module/Docs/GSM%20Introduction.pdf)
- [X] [Introduction-Manual](Sections/GSM%20Module/Docs/SIM800_Hardware%20Design_V1.09.pdf)
- [X] [AT Commands Introduction](Sections/GSM%20Module/Docs/AT%20Commands%20Introduction.pdf)
- **Examples**
  - [X] [GSM-SendSMS](Sections/GSM%20Module/Examples/GSM-SendSMS/)
  - [X] [GSM-ReceiveSMS](Sections/GSM%20Module/Examples/GSM-ReceiveSMS/)
  - [X] [GSM-Advance](Sections/GSM%20Module/Examples/GSM-Advance/)

### I2C

- [X] [Introduction](Sections/I2C/Docs/I2C%20Introduction.pdf)
- [X] [Introduction - Manual](Docs/STM32F4x%20Refrence%20Manual.pdf#page=839)
- **Examples**
  - [X] [I2C-Master](Sections/I2C/Examples/I2C-Master/)
  - [X] [I2C-Slave](Sections/I2C/Examples/I2C-Slave/)
  - [X] [I2C-Master-Advance](Sections/I2C/Examples/I2C-Master-Advance/)
  - [X] [I2C-Slave-Advance](Sections/I2C/Examples/I2C-Slave-Advance/)

#### EEPROM (AT24Cxxx)

- [X] [Introduction](Sections/EEPROM/Docs/EEPROM%20Introduction.pdf)
- [X] [Introduction - Manual](Sections/EEPROM/Docs/AT24C32-64.pdf)
- [X] [Memory Map](Sections/EEPROM/Docs/Memory%20Map.pdf)
- [X] [Storage Interface](Sections/EEPROM/Docs/Storage%20Interface.pdf)
- **Examples**
  - [X] [AT24Cxxx-ResetCounter](Sections/EEPROM/Examples/EEPROM-ResetCounter/)
  - [X] [AT24Cxxx-SafeWrite](Sections/EEPROM/Examples/EEPROM-SafeWrite/)
  - [X] [AT24Cxxx-Database](Sections/EEPROM/Examples/EEPROM-Database/)
  - [X] [AT24Cxxx-Database-Advance](Sections/EEPROM/Examples/EEPROM-Database-Advance/)

#### RTC-External (DS1307)

- [X] [Introduction - Manual](Sections/RTC-External/Docs/DS1307.pdf)
- **Examples**
  - [X] [DS1307-Simple](Sections/RTC-External/Examples/RTC-DS1307-Simple/)
  - [X] [DS1307-Advance](Sections/RTC-External/Examples/RTC-DS1307-Advance/)

### SPI (Serial Peripheral Interface)

- [X] [Introduction](Sections/SPI/Docs/SPI%20Introduction.pdf)
- [X] [Introduction - Manual](Docs/STM32F4x%20Refrence%20Manual.pdf#page=873)
- **Examples**
  - [X] [SPI-Master](Sections/SPI/Examples/SPI-Master/)
  - [X] [SPI-Slave](Sections/SPI/Examples/SPI-Slave/)

#### Digital Pot (MCP4131)

- [X] [Introduction Manual](Sections/MCP4131/Docs/MCP413x%20Datasheet.pdf)
- **Examples**
  - [X] [MCP-Example](Sections/MCP4131/Examples/MCP4131-Example/)

### DAC (Digital to Analog Converter)

- [X] [Introduction](Sections/DAC/Docs/DAC%20Introduction.pdf)
- [X] [Introduction - Manual](Docs/STM32F4x%20Refrence%20Manual.pdf#page=433)
- **Examples**
  - [X] [DAC-SoftwareTrigger](Sections/DAC/Examples/DAC-SoftwareTrigger/)
  - [X] [DAC-DMA](Sections/DAC/Examples/DAC-DMA/)
  - [X] [DAC-LookupTable](Sections/DAC/Examples/DAC-LookupTable/)

#### Audio

- [X] [Introduction](Sections/Audio/Docs/Audio%20-%20Introduction.pdf)
- **Examples**
  - [X] [Audio-PlaySound](Sections/Audio/Examples/Audio-PlaySound/)

### RTC (Real-Time Calendar)

- [X] [Introduction](Sections/RTC/Docs/RTC%20Introduction.pdf)
- [X] [Introduction - Manual](Docs/STM32F4x%20Refrence%20Manual.pdf#page=798)
- **Examples**
  - [X] [RTC-Simple](Sections/RTC/Examples/RTC-Simple/)
  - [X] [RTC-Alarm](Sections/RTC/Examples/RTC-Alarm/)

#### Backup SRAM

- [X] [Introduction-Manual](Docs/STM32F4x%20Refrence%20Manual.pdf#page=119)
- **Examples**
  - [X] [BKPSRAM-Simple](Sections/Backup%20SRAM/Examples/BKPSRAM-Simple/)
  - [X] [BKPSRAM-ResetCount](Sections/Backup%20SRAM/Examples/BKPSRAM-ResetCount/)
  - [X] [BKPSRAM-Database](Sections/Backup%20SRAM/Examples/BKPSRAM-Database/)

### IWDG (Independent Watchdog)

- [X] [Introduction](Sections/IWDG/Docs/IWDG%20Introduction.pdf)
- [X] [Introduction - Manual](Docs/STM32F4x%20Refrence%20Manual.pdf#page=708)
- **Examples**
  - [X] [IWDG-Example](Sections/IWDG/Examples/IWDG-Example/)

### WWDG (Window Watchdog)

- [X] [Introduction](Sections/WWDG/Docs/WWDG%20Introduction.pdf)
- [X] [Introduction - Manual](Docs/STM32F4x%20Refrence%20Manual.pdf#page=713)
- **Examples**
  - [X] [WWDG-Example](Sections/WWDG/Examples/WWDG-Example/)

### FLASH

- [X] [Introduction](Sections/FLASH/Docs/FLASH%20Introduction.pdf)
- [X] [Introduction - Manual](Docs/STM32F4x%20Refrence%20Manual.pdf#page=73)
- **Examples**
  - [X] [FLASH-Database](Sections/FLASH/Examples/FLASH-Database/)
  - [X] [FLASH-Database-Advance](Sections/FLASH/Examples/FLASH-Database-Advance/)

### Power Modes

- [X] [Introduction](Sections/Power%20Modes/Docs/Power%20Modes%20Introduction.pdf)
- [X] [Introduction - Manual](Sections/Power%20Modes/Docs/STM32F4%20Power%20Modes%20Application%20Note.pdf)
- [X] STM32CubeMX - Tools 
- **Examples**
  - [X] [PowerModes-Sleep](Sections/Power%20Modes/Examples/PowerModes-Sleep/)
  - [X] [PowerModes-Sleep-UART-DMA](Sections/Power%20Modes/Examples/PowerModes-Sleep-UART-DMA/)
  - [X] [PowerModes-Stop](Sections/Power%20Modes/Examples/PowerModes-Stop/)
  - [X] [PowerModes-StandBy](Sections/Power%20Modes/Examples/PowerModes-StandBy/)

### Hard Fault

- [X] [Introduction](Docs/ARM%20Cortex-M%20Device%20Generic%20User%20Manaual.pdf)
- **Examples**
  - [X] [HardFault-Example](Sections/Hard%20Fault/Examples/Hard%20Fault-Example/)
  - [X] [HardFault-BKPSRAM](Sections/Hard%20Fault/Examples/HardFault-BKPSRAM/)
  - [X] [HardFault-Cause](Sections/Hard%20Fault/Examples/HardFault-Cause/)
