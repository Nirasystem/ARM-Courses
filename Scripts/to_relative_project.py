import os
import fileinput

def find_and_replace(directory):
    # Iterate through all subdirectories and files in the specified directory
    for root, dirs, files in os.walk(directory):
        for file in files:
            # Check if the file has a .uvprojx extension
            if file.endswith(".uvprojx"):
                file_path = os.path.join(root, file)
                replace_lines(file_path)

def replace_lines(file_path):
    # Specify the lines to be replaced and their replacements
    replacements = [
        ("<IncludePath>../Core/Inc;../Drivers/STM32F4xx_HAL_Driver/Inc;../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy;../Drivers/CMSIS/Device/ST/STM32F4xx/Include;../Drivers/CMSIS/Include",
         "<IncludePath>../Core/Inc;../../../../../STM32F4Drivers/STM32F4xx_HAL_Driver/Inc;../../../../../STM32F4Drivers/STM32F4xx_HAL_Driver/Inc/Legacy;../../../../../STM32F4Drivers/CMSIS/Device/ST/STM32F4xx/Include;../../../../../STM32F4Drivers/CMSIS/Include"),
        ("../Drivers/STM32F4xx_HAL_Driver/Src/",
         "../../../../../STM32F4Drivers/STM32F4xx_HAL_Driver/Src/"),
        ("../Drivers/STM32F4xx_HAL_Driver/Src/Legacy/",
         "../../../../../STM32F4Drivers/STM32F4xx_HAL_Driver/Src/Legacy/")
    ]

    # Use fileinput to replace the lines in the file
    with fileinput.FileInput(file_path, inplace=True) as file:
        for line in file:
            for old_line, new_line in replacements:
                line = line.replace(old_line, new_line)
            print(line, end='')

if __name__ == "__main__":
    find_and_replace('Sections')
