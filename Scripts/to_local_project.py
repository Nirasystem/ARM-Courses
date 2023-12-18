import os
import fileinput
import shutil

def find_and_replace(directory):
    # Iterate through all subdirectories and files in the specified directory
    for root, dirs, files in os.walk(directory):
        for file in files:
            # Check if the file has a .uvprojx extension
            if file.endswith(".uvprojx"):
                file_path = os.path.join(root, file)
                replace_lines(file_path)
                copy_drivers_directory(root)

def replace_lines(file_path):
    # Specify the lines to be replaced and their replacements
    replacements = [
        ("<IncludePath>../Core/Inc;../../../../../STM32F4Drivers/STM32F4xx_HAL_Driver/Inc;../../../../../STM32F4Drivers/STM32F4xx_HAL_Driver/Inc/Legacy;../../../../../STM32F4Drivers/CMSIS/Device/ST/STM32F4xx/Include;../../../../../STM32F4Drivers/CMSIS/Include",
         "<IncludePath>../Core/Inc;../Drivers/STM32F4xx_HAL_Driver/Inc;../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy;../Drivers/CMSIS/Device/ST/STM32F4xx/Include;../Drivers/CMSIS/Include"),
        ("../../../../../STM32F4Drivers/STM32F4xx_HAL_Driver/Src/",
         "../Drivers/STM32F4xx_HAL_Driver/Src/"),
        ("../../../../../STM32F4Drivers/STM32F4xx_HAL_Driver/Src/Legacy/",
         "../Drivers/STM32F4xx_HAL_Driver/Src/Legacy/")
    ]

    # Use fileinput to replace the lines in the file
    with fileinput.FileInput(file_path, inplace=True) as file:
        for line in file:
            for old_line, new_line in replacements:
                line = line.replace(old_line, new_line)
            print(line, end='')

def copy_drivers_directory(root):
    # Get the relative path to the directory containing the .uvprojx file
    relative_path = os.path.relpath(root, os.getcwd())

    # Check if the relative path is not empty
    if relative_path != ".":
        # Build the destination directory path
        print("Example: ", relative_path)
        destination_dir = os.path.join(relative_path, "../Drivers")

        # Check if the destination directory does not exist, then copy the "STM32F4Drivers" directory
        if not os.path.exists(destination_dir):
            source_dir = os.path.join(".", "STM32F4Drivers")
            shutil.copytree(source_dir, destination_dir)

if __name__ == "__main__":
    find_and_replace('Sections')
