# Set minimum required version of CMake
cmake_minimum_required(VERSION 3.12)

# Include build functions from Pico SDK
# include($ENV{PICO_SDK_PATH}/external/pico_sdk_import.cmake)
include (pico_sdk_import.cmake)
include (FreeRTOS_Kernel_import.cmake)

# Set name of project (as PROJECT_NAME) and C/C++ standards
project(Wooting C CXX ASM)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

set(FAMILY rp2040)
set(BOARD pico_sdk)

add_compile_options(-Wall -O2)

# Creates a pico-sdk subdirectory in our project for the libraries
pico_sdk_init()

add_subdirectory(src)