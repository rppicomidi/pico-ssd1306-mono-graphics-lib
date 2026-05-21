cmake_minimum_required(VERSION 3.13)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(BOARD pico-sdk)
set(TINYUSB_OPT_OS OPT_OS_FREERTOS)
#set(PICO_BOARD pico CACHE STRING "Board type")

# Pull in Raspberry Pi Pico SDK (must be before project)
include(pico_sdk_import.cmake)

# locate the copy of the FreeRTOS Kernel
include(FreeRTOS_Kernel_import.cmake)

project(1display-i2c-freertos-example C CXX ASM)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/../../../ lib)
add_executable(1display-i2c-freertos-example
    1display-i2c-example.cpp
    )

target_include_directories(1display-i2c-freertos-example PRIVATE 
    ${CMAKE_CURRENT_LIST_DIR}
    ${CMAKE_CURRENT_LIST_DIR}/../../../fonts
)

target_compile_definitions(1display-i2c-freertos-example PRIVATE
# OS_FREERTOS must be defined or else the critical section macros will not be defined properly
    OS_FREERTOS=1
# The following 2 definitions are for my hardware, which maps the UART0 pins away from default
    PICO_DEFAULT_UART_TX_PIN=16
    PICO_DEFAULT_UART_RX_PIN=17
    )

target_link_options(1display-i2c-freertos-example PRIVATE -Xlinker --print-memory-usage)
target_compile_options(1display-i2c-freertos-example PRIVATE -Wall -Wextra)
target_link_libraries(1display-i2c-freertos-example ssd1306i2c FreeRTOS-Kernel-Heap4 ssd1306 mono_graphics_lib)
pico_add_extra_outputs(1display-i2c-freertos-example)

