cmake_minimum_required(VERSION 3.13)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(BOARD pico-sdk)

# Pull in Raspberry Pi Pico SDK (must be before project)
include(pico_sdk_import.cmake)

project(1display-i2c-example C CXX ASM)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/../../../ lib)
add_executable(1display-i2c-example
    1display-i2c-example.cpp
    )

target_include_directories(1display-i2c-example PRIVATE 
    ${CMAKE_CURRENT_LIST_DIR}
    ${CMAKE_CURRENT_LIST_DIR}/../../../fonts
)

target_compile_definitions(1display-i2c-example PRIVATE
    PICO_DEFAULT_UART_TX_PIN=16
    PICO_DEFAULT_UART_RX_PIN=17
    OLED_SDA_GPIO=0
    OLED_SCL_GPIO=1
    PARAM_ASSERTIONS_ENABLED_I2C=1
)
target_link_options(1display-i2c-example PRIVATE -Xlinker --print-memory-usage)
target_compile_options(1display-i2c-example PRIVATE -Wall -Wextra)
target_link_libraries(1display-i2c-example ssd1306pioi2c ssd1306 mono_graphics_lib)
pico_add_extra_outputs(1display-i2c-example)

