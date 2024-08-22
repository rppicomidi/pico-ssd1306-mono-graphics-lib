cmake_minimum_required(VERSION 3.13)

add_library(ssd1306i2c INTERFACE)
target_sources(ssd1306i2c INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/ssd1306i2c.cpp
)
target_include_directories(ssd1306i2c INTERFACE ${CMAKE_CURRENT_LIST_DIR})
#set PARAM_ASSERTIONS_ENABLED_I2C=0 to disable the parameter checking assertions
target_compile_definitions(ssd1306i2c INTERFACE
PARAM_ASSERTIONS_ENABLED_I2C=1
)
target_link_libraries(ssd1306i2c INTERFACE pico_stdlib hardware_i2c hardware_gpio)

add_library(ssd1306pioi2c INTERFACE)
target_sources(ssd1306pioi2c INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/ssd1306pioi2c.cpp
    ${CMAKE_CURRENT_LIST_DIR}/ssd1306pioi2c.h
)
pico_generate_pio_header(ssd1306pioi2c ${CMAKE_CURRENT_LIST_DIR}/i2c.pio)
target_include_directories(ssd1306pioi2c INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}
)
target_link_libraries(ssd1306pioi2c INTERFACE pico_stdlib hardware_pio hardware_i2c hardware_gpio)

add_library(ssd1306 INTERFACE)
target_sources(ssd1306 INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/ssd1306.cpp
)
target_include_directories(ssd1306 INTERFACE ${CMAKE_CURRENT_LIST_DIR})
target_link_libraries(ssd1306 INTERFACE pico_stdlib)

add_library(mono_graphics_lib INTERFACE)
target_sources(mono_graphics_lib INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/mono_graphics_lib.cpp
)
target_include_directories(mono_graphics_lib INTERFACE ${CMAKE_CURRENT_LIST_DIR} ${CMAKE_CURRENT_LIST_DIR}/../../ext_lib)
target_link_libraries(mono_graphics_lib INTERFACE pico_stdlib)

add_library(text_box INTERFACE)
target_sources(text_box INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/text_box.cpp
)
target_include_directories(text_box INTERFACE ${CMAKE_CURRENT_LIST_DIR})
target_link_libraries(text_box INTERFACE mono_graphics_lib pico_stdlib)

