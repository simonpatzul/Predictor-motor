cmake_minimum_required(VERSION 3.13)

set(PICO_BOARD pico)
include(pico_sdk_import.cmake)

project(lab4_observadores C CXX ASM)
set(CMAKE_CXX_STANDARD 17)
pico_sdk_init()

add_executable(lab4_fw
    src/main.cpp
    src/encoder.cpp
    src/motor_driver.cpp
    src/limit_switch.cpp
)
pico_generate_pio_header(lab4_fw ${CMAKE_CURRENT_LIST_DIR}/src/quadrature_encoder.pio)
target_include_directories(lab4_fw PRIVATE include)
target_link_libraries(lab4_fw pico_stdlib hardware_pio hardware_pwm)

pico_enable_stdio_usb(lab4_fw 1)
pico_enable_stdio_uart(lab4_fw 0)
pico_add_extra_outputs(lab4_fw)
