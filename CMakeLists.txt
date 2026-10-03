cmake_minimum_required(VERSION 3.13)

include(pico-sdk/pico_sdk_init.cmake)

project(pico_uart_bridge C CXX ASM)

if (NOT PICO_BOARD)
    set(PICO_BOARD pico CACHE STRING "Target board")
endif()

pico_sdk_init()

add_executable(uart_bridge
    uart-bridge.c
    usb_descriptors.c
)

target_include_directories(uart_bridge PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}
)

target_link_libraries(uart_bridge
    pico_stdlib
    hardware_uart
    tinyusb_device
)

target_compile_definitions(uart_bridge PRIVATE
    CFG_TUSB_OS=OPT_OS_PICO
)

pico_enable_stdio_usb(uart_bridge 0)
pico_enable_stdio_uart(uart_bridge 0)

pico_add_extra_outputs(uart_bridge)
