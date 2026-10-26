# == DO NOT EDIT THE FOLLOWING LINES for the Raspberry Pi Pico VS Code Extension to work ==
if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()

set(sdkVersion 2.3.1)
set(toolchainVersion 15_2_Rel1)
set(picotoolVersion 2.3.1)

set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)

if(EXISTS ${picoVscode})
    include(${picoVscode})
endif()

# ====================================================================================

# Target board: Waveshare RP2040-Zero
set(PICO_BOARD waveshare_rp2040_zero CACHE STRING "Board type")

cmake_minimum_required(VERSION 3.13)

include(pico_sdk_import.cmake)

project(conscientia C CXX ASM)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

add_executable(conscientia
    src/main.c
    src/drivers/bmp280.c
)

target_include_directories(conscientia PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}/src
)

target_link_libraries(conscientia
    pico_stdlib
    hardware_i2c
)

pico_enable_stdio_usb(conscientia 1)
pico_enable_stdio_uart(conscientia 0)

pico_add_extra_outputs(conscientia)