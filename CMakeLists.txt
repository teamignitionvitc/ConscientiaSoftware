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

# ──────────────────────────────────────────────────────────────────────────────
# FatFs R0.15 — Fetch from elm-chan.org or use local copy in lib/fatfs/
# ──────────────────────────────────────────────────────────────────────────────
set(FATFS_LOCAL_DIR ${CMAKE_CURRENT_LIST_DIR}/lib/fatfs)

if(EXISTS ${FATFS_LOCAL_DIR}/source/ff.c)
    # Use manually-placed local copy
    set(FATFS_SRC_DIR ${FATFS_LOCAL_DIR}/source)
    message(STATUS "FatFs: using local copy at ${FATFS_LOCAL_DIR}")
else()
    # Auto-download
    include(FetchContent)
    FetchContent_Declare(
        fatfs
        URL      http://elm-chan.org/fsw/ff/arc/ff15.zip
    )
    FetchContent_GetProperties(fatfs)
    if(NOT fatfs_POPULATED)
        FetchContent_Populate(fatfs)
    endif()
    set(FATFS_SRC_DIR ${fatfs_SOURCE_DIR}/source)
    message(STATUS "FatFs: fetched to ${FATFS_SRC_DIR}")
endif()

# Override template ffconf.h with our project configuration
configure_file(
    ${CMAKE_CURRENT_LIST_DIR}/src/lib/ffconf.h
    ${FATFS_SRC_DIR}/ffconf.h
    COPYONLY
)

# Build FatFs as a static library
add_library(fatfs STATIC
    ${FATFS_SRC_DIR}/ff.c
    ${FATFS_SRC_DIR}/ffunicode.c
    ${CMAKE_CURRENT_LIST_DIR}/src/lib/diskio.c
)
target_include_directories(fatfs PUBLIC
    ${FATFS_SRC_DIR}
    ${CMAKE_CURRENT_LIST_DIR}/src
)
target_link_libraries(fatfs pico_stdlib)

# ──────────────────────────────────────────────────────────────────────────────
# Main executable
# ──────────────────────────────────────────────────────────────────────────────
add_executable(conscientia
    src/main.c
    src/drivers/bmp280.c
    src/drivers/mpu6050.c
    src/drivers/bh1750.c
    src/drivers/pms5003.c
    src/drivers/mq135.c
    src/drivers/l89ha.c
    src/drivers/lora_e32.c
    src/drivers/sd_card.c
    src/drivers/servo.c
    src/payload/air_quality.c
    src/payload/particulate_analysis.c
    src/payload/light_fusion.c
    src/payload/wind_inference.c
    src/modules/sensor_manager.c
    src/modules/data_logger.c
    src/modules/telemetry.c
    src/modules/flight_state.c
    src/modules/recovery.c
)

pico_generate_pio_header(conscientia
    ${CMAKE_CURRENT_LIST_DIR}/src/drivers/uart_rx.pio
)

target_include_directories(conscientia PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}/src
)

target_link_libraries(conscientia
    pico_stdlib
    hardware_i2c
    hardware_pio
    hardware_adc
    hardware_uart
    hardware_spi
    hardware_pwm
    hardware_clocks
    fatfs
)

pico_enable_stdio_usb(conscientia 1)
pico_enable_stdio_uart(conscientia 0)

pico_add_extra_outputs(conscientia)