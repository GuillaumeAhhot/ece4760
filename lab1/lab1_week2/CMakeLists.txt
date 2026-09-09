# Generated Cmake Pico project file

cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Initialise pico_sdk from installed location
# (note this can come from environment, CMake cache etc)

# == DO NOT EDIT THE FOLLOWING LINES for the Raspberry Pi Pico VS Code Extension to work ==
if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(sdkVersion 2.3.0)
set(toolchainVersion 15_2_Rel1)
set(picotoolVersion 2.3.0)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================
set(PICO_BOARD pico2 CACHE STRING "Board type")

# Pull in Raspberry Pi Pico SDK (must be before project)
include(pico_sdk_import.cmake)

project(lab1_week2 C CXX ASM)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

# Add executable. Default name is the project name, version 0.1

add_executable(lab1_week2 lab1_week2.c )

pico_set_program_name(lab1_week2 "lab1_week2")
pico_set_program_version(lab1_week2 "0.1")

# Modify the below lines to enable/disable output over UART/USB
pico_enable_stdio_uart(lab1_week2 1)
pico_enable_stdio_usb(lab1_week2 0)

# Add the standard library to the build
target_link_libraries(lab1_week2
    pico_stdlib
    pico_sync
    hardware_sync
    hardware_clocks
    hardware_vreg
    hardware_adc
    hardware_spi
    hardware_timer
    hardware_irq
    pico_bootsel_via_double_reset
)

# Add the standard include files to the build
target_include_directories(lab1_week2 PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}
)

pico_add_extra_outputs(lab1_week2)

