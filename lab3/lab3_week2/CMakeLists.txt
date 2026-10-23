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
set(sdkVersion 2.3.1)
set(toolchainVersion 15_2_Rel1)
set(picotoolVersion 2.3.1)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================
set(PICO_BOARD pico2 CACHE STRING "Board type")

# Pull in Raspberry Pi Pico SDK (must be before project)
include(pico_sdk_import.cmake)

project(lab3_week2 C CXX ASM)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

# Add executable FIRST: every command below needs the target to exist
add_executable(lab3_week2
        lab3_week2.c
        mpu6050.c
        VGA/vga16_graphics_v3.c
)

pico_set_program_name(lab3_week2 "lab3_week2")
pico_set_program_version(lab3_week2 "0.1")

# Modify the below lines to enable/disable output over UART/USB
pico_enable_stdio_uart(lab3_week2 0)
pico_enable_stdio_usb(lab3_week2 1)

# Assemble the VGA PIO programs into headers
pico_generate_pio_header(lab3_week2 ${CMAKE_CURRENT_LIST_DIR}/VGA/hsync.pio)
pico_generate_pio_header(lab3_week2 ${CMAKE_CURRENT_LIST_DIR}/VGA/vsync.pio)
pico_generate_pio_header(lab3_week2 ${CMAKE_CURRENT_LIST_DIR}/VGA/rgb.pio)

target_link_libraries(lab3_week2
        pico_stdlib
        pico_bootsel_via_double_reset
        pico_multicore
        hardware_pwm
        hardware_dma
        hardware_irq
        hardware_adc
        hardware_pio
        hardware_sync
        hardware_i2c
        hardware_clocks)

target_include_directories(lab3_week2 PRIVATE ${CMAKE_CURRENT_LIST_DIR})

pico_add_extra_outputs(lab3_week2)