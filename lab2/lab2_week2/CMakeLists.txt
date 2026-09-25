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

project(lab2_week1 C CXX ASM)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

# Add executable. Both your top-level source AND the VGA library source
# must be listed here, or the VGA functions never get compiled/linked.
add_executable(lab2_week2
        lab2_week2.c
        VGA/vga16_graphics_v3.c
)

# Generate headers from the PIO assembly files.
# Must come AFTER add_executable, since these attach to the target.
pico_generate_pio_header(lab2_week2 ${CMAKE_CURRENT_LIST_DIR}/VGA/hsync.pio)
pico_generate_pio_header(lab2_week2 ${CMAKE_CURRENT_LIST_DIR}/VGA/vsync.pio)
pico_generate_pio_header(lab2_week2 ${CMAKE_CURRENT_LIST_DIR}/VGA/rgb.pio)

pico_set_program_name(lab2_week2 "lab2_week2")
pico_set_program_version(lab2_week2 "0.1")

# Modify the below lines to enable/disable output over UART/USB
pico_enable_stdio_uart(lab2_week2 0)
pico_enable_stdio_usb(lab2_week2 0)

# Add the standard library to the build
target_link_libraries(lab2_week2
        pico_stdlib
        pico_divider
        pico_multicore
        pico_sync
        hardware_pio
        hardware_dma
        hardware_spi
        hardware_adc
        hardware_gpio
        hardware_irq
        hardware_clocks
        hardware_pll
        )

# Add the standard include files to the build.
# NOTE: this is include DIRECTORIES only, never list a .c file here.
target_include_directories(lab2_week2 PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}
        ${CMAKE_CURRENT_LIST_DIR}/VGA
)

pico_add_extra_outputs(lab2_week2)