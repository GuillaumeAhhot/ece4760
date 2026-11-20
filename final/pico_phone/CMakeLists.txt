# Generated Cmake Pico project file

cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

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
set(PICO_BOARD pico_w CACHE STRING "Board type")

include(pico_sdk_import.cmake)

project(pico_phone C CXX ASM)

pico_sdk_init()

add_executable(pico_phone
        pico_phone.c
        dhcpserver/dhcpserver.c
        )

pico_generate_pio_header(pico_phone ${CMAKE_CURRENT_LIST_DIR}/i2s_in.pio)

pico_set_program_name(pico_phone "pico_phone")
pico_set_program_version(pico_phone "0.1")

pico_enable_stdio_uart(pico_phone 0)
pico_enable_stdio_usb(pico_phone 1)

target_include_directories(pico_phone PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}
        ${CMAKE_CURRENT_LIST_DIR}/dhcpserver
        )

target_link_libraries(pico_phone
        pico_stdlib
        pico_cyw43_arch_lwip_threadsafe_background
        hardware_dma
        hardware_pio
        hardware_spi
        hardware_clocks
        hardware_watchdog
        )

pico_add_extra_outputs(pico_phone)