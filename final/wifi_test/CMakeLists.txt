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

# Pull in Raspberry Pi Pico SDK (must be before project)
include(pico_sdk_import.cmake)

project(wifi_test C CXX ASM)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

add_executable(wifi_test wifi_test.c dhcpserver/dhcpserver.c)

pico_set_program_name(wifi_test "wifi_test")
pico_set_program_version(wifi_test "0.1")

# USB serial on (needed for printf)
pico_enable_stdio_uart(wifi_test 0)
pico_enable_stdio_usb(wifi_test 1)

# lwipopts.h lives next to this file, so it must be on the include path
target_include_directories(wifi_test PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}
        ${CMAKE_CURRENT_LIST_DIR}/dhcpserver
)

target_link_libraries(wifi_test
        pico_stdlib
        pico_cyw43_arch_lwip_threadsafe_background
        )

pico_add_extra_outputs(wifi_test)