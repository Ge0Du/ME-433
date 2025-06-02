cmake_minimum_required(VERSION 3.13)

# Use C11 and C++17
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# == VS Code Pico Extension Boilerplate (Do Not Edit) ==
if (WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(sdkVersion 2.1.1)
set(toolchainVersion 14_2_Rel1)
set(picotoolVersion 2.1.1)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================
set(PICO_BOARD pico2 CACHE STRING "Board type")

# Pull in the Raspberry Pi Pico SDK
include(pico_sdk_import.cmake)

project(HW18 C CXX ASM)

# Initialize the Pico SDK
pico_sdk_init()

# Create an executable named “HW18” from HW18.c + ov7670_regs.c
add_executable(HW18
    HW18.c
    ov7670_regs.c
)

# (Optional) Program name and version
pico_set_program_name(HW18 "HW18")
pico_set_program_version(HW18 "0.1")

# Disable UART stdio, disable USB stdio (you said you don’t need USB‐CDC)
pico_enable_stdio_uart(HW18 0)
pico_enable_stdio_usb(HW18 0)

# Link in necessary Pico libraries:
target_link_libraries(HW18
    pico_stdlib
    hardware_pwm
    hardware_i2c
    hardware_gpio
    pico_multicore
    pico_stdio_usb
)

# Include any headers in this folder
target_include_directories(HW18 PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}
)

# Generate UF2/ELF/HEX outputs automatically
pico_add_extra_outputs(HW18)
