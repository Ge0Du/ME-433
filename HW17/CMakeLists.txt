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

# Pull in the Raspberry Pi Pico SDK (PICO_SDK_PATH must point to folder containing pico_sdk_import.cmake)
include(pico_sdk_import.cmake)

project(HW17 C CXX ASM)

# Initialize the Pico SDK
pico_sdk_init()

# Create an executable named “HW17” from HW17.c and ov7670_regs.c
add_executable(HW17
    HW17.c
    cam.c
)

# (Optional) Human-readable program name and version
pico_set_program_name(HW17 "HW17")
pico_set_program_version(HW17 "0.1")

# ------------------------------------------------------------------------------------------------
# Disable all stdio (no USB-CDC, no UART)
# ------------------------------------------------------------------------------------------------
pico_enable_stdio_uart(HW17 0)
pico_enable_stdio_usb(HW17 0)

# ------------------------------------------------------------------------------------------------
# Link in necessary Pico libraries:
#   • pico_stdlib        – standard C library support (sleep_ms(), etc.)
#   • hardware_pwm       – for PWM (motors and camera MCLK)
#   • hardware_i2c       – for OV7670 register writes/reads
#   • hardware_gpio      – for general GPIO and WS2812 bit-banging
# ------------------------------------------------------------------------------------------------
target_link_libraries(HW17
    pico_stdlib
    hardware_pwm
    hardware_i2c
    hardware_gpio
    pico_stdio_usb
    pico_multicore
)

# ------------------------------------------------------------------------------------------------
# Include any headers in this folder (if you add more .h files later)
# ------------------------------------------------------------------------------------------------
target_include_directories(HW17 PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}
)

# ------------------------------------------------------------------------------------------------
# Generate UF2/ELF/HEX outputs automatically
# ------------------------------------------------------------------------------------------------
pico_add_extra_outputs(HW17)
