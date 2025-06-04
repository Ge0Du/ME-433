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
set(sdkVersion 2.1.1)
set(toolchainVersion 14_2_Rel1)
set(picotoolVersion 2.1.1)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================

# Specify which Pico board you’re targeting:
set(PICO_BOARD pico2 CACHE STRING "Board type")

# Pull in Raspberry Pi Pico SDK (must be before project)
include(pico_sdk_import.cmake)

project(HW12 C CXX ASM)
pico_sdk_init()

# Add all source files related to HW12
add_executable(HW12
    cam.c
    HW12.c
)

# Give the UF2 a more friendly name/version if you like:
pico_set_program_name(HW12 "HW12_OV7670")
pico_set_program_version(HW12 "0.1")

# Enable USB CDC stdio (printf/scanf over USB). Disable UART stdio.
pico_enable_stdio_uart(HW12 0)
pico_enable_stdio_usb (HW12 1)

# Link in pico_stdlib plus the I2C and PWM hardware libraries:
target_link_libraries(HW12
    pico_stdlib
    hardware_i2c
    hardware_pwm
)

# Make sure our local headers (cam.h, ov7670.h, etc.) are visible:
target_include_directories(HW12 PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}
)

# Create the UF2/.elf/.hex outputs
pico_add_extra_outputs(HW12)
