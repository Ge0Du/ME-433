cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# ==============================================
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
set(PICO_BOARD pico2 CACHE STRING "Board type")

# Pull in Raspberry Pi Pico SDK (must be before project)
include(pico_sdk_import.cmake)

project(HW13 C CXX ASM)

# Initialize the Raspberry Pi Pico SDK
pico_sdk_init()

# ---------------------------------------------------
# Add executable: include your main source and the SSD1306 driver
# ---------------------------------------------------
add_executable(HW13
    HW13.c
    ssd1306.c
)

pico_set_program_name(HW13 "HW13")
pico_set_program_version(HW13 "0.1")

# ---------------------------------------------------
# Enable USB stdio (for printf debugging over USB)
# If you prefer UART, set pico_enable_stdio_uart(HW13 1) instead
# ---------------------------------------------------
pico_enable_stdio_uart(HW13 0)
pico_enable_stdio_usb(HW13 1)

# ---------------------------------------------------
# Include current directory so HW13.c can see ssd1306.h
# ---------------------------------------------------
target_include_directories(HW13 PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}
)

# ---------------------------------------------------
# Link against pico_stdlib and the I2C driver (hardware_i2c)
# ---------------------------------------------------
target_link_libraries(HW13
    pico_stdlib
    hardware_i2c
)

# ---------------------------------------------------
# Create map/bin/hex files
# ---------------------------------------------------
pico_add_extra_outputs(HW13)
