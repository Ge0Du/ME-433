# Generated Cmake Pico project file

cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Initialise pico_sdk from installed location
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

# Set target board
set(PICO_BOARD pico2 CACHE STRING "Board type")

# Required TinyUSB configuration macros
add_compile_definitions(
    CFG_TUSB_MCU=OPT_MCU_RP2040
    CFG_TUSB_OS=OPT_OS_PICO
    CFG_TUSB_RHPORT0_MODE=OPT_MODE_DEVICE
)

# Pull in the Raspberry Pi Pico SDK
include(pico_sdk_import.cmake)

project(HW11_Project C CXX ASM)

# Initialize the SDK
pico_sdk_init()

# Add your source files
add_executable(HW11
    HW11.c
    usb_descriptors.c
    core1.c  # optional, if still using
)

# Optional: set program metadata
pico_set_program_name(HW11 "HW11")
pico_set_program_version(HW11 "0.1")

# Enable USB output, disable UART
pico_enable_stdio_usb(HW11 1)
pico_enable_stdio_uart(HW11 0)

# Link required libraries
target_link_libraries(HW11
    pico_stdlib
    tinyusb_device
    tinyusb_board
    pico_multicore
    hardware_adc  # if still used
)

# Include current directory
target_include_directories(HW11 PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}
)

# Generate UF2 file etc.
pico_add_extra_outputs(HW11)
