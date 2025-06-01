cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# == VS Code Pico Extension boilerplate (do not remove) ==
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

include(pico_sdk_import.cmake)
project(HW16 C CXX ASM)
pico_sdk_init()

add_executable(HW16 HW16.c)

pico_set_program_name(HW16 "HW16")
pico_set_program_version(HW16 "0.1")

# Enable USB stdio (so stdio_usb_connected() and getchar() work over USB)
pico_enable_stdio_uart(HW16 0)
pico_enable_stdio_usb(HW16 1)

# Link in the standard library (pico_stdlib) and the PWM driver
target_link_libraries(HW16
    pico_stdlib
    hardware_pwm       # <— This is the critical line for pwm_get_wrap()
)

# Optional: include local headers if you have any
target_include_directories(HW16 PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}
)

pico_add_extra_outputs(HW16)
