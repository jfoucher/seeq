cmake_minimum_required(VERSION 3.13)
include(pico_sdk_import.cmake)

project(seeq C CXX ASM)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(PICO_SDK_PATH $ENV{PICO_SDK_PATH})

add_compile_options(-Wall
        -Wno-format          # int != int32_t as far as the compiler is concerned because gcc has int32_t as long int
        -Wno-unused-function # we have some for the docs that aren't called
        -Wno-maybe-uninitialized
        )

add_definitions(-DPICO=1)
add_definitions(-DUSE_MEMORY_POINTER=1)

pico_sdk_init()

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)


add_executable(seeq)

target_sources(seeq PUBLIC
        ${CMAKE_CURRENT_LIST_DIR}/main.cpp
        ${CMAKE_CURRENT_LIST_DIR}/usb_descriptors.c
        ${CMAKE_CURRENT_LIST_DIR}/lib/midi/midi.c
        )

add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/lib/encoder)
add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/lib/OLED)
add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/lib/midi_uart_lib)
# add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/lib/keypad)

# Make sure TinyUSB can find tusb_config.h
target_include_directories(seeq PUBLIC ${CMAKE_CURRENT_LIST_DIR})

# In addition to pico_stdlib required for common PicoSDK functionality, add dependency on tinyusb_device
# for TinyUSB device support and tinyusb_board for the additional board support library used by the example
target_link_libraries(seeq PUBLIC 
pico_stdlib 
pico_unique_id
tinyusb_device 
tinyusb_board
hardware_timer
hardware_pio
hardware_irq
hardware_vreg
pico_multicore
encoder
OLED
midi_uart_lib
# keypad
)

# enable usb output, disable uart output
# pico_enable_stdio_usb(seeq 0)
# pico_enable_stdio_uart(seeq 0)


# Uncomment this line to enable fix for Errata RP2040-E5 (the fix requires use of GPIO 15)
#target_compile_definitions(seeq PUBLIC PICO_RP2040_USB_DEVICE_ENUMERATION_FIX=1)

pico_add_extra_outputs(seeq)
