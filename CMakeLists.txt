

cmake_minimum_required(VERSION 3.12)
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

# add_subdirectory(FatFs_SPI build)

# add_subdirectory(scanvideo)

add_executable(seeq
    main.cpp
)

add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/lib/encoder)
add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/lib/OLED)

# Link libraries
target_link_libraries(seeq
    pico_stdlib
    hardware_timer
    hardware_pio
    hardware_irq
    hardware_vreg
    pico_multicore
    encoder
    OLED
)


pico_set_program_name(seeq "seeq")
pico_set_program_version(seeq "0.0.1")

# generate PIO headers
pico_generate_pio_header(seeq ${CMAKE_CURRENT_LIST_DIR}/lib/encoder/encoder.pio)

# Get number of SD cards from the environment, else set to 1:
#   In Visual Studio this can be set in .vscode/settings.json. E.g.:
#      "cmake.configureArgs": ["-DN_SD_CARDS=2"],
# IF (NOT DEFINED N_SD_CARDS)
#     SET(N_SD_CARDS 1)
# ENDIF()
# # Set DEBUG and N_SD_CARDS compile definitions:
# target_compile_definitions(picovic PUBLIC DEBUG=1 N_SD_CARDS=${N_SD_CARDS})


# enable usb output, disable uart output
pico_enable_stdio_usb(seeq 1)
pico_enable_stdio_uart(seeq 0)

# create map/bin/hex/uf2 file etc.
pico_add_extra_outputs(seeq)
