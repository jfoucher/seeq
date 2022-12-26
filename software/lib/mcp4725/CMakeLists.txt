add_library(mcp4725
    mcp4725.h
    mcp4725.c
)

target_link_libraries(mcp4725
    pico_stdlib
    hardware_i2c
)
