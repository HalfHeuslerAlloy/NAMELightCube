add_executable(CubeController8x8x32
CubeController8x8x32.c
)

# Add pico_stdlib library which aggregates commonly used features
target_link_libraries(CubeController8x8x32
pico_stdlib
pico_multicore)

pico_enable_stdio_usb(CubeController8x8x32 1)
pico_enable_stdio_uart(CubeController8x8x32 0)

# create map/bin/hex/uf2 file in addition to ELF.
pico_add_extra_outputs(CubeController8x8x32)