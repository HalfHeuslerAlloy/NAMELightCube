add_executable(CubeController16x16x16
CubeController16x16x16.c
)

# Add pico_stdlib library which aggregates commonly used features
target_link_libraries(CubeController16x16x16
pico_stdlib
pico_multicore)

pico_enable_stdio_usb(CubeController16x16x16 1)
pico_enable_stdio_uart(CubeController16x16x16 0)

# create map/bin/hex/uf2 file in addition to ELF.
pico_add_extra_outputs(CubeController16x16x16)