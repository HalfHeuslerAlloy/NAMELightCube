cmake_minimum_required(VERSION 3.13)

# initialize the SDK based on PICO_SDK_PATH
# note: this must happen before project()
include(pico_sdk_import.cmake)

project(Firmware)

# initialize the Raspberry Pi Pico SDK
pico_sdk_init()

# rest of your project

add_subdirectory(helloworld)
add_subdirectory(CubeBitBanging)
add_subdirectory(CubeBitBangingCommMulticore)
add_subdirectory(MultiCubeController)
add_subdirectory(CubeController16x16x16)