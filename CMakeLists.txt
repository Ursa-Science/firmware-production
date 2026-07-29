cmake_minimum_required(VERSION 3.22)

project(ursa-canopen-modules C ASM)

set(URSA_ROOT "${CMAKE_CURRENT_SOURCE_DIR}")
include("${CMAKE_CURRENT_SOURCE_DIR}/cmake/stm32g431.cmake")

# Build every device by default; a single device can be built with
#   cmake --build --preset <name> --target <name>
add_ursa_device(valve)
add_ursa_device(pump)
add_ursa_device(phtemp)
