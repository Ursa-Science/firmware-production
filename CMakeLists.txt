cmake_minimum_required(VERSION 3.22)

project(ursa-canopen-modules C ASM)

set(URSA_ROOT "${CMAKE_CURRENT_SOURCE_DIR}")
include("${CMAKE_CURRENT_SOURCE_DIR}/cmake/stm32g431.cmake")

# One target per device per CANopen node ID: <device>-nNN (pump-n01, pump-n02, ...).
# To commission a module at a new node ID, add the ID here - no OD regeneration.
# Build one with   cmake --build --preset default --target pump-n02
add_ursa_device(valve  NODE_IDS 8 9)
add_ursa_device(pump   NODE_IDS 1 2)
add_ursa_device(phtemp NODE_IDS 4 5 31 32)
