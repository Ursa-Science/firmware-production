# Toolchain file for the STM32 ARM Cortex-M cross build.
#
# PINNED to GCC 14.3.rel1, the toolchain bundled with STM32CubeIDE. This is the
# compiler that built every binary flashed to hardware to date, so the CMake
# migration changes only the build system and not the compiler.
#
# STM32CubeCLT 1.18.0 ships GCC 13.3.1 instead. It is the better long-term choice
# (CLI-native, no IDE dependency, friendlier to CI) but switching to it changes
# codegen -- including newlib internals -- so treat that as its own validated
# change, not a drive-by. Override with -DTOOLCHAIN_BIN=<path> to try it.

set(URSA_EXPECTED_GCC_VERSION "14.3.1")

set(CMAKE_SYSTEM_NAME      Generic)
set(CMAKE_SYSTEM_PROCESSOR arm)

# Don't try to run the produced binaries when probing the compiler.
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

if(NOT TOOLCHAIN_BIN)
    # The CubeIDE plugin directory embeds a build timestamp that changes whenever
    # the IDE updates, so glob for it rather than hardcoding the full path.
    file(GLOB _ursa_candidates
         "/Applications/STM32CubeIDE.app/Contents/Eclipse/plugins/com.st.stm32cube.ide.mcu.externaltools.gnu-tools-for-stm32.14.3.rel1.*/tools/bin")
    list(FILTER _ursa_candidates EXCLUDE REGEX "\\.jar$")
    list(SORT _ursa_candidates)
    list(REVERSE _ursa_candidates)   # newest plugin build first

    foreach(_c IN LISTS _ursa_candidates)
        if(EXISTS "${_c}/arm-none-eabi-gcc")
            set(TOOLCHAIN_BIN "${_c}")
            break()
        endif()
    endforeach()

    if(NOT TOOLCHAIN_BIN)
        message(FATAL_ERROR
            "Could not locate the pinned GCC ${URSA_EXPECTED_GCC_VERSION} toolchain under "
            "STM32CubeIDE.app. Install it, or pass -DTOOLCHAIN_BIN=<path> explicitly.\n"
            "Deliberately NOT falling back to STM32CubeCLT's GCC 13.3.1: it produces "
            "different binaries, and that substitution should be explicit. See BUILD_NOTES.md.")
    endif()
endif()

set(CMAKE_C_COMPILER   "${TOOLCHAIN_BIN}/arm-none-eabi-gcc")
set(CMAKE_ASM_COMPILER "${TOOLCHAIN_BIN}/arm-none-eabi-gcc")
set(CMAKE_CXX_COMPILER "${TOOLCHAIN_BIN}/arm-none-eabi-g++")
set(CMAKE_OBJCOPY      "${TOOLCHAIN_BIN}/arm-none-eabi-objcopy" CACHE FILEPATH "objcopy")
set(CMAKE_SIZE         "${TOOLCHAIN_BIN}/arm-none-eabi-size"    CACHE FILEPATH "size")

# Fail early and loudly if the compiler on PATH is not the pinned version.
# A silent version drift shows up much later as an unexplained binary diff.
execute_process(
    COMMAND "${CMAKE_C_COMPILER}" -dumpversion
    OUTPUT_VARIABLE _ursa_gcc_version
    OUTPUT_STRIP_TRAILING_WHITESPACE
    ERROR_QUIET
)
if(NOT _ursa_gcc_version STREQUAL URSA_EXPECTED_GCC_VERSION)
    message(FATAL_ERROR
        "Toolchain version mismatch.\n"
        "  expected: GCC ${URSA_EXPECTED_GCC_VERSION}\n"
        "  found:    GCC ${_ursa_gcc_version}\n"
        "  at:       ${TOOLCHAIN_BIN}\n"
        "Change URSA_EXPECTED_GCC_VERSION in this file if the pin is being moved "
        "deliberately, and record the change in BUILD_NOTES.md.")
endif()
message(STATUS "URSA toolchain: GCC ${_ursa_gcc_version} (pinned) at ${TOOLCHAIN_BIN}")

set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)
