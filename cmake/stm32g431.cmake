# Shared MCU settings and the add_ursa_device() helper.
#
# Flags below are lifted verbatim from the STM32CubeIDE-generated makefiles
# (devices/*/Debug/subdir.mk) so CMake output matches what the IDE produced.
# Two deliberate omissions, both report-only and non-codegen:
#   -fcyclomatic-complexity  (ST-patched GCC, emits a metrics report)
#   -fstack-usage            (kept, see below - emits .su files, harmless)

set(URSA_MCU_FLAGS
    -mcpu=cortex-m4
    -mfpu=fpv4-sp-d16
    -mfloat-abi=hard
    -mthumb
)

set(URSA_COMMON_DEFINES
    DEBUG
    SRAM_SIZE=SRAM1_SIZE_MAX
    USE_HAL_DRIVER
    STM32G431xx
    USE_CUBEMX
    USE_FDHAL
    PCLK=48
    USE_LSS_SERVER=0
)

set(URSA_LINKER_SCRIPT "${CMAKE_CURRENT_LIST_DIR}/STM32G431KBTX_FLASH.ld")

# Vendor + shared sources. These are compiled into EACH device target rather than
# into one shared library, because every device supplies its own
# stm32g4xx_hal_conf.h / mcohw_cfg.h / nodecfg.h that these sources include.
file(GLOB URSA_HAL_SOURCES CONFIGURE_DEPENDS
     "${URSA_ROOT}/vendor/STM32G4xx_HAL_Driver/Src/*.c")
list(FILTER URSA_HAL_SOURCES EXCLUDE REGEX "_template\\.c$")

file(GLOB URSA_MCO_SOURCES CONFIGURE_DEPENDS "${URSA_ROOT}/shared/mco/*.c")
set(URSA_STARTUP "${URSA_ROOT}/shared/startup/startup_stm32g431kbtx.s")

# add_ursa_device(<name>)
#   Expects devices/<name>/ to contain Core/{Inc,Src}, MCO_Target/, MCO_CiA401__User/.
#   Optional per-device knobs, set before calling:
#     ${name}_EXTRA_DEFINES  - extra -D symbols
#     ${name}_EXTRA_SOURCES  - extra .c files
function(add_ursa_device name)
    set(dev "${URSA_ROOT}/devices/${name}")

    file(GLOB dev_sources CONFIGURE_DEPENDS
         "${dev}/Core/Src/*.c"
         "${dev}/MCO_Target/*.c"
         "${dev}/MCO_CiA401__User/*.c")

    add_executable(${name}
        ${dev_sources}
        ${URSA_HAL_SOURCES}
        ${URSA_MCO_SOURCES}
        ${URSA_STARTUP}
        ${${name}_EXTRA_SOURCES}
    )

    target_include_directories(${name} PRIVATE
        "${dev}/Core/Inc"
        "${dev}/MCO_Target"
        "${dev}/MCO_CiA401__User"
        "${dev}/MCO_CiA401__User/EDS"
        "${URSA_ROOT}/shared/mco"
        "${URSA_ROOT}/vendor/STM32G4xx_HAL_Driver/Inc"
        "${URSA_ROOT}/vendor/STM32G4xx_HAL_Driver/Inc/Legacy"
        "${URSA_ROOT}/vendor/CMSIS/Device/ST/STM32G4xx/Include"
        "${URSA_ROOT}/vendor/CMSIS/Include"
    )

    target_compile_definitions(${name} PRIVATE
        ${URSA_COMMON_DEFINES}
        ${${name}_EXTRA_DEFINES}
    )

    target_compile_options(${name} PRIVATE
        ${URSA_MCU_FLAGS}
        $<$<COMPILE_LANGUAGE:C>:-std=gnu11>
        -g -Os
        -ffunction-sections -fdata-sections
        -Wall
        -fstack-usage
        --specs=nano.specs
    )

    target_link_options(${name} PRIVATE
        ${URSA_MCU_FLAGS}
        -T${URSA_LINKER_SCRIPT}
        --specs=nosys.specs
        --specs=nano.specs
        -static
        -Wl,--gc-sections
        -Wl,-Map=$<TARGET_FILE_DIR:${name}>/${name}.map
        -Wl,--start-group -lc -lm -Wl,--end-group
    )
    set_target_properties(${name} PROPERTIES
        SUFFIX ".elf"
        LINK_DEPENDS "${URSA_LINKER_SCRIPT}"
    )

    # Both .bin (for J-Link/ST-Link raw flashing) and .hex, plus a size report.
    add_custom_command(TARGET ${name} POST_BUILD
        COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:${name}> $<TARGET_FILE_DIR:${name}>/${name}.bin
        COMMAND ${CMAKE_OBJCOPY} -O ihex   $<TARGET_FILE:${name}> $<TARGET_FILE_DIR:${name}>/${name}.hex
        COMMAND ${CMAKE_SIZE} $<TARGET_FILE:${name}>
        COMMENT "Generating ${name}.bin / ${name}.hex"
    )
endfunction()
