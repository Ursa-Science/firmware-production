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

# add_ursa_device(<name> NODE_IDS <id> [<id>...])
#   Expects devices/<name>/ to contain Core/{Inc,Src}, MCO_Target/, MCO_CiA401__User/.
#   Defines one target per CANopen node ID, named <name>-nNN (e.g. pump-n02), each
#   compiled with -DNODEID=<id>. That overrides the NODEID_DCF default baked into the
#   generated EDS/pimg.h (see nodecfg.h), so ONE set of CANopen Architect output
#   serves every node ID - do not regenerate the OD per node. There is deliberately
#   no bare <name> target: every artifact names the node it was built for.
#   Optional per-device knobs, set before calling:
#     ${name}_EXTRA_DEFINES  - extra -D symbols
#     ${name}_EXTRA_SOURCES  - extra .c files
function(add_ursa_device name)
    cmake_parse_arguments(PARSE_ARGV 1 arg "" "" "NODE_IDS")
    if(NOT arg_NODE_IDS)
        message(FATAL_ERROR "add_ursa_device(${name}): NODE_IDS is required")
    endif()

    set(targets "")
    foreach(id IN LISTS arg_NODE_IDS)
        if(NOT id MATCHES "^[0-9]+$" OR id LESS 1 OR id GREATER 127)
            message(FATAL_ERROR "add_ursa_device(${name}): node ID '${id}' is not in 1..127")
        endif()
        if(id LESS 10)
            set(target "${name}-n0${id}")
        else()
            set(target "${name}-n${id}")
        endif()
        _ursa_device_target(${name} ${target} ${id})
        list(APPEND targets ${target})
    endforeach()

    # Convenience: `--target <name>-all` builds every node variant of one device.
    add_custom_target(${name}-all DEPENDS ${targets})
endfunction()

function(_ursa_device_target name target node_id)
    set(dev "${URSA_ROOT}/devices/${name}")

    file(GLOB dev_sources CONFIGURE_DEPENDS
         "${dev}/Core/Src/*.c"
         "${dev}/MCO_Target/*.c"
         "${dev}/MCO_CiA401__User/*.c")

    add_executable(${target}
        ${dev_sources}
        ${URSA_HAL_SOURCES}
        ${URSA_MCO_SOURCES}
        ${URSA_STARTUP}
        ${${name}_EXTRA_SOURCES}
    )

    target_include_directories(${target} PRIVATE
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

    target_compile_definitions(${target} PRIVATE
        ${URSA_COMMON_DEFINES}
        ${${name}_EXTRA_DEFINES}
        NODEID=${node_id}
    )

    target_compile_options(${target} PRIVATE
        ${URSA_MCU_FLAGS}
        $<$<COMPILE_LANGUAGE:C>:-std=gnu11>
        -g -Os
        -ffunction-sections -fdata-sections
        -Wall
        -fstack-usage
        --specs=nano.specs
    )

    target_link_options(${target} PRIVATE
        ${URSA_MCU_FLAGS}
        -T${URSA_LINKER_SCRIPT}
        --specs=nosys.specs
        --specs=nano.specs
        -static
        -Wl,--gc-sections
        -Wl,-Map=$<TARGET_FILE_DIR:${target}>/${target}.map
        -Wl,--start-group -lc -lm -Wl,--end-group
    )
    set_target_properties(${target} PROPERTIES
        SUFFIX ".elf"
        LINK_DEPENDS "${URSA_LINKER_SCRIPT}"
    )

    # Both .bin (for J-Link/ST-Link raw flashing) and .hex, plus a size report.
    add_custom_command(TARGET ${target} POST_BUILD
        COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:${target}> $<TARGET_FILE_DIR:${target}>/${target}.bin
        COMMAND ${CMAKE_OBJCOPY} -O ihex   $<TARGET_FILE:${target}> $<TARGET_FILE_DIR:${target}>/${target}.hex
        COMMAND ${CMAKE_SIZE} $<TARGET_FILE:${target}>
        COMMENT "Generating ${target}.bin / ${target}.hex"
    )
endfunction()
