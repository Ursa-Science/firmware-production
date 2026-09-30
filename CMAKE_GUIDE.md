e# CMake Build Guide — URSA CANopen Slave Modules

A step-by-step explanation of how the firmware for the valve, pump, and pH/temp
modules is built with CMake — **what** each piece configures and **why** it is
configured that way. Read this alongside the four build files it describes.

> Scope: this guide covers the build system only. It intentionally does **not**
> cover release tagging / versioning — that process is not yet defined. It will be
> added here once the plan is settled.

---

## 1. The big picture

Four files drive the whole build:

| File | Role |
|------|------|
| `cmake/arm-none-eabi-gcc.cmake` | **Toolchain file** — tells CMake to cross-compile for the STM32 with a specific, pinned ARM GCC. Read *before* the project is configured. |
| `cmake/stm32g431.cmake` | **Shared MCU settings + the `add_ursa_device()` helper** — all the flags, defines, include paths, and output rules common to every device, in one reusable function. |
| `CMakeLists.txt` | **Project entry point** — includes the helper and declares the three devices. |
| `CMakePresets.json` | **Canned configure/build commands** — so nobody has to remember the toolchain path or generator. |

Flow when you run a build:

```
cmake --preset default
        │
        ├─ loads CMakePresets.json  → generator = Ninja, toolchain file, build/ dir
        ├─ runs the toolchain file  → picks + verifies GCC 14.3.1, sets cross-compile mode
        └─ runs CMakeLists.txt      → include(stm32g431.cmake); add_ursa_device(valve|pump|phtemp NODE_IDS ...)
                                         │
                                         └─ each add_ursa_device() defines one .elf target PER NODE ID
                                            (<device>-nNN) + its .bin/.hex/size post-build step

cmake --build --preset valve-n09  → compiles that target, emits build/valve-n09-250k.{elf,bin,hex,map}
```

The design goal behind all of it: **one build, one copy of the shared code, three
devices** — so a fix in the CANopen stack or a HAL update happens once, not three
times.

---

## 2. The toolchain file — `cmake/arm-none-eabi-gcc.cmake`

A *toolchain file* is special: CMake reads it before it knows anything about the
project, to answer "what compiler am I using and for what target?" This is the
right place for cross-compilation setup because it must be established before the
first compiler probe.

### 2.1 Declaring the target is not the host

```cmake
set(CMAKE_SYSTEM_NAME      Generic)   # bare-metal, no OS
set(CMAKE_SYSTEM_PROCESSOR arm)
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)
```

- `CMAKE_SYSTEM_NAME Generic` tells CMake this is a bare-metal target (no operating
  system). Setting it at all is what flips CMake into **cross-compiling** mode.
- `CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY` — **why:** when CMake first checks
  the compiler works, it tries to build *and link a small executable*. Our link
  step needs a linker script and startup code that don't exist yet at probe time,
  so a normal test executable would fail to link and abort configuration. Telling
  CMake to compile only to a static library skips the link during the probe.

### 2.2 Pinning the compiler — the most important decision

```cmake
set(URSA_EXPECTED_GCC_VERSION "14.3.1")
```

**Why pin at all:** two ARM toolchains exist on the build machine — STM32CubeIDE
ships GCC **14.3.1**, STM32CubeCLT ships GCC **13.3.1** — and they produce
*different binaries* (different newlib internals, different code size). Every
binary flashed to hardware to date was built with 14.3.1. Pinning guarantees the
CMake build reproduces that exact compiler, so the migration to CMake changes only
the *build system*, never the *compiler*. Moving to 13.3.1 later is defensible
(it's CLI-native and better for CI) but must be treated as its own validated
change, not an accident of whichever tool is on `PATH`.

### 2.3 Finding the toolchain by glob, not hardcoded path

```cmake
if(NOT TOOLCHAIN_BIN)
    file(GLOB _ursa_candidates
         ".../gnu-tools-for-stm32.14.3.rel1.*/tools/bin")
    ...pick the newest that actually contains arm-none-eabi-gcc...
    if(NOT TOOLCHAIN_BIN)
        message(FATAL_ERROR "Could not locate the pinned GCC ... ")
    endif()
endif()
```

- **Why glob:** the STM32CubeIDE plugin directory name embeds a build timestamp
  that changes every time the IDE updates. A hardcoded path would silently break
  on the next update; the glob survives it.
- **Why `FATAL_ERROR` and no fallback:** if 14.3.1 isn't found, the build stops
  with a clear message. It deliberately does **not** silently fall back to
  CubeCLT's 13.3.1 — a silent substitution would reintroduce exactly the
  "which compiler built this?" ambiguity the pin exists to remove.
- **Escape hatch:** pass `-DTOOLCHAIN_BIN=<path>` to point somewhere else (e.g. to
  try 13.3.1 deliberately). The `if(NOT TOOLCHAIN_BIN)` guard means an explicit
  override wins.

### 2.4 Naming the tools

```cmake
set(CMAKE_C_COMPILER   "${TOOLCHAIN_BIN}/arm-none-eabi-gcc")
set(CMAKE_ASM_COMPILER "${TOOLCHAIN_BIN}/arm-none-eabi-gcc")   # gcc assembles the .s startup
set(CMAKE_CXX_COMPILER "${TOOLCHAIN_BIN}/arm-none-eabi-g++")
set(CMAKE_OBJCOPY      "${TOOLCHAIN_BIN}/arm-none-eabi-objcopy" CACHE FILEPATH "objcopy")
set(CMAKE_SIZE         "${TOOLCHAIN_BIN}/arm-none-eabi-size"    CACHE FILEPATH "size")
```

`OBJCOPY` and `SIZE` are captured here because the per-device output step (§3.7)
uses them to make the `.bin`/`.hex` and print the size report.

### 2.5 Asserting the version — fail loud, fail early

```cmake
execute_process(COMMAND "${CMAKE_C_COMPILER}" -dumpversion OUTPUT_VARIABLE _ursa_gcc_version ...)
if(NOT _ursa_gcc_version STREQUAL URSA_EXPECTED_GCC_VERSION)
    message(FATAL_ERROR "Toolchain version mismatch. expected 14.3.1 / found ${_ursa_gcc_version} ...")
endif()
message(STATUS "URSA toolchain: GCC ${_ursa_gcc_version} (pinned) at ${TOOLCHAIN_BIN}")
```

**Why:** even with the glob, someone could override `TOOLCHAIN_BIN` to the wrong
compiler. This actively runs `gcc -dumpversion` and aborts on any mismatch, so a
wrong compiler is caught at *configure* time with a clear message — instead of
surfacing weeks later as an unexplained change in binary size. The `STATUS` line
prints the resolved compiler on every configure, so it's never a guess. (Verified:
pointing `TOOLCHAIN_BIN` at CubeCLT's 13.3.1 correctly aborts.)

### 2.6 Where CMake is allowed to search

```cmake
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)   # host tools OK
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)    # only target libs
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)    # only target headers
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)
```

Standard cross-compile hygiene: `find_library`/`find_path` must only pick up
**target** (ARM) libraries and headers, never the host Mac's, while host-run
*programs* are still findable. Prevents accidentally linking a macOS library.

---

## 3. Shared settings and the device factory — `cmake/stm32g431.cmake`

This file is the heart of the build. It defines the flags every device shares,
then wraps the entire "how to build one device" recipe in a single function so
each device is one line.

### 3.1 CPU / FPU flags

```cmake
set(URSA_MCU_FLAGS -mcpu=cortex-m4 -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb)
```

The STM32G431 is a Cortex-M4 with a single-precision hardware FPU. These four
flags must be **identical** on both the compile and link lines (they are — §3.5,
§3.6), otherwise GCC links incompatible libraries. `-mfloat-abi=hard` uses FPU
registers for float arguments; `-mthumb` selects the Thumb-2 instruction set.

*Provenance:* every flag in this file was lifted verbatim from the
STM32CubeIDE-generated makefiles (`devices/*/Debug/subdir.mk`) so the CMake output
matches what the IDE produced. Two report-only flags were dropped:
`-fcyclomatic-complexity` (ST-patched-GCC metrics only, no codegen effect);
`-fstack-usage` was kept (emits harmless `.su` files).

### 3.2 Common preprocessor defines

```cmake
set(URSA_COMMON_DEFINES DEBUG SRAM_SIZE=SRAM1_SIZE_MAX USE_HAL_DRIVER
    STM32G431xx USE_CUBEMX USE_FDHAL PCLK=48 USE_LSS_SERVER=0)
```

These are identical across all three devices (confirmed against the IDE builds).
`STM32G431xx` selects the CMSIS device header; `USE_HAL_DRIVER` enables the HAL;
`USE_FDHAL` selects the FDCAN hardware layer. **Note what is *not* here:** CAN
bitrate and node ID are *not* compile defines — they live in each device's
`main.h` / object dictionary, which is why they don't appear in this shared list.

### 3.3 The single linker script

```cmake
set(URSA_LINKER_SCRIPT "${CMAKE_CURRENT_LIST_DIR}/STM32G431KBTX_FLASH.ld")
```

All three devices are the same MCU (STM32G431KBT6, 128 KB flash / 32 KB RAM), so
one linker script serves all of them. `CMAKE_CURRENT_LIST_DIR` resolves relative
to this `cmake/` folder.

### 3.4 Gathering shared source — and why it's compiled *per device*

```cmake
file(GLOB URSA_HAL_SOURCES CONFIGURE_DEPENDS "${URSA_ROOT}/vendor/STM32G4xx_HAL_Driver/Src/*.c")
list(FILTER URSA_HAL_SOURCES EXCLUDE REGEX "_template\\.c$")
file(GLOB URSA_MCO_SOURCES CONFIGURE_DEPENDS "${URSA_ROOT}/shared/mco/*.c")
set(URSA_STARTUP "${URSA_ROOT}/shared/startup/startup_stm32g431kbtx.s")
```

- The HAL and the CANopen stack (`shared/mco`) are collected once here.
- `_template.c` files ship with the HAL as copy-paste examples and must be
  excluded or they break the build.
- **The critical design decision:** these shared sources are compiled *into each
  device target* (see §3.5) rather than built once into a shared static library.
  **Why:** each device supplies its *own* `stm32g4xx_hal_conf.h`, `mcohw_cfg.h`,
  and `nodecfg.h`, which these shared `.c` files `#include`. A single shared
  library would bake in one device's configuration and silently miscompile the
  other two. Per-device compilation costs a little build time; correctness wins.
- **On `GLOB` + `CONFIGURE_DEPENDS`:** globbing sources is normally discouraged in
  CMake because new files aren't picked up automatically. `CONFIGURE_DEPENDS` makes
  the build re-glob when the directory changes, which removes that footgun and
  suits vendored trees (HAL, stack) we don't hand-edit.

### 3.5 `add_ursa_device()` — the per-device recipe

Everything below lives inside `function(add_ursa_device name)`. Calling it once
per device produces one fully-configured firmware target.

**Sources** — device code + the shared sets:

```cmake
file(GLOB dev_sources CONFIGURE_DEPENDS
     "${dev}/Core/Src/*.c" "${dev}/MCO_Target/*.c" "${dev}/MCO_CiA401__User/*.c")
add_executable(${name} ${dev_sources} ${URSA_HAL_SOURCES} ${URSA_MCO_SOURCES}
               ${URSA_STARTUP} ${${name}_EXTRA_SOURCES})
```

Each device contributes its `Core/`, `MCO_Target/` (hardware layer), and
`MCO_CiA401__User/` (object dictionary). `${${name}_EXTRA_SOURCES}` is an optional
hook (§3.8).

**Include paths** — device-specific dirs first, then shared/vendor:

```cmake
target_include_directories(${name} PRIVATE
    "${dev}/Core/Inc" "${dev}/MCO_Target" "${dev}/MCO_CiA401__User" "${dev}/MCO_CiA401__User/EDS"
    "${URSA_ROOT}/shared/mco"
    "${URSA_ROOT}/vendor/STM32G4xx_HAL_Driver/Inc" ".../Inc/Legacy"
    "${URSA_ROOT}/vendor/CMSIS/Device/ST/STM32G4xx/Include" ".../CMSIS/Include")
```

Order matters: the device's own headers come first, so each device's
`stm32g4xx_hal_conf.h` (etc.) is the one that wins — the mechanism that makes
per-device compilation of shared code correct. `PRIVATE` = these apply to this
target only.

**Defines:**

```cmake
target_compile_definitions(${name} PRIVATE ${URSA_COMMON_DEFINES} ${${name}_EXTRA_DEFINES})
```

Shared defines plus an optional per-device hook.

### 3.6 Compile and link options

```cmake
target_compile_options(${name} PRIVATE
    ${URSA_MCU_FLAGS}
    $<$<COMPILE_LANGUAGE:C>:-std=gnu11>   # C-only, so it isn't passed to the .s file
    -g -Os                                # debug info; optimize for size (128 KB flash)
    -ffunction-sections -fdata-sections   # one section per function/datum...
    -Wall -fstack-usage --specs=nano.specs)

target_link_options(${name} PRIVATE
    ${URSA_MCU_FLAGS}                     # SAME cpu/fpu flags as compile
    -T${URSA_LINKER_SCRIPT}               # our linker script
    --specs=nosys.specs --specs=nano.specs -static
    -Wl,--gc-sections                     # ...so the linker can drop unused ones
    -Wl,-Map=$<TARGET_FILE_DIR:${name}>/${name}.map
    -Wl,--start-group -lc -lm -Wl,--end-group)
```

Key pairings and why:

- `-ffunction-sections -fdata-sections` (compile) **+** `-Wl,--gc-sections` (link)
  work as a pair: split everything into its own section, then garbage-collect
  unreferenced ones. Critical on a 128 KB part.
- `-Os` optimizes for size for the same reason.
- `--specs=nano.specs` uses newlib-nano (a much smaller libc). `nosys.specs` on the
  link line stubs out OS syscalls (no filesystem/OS on bare metal).
- `${URSA_MCU_FLAGS}` appears in **both** lists — the cpu/fpu/abi must match at
  compile and link or GCC picks incompatible multilib variants.
- The generator expression `$<$<COMPILE_LANGUAGE:C>:-std=gnu11>` applies the C
  standard only to C files, so it isn't (wrongly) passed to the assembler for the
  `.s` startup file.
- `-Wl,--start-group -lc -lm -Wl,--end-group` resolves circular references between
  libc and libm.

```cmake
set_target_properties(${name} PROPERTIES SUFFIX ".elf" LINK_DEPENDS "${URSA_LINKER_SCRIPT}")
```

`SUFFIX ".elf"` names the output `valve.elf` (not `valve`). `LINK_DEPENDS` re-links
if the linker script changes.

### 3.7 Producing `.bin`, `.hex`, and a size report

```cmake
add_custom_command(TARGET ${name} POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:${name}> .../${name}.bin
    COMMAND ${CMAKE_OBJCOPY} -O ihex   $<TARGET_FILE:${name}> .../${name}.hex
    COMMAND ${CMAKE_SIZE} $<TARGET_FILE:${name}>
    COMMENT "Generating ${name}.bin / ${name}.hex")
```

The linker makes an `.elf` (with debug info and addresses); flashing tools want a
raw image. This post-build step runs automatically after every link:

- `.bin` — raw image for `st-flash`/J-Link (flashed at `0x08000000`).
- `.hex` — Intel HEX, carries its own addresses (for STM32CubeProgrammer etc.).
- `size` — prints text/data/bss so flash/RAM usage is visible on every build.

### 3.8 The per-device escape hatches

```cmake
#   ${name}_EXTRA_DEFINES  - extra -D symbols for one device
#   ${name}_EXTRA_SOURCES  - extra .c files for one device
```

Today all three devices are structurally identical, so these are unused. They
exist so a future device that needs one extra define or source can set, e.g.,
`set(pump_EXTRA_DEFINES SOME_FLAG)` *before* `add_ursa_device(pump ...)` — without
forking the shared recipe.

### 3.9 Node IDs — one target per node

```cmake
add_ursa_device(pump NODE_IDS 1 2)   # -> targets pump-n01, pump-n02 (+ pump-all)
```

`add_ursa_device()` is a thin loop: for each ID in `NODE_IDS` it calls the internal
`_ursa_device_target()` (the recipe described in 3.1-3.8) with target name
`<device>-nNN` and adds `NODEID=<id>` to the compile definitions.

**Why this works with a single generated OD:** CANopen Architect bakes the node ID
into exactly one macro, `NODEID_DCF` in `EDS/pimg.h`. `nodecfg.h` turns that into
`NODEID` under `#if !defined(NODEID)`, so a `-DNODEID=2` on the command line wins.
Everything else follows at runtime: `MCO_DefaultResetCommunication(NODEID, ...)`
sets `MY_NODE_ID`, and PDO/SDO/EMCY/heartbeat COB-IDs are all computed from it
(the OD stores PDO COB-IDs as 0 = "default for my node"). The one compile-time
straggler, the `[1014h]` EMCY COB-ID readback, is re-pointed at `NODEID` in each
device's hand-maintained `procimg.h`, right after it includes `pimg.h`.

**Why not regenerate per node:** six generated files per node per OD change, and
the copies drift (the first node-2 regeneration came out with a stale
`RevisionNumber`). `pump-n01.bin` and `pump-n02.bin` differ by exactly two bytes:
the node ID passed to the stack and the `[1014h]` readback.

**Why no bare `pump` target:** so a node ID is always named at build and flash
time, and a stale `build/pump.bin` can never be mistaken for current output.

### 3.10 CAN bitrate — one bus-wide setting

```cmake
set(URSA_CAN_BITRATES 20 50 125 250 500 800 1000)
set(URSA_CAN_BITRATE 250 CACHE STRING "CAN bus bitrate in kbit/s (...)")
```

Bitrate belongs to the **bus**, not to a node, so it is one cache variable rather
than a per-device list. Configure fails on any value outside the list, which is
exactly the set the FDCAN driver and `MX_FDCAN1_Init()` carry bit timing for
(80 MHz kernel clock). Two things follow from the value:

- every target gets `-DCAN_BITRATE_<n>K`. `main.h` used to hold an
  "uncomment ONE" selector; it is now only a guarded 250k fallback for builds
  that bypass CMake. The symbol drives both init paths: the CubeMX
  `MX_FDCAN1_Init()` in `main.c` and, via `mcohw_cfg.h` → `CAN_BITRATE`, the
  `MCO_DefaultResetCommunication()` call that re-initialises FDCAN for the stack.
- `OUTPUT_NAME` becomes `<device>-nNN-<rate>k`, so the rate is in every
  `.elf/.bin/.hex/.map` name. The **target** name stays `<device>-nNN`, which is
  why build presets don't multiply per bitrate.

The `default` configure preset pins 250 in its `cacheVariables`, so
`cmake --preset default` always returns `build/` to 250k even if someone passed
`-DURSA_CAN_BITRATE=...` earlier. Other rates use `can-<rate>k` presets with their
own `build-<rate>k/` directory — no full-rebuild thrash when you hop between rates.

**CANopen Architect's bitrate selector is inert here.** Comparing same-day exports
at 50k/125k/500k/800k/1M showed it changes exactly two things: `CAN_BITRATE_DCF`
in `stackinit.h` and `BaudRate=` in the DCF. The EDS, `pimg.h` and
`entriesandreplies.h` are identical. `CAN_BITRATE_DCF` is only a fallback inside
`MCO_Init()` for a bitrate argument of 0, which this firmware never passes (and
`mco.c` does not include `stackinit.h`). Verified: forcing `CAN_BITRATE_DCF 1000`
produces a byte-identical image. So, as with node IDs, never keep per-bitrate
copies of the generated files.

---

## 4. The entry point — `CMakeLists.txt`

```cmake
cmake_minimum_required(VERSION 3.22)
project(ursa-canopen-modules C ASM)          # C and assembly (the .s startup)
set(URSA_ROOT "${CMAKE_CURRENT_SOURCE_DIR}")  # repo root, used throughout stm32g431.cmake
include("${CMAKE_CURRENT_SOURCE_DIR}/cmake/stm32g431.cmake")
add_ursa_device(valve  NODE_IDS 9)
add_ursa_device(pump   NODE_IDS 1 2)
add_ursa_device(phtemp NODE_IDS 4 5 31 32)
```

Deliberately tiny. `project(... C ASM)` enables both the C and assembly languages
(the startup file is `.s`). `URSA_ROOT` is the anchor every path in
`stm32g431.cmake` hangs off. Adding a device is genuinely one line here (plus its
folder — see §6).

---

## 5. Canned commands — `CMakePresets.json`

```json
{ "configurePresets": [ { "name": "default", "generator": "Ninja",
    "binaryDir": "${sourceDir}/build",
    "toolchainFile": "${sourceDir}/cmake/arm-none-eabi-gcc.cmake",
    "cacheVariables": { "CMAKE_BUILD_TYPE": "Debug" } } ],
  "buildPresets": [ { "name": "default", ... },
    { "name": "valve-n09", "targets": ["valve-n09"] }, { "name": "pump-n01", ... },
    { "name": "pump-n02", ... }, { "name": "phtemp-n04", ... } ] }
```

**Why presets exist:** they capture the generator (Ninja), the build directory
(`build/`), and — importantly — the toolchain file, so nobody has to remember to
pass `-DCMAKE_TOOLCHAIN_FILE=...` by hand. `${sourceDir}` keeps it portable across
machines. The per-device build presets are just conveniences that map
`--preset valve-n09` to building the `valve-n09` target. A node ID without its
own preset still builds with `cmake --build --preset default --target <device>-nNN`.

Usage:

```bash
cmake --preset default          # configure once
cmake --build --preset default   # build every device at every node ID (250k)
cmake --build --preset pump-n02  # build just one

cmake --preset can-500k                              # a different bus speed...
cmake --build --preset can-500k --target pump-n02    # ...lands in build-500k/
```

---

## 6. How to add a new device

1. Create `devices/<name>/` containing `Core/{Inc,Src}/`, `MCO_Target/`, and
   `MCO_CiA401__User/` (with its EDS). Easiest start: copy the closest existing
   device and adjust its `main.h`, `MCO_Target/mcohw_cfg.h`, and object dictionary.
2. Add one line to `CMakeLists.txt`: `add_ursa_device(<name> NODE_IDS <id>...)`.
3. Add a build preset to `CMakePresets.json` (copy an existing one, change the
   name and target).
4. `cmake --preset default && cmake --build --preset <name>-nNN`.

To build an **existing** device for an additional node ID, just append the ID to
its `NODE_IDS` list (and optionally add a preset) — no CANopen Architect run.

If the new device needs one extra define or source file, set
`<name>_EXTRA_DEFINES` / `<name>_EXTRA_SOURCES` before its `add_ursa_device()` call
rather than editing the shared recipe.

---

## 7. Design decisions at a glance

| Decision | Why |
|----------|-----|
| Pin GCC 14.3.1, assert the version | Reproduce the exact compiler that built every flashed binary; migration changes the build system only. |
| Glob for the toolchain, `FATAL_ERROR` if absent | Survive IDE updates; never silently substitute a different compiler. |
| Compile shared HAL/stack *into each device* | Each device has its own `*_conf.h` / `mcohw_cfg.h`; a shared lib would miscompile the others. |
| One vendored HAL + one linker script | Same MCU across all devices; de-duplicate. |
| Flags copied verbatim from the IDE build | Make the CMake output match the validated CubeIDE output. |
| `-Os` + section GC + newlib-nano | 128 KB flash budget. |
| `.bin` + `.hex` + size on every build | Ready-to-flash artifacts and visible flash/RAM usage without extra steps. |
| Thin `CMakeLists.txt`, one call per device | Adding a device is one line; the recipe lives in one place. |

---

## 8. Related docs

- **BUILD_NOTES.md** — the detailed running log: findings, the toolchain
  investigation, EDS regeneration, flashing gotchas (watchdog/NRST).
- **Context.md** — condensed state snapshot.
- **README.md** — top-level quick start (build/flash/config).
