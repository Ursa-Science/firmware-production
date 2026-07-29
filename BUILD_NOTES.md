# URSA CANopen slave modules — CMake build notes

Running log of what has worked and what has not during the STM32CubeIDE → CMake
migration. Purpose: carry context across sessions. Newest findings at the bottom
of each section.

---

## Current state

| Device | Builds | HAL | Validated on hardware |
|---|---|---|---|
| valve  | yes | 1.2.6 | **VALIDATED** — UART + full CAN (see below) |
| pump   | yes | 1.2.6 (upgraded from 1.2.5) | **not yet** |
| phtemp | yes | 1.2.6 | **not yet** |

### valve — hardware validation record (2026-07-29)
CMake build (GCC 14.3.1), flashed via `st-flash` 1.8.0 over ST-LINK V3 (V3J15),
under-reset connect, flash written and verified. Probe confirmed chipid 0x468 /
STM32G43x_G44x, 128K flash / 32K SRAM — matches linker script.
- UART: verified good (earlier).
- NMT: PreOperational → Operational (NMT start) working.
- PDO: RPDO open/close actuation working on a powered, connected valve.
- TPDO: transmitting at the expected interval on the bus.
Conclusion: the CMake-built valve firmware is functionally equivalent to the
CubeIDE build on real hardware. The migration is proven for this device.

Build: `cmake --preset default && cmake --build --preset default`
Single device: `cmake --build --preset valve`
Artifacts land in `build/<device>.{elf,bin,hex,map}`.

---

## Repository layout and why

```
cmake/     toolchain file, shared MCU flags, the single linker script
vendor/    STM32 HAL 1.2.6 + CMSIS — ONE copy for all devices
shared/    mco/ (CANopen stack, was byte-identical across devices), startup/
devices/   valve/ pump/ phtemp/ — Core/, MCO_Target/, MCO_CiA401__User/ only
```

**Key design point:** vendor HAL and shared MCO sources are compiled *into each
device target*, not built once as a shared static library. Each device supplies
its own `stm32g4xx_hal_conf.h`, `mcohw_cfg.h` and `nodecfg.h`, which those
sources `#include`. A single shared library would bake in one device's config
and silently miscompile the others.

---

## Findings

### Established before migration
- All three devices are the **same MCU**: `STM32G431KBT6`, 128K flash / 32K RAM.
  The folder formerly named `pHTempModule_stm32g431kt6` was a misnomer (it is a
  KB part); renamed to `phtemp`. One linker script now serves all three.
- `MCO/` (CANopen stack) and `Core/Startup/` were **byte-identical** across all
  three projects → hoisted to `shared/`.
- HAL versions were split: pump on **1.2.5**, valve and phtemp on **1.2.6**.
  valve's and phtemp's 1.2.6 copies were byte-identical apart from phtemp adding
  five I²C files. Adopted **phtemp's copy** (the 1.2.6 I²C superset) as the
  single `vendor/` HAL. Only pump changed version.
- 1.2.5 → 1.2.6 is mostly added `const` qualifiers on HAL signatures
  (`HAL_GPIO_Init(GPIO_InitTypeDef const *)`). Source-compatible in the upgrade
  direction; pump compiled against 1.2.6 with no errors.
- All three devices use **identical** `-D` defines. CAN bitrate and node ID come
  from each device's `main.h`, not from the build system.
- The CubeIDE build referenced MCO headers via **absolute paths into a different
  checkout** (`/Users/dakotaward/code/CANopen-nucleog43k1b/CustomBoard/...`).
  That tree still exists and was verified byte-identical to the working copy, so
  nothing was stale — but it is exactly the fragility this migration removes.

### Toolchain — RESOLVED: pinned to GCC 14.3.1
Two ARM toolchains are on this machine and they produce different binaries:

| Source | GCC | Path |
|---|---|---|
| STM32CubeCLT 1.18.0 | 13.3.1 | `/opt/ST/STM32CubeCLT_1.18.0/GNU-tools-for-STM32/bin` |
| STM32CubeIDE (newer) | **14.3.1** ← pinned | `.../plugins/...gnu-tools-for-stm32.14.3.rel1.macos64.../tools/bin` |

**Pinned to 14.3.1** — the compiler that built every binary flashed to hardware
to date. This keeps the migration to a single variable (build system) rather than
two (build system + compiler) through hardware validation.

Implementation in `cmake/arm-none-eabi-gcc.cmake`:
- The CubeIDE plugin path embeds a build timestamp that changes on IDE update, so
  the toolchain is located by **glob**, newest plugin build first — not hardcoded.
- If 14.3.1 is not found, configure **fails hard**. It deliberately does *not*
  fall back to CubeCLT's 13.3.1; a silent substitution would reintroduce exactly
  the ambiguity this pin resolves.
- A version assertion runs at configure time and aborts on any mismatch.
  Verified working: pointing `-DTOOLCHAIN_BIN` at CubeCLT correctly aborts with
  `expected 14.3.1 / found 13.3.1`.
- Configure prints the resolved toolchain: `-- URSA toolchain: GCC 14.3.1 (pinned) at ...`

Moving the pin later (e.g. to CubeCLT 13.3.1 for CI) means changing
`URSA_EXPECTED_GCC_VERSION` in that file — and it should be treated as its own
validated change with a hardware re-test, not a drive-by.

### Validation of the CMake build against the CubeIDE reference (valve)

Compared `build-gcc14/valve.elf` against the CubeIDE `Debug/*.elf`, both GCC 14.3.1:

- Symbol set: **identical** (457 symbols, none added or removed)
- Per-symbol sizes: **identical** — zero symbols differ
- `.rodata`: **identical** (0x6f0)
- `.data`: **identical** (0xb4)
- RAM total: **identical** — `.bss` −4 bytes and `._user_heap_stack` +4 cancel
- `.text`: +4 bytes — alignment/link-order padding, not code
- `.debug_*`: larger, because all HAL sources are compiled including modules a
  given device does not enable. They generate no code (their content is behind
  `HAL_x_MODULE_ENABLED`) but do carry debug info. Inflates the `.elf` only;
  does not affect the flashed image.

Flash images are **not** byte-identical: the 4-byte `.text` padding shifts every
absolute address, so the vector table differs from byte 5 onward. Byte-identity
was never achievable given that padding. The symbol-level equivalence above is
the stronger evidence and it is clean.

When built with CubeCLT 13.3.1 instead, `.text` is 33748 vs the reference 33780
(32 bytes smaller). That delta includes newlib internals (`_printf_i`,
`__sflush_r`, `memmove`, `_sbrk`), which our source cannot influence — it is
purely the compiler version difference.

---

## Change procedure: what is shared vs per-device

Which directory you edit determines how many devices you have just changed.

| Edit here | Affects | Contains |
|---|---|---|
| `shared/mco/` | **all 3 devices** | the CANopen stack itself — `mco.c`, `mcop.c`, `xsdo.c`, `canfifo.c` |
| `vendor/` | **all 3 devices** | HAL + CMSIS |
| `devices/<dev>/MCO_Target/` | that device only | `mcohw_cfg.h` (valve `TIM7` vs pump `TIM2`), CAN pin mapping, LEDs |
| `devices/<dev>/MCO_CiA401__User/` | that device only | object dictionary, EDS/DCF, `user_cbdata.c`, node ID |
| `devices/<dev>/Core/` | that device only | application code |

The shared sources are compiled **into each device target** (9 MCO source files ×
3 devices = 27 object files), so there is no shared library to rebuild — but a
single edit to `shared/mco/` does land in all three binaries.

**Before editing, ask: is this a stack fix or a device binding?** Most things that
feel device-specific — node ID, bitrate, timer choice, OD entries — already live
in the per-device tiers and are naturally isolated. Genuine stack changes are rare
and inherently global.

### Workflow for a change to `shared/mco/` or `vendor/`

```bash
cmake --build --preset default     # rebuild all three; cheap, proves nothing broke
arm-none-eabi-size build/*.elf     # compare against previous sizes
# flash the device under test, then RE-VALIDATE THE OTHER TWO before considering it done
```

That last step is the obligation the old per-device-copy layout did not impose.
Skipping it leaves pump and phtemp carrying an untested stack change until someone
flashes them weeks later. The size comparison is a useful cheap check in the
meantime: if a change believed to be valve-only moves pump's or phtemp's `.text`,
it reached further than intended.

### If one device genuinely needs to diverge

In order of preference:
1. Compile-time guard — add a define for that device and `#ifdef` in the shared
   source. Fine for one or two small deltas; unreadable past that.
2. Move it to the per-device tier — if the divergence is really hardware binding,
   it belongs in `MCO_Target/`, not the stack.
3. Fork the file out of `shared/` back into that device. Last resort: it
   re-creates the three-copy problem for that file, so document the reason.

Avoid a branch per device — that reintroduces the cherry-pick burden this layout
was built to remove.

---

## Object dictionary / EDS regeneration

**The files in `devices/<dev>/MCO_CiA401__User/EDS/` are tool output, not source.**
Hand edits to them are silently lost on the next regeneration.

Generated by **CANopen Architect Professional 11.62.6737** (Windows GUI tool) from
a single project file:

```
E:\ursaScience\Modules\Modules-1000kbs.cax
```

Generated artifacts per device: `pimg.h`, `stackinit.h`, `entriesandreplies.h`,
`<Device>_public.h`, plus the `.eds` / `.dcf` pair.

### The `.cax` stays outside this repo — decided, do not re-raise

All three devices' object dictionaries derive from that one `.cax`, which lives on
a Windows machine. **Decision (Dakota): it is deliberately not vendored here.**
This repo holds the *generated* MicroCANopen files, not the generator input. Do
not propose adding the `.cax` in future sessions.

Consequence to be aware of, not to fix: the checked-in headers cannot be
regenerated or verified from this repo alone. CANopen Architect on Windows is the
only path, and there is no dependency edge from `.cax` / `.eds` to the generated
headers for CMake to track. Regeneration is necessarily a manual step, which is
why the procedure below is written out.

### Procedure when an object dictionary changes

This is the agreed workflow. The generated MicroCANopen files are checked in
deliberately — the tool that produces them does not run on macOS, so the repo
carries the output.

1. **Generate.** In CANopen Architect Professional (Windows), edit the EDS for the
   device(s) concerned and regenerate the MicroCANopen files.
2. **Copy in.** Drop the regenerated files into
   `devices/<dev>/MCO_CiA401__User/EDS/` — only the affected device's directory.
   The `.cax` holds all three devices, so it is easy to regenerate and overwrite
   more than intended. Check `git status` before staging.
3. **Rebuild.** `cmake --build --preset default`
4. **Commit** the regenerated files with a note of what actually changed in the
   OD ("added 0x6100 sub3", not "regenerated EDS"). The generated headers are
   large and diff noisily, so the message is what makes the change reviewable.
5. **Re-flash and re-validate** the device(s) whose OD changed. An OD change
   alters SDO/PDO layout, so bus-level behaviour must be re-checked, not just
   that the board boots.

Note on ordering: rebuild (3) before commit (4) so a regeneration that does not
compile never reaches the history. If you prefer committing first, at minimum
build before pushing.

Scope: an OD change is **per-device** — it touches only that device's directory
and only that binary needs re-validating. This is unlike a `shared/mco/` change,
which obliges a three-device retest (see the change procedure section above).

---

## What has NOT worked / gotchas

- `git mv` fails after `rm -rf` on tracked paths ("bad source"). Use plain `mv`
  and let git detect renames at `git add` time.
- macOS `awk` has no `strtonum`; compare `nm -S` hex size fields as strings.
- `-fcyclomatic-complexity` from the CubeIDE flags is **omitted** in CMake. It is
  an ST-patched GCC report-only flag with no codegen effect.
- `_MCOPVERSION_` redefined warning: `shared/mco/mco.h` declares **7**, every
  device's `MCO_CiA401__User/EDS/pimg.h` declares **5** — CANopen Architect
  generated the headers against an older MicroCANopen Plus than the stack being
  compiled. **KNOWN AND ACCEPTED — do not re-raise this.** Confirmed by Dakota as
  not significant. It is also pre-existing, present in the original CubeIDE build,
  and not introduced by the CMake migration. Do not spend time investigating it or
  flag it as a finding in future sessions.
- `nodecfg.h`, `procimg.h` and `user_od.c` are currently **byte-identical** across
  all three devices (only `user_cbdata.c` differs). They are deliberately **not**
  hoisted to `shared/`: they are per-device by design and coincide only because
  the devices are similar today. They diverge the moment one device's OD changes.
  Duplication is the correct call here.
- valve's EDS files contain **spaces** in their names
  (`Valve Module-n09-250kbs.eds`) while pump and phtemp use hyphens. Not currently
  a problem — CMake never references them by name — but it will bite any future
  script or CI step that is not carefully quoted. Worth normalising; not done yet,
  as renaming means regenerating from the `.cax` to keep names consistent there.
- Unused-function warnings in `devices/pump/Core/Src/tmc2209_uart.c`
  (`TMC2209_WriteVerified`, `TMC2209_ReadRegisterDiag`). Pre-existing.

---

## Deferred on purpose

- `MCO_Target/` is still **per-device**. Deltas are small (pump `TIM2` vs valve
  `TIM7`, comment drift) and could collapse into one shared file plus a
  `board.h`. Deliberately not done in the same pass as the CMake migration, to
  avoid changing two variables at once. Revisit after hardware validation.
- The `Core/Src` skeleton (`main.c`, `log.c`, `mco_events.c`, `led_control.c`)
  differs per device and was left alone for the same reason.
- CubeMX `.ioc` files were **deleted** (CMake-only was the chosen path). They
  remain in git history at the baseline commit if pin/peripheral regeneration is
  ever needed.

---

## Next steps

1. ~~Resolve the toolchain question and pin it.~~ Done — pinned to GCC 14.3.1.
2. ~~**valve** — validate on hardware.~~ DONE 2026-07-29: UART + full CAN
   (NMT start, RPDO open/close, TPDO interval). Migration proven for valve.
   A CAN harness that exercises all three devices is still worth the setup cost,
   since any `shared/mco/` change obliges a three-device retest.
3. Then **pump** — highest regression risk, it is the device whose HAL moved
   1.2.5 → 1.2.6. Watch stepper/TMC2209 UART behaviour specifically.
4. Then **phtemp**.
5. After all three validate: write the CMake walkthrough + tag strategy doc, and
   push to the `firmware-production` repo.
