# Context.md — AI rapid-refresh

**Audience: AI assistants only.** Dense state snapshot for reloading context fast.
Not a tutorial and not the running log — that is `BUILD_NOTES.md`, which holds the
detailed findings, gotchas, and history. Read this first, then BUILD_NOTES.md for
depth. Keep this file terse and current; prune stale lines rather than appending.

Last updated: 2026-07-29.

---

## What this project is

Migrating three URSA CANopen slave modules (valve, pump, phtemp) from per-device
STM32CubeIDE projects to ONE shared CMake build. All three are the SAME MCU
(STM32G431KBT6, 128K flash / 32K RAM). Firmware = MicroCANopen Plus stack + STM32
HAL. CAN slaves on a 250 kbit/s bus.

- Working dir (this repo, git, local only): `/Users/dakotaward/code/Cowork-optimizing/ursa-cmake-build`
- Target org repo (push only after HW validation): `/Users/dakotaward/code/firmware-production`
- Spec: `firmware-production/specs.md`

## Status

| Device | Builds | HAL | Hardware |
|---|---|---|---|
| valve  | yes | 1.2.6 | **VALIDATED** 2026-07-29 — UART + full CAN (NMT start, RPDO open/close, TPDO interval). Flashed via st-flash 1.8.0 / ST-LINK V3. |
| pump   | yes | 1.2.6 (was 1.2.5) | not yet — highest regression risk (HAL moved) |
| phtemp | yes | 1.2.6 | **VALIDATED** — CAN on bus + responds to commands; temp probe (DQ→PA6) fix validated 2026-07-29 |

Migration is PROVEN on hardware for valve AND phtemp. CMake+14.3.1 build is
functionally equivalent to the CubeIDE build on real devices. pump not yet
hardware-validated.

### Repo location (as of 2026-07-29)
Code has been copied into the org repo **/Users/dakotaward/code/firmware-production**
(GitHub: Ursa-Science/firmware-production), which is now the authoritative repo.
The old staging repo `/Users/dakotaward/code/Cowork-optimizing/ursa-cmake-build`
was the pre-validation test area. **To avoid a NEW drift (working-dir vs
firmware-production), pick ONE going forward** — recommended: work in
firmware-production and retire the staging dir. Do NOT maintain edits in both.
(This is the same single-source-of-truth lesson as the CubeIDE-tree drift below.)

### phtemp debugging (2026-07-29)
- CAN now works: present on bus, responds to commands. (Earlier "no enumeration"
  was a flash/watchdog issue — see below — not a firmware CAN fault.)
- **Flash gotcha:** phtemp firmware runs an IWDG (2048ms). The phtemp bench
  fixture has **NRST NOT wired**, so a plain `st-flash --reset write` (a ~5s op)
  gets interrupted by the watchdog mid-write → PGSERR / erase failures / the chip
  reset-loops. Fix: wire ST-LINK NRST→board NRST and flash with
  `--connect-under-reset` (this is why valve, whose fixture HAS NRST, never hit
  it). valve ALSO has an IWDG — the differentiator is fixture NRST wiring, not the
  watchdog itself.
- **Temp probe fix (DQ pin):** hardware DQ was reassigned to **PA6**. Our CMake
  tree lagged — it had DQ on **PB0** and PA6 configured as dead `ST1` output
  (referenced by no code). Fixed 2026-07-29: main.h → `DQ_Pin=PA6/GPIOA`, removed
  ST1/ST2 defines + their GPIO init in main.c. phtemp main.c/main.h are now
  byte-identical to the reference tree. Rebuilt: build/phtemp.bin md5
  ff448d83425fe6e78b5c7baf3e8f2b0d. **Pending: reflash (under-reset, NRST wired)
  and confirm the temp probe reads.**

Build all: `cmake --preset default && cmake --build --preset default`
One device: `cmake --build --preset valve`  → `build/<device>.{elf,bin,hex,map}`

## Layout (post-migration)

```
cmake/    toolchain file (arm-none-eabi-gcc.cmake), stm32g431.cmake (flags + add_ursa_device()), 1 linker script
vendor/   HAL 1.2.6 + CMSIS — ONE copy, all devices
shared/   mco/ (CANopen stack, was byte-identical across devices), startup/
devices/  valve|pump|phtemp/ : Core/, MCO_Target/, MCO_CiA401__User/
```
Shared + vendor sources compiled INTO each device target (each has its own
hal_conf / mcohw_cfg / nodecfg), NOT built as a shared lib. 9 mco files × 3 = 27 objs.

## Blast radius (which dir → how many devices)

- `shared/mco/`, `vendor/`  → ALL 3. A change here obliges a 3-device hardware retest.
- `devices/<d>/MCO_Target/`      → that device only (e.g. valve TIM7 vs pump TIM2).
- `devices/<d>/MCO_CiA401__User/` → that device only (object dictionary, EDS, node id).
- `devices/<d>/Core/`            → that device only (app code).

## Decisions locked (do NOT re-raise)

1. **Toolchain pinned to GCC 14.3.1** (STM32CubeIDE's, inside the .app bundle).
   Reason: it built every binary flashed to date → migration changes only the
   build system, not the compiler. Located by glob; configure HARD-FAILS if
   absent (no silent fallback to CubeCLT 13.3.1); version asserted at configure.
   CubeCLT ships 13.3.1 and is the better long-term/CI choice but produces
   different codegen → treat any switch as its own validated change. Override:
   `-DTOOLCHAIN_BIN=<path>`.
2. **CMake-only**; CubeIDE artifacts (.cproject/.project/.settings/.ioc/Debug)
   deleted. `.ioc` recoverable from baseline commit 81578d1 if ever needed.
3. **`_MCOPVERSION_` 5 (generated headers) vs 7 (stack) skew**: KNOWN, ACCEPTED,
   not significant (confirmed by Dakota). Pre-existing. Do not investigate.
4. **`.cax` (CANopen Architect project, on a Windows E:\ drive) is NOT vendored.**
   Repo holds generated MicroCANopen output, not the generator input. Consequence
   accepted: generated EDS headers can only be regenerated on Windows.
5. **MCO_Target/ and Core/Src skeleton left per-device** for now (small deltas;
   avoid changing two variables at once during the CMake migration). Revisit after
   HW validation.
6. `nodecfg.h`/`procimg.h`/`user_od.c` are byte-identical across devices today but
   deliberately NOT hoisted — per-device by design, will diverge.

## Workflows

**Object dictionary / EDS change (per-device):** edit in CANopen Architect
(Windows) → regenerate → copy files into `devices/<d>/MCO_CiA401__User/EDS/` (only
that device; check `git status`) → `cmake --build` → commit with a note of the
actual OD change → re-flash & re-validate that device (SDO/PDO layout changed).
EDS/ files are TOOL OUTPUT, checked in deliberately (generator not on macOS);
hand-edits are lost on regen.

**Shared stack change (all devices):** edit `shared/mco/` → `cmake --build --preset
default` → `arm-none-eabi-size build/*.elf` (if a "valve-only" change moved pump/
phtemp .text, it reached too far) → flash device under test → RE-VALIDATE OTHER
TWO before done.

## Open threads / next

- valve DONE (validated 2026-07-29). st-flash 1.8.0 handled ST-LINK V3 fine (V3
  caveat did not bite). Flash cmd: `st-flash --connect-under-reset --reset write
  build/valve.bin 0x08000000`.
- NEXT: validate **phtemp** (Dakota chose to do phtemp before pump). Then pump
  (watch TMC2209 UART / stepper — its HAL moved 1.2.5→1.2.6, highest regression
  risk). A CAN harness exercising all 3 is worth it (3-device retest rule).
- **Tagging DEFERRED by decision (2026-07-29): do not tag any device yet.** Revisit
  the tag/versioning scheme LATER, after more devices are validated. Do not raise
  tagging again until Dakota brings it up. (Context: tag = qualified release, needed
  eventually for field traceability + the manufacturing pipeline.)
- Flashing is via st-flash (Dakota's habit), not STM32_Programmer_CLI. Cmd:
  `st-flash --connect-under-reset --reset write build/<dev>.bin 0x08000000`
  (use `--connect-under-reset` when the target firmware runs an IWDG — needs NRST
  wired on the fixture; without NRST, drop to `--reset` on a blank chip only).

### OPEN ACTION — tree drift / single source of truth (raised 2026-07-29)
The old CubeIDE tree `/Users/dakotaward/code/CANopen-nucleog43k1b/CustomBoard/` and
this CMake tree have DIVERGED. The DQ→PA6 change was made in the CubeIDE tree and
did NOT reach here until manually fixed (see phtemp temp probe fix). The whole
point of the migration was ONE source of truth. TODO before more feature work:
- Reconcile each device's sources between the two trees (for phtemp, only
  main.c/main.h differed and are now aligned; valve/pump not yet re-checked).
- Formally RETIRE the CubeIDE tree so future pin/tuning changes land only here.
- Until retired, every "is this change in the right tree?" question will recur.
- Do NOT edit code in the CubeIDE tree (Dakota's instruction); it is reference-only.
- Then: write CMake walkthrough + tag-strategy doc (specs item 4), push to
  firmware-production.

### Manufacturing pipeline (discussed 2026-07-27, NOT yet built)
Test stations on Linux boxes, 3 functional-test branches (pHtemp/valve/pump),
first step flashes "recent" .bin. Current repo does NOT support this — no artifact
publishing; `.bin` only lands in local `build/`. Design direction agreed in
discussion, not implemented:
- Stations FETCH prebuilt qualified artifacts + verify sha256 + flash. Stations do
  NOT build (14.3.1 pin lives in a macOS .app → not reproducible on Linux runners).
- Artifacts keyed by TAG (e.g. `valve/v1.2.0`), immutable, built once by CI/Mac;
  publish .bin + .sha256 + manifest(version, git SHA, toolchain, date). Stations
  pin a version, never chase latest.
- Toolchain tension surfaces here: Mac build keeps 14.3.1; Linux CI would force the
  13.3.1 question. Defer until devices validated.
- Suggested first step: a `release.sh <tag>` → build → emit .bin/.sha256/manifest
  into a versioned dir (works with manual upload, CI-ready later). NOT written yet.
- Artifact host (GitHub Releases? file server?), existing station framework, and CI
  availability are UNKNOWN — must ask before building the pipeline.

### Identity object (0x1018) findings — relevant to functional test
- Product Code (sub2) cleanly IDs device type over CAN: pump=3, phtemp=4, valve=5.
  Station can verify-by-bus it flashed the right image.
- Revision Number (sub3) is STATIC 0x00010001 on all 3 → does NOT track firmware
  version. Device can't report its fw version over CAN. Would need build-time stamp.
- Serial Number (sub4) is a per-TYPE constant (valve 0x123444 etc.), NOT per-unit →
  can't uniquely ID a unit on the bus. Manufacturing needs per-unit provisioning at
  test time (firmware change, not build-system).
- `shared/mco/svninfo.h` is a DEAD SVN-era file (REV 5559 hardcoded, referenced by
  nothing). Vestige of the version-stamping now needed. Wire to git or delete.
