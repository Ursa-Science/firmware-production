# Context.md — AI rapid-refresh

**Audience: AI assistants only.** Dense state snapshot for reloading context fast.
Not a tutorial and not the running log — that is `BUILD_NOTES.md`, which holds the
detailed findings, gotchas, and history. Read this first, then BUILD_NOTES.md for
depth. Keep this file terse and current; prune stale lines rather than appending.

Last updated: 2026-08-10.

---

## What this project is

Migrating three URSA CANopen slave modules (valve, pump, phtemp) from per-device
STM32CubeIDE projects to ONE shared CMake build. All three are the SAME MCU
(STM32G431KBT6, 128K flash / 32K RAM). Firmware = MicroCANopen Plus stack + STM32
HAL. CAN slaves on a 250 kbit/s bus.

- Working dir = THIS repo: `/Users/dakotaward/code/firmware-production`
  (authoritative; the old Cowork-optimizing/ursa-cmake-build staging dir is retired)
- Spec: `specs.md`
- **CAN master = `~/code/CANOpenGateway`** (Lely coapp C++ bridge + TypeScript
  REST/SSE/MCP API, port 8080; python-canopen driver retired). It consumes
  TPDOs into a per-device shadow and EMCYs natively; SYNC 10 Hz default,
  runtime-settable. Keeps its OWN copies of the device EDS/DCF in
  `bridge/devices/` baked into the bridge Docker image — every EDS change
  must be copied there too + image rebuilt.

## Status

| Device | Builds | HAL | Hardware |
|---|---|---|---|
| valve  | yes | 1.2.6 | **VALIDATED** 2026-07-29 — UART + full CAN (NMT start, RPDO open/close, TPDO interval). Flashed via st-flash 1.8.0 / ST-LINK V3. |
| pump   | yes | 1.2.6 (was 1.2.5) | **VALIDATED** 2026-08-07 (Firmware 2.0.0, Nema-17): CAN (2026-08-05: NMT, PDO run@10ml/min, quick stop, step rate) + TMC2209 UART CONFIG VERIFIED BY CHIP + PA5/DIAG refactor + RTT logging, all on good board via RTT observation. |
| phtemp | yes | 1.2.6 | **VALIDATED** — CAN on bus + responds to commands; temp probe (DQ→PA6) fix validated 2026-07-29 |

Migration is PROVEN on hardware for ALL THREE devices. CMake+14.3.1 build is
functionally equivalent to the CubeIDE build on real devices.

### pump TMC2209 UART — CLOSED 2026-08-07 (see TMC2209_UART_AUDIT.md for full record)
- **Logging is now SEGGER RTT over SWD** (commit `28e2a3f`): same DBG_*/Log API,
  RTT channel 0, 4K buffer, PRIMASK lock. USART2 (PA2/PA3, PDN_UART net) is
  TMC-only. View: `probe-rs attach --chip STM32G431KBTx build/pump.elf` (works
  with ST-LINK; JLinkRTTViewer with J-Link). NEVER attach a serial adapter to
  PA2/PA3 — a 5 V-logic adapter TX on that net kills comms and can damage the
  TMC PDN pin.
- **Problem 1 (bad board)** = pure hardware (PDN clamped 0.60 V, MCU pin or GND
  short); board RETIRED. Likely origin: 5 V adapter TX clamp current into PDN.
- **Problem 2 ("df4a262 regression")** = observer artifact, NOT a firmware bug.
  The PA5→DIAG/EXTI refactor re-applied on the RTT base (`86af79f`) verifies
  cleanly (CONFIG VERIFIED BY CHIP, MODER=0xAA9652AE, DIAG armed healthy, motor
  runs). All prior "RX dead" evidence was read through a serial monitor that was
  electrically perturbing the TMC bus, with warm resets that never reset the TMC.
- **TMC2209 current config LANDED by bench sweep 2026-08-07** (commit `1ee0ed9`):
  IRUN=18 (1.48 A pk / 1.05 A rms), IHOLD=8, VSENSE must stay 0. Full sweep
  table + rationale in the header doc block of
  `devices/pump/Core/Inc/tmc2209_uart.h`. Do not re-tune without new data.
- **Runtime TMC health poll** (commit `ab86db1`): DRV_STATUS read every 5 s
  while moving → `[TMC]` RTT line (CS_actual + decoded faults). Gated by
  `DBG_TMC_ENABLE` in log.h — flag 0 compiles out the UART reads too. Same
  commit FIXED the DRV_STATUS bit defines (were TMC2130-layout; TMC2209 flags
  are bits 0..11).
- **BUG (open, fix in fault-feedback Phase 1):** `main.c` writes
  `gProcImg[0x67/0x68]` but PIMGEND=0x66 — OUT-OF-BOUNDS past the process
  image on every boot. The claimed P250000/P250100 pimg symbols don't exist.

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
  ff448d83425fe6e78b5c7baf3e8f2b0d; temp probe validated 2026-07-29.

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

- **NEXT: pump fault feedback → master. Plan is `PUMP_FAULT_FEEDBACK_PLAN.md`**
  (read it before starting; corrected 2026-08-10 for the CANOpenGateway
  master). Key facts: a GENERIC fault already reaches the bus (DIAG→EXTI→
  FAULT state→TPDO1 StatusWord 0x0008 + TPDO2 PumpState/ErrorRegister every
  SYNC; pump TPDOs use MCO_InitTPDOFull so the valve 0x1001 bug doesn't
  apply). Gaps: no fault cause; EMCY compiled in (USE_EMCY=1,
  MCOP_PushEMCY) but never called; TMC detail never leaves the board.
  Phase 0 = verify existing path via gateway REST/SSE (debugger-forced
  fault_active; do NOT short motor phases). Phase 1 = firmware-only (fix
  gProcImg OOB bug, cause-coded 0x1001 bits, EMCY on fault entry/exit).
  Phase 2 = EDS objects 0x2500-0x2503 (Windows; propagate to BOTH repos +
  bridge image rebuild). Phase 3 = app layer (MIK web app), not gateway.
- **Pump stepper-code audit (2026-08-11): items 1-3 DONE.** (1) `d14edb7`
  pure removal (−483 lines dead API/state) — HW-validated same day (bench log:
  init verified, run/stop/quickstop at 10-79 RPM clean, step rate 1.00x).
  (2)+(3) `1112e73`: microstep resolution is now the SINGLE compile knob
  **TMC_MICROSTEP** (tmc2209_uart.h, default 8) driving CHOPCONF.MRES + all
  stepper ARR math + STEPS_PER_REV/BASE_STEPS_PER_ML (steps/ml scales with it
  by construction); MS1/MS2 are permanently UART-address pins; Stepper_Microstep_t
  enum + pin-mapping switch deleted; stale IRUN=22 comments fixed (landed 18).
  To retune resolution: change TMC_MICROSTEP, rebuild, re-run catch test.
  `1112e73` HW-VALIDATED 2026-08-11 (bench log: ARRs and flow→RPM mappings
  bit-identical to pre-refactor; real AVR446 decel ramp exercised and clean).
  Remaining item (4), optional dedup refactors: TMC read/ReadDiag paths,
  hybrid-stop sequence (4 copies), pwm_control.arr_value mirror. The dose
  complete/timeout dedup is CANCELLED — superseded by the dose-transfer plan
  below.
- **PLANNED, BLOCKED: strip dosing engine from pump firmware → MIK owns
  dosing.** Plan + FE-conversation agenda: `PUMP_DOSE_TRANSFER_PLAN.md`.
  Do NOT start until Dakota's in-depth FE conversation settles the open
  questions (volume truth, stop-latency tolerance, heartbeat runaway
  failsafe, OD contract). EDS changes must be batched with fault-feedback
  Phase 2 objects into ONE regen cycle. Firmware ends up a plain CiA 402
  velocity device (~400 lines of dose state out of motor_control.c).
- Pump logging = RTT (`probe-rs attach --chip STM32G431KBTx build/pump.elf`);
  console noise: `[STEPPER]` step-rate block prints 1/s — set
  DBG_STEPPER_ENABLE 0 for quieter sessions (timer proven correct). Known
  cosmetic: first step-rate sample after a start straddles the accel ramp and
  prints a bogus "TIMER/ISR fault" analysis line; converges 1.00x next sample.
- valve DONE (2026-07-29); flash cmd: `st-flash --connect-under-reset --reset
  write build/valve.bin 0x08000000`. A CAN harness exercising all 3 is still
  worth building (3-device retest rule).
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
