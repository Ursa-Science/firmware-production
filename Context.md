# Context.md — AI rapid-refresh

**Audience: AI assistants only.** Dense state snapshot for reloading context fast.
Not a tutorial and not the running log — that is `BUILD_NOTES.md`, which holds the
detailed findings, gotchas, and history. Read this first, then BUILD_NOTES.md for
depth. Keep this file terse and current; prune stale lines rather than appending.

Last updated: 2026-09-30 (end of day). **main == origin/main == 7f15268**;
branch build/node-id-parameter fast-forward merged and pushed (can be deleted).

**VALVE MODULE REFACTOR — Phases 1–2 CODED 2026-09-30, UNCOMMITTED, NOT
FLASHED, NOT BENCH-VALIDATED.** Plan `docs/VALVE_REFACTOR_PLAN.md` (reviewed
same day: D3 rewritten — FAULT is contract-only, stack warnings 0x48xx are
log-only, fatal ≥0x8000 closes then resets; new D9 = a master SDO write to
0x1016 would disarm the firmware-armed HB consumer, Phase 0 must trace what
the gateway writes at node-9 boot; EDS 28→**21** objects). Architect edit
list `docs/VALVE_ARCHITECT_REGEN.md` (Windows step, Phase 3, not done).
Coded (devices/valve only, builds clean, pump/phtemp sizes unchanged):
RTT logging (SEGGER files + log.c from phtemp, USART2 IRQ never enabled),
`FIRMWARE_VERSION "3.0.0"` banner, master-HB consumer (127, 2500 ms) armed
after the TPDO2 re-assert, `ValveControl_ForceClosed()` replaces
ApplyFailSafe (relay OFF + position CLOSED + PI CW zeroed + edge detector
reset) on HB loss / any NMT exit / fatal stack error, DISABLED→IDLE snapshots
the CW level (resume-reopen bug fix), 0x1001 both homes edge-wise,
HEARTBEAT_RESTORED clears stack bit 0, CW bit 7 reset works in every state
+ EMCY 0x0000, 0x2222 demo SDO gone, 0x2100/0x2101/0x2300 accessors + the
unreachable motion-timeout fault branch gone. **Node 8 added as a second
valve target 2026-10-01** (`add_ursa_device(valve NODE_IDS 8 9)`, preset
`valve-n08`): images build/valve-n{08,09}-250k.bin (text 34524, bss 8004
incl. 4 K RTT), n08 vs n09 differ in 7 node-ID-derived bytes. Gateway will
need a node-8 EDS/DCF pair (DCF NodeID edited) like phtemp 5/31/32. Bench
playbook (CANopen Magic frames, nodes 8/9, sections A–N):
**docs/VALVE_BENCH_TEST.md**. **Node 9 FLASHED + bench-tested 2026-10-01
(CANopen Magic only, no gateway):** TPDO2 DLC 1, open/close 49–50 ms, HB loss
→ 0x8130 at 2499.8 ms + TPDOs stop, recovery EMCY 0x0000, NMT start after
loss stays CLOSED (resume-reopen bug fixed), fresh Open works, all NMT exits
close. Traces `cantrace-testing-valve-{baseline,phase1_2_testing}.csv` on the
T5. Still owed: every SDO read (identity, 0x6042/0x6040/0x1001 after loss,
0x1003), reset F, 0x6100 provocation K, node 8, two-valve N, and the
**gateway-as-master trace for D9 (0x1016 write?)**. Observation: power-cycling
the valve holds the bus in error frames for ~1 s. NEXT: finish the SDO rows,
commit Phases 1–2, Architect regen (docs/VALVE_ARCHITECT_REGEN.md).
Start from: `devices/valve/`, the pump + phtemp precedents (dumb-module
pattern: MIK owns config/science, module speaks native units; 0x1001 to BOTH
homes; cause-coded EMCYs; master-HB consumer; RTT logging; no app-level TPDO
triggers — the stack's COS scan sends any PI change). Known valve items:
still on the USART2 ring-buffer log (last device); carries the EmSA 0x2222
demo SDO handler; gateway overrides record a 0x1001 TPDO/SDO disagreement
(the dual-source bug — same fix as pump/phtemp); TPDO2 empty/DLC=0 fix
applied 2026-08 but never flashed (see memory + BUILD_NOTES); RevisionNumber
still 0x00010001; valve EDS file names contain spaces.

**PHTEMP — DONE + HW-VALIDATED 2026-09-30, fw 4.0.0, OD rev 0x00020000.**
Dumb module: 0x6003 = mV×10, 0x6010 = °C×10, health; MIK owns pH/cal.
Highlights (details in docs/PHTEMP_REFACTOR_PLAN.md + BUILD_NOTES.md):
- OD regen #1 (cal objects gone, TPDO1 4 B) + regen #2 (0x2400-0x2402 delta
  thresholds + dead comm objects gone; the MCO stack sends every PI change
  itself, so app thresholds were inert). Cadence knob = 0x1800:03 inhibit.
- 0x1001 written to BOTH homes; EMCY 0x5000 (sensor), 0xFF00 (FAULT),
  0x0000 (reset); absent-probe reporting + re-init on CW 0x80; warm-up latch.
- Master-HB consumer (node 127, 2500 ms): loss → stack 0x8130 + PRE-OP, NO
  FAULT; recovery clears stack ER bit 0; resume = NMT start. **HB loss +
  recovery bench-tested on nodes 4 and 5, 2026-09-30.**
- Logging = SEGGER RTT/SWD (`probe-rs attach --chip STM32G431KBTx
  build/phtemp-nNN-250k.elf`). Node IDs 4/5/31/32 are build targets.
- Bench playbook (CANopen Magic frames, 4 nodes): docs/PHTEMP_BENCH_TEST.md.
  Traces on Samsung T5 `cantrace-testing-pHtemp-refactor{,2}.csv`.
- Gateway: cutover to fw 4.0.0 done on the reference machine (calibration
  works); FE guide docs/PHTEMP_FE_CUTOVER_GUIDE.md. Still owed by the FE:
  re-copy the regen-#2 EDS/DCF, drop the `2400:0` override, per-node EDS+DCF
  pairs for 5/31/32 (DCF NodeID edited — gateway trusts DCF NodeID first).
- Gotchas learned (all in BUILD_NOTES): MCO_TriggerTPDO snapshots the PI at
  call time; serial on the bus comes from MCOUSER_GetSerial (0x12345678),
  not the OD; unknown-object SDO abort is 0x08000000; Architect device
  rename renames output files without changing DCF NodeID — keep the
  canonical `-n04-` name; `.cax` = Modules-base.cax and is in sync.

**PHTEMP DUMB-MODULE REFACTOR (2026-09-22, committed 3f3c7e1 on
build/node-id-parameter):** OD regenerated (11 cal objects gone, TPDO1 6→4 B
= mV|temp, 0x2400 = MillivoltDeltaThreshold) + firmware rewired + 7 review
fixes (0x1001 to BOTH homes, EMCY 0x5000/0xFF00/0x0000, warm-up latch,
absent-probe reporting + re-init on CW 0x80, EmSA 0x2222 demo SDO removed,
dead code). **0x6003 + 0x2400 = mV × 10** (like Temperature; ≈10240 at pH 7).
RevisionNumber HAND-EDITED to **0x00020000** (regen produced 0x00010002) in
.eds/.dcf/pimg.h → **.cax is BEHIND, sync before the next phtemp regen.** FW
banner 4.0.0. Build: `--preset phtemp-n04` → build/phtemp-n04-250k.bin, text
41384 B. NOT bench-validated. **NEXT = coupled gateway cutover**: the master
image checks 1018:03 at boot, so gateway EDS/DCF + overrides (drop `6000:0`,
`6003:0`/`2400:0` scale 0.1) + docs/tests + image rebuild + flash are ONE
window. Plan + checklists: docs/PHTEMP_REFACTOR_PLAN.md,
docs/PHTEMP_EDS_REGEN_CHANGELIST.md. Follow-ups: pump/valve still carry the
0x2222 demo SDO (per-device change → re-validate each); EDS deletion candidates
(0x1010/1011/1020/1002/1012/1013/1006/1007/1019/1028/2402) for the next regen.

**NODE ID = BUILD PARAMETER (2026-09-21, branch build/node-id-parameter):**
`add_ursa_device(<dev> NODE_IDS ...)` → one target per node, `<dev>-nNN`
(pump-n01, pump-n02, valve-n09, phtemp-n04/n05/n31/n32), each `-DNODEID=<id>` overriding
NODEID_DCF via the existing `#if !defined(NODEID)` in nodecfg.h. NO bare
pump/valve/phtemp targets or presets any more — old `build/pump.*` on disk is
STALE, don't flash it. One canonical EDS/OD per device; NEVER regenerate per node
(node-2 regen came out with stale RevisionNumber 0x00010001 — rejected, not
imported). [1014h] readback re-pointed to NODEID in each procimg.h. Verified:
n01/n09/n04 images byte-identical to pre-change main; pump-n02 differs from
pump-n01 by 2 bytes (node ID + 1014h). pump-n02 NOT yet bench-validated.
OrderCode stays URSA-PMP-001 for all pumps. Details: CMAKE_GUIDE.md §3.9.

**CAN BITRATE = BUILD PARAMETER (2026-09-21, same branch):** bus-wide cache var
`URSA_CAN_BITRATE` (20/50/125/250/500/800/1000; `default` preset pins 250) →
`-DCAN_BITRATE_<n>K`; main.h "uncomment ONE" selector replaced by a guarded 250k
fallback. Artifacts are now `<dev>-nNN-<rate>k.*` (e.g. build/pump-n02-250k.bin);
targets/presets keep the `<dev>-nNN` name. Other rates: `cmake --preset can-500k`
→ `build-500k/`. Architect's bitrate selector is INERT (only CAN_BITRATE_DCF in
stackinit.h + DCF BaudRate=; proven byte-identical) — leave it at 250, never keep
per-bitrate generated copies. Verified: 250k images byte-identical to 8f92013;
500k build sets prescaler 10 + passes 500 to the stack. Non-250k rates NOT
bench-validated. EDS claims BaudRate_10=1 but firmware has no 10k timing.
Details: CMAKE_GUIDE.md §3.10.

SINCE 79b4e7e (not yet all committed): (1) **StepsRemaining fidelity fix**
(Q1/Q7) in motor_control.c — 0x2603 latches true steps-not-delivered at every
stop instead of the stepper counter zeroing on abort; Phase A DONE, Phase B
(pause physical over-deliver) gated. See docs/PUMP_STEPSREMAINING_FIDELITY_PLAN.md.
(2) **OD RevisionNumber bumped 0x00010001→0x00020000** (0x1018sub3 + [DeviceInfo])
to advertise the steps-only OD; regenerated MicroCANopen sources in-tree; FW now
reports **2.0.0**. Gateway bridge/devices/ copies NOT yet updated. Both changes
VALIDATED in CAN trace 2026-08-25 (quick-stop holds StepsRemaining; dosing exact;
~2.5s HB-loss watchdog fires).

⚠️ **BLOCKER (HW, 2026-08-25): pump TMC2209 UART RX path dead** — PDN_UART idles
at **1.46 V** (should be ~2.6 V), below V_IH → IFCNT reads fail, config written
UNVERIFIED. Developed fault after a session of abrupt de-energizations; persists
across a 10-min cold boot (∴ not firmware — no NVM). Divider math ⇒ new ~5 kΩ
leak to GND on PDN = damaged pin, most likely the TMC2209 module. Motor still
doses at correct 1/8 (pin-implied); SpreadCycle + current UNVERIFIED. Next step:
remove module + re-measure PDN idle. Full writeup + test plan:
docs/PUMP_TMC2209_RX_DEAD_DEBUG_TICKET.md.

main = origin/main = 7f15268 (2026-09-30). Pump image = build/pump-n01-250k.bin
(FW 2.0.0). Phtemp images = build/phtemp-n{04,05,31,32}-250k.bin (FW 4.0.0,
OD rev 2, RTT, HB consumer — flashed + validated on 4/5/31).

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
| phtemp | yes | 1.2.6 | **VALIDATED 2026-09-30, fw 4.0.0 / OD rev 0x00020000** — identity, TPDOs (4 B), pH 4/7/10 levels, temp-unplug EMCY byte-exact, 0x1001 both homes, RTT logging, master-HB loss + recovery (nodes 4 + 5). Temp probe DQ→PA6 (2026-07-29). |

Migration is PROVEN on hardware for ALL THREE devices. CMake+14.3.1 build is
functionally equivalent to the CubeIDE build on real devices.

### pump TMC2209 UART — CLOSED 2026-08-07 (see TMC2209_UART_AUDIT.md for full record)
- **Logging is now SEGGER RTT over SWD** (commit `28e2a3f`): same DBG_*/Log API,
  RTT channel 0, 4K buffer, PRIMASK lock. USART2 (PA2/PA3, PDN_UART net) is
  TMC-only. View: `probe-rs attach --chip STM32G431KBTx build/pump-n01-250k.elf` (works
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
- OOB `gProcImg[0x67/0x68]` boot-write bug: FIXED in fault-feedback Phase 1
  (`0e7ea36`, writes deleted; values return in Phase 2 with real 0x2500/0x2501).

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
One device: `cmake --build --preset valve-n09` (pump-n01, pump-n02, phtemp-n04, -n05, -n31, -n32)
→ `build/<dev>-nNN-250k.{elf,bin,hex,map}`. No bare `<dev>` targets exist.

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
   accepted: generated EDS headers can only be regenerated on Windows. Project
   is now `E:\ursaScience\Modules\Modules-base.cax` (was Modules-1000kbs.cax).
   **Hand-edits to generated files are allowed only for value changes that
   touch no index table** (e.g. the phtemp RevisionNumber 2026-09-22), must be
   marked `HAND-EDITED <date>` in every touched file, and must be mirrored in
   the .cax before the next regen of that device or they silently revert.
   Object add/delete/rename → regen only.
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

- **Pump fault feedback (plan `docs/PUMP_FAULT_FEEDBACK_PLAN.md`): Phase 0a
  DONE, Phase 1 CODED + BENCH-VALIDATED 2026-08-12** (PROOF3 trace: EMCY
  `21 71 01 00 00 00 00 03` on real ISR fault path; SDO 0x1001==TPDO2==0x01;
  reset → EMCY 0x0000; DIAG-classification branches code-reviewed only).
  Adjacent bug (driver stayed ENERGIZED after fault reset from stepper-origin
  fault, post-reset SW=0x0250 bit4) FIXED `0ec936a` + HW-validated same day
  (post-reset SW=0x0240, zero hold current). Bench gotchas now in plan doc:
  pump IWDG kills halted debug sessions (~2 s budget; freeze via
  DBGMCU_APB1FZR1 bit12 + gdb `set mem inaccessible-by-default off`);
  build has 1-byte enums (Tag_ABI_enum_size=small) — take struct-member
  addresses from DWARF (`gdb --batch -ex "print &'file.c'::var.member"`),
  never hand-compute.
  Phase 0a (CANopen Magic bench, traces on Samsung T5) proved fault→both
  TPDOs within one SYNC + edge-triggered reset (repeat `84` frame is a NO-OP;
  use `04`→`84`), and caught the **0x1001 dual-source bug**: SDO + EMCY serve
  gMCOConfig.error_register, TPDO2 serves the procimg copy; app only wrote
  procimg → SDO said 0x00 during fault. Phase 0b (gateway) SKIPPED for now
  (carry-forward: does dcfgen boot DCF write 0x1016? gateway shadow on the
  0x1001 disagreement?). Phase 1 changes (devices/pump only, builds clean):
  main.c OOB gProcImg[0x67/0x68] writes DELETED; MotorControl_EnterFault()
  classifies cause from DRV_STATUS at DIAG-fault time (short→ER|=0x02/EMCY
  0x2310, overtemp→ER|=0x08/0x4210, stepper→0x7121, unreadable→ER|=0x10),
  writes BOTH 0x1001 homes, pushes EMCY w/ DRV_STATUS snapshot + source byte
  (future 0x2503 enum); fault reset clears both (motor-owned bits only) +
  EMCY 0x0000. NOTE: debugger-poking fault_active no longer shows ER=0x01
  (bypasses EnterFault by design) — validate via STEPPER position-limit
  poke or gdb `call`. Phase 2 = EDS 0x2500-0x2503 — DONE (landed + validated
  with the dose strip; see the dose-strip gotchas entry below). Phase 3 = MIK
  app layer (still future).
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
  below. Comment sweep `ee232dc` (plan markers stripped, comments distilled)
  HW-VALIDATED 2026-08-11; pump RTT strings renamed: "Phase 3.2:" prefixes
  gone, "Fix16:" gone, "Phase1→Phase2 latency" → "Deferred stop: ISR→Poll
  latency" — update any log greps.
- **Fault-injection playbook (2026-08-13): `docs/PUMP_FAULT_INJECTION_PLAYBOOK.md`**
  — full emitter map (app A1-A10 + stack S1-S4 → EMCY/TPDO/SDO) and bench
  injection commands (probe-rs no-halt pokes + CAN-side frames) with
  DWARF-derived addresses @ `bb64c23` (re-derive after ANY rebuild — gdb
  one-liner in the doc). Use it to validate Phase 2 / dose-strip firmware.
  Notable analysis findings: heartbeat-lost stop reports PumpState=STOPPED
  not FAULT (only stack EMCY 0x8130 announces it); any stack EMCY latches
  gMCOConfig ER bit0 until a fault/NMT reset; A8/A9 (init-fail, TX-overflow
  fatal) are emitters with no practical injection.
- **Dose-strip refactor → "dumb pump" step-counter: COMPLETE + HW-VALIDATED +
  on main (pushed 2026-08-24).** motor_control.c 1559→1301 (dose engine gone);
  MIK owns all ml/tubing math. Commits: 19b0ede (EDS regen), 1f89a6f (firmware),
  4ef6b5e (Q3 HB), 79b4e7e (docs); main==origin/main. Pump at node 1 (master on
  a HIGH node id, so the old node-1 HB collision is moot). Plan/checklist:
  `docs/PUMP_DOSE_TRANSFER_PLAN.md`. FE contract: `docs/MIK_INTEGRATION_BRIEF.md`
  + `docs/PUMP_FE_CONTROL_BRIEF.md` (operations; the latter is UNTRACKED/uncommitted).
  OD now: RPDO1=CW(0x6040)+StepRate(0x2600 i16, sign=dir)+TargetSteps(0x2601 u32,
  0=continuous), 8B; TPDO1=SW(0x6041)+ActualStepRate(0x2602), 4B; TPDO2=
  PumpState(0x2400)+ErrReg(0x1001)+StepsRemaining(0x2603), 6B; fault 0x2500-0x2503
  SDO-only. Deleted 0x2200/0x2300-0x2305/0x6042/0x6043/0x2100. Validated end-to-end
  (continuous, normal/quick stop, disable, NMT, counted exact-N, pause/resume,
  CW-edge re-arm, HB runaway; no faults). Traces on Samsung T5:
  cantrace-dosingstrip1/2/3 + consumerHB-test1. EDS/DCF also propagated to the
  gateway repo by Dakota.
- **Dose-strip DURABLE GOTCHAS (read before touching pump command handling):**
  • **RPDO1 is SYNCHRONOUS (TType 1)** → the stack re-copies the buffered frame
    (incl. TargetSteps) into the process image every SYNC (mcop.c:1431). So
    starting/re-starting ANY run needs a **ControlWord EDGE** (0x0007→0x000F);
    TargetSteps alone can't signal "fresh command." The `run_consumed` latch (set
    on TARGET_REACHED) is cleared by a CW edge ONLY. consume-on-latch
    SetTargetSteps(0) is futile under sync-RPDO (kept only as event-driven
    defense). This is THE re-arm rule — mirrored in the FE brief.
  • **Rate path**: motor_control converts steps/s→RPM and drives the validated
    Stepper_SetSpeedRPM ramp; native Stepper_SetStepRate/ARR-direct is DEFERRED.
    Consequence: RPM quantization remains and ActualStepRate is an RPM round-trip
    (exact only at rates mapping to whole RPM). Stripping RPM is a separate future
    change needing its own ramp re-validation.
  • **Pause/resume**: HALT (0x010F) captures Stepper_GetStepsRemaining() into
    motor_ctrl.paused_steps_remaining BEFORE decel; resume (0x000F) restarts that
    held count (does NOT re-read OD TargetSteps). Exact at low (instant-stop,
    ARR>700) rates; slight OVER-deliver at high rates (decel steps emitted but not
    re-counted).
  • **Fault-feedback Phase 2 DONE** (landed with dose strip): main.c writes 0x2500
    TMCInitStatus + 0x2501 FailedStep AFTER MCOUSER_ResetCommunication (its
    UpdateSystemFromOD can touch the PI); MotorControl_EnterFault writes 0x2502
    LastDrvStatus + 0x2503 FaultSource; fault reset clears them. Fault-plan Phase 3
    (MIK app layer) still future.
- **Q3 heartbeat-lost runaway backstop: DONE + HW-VALIDATED + pushed
  (commit 4ef6b5e).** MCOP_InitHBConsumer(1, 127, 2500) armed in
  MCOUSER_ResetCommunication (user_STM32.c) — watches master node 127's 1 s HB
  (COB-ID 0x77F), 2500 ms timeout; armed in firmware (NOT the 0x1016 OD default →
  no regen), re-arms on every comm reset. On loss: MCOUSER_HeartbeatLost →
  EmergencyStop (estop + de-energize) → PRE-OP. **ABORT-NOW** policy (motion-state
  based → also aborts a dose in progress = partial volume; revisit "let bounded
  dose finish" if wanted). No false trip at boot: consumer sits in HBCONS_INIT
  until it sees the first HB — lost only fires from ACTIVE (mcop.c:1664).
  MIK REQUIREMENT: master must keep its 1 s HB alive the whole time it commands
  motion (>2.5 s gap mid-run → estop + PRE-OP, needs re-establish + NMT-start).
- **Dose-strip / pump REMAINING (all optional / non-blocking):** commit the FE
  brief `docs/PUMP_FE_CONTROL_BRIEF.md` (currently untracked); optional high-rate
  decel-caveat test (StepRate 2400 sps → observe pause/resume over-deliver);
  native steps/s rate path (drops the RPM hop; needs ramp re-validation); MIK
  app-layer integration (fault-plan Phase 3).
- Pump logging = RTT (`probe-rs attach --chip STM32G431KBTx build/pump-n01-250k.elf`);
  console noise: `[STEPPER]` step-rate block prints 1/s — set
  DBG_STEPPER_ENABLE 0 for quieter sessions (timer proven correct). Known
  cosmetic: first step-rate sample after a start straddles the accel ramp and
  prints a bogus "TIMER/ISR fault" analysis line; converges 1.00x next sample.
- valve DONE (2026-07-29); flash cmd: `st-flash --connect-under-reset --reset
  write build/valve-n09-250k.bin 0x08000000`. A CAN harness exercising all 3 is
  still worth building (3-device retest rule).
- **Tagging DEFERRED by decision (2026-07-29): do not tag any device yet.** Revisit
  the tag/versioning scheme LATER, after more devices are validated. Do not raise
  tagging again until Dakota brings it up. (Context: tag = qualified release, needed
  eventually for field traceability + the manufacturing pipeline.)
- Flashing is via st-flash (Dakota's habit), not STM32_Programmer_CLI. Cmd:
  `st-flash --connect-under-reset --reset write build/<dev>-nNN-250k.bin 0x08000000`
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
- Revision Number (sub3) = **OD layout generation**, not firmware version:
  upper word bumps on a breaking OD layout change, lower word on compatible
  additions. pump 0x00020000 (steps-only OD), phtemp 0x00020000 (dumb-module
  OD, 2026-09-22), valve still 0x00010001. **The gateway's master image checks
  it at slave boot** → an OD revision bump and the gateway EDS update are one
  coupled cutover. FW version is a separate string (banner only; 0x100A still
  "1.0" on all 3 — wiring it to FIRMWARE_VERSION is an open item).
- Serial Number (sub4) is a per-TYPE constant (valve 0x123444 etc.), NOT per-unit →
  can't uniquely ID a unit on the bus. Manufacturing needs per-unit provisioning at
  test time (firmware change, not build-system).
- `shared/mco/svninfo.h` is a DEAD SVN-era file (REV 5559 hardcoded, referenced by
  nothing). Vestige of the version-stamping now needed. Wire to git or delete.
