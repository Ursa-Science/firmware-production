# Valve — CANopen Architect regen: dumb-module OD (rev 0x00020000)

**Purpose.** Mechanical edit list for CANopen Architect (Windows,
`E:\ursaScience\Modules\Modules-base.cax`, device **Valve Module**, node 9,
product code 5) for Phase 3 of `VALVE_REFACTOR_PLAN.md`: delete the seven
objects the firmware no longer uses, bump the OD revision, and normalise the
output file name. After these edits: regenerate, copy the six output files
into the repo, build, propagate to the gateway in ONE window (the master
checks 1018:03 at boot), re-validate.

**Prerequisite — firmware first (Phases 1–2, started 2026-09-30).** The
valve firmware must already build with no reference to `P210000_*`,
`P210100_*`, `P230000_*` (accessors deleted from `procimg_api.h`), so the tree
builds against the current OD (objects present but unused) *and* against the
regenerated one. `grep -rn "P2[13][01]0" devices/valve/Core` must be empty
before you open Architect. Everything in this file is one Windows cycle.

**While you are in Architect:** the pHTemp device still carries the
2026-09-22 hand-edit debt (`.cax` RevisionNumber behind the repo's
0x00020000 — see Context.md). Fix that in the same session so the next phtemp
regen does not silently revert it. Not part of this valve regen otherwise.

Sections A–D are required; E is conditional on the D9 trace; F is what must
NOT change.

---

## A. Device Info → RevisionNumber = **0x00020000**

Today 0x00010001 (`[DeviceInfo]` and `[1018sub3]`, `pimg.h OD_REVISION`).
Upper-word bump = breaking layout change (objects deleted). Set it in
Architect — **no hand-edit this time** (the .cax is the source; the phtemp
hand-edit is the debt we are avoiding).

Also in Device Info: leave `OrderCode=URSA-VLV-001`, `ProductNumber=5`,
`VendorNumber=0x123`, `SerialNumber 0x123444` (per-type constant, known
limitation), `ProductName=Valve Module`.

## B. Rename the output (D8)

Output file base name **`Valve Module-n09-250kbs`** → **`ValveModule-n09-250kbs`**
(no spaces; the gateway's copies in `bridge/devices/` are already
hyphenated, and the space breaks every shell path). Keep `-n09-` and the
node-ID/bitrate selectors at 9 / 250: renaming does *not* change the DCF
NodeID, and both selectors are inert for the firmware (node ID and bitrate
are CMake parameters — Context.md "NODE ID = BUILD PARAMETER").

Expected generated header name: `ValveModule_n09_250kbs_public.h` (was
`Valve_Module_n09_250kbs_public.h`). `stackinit.h`/`pimg.h`/
`entriesandreplies.h` names do not change.

## C. Delete these 7 objects (28 → 21)

| Index | ParameterName | Type / access | PI today | Why |
|---|---|---|---|---|
| 0x1006 | Communication Cycle Period | U32 rw | 0x0C, 4 B | no SYNC use: RPDO1 TType 255, TPDOs TType 254; phtemp regen #2 precedent |
| 0x1007 | Synchronous Window Length | U32 rw | 0x10, 4 B | same |
| 0x1019 | Synchronous Counter Overflow Value | U8 const | SDOREPLY constant (not in PI) | same |
| 0x1020 | Verify Configuration (sub0–2) | U32 rw ×2 | 0x36 + 0x3A, 8 B | `USE_STORE_PARAMETERS 0`; contradicts "firmware has no persistence" |
| 0x2100 | FailSafePosition | U8 rw | 0x3F, 1 B | D1: fail-safe is hard-wired CLOSED; the configurable field mis-specified itself in the gateway enum |
| 0x2101 | ManaualOverride *(sic)* | Boolean ro | 0x40, 1 B | never read by firmware; ro constant 0 |
| 0x2300 | MotionTimeout | U16 rw | 0x41, 2 B | the fault branch it bounded was unreachable (50 ms settle); deleted from firmware |

None of the seven is PDO-mapped, so there is no mapping to unpick first.
Twenty PI bytes are freed.

**Do NOT delete 0x1016** even though its sub1 default is empty: the generated
`pimg.h` derives `NR_OF_HB_CONSUMER 1` from the entry count — remove the
object and the master-heartbeat consumer compiles out of the stack. (D9 in
the plan.)

## D. Verify the PDO / comm parameters are unchanged

| Object | Field | Value | Meaning |
|---|---|---|---|
| 0x1400:01 / :02 | RPDO1 COB-ID / TType | `$NODEID+0x200` / 255 | ControlWord, event-driven |
| 0x1600 | RPDO1 mapping | `0x60400010` | CW, 2 B |
| 0x1800:02 / :03 / :05 | TPDO1 TType / inhibit / event timer | 254 / 10 / 100 | SW+ValveState, 3 B, COS + 100 ms timer |
| 0x1A00 | TPDO1 mapping | `0x60410010`, `0x60420008` | |
| 0x1801:02 / :03 / :05 | TPDO2 TType / inhibit / event timer | 254 / 10 / 1000 | ErrorRegister, 1 B, COS + 1 s timer |
| 0x1A01 | TPDO2 mapping | `0x10010008` | leading 0x1001 — keep the `MCO_InitTPDOFull` re-assert in `user_STM32.c` (D5 parks the consolidation) |
| 0x1017 | Producer heartbeat | 1000 | |
| 0x1014 / 0x1015 | EMCY COB-ID / inhibit | `$NODEID+0x80` / 0 | this refactor *uses* EMCY |

All numeric comm fields stay `rw`; the MIK can raise 0x1800:03 to slow TPDO1.

## E. Conditional (D9): 0x1016:01 default = `0x007F09C4`

Only if the Phase 0 gateway trace shows the master SDO-writing 0x1016:01
at node-9 boot. Then set `[1016sub1] DefaultValue=0x007F09C4` (node 127,
2500 ms) so the OD, the gateway's DCF copy and the firmware arm agree and a
dcfgen-generated write carries the same value instead of zero. If the trace
shows no write to 0x1016 (expected: the gateway DCF copy has an empty
default and `heartbeat_consumer` is commented out in `overrides.yml`), leave
it empty — the firmware arms the consumer itself after
`MCO_DefaultResetCommunication`. Record the outcome in the plan (D9).

## F. Do NOT touch (verify unchanged after regen)

- 0x1000 (0x00000408), 0x1001, 0x1003 (4 entries), 0x1008 / 0x1009 / 0x100A
  strings ("Valve Module", "1.0", "1.0"), 0x1018 sub1/2/4.
- 0x6040 ControlWord (rww), 0x6041 StatusWord (ro), 0x6042 ValveState (ro,
  default 1 = Closed).
- 0x2000 LEDControl (rw) — kept pending the MIK's answer on whether it writes it.
- `[Comments]` lines 1–3 (CiA 408 blurb) — harmless either way.

---

## Regenerate + propagate

1. **Regenerate** MicroCANopen Plus output for the valve device only.
2. **Copy** the six files over `devices/valve/MCO_CiA401__User/EDS/`:
   `pimg.h`, `entriesandreplies.h`, `stackinit.h`,
   `ValveModule_n09_250kbs_public.h`, `ValveModule-n09-250kbs.eds`,
   `ValveModule-n09-250kbs.dcf`. **Delete** the three old space-named files
   (`Valve Module-n09-250kbs.eds/.dcf`, `Valve_Module_n09_250kbs_public.h`)
   in the same commit — the CMake glob does not pick up EDS files, but
   `git status` must show only `devices/valve/.../EDS/`.
3. **Expected diff** (sanity check before building):
   - `.eds`: `[OptionalObjects] SupportedObjects 21 → 16` (0x1006, 0x1007,
     0x1019, 0x1020 gone); `[ManufacturerObjects] SupportedObjects 4 → 1`
     (only 0x2000); `RevisionNumber=0x00020000` in `[DeviceInfo]` and
     `[1018sub3]`; `FileName=…\ValveModule-n09-250kbs.eds`.
   - `pimg.h`: `OD_REVISION 0x00020000L`; no `P100600_*`, `P100700_*`,
     `P102001_*`, `P102002_*`, `P210000_*`, `P210100_*`, `P230000_*`;
     `NR_OF_HB_CONSUMER 1` still present; `PIMGEND` 0x42 → **≈0x2D** (20
     bytes freed; the exact value depends on Architect's alignment padding —
     do not hand-compute, read it off the file); `P200000_LEDControl` moves
     from 0x3E to the new tail; `P604000/P604100/P604200/P100100` stay at
     0x00/0x04/0x06/0x08 (nothing before them was deleted). The
     `PIMGDEFAULTS` initialiser loses the `[1006]…[2300]` rows.
   - `entriesandreplies.h`: the `SDOREPLY(0x1006/0x1019/0x1020,0x00 …)`
     constants and `ODENTRY(0x1007 / 0x1020,01 / 0x1020,02 / 0x2100 / 0x2101 /
     0x2300 …)` lines gone, and their rows in the index / type / location
     tables.
   - `stackinit.h`: unchanged apart from the header date — the three init
     lines must still read `MCO_InitRPDO(1, NODEID+0x200, 2, P604000_ControlWord)`,
     `MCO_InitTPDOFull(1, NODEID+0x180, 100, 1, 254, 3, P604100_StatusWord)`,
     `MCO_InitTPDOFull(2, NODEID+0x280, 1000, 1, 254, 1, P100100_Error_Register)`.
   - Header comment "Generated from project …Modules-base.cax" (was
     `Modules-1000kbs.cax`) — expected, the project was renamed.
   - If E was done: `[1016sub1] DefaultValue=0x007F09C4`.
4. **Build:** `cmake --build --preset valve-n09` → clean link; `.text` drops
   a little (PI shrinks, seven OD table rows gone). `arm-none-eabi-size
   build/*.elf` must show pump/phtemp `.text` unchanged.
5. **Gateway cutover — ONE window** (`~/code/CANOpenGateway`):
   - copy the new `.eds` + `.dcf` into `bridge/devices/` over the existing
     `ValveModule-n09-250kbs.{eds,dcf}`;
   - `bridge/devices/overrides.yml`, `ValveModule-n09-250kbs` block:
     **delete the whole `sdo:` list** (`0x2300 = 1` — a boot-time SDO write to
     a deleted object aborts and fails node-9 boot), delete the `"2300:0"`,
     `"2100:0"`, `"2101:0"` object entries, rewrite the two device-level
     notes (0x6100 is now decoded: TX-buffer-overflow warning class
     0x48xx; the 1001 TPDO/SDO disagreement is fixed by both-homes writes);
   - update the tests/docs that pin the old objects:
     `bridge/test/test_gen_network.py:105-106`, `bridge/gen-network.py:37`,
     `api/test/server-routes.test.ts:321-359`, `docs/device-management.md`,
     `docs/plans/agent-hardware-context.md`;
   - rebuild the bridge image; flash the valve; boot check — no 1018:03
     mismatch, no SDO abort in the boot sequence.
6. **FE guide:** `docs/VALVE_FE_CUTOVER_GUIDE.md` (to write in Phase 6):
   HB requirement, CW-edge re-arm after any restart, FailSafe write path
   gone, any NMT exit closes the valve, recovery = NMT start + fresh Open.

## Re-validation (bench, CANopen Magic, node 9)

- [ ] SDO read 0x1006 / 0x1007 / 0x1019 / 0x1020:01 / 0x2100 / 0x2101 /
      0x2300 / 0x2222:23 → abort `80 … 00 00 00 08` (0x08000000 — the stack's
      unknown-object code, not 0x06020000).
- [ ] SDO 0x1018:03 = `00 00 02 00`; 0x1018:02 = 5; 0x1016:00 = 1;
      0x1016:01 reads back `C4 09 7F 00` once the firmware has armed it.
- [ ] TPDO1 `189` 3 B every ~100 ms in OP; TPDO2 `289` **1 B** (DLC=1, never
      DLC=0) every ~1 s; 0x2000 write/read round-trip.
- [ ] Open/close (`209#09 00` / `209#0A 00`) unchanged: SW 0x0602 / 0x0601,
      ValveState 2 / 1 within ~50 ms.
- [ ] Then the full B1–B12 checklist in `VALVE_REFACTOR_PLAN.md` §6 on the
      final image.
