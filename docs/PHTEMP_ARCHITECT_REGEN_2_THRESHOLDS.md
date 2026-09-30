# pH/Temp — CANopen Architect regen #2: delete the delta-threshold objects

**Purpose.** Mechanical edit list for CANopen Architect (Windows,
`E:\ursaScience\Modules\Modules-base.cax`, device **PH-TempModule**) to
remove the three app-level TPDO delta-threshold objects and to bring the
project's RevisionNumber in line with the hand-edit made on 2026-09-22.
After these edits: regenerate, copy the six output files into the repo,
build, propagate to the gateway, re-validate.

**Why (decided 2026-09-30, trace-proven).** MicroCANopen transmits any
TPDO with a non-zero inhibit time whenever a mapped process-image byte
changes. The electrode average changes every 400 ms, so TPDO1 was sent on
every sample regardless of 0x2400; the app-level trigger only pre-empted the
stack by milliseconds and, until fixed, sent a stale duplicate. The objects
never reduced traffic. The standard TPDO communication parameters
(0x1800:03 inhibit, 0x1800:05 event timer, and 0x1801:03/05 for TPDO2) are
the real cadence knobs and are already `rw`.

**Firmware side is already done** (2026-09-30): no code references
`P240000` / `P240100` / `P240200` any more, the trigger function and its
process-image getters are deleted, and the tree builds against the current
OD (objects present but unused) and against the regenerated one.

Do these three things and nothing else in section A–C. Section D is an
optional batch; section E is what must NOT change.

---

## A. Device Info → RevisionNumber = **0x00020000**

The 2026-09-22 regen produced 0x00010002 and it was hand-edited to
0x00020000 in the .eds/.dcf/pimg.h (marked `HAND-EDITED 2026-09-22`). The
.cax still says 0x00010002 (or 0x00010001). **Set it to 0x00020000 first**,
or this regen silently reverts the identity and the gateway will reject the
module at boot.

Decision on bumping again for this deletion: **stay at 0x00020000.**
Deleting rw objects is a breaking layout change by our rule, but layout 2
has only ever existed on the bench and the reference machine, and the
gateway has to re-copy the EDS for this regen anyway. Treat this as
"layout 2 finalised before first field deployment". If you would rather be
strict, use 0x00030000 — then the gateway cutover (rev check at boot)
repeats in full.

## B. Delete these 3 objects

| Index | ParameterName | Type | Access | Why |
|---|---|---|---|---|
| 0x2400 | MillivoltDeltaThreshold | U16 | rw | app trigger removed; stack COS sends anyway |
| 0x2401 | TempDeltaTheshold *(sic)* | I16 | rw | same |
| 0x2402 | StatusDeltaThreshold | U8 | rw | was a bitfield "magnitude" — never meaningful |

None of the three is PDO-mapped, so there is no mapping to unpick first.

## C. Verify TPDO comm params are unchanged (they are the new knobs)

| Object | Field | Value | Meaning |
|---|---|---|---|
| 0x1800:02 | Transmission type | 0xFF | event-driven |
| 0x1800:03 | Inhibit time | 500 | × 100 µs = 50 ms → TPDO1 at most 20 Hz; in practice one per 400 ms sample |
| 0x1800:05 | Event timer | 1000 | ms — a TPDO1 at least every second |
| 0x1801:02 | Transmission type | 0xFF | |
| 0x1801:03 | Inhibit time | 5000 | 500 ms |
| 0x1801:05 | Event timer | 5000 | 5 s |

All four numeric fields are `rw`; the MIK (or dcfgen at boot) can raise
0x1800:03 to slow TPDO1 down. Leave the defaults as they are.

## D. Optional in the same regen — dead communication objects

These were identified 2026-09-22 (plan doc, "EDS objects that can still
go"). Deleting them needs **no firmware change** (nothing reads them; the
stack's handlers are index-based and tolerate their absence) and no gateway
change. Do them now if you want one Windows cycle instead of two; skip if
you want this regen minimal.

| Index | Name | Why it is dead |
|---|---|---|
| 0x1010 | Store Parameters | `USE_STORE_PARAMETERS 0` |
| 0x1011 | Restore Default Parameters | same |
| 0x1020 | Verify Configuration | only meaningful with store parameters |
| 0x1002 | Manufacturer Status Register | never written; reads 0 |
| 0x1012 | Time Stamp COB-ID | no time consumer on the module |
| 0x1013 | High Resolution Time Stamp | same |
| 0x1006 | Communication Cycle Period | no synchronous PDOs, no SYNC producer |
| 0x1007 | Synchronous Window Length | same |
| 0x1019 | Synchronous Counter Overflow | same |
| 0x1028 | Emergency Consumer | slave consumes no EMCY |

**Do NOT delete** 0x1016 Consumer Heartbeat or 0x1017 Producer Heartbeat —
the gateway's dcfgen writes both at every boot. Also keep 0x1003, 0x1005 (if
present), 0x1014, 0x1015, 0x1F80, 0x1008/0x1009/0x100A, 0x1018, 0x1200.

## E. Do NOT touch (verify unchanged after regen)

- RPDO1 0x1400/0x1600 — ControlWord only, 2 B.
- TPDO1 0x1800/0x1A00 — `[0x6003 pHMillivolts | 0x6010 Temperature]`, 4 B.
- TPDO2 0x1801/0x1A01 — `[0x2300 | 0x6001 | 0x6011 | 0x1001]`, 4 B.
- 0x6001, 0x6002, 0x6003, 0x6010, 0x6011, 0x6012, 0x6040, 0x6041, 0x2300,
  0x2000 LEDControl, 0x1001, and all identity/comm objects not listed in D.
- Node ID and bitrate selectors: leave at 4 / 250 (both inert for the
  firmware; node ID and bitrate are CMake parameters).

---

## Regenerate + propagate

1. **Regenerate** MicroCANopen Plus output.
2. **Copy** the six files over `devices/phtemp/MCO_CiA401__User/EDS/`:
   `pimg.h`, `entriesandreplies.h`, `stackinit.h`,
   `PH_TempModule_n04_250kbs_public.h`, `PH-TempModule-n04-250kbs.eds`,
   `PH-TempModule-n04-250kbs.dcf`. `git status` must show only phtemp EDS.
3. **Expected diff** (sanity check before building):
   - `.eds` `[ManufacturerObjects]` SupportedObjects 5 → **2** (0x2000,
     0x2300); `[2400]`, `[2401]`, `[2402]` sections gone; `RevisionNumber=
     0x00020000` in `[DeviceInfo]` and `[1018sub3]`.
   - `pimg.h`: `OD_REVISION 0x00020000L`; no `P240000_*`, `P240100_*`,
     `P240200_*`; `PIMGEND` 0x68 → **0x63** (5 bytes of PI freed); objects
     after 0x2000 shift down by 5; the `HAND-EDITED` comment is gone (fine —
     the value is now genuine tool output).
   - `entriesandreplies.h`: three `ODENTRY(0x240x …)` lines and their
     entries in the index / type / location tables gone.
   - `stackinit.h`: unchanged apart from the header date (TPDO init args
     `1000, 50, 255, 4` and `5000, 500, 255, 4` must still be there).
   - `[Comments] Line4` hand-edit note disappears from .eds/.dcf (regen
     rewrites them). That is expected.
   - If section D was done: those sections/entries gone too; `PIMGEND`
     smaller still.
4. **Build:** `cmake --build --preset phtemp-n04` (and `-n31`, `-n32`) →
   clean link; text should drop a little.
5. **Gateway** (`~/code/CANOpenGateway`): copy the new `.eds` + `.dcf` into
   `bridge/devices/` (also the `-n31`/`-n32` pairs if they exist by then —
   same content, DCF NodeID edited); in `bridge/devices/overrides.yml`
   **remove the `"2400:0"` entry** under the phtemp block (setdefault would
   otherwise create a phantom object); rebuild the bridge image. No revision
   change → no boot-check cutover, but the image must carry the new EDS so
   `devices.json` stops advertising 0x2400-0x2402.
6. **FE guide:** `docs/PHTEMP_FE_CUTOVER_GUIDE.md` §4.5 now says to use
   0x1800:03/05 instead of 0x2400/0x2401 — already updated 2026-09-30.

## Re-validation (bench)

- [ ] SDO read 0x2400 / 0x2401 / 0x2402 → abort (`80 … 00 00 00 08`).
- [ ] TPDO1 cadence unchanged: one `0x19F` per new sample (~400 ms), never
      two within 50 ms with different data; `0x19F` at least every 1 s.
- [ ] SDO write 0x1800:03 = 10000 (1 s) in PRE-OP → TPDO1 ≤ 1 Hz in OP;
      restore 500.
- [ ] TPDO2 still changes within 500 ms of a SensorStatus change.
- [ ] Identity 0x1018:03 = `00 00 02 00`; gateway boots node clean.
- [ ] `devices.json` node 4/31/32: no `2400`, `2401`, `2402`.
