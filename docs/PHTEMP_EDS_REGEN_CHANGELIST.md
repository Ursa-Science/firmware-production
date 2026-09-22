# pH/Temp EDS Regen Change-List — CANopen Architect (Step 2)

**STATUS 2026-09-22: REGEN DONE + REVIEWED + IN-TREE (uncommitted).** Sections
A/B/C were applied in Architect (project is now `Modules-base.cax`, not
`Modules-1000kbs.cax`) and the six generated files are in
`devices/phtemp/MCO_CiA401__User/EDS/`. `cmake --build --preset phtemp-n04`
links clean, text 42172 → 42100 B. Two deviations from this list were found at
review and resolved by HAND-EDIT (see "Post-regen review" at the bottom):
RevisionNumber is now **0x00020000** (was regenerated as 0x00010002), and the
0x2400 default stays **10** (the "→5" below is withdrawn). **Firmware review
fixes landed the same day** (7 items, plan doc "Firmware review findings"):
**0x6003 and 0x2400 are now mV × 10** — a unit change with NO EDS edit (like
Temperature ×10), but the gateway override needs `scale: 0.1`. Build text
41384 B. Gateway copies NOT yet updated. Next: commit → gateway cutover →
flash → re-validate.

**Purpose.** Mechanical, one-pass edit list for the phtemp Object Dictionary in
CANopen Architect (Windows, `E:\ursaScience\Modules\Modules-1000kbs.cax`), to
match the Step-1 firmware strip. After these edits: regenerate, copy the output
into the repo, propagate to the gateway, rebuild, re-validate.

- **Device:** PH-TempModule, node **4**, product code 4, device type 0x00000404.
- **Net effect:** remove all pH/calibration OD objects; TPDO1 shrinks 6 B → 4 B.
- **Source of truth for the "before" values:** the current
  `devices/phtemp/MCO_CiA401__User/EDS/PH-TempModule-n04-250kbs.eds`.

Do these three things and nothing else: (A) delete 11 objects, (B) remap TPDO1,
(C) rename 0x2400. Everything not listed stays exactly as-is.

---

## A. Delete these 11 objects

| Index | ParameterName | Type | Access | Why |
|---|---|---|---|---|
| 0x6000 | pHValue | U16 | ro | MIK computes pH from mV (also unmap from TPDO1 — see B) |
| 0x2200 | pHCalibrationCommand | U8 | wo | on-module cal removed |
| 0x2201 | pHCalibrationStatus | U8 | ro | on-module cal removed |
| 0x2202 | CalibrationBuffer4 | I16 | rw | dead (never read by firmware) |
| 0x2203 | CalibrationBuffer7 | I16 | rw | dead |
| 0x2204 | CalibrationBuffer10 | I16 | rw | dead |
| 0x2205 | pHCalibrationMode | U8 | ro | on-module cal removed |
| 0x2210 | TempOffset | I16 | rw | MIK owns temp trim |
| 0x2220 | pHElectrodeStatus | U8 | ro | cal-derived; MIK derives from mV/quality |
| 0x2221 | ElectrodeAge | U16 | ro | dead |
| 0x2222 | LastCalibrationDate | U32 | ro | dead |

> Delete 0x6000 **after** doing step B (unmap it from TPDO1 first, or the tool
> may warn about a mapped object being removed).

---

## B. Remap TPDO1 (0x1A00) — drop pHValue, keep mV + Temperature

**Before (3 entries, 6 bytes):**

| sub | value | maps |
|---|---|---|
| sub0 (NumberOfEntries) | 3 | |
| sub1 | 0x60000010 | pHValue (16-bit) |
| sub2 | 0x60100010 | Temperature (16-bit) |
| sub3 | 0x60030010 | pHMillivolts (16-bit) |

**After (2 entries, 4 bytes):**

| sub | value | maps |
|---|---|---|
| sub0 (NumberOfEntries) | **2** | |
| sub1 | **0x60030010** | **pHMillivolts (16-bit)** |
| sub2 | 0x60100010 | Temperature (16-bit) |
| ~~sub3~~ | *(delete)* | |

Net: `[pHMillivolts(2) | Temperature(2)]`. TPDO1 comm params (0x1800: COB-ID
$NODEID+0x180, TxType 0xFF, inhibit 500, event 1000) are **unchanged**.

---

## C. Rename 0x2400 → MillivoltDeltaThreshold (CONFIRMED 2026-08-24)

The object stays the same slot/type; only its name + default change so the
gateway/MIK see honest units.

| field | before | after |
|---|---|---|
| ParameterName | pHDeltaThreshold | **MillivoltDeltaThreshold** |
| DataType | 0x0006 (U16) | unchanged |
| AccessType | rw | unchanged |
| DefaultValue | 10 | ~~5~~ **10 — WITHDRAWN 2026-09-22, left at 10.** Unit is now **mV × 10** (same as 0x6003), so 10 = **1.0 mV** ≈ 0.03–0.05 pH through the pH-2 Click front-end (G≈0.3–0.5, ≈18–30 ADC-mV per pH) — close to the old pH×100 default of 0.10 pH. The MIK owns config and SDO-writes 0x2400 on connect; the firmware default is a fallback. The 1000 ms event timer bounds latency regardless. |

> **Firmware side done.** `procimg_api.h` reads via
> `P240000_MillivoltDeltaThreshold`. The transitional shim that aliased the
> new name to the old generated symbol (so the tree built both before and
> after the regen) was **deleted 2026-09-22** once the regenerated `pimg.h`
> defined the symbol directly (`cmake --build --preset phtemp-n04` links clean).

---

## Do NOT touch (verify unchanged)

- **RPDO1** 0x1400/0x1600 — ControlWord only (0x60400010).
- **TPDO2** 0x1801/0x1A01 — SensorStatus(0x23000008) + pHSignalQuality(0x60010008)
  + TempSignalQuality(0x60110008) + ErrorRegister(0x10010008) = 4 B.
- **Kept objects (names stay — they map to firmware symbols):** 0x6001
  pHSignalQuality, 0x6002 pHSensorStatus, 0x6003 pHMillivolts, 0x6010
  Temperature, 0x6011 TempSignalQuality, 0x6012 TempSensorStatus, 0x2300
  SensorStatus, 0x2401 TempDeltaTheshold, 0x2402 StatusDeltaThreshold, 0x2000
  LEDControl, 0x6040 ControlWord, 0x6041 StatusWord, 0x1001 ErrorRegister, all
  0x1000/0x1008/0x1018/0x1200/0x1F80 comms + identity.
- **0x6040 ControlWord / 0x6041 StatusWord** stay U16 — the retired cal
  ControlWord bits (0,1) and StatusWord bit 2 are firmware-side only; no OD change.

## Revision Number (0x1018 sub3) — DECIDED 2026-09-22: **0x00020000**

Convention (mirrors the pump, which went 0x00010001 → **0x00020000** for its
steps-only OD): **upper 16 bits = OD layout generation, bumped on any
breaking layout change; lower 16 bits = additive/compatible OD tweaks.** The
regen produced 0x00010002 (a lower-word bump); that is wrong under this rule and
was hand-edited to 0x00020000 in the .eds, .dcf and `pimg.h` (OD_REVISION +
PIMGDEFAULTS [1018,03] = `00 00 02 00`). Firmware banner bumped 3.3.1 → **4.0.0**
(breaking contract). The FW version string and the OD revision are independent
numbers; on the pump they coincide at 2.0.0 / 0x00020000 by accident, not rule.

**.cax is now behind the tree.** Set RevisionNumber = 0x00020000 in
`Modules-base.cax` (PH-TempModule) BEFORE the next Architect regen, or the tool
reverts it. The hand-edit is marked in all three files (`HAND-EDITED
2026-09-22`, EDS/DCF `[Comments] Line4`).

**Consequence — the gateway checks this at boot.** `gen-network.py` leaves the
1018:03 revision check active (only the serial check is disabled), so the
master image built from the OLD phtemp EDS rejects a module reporting
0x00020000, and vice-versa. Firmware flash and gateway EDS/image update are one
coupled cutover; see below.

---

## Regenerate + propagate

1. ~~**Regenerate** MicroCANopen Plus output in CANopen Architect.~~ DONE
   2026-09-22 14:30 from `E:\ursaScience\Modules\Modules-base.cax`.
2. ~~**Copy generated files** over the existing set~~ DONE — the six files in
   `devices/phtemp/MCO_CiA401__User/EDS/`; `git status` shows only phtemp.
   Note: `PH_TempModule_n04_250kbs_public.h` is included by nothing in the
   tree; it is carried as tool output only.
3. **Step 3 firmware follow-ups (macOS):** DONE 2026-09-22.
   - ~~Delete the transitional shim in `procimg_api.h`~~ deleted.
   - `cmake --build --preset phtemp-n04` links clean;
     `build/phtemp-n04-250k.elf` text 42100 B (was 42172). The revision bytes
     `04 00 00 00 | 00 00 02 00 | 57 34 12 00` are present in the .bin.
   - Sanity grep: no `P600000` / `P2200` / `P2210` / `P2220` symbols in `Core/`.
   - **Commit as its own commit** ("phtemp: regenerate OD — delete cal objects,
     TPDO1 6→4 B, 0x2400→MillivoltDeltaThreshold, rev 0x00020000"), not folded
     into the build-parameter branch's history.
4. **Gateway cutover — ONE window, in this order** (revision check makes the
   two halves inseparable):
   1. Copy the new `.eds` + `.dcf` into `~/code/CANOpenGateway/bridge/devices/`
      (overwrite the 05-24 copies; same filenames, so the `-n04` node-ID hint
      and the DCF NodeID=4 are unchanged).
   2. In `bridge/devices/overrides.yml` under `PH-TempModule-n04-250kbs`:
      **delete the `"6000:0"` entry** (`merge_object_overrides()` uses
      `setdefault`, so a stale key silently creates a phantom pHValue object
      in devices.json instead of failing), and change `"6003:0"` to
      `{unit: mV, scale: 0.1}` plus add `"2400:0": {unit: mV, scale: 0.1}` —
      both objects are **mV × 10** as of firmware 4.0.0.
   3. Sweep gateway docs/tests that still describe TPDO1 as 6 B with pHValue:
      `CLAUDE.md` (~L110, ~L312), `README.md` (~L587),
      `docs/device-management.md` (~L299, ~L507-512), `docs/plans/*` (historical,
      mark as superseded), and the briefing test in
      `api/test/server-routes.test.ts` (~L367).
   4. Rebuild the bridge Docker image (EDS catalog is baked in).
   5. Flash `build/phtemp-n04-250k.bin`
      (`st-flash --connect-under-reset --reset write … 0x08000000`, NRST wired).
   6. Restart the bridge → node 4 must boot with no identity mismatch.

## Re-validation (bench, after flash)

- [ ] **Gateway boot of node 4 is clean** — no 1018:03 revision mismatch; SDO
      read 0x1018:03 = `00 00 02 00`.
- [ ] TPDO1 (0x184) is now **4 bytes** `[mV(2) | Temp(2)]` — no more frozen
      `BC 02` pHValue prefix.
- [ ] SDO read of a deleted object (e.g. 0x6000, 0x2200, 0x2210, 0x2220) →
      **abort 0x06020000** ("object does not exist").
- [ ] SDO read of 0x6003 (mV × 10, ≈10240 at pH 7) + 0x6010 (Temp) → live
      values; gateway shows 0x6003 scaled to mV.
- [ ] 0x1001 SDO read == TPDO2 byte 3 (both homes) with a probe unplugged;
      EMCY 0x5000 on unplug, 0xFF00 on FAULT, 0x0000 on CW 0x80 reset.
- [ ] SDO read 0x2222:23 → abort 0x06020000 (EmSA demo handler removed).
- [ ] TPDO2 (0x284) unchanged 4 B; SensorStatus/qualities/ErrReg correct.
- [ ] NMT PRE-OP→OP clean, heartbeat steady, ErrReg 0x1001 = 00.
- [ ] Gateway shadow + MIK display read mV/temp with correct units; devices.json
      for node 4 has no `6000:0`.
- [ ] Boot banner prints `Firmware: 4.0.0`, `Node ID: 0x04`.
- [ ] valve/pump EDS untouched (`git status`).

## Post-regen review (2026-09-22)

What the regen got right: all 11 deletions, TPDO1 = 2 entries × 16 bit (mV
first), TPDO2/RPDO1 untouched, 0x2400 renamed, `NODEID_DCF 0x04` and
`CAN_BITRATE_DCF 250` unchanged (so the node-ID / bitrate build parameters are
unaffected). Cosmetic generator drift: `[1A00sub0]` is now named "Highest
Subindex" instead of "Number of Entries" (harmless; the public.h macro that
changed is unreferenced).

Deviations and their resolution:

| Item | Regen produced | Resolution |
|---|---|---|
| 0x1018:03 RevisionNumber | 0x00010002 | hand-edited → **0x00020000** (pump convention). Sync the .cax. |
| 0x2400 DefaultValue | 10 | kept 10; "→5" withdrawn (MIK writes it anyway). Unit now mV × 10 → 10 = 1.0 mV. |
| 0x6003 unit | integer mV (05-24 contract) | **mV × 10** in firmware 4.0.0 (0.1 mV resolution). EDS unchanged — CiA 306 has no unit field; gateway override `scale: 0.1`. |
| FIRMWARE_VERSION | n/a | 3.3.1 → **4.0.0** in `Core/Src/main.c`. |
| Firmware fixes (7) | n/a | 0x1001 both homes, EMCY 0x5000/0xFF00/0x0000, warm-up latch, absent-probe reporting + re-init, 0x2222 demo removed (phtemp only), dead code/comments. Text 42100 → 41384 B. |

Build-system interaction (branch `build/node-id-parameter`): the stale
`--preset phtemp` / `build/phtemp.elf` names in this doc are fixed above. The
OD carries `n04`/`250kbs` in file names and macro prefixes; in this repo those
are now cosmetic (node ID and bitrate come from CMake). On the gateway they are
**load-bearing**: `gen-network.py` resolves node ID from the DCF
`[DeviceCommissioning] NodeID` first, then the `-nNN` filename. One EDS+DCF pair
per commissioned node ID is therefore still required in `bridge/devices/`
(e.g. a pump at node 2 needs a `PumpModule-n02-250kbs.dcf` with NodeID=2 —
copy + edit, never a per-node Architect regen).
```
Before:  TPDO1 = [ pHValue(2) | Temp(2) | mV(2) ]   (6 B, pHValue frozen 700)
After:   TPDO1 = [ mV(2)      | Temp(2)          ]   (4 B, both live)
```
