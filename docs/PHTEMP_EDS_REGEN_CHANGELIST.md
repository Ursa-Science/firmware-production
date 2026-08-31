# pH/Temp EDS Regen Change-List — CANopen Architect (Step 2)

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
| DefaultValue | 10 | **5** (mV; pick to taste) |

> **Firmware side already done.** `procimg_api.h` now reads via
> `P240000_MillivoltDeltaThreshold`, plus a transitional shim near the top that
> aliases the new name to the old generated symbol so the tree builds BOTH
> before and after the regen (verified: `cmake --build --preset phtemp` links
> clean). After the regenerated `pimg.h` defines `P240000_MillivoltDeltaThreshold`
> directly, delete the shim block (marked with a "remove after regen" comment) —
> optional cleanup, harmless if left.

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

## Optional decision — Revision Number (0x1018 sub3)

Currently 0x00010001 (static across all three modules; tagging deferred per
Context.md). This is a **breaking** OD-layout change, so bumping to **0x00010002**
would let the gateway/MIK distinguish old vs new layout. Policy call — leave as
0x00010001 unless Dakota wants to start using the revision field. *Default: leave.*

---

## Regenerate + propagate

1. **Regenerate** MicroCANopen Plus output in CANopen Architect.
2. **Copy generated files** over the existing set in
   `devices/phtemp/MCO_CiA401__User/EDS/`:
   `pimg.h`, `entriesandreplies.h`, `stackinit.h`,
   `PH_TempModule_n04_250kbs_public.h`, `PH-TempModule-n04-250kbs.eds`,
   `PH-TempModule-n04-250kbs.dcf`. `git status` must show **only** phtemp EDS
   files changed.
3. **Step 3 firmware follow-ups (macOS):**
   - Delete the transitional shim in `procimg_api.h` (the
     `#ifndef P240000_MillivoltDeltaThreshold` block) — the regenerated `pimg.h`
     now defines it directly. Optional/harmless if left.
   - `cmake --build --preset phtemp` → expect clean link. `arm-none-eabi-size
     build/phtemp.elf` (text should drop a little vs 42172 B — objects gone).
   - Sanity grep: no `P600000` / `P2200` / `P2210` / `P2220` symbols referenced
     in `Core/` (there should be none — Step 1 already removed the writers).
4. **Gateway:** copy `.eds` + `.dcf` into `~/code/CANOpenGateway/bridge/devices/`
   → rebuild the bridge Docker image (baked-in EDS catalog).

## Re-validation (bench, after flash)

- [ ] TPDO1 (0x184) is now **4 bytes** `[mV(2) | Temp(2)]` — no more frozen
      `BC 02` pHValue prefix.
- [ ] SDO read of a deleted object (e.g. 0x6000, 0x2200, 0x2210, 0x2220) →
      **abort 0x06020000** ("object does not exist").
- [ ] SDO read of 0x6003 (mV) + 0x6010 (Temp) → live values.
- [ ] TPDO2 (0x284) unchanged 4 B; SensorStatus/qualities/ErrReg correct.
- [ ] NMT PRE-OP→OP clean, heartbeat steady, ErrReg 0x1001 = 00.
- [ ] Gateway shadow + MIK display read mV/temp with correct units.
- [ ] valve/pump EDS untouched (`git status`).
```
Before:  TPDO1 = [ pHValue(2) | Temp(2) | mV(2) ]   (6 B, pHValue frozen 700)
After:   TPDO1 = [ mV(2)      | Temp(2)          ]   (4 B, both live)
```
