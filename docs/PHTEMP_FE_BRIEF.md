# pH/Temp module fw 4.0.0 — FE brief (gateway + MIK), 2026-09-30

**State:** firmware-production `main` @ 7f15268. Bench-validated on the two
units you will get, **nodes 4 and 5** (CANopen Magic traces + RTT). The
reference machine already runs the first cutover (calibration works). This
brief is what changed *since* that cutover plus everything still owed on
your side. Full detail: `docs/PHTEMP_FE_CUTOVER_GUIDE.md` (contract),
`docs/PHTEMP_BENCH_TEST.md` (frames + expected replies).

## 1. What the module is now

A dumb sensor: it reports raw electrode voltage, raw temperature and its
own health. The MIK owns every conversion, calibration constant and their
persistence. Nothing survives a module reset; whatever you configure, you
re-apply after every boot.

| | value |
|---|---|
| Identity | vendor 0x123, product 4, **revision 0x00020000** (the gateway checks this at boot — old EDS ⇒ node rejected) |
| TPDO1 (0x180+N), 4 B | `0x6003 pHMillivolts` U16 **mV × 10**, `0x6010 Temperature` I16 °C × 10 |
| TPDO2 (0x280+N), 4 B | `0x2300 SensorStatus`, `0x6001 pHQuality`, `0x6011 TempQuality`, `0x1001 ErrReg` |
| RPDO1 (0x200+N), 2 B | `0x6040 ControlWord` — only bit 7 (fault reset, rising edge) does anything |
| SDO-only | 0x6002/0x6012 driver states, 0x6041 StatusWord, 0x2000 LED |
| Heartbeat | 1000 ms producer; **consumes the master's (node 127) heartbeat, 2.5 s timeout** |
| Units handed over | **node 4** (0x184/0x284/0x204/0x604/0x704) and **node 5** (0x185/0x285/0x205/0x605/0x705); one OD for both |

Measured with the probe emulator: pH 7 ≈ 10170, pH 4 ≈ 12340, pH 10 ≈ 7960
on 0x6003 — about **73 mV per pH at the ADC**, so 0.1 mV ≈ 0.0014 pH.

## 2. Deleted objects — nothing may read or write these

| Index | was | since |
|---|---|---|
| 0x6000 pHValue | module's own pH | regen #1 (you have this) |
| 0x2200–0x2205, 0x2210, 0x2220–0x2222 | calibration objects, temp offset, electrode status | regen #1 |
| **0x2400 / 0x2401 / 0x2402** | delta thresholds | **regen #2 — new** |
| **0x1002, 0x1006, 0x1007, 0x1010, 0x1011, 0x1012, 0x1013, 0x1019, 0x1020** | unused comm objects | **regen #2 — new** |

SDO access to any of them aborts with 0x08000000 (MicroCANopen's code for
"no such object"). 0x1016, 0x1017, 0x1028 still exist.

## 3. TPDO cadence — there is no threshold knob any more

The stack transmits TPDO1 whenever the mapped value changes, which is
every 400 ms electrode sample when RUNNING, and TPDO2 whenever a status or
quality byte changes. The thresholds we told you about on 2026-09-22 never
limited traffic and are gone. To slow TPDO1 down, write the standard
parameters (in PRE-OP, or via `overrides.yml` `sdo:` so dcfgen does it at
boot):

| Object | unit | default | effect |
|---|---|---|---|
| 0x1800:03 inhibit time | 100 µs | 500 (50 ms) | e.g. 10000 → at most 1 Hz |
| 0x1800:05 event timer | ms | 1000 | a frame at least this often |
| 0x1801:03 / :05 | same for TPDO2 | 500 ms / 5 s | |

## 4. Errors and EMCYs you will see

0x1001 now agrees between SDO reads and TPDO2 (it did not before).

| ErrReg | meaning |
|---|---|
| bit 0 | generic — set with any other bit, or on module FAULT, or by the stack on its own EMCYs |
| bit 5 | a measurement channel is faulted: pH ADC or temp probe in ERROR **or absent at boot** |

| EMCY | who | when | MSEF |
|---|---|---|---|
| 0x5000 | app | one sensor faulted, once per episode | `01`=pH ADC / `02`=temp probe, driver state |
| 0xFF00 | app | module FAULT | cause `01` nothing ready after warm-up / `02` both channels faulted / `03` stack fatal; then pH state, temp state |
| 0x0000 | app | ControlWord bit-7 reset accepted | — |
| 0x8130 | stack | **your heartbeat stopped** for 2.5 s — node drops to PRE-OP, TPDOs stop | `7F` |
| 0x0000 | stack | your heartbeat is back; 0x1001 bit 0 released | — |

Behaviours that changed:
- **One faulted probe does not fault the module.** It keeps publishing the
  other channel with bit 5 set. Treat 0x5000 + bit 5 as a warning.
- **A probe missing at boot is now reported** (bit 5, StatusWord bit 4/5,
  EMCY 0x5000) and the module still reaches RUNNING on the other channel.
  A ControlWord reset retries the probe init, so a probe plugged in later is
  picked up without a power cycle.
- **An unplugged electrode is NOT detected by the module** (open input reads
  ≈ pH 7). Detect it host-side: `0x6001 pHQuality` collapses to ~10 within
  500 ms; it recovers when the electrode is back.
- **Heartbeat loss is not a fault.** After a 0x8130 the module sits in
  PRE-OP. When the MIK is back: **send NMT start again** (`01 04` / `01 05`).
  No ControlWord reset needed; 0x1001 clears on its own. Tested on both units.

## 5. What you need to change

### Gateway (`CANOpenGateway`)

1. **Re-copy the EDS + DCF** from
   `firmware-production/devices/phtemp/MCO_CiA401__User/EDS/PH-TempModule-n04-250kbs.{eds,dcf}`
   into `bridge/devices/` — the regen-#2 files (12 objects fewer than the
   pair you copied for the first cutover). Revision is unchanged
   (0x00020000), so no boot-check cutover this time, but until you do this
   `devices.json` advertises objects that abort.
2. **Node 5 needs its own pair.** Copy the n04 pair to
   `PH-TempModule-n05-250kbs.{eds,dcf}` and edit the DCF
   `[DeviceCommissioning] NodeID=0x05`. gen-network resolves the node from
   the DCF NodeID first, the filename second — a mismatch registers the
   wrong node.
3. **`bridge/devices/overrides.yml`**, one phtemp block per stem
   (`PH-TempModule-n04-250kbs:` and `PH-TempModule-n05-250kbs:`):
   - keep `"6003:0": {unit: mV, scale: 0.1}` and `"6010:0": {unit: °C, scale: 0.1}`
   - **remove `"6000:0"` and `"2400:0"` if present** — the merge uses
     `setdefault`, so a stale key silently creates a phantom object instead
     of failing
   - optional: `sdo:` list for 0x1800:03 if you want a slower TPDO1
4. Sweep docs/tests that still describe TPDO1 as 6 bytes with pHValue
   (`CLAUDE.md`, `README.md`, `docs/device-management.md`, the briefing
   test in `api/test/server-routes.test.ts`).
5. `cd bridge && make` (must not warn about the phtemp overrides), rebuild
   the bridge image, restart, confirm nodes 4 and 5 boot clean and the
   trace decodes 0x184/0x185 and 0x284/0x285 as two nodes.

### MIK

1. **Input unit:** 0x6003 is mV × 10. Divide once at the boundary (or use
   `engValue`). **Migrate or invalidate stored calibration points** captured
   before fw 4.0.0 — they are integer mV; mixing fails silently.
2. Delete every read of 0x6000 and every write to 0x2400/0x2401. The only
   pH is your derived one.
3. Electrode-disconnect detection from 0x6001 quality + mV stability (§4).
4. Temperature offset stays host-side (`T = 0x6010/10 + offset`).
5. **Heartbeat contract:** keep your 1 s heartbeat (node 127) alive while
   any module is OPERATIONAL. Handle 0x8130 as "we went quiet", then NMT
   start the node when you are back. Do not send a fault reset for it.
6. Decode 0x1001 bits 0/5 and the EMCY table above; the earlier brief's
   codes are unchanged, 0x8130 and the stack's 0x0000 are new.
7. Two units means two calibration records, keyed by node ID (4 and 5).
   Both report the same serial 0x12345678, so the serial cannot tell them
   apart.

## 6. Bench check after your changes

From `docs/PHTEMP_BENCH_TEST.md`, using the node 4 and node 5 columns:
sections B (identity, revision), C (all deletions abort), D/E (TPDOs +
values), F (0x1800:03 works), K (both units together), L (heartbeat loss →
0x8130 → PRE-OP → recovery → NMT start → RUNNING). `devices.json` for
nodes 4 and 5 must contain no `6000`, `2400`, `2401`, `2402`.
