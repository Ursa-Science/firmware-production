# pH/Temp module v4.0.0 — gateway + MIK cutover guide

**Audience:** the FE owner of `CANOpenGateway` and the MIK, plus their agent.
**Firmware side is done** (branch `build/node-id-parameter`, commit "phtemp:
dumb-module OD regen + firmware 4.0.0"). This guide is the gateway/MIK half.
Nothing here is optional; the two halves must land together (see §2).

Source of truth for the contract: `docs/PHTEMP_REFACTOR_PLAN.md` (Decisions,
contract note) and `docs/PHTEMP_EDS_REGEN_CHANGELIST.md` in
`firmware-production`. Read those before touching code.

---

## 1. What changed on the wire

Node 4, product code 4. The module no longer computes pH. It reports raw
electrode voltage + temperature + health; the host owns every conversion,
calibration constant, and its persistence.

| Object | Before (05-24 EDS) | Now (v4.0.0, rev 0x00020000) |
|---|---|---|
| 0x1018:03 RevisionNumber | 0x00010001 | **0x00020000** — checked by the master at boot |
| 0x6000 pHValue | U16 pH×100, TPDO1 word 0 | **deleted** |
| 0x6003 pHMillivolts | U16 integer mV | **U16 mV × 10** (0..20480, ≈10240 at pH 7) |
| 0x6010 Temperature | I16 °C×10 | unchanged |
| 0x2400 / 0x2401 / 0x2402 | delta thresholds (pH×100, °C×10, status) | **deleted** (2026-09-30). They never limited traffic: the stack sends on any process-image change. Use 0x1800:03 / 0x1800:05 (see §4.5) |
| 0x2200–0x2205, 0x2210, 0x2220–0x2222 | cal command/status/buffers/mode, temp offset, electrode status/age/date | **deleted** |
| TPDO1 (0x184) | 6 B `[pHValue \| Temp \| mV]` | **4 B `[mV×10 \| Temp]`** |
| TPDO2 (0x284) | `[SensorStatus \| pHQual \| TempQual \| ErrReg]` | unchanged, but ErrReg is now correct (§4) |
| RPDO1 (0x204) | ControlWord | unchanged; only bit 7 (fault reset) does anything |
| 0x6041 StatusWord bits 4/5 | sensor in ERROR | sensor in ERROR **or absent at init** |
| EMCY | stack-only (0x8130 etc.) | + **0x5000** sensor fault, **0xFF00** module FAULT, **0x0000** reset |
| 0x2222:23 / :24 | answered with a demo string (vendor sample code) | abort 0x06020000 |

Not changed: 0x6001/0x6011 signal quality, 0x6002/0x6012 driver states,
0x2300 SensorStatus, 0x2401/0x2402, 0x2000 LED, heartbeat 1000 ms, all COB-IDs.

---

## 2. Why this is one cutover, not two

`bridge/gen-network.py` feeds dcfgen the EDS; dcfgen bakes 1018:03 into the
master image and the master verifies it when the slave boots (only the serial
check is disabled). Consequences:

- old bridge image + new firmware → node 4 rejected at boot
- new bridge image + old firmware → node 4 rejected at boot

So: gateway files, overrides, code, image rebuild **first**; flash **second**;
restart bridge **third**. Do not deploy the new image to a bus that still has a
v3.x phtemp on it.

---

## 3. Gateway changes (`~/code/CANOpenGateway`)

### 3.1 Device catalog

Overwrite the two 05-24 files in `bridge/devices/` with the regenerated pair
from `firmware-production/devices/phtemp/MCO_CiA401__User/EDS/`:

```
PH-TempModule-n04-250kbs.eds
PH-TempModule-n04-250kbs.dcf
```

Same filenames on purpose: node-ID resolution uses the DCF
`[DeviceCommissioning] NodeID` (=4) then the `-n04` filename hint. Both files
carry a `[Comments] Line4` hand-edit marker for the revision bump; leave it.

### 3.2 `bridge/devices/overrides.yml` — phtemp block

Replace the `PH-TempModule-n04-250kbs:` block with:

```yaml
  PH-TempModule-n04-250kbs:
    objects:
      "6003:0":              # pHMillivolts — raw electrode voltage, mV x 10
        unit: mV             # (fw >= 4.0.0). Input to host calibration.
        scale: 0.1
      "6010:0":              # Temperature (°C x 10)
        unit: °C
        scale: 0.1
      "6041:0":
        notes:
          - "Bits 4/5 (pH / temp fault) also set when the probe was absent at
             boot; a ControlWord 0x80 reset retries the probe init once."
```

**Delete the `"6000:0"` entry.** `merge_object_overrides()` uses `setdefault`,
so a key for an object the EDS no longer has silently creates a phantom
`6000:0` in `devices.json` instead of failing. Verify after `make`:
`devices.json` node 4 must contain no `6000`.

### 3.3 Docs and tests that still describe the old layout

Grep for `6000`, `pHValue`, `pHDeltaThreshold` and fix wording, not just
values. Known hits:

| File | What it says | Change to |
|---|---|---|
| `CLAUDE.md` ~L110 | TPDO1 = pHValue / Temperature / pHMillivolts | TPDO1 = pHMillivolts(×10) / Temperature, 4 B |
| `CLAUDE.md` ~L312 | "the module's own pHValue is not the reading to use" | delete — there is no module pH any more |
| `README.md` ~L587 | same | same |
| `docs/device-management.md` ~L299, ~L507–512 | 6-byte TPDO1, "uncorrected Nernst" | 4-byte TPDO1; host is the only pH source |
| `docs/plans/calibration-capability.md`, `docs/plans/agent-hardware-context.md` | historical | add a "superseded 2026-09-22 by fw 4.0.0" header, do not rewrite |
| `api/test/server-routes.test.ts` ~L367 | "Both 0x6000 and derived.ph are served" | only `derived.ph` exists; assert the raw object is *absent* |

### 3.4 Rebuild

```bash
cd ~/code/CANOpenGateway/bridge && make          # gen-network.py + dcfgen + compile: proves the EDS parses and node 4 resolves
docker build -t canopen-bridge ~/code/CANOpenGateway/bridge
cd ~/code/CANOpenGateway/api && npm test
```

`make` must not print `WARNING: ... overrides.yml` for the phtemp block.

---

## 4. MIK / API refactor

### 4.1 Calibration input unit (`api/src/calibration.ts`)

The Nernst path takes `inputs: { raw: '6003:0', temp: '6010:0' }`. The raw
shadow value for 6003 is now **mV × 10**. Either consume `engValue`
(= raw × 0.1 from the override) or divide by 10 before the Nernst step. Do
this once at the boundary; nothing downstream should know about ×10.

**Stored calibration points are in the old unit.** Any persisted reference
captured against a v3.x module holds integer mV. Migrate on load (`×10` if
the record predates fw 4.0.0 — key it on the node's 1018:03 or a schema
version) or invalidate and force a recalibration. Silent mixing is the
failure mode to design against: a pH-7 point at "1024" vs a live reading of
"10240" projects to garbage without an error.

Resolution note: 0.1 mV ≈ 0.004 pH through the front end (≈18–30 mV per pH
after the pH-2 Click gain). Integer mV was ≈0.03–0.06 pH; that was the
reason for the change.

### 4.2 Remove every pHValue fallback

`derived.ph` is the only pH. Delete code paths that read `6000:0`, the
"prefer this over the raw object" briefing language, and any UI row for the
device's own pH.

### 4.3 Electrode-disconnect detection is host-side

The module does not flag an unplugged electrode: the front end biases an open
input to ≈Vref/2, which reads as pH 7. Derive "electrode present" from
`6001:0` pHSignalQuality (falls with ADC noise) and mV stability over time.
`6002:0` / StatusWord bit 4 only tell you the ADC chip is dead or absent.

### 4.4 Temperature offset

Unchanged from the previous brief: `T_shown = 0x6010/10 + temp_offset_c10/10`,
stored per unit on the host. 0x2210 is gone; there is no CAN write path.

### 4.5 TPDO cadence (nothing to push unless you want it slower)

There are no manufacturer threshold objects (0x2400–0x2402 were deleted
2026-09-30 — writing them aborts). The module transmits TPDO1 whenever the
mapped value changes, which for the electrode is every 400 ms sample, and
TPDO2 whenever a status/quality byte changes. The knobs are the standard
CANopen ones, all `rw`, applied by dcfgen at boot or by SDO in PRE-OP:

| Object | Unit | Default | Effect |
|---|---|---|---|
| 0x1800:03 TPDO1 inhibit time | 100 µs | 500 (50 ms) | raise to cap the rate, e.g. 10000 = at most 1 Hz |
| 0x1800:05 TPDO1 event timer | ms | 1000 | a frame at least this often even if nothing changed |
| 0x1801:03 TPDO2 inhibit time | 100 µs | 5000 (500 ms) | |
| 0x1801:05 TPDO2 event timer | ms | 5000 | |

The module holds nothing across a reset; if you change these, re-apply after
every boot (put them in `overrides.yml` `sdo:` for the node, as the valve does
for 0x2300).

### 4.6 Error register and EMCY decoding

0x1001 now agrees between SDO reads and TPDO2 (it did not before). Bits:

- bit 0 generic — set whenever any other bit is set, or on module FAULT
- bit 5 device-profile — a measurement channel is faulted (pH ADC or temp
  probe in ERROR, or absent since boot)

EMCYs from the application (stack ones such as 0x8130 heartbeat unchanged):

| Code | Meaning | MSEF[0] | MSEF[1], [2] |
|---|---|---|---|
| 0x5000 | sensor fault, once per episode | 1 = pH ADC, 2 = temp probe | driver state |
| 0xFF00 | module entered FAULT | 1 = no channel ready after warm-up, 2 = both channels faulted, 3 = stack emergency stop | pH state, temp state |
| 0x0000 | ControlWord bit-7 reset accepted | — | — |

A single faulted channel does **not** put the module in FAULT; it keeps
publishing the other channel. Surface bit 5 + the 0x5000 EMCY as a warning,
not an outage. FAULT (StatusWord bit 3, SensorStatus bit 3) is cleared by a
rising edge on ControlWord bit 7 (`0x04 → 0x84`); the reset also retries a
probe that was absent at boot.

---

## 5. Flash and verify (after §3 is merged and the image is built)

```bash
st-flash --connect-under-reset --reset write ~/code/firmware-production/build/phtemp-n04-250k.bin 0x08000000
docker compose restart bridge          # from the fleet-composition checkout
```

Pass criteria, in order:

1. Node 4 boots with no identity mismatch in the bridge log; SDO 0x1018:03 =
   `00 00 02 00`.
2. TPDO1 is 4 bytes; word 0 ≈ 10240 in pH-7 buffer; API shows 0x6003 scaled
   to mV and `derived.ph` ≈ 7 once calibrated.
3. SDO read 0x6000 → abort 0x06020000; `devices.json` node 4 has no `6000`.
4. Unplug the temp probe: TPDO2 byte 3 == SDO 0x1001 == 0x21, one EMCY
   0x5000 with MSEF[0]=2, module stays RUNNING. Plug in, write CW 0x80:
   bits clear, EMCY 0x0000.
5. Boot with the electrode ADC disconnected: RUNNING on temp alone, StatusWord
   bit 4 set, 0x1001 0x21, EMCY 0x5000/1 (old firmware would FAULT here).
6. Both probes off: FAULT, EMCY 0xFF00 cause 2; CW 0x04→0x84 → DISABLED,
   EMCY 0x0000.
7. `api` tests green; briefing for node 4 names `derived.ph` only.

---

## 6. Out of scope here (tracked in the firmware repo)

- Pump and valve still carry the vendor demo SDO handler on 0x2222 (their
  own per-device change + revalidation).
- Next phtemp EDS regen will delete dead comm objects (0x1010/0x1011, 0x1020,
  0x1002, 0x1012/0x1013, 0x1006/0x1007/0x1019, 0x1028, 0x2402) and must first
  sync RevisionNumber 0x00020000 into the `.cax`. None of those are read by
  the gateway; 0x1016/0x1017 stay because dcfgen writes them at boot.
- 0x100A still reads "1.0"; the firmware version is only in the boot banner.
