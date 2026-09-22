# pH/Temp Calibration-Strip Plan — firmware → MIK ("dumb module")

**Status: IN PROGRESS — all decisions SETTLED 2026-08-24; Steps 1-3 DONE
2026-09-22, awaiting commit + gateway cutover + bench re-validation.**
**Step 2 (EDS regen) + Step 3 (rewire) DONE 2026-09-22** — regenerated OD in
tree (11 objects gone, TPDO1 6→4 B, 0x2400 = MillivoltDeltaThreshold), shim
removed, RevisionNumber hand-set to **0x00020000** (pump convention), FW banner
**4.0.0**. **Firmware review fixes landed the same day** (all 7 items in
"Firmware review findings" below): 0x1001 to both homes, EMCYs, warm-up
latch, absent-probe reporting + re-init on fault reset, 0x2222 demo SDO
removed, dead code/comments. **0x6003 and 0x2400 are now mV × 10** (see
Decisions). `cmake --build --preset phtemp-n04` links clean (text 41384 B).
Review findings + the remaining cutover steps: see "Review 2026-09-22" below
and `docs/PHTEMP_EDS_REGEN_CHANGELIST.md`.
**Step 1 (firmware-first strip) CODE COMPLETE + BUILDS + HW-VALIDATED 2026-08-24**
— cal engine removed from `ph_sensor.{c,h}` (now a raw-mV driver),
`sensor_control.c`, `procimg_api.h`, `sensor_control.h`; temp offset removed from
`temp_sensor.{c,h}`. Build then: `--preset phtemp` (now `phtemp-n04`), text 42172 B.
**Bench proof: `cantrace-ph-calibration-strip.csv` (Samsung T5), node 4, flashed
+ NMT-Start.** NMT PRE-OP→OP clean, HB steady, ErrReg 0x1001=00. TPDO1
`[pHValue|Temp|mV]`: pHValue FROZEN at 0x02BC=700 (pimg default — on-module pH
compute is gone; also confirmed by SDO 0x6000 read = BC 02) while Temperature
(~21.8°C, 0x00DA) and mV (live 1406–1769 mV) both update. TPDO2 SensorStatus=0x03
(electrode+temp OK, no CALIBRATING), TempQual=100%, pHQual tracks ADC variance.
EDS still exposes the doomed cal objects (deleted at Step 2 regen); until then
0x6000 stays frozen by design. Mirrors the completed pump
dose-strip refactor (`docs/PUMP_DOSE_TRANSFER_PLAN.md`): move all calibration
computation and its persistence up to the MIK; the module reports only what a
sensor physically measures.

---

## Goal — the "dumb pH/Temp module"

Strip the pH calibration engine and the Nernst math out of the phtemp firmware.
The module reports two raw physical quantities and its own health; the MIK owns
every conversion and all calibration state.

- **Firmware reports: electrode millivolts + temperature (+ sensor health).**
  Nothing pH-unit, nothing calibration-derived.
- **MIK owns ALL pH arithmetic**: Nernst equation, temperature compensation,
  1/2/3-point calibration slopes/offsets, electrode-health scoring, and the
  persistence of every calibration constant (per-probe, per-buffer).
- Consistent with the locked project rule *"Firmware has no persistence; front
  end owns config"* — calibration already lived only in RAM on the module and
  was lost on every reset, so there is **nothing to migrate, only to delete**.
  The MIK already had to re-establish cal after any power cycle.

This is the exact philosophy shift the pump got: the module speaks only native
units (mV, °C×10), the MIK does the science.

---

## Audit — what the phtemp firmware does today

### pH path (`Core/Src/ph_sensor.c`, ~770 lines)
- **Raw acquisition (KEEP):** MCP3221 12-bit I²C ADC, 10 Hz, 4-sample average →
  `voltage` (0–2.048 V) → `pH_GetMillivolts()` returns `voltage×1000` (0–2048
  mV, ADC-referred). Rolling 8-sample stddev → signal quality (ADC-noise based).
- **Calibration engine (STRIP):** `pH_Cal_t` struct (neutral voltage, cal
  slope/offset, acid/alka piecewise segments, 3× buffer references),
  `pH_Calibrate()` with 6 commands (OFFSET / PH7 / PH4 / PH10 / 3POINT / RESET),
  `pH_NernstRaw()`, `pH_NernstCalculate()`, `pH_ResetCalibration()`,
  `pH_GetCalMode()`.
- **Nernst / temp-compensation (STRIP):** `pH_SetTemperature()` feeds DS18B20
  temp into the Nernst slope; only used to produce the pH value — irrelevant
  once pH leaves the module.
- **Electrode status (STRIP, salvage 2 bits):** `pH_GetElectrodeStatus()` —
  6-bit field; 4 bits are calibration-derived (CALIBRATED, SLOPE_OK, OFFSET_OK
  and the neutral-voltage check). Only CONNECTED (mV in range) and RESPONDING
  (I²C OK) are pure sensor-health → fold into SensorStatus, drop the object.

### Temperature path (`Core/Src/temp_sensor.c`)
- **KEEP:** DS18B20 1-Wire bit-bang, CRC8, raw register → °C×10 (fixed physical
  transform, LSB 0.0625 °C — this is unit conversion, *not* calibration). CRC →
  temp signal quality.
- **SETTLED (2026-08-24): remove.** `Temp_SetOffset()` / `ts.offset` /
  `0x2210 TempOffset` is a per-probe user trim = calibration. The module reports
  strictly raw °C×10; the MIK applies the offset. Keeps calibration
  single-owner, matching the pump's FlowCorrectionFactor deletion.

### Orchestration (`Core/Src/sensor_control.c`, CiA-404 bridge)
- **KEEP:** NMT-gated state machine (DISABLED→WARMING_UP→RUNNING→FAULT),
  ControlWord/StatusWord, delta-threshold TPDO triggering, process-image writes,
  LED control, MCO event handling. (Directly parallels the pump's `motor_control`.)
- **STRIP:** `SensorControl_CheckCalibrationCommand()`, the pH-cal and temp-cal
  branches in `SensorControl_ProcessControlWord()`, `pH_SetTemperature()` feed,
  `Temp_SetOffset()` feed, `SW_CALIBRATING` StatusWord bit, and the pHValue /
  CalMode / ElectrodeStatus process-image writes.

### Object dictionary (`MCO_CiA401__User/EDS/`, node 4, product code 4)
- **Dead OD entries confirmed by grep** (present in EDS + procimg_api.h, never
  read/written by any firmware logic): `0x2202/0x2203/0x2204 CalibrationBuffer4/
  7/10`, `0x2221 ElectrodeAge`, `0x2222 LastCalibrationDate`. Pure vestige.
- **MCP4017 digital gain rheostat (0x2E):** `#define`d in `ph_sensor.h` but
  **never driven in code** — the analog front-end gain sits at its power-on
  default. Not part of this refactor; noted so MIK cal treats front-end gain as
  a fixed constant (see Contract note). Latent item if variable gain is ever
  wanted.

### Persistence
- None. No NVM writes anywhere in phtemp. Confirms the clean-delete path.

---

## Critical contract note — what "mV" means

`0x6003 pHMillivolts` is the **ADC-referred voltage in mV × 10, 0–20480**
(Vref = 2.048 V, MCP1501 precision ref; 0.1 mV resolution — the same ×10
convention as `0x6010 Temperature`, DECIDED 2026-09-22), i.e. the electrode
signal *after* the MikroE pH-2 Click analog front-end (gain G + Vref/2 bias),
not the bare ±mV electrode potential. The MIK's calibration must therefore
invert the front-end: the old firmware `cal_slope ≈ 1/G` (≈ 0.3–0.5) and
`neutral_voltage ≈ Vref/2` encode exactly this. Because the MCP4017 gain is
fixed at POR default, G is a stable per-hardware constant — the MIK's
1/2/3-point cal absorbs it. **MIK owners: mV = 0x6003 / 10; pH =
Nernst(mV, T) then apply your slope/offset; 0x6003 reads ≈ 10240 at pH 7.**
Why ×10: one pH unit is ≈18–30 ADC-mV through the front-end, so integer mV
quantised to ≈0.03–0.06 pH; 0.1 mV gives ≈0.004 pH and keeps the 4-sample
average's fractional counts (ADC LSB is 0.5 mV). `0x2400
MillivoltDeltaThreshold` uses the same unit (default 10 = 1.0 mV).

---

## Target OD contract

| Object | Name | Type | Disposition |
|---|---|---|---|
| 0x6003 | pHMillivolts | U16 **mV×10** | **KEEP** — primary pH output (ADC mV × 10, 0–20480) |
| 0x6010 | Temperature | I16 °C×10 | **KEEP** — raw temp |
| 0x6001 | pHSignalQuality | U8 | KEEP (ADC-noise health) |
| 0x6011 | TempSignalQuality | U8 | KEEP (CRC health) |
| 0x6002 | pHSensorStatus | U8 | KEEP (I²C/error state) |
| 0x6012 | TempSensorStatus | U8 | KEEP (1-Wire state) |
| 0x2300 | SensorStatus | U8 bitfield | KEEP (health rollup; absorbs CONNECTED/RESPONDING) |
| 0x1001 | ErrorRegister | U8 | KEEP |
| 0x6040 | ControlWord | U16 | KEEP, strip cal bits 0 & 1 (keep fault-reset bit 7) |
| 0x6041 | StatusWord | U16 | KEEP, strip SW_CALIBRATING bit 2 |
| 0x2400 | pHDeltaThreshold | U16 **mV×10** | KEEP, **repurposed → MillivoltDeltaThreshold** (was pH×100; now mV×10, default 10 = 1.0 mV) |
| 0x2401 | TempDeltaThreshold | I16 | KEEP |
| 0x2402 | StatusDeltaThreshold | U8 | KEEP |
| 0x2000 | LEDControl | U8 | KEEP |
| **0x6000** | **pHValue** | U16 | **DELETE** — MIK computes pH |
| **0x2200** | **pHCalibrationCommand** | U8 | **DELETE** |
| **0x2201** | **pHCalibrationStatus** | U8 | **DELETE** |
| **0x2202/03/04** | **CalibrationBuffer4/7/10** | I16 | **DELETE** (already dead) |
| **0x2205** | **pHCalibrationMode** | U8 | **DELETE** |
| **0x2220** | **pHElectrodeStatus** | U8 | **DELETE** (cal-derived; salvage 2 bits into 0x2300) |
| **0x2221** | **ElectrodeAge** | U16 | **DELETE** (dead) |
| **0x2222** | **LastCalibrationDate** | U32 | **DELETE** (dead) |
| **0x2210** | **TempOffset** | I16 | **DELETE** — MIK owns temp trim (settled) |

### PDO layout

- **RPDO1** (0x200+node): ControlWord (2 B) — *unchanged*.
- **TPDO1** (0x180+node): was pHValue(2)+Temperature(2)+pHMillivolts(2)=6 B →
  **pHMillivolts(2)+Temperature(2)=4 B** (drop pHValue). This is the measurement
  PDO; delta-triggered on mV / temp change.
- **TPDO2** (0x280+node): SensorStatus(1)+pHSignalQuality(1)+TempSignalQuality(1)
  +ErrorRegister(1)=4 B — *unchanged*.

---

## Scope of change (firmware, `devices/phtemp` only)

**`ph_sensor.c` / `ph_sensor.h`** — reduces to a raw electrode-mV driver:
- Delete `pH_Cal_t`, `pH_Calibrate()`, `pH_NernstRaw()`, `pH_NernstCalculate()`,
  `pH_ResetCalibration()`, `pH_GetCalMode()`, `pH_GetElectrodeStatus()`,
  `pH_SetTemperature()`, `pH_GetValue()`; the `ph_value`/`ph_float`/
  `temperature_c`/`cal` context fields; `PH_STATE_CALIBRATING`; `pH_CalMode_t`;
  all `PH_CAL_*`, `ELEC_*`, Nernst-constant, and `PH_NEUTRAL/SLOPE/OFFSET`
  defines; the MCP4017 defines (unused).
- Keep `pH_Init`, `pH_Process` (ADC read + 4-sample average + mV), `pH_GetMillivolts`,
  `pH_GetRawADC`, `pH_GetSignalQuality`, `pH_GetState`, `pH_ClearError`.
- `pH_Process()` loses the CALIBRATING gate and the pH/clamp block.
- **Naming:** minimal-churn option keeps the `pH_`/`ph_sensor` names (it is still
  the pH-electrode ADC). Rename to `electrode_sensor`/`mv_sensor` only if we want
  the file name to reflect "no pH here" — Open Decision.

**`temp_sensor.c` / `temp_sensor.h`** — delete `Temp_SetOffset()`, the `ts.offset`
field, and the `temp_x10 += ts.offset` line in `Temp_ReadResult()`. `Temp_GetValue()`
now returns strictly raw °C×10 (still the fixed 0.0625 °C→×10 register transform).

**`sensor_control.c`** — strip cal orchestration:
- Remove `SensorControl_CheckCalibrationCommand()` and its call.
- Remove cal branches from `SensorControl_ProcessControlWord()` (bit 0 pH-cal,
  bit 1 temp-cal); keep bit 7 fault-reset.
- Remove `pH_SetTemperature()` feed and `Temp_SetOffset()` feed (+ `last_applied_offset`,
  `last_cal_command`).
- `SensorControl_GenerateStatusWord()`: drop `SW_CALIBRATING`.
- `SensorControl_UpdateProcessImage()`: drop `ProcImg_SetpHValue`,
  `ProcImg_SetCalibrationMode`, `ProcImg_SetElectrodeStatus`; keep mV, temp,
  qualities, statuses, StatusWord, SensorStatus, ErrorRegister.
  **DECIDED 2026-09-22: no CONNECTED salvage.** SensorStatus bit 0 keeps its
  current RESPONDING meaning (ADC initialised, not IDLE/ERROR). A mV range
  check cannot detect an open electrode — the pH-2 Click front-end biases an
  open input to ≈Vref/2, which reads as pH 7. Disconnect detection is
  host-side, from pHSignalQuality (0x6001) and mV stability; say so in the MIK
  brief.
- `SensorControl_CheckDeltaTrigger()`: replace the pH-value delta with an
  **mV delta** vs `MillivoltDeltaThreshold` (0x2400); keep temp + status deltas.

**`procimg_api.h`** — delete the accessors for every DELETED object
(GetCalibrationCommand, SetCalibrationStatus, Get/SetCalibrationBuffer4/7/10,
Get/SetCalibrationMode, SetElectrodeStatus, SetElectrodeAge,
SetLastCalibrationDate, SetpHValue, and GetTempOffset). Keep mV, temp,
qualities, statuses, thresholds, LED, CW/SW.

**EDS regen (Windows / CANopen Architect — the only non-macOS step):** delete the
objects above; drop pHValue from the TPDO1 mapping (6 B → 4 B). Regenerate →
copy into `devices/phtemp/MCO_CiA401__User/EDS/` → **also propagate to the
gateway** (`~/code/CANOpenGateway/bridge/devices/`) + rebuild the bridge Docker
image (per the EDS-change workflow in Context.md). One regen, both repos, one
revalidation.

---

## Decisions (all SETTLED 2026-08-24)

- **Temp offset (0x2210): DELETE.** Module reports raw temp; MIK owns the
  per-probe trim and its persistence. See "MIK temp-offset ownership" below.
- **Signal-quality objects (0x6001/0x6011): KEEP.** Pure sensor health (ADC
  stddev, CRC), not calibration — cheap diagnostics worth keeping on the bus.
  The MIK may *additionally* derive pH-signal stability from the mV stream, but
  the module still reports its own.
- **Module naming: KEEP `pH_`/`ph_sensor`.** No file/symbol rename (minimal
  churn — it is still the pH-electrode ADC driver). Update only the header
  doc-block to state "raw electrode mV; no pH/Nernst/calibration on-module."
- **MillivoltDeltaThreshold (0x2400): repurpose to mV delta.** 0x2400 keeps its
  slot/type (U16) but now thresholds TPDO1 on millivolt change (1 mV
  resolution) instead of pH×100. Confirm the MIK sets a sensible default (the
  current pH×100=10 default → pick an mV equivalent at regen).
  **2026-09-22: unit is mV × 10 (matches 0x6003); default 10 = 1.0 mV**
  (≈0.03–0.05 pH through the front-end, close to the old 0.10 pH intent).
  MIK owns config and writes 0x2400 on connect; the firmware value is a fallback.
- **0x6003 unit = mV × 10 (DECIDED 2026-09-22, breaking vs. the 05-24 EDS
  which was integer mV).** Rationale in the contract note above. No EDS
  rename (Temperature is ×10 and un-suffixed too); gateway override
  `6003:0: {unit: mV, scale: 0.1}`.
- **ErrorRegister / EMCY mapping (DECIDED 2026-09-22, phtemp-specific — NOT
  the pump's motor codes).** 0x1001: bit 0 generic (set when any other bit is
  set), bit 5 device-profile = a measurement channel faulted (pH ADC or temp
  probe in ERROR or absent at init). Bits 1–4 unused on purpose (bit 3 = device
  overheating, bit 4 = CAN comms — neither applies). EMCY 0x5000 device
  hardware per sensor-fault episode, MSEF[0] 1 = pH ADC / 2 = temp probe,
  MSEF[1] driver state; EMCY 0xFF00 device-specific on application FAULT,
  MSEF[0] cause 1 = no sensor ready after warm-up / 2 = both channels
  faulted / 3 = stack emergency stop, MSEF[1..2] pH/temp state; EMCY 0x0000 on
  ControlWord bit-7 reset. Register written to BOTH homes (PI for TPDO2,
  `gMCOConfig.error_register` for SDO + EMCY), edge-wise so the stack's own
  bit-0 use is not stomped.
- **Identity (2026-09-22, mirrors the pump): RevisionNumber 0x1018:03 =
  0x00020000.** Upper word = OD layout generation (bump on breaking layout
  change), lower word = compatible additions. FW version string is a separate
  number (4.0.0). Hand-edited into .eds/.dcf/pimg.h; **sync the .cax before the
  next Architect regen.** 0x1009/0x100A stay "1.0" placeholders for now (see
  "EDS objects that can still go").
- **Cutover is coupled (2026-09-22).** The gateway's master image checks
  1018:03 at slave boot (`gen-network.py` disables only the serial check), so
  the new firmware and the new gateway EDS/image must land in the same window.
  Order: gateway files + overrides + docs + image rebuild → flash → boot check.

## MIK temp-offset ownership

The module now reports **raw** °C×10 (`0x6010`), exactly as the DS18B20's
register gives it (LSB 0.0625 °C, factory-trimmed to ±0.5 °C). The MIK is the
sole owner of any per-probe correction:

- **Storage.** The MIK keeps a persisted `temp_offset_c10` per probe/module
  (same store as the pH calibration constants — one calibration record per
  physical unit, identified by node ID / serial). Firmware holds nothing across
  a reset, so the MIK is authoritative by construction; there is no
  read-back-merge step and no risk of two owners disagreeing.
- **Apply.** On every temperature sample the MIK reads raw `0x6010` and displays
  `T_shown = T_raw + temp_offset_c10`. The module never sees the offset. (This
  is byte-identical to what `Temp_ReadResult()` used to do with `ts.offset` —
  the arithmetic just moves upstairs.)
- **Set (one-point offset cal).** User puts the probe in a known reference
  (ice-bath 0.0 °C, a calibrated thermometer, or a bath at a known temp). MIK
  takes `temp_offset_c10 = T_reference − T_raw` from a settled raw reading and
  persists it. Optional: average N raw samples first, mirroring the old
  4-sample settle.
- **Reset.** MIK sets `temp_offset_c10 = 0` — no module round-trip needed.
- **No CAN write path required.** Because correction is display-side, the old
  `0x2210` write + ControlWord bit 1 handshake disappears entirely; nothing on
  the bus carries the offset. (If the MIK ever wants the *module* to pre-correct
  — e.g. a headless logger reading TPDOs directly — that's a future OD addition,
  not part of this refactor.)

## Sequencing

1. **Firmware-first strip** of `ph_sensor` + `sensor_control` + `procimg_api`
   (OD-independent deletions build clean against the *current* EDS as long as we
   don't touch the removed OD entries' storage). Bench-check mV + temp still
   report over CAN.
2. ~~**EDS regen** (delete objects + TPDO1 remap) as ONE Windows cycle~~ DONE
   2026-09-22 (in tree, uncommitted). Gateway half NOT done — it is part of
   the cutover in step 3b. **Mechanical change-list + review:
   `docs/PHTEMP_EDS_REGEN_CHANGELIST.md`.**
3. ~~**Rewire** `sensor_control`/`procimg_api` onto the regenerated OD~~ DONE
   (mV delta threshold live; shim deleted; no SensorStatus salvage by
   decision). Remaining:
   - **3a. Commit** the regen + rewire as one commit on top of the
     build-parameter branch (or after it merges), message naming the OD change.
   - **3b. Gateway cutover** (one window): copy .eds/.dcf → in
     `bridge/devices/overrides.yml` delete `"6000:0"` and set
     `"6003:0": {unit: mV, scale: 0.1}` (+ `"2400:0": {unit: mV, scale: 0.1}`)
     → sweep CLAUDE.md / README /
     docs/device-management.md / `api/test/server-routes.test.ts` for the
     6-byte TPDO1 + pHValue wording → rebuild bridge image → flash
     `build/phtemp-n04-250k.bin` → node 4 boots clean → bench checklist.
   - **3c. Sync the .cax** (RevisionNumber 0x00020000) before anyone
     regenerates phtemp again.
4. **MIK side (separate, upstream):** implement Nernst + temp-comp + 1/2/3-point
   cal + electrode-health + persistence, consuming mV/temp. Produce a
   `MIK_INTEGRATION_BRIEF` addendum (the pH contract) mirroring the pump brief.
   Include: disconnect detection is host-side (quality/stability), 0x2400/0x2401
   are the MIK's TPDO-cadence knobs (write on connect), and the mV resolution
   caveat from the firmware review below.

## Validation checklist

- [ ] mV (0x6003) + Temperature (0x6010) report correctly over CAN (SDO + TPDO1),
      hand-warm test on the probe (temp) and buffer-swap test (mV moves).
- [ ] TPDO1 is 4 B (mV+temp); pHValue/cal objects SDO-read as *absent* (abort),
      matching the target contract.
- [ ] Signal-quality + sensor-status + StatusWord still behave (warmup→running,
      fault on dual-sensor error, fault-reset via CW bit 7).
- [ ] No CALIBRATING state/bit reachable; ControlWord bits 0/1 are no-ops.
- [ ] **0x6003 ≈ 10240 at pH 7 buffer** (mV × 10); TPDO1 word 0 matches the
      SDO read; a 0x2400 write of 10 (1.0 mV) delta-triggers TPDO1 on a
      buffer swap within the 500 ms inhibit.
- [ ] **0x1001 both homes agree**: with one probe unplugged, SDO read 0x1001
      == TPDO2 byte 3 == 0x21; EMCY 0x5000 seen once (MSEF[0] = 1 pH / 2 temp).
- [ ] **Absent probe at boot** is announced: boot with the DS18B20 unplugged →
      SW bit 5, SensorStatus bit 1 clear, 0x1001 = 0x21, EMCY 0x5000/2; module
      still reaches RUNNING on pH alone. Plug it in → CW 0x80 → re-init
      succeeds, bits clear, EMCY 0x0000.
- [ ] **Temp-only warm-up**: boot with the MCP3221 disconnected → RUNNING on
      temp alone (not FAULT); SW bit 4 + 0x1001 0x21 + EMCY 0x5000/1.
- [ ] **FAULT path**: both probes unplugged → FAULT, EMCY 0xFF00 cause 2
      (or cause 1 if from warm-up); CW 0x04→0x84 → DISABLED, EMCY 0x0000.
- [ ] SDO read 0x2222:23 → abort 0x06020000 (demo handler gone).
- [ ] Flash note: phtemp fixture has **NRST NOT wired** + a 2048 ms IWDG →
      flash with `st-flash --connect-under-reset --reset` (wire ST-LINK NRST) to
      avoid the mid-write watchdog PGSERR (see Context.md phtemp flash gotcha).
- [ ] **Gateway boot of node 4 clean** — no 1018:03 identity mismatch;
      SDO 0x1018:03 reads `00 00 02 00`; banner says `Firmware: 4.0.0`.
- [ ] Gateway bridge shadow + MIK display read mV/temp with correct units;
      devices.json for node 4 has no `6000:0`.
- [ ] valve/pump untouched (change confined to `devices/phtemp` + its EDS).
- [ ] MIK end-to-end: buffer pH 4/7/10 → MIK computes pH from mV within tolerance
      of the old on-module 3-point result (regression vs prior behavior).

---

## Review 2026-09-22 (post-regen, post-build-parameter change)

Reviewed: the regenerated OD vs. the changelist, the tree on branch
`build/node-id-parameter` (node ID + bitrate now CMake parameters), the gateway
repo's consumption of the phtemp EDS, and the phtemp firmware itself.

**Verified.** Regen matches sections A/B/C; build links; text 42172 → 42100 B;
`NODEID_DCF`/`CAN_BITRATE_DCF` untouched so the build parameters still apply;
gateway `bridge/devices/` copies are the pre-regen files (05-24).

**Findings → actions taken.**
- Revision convention inconsistent across devices (pump major-bumped, phtemp
  regen minor-bumped, valve untouched) → **0x00020000 hand-set**, rule written
  under Decisions. Valve stays 0x00010001 until its OD next changes.
- Cutover sequencing in the old plan (flash, then propagate) would dead-node
  the module on the revision check → **cutover rewritten** as one window.
- Gateway `overrides.yml` still carries `"6000:0"` (would become a phantom
  object), and gateway docs/tests still describe the 6-byte TPDO1 → **sweep
  added to the cutover**.
- 0x2400 default 10 vs. changelist's 5 → **kept 10, doc corrected**.
- "Optional" SensorStatus salvage left open → **closed: no salvage**.
- Stale target names (`--preset phtemp`, `build/phtemp.elf`) → corrected in both
  docs. Also stale elsewhere: Context.md L138/L142, BUILD_NOTES.md L29, and the
  `.cax` name (`Modules-base.cax` now) in BUILD_NOTES.md.

**Findings → still open (not phtemp-blocking).**
- CMAKE_GUIDE §3.9 says nothing about the gateway side of "node ID as build
  parameter": the gateway still needs one EDS+DCF pair per commissioned node
  (DCF `NodeID` / `-nNN` filename resolve the node). Pump-n02 has no gateway
  pair; `overrides.yml` keys pump metadata by `PumpModule-n02-250kbs` while the
  file present is `-n01`, and those overrides still name the deleted 6042/6043
  dose objects. Gateway-repo housekeeping.
- CubeIDE `.cproject/.project/.mxproject` files are still tracked under
  `devices/phtemp` and `devices/valve` although Context.md says they were
  deleted. BUILD_NOTES.md's status table still marks pump/phtemp unvalidated.

## EDS objects that can still go (config is owned upstream)

Asked 2026-09-22: with the gateway/MIK owning config, what else in the phtemp
EDS is dead weight? Inventory of the 39 objects after the regen, against what
the stack (`nodecfg.h`) actually compiles and what the gateway writes at boot.
**None of these were deleted — they need an Architect regen (hand-editing the
`entriesandreplies.h` index tables for deletions is not worth the risk).
Batch them into the next regen together with the .cax revision sync.**

| Object | Verdict | Why |
|---|---|---|
| 0x1010 / 0x1011 Store/Restore Parameters | **DELETE** | `USE_STORE_PARAMETERS 0` — advertised but non-functional; contradicts "firmware has no persistence". |
| 0x1020 Verify Configuration | **DELETE** | Only meaningful with store parameters. Two rw U32 in the PI that nothing reads. |
| 0x1002 Manufacturer Status Register | **DELETE** | PI entry, never written; reads 0 forever. |
| 0x1012 / 0x1013 Time Stamp COB-ID / Hi-res Time Stamp | **DELETE** | No consumer of time on the module (the `USECB_TIMEOFDAY` callback is a stub). |
| 0x1006 / 0x1007 / 0x1019 SYNC period / window / counter | **DELETE** | All phtemp PDOs are event-driven (TType 0xFF); the module neither produces SYNC nor has synchronous PDOs. The gateway's 10 Hz SYNC is for the pump. |
| 0x1028 Emergency Consumer | **DELETE (verify)** | Slave consumes no EMCY; stack has a write handler but no consumer path is configured. Check `mco.c:2895` does not need the entry to exist. |
| 0x2402 StatusDeltaThreshold | **DELETE** | Firmware does an arithmetic diff on a bitfield, then also triggers on any bit change — the object is effectively a boolean that is always true. Hardcode "any change". |
| 0x1009 / 0x100A HW / SW version strings | **KEEP but wire** | Both are "1.0" placeholders. Wire 0x100A to `FIRMWARE_VERSION` at boot so the fw version is readable over CAN (Context.md identity finding); 0x1009 to a real HW rev when there is one. |
| 0x1003 Pre-defined Error Field | KEEP | Stack-owned EMCY history (`USE_EMCY 1`, 4 entries). |
| 0x1015 EMCY Inhibit | KEEP | Stack uses it; 2 bytes. |
| 0x1016 / 0x1017 Consumer / Producer Heartbeat | **KEEP — gateway writes them at boot** | `gen-network.py` defaults `heartbeat_consumer: true`, `heartbeat_producer: 1000`; dcfgen puts SDO writes to both in the concise DCF. Deleting either breaks boot. |
| 0x1F80 NMT Startup | KEEP | const, some masters read it. |
| 0x2000 LEDControl | KEEP (ask MIK) | MIK-driven LED mode; delete only if the MIK never writes it. |
| 0x2400 / 0x2401 mV / Temp delta thresholds | KEEP | The MIK's only knobs on TPDO1 cadence / bus load. MIK writes on connect. |
| 0x6002 / 0x6012 pH / Temp SensorStatus | KEEP (bench diagnostics) | SDO-only raw driver enums; overlap with StatusWord bits 0/1/4/5. Cheap; revisit. |
| 0x6040 / 0x6041 CW / SW | KEEP | CiA 404 pattern, pump parity, fault-reset path. Note SW is *not* mapped in any TPDO (SDO-only); TPDO2 carries the 1-byte 0x2300 subset. |
| 0x2300 SensorStatus | KEEP (layout-3 candidate) | Duplicates SW bits 0/1/3. Consolidating (map SW into TPDO2, drop 0x2300) is a breaking layout change — not now, this layout is about to be bench-validated. |

Net if the DELETE rows go: 12 objects, ~20 PI bytes, and an EDS that only
advertises what the module does.

## Firmware review findings 2026-09-22 (phtemp `Core/`) — FIXED same day

Reviewed `ph_sensor.c`, `temp_sensor.c`, `sensor_control.c`, `procimg_api.h`,
`main.c`, `user_cbdata.c`, `user_STM32.c` against the new OD. The strip is
complete and clean: no pH/Nernst/cal code path remains, no deleted OD symbol
is referenced, mV delta triggering is wired to 0x2400. Items below were found
at review and **all fixed 2026-09-22 (build 41384 B text; bench validation
pending — see the checklist above).** What was done, per item:

| # | Fix |
|---|---|
| 1 | `SensorControl_UpdateErrorRegister()` writes both homes edge-wise; PI copy mirrors the merged register. Mapping under Decisions. |
| 2 | 0x6003 / 0x2400 = mV × 10; `pH_GetMillivoltsX10()` integer path from the 4-sample sum; float removed. |
| 3 | `SensorControl_EnterFault(cause)` → EMCY 0xFF00; `SensorControl_ReportSensorErrors()` → EMCY 0x5000 per channel episode; `SensorControl_ClearFault()` → EMCY 0x0000. |
| 4 | WARMING_UP latches `temp_seen_ready` / `ph_seen_ready` every loop; gate uses the latches. |
| 5 | `PH_IS_FAULTED()` / `TEMP_IS_FAULTED()` treat init-failure as fault (SW bits 4/5, 0x1001 bit 5, EMCY); fault reset retries `pH_Init` / `Temp_Init` once. |
| 6 | phtemp `user_cbdata.c`: the four `MCOUSER_AppSDO*` callbacks are "not handled" stubs; demo buffers + `for(;;)` gone (−288 B bss). **Pump and valve still carry the demo** — same one-file change each, but each obliges that device's re-flash + re-validate, so tracked as follow-ups (`devices/pump|valve/MCO_CiA401__User/user_cbdata.c`). |
| 7 | `SensorControl_Reset()` deleted (COMM_RESET re-inits; CW bit 7 clears); `pH_ClearError()` zeroes the quality window; stale comments fixed; `cur_sw` removed. LED processing on the not-OP path deliberately left as is. |

Original findings, kept for the record:

1. **0x1001 dual-source bug — same as the pump's Phase 0a finding.**
   `SensorControl_UpdateProcessImage()` writes ErrorRegister only into the
   process image (`ProcImg_SetErrorRegister`). The stack serves **SDO reads of
   0x1001 from `gMCOConfig.error_register`** (`shared/mco/mco.c:1285`) and
   uses the same field in EMCY frames. So TPDO2 byte 3 shows the app's fault
   bits while an SDO read of 0x1001 returns 0 (unless a stack EMCY set bit 0).
   The valve has the identical symptom recorded in the gateway overrides.
   Fix as the pump did (`motor_control.c:256-263`): write both homes on fault
   entry/exit, clear only app-owned bits.
2. **No EMCY on application FAULT.** The pump pushes cause-coded EMCYs;
   phtemp only flips SW/SensorStatus/0x1001. A gateway consumer that relies
   on EMCY (event log, 0x1003 history) sees nothing when both sensors die.
   Add `MCOP_PushEMCY` at FAULT entry (e.g. 0xFF00 device-specific + which
   sensor) and 0x0000 on reset.
3. **Warm-up race for a temp-only module.** `WARMING_UP → RUNNING` requires
   `Temp_GetState() == TEMP_STATE_READY` at the instant the 2 s timer expires,
   but `Temp_Process()` auto-restarts a conversion on the very next loop after
   READY, so READY is visible for ~one iteration per 750 ms cycle. With the
   ADC present, `ph_ready` masks it (pH stays READY). If the MCP3221 is
   absent, a healthy DS18B20 almost always lands in FAULT ("no sensors ready")
   — contradicting the "single sensor failure doesn't fault" rule two cases
   down. Use `TEMP_IS_ACTIVE()` or a has-valid-reading latch for the gate.
4. **mV resolution is a contract question — decide before this layout ships.**
   0x6003 is integer mV, but the ADC LSB is 0.5 mV (12-bit / 2.048 V) and the
   4-sample average has more. Through the front-end one pH unit is ≈18–30
   ADC-mV, so 1 mV quantisation is ≈0.03–0.06 pH — coarse for a lab reading
   (typical target 0.01 pH). Options: report raw ADC counts in 0x6003 (0.5 mV,
   free), or the 4-sample sum (14-bit). Either is a breaking unit change for
   0x6003, so it belongs in *this* layout generation, not the next.
   (Temperature is fine: °C×10 vs. the DS18B20's 0.0625 °C is acceptable.)
5. **Silent absent-ADC at boot.** If `pH_Init` fails, `ph_initialized` stays
   false, so `SW_PH_FAULT` and ErrorRegister bit 0x20 (both gated on
   `ph_initialized`) never set; the only trace is SensorStatus bit 0 clear.
   Same for the DS18B20. Report init failure as the sensor's FAULT bit.
6. **Phantom SDO object 0x2222:23/0x2222:24.** `user_cbdata.c` still carries
   the EmSA custom-segmented-SDO demo on index 0x2222 (alternating test
   strings; `USECB_APPSDO_READ/WRITE 1`). 0x2222 was LastCalibrationDate until
   this regen; it is now deleted from the OD, yet an SDO read of 0x2222:23
   returns "Test of custom entry…". Pump and valve carry the same demo. Delete
   the demo branches (or set `USECB_APPSDO_* 0` in `nodecfg.h`) on all three.
7. **`SensorControl_Reset()` has no callers** — leaving OP never clears sensor
   errors or the last-TPDO trackers; only CW bit 7 does. Either call it from
   the NMT-change event or delete it.
8. **`pH_ClearError()` does not reset the quality window**, so the first
   post-recovery quality value mixes pre-fault samples. Zero `quality_count`.
9. **Stale comments** now that cal is gone: `sensor_control.c` L8 ("cal, fault
   reset"), L61-66 (`TEMP_IS_ACTIVE` mentions feeding pH compensation),
   L134 ("DQ pin (PB0)" — it is PA6), L412-416 (TPDO2 "+ StatusWord" — SW is
   not mapped in TPDO2), and the unused `cur_sw` in `CheckDeltaTrigger`.
10. **Cosmetic:** `pH_GetMillivolts()` goes through float for what is exactly
    `raw / 2`; `LEDControl_Process()` is skipped on the not-OP path so an LED
    mode written while PRE-OP does not apply until OP.
