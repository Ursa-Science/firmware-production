# pH/Temp Calibration-Strip Plan — firmware → MIK ("dumb module")

**Status: IN PROGRESS — all decisions SETTLED 2026-08-24.**
**Step 1 (firmware-first strip) CODE COMPLETE + BUILDS + HW-VALIDATED 2026-08-24**
— cal engine removed from `ph_sensor.{c,h}` (now a raw-mV driver),
`sensor_control.c`, `procimg_api.h`, `sensor_control.h`; temp offset removed from
`temp_sensor.{c,h}`. `cmake --build --preset phtemp` links clean (text 42172 B).
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

`0x6003 pHMillivolts` is the **ADC-referred voltage, 0–2048 mV** (Vref =
2.048 V, MCP1501 precision ref), i.e. the electrode signal *after* the MikroE
pH-2 Click analog front-end (gain G + Vref/2 bias), not the bare ±mV electrode
potential. The MIK's calibration must therefore invert the front-end: the old
firmware `cal_slope ≈ 1/G` (≈ 0.3–0.5) and `neutral_voltage ≈ Vref/2` encode
exactly this. Because the MCP4017 gain is fixed at POR default, G is a stable
per-hardware constant — the MIK's 1/2/3-point cal absorbs it. **MIK owners: pH =
Nernst(mV, T) then apply your slope/offset; the mV is ~1.024 V at pH 7.**

---

## Target OD contract

| Object | Name | Type | Disposition |
|---|---|---|---|
| 0x6003 | pHMillivolts | U16 | **KEEP** — primary pH output (ADC mV) |
| 0x6010 | Temperature | I16 °C×10 | **KEEP** — raw temp |
| 0x6001 | pHSignalQuality | U8 | KEEP (ADC-noise health) |
| 0x6011 | TempSignalQuality | U8 | KEEP (CRC health) |
| 0x6002 | pHSensorStatus | U8 | KEEP (I²C/error state) |
| 0x6012 | TempSensorStatus | U8 | KEEP (1-Wire state) |
| 0x2300 | SensorStatus | U8 bitfield | KEEP (health rollup; absorbs CONNECTED/RESPONDING) |
| 0x1001 | ErrorRegister | U8 | KEEP |
| 0x6040 | ControlWord | U16 | KEEP, strip cal bits 0 & 1 (keep fault-reset bit 7) |
| 0x6041 | StatusWord | U16 | KEEP, strip SW_CALIBRATING bit 2 |
| 0x2400 | pHDeltaThreshold | U16 | KEEP, **repurpose → MillivoltDeltaThreshold** (was pH×100) |
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
  qualities, statuses, StatusWord, SensorStatus, ErrorRegister. Optionally OR the
  salvaged CONNECTED (mV in 100–3200 range) and RESPONDING (not ERROR) bits into
  SensorStatus.
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
2. **EDS regen** (delete objects + TPDO1 remap) as ONE Windows cycle →
   firmware-production EDS + CANOpenGateway `bridge/devices/` + bridge image
   rebuild. **Mechanical change-list: `docs/PHTEMP_EDS_REGEN_CHANGELIST.md`.**
3. **Rewire** `sensor_control`/`procimg_api` onto the regenerated OD (mV delta
   threshold, salvaged SensorStatus bits) → build → flash → revalidate.
4. **MIK side (separate, upstream):** implement Nernst + temp-comp + 1/2/3-point
   cal + electrode-health + persistence, consuming mV/temp. Produce a
   `MIK_INTEGRATION_BRIEF` addendum (the pH contract) mirroring the pump brief.

## Validation checklist

- [ ] mV (0x6003) + Temperature (0x6010) report correctly over CAN (SDO + TPDO1),
      hand-warm test on the probe (temp) and buffer-swap test (mV moves).
- [ ] TPDO1 is 4 B (mV+temp); pHValue/cal objects SDO-read as *absent* (abort),
      matching the target contract.
- [ ] Signal-quality + sensor-status + StatusWord still behave (warmup→running,
      fault on dual-sensor error, fault-reset via CW bit 7).
- [ ] No CALIBRATING state/bit reachable; ControlWord bits 0/1 are no-ops.
- [ ] Flash note: phtemp fixture has **NRST NOT wired** + a 2048 ms IWDG →
      flash with `st-flash --connect-under-reset --reset` (wire ST-LINK NRST) to
      avoid the mid-write watchdog PGSERR (see Context.md phtemp flash gotcha).
- [ ] Gateway bridge shadow + MIK display read mV/temp with correct units.
- [ ] valve/pump untouched (change confined to `devices/phtemp` + its EDS).
- [ ] MIK end-to-end: buffer pH 4/7/10 → MIK computes pH from mV within tolerance
      of the old on-module 3-point result (regression vs prior behavior).
