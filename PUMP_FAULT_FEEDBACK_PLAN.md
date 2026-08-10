# Pump Fault Feedback over CAN — Planning Document

Status: **PLANNING.** Research done 2026-08-10; no implementation started.
Goal: the master must learn — promptly and with cause — when the pump's
TMC2209/DIAG fault path fires, via PDO (and EMCY where it fits).

---

## 1. Verified current state (assumption check)

The working assumption was "currently there is no error feedback." Research
shows that is **partly incorrect — a generic fault DOES reach the bus today**:

**The chain that already works (code-verified 2026-08-10):**
1. TMC2209 DIAG (PA5, EXTI rising, armed at boot) → `HAL_GPIO_EXTI_Rising_Callback`
   → `Stepper_NotifyDriverFault()` sets `STEPPER_EVT_DRV_FAULT` (stepper.c:1494).
2. Main loop (`MotorControl_ProcessStepperEvents`, motor_control.c:245) confirms
   DIAG by **level** (real errors are latched; transient pulses are logged and
   counted, not faulted), then: `MOTOR_STATE_FAULT`, `PUMPSTATE_FAULT`,
   `fault_active=true`, dose aborted, step gen stopped, driver de-energized
   (EN high also clears the TMC's drv_err latch).
3. `MotorControl_UpdateProcessImage()` publishes every loop:
   - StatusWord 0x6041 → CiA-402 FAULT pattern (0x0008)
   - PumpState 0x2400 → `PUMPSTATE_FAULT` (fault overrides pump_state)
   - ErrorRegister 0x1001 → **0x01 (generic error)**
4. TPDOs (from EDS + `stackinit.h`, initialized explicitly via
   `MCO_InitTPDOFull` — NOT vulnerable to the valve's 0x1001-resolution bug):
   - **TPDO1** (0x180+id, sync type 1): StatusWord + ActualFlowRate
   - **TPDO2** (0x280+id, sync type 1): PumpState + ErrorRegister + DoseDelivered
     (1+1+4 = 6 of 8 bytes used)

So with the master's 100 ms SYNC producer running, a DIAG fault is visible to
the master **within one SYNC period, redundantly in both TPDOs** (StatusWord
FAULT + PumpState FAULT + ErrorRegister 0x01).

**What is genuinely missing:**

| Gap | Detail |
|---|---|
| No fault CAUSE | Everything collapses to "generic". Master cannot distinguish DIAG short/overtemp vs stepper position error vs TMC init/verify failure vs comms loss to TMC. |
| ErrorRegister underused | 0x1001 has standard bits (bit1 current, bit3 temperature…) — we only ever set bit0. Firmware-only fix; object is already mapped in TPDO2. |
| No TMC detail | DRV_STATUS (which phase shorted, otpw vs ot, CS_actual) exists on-chip and the runtime read path is now safe (logging is RTT-only) — but it never leaves the board except via RTT. |
| EMCY unused | `USE_EMCY=1` is compiled in and `MCOP_PushEMCY()` exists (mcop.c:670) — the stack can send standard, event-driven Emergency frames with an 8-byte payload. We never call it for motor/TMC faults. |
| No event-driven PDO | Both TPDOs are sync-type-1. 100 ms latency is acceptable, but if SYNC ever stops, fault visibility stops with it. EMCY closes this hole. |
| Master-side handling unverified | Whether the MIK python driver reacts to PUMPSTATE_FAULT / StatusWord FAULT / ErrorRegister is outside this repo. Interface contract must be agreed. |

**BUG found during research (fix regardless of this plan):**
`main.c:342` writes `gProcImg[0x67]` and `gProcImg[0x68]` (tmc_status,
init_failed_step) — but `PIMGEND=0x66`, so the process image array ends at
index 0x66. These are **out-of-bounds writes past the end of gProcImg on
every boot**, clobbering whatever the linker placed next. The comment claims
offsets "match P250000/P250100 in pimg.h" — those symbols do not exist.
Remove the writes now; reintroduce properly in Phase 2 when 0x2500/0x2501
exist in the EDS.

---

## 2. Proposed plan (phased)

### Phase 0 — verify the existing path end-to-end (bench, no code)
- Induce a fault safely (e.g. temporarily force `fault_active` via debugger, or
  a stepper position-error event) and confirm on the bus (candump / python
  master): TPDO2 shows PumpState=FAULT + ErrorRegister=0x01, TPDO1 shows
  StatusWord=0x0008, within one SYNC.
- Confirm what the MIK python driver currently *does* with these (log?
  ignore? alarm?). This defines the real starting point for master-side work.
- Exit criterion: documented trace of a fault reaching the master process.

### Phase 1 — firmware-only enrichment (no EDS change, no Windows)
- **Fix the OOB write bug** (delete the `0x67/0x68` writes).
- **Cause-coded ErrorRegister**: on fault, set standard 0x1001 bits from the
  cause — read DRV_STATUS at fault time (safe now): short → bit0|bit1
  (generic+current), ot/otpw → bit0|bit3 (generic+temperature), stepper/other
  → bit0. Master gets first-level cause with zero OD changes (0x1001 is
  already in TPDO2).
- **EMCY on fault entry/exit**: call `MCOP_PushEMCY()` when entering FAULT
  (error code 0x2310 current / 0x4210 temperature / 0x7121 motor blocked /
  0x1000 generic, per CiA 301/402 conventions) and EMCY 0x0000 on fault reset.
  8-byte payload has room for: DRV_STATUS snapshot (4B) + fault source enum
  (1B). Event-driven, SYNC-independent, natively understood by python-canopen.
- Blast radius: `devices/pump/Core` only. Re-validate pump on bench.

### Phase 2 — EDS extension for on-demand detail (needs CANopen Architect / Windows)
- Add manufacturer objects (regenerate EDS per repo workflow — tool output,
  copied into `devices/pump/MCO_CiA401__User/EDS/`, never hand-edited):
  - 0x2500 u8  TMC init status (tmc_status)
  - 0x2501 u8  TMC init failed step
  - 0x2502 u32 last DRV_STATUS snapshot (at fault, or periodic)
  - 0x2503 u8  fault source (enum: none/diag-short/diag-ot/stepper/tmc-init/tmc-comm)
- Reinstate the Phase-1-deleted status writes against the REAL offsets.
- TPDO2 has 2 spare bytes: map 0x2503 (fault source) + 0x2502 low byte, or
  keep TPDO2 as-is and leave 0x25xx for SDO-on-demand after an EMCY — decide
  with the master team (SDO read after EMCY is the cleaner contract).
- Re-flash + re-validate pump (SDO/PDO layout changed); update master EDS copy.

### Phase 3 — master-side handling (MIK repo, out of scope here)
- python driver: subscribe EMCY; on EMCY or PUMPSTATE_FAULT, stop dosing
  workflow, surface alarm, optionally SDO-read 0x2500-0x2503 for diagnosis.
- Define recovery contract: CW RESET bit (already implemented FW-side) clears
  `fault_active`; EMCY 0x0000 confirms recovery.

## 3. Constraints & notes
- EDS regeneration is Windows-only (CANopen Architect, `.cax` not vendored) —
  Phase 2 must batch all object additions into one regen session.
- Pump TPDOs are re-asserted via `MCO_InitTPDOFull` in `stackinit.h`; any
  mapping change must update BOTH the EDS and the stackinit macro (they are
  generated together — verify after regen).
- All phases: pump-only blast radius (`devices/pump/`), no shared/mco changes
  anticipated (EMCY already compiled in).
- Bench validation channel: RTT logs + candump; DIAG fault can be provoked
  non-destructively by the debugger (set `fault_active`) — do NOT short motor
  phases to test.

## 4. Open questions
1. Does the deployed master enable/require the 100 ms SYNC in all operating
   modes (fault visibility today depends on it until Phase 1 EMCY lands)?
2. Master team preference: fault detail in TPDO2 spare bytes vs SDO-on-demand
   after EMCY? (Recommend: EMCY + SDO-on-demand; keeps PDO layout stable.)
3. Should valve/phtemp adopt the same EMCY pattern later? (Same stack,
   USE_EMCY status per-device to check; separate plan if so.)
