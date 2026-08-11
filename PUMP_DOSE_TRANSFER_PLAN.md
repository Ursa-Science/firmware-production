# Pump Dose-Engine Transfer Plan — firmware → MIK

**Status: PLANNED — BLOCKED.** Do not start until Dakota has had an in-depth
conversation with the FE (MIK app) side and the open questions below are
answered. This document is the agenda for that conversation and the record of
its outcome (fill in the Decisions section as they land).

Written 2026-08-11, following the pump stepper-code audit (see Context.md).

---

## Goal

Strip the dosing engine out of the pump firmware. Dose sequencing, completion,
and timeout become the MIK application's responsibility. The firmware becomes a
plain CiA 402 velocity device:

- **In:** ControlWord (0x6040) + TargetFlowRate (0x6042)
- **Out:** StatusWord (0x6041), ActualFlowRate (0x6043), PumpState (0x2400),
  ErrorRegister / fault detail (per PUMP_FAULT_FEEDBACK_PLAN.md)

Rationale: KISS — the dose tracker is the hairiest state logic left in
motor_control.c (~400 lines: latching, pause tracking, `just_completed`
restart-blocking, completion-vs-timeout twins), and the front end already owns
config/calibration (firmware has no persistence). Consistent with the
one-object-one-writer direction of the MIK transport refactor.

## Scope of removal (firmware, devices/pump only)

From `motor_control.c`:
- `DoseTracker_t` + `dose_tracker` state, pause/resume tracking
- Dose parameter latching + consume-on-latch in `ExecuteEntryActions`
- `MotorControl_CheckDoseProgress()` (completion + timeout twins)
- `HandleDoseCompletionFlags()`, dose blocking in `CheckTransitionBlocking`,
  `log_once.dose_blocking`
- Dose-abort code in quickstop / fault / disable / NMT-reset paths
- `DOSE_STATUS_*` defines, DoseStatus/DoseDelivered process-image updates

From the OD/EDS (see Process below): 0x2300 DoseVolume, 0x2301 DoseFlowRate,
0x2303 DoseStatus, 0x2304 DoseDelivered, 0x2305 DoseTimeout — subject to
Q1/Q4 decisions (a position/delivered counter may stay or replace them).
`procimg_api.h` dose accessors go with whatever the OD decision is.

**Stays:** ControlWord bit 4 (LAST_KNOWN_FLOWRATE) and `last_known_target_flow`
— flow control remains firmware. FlowCorrectionFactor (0x2200) — see Q1.

## Open questions for the FE conversation

**Q1 — Where does delivered-volume truth live?**
Today firmware integrates actual steps (`Stepper_GetPosition` delta) — ground
truth. If MIK doses by flow × time it inherits ramp and latency error.
Option A: firmware keeps exposing a step/position or delivered-volume counter
in a TPDO; MIK integrates from step-truth. Cheap to keep, preserves the
catch-test → FlowCorrectionFactor calibration loop.
Option B: MIK integrates SYNC-paced ActualFlowRate (10 Hz) over time; accept
the error. Decide, and decide who applies FlowCorrectionFactor.

**Q2 — Stop-latency overshoot tolerance.**
Firmware-side dose stop = one main-loop cycle. MIK-side stop = CAN + gateway +
app latency (tens of ms, jittery). ~0.03 ml per 20 ms at 100 ml/min. State the
accepted dosing tolerance explicitly; if it's tight, Q1 Option A plus a
firmware "stop at step count N" primitive is the fallback (a much smaller
primitive than the current dose engine).

**Q3 — Runaway failsafe when the MIK dies mid-run.**
With dosing upstairs, a dead master = pump runs at commanded flow forever. The
deleted dose timeout is currently a quiet runaway guard. The existing
`MCO_EVENT_HEARTBEAT_LOST → MotorControl_EmergencyStop()` path is the intended
backstop — VERIFY heartbeat consumption of the master is actually configured
on the pump node, and bench-test it (kill the gateway mid-run, confirm the
pump stops). Decide the required stop deadline (heartbeat period × factor).

**Q4 — OD contract / EDS churn.**
Remove 0x2300-0x2305 outright, or keep them reserved to reduce app-side
migration pain? Whatever is decided: EDS changes are a Windows CANopen
Architect regen, propagated to BOTH repos (this one + CANOpenGateway
`bridge/devices/` + bridge image rebuild) + MIK update. **Batch this with the
fault-feedback Phase 2 objects (0x2500-0x2503) — one regen / propagate /
revalidate cycle, not two.**

**Q5 — Who owns the dose UX states?**
DoseStatus semantics (IDLE/RUNNING/COMPLETE/TIMEOUT/ABORTED/ERROR) move to the
MIK. Side benefit: the old driver bug ("false dose-ended monitored PumpState
instead of DoseStatus") becomes structurally moot — MIK owns completion
outright. Confirm the MIK state model covers pause/resume (HALT bit) the way
the firmware tracker did.

## Decisions (fill in after the FE conversation)

- Q1:
- Q2:
- Q3:
- Q4:
- Q5:

## Sequencing (agreed direction, gated on the above)

1. **Pump fault feedback Phase 0/1** (already NEXT in Context.md;
   firmware-only, independent of this plan).
2. **Dose strip + fault-feedback Phase 2 EDS objects as ONE combined OD
   change** (single Windows regen + both-repo propagation + bridge image
   rebuild + revalidation).
3. **Surviving audit-item-4 dedups** whenever convenient (TMC
   read/ReadDiag paths, 4× hybrid-stop sequence in stepper.c,
   pwm_control.arr_value mirror). The dose complete/timeout dedup is
   CANCELLED — that code is deleted by this plan, don't polish it.

## Validation checklist for the strip

- [ ] Continuous run/stop/quickstop regression (same bench sequence as the
      2026-08-11 audit validations)
- [ ] MIK-driven dose end-to-end vs. catch test (accuracy within the Q2
      tolerance)
- [ ] Heartbeat-lost runaway test: kill gateway mid-run → pump stops within
      the Q3 deadline
- [ ] EDS: SDO read of removed/kept objects matches the Q4 contract;
      bridge shadow + MIK display correct
- [ ] valve/phtemp untouched (change confined to devices/pump + its EDS)
