# Pump Dose-Engine Transfer Plan — firmware → MIK

**Status: DIRECTION SETTLED 2026-08-12 — step-counter model.** Q1/Q2/Q5 are
resolved (see Decisions). Still open before implementation: Q3 heartbeat
bench test, and the Q4 index-naming choice for the regen. EDS changes must
be batched with fault-feedback Phase 2 objects (0x2500-0x2503) into ONE
Windows regen / propagate / revalidate cycle.

Written 2026-08-11 after the pump stepper-code audit; direction revised
2026-08-12 per `dose-engine refactoring-prompt.md` (step counter, not pure
velocity device) and the rate-units discussion.

---

## Goal — the "dumb pump"

Strip all dosing computation and all ml-awareness out of the pump firmware.
The MIK tells the pump how to run; the pump does it. No hidden dosing math,
no fluid units below the CAN bus. The firmware becomes a CiA 402 stepper
device with one small primitive: **run N steps at rate R, stop at zero.**

- MIK owns ALL ml arithmetic and calibration, including tubing sizes: one
  `steps_per_ml[tubing]` table (factory default per size + user fine-trim
  via catch test). Every ml the user sees is MIK arithmetic.
- Firmware speaks only native units: microsteps and microsteps/sec.
- A dosed run is **self-terminating** (stops at 0 steps even if the MIK
  dies) — strictly better than the old dose timeout it replaces.

## The contract (pump OD, motion quantities)

| Object | Meaning | Type | Direction |
|---|---|---|---|
| StepRate (repurposed 0x6042) | signed µsteps/sec; sign = direction (dose/draw) | int16 | MIK → pump (RPDO1) |
| TargetSteps (new index) | steps to run then auto-stop; 0 = continuous/jog | uint32 | MIK → pump (RPDO1) |
| ActualStepRate (repurposed 0x6043) | measured µsteps/sec, signed | int16 | pump → MIK (TPDO1) |
| StepsRemaining (new index) | countdown to auto-stop | uint32 | pump → MIK (TPDO2) |

- **RPDO1 = ControlWord(2) + StepRate(2) + TargetSteps(4) = exactly 8
  bytes.** The entire dose command (enable + rate + step count) arrives in
  ONE atomic CAN frame — no latch-ordering races by construction.
- **TPDO1 = StatusWord(2) + ActualStepRate(2)** (unchanged size).
- **TPDO2 = PumpState(1) + ErrorRegister(1) + StepsRemaining(4)** —
  StepsRemaining takes DoseDelivered's slot, same 6-byte size.
- int16 range check: 120 RPM ceiling at 1/8 µstep = 3,200 steps/s; 1 ml/min
  at current calibration ≈ 24 steps/s → 1 step/s resolution ≈ 0.04 ml/min.
  The old "minimum 1 RPM" hack and RPM quantization disappear — firmware
  computes timer ARR directly from steps/s.

**Deleted from the pump entirely (ml-awareness):** 0x2200
FlowCorrectionFactor, 0x2300 DoseVolume, 0x2301 DoseFlowRate, 0x2302
DoseCommand (dead — no firmware reader today), 0x2303 DoseStatus, 0x2304
DoseDelivered, 0x2305 DoseTimeout. In code: BASE_STEPS_PER_ML,
GetStepsPerMl(), mlPerMin_to_rpm(), rpm_to_mlPerMin(), and the dose
accessors in procimg_api.h.

## Workflows (MIK → pump)

**A. Prime / purge (continuous jog)**
1. User picks tubing size + rate in GUI. MIK: `rate_sps = ml_min ×
   steps_per_ml[tubing] / 60`.
2. RPDO1: CW=0x000F, StepRate=rate_sps, TargetSteps=0.
3. Pump runs until MIK commands stop (CW=0x0007 decel stop, or quick-stop
   bit drop). Heartbeat-lost is the runaway backstop (see Q3).

**B. Calibration (catch test, per tubing size)**
1. Prime line (workflow A), place graduated cylinder.
2. MIK commands a fixed step count at moderate rate (TargetSteps=N sized to
   ~10 ml on the current estimate).
3. Pump runs exactly N steps and stops itself — firmware is identical in
   calibration and production; nothing to switch on or off.
4. User enters measured ml. MIK stores `steps_per_ml[tubing] =
   N / measured_ml`. Repeat to converge.

**C. Dose / draw**
1. MIK: `TargetSteps = ml × steps_per_ml[tubing]`, `StepRate = ±rate_sps`
   (negative = draw).
2. One RPDO1 frame: CW=0x000F + StepRate + TargetSteps. Pump latches steps
   (consume-on-latch), decrements in the step ISR, begins decel when
   remaining ≤ ramp steps (AVR446 `accel_step_n`), stops at 0 — step-exact
   including the decel ramp.
3. MIK watches TPDO2 StepsRemaining for progress (`delivered_ml =
   (N − remaining) / steps_per_ml`); COMPLETE = remaining 0 + StatusWord
   stopped. All dose UX states (RUNNING/COMPLETE/ABORTED/…) live in MIK.

**D. Pause / resume mid-dose**
1. Pause: HALT bit (CW=0x010F). Pump decels; StepsRemaining freezes — no
   pause-time bookkeeping exists or is needed (no timeout anymore).
2. Resume: clear HALT. Pump resumes remaining count at the same StepRate.

**E. Abort**
1. Quick stop (CW bit 2 → 0) or disable (CW=0x0000). Pump stops /
   de-energizes and clears the remaining count. MIK marks ABORTED and knows
   delivered volume from the last StepsRemaining.

**F. Failsafe (nobody home)**
- Dosed run: self-terminating at 0 steps.
- Continuous run: `MCO_EVENT_HEARTBEAT_LOST → MotorControl_EmergencyStop()`
  — verification + bench test still required (Q3).

## Scope of change (firmware, devices/pump only)

Removed from `motor_control.c` (~400 lines):
- `DoseTracker_t` + `dose_tracker` state, pause/resume time tracking
- Dose parameter latching in `ExecuteEntryActions`
- `MotorControl_CheckDoseProgress()` (completion + timeout twins)
- `HandleDoseCompletionFlags()`, dose blocking in `CheckTransitionBlocking`,
  `log_once.dose_blocking`
- Dose-abort code in quickstop / fault / disable / NMT-reset paths
- `DOSE_STATUS_*` defines, DoseStatus/DoseDelivered process-image updates
- ml converters: `mlPerMin_to_rpm`, `rpm_to_mlPerMin`, `GetStepsPerMl`,
  `BASE_STEPS_PER_ML`

Added:
- `stepper.c/h` (~50 lines): `Stepper_SetStepRate(int16 sps)` (ARR direct
  from steps/s), `Stepper_StartSteps(uint32 count)` alongside
  `Stepper_StartJog()`; ISR decrements `steps_remaining`, flips to the
  existing DECELERATING path when `remaining ≤ accel_step_n`, fires new
  `STEPPER_EVT_TARGET_REACHED`. Count 0 / jog = unlimited.
- `motor_control.c`: in `ExecuteEntryActions(RUNNING)` — read TargetSteps;
  if >0 latch + zero the OD value (consume-on-latch, proven pattern) +
  `Stepper_StartSteps()`, else `Stepper_StartJog()`. TARGET_REACHED event →
  ENABLED_STOPPED (same handling as the STOPPED event).
- `procimg_api.h`: Get TargetSteps / Set StepsRemaining accessors replace
  the five dose accessors.

**Stays:** ControlWord bit 4 (LAST_KNOWN rate latch) semantics — now over
StepRate instead of TargetFlowRate.

## Decisions

- **Q1 (volume truth): RESOLVED — firmware step-truth.** MIK commands
  steps; pump executes exactly N steps (decel-ramp-aware); MIK converts
  steps↔ml with its own per-tubing calibration. FlowCorrectionFactor is
  deleted from the pump — calibration has ONE owner (MIK).
- **Q2 (stop latency): RESOLVED — moot.** Auto-stop is in the step ISR;
  step-exact delivery, no CAN/gateway/app latency in the stop path.
- **Q3 (runaway failsafe): PARTIALLY OPEN — and the verify came back
  NEGATIVE (2026-08-12).** Dosed runs are inherently bounded. Continuous
  jog relies on heartbeat-lost → E-stop, but the pump boots with NO
  heartbeat consumer armed (`INITHBCONSUMER_CALLS` empty in stackinit.h;
  0x1016 EDS defaults empty) — the backstop is dead code unless the master
  SDO-writes 0x1016 at startup. Check whether the gateway's dcfgen concise
  DCF does this (fault-plan Phase 0b); if not, arm it via gateway/DCF
  config (or EDS default in the batched regen), then bench-test kill-
  gateway-mid-jog and set the stop deadline (heartbeat period × factor).
- **Q4 (OD contract): DIRECTION SET, naming open.** Remove 0x2300-0x2305 +
  0x2200 outright (no reserved stubs). OPEN: since 0x6042/0x6043 change
  SEMANTICS (ml/min → steps/s), decide whether to rename in place
  (TargetStepRate/ActualStepRate) or move to manufacturer-specific indices
  so no old MIK/gateway code silently misreads units. Batch with
  fault-feedback Phase 2 (0x2500-0x2503): one regen, both repos
  (firmware-production + CANOpenGateway `bridge/devices/`), bridge image
  rebuild, one revalidation.
- **Q5 (dose UX states): RESOLVED — MIK owns them.** DoseStatus semantics
  move upstairs; the old "false dose-ended monitored PumpState" driver bug
  class is structurally moot.
- **NEW — stale-ControlWord after dose completes: OPEN.** With TargetSteps
  consumed, a lingering 0x000F would start CONTINUOUS mode. Either keep a
  slim `just_completed` restart-block in firmware (status-quo semantics),
  or make MIK drop the enable bits as part of its dose-complete handling
  (simpler firmware, contract obligation upstairs). Decide with the MIK
  implementation.

## Sequencing

1. **Pump fault feedback Phase 0/1** (already NEXT in Context.md;
   firmware-only, independent of this plan).
2. **Dose strip + step-counter primitive + fault-feedback Phase 2 EDS
   objects as ONE combined OD change** (single Windows regen + both-repo
   propagation + bridge image rebuild + revalidation).
3. **Surviving audit-item-4 dedups** whenever convenient (TMC
   read/ReadDiag paths, 4× hybrid-stop sequence in stepper.c,
   pwm_control.arr_value mirror). The dose complete/timeout dedup is
   CANCELLED — that code is deleted by this plan, don't polish it.

## Validation checklist

- [ ] Continuous run/stop/quickstop regression (same bench sequence as the
      2026-08-11 audit validations), now commanded in steps/s
- [ ] Step-exactness: command TargetSteps=N, verify position delta == N
      (including decel ramp) at several rates
- [ ] MIK-driven dose end-to-end vs. catch test (accuracy limited only by
      MIK calibration, not step execution)
- [ ] Pause/resume mid-dose: StepsRemaining freezes and resumes; total
      still exactly N
- [ ] Abort mid-dose: remaining cleared, StepsRemaining report consistent
- [ ] Heartbeat-lost runaway test: kill gateway mid-JOG → pump stops within
      the Q3 deadline
- [ ] EDS: SDO read of removed/kept objects matches the Q4 contract;
      bridge shadow + MIK display correct (steps/s units!)
- [ ] valve/phtemp untouched (change confined to devices/pump + its EDS)
