# Pump firmware — StepsRemaining fidelity at stop (Q1 + Q7)

**Target:** `firmware-production/devices/pump/`
**Date:** 2026-08-25
**Status:** **Phase A IMPLEMENTED 2026-08-25** (not yet built/flashed — user builds manually).
**Phase B still gated** (measure-first). Raised by FE integration questions
(see `PUMP_FE_QUESTIONS_ANSWERED.md`, Q1 and Q7).

> **Implementation note (Phase A, `Core/Src/motor_control.c`):** added
> `motor_ctrl.reported_steps_remaining`; 0x2603 now publishes it. It tracks the live stepper
> counter while RUNNING and is latched to the true steps-not-delivered at every terminal path,
> captured BEFORE the stepper zeroes:
> - struct field (~L157); init/NMT-reset → 0 (~L381, L508)
> - quick-stop capture (~L613); DISABLED-entry capture (~L820); fault capture in `EnterFault`
>   (~L261); e-stop/heartbeat-loss capture in `EmergencyStop` (~L476)
> - natural completion → 0 (TARGET_REACHED, ~L315); fault-reset → 0 (~L587)
> - pause holds the paused count (~L783); RUNNING entry seeds the move length (~L894)
> - publish logic: track live while RUNNING else hold (~L1110-1112)
>
> Phase B (pause physical over-delivery, Q7) is intentionally NOT implemented — see §3, it needs
> the bench measurement gate and touches the validated decel/ISR path. Phase A does not and cannot
> fix Q7 (the over-delivery is physical, on resume); it only makes the *reported* value honest.

---

## 1. Problem

The MIK derives delivered volume as `delivered = (TargetSteps − StepsRemaining) / steps_per_ml`.
That formula is only correct if `StepsRemaining` (OD 0x2603, TPDO2) equals the steps **not yet
delivered** at the moment motion ended. Today it does not, in two cases:

**(Q1) Abort zeroes the counter.** 0x2603 is rewritten every process cycle from the live stepper
counter (`motor_control.c:1062`). Quick-stop (0x000B) and disable (0x0000) call
`Stepper_Stop()`/`Stepper_Disable()`, which set `steps_remaining = 0` (`stepper.c:234, 267`), and
that happens in `ProcessControlWord` **before** `UpdateProcessImage` in the same cycle. So the
first TPDO2 after an abort already carries 0 → the MIK computes `(N − 0)/spml = N` = **full volume
for a half-delivered dose.**

**(Q7) Pause orphans the decel steps.** `steps_remaining` is only decremented while
`step_target_active` is true (`stepper.c:554`). A pause (`Stepper_NormalStop`) clears that flag at
decel-*start* (`:304`), so the steps pulsed during the pause decel are delivered but never
subtracted. On resume the firmware replays the decel-start count (`paused_steps_remaining`,
captured at `motor_control.c:750`) → net **over-delivery ≈ the decel-ramp step count** (≈0 below
`LOW_SPEED_ARR_THRESHOLD=700`, growing with speed² above it).

Normal exact-cutoff completion is already correct: the ISR keeps `step_target_active` true through
its own decel (`:576-583`), counting down to exactly 0.

### Current behavior matrix

| End of motion | 0x2603 reported | Correct? |
|---|---|---|
| natural completion (TARGET_REACHED) | 0 | ✓ (full delivered) |
| 0x0007 pause | frozen at decel-start | slightly high (understates delivered) |
| 0x000B quick stop | 0 | ✗ (reports full) |
| 0x0000 disable | 0 | ✗ (reports full) |

---

## 2. Goal

`0x2603` reports true steps-not-delivered at the instant motion ends, and **holds** that value
until the next run arms — so `(N − 0x2603)/spml` is exact and uniform for completion, pause, and
abort. No MIK-side special-casing.

---

## 3. Approach

Two independent pieces. Phase A (abort fidelity) is small, low-risk, high-value — do first.
Phase B (pause exactness) is deeper (ISR/ramp interaction) — measure the bound first, then decide.

### Phase A — hold true remaining on abort/stop (fixes Q1)

Decouple the reported OD value from the live (zeroed) stepper counter.

1. Add `uint32_t reported_steps_remaining;` to the `motor_ctrl` struct.
2. In each terminal path, capture the count **before** the stepper zeroes it:
   - `HandleQuickStop` (`:581`) — `reported = Stepper_GetStepsRemaining()` before
     `Stepper_Stop()/Stepper_Disable()`.
   - DISABLED entry in `ExecuteEntryActions` (`:781-794`) — capture before `Stepper_Stop()`.
   - `MotorControl_EnterFault` / DIAG-fault path (`:210`, `:325-326`) — capture before stop.
   - Natural completion (STEPPER_EVT_TARGET_REACHED, `:296-298`) — set `reported = 0`.
   - Fresh RUN arm (RUNNING entry with a new TargetSteps, `:839-846`) — set `reported = steps`
     (the just-latched target) so a new dose starts from full.
3. In `MotorControl_UpdateProcessImage` (`:1062`): while `current_state == RUNNING`, track
   `reported = Stepper_GetStepsRemaining()` and publish it; otherwise publish the **held**
   `reported` value (do not overwrite from the zeroed live counter).

Net: 0x2603 tracks live during a run, then latches the true remaining at whatever stopped it.

### Phase B — make pause exact (fixes Q7) — measure first

Option B1 (recommended if we act): in the pause path, keep counting through the decel so the
resume count and 0x2603 reflect steps actually pulsed. Concretely, let the pause decel run with
the step target still active (or capture `paused_steps_remaining` at the STOPPED event instead of
at decel-start `:750`), so the decel steps are subtracted. Requires a small "pausing" distinction
so `Stepper_NormalStop` doesn't clear `step_target_active` early for the pause case only — must not
disturb the validated normal completion path (`:576-583`) or the generic `NormalStop` used
elsewhere.

Option B2 (accept + document): leave the mechanism; publish the exact over-delivery bound as a
function of step rate (derived from `DECEL_RATE_RPM_PER_SEC=80`, `STEPPER_STEPS_PER_REV=200`,
`TMC_MICROSTEP=8`, and the `current_arr*3` decel target), and have the MIK keep dosing rates below
a documented threshold when pause-exactness matters.

**Decision gate:** bench-measure the actual over-delivery at 50 / 100% of ceiling before choosing
B1 vs B2. Per the "don't over-polish firmware dose code" guidance, if the real-world overshoot is
small at operational rates, B2 (document the bound) may be sufficient and B1 is deferred.

---

## 4. Files touched (Phase A)

- `Core/Src/motor_control.c` — struct field; captures in `HandleQuickStop`, `ExecuteEntryActions`
  (DISABLED + RUNNING), `MotorControl_EnterFault`, the TARGET_REACHED handler; publish logic in
  `MotorControl_UpdateProcessImage`.
- `Core/Src/motor_control.c` init (`MotorControl_Init` `:341`, NMT reset `:461`) — initialize/reset
  `reported_steps_remaining = 0`.
- No OD/EDS change (0x2603 already exists and is TPDO2-mapped). No stepper.c change in Phase A.

Phase B additionally touches `Core/Src/stepper.c` (pause decel / step_target handling) and the
pause capture in `motor_control.c:750` — scope confirmed at the decision gate.

---

## 5. Test plan (bench, with an LSS/OD-capable master)

For each stop word, run a known dose and abort partway, then read 0x2603 and compute delivered:

1. Arm + run a dose of N steps; at ~50% (verify via live 0x2603 countdown) send:
   - **0x0007** → expect 0x2603 ≈ remaining; resume → total = exactly N (Phase B target).
   - **0x000B** → expect 0x2603 = remaining-at-abort (Phase A); `(N − 0x2603)/spml` = the ~50%
     actually delivered (NOT full N).
   - **0x0000** → same as 0x000B.
2. Let a dose complete naturally → 0x2603 = 0, delivered = N. (regression: unchanged)
3. Confirm a fresh dose after any of the above starts from full N (reported reset on arm).
4. Regression: continuous jog (TargetSteps=0) still reports 0x2603 = 0 throughout.
5. Phase B: measure over-delivery on a paused/resumed dose at high rate to size B1 vs B2.

---

## 6. Risks / notes

- **Ordering:** the capture must precede `Stepper_Stop()/Stepper_Disable()` in every abort path —
  the whole bug is that those zero the counter. Easy to miss one path (fault/DIAG path especially).
- **Continuous jog:** `steps_remaining==0` for a jog; the held value must stay 0, not latch a stale
  dose remainder. Reset-on-arm covers this.
- **NMT reset / heartbeat-loss abort:** `MotorControl_EmergencyStop` (`:448`) also stops+disables;
  decide whether it should latch remaining too (probably yes, for the heartbeat-loss "best
  estimate" case the FE relies on).
- **Phase B must not touch the validated exact-cutoff completion path** (`stepper.c:576-583`) — that
  path is correct today; only the external-pause path clears the flag early.
- Plan-stage only; user builds/flashes manually. Bump `0x1018sub3` if this ships as part of the
  steps-only OD generation (see FE Q8).
