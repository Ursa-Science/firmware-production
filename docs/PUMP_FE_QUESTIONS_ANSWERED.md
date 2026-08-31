# Pump firmware — answers to the MIK/gateway integration questions

**From:** firmware
**Re:** your 8 open questions on the steps-only contract
**Date:** 2026-08-25
**Basis:** all answers are from the actual firmware in `firmware-production/devices/pump/`
(code file:line cited), not the briefs. Where the code contradicts a brief's wording, it's
flagged — that's where the conflict you hit comes from.

> **One correction up front, because it touches several answers:** `0x0080` is **not** a
> control word to send on its own. In `0x0080`, bit 2 (quick-stop, active-low) is 0, so it
> asserts a quick-stop at the same instant as the fault-reset. Fault reset must be sent as
> **`0x0084`** (bit 7 + bit 2 held high), preceded by `0x0004`. See Q2.

SYNC is 10 Hz (100 ms/period) in these examples.

> **Update (2026-08-25) — read this first:** the Q1 fix is now **implemented, shipped in
> FW 2.0.0, and hardware-validated** (it was "plan-stage" in the first draft). Q1, Q7, and Q8
> below reflect the as-shipped firmware. The OD **RevisionNumber is now 0x00020000** and the pump
> reports **FW 2.0.0** — gate on the revision (see Q8). **You do NOT need the interim MIK
> workaround that the earlier draft described.** (Line-number citations point at the pre-fix code
> where they explain the *original* behavior; the current line numbers shifted slightly with the
> fix.)

---

## BLOCKING

### Q1 — Does StepsRemaining survive a stop, and does it differ per stop word?

**FIXED and hardware-validated in FW 2.0.0 (OD revision 0x00020000).** Your concern was correct
about the *old* behavior — it's now resolved. `StepsRemaining` (0x2603) **latches the true
steps-not-delivered at the instant motion ends and holds it** until the next run arms. So
`delivered = (TargetSteps − StepsRemaining) / steps_per_ml` is now **exact and uniform** across
quick-stop, disable, pause, and natural completion. **No special-casing and no interim workaround
needed on your side.**

**Current behavior (FW ≥ 2.0.0):**

| Stop word | 0x2603 after stop | `(N − 0x2603)/spml` |
|---|---|---|
| **0x0007** (decel-stop / pause) | held at remaining-at-pause | correct remaining ✓ |
| **0x000B** (quick stop) | **held at remaining-at-abort** | correct delivered ✓ |
| **0x0000** (disable) | **held at remaining-at-abort** | correct delivered ✓ |
| natural completion | 0 (counted down) | full N ✓ |

**Validated on the bench (CAN trace, 2026-08-25):** a 16000-step dose quick-stopped mid-run
reported StepsRemaining **10776** and **11763** (not 0) → delivered computed correctly as the
partial amount. Before the fix, quick-stop/disable zeroed the counter in the same cycle → the
master read 0 → a partial dose looked fully delivered. **That failure mode is gone.**

**How to detect it's the fixed firmware:** gate on the OD RevisionNumber (0x1018sub3) —
**revision ≥ 0x00020000 ⇒ StepsRemaining is honest at abort** (see Q8). Older builds report
0x00010001 and still have the zero-on-abort behavior.

(Background — the *original* bug, for the record: 0x2603 was rewritten every cycle from the live
stepper counter, which `Stepper_Stop()`/`Stepper_Disable()` zero on abort, so a quick-stop/disable
published 0 the same cycle. The fix latches the true remaining before the counter is zeroed and
holds it while stopped. Plan + implementation notes: `PUMP_STEPSREMAINING_FIDELITY_PLAN.md`.)

**One remaining nuance — pause only (see Q7):** during a pause the *reported* value is honest, but
the motor can still physically **over-deliver by a few ramp-down steps on resume at high step
rates**. That's a separate, still-gated firmware item (Phase B) and does **not** affect
abort/disable/completion accuracy.

### Q2 — Producing the bit-7 fault-reset edge on a cyclic RPDO

Fault reset is **edge-triggered**: `rising_edges = (control_word ^ last_control_word) &
control_word`, evaluated once per cycle (`motor_control.c:886`); `HandleFaultReset` fires only on
the 0→1 transition of bit 7 (`:539`). A synchronous RPDO re-latches the same payload every SYNC,
so **holding the bit high does nothing after the first cycle**.

- **Sequence:** `0x0004` (bit 7 low, bit 2 high) → `0x0084` (bit 7 high, bit 2 high). **One SYNC
  frame with the raised bit registers it** — you need the transition, not a hold duration. Then
  return to `0x0004` so the next fault is resettable.
- **Keep bit 2 high throughout** (`0x04`/`0x84`, never bare `0x80`), or you fire a quick-stop
  falling edge at the same time.
- **State after reset: DISABLED / de-energized.** `HandleFaultReset` calls `Stepper_Disable()`
  and sets `current_state = DISABLED` (`:551-554`). Expected post-reset telemetry: StatusWord
  `0x0240`, PumpState 1 (STOPPED), ErrorRegister cleared, plus an EMCY `0x0000` recovery frame.
  You must **re-arm** (`0x0007` → `0x000F`) to run again. If you ever see StatusWord `0x0250`
  (bit 4 still set), that's a stale build — the de-energize fix isn't flashed.

### Q3 — FaultSource (0x2503), EMCY codes, DRV_STATUS bits

Authoritative from `motor_control.c` `MotorControl_EnterFault` (`:210-258`) and the enum
(`:106-112`):

**FaultSource `0x2503`** — decode by this; it's more specific than the EMCY code:

| Value | Name | Trigger | EMCY code | ErrorReg bit |
|---|---|---|---|---|
| 0x00 | NONE | no fault / after reset | 0x0000 | cleared |
| 0x01 | DIAG_SHORT | DRV_STATUS S2GA/S2GB/S2VSA/S2VSB | **0x2310** current | current |
| 0x02 | DIAG_OT | DRV_STATUS OT/OTPW | **0x4210** temperature | temperature |
| 0x03 | STEPPER | stepper-layer error (position runaway) | **0x7121** motor blocked | generic |
| 0x04 | TMC_INIT | *reserved — Phase 2, not emitted yet* | — | — |
| 0x05 | TMC_COMM | DIAG latched but DRV_STATUS unreadable (UART fail) | **0x1000** generic | +communication |
| 0x06 | DIAG_UNKNOWN | DIAG latched, no cause flag set | **0x1000** generic | generic |

Note 0x1000 covers both 0x05 and 0x06, so use FaultSource to disambiguate. **0x04 never appears
yet** (reserved for boot-time init failure in Phase 2).

**EMCY payload layout** (`MCOP_PushEMCY`, `:257`): bytes 0-1 = code (LE), byte 2 = ErrorRegister,
bytes 3-6 = DRV_STATUS snapshot (32-bit LE), byte 7 = FaultSource. Example short-fault frame:
`10 23 <ER> <DRV_STATUS LE> 01`.

**DRV_STATUS bits for LastDrvStatus `0x2502`** (`tmc2209_uart.h:169-183`; full map = TMC2209
datasheet DRV_STATUS 0x6F):

| Bit | Flag | | Bit | Flag |
|---|---|---|---|---|
| 0 | OTPW (overtemp pre-warning) | | 4 | S2VSA (short to supply A) |
| 1 | OT (overtemp shutdown) | | 5 | S2VSB (short to supply B) |
| 2 | S2GA (short to GND A) | | 31 | STST (standstill) |
| 3 | S2GB (short to GND B) | | | |

Also SDO-only: 0x2500 = TMC init status, 0x2501 = TMC init failed-step. Safe to decode now.

---

## NEEDED, NOT BLOCKING

### Q4 — Master-heartbeat watchdog: exact/fixed? Is 1 s required?

**The pump imposes no fixed watchdog — the master provisions it.** The consumer object `0x1016`
is **empty at boot** (all subs blank in the EDS; consumer unarmed), so nothing is monitored until
the master SDO-writes `0x1016` sub with `(node_id << 16 | timeout_ms)` — and that write is
**volatile, lost on every pump reset**, so re-arm on each connect. The "~2.5 s" in the brief is
just the value we suggest you write, not a firmware constant.

This inverts your worry: a 3 s master period only trips the watchdog if *you* wrote a timeout
shorter than 3 s. Since the master sets both, the rule is simply **timeout > your own producer
period** (e.g. produce every 1 s, write timeout 2500 ms). The pump's *own* producer heartbeat is
`0x1017 = 1000` (1 s) by default. So 1 s is a recommendation; the requirement is master-period <
the timeout you arm in 0x1016.

### Q5 — Arm dwell: is one SYNC of 0x0007 enough before 0x000F?

**No minimum hold time exists in firmware.** Arm→run is state-based, not timed: `0x0007` moves the
pump to ENABLED_STOPPED in one cycle (`DetermineTargetState`, `:665-691`), then `0x000F`
transitions to RUNNING on the next cycle. One SYNC of `0x0007` suffices; your 2-SYNC / 200 ms
hold is safe and conservative. Only deferral: if you arm immediately after a prior move that's
**still ramping down**, RUNNING waits until the stepper actually stops (`CheckTransitionBlocking`,
`:708`) — a wait-until-stopped, not a fixed dwell.

### Q6 — Withdraw: negative StepRate supported? Reverse limits?

**Supported and symmetric.** Direction = sign of StepRate (`:816-820`); the same speed ceiling
(120 RPM ≈ 3200 µsteps/s) applies to the magnitude regardless of sign. **No reverse-specific
duty-cycle or duration limit exists in firmware** — reverse runs exactly like forward, and the
TMC's thermal protection (overtemp → DIAG → FAULT 0x4210) applies equally both directions. Any
reverse-run mechanical/tubing limit is a MIK/hardware policy, not enforced by firmware.

### Q7 — Pause over-delivery: how many steps, above what rate?

**Root cause (confirmed in the ISR):** `steps_remaining` is only decremented while
`step_target_active` is true (`stepper.c:554`), but a pause (`Stepper_NormalStop`) clears that
flag at decel-*start* (`:304`). So the decel steps during the pause are delivered but never
subtracted, and resume replays the decel-start count → over-delivery ≈ the decel-ramp step count.
Below the low-speed threshold (`LOW_SPEED_ARR_THRESHOLD=700`) the stop is a deferred instant stop
(`:310-321`) → ~0 extra steps (why it's "exact at normal rates"). Above it, the overshoot grows
with the square of speed. The normal exact-cutoff completion keeps the flag set through decel
(`:576-583`), which is why a non-paused dose is exact.

The docs give no number because it's ramp-dependent. **Current status (FW 2.0.0):** the Q1 fix
(Phase A) makes the **reported** 0x2603 honest during a pause, but the **physical** over-delivery
on resume at high rate is a separate item (**Phase B — not yet implemented**, gated on a bench
measurement of the actual overshoot; it touches the validated decel/ISR path). Net for you today:
- Abort / disable / natural completion: **exact** (Q1 fix).
- Paused-then-resumed dose at high step rate: may over-deliver by a few ramp-down steps (~0 below
  the low-speed instant-stop threshold). Keep dosing rates moderate if pause-exactness matters, or
  tell us and we'll either quantify the exact bound or schedule Phase B.

### Q8 — Microstepping and revision

**Confirmed 1/8:** `TMC_MICROSTEP = 8` (`tmc2209_uart.h:134`), pushed into CHOPCONF.MRES over
UART. Firmware's mechanical constant is 200 full-steps/rev × 8 = **1600 µsteps/motor-rev**. The
**~1440 steps/mL is a MIK calibration number** (pump-head displacement per rev), not a firmware
value — the firmware is deliberately tubing- and mL-agnostic. Both your asks are adopted:

- **We will notify before microstep changes.** `TMC_MICROSTEP` is treated as a contract constant;
  any change invalidates every `steps_per_ml` and will be flagged as a breaking change.
- **RevisionNumber (`0x1018sub3`) is now bumped — DONE.** It went `0x00010001 → 0x00020000` for the
  steps-only OD generation (both `[DeviceInfo] RevisionNumber` and `0x1018sub3`), the MicroCANopen
  sources were regenerated, and the pump reports **FW 2.0.0**. This directly fixes your
  silent-failure case (old firmware byte-identical on 0x1018/TPDO1, reporting mL/min into the
  ActualStepRate slot): the MIK can now **gate on `0x1018sub3 ≥ 0x00020000`** and refuse/adapt for
  an older build instead of turning stale data into a plausible wrong number.

  **Action on your side:** the gateway keeps its own copies of the device EDS/DCF in
  `bridge/devices/` (baked into the bridge image). Copy the regenerated
  `PumpModule-n01-250kbs.{eds,dcf}` (revision 0x00020000) into `bridge/devices/` and rebuild the
  bridge image, or the master will still be decoding against the old revision.

---

## Summary of actions

| # | Item | Resolution |
|---|---|---|
| Q1 | StepsRemaining zeroed on abort | **FIXED + hardware-validated in FW 2.0.0.** 0x2603 now holds true remaining at abort/disable/pause. No MIK workaround needed. Gate on revision ≥ 0x00020000. |
| Q2 | Fault-reset edge | Use `0x0004`→`0x0084`, one SYNC, re-arm after; drive is DISABLED post-reset. (Not yet HW-tested — no fault injected in the trace.) |
| Q3 | Fault decode | Tables above are firmware-authoritative — decode away. |
| Q4 | Heartbeat watchdog | Master-provisioned via 0x1016 (volatile, re-arm on connect); timeout > your HB period. |
| Q5 | Arm dwell | No minimum; one SYNC of 0x0007 is enough. |
| Q6 | Withdraw | Symmetric to forward; no reverse-specific limits. |
| Q7 | Pause over-deliver | Reported value now honest (Q1 fix). Physical over-deliver on resume at high rate = Phase B, **not yet done** (gated). Abort/completion exact. |
| Q8 | Microstep / revision | 1/8 confirmed; RevisionNumber bumped to 0x00020000 (**DONE**, FW 2.0.0). **FE action:** copy regenerated EDS/DCF into gateway `bridge/devices/` + rebuild image. |
