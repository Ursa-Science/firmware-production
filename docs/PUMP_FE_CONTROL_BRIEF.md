# Pump Motor Control — Front-End Operations Brief

**For: MIK / front-end team.** How to drive the pump for continuous runs and
dosing, including arming, re-engaging, pause/resume, and what happens when the
master heartbeat is lost. All behavior below is validated on hardware.

For the object dictionary / PDO field layout and units, see
[`MIK_INTEGRATION_BRIEF.md`](MIK_INTEGRATION_BRIEF.md). This brief is the
**sequencing** contract.

---

## 0. The essentials (read first)

- **Pump node ID = 1.** RPDO1 = `0x201`, TPDO1 = `0x181`, TPDO2 = `0x281`.
- **SYNC is mandatory.** RPDO1/TPDO1/TPDO2 are synchronous — the pump applies a
  command and emits feedback **on each SYNC**, not on receipt. The master must
  produce SYNC continuously (gateway does 10 Hz).
- **RPDO1 = `ControlWord(2) + StepRate(2) + TargetSteps(4)`**, little-endian, 8 bytes.
  - `StepRate` — **signed** µsteps/sec. Sign = direction (**+ dose/forward, − draw/reverse**).
  - `TargetSteps` — µsteps to run then auto-stop. **0 = continuous.**
  - The MIK computes both from its own calibration:
    `rate_sps = ml_min × steps_per_ml[tubing] / 60`, `TargetSteps = ml × steps_per_ml[tubing]`.
- **The pump keeps NO fluid units.** It never sees ml, RPM, or tubing size.

### ControlWord values you will use

| CW | Meaning |
|---|---|
| `0x0000` | Disable — de-energize |
| `0x0007` | Enabled, stopped — driver energized, no motion (the "arm"/idle state) |
| `0x000F` | Run — operation enabled |
| `0x010F` | Halt / pause (run bits + halt bit 8) |
| `0x000B` | Quick stop — immediate stop + de-energize |

### Feedback to watch

- **TPDO2 `0x281`** = `PumpState(1) + ErrorReg(1) + StepsRemaining(4)`.
  PumpState: `0` INIT, `1` STOPPED, `2` STARTING, `3` RUNNING, `4` STOPPING,
  `5` FAULT, `6` HALT.
- **TPDO1 `0x181`** = `StatusWord(2) + ActualStepRate(2)`.

---

## ⭐ The one rule that governs all sequencing: a **new run needs a ControlWord edge**

RPDO1 is synchronous, so the pump re-applies the **last frame you sent on every
SYNC** — including `TargetSteps`. That means **`TargetSteps` alone cannot signal
"new command."** A lingering `0x000F` will **not** (re)start a run.

**To start OR restart any run — continuous or dosed — drive a ControlWord edge:**
hold `0x0007`, then send `0x000F`. The `0x0007 → 0x000F` transition is the "go."

After a **dosed** run auto-completes, the pump deliberately **stays stopped**
even though your `ControlWord` is still `0x000F` (a "run-consumed" latch). It
will not move again until you provide a fresh edge. This is by design — it
prevents a completed dose from silently re-triggering.

---

## 1. Continuous vs. dosing — the two modes

Both use RPDO1; the only difference is `TargetSteps`.

### Continuous (prime / purge / free-run)
`TargetSteps = 0`. Runs until you stop it (or the heartbeat backstop fires).

| Step | CtrlWord | RPDO1 (`0x201`, LE) | Result |
|---|---|---|---|
| Arm | `0x0007` | `07 00 <rate> 00 00 00 00` | Energized, stopped |
| Run | `0x000F` | `0F 00 <rate> 00 00 00 00` | Runs continuously at `rate` |
| Stop | `0x0007` | `07 00 …` | Graceful decel, stays energized |

`<rate>` = StepRate int16 LE. Example 800 steps/s = `20 03`.

### Dosing (counted, self-terminating)
`TargetSteps = N > 0`. Delivers **exactly N µsteps** then auto-stops — even if
the MIK dies mid-dose (self-terminating; there is no firmware timeout).

| Step | CtrlWord | RPDO1 | Result |
|---|---|---|---|
| Arm | `0x0007` | `07 00 <rate> 00 00 00 00` | Energized, stopped |
| Dose | `0x000F` | `0F 00 <rate> <N (4B LE)>` | Runs exactly N steps, then auto-stops |

Watch **TPDO2 `StepsRemaining`** count down to `0`; at `0`, PumpState → `1`
STOPPED. Delivered volume: `delivered_ml = (N − StepsRemaining) / steps_per_ml`.
**COMPLETE** = `StepsRemaining == 0` **and** StatusWord shows stopped.

> The pump stays energized (holding torque) after a completed dose — it is in
> "enabled, stopped," not disabled.

---

## 2. Arming, re-engaging a dose, re-engaging continuous after a dose

Every "start" is the same primitive: **`0x0007` → `0x000F`** (the edge), with
the RPDO1 payload carrying the rate and the `TargetSteps` for that run.

### Arm + run a dose
1. `0x0007` + rate + `TargetSteps=0` — arm (energize, stopped).
2. `0x000F` + rate + `TargetSteps=N` — dose starts.
3. Pump auto-stops at N; holds `run-consumed` (will not restart on the lingering `0x000F`).

### Re-engage another dose (after one completes)
The pump is sitting stopped, run-consumed, CtrlWord still `0x000F`. To dose again:
1. `0x0007` + rate + `TargetSteps=<new N>` — **the edge back to `0x0007` clears the latch.**
2. `0x000F` + rate + `TargetSteps=<new N>` — new dose starts.

> Sending a new `TargetSteps` **without** the CtrlWord edge does nothing —
> the edge is what re-arms.

### Re-engage a continuous run (after a dose completes)
Same edge, but `TargetSteps=0`:
1. `0x0007` + rate + `TargetSteps=0`
2. `0x000F` + rate + `TargetSteps=0` — continuous run starts.

> Because of the run-consumed latch, you **cannot** turn a just-completed dose
> into a continuous run by only zeroing `TargetSteps` — you must cycle the
> ControlWord. (This is the safety guard against an unbounded run after a dose.)

---

## 3. Halt / resume, and re-engaging afterward

**Pause** works for both continuous and dosed runs and preserves the remaining
count so a dose resumes exactly.

### Pause
- Send `0x010F` (Halt). Motor stops; **PumpState → `6` HALT.**
- `StepsRemaining` **freezes** at its current value (a dosed run holds its
  remaining count; a continuous run holds `0`).

### Resume
- Send `0x000F` (this `0x010F → 0x000F` transition is a valid edge).
- A **dosed** run **continues its held count** and finishes at exactly N total —
  you do **not** resend `TargetSteps`; the pump remembers it.
- A **continuous** run simply resumes.

### After a pause — what to send next
| You want | Send | Notes |
|---|---|---|
| Resume the same run/dose | `0x000F` | Continues held count / continuous run |
| Abort instead of resuming | `0x000B` (quick stop) or `0x0000` (disable) | **Clears the held count** — the paused dose is discarded |
| Start a *different* run after a completed one | `0x0007` → `0x000F` with new payload | Fresh edge, per §2 |

> **Pause precision:** exact at normal dosing rates (the motor stops instantly).
> At high step rates a paused/resumed dose can slightly over-deliver (a few
> ramp-down steps aren't re-counted). Keep dosing rates moderate if exactness
> across a pause matters.

---

## 4. Master heartbeat lost — the runaway backstop

The pump watches the **master's heartbeat** (node **127**, expected every
**1 s**). This is a safety backstop for a run that outlives its controller.

### Requirement on the master / MIK
- The master **must emit a node-127 heartbeat (COB-ID `0x77F`) at ~1 s** and
  **keep it alive the entire time it commands motion.**
- The pump only starts watching **after it has seen the first heartbeat**, so a
  boot with no master present will **not** false-trip.

### What happens if the heartbeat stops (> ~2.5 s gap) while running
1. Motor **e-stops and de-energizes** immediately.
2. Node drops to **PRE-OPERATIONAL** — the pump's own heartbeat `0x701` flips to
   `7F`, and **its TPDOs stop** (you lose feedback).
3. **Any in-flight dose is ABORTED**, not completed.

### Consequences of an aborted dose — what the MIK must handle
- **Delivered volume is partial and indeterminate from that point on.** Once the
  pump is in PRE-OP its TPDOs stop, so the last `StepsRemaining` you received
  before the drop is your best estimate of delivered volume
  (`delivered ≈ (N − last_StepsRemaining) / steps_per_ml`).
- The MIK should treat this as an **aborted dose**, surface it to the user, and
  **not** assume the target volume was delivered.

### Recovery
1. Restore the master heartbeat (bring the controller/link back).
2. **NMT-start** the pump (`0x000` → `01 01`) to return it to OPERATIONAL — it
   does not auto-recover to operational on its own.
3. Re-arm per §2 (`0x0007` → `0x000F`) to run again.

> **Scope:** a dose is already self-bounded (it stops at N on its own), so this
> backstop primarily protects **continuous** runs. The current policy is
> **abort-now** — a lost master stops everything, including a dose in progress.
> (A future option could let a bounded dose finish before stopping; not
> implemented today.)

---

## Quick reference — ControlWord transitions

```
DISABLED ──0x0007──► ENABLED/STOPPED ──0x000F──► RUNNING
   ▲                      ▲   │                     │
   │                      │   │◄──────0x0007─────────┘   (graceful stop)
   │                      │   └──0x000F+TargetSteps──► DOSING ──auto-stop──► ENABLED/STOPPED (run-consumed)
   │                      │
 0x0000                 0x010F ◄──► 0x000F   (pause / resume)
   │
   └──0x000B (quick stop) from anywhere ──► de-energized

Restart after a completed dose / to change mode: ALWAYS 0x0007 → 0x000F (edge).
```
