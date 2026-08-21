# Pump ↔ MIK Integration Brief — "Dumb Pump" Step-Counter Change

**For: MIK / front-end team.** The pump firmware is dropping all fluid math.
It now speaks only **steps** and **steps/sec**. The MIK owns every ml
calculation and all calibration (including tubing size). This brief covers
only what changes on the MIK side.

Indices below are the planned contract; final numbers are confirmed at the
CANopen EDS regen but the types/semantics are locked.

---

## EDS changes

### Deleted — MIK must STOP using these
| Index | Name | Why gone |
|---|---|---|
| 0x6042 | TargetFlowRate (ml/min) | replaced by StepRate |
| 0x6043 | ActualFlowRate (ml/min) | replaced by ActualStepRate |
| 0x2200 | FlowCorrectionFactor | calibration now MIK-side only |
| 0x2300 | DoseVolume | MIK sends steps, not ml |
| 0x2301 | DoseFlowRate | " |
| 0x2302 | DoseCommand | dead |
| 0x2303 | DoseStatus | MIK owns dose state now |
| 0x2304 | DoseDelivered | replaced by StepsRemaining |
| 0x2305 | DoseTimeout | no firmware timeout (self-terminating) |
| 0x2100 | MaxFlowRate (ml/min) | unused; limits are MIK-side |

### Added / new — MIK adopts these
| Index | Name | Type | Dir | Units |
|---|---|---|---|---|
| 0x2600 | StepRate | int16 | MIK → pump | signed µsteps/sec (sign = direction) |
| 0x2601 | TargetSteps | uint32 | MIK → pump | µsteps to run then auto-stop; 0 = continuous |
| 0x2602 | ActualStepRate | int16 | pump → MIK | measured signed µsteps/sec |
| 0x2603 | StepsRemaining | uint32 | pump → MIK | countdown to auto-stop |
| 0x2500–0x2503 | TMC fault detail | u8/u32 | pump → MIK (SDO) | init status, failed step, DRV_STATUS, fault source |

### Unchanged (still CiA-402)
0x6040 ControlWord, 0x6041 StatusWord, 0x2400 PumpState, 0x1001 ErrorRegister.

---

## PDO layout (what MIK sends / receives)

**RPDO1 — MIK → pump (COB-ID 0x200+node, 8 bytes, one atomic frame):**
`ControlWord(2) + StepRate(2) + TargetSteps(4)`

**TPDO1 — pump → MIK (0x180+node, 4 bytes):**
`StatusWord(2) + ActualStepRate(2)`

**TPDO2 — pump → MIK (0x280+node, 6 bytes):**
`PumpState(1) + ErrorRegister(1) + StepsRemaining(4)`

ControlWord bits are standard CiA-402, unchanged: `0x000F` run, `0x0007`
decel-stop, bit2 clear = quick-stop, `0x0000` disable, bit7 = fault reset,
bit8 (`0x010F`) = halt/pause. PumpState enum: 0 INIT, 1 STOPPED, 2 STARTING,
3 RUNNING, 4 STOPPING, 5 FAULT, 6 HALT.

---

## MIK calculations

MIK keeps one calibration number per tubing size, `steps_per_ml[tubing]`,
which absorbs tubing displacement AND the pump's microstep resolution.

**Calibration (catch test, per tubing size):**
```
1. Command a known step count:  TargetSteps = N   (e.g. ~10 ml worth on the current estimate)
2. User measures actual delivered volume:  measured_ml
3. steps_per_ml[tubing] = N / measured_ml
   (repeat to converge; store per tubing size)
```

**Dose / draw:**
```
TargetSteps = round( volume_ml * steps_per_ml[tubing] )
StepRate    = round( ±rate_ml_min * steps_per_ml[tubing] / 60 )   // + = dispense, − = withdraw
→ send one RPDO1: ControlWord=0x000F, StepRate, TargetSteps
```

**Prime / purge (continuous):** `TargetSteps = 0`, `StepRate = ±rate`.
Runs until MIK stops it (`0x0007` or quick-stop).

**Progress / display (from TPDO2):**
```
delivered_ml = (TargetSteps − StepsRemaining) / steps_per_ml[tubing]
```

**Constraints:**
- `|StepRate|` must stay within the pump's speed ceiling (~3200 µsteps/sec at
  the current 1/8 microstep). Firmware clamps, but MIK should not exceed.
- `steps_per_ml` is valid for the firmware's current microstep setting. If
  firmware microstepping ever changes, all tubing sizes must be re-calibrated.

_Example (1/8 µstep → ~1440 steps/ml): a 10 ml dose at 50 ml/min →
TargetSteps = 14400, StepRate = 1200. Pump runs ~12 s and stops itself._

---

## Behaviors the MIK must handle

- **Completion / "all done":** not a pushed message — MIK infers it from
  telemetry: **`StepsRemaining == 0` AND pump stopped** (PumpState=STOPPED /
  ActualStepRate=0). `StepsRemaining` is latched (holds 0), so it can't be
  missed between SYNC frames.
- **Completed vs aborted:** stopped with `StepsRemaining == 0` = full delivery;
  stopped with `StepsRemaining > 0` = stopped early, and delivered volume =
  `(TargetSteps − StepsRemaining) / steps_per_ml`.
- **Fresh command required after a dose:** once a counted run auto-stops, the
  pump will NOT restart on a lingering enable ControlWord — MIK must issue a
  new command (new TargetSteps / ControlWord edge) to run again.
- **Pause / resume:** halt bit (`0x010F`) freezes `StepsRemaining`; clearing it
  resumes the same remaining count at the same StepRate.
- **Faults:** cause-coded EMCY frames + `PumpState=FAULT(5)` + `ErrorRegister`
  bits + StatusWord fault pattern. MIK stops the workflow and may SDO-read
  0x2500–0x2503 for TMC diagnostic detail. Recovery: ControlWord fault-reset
  bit; EMCY 0x0000 confirms.
