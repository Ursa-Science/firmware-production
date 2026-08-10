# Pump Fault Feedback over CAN — Planning Document

Status: **PLANNING.** Research done 2026-08-10; corrected same day for the
refactored master (CANOpenGateway). No implementation started.
Goal: the master must learn — promptly and with cause — when the pump's
TMC2209/DIAG fault path fires, via PDO (and EMCY where it fits).

## The master (refactored 2026: CANOpenGateway)

The python-canopen MIK driver is GONE. The master is now
**`~/code/CANOpenGateway`**: a C++ Lely-coapp daemon (`bridge/master-bridge.cpp`,
node 127) owning the bus, exposed through a TypeScript Fastify API
(REST + SSE + MCP, port 8080). Facts that shape this plan:

- **EMCY is a first-class citizen already.** The bridge consumes slave EMCYs;
  the API keeps a per-device `emcyHistory` (last 100, `{eec, er, msef[]}`
  decorated with `eecText`/`erFlags`), persists them to the event/alarm log,
  and delivers `emcy` frames over SSE at HIGH priority — flushed AHEAD of
  batched telemetry, never dropped under back-pressure. Firmware-side EMCY
  (Phase 1) lands in ready-made plumbing with zero gateway changes.
- **TPDOs land in a device shadow** under the slave's own object indices
  (`dcfgen -r` remote PDO mapping): PumpState 0x2400 / ErrorRegister 0x1001 /
  StatusWord 0x6041 are already queryable via `GET /v1/devices/2` and
  persisted as on-change telemetry.
- **SYNC**: gateway produces it, 10 Hz default — but runtime-configurable via
  `POST /v1/master/sync` including 0 = OFF. Sync-type-1 TPDO fault visibility
  is therefore operator-disableable; EMCY (SYNC-independent) closes that hole.
- **EDS distribution**: the gateway keeps its own copies —
  `bridge/devices/PumpModule-n02-250kbs.{eds,dcf}` — compiled into
  `master.dcf` at startup (gen-network.py + dcfgen) and BAKED INTO the bridge
  Docker image. Any EDS change must be copied there too and the bridge image
  rebuilt (native: `cd bridge && make` + restart).
- API decoration/self-documentation derives from the EDS, so new 0x25xx
  objects become named, discoverable API objects automatically.

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
| No event-driven PDO | Both TPDOs are sync-type-1. 100 ms latency is fine at the gateway's default 10 Hz SYNC — but SYNC is runtime-disableable (`POST /v1/master/sync {periodUs:0}`), and fault visibility dies with it. EMCY closes this hole. |
| No application-layer reaction | The gateway already ingests, decorates, persists, and streams both the TPDO fault fields and EMCYs — but it is deliberately policy-free. Whatever consumes its REST/SSE (MIK web app) must react to `emcy` events / PumpState=FAULT (stop workflow, raise alarm). |

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

### Phase 0 — verify the existing path end-to-end (bench, no firmware code)
- Induce a fault safely (force `fault_active` via debugger — do NOT short
  motor phases) and confirm through the gateway:
  - `GET /v1/devices/2` shadow shows PumpState=FAULT (0x2400) and
    ErrorRegister=0x01 (0x1001) within one SYNC (100 ms @ default 10 Hz);
  - `/v1/events` SSE emits the value changes; `/v1/devices/2/telemetry`
    persists the transition (on-change);
  - candump as ground truth if desired.
  (The gateway's MCP endpoint — `claude mcp add --transport http gateway
  http://localhost:8080/mcp` — makes this scriptable/agent-drivable.)
- Exit criterion: documented trace of a fault reaching the gateway shadow,
  SSE stream, and persisted event history.

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
  8-byte payload has room for: DRV_STATUS snapshot (4B in msef) + fault source
  enum (1B). Event-driven, SYNC-independent — and the gateway already
  ingests it: `emcyHistory` in the device shadow, decorated `eecText`/
  `erFlags`, persisted alarm log, HIGH-priority SSE delivery. Zero gateway
  changes needed.
- Blast radius: `devices/pump/Core` only. Re-validate pump on bench,
  verifying through the gateway (Phase 0 method) that the EMCY appears with
  correct eec/er/msef decoration.

### Phase 2 — EDS extension for on-demand detail (needs CANopen Architect / Windows)
- Add manufacturer objects (regenerate EDS per repo workflow — tool output,
  copied into `devices/pump/MCO_CiA401__User/EDS/`, never hand-edited):
  - 0x2500 u8  TMC init status (tmc_status)
  - 0x2501 u8  TMC init failed step
  - 0x2502 u32 last DRV_STATUS snapshot (at fault, or periodic)
  - 0x2503 u8  fault source (enum: none/diag-short/diag-ot/stepper/tmc-init/tmc-comm)
- Reinstate the Phase-1-deleted status writes against the REAL offsets.
- TPDO2 has 2 spare bytes: map 0x2503 (fault source) + 0x2502 low byte, or
  keep TPDO2 as-is and leave 0x25xx for SDO-on-demand after an EMCY.
  **Recommendation: SDO-on-demand** — the gateway makes it trivial
  (`POST /v1/devices/2/sdo/read` accepts `{"name":"..."}` lookups from the
  EDS) and keeps the PDO layout stable.
- **EDS propagation (two repos + image rebuild):** regenerated
  `PumpModule-n02-250kbs.{eds,dcf}` must be copied BOTH into this repo's
  `devices/pump/MCO_CiA401__User/EDS/` AND into CANOpenGateway
  `bridge/devices/` (same filename stem), then the bridge Docker image
  rebuilt (device catalog is baked in; native = `cd bridge && make` +
  restart). dcfgen recompiles `master.dcf` at startup; the new 0x25xx
  objects self-document in the API from the EDS.
- Re-flash + re-validate pump (SDO/PDO layout changed).

### Phase 3 — application-layer handling (MIK web app, out of scope here)
- The gateway itself needs NO changes — ingestion, decoration, persistence,
  and alarm-first SSE ordering are already built.
- The consuming application (MIK front end / workflow layer) must: subscribe
  `/v1/events` SSE; on `emcy` (node 2) or PumpState→FAULT, stop the dosing
  workflow and surface the alarm; optionally SDO-read 0x2500-0x2503 through
  the gateway for diagnosis detail.
- Define recovery contract: CW RESET bit via `POST /v1/devices/2/pdo/write`
  (already implemented FW-side) clears `fault_active`; EMCY 0x0000 confirms
  recovery; shadow PumpState leaving FAULT is the app-visible signal.

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
1. SYNC dependence: gateway default is 10 Hz but `POST /v1/master/sync
   {periodUs:0}` disables it at runtime — until Phase 1 EMCY lands, that
   also silently disables fault visibility. Should the app layer guard
   against disabling SYNC while a pump is enabled?
2. Confirm final answer on fault detail transport: TPDO2 spare bytes vs
   SDO-on-demand after EMCY (plan recommends SDO-on-demand; gateway named
   SDO reads make it cheap).
3. Should valve/phtemp adopt the same EMCY pattern later? (Same stack;
   check USE_EMCY per device; their EDS/DCF pairs are already in the
   gateway's `bridge/devices/`. Separate plan if so.)
4. Does `dcfgen`/the bridge boot-time config (concise DCF slave downloads)
   need the new 0x25xx objects declared writable/readable in any particular
   way? Verify after the first Phase-2 regen that `gen-network.py` +
   `dcfgen` accept the extended EDS cleanly.
