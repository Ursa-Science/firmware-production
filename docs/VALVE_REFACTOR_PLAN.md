# Valve Dumb-Module Refactor — RTT logging · minimal EDS · heartbeat-loss close + EMCY

**Status: PLAN reviewed 2026-09-30 (audit verified against source; D3/D4
reconciled, 0x1016 SDO-disarm risk added as D9, object count corrected to 21);
Phases 1–2 implementation started same day.** Third application of the
pattern proven on the pump (`PUMP_DOSE_TRANSFER_PLAN.md`, `PUMP_FAULT_FEEDBACK_PLAN.md`)
and phtemp (`PHTEMP_REFACTOR_PLAN.md`, `PHTEMP_EDS_REGEN_CHANGELIST.md`,
`PHTEMP_ARCHITECT_REGEN_2_THRESHOLDS.md`). Device: `devices/valve`, node 9,
product code 5, target `valve-n09`. Decisions marked **[D#]** need Dakota's
yes/no before the step that depends on them; each has a recommendation.

Three goals:

1. Retire USART2 ring-buffer logging → SEGGER RTT over SWD (valve is the last device on UART).
2. Cut the valve EDS to objects the module actually needs.
3. Enable EMCY and **close the valve when the MIK heartbeat is lost** (today nothing arms a consumer, so nothing happens).

---

## 1. Audit — what the valve does today (read from source 2026-09-30)

### 1.1 Heartbeat loss does nothing (goal 3 is greenfield)
- **No consumer is armed.** `grep MCOP_InitHBConsumer devices/valve` = no hits;
  `MCOUSER_ResetCommunication` (`MCO_Target/user_STM32.c:132`) only calls
  `MCO_DefaultResetCommunication` + the TPDO2 re-assert. The EDS 0x1016:01
  default is empty, and nothing writes the slave's 0x1016 (the gateway's
  `heartbeat_consumer` makes the *master* watch the slave — the same finding
  that drove phtemp's 2026-09-30 fix). So `MCOUSER_HeartbeatLost` never fires.
- **Fail-safe is configurable and defaults to "leave it".** `ValveControl_ApplyFailSafe()`
  reads OD 0x2100; EDS default = 0 = `FAILSAFE_AS_IS` → **an open valve stays
  open** on HB loss, PRE-OP, NMT stop, or IWDG reset. The gateway override labels
  0x2100 `{0: Closed, 1: Open}` — wrong vs. firmware (0=AS_IS, 1=OPEN, 2=CLOSED);
  a MIK writing 0 "closed" is asking for AS_IS. The field is a footgun, not a feature.
- **Position report goes stale.** Even with fail-safe CLOSED, `ApplyFailSafe` drops
  the relay but never updates `valve_position`; SDO 0x6042 keeps saying OPEN in
  PRE-OP (TPDO1 has stopped, so an SDO read is the only view).
- **Resume re-opens the valve (safety bug, found by code reading — bench B4 proves
  it on today's firmware before we fix it).** RPDO1 only lands in
  the process image in OP. After HB loss the PI still holds the last ControlWord
  (e.g. 0x0009 = Open+Enable). The NMT gate sets `last_control_word = 0`, so when the
  master sends NMT start the FSM goes IDLE and sees a fresh rising edge on bit 0
  → **valve opens itself the instant the master returns**, without a command.
- The callback also re-requests PRE-OP (`MCO_HandleNMTRequest`), redundant: the
  stack forces PRE-OP itself right after `MCOUSER_HeartbeatLost` returns
  (phtemp plan, "Master-heartbeat policy").

### 1.2 EMCY is wired but unused
`USE_EMCY 1`, `USECB_EMCY 1`, but `MCOUSER_EMCY` is `return 0`; the valve pushes no
application EMCY. ErrorRegister has the **same dual-source bug** as the pump/phtemp:
`error_register` is a file-static shadow written only to the PI (`ProcImg_SetErrorRegister`,
TPDO2) while SDO reads and EMCY frames use `gMCOConfig.error_register`. Gateway
overrides record exactly this ("TPDO2 reports 0 while an SDO read returns 1"), and
the fault-reset test that "did not clear it" is explained: CW bit 7 clears the shadow
only (and only if `fault_active`), never the stack's latched bit 0.

**The mystery recurring EMCY 0x6100 / MSEF `[0x48, 0x20|0x30|0x40, …]` is decodable
from source** (gateway notes say "no vendor table"):
`MCOUSER_FatalError(ErrCode)` pushes `EMCY_INTERN_SW (0x6100)` with
`em_1 = ErrCode>>8, em_2 = ErrCode`, and `mco.h:136-139` defines
0x4820 = TPDO lost, 0x4830 = SDO lost, 0x4840 = heartbeat lost —
**"transmit buffer overflow" warnings** raised when `MCOHW_PushMessage` fails
(`mco.c:3154`, `mcop.c:1470`). The stack also sets ER bit 0 on any EMCY
(`mcop.c:698`) and never clears it. Meaning: the valve's CAN TX FIFO was full,
i.e. frames weren't leaving (no ACK / bus-off / no other node awake — TPDO1 every
100 ms + TPDO2 1 s + HB 1 s on a bus nobody acks fills it quickly). **Hypothesis —
confirm on a trace** (bench step B6); it is a bus/hardware symptom, not a valve
state-machine fault, and the plan must not turn it into a FAULT.

### 1.3 Dead weight in firmware + OD
| Item | Finding |
|---|---|
| **FAULT via motion timeout** | `TickMotion` faults at 2×`MotionTimeout`, but motion "completes" after the fixed 50 ms settle (`VALVE_MOTION_SETTLE_MS`, Clippard EV-2M-24, no position sensor), so the fault branch is **unreachable**. 0x2300 is dead config; the gateway still writes it to 1 s at every boot (`overrides.yml` `sdo: 0x2300=1`). |
| 0x2101 ManualOverride (typo "ManaualOverride") | accessor exists, **no caller**; `ro` with default 0. Dead. |
| 0x2100 FailSafePosition | replaced by the hard-wired policy below. |
| EmSA 0x2222 demo segmented-SDO handler | still live in `user_cbdata.c:334-420` (phantom object 0x2222:23/24; `for(;;)`-style demo buffers). Pump + phtemp already dropped / will drop it. |
| 0x1006 / 0x1007 / 0x1019 (SYNC period/window/overflow), 0x1020 (verify config) | advertised, never used: RPDO1 TType 255 (event), no SYNC consumers, `USE_STORE_PARAMETERS 0`. Same verdicts as phtemp regen #2. |
| `MCO_EVENT_HEARTBEAT_LOST` listener | exists (`ValveControl_OnHeartbeatLost`) but can never fire (see 1.1). |
| 1 Hz `ValveControl_RunDiagnostics` | prints `MotionTmr=%lu/%us` from 0x2300 — goes away with the object. |

### 1.4 Logging
USART2 ring buffer (`log.c` 137 L, `Log_Init(&huart2)`, `HAL_NVIC_EnableIRQ(USART2_IRQn)`
at main.c:146, polled-UART `Error_Handler`, `USART2_IRQHandler → Log_TxISR`). Pump and
phtemp `log.c` are already RTT (identical backend, 4 K up-buffer, `NO_BLOCK_SKIP`).
`DBG_FAILSAFE_ENABLE` is **0** today — the very subsystem this work exercises.

---

## 2. Decisions

- **[D1] Fail-safe = always CLOSED, hard-wired; delete 0x2100.** Relay de-energized
  = closed, so this matches power-loss behaviour and the boot state
  (`MX_GPIO_Init` drives PA10 LOW). Triggers: master-HB loss, **any** exit from
  NMT OP (PRE-OP / STOP / reset-comm), MCO fatal error, IWDG reset (already closed at
  boot). Rationale: a configurable "as-is/open" option on a no-feedback actuator is
  exactly the field that mis-specified itself in the gateway enum. If fail-open is
  ever needed it becomes a compile-time constant or a *new* object — not a runtime
  write that defaults to the dangerous value. **Recommend: yes.**
- **[D2] HB loss → close + PRE-OP, no FAULT (phtemp semantics; pump e-stops + PRE-OP
  without FAULT too).** Stack pushes EMCY 0x8130 and forces PRE-OP; app closes the relay
  *synchronously inside the event callback* (the callback runs before the stack's
  PRE-OP; do not wait for the NMT gate's next loop). Valve comes back **CLOSED** and
  never auto-reopens; resume = master NMT start, then a fresh ControlWord edge.
  FAULT would latch until a CW reset after the master returned — wrong for a comm event.
  **Recommend: yes.** Consumer = node 127, 2500 ms (pump/phtemp numbers), armed after
  `MCO_DefaultResetCommunication` so its `UpdateSystemFromOD` can't clobber it.
- **[D3] FAULT state: keep the contract plumbing, but nothing enters it.**
  With the motion-timeout path gone the valve has no sensor-detectable fault.
  Keep `VALVE_STATE_FAULT`, `SW_FAULT` (bit 3), CW bit 7 reset and the
  0x1001/EMCY plumbing (the gateway enum documents `128: Reset Fault`; the MIK
  already sends it) so the wire contract is stable — but **do not enter FAULT
  from stack events.** *Review finding 2026-09-30:* the original wording
  ("entered via `MCO_EVENT_FATAL_ERROR` ≥ warning threshold") contradicted D4:
  the only warning-class codes the stack can raise (0x4000 ≤ code < 0x8000,
  `mco.h:135-141`) are exactly the TX-buffer-overflow codes 0x4810–0x4840 and
  the mapping code 0x6000, and every code ≥ 0x8000 calls
  `MCOUSER_ResetApplication` ~10 ms later, so a FAULT from those never
  survives. As written, FAULT would have fired on precisely the recurring
  0x6100/0x48xx event D4 excludes — a regression from today. Resolution: the
  fatal-error listener logs the code; for codes **≥ `ERR_FATAL` (0x8000)** it
  also calls `ForceClosed` (the stack resets the MCU ~10 ms later, so this only
  drops the relay a few ms earlier than the reset would); for warning-class
  codes (< 0x8000, i.e. the 0x48xx TX overflows) it does **nothing** — closing
  an open valve on every TX-FIFO hiccup while the stack stays in OP would be
  worse than today. No app EMCY (the stack already pushed 0x6100). Revisit
  when a feedback sensor exists.
- **[D4] TX-overflow warnings (0x6100/0x48xx) are NOT faults** — unchanged stack
  behaviour, but CW bit 7 reset (and HB-restored) must clear the stack-latched ER
  bit 0 when the app has nothing to report (phtemp `HEARTBEAT_RESTORED` pattern),
  so 0x1001 can return to 0x00 without a power cycle. Consistent with D3 as
  revised. **Recommend: yes.**
- **[D5] TPDO layout unchanged** (TPDO1 = SW + ValveState, TPDO2 = ErrorRegister).
  Moving ErrorRegister into TPDO1 would delete TPDO2 *and* the
  `MCO_InitTPDOFull` re-assert workaround (the leading-0x1001 lookup bug) and halve
  bus chatter — but it is a breaking PDO change for the gateway shadow. Park as a
  "layout-3 candidate" exactly like phtemp's 0x2300 consolidation. **Recommend: park.**
- **[D6] Residual risk, accept + document:** the consumer sits in `HBCONS_INIT` until
  it sees the master's first HB (no false trip at boot/bench). A master that sends NMT
  start + Open *without ever producing a HB* is unprotected. The gateway always
  produces 1 s HB before start. MIK requirement (same wording as pump): keep the 1 s
  HB alive whenever the valve may be open; a >2.5 s gap closes it. Optional hardening
  (refuse Open until the consumer is ACTIVE) deferred. **Recommend: accept.**
- **[D7] Identity:** RevisionNumber 0x00010001 → **0x00020000** (breaking: objects
  deleted), set *in Architect* this time (.cax is the source; no hand-edit). Bump FW
  banner to **3.0.0** (pump 2.0.0, phtemp 4.0.0 — pick per Dakota's numbering;
  nothing depends on it). 0x1009/0x100A stay "1.0" placeholders.
- **[D8] Node name normalisation:** rename the Architect device output to
  `ValveModule-n09-250kbs` (no spaces; the gateway's copies are already hyphenated).
  Keep `-n09-` — renaming in Architect does not change the DCF NodeID (phtemp gotcha).
- **[D9] A master SDO write to 0x1016 disarms the firmware-armed consumer**
  *(review finding 2026-09-30, applies to phtemp too).* The stack's SDO write
  handler for 0x1016:01 calls `MCOP_InitHBConsumer(sub, node, time)` with
  whatever arrives (`mco.c:2687`); a zero write sets the consumer `HBCONS_OFF`.
  If the master's boot DCF writes 0x1016:01 (the gateway DCF copy has an empty
  default and `heartbeat_consumer` is commented out in `overrides.yml`, so it
  probably does not — **unverified**; this is the pump plan's open carry-forward
  "does dcfgen boot DCF write 0x1016?"), the consumer is killed right after every
  boot: B3 on the CANopen Magic bench passes while the gateway never gets
  protection. Phase 0 must trace what the master writes to node 9 at boot.
  If it does write 0x1016, put the consumer in the EDS default at the regen
  (0x1016:01 = `0x007F09C4` = node 127, 2500 ms) so the OD, the gateway's DCF
  copy and the firmware all agree, and keep the firmware arm as belt-and-braces.
  **Recommend: trace first; EDS default only if the master writes it.**

---

## 3. Target EDS (Architect regen — the only Windows step)

Project `E:\ursaScience\Modules\Modules-base.cax`, valve device (node 9).
28 objects today → **21** (seven deleted; the first draft said 22). Mechanical
edit list: `docs/VALVE_ARCHITECT_REGEN.md`.

| Object | Verdict | Why |
|---|---|---|
| 0x1000 / 0x1001 / 0x1018 | KEEP (mandatory) | 0x1018:03 → `0x00020000` |
| 0x1003 Pre-defined Error Field | KEEP **— verify** | stack EMCY history; gateway notes say reads fail on the valve ("General error / Unsupported access") — check on bench whether phtemp/pump behave the same (B7); if broken on all, file separately |
| 0x1008 / 0x1009 / 0x100A | KEEP | identity strings; wire 0x100A to `FIRMWARE_VERSION` later (shared open item) |
| 0x1014 / 0x1015 | KEEP | EMCY COB-ID / inhibit — this task *uses* EMCY |
| 0x1016 / 0x1017 | KEEP | gateway writes both at boot (deleting either fails node boot). **Second reason for 0x1016:** the generated `pimg.h` derives `NR_OF_HB_CONSUMER 1` from its entry count — delete it and the consumer compiles out. See D9 for the optional 0x1016:01 default |
| 0x1400/0x1600, 0x1800/0x1A00, 0x1801/0x1A01 | KEEP unchanged | RPDO1 CW; TPDO1 SW+ValveState 3 B; TPDO2 ER 1 B |
| 0x6040 / 0x6041 / 0x6042 | KEEP | CiA-408 control/status/state; contract stays |
| 0x2000 LEDControl | KEEP (ask MIK) | delete only if MIK never writes it |
| **0x1006 / 0x1007 / 0x1019** | **DELETE** | no SYNC use (RPDO TType 255) — phtemp regen #2 precedent |
| **0x1020** | **DELETE** | no store/verify; contradicts "firmware has no persistence" |
| **0x2100 FailSafePosition** | **DELETE** | [D1] hard-wired CLOSED |
| **0x2101 ManualOverride** | **DELETE** | no caller; misspelt; ro constant |
| **0x2300 MotionTimeout** | **DELETE** | fault path unreachable; 50 ms settle constant is the real timing |

Also set `RevisionNumber = 0x00020000`, device name `ValveModule`, regen, copy into
`devices/valve/MCO_CiA401__User/EDS/` (six files: `.eds .dcf *_public.h entriesandreplies.h pimg.h stackinit.h`).
Expect `PIMGEND` to shrink and **every offset after 0x3E to shift** — that is why
`procimg_api.h` is never hand-patched, only rewired. Write the regen as
`docs/VALVE_ARCHITECT_REGEN.md` (mirror `PHTEMP_ARCHITECT_REGEN_2_THRESHOLDS.md`)
before opening Architect.

---

## 4. Firmware changes (`devices/valve` only)

### 4.1 RTT logging (Phase 1 — independent of everything else)
1. Copy from `devices/phtemp/Core/`: `Src/SEGGER_RTT.c`, `Inc/SEGGER_RTT.h`,
   `Inc/SEGGER_RTT_Conf.h`, `Src/log.c` (byte-identical to pump's apart from comments).
   CMake globs `Core/Src/*.c`, no list to edit.
2. `log.h`: keep the `DBG_*` macro surface (no call-site edits); replace the ring-buffer
   doc/`LOG_RING_*` and drop the `UART_HandleTypeDef` dependency where phtemp did.
   Flip `DBG_FAILSAFE_ENABLE` to **1** (this work lives there).
3. `main.c`: `Log_Init(&huart2)` stays as phtemp's compat shim; **delete** the
   `HAL_NVIC_SetPriority/EnableIRQ(USART2_IRQn)` block; `PUTCHAR` → `Log_PutChar`
   (already); replace the polled-UART `Error_Handler` body with phtemp's
   `SEGGER_RTT_WriteString` + register-level LED blink (valve LED pins: same tri-LED,
   Fault = PB6 already used). Banner: add `Firmware: %s` — the valve has **no**
   `FIRMWARE_VERSION` define today (the banner is a hard-coded "v1.0" printf), so
   D7 means adding the define, not just printing it.
4. `stm32g4xx_it.c`: `USART2_IRQHandler` stays as the no-op `Log_TxISR` (vector links,
   IRQ never enabled) — identical to phtemp. **Optional Phase 1b** (not precedent):
   delete `MX_USART2_UART_Init`, `huart2`, the msp block and the IRQ handler so PA2/PA3
   float analog; do it only if Dakota wants UART *gone*, not just unused.
5. Docs: `BUILD_NOTES.md` valve row + Context.md: "all three devices on RTT";
   `probe-rs attach --chip STM32G431KBTx build/valve-n09-250k.elf`.
6. Flash caveat: valve fixture **has NRST**, IWDG runs — `st-flash --connect-under-reset
   --reset write build/valve-n09-250k.bin 0x08000000`; never halt the core from the viewer.

### 4.2 Heartbeat consumer + fail-safe close (Phase 2)
- `MCO_Target/user_STM32.c`: add `MASTER_HB_NODE_ID 127` / `MASTER_HB_TIMEOUT_MS 2500`
  and `MCOP_InitHBConsumer(1, 127, 2500)` **after** the defaults and after the TPDO2
  `MCO_InitTPDOFull` re-assert, guarded by `if (result)` — same shape as phtemp L132-162.
  `MCOUSER_HeartbeatLost`: fire `MCO_EVENT_HEARTBEAT_LOST`, **delete** the
  `MCO_HandleNMTRequest(NMTMSG_PREOP)` line (stack does it; comment why).
- `mco_events.h/.c`: add `MCO_EVENT_HEARTBEAT_RESTORED` (valve's dispatcher is
  per-type: bump `MCO_EVENT_COUNT`). `user_cbdata.c::MCOUSER_EMCY`: on
  `ev_clr && emcy_code == EMCY_HB_ERR` fire it (`em_1` = node), return 0 so the stack
  still emits its own recovery frame.
- `valve_control.c`:
  - `ValveControl_ApplyFailSafe()` → **`ValveControl_ForceClosed(reason)`**: relay OFF,
    `valve_position = VALVE_POS_CLOSED`, **zero the PI ControlWord**
    (new `ProcImg_SetControlWord(0)`), `last_control_word = 0`, log the reason. Idempotent
    (events repeat). Delete the 0x2100 switch, `FAILSAFE_*` defines, and
    `ProcImg_{Get,Set}FailSafePosition`.
  - Callers: HB-lost event (synchronous), NMT gate on leaving OP, fatal-error event,
    boot (`IWDGRST` block keeps `ValveDriver_Init()` + `ForceClosed`).
  - **Re-arm rule (closes the resume-reopen bug):** on DISABLED→IDLE set
    `last_control_word = ProcImg_GetControlWord()` (level present at entry is "already
    seen"). Opening after (re)start therefore needs a fresh 0→1 edge on bit 0, mirroring
    the pump's "CW edge required" rule; belt-and-braces with the PI zeroing above.
  - Delete `TickMotion`'s fault branch + `ProcImg_GetMotionTimeout` uses; motion
    completion stays at `VALVE_MOTION_SETTLE_MS`. Drop `MotionTmr` from the 1 Hz
    diagnostic line (add `NMT=` instead; the stack does not export the consumer
    state — `gHBCons` is file-static to mcop.c — so no `HB=` field).
  - `MCO_EVENT_FATAL_ERROR` listener: log the code; `ForceClosed` only when
    `error_code >= ERR_FATAL` (MCU reset follows); warning-class codes (0x48xx)
    are log-only. No FAULT, no app EMCY (D3 as revised).

### 4.3 EMCY + ErrorRegister (Phase 2)
Register written to **both homes** edge-wise (PI for TPDO2, `gMCOConfig.error_register`
for SDO + EMCY), owning only its own bits — copy phtemp's
`SensorControl_UpdateErrorRegister` shape:

| Event | 0x1001 | EMCY |
|---|---|---|
| Master HB lost | stack sets bit 0 | **0x8130** (stack, automatic); app logs `[FAILSAFE]` + closes |
| HB restored | app clears bit 0 if it has nothing else (`HEARTBEAT_RESTORED`) | stack recovery frame (0x0000-class) |
| Stack fatal/warning error (`MCO_EVENT_FATAL_ERROR`) | stack sets bit 0 (its own 0x6100 push) | **0x6100** (stack, unchanged). App: log; `ForceClosed` only if code ≥ 0x8000 (reset follows); warning-class = log only; **no FAULT, no app EMCY** (D3 as revised) |
| CW bit 7 reset | app bits cleared + stack bit 0 cleared if nothing else pending | **0x0000** if anything was set |
| TX-overflow warnings 0x48xx | stack sets bit 0 (unchanged) | 0x6100 (stack, unchanged) — documented as bus symptom, cleared by reset |

Bits 1–5 unused on purpose (no current/voltage/temperature/comm-fault sensing on this
module, and no app-detectable fault since D3). `error_register` file-static goes away;
`ProcImg_SetErrorRegister` mirrors the merged register every loop. Fault reset must
work in *every* state, not only `fault_active` (today a reset with no latched app
fault is a no-op — that is the "reset didn't clear ER=1" symptom).

### 4.4 Remove the 0x2222 demo SDO
`user_cbdata.c`: `MCOUSER_AppSDO*` → "not handled" stubs, delete demo buffers
(phtemp finding #6). Verification: SDO read 0x2222:23 aborts with 0x08000000.

### 4.5 Rewire onto the regenerated OD (Phase 3)
`procimg_api.h`: delete accessors for 0x2100 / 0x2101 / 0x2300; add `SetControlWord`.
Confirm `pimg.h` offsets via the build (no magic numbers). Firmware-first ordering is
possible exactly as phtemp did: Phases 1–2 build against the *current* OD (0x2100/2300
just unused), so the behaviour can be bench-proven before Architect is touched.

---

## 5. Sequencing

| # | Phase | Output | Gate |
|---|---|---|---|
| 0 | Verify starting state: `build/valve-n09-250k.bin` currently flashed? does TPDO2 still emit DLC=0 (un-flashed 2026-08 fix, `MCOUSER_ResetCommunication`)? **Does the gateway master SDO-write 0x1016:01 at node-9 boot (D9)?** | trace of boot on the bench valve, once with the gateway as master | know baseline before changing anything |
| 1 | **RTT swap** (4.1) | commit "valve: RTT logging" | boots, banner + state lines over probe-rs, CAN behaviour unchanged (open/close, TPDO1 100 ms) |
| 2 | **Behaviour** (4.2–4.4) on the current OD | commit "valve: master HB consumer, close on loss, EMCY, both-homes 0x1001" | bench B1–B9 below |
| 3 | **Architect regen** (§3) + rewire (4.5) | regen doc + EDS files + commit; `.cax` synced, rev 0x00020000 | builds; SDO to deleted objects aborts; text size ↓ |
| 4 | **Gateway cutover, ONE window** | new `ValveModule-n09-250kbs.{eds,dcf}` in `bridge/devices/`; `overrides.yml`: **delete the `sdo: 0x2300=1` block** (else the boot-time SDO write to a deleted object fails node boot), delete `2100:0`, `2101:0`, `2300:0` entries, fix/retire the 0x6100 and 1001 notes; image rebuild. **Concrete sweep targets** (grep 2026-09-30): `bridge/test/test_gen_network.py:105-106` asserts `2300:0` unit/notes; `bridge/gen-network.py:37` and `api/test/server-routes.test.ts:321-359` carry the 0x6100 note + a `2300:0` fixture; `docs/device-management.md:230,254` and `docs/plans/agent-hardware-context.md:12-90` describe MotionTimeout/FailSafe/ManualOverride. Those tests fail on the cutover if not updated in the same window | master boots node 9 with no 1018:03 mismatch |
| 5 | Flash final image + full bench (B1–B12) | traces on Samsung T5 `cantrace-valve-refactor*.csv` | checklist green |
| 6 | Docs: Context.md, BUILD_NOTES, this plan → DONE; `VALVE_FE_CUTOVER_GUIDE.md` (MIK: HB requirement, CW-edge re-arm, FailSafe gone, recovery = NMT start + fresh open); MIK_INTEGRATION_BRIEF valve section | | |

The cutover is coupled for the same reason as phtemp: the master image checks 1018:03
at boot. Order inside the window: gateway files + overrides → image → flash → boot check.

---

## 6. Bench validation checklist (CANopen Magic, node 9)

COB-IDs: NMT `000`, RPDO1 `209` (CW, 2 B LE), TPDO1 `189` (SW+ValveState, 3 B),
TPDO2 `289` (ER, 1 B), EMCY `089`, node HB `709`, master HB `77F` (send `77F#05` @1 s).
CW: Open `0x0009`, Close `0x000A`, Reset `0x0080`. ValveState 1=Closed 2=Open 3=Moving.
Copy layout/style from `docs/PHTEMP_BENCH_TEST.md`; probe-rs RTT open alongside.

*Bench status 2026-10-01 (node 9, fw 3.0.0 on the current OD, CANopen Magic
only; trace `cantrace-testing-valve-phase1_2_testing.csv`; full frame-level
playbook + what is still open: `docs/VALVE_BENCH_TEST.md`):*

- [ ] **B1 Identity**: SDO 0x1018:03 = `00 00 02 00`; 0x1018:02 = 5; banner shows FW version. *(post-regen; no SDO reads done yet)*
- [x] **B2 Baseline cmd**: NMT start → TPDO1 every ~100 ms; `209#09 00` → relay on,
      SW `0x0602`-class settled open within ~50 ms; `209#0A 00` → closed `0x0601`.
      *PASS 2026-10-01: TPDO1 100.0 ms median; moving→settled 49–50 ms; TPDO2 DLC 1.*
- [x] **B3 HB loss while OPEN** *(the feature)*: master HB running, valve open; stop the
      HB frames → within **2.5 s (+≤1 loop)**: relay OFF (scope/LED), EMCY `089#30 81 <ER> …`
      (0x8130), node HB changes to `7F`(PRE-OP), RTT `[FAILSAFE] Heartbeat lost … closing`;
      SDO 0x6042 reads **1 (Closed)** (not stale 2).
      *PASS on the wire 2026-10-01: EMCY `30 81 01 7F …` 2499.8 ms after the last 0x77F,
      TPDOs stop, HB `7F`. SDO 0x6042/0x6040 reads still owed.*
- [x] **B4 Resume does NOT reopen**: restore HB; `000#01 09` (NMT start) and send
      **no RPDO** → valve stays CLOSED, state IDLE (the PI CW was zeroed by
      `ForceClosed` and the DISABLED→IDLE snapshot swallows any stale level). Then
      send `209#09 00` once → valve OPENS: after recovery any Open frame is a fresh
      command, because the module's view of the CW went 0→9 even if the master's
      shadow never changed. Both halves are the contract; the FE guide must say so.
      *(The first half fails on today's firmware — capture that first as the
      regression proof.)* *PASS 2026-10-01: valve open at HB loss; after
      recovery + NMT start with no RPDO, TPDO1 = `01 06 01` (closed) for 4.7 s;
      one `209#09 00` opened it. Old-image regression proof not captured
      (old image has no consumer; the NMT variant in the playbook is the way).*
- [x] **B5 HB restored clears ER**: after B3/B4, SDO 0x1001 = 0x00 without CW reset
      (HEARTBEAT_RESTORED path); TPDO2 agrees. *PASS indirectly 2026-10-01: stack
      recovery EMCY 0x0000 0.5 ms after the first returning 0x77F, and TPDO2 read
      `00` (not `01`) once back in OP. SDO 0x1001 read still owed.*
- [ ] **B6 Boot with no master**: power up, no HB on bus → no false 0x8130, valve closed,
      stays in PRE-OP until NMT start. Capture a TX-overflow EMCY on an unterminated/no-ack
      bench bus to confirm the 0x6100/0x48xx decode (§1.2). **Also capture one with the
      gateway live**: the deployed valve throws them while the master is ACKing, so
      "nobody acks" does not explain the field occurrence — find what does.
- [ ] **B7 0x1001 both homes + stack-error close**: inject a stack error via
      `MCOUSER_FatalError(0x4820)` (gdb `call`, per `PUMP_FAULT_INJECTION_PLAYBOOK.md`
      technique — probe-rs no-halt pokes, never halt: IWDG) while OPEN: EMCY
      `0x6100` MSEF `48 20`; valve **stays OPEN**, FSM IDLE (**no** FAULT, SW bit 3
      clear, no close — warning class, D3); SDO 0x1001 == TPDO2 byte == 0x01 (both
      homes agree). CW `0x0080` edge → 0x1001 = 0 in both homes, EMCY `0x0000`, valve
      still open. Also read 0x1003 (see §3 caveat).
- [ ] **B8 Reset with no latched fault** is harmless and clears a stack-set bit 0 (the
      2026-08-21 "reset did not clear ER=1" case).
- [x] **B9 NMT exits close the valve**: from OPEN, `000#80 09` (PRE-OP), `000#02 09`
      (STOP), `000#82 09` (reset-comm) — each closes + SDO 0x6042=1.
      *PASS 2026-10-01: after each of `80`/`02`/`82` the next NMT start reported
      `01 06 01` (closed, idle) with no RPDO sent, and a fresh `09 00` opened it.
      SDO 0x6042 reads still owed.*
- [ ] **B10 Deleted objects abort** (0x08000000): 0x2100, 0x2101, 0x2300, 0x2222:23,
      0x1006/1007/1019/1020. Kept objects read: 0x2000 write/read round-trip.
- [ ] **B11 IWDG recovery**: stall via poke → reset → relay closed at boot, no stale open.
- [ ] **B12 Gateway end-to-end**: node 9 boots clean (no SDO write to deleted 0x2300),
      dashboard shows closed/open correctly, pull the gateway/MIK power with valve open →
      closes within 2.5 s; `devices.json` for node 9 has no 2100/2101/2300.
- [ ] pump/phtemp untouched: `git diff --stat` confined to `devices/valve`, docs;
      `arm-none-eabi-size build/*.elf` pump/phtemp `.text` unchanged.

---

## 7. Risks / watch-list

- **Coupled cutover** — deleting 0x2300 while the gateway still writes it breaks node-9
  boot; the override edit and the EDS copy must land together (Phase 4).
- **PIMG offsets shift** on regen — only rebuild-and-check, never hand-edit `pimg.h`.
- **Behaviour change for the MIK** (must be in the FE guide): open after any restart
  needs a fresh CW edge; HB must be alive while open; FailSafe write path is gone;
  **any NMT exit closes the valve**, so a gateway restart that resets the bus now
  closes an open valve (correct, but visible).
- **D9 (0x1016 SDO disarm)** is the one item that can make the bench pass and the
  field fail silently — do not skip the Phase 0 gateway trace. *Still open
  2026-10-01: both bench traces so far were CANopen Magic-only.*
- **Power-cycling the valve holds the bus in error for ~0.75–1.2 s** (baseline
  trace: 2877 error frames / 747 ms; phase-1/2 trace: 4617 / 1.2 s), during
  which the master's SYNC cannot be sent. Under 2.5 s, so no HB consumer trips,
  but a valve power-cycle on a live bus is not transparent to the other nodes.
  Hardware/fixture observation (transceiver during MCU power-up?), not firmware;
  check whether pump/phtemp boards do the same.
- **CANopen Magic's node-127 producer was sending DLC 5 all-zero frames** (decoded
  as "Bootup"). The stack's consumer accepts any frame on 0x77F (boot-up
  explicitly included, `mcop.c MCOP_ConsumeHB`), so the tests are valid, but set
  the producer to DLC 1 `05` for clean traces.
- **No 0x6100 in 126 s of bench traffic** with CANopen Magic ACKing. The field
  occurrence (live gateway) remains unexplained; B6 still owed.
- Hard-wired CLOSED assumes the physical valve is normally-closed on relay OFF (true for
  the Clippard EV-2M-24 install, `valve-hardware-clippard-ev2m24`); re-decide [D1] for any
  normally-open variant.
- Halted debug sessions trip the IWDG (2 s) — use non-halting RTT/probe-rs only.
- `.cax` is Windows-only: expect the regen to be the long pole; everything else
  (Phases 1–2) is macOS-buildable and bench-provable first.
- Out of scope, noted: ValveState 0 "Unknown" after HALT leaves the relay in its current
  state (HALT during the 50 ms window) — harmless at 50 ms, left alone.
