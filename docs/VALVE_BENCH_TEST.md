# Valve fw 3.0.0 (Phases 1–2, current 28-object OD) — CANopen Magic bench playbook, nodes 8 / 9

Frames are written the way CANopen Magic's transmit list and trace show them:
**COB-ID, DLC, raw data bytes** (hex, exactly as on the wire = little endian
for multi-byte values). Bus 250 kbit/s. Record the whole session; name
traces `cantrace-valve-nNN-<test>.csv` (Samsung T5).

Every section is written once. Where a row says `SDO-REQ`, `TPDO1`, `NN`
etc., substitute from the node table. Both nodes run the **same image apart
from the node ID** (`valve-nNN-250k.bin`, identical except for the node-ID
bytes), so expected data bytes are identical across nodes.

This playbook is for the firmware **before the Architect regen** (Phase 3):
0x2100 / 0x2101 / 0x2300 still exist in the OD but nothing reads them, the
revision is still 0x00010001, and the valve name on the bus is still
"Valve Module". Rows marked **[post-regen]** change after Phase 3.

Logging is **SEGGER RTT over SWD** (no serial adapter — PA2/PA3 are dead),
non-halting. The IWDG (2048 ms) is running: never halt the core.

```
probe-rs attach --chip STM32G431KBTx build/valve-nNN-250k.elf
```

Flash (fixture has NRST, IWDG runs):

```
st-flash --connect-under-reset --reset write build/valve-nNN-250k.bin 0x08000000
```

**Validated so far (node 9, 2026-10-01, trace
`cantrace-testing-valve-phase1_2_testing.csv`, CANopen Magic only — no
gateway):** A, D (TPDO2 **DLC 1**, `TPDO1` 100.0 ms median cadence), E
(open/close settle 49–50 ms, repeated Open/Close = no-op), G (0x8130
2499.8 ms after the last 0x77F, TPDOs stop), H steps 1–3 (recovery EMCY
0x0000 on the first 0x77F; NMT start with no RPDO → `01 06 01` closed; one
`09 00` opens), J rows `80`/`02`/`82` + restart-stays-closed + fresh Open.
**Not yet done:** every SDO row (B, C, G steps 3–6, I), F, K, M, N, node 8.
Relay state was observed by ear/eye, not on the trace. Baseline trace
(`cantrace-testing-valve-baseline.csv`, old image) never reached OP, so the
TPDO2 DLC=0 question has no "before" number — only the "after" (DLC 1).
Both traces start with a ~0.75–1.2 s error-frame storm on power-cycle of the
valve (bus unusable, SYNC suppressed) — fixture/hardware observation, see
plan §7.

## Node table

| Symbol | meaning | DLC | node 8 | node 9 |
|---|---|---|---|---|
| `NN` | node byte in NMT frames | — | `08` | `09` |
| `SDO-REQ` | SDO request, you → node (0x600+N) | 8 | 0x608 | 0x609 |
| `SDO-RSP` | SDO response, node → you (0x580+N) | 8 | 0x588 | 0x589 |
| `RPDO1` | ControlWord `lo hi`, you → node (0x200+N) | 2 | 0x208 | 0x209 |
| `TPDO1` | `SW lo, SW hi, ValveState` (0x180+N) | **3** | 0x188 | 0x189 |
| `TPDO2` | `ErrorRegister` (0x280+N) | **1** | 0x288 | 0x289 |
| `EMCY` | emergency (0x080+N) | 8 | 0x088 | 0x089 |
| `HB` | heartbeat (0x700+N) | 1 | 0x708 | 0x709 |
| NMT | you → all, always 0x000 | 2 | — | — |
| master HB | CANopen Magic HB producer, node 127 | 1 | 0x77F `05` @ 1000 ms | same |
| image | `build/valve-nNN-250k.bin` | | n08 | n09 |
| banner | RTT `Node ID:` | | 0x08 | 0x09 |

SDO reply byte 0: `4F` = 1 data byte, `4B` = 2, `47` = 3, `43` = 4, `60` =
write accepted, `80` = abort with the code in bytes 4-7 (LE). MicroCANopen
aborts a read of a non-existent object with **0x08000000** (`00 00 00 08`),
not the CiA 0x06020000 — any abort is a pass. SDO index bytes are
`lo hi sub` — 0x1800:03 is `00 18 03`.

ControlWord values the MIK actually sends (bit 3 = Enable Operation is an
interlock, so Open/Close are 0x09/0x0A, never 0x01/0x02):

| `RPDO1` data | CW | meaning |
|---|---|---|
| `00 00` | 0x0000 | idle / clear |
| `09 00` | 0x0009 | **Open** (bit 0 rising edge + enable) |
| `0A 00` | 0x000A | **Close** (bit 1 rising edge + enable; bit 0 drops → the next `09 00` is a fresh edge) |
| `80 00` | 0x0080 | error reset (bit 7 rising edge) |
| `09 01` | 0x0109 | Halt (bit 8) during motion — only a 50 ms window, not exercised here |

Open and Close are **edges**: a repeated `09 00` with bit 0 already high is a
no-op. To re-open after an Open: `0A 00` (or `00 00`) then `09 00`.

StatusWord (0x6041) values you will see:

| SW | bytes on wire | meaning |
|---|---|---|
| 0x0000 | `00 00` | DISABLED (PRE-OP / STOP / before NMT start) |
| 0x0601 | `01 06` | IDLE, closed, target reached, remote |
| 0x0602 | `02 06` | IDLE, open, target reached, remote |
| 0x0204 | `04 02` | moving (≤ 50 ms) |
| 0x0200 | `00 02` | IDLE, position UNKNOWN (only after a Halt) |

ValveState (0x6042, `TPDO1` byte 2): `00` Unknown, `01` Closed, `02` Open,
`03` Moving.

---

## A. Power-on → PRE-OPERATIONAL

Nothing to send. Expect:

| COB-ID | DLC | Data | Meaning |
|---|---|---|---|
| `HB` | 1 | `00` | boot-up |
| `HB` | 1 | `7F` every 1000 ms | heartbeat, PRE-OP |
| `TPDO1` / `TPDO2` | — | none | TPDOs are OP-only |
| `EMCY` | — | none | no master yet → no 0x8130 (consumer waits for the first 0x77F) |

Relay OFF (no click, valve closed). RTT:

```
=== Valve Module (CiA 408) ===
Firmware: 3.0.0
Node ID: 0x09, Bitrate: 250 kbit/s
[VALVE] Init complete, state=DISABLED, relay OFF (closed)
[MAIN] Init complete, entering main loop
[VALVE] State=DISABLED Pos=CLOSED SW=0x0000 Relay=OFF NMT=0x7F CW=0x0000 Err=0x00   (every 1 s)
```

## B. Identity (works in PRE-OP)

| Send `SDO-REQ` (DLC 8) | Expect `SDO-RSP` (DLC 8) | Meaning |
|---|---|---|
| `40 00 10 00 00 00 00 00` | `43 00 10 00 08 04 00 00` | device type 0x00000408 (CiA 408) |
| `40 18 10 01 00 00 00 00` | `43 18 10 01 23 01 00 00` | vendor 0x123 |
| `40 18 10 02 00 00 00 00` | `43 18 10 02 05 00 00 00` | product 5 = valve |
| `40 18 10 03 00 00 00 00` | `43 18 10 03 01 00 01 00` | revision 0x00010001 — **[post-regen]** `00 00 02 00` |
| `40 18 10 04 00 00 00 00` | `43 18 10 04 78 56 34 12` | serial 0x12345678 from `MCOUSER_GetSerial()` (NOT the EDS's 0x123444); same on every unit |
| `40 08 10 00 00 00 00 00` | `41 08 10 00 0C 00 00 00` | device name, 12 B segmented ("Valve Module") — [post-regen] unchanged (name string stays) |
| `40 0A 10 00 00 00 00 00` | `47 0A 10 00 31 2E 30 00` | sw version string "1.0" (known placeholder; the real version is the RTT banner) |
| `40 17 10 00 00 00 00 00` | `4B 17 10 00 E8 03 00 00` | producer heartbeat 1000 ms |
| `40 16 10 01 00 00 00 00` | `43 16 10 01 C4 88 00 00` | **consumer armed**: the stack packs `(127 << 8) + 2500` = 0x88C4 (stack quirk, not the CiA layout — do not decode). `00 00 00 00` here means the consumer is OFF → plan D9, something wrote 0x1016 |
| `40 00 18 03 00 00 00 00` | `4B 00 18 03 0A 00 00 00` | TPDO1 inhibit 10 × 100 µs = 1 ms |
| `40 00 18 05 00 00 00 00` | `4B 00 18 05 64 00 00 00` | TPDO1 event timer 100 ms |
| `40 01 18 05 00 00 00 00` | `4B 01 18 05 E8 03 00 00` | TPDO2 event timer 1000 ms |
| `40 01 1A 01 00 00 00 00` | `43 01 1A 01 08 00 01 10` | TPDO2 mapping 0x10010008 (ErrorRegister, 8 bit) |

## C. Objects that must / must not answer

| Send `SDO-REQ` | Expect `SDO-RSP` now | **[post-regen]** | Object |
|---|---|---|---|
| `40 22 22 23 00 00 00 00` | `80 22 22 23 00 00 00 08` | same | 0x2222:23 EmSA demo — must abort, NOT return a string (the demo handler is deleted) |
| `40 00 21 00 00 00 00 00` | `4F 00 21 00 00 00 00 00` | `80 00 21 00 00 00 00 08` | 0x2100 FailSafePosition — still in the OD, **inert** (writing 2 or 1 changes nothing) |
| `40 01 21 00 00 00 00 00` | `4F 01 21 00 00 00 00 00` | abort | 0x2101 ManualOverride — inert |
| `40 00 23 00 00 00 00 00` | `4B 00 23 00 0A 00 00 00` | abort | 0x2300 MotionTimeout — inert (10 s default; the gateway's boot write of 1 is accepted and ignored) |
| `40 06 10 00 00 00 00 00` | `43 06 10 00 00 00 00 00` | abort | 0x1006 |
| `40 20 10 01 00 00 00 00` | `43 20 10 01 00 00 00 00` | abort | 0x1020:01 |
| `40 00 20 00 00 00 00 00` | `4F 00 20 00 00 00 00 00` | same | 0x2000 LEDControl — kept; `2F 00 20 00 01 00 00 00` → `60 00 20 00 …` write round-trip |
| `40 16 10 01 00 00 00 00` | `43 16 10 01 …` | same | 0x1016:01 must keep answering (deleting 0x1016 would compile the consumer out) |

Proof that 0x2100 is inert on this build: `2F 00 21 00 01 00 00 00` (write
"OPEN") → `60 …`, then run section J — the valve still closes on every
NMT exit.

## D. NMT start → OPERATIONAL (closed, idle)

| Send 0x000 (DLC 2) | Meaning |
|---|---|
| `01 NN` | NMT start this node |

Expect:

| COB-ID | DLC | Data | Meaning |
|---|---|---|---|
| `HB` | 1 | `05` | OPERATIONAL |
| `TPDO1` | **3** | `01 06 01` | within 1 ms: SW 0x0601 (closed, target reached, remote), ValveState 1; then every ~100 ms (event timer) |
| `TPDO2` | **1** | `00` | ErrorRegister 0; every ~1000 ms |

**`TPDO2` must have DLC 1.** A `TPDO2` with DLC 0 is the un-flashed
2026-08 bug (TPDO2 re-assert in `MCOUSER_ResetCommunication`) — note it in
the trace as the Phase 0 baseline if you see it on the *old* image; it must
be gone on 3.0.0.

| Send `SDO-REQ` | Expect `SDO-RSP` | Meaning |
|---|---|---|
| `40 41 60 00 00 00 00 00` | `4B 41 60 00 01 06 00 00` | StatusWord 0x0601 |
| `40 42 60 00 00 00 00 00` | `4F 42 60 00 01 00 00 00` | ValveState 1 = Closed |
| `40 40 60 00 00 00 00 00` | `4B 40 60 00 00 00 00 00` | ControlWord 0 |
| `40 01 10 00 00 00 00 00` | `4F 01 10 00 00 00 00 00` | ErrorRegister 0 == `TPDO2` |

RTT: `NMT change: 0x05`, `NMT Operational -> IDLE (CW at entry 0x0000)`,
`State: 0 -> 1`.

## E. Open / close

| Step | Send `RPDO1` (DLC 2) | Expect | Meaning |
|---|---|---|---|
| 1 | `09 00` | relay click ON; `TPDO1` `04 02 03` (moving) then within ~50 ms `02 06 02` (open); RTT `Open command -> OPENING`, `Motion complete -> OPEN (5x ms)` | open |
| 2 | `09 00` again | nothing (no edge); RTT nothing at level 1 | edge rule |
| 3 | `0A 00` | relay OFF; `TPDO1` `04 02 03` then `01 06 01`; RTT `Close command -> CLOSING`, `Motion complete -> CLOSED` | close |
| 4 | `09 00` | opens again (bit 0 was low after step 3) | fresh edge |
| 5 | `00 00` then `09 00` | from open: `00 00` does nothing (no edge, not a close); `09 00` → `Open ignored: already OPEN` (verbose only) | Open at open = no-op |
| 6 | `0A 00` | closed | leave closed |

| Send `SDO-REQ` (while open) | Expect `SDO-RSP` |
|---|---|
| `40 41 60 00 00 00 00 00` | `4B 41 60 00 02 06 00 00` |
| `40 42 60 00 00 00 00 00` | `4F 42 60 00 02 00 00 00` |
| `40 40 60 00 00 00 00 00` | `4B 40 60 00 09 00 00 00` — the PI holds the last RPDO |

`TPDO1` cadence: one frame within 1 ms of every change (COS) plus one every
100 ms (event timer); inhibit is 1 ms so the moving→settled pair is two
frames ~50 ms apart.

## F. Error reset with nothing latched (harmless)

| Send `RPDO1` (DLC 2) | Expect |
|---|---|
| `00 00` then `80 00` | no EMCY, no state change, valve position unchanged; SDO 0x1001 → `00`. (By SDO instead: `2B 40 60 00 00 00 00 00` then `2B 40 60 00 80 00 00 00`.) |

A repeated `80 00` is a no-op (edge). This is the "reset did not clear
ER=1" case from the gateway notes — it is only meaningful after section K
or L has set bit 0.

## G. Master heartbeat lost while OPEN — the feature

Setup: CANopen Magic's heartbeat producer as **node 127 (0x77F), 1000 ms**
running **before** NMT start (the consumer locks on the first 0x77F it
sees; it needs to be ACTIVE before you pull it). Valve node in OP and OPEN
(section E step 1).

| Step | Do | Expect |
|---|---|---|
| 1 | producer on; `01 NN`; `RPDO1` `09 00` | OP, open: `TPDO1` `02 06 02` |
| 2 | **stop the 0x77F producer** | within **2.5 s** of the last 0x77F: relay click OFF; `EMCY` `30 81 01 7F 00 00 00 00` (0x8130, ErrReg 0x01, lost node 127); `HB` → `7F`; `TPDO1`/`TPDO2` stop. RTT: `[FAILSAFE ERROR] Heartbeat lost from node 127 -> closing; stack forces PRE-OP`, `[FAILSAFE] Forced CLOSED (master heartbeat lost): relay ON -> OFF, pos 2 -> 1`, `NMT change: 0x7F`, `NMT not Operational (0x7F) -> DISABLED` |
| 3 | `SDO-REQ` `40 42 60 00 00 00 00 00` | `4F 42 60 00 01 00 00 00` — ValveState **1 = Closed**, not the stale 2 |
| 4 | `SDO-REQ` `40 41 60 00 00 00 00 00` | `4B 41 60 00 00 00 00 00` — DISABLED |
| 5 | `SDO-REQ` `40 40 60 00 00 00 00 00` | `4B 40 60 00 00 00 00 00` — **ControlWord zeroed** (the buffered 0x0009 is gone) |
| 6 | `SDO-REQ` `40 01 10 00 00 00 00 00` | `4F 01 10 00 01 00 00 00` — stack-set bit 0 |

The close happens *inside* the stack's heartbeat check, before PRE-OP, so
on the trace the relay is off before the EMCY frame (scope the relay pin
if you want the number; ≤ 1 loop iteration).

## H. Resume does NOT reopen; a fresh Open does

Continue from G with the valve closed in PRE-OP.

| Step | Do | Expect |
|---|---|---|
| 1 | **restart the 0x77F producer** | on its first frame: `EMCY` `00 00 00 00 00 00 00 00` (stack recovery); SDO `40 01 10 00 …` → `4F 01 10 00 00 00 00 00` (firmware released bit 0 — HEARTBEAT_RESTORED path, no ControlWord reset needed); still PRE-OP (`HB` `7F`); valve still closed. RTT `Heartbeat restored from node 127 (valve stays CLOSED until NMT start + Open)` |
| 2 | 0x000 `01 NN`, send **no RPDO** | `HB` `05`; `TPDO1` `01 06 01` (IDLE, **closed**); `TPDO2` `00`; relay stays OFF for as long as you wait. RTT `NMT Operational -> IDLE (CW at entry 0x0000)` |
| 3 | `RPDO1` `09 00` | relay ON, `TPDO1` `02 06 02` — any Open frame after recovery is a fresh command (the module's CW went 0 → 9 even if the master's shadow never changed) |
| 4 | (variant) in step 2 send `09 00` **before** `01 NN` | nothing: RPDOs are not applied in PRE-OP. Then `01 NN` → closed/idle; a second `09 00` opens |
| 5 | (variant) stop the producer while already PRE-OP and closed | same 0x8130, no relay change (already off); recovery as step 1 |
| 6 | (variant) power-cycle with the producer off | no EMCY, no trip: `HB` `7F` only; then `01 NN` + `09 00` works with no master HB at all — **this is the accepted residual risk (plan D6)**; the MIK must produce HB before opening anything |

Optional regression proof on the OLD image (`e8e8faf…`, Context "pre-3.0.0"):
it has no consumer, so G never trips. The same bug is visible via NMT:
`2F 00 21 00 02 00 00 00` (FailSafe = CLOSED), `01 NN`, `09 00` (open),
`80 NN` (relay off, PRE-OP), `01 NN` → **the valve re-opens by itself** (SDO
0x6042 → 2, no RPDO sent). On 3.0.0 the same sequence (section J) leaves it
closed.

## I. 0x1001 both homes at every step

At every point in G/H/K/L: SDO `40 01 10 00 00 00 00 00` data byte **==**
the last `TPDO2` byte (in OP) — never the gateway's old "TPDO2 says 0, SDO
says 1". `TPDO2` is COS-triggered, so a change of 0x1001 produces a `TPDO2`
within 1 ms of the change while in OP.

After an EMCY, the pre-defined error field (the gateway said reads failed —
check):

| Send `SDO-REQ` | Expect `SDO-RSP` | Meaning |
|---|---|---|
| `40 03 10 00 00 00 00 00` | `4F 03 10 00 01 00 00 00` | 1 entry after one 0x8130 (grows with each EMCY, max 4) |
| `40 03 10 01 00 00 00 00` | `43 03 10 01 30 81 7F 00` | newest entry: code 0x8130, info = node 127 |
| `2F 03 10 00 00 00 00 00` | `60 03 10 00 00 00 00 00` | write 0 clears the history; sub0 → `00` |

If these abort on both valves *and* on a phtemp, file it as a stack-wide
item (plan §3 caveat), not a valve bug.

## J. Every NMT exit closes the valve

From OP and OPEN each time (`01 NN`, `0A 00`, `09 00` to re-arm between rows).

| Send 0x000 (DLC 2) | Expect | Meaning |
|---|---|---|
| `80 NN` | relay OFF; `HB` `7F`; TPDOs stop; SDO 0x6042 → `01`, 0x6041 → `00 00`, 0x6040 → `00 00`; RTT `Forced CLOSED (NMT left Operational)` | PRE-OP |
| `01 NN` | `HB` `05`; `TPDO1` `01 06 01` — **stays closed** (no RPDO sent) | start |
| `09 00` on `RPDO1` | opens | fresh edge |
| `02 NN` | relay OFF; `HB` `04`; SDO does **not** answer (STOPPED) | STOP |
| `01 NN` | closed/idle; then `09 00` opens | |
| `82 NN` | relay OFF; `HB` `00` then `7F`; SDO 0x1016:01 → `C4 88 00 00` again (consumer re-armed on comm reset); `TPDO2` DLC 1 after the next `01 NN` | reset communication |
| `81 NN` | same as `82` plus the RTT banner (`Firmware: 3.0.0`) | reset node |

Gateway consequence to tell the FE: a master restart that resets the bus now
closes an open valve. That is the intended semantics.

## K. Stack warning (TX-buffer overflow, EMCY 0x6100) — must NOT close the valve

This is the "recurring 0x6100 / MSEF 48 xx" seen on the deployed valve,
decoded (plan §1.2): `MCOUSER_FatalError(0x4820)` = TPDO lost because the
CAN TX FIFO (32 deep) was full. Plan D3/D4: warning class → **log only**.

Provocation on the bench, best effort: valve in OP and OPEN, then **unplug
the CAN cable at the valve** (keep it powered; the stub stays terminated on
the valve side if your fixture has the resistor there) for ~5 s and plug it
back in. With no ACK the FIFO fills in ~3 s (TPDO1 every 100 ms + TPDO2 + HB).

| COB-ID | DLC | Data | Expect |
|---|---|---|---|
| burst on re-plug | — | queued frames | the FIFO drains; possibly several |
| `EMCY` | 8 | `00 61 01 48 20 00 00 00` (and/or `48 30`, `48 40`, `48 10`) | 0x6100, ErrReg 0x01, MSEF = stack code 0x4820 TPDO / 0x4830 SDO / 0x4840 HB / 0x4810 EMCY lost |
| `TPDO1` | 3 | `02 06 02` | **valve still OPEN**, still OP (`HB` `05`) — nothing closed, no SW bit 3 |
| `TPDO2` | 1 | `01` | bit 0 set by the stack; SDO 0x1001 → `01` (both homes agree) |

RTT: `[CAN ERROR] MCO warning 0x4820 (EMCY 0x6100, TX-overflow class) - no valve action`.

Then section F: `00 00`, `80 00` on `RPDO1` → `EMCY` `00 00 00 00 00 00 00 00`,
SDO 0x1001 → `00`, `TPDO2` `00`, **valve still open**.

If the unplug does not provoke it (the FDCAN may go bus-off first and
recover silently), inject instead per `PUMP_FAULT_INJECTION_PLAYBOOK.md`
technique: non-halting gdb `call MCOUSER_FatalError(0x4820)` — expect
exactly the rows above. **Record every 0x6100 you see with timestamps and
what the master was doing**: the field occurrence happens with a live,
ACKing gateway and is still unexplained.

## L. Fatal stack error → close, then reset (optional, gdb)

`call MCOUSER_FatalError(0x8010)` (any code ≥ 0x8000): RTT
`[FAILSAFE ERROR] MCO fatal error 0x8010 -> closing (stack reset follows)`,
relay OFF, `EMCY` `00 61 01 80 10 00 00 00`, then ~10 ms later the stack
resets the MCU → `HB` `00`, `7F`, banner. Not worth a bench slot unless
something in K looked wrong.

## M. IWDG recovery

Stall the main loop (non-halting poke of a spin, or simply halt the core
with the probe for > 2 s and resume): the IWDG resets the chip. Expect `HB`
`00` then `7F`, relay OFF, RTT banner with
`*** IWDG RESET DETECTED - previous main loop stall; valve held CLOSED ***`.
If the valve was open before the stall it is closed afterwards and stays
closed until `01 NN` + `09 00`.

## N. Both valves on the bus together

Producer 0x77F on. Both nodes powered.

| Step | Send | Expect |
|---|---|---|
| 1 | 0x000 DLC 2 `01 00` (start all) | `HB` 0x708 and 0x709 → `05`; `TPDO1` on 0x188 **and** 0x189 (`01 06 01`), `TPDO2` on 0x288 and 0x289 (`00`), no collisions |
| 2 | 0x208 `09 00` | only node 8's relay clicks; 0x188 → `02 06 02`; 0x189 stays `01 06 01` |
| 3 | 0x209 `09 00` | node 9 opens too |
| 4 | identity on 0x608 and 0x609 | each answers on its own 0x588 / 0x589 with the section-B bytes (same serial 0x12345678 on both — the gateway must keep its serial check off, it does by default) |
| 5 | **stop the producer** | within 2.5 s **both** relays off; `EMCY` on 0x088 **and** 0x089 `30 81 01 7F 00 00 00 00`; both `HB` → `7F` |
| 6 | producer on; `01 00` | both `01 06 01`, both closed; 0x208 `09 00` opens node 8 only |

Nodes 8 and 9 are adjacent IDs — check the trace decodes 0x188 vs 0x189 and
0x288 vs 0x289 as separate nodes.

## Pass bar — "functioning as normal" for this build

- A through E pass on both nodes; every SDO reply matches the table;
  `TPDO1` always DLC 3, `TPDO2` always **DLC 1** (never 0); 0x2222:23 aborts.
- G: relay off and `EMCY` 0x8130 within 2.5 s of the last master HB; SDO
  0x6042 = 1, 0x6040 = 0 afterwards.
- H: after NMT start with no RPDO the valve **stays closed** for as long as you
  watch; one `09 00` opens it; 0x1001 back to 0 on recovery with no reset.
- I: SDO 0x1001 == `TPDO2` byte at every step.
- J: all four NMT exits close the valve; none of the restarts re-opens it.
- K: a 0x6100/0x48xx EMCY leaves the valve open and in OP; reset (F) clears
  0x1001 to 0 with one `EMCY` 0x0000.
- N: both valves trip together on HB loss, each on its own EMCY COB-ID.
- RTT 1 s `[VALVE] State=… Err=0x..` line shows `Err=` equal to the SDO read
  of 0x1001 and `CW=0x0000` after every forced close.
- Baseline (Phase 0, old image, before flashing 3.0.0): note whether `TPDO2`
  came out DLC 0, and — with the **gateway** as master, not CANopen Magic —
  whether the boot sequence contains an SDO write to 0x1016:01 (plan D9).
