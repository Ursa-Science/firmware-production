# Valve module fw 3.0.0 — FE brief (gateway + MIK), 2026-10-01

**State:** firmware-production `main` @ 8b088c8 (behaviour) + the OD regen
that follows it (revision 0x00020000, 21 objects). Bench-validated on
**node 9** with CANopen Magic (open/close, heartbeat loss, every NMT exit,
resume-without-reopen). You will get **two valves, nodes 8 and 9**, same
image apart from the node ID. Full detail: `docs/VALVE_REFACTOR_PLAN.md`
(decisions), `docs/VALVE_BENCH_TEST.md` (frames + expected replies).

## 1. What the module is now

An on/off solenoid valve (Clippard EV-2M-24, normally closed) behind a
relay, with no position sensor. "Open" means the relay is energised;
"closed" means it is not. The module reports what it commanded, settled
after a fixed 50 ms. Nothing survives a module reset.

| | value |
|---|---|
| Identity | vendor 0x123, product 5, **revision 0x00020000** (the gateway checks this at boot — old EDS ⇒ node rejected) |
| RPDO1 (0x200+N), 2 B | `0x6040 ControlWord` U16 |
| TPDO1 (0x180+N), **3 B** | `0x6041 StatusWord` U16, `0x6042 ValveState` U8 — on every change and at least every 100 ms |
| TPDO2 (0x280+N), **1 B** | `0x1001 ErrorRegister` — on every change and at least every 1000 ms |
| SDO-only | 0x2000 LEDControl (unchanged), identity, comm params |
| Heartbeat | 1000 ms producer; **consumes the master's (node 127) heartbeat, 2.5 s timeout** |
| Units | **node 8** (0x188/0x288/0x208/0x608/0x708) and **node 9** (0x189/0x289/0x209/0x609/0x709); one OD for both |

## 2. How to control it

Commands are **rising edges** on ControlWord bits, with bit 3 (Enable
Operation) held as an interlock. The node must be OPERATIONAL (RPDOs are
ignored in PRE-OP).

| Send on RPDO1 (LE) | CW | does |
|---|---|---|
| `09 00` | 0x0009 | **Open** — bit 0 went 0→1 with bit 3 set |
| `0A 00` | 0x000A | **Close** — bit 1 went 0→1 with bit 3 set; also drops bit 0 so the next `09 00` is an edge |
| `00 00` | 0x0000 | nothing on its own; clears bits so the next command is an edge |
| `80 00` | 0x0080 | **Error reset** — bit 7 went 0→1 (see §4) |

Rules that follow from "edge":
- Sending `09 00` while the ControlWord is already 0x0009 does nothing.
  Alternate Open/Close (`09 00` / `0A 00`), or send `00 00` between repeats.
- Close beats Open if both edges arrive in the same frame.
- After **any** restart of the node (NMT start after PRE-OP/STOP/reset, after
  a heartbeat loss, after power-up) the valve is **closed** and whatever
  ControlWord you last sent is forgotten. To open again send a fresh `09 00`.
  Re-sending your last value is fine — from the module's side it is a new
  edge — but do not assume the old state carried over.

What you get back, on TPDO1 within 1 ms of each change:

| StatusWord | ValveState | meaning |
|---|---|---|
| 0x0000 | 1 | DISABLED — node not OPERATIONAL |
| 0x0601 | 1 | idle, **closed**, target reached, remote |
| 0x0204 | 3 | moving (≤ 50 ms after a command) |
| 0x0602 | 2 | idle, **open**, target reached, remote |
| 0x0200 | 0 | idle, position unknown — only after a Halt (bit 8) mid-motion; not used by the MIK |

StatusWord bits: 0 closed, 1 opened, 2 moving, 3 fault (never set on this
firmware, see §4), 9 remote (OPERATIONAL), 10 target reached.

## 3. Fail-safe — the valve closes itself

The valve goes to **closed** (relay off) and stays there, reporting
ValveState 1, whenever:

- **your heartbeat stops for 2.5 s** — the stack sends EMCY 0x8130 and drops
  the node to PRE-OP; the relay is already off before that EMCY leaves;
- the node leaves OPERATIONAL for any reason — NMT PRE-OP, STOP, reset
  communication, reset node;
- the module's stack hits a fatal error (it then resets itself);
- power-up or watchdog reset (relay pin defaults low).

It **never re-opens by itself.** The old "FailSafePosition" object (0x2100,
default *as-is*, gateway enum inverted) is gone; closed is hard-wired. If
you need fail-open on some future variant that is a firmware change, not a
configuration.

Consequences for the MIK:
- **Keep your 1 s heartbeat (node 127) alive while any valve may be open.**
  A gap over 2.5 s closes it. The consumer locks on the first heartbeat it
  sees, so a master that never produces one is unprotected — always start
  the heartbeat before the first NMT start.
- A gateway restart that resets the bus closes an open valve. Expected.
- Recovery from a heartbeat loss is: heartbeat back → NMT start → `09 00`.
  No fault reset involved.
- The module's red LED **double-flashes** from the loss until the next
  master heartbeat is seen (CiA 303-3 ERR indicator, owned by the stack).
  NMT start/PRE-OP/STOP and a ControlWord reset do not clear it; the next
  0x77F frame or a reset-communication does. It is an indication, not a
  latched fault.

## 4. Errors, EMCYs, and how to reset

0x1001 now agrees between SDO reads and TPDO2 (the "TPDO2 says 0, SDO says
1" disagreement in `overrides.yml` is fixed). Only bit 0 is ever used; the
module has no sensor that could set bits 1–5.

| EMCY | who | when | MSEF |
|---|---|---|---|
| 0x8130 | stack | **your heartbeat stopped** for 2.5 s — valve closed, node PRE-OP, TPDOs stop, 0x1001 bit 0 set | `7F` |
| 0x0000 | stack | your heartbeat is back; 0x1001 bit 0 released by the firmware | — |
| 0x0000 | stack | at every boot / reset communication ("no error") | — |
| 0x0000 | app | ControlWord bit-7 reset accepted while 0x1001 was non-zero | — |
| 0x6100 | stack | **CAN transmit FIFO overflow** — the recurring one on the deployed valve, now decoded | `48 20` TPDO lost / `48 30` SDO lost / `48 40` heartbeat lost / `48 10` EMCY lost |

**There is no FAULT state on this firmware.** StatusWord bit 3 and the bit-7
reset are kept for contract stability, but nothing enters FAULT: the old
motion-timeout fault (0x2300) was unreachable and is deleted, and 0x6100 is
a bus symptom, not a valve fault — the valve keeps running and stays open
through one. Treat 0x6100 as a diagnostics event and tell us when you see
it with the gateway live; the field occurrence is still unexplained.

### Resetting 0x1001 after a heartbeat loss

Nothing to reset. When your heartbeat is back the firmware clears bit 0
and the stack sends EMCY 0x0000. Then NMT start (`01 09` / `01 08` on
0x000) and, if you want it open, `09 00`.

### Resetting 0x1001 by command (e.g. after a 0x6100)

Only works in OPERATIONAL (the ControlWord is not processed in PRE-OP):

| COB-ID | DLC | data | |
|---|---|---|---|
| 0x000 | 2 | `01 09` | NMT start, if not already OP |
| 0x209 | 2 | `00 00` | bit 7 low |
| 0x209 | 2 | `80 00` | bit 7 rising edge |
| 0x089 | 8 | `00 00 00 00 00 00 00 00` | expect EMCY 0x0000 (only if something was set) |
| 0x289 | 1 | `00` | expect TPDO2 within 1 ms |

A reset does not move the valve. Verify with SDO 0x609 `40 01 10 00 00 00
00 00` → 0x589 `4F 01 10 00 00 00 00 00`. Node 8: 0x208 / 0x088 / 0x288 /
0x608 / 0x588 and `01 08`.

## 5. Deleted objects — nothing may read or write these

| Index | was | why |
|---|---|---|
| **0x2100 FailSafePosition** | runtime fail-safe choice | hard-wired closed (§3) |
| **0x2101 ManualOverride** (EDS typo "ManaualOverride") | never read | dead |
| **0x2300 MotionTimeout** | stuck-valve fault bound, gateway wrote 1 s at boot | fault path was unreachable; 50 ms settle is fixed |
| 0x1006 / 0x1007 / 0x1019 | SYNC objects | no synchronous PDOs |
| 0x1020 | verify configuration | no persistence |
| 0x2222 | EmSA demo SDO strings | removed (was never in the EDS) |

SDO access to any of them aborts with 0x08000000. 0x1016, 0x1017, 0x1003,
0x2000 still exist. **Deleting the boot-time `sdo: 0x2300` write in
`overrides.yml` is mandatory**: an SDO write to a missing object aborts
and fails the node's boot sequence.

## 6. What you need to change

### Gateway (`CANOpenGateway`) — one window, because the master checks 1018:03

1. **Copy the new EDS + DCF** from
   `firmware-production/devices/valve/MCO_CiA401__User/EDS/ValveModule-n09-250kbs.{eds,dcf}`
   over `bridge/devices/ValveModule-n09-250kbs.{eds,dcf}` (same names you
   already use; the repo's old space-named files are gone).
2. **Node 8 needs its own pair.** Copy the n09 pair to
   `ValveModule-n08-250kbs.{eds,dcf}` and edit the DCF
   `[DeviceCommissioning] NodeID=0x08`. gen-network resolves the node from
   the DCF NodeID first, the filename second.
3. **`bridge/devices/overrides.yml`**, `ValveModule-n09-250kbs` block (and a
   twin for n08):
   - **delete the whole `sdo:` list** (the `0x2300 = 1` write);
   - delete the `"2300:0"`, `"2100:0"`, `"2101:0"` entries (the merge uses
     `setdefault`, so a stale key creates a phantom object instead of failing);
   - rewrite the two device-level notes: 0x6100 is decoded (TX-FIFO
     overflow, MSEF `48 xx`), and the 1001 TPDO/SDO disagreement is fixed;
   - keep the `6042:0` and `6040:0` enums — they are still right.
4. Tests/docs that pin the old objects: `bridge/test/test_gen_network.py`
   (asserts `2300:0`), `bridge/gen-network.py` and
   `api/test/server-routes.test.ts` (0x6100 note + `2300:0` fixture),
   `docs/device-management.md`, `docs/plans/agent-hardware-context.md`.
5. `cd bridge && make`, rebuild the image, restart, confirm nodes 8 and 9
   boot with no 1018:03 mismatch and no SDO abort in the boot sequence, and
   that `devices.json` for both has no `2100`, `2101`, `2300`.
6. **Please capture one boot trace with the gateway as master** and send it:
   we need to see whether dcfgen writes 0x1016:01 at boot (plan D9 — a write
   of 0 there would silently disarm the valve's heartbeat consumer).

### MIK

1. Open/Close as edges with bit 3 set (§2); after any node restart or
   heartbeat loss, re-send Open explicitly — do not assume the valve kept
   state.
2. Heartbeat contract: 1 s heartbeat on node 127 whenever a valve may be
   open; start it before the first NMT start.
3. Handle 0x8130 as "we went quiet; the valve is closed", then NMT start
   when you are back. Do not send a fault reset for it.
4. Drop every read/write of 0x2100 and 0x2300. There is no fail-safe
   option and no motion timeout to configure.
5. Decode ValveState 1/2/3 from TPDO1 byte 2 (unchanged) and 0x1001 bit 0
   from TPDO2 (now trustworthy). Treat 0x6100 as diagnostics, not a fault.
6. Two valves = two node IDs (8 and 9). Both report serial 0x12345678, so
   the serial cannot tell them apart.

## 7. Bench check after your changes

From `docs/VALVE_BENCH_TEST.md`, node 8 and node 9 columns: B (identity,
revision `00 00 02 00`), C (deletions abort), D/E (TPDOs, open/close), G/H
(heartbeat loss → closed → recovery → NMT start → stays closed → Open
opens), N (both units together). Plus the gateway boot trace for D9.
