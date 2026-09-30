# pH/Temp fw 4.0.0 (OD regen #2) — CANopen Magic bench playbook, nodes 4 / 5 / 31 / 32

Frames are written the way CANopen Magic's transmit list and trace show them:
**COB-ID, DLC, raw data bytes** (hex, exactly as on the wire = little endian
for multi-byte values). Bus 250 kbit/s. Record the whole session; name
traces `cantrace-phtemp-nNN-<test>.csv`.

Every section below is written once. Where a row says `SDO-REQ`, `TPDO1`,
`NN` etc., substitute from the node table. All four nodes run the **same
image apart from the node ID** (`phtemp-nNN-250k.bin`, identical except for
three bytes), so expected data bytes are identical across nodes.

Logging is **SEGGER RTT over SWD** (no serial adapter), non-halting:

```
probe-rs attach --chip STM32G431KBTx build/phtemp-nNN-250k.elf
```

## Node table

| Symbol | meaning | DLC | node 4 | node 5 | node 31 | node 32 |
|---|---|---|---|---|---|---|
| `NN` | node byte in NMT frames | — | `04` | `05` | `1F` | `20` |
| `SDO-REQ` | SDO request, you → node (0x600+N) | 8 | 0x604 | 0x605 | 0x61F | 0x620 |
| `SDO-RSP` | SDO response, node → you (0x580+N) | 8 | 0x584 | 0x585 | 0x59F | 0x5A0 |
| `RPDO1` | ControlWord, you → node (0x200+N) | 2 | 0x204 | 0x205 | 0x21F | 0x220 |
| `TPDO1` | `mV×10 lo hi, temp×10 lo hi` (0x180+N) | 4 | 0x184 | 0x185 | 0x19F | 0x1A0 |
| `TPDO2` | `SensorStatus, pHQual, TempQual, ErrReg` (0x280+N) | 4 | 0x284 | 0x285 | 0x29F | 0x2A0 |
| `EMCY` | emergency (0x080+N) | 8 | 0x084 | 0x085 | 0x09F | 0x0A0 |
| `HB` | heartbeat (0x700+N) | 1 | 0x704 | 0x705 | 0x71F | 0x720 |
| NMT | you → all, always 0x000 | 2 | — | — | — | — |
| image | `build/phtemp-nNN-250k.bin` | | n04 | n05 | n31 | n32 |
| banner | RTT `Node ID:` | | 0x04 | 0x05 | 0x1F | 0x20 |

SDO reply byte 0: `4F` = 1 data byte, `4B` = 2, `47` = 3, `43` = 4, `60` =
write accepted, `80` = abort with the code in bytes 4-7 (LE). MicroCANopen
aborts a read of a non-existent object with **0x08000000** (`00 00 00 08`),
not the CiA 0x06020000 — any abort is a pass.

**Validated so far (node 31):** A, B, C, D, E, H1, H2-by-quality, and the pH
7/4/10 switching in F — traces `cantrace-testing-pHtemp-refactor.csv` and
`-refactor2.csv`, 2026-09-30. After OD regen #2 (thresholds + dead comm
objects removed) re-run **C** and **E** at minimum.

---

## A. Power-on → PRE-OPERATIONAL

Nothing to send. Expect:

| COB-ID | DLC | Data | Meaning |
|---|---|---|---|
| `HB` | 1 | `00` | boot-up |
| `HB` | 1 | `7F` every 1000 ms | heartbeat, PRE-OP |
| `TPDO1` / `TPDO2` | — | none | TPDOs are OP-only |

RTT: `Firmware: 4.0.0`, `Node ID: 0x..` per the node table, both `Init … OK`.
A probe missing at boot produces an EMCY here too (section H3/H4).

## B. Identity (works in PRE-OP)

| Send `SDO-REQ` (DLC 8) | Expect `SDO-RSP` (DLC 8) | Meaning |
|---|---|---|
| `40 00 10 00 00 00 00 00` | `43 00 10 00 04 04 00 00` | device type 0x00000404 |
| `40 18 10 01 00 00 00 00` | `43 18 10 01 23 01 00 00` | vendor 0x123 |
| `40 18 10 02 00 00 00 00` | `43 18 10 02 04 00 00 00` | product 4 = phtemp |
| `40 18 10 03 00 00 00 00` | `43 18 10 03 00 00 02 00` | **revision 0x00020000** (gateway checks this at boot) |
| `40 18 10 04 00 00 00 00` | `43 18 10 04 78 56 34 12` | serial 0x12345678 — from `MCOUSER_GetSerial()`, same on every unit |
| `40 0A 10 00 00 00 00 00` | `47 0A 10 00 31 2E 30 00` | sw version string "1.0" (known placeholder) |
| `40 17 10 00 00 00 00 00` | `4B 17 10 00 E8 03 00 00` | producer heartbeat 1000 ms |
| `40 03 18 00 00 00 00 00` | `4B 00 18 03 F4 01 00 00` | TPDO1 inhibit 500 × 100 µs = 50 ms |
| `40 05 18 00 00 00 00 00` | `4B 00 18 05 E8 03 00 00` | TPDO1 event timer 1000 ms |

(Careful: SDO index bytes are `lo hi sub` — 0x1800:03 is `00 18 03`.)

## C. Deleted objects must abort

| Send `SDO-REQ` | Expect `SDO-RSP` | Object |
|---|---|---|
| `40 00 60 00 00 00 00 00` | `80 00 60 00 00 00 00 08` | 0x6000 pHValue (regen #1) |
| `40 00 22 00 00 00 00 00` | `80 00 22 00 00 00 00 08` | 0x2200 pHCalibrationCommand (regen #1) |
| `40 10 22 00 00 00 00 00` | `80 10 22 00 00 00 00 08` | 0x2210 TempOffset (regen #1) |
| `40 00 24 00 00 00 00 00` | `80 00 24 00 00 00 00 08` | **0x2400 MillivoltDeltaThreshold (regen #2)** |
| `40 01 24 00 00 00 00 00` | `80 01 24 00 00 00 00 08` | 0x2401 TempDeltaThreshold (regen #2) |
| `40 02 24 00 00 00 00 00` | `80 02 24 00 00 00 00 08` | 0x2402 StatusDeltaThreshold (regen #2) |
| `40 10 10 01 00 00 00 00` | `80 10 10 01 00 00 00 08` | 0x1010 Store Parameters (regen #2) |
| `40 06 10 00 00 00 00 00` | `80 06 10 00 00 00 00 08` | 0x1006 Comm Cycle Period (regen #2) |
| `40 22 22 23 00 00 00 00` | `80 22 22 23 00 00 00 08` | 0x2222:23 vendor demo — must be an abort, NOT a string |

Still present and must answer: `40 16 10 01 …` (0x1016:01) → `43 16 10 01 …`,
`40 28 10 01 …` (0x1028:01) → `43 28 10 01 …`.

## D. NMT start → RUNNING

| Send 0x000 (DLC 2) | Meaning |
|---|---|
| `01 NN` | NMT start this node |

Expect:

| COB-ID | DLC | Data | Meaning |
|---|---|---|---|
| `HB` | 1 | `05` | OPERATIONAL |
| `TPDO2` | 4 | `03 qq qq 00` | within 1 ms: SensorStatus 03, pH quality, temp quality, ErrReg 00 |
| `TPDO1` | 4 | `mm mm tt tt` | within 1 ms, then one per new sample (~400–500 ms) and at least every 1000 ms |

`TPDO2` repeats every ≤5000 ms and on any SensorStatus/quality change.

| Send `SDO-REQ` | Expect `SDO-RSP` | Meaning |
|---|---|---|
| `40 41 60 00 00 00 00 00` (in first 2 s) | `4B 41 60 00 43 02 00 00` | StatusWord 0x0243: ready bits + warming-up + remote |
| `40 41 60 00 00 00 00 00` (after 2 s) | `4B 41 60 00 03 02 00 00` | 0x0203 |
| `40 00 23 00 00 00 00 00` | `4F 00 23 00 03 00 00 00` | SensorStatus: electrode + temp OK |
| `40 01 10 00 00 00 00 00` | `4F 01 10 00 00 00 00 00` | ErrorRegister 0 |

## E. Live measurement objects

| Send `SDO-REQ` | Expect `SDO-RSP` | Meaning |
|---|---|---|
| `40 03 60 00 00 00 00 00` | `4B 03 60 00 xx xx 00 00` | mV×10; must equal `TPDO1` bytes 0-1 |
| `40 10 60 00 00 00 00 00` | `4B 10 60 00 xx xx 00 00` | °C×10 (20.8 °C = `D0 00`); must equal `TPDO1` bytes 2-3; hand-warm → rises |
| `40 01 60 00 00 00 00 00` | `4F 01 60 00 qq 00 00 00` | pH quality 0-100 (≥ 0x5A settled) |
| `40 11 60 00 00 00 00 00` | `4F 11 60 00 64 00 00 00` | temp quality 100 (CRC OK) |
| `40 02 60 00 00 00 00 00` | `4F 02 60 00 02 00 00 00` | pH driver state 2 = READY |
| `40 12 60 00 00 00 00 00` | `4F 12 60 00 0x 00 00 00` | temp driver state 1/2/3 cycling |

Reference levels with the probe emulator (measured node 31, 2026-09-30,
≈73 mV per pH at the ADC):

| Emulator | 0x6003 (mV×10) | `TPDO1` bytes 0-1 |
|---|---|---|
| pH 4 | ≈ 12340 | `34 30` |
| pH 7 | ≈ 10170 | `BA 27` |
| pH 10 | ≈ 7960 | `18 1F` |

Switching levels: the new value appears on `TPDO1` within one sample; pH
quality on `TPDO2` byte 1 dips to `0A` for a few seconds while the input
settles, then returns to ≥ `5A`.

## F. TPDO1 cadence (standard comm params; write in PRE-OP)

No threshold objects exist. The stack sends `TPDO1` on every process-image
change (one per ~400 ms electrode sample when RUNNING) subject to the
inhibit time, and at least every event-timer period. To cap the rate:

| Step | Send | Expect |
|---|---|---|
| 1 | 0x000 DLC 2 `80 NN` | `HB` `7F` — PRE-OP |
| 2 | `SDO-REQ` `2B 00 18 03 10 27 00 00` | `SDO-RSP` `60 00 18 03 00 00 00 00` — inhibit = 10000 → 1 s |
| 3 | `SDO-REQ` `40 00 18 03 00 00 00 00` | `SDO-RSP` `4B 00 18 03 10 27 00 00` |
| 4 | 0x000 DLC 2 `01 NN` | `HB` `05`; `TPDO1` now ≤ 1 Hz |
| 5 | 0x000 `80 NN`, then `SDO-REQ` `2B 00 18 03 F4 01 00 00` | `60 00 18 03 …` — restore 50 ms |

Power-cycle → back to 500 (no persistence). Pass bar at the default: **one
`TPDO1` per new sample, never two within 50 ms with different data** (the
2026-09-30 duplicate bug is fixed and re-verified in trace 2).

## G. ControlWord fault reset (edge-triggered)

Rising edge on bit 7; a repeated `84` is a no-op. RPDO1 is event-driven, no
SYNC needed. Send both, in order:

| Send `RPDO1` (DLC 2) | Meaning |
|---|---|
| `04 00` | bit 7 low |
| `84 00` | bit 7 rising edge → reset |

By SDO instead: `SDO-REQ` `2B 40 60 00 04 00 00 00` then `2B 40 60 00 84 00 00 00`.

With nothing faulted: no EMCY (nothing to clear); the state machine re-runs
DISABLED → WARMING_UP → RUNNING (StatusWord 0x0243 for 2 s, then 0x0203).

## H. Single sensor fault — module keeps running

### H1. Temp probe unplugged while RUNNING (verified byte-exact, node 31)

| COB-ID | DLC | Data | Expect |
|---|---|---|---|
| `EMCY` | 8 | `00 50 21 02 04 00 00 00` | within ~1 s: 0x5000, ErrReg 0x21, sensor 2 = temp, state 4 = ERROR |
| `TPDO2` | 4 | `01 qq 64 00` then `01 qq 00 21` | two frames 500 ms apart (driver passes through a retry state before ERROR); the second is the settled one |
| `TPDO1` | 4 | `mm mm tt tt` | keeps coming; temp bytes frozen at last good value, mV live |

| Send `SDO-REQ` | Expect `SDO-RSP` | Meaning |
|---|---|---|
| `40 01 10 00 00 00 00 00` | `4F 01 10 00 21 00 00 00` | **SDO 0x1001 == `TPDO2` byte 3** |
| `40 41 60 00 00 00 00 00` | `4B 41 60 00 21 02 00 00` | 0x0221: temp-fault bit 5, pH ready, remote |

Re-plug → nothing changes (errors latch). Send G:

| COB-ID | DLC | Data | Meaning |
|---|---|---|---|
| `EMCY` | 8 | `00 00 00 00 00 00 00 00` | error reset |
| `TPDO2` | 4 | `03 qq qq 00` | after the 2 s warm-up; SDO 0x6041 → `03 02` |

### H2. pH ADC removed (pull the pH-2 Click) while RUNNING

| COB-ID | DLC | Data | Expect |
|---|---|---|---|
| `EMCY` | 8 | `00 50 21 01 03 00 00 00` | within ~0.5 s: sensor 1 = pH, state 3 = ERROR |
| SDO 0x6041 | | `4B 41 60 00 12 02 00 00` | 0x0212: pH-fault bit 4, temp ready, remote |

**Unplugging only the electrode is a different case and produces NO EMCY —
by design.** The ADC still answers; the module cannot tell an open electrode
from a reading. Verified: mV drifts (not to 0), pH quality on `TPDO2` byte 1
falls to `0A` within 500 ms and recovers on re-plug. That is the host-side
disconnect signal.

### H3. Boot with the temp probe unplugged

| Step | COB-ID | DLC | Data | Expect |
|---|---|---|---|---|
| power on | `EMCY` | 8 | `00 50 21 02 04 00 00 00` | right after boot-up, still PRE-OP |
| send 0x000 | | 2 | `01 NN` | RUNNING on pH alone: StatusWord 0x0221, `TPDO1` flowing |
| plug in, send `RPDO1` | | 2 | `04 00` then `84 00` | init retried: EMCY `00 00 …`, StatusWord 0x0203, temp live |

### H4. Boot with the pH ADC absent

Same as H3 with MSEF `01 03`; module runs on temp alone (StatusWord 0x0212).
Old firmware faulted here (warm-up race) — must now reach RUNNING.

## I. Both sensors faulted → FAULT

Unplug both while RUNNING. Expect:

| COB-ID | DLC | Data | Meaning |
|---|---|---|---|
| `EMCY` | 8 | `00 50 21 01 03 00 00 00` | pH (order of the two 0x5000 may vary) |
| `EMCY` | 8 | `00 50 21 02 04 00 00 00` | temp |
| `EMCY` | 8 | `00 FF 21 02 03 04 00 00` | **0xFF00, cause 2 = both channels faulted**; MSEF = pH state, temp state |
| `TPDO1` | 4 | stale | event timer still emits the last values |

| Send `SDO-REQ` | Expect `SDO-RSP` | Meaning |
|---|---|---|
| `40 41 60 00 00 00 00 00` | `4B 41 60 00 38 02 00 00` | 0x0238: fault + pH-fault + temp-fault + remote |
| `40 00 23 00 00 00 00 00` | `4F 00 23 00 08 00 00 00` | SensorStatus: fault bit only |
| `40 01 10 00 00 00 00 00` | `4F 01 10 00 21 00 00 00` | ErrReg 0x21 |

Reset (G) with both still unplugged: `EMCY` `00 00 …`, then both 0x5000
again, then after 2 s `00 FF 21 01 03 04 00 00` (cause 1 = nothing ready
after warm-up). Reset with both plugged in: `00 00 …` only, then RUNNING.

## J. NMT stop / pre-op / reset

| Send 0x000 (DLC 2) | Expect | Meaning |
|---|---|---|
| `80 NN` | `HB` `7F`; TPDOs stop; SDO 0x6041 → `03 00`; SDO still answers | PRE-OP |
| `02 NN` | `HB` `04`; SDO does **not** answer | STOPPED |
| `01 NN` | back to section D | start |
| `81 NN` | `HB` `00` then `7F`; 0x1800:03 back to `F4 01` | reset node |

## K. Several units on the bus together

| Send 0x000 (DLC 2) | Expect |
|---|---|
| `01 00` (start all) | every `HB` in the node table → `05`; each node's `TPDO1`/`TPDO2` on its own COB-IDs, no collisions |
| identity read on each `SDO-REQ` | each `SDO-RSP` `43 18 10 03 00 00 02 00` |

Nodes 4 and 5 are adjacent IDs — check the trace decodes 0x184 vs 0x185 and
0x284 vs 0x285 as separate nodes. All units share serial 0x12345678, so the
gateway must keep its serial check disabled (it does by default).

## L. Master heartbeat lost / restored

The firmware arms a heartbeat consumer on the master, **node 127, 2500 ms**
(same as the pump). It locks on the first heartbeat it sees, so nothing
trips at boot or on a bench with no master. On loss the **stack** sends
EMCY 0x8130 and forces PRE-OP; the firmware only logs it — no FAULT, no
ControlWord reset needed afterwards.

Setup: CANopen Magic's heartbeat producer as **node 127 (0x77F), 1000 ms**
must be running before step 1.

| Step | Do | Expect |
|---|---|---|
| 1 | producer on; 0x000 DLC 2 `01 NN` | OP, TPDOs flowing (section D) |
| 2 | **stop the 0x77F producer** | within 2.5 s: `EMCY` `30 81 01 7F 00 00 00 00` (0x8130, ErrReg 0x01, lost node 127); `HB` → `7F`; `TPDO1`/`TPDO2` stop; SDO `40 01 10 00 …` → `4F 01 10 00 01 00 00 00`; RTT `Heartbeat lost: node=127 -> PRE-OP (stack)` |
| 3 | **restart the producer** | on its first frame: `EMCY` `00 00 00 00 00 00 00 00` (stack pushes EMCY_NO_ERROR after the firmware has released ErrReg bit 0 — `mcop.c` recovery path); SDO 0x1001 → `00`; still PRE-OP (`HB` `7F`); RTT `Heartbeat restored: node=127` |
| 4 | 0x000 `01 NN` | OP, StatusWord 0x0243 for 2 s, then 0x0203, TPDOs resume — **no `RPDO1` reset needed** |
| 5 | (variant) stop the producer while already PRE-OP | same 0x8130, node stays PRE-OP; recovery as step 3 |
| 6 | (variant) power-cycle with the producer off | no EMCY, no trip: `HB` `7F` only; consumer waits for the first 127 heartbeat |

If a probe was also faulted at the time of the loss, ErrReg stays 0x21 after
recovery (bit 5 is the app's) — only the stack's bit 0 is released.

## Pass bar — "functioning as normal" for this build

- A through E pass on every node under test; every SDO reply matches the table.
- C: 0x2400 and the other regen-#2 deletions abort; 0x1016/0x1028 still answer.
- F: one `TPDO1` per new sample, never two within 50 ms with different data.
- H: SDO 0x1001 equals `TPDO2` byte 3 at every step; exactly one 0x5000 EMCY
  per fault episode.
- I: exactly one 0xFF00 per FAULT entry, one 0x0000 per accepted reset.
- L: 0x8130 within 2.5 s of the master's last heartbeat, node in PRE-OP,
  0x1001 back to 0 on recovery, RUNNING again after a plain NMT start.
- No frame on `TPDO1`/`TPDO2` with DLC other than 4; nothing answers on 0x6000.
- RTT `DIAG:` lines every 5 s show `ER=` equal to the SDO read of 0x1001.
