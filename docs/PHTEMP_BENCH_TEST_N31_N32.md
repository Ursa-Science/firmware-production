# pH/Temp fw 4.0.0 — CANopen Magic bench playbook, nodes 31 + 32

Every frame is given the way CANopen Magic's transmit list and trace show
it: **COB-ID, DLC, raw data bytes (hex, exactly as on the wire = little
endian for multi-byte values)**. "Send" rows go into the transmit list;
"Expect" rows are what the trace must show. Bus 250 kbit/s. Record the whole
session; name traces `cantrace-phtemp-n31-<test>.csv` / `-n32-`.

Logging is **SEGGER RTT over SWD** — no serial adapter. To watch a unit while
it runs (non-halting; the IWDG keeps running):

```
probe-rs attach --chip STM32G431KBTx build/phtemp-n31-250k.elf
```

## COB-IDs

| Message | node 31 (0x1F) | node 32 (0x20) | DLC |
|---|---|---|---|
| SDO request (you → node) | 0x61F | 0x620 | 8 |
| SDO response (node → you) | 0x59F | 0x5A0 | 8 |
| RPDO1 ControlWord (you → node) | 0x21F | 0x220 | 2 |
| TPDO1 `mV×10 lo, hi, temp×10 lo, hi` | 0x19F | 0x1A0 | 4 |
| TPDO2 `SensorStatus, pHQual, TempQual, ErrReg` | 0x29F | 0x2A0 | 4 |
| EMCY | 0x09F | 0x0A0 | 8 |
| Heartbeat | 0x71F | 0x720 | 1 |
| NMT (you → all) | 0x000 | 0x000 | 2 |

The playbook is written for **node 31**. For node 32 use the IDs above and
node byte `20` instead of `1F` in NMT frames. SDO reply byte 0: `4F` = 1 data
byte, `4B` = 2, `47` = 3, `43` = 4, `60` = write accepted, `80` = abort
(bytes 4-7 = abort code LE). **MicroCANopen answers a read of a non-existent
object with 0x08000000 "general error" (`00 00 00 08`), not the CiA
0x06020000** — stack behaviour, same on valve and pump; any abort is a pass.

**Validated 2026-09-30, node 31** (`cantrace-testing-pHtemp-refactor.csv` +
RTT): sections A, B, C, D, E, H1, H2-by-quality pass. Two corrections from
that trace are folded in below (serial value, abort code). One firmware bug
found and fixed the same day: TPDO1 was sent twice per sample, stale value
then new value 50 ms apart — see the note under E.

---

## A. Power-on → PRE-OPERATIONAL

Nothing to send. Expect:

| COB-ID | DLC | Data | Meaning |
|---|---|---|---|
| 0x71F | 1 | `00` | boot-up |
| 0x71F | 1 | `7F` every 1000 ms | heartbeat, PRE-OP |
| 0x19F / 0x29F | — | none | TPDOs are OP-only |

RTT: `Firmware: 4.0.0`, `Node ID: 0x1F`. If a probe is missing at boot an
EMCY appears here too (see H).

## B. Identity (works in PRE-OP)

| Send COB-ID | DLC | Data | Expect COB-ID | DLC | Data | Meaning |
|---|---|---|---|---|---|---|
| 0x61F | 8 | `40 00 10 00 00 00 00 00` | 0x59F | 8 | `43 00 10 00 04 04 00 00` | device type 0x00000404 |
| 0x61F | 8 | `40 18 10 01 00 00 00 00` | 0x59F | 8 | `43 18 10 01 23 01 00 00` | vendor 0x123 |
| 0x61F | 8 | `40 18 10 02 00 00 00 00` | 0x59F | 8 | `43 18 10 02 04 00 00 00` | product 4 = phtemp |
| 0x61F | 8 | `40 18 10 03 00 00 00 00` | 0x59F | 8 | `43 18 10 03 00 00 02 00` | **revision 0x00020000** |
| 0x61F | 8 | `40 18 10 04 00 00 00 00` | 0x59F | 8 | `43 18 10 04 78 56 34 12` | serial 0x12345678 — served by the `MCOUSER_GetSerial()` callback in `user_STM32.c`, NOT the OD default 0x123457 (that is the hook for per-unit serials) |
| 0x61F | 8 | `40 0A 10 00 00 00 00 00` | 0x59F | 8 | `47 0A 10 00 31 2E 30 00` | sw version "1.0" (known placeholder) |
| 0x61F | 8 | `40 17 10 00 00 00 00 00` | 0x59F | 8 | `4B 17 10 00 E8 03 00 00` | producer heartbeat 1000 ms |

## C. Deleted objects must abort

| Send COB-ID | DLC | Data | Expect COB-ID | DLC | Data | Object |
|---|---|---|---|---|---|---|
| 0x61F | 8 | `40 00 60 00 00 00 00 00` | 0x59F | 8 | `80 00 60 00 00 00 00 08` | 0x6000 pHValue (verified 2026-09-30) |
| 0x61F | 8 | `40 00 22 00 00 00 00 00` | 0x59F | 8 | `80 00 22 00 00 00 00 08` | 0x2200 pHCalibrationCommand |
| 0x61F | 8 | `40 10 22 00 00 00 00 00` | 0x59F | 8 | `80 10 22 00 00 00 00 08` | 0x2210 TempOffset |
| 0x61F | 8 | `40 20 22 00 00 00 00 00` | 0x59F | 8 | `80 20 22 00 00 00 00 08` | 0x2220 pHElectrodeStatus |
| 0x61F | 8 | `40 22 22 23 00 00 00 00` | 0x59F | 8 | `80 22 22 23 00 00 00 08` | 0x2222:23 vendor demo — must be an abort, NOT a string |

## D. NMT start → RUNNING

| Send COB-ID | DLC | Data | Meaning |
|---|---|---|---|
| 0x000 | 2 | `01 1F` | NMT start node 31 |

Expect:

| COB-ID | DLC | Data | Meaning |
|---|---|---|---|
| 0x71F | 1 | `05` | OPERATIONAL |
| 0x29F | 4 | `03 qq qq 00` | within 2 s: SensorStatus 03, pH quality, temp quality, ErrReg 00 |
| 0x19F | 4 | `mm mm tt tt` | within 2 s, then every ≤1000 ms and on a delta |

`0x29F` repeats every ≤5000 ms and on any SensorStatus bit change.

| Send COB-ID | DLC | Data | Expect COB-ID | DLC | Data | Meaning |
|---|---|---|---|---|---|---|
| 0x61F | 8 | `40 41 60 00 00 00 00 00` (in first 2 s) | 0x59F | 8 | `4B 41 60 00 43 02 00 00` | StatusWord 0x0243: ready bits + warming-up + remote |
| 0x61F | 8 | `40 41 60 00 00 00 00 00` (after 2 s) | 0x59F | 8 | `4B 41 60 00 03 02 00 00` | 0x0203 |
| 0x61F | 8 | `40 00 23 00 00 00 00 00` | 0x59F | 8 | `4F 00 23 00 03 00 00 00` | SensorStatus: electrode + temp OK |
| 0x61F | 8 | `40 01 10 00 00 00 00 00` | 0x59F | 8 | `4F 01 10 00 00 00 00 00` | ErrorRegister 0 |

## E. Live measurement objects

| Send COB-ID | DLC | Data | Expect COB-ID | DLC | Data | Meaning |
|---|---|---|---|---|---|---|
| 0x61F | 8 | `40 03 60 00 00 00 00 00` | 0x59F | 8 | `4B 03 60 00 xx xx 00 00` | mV×10. pH-7 buffer ≈ 10240 = `00 28`. Must equal 0x19F bytes 0-1 |
| 0x61F | 8 | `40 10 60 00 00 00 00 00` | 0x59F | 8 | `4B 10 60 00 xx xx 00 00` | °C×10 (22.0 °C = `DC 00`). Must equal 0x19F bytes 2-3; hand-warm → rises |
| 0x61F | 8 | `40 01 60 00 00 00 00 00` | 0x59F | 8 | `4F 01 60 00 qq 00 00 00` | pH quality 0-100 (≥ 0x5A settled in buffer) |
| 0x61F | 8 | `40 11 60 00 00 00 00 00` | 0x59F | 8 | `4F 11 60 00 64 00 00 00` | temp quality 100 (CRC OK) |
| 0x61F | 8 | `40 02 60 00 00 00 00 00` | 0x59F | 8 | `4F 02 60 00 02 00 00 00` | pH driver state 2 = READY |
| 0x61F | 8 | `40 12 60 00 00 00 00 00` | 0x59F | 8 | `4F 12 60 00 0x 00 00 00` | temp driver state 1/2/3 cycling |
| 0x61F | 8 | `40 00 24 00 00 00 00 00` | 0x59F | 8 | `80 00 24 00 00 00 00 08` | 0x2400 deleted (after Architect regen #2; before it the object still answers `0A 00` but nothing reads it) |
| 0x61F | 8 | `40 00 18 03 00 00 00 00` | 0x59F | 8 | `4B 00 18 03 F4 01 00 00` | TPDO1 inhibit time 500 × 100 µs = 50 ms |
| 0x61F | 8 | `40 00 18 05 00 00 00 00` | 0x59F | 8 | `4B 00 18 05 E8 03 00 00` | TPDO1 event timer 1000 ms |

## F. TPDO1 cadence (standard comm params; do this in PRE-OP)

There are no threshold objects any more. The stack sends TPDO1 on every
process-image change (one per 400 ms electrode sample when RUNNING) subject
to the inhibit time, and at least every event-timer period. Cap the rate
with 0x1800:03 (units of 100 µs):

| Send COB-ID | DLC | Data | Expect COB-ID | DLC | Data | Meaning |
|---|---|---|---|---|---|---|
| 0x000 | 2 | `80 1F` | 0x71F `7F` | | | PRE-OP first (comm params) |
| 0x61F | 8 | `2B 00 18 03 10 27 00 00` | 0x59F | 8 | `60 00 18 03 00 00 00 00` | inhibit = 10000 → 1 s |
| 0x61F | 8 | `40 00 18 03 00 00 00 00` | 0x59F | 8 | `4B 00 18 03 10 27 00 00` | reads back |
| 0x000 | 2 | `01 1F` | 0x71F `05` | | | OP → `0x19F` now ≤ 1 Hz |
| 0x000 | 2 | `80 1F` then `2B 00 18 03 F4 01 00 00` | `60 00 18 03 …` | | | restore 500 (50 ms) |

Power-cycle → back to 500 (no persistence). With the default 50 ms inhibit,
expect **one `0x19F` per new sample**, never two within 50 ms carrying
different data (the 2026-09-30 duplicate bug is fixed), and one at least
every 1000 ms when the value is static.

## G. ControlWord fault reset (edge-triggered)

Fault reset is a **rising edge on bit 7**; a repeated `84` frame is a no-op.
RPDO1 is event-driven, no SYNC needed. Send both, in order:

| Send COB-ID | DLC | Data | Meaning |
|---|---|---|---|
| 0x21F | 2 | `04 00` | ControlWord 0x0004 (bit 7 low) |
| 0x21F | 2 | `84 00` | ControlWord 0x0084 (bit 7 rising edge) |

Same thing by SDO:

| Send COB-ID | DLC | Data |
|---|---|---|
| 0x61F | 8 | `2B 40 60 00 04 00 00 00` |
| 0x61F | 8 | `2B 40 60 00 84 00 00 00` |

With nothing faulted this produces **no** EMCY (nothing to clear); the state
machine just re-runs DISABLED → WARMING_UP → RUNNING (StatusWord 0x0243 for
2 s, then 0x0203).

## H. Single sensor fault — module keeps running

### H1. Temp probe unplugged while RUNNING

| COB-ID | DLC | Data | Expect |
|---|---|---|---|
| 0x09F | 8 | `00 50 21 02 04 00 00 00` | EMCY within ~1 s: code 0x5000, ErrReg 0x21, MSEF: sensor 2 = temp, state 4 = ERROR (**verified byte-exact 2026-09-30**) |
| 0x29F | 4 | `01 qq 64 00` then `01 qq 00 21` | two frames 500 ms apart (TPDO2 inhibit): the driver passes through a retry state before ERROR, so the first frame has the temp-OK bit clear while quality/ErrReg are still old. Cosmetic; the second frame is the settled one. |
| 0x19F | 4 | `mm mm tt tt` | keeps coming; temp bytes frozen at last good value, mV live (verified: temp stuck at `CE 00` = 20.6 °C) |

| Send COB-ID | DLC | Data | Expect COB-ID | DLC | Data | Meaning |
|---|---|---|---|---|---|---|
| 0x61F | 8 | `40 01 10 00 00 00 00 00` | 0x59F | 8 | `4F 01 10 00 21 00 00 00` | **SDO 0x1001 == TPDO2 byte 3** (the old dual-source bug is gone) |
| 0x61F | 8 | `40 41 60 00 00 00 00 00` | 0x59F | 8 | `4B 41 60 00 21 02 00 00` | 0x0221: temp-fault bit 5, pH ready, remote |

Re-plug the probe → nothing changes (errors latch). Send the G reset:

| COB-ID | DLC | Data | Expect |
|---|---|---|---|
| 0x09F | 8 | `00 00 00 00 00 00 00 00` | EMCY 0x0000 error reset |
| 0x29F | 4 | `03 qq qq 00` | after the 2 s warm-up |
| SDO 0x6041 | | `03 02` | 0x0203 again |

### H2. pH ADC removed (pull the pH-2 Click) while RUNNING

| COB-ID | DLC | Data | Expect |
|---|---|---|---|
| 0x09F | 8 | `00 50 21 01 03 00 00 00` | EMCY within ~0.5 s: sensor 1 = pH, state 3 = ERROR |
| SDO 0x6041 | | `4B 41 60 00 12 02 00 00` | 0x0212: pH-fault bit 4, temp ready, remote |

**Unplugging only the electrode (not the Click) is a different case and
produces NO EMCY — by design.** The ADC still answers on I2C, so nothing is
"faulted"; the module cannot tell an open electrode from a real reading.
Verified 2026-09-30: electrode out → mV drifted to ~917–957 (not 0) and
**pH quality on 0x29F byte 1 fell from 0x41 (65 %) to 0x0A (10 %) within
500 ms**; electrode back → mV ≈ 10140, quality back to 0x56. That quality
drop is the host-side disconnect signal the MIK is expected to use.

### H3. Boot with the temp probe unplugged (new in 4.0.0 — was silent)

| Step | COB-ID | DLC | Data | Expect |
|---|---|---|---|---|
| power on | 0x09F | 8 | `00 50 21 02 04 00 00 00` | right after boot-up, still PRE-OP |
| send | 0x000 | 2 | `01 1F` | module reaches RUNNING on pH alone: StatusWord 0x0221, 0x19F flowing |
| plug in, send | 0x21F | 2 | `04 00` then `84 00` | init retried: EMCY `00 00 …`, StatusWord 0x0203, temp live |

### H4. Boot with the pH ADC absent

Same as H3 with MSEF `01 03`; module runs on temp alone (StatusWord 0x0212).
Old firmware faulted here (warm-up race) — this must now reach RUNNING.

## I. Both sensors faulted → FAULT

Unplug both while RUNNING. Expect:

| COB-ID | DLC | Data | Meaning |
|---|---|---|---|
| 0x09F | 8 | `00 50 21 01 03 00 00 00` | pH sensor fault (order of the two 0x5000 may vary) |
| 0x09F | 8 | `00 50 21 02 04 00 00 00` | temp sensor fault |
| 0x09F | 8 | `00 FF 21 02 03 04 00 00` | **EMCY 0xFF00, cause 2 = both channels faulted**; MSEF = pH state, temp state |
| 0x19F | 4 | stale | no delta triggering in FAULT; event timer still emits the last values |

| Send COB-ID | DLC | Data | Expect COB-ID | DLC | Data | Meaning |
|---|---|---|---|---|---|---|
| 0x61F | 8 | `40 41 60 00 00 00 00 00` | 0x59F | 8 | `4B 41 60 00 38 02 00 00` | 0x0238: fault + pH-fault + temp-fault + remote |
| 0x61F | 8 | `40 00 23 00 00 00 00 00` | 0x59F | 8 | `4F 00 23 00 08 00 00 00` | SensorStatus: fault bit only |
| 0x61F | 8 | `40 01 10 00 00 00 00 00` | 0x59F | 8 | `4F 01 10 00 21 00 00 00` | ErrReg 0x21 |

Reset (G) with both still unplugged:

| COB-ID | DLC | Data | Meaning |
|---|---|---|---|
| 0x09F | 8 | `00 00 00 00 00 00 00 00` | reset accepted |
| 0x09F | 8 | `00 50 21 01 03 00 00 00` | re-announced |
| 0x09F | 8 | `00 50 21 02 04 00 00 00` | re-announced |
| 0x09F | 8 | `00 FF 21 01 03 04 00 00` | after 2 s: cause 1 = nothing ready after warm-up |

Reset with both plugged back in: `00 00 …` only, then RUNNING.

## J. NMT stop / pre-op / reset

| Send COB-ID | DLC | Data | Expect | Meaning |
|---|---|---|---|---|
| 0x000 | 2 | `80 1F` | 0x71F `7F`; TPDOs stop; SDO 0x6041 → `03 00` (0x0003, no remote bit); SDO still answers | enter PRE-OP |
| 0x000 | 2 | `02 1F` | 0x71F `04`; SDO does **not** answer | STOPPED |
| 0x000 | 2 | `01 1F` | back to section D | start |
| 0x000 | 2 | `81 1F` | 0x71F `00` then `7F`; 0x1800:03 back to `F4 01` | reset node |

## K. Both units on the bus together

| Send COB-ID | DLC | Data | Expect |
|---|---|---|---|
| 0x000 | 2 | `01 00` | 0x71F `05` **and** 0x720 `05`; 0x19F/0x1A0 and 0x29F/0x2A0 interleave with no collisions |
| 0x61F | 8 | `40 18 10 03 00 00 00 00` | 0x59F `43 18 10 03 00 00 02 00` |
| 0x620 | 8 | `40 18 10 03 00 00 00 00` | 0x5A0 `43 18 10 03 00 00 02 00` |

## Pass bar — "functioning as normal" for this build

- A through E pass on both nodes; every SDO reply matches the table.
- H: SDO 0x1001 equals 0x29F/0x2A0 byte 3 at every step; exactly one 0x5000
  EMCY per fault episode.
- I: exactly one 0xFF00 per FAULT entry and one 0x0000 per accepted reset.
- No frame on 0x19F/0x1A0 or 0x29F/0x2A0 with DLC other than 4; nothing
  answers on object 0x6000.
- RTT `DIAG:` lines every 5 s show `ER=` equal to the SDO read of 0x1001.
