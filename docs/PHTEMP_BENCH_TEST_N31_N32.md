# pH/Temp fw 4.0.0 — CANopen Magic bench script, nodes 31 + 32

Frames are `COB-ID#bytes` (hex), the format used in the pump playbook; paste
the ID and data into CANopen Magic's transmit list. Bus 250 kbit/s. Record
the whole session; name traces `cantrace-phtemp-n31-<test>.csv` / `-n32-`.

Logging is now **SEGGER RTT over SWD** (no serial adapter). To watch a unit
while it runs (does not halt the core; IWDG keeps running):

```
probe-rs attach --chip STM32G431KBTx build/phtemp-n31-250k.elf
```

## COB-IDs

| | node 31 (0x1F) | node 32 (0x20) |
|---|---|---|
| SDO request (you → node) | `61F` | `620` |
| SDO response | `59F` | `5A0` |
| RPDO1 ControlWord (you → node) | `21F` | `220` |
| TPDO1 `[mV×10 lo hi \| temp×10 lo hi]` | `19F` | `1A0` |
| TPDO2 `[SensorStatus \| pHQual \| TempQual \| ErrReg]` | `29F` | `2A0` |
| EMCY | `09F` | `0A0` |
| Heartbeat | `71F` | `720` |

Everything below is written for node 31. For node 32 substitute the IDs
above and the node byte `20` in NMT commands. SDO data is little-endian.

SDO reply prefixes: `4F` = 1 byte, `4B` = 2 bytes, `43` = 4 bytes, `47` =
3 bytes, `60` = write OK, `80 … 00 00 02 06` = abort "object does not exist".

---

## A. Power-on (PRE-OPERATIONAL)

Expect, unprompted:

| Frame | Meaning |
|---|---|
| `71F#00` | boot-up |
| `71F#7F` every 1000 ms | heartbeat, PRE-OP |
| no `19F` / `29F` | TPDOs are OP-only |
| RTT: `Firmware: 4.0.0`, `Node ID: 0x1F` | banner |

If a probe is missing at boot you also get an EMCY here (see H).

## B. Identity (works in PRE-OP)

| Send | Expect |
|---|---|
| `61F#40 00 10 00 00 00 00 00` | `59F#43 00 10 00 04 04 00 00` — device type 0x0404 |
| `61F#40 18 10 01 00 00 00 00` | `59F#43 18 10 01 23 01 00 00` — vendor 0x123 |
| `61F#40 18 10 02 00 00 00 00` | `59F#43 18 10 02 04 00 00 00` — product 4 (phtemp) |
| `61F#40 18 10 03 00 00 00 00` | `59F#43 18 10 03 00 00 02 00` — **revision 0x00020000** |
| `61F#40 18 10 04 00 00 00 00` | `59F#43 18 10 04 57 34 12 00` — serial (per-type constant) |
| `61F#40 0A 10 00 00 00 00 00` | `59F#47 0A 10 00 31 2E 30 00` — sw version string "1.0" (placeholder, known) |
| `61F#40 17 10 00 00 00 00 00` | `59F#4B 17 10 00 E8 03 00 00` — producer HB 1000 ms |

## C. Deleted objects must abort

| Send | Expect |
|---|---|
| `61F#40 00 60 00 00 00 00 00` (pHValue) | `59F#80 00 60 00 00 00 02 06` |
| `61F#40 00 22 00 00 00 00 00` (pHCalibrationCommand) | `59F#80 00 22 00 00 00 02 06` |
| `61F#40 10 22 00 00 00 00 00` (TempOffset) | `59F#80 10 22 00 00 00 02 06` |
| `61F#40 20 22 00 00 00 00 00` (pHElectrodeStatus) | `59F#80 20 22 00 00 00 02 06` |
| `61F#40 22 22 23 00 00 00 00` (vendor demo entry) | `59F#80 22 22 23 00 00 00 06` or `…02 06` — any abort; **not** a string |

## D. NMT start → RUNNING

| Send | Expect |
|---|---|
| `000#01 1F` | `71F#05` (OP); within 2 s: `29F#03 qq qq 00` then `19F#mm mm tt tt` |

`19F` repeats at ≤1000 ms (event timer) and on a delta; `29F` at ≤5000 ms
and on any SensorStatus bit change.

| Send | Expect |
|---|---|
| `61F#40 41 60 00 00 00 00 00` during the first 2 s | `59F#4B 41 60 00 43 02 00 00` — 0x0243: ready bits + warming-up (bit 6) + remote |
| same, after 2 s | `59F#4B 41 60 00 03 02 00 00` — 0x0203 |
| `61F#40 00 23 00 00 00 00 00` | `59F#4F 00 23 00 03 00 00 00` — SensorStatus: electrode + temp OK |
| `61F#40 01 10 00 00 00 00 00` | `59F#4F 01 10 00 00 00 00 00` — ErrorRegister 0 |

## E. Live measurement objects

| Send | Expect |
|---|---|
| `61F#40 03 60 00 00 00 00 00` | `59F#4B 03 60 00 xx xx 00 00` — mV×10. pH-7 buffer ≈ 10240 = `00 28`; must equal TPDO1 bytes 0-1 |
| `61F#40 10 60 00 00 00 00 00` | `59F#4B 10 60 00 xx xx 00 00` — °C×10 (22.0 °C = `DC 00`); must equal TPDO1 bytes 2-3; hand-warm the probe → rises |
| `61F#40 01 60 00 00 00 00 00` | `59F#4F 01 60 00 qq 00 00 00` — pH quality 0-100 (≥90 settled in buffer) |
| `61F#40 11 60 00 00 00 00 00` | `59F#4F 11 60 00 64 00 00 00` — temp quality 100 (CRC ok) |
| `61F#40 02 60 00 00 00 00 00` | `59F#4F 02 60 00 02 00 00 00` — pH driver state 2 = READY |
| `61F#40 12 60 00 00 00 00 00` | `59F#4F 12 60 00 0x …` — temp state 1/2/3 cycling (converting/reading/ready) |
| `61F#40 00 24 00 00 00 00 00` | `59F#4B 00 24 00 0A 00 00 00` — mV delta threshold 10 = 1.0 mV |
| `61F#40 01 24 00 00 00 00 00` | `59F#4B 01 24 00 05 00 00 00` — temp delta 0.5 °C |

## F. Config write (what the MIK does on connect)

| Send | Expect |
|---|---|
| `61F#2B 00 24 00 32 00 00 00` | `59F#60 00 24 00 00 00 00 00` — threshold = 50 (5.0 mV) |
| `61F#40 00 24 00 00 00 00 00` | `59F#4B 00 24 00 32 00 00 00` — reads back |
| `61F#2B 00 24 00 0A 00 00 00` | `59F#60 …` — restore 10 |

Delta trigger: with threshold 10, move the electrode pH 7 → pH 4 buffer.
Expect a `19F` within 500 ms of the change (inhibit time), not only at the
1 s event timer. Power-cycle → threshold is back to 10 (no persistence).

## G. ControlWord fault reset (edge-triggered)

Fault reset is a **rising edge on bit 7**; a repeated `84` is a no-op.
RPDO1 is event-driven (no SYNC needed):

```
21F#04 00        then        21F#84 00
```

or by SDO: `61F#2B 40 60 00 04 00 00 00` then `61F#2B 40 60 00 84 00 00 00`.

With nothing faulted this is silent (no EMCY 0x0000 — nothing to clear),
except the state machine runs DISABLED → WARMING_UP → RUNNING again
(StatusWord shows 0x0243 for 2 s).

## H. Single sensor fault (module keeps running)

**Temp probe unplugged while RUNNING:**

| Expect | Frame |
|---|---|
| EMCY within ~1 s | `09F#00 50 21 02 04 00 00 00` — 0x5000, ER 0x21, sensor 2 = temp, state 4 = ERROR |
| TPDO2 | `29F#01 qq 00 21` — temp-OK bit clear, temp quality 0, ErrReg 0x21 |
| `61F#40 01 10 00 …` | `59F#4F 01 10 00 21 …` — **SDO and TPDO2 agree** (the old dual-source bug) |
| `61F#40 41 60 00 …` | `59F#4B 41 60 00 21 02 00 00` — 0x0221: temp-fault bit 5, pH ready, remote |
| `19F` | keeps coming; temp bytes frozen at the last good value, mV live |

Re-plug the probe → nothing changes (errors latch). Send the G reset →
`09F#00 00 00 00 00 00 00 00` (EMCY 0x0000), 2 s warm-up, then 0x0203 /
`29F#03 …` / ErrReg 0.

**pH ADC removed (pull the pH-2 Click) while RUNNING:**

| Expect | Frame |
|---|---|
| EMCY within ~0.5 s | `09F#00 50 21 01 03 00 00 00` — sensor 1 = pH, state 3 = ERROR |
| `61F#40 41 60 00 …` | `59F#4B 41 60 00 12 02 00 00` — 0x0212: pH-fault bit 4, temp ready, remote |

**Boot with the temp probe unplugged (new in 4.0.0 — was silent before):**

| Expect | Frame |
|---|---|
| right after boot-up, still PRE-OP | `09F#00 50 21 02 04 00 00 00` |
| `000#01 1F` | module reaches RUNNING on pH alone (0x0221), TPDO1 flowing |
| plug in, then `21F#04 00` / `21F#84 00` | init retried: EMCY 0x0000, 0x0203, temp live |

**Boot with the pH ADC absent:** same pattern with `01 03`; module runs on
temp alone (0x0212). Old firmware faulted here (warm-up race).

## I. Both sensors faulted → FAULT

Unplug both while RUNNING:

| Expect | Frame |
|---|---|
| two EMCY 0x5000 (order varies) | `09F#00 50 21 01 03 …`, `09F#00 50 21 02 04 …` |
| EMCY 0xFF00, cause 2 | `09F#00 FF 21 02 03 04 00 00` — both channels faulted; MSEF = pH state, temp state |
| `61F#40 41 60 00 …` | `59F#4B 41 60 00 38 02 00 00` — 0x0238: fault + pH-fault + temp-fault + remote |
| `61F#40 00 23 00 …` | `59F#4F 00 23 00 08 …` — SensorStatus fault bit only |
| TPDO1 | stops (no delta triggering in FAULT); event timer still emits stale values |

Reset with both still unplugged: EMCY 0x0000, then both 0x5000 again, then
after 2 s `09F#00 FF 21 01 03 04 00 00` (cause 1 = nothing ready after
warm-up). Reset with both plugged in: EMCY 0x0000 → RUNNING.

## J. NMT stop / pre-op

| Send | Expect |
|---|---|
| `000#80 1F` (pre-op) | `71F#7F`, TPDOs stop; `6041` → 0x0003 (no remote bit); SDO still answers |
| `000#02 1F` (stop) | `71F#04`; SDO does **not** answer |
| `000#01 1F` | back to D |
| `000#81 1F` (reset node) | `71F#00` boot-up, then `7F`; thresholds back to defaults |

## K. Both units on the bus together

| Send | Expect |
|---|---|
| `000#01 00` (start all) | `71F#05` and `720#05`; `19F`/`1A0`, `29F`/`2A0` interleave with no overlap; both SDO channels answer independently |
| `61F#40 18 10 03 …` and `620#40 18 10 03 …` | both return `00 00 02 00` |

## What "functioning as normal" means for this build

- A, B, C, D, E pass on both nodes.
- H: SDO 0x1001 equals TPDO2 byte 3 at every step, and every fault episode
  produces exactly one 0x5000 EMCY.
- I: one 0xFF00 per FAULT entry, one 0x0000 per accepted reset.
- Nothing on `19F`/`29F` longer than 4 bytes; nothing answers on 0x6000.
- RTT shows `DIAG:` lines every 5 s with `ER=` matching the SDO read.
