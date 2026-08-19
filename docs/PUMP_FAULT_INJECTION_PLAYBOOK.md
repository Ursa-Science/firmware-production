# Pump Fault-Injection Playbook

Complete map of every point where the pump firmware emits error information
onto the CAN bus (EMCY + TPDO + SDO), and bench commands to inject each
condition. Written 2026-08-13 (pre-Phase-2 baseline), analysis at commit
`bb64c23`. Companion to `PUMP_FAULT_FEEDBACK_PLAN.md`.

**ADDRESS FRESHNESS:** all RAM addresses below are DWARF-derived from
`build/pump.elf` at commit `bb64c23`. Any rebuild can move them — re-derive
before use (build has 1-byte enums; NEVER hand-compute offsets):

```
arm-none-eabi-gdb build/pump.elf --batch \
  -ex "print/x &'stepper.c'::stepper.current_position" \
  -ex "print/x &'stepper.c'::stepper_pending_events" \
  -ex "print/x &'motor_control.c'::motor_ctrl.fault_active" \
  -ex "print/x &'motor_control.c'::motor_ctrl.error_register" \
  -ex "print/x &gMCOConfig.error_register"
```

Current values: `stepper.current_position` 0x20001634;
`stepper_pending_events` 0x20001628; `motor_ctrl.fault_active` 0x200015E2;
`motor_ctrl.error_register` 0x200015E3; `gMCOConfig.error_register`
0x20001C51; procimg ErrorRegister byte = gProcImg(0x20000064)+0x09 =
0x2000006D.

**Bench rules (hard-won 2026-08-12):** one probe owner at a time (kill RTT
attach / gdb server before probe-rs read/write); `probe-rs write/read` are
no-halt and IWDG-safe — interactive gdb halts are NOT (~2 s budget unless
DBGMCU_APB1FZR1 bit12 set); fault reset is edge-triggered → always the
two-step `202#04 00 00 00` then `202#84 00 00 00`; StatusWord bit 4 after
reset is the stale-build tell (must be 0).

---

## 1. Emitter map — where error info reaches the bus

### Continuous state piping (every SYNC while OPERATIONAL)

| Object | PDO | Source variable | Fault encoding |
|---|---|---|---|
| StatusWord 0x6041 | TPDO1 0x182 | derived in `MotorControl_GenerateStatusWord()` | 0x0008 pattern when `fault_active \|\| stepper_status.error`; bit 4 = driver energized; bit 9 = OP |
| PumpState 0x2400 | TPDO2 0x282 | `motor_ctrl.pump_state` | 0x05 FAULT (override while `fault_active`) |
| ErrorRegister 0x1001 | TPDO2 0x282 | `motor_ctrl.error_register` → procimg | cause-coded bits (below) |
| ErrorRegister 0x1001 | SDO + EMCY byte 2 | `gMCOConfig.error_register` | second home — synced by EnterFault/reset since Phase 1 |

### Application-layer emitters (motor_control.c, user_STM32.c)

| # | Condition | Detection | EMCY (code + mfr bytes) | ER 0x1001 | Injectable? |
|---|---|---|---|---|---|
| A1 | Stepper position runaway (\|pos\| ≥ 2e9) | TIM1 ISR → `STEPPER_EVT_ERROR` → `EnterFault(false)` | `21 71 ER 00 00 00 00 03` (0x7121) | 0x01 | **YES — cmd 2.1 (validated)** |
| A2 | TMC DIAG latched: short to GND/VS | PA5 EXTI + level confirm → `EnterFault(true)`, DRV_STATUS S2G*/S2VS* | `10 23 ER <DRV_STATUS LE> 01` (0x2310) | 0x03 | NO — needs real chip fault |
| A3 | TMC DIAG latched: overtemp | same, DRV_STATUS OT/OTPW | `10 42 ER <DRV_STATUS LE> 02` (0x4210) | 0x09 | NO — needs real chip fault |
| A4 | TMC DIAG latched: no cause flag | same, healthy DRV_STATUS | `00 10 ER <DRV_STATUS LE> 06` (0x1000) | 0x01 | **YES — cmd 2.4** |
| A5 | TMC DIAG latched: DRV_STATUS unreadable | same, UART read fails | `00 10 ER 00 00 00 00 05` (0x1000) | 0x11 | **YES — cmd 2.5** |
| A6 | DIAG transient pulse (not latched) | level confirm reads low | — none (RTT log + counter only) | — | (side effect of 2.4 done wrong) |
| A7 | Fault reset (CW bit 7 rising) | `HandleFaultReset` | `00 00 00 00 00 00 00 00` | cleared (motor bits) | **YES — 04→84 two-step** |
| A8 | MotorControl_Init fail at COMM_RESET | `MCO_EVENT_COMM_RESET` handler | — **no EMCY**; `gMCOConfig.error_register \|= 0x20` only | 0x20 SDO-side only | not practical |
| A9 | Stack fatal error (TX buffer overflow ERROFL_*, HW init ERRFT_*) | `MCOUSER_FatalError` (user_STM32.c) | `00 61 ER <code hi> <code lo> 0 0 0` (0x6100) + possible app reset | bit0 via PushEMCY | not practical (queue overflow) |
| A10 | Master heartbeat lost | stack consumer → `MCO_EVENT_HEARTBEAT_LOST` → `EmergencyStop()` | stack sends `30 81 ER <node> 00 00 00 00` (0x8130) | bit0 via PushEMCY | **YES — cmd 2.6 (CAN-side)** |

A10 caveats: consumer is UNARMED at boot (0x1016 blank, INITHBCONSUMER_CALLS
empty) — the whole path is dead until the master SDO-writes 0x1016 (volatile,
lost on reset). And `EmergencyStop()` sets PumpState=STOPPED, NOT FAULT — the
motor stop itself is invisible in StatusWord/PumpState; only the stack EMCY
0x8130 announces it. Assess in dose-strip planning (runaway failsafe UX).

### Stack-layer emitters (shared/mco — same code all 3 devices)

| # | Condition | EMCY | Auto-clears? | Injectable? |
|---|---|---|---|---|
| S1 | RPDO received with wrong DLC | `10 82 ER <pdo#lo> <pdo#hi> <want> <got> 00` (0x8210) | yes — EMCY 0x0000 on next good RPDO | **YES — cmd 2.7 (CAN-side)** |
| S2 | Invalid NMT command byte | `00 82 ER <cmd> 00 00 00 00` (0x8200) | yes — on next valid NMT | **YES — cmd 2.8 (CAN-side)** |
| S3 | SYNC with wrong length | 0x8240 | yes | CAN-side (send `080` with DLC>0) — unverified |
| S4 | Boot / comm reset | `00 00 ...` (EMCY 0x0000 announce) | n/a | power cycle (seen in every boot trace) |

NOTE: `MCOP_PushEMCY` sets gMCOConfig ER bit 0 for ANY nonzero code — after
any stack EMCY (S1/S2/S3), SDO reads of 0x1001 return ≥0x01 until a fault
reset or NMT reset clears it. Expected, not a bug.

---

## 2. Injection commands

All probe-rs commands are no-halt (safe with IWDG). Precondition for all:
node OPERATIONAL (`000#01 02`), SYNC producer running, Magic recording.

### 2.1 Stepper runaway → EMCY 0x7121, source 3 — VALIDATED 2026-08-12

Motor must be jogging (`202#07 00 0A 00` then `202#0F 00 0A 00`):

```
probe-rs write --chip STM32G431KBTx b32 0x20001634 0x773593F6
```

Expect ≤50 ms: EMCY `082#21 71 01 00 00 00 00 03`, motor stops, TPDOs
`182#08 02 00 00` / `282#05 01 00 00 00 00`, SDO 0x1001 → 0x01.
Cleanup: `04`→`84` reset, then **zero the position or power cycle** (else
instant re-fault on next start): `probe-rs write --chip STM32G431KBTx b32
0x20001634 0x00000000`.

### 2.2 Reporting-only fault flag (negative test: SW/PumpState WITHOUT ER/EMCY)

```
probe-rs write --chip STM32G431KBTx b8 0x200015E2 0x01
```

Expect: TPDO1 `08 02 00 00`, TPDO2 `05 00 ...` — **ER byte stays 0x00 and
NO EMCY fires** (bypasses EnterFault by design). Proves the TPDO piping is
independent of the cause machinery. Cleanup: `04`→`84`.

### 2.3 Stack-home ER poke (dual-source regression check)

```
probe-rs write --chip STM32G431KBTx b8 0x20001C51 0x02
```

Expect: SDO read 0x1001 (`602#40 01 10 00 ...`) returns **0x02** while TPDO2
ER byte stays 0x00 and no fault is active. Confirms which home the SDO
server reads. Cleanup: `04`→`84` (0x02 is a motor-owned bit) or write 0x00.

### 2.4 DIAG-latched entry, healthy chip → EMCY 0x1000, source 6 (DIAG_UNKNOWN)

⚠ **Drives PA5 as an output against the TMC's DIAG output — contention.**
Prefer motor-power rail off if the board allows; otherwise keep the window
short and restore immediately. Motor idle. MODER baseline is 0xAA9652AE
(verify first — abort if different):

```
probe-rs read  --chip STM32G431KBTx b32 0x48000000 1
probe-rs write --chip STM32G431KBTx b32 0x48000000 0xAA9656AE
probe-rs write --chip STM32G431KBTx b32 0x48000018 0x00000020
probe-rs write --chip STM32G431KBTx b32 0x20001628 0x00000008
```

(PA5→output, drive high, then post `STEPPER_EVT_DRV_FAULT`; level-confirm
reads high → `EnterFault(true)` → healthy DRV_STATUS → source 6.)
Expect: EMCY `082#00 10 01 xx xx xx xx 06` (real DRV_STATUS snapshot in
bytes 3-6), fault TPDOs, driver disabled. **Restore immediately:**

```
probe-rs write --chip STM32G431KBTx b32 0x48000000 0xAA9652AE
```

then `04`→`84` reset. (Writing the event word directly, not OR — fine while
idle; don't do this mid-motion.)

### 2.5 TMC comm-fail branch → EMCY 0x1000, source 5, ER 0x11

Same as 2.4 but kill USART2 first so the DRV_STATUS read times out
(USART2 CR1 @ 0x40004400, clear UE bit 0):

```
probe-rs read  --chip STM32G431KBTx b32 0x40004400 1        # note value V
probe-rs write --chip STM32G431KBTx b32 0x40004400 <V & ~1>
```

then the three 2.4 writes. Expect EMCY `082#00 10 11 00 00 00 00 05`
(ER 0x11 = generic+communication, zero snapshot). Cleanup: restore CR1 to V,
restore MODER, `04`→`84`, then **power cycle** before trusting TMC comms
again.

### 2.6 Heartbeat-lost runaway stop (CAN-side; doubles as dose-plan Q3 test)

1. Arm the consumer for master node 0x7F, 1000 ms (SDO write 0x1016 sub1 =
   0x007F03E8): `602#23 16 10 01 E8 03 7F 00`
2. Transmit master heartbeat `77F#05` cyclic at 500 ms. Start the motor
   jogging.
3. Stop the heartbeat transmission.

Expect within ~1 s: EMCY `082#30 81 01 7F 00 00 00 00` (0x8130) from the
stack + motor stops. **Observe: PumpState goes STOPPED not FAULT — no
fault-state latch.** Consumer arming is volatile (lost on node reset).

### 2.7 RPDO length error (CAN-side)

Send RPDO1 with wrong DLC, e.g. `202#0F 00` (DLC 2, expected 4).
Expect: EMCY `082#10 82 01 01 00 04 02 00` (PDO 1, want 4, got 2); the
short frame is NOT processed (no motor action). Auto-clear: next correct
4-byte RPDO1 → EMCY `00 00 ...`.

### 2.8 Invalid NMT command (CAN-side)

Send `000#FF 02` (0xFF is not a valid NMT command).
Expect: EMCY `082#00 82 01 FF 00 00 00 00` (0x8200 + offending byte).
Auto-clear on next valid NMT command.

---

## 3. Coverage summary vs the emitter map

Injectable on the bench: A1 (validated), A4, A5, A7 (validated), A10, S1,
S2, S3(unverified), S4. Not injectable without real hardware faults: A2/A3
(need a genuine TMC short/overtemp — DRV_STATUS lives in the chip), A8, A9.
The A2/A3 EMCY *transport* is shared with A4/A5 (same EnterFault code path,
same frame layout) — only the classification branch differs, and that logic
is 6 lines of bit-tests. Residual risk accepted.
