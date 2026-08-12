# TMC2209 UART — Investigation & Audit (pump module)

Status: **CLOSED 2026-08-07.** Both problems resolved: one hardware (board
retired), one an observer artifact (there was never a firmware regression).
Preserved as reference for the architecture, evidence, and debug procedures.

This documents a deep investigation into why the pump module's TMC2209 stepper
driver reported `CONFIG NOT VERIFIED / RX path dead` over UART.

---

## TL;DR (final)

- **The firmware comms code is correct.** Driver, init, echo-drain, USART2 config,
  and IRQ handling were all audited. Never the cause.
- **Problem 1 — bad board (HARDWARE, board retired):** PDN_UART clamped at
  **0.60 V** (good board idles ~2.76 V); ~1 mA sink overpowering a healthy 2.6 kΩ
  pull-up → no idle-high → no UART. Localized to board level (MCU pin or GND
  short suspected; module swap never performed — board retired instead).
  Plausible origin: a 5 V-logic USB-UART adapter TX left connected to the PDN
  net pushes continuous clamp current through the TMC's PDN input (bench-measured
  3.5 V bus idle, above the 3.3 V rail) — exactly the damage signature seen.
- **Problem 2 — "df4a262 regression": RESOLVED, not a firmware bug.** The
  PA5 → DIAG/EXTI refactor was re-applied on a clean RTT-logging base
  (`86af79f`), flashed to the known-good board, power-cycled, and observed over
  RTT with **nothing attached to the UART**: `CONFIG VERIFIED BY CHIP`, GCONF/
  CHOPCONF readback MATCH, `MODER=0xAA9652AE` (PA5 input — refactor confirmed
  present), DIAG monitor armed healthy, motor runs at correct step rate.
  The original "regression" evidence was all read through the serial console —
  i.e. with a monitor electrically on the TMC bus (5 V push-pull adapter TX
  holding the open-drain bus high) and MCU-only warm resets (the TMC never
  power-cycles with the MCU). The observer was the fault.
- **Root fix (commit `28e2a3f`):** logging moved off USART2 entirely, to SEGGER
  RTT over SWD. The UART is now TMC-only; the observer-interference failure
  class is gone by construction. See "Logging architecture" below.

---

## Logging architecture — current (since `28e2a3f`, 2026-08-07)

**Logging is SEGGER RTT over SWD; USART2 belongs exclusively to the TMC2209.**

- `log.c` keeps the same `DBG_*`/`Log_Write` API but writes to RTT channel 0
  (`NO_BLOCK_SKIP`, 4 KB up-buffer, PRIMASK lock — see comments in
  `SEGGER_RTT_Conf.h` for why the SEGGER defaults were changed).
- View live with `probe-rs attach --chip STM32G431KBTx build/pump.elf`
  (ST-LINK), OpenOCD `rtt server`, or `JLinkRTTViewer` (J-Link). The first 4 KB
  of boot log survives in RAM even if the host attaches late; post-mortem via
  GDB at symbol `_SEGGER_RTT`.
- Bench rule that remains: **never connect a serial adapter's TX to PA2/PA3**,
  and never a 5 V-logic adapter at all — the net is the TMC bus, and 5 V TX
  drive both kills comms and can permanently damage the TMC PDN pin (Problem 1's
  likely origin). There is no longer any reason to attach a serial adapter.

## System architecture — the root fragility (historical, pre-`28e2a3f`)

**Debug logging and the TMC2209 shared ONE UART: USART2 on PA2/PA3.**

- Debug console: `Log_Init(&huart2)`, drained by the USART2 TXE ISR (`log.h`,
  `stm32g4xx_it.c::USART2_IRQHandler` → `Log_TxISR()`).
- TMC2209 datagrams: same USART2, direct register access (`tmc2209_uart.c`).
- Circuit (two-wire → one-wire, from `tmc2209_uart.c` header):
  ```
                     3V3
                      |
                   R12 (~2.6k measured) pull-up
                      |
    PA2 (TX, AF_OD) --[R7 1k]--+-- TMC2209 PDN_UART
    PA3 (RX, AF_OD) -----------+
  ```
- There is **no separate debug UART** (no free pins — confirmed with the board
  owner). So any serial monitor you attach to read logs is electrically on the
  TMC2209 bus. The `0x0F` (Shift-In) terminal-reset hack in `main.c` exists
  precisely because TMC datagram bytes leak onto the debug console.

Consequence proven on the bench: a serial adapter with its TX connected drives the
shared bus and both (a) corrupts logging and (b) can suppress TMC replies. Monitor
**RX-only** (adapter TX physically disconnected) if you must watch logs — but note
the deeper findings below made the monitor a secondary concern.

---

## Problem 1 — bad board: PDN clamped at 0.60 V (HARDWARE — board retired 2026-08-07)

Final status: clamp still present after all firmware fixes; confirmed pure
hardware (MCU pin or GND short suspected). Board retired without performing the
module swap. Evidence below kept for reference.

### Evidence
- Boot log: `CONFIG NOT VERIFIED ... no reply on addr 0-3 ... RX path dead`.
- Debugger read of `diag_data` (valid boot, `brr = 694 = 0x2B6`): **all four
  attempts fail at `echo_drain_ok = false`, `error = TMC2209_ERR_UART_TX`**, RX saw
  nothing (`isr_at_rx_fail = 0x600080`: TXE set, no RXNE/FE/ORE). The MCU cannot
  even receive the **echo of its own transmission** on the shared net — a pure
  PA2→PDN→PA3 loopback failure that does not depend on the TMC replying.
- **Multimeter: PDN idle = 0.60 V** on the bad board vs **2.76 V** on a known-good
  board. `R12 = 2.6 kΩ` (healthy, even stronger than the 5 k spec).
- Math: a 2.6 kΩ pull-up holding at 0.60 V means ~1 mA is being sunk to ground —
  an effective ~580 Ω pulldown / active clamp.

### Ruled out
- Pull-up / R12 (measured healthy).
- Firmware (same firmware verifies fine on the good board).
- DIAG(PA5)↔PDN(PA3) short (continuity = OPEN on the bad board).

### Remaining suspects (localize with the module swap)
1. **Bad TMC2209 module** clamping PDN low (top suspect — PA3/PDN connects directly
   to the chip, and 0.6 V fits a damaged/stuck PDN pin).
2. **PCB short** on the PDN net to ground.
3. **Stuck-low MCU pin** (PA2) — matches prior project history (a previous pump
   board's dead USART2_RX/PA3 silicon was fixed by an MCU swap).

### Next step (not done yet)
**Swap the TMC2209 module between the bad and good boards.** Fault follows the
module → bad chip. Fault stays with the board → PCB/MCU; then measure PA2 vs PA3
across R7 (PA2≈0 V = stuck-low TX; PA2≈PA3≈0.6 V = PDN short to ground).

---

## Problem 2 — suspected regression in commit `df4a262` (RESOLVED 2026-08-07: observer artifact, refactor is good)

`df4a262 "Refactor: PA5 → EXTI fault-input change"`.

**Resolution:** the refactor re-applied on the RTT base (`86af79f`) verifies
cleanly on the known-good board when observed over RTT with the UART untouched
(power-cycle, `CONFIG VERIFIED BY CHIP`, `MODER=0xAA9652AE`). The original
`RX dead` observations were made through the serial console — a monitor
electrically on the TMC bus (5 V push-pull TX) plus warm resets that never
reset the TMC. The sections below record the (now-explained) evidence.
The refactor is correct and kept: PA5 really is the module's DIAG output, and
the old output config fought the module's push-pull driver.

### What it changed
- PA5 was mislabeled `SPREAD_Pin` (A4988-era "MS3"). The PCB netlist actually
  routes the BTT module's **DIAG** output to PA5 (U8 pad 17). The old firmware
  wrongly **drove PA5 as an output** (fighting the module's push-pull DIAG).
- The refactor makes PA5 an **EXTI rising input with pulldown** (`GPIO_MODE_IT_RISING`),
  and arms the DIAG fault monitor in `main()` after TMC/motor init.
- It also edited `motor_control.c` (+62) and `stepper.c` (+40).
- It does **NOT** touch `tmc2209_uart.c` or the USART2 init.

### The regression
- **Known-good board + pre-`df4a262` firmware → `CONFIG VERIFIED`** (PDN 2.76 V).
- **Same board + `df4a262` firmware → `RX dead`**, but **PDN idles HIGH**
  (`IDR = 0xBD1D`, PA3 = 1) — so the bus is electrically fine. This is a firmware
  regression, a *different* failure mode than Problem 1's clamp.
- MODER confirms the flashed build: `0xAA9656AE` (PA5 output, working) →
  `0xAA9652AE` (PA5 input, broken).

### Why (NOT yet explained)
Making PA5 an input should not break PA2/PA3 UART. The break must be a side effect
of `df4a262` — candidates: the `motor_control.c`/`stepper.c` edits, or a DIAG-line
electrical interaction at boot (e.g., DIAG now free to be driven by the module vs.
previously forced low). **To be bisected hunk-by-hunk.**

### Next step
1. **Revert to confirm** (restores the reference board and proves the regression):
   ```bash
   git checkout 7db97b5 -- devices/pump   # build just before the PA5 refactor
   cmake --build --preset pump            # flash build/pump.bin to the good board
   git checkout HEAD -- devices/pump      # put the tree back afterward
   ```
2. Bisect `df4a262`'s hunks (PA5 GPIO vs motor_control vs stepper) to find the
   exact breaking change.

---

## Key implication — firmware and the PA5 change are coupled

The PA5 refactor breaks boards that don't expect it. Whatever the mechanism, the
lesson stands: the PA5/DIAG firmware and any matching board expectation must be
applied and versioned **together** across the fleet, or boards silently lose TMC
UART. Track which boards/firmware are paired.

---

## Confirmation channels (no free logging pin)

- **Serial console = the TMC bus.** Monitor RX-only (adapter TX disconnected) or
  you disturb TMC comms.
- **CAN/SDO readback is NOT wired.** Code comments cite OD `0x2500`/`0x2501`, but
  those objects **do not exist in the current EDS** (manufacturer objects stop at
  `0x2400`). The `gProcImg[0x67/0x68]` writes land at an unmapped offset — a latent
  bug; and adding real health objects needs CANopen Architect (Windows).
- **Debugger (SWD) is the clean read** — it does not touch PA2/PA3. Freeze the IWDG
  first or it resets you mid-halt:
  ```
  st-util                                   # terminal 1
  arm-none-eabi-gdb build/pump.elf -ex "target extended-remote :4242"
  (gdb) set *(unsigned int*)0xE0042008 = *(unsigned int*)0xE0042008 | 0x1000  # DBG_IWDG_STOP
  (gdb) break main.c:364
  (gdb) set *(unsigned int*)0xE000ED0C = 0x05FA0004    # SYSRESETREQ (monitor reset unsupported)
  (gdb) continue
  (gdb) print 'tmc2209_uart.c'::diag_data              # brr==0x2B6 proves a real boot
  ```
  Read `addr_found` (chip replied?), `config_verified`, `blind_fallback`, `brr`.

---

## Firmware audit result (reference)

Audited and found correct — none of this is the cause:
- USART2: 115200 8N1, oversampling-16, FIFO disabled, TX_RX; PA2/PA3 = AF7, AF_OD,
  pull-up — appropriate for the open-drain single-wire scheme.
- Driver: echo-drain on the two-wire circuit, IFCNT-verified writes, address
  discovery by IFCNT read, auto-baud preamble, push-pull-then-open-drain, watchdog
  kicking, ~9600 baud fallback. All correct.
- Boot init runs with `HAL_NVIC_DisableIRQ(USART2_IRQn)` before `Log_Init`, so the
  smoke test is interrupt-clean.
- **Latent (not the current bug):** runtime TMC reads (`ReadStallGuard`,
  `ReadDrvStatus`) do NOT disable the USART2 IRQ, so a live read while logging is
  active would collide. Config is boot-only today, so it hasn't bitten. Wrap
  runtime transactions in an IRQ-disable if live reads are ever added.
