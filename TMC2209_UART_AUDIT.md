# TMC2209 UART — Investigation & Audit (pump module)

Status: **OPEN.** Two distinct problems identified (one hardware, one firmware).
Last updated: 2026-08-07.

This documents a deep investigation into why the pump module's TMC2209 stepper
driver reports `CONFIG NOT VERIFIED / RX path dead` over UART. Firmware side was
audited end-to-end and is sound; the faults are (1) a hardware clamp on one board
and (2) a firmware regression introduced by the PA5 refactor.

---

## TL;DR

- **The firmware comms code is correct.** Driver, init, echo-drain, USART2 config,
  and IRQ handling were all audited. Not the cause.
- **Problem 1 — bad board (hardware):** PDN_UART is clamped at **0.60 V** (should
  idle ~2.76 V). Something sinks ~1 mA, overpowering a healthy 2.6 kΩ pull-up. No
  idle-high → no UART. Root cause is board-level (bad TMC module, PCB short, or a
  stuck-low MCU pin). **Not yet localized — do the module swap.**
- **Problem 2 — firmware regression `df4a262`:** flashing the "PA5 → EXTI
  fault-input" refactor onto a **known-good** board breaks its UART (`RX dead`)
  even though its bus is electrically fine (PDN idles HIGH). Only the firmware
  changed. The commit does **not** touch the UART driver, so it breaks UART via a
  side effect that is **not yet explained.** **Revert to confirm; then bisect.**

---

## System architecture — the root fragility

**Debug logging and the TMC2209 share ONE UART: USART2 on PA2/PA3.**

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

## Problem 1 — bad board: PDN clamped at 0.60 V (HARDWARE)

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

## Problem 2 — firmware regression in commit `df4a262` (FIRMWARE)

`df4a262 "Refactor: PA5 → EXTI fault-input change"`.

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
