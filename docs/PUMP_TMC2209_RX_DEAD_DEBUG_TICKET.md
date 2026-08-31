# Debug Ticket — Pump TMC2209 UART RX path dead (PDN idles at 1.46 V)

**Opened:** 2026-08-25
**Device:** pump (node 1), firmware-production build, FW 2.0.0 (`pump.bin`, OD revision 0x00020000)
**Severity:** hardware fault — UART config cannot be verified; motor still runs blind
**Status:** OPEN — awaiting module-vs-MCU localization test (§5)

---

## 1. Summary

The TMC2209 stopped answering UART reads. Boot smoke test (IFCNT read on addr 0–3, both
115200 and ~9600 baud) fails on every attempt; the driver falls back to **unverified blind
writes**. Root cause localized to an **electrical fault on the single-wire PDN_UART net: it
idles at 1.46 V, well below V_IH (~2.3 V)**, so the line can neither be framed by the MCU RX
nor arm the TMC's auto-baud. TX is fine and the motor still steps.

**This is not firmware.** The TMC init path (`tmc2209_uart.c`, `main.c` init call) was untouched
by the recent StepsRemaining / OD-revision work, and the *same binary* verified + ran SpreadCycle
in the CAN trace immediately prior. The firmware has **no NVM**, so a fault that survives a full
power-down cannot be firmware state — by elimination it is hardware.

## 2. Timeline

- Prior CAN trace (`cantrace-testing-pump-refactor.csv`): pump dosed correctly in SpreadCycle;
  UART verified. That session included **repeated abrupt de-energizations under load** — multiple
  `0x000B` quick-stops mid-dose at 800 steps/s, plus a heartbeat-loss e-stop.
- Afterward: TMC2209 unresponsive on UART. No wiring changed.
- Confirmed persistent across a **10-minute full power-down** (rails discharged) — reproduces
  **byte-for-byte identically** every boot (same BRR/ISR/IDR/MODER). Deterministic, not marginal.

## 3. Boot log evidence (12:58:26, post 10-min cold boot)

```
TMC2209 init: *** CONFIG NOT VERIFIED *** failed at step SMOKE_TEST (err=2)
  No TMC2209 answered an IFCNT read on addr 0-3 — RX path is dead or chip not in UART mode
MS1=0 MS2=0 -> addr=0x00 (pin-implied)
Addr probe: no reply on addr 0-3 (RX path dead)
Preamble TX: OK
USART2: BRR=0x02B6 CR1=0x0000000D CR3=0x00000000
  ISR@entry=0x006200E2 ISR@postPreamble=0x006000F8
  GPIOA IDR=0x0000F411 MODER=0xAA9652AE
  MODE: UNVERIFIED blind writes — verification failed on every baud
```

Decode:
- `err=2` = `TMC2209_ERR_UART_RX` (enum OK=0, UART_TX=1, **UART_RX=2**) → read timed out; no reply.
- `CR1=0x0D` = UE+RE+TE (RX enabled), `CR3=0x00` = full-duplex (no HDSEL) — config correct.
- `Preamble TX: OK` → MCU transmit path healthy.
- `ISR@entry=0x00**62**00E2`: **FE (framing error) set before the preamble** — the PDN line is not
  a clean idle-high when sampled. USART2 is TMC-only (logging is on RTT), so this is the PDN
  electrical state, not stray debug traffic. `postPreamble=…00F8` has ORE (undrained preamble echo).
- Motor still steps: STEP/DIR + blind writes work → TMC core alive; only the UART reply path is dead.

## 4. Root cause (electrical) — new leak-to-ground on the PDN net

**Bench: PDN_UART idles at 1.46 V** (healthy on this board ≈ 2.63 V — R12 5 kΩ pull-up to 3V3 vs
the TMC's ~20 kΩ internal pull-down).

Divider math from 1.46 V:
- If R12 were simply **open**, only the internal pull-down remains → line would sit near **0 V**,
  not 1.46 V. So a pull-up is still present.
- 1.46 V with a 5 kΩ pull-up ⇒ effective pull-down ≈ 4 kΩ ⇒ a **new ~5 kΩ leak to GND has appeared
  in parallel** with the normal ~20 kΩ:  `20k ∥ X = 3.96k ⇒ X ≈ 4.9 kΩ`.

A new ~5 kΩ path to ground on that pin is the signature of a **damaged/leaky I/O pin** (a conducting
ESD clamp diode) — which kills that pin's UART function while separate pins (STEP/DIR) keep working.
Matches the symptom set exactly and is consistent with **inductive-spike damage from the session's
abrupt de-energizations**.

## 5. Localization test (do this next — cheap → definitive)

**Power off → remove the TMC2209 module → power on → re-measure PDN idle at the MCU pin:**

- **Rises to ~3.3 V** → the ~5 kΩ leak was inside the module → **TMC2209 PDN pin damaged →
  swap the module** (leading hypothesis). MCU + R12 are fine.
- **Still ~1.46 V** → leak is board-side:
  - Ohm **R12** (power off): expect ~5 kΩ.
  - Diode-test **PA3↔GND** (power off): healthy ≈ 0.5–0.7 V one way / OL the other. Low-or-OL both
    ways = PA3 clamp damaged → MCU (the historical culprit; fixed before by MCU replacement).

Optional confirm: scope PA3/PDN during a read request — no reply waveform = chip not answering;
reply present but MCU misreads = MCU RX/levels.

## 6. Expected fix & follow-up

- Most likely: **replace the TMC2209 module**; PDN idle should return to ~2.6 V and the IFCNT probe
  should verify on next boot.
- If module swap is the fix, add a **hardware follow-up**: phase snubbing/clamping or a gentler
  de-energize ramp so future hard-stop sessions don't keep damaging modules (repeated abrupt
  de-energization under load is the suspected stressor).

## 7. Impact while unresolved

- **Dosing volume is still accurate**: microstep = **1/8 by the MS pins** (MS1=MS2=0), which matches
  firmware `TMC_MICROSTEP=8` — step→volume math holds regardless of UART verification (proven by the
  prior trace dosing correctly).
- **Unverified / do not trust**: SpreadCycle vs StealthChop, and IRUN/IHOLD current (if GCONF was
  lost, `I_SCALE_ANALOG` defaults to 1 → phase current from the module VREF pot, not UART settings).
  So torque/noise/thermal behavior is unconfirmed.

## 8. Related history

Prior TMC2209 UART RX investigations (Nema17 / Cowork tree) root-caused an RX failure to the MCU
(PA3 / USART2_RX silicon or joint) and cold-boot auto-baud lock; the definitive fix there was
**MCU replacement**. See that tree's `Context.md` and `docs/TMC2209_UART_AUDIT.md`. **This instance
differs**: it is a *developed* fault on a previously-working board with a measured low idle level
(1.46 V) pointing to a leak on the PDN net — module-side first, MCU-side second.
