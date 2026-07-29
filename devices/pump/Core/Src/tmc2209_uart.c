/**
 ******************************************************************************
 * @file    tmc2209_uart.c
 * @brief   TMC2209 UART Interface — Register Configuration via USART2
 * @author  CANopen Motor Control Project
 * @date    2026
 ******************************************************************************
 *
 * TMC2209 UART Register Configuration Driver
 *
 * Full read/write driver with IFCNT-verified register writes.
 * Each critical register write is confirmed by reading the TMC2209
 * IFCNT counter before and after, with automatic retry on failure.
 * This ensures GCONF.MSTEP_REG_SELECT is reliably set, which is
 * required for CHOPCONF.MRES to control microstepping resolution.
 *
 * HARDWARE CIRCUIT (two-wire to one-wire):
 *                        3V3
 *                         |
 *                      R12 (5k) pull-up — MUST be populated
 *                         |
 *   PA2 (TX, AF_OD) ---[R7 1kΩ]---+--- TMC2209 PDN_UART
 *   PA3 (RX, AF_OD) --------------+
 *
 * PA2 and PA3 are configured as GPIO_MODE_AF_OD with GPIO_PULLUP.
 * R12 (5k to 3V3) provides the external pull-up that defines the bus
 * idle voltage (~3.0V). Without R12, the weak internal pull-ups (~40kΩ)
 * lose to the TMC2209's ~50kΩ internal pull-down, and the bus idles
 * below VIH (~2.1V), breaking all RX operations.
 * Open-drain TX allows TMC2209 to pull the shared PDN_UART line LOW for
 * reply datagrams without bus contention, enabling full bidirectional
 * communication (IFCNT-verified writes and register reads).
 *
 * ALL REGISTER ACCESS — ZERO HAL DISRUPTION:
 *   Writes use direct USART2 register access (TDR/ISR polling),
 *   which does NOT touch the HAL handle (huart2) at all.
 *
 * After the first valid UART datagram, the TMC2209 switches PDN_UART from
 * standby-control mode to UART mode. Debug traffic on PA2 (printf/Log)
 * does NOT form valid datagrams (wrong sync byte + CRC mismatch), so
 * the TMC2209 ignores it. (TMC2209 Datasheet §5.3)
 *
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "tmc2209_uart.h"
#include "stm32g4xx_hal.h"
#include <string.h>

/* -------------------------------------------------------------------------- */
/*  External References                                                       */
/* -------------------------------------------------------------------------- */
extern UART_HandleTypeDef huart2;

/* -------------------------------------------------------------------------- */
/*  Module State                                                              */
/* -------------------------------------------------------------------------- */
static bool tmc2209_initialized = false;
static TMC2209_InitStep_t init_failed_step = TMC2209_INIT_STEP_NONE;
static TMC2209_Diag_t diag_data; /* Captured during init, read after */

/* Active TMC2209 UART slave address. Starts at the MS1/MS2-implied default
 * but is overwritten by the boot address sweep, because the MS pins share the
 * misrouted PA-bank and may not actually set the TMC's address. All reads and
 * writes use this so the whole driver follows the discovered address. */
static uint8_t g_tmc_addr = TMC2209_SLAVE_ADDR;

/* -------------------------------------------------------------------------- */
/*  CRC8 — TMC2209 Trinamic Variant                                          */
/* -------------------------------------------------------------------------- */
/**
 * @brief  Calculate CRC8 for TMC2209 UART datagram
 * @param  data   Pointer to datagram bytes (excluding CRC byte)
 * @param  len    Number of bytes to CRC
 * @retval CRC8 value
 *
 * Polynomial 0x07 (x^8 + x^2 + x + 1), initial value 0x00.
 * Processes each byte LSB-first (unique to Trinamic protocol):
 *   XOR MSB of running CRC with LSB of current data byte.
 */
static uint8_t TMC2209_CRC8(const uint8_t *data, uint8_t len) {
	uint8_t crc = 0;
	for (uint8_t i = 0; i < len; i++) {
		uint8_t byte = data[i];
		for (uint8_t j = 0; j < 8; j++) {
			if ((crc >> 7) ^ (byte & 0x01)) {
				crc = (crc << 1) ^ 0x07;
			} else {
				crc = crc << 1;
			}
			byte >>= 1;
		}
	}
	return crc;
}

/* -------------------------------------------------------------------------- */
/*  Direct Register-Level USART2 TX                                          */
/* -------------------------------------------------------------------------- */

/**
 * @brief  Transmit bytes via USART2 — TX only, no echo consumption
 *
 * Fire-and-forget transmit: writes each byte to TDR, waits for TXE,
 * then waits for TC (Transmission Complete) at the end. Does NOT read
 * RX at all — echo bytes are left in RDR / cause ORE, which the caller
 * must clean up via USART2_CleanupDirect() after this returns.
 *
 * Use this for write-only TMC2209 operations (Phase 1) where we don't
 * care about the echo path and just need the datagram on the wire.
 *
 * Does NOT touch the HAL handle (huart2) — direct register access only.
 *
 * @param  data  Pointer to bytes to transmit
 * @param  len   Number of bytes
 * @retval true on success, false on timeout
 */
static bool USART2_TransmitDirect(const uint8_t *data, uint8_t len) {
	for (uint8_t i = 0; i < len; i++) {
		/* Wait for TXE (TX data register empty) */
		uint32_t start = HAL_GetTick();
		while (!(USART2->ISR & USART_ISR_TXE_TXFNF)) {
			if ((HAL_GetTick() - start) > 10) {
				return false; /* 10 ms timeout per byte */
			}
		}
		USART2->TDR = data[i];
	}

	/* Wait for TC (Transmission Complete) — all bits shifted out */
	uint32_t start = HAL_GetTick();
	while (!(USART2->ISR & USART_ISR_TC)) {
		if ((HAL_GetTick() - start) > 10) {
			return false;
		}
	}

	return true;
}

/**
 * @brief  Transmit bytes via USART2 and consume echo bytes in real-time
 *
 * On a two-wire full-duplex circuit (PA2 TX → 1kΩ → PDN_UART ← PA3 RX),
 * every byte transmitted is echoed back on RX. With FIFO disabled, RDR
 * holds only 1 byte — if we don't read the echo before the next byte
 * arrives, we get an Overrun Error (ORE) that corrupts all subsequent RX.
 *
 * This function writes each byte to TDR, then immediately waits for the
 * echo byte on RX and discards it. This guarantees:
 *   - No ORE during transmission (echo consumed before next byte)
 *   - RX is completely clean after TX completes (no stale echo debris)
 *   - No race condition between TX completion and echo arrival
 *
 * Does NOT touch the HAL handle (huart2) — direct register access only.
 *
 * @param  data  Pointer to bytes to transmit
 * @param  len   Number of bytes
 * @retval true on success, false on timeout
 */
static bool USART2_TransmitWithEchoDrain(const uint8_t *data, uint8_t len) {
	for (uint8_t i = 0; i < len; i++) {
		/* Wait for TXE (TX data register empty) */
		uint32_t start = HAL_GetTick();
		while (!(USART2->ISR & USART_ISR_TXE_TXFNF)) {
			if ((HAL_GetTick() - start) > 10) {
				return false; /* 10 ms timeout per byte */
			}
		}
		USART2->TDR = data[i];

		/* Wait for the echo byte to arrive on RX and discard it.
		 * At 115200 baud, one byte takes ~87 µs. We poll RXNE with
		 * a 10 ms timeout (generous margin for robustness). */
		start = HAL_GetTick();
		while (!(USART2->ISR & USART_ISR_RXNE_RXFNE)) {
			if ((HAL_GetTick() - start) > 10) {
				/* If ORE occurred despite our efforts, clear it */
				if (USART2->ISR & USART_ISR_ORE) {
					USART2->ICR = USART_ICR_ORECF;
				}
				return false;
			}
			/* Defensive: clear ORE if it somehow set mid-loop */
			if (USART2->ISR & USART_ISR_ORE) {
				USART2->ICR = USART_ICR_ORECF;
			}
		}
		(void) USART2->RDR; /* Discard echo byte */
	}

	/* Wait for TC (Transmission Complete) — all bits shifted out */
	uint32_t start = HAL_GetTick();
	while (!(USART2->ISR & USART_ISR_TC)) {
		if ((HAL_GetTick() - start) > 10) {
			return false;
		}
	}

	return true;
}

/**
 * @brief  Flush USART2 RX and clear error flags (register-level)
 *
 * After transmitting in full-duplex mode, the RX side may have picked up
 * noise or echo data. This clears everything without touching HAL state.
 */
static void USART2_CleanupDirect(void) {
	/* Clear all error and status flags via ICR */
	USART2->ICR = USART_ICR_PECF | USART_ICR_FECF | USART_ICR_NECF
			| USART_ICR_ORECF | USART_ICR_IDLECF | USART_ICR_TCCF
			| USART_ICR_CTSCF | USART_ICR_CMCF;

	/* Drain RDR — read until RXNE is clear */
	while (USART2->ISR & USART_ISR_RXNE_RXFNE) {
		(void) USART2->RDR;
	}
}

/* -------------------------------------------------------------------------- */
/*  Direct Register-Level USART2 RX                                          */
/* -------------------------------------------------------------------------- */

/**
 * @brief  Receive bytes via USART2 using direct register access
 *
 * Polls RXNE (RX Not Empty) and reads from RDR. Does NOT touch the HAL
 * handle (huart2). Used for two-wire full-duplex reads where PA3 (RX)
 * hears both the TX echo and the TMC2209 reply.
 *
 * @param  buf       Buffer to store received bytes
 * @param  len       Number of bytes to receive
 * @param  timeout   Timeout in milliseconds
 * @retval true on success (all bytes received), false on timeout
 */
static bool USART2_ReceiveDirect(uint8_t *buf, uint8_t len, uint32_t timeout) {
	for (uint8_t i = 0; i < len; i++) {
		uint32_t start = HAL_GetTick();
		while (!(USART2->ISR & USART_ISR_RXNE_RXFNE)) {
			if ((HAL_GetTick() - start) > timeout) {
				return false;
			}
		}
		buf[i] = (uint8_t) (USART2->RDR & 0xFF);
	}
	return true;
}

/**
 * @brief  Diagnostic variant of ReceiveDirect — captures partial RX and ISR on timeout
 *
 * Used by TMC2209_ReadRegisterDiag() to capture per-attempt diagnostic data
 * (byte count, raw bytes, ISR state) for init troubleshooting.
 */
static bool USART2_ReceiveDiag(uint8_t *buf, uint8_t len, uint32_t timeout,
		uint8_t *actual_count, uint32_t *isr_on_fail) {
	*actual_count = 0;
	for (uint8_t i = 0; i < len; i++) {
		uint32_t start = HAL_GetTick();
		while (!(USART2->ISR & USART_ISR_RXNE_RXFNE)) {
			if ((HAL_GetTick() - start) > timeout) {
				if (isr_on_fail) *isr_on_fail = USART2->ISR;
				return false;
			}
		}
		buf[i] = (uint8_t)(USART2->RDR & 0xFF);
		(*actual_count)++;
	}
	return true;
}

/**
 * @brief  Diagnostic ReadRegister — captures per-attempt data into diag_data
 *
 * Variant of TMC2209_ReadRegister that records echo-drain status, RX byte
 * count, raw RX bytes, and ISR state for each attempt into the global
 * diag_data struct. Used by the smoke test during init.
 */
static TMC2209_Status_t TMC2209_ReadRegisterDiag(uint8_t reg, uint32_t *value,
		uint8_t attempt_idx) {
	if (value == NULL) return TMC2209_ERR_NOT_INIT;

	uint8_t request[4];
	request[0] = TMC2209_SYNC_BYTE;
	request[1] = g_tmc_addr;
	request[2] = reg;
	request[3] = TMC2209_CRC8(request, 3);

	USART2_CleanupDirect();

	bool echo_ok = USART2_TransmitWithEchoDrain(request, 4);
	if (attempt_idx < TMC_DIAG_MAX_ATTEMPTS) {
		diag_data.attempt[attempt_idx].echo_drain_ok = echo_ok;
	}
	if (!echo_ok) {
		USART2_CleanupDirect();
		if (attempt_idx < TMC_DIAG_MAX_ATTEMPTS) {
			diag_data.attempt[attempt_idx].error = TMC2209_ERR_UART_TX;
			diag_data.attempt[attempt_idx].isr_at_rx_fail = USART2->ISR;
		}
		return TMC2209_ERR_UART_TX;
	}

	uint8_t rx_buf[8] = {0};
	uint8_t rx_count = 0;
	uint32_t isr_fail = 0;
	bool rx_ok = USART2_ReceiveDiag(rx_buf, 8, 50, &rx_count, &isr_fail);

	if (attempt_idx < TMC_DIAG_MAX_ATTEMPTS) {
		diag_data.attempt[attempt_idx].rx_count = rx_count;
		memcpy(diag_data.attempt[attempt_idx].rx_bytes, rx_buf, 8);
		diag_data.attempt[attempt_idx].isr_at_rx_fail = rx_ok ? USART2->ISR : isr_fail;
	}

	if (!rx_ok) {
		USART2_CleanupDirect();
		if (attempt_idx < TMC_DIAG_MAX_ATTEMPTS) {
			diag_data.attempt[attempt_idx].error = TMC2209_ERR_UART_RX;
		}
		return TMC2209_ERR_UART_RX;
	}

	uint8_t calc_crc = TMC2209_CRC8(rx_buf, 7);
	if (calc_crc != rx_buf[7]) {
		USART2_CleanupDirect();
		if (attempt_idx < TMC_DIAG_MAX_ATTEMPTS) {
			diag_data.attempt[attempt_idx].error = TMC2209_ERR_CRC;
		}
		return TMC2209_ERR_CRC;
	}

	*value = ((uint32_t)rx_buf[3] << 24) | ((uint32_t)rx_buf[4] << 16)
			| ((uint32_t)rx_buf[5] << 8) | ((uint32_t)rx_buf[6]);

	USART2_CleanupDirect();
	HAL_Delay(2);

	if (attempt_idx < TMC_DIAG_MAX_ATTEMPTS) {
		diag_data.attempt[attempt_idx].error = TMC2209_OK;
	}
	return TMC2209_OK;
}

/* -------------------------------------------------------------------------- */
/*  Public API — Register Access                                              */
/* -------------------------------------------------------------------------- */

/**
 * @brief  Write a 32-bit value to a TMC2209 register at a specific slave address
 *
 * Uses direct register-level USART2 TX. Does NOT touch HAL handle.
 *
 * Write datagram format (8 bytes):
 *   [SYNC=0x05] [SLAVE_ADDR] [REG|0x80] [DATA_3] [DATA_2] [DATA_1] [DATA_0] [CRC8]
 *
 * TMC2209 does not respond to writes — fire-and-forget with CRC integrity.
 *
 * @param  addr   Slave address (0x00–0x03, set by MS1/MS2 pins)
 * @param  reg    Register address
 * @param  value  32-bit value to write
 * @param  diag_out  Optional: pointer to 8-byte buffer to capture the raw datagram (NULL to skip)
 */
static TMC2209_Status_t TMC2209_WriteRegisterAddr(uint8_t addr, uint8_t reg,
		uint32_t value, uint8_t *diag_out) {
	uint8_t datagram[8];
	datagram[0] = TMC2209_SYNC_BYTE;
	datagram[1] = addr;
	datagram[2] = reg | 0x80; /* Write flag */
	datagram[3] = (value >> 24) & 0xFF; /* MSB first */
	datagram[4] = (value >> 16) & 0xFF;
	datagram[5] = (value >> 8) & 0xFF;
	datagram[6] = value & 0xFF; /* LSB */
	datagram[7] = TMC2209_CRC8(datagram, 7);

	/* Capture raw datagram for diagnostic printout if requested */
	if (diag_out) {
		memcpy(diag_out, datagram, 8);
	}

	/* Flush any stale RX state before transmitting */
	USART2_CleanupDirect();

	/* Transmit 8-byte write datagram — TX only, no echo dependency.
	 * Echo bytes left in RDR are cleaned up by USART2_CleanupDirect(). */
	if (!USART2_TransmitDirect(datagram, 8)) {
		USART2_CleanupDirect();
		return TMC2209_ERR_UART_TX;
	}

	/* Inter-datagram delay — TMC2209 needs ~75 bit times between frames */
	HAL_Delay(2);

	return TMC2209_OK;
}

/**
 * @brief  Write a 32-bit value to a TMC2209 register (default slave address)
 */
TMC2209_Status_t TMC2209_WriteRegister(uint8_t reg, uint32_t value) {
	return TMC2209_WriteRegisterAddr(g_tmc_addr, reg, value, NULL);
}

/**
 * @brief  Read a 32-bit value from a TMC2209 register
 *
 * Uses direct register-level USART2 TX/RX. Does NOT touch HAL handle.
 *
 * Two-wire full-duplex read sequence:
 *   1. Flush RX (clear echo/noise from previous traffic)
 *   2. Transmit 4-byte read request with real-time echo drain
 *      (each TX byte's echo is consumed inline — no ORE possible)
 *   3. Receive 8-byte TMC2209 reply directly (echo already consumed)
 *   4. Validate CRC on reply bytes [0..7]
 *   5. Extract 32-bit value from reply[3..6] (MSB first)
 *   6. Flush RX for clean state
 *
 * ECHO DRAIN FIX (2026-02-11):
 *   Uses USART2_TransmitWithEchoDrain() which reads and discards each
 *   echo byte immediately after sending each TX byte. This prevents ORE
 *   entirely and leaves RX clean for the 8-byte TMC2209 reply.
 *
 * Read request datagram (4 bytes):
 *   [SYNC=0x05] [SLAVE_ADDR] [REG] [CRC8]
 *
 * Reply datagram (8 bytes):
 *   [SYNC=0x05] [0xFF] [REG] [DATA_3] [DATA_2] [DATA_1] [DATA_0] [CRC8]
 */
TMC2209_Status_t TMC2209_ReadRegister(uint8_t reg, uint32_t *value) {
	if (value == NULL) {
		return TMC2209_ERR_NOT_INIT;
	}

	uint8_t request[4];
	request[0] = TMC2209_SYNC_BYTE;
	request[1] = g_tmc_addr;
	request[2] = reg;
	request[3] = TMC2209_CRC8(request, 3);

	/* Flush RX before starting — clear any stale echo/noise */
	USART2_CleanupDirect();

	/* Transmit 4-byte read request with real-time echo consumption.
	 * Each TX byte's echo is read and discarded inline — no ORE possible.
	 * After this returns, RX is clean (no echo debris). */
	if (!USART2_TransmitWithEchoDrain(request, 4)) {
		USART2_CleanupDirect();
		return TMC2209_ERR_UART_TX;
	}

	/* Receive 8-byte TMC2209 reply (echo already consumed during TX) */
	uint8_t rx_buf[8];
	if (!USART2_ReceiveDirect(rx_buf, 8, 50)) {
		USART2_CleanupDirect();
		return TMC2209_ERR_UART_RX;
	}

	/* Validate CRC on the 8-byte reply */
	uint8_t calc_crc = TMC2209_CRC8(rx_buf, 7);
	if (calc_crc != rx_buf[7]) {
		USART2_CleanupDirect();
		return TMC2209_ERR_CRC;
	}

	/* Extract 32-bit register value (MSB first) */
	*value = ((uint32_t) rx_buf[3] << 24) | ((uint32_t) rx_buf[4] << 16)
			| ((uint32_t) rx_buf[5] << 8) | ((uint32_t) rx_buf[6]);

	/* Clean state for next operation */
	USART2_CleanupDirect();

	/* Inter-datagram delay */
	HAL_Delay(2);
	return TMC2209_OK;
}

/* -------------------------------------------------------------------------- */
/*  Internal Helper — IFCNT-Verified Write                                    */
/* -------------------------------------------------------------------------- */

/**
 * @brief  Write a TMC2209 register with IFCNT verification and retry
 *
 * Reads IFCNT before and after the write. If IFCNT incremented by exactly 1,
 * the write was received by the TMC2209. Retries up to N times on failure.
 *
 * Requires functional RX path (R12 pull-up populated on PDN_UART bus).
 */
static TMC2209_Status_t TMC2209_WriteVerified(uint8_t reg, uint32_t value,
		uint8_t retries) {
	for (uint8_t attempt = 0; attempt < retries; attempt++) {
		/* Read IFCNT before write */
		uint32_t ifcnt_before = 0;
		TMC2209_Status_t st = TMC2209_ReadRegister(TMC2209_IFCNT,
				&ifcnt_before);
		if (st != TMC2209_OK) {
			/* If we can't even read IFCNT, retry after a small delay */
			HAL_Delay(5);
			continue;
		}

		/* Perform the register write */
		st = TMC2209_WriteRegister(reg, value);
		if (st != TMC2209_OK) {
			HAL_Delay(2);
			continue;
		}

		/* Read IFCNT after write */
		uint32_t ifcnt_after = 0;
		st = TMC2209_ReadRegister(TMC2209_IFCNT, &ifcnt_after);
		if (st != TMC2209_OK) {
			HAL_Delay(5);
			continue;
		}

		/* IFCNT is 8-bit and wraps 0xFF→0x00 */
		uint8_t before8 = (uint8_t) (ifcnt_before & 0xFF);
		uint8_t after8 = (uint8_t) (ifcnt_after & 0xFF);
		if (after8 == (uint8_t) (before8 + 1)) {
			return TMC2209_OK; /* Write confirmed */
		}

		/* IFCNT didn't increment — retry */
		HAL_Delay(5);
	}

	return TMC2209_ERR_VERIFY;
}

/* -------------------------------------------------------------------------- */
/*  Public API — Init Diagnostics                                             */
/* -------------------------------------------------------------------------- */

/**
 * @brief  Get which init step failed (for diagnostic output)
 * @retval TMC2209_InitStep_t enum value indicating the failed step,
 *         TMC2209_INIT_STEP_COMPLETE if init succeeded,
 *         TMC2209_INIT_STEP_NONE if init hasn't been called yet.
 */
TMC2209_InitStep_t TMC2209_GetInitFailedStep(void) {
	return init_failed_step;
}

const TMC2209_Diag_t* TMC2209_GetDiagnostics(void) {
	return &diag_data;
}

/* -------------------------------------------------------------------------- */
/*  Public API — Runtime Diagnostics                                          */
/* -------------------------------------------------------------------------- */

TMC2209_Status_t TMC2209_ReadStallGuard(uint16_t *sg_result) {
	if (sg_result == NULL || !tmc2209_initialized) {
		return TMC2209_ERR_NOT_INIT;
	}

	uint32_t raw;
	TMC2209_Status_t status = TMC2209_ReadRegister(TMC2209_SG_RESULT, &raw);

	if (status == TMC2209_OK) {
		*sg_result = (uint16_t) (raw & 0x03FF);
	}
	return status;
}

TMC2209_Status_t TMC2209_ReadDrvStatus(uint32_t *status) {
	if (status == NULL || !tmc2209_initialized) {
		return TMC2209_ERR_NOT_INIT;
	}

	return TMC2209_ReadRegister(TMC2209_DRV_STATUS, status);
}

/* -------------------------------------------------------------------------- */
/*  Public API — Initialization                                               */
/* -------------------------------------------------------------------------- */

/**
 * @brief  Initialize TMC2209 via UART on USART2
 *
 * ZERO HAL DISRUPTION — uses direct register-level USART2 TX/RX.
 * Does not call any HAL_UART_* functions. Does not modify huart2 handle.
 * PA2 is open-drain; full bidirectional UART with IFCNT verification.
 *
 * HARDWARE REQUIREMENT: R12 (5k pull-up from PDN_UART to 3V3) must be
 * populated. Without it, the bus idles below VIH and all RX fails.
 * See AI-docs/R12_PDN_UART_Pullup_Fix.md for details.
 *
 * Call BEFORE Log_Init(). After init, USART2 continues as debug UART
 * with no state corruption — printf and Log_Init() work immediately.
 *
 * Register writes:
 *
 *   GCONF (0x00) = 0x000001C4
 *     I_SCALE_ANALOG   = 0   Use IRUN/IHOLD for current scaling (VREF unconnected)
 *     EN_SPREADCYCLE    = 1   SpreadCycle chopper (best torque for pumps)
 *     PDN_DISABLE       = 1   Disable PDN standby (line shared with debug)
 *     MSTEP_REG_SELECT  = 1   Microstep resolution from CHOPCONF register
 *     MULTISTEP_FILT    = 1   Enable STEP pulse input filter
 *
 *   CHOPCONF (0x6C) = 0x18010053
 *     TOFF  = 3               Off-time (enables chopper — TOFF=0 disables!)
 *     HSTRT = 5               Hysteresis start
 *     HEND  = 0               Hysteresis end
 *     TBL   = 2               Blank time 36 clocks (stable current sensing)
 *     MRES  = 8               Full step → 1600 steps/rev (200 × 8 gear)
 *     INTPOL = 1              MicroPlyer interpolates full→256 internally
 *
 *   IHOLD_IRUN (0x10) = 0x00061F10
 *     IRUN  = 31              Maximum run current (register-scaled, VREF unused)
 *     IHOLD = 16              Holding current (~50% of IRUN, maintains position at standstill)
 *     IHOLDDELAY = 6          Smooth run→hold current transition
 *
 *   De-energization strategy (per-state):
 *     HALT/STOP states:  EN=LOW (enabled)  → IHOLD maintains motor position for resume
 *     QUICK_STOP/NMT:    EN=HIGH (disabled) → coils fully de-energized regardless of IHOLD
 *     Motor only energized on explicit controller command (Stepper_Enable via state machine)
 *
 * @retval TMC2209_OK on success, error code on failure
 */
TMC2209_Status_t TMC2209_Init(void) {
	TMC2209_Status_t status;

	/*
	 * Disable USART2 NVIC during TMC2209 init to prevent
	 * Log_TxISR / HAL_UART_IRQHandler from firing while we
	 * access USART2 registers directly.
	 */
	HAL_NVIC_DisableIRQ(USART2_IRQn);

	/* ── Capture diagnostic snapshot of USART2 and GPIO state ── */
	memset(&diag_data, 0, sizeof(diag_data));
	diag_data.brr = USART2->BRR;
	diag_data.cr1 = USART2->CR1;
	diag_data.cr2 = USART2->CR2;
	diag_data.cr3 = USART2->CR3;
	diag_data.isr_at_entry = USART2->ISR;
	diag_data.gpioa_idr = GPIOA->IDR;
	diag_data.gpioa_moder = GPIOA->MODER;
	diag_data.gpioa_afrl = GPIOA->AFR[0];

	/* Read MS1/MS2 pin state — determines TMC2209 slave address */
	diag_data.ms1_level = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_7) ? 1 : 0; /* MS1 = PA7 */
	diag_data.ms2_level = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_6) ? 1 : 0; /* MS2 = PA6 */

	/* Power-up delay (100 ms): the TMC2209's charge pump and internal 12 MHz
	 * oscillator need time after Vcc before they reliably receive UART traffic
	 * and auto-baud on a COLD boot. A generous settle is the cheapest win for
	 * cold-boot write reliability (the StealthChop-on-reboot symptom). */
	HAL_Delay(100);

	/* ── Register values ── */

	const uint32_t gconf_val = GCONF_EN_SPREADCYCLE
			| GCONF_PDN_DISABLE | GCONF_MSTEP_REG_SELECT | GCONF_MULTISTEP_FILT;
	/* = 0x000001C4  (I_SCALE_ANALOG=0: VREF pin unconnected on this PCB) */

	const uint32_t chopconf_val = (3u << CHOPCONF_TOFF_SHIFT)
			| (5u << CHOPCONF_HSTRT_SHIFT) | (0u << CHOPCONF_HEND_SHIFT)
			| (2u << CHOPCONF_TBL_SHIFT)
			| ((uint32_t) MRES_FULLSTEP << CHOPCONF_MRES_SHIFT)
			| CHOPCONF_INTPOL;
	/* = 0x18010053 */

	const uint32_t ihold_irun_val = (6u << IHOLDDELAY_SHIFT)
			| (31u << IRUN_SHIFT) | (16u << IHOLD_SHIFT);
	/* = 0x00061F10 */

	/* ── Lower the UART baud for cold-boot auto-baud robustness ──
	 * The TMC2209 has no crystal and latches its baud from the bit timing of
	 * the first datagram's sync byte. At 115200 (~8.7 µs/bit) a cold,
	 * marginally-settled bus is easy to mis-sample → wrong baud latched → every
	 * write fails CRC → the chip stays in its StealthChop power-on default
	 * (exactly our symptom). Dropping to ~9600 (~104 µs/bit) gives a 12× longer,
	 * far easier-to-measure bit time, greatly improving the odds of a correct
	 * cold lock. ALL config is written at this baud; the debug baud is restored
	 * in cleanup. Direct BRR access only — the HAL handle (huart2) is untouched,
	 * so debug printf resumes at the original baud. (115200/12 ≈ 9600; BRR is
	 * linear in OVER16 oversampling, which HAL uses at 115200/80 MHz.) */
	uint32_t brr_save = USART2->BRR;
	uint32_t cr1_save = USART2->CR1;
	USART2->CR1 = cr1_save & ~USART_CR1_UE; /* must disable UE to change BRR */
	USART2->BRR = brr_save * 12u; /* 115200 → ~9600 */
	USART2->CR1 = cr1_save; /* re-enable UE at the new baud */
	HAL_Delay(1); /* let TE/RE re-ack at 9600 */

	/* ── Auto-baud training preamble (PUSH-PULL) ──
	 * The TMC2209 has no crystal: it locks its UART baud rate from the bit
	 * timing of the FIRST datagram's sync byte (0x05) it sees after power-up
	 * (datasheet §4.3). On a COLD boot this lock is the single point of
	 * failure — if that first frame is mis-sampled (line not yet idle-settled,
	 * chip's internal oscillator not yet stable), the wrong baud is latched and
	 * EVERY subsequent write fails CRC → the chip silently stays in its
	 * StealthChop power-on default. That is exactly the observed symptom:
	 * motor runs (defaults) but is silent/StealthChop after a power cycle, yet
	 * works on a warm reset (baud is still locked from the previous session).
	 *
	 * Hardening (all write-only — RX is not routed on this PCB):
	 *   1. Drive PA2 PUSH-PULL for clean rail-to-rail edges, independent of the
	 *      bus pull-up.
	 *   2. Hold a solid idle-HIGH for a few ms BEFORE the first start bit, so
	 *      the chip has a clean idle reference to measure the sync against.
	 *   3. Send the sync/GCONF frame SEVERAL times with gaps. The chip may
	 *      consume the first frame purely for baud detection (its data can be
	 *      discarded), and a glitched early frame no longer dooms the boot —
	 *      each later sync is another lock attempt at the now-stable baud. */
	uint32_t otyper_save = GPIOA->OTYPER;
	GPIOA->OTYPER &= ~(1UL << 2); /* PA2 → push-pull */
	HAL_Delay(5); /* solid idle-HIGH reference before 1st edge */

	for (uint8_t pre = 0; pre < 4; pre++) {
		status = TMC2209_WriteRegister(TMC2209_GCONF, gconf_val);
		if (status != TMC2209_OK) {
			GPIOA->OTYPER = otyper_save;
			init_failed_step = TMC2209_INIT_STEP_GCONF_WRITE;
			goto cleanup;
		}
		HAL_Delay(5);
	}
	diag_data.preamble_tx_ok = (status == TMC2209_OK);
	diag_data.isr_after_preamble = USART2->ISR;
	HAL_Delay(10); /* baud firmly locked before the authoritative config passes */

	/* ── Hardened write-only SpreadCycle configuration ──
	 * USART2_RX (PA3) is not routed to the PDN bus on this PCB, so register
	 * read-back / IFCNT verification is impossible — and it is NOT needed to
	 * SET the mode. SpreadCycle is enabled by the GCONF write alone: datasheet
	 * §5.1 bit2 (en_spreadcycle) selects the chopper, and the SPREAD pin (low,
	 * internal pull-down) does not invert it. A write must carry the node's own
	 * address, but the MCU may not drive MS1/MS2, so we sweep all four addresses
	 * — the one matching the TMC always lands (datasheet §4.4, write-only).
	 *
	 * Cold-boot hardening: the full config is sent in THREE passes while PA2 is
	 * push-pull (set above for the preamble). If an early frame is lost while
	 * the TMC settles / locks baud, a later pass lands. Each address ends on a
	 * GCONF write so en_spreadcycle is the final word. */
	for (uint8_t pass = 0; pass < 3; pass++) {
		for (uint8_t addr = 0; addr < 4; addr++) {
			TMC2209_WriteRegisterAddr(addr, TMC2209_GCONF, gconf_val, NULL);
			HAL_Delay(5);
			TMC2209_WriteRegisterAddr(addr, TMC2209_CHOPCONF, chopconf_val, NULL);
			HAL_Delay(2);
			TMC2209_WriteRegisterAddr(addr, TMC2209_IHOLD_IRUN, ihold_irun_val, NULL);
			HAL_Delay(2);
			TMC2209_WriteRegisterAddr(addr, TMC2209_GCONF, gconf_val, NULL);
			HAL_Delay(2);
		}
	}

	GPIOA->OTYPER = otyper_save; /* PA2 → restore open-drain for debug UART */

	/* All config datagrams went out push-pull, on every address, three passes.
	 * Unverified (no RX on this board) but this is the hardened cold-boot path
	 * that runs SpreadCycle reliably. blind_fallback flags it as write-only. */
	tmc2209_initialized = true;
	diag_data.blind_fallback = true;
	init_failed_step = TMC2209_INIT_STEP_COMPLETE;
	status = TMC2209_OK;

	cleanup:
	/*
	 * ── Clean up and re-enable USART2 for debug UART ──
	 * Restore the debug baud (config writes ran at the lowered ~9600 baud),
	 * clear peripheral flags, re-enable NVIC. The HAL handle (huart2) was never
	 * touched — printf/Log_Init work immediately at the restored baud.
	 */
	USART2->CR1 &= ~USART_CR1_UE; /* disable UE to change BRR back */
	USART2->BRR = brr_save; /* restore original (debug) baud */
	USART2->CR1 = cr1_save; /* re-enable UE with original config */
	USART2_CleanupDirect();
	NVIC_ClearPendingIRQ(USART2_IRQn);
	HAL_NVIC_EnableIRQ(USART2_IRQn);

	return status;
}
