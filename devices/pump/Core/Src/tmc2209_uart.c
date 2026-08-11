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

/* Active TMC2209 UART slave address. Starts at the MS1/MS2-implied default but
 * is overwritten by TMC2209_ResolveAddress() at init, which discovers the real
 * address by reading IFCNT at 0-3. The MS pins reuse A4988 copper on this PCB
 * and have never been confirmed to reach the driver module, so the address is
 * discovered rather than assumed. All reads and writes use this. */
static uint8_t g_tmc_addr = TMC2209_SLAVE_ADDR;

/* Refreshes the independent watchdog during init. Defined further down (it
 * needs hiwdg), forward-declared here because the retry loops above use it:
 * a fully-failing read can burn ~450 ms, and the IWDG is already running at
 * ~2048 ms by the time TMC2209_Init() is called. */
static void TMC2209_KickWatchdog(void);

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
		/* Each attempt is up to two full reads plus a write; on a dead RX path
		 * that is several hundred ms, so keep the watchdog fed per attempt. */
		TMC2209_KickWatchdog();

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

TMC2209_Status_t TMC2209_ReadDrvStatus(uint32_t *status) {
	if (status == NULL || !tmc2209_initialized) {
		return TMC2209_ERR_NOT_INIT;
	}

	return TMC2209_ReadRegister(TMC2209_DRV_STATUS, status);
}

/* -------------------------------------------------------------------------- */
/*  Init Tuning Constants                                                     */
/* -------------------------------------------------------------------------- */

/* Auto-baud training frames sent before the first verified transaction.
 * The TMC2209 has no crystal and latches its baud from the bit timing of the
 * first datagram's sync byte (datasheet §4.3). Two frames give the chip a
 * second lock attempt if the first is mis-sampled on a cold start. */
#define TMC_PREAMBLE_FRAMES     2u

/* IFCNT-verified write attempts per register before giving up. */
#define TMC_WRITE_RETRIES       2u

/* Baud rates tried, in order: index 0 = debug baud (115200), index 1 = that
 * baud / 12 (~9600). The slow rate is the historical cold-boot fallback — a
 * 12x longer bit time is far easier for the chip's crystal-less auto-baud to
 * measure on a marginal cold bus. It is now a fallback, not the default,
 * because the primary path proves itself by read-back. */
#define TMC_BAUD_ATTEMPTS       2u

/* Read-back comparison masks — only the fields this driver actually writes are
 * compared, so reserved/undefined bits reading back differently cannot cause a
 * false verification failure.
 *   GCONF    bits 0-9    (I_SCALE_ANALOG .. MULTISTEP_FILT)
 *   CHOPCONF bits 0-10 (TOFF/HSTRT/HEND), 15-16 (TBL), 24-27 (MRES), 28 (INTPOL) */
#define TMC_GCONF_VERIFY_MASK       0x000003FFUL
#define TMC_CHOPCONF_VERIFY_MASK    0x1F0387FFUL /* incl. bit 17 vsense */

/* -------------------------------------------------------------------------- */
/*  Internal Helpers — Init                                                   */
/* -------------------------------------------------------------------------- */

/* IWDG is started in main() BEFORE TMC2209_Init() runs (MX_IWDG_Init at
 * ~2048 ms). A failing init that exhausts both baud attempts and all write
 * retries can exceed that, so the watchdog is kicked between init phases. */
extern IWDG_HandleTypeDef hiwdg;

static void TMC2209_KickWatchdog(void) {
	if (hiwdg.Instance != NULL) {
		(void) HAL_IWDG_Refresh(&hiwdg);
	}
}

/**
 * @brief  Retune USART2's baud by scaling the saved BRR divisor
 * @param  brr_base    The original (debug) BRR value
 * @param  multiplier  1 = debug baud, 12 = ~9600 for the cold-boot fallback
 *
 * Direct register access only — the HAL handle (huart2) is never touched, so
 * printf/Log_Init resume at the original baud once cleanup restores BRR.
 * BRR is linear in OVER16 oversampling, which HAL uses at 115200 / 80 MHz.
 */
static void USART2_SetBaudDivisor(uint32_t brr_base, uint32_t multiplier) {
	uint32_t cr1 = USART2->CR1;
	USART2->CR1 = cr1 & ~USART_CR1_UE; /* UE must be clear to change BRR */
	USART2->BRR = brr_base * multiplier;
	USART2->CR1 = cr1; /* re-enable UE at the new baud */
	HAL_Delay(1); /* let TE/RE re-ack */
}

/**
 * @brief  Send the auto-baud training preamble (write-only, push-pull TX)
 *
 * PA2 is driven push-pull for the preamble so the first edges are clean
 * rail-to-rail regardless of the R12 pull-up, and a solid idle-HIGH is held
 * before the first start bit to give the chip a stable reference. PA2 is
 * returned to open-drain before this returns — the TMC2209 must be able to
 * pull the shared PDN line low for its replies.
 *
 * @retval true if every preamble frame reached the wire
 */
static bool TMC2209_SendPreamble(uint32_t gconf_val) {
	uint32_t otyper_save = GPIOA->OTYPER;
	GPIOA->OTYPER &= ~(1UL << 2); /* PA2 → push-pull */
	HAL_Delay(5); /* solid idle-HIGH before the first edge */

	bool ok = true;
	for (uint8_t i = 0; i < TMC_PREAMBLE_FRAMES; i++) {
		if (TMC2209_WriteRegisterAddr(TMC2209_SLAVE_ADDR, TMC2209_GCONF,
				gconf_val, NULL) != TMC2209_OK) {
			ok = false;
			break;
		}
		HAL_Delay(5);
	}

	GPIOA->OTYPER = otyper_save; /* → open-drain: TMC needs the line for replies */
	HAL_Delay(2);
	return ok;
}

/**
 * @brief  Discover the TMC2209's UART slave address by reading IFCNT
 *
 * The address is normally set by MS1/MS2 (both driven low → 0x00), but this
 * PCB reuses A4988 copper for the MS pins and their routing has never been
 * confirmed, so the address is discovered rather than assumed. The first
 * address 0-3 that answers an IFCNT read wins and is latched into g_tmc_addr,
 * which every subsequent read and write uses.
 *
 * This is a READ-based probe. It is not the old blind 4-address write sweep —
 * nothing is written until an address has actually answered.
 *
 * @retval true if a TMC2209 responded on some address
 */
static bool TMC2209_ResolveAddress(void) {
	for (uint8_t addr = 0; addr < 4; addr++) {
		TMC2209_KickWatchdog(); /* a non-answering address can cost ~450 ms */
		g_tmc_addr = addr;
		uint32_t ifcnt = 0;
		if (TMC2209_ReadRegisterDiag(TMC2209_IFCNT, &ifcnt, addr)
				== TMC2209_OK) {
			diag_data.addr_found = true;
			diag_data.resolved_addr = addr;
			return true;
		}
	}

	g_tmc_addr = TMC2209_SLAVE_ADDR; /* leave a sane default behind */
	diag_data.addr_found = false;
	return false;
}

/**
 * @brief  Write the full configuration with IFCNT verification + read-back
 *
 * Every write is confirmed by the IFCNT counter incrementing. GCONF and
 * CHOPCONF are additionally read back and compared field-by-field.
 *
 * IHOLD_IRUN (0x10) is a WRITE-ONLY register on the TMC2209 — it cannot be
 * read back at all, so the IFCNT increment is the only proof available for
 * the run/hold current. That is a hardware property, not a shortcut.
 *
 * @retval TMC2209_OK only when every write was confirmed AND both readable
 *         registers match their target
 */
static TMC2209_Status_t TMC2209_ConfigureVerified(uint32_t gconf_val,
		uint32_t chopconf_val, uint32_t ihold_irun_val) {
	TMC2209_Status_t st;

	/* ── GCONF (RW) ── must land first: MSTEP_REG_SELECT is what makes
	 * CHOPCONF.MRES control microstepping at all. */
	st = TMC2209_WriteVerified(TMC2209_GCONF, gconf_val, TMC_WRITE_RETRIES);
	if (st != TMC2209_OK) {
		init_failed_step = TMC2209_INIT_STEP_GCONF_WRITE;
		return st;
	}
	TMC2209_KickWatchdog();

	/* ── CHOPCONF (RW) ── */
	st = TMC2209_WriteVerified(TMC2209_CHOPCONF, chopconf_val,
			TMC_WRITE_RETRIES);
	if (st != TMC2209_OK) {
		init_failed_step = TMC2209_INIT_STEP_CHOPCONF_WRITE;
		return st;
	}
	TMC2209_KickWatchdog();

	/* ── IHOLD_IRUN (write-only) ── IFCNT increment is the only possible proof */
	st = TMC2209_WriteVerified(TMC2209_IHOLD_IRUN, ihold_irun_val,
			TMC_WRITE_RETRIES);
	if (st != TMC2209_OK) {
		init_failed_step = TMC2209_INIT_STEP_IHOLD_WRITE;
		return st;
	}
	diag_data.ihold_write_ok = true;
	TMC2209_KickWatchdog();

	/* ── TPOWERDOWN (write-only) ── standstill → IHOLD delay, written
	 * explicitly so standstill heating is deterministic rather than relying
	 * on the reset default. */
	st = TMC2209_WriteVerified(TMC2209_TPOWERDOWN,
			(uint32_t) TMC_TPOWERDOWN_SETTING, TMC_WRITE_RETRIES);
	if (st != TMC2209_OK) {
		init_failed_step = TMC2209_INIT_STEP_TIMING_WRITE;
		return st;
	}

	/* ── TPWMTHRS (write-only) ── the StealthChop/SpreadCycle crossover.
	 * Only meaningful in HYBRID; written as 0 otherwise so a stale value can
	 * never survive a warm reset and silently create a hybrid. */
#if (TMC_CHOPPER_MODE == TMC_CHOPPER_HYBRID)
	st = TMC2209_WriteVerified(TMC2209_TPWMTHRS, (uint32_t) TMC_TPWMTHRS,
			TMC_WRITE_RETRIES);
#else
	st = TMC2209_WriteVerified(TMC2209_TPWMTHRS, 0u, TMC_WRITE_RETRIES);
#endif
	if (st != TMC2209_OK) {
		init_failed_step = TMC2209_INIT_STEP_TIMING_WRITE;
		return st;
	}
	diag_data.timing_write_ok = true;
	TMC2209_KickWatchdog();

	/* ── Read back GCONF and compare ── */
	uint32_t val = 0;
	st = TMC2209_ReadRegister(TMC2209_GCONF, &val);
	diag_data.gconf_readback = val;
	diag_data.gconf_readback_ok = (st == TMC2209_OK);
	if (st != TMC2209_OK
			|| (val & TMC_GCONF_VERIFY_MASK)
					!= (gconf_val & TMC_GCONF_VERIFY_MASK)) {
		init_failed_step = TMC2209_INIT_STEP_GCONF_READBACK;
		return TMC2209_ERR_VERIFY;
	}

	/* ── Read back CHOPCONF and compare ── */
	val = 0;
	st = TMC2209_ReadRegister(TMC2209_CHOPCONF, &val);
	diag_data.chopconf_readback = val;
	diag_data.chopconf_readback_ok = (st == TMC2209_OK);
	if (st != TMC2209_OK
			|| (val & TMC_CHOPCONF_VERIFY_MASK)
					!= (chopconf_val & TMC_CHOPCONF_VERIFY_MASK)) {
		init_failed_step = TMC2209_INIT_STEP_CHOPCONF_READBACK;
		return TMC2209_ERR_VERIFY;
	}

	/* ── Sample DRV_STATUS ── informational, never fails the init. The motor
	 * is at standstill here, so STST will be set and the open-load flags
	 * (OLA/OLB) are NOT meaningful; the useful bits at boot are OT/OTPW and
	 * the short-detect flags, plus CS_ACTUAL reflecting IHOLD. */
	val = 0;
	diag_data.drv_status_ok = (TMC2209_ReadRegister(TMC2209_DRV_STATUS, &val)
			== TMC2209_OK);
	diag_data.drv_status = val;

	return TMC2209_OK;
}

/**
 * @brief  Last-resort unverified configuration write
 *
 * Runs ONLY after every verified attempt has failed. Writes the config at the
 * slow fallback baud with push-pull TX across all four possible addresses, so
 * the pump stays usable for diagnosis on a board whose RX path is broken.
 *
 * This path is UNVERIFIED by construction. The caller keeps the failing status
 * code and flags diag_data.blind_fallback so the boot log says so loudly —
 * it must never report success.
 */
static void TMC2209_WriteBlindFallback(uint32_t brr_base, uint32_t gconf_val,
		uint32_t chopconf_val, uint32_t ihold_irun_val) {
	USART2_SetBaudDivisor(brr_base, 12u);

	uint32_t otyper_save = GPIOA->OTYPER;
	GPIOA->OTYPER &= ~(1UL << 2); /* PA2 → push-pull */
	HAL_Delay(5);

	for (uint8_t addr = 0; addr < 4; addr++) {
		TMC2209_WriteRegisterAddr(addr, TMC2209_GCONF, gconf_val, NULL);
		TMC2209_WriteRegisterAddr(addr, TMC2209_CHOPCONF, chopconf_val, NULL);
		TMC2209_WriteRegisterAddr(addr, TMC2209_IHOLD_IRUN, ihold_irun_val,
				NULL);
		/* End on GCONF so en_spreadcycle is the last word for this address */
		TMC2209_WriteRegisterAddr(addr, TMC2209_GCONF, gconf_val, NULL);
	}

	GPIOA->OTYPER = otyper_save;
	TMC2209_KickWatchdog();
}

/* -------------------------------------------------------------------------- */
/*  Public API — Initialization                                               */
/* -------------------------------------------------------------------------- */

/**
 * @brief  Initialize TMC2209 via UART on USART2
 *
 * ZERO HAL DISRUPTION — direct register-level USART2 TX/RX throughout. No
 * HAL_UART_* calls, huart2 is never modified. Call BEFORE Log_Init(); after
 * init, USART2 continues as the debug UART with no state corruption.
 *
 * SEQUENCE (Phase 1, 2026-08-05 — verified-write rework):
 *   1. 100 ms power-up settle (charge pump + internal oscillator).
 *   2. For each baud in {debug (115200), debug/12 (~9600)}:
 *        a. Push-pull auto-baud training preamble, then back to open-drain.
 *        b. Resolve the slave address by READING IFCNT at 0..3.
 *        c. IFCNT-verified writes of GCONF, CHOPCONF, IHOLD_IRUN.
 *        d. Read GCONF and CHOPCONF back and compare against target.
 *        Success on any baud ends the loop.
 *   3. If every attempt failed: write the config unverified as a last resort
 *      (so the pump still runs for diagnosis) and return the failing status.
 *
 * The init result is honest: TMC2209_OK is returned ONLY when the chip has
 * confirmed the configuration. This matters because IRUN/IHOLD set the motor's
 * phase current — if the GCONF write is lost, I_SCALE_ANALOG reverts to its
 * power-on default of 1 and current falls back to the module's VREF pot.
 *
 * HARDWARE REQUIREMENT: R12 (5k pull-up from PDN_UART to 3V3) must be
 * populated, or the bus idles below VIH and all RX fails.
 * See AI-docs/R12_PDN_UART_Pullup_Fix.md.
 *
 * Register writes:
 *
 *   GCONF (0x00) = 0x000001C4 SpreadCycle / 0x000001C0 StealthChop+Hybrid
 *     I_SCALE_ANALOG   = 0   Use IRUN/IHOLD for current scaling (NOT the VREF pot)
 *     EN_SPREADCYCLE    = *   1 = SpreadCycle, 0 = StealthChop (TMC_CHOPPER_MODE)
 *     PDN_DISABLE       = 1   Disable PDN standby (line shared with debug)
 *     MSTEP_REG_SELECT  = 1   Microstep resolution from CHOPCONF register
 *     MULTISTEP_FILT    = 1   Enable STEP pulse input filter
 *
 *   CHOPCONF (0x6C) = 0x15010053 (at the default TMC_MICROSTEP = 8)
 *     TOFF  = 3               Off-time (enables chopper — TOFF=0 disables!)
 *     HSTRT = 5               Hysteresis start
 *     HEND  = 0               Hysteresis end
 *     TBL   = 2               Blank time 36 clocks (stable current sensing)
 *     VSENSE = 0              V_fs 325 mV — required for IRUN's current range
 *     MRES  = TMC_MRES        From TMC_MICROSTEP (default 1/8 → 1600 µsteps/rev)
 *     INTPOL = 1              MicroPlyer interpolates to 256 internally
 *
 *   IHOLD_IRUN (0x10) = 0x00061208  [WRITE-ONLY register — no read-back]
 *     IRUN  = 18              1.48 A peak / 1.05 A rms (2026-08-07 bench sweep;
 *                             table + rationale in tmc2209_uart.h)
 *     IHOLD = 8               0.70 A peak — standstill is the worst thermal case
 *     IHOLDDELAY = 6          Smooth run→hold current transition
 *
 *   TPOWERDOWN (0x11) = 20    ~0.44 s standstill → IHOLD  [WRITE-ONLY]
 *   TPWMTHRS   (0x13) = 0     StealthChop/SpreadCycle crossover, HYBRID only
 *                             [WRITE-ONLY] — written 0 outside HYBRID so a
 *                             stale value cannot survive a warm reset
 *
 *   De-energization strategy (per-state):
 *     HALT/STOP states:  EN=LOW (enabled)  → IHOLD maintains motor position for resume
 *     QUICK_STOP/NMT:    EN=HIGH (disabled) → coils fully de-energized regardless of IHOLD
 *     Motor only energized on explicit controller command (Stepper_Enable via state machine)
 *
 * @retval TMC2209_OK when the configuration is confirmed by the chip,
 *         TMC2209_ERR_UART_RX when no address answered (RX path dead),
 *         TMC2209_ERR_VERIFY when writes could not be confirmed or a
 *         read-back mismatched, TMC2209_ERR_UART_TX on a preamble TX failure.
 */
TMC2209_Status_t TMC2209_Init(void) {
	TMC2209_Status_t status = TMC2209_ERR_VERIFY;

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

	/* Read MS1/MS2 pin state — nominally determines the TMC2209 slave address */
	diag_data.ms1_level = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_7) ? 1 : 0; /* MS1 = PA7 */
	diag_data.ms2_level = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_6) ? 1 : 0; /* MS2 = PA6 */

	tmc2209_initialized = false;
	init_failed_step = TMC2209_INIT_STEP_NONE;
	g_tmc_addr = TMC2209_SLAVE_ADDR;

	/* Power-up delay (100 ms): the TMC2209's charge pump and internal 12 MHz
	 * oscillator need time after Vcc before they reliably receive UART traffic
	 * and auto-baud on a COLD boot. */
	HAL_Delay(100);

	/* ── Register values ── */

	/* GCONF — I_SCALE_ANALOG stays 0 so IRUN/IHOLD (not the module's VREF pot)
	 * set the current. If this write is ever lost the chip defaults it to 1 and
	 * the pot silently takes over, which is why init verifies rather than hopes.
	 * EN_SPREADCYCLE is set only for SpreadCycle; STEALTHCHOP and HYBRID both
	 * run the StealthChop chopper and differ only by TPWMTHRS. */
	const uint32_t gconf_val = GCONF_PDN_DISABLE | GCONF_MSTEP_REG_SELECT
			| GCONF_MULTISTEP_FILT
#if (TMC_CHOPPER_MODE == TMC_CHOPPER_SPREADCYCLE)
			| GCONF_EN_SPREADCYCLE
#endif
			;
	/* SpreadCycle = 0x000001C4, StealthChop/Hybrid = 0x000001C0 */

	/* CHOPCONF — vsense stays 0 (V_fs 325 mV) so IRUN keeps its full range. */
	const uint32_t chopconf_val = (3u << CHOPCONF_TOFF_SHIFT)
			| (5u << CHOPCONF_HSTRT_SHIFT) | (0u << CHOPCONF_HEND_SHIFT)
			| (2u << CHOPCONF_TBL_SHIFT)
			| ((uint32_t) TMC_MRES << CHOPCONF_MRES_SHIFT)
			| CHOPCONF_INTPOL;
	/* = 0x15010053 at the default TMC_MICROSTEP = 8 (MRES encoding 5):
	 * 200 full steps x 8 = 1600 pulses per pump rev, numerically identical to
	 * the WPX-1's 200 x 8 gearbox, so the entire step-frequency domain in
	 * stepper.c is unchanged. INTPOL still interpolates to 256 internally. */

	/* IHOLD_IRUN — see the current table in tmc2209_uart.h. Sized for the
	 * 17HS19-2004S1 (2.0 A/phase): IRUN 18 = 1.48 A peak, landed by the
	 * 2026-08-07 bench sweep (do not re-tune without new data). IHOLD 8 =
	 * 0.70 A peak. The old IRUN=31 (2.50 A peak, 2.5 W in the driver) is what
	 * cooked the board on the WPX-1. */
	const uint32_t ihold_irun_val =
			((uint32_t) TMC_IHOLDDELAY_SETTING << IHOLDDELAY_SHIFT)
					| ((uint32_t) TMC_IRUN_SETTING << IRUN_SHIFT)
					| ((uint32_t) TMC_IHOLD_SETTING << IHOLD_SHIFT);
	/* = 0x00061208 (IHOLDDELAY=6, IRUN=18, IHOLD=8) */

	const uint32_t brr_save = USART2->BRR;
	const uint32_t cr1_save = USART2->CR1;

	/* Baud divisor per attempt: 1 = debug baud, 12 = ~9600 cold-boot fallback */
	static const uint8_t baud_div[TMC_BAUD_ATTEMPTS] = { 1u, 12u };

	for (uint8_t attempt = 0; attempt < TMC_BAUD_ATTEMPTS; attempt++) {
		TMC2209_KickWatchdog();

		diag_data.config_baud_div = baud_div[attempt];
		USART2_SetBaudDivisor(brr_save, baud_div[attempt]);

		/* (a) Auto-baud training */
		diag_data.preamble_tx_ok = TMC2209_SendPreamble(gconf_val);
		diag_data.isr_after_preamble = USART2->ISR;
		if (!diag_data.preamble_tx_ok) {
			init_failed_step = TMC2209_INIT_STEP_GCONF_WRITE;
			status = TMC2209_ERR_UART_TX;
			continue;
		}
		HAL_Delay(10); /* let the baud lock settle before the first read */
		TMC2209_KickWatchdog();

		/* (b) Who is out there? */
		if (!TMC2209_ResolveAddress()) {
			init_failed_step = TMC2209_INIT_STEP_SMOKE_TEST;
			status = TMC2209_ERR_UART_RX;
			continue;
		}
		TMC2209_KickWatchdog();

		/* (c) + (d) Verified writes and read-back */
		status = TMC2209_ConfigureVerified(gconf_val, chopconf_val,
				ihold_irun_val);
		if (status == TMC2209_OK) {
			break;
		}
	}

	if (status == TMC2209_OK) {
		diag_data.config_verified = true;
		diag_data.blind_fallback = false;
		init_failed_step = TMC2209_INIT_STEP_COMPLETE;
		tmc2209_initialized = true;
	} else {
		/* Every verified attempt failed. Write the config anyway so the motor
		 * remains usable for diagnosis, but keep the failing status — this
		 * path must never masquerade as a successful init. */
		diag_data.config_verified = false;
		diag_data.blind_fallback = true;
		TMC2209_WriteBlindFallback(brr_save, gconf_val, chopconf_val,
				ihold_irun_val);
		/* Motor operations are deliberately NOT gated on this: the pump can
		 * still run (on defaults or on the unverified write), and the fault is
		 * surfaced via the return status, the boot log, and OD 0x2500/0x2501. */
		tmc2209_initialized = true;
	}

	/*
	 * ── Clean up and re-enable USART2 for debug UART ──
	 * Restore the debug baud, clear peripheral flags, re-enable NVIC. The HAL
	 * handle (huart2) was never touched — printf/Log_Init work immediately at
	 * the restored baud.
	 */
	USART2->CR1 &= ~USART_CR1_UE; /* disable UE to change BRR back */
	USART2->BRR = brr_save; /* restore original (debug) baud */
	USART2->CR1 = cr1_save; /* re-enable UE with original config */
	USART2_CleanupDirect();
	NVIC_ClearPendingIRQ(USART2_IRQn);
	HAL_NVIC_EnableIRQ(USART2_IRQn);
	TMC2209_KickWatchdog();

	return status;
}
