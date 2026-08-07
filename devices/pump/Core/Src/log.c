/**
 ******************************************************************************
 * @file    log.c
 * @brief   Non-blocking logging over SEGGER RTT (SWD) — USART2 is TMC-only
 * @note    All DBG_* macros in log.h route through Log_Write(), which formats
 *          into a stack buffer and hands it to SEGGER_RTT_Write().  The RTT
 *          up-buffer lives in RAM and is drained by the debug probe over
 *          SWDIO/SWCLK in the background — the UART (shared with the TMC2209
 *          on PDN_UART) is never touched by logging.
 *
 *          RTT mode is NO_BLOCK_SKIP: with no probe attached (or a full
 *          buffer) writes are dropped, never blocked — same contract as the
 *          old USART2 ring buffer.  The buffer is drained only by the host,
 *          so with no probe attached it retains the FIRST BUFFER_SIZE_UP
 *          bytes of boot output (readable later via GDB: _SEGGER_RTT).
 *
 *          View live with any of:
 *            probe-rs attach --chip STM32G431KBTx build/pump.elf   (ST-LINK)
 *            OpenOCD rtt server                                    (ST-LINK)
 *            JLinkRTTViewer                                        (J-Link)
 ******************************************************************************
 */

#include "log.h"
#include "SEGGER_RTT.h"
#include <stdarg.h>
#include <stdio.h>

/* ─── Private state ──────────────────────────────────────────────────────── */

static uint8_t log_ready = 0;

/* ─── Public API ─────────────────────────────────────────────────────────── */

void Log_Init(UART_HandleTypeDef *huart) {
	/* huart is kept for API compatibility but unused: logging no longer
	 * touches USART2 (it belongs exclusively to the TMC2209). */
	(void) huart;
	SEGGER_RTT_Init();
	log_ready = 1;
}

uint8_t Log_IsReady(void) {
	return log_ready;
}

void Log_PutChar(uint8_t ch) {
	/* Safe before Log_Init(): SEGGER_RTT_Write self-initialises on first use. */
	SEGGER_RTT_Write(0, &ch, 1);
}

void Log_Write(Log_Level_t lvl, Log_Subsystem_t sys, const char *fmt, ...) {
	(void) lvl; /* Filtering is done at the macro level in log.h */
	(void) sys;

	char tmp[LOG_FMT_BUF_SIZE];
	va_list ap;
	va_start(ap, fmt);
	int len = vsnprintf(tmp, sizeof(tmp), fmt, ap);
	va_end(ap);

	if (len <= 0) {
		return;
	}
	if ((unsigned) len >= sizeof(tmp)) {
		len = sizeof(tmp) - 1; /* Truncated */
	}

	SEGGER_RTT_Write(0, tmp, (unsigned) len);
}

/* ─── ISR entry point (legacy) ───────────────────────────────────────────── */

void Log_TxISR(void) {
	/* No-op: RTT needs no TX interrupt.  Kept so USART2_IRQHandler in
	 * stm32g4xx_it.c links unchanged; USART2 TXE is never enabled. */
}
