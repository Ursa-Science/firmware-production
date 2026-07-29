/**
 ******************************************************************************
 * @file    log.c
 * @brief   Non-blocking ring-buffer logging with USART2 TXE interrupt drain
 * @note    All DBG_* macros in debug_config.h route through Log_Write(),
 *          which enqueues formatted text into a 1 KB ring buffer.  The USART2 TXE ISR drains the
 *          buffer one byte per interrupt, giving zero-blocking behaviour.
 ******************************************************************************
 */

#include "log.h"
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

/* ─── Private state ──────────────────────────────────────────────────────── */

typedef struct {
	uint8_t buf[LOG_RING_SIZE];
	volatile uint16_t head; /**< Next write position (main context)  */
	volatile uint16_t tail; /**< Next read  position (ISR context)   */
	UART_HandleTypeDef *huart; /**< USART2 handle                       */
	uint8_t ready; /**< Set after Log_Init()                */
} Log_Ring_t;

static Log_Ring_t ring = { .head = 0, .tail = 0, .huart = NULL, .ready = 0 };

/* ─── Helpers ────────────────────────────────────────────────────────────── */

/**
 * @brief  Enqueue one byte into the ring buffer (ISR-safe)
 * @retval 1 on success, 0 if buffer full (byte dropped)
 */
static inline uint8_t ring_put(uint8_t ch) {
	uint16_t next = (ring.head + 1u) & LOG_RING_MASK;

	/* Read tail with interrupts briefly disabled to prevent a torn read
	 if the TXE ISR fires between the two bytes of a 16-bit load.
	 On Cortex-M4, 16-bit aligned loads are inherently atomic, but we
	 keep the guard for portability and to satisfy static analysers. */
	uint32_t primask = __get_PRIMASK();
	__disable_irq();
	uint16_t t = ring.tail;
	__set_PRIMASK(primask);

	if (next == t) {
		return 0; /* Full — silently drop */
	}

	ring.buf[ring.head] = ch;
	ring.head = next;
	return 1;
}

/**
 * @brief  Kick the TXE interrupt so draining starts (idempotent)
 */
static inline void ring_kick_tx(void) {
	if (ring.huart != NULL) {
		__HAL_UART_ENABLE_IT(ring.huart, UART_IT_TXE);
	}
}

/* ─── Public API ─────────────────────────────────────────────────────────── */

void Log_Init(UART_HandleTypeDef *huart) {
	ring.huart = huart;
	ring.head = 0;
	ring.tail = 0;
	ring.ready = 1;

	/* Make sure TXE interrupt is disabled until we have data */
	__HAL_UART_DISABLE_IT(huart, UART_IT_TXE);
}

uint8_t Log_IsReady(void) {
	return ring.ready;
}

void Log_PutChar(uint8_t ch) {
	ring_put(ch);
	ring_kick_tx();
}

void Log_Write(Log_Level_t lvl, Log_Subsystem_t sys, const char *fmt, ...) {
	(void) lvl; /* Filtering is done at the macro level in debug_config.h */
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

	/* Enqueue the formatted string into the ring buffer */
	for (int i = 0; i < len; i++) {
		if (!ring_put((uint8_t) tmp[i])) {
			break; /* Buffer full — stop, don't block */
		}
	}

	ring_kick_tx();
}

/* ─── ISR entry point ────────────────────────────────────────────────────── */

void Log_TxISR(void) {
	if (ring.huart == NULL) {
		return;
	}

	USART_TypeDef *uart = ring.huart->Instance;

	/* Check that TXE (TX data register empty) interrupt is both
	 enabled and pending.  STM32G4 names the flag TXE_TXFNF. */
	if ((uart->CR1 & USART_CR1_TXEIE_TXFNFIE)
			&& (uart->ISR & USART_ISR_TXE_TXFNF)) {
		uint16_t h = ring.head; /* Atomic 16-bit read on Cortex-M4 */
		uint16_t t = ring.tail;

		if (t != h) {
			uart->TDR = ring.buf[t];
			ring.tail = (t + 1u) & LOG_RING_MASK;
		} else {
			/* Buffer empty — disable TXE to stop further interrupts */
			__HAL_UART_DISABLE_IT(ring.huart, UART_IT_TXE);
		}
	}
}
