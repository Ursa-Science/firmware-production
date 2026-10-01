/**
 ******************************************************************************
 * @file    log.h
 * @brief   Non-blocking logging over SEGGER RTT with subsystem debug control
 * @note    All DBG_* macros route through Log_Write(), which formats into a
 *          stack buffer and writes it to SEGGER RTT channel 0 (a RAM buffer
 *          drained by the debug probe over SWD).  Same backend as the pump
 *          and phtemp; the old USART2 TXE ring buffer is gone (the valve was
 *          the last device on it, retired 2026-09-30).
 *          Call Log_Init(&huart2) once at boot (huart kept for API compat,
 *          unused).  Log_TxISR() is a legacy no-op retained for
 *          stm32g4xx_it.c.
 *
 *          View: probe-rs attach --chip STM32G431KBTx build/valve-n09-250k.elf
 *          The IWDG (2048 ms) keeps running under a non-halting attach; never
 *          halt the core from the viewer.
 ******************************************************************************
 */

#ifndef LOG_H
#define LOG_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdio.h>
#include "stm32g4xx_hal.h"

/*============================================================================*/
/*                        FORMAT BUFFER CONFIGURATION                         */
/*============================================================================*/

/** Max formatted message length (stack-allocated per Log_Write call) */
#define LOG_FMT_BUF_SIZE    128u

/*============================================================================*/
/*                         MASTER DEBUG CONTROL                               */
/*============================================================================*/

/**
 * DEBUG_MASTER_ENABLE - Global kill switch for ALL debug output
 *
 * Set to 1: Enable debug system (subsystem flags control individual output)
 * Set to 0: Production use - ALL debug output disabled regardless of flags
 */
#define DEBUG_MASTER_ENABLE     1

/**
 * DEBUG_LEVEL - Verbosity control
 *
 * 0 = Errors only (critical failures)
 * 1 = Normal (errors + state changes + key events)
 * 2 = Verbose (all debug output including timing measurements)
 */
#define DEBUG_LEVEL             1

/*============================================================================*/
/*                        SUBSYSTEM DEBUG FLAGS                               */
/*============================================================================*/
/* Enable/disable debug output for specific subsystems                        */
/* Only effective when DEBUG_MASTER_ENABLE = 1                                */
/*============================================================================*/

/** Valve state machine transitions, relay control, motion timing */
#define DBG_VALVE_ENABLE        1

/** Fail-safe close: heartbeat loss, NMT exit, stack error, watchdog recovery */
#define DBG_FAILSAFE_ENABLE     1

/** CAN/MCO stack events — NMT changes, heartbeat, RPDO/TPDO */
#define DBG_CAN_ENABLE          1

/** LED mode changes, PWM updates */
#define DBG_LED_ENABLE          0

/*============================================================================*/
/*                          LOG TYPES                                         */
/*============================================================================*/

/** Log levels (used for filtering at call site) */
typedef enum {
	LOG_LEVEL_ERROR = 0, LOG_LEVEL_INFO = 1, LOG_LEVEL_DEBUG = 2
} Log_Level_t;

/** Subsystem tags (for structured log output) */
typedef enum {
	LOG_SYS_MAIN = 0, LOG_SYS_VALVE, LOG_SYS_FAILSAFE, LOG_SYS_CAN, LOG_SYS_LED
} Log_Subsystem_t;

/*============================================================================*/
/*                          PUBLIC API                                        */
/*============================================================================*/

/**
 * @brief  Initialize logging — initialises the RTT control block
 * @param  huart  Unused (kept for API compatibility with the UART backend)
 * @note   Log_PutChar/Log_Write are safe before this call: RTT
 *         self-initialises on first write.
 */
void Log_Init(UART_HandleTypeDef *huart);

/**
 * @brief  Check if logging is initialized
 * @retval 1 if ready, 0 if not
 */
uint8_t Log_IsReady(void);

/**
 * @brief  Write a single character to RTT channel 0
 * @param  ch  Character to write
 * @note   Non-blocking — drops the character if the RTT buffer is full
 */
void Log_PutChar(uint8_t ch);

/**
 * @brief  Formatted log output to RTT channel 0 (non-blocking)
 * @param  lvl  Log level (for filtering)
 * @param  sys  Subsystem identifier
 * @param  fmt  printf-style format string
 * @note   Filtering is done at the macro level (DBG_* macros).
 *         lvl and sys params are reserved for future use.
 */
void Log_Write(Log_Level_t lvl, Log_Subsystem_t sys, const char *fmt, ...);

/**
 * @brief  Legacy no-op — RTT needs no TX interrupt
 * @note   Still referenced by USART2_IRQHandler() in stm32g4xx_it.c so the
 *         vector links; the USART2 IRQ is never enabled.
 */
void Log_TxISR(void);

/*============================================================================*/
/*                          DEBUG MACROS                                      */
/*============================================================================*/

/* Legacy compatibility */
#define ENABLE_DIAGNOSTICS      DEBUG_MASTER_ENABLE

#if DEBUG_MASTER_ENABLE

/**
 * DBG_PRINT(subsystem, fmt, ...) - Print if subsystem enabled
 * @param subsystem  One of: VALVE, FAILSAFE, CAN, LED
 * @param fmt        printf format string (no \r\n needed, auto-appended)
 */
#define DBG_PRINT(subsystem, fmt, ...) \
    do { \
        if (DBG_##subsystem##_ENABLE) { \
            Log_Write(LOG_LEVEL_INFO, LOG_SYS_##subsystem, \
                      "[" #subsystem "] " fmt "\r\n", ##__VA_ARGS__); \
        } \
    } while(0)

/**
 * DBG_PRINT_V(subsystem, fmt, ...) - Print only at verbose level (2)
 */
#define DBG_PRINT_V(subsystem, fmt, ...) \
    do { \
        if (DBG_##subsystem##_ENABLE && DEBUG_LEVEL >= 2) { \
            Log_Write(LOG_LEVEL_DEBUG, LOG_SYS_##subsystem, \
                      "[" #subsystem "] " fmt "\r\n", ##__VA_ARGS__); \
        } \
    } while(0)

/**
 * DBG_ENTER(subsystem, func) - Function entry trace (verbose only)
 */
#define DBG_ENTER(subsystem, func) \
    do { \
        if (DBG_##subsystem##_ENABLE && DEBUG_LEVEL >= 2) { \
            Log_Write(LOG_LEVEL_DEBUG, LOG_SYS_##subsystem, \
                      "[" #subsystem "] >> %s()\r\n", func); \
        } \
    } while(0)

/**
 * DBG_ERROR(subsystem, fmt, ...) - Always print errors (level 0+)
 */
#define DBG_ERROR(subsystem, fmt, ...) \
    do { \
        if (DBG_##subsystem##_ENABLE) { \
            Log_Write(LOG_LEVEL_ERROR, LOG_SYS_##subsystem, \
                      "[" #subsystem " ERROR] " fmt "\r\n", ##__VA_ARGS__); \
        } \
    } while(0)

/**
 * DBG_STATE(subsystem, fmt, ...) - State change logging (level 1+)
 */
#define DBG_STATE(subsystem, fmt, ...) \
    do { \
        if (DBG_##subsystem##_ENABLE && DEBUG_LEVEL >= 1) { \
            Log_Write(LOG_LEVEL_INFO, LOG_SYS_##subsystem, \
                      "[" #subsystem "] " fmt "\r\n", ##__VA_ARGS__); \
        } \
    } while(0)

/**
 * DBG_HW(subsystem, fmt, ...) - Hardware register logging (verbose only)
 */
#define DBG_HW(subsystem, fmt, ...) \
    do { \
        if (DBG_##subsystem##_ENABLE && DEBUG_LEVEL >= 2) { \
            Log_Write(LOG_LEVEL_DEBUG, LOG_SYS_##subsystem, \
                      "[" #subsystem " HW] " fmt "\r\n", ##__VA_ARGS__); \
        } \
    } while(0)

/**
 * DBG_BLOCK(subsystem) - Execute code block only if subsystem debug enabled
 * Usage: DBG_BLOCK(VALVE) { ... diagnostic code ... }
 */
#define DBG_BLOCK(subsystem) \
    if (DBG_##subsystem##_ENABLE)

/**
 * DBG_BLOCK_V(subsystem) - Execute code block only at verbose level
 */
#define DBG_BLOCK_V(subsystem) \
    if (DBG_##subsystem##_ENABLE && DEBUG_LEVEL >= 2)

#else /* DEBUG_MASTER_ENABLE == 0 */

/* No-op macros when debug disabled (zero overhead) */
#define DBG_PRINT(subsystem, fmt, ...)      ((void)0)
#define DBG_PRINT_V(subsystem, fmt, ...)    ((void)0)
#define DBG_ENTER(subsystem, func)          ((void)0)
#define DBG_ERROR(subsystem, fmt, ...)      ((void)0)
#define DBG_STATE(subsystem, fmt, ...)      ((void)0)
#define DBG_HW(subsystem, fmt, ...)         ((void)0)
#define DBG_BLOCK(subsystem)                if (0)
#define DBG_BLOCK_V(subsystem)              if (0)

#endif /* DEBUG_MASTER_ENABLE */

#ifdef __cplusplus
}
#endif

#endif /* LOG_H */
