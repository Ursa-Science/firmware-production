/**
 ******************************************************************************
 * @file    log.h
 * @brief   Non-blocking ring-buffer logging with subsystem debug control
 * @note    All DBG_* macros route through Log_Write(), which enqueues
 *          formatted text into a 1 KB ring buffer.  The USART2 TXE ISR
 *          drains the buffer one byte per interrupt, giving zero-blocking
 *          behaviour.
 *
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
/*                         RING BUFFER CONFIGURATION                          */
/*============================================================================*/

/** Ring buffer size — must be power of 2 */
#define LOG_RING_SIZE       1024u
#define LOG_RING_MASK       (LOG_RING_SIZE - 1u)

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
#define DEBUG_LEVEL             2

/*============================================================================*/
/*                        SUBSYSTEM DEBUG FLAGS                               */
/*============================================================================*/
/* Enable/disable debug output for specific subsystems                        */
/* Only effective when DEBUG_MASTER_ENABLE = 1                                */
/*============================================================================*/

/** pH sensor driver (ph_sensor.c) — ADC reads, Nernst calculation, quality */
#define DBG_PH_ENABLE           1

/** Temperature sensor driver (temp_sensor.c) — DS18B20 1-Wire, conversion */
#define DBG_TEMP_ENABLE         1

/** Sensor control state machine (sensor_control.c) — CiA 404 states, TPDO */
#define DBG_SENSOR_ENABLE       1

/** Calibration subsystem — multi-point pH cal, temp offset */
#define DBG_CAL_ENABLE          1

/** CAN/MCO stack events — NMT changes, heartbeat, RPDO/TPDO */
#define DBG_CAN_ENABLE          0

/*============================================================================*/
/*                          LOG TYPES                                         */
/*============================================================================*/

/** Log levels (used for filtering at call site) */
typedef enum {
	LOG_LEVEL_ERROR = 0, LOG_LEVEL_INFO = 1, LOG_LEVEL_DEBUG = 2
} Log_Level_t;

/** Subsystem tags (for structured log output) */
typedef enum {
	LOG_SYS_MAIN = 0,
	LOG_SYS_PH,
	LOG_SYS_TEMP,
	LOG_SYS_SENSOR,
	LOG_SYS_CAL,
	LOG_SYS_CAN
} Log_Subsystem_t;

/*============================================================================*/
/*                          PUBLIC API                                        */
/*============================================================================*/

/**
 * @brief  Initialize logging — stores UART handle, resets ring buffer
 * @param  huart  Pointer to USART2 handle (debug UART)
 * @note   Call AFTER MX_USART2_UART_Init() and BEFORE any log output.
 *         The caller must also enable USART2 global interrupt in NVIC.
 */
void Log_Init(UART_HandleTypeDef *huart);

/**
 * @brief  Check if logging is initialized
 * @retval 1 if ready, 0 if not
 */
uint8_t Log_IsReady(void);

/**
 * @brief  Enqueue a single character into the ring buffer
 * @param  ch  Character to enqueue
 * @note   Non-blocking — drops character if buffer full
 */
void Log_PutChar(uint8_t ch);

/**
 * @brief  Formatted log output — enqueues into ring buffer (non-blocking)
 * @param  lvl  Log level (for filtering)
 * @param  sys  Subsystem identifier
 * @param  fmt  printf-style format string
 * @note   Filtering is done at the macro level (DBG_* macros).
 *         lvl and sys params are reserved for future use.
 */
void Log_Write(Log_Level_t lvl, Log_Subsystem_t sys, const char *fmt, ...);

/**
 * @brief  USART2 TXE ISR entry point — drains ring buffer one byte per call
 * @note   Call this from USART2_IRQHandler() in stm32g4xx_it.c
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
 * @param subsystem  One of: PH, TEMP, SENSOR, CAL, CAN
 * @param fmt        printf format string
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
 * DBG_STATE(su bsystem
 , fmt, ...) - State change logging (level 1+)
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
 * Usage: DBG_BLOCK(PH) { ... diagnostic code ... }
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
