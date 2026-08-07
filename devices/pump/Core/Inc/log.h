/**
 ******************************************************************************
 * @file    log.h
 * @brief   Non-blocking logging — ring buffer, debug macros, subsystem control
 ******************************************************************************
 *
 * Single header for the entire debug/logging subsystem.
 *
 * CONFIGURATION (edit the defines below):
 *   DEBUG_MASTER_ENABLE  — global kill switch (0 = production, zero overhead)
 *   DEBUG_LEVEL          — 0 errors, 1 normal, 2 verbose
 *   DBG_*_ENABLE         — per-subsystem on/off
 *
 * APPLICATION USAGE:
 *   DBG_PRINT(MOTOR, "speed=%u", rpm);    // normal
 *   DBG_PRINT_V(MOTOR, "arr=%lu", arr);   // verbose only
 *   DBG_ERROR(CAN, "bus off!");           // always (when subsystem enabled)
 *   DBG_STATE(MOTOR, "IDLE→RUN");         // state changes
 *   DBG_HW(CAN, "NBTP=0x%08lX", reg);    // register dumps
 *   DBG_BLOCK(CAN) { ... }               // conditional code block
 *
 * TRANSPORT:
 *   Output goes to SEGGER RTT channel 0 (RAM buffer drained by the debug
 *   probe over SWD, NO_BLOCK_SKIP) — zero blocking, and the UART shared
 *   with the TMC2209 is never touched.  See log.c for host-side viewers.
 *
 *   Call Log_Init(&huart2) once at boot (huart kept for API compat, unused).
 *   Log_TxISR() is a legacy no-op retained for stm32g4xx_it.c.
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
/*                         MASTER DEBUG CONTROL                               */
/*============================================================================*/

/**
 * Set to 1 for development, 0 for production (zero overhead).
 */
#define DEBUG_MASTER_ENABLE     1

/**
 * 0 = errors only   1 = + state changes   2 = + verbose / HW
 */
#define DEBUG_LEVEL             2

/*============================================================================*/
/*                        SUBSYSTEM DEBUG FLAGS                               */
/*============================================================================*/

#define DBG_STEPPER_ENABLE      1   /**< stepper.c low-level control     */
#define DBG_MOTOR_ENABLE        1   /**< motor_control.c state machine   */
#define DBG_DOSE_ENABLE         1   /**< Dose tracking                   */
#define DBG_TIMER_ENABLE        1   /**< Timer/PWM hardware              */
#define DBG_IDLE_ENABLE         0   /**< IDLE state monitoring           */
#define DBG_CAN_ENABLE          0   /**< CAN/FDCAN diagnostics           */
#define DBG_GENERAL_ENABLE      1   /**< General / boot-up messages      */

/* Legacy alias */
#define ENABLE_DIAGNOSTICS      DEBUG_MASTER_ENABLE

/*============================================================================*/
/*                        FORMAT BUFFER CONFIGURATION                         */
/*============================================================================*/

#define LOG_FMT_BUF_SIZE    128u            /**< Stack buffer in Log_Write */

/*============================================================================*/
/*                              TYPES                                         */
/*============================================================================*/

typedef enum {
	LOG_ERROR = 0, LOG_STATE = 1, LOG_INFO = 1, LOG_VERBOSE = 2, LOG_HW = 2
} Log_Level_t;

typedef enum {
	LOG_SYS_STEPPER = 0,
	LOG_SYS_MOTOR,
	LOG_SYS_DOSE,
	LOG_SYS_TIMER,
	LOG_SYS_IDLE,
	LOG_SYS_CAN,
	LOG_SYS_GENERAL,
	LOG_SYS_COUNT
} Log_Subsystem_t;

/*============================================================================*/
/*                            PUBLIC API                                      */
/*============================================================================*/

/** Initialise ring buffer + UART handle.  Call once after UART init. */
void Log_Init(UART_HandleTypeDef *huart);

/** Non-blocking formatted write to ring buffer. */
void Log_Write(Log_Level_t lvl, Log_Subsystem_t sys, const char *fmt, ...) __attribute__((format(printf, 3, 4)));

/** Enqueue a single raw byte (used by __io_putchar). */
void Log_PutChar(uint8_t ch);

/** USART TXE ISR — call from USART2_IRQHandler. */
void Log_TxISR(void);

/** True after Log_Init() has been called. */
uint8_t Log_IsReady(void);

/*============================================================================*/
/*                          DEBUG MACROS                                      */
/*============================================================================*/

#if DEBUG_MASTER_ENABLE

#define DBG_PRINT(subsystem, fmt, ...) \
    do { if (DBG_##subsystem##_ENABLE) { \
        Log_Write(LOG_INFO, LOG_SYS_##subsystem, \
                  "[" #subsystem "] " fmt "\r\n", ##__VA_ARGS__); \
    } } while(0)

#define DBG_PRINT_V(subsystem, fmt, ...) \
    do { if (DBG_##subsystem##_ENABLE && DEBUG_LEVEL >= 2) { \
        Log_Write(LOG_VERBOSE, LOG_SYS_##subsystem, \
                  "[" #subsystem "] " fmt "\r\n", ##__VA_ARGS__); \
    } } while(0)

#define DBG_ENTER(subsystem, func) \
    do { if (DBG_##subsystem##_ENABLE && DEBUG_LEVEL >= 2) { \
        Log_Write(LOG_VERBOSE, LOG_SYS_##subsystem, \
                  "[" #subsystem "] >> %s()\r\n", func); \
    } } while(0)

#define DBG_ERROR(subsystem, fmt, ...) \
    do { if (DBG_##subsystem##_ENABLE) { \
        Log_Write(LOG_ERROR, LOG_SYS_##subsystem, \
                  "[" #subsystem " ERROR] " fmt "\r\n", ##__VA_ARGS__); \
    } } while(0)

#define DBG_STATE(subsystem, fmt, ...) \
    do { if (DBG_##subsystem##_ENABLE && DEBUG_LEVEL >= 1) { \
        Log_Write(LOG_STATE, LOG_SYS_##subsystem, \
                  "[" #subsystem "] " fmt "\r\n", ##__VA_ARGS__); \
    } } while(0)

#define DBG_HW(subsystem, fmt, ...) \
    do { if (DBG_##subsystem##_ENABLE && DEBUG_LEVEL >= 2) { \
        Log_Write(LOG_HW, LOG_SYS_##subsystem, \
                  "[" #subsystem " HW] " fmt "\r\n", ##__VA_ARGS__); \
    } } while(0)

#define DBG_BLOCK(subsystem)    if (DBG_##subsystem##_ENABLE)
#define DBG_BLOCK_V(subsystem)  if (DBG_##subsystem##_ENABLE && DEBUG_LEVEL >= 2)

#else /* DEBUG_MASTER_ENABLE == 0 — zero overhead */

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
