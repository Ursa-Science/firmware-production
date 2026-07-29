/**
 ******************************************************************************
 * @file    motor_control.h
 * @brief   Motor Control Bridge for MCO Stack Integration
 * @author  CanOpen Motor Control Project
 * @date    2026
 ******************************************************************************
 */

#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include <stdbool.h>
#include "main.h"

/* Exported types ------------------------------------------------------------*/

/**
 * @brief Motor control state machine states
 * @note Used to prevent duplicate command execution in RPDO mode
 */
typedef enum {
	MOTOR_STATE_DISABLED = 0,      // Motor driver disabled
	MOTOR_STATE_ENABLED_STOPPED,   // Driver enabled but motor stopped
	MOTOR_STATE_RUNNING,           // Motor actively running
	MOTOR_STATE_QUICKSTOPPING,     // Quick stop in progress
	MOTOR_STATE_FAULT              // Fault condition
} MotorState_t;

/* Exported functions --------------------------------------------------------*/

/**
 * @brief Initialize motor control bridge
 * @retval true if successful
 */
bool MotorControl_Init(void);

/**
 * @brief Process motor control (call from MCO main loop)
 */
void MotorControl_Process(void);

/**
 * @brief Check if motor control is operational
 * @retval true if motors can be controlled
 */
bool MotorControl_IsOperational(void);

/**
 * @brief Emergency stop from MCO error handling
 */
void MotorControl_EmergencyStop(void);

/**
 * @brief Reset motor control on NMT state change
 */
void MotorControl_Reset(void);

/**
 * @brief Register motor control as MCO event listener
 * @note  Call from main() before MCOUSER_ResetCommunication()
 */
void MotorControl_RegisterMCOEvents(void);

/**
 * @brief Run all periodic diagnostics (1 s shared gate)
 * @note  Consolidates PrintStatus, PrintTimerStats, PrintDiagnostics behind a
 *        single public entry point with a 1 s early-exit gate.  Each sub-function
 *        retains its own internal throttle (1 s or 5 s).
 *        Call from the main while(1) loop.
 */
void MotorControl_RunDiagnostics(void);

#ifdef __cplusplus
}
#endif

#endif /* MOTOR_CONTROL_H */
