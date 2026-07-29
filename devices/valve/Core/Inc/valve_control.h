/**************************************************************************
 MODULE:    VALVE_CONTROL
 CONTAINS:  CiA 408 valve state machine — types, constants, and API
 Manages DISABLED ↔ IDLE ↔ OPENING/CLOSING ↔ FAULT states.
 Processes ControlWord (RPDO1), updates StatusWord + ValveState
 (TPDO1) and ErrorRegister (TPDO2) in the process image.
 ***************************************************************************/

#ifndef _VALVE_CONTROL_H
#define _VALVE_CONTROL_H

#include <stdint.h>
#include <stdbool.h>

/**************************************************************************
 DEFINES: Valve FSM States (internal)
 ***************************************************************************/
typedef enum {
	VALVE_STATE_DISABLED = 0, /* Power-on default; NMT not Operational */
	VALVE_STATE_IDLE = 1, /* At known position, ready for commands */
	VALVE_STATE_OPENING = 2, /* Relay ON, moving toward open */
	VALVE_STATE_CLOSING = 3, /* Relay OFF, moving toward closed */
	VALVE_STATE_FAULT = 4 /* Error condition, fail-safe applied */
} ValveState_FSM_t;

/**************************************************************************
 DEFINES: ValveState OD values [0x6042] — reported in TPDO1
 ***************************************************************************/
#define VALVE_POS_UNKNOWN   0   /* Position not determined (after HALT) */
#define VALVE_POS_CLOSED    1   /* Valve fully closed (relay OFF) */
#define VALVE_POS_OPEN      2   /* Valve fully open (relay ON) */
#define VALVE_POS_MOVING    3   /* Valve in transition */

/**************************************************************************
 DEFINES: ControlWord bit masks [0x6040]
 ***************************************************************************/
#define CW_OPEN             (1U << 0)   /* Bit 0: Open command (rising edge) */
#define CW_CLOSE            (1U << 1)   /* Bit 1: Close command (rising edge) */
#define CW_ENABLE_OP        (1U << 3)   /* Bit 3: Enable operation (level) */
#define CW_FAULT_RESET      (1U << 7)   /* Bit 7: Fault reset (rising edge) */
#define CW_HALT             (1U << 8)   /* Bit 8: Halt (rising edge) */

/**************************************************************************
 DEFINES: StatusWord bit masks [0x6041]
 ***************************************************************************/
#define SW_CLOSED            (1U << 0)   /* Bit 0: Valve fully closed */
#define SW_OPENED            (1U << 1)   /* Bit 1: Valve fully open */
#define SW_MOVING            (1U << 2)   /* Bit 2: Valve in transition */
#define SW_FAULT             (1U << 3)   /* Bit 3: Fault active */
#define SW_REMOTE            (1U << 9)   /* Bit 9: NMT Operational */
#define SW_TARGET_REACHED    (1U << 10)  /* Bit 10: Reached commanded position */

/**************************************************************************
 DEFINES: FailSafePosition values [0x2100]
 ***************************************************************************/
#define FAILSAFE_AS_IS       0
#define FAILSAFE_OPEN        1
#define FAILSAFE_CLOSED      2

/**************************************************************************
 GLOBAL FUNCTIONS
 ***************************************************************************/

/**
 * @brief  Initialize valve control state machine
 *         Registers for MCO events, sets initial state to DISABLED,
 *         applies fail-safe on boot if watchdog reset detected.
 */
void ValveControl_Init(void);

/**
 * @brief  Main loop entry point — call every iteration
 *         NMT gate → state machine tick → process image update
 */
void ValveControl_Process(void);

/**
 * @brief  Apply fail-safe position based on OD 0x2100
 *         Called on heartbeat loss, NMT non-Operational, IWDG reset
 */
void ValveControl_ApplyFailSafe(void);

/**
 * @brief  1 Hz diagnostic output via UART
 *         Call from main loop; internally rate-limits to 1 Hz
 */
void ValveControl_RunDiagnostics(void);

/**
 * @brief  Get current FSM state (for LED control, etc.)
 */
ValveState_FSM_t ValveControl_GetState(void);

/**
 * @brief  Get current valve position (OD 0x6042 value)
 */
uint8_t ValveControl_GetPosition(void);

#endif // _VALVE_CONTROL_H
