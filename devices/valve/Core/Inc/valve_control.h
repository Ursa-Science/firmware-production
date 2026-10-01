/**************************************************************************
 MODULE:    VALVE_CONTROL
 CONTAINS:  CiA 408 valve state machine — types, constants, and API
 Manages DISABLED ↔ IDLE ↔ OPENING/CLOSING (↔ FAULT) states.
 Processes ControlWord (RPDO1), updates StatusWord + ValveState
 (TPDO1) and ErrorRegister (TPDO2 + SDO) in the process image.
 Fail-safe is hard-wired CLOSED (relay OFF) — see ValveControl_ForceClosed().
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
	VALVE_STATE_FAULT = 4 /* Contract placeholder: no code path enters it today
	                         (no feedback sensor). SW bit 3 / CW bit 7 kept so
	                         the wire contract is stable. */
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
#define CW_FAULT_RESET      (1U << 7)   /* Bit 7: Fault / error reset (rising edge) */
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
 GLOBAL FUNCTIONS
 ***************************************************************************/

/**
 * @brief  Initialize valve control state machine
 *         Registers for MCO events, sets initial state to DISABLED,
 *         relay OFF (closed).
 */
void ValveControl_Init(void);

/**
 * @brief  Main loop entry point — call every iteration
 *         NMT gate → state machine tick → process image update
 */
void ValveControl_Process(void);

/**
 * @brief  Fail-safe: drive the relay OFF (valve CLOSED) and forget any
 *         buffered Open command, so the valve never re-opens by itself.
 * @param  reason  Short text for the RTT log (e.g. "master heartbeat lost")
 * @note   Idempotent — safe to call repeatedly (events repeat). Called from
 *         the heartbeat-lost event (synchronously, inside the stack callback),
 *         the NMT gate on any exit from Operational, and the stack
 *         fatal-error event (fatal class only). Boot and IWDG-reset are
 *         covered by MX_GPIO_Init/ValveDriver_Init driving the relay LOW.
 *         Re-opening afterwards needs NMT Operational AND a fresh 0→1 edge on
 *         ControlWord bit 0 (with bit 3 set).
 */
void ValveControl_ForceClosed(const char *reason);

/**
 * @brief  1 Hz diagnostic output via RTT
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
