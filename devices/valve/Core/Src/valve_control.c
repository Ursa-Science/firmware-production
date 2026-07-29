/**************************************************************************
 MODULE:    VALVE_CONTROL
 CONTAINS:  CiA 408 valve state machine implementation
 Processes ControlWord via RPDO1, drives relay via valve_driver,
 updates StatusWord + ValveState (TPDO1) and ErrorRegister (TPDO2)
 in the MCO process image.

 State machine: DISABLED ↔ IDLE ↔ OPENING/CLOSING ↔ FAULT
 NMT-gated: must be NMT Operational for valve to respond.
 Edge detection on ControlWord bits (same pattern as pump module).
 Time-based motion model (no position sensor).

 COPYRIGHT: Ursa Science 2026
 ***************************************************************************/

#include "valve_control.h"
#include "valve_driver.h"
#include "procimg_api.h"
#include "mco_events.h"
#include "log.h"
#include "main.h"
#include "mcop_inc.h"

/**************************************************************************
 LOCAL DEFINES
 ***************************************************************************/
/* Clippard EV-2M-24 solenoid valve: response time 5-10 ms nominal (datasheet)
 * — it snaps to position, there is no mechanical travel to wait out. With no
 * position sensor we declare the commanded position reached after a short fixed
 * settle (generous margin over the 10 ms response plus relay/loop jitter),
 * rather than waiting the multi-second MotionTimeout. MotionTimeout (0x2300) is
 * retained only as the stuck-valve fault sanity bound, not for normal timing. */
#define VALVE_MOTION_SETTLE_MS 50u

/**************************************************************************
 LOCAL VARIABLES
 ***************************************************************************/
static ValveState_FSM_t fsm_state; /* Current FSM state */
static uint8_t valve_position; /* Current position (VALVE_POS_*) */
static uint16_t last_control_word; /* Previous CW for edge detection */
static uint32_t motion_start_ms; /* HAL_GetTick() when motion began */
static uint16_t status_word; /* Current StatusWord shadow */
static uint8_t error_register; /* Current ErrorRegister shadow */
static uint32_t diag_last_ms; /* Last diagnostics print timestamp */
static bool fault_active; /* Fault flag */

/**************************************************************************
 LOCAL FUNCTION PROTOTYPES
 ***************************************************************************/
static void ValveControl_ProcessControlWord(void);
static void ValveControl_TickMotion(void);
static uint16_t ValveControl_GenerateStatusWord(void);
static void ValveControl_UpdateProcessImage(void);
static void ValveControl_SetState(ValveState_FSM_t new_state);
static void ValveControl_OnNMTChange(const MCO_Event_t *event);
static void ValveControl_OnHeartbeatLost(const MCO_Event_t *event);

/**************************************************************************
 GLOBAL FUNCTIONS
 ***************************************************************************/

void ValveControl_Init(void) {
	fsm_state = VALVE_STATE_DISABLED;
	valve_position = VALVE_POS_CLOSED; /* Default per EDS (pimg.h default = 0x01) */
	last_control_word = 0;
	motion_start_ms = 0;
	status_word = 0;
	error_register = 0;
	diag_last_ms = 0;
	fault_active = false;

	/* Initialize valve driver (relay OFF) */
	ValveDriver_Init();

	/* Register for MCO events */
	MCO_Events_Register(MCO_EVENT_NMT_CHANGE, ValveControl_OnNMTChange);
	MCO_Events_Register(MCO_EVENT_HEARTBEAT_LOST, ValveControl_OnHeartbeatLost);

	/* Update process image with initial values */
	ValveControl_UpdateProcessImage();

	DBG_STATE(VALVE, "Init complete, state=DISABLED");
}

void ValveControl_Process(void) {
	/*--------------------------------------------------------------
	 * NMT Gate: If not Operational, force DISABLED
	 *--------------------------------------------------------------*/
	if (MY_NMT_STATE != NMTSTATE_OP) {
		if (fsm_state != VALVE_STATE_DISABLED) {
			DBG_STATE(VALVE, "NMT not Operational (0x%02X) -> DISABLED",
					MY_NMT_STATE);

			ValveControl_ApplyFailSafe();
			ValveControl_SetState(VALVE_STATE_DISABLED);
			last_control_word = 0;
		}
		ValveControl_UpdateProcessImage();
		return;
	}

	/* If we just became Operational and were DISABLED, transition to IDLE */
	if (fsm_state == VALVE_STATE_DISABLED && !fault_active) {
		DBG_STATE(VALVE, "NMT Operational -> IDLE");
		ValveControl_SetState(VALVE_STATE_IDLE);
	}

	/*--------------------------------------------------------------
	 * Process ControlWord (edge detection + command dispatch)
	 *--------------------------------------------------------------*/
	ValveControl_ProcessControlWord();

	/*--------------------------------------------------------------
	 * Tick motion timeout (for OPENING/CLOSING states)
	 *--------------------------------------------------------------*/
	ValveControl_TickMotion();

	/*--------------------------------------------------------------
	 * Update process image (StatusWord, ValveState, ErrorRegister)
	 *--------------------------------------------------------------*/
	ValveControl_UpdateProcessImage();
}

void ValveControl_ApplyFailSafe(void) {
	uint8_t fail_safe = ProcImg_GetFailSafePosition();

	switch (fail_safe) {
	case FAILSAFE_OPEN:
		ValveDriver_SetRelay(true);
		DBG_STATE(FAILSAFE, "Applied: OPEN (relay ON)");
		break;

	case FAILSAFE_CLOSED:
		ValveDriver_SetRelay(false);
		DBG_STATE(FAILSAFE, "Applied: CLOSED (relay OFF)");
		break;

	case FAILSAFE_AS_IS:
	default:
		DBG_STATE(FAILSAFE, "Applied: AS_IS (no change)");
		break;
	}
}

void ValveControl_RunDiagnostics(void) {
	uint32_t now = HAL_GetTick();

	if ((now - diag_last_ms) < 1000) {
		return; /* Not yet 1 second since last output */
	}
	diag_last_ms = now;

	uint16_t timeout_s = ProcImg_GetMotionTimeout();
	uint32_t elapsed_s = 0;

	if (fsm_state == VALVE_STATE_OPENING || fsm_state == VALVE_STATE_CLOSING) {
		elapsed_s = (now - motion_start_ms) / 1000;
	}

	static const char *const state_names[] = { "DISABLED", "IDLE", "OPENING",
			"CLOSING", "FAULT" };
	static const char *const pos_names[] = { "UNKNOWN", "CLOSED", "OPEN",
			"MOVING" };

	const char *sname =
			(fsm_state <= VALVE_STATE_FAULT) ? state_names[fsm_state] : "???";
	const char *pname =
			(valve_position <= VALVE_POS_MOVING) ?
					pos_names[valve_position] : "???";

	DBG_PRINT(VALVE,
			"State=%s Pos=%s SW=0x%04X Relay=%s MotionTmr=%lu/%us Err=0x%02X",
			sname, pname, status_word,
			ValveDriver_GetRelayState() ? "ON" : "OFF", elapsed_s, timeout_s,
			error_register);
}

ValveState_FSM_t ValveControl_GetState(void) {
	return fsm_state;
}

uint8_t ValveControl_GetPosition(void) {
	return valve_position;
}

/**************************************************************************
 LOCAL FUNCTIONS
 ***************************************************************************/

/**
 * @brief  Set new FSM state with logging
 */
static void ValveControl_SetState(ValveState_FSM_t new_state) {
	if (fsm_state != new_state) {
		DBG_STATE(VALVE, "State: %u -> %u", (unsigned )fsm_state,
				(unsigned )new_state);
		fsm_state = new_state;
	}
}

/**
 * @brief  Process ControlWord from RPDO1
 *         Rising-edge detection for commands: Open (bit0), Close (bit1),
 *         Fault Reset (bit7), Halt (bit8). Level check for Enable Op (bit3).
 */
static void ValveControl_ProcessControlWord(void) {
	uint16_t cw = ProcImg_GetControlWord();
	uint16_t rising = (cw & ~last_control_word); /* Bits that went 0→1 */

	/* Track which CW bits we have actually inspected this cycle.
	 * Only inspected bits advance the edge detector — this prevents
	 * rising edges on Open/Close from being silently consumed while
	 * the FSM is in a state that cannot act on them (OPENING/CLOSING).
	 *
	 * Halt and FaultReset are always inspected (they apply in any state).
	 * Open/Close bits are only inspected when the FSM is IDLE + enabled. */
	uint16_t inspected = CW_FAULT_RESET | CW_HALT;

	/*--- Fault Reset (bit 7 rising edge) — highest priority ---*/
	if (rising & CW_FAULT_RESET) {
		if (fault_active) {
			DBG_STATE(VALVE, "Fault Reset -> DISABLED");
			fault_active = false;
			error_register = 0;
			valve_position = VALVE_POS_UNKNOWN;
			ValveControl_SetState(VALVE_STATE_DISABLED);

			/* If NMT Operational, immediately go to IDLE */
			if (MY_NMT_STATE == NMTSTATE_OP) {
				ValveControl_SetState(VALVE_STATE_IDLE);
			}
		}
		/* Update edge detector for inspected bits only, then return */
		last_control_word = (cw & inspected) | (last_control_word & ~inspected);
		return; /* Don't process other commands on same cycle as fault reset */
	}

	/*--- Halt (bit 8 rising edge) — stop any motion immediately ---*/
	if (rising & CW_HALT) {
		if (fsm_state == VALVE_STATE_OPENING
				|| fsm_state == VALVE_STATE_CLOSING) {
			DBG_STATE(VALVE, "HALT -> IDLE (position UNKNOWN)");
			/* Relay stays in current electrical state */
			valve_position = VALVE_POS_UNKNOWN;
			ValveControl_SetState(VALVE_STATE_IDLE);
		}
		last_control_word = (cw & inspected) | (last_control_word & ~inspected);
		return;
	}

	/*--- Block commands if fault active ---*/
	if (fault_active) {
		last_control_word = (cw & inspected) | (last_control_word & ~inspected);
		return;
	}

	/*--- Block open/close if not in IDLE or Enable Operation not set ---*/
	if (fsm_state != VALVE_STATE_IDLE) {
		/* Do NOT advance edge detector for Open/Close bits —
		 * the command will be re-detected once we reach IDLE. */
		last_control_word = (cw & inspected) | (last_control_word & ~inspected);
		return;
	}

	if (!(cw & CW_ENABLE_OP)) {
		last_control_word = (cw & inspected) | (last_control_word & ~inspected);
		return; /* Enable Operation (bit 3) must be set */
	}

	/* We are IDLE + enabled — Open/Close bits are now inspected */
	inspected |= CW_OPEN | CW_CLOSE | CW_ENABLE_OP;

	/*--- Close (bit 1) takes priority over Open (bit 0) for safety ---*/
	if (rising & CW_CLOSE) {
		if (valve_position != VALVE_POS_CLOSED) {
			DBG_STATE(VALVE, "Close command -> CLOSING");
			ValveDriver_SetRelay(false);
			valve_position = VALVE_POS_MOVING;
			motion_start_ms = HAL_GetTick();
			ValveControl_SetState(VALVE_STATE_CLOSING);
		} else {
			DBG_PRINT_V(VALVE, "Close ignored: already CLOSED");
		}
		last_control_word = (cw & inspected) | (last_control_word & ~inspected);
		return;
	}

	if (rising & CW_OPEN) {
		if (valve_position != VALVE_POS_OPEN) {
			DBG_STATE(VALVE, "Open command -> OPENING");
			ValveDriver_SetRelay(true);
			valve_position = VALVE_POS_MOVING;
			motion_start_ms = HAL_GetTick();
			ValveControl_SetState(VALVE_STATE_OPENING);
		} else {
			DBG_PRINT_V(VALVE, "Open ignored: already OPEN");
		}
	}

	/* Update edge detector for all inspected bits */
	last_control_word = (cw & inspected) | (last_control_word & ~inspected);
}

/**
 * @brief  Tick the motion timeout for OPENING/CLOSING states
 *         Normal completion after VALVE_MOTION_SETTLE_MS (fast solenoid, ~5-10 ms
 *         response — see datasheet note at VALVE_MOTION_SETTLE_MS).
 *         Fault if exceeds 2× MotionTimeout (stuck valve sanity bound).
 */
static void ValveControl_TickMotion(void) {
	if (fsm_state != VALVE_STATE_OPENING && fsm_state != VALVE_STATE_CLOSING) {
		return;
	}

	uint32_t now = HAL_GetTick();
	uint16_t timeout_s = ProcImg_GetMotionTimeout();
	uint32_t elapsed = now - motion_start_ms;

	/* Convert timeout to milliseconds */
	uint32_t timeout_ms = (uint32_t) timeout_s * 1000UL;
	uint32_t fault_timeout_ms = timeout_ms * 2;

	/*--- Check for fault (2× timeout — stuck valve) ---*/
	if (elapsed >= fault_timeout_ms) {
		DBG_ERROR(VALVE, "Motion fault: elapsed %lu ms > 2x timeout %lu ms",
				elapsed, fault_timeout_ms);

		fault_active = true;
		error_register = 0x01; /* Generic error */
		valve_position = VALVE_POS_UNKNOWN;

		ValveControl_ApplyFailSafe();
		ValveControl_SetState(VALVE_STATE_FAULT);
		return;
	}

	/*--- Check for normal completion (fixed settle — see VALVE_MOTION_SETTLE_MS) ---*/
	if (elapsed >= VALVE_MOTION_SETTLE_MS) {
		if (fsm_state == VALVE_STATE_OPENING) {
			valve_position = VALVE_POS_OPEN;
			DBG_STATE(VALVE, "Motion complete -> OPEN (%lu ms)", elapsed);
		} else /* VALVE_STATE_CLOSING */
		{
			valve_position = VALVE_POS_CLOSED;
			DBG_STATE(VALVE, "Motion complete -> CLOSED (%lu ms)", elapsed);
		}

		ValveControl_SetState(VALVE_STATE_IDLE);
	}
}

/**
 * @brief  Generate StatusWord from current FSM state and position
 */
static uint16_t ValveControl_GenerateStatusWord(void) {
	uint16_t sw = 0;

	/* SW_REMOTE: set when NMT Operational */
	if (MY_NMT_STATE == NMTSTATE_OP) {
		sw |= SW_REMOTE;
	}

	switch (fsm_state) {
	case VALVE_STATE_DISABLED:
		/* All clear except remote might be set if we just transitioned */
		sw &= ~SW_REMOTE; /* Not remote when disabled */
		break;

	case VALVE_STATE_IDLE:
		if (valve_position == VALVE_POS_CLOSED) {
			sw |= SW_CLOSED | SW_TARGET_REACHED;
		} else if (valve_position == VALVE_POS_OPEN) {
			sw |= SW_OPENED | SW_TARGET_REACHED;
		}
		/* UNKNOWN position: just SW_REMOTE, no position bits */
		break;

	case VALVE_STATE_OPENING:
	case VALVE_STATE_CLOSING:
		sw |= SW_MOVING;
		break;

	case VALVE_STATE_FAULT:
		sw |= SW_FAULT;
		break;
	}

	return sw;
}

/**
 * @brief  Write StatusWord, ValveState, ErrorRegister to process image
 */
static void ValveControl_UpdateProcessImage(void) {
	status_word = ValveControl_GenerateStatusWord();

	ProcImg_SetStatusWord(status_word);
	ProcImg_SetValveState(valve_position);
	ProcImg_SetErrorRegister(error_register);
}

/**************************************************************************
 MCO EVENT CALLBACKS
 ***************************************************************************/

/**
 * @brief  Called when NMT state changes (via MCO event system)
 */
static void ValveControl_OnNMTChange(const MCO_Event_t *event) {
	uint8_t nmt = event->nmt_state;

	DBG_STATE(CAN, "NMT change: 0x%02X", nmt);

	if (nmt != NMTSTATE_OP) {
		/* NMT left Operational — force to DISABLED + fail-safe
		 * (actual state change handled in ValveControl_Process NMT gate) */
	}
}

/**
 * @brief  Called when consumer heartbeat is lost (via MCO event system)
 */
static void ValveControl_OnHeartbeatLost(const MCO_Event_t *event) {
	DBG_ERROR(FAILSAFE, "Heartbeat lost from node 0x%02X!", event->node_id);

	/* Apply fail-safe position immediately */
	ValveControl_ApplyFailSafe();

	/* The MCO stack already calls MCO_HandleNMTRequest(NMTMSG_PREOP)
	 * in MCOUSER_HeartbeatLost(), which will trigger the NMT gate
	 * in ValveControl_Process() to move to DISABLED state. */
}

/**************************************************************************
 END-OF-FILE
 ***************************************************************************/
