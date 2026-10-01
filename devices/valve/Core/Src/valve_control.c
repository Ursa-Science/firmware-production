/**************************************************************************
 MODULE:    VALVE_CONTROL
 CONTAINS:  CiA 408 valve state machine implementation
 Processes ControlWord via RPDO1, drives relay via valve_driver,
 updates StatusWord + ValveState (TPDO1) and ErrorRegister (TPDO2 + SDO)
 in the MCO process image.

 State machine: DISABLED ↔ IDLE ↔ OPENING/CLOSING (↔ FAULT, unused)
 NMT-gated: must be NMT Operational for valve to respond.
 Edge detection on ControlWord bits (same pattern as pump module).
 Time-based motion model (no position sensor).

 Fail-safe policy (VALVE_REFACTOR_PLAN.md D1/D2, 2026-09-30): the valve is
 CLOSED (relay OFF) whenever the master heartbeat is lost, whenever the
 node leaves NMT Operational for any reason, on a fatal stack error, and at
 boot/IWDG reset. It never re-opens by itself: after any of those the
 master must send NMT start AND a fresh ControlWord Open edge.

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
 * position sensor we declare the commanded position reached after a short
 * fixed settle (generous margin over the 10 ms response plus relay/loop
 * jitter). There is no stuck-valve detection: nothing on this module can
 * observe it (the former 0x2300 MotionTimeout fault bound was unreachable
 * behind this settle and was deleted). */
#define VALVE_MOTION_SETTLE_MS 50u

/* ErrorRegister (0x1001) bits — CiA 301 */
#define ERREG_GENERIC          0x01u /* bit 0: generic error (also set by the stack on any EMCY it pushes) */
#define ERREG_APP_BITS         (ERREG_GENERIC) /* bits this module owns */

/**************************************************************************
 LOCAL VARIABLES
 ***************************************************************************/
static ValveState_FSM_t fsm_state; /* Current FSM state */
static uint8_t valve_position; /* Current position (VALVE_POS_*) */
static uint16_t last_control_word; /* Previous CW for edge detection */
static uint32_t motion_start_ms; /* HAL_GetTick() when motion began */
static uint16_t status_word; /* Current StatusWord shadow */
static uint8_t last_err_reg; /* App-owned 0x1001 bits last applied to both homes */
static uint32_t diag_last_ms; /* Last diagnostics print timestamp */
static bool fault_active; /* Fault latch (never set today — see VALVE_STATE_FAULT) */

/**************************************************************************
 LOCAL FUNCTION PROTOTYPES
 ***************************************************************************/
static void ValveControl_ProcessControlWord(void);
static void ValveControl_TickMotion(void);
static uint16_t ValveControl_GenerateStatusWord(void);
static void ValveControl_UpdateErrorRegister(void);
static void ValveControl_ClearErrors(void);
static void ValveControl_UpdateProcessImage(void);
static void ValveControl_SetState(ValveState_FSM_t new_state);
static void ValveControl_OnNMTChange(const MCO_Event_t *event);
static void ValveControl_OnHeartbeatLost(const MCO_Event_t *event);
static void ValveControl_OnHeartbeatRestored(const MCO_Event_t *event);
static void ValveControl_OnFatalError(const MCO_Event_t *event);

/**************************************************************************
 GLOBAL FUNCTIONS
 ***************************************************************************/

void ValveControl_Init(void) {
	fsm_state = VALVE_STATE_DISABLED;
	valve_position = VALVE_POS_CLOSED; /* Relay is OFF from boot / ValveDriver_Init */
	last_control_word = 0;
	motion_start_ms = 0;
	status_word = 0;
	last_err_reg = 0;
	diag_last_ms = 0;
	fault_active = false;

	/* Initialize valve driver (relay OFF = closed) */
	ValveDriver_Init();

	/* Register for MCO events */
	MCO_Events_Register(MCO_EVENT_NMT_CHANGE, ValveControl_OnNMTChange);
	MCO_Events_Register(MCO_EVENT_HEARTBEAT_LOST, ValveControl_OnHeartbeatLost);
	MCO_Events_Register(MCO_EVENT_HEARTBEAT_RESTORED,
			ValveControl_OnHeartbeatRestored);
	MCO_Events_Register(MCO_EVENT_FATAL_ERROR, ValveControl_OnFatalError);

	/* Update process image with initial values */
	ValveControl_UpdateProcessImage();

	DBG_STATE(VALVE, "Init complete, state=DISABLED, relay OFF (closed)");
}

void ValveControl_Process(void) {
	/*--------------------------------------------------------------
	 * NMT Gate: If not Operational, force DISABLED + CLOSED
	 *--------------------------------------------------------------*/
	if (MY_NMT_STATE != NMTSTATE_OP) {
		if (fsm_state != VALVE_STATE_DISABLED) {
			DBG_STATE(VALVE, "NMT not Operational (0x%02X) -> DISABLED",
					MY_NMT_STATE);
			ValveControl_ForceClosed("NMT left Operational");
			ValveControl_SetState(VALVE_STATE_DISABLED);
		}
		ValveControl_UpdateProcessImage();
		return;
	}

	/* If we just became Operational and were DISABLED, transition to IDLE */
	if (fsm_state == VALVE_STATE_DISABLED && !fault_active) {
		/* Re-arm rule: whatever ControlWord level is present at entry counts
		 * as already seen. RPDO1 only lands in the process image in OP, so a
		 * stale Open (0x0009) buffered before a heartbeat loss / NMT stop
		 * must not fire as a rising edge the moment the master restarts us.
		 * ForceClosed() has normally zeroed the PI copy already; this is the
		 * belt-and-braces for the case where the master re-sent its shadow
		 * during the same loop. Opening needs a fresh 0->1 edge on bit 0. */
		last_control_word = ProcImg_GetControlWord();
		DBG_STATE(VALVE, "NMT Operational -> IDLE (CW at entry 0x%04X)",
				last_control_word);
		ValveControl_SetState(VALVE_STATE_IDLE);
	}

	/*--------------------------------------------------------------
	 * Process ControlWord (edge detection + command dispatch)
	 *--------------------------------------------------------------*/
	ValveControl_ProcessControlWord();

	/*--------------------------------------------------------------
	 * Tick motion settle (for OPENING/CLOSING states)
	 *--------------------------------------------------------------*/
	ValveControl_TickMotion();

	/*--------------------------------------------------------------
	 * Update process image (StatusWord, ValveState, ErrorRegister)
	 *--------------------------------------------------------------*/
	ValveControl_UpdateProcessImage();
}

void ValveControl_ForceClosed(const char *reason) {
	bool relay_was_on = ValveDriver_GetRelayState();
	uint8_t prev_position = valve_position;

	/* 1. Physical: relay OFF = valve closed (normally-closed Clippard). */
	ValveDriver_SetRelay(false);

	/* 2. Reported position follows the relay, so an SDO read of 0x6042 in
	 *    PRE-OP says CLOSED instead of the stale pre-event value. */
	valve_position = VALVE_POS_CLOSED;

	/* 3. Forget any buffered command: zero the process-image ControlWord
	 *    (RPDO1 data survives PRE-OP otherwise) and the edge detector, so
	 *    the next Open needs a fresh 0->1 edge from the master. */
	ProcImg_SetControlWord(0);
	last_control_word = 0;

	/* 4. A motion in progress is over (relay is OFF now); an IDLE stays
	 *    IDLE at the new position. DISABLED/FAULT are left to their owners
	 *    (NMT gate / fault reset). */
	if (fsm_state == VALVE_STATE_OPENING || fsm_state == VALVE_STATE_CLOSING) {
		ValveControl_SetState(VALVE_STATE_IDLE);
	}

	if (relay_was_on || prev_position != VALVE_POS_CLOSED) {
		DBG_STATE(FAILSAFE, "Forced CLOSED (%s): relay %s -> OFF, pos %u -> 1",
				reason, relay_was_on ? "ON" : "OFF", (unsigned) prev_position);
	}

	ValveControl_UpdateProcessImage();
}

void ValveControl_RunDiagnostics(void) {
	uint32_t now = HAL_GetTick();

	if ((now - diag_last_ms) < 1000) {
		return; /* Not yet 1 second since last output */
	}
	diag_last_ms = now;

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
			"State=%s Pos=%s SW=0x%04X Relay=%s NMT=0x%02X CW=0x%04X Err=0x%02X",
			sname, pname, status_word,
			ValveDriver_GetRelayState() ? "ON" : "OFF", MY_NMT_STATE,
			ProcImg_GetControlWord(), gMCOConfig.error_register);
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

	/*--- Fault / error reset (bit 7 rising edge) — highest priority ---
	 * Works in EVERY state, not only with a latched app fault: it is also
	 * how the MIK clears a stack-set ErrorRegister bit 0 (EMCY 0x8130 /
	 * 0x6100 history) without a power cycle. */
	if (rising & CW_FAULT_RESET) {
		ValveControl_ClearErrors();

		if (fault_active) {
			DBG_STATE(VALVE, "Fault Reset -> DISABLED");
			fault_active = false;
			valve_position = ValveDriver_GetRelayState() ? VALVE_POS_OPEN : VALVE_POS_CLOSED;
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
 * @brief  Tick the motion settle for OPENING/CLOSING states
 *         Completion after VALVE_MOTION_SETTLE_MS (fast solenoid, ~5-10 ms
 *         response — see datasheet note at VALVE_MOTION_SETTLE_MS).
 */
static void ValveControl_TickMotion(void) {
	if (fsm_state != VALVE_STATE_OPENING && fsm_state != VALVE_STATE_CLOSING) {
		return;
	}

	uint32_t elapsed = HAL_GetTick() - motion_start_ms;

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
 * @brief  Compute the app-owned 0x1001 bits and apply them to BOTH homes.
 * @note   The stack serves SDO reads of 0x1001 and the EMCY error-register
 *         byte from gMCOConfig.error_register (mco.c); TPDO2 carries the
 *         process-image copy. The old code wrote only the PI, so an SDO read
 *         said 0x01 (stack-set, from its own EMCYs) while TPDO2 showed 0x00 —
 *         the disagreement recorded in the gateway overrides. Edge-wise
 *         set/clear so a stack-set generic bit is not stomped each loop; the
 *         PI copy mirrors the merged register so both homes always agree.
 *         Same shape as phtemp's SensorControl_UpdateErrorRegister.
 */
static void ValveControl_UpdateErrorRegister(void) {
	uint8_t err = fault_active ? ERREG_GENERIC : 0u;

	uint8_t set_bits = err & (uint8_t) ~last_err_reg;
	uint8_t clr_bits = last_err_reg & (uint8_t) ~err;
	if (set_bits) {
		gMCOConfig.error_register |= set_bits;
	}
	if (clr_bits) {
		gMCOConfig.error_register &= (uint8_t) ~clr_bits;
	}
	last_err_reg = err;

	ProcImg_SetErrorRegister(gMCOConfig.error_register);
}

/**
 * @brief  ControlWord bit-7 reset: clear 0x1001 bit 0 in both homes (ours
 *         and the stack's latched copy) and announce EMCY 0x0000 if there was
 *         anything to clear. Bits 1-7 are never set by this module.
 */
static void ValveControl_ClearErrors(void) {
	bool had_error = (gMCOConfig.error_register != 0u);

	gMCOConfig.error_register &= (uint8_t) ~ERREG_APP_BITS;
	last_err_reg = 0;
	ProcImg_SetErrorRegister(gMCOConfig.error_register);

	if (had_error) {
		DBG_STATE(VALVE, "EMCY 0x0000: error reset (ER now 0x%02X)",
				gMCOConfig.error_register);
		(void) MCOP_PushEMCY(EMCY_NO_ERROR, 0, 0, 0, 0, 0);
	} else {
		DBG_PRINT_V(VALVE, "Reset with nothing latched: no-op");
	}
}

/**
 * @brief  Write StatusWord, ValveState, ErrorRegister to process image
 */
static void ValveControl_UpdateProcessImage(void) {
	status_word = ValveControl_GenerateStatusWord();

	ProcImg_SetStatusWord(status_word);
	ProcImg_SetValveState(valve_position);
	ValveControl_UpdateErrorRegister();
}

/**************************************************************************
 MCO EVENT CALLBACKS
 ***************************************************************************/

/**
 * @brief  Called when NMT state changes (via MCO event system)
 */
static void ValveControl_OnNMTChange(const MCO_Event_t *event) {
	DBG_STATE(CAN, "NMT change: 0x%02X", event->nmt_state);
	/* Leaving Operational is handled by the NMT gate in ValveControl_Process()
	 * next loop (ForceClosed + DISABLED); nothing to do here. */
}

/**
 * @brief  Master heartbeat lost (via MCO event system).
 * @note   Runs INSIDE the stack's MCOP_ProcessHBCheck, before it forces
 *         PRE-OP, so the relay is off before TPDOs stop. The stack has
 *         already pushed EMCY 0x8130 and set ErrorRegister bit 0. Not a
 *         FAULT (D2): resume = master NMT start + fresh Open edge.
 */
static void ValveControl_OnHeartbeatLost(const MCO_Event_t *event) {
	DBG_ERROR(FAILSAFE, "Heartbeat lost from node %u -> closing; stack forces PRE-OP",
			event->node_id);
	ValveControl_ForceClosed("master heartbeat lost");
}

/**
 * @brief  Master heartbeat back (stack recovery EMCY, via user_cbdata.c).
 * @note   The stack set ErrorRegister bit 0 on 0x8130 and never clears it.
 *         Clear it now if this module itself has nothing to report, so
 *         0x1001 reads 0x00 again without a ControlWord reset. The PI copy
 *         mirrors the merged register so TPDO2 and SDO stay in agreement.
 */
static void ValveControl_OnHeartbeatRestored(const MCO_Event_t *event) {
	DBG_STATE(FAILSAFE, "Heartbeat restored from node %u (valve stays CLOSED until NMT start + Open)",
			event->node_id);
	if (last_err_reg == 0u) {
		gMCOConfig.error_register &= (uint8_t) ~ERREG_GENERIC;
		ProcImg_SetErrorRegister(gMCOConfig.error_register);
	}
}

/**
 * @brief  Stack error callback (MCOUSER_FatalError), fatal OR warning class.
 * @note   D3/D4: the only warning-class codes the stack raises are the
 *         transmit-FIFO overflows 0x4810-0x4840 (EMCY 0x6100, MSEF 48 xx) —
 *         a bus symptom, NOT a valve fault, and the stack stays in OP. They
 *         are logged only; closing an open valve on every TX hiccup would
 *         be worse than today. Codes >= ERR_FATAL make the stack reset the
 *         MCU ~10 ms later; dropping the relay first costs nothing.
 */
static void ValveControl_OnFatalError(const MCO_Event_t *event) {
	uint16_t code = event->error_code;

	if (code >= ERR_FATAL) {
		DBG_ERROR(FAILSAFE, "MCO fatal error 0x%04X -> closing (stack reset follows)",
				code);
		ValveControl_ForceClosed("stack fatal error");
	} else {
		DBG_ERROR(CAN, "MCO warning 0x%04X (EMCY 0x6100, TX-overflow class) - no valve action",
				code);
	}
}

/**************************************************************************
 END-OF-FILE
 ***************************************************************************/
