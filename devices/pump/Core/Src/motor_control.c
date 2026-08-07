/**
 ******************************************************************************
 * @file    motor_control.c
 * @brief   Pump Control Layer - EDS-Compliant CANopen Integration
 * @author  CANopen Motor Control Project
 * @date    2025
 * @note    motor control with CiA 402 interface
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "motor_control.h"
#include "stepper.h"
#include "led_control.h"
#include "mco_events.h"
#include "mcop_inc.h"
#include "procimg_api.h"
#include "log.h"
#include <string.h>
#include <stdlib.h>
#include <stdbool.h>
#include <math.h>
#include <stdio.h>

/* External timer handles from main.c (injected into LEDControl_Init) */
extern TIM_HandleTypeDef htim2;  // Red LED - TIM2 CH1
extern TIM_HandleTypeDef htim3;  // Blue LED - TIM3 CH2
extern TIM_HandleTypeDef htim4;  // Green LED - TIM4 CH1

/* Private defines -----------------------------------------------------------*/

// WPX-1 pump with 1:8 gear ratio: 200 motor steps × 8 = 1600 full-steps/rev
// TMC2209 GCONF.MSTEP_REG_SELECT=1 via IFCNT-verified UART write
// CHOPCONF.MRES=8 (full step), MicroPlyer interpolates to 256 internally
// Each timer pulse = 1 full step
#define STEPS_PER_REV           1600

// Calibration Constants
// BASE_STEPS_PER_ML: Theoretical value based on target of 100 ml/min @ 90 RPM
// Calculation: (90 RPM * 1600 steps/rev) / 100 ml/min = 1440 steps/ml
#define BASE_STEPS_PER_ML       1440.0f


// Safety limits (tested 01/20/2026)
#define MAX_SAFE_RPM            180      // Maximum safe RPM from testing
#define MAX_FLOW_ML_MIN         180     // Maximum flow rate: 100 ml/min @ 90 RPM

// CiA 402 ControlWord bits (0x6040) - DS402 Standard Mapping
#define CONTROLWORD_SWITCH_ON       (1 << 0)  // Bit 0: Switch On (enable power stage)
#define CONTROLWORD_ENABLE_VOLTAGE  (1 << 1)  // Bit 1: Enable Voltage
#define CONTROLWORD_QUICKSTOP       (1 << 2)  // Bit 2: Quick Stop (ACTIVE LOW - 0=stop active, 1=normal)
#define CONTROLWORD_ENABLE_OP       (1 << 3)  // Bit 3: Enable Operation (permit motion)
#define CONTROLWORD_LAST_KNOWN_FLOWRATE (1 << 4)  // Bit 4: Use last known flow rate (manufacturer-specific)
#define CONTROLWORD_FAULT_RESET     (1 << 7)  // Bit 7: Fault Reset
#define CONTROLWORD_HALT            (1 << 8)  // Bit 8: Halt (temporary pause)

// CiA 402 StatusWord bits (0x6041) - used in MotorControl_GenerateStatusWord()
// These composite patterns encode the device state in specific bit combinations
// Pattern format: xxxx xRTx xQSO SEDV where:
//   Bit 0 (V): Ready to switch on
//   Bit 1 (S): Switched on
//   Bit 2 (O): Operation enabled
//   Bit 3 (F): Fault (not shown in pattern notation)
//   Bit 4 (E): Voltage enabled
//   Bit 5 (Q): Quick stop (ACTIVE LOW - 0=quick stop active)
//   Bit 6 (D): Switch on disabled
//   Bit 9 (R): Remote
//   Bit 10 (T): Target reached
// Note: Bits 7,8,11-15 are operation mode specific or manufacturer specific
#define STATUSWORD_VOLTAGE_ENABLED  (1 << 4)   // Bit 4: Voltage enabled (power stage available)
#define STATUSWORD_REMOTE           (1 << 9)   // Bit 9: Remote (controlled via network)
#define STATUSWORD_TARGET_REACHED   (1 << 10)  // Bit 10: Target reached (at commanded speed)
#define STATUS_NOT_READY_TO_SWITCH_ON  0x0000  // xxxx xxxx x0xx 0000
#define STATUS_SWITCH_ON_DISABLED      0x0040  // xxxx xxxx x1xx 0000
#define STATUS_READY_TO_SWITCH_ON      0x0021  // xxxx xxxx x01x 0001
#define STATUS_SWITCHED_ON             0x0023  // xxxx xxxx x01x 0011
#define STATUS_OPERATION_ENABLED       0x0027  // xxxx xxxx x01x 0111
#define STATUS_QUICK_STOP_ACTIVE       0x0007  // xxxx xxxx x00x 0111
#define STATUS_FAULT_REACTION_ACTIVE   0x000F  // xxxx xxxx x00x 1111
#define STATUS_FAULT                   0x0008  // xxxx xxxx x00x 1000

// Pump state values (0x2400)
#define PUMPSTATE_INIT          0x00
#define PUMPSTATE_STOPPED       0x01
#define PUMPSTATE_STARTING      0x02
#define PUMPSTATE_RUNNING       0x03
#define PUMPSTATE_STOPPING      0x04
#define PUMPSTATE_FAULT         0x05
#define PUMPSTATE_HALT          0x06

// Dose status values (0x2303)
#define DOSE_STATUS_IDLE        0x00
#define DOSE_STATUS_RUNNING     0x01
#define DOSE_STATUS_COMPLETE    0x02
#define DOSE_STATUS_TIMEOUT     0x03
#define DOSE_STATUS_ABORTED     0x04
#define DOSE_STATUS_ERROR       0x05
/* Private typedef -----------------------------------------------------------*/

/**
 * @brief Dose tracking structure for latched parameter approach
 * @note Parameters are latched at START command when DoseVolume > 0
 */
typedef struct {
	uint32_t target_volume_ml;      // DoseVolume (ml × 1000)
	int16_t flow_rate_ml_min;       // DoseFlowRate (signed for bidirection)
	uint16_t timeout_sec;           // DoseTimeout (seconds)
	bool dose_active;               // true when dose is running
	uint32_t start_time_ms;         // HAL_GetTick() at start
	int32_t start_position;         // Stepper position at start
	uint32_t delivered_ml;          // Current progress (ml × 1000)

	// HALT/Resume pause tracking
	uint32_t pause_start_ms; // HAL_GetTick() when HALT started (0 = not paused)
	uint32_t total_pause_ms;        // Cumulative pause time in milliseconds

	// Dose completion tracking for restart blocking
	uint8_t just_completed; // Blocks restart with stale ControlWord after dose ends
	int16_t flow_rate_at_completion;  // Tracks flow rate for change detection
} DoseTracker_t;

typedef struct {
	bool initialized;
	uint32_t last_update;
	uint16_t last_control_word;
	int16_t target_flow_ml_min;

	// State machine (prevents duplicate command execution in RPDO mode)
	MotorState_t current_state;
	
	// State tracking
	uint8_t pump_state;
	bool fault_active;

	// Dose tracking (latched parameters approach)
	DoseTracker_t dose_tracker;

} MotorControl_t;

/* Private variables ---------------------------------------------------------*/
static MotorControl_t motor_ctrl = { 0 };

// CiA 402 Compliance: Last known TargetFlowRate to persist across state transitions
// This static variable maintains the last received TargetFlowRate value, allowing
// the motor to resume at the correct speed after halt or quick stop operations
// when master sets CONTROLWORD_LAST_KNOWN_FLOWRATE (bit 4).
// BEHAVIOR CHANGE: When bit 4 = 0 (normal mode), TargetFlowRate from RPDO is used directly
//                  When bit 4 = 1 (last known mode), this stored value is used instead
static int16_t last_known_target_flow_ml_min = 0;

// one-shot log flags (prevents UART ring-buffer flooding)
// Each flag is reset at its own semantic point — NOT a blanket "reset all".
// See LogOnceFlags_t members for ownership comments.
typedef struct {
	bool dose_blocking;   // CheckTransitionBlocking / HandleDoseCompletionFlags
	bool holding; // HandleQuickStopRecovery (quickstop complete, bit still active)
	bool waiting;          // HandleQuickStopRecovery (motor still decelerating)
	bool decel_blocking; // CheckTransitionBlocking (stepper still decelerating)
} LogOnceFlags_t;

static LogOnceFlags_t log_once = { 0 };

/* Private function prototypes -----------------------------------------------*/
static uint16_t mlPerMin_to_rpm(uint16_t ml_per_min);
static uint16_t rpm_to_mlPerMin(uint16_t rpm);
static void MotorControl_ProcessControlWord(uint16_t control_word);
static bool HandleFaultReset(uint16_t rising_edges);
static bool HandleQuickStop(uint16_t falling_edges, uint16_t control_word);
static bool HandleQuickStopRecovery(uint16_t control_word);
static MotorState_t DetermineTargetState(uint16_t control_word);
static void HandleDoseCompletionFlags(uint16_t control_word);
static bool CheckTransitionBlocking(MotorState_t target_state,
		uint16_t control_word);
static void ExecuteExitActions(MotorState_t target_state);
static void ExecuteEntryActions(MotorState_t target_state);
static uint16_t MotorControl_GenerateStatusWord(void);
static void MotorControl_UpdateProcessImage(void);
static void MotorControl_CheckDoseProgress(void);
static void MotorControl_ProcessStepperEvents(void);
static void MotorControl_PrintStatus(void);
static void MotorControl_PrintTimerStats(void);
static void MotorControl_PrintDiagnostics(void);

/* Phase 3: Stepper Event Handling (CT-03) -----------------------------------*/

/**
 * @brief Process stepper events in main loop context (CT-03)
 * @note  Called from MotorControl_Process() each cycle.
 *        Reads-and-clears event bitfield atomically via Stepper_GetPendingEvents().
 */
static void MotorControl_ProcessStepperEvents(void) {
	uint32_t events = Stepper_GetPendingEvents();
	if (events == 0) {
		return;
	}

	if (events & STEPPER_EVT_STOPPED) {
		if (motor_ctrl.current_state == MOTOR_STATE_RUNNING
				|| motor_ctrl.current_state == MOTOR_STATE_QUICKSTOPPING) {
			DBG_STATE(MOTOR,
					"Stepper STOPPED event → transitioning to ENABLED_STOPPED");
			motor_ctrl.current_state = MOTOR_STATE_ENABLED_STOPPED;
			if (ProcImg_GetControlWord() & CONTROLWORD_HALT) {
				motor_ctrl.pump_state = PUMPSTATE_HALT;
			} else {
				motor_ctrl.pump_state = PUMPSTATE_STOPPED;
			}
		}
	}

	if (events & STEPPER_EVT_AT_SPEED) {
		DBG_PRINT_V(MOTOR, "Stepper reached target speed");
	}

	if (events & STEPPER_EVT_ERROR) {
		DBG_ERROR(MOTOR, "Stepper ERROR event → entering FAULT state");
		motor_ctrl.current_state = MOTOR_STATE_FAULT;
		motor_ctrl.pump_state = PUMPSTATE_FAULT;
		motor_ctrl.fault_active = true;

		// Abort dose if active
		if (motor_ctrl.dose_tracker.dose_active) {
			DBG_STATE(DOSE,
					"Stepper error aborted dose at %lu.%03lu ml of %lu.%03lu ml",
					motor_ctrl.dose_tracker.delivered_ml / 1000,
					motor_ctrl.dose_tracker.delivered_ml % 1000,
					motor_ctrl.dose_tracker.target_volume_ml / 1000,
					motor_ctrl.dose_tracker.target_volume_ml % 1000);
			motor_ctrl.dose_tracker.dose_active = false;
		}
	}
}

/* Public functions ----------------------------------------------------------*/

/**
 * @brief Initialize motor control bridge
 */
bool MotorControl_Init(void) {
	// Initialize stepper motor
	if (!Stepper_Init()) {
		return false;
	}

	// Initialize LED control (inject timer handles — TC-03 decoupling)
	if (!LEDControl_Init(&htim2, TIM_CHANNEL_1, &htim3, TIM_CHANNEL_2, &htim4,
			TIM_CHANNEL_1)) {
		return false;
	}

	motor_ctrl.initialized = true;
	motor_ctrl.last_update = HAL_GetTick();
	motor_ctrl.last_control_word = 0x0004; // Bit 2 set = "quick stop not active" baseline
	motor_ctrl.target_flow_ml_min = 0;
	motor_ctrl.current_state = MOTOR_STATE_DISABLED; // Initialize state machine
	motor_ctrl.pump_state = PUMPSTATE_INIT;
	motor_ctrl.fault_active = false;

	// Initialize dose tracker
	memset(&motor_ctrl.dose_tracker, 0, sizeof(DoseTracker_t));
	motor_ctrl.dose_tracker.dose_active = false;

	return true;
}

/**
 * @brief Process motor control (call from MCO main loop)
 */
void MotorControl_Process(void) {
	if (!motor_ctrl.initialized)
		return;

	Stepper_Poll();  // Execute any ISR-deferred stop operations
	MotorControl_ProcessStepperEvents(); // process stepper event bitfield

	// If NMT state is not operational, ensure motor is stopped AND de-energized
	if (MY_NMT_STATE != NMTSTATE_OP) {
		if (motor_ctrl.current_state != MOTOR_STATE_DISABLED) {
			Stepper_Stop();
			Stepper_Disable();
			motor_ctrl.current_state = MOTOR_STATE_DISABLED;
			motor_ctrl.pump_state = PUMPSTATE_STOPPED;
		}
		MotorControl_UpdateProcessImage();
		return;
	}

	// Read ControlWord from process image (0x6040)
	uint16_t control_word = ProcImg_GetControlWord();

	// Read TargetFlowRate from process image (0x6042 - ml/min, SIGNED)
	int16_t target_flow = ProcImg_GetTargetFlowRate();

	// === CONTROLWORD BIT 4: LAST_KNOWN_FLOWRATE Feature ===
	// Master has explicit control over flow rate latching via bit 4:
	//   Bit 4 = 0 (CLEAR): Normal mode - use TargetFlowRate from RPDO directly
	//                      Zero values actually mean zero (stop), not "don't care"
	//   Bit 4 = 1 (SET):   Last known mode - ignore RPDO, use stored last known value
	//                      Useful for HALT/resume without resending flow rate
	//
	// Always update last_known when receiving non-zero value (for future use)
	if (target_flow != 0) {
		last_known_target_flow_ml_min = target_flow;
	}

	// Apply flow rate based on bit 4 state
	if (control_word & CONTROLWORD_LAST_KNOWN_FLOWRATE) {
		// Bit 4 SET: Master requests last known flow rate (ignore current RPDO value)
		motor_ctrl.target_flow_ml_min = last_known_target_flow_ml_min;
	} else {
		// Bit 4 CLEAR: Normal mode - use RPDO value directly (zero means stop)
		motor_ctrl.target_flow_ml_min = target_flow;
	}

	// Process control word commands
	MotorControl_ProcessControlWord(control_word);

	// === DOSE PROGRESS MONITORING ===
	// Check dose progress if active (zero overhead when dose_active==false)
	if (motor_ctrl.dose_tracker.dose_active) {
		MotorControl_CheckDoseProgress();
	}

	if (motor_ctrl.current_state == MOTOR_STATE_ENABLED_STOPPED
			&& !Stepper_IsMoving()) {
		static uint32_t last_idle_audit = 0;
		uint32_t now = HAL_GetTick();
		if (now - last_idle_audit >= 1000) {
			last_idle_audit = now;
			Stepper_SuppressIdleInterrupts(); // Detect and suppress unauthorized interrupt enable
			Stepper_SetOutputProtection(true); // Ensure outputs safe during IDLE
			Stepper_CheckIdleState();          // Monitor for spurious pulses
		}
	}

	// Update process image with status
	MotorControl_UpdateProcessImage();

	// Process LED control (reads from 0x2000)
	LEDControl_Process();

	motor_ctrl.last_control_word = control_word;
	motor_ctrl.last_update = HAL_GetTick();
}

/**
 * @brief Check if motor control is operational
 */
bool MotorControl_IsOperational(void) {
	return (MY_NMT_STATE == NMTSTATE_OP) && motor_ctrl.initialized;
}

/**
 * @brief Emergency stop from MCO error handling
 */
void MotorControl_EmergencyStop(void) {
	if (motor_ctrl.initialized) {
		// Audit 5 Fix 3: Only call Stepper_Stop/Disable if motor is actually moving
		// or state indicates RUNNING. Prevents repeated heavy shutdown when called
		// from MCO error/heartbeat-lost events while motor is already stopped.
		// Stepper_Stop() IDLE guard (Fix 1) provides additional safety net.
		if (Stepper_IsMoving()
				|| motor_ctrl.current_state == MOTOR_STATE_RUNNING) {
			Stepper_Stop();
			Stepper_Disable();
		}
		motor_ctrl.pump_state = PUMPSTATE_STOPPED;
	}
}

/**
 * @brief Reset motor control on NMT state change
 * @note  Also resets current_state to DISABLED to prevent state machine
 *        from becoming stuck in RUNNING state during NMT RESET_COMMUNICATION.
 * @note  Clears cached TargetFlowRate on NMT reset to ensure
 *        clean state after communication reset.
 */
void MotorControl_Reset(void) {
	if (motor_ctrl.initialized) {
		// Audit 5 Fix 2: Only call Stepper_Stop/Disable when not already DISABLED
		// Prevents repeated heavy shutdown when called from MCO NMT_CHANGE events
		// while motor is already stopped. State/variable resets below still execute.
		if (motor_ctrl.current_state != MOTOR_STATE_DISABLED) {
			Stepper_Stop();
			Stepper_Disable(); // De-energize coils on NMT reset (EN pin HIGH)
		}
		motor_ctrl.last_control_word = 0x0004; //Bit 2 set = "quick stop not active" baseline
		motor_ctrl.target_flow_ml_min = 0;
		motor_ctrl.pump_state = PUMPSTATE_STOPPED;
		motor_ctrl.fault_active = false;
		motor_ctrl.current_state = MOTOR_STATE_DISABLED;
		last_known_target_flow_ml_min = 0;

		// Clear dose tracker on NMT reset to prevent stale state from previous dose
		memset(&motor_ctrl.dose_tracker, 0, sizeof(DoseTracker_t));
		motor_ctrl.dose_tracker.dose_active = false;

		// Reset all one-shot log flags on NMT reset
		memset(&log_once, 0, sizeof(log_once));
	}
}

/* Private function implementations ------------------------------------------*/

/**
 * @brief Get effective steps/ml using dynamic FlowCorrectionFactor from OD 0x2200
 * @return BASE_STEPS_PER_ML × correction factor (clamped to [0.1, 10.0] for safety)
 * @note Reads FlowCorrectionFactor from process image each call — master can update
 *       via SDO at any time. Safety clamp prevents division-by-zero or absurd values
 *       if the master writes garbage to 0x2200.
 */
static inline float GetStepsPerMl(void) {
	float correction = ProcImg_GetFlowCorrectionFactor();
	if (correction < 0.1f || correction > 10.0f) {
		correction = 1.0f;  // Fallback to uncorrected
	}
	return BASE_STEPS_PER_ML * correction;
}

/**
 * @brief Convert ml/min to RPM
 * @note FIX: Returns minimum 1 RPM for non-zero flow rates to prevent fallback
 *       to enum-based speed (300 RPM) when calculation rounds down to 0.
 *       Example: 1ml/min = 0.9 RPM → was truncating to 0 → fallback to 300 RPM
 */
static uint16_t mlPerMin_to_rpm(uint16_t ml_per_min) {
	if (ml_per_min == 0)
		return 0;

	// Convert: ml/min → steps/sec → RPM
	// steps_per_sec = (ml_per_min / 60.0) * STEPS_PER_ML
	// rpm = (steps_per_sec * 60) / STEPS_PER_REV

	float steps_per_ml = GetStepsPerMl();
	float steps_per_sec = (ml_per_min / 60.0f) * steps_per_ml;
	uint16_t rpm = (uint16_t) ((steps_per_sec * 60.0f) / STEPS_PER_REV);

	// Ensure minimum 1 RPM for non-zero flow rates
	// This prevents fallback to enum speed (300 RPM) when calculation
	// rounds down to 0 (e.g., 1ml/min = 0.9 RPM → 0 → fallback to 30 RPM)
	if (rpm == 0) {
		rpm = 1;  // Minimum 1 RPM when flow rate is non-zero
	}

	// Clamp to safe range
	if (rpm > STEPPER_MAX_SPEED_RPM) {
		rpm = STEPPER_MAX_SPEED_RPM;
	}

	return rpm;
}

/**
 * @brief Convert RPM to ml/min
 */
static uint16_t rpm_to_mlPerMin(uint16_t rpm) {
	if (rpm == 0)
		return 0;

	// Convert: RPM → steps/sec → ml/min
	// steps_per_sec = (rpm * STEPS_PER_REV) / 60.0
	// ml_per_min = (steps_per_sec * 60) / STEPS_PER_ML

	float steps_per_ml = GetStepsPerMl();
	float steps_per_sec = (rpm * STEPS_PER_REV) / 60.0f;
	uint16_t ml_per_min = (uint16_t) ((steps_per_sec * 60.0f) / steps_per_ml);

	return ml_per_min;
}

/**
 * @brief Handle FAULT RESET command (Bit 7 rising edge)
 * @param rising_edges  Bits that transitioned 0→1
 * @return true if fault was reset (caller should return)
 */
static bool HandleFaultReset(uint16_t rising_edges) {
	if (rising_edges & CONTROLWORD_FAULT_RESET) {
		DBG_STATE(MOTOR, "ACTION: RESET fault");
		Stepper_ClearError();
		motor_ctrl.fault_active = false;
		motor_ctrl.current_state = MOTOR_STATE_DISABLED;
		motor_ctrl.pump_state = PUMPSTATE_STOPPED;
		return true;
	}
	return false;
}

/**
 * @brief Handle QUICK STOP command (Bit 2 falling edge — active low per CiA 402)
 * @param falling_edges  Bits that transitioned 1→0
 * @param control_word   Current control word (unused but kept for signature consistency)
 * @return true if quick stop was initiated (caller should return)
 */
static bool HandleQuickStop(uint16_t falling_edges, uint16_t control_word) {
	(void) control_word;  // Reserved for future use
	if (falling_edges & CONTROLWORD_QUICKSTOP) {
		DBG_STATE(MOTOR,
				"ACTION: QUICKSTOP (immediate stop + de-energize - bit 2: 1→0)");
		Stepper_Stop();
		Stepper_Disable(); // QUICK_STOP: Full de-energization (EN=HIGH, coils off)
		motor_ctrl.current_state = MOTOR_STATE_QUICKSTOPPING;
		motor_ctrl.pump_state = PUMPSTATE_STOPPING;

		// Clear dose active flag if dose was running
		if (motor_ctrl.dose_tracker.dose_active) {
			DBG_STATE(DOSE,
					"QUICKSTOP aborted dose at %lu.%03lu ml of %lu.%03lu ml",
					motor_ctrl.dose_tracker.delivered_ml / 1000,
					motor_ctrl.dose_tracker.delivered_ml % 1000,
					motor_ctrl.dose_tracker.target_volume_ml / 1000,
					motor_ctrl.dose_tracker.target_volume_ml % 1000);
			motor_ctrl.dose_tracker.dose_active = false;
		}
		return true;
	}
	return false;
}

/**
 * @brief Handle QUICKSTOPPING state recovery
 * @param control_word  Current control word
 * @return true if still in QUICKSTOPPING (caller should return), false if recovered
 * @note QUICKSTOPPING is STICKY until bit 2 returns to 1 (normal operation)
 */
static bool HandleQuickStopRecovery(uint16_t control_word) {
	if (motor_ctrl.current_state != MOTOR_STATE_QUICKSTOPPING) {
		return false;
	}


	if (!Stepper_IsMoving()) {
		log_once.waiting = false;  // Reset waiting flag for next quickstop
		// Motor has stopped - check if QUICKSTOP bit has been released
		if (control_word & CONTROLWORD_QUICKSTOP) {
			// QUICKSTOP bit is now 1 (normal operation) - exit QUICKSTOPPING state
			// Transition to DISABLED (not ENABLED_STOPPED) because QUICK_STOP
			// de-energized the motor. Master must explicitly re-enable via
			// proper CiA 402 state machine transitions (bits 0,1,3).
			DBG_STATE(MOTOR,
					"QUICKSTOP complete - bit released, transitioning to DISABLED (motor de-energized)");
			log_once.holding = false;  // Reset for next quickstop
			motor_ctrl.current_state = MOTOR_STATE_DISABLED;
			motor_ctrl.pump_state = PUMPSTATE_STOPPED;
			// Return false to let normal state machine handle further transitions
			return false;
		} else {
			// QUICKSTOP bit still 0 (stop active)
			// CiA 402: From QUICK STOP ACTIVE, clearing bits 0+1 → SWITCH ON DISABLED
			if (!(control_word
					& (CONTROLWORD_SWITCH_ON | CONTROLWORD_ENABLE_VOLTAGE))) {
				// Bits 0,1 cleared → master wants full disable (e.g. controlword 0x0000)
				DBG_STATE(MOTOR,
						"QUICKSTOP complete + bits 0,1 cleared → transitioning to DISABLED");
				log_once.holding = false;
				Stepper_Disable();
				motor_ctrl.current_state = MOTOR_STATE_DISABLED;
				motor_ctrl.pump_state = PUMPSTATE_STOPPED;
				return true;
			}
			// Bits 0 or 1 still set but bit 2 still 0 - hold QUICKSTOPPING
			if (!log_once.holding) {
				DBG_PRINT_V(MOTOR,
						"QUICKSTOP complete (motor stopped) but bit still active - holding QUICKSTOPPING state");
				log_once.holding = true;
			}
			return true;  // Stay in QUICKSTOPPING until bit is released
		}
	} else {
		log_once.holding = false;  // Reset holding flag for next state
		// Still stopping - wait another cycle
		if (!log_once.waiting) {
			DBG_PRINT_V(MOTOR,
					"QUICKSTOP in progress - waiting for motor stop");
			log_once.waiting = true;
		}
		return true;
	}
}

/**
 * @brief Determine target state from CiA 402 ControlWord bits
 * @param control_word  Current control word
 * @return Target motor state based on bit parsing
 */
static MotorState_t DetermineTargetState(uint16_t control_word) {
	MotorState_t target_state = motor_ctrl.current_state;

	if (!(control_word & CONTROLWORD_SWITCH_ON)) {
		// Bit 0 not set - power stage disabled
		target_state = MOTOR_STATE_DISABLED;
	} else if (!(control_word & CONTROLWORD_ENABLE_VOLTAGE)) {
		// Bit 1 not set - voltage not enabled
		target_state = MOTOR_STATE_DISABLED;
	} else if (!(control_word & CONTROLWORD_ENABLE_OP)) {
		// Bits 0,1 set but bit 3 not set - power on but no motion permitted
		target_state = MOTOR_STATE_ENABLED_STOPPED;
	} else if (control_word & CONTROLWORD_HALT) {
		// Halt active (bit 8) - pause operation but maintain power
		target_state = MOTOR_STATE_ENABLED_STOPPED;
	} else if (!(control_word & CONTROLWORD_QUICKSTOP)) {
		// Bit 2 clear = Quick Stop active (LEVEL CHECK defense-in-depth)
		// Prevents RUNNING transition even if edge detection missed the 1→0 transition
		// (e.g., first ControlWord after reset already has bit 2=0)
		target_state = MOTOR_STATE_QUICKSTOPPING;
	} else {
		// All enable bits set, no halt, quick stop not active - full operation
		target_state = MOTOR_STATE_RUNNING;
	}

	return target_state;
}

/**
 * @brief Clear dose completion flags on ControlWord or FlowRate change
 * @param control_word  Current control word
 * @note Resets log_once.dose_blocking when flag is cleared
 */
static void HandleDoseCompletionFlags(uint16_t control_word) {
	if (!motor_ctrl.dose_tracker.just_completed) {
		return;
	}

	bool controlword_changed = (control_word != motor_ctrl.last_control_word);
	bool flowrate_changed = (last_known_target_flow_ml_min
			!= motor_ctrl.dose_tracker.flow_rate_at_completion);

	if (controlword_changed || flowrate_changed) {
		motor_ctrl.dose_tracker.just_completed = 0;
		motor_ctrl.dose_tracker.flow_rate_at_completion =
				last_known_target_flow_ml_min; // Update tracking variable

		// Reset one-shot log flag so next dose completion logs once
		log_once.dose_blocking = false;

		if (controlword_changed && flowrate_changed) {
			DBG_STATE(DOSE,
					"Dose completion flag cleared - new ControlWord (0x%04X) AND FlowRate (%d ml/min)",
					control_word, last_known_target_flow_ml_min);
		} else if (controlword_changed) {
			DBG_STATE(DOSE,
					"Dose completion flag cleared - new ControlWord: 0x%04X",
					control_word);
		} else {
			DBG_STATE(DOSE,
					"Dose completion flag cleared - new TargetFlowRate: %d ml/min",
					last_known_target_flow_ml_min);
		}
	}
}

/**
 * @brief Check if state transition should be blocked (deceleration or dose completion)
 * @param target_state   Desired target state
 * @param control_word   Current control word (for dose blocking log)
 * @return true if transition is blocked (caller should return)
 * @note Uses log_once.decel_blocking and log_once.dose_blocking
 */
static bool CheckTransitionBlocking(MotorState_t target_state,
		uint16_t control_word) {
	if (target_state != MOTOR_STATE_RUNNING
			|| motor_ctrl.current_state != MOTOR_STATE_ENABLED_STOPPED) {
		return false;
	}

	// Check if stepper has finished decelerating
	if (Stepper_IsMoving()) {
		// Still decelerating - block transition, stay in ENABLED_STOPPED
		if (!log_once.decel_blocking) {
			DBG_PRINT_V(MOTOR,
					"Blocking RUNNING transition - stepper still decelerating");
			log_once.decel_blocking = true;
		}
		return true;
	}

	// Stepper finished decelerating — reset one-shot flag for next time
	log_once.decel_blocking = false;

	// Dose completion blocking: block RUNNING with stale ControlWord after dose ends
	if (motor_ctrl.dose_tracker.just_completed) {
		if (!log_once.dose_blocking) {
			DBG_STATE(DOSE,
					"Blocking RUNNING transition - dose just completed (stale ControlWord 0x%04X)",
					control_word);
			log_once.dose_blocking = true;
		}
		return true;
	}

	return false;
}

/**
 * @brief Execute exit actions when leaving current state
 * @param target_state  State being transitioned TO (used for dose pause tracking)
 */
static void ExecuteExitActions(MotorState_t target_state) {
	switch (motor_ctrl.current_state) {
	case MOTOR_STATE_RUNNING:
		// Exiting RUNNING - graceful deceleration stop
		DBG_STATE(MOTOR,
				"ACTION: Stopping motor with deceleration (exit RUNNING state)");
		Stepper_NormalStop();
		motor_ctrl.pump_state = PUMPSTATE_STOPPING;

		// Dose pause tracking: record pause start time
		if (motor_ctrl.dose_tracker.dose_active
				&& target_state == MOTOR_STATE_ENABLED_STOPPED) {
			motor_ctrl.dose_tracker.pause_start_ms = HAL_GetTick();
			DBG_STATE(DOSE, "HALT initiated - pause tracking started");
		}
		break;

	case MOTOR_STATE_ENABLED_STOPPED:
	case MOTOR_STATE_QUICKSTOPPING:
	case MOTOR_STATE_FAULT:
	case MOTOR_STATE_DISABLED:
	default:
		// No exit actions needed
		break;
	}
}

/**
 * @brief Execute entry actions when entering target state
 * @param target_state  State being transitioned INTO
 * @note Handles dose latching, direction/speed config, and Stepper_StartJog()
 */
static void ExecuteEntryActions(MotorState_t target_state) {
	switch (target_state) {
	case MOTOR_STATE_DISABLED:
		// Entering DISABLED - disable driver
		DBG_STATE(MOTOR, "ACTION: Disabling motor driver");
		if (Stepper_IsMoving()) {
			Stepper_Stop();
		}
		Stepper_Disable();
		// De-energizing ABANDONS any in-flight dose (consistent with QUICKSTOP).
		// Without this, dose_active leaks true across a soft-disable (e.g. 0x0F→0x06
		// with the quick-stop bit still set) and a later re-enable would RESUME the
		// old dose instead of starting fresh. 0x2300 is already 0 (consumed at latch),
		// so clearing dose_active is all that's needed to fully drop the dose intent.
		motor_ctrl.dose_tracker.dose_active = false;
		motor_ctrl.pump_state = PUMPSTATE_STOPPED;
		break;

	case MOTOR_STATE_ENABLED_STOPPED:
		// Entering ENABLED_STOPPED - enable driver only (no speed config yet)
		DBG_STATE(MOTOR, "ACTION: Enabling motor driver (stopped)");
		Stepper_Enable();
		Stepper_SetOutputProtection(true); // Safe outputs during IDLE
		if (ProcImg_GetControlWord() & CONTROLWORD_HALT) {
			motor_ctrl.pump_state = PUMPSTATE_HALT;
		} else {
			motor_ctrl.pump_state = PUMPSTATE_STOPPED;
		}
		break;

	case MOTOR_STATE_RUNNING: {
		// Entering RUNNING - configure speed/direction before starting
		DBG_STATE(MOTOR, "ACTION: Starting motor");

		// Dose pause tracking: accumulate pause time if resuming from HALT
		if (motor_ctrl.dose_tracker.dose_active
				&& motor_ctrl.dose_tracker.pause_start_ms > 0) {
			uint32_t pause_duration = HAL_GetTick()
					- motor_ctrl.dose_tracker.pause_start_ms;
			motor_ctrl.dose_tracker.total_pause_ms += pause_duration;
			motor_ctrl.dose_tracker.pause_start_ms = 0;
			DBG_STATE(DOSE,
					"Resume from HALT - pause duration: %lu ms (total: %lu ms)",
					pause_duration, motor_ctrl.dose_tracker.total_pause_ms);
		}

		// Dose parameter latching
		uint32_t dose_volume = ProcImg_GetDoseVolume();

		if (dose_volume > 0 && !motor_ctrl.dose_tracker.dose_active) {
			// NEW DOSE MODE: Latch parameters at START command
			motor_ctrl.dose_tracker.target_volume_ml = dose_volume;
			motor_ctrl.dose_tracker.flow_rate_ml_min =
					ProcImg_GetDoseFlowRate();
			motor_ctrl.dose_tracker.timeout_sec = ProcImg_GetDoseTimeout();

			motor_ctrl.dose_tracker.dose_active = true;
			motor_ctrl.dose_tracker.start_time_ms = HAL_GetTick();
			motor_ctrl.dose_tracker.start_position = Stepper_GetPosition();
			motor_ctrl.dose_tracker.delivered_ml = 0;

			/* CONSUME-ON-LATCH: the dose intent now lives solely in dose_tracker
			 * (dose_active + target_volume_ml). Zero the OD setpoint 0x2300
			 * immediately so it can never go stale: every stop/abort/disable/
			 * fault/reset path already clears dose_active, and with 0x2300 already
			 * 0 there is nothing left to re-arm a dose on the next START. All OD
			 * dose params (0x2301 flow, 0x2305 timeout) were already read above.
			 * NOTE: 0x2300 now reads 0 during an active dose — progress is reported
			 * separately on DoseDelivered (0x2304); the master must not read 0x2300
			 * back for armed-volume/progress. */
			ProcImg_SetDoseVolume(0);

			motor_ctrl.target_flow_ml_min =
					motor_ctrl.dose_tracker.flow_rate_ml_min;

			DBG_STATE(DOSE,
					"NEW DOSE - Latched parameters: Volume=%lu.%03lu ml, FlowRate=%d ml/min, Timeout=%u sec",
					motor_ctrl.dose_tracker.target_volume_ml / 1000,
					motor_ctrl.dose_tracker.target_volume_ml % 1000,
					motor_ctrl.dose_tracker.flow_rate_ml_min,
					motor_ctrl.dose_tracker.timeout_sec);
		} else if (motor_ctrl.dose_tracker.dose_active) {
			// RESUME DOSE: Use latched parameters, preserve volume tracking
			motor_ctrl.target_flow_ml_min =
					motor_ctrl.dose_tracker.flow_rate_ml_min;
			DBG_STATE(DOSE,
					"RESUME DOSE - Continuing with latched parameters (Volume=%lu.%03lu ml, FlowRate=%d ml/min)",
					motor_ctrl.dose_tracker.target_volume_ml / 1000,
					motor_ctrl.dose_tracker.target_volume_ml % 1000,
					motor_ctrl.dose_tracker.flow_rate_ml_min);
		} else {
			// CONTINUOUS MODE: Normal operation (dose_volume = 0)
			motor_ctrl.dose_tracker.dose_active = false;
			DBG_STATE(DOSE, "Continuous mode (DoseVolume=0)");
		}

		// Direction/speed configuration
		Stepper_Direction_t direction;
		int16_t flow_magnitude;
		if (motor_ctrl.target_flow_ml_min < 0) {
			direction = STEPPER_DIR_CCW;
			flow_magnitude = -motor_ctrl.target_flow_ml_min;
		} else {
			direction = STEPPER_DIR_CW;
			flow_magnitude = motor_ctrl.target_flow_ml_min;
		}

		Stepper_SetDirection(direction);
		uint16_t target_rpm = mlPerMin_to_rpm((uint16_t) flow_magnitude);
		Stepper_SetSpeedRPM(target_rpm);
		DBG_STATE(MOTOR, "ACTION: Configured DIR=%s, RPM=%u",
				(direction == STEPPER_DIR_CW) ? "CW" : "CCW", target_rpm);
		DBG_STATE(MOTOR,
				"STARTUP DIAG: flow=%d ml/min, magnitude=%d, dir=%s, rpm=%u, ARR=%lu",
				motor_ctrl.target_flow_ml_min, flow_magnitude,
				(direction == STEPPER_DIR_CW) ? "CW" : "CCW", target_rpm,
				(unsigned long )Stepper_GetTimerARR());

		// Enable driver if not already enabled (handles DISABLED→RUNNING case)
		Stepper_Enable();
		Stepper_SetOutputProtection(false);

		// Start motor
		motor_ctrl.pump_state = PUMPSTATE_STARTING;
		Stepper_StartJog();
		motor_ctrl.pump_state = PUMPSTATE_RUNNING;
		break;
	}

	case MOTOR_STATE_QUICKSTOPPING:
	case MOTOR_STATE_FAULT:
	default:
		// Should not reach here (handled above)
		break;
	}
}

/**
 * @brief Process CiA 402 ControlWord commands
 * @note State machine prevents duplicate command execution in RPDO mode
 */
static void MotorControl_ProcessControlWord(uint16_t control_word) {
	// Detect rising and falling edges for command detection
	uint16_t rising_edges = (control_word ^ motor_ctrl.last_control_word)
			& control_word;
	uint16_t falling_edges = (control_word ^ motor_ctrl.last_control_word)
			& motor_ctrl.last_control_word;

	// === DIAGNOSTIC LOGGING: Track control word changes ===
	DBG_BLOCK_V(MOTOR) {
		if (control_word != motor_ctrl.last_control_word) {
			DBG_PRINT_V(MOTOR,
					"CW Change: 0x%04X -> 0x%04X (Rising: 0x%04X, Falling: 0x%04X)",
					motor_ctrl.last_control_word, control_word, rising_edges,
					falling_edges);

			// Log individual bit states
			DBG_PRINT_V(MOTOR,
					"Bits: EN=%d START=%d QSTOP=%d RESET=%d | Flow=%d ml/min",
					(control_word & CONTROLWORD_SWITCH_ON) ? 1 : 0,
					(control_word & CONTROLWORD_ENABLE_OP) ? 1 : 0,
					(control_word & CONTROLWORD_QUICKSTOP) ? 1 : 0,
					(control_word & CONTROLWORD_FAULT_RESET) ? 1 : 0,
					motor_ctrl.target_flow_ml_min);
		}
	}

	// Immediate commands (early returns)
	if (HandleFaultReset(rising_edges))
		return;
	if (HandleQuickStop(falling_edges, control_word))
		return;
	if (HandleQuickStopRecovery(control_word))
		return;

	// Hardware fault guard
	if (motor_ctrl.fault_active)
		return;

	// State machine
	MotorState_t target_state = DetermineTargetState(control_word);
	HandleDoseCompletionFlags(control_word);

	if (target_state != motor_ctrl.current_state) {
		if (CheckTransitionBlocking(target_state, control_word))
			return;
		DBG_STATE(MOTOR, "Transition %d -> %d", motor_ctrl.current_state,
				target_state);
		ExecuteExitActions(target_state);
		ExecuteEntryActions(target_state);
		motor_ctrl.current_state = target_state;
	}

	// Continuous HALT status tracking (no state transition needed)
	if (motor_ctrl.current_state == MOTOR_STATE_ENABLED_STOPPED) {
		if (control_word & CONTROLWORD_HALT) {
			motor_ctrl.pump_state = PUMPSTATE_HALT;
		} else if (motor_ctrl.pump_state == PUMPSTATE_HALT) {
			motor_ctrl.pump_state = PUMPSTATE_STOPPED;
		}
	}
}

/**
 * @brief Generate CiA 402 StatusWord
 * @note Implements CiA 402 Table 62 compliant state encoding
 * @note State patterns encode device state in specific bit combinations
 */
static uint16_t MotorControl_GenerateStatusWord(void) {
	uint16_t status_word = 0;

	// Get stepper hardware status
	Stepper_Status_t stepper_status;
	Stepper_GetStatus(&stepper_status);

	// Determine fault condition
	bool is_fault = motor_ctrl.fault_active || stepper_status.error;

	// Select base state pattern based on state machine
	if (is_fault) {
		// FAULT state (xxxx xxxx x00x 1000)
		status_word = STATUS_FAULT;
	} else if (motor_ctrl.current_state == MOTOR_STATE_QUICKSTOPPING) {
		// QUICK_STOP_ACTIVE state (xxxx xxxx x00x 0111)
		status_word = STATUS_QUICK_STOP_ACTIVE;
	} else if (motor_ctrl.current_state == MOTOR_STATE_DISABLED) {
		// SWITCH_ON_DISABLED state (xxxx xxxx x1xx 0000)
		status_word = STATUS_SWITCH_ON_DISABLED;
	} else if (motor_ctrl.current_state == MOTOR_STATE_ENABLED_STOPPED) {
		// SWITCHED_ON state (xxxx xxxx x01x 0011)
		status_word = STATUS_SWITCHED_ON;
	} else if (motor_ctrl.current_state == MOTOR_STATE_RUNNING) {
		// OPERATION_ENABLED state (xxxx xxxx x01x 0111)
		status_word = STATUS_OPERATION_ENABLED;
	} else {
		// Default to NOT_READY_TO_SWITCH_ON (should not reach here)
		status_word = STATUS_NOT_READY_TO_SWITCH_ON;
	}

	// === ADDITIONAL BITS (bits 4, 9, 10) ===

	// Bit 4: Voltage enabled (power stage voltage available)
	// Set when stepper driver is enabled and no fault
	if (stepper_status.enabled && !is_fault) {
		status_word |= STATUSWORD_VOLTAGE_ENABLED;
	}

	// Bit 9: Remote (controlled via network)
	// Set when in NMT operational state
	if (MY_NMT_STATE == NMTSTATE_OP) {
		status_word |= STATUSWORD_REMOTE;
	}

	// Bit 10: Target reached (at commanded speed)
	// Set when motor is running at stable speed (not accelerating/decelerating)
	if (motor_ctrl.current_state == MOTOR_STATE_RUNNING && Stepper_IsMoving()) {
		status_word |= STATUSWORD_TARGET_REACHED;
	}

	return status_word;
}


/**
 * @brief Check dose progress and handle completion/timeout
 * @note Only called when dose_active == true (zero overhead for continuous mode)
 */
static void MotorControl_CheckDoseProgress(void) {
	if (!motor_ctrl.dose_tracker.dose_active) {
		return;  // Not in dose mode, skip check
	}

	// Calculate delivered volume from stepper position
	int32_t current_position = Stepper_GetPosition();
	int32_t steps_moved = labs(
			current_position - motor_ctrl.dose_tracker.start_position);
	motor_ctrl.dose_tracker.delivered_ml = (uint32_t) ((float) steps_moved
			* 1000.0f / GetStepsPerMl());

	// Check for completion (target volume reached)
	if (motor_ctrl.dose_tracker.delivered_ml
			>= motor_ctrl.dose_tracker.target_volume_ml) {
		DBG_STATE(DOSE,
				"Target volume reached: %lu.%03lu ml (target: %lu.%03lu ml)",
				motor_ctrl.dose_tracker.delivered_ml / 1000,
				motor_ctrl.dose_tracker.delivered_ml % 1000,
				motor_ctrl.dose_tracker.target_volume_ml / 1000,
				motor_ctrl.dose_tracker.target_volume_ml % 1000);
		Stepper_NormalStop();  // Graceful deceleration
		motor_ctrl.dose_tracker.dose_active = false;

		// Update state machine to ENABLED_STOPPED
		// Without this, state remains MOTOR_STATE_RUNNING after dose completes,
		// causing next RPDO START command to be ignored (target_state == current_state)
		motor_ctrl.current_state = MOTOR_STATE_ENABLED_STOPPED;
		motor_ctrl.pump_state = PUMPSTATE_STOPPED;

		// Set dose completion flag to prevent unwanted restart
		// This blocks RUNNING transition with stale 0x000F ControlWord
		motor_ctrl.dose_tracker.just_completed = 1;
		motor_ctrl.dose_tracker.flow_rate_at_completion =
				last_known_target_flow_ml_min; // Capture current flow rate for change detection
		DBG_STATE(DOSE,
				"Dose completion flag set - blocking stale RUNNING command (flow=%d ml/min)",
				last_known_target_flow_ml_min);

		// Clear DoseVolume to prevent re-latching on next START
		ProcImg_SetDoseVolume(0);

		return;
	}

	// Check for timeout (if configured)
	if (motor_ctrl.dose_tracker.timeout_sec > 0) {
		// Calculate elapsed time with pause adjustment
		uint32_t wall_time_ms = HAL_GetTick()
				- motor_ctrl.dose_tracker.start_time_ms;
		uint32_t active_pause_ms = 0;

		// Add current pause duration if currently paused
		if (motor_ctrl.dose_tracker.pause_start_ms > 0) {
			active_pause_ms = HAL_GetTick()
					- motor_ctrl.dose_tracker.pause_start_ms;
		}

		// Effective elapsed time = wall time - total accumulated pause - active pause
		uint32_t effective_elapsed_ms = wall_time_ms
				- motor_ctrl.dose_tracker.total_pause_ms - active_pause_ms;
		uint32_t elapsed_sec = effective_elapsed_ms / 1000;

		if (elapsed_sec >= motor_ctrl.dose_tracker.timeout_sec) {
			DBG_STATE(DOSE,
					"Timeout after %lu seconds active time (delivered: %lu.%03lu ml of %lu.%03lu ml)",
					elapsed_sec, motor_ctrl.dose_tracker.delivered_ml / 1000,
					motor_ctrl.dose_tracker.delivered_ml % 1000,
					motor_ctrl.dose_tracker.target_volume_ml / 1000,
					motor_ctrl.dose_tracker.target_volume_ml % 1000);
			DBG_PRINT_V(DOSE,
					"Pause time breakdown: total_pause=%lu ms, active_pause=%lu ms",
					motor_ctrl.dose_tracker.total_pause_ms, active_pause_ms);
			Stepper_NormalStop();  // Graceful deceleration
			motor_ctrl.dose_tracker.dose_active = false;

			// Update state machine to ENABLED_STOPPED
			// Without this, state remains MOTOR_STATE_RUNNING after timeout,
			// causing next RPDO START command to be ignored (target_state == current_state)
			motor_ctrl.current_state = MOTOR_STATE_ENABLED_STOPPED;
			motor_ctrl.pump_state = PUMPSTATE_STOPPED;

			// Set dose completion flag to prevent unwanted restart
			motor_ctrl.dose_tracker.just_completed = 1;
			motor_ctrl.dose_tracker.flow_rate_at_completion =
					last_known_target_flow_ml_min;
			DBG_STATE(DOSE,
					"Timeout - dose completion flag set - blocking stale RUNNING command");

			// Clear DoseVolume to prevent re-latching on next START
			ProcImg_SetDoseVolume(0);

			return;
		}
	}

	// Update DoseDelivered in process image for TPDO2 reporting
	ProcImg_SetDoseDelivered(motor_ctrl.dose_tracker.delivered_ml);
}

/**
 * @brief Update process image with current status
 */
static void MotorControl_UpdateProcessImage(void) {
	// Generate StatusWord (0x6041)
	uint16_t status_word = MotorControl_GenerateStatusWord();
	ProcImg_SetStatusWord(status_word);

	// Calculate ActualFlowRate (0x6043 - ml/min, SIGNED with direction)
	int16_t actual_flow;

	if (Stepper_IsMoving()) {
		// Motor is running - report actual speed with direction
		uint16_t actual_rpm = Stepper_GetSpeedRPM();
		uint16_t flow_magnitude = rpm_to_mlPerMin(actual_rpm);

		Stepper_Status_t stepper_status;
		Stepper_GetStatus(&stepper_status);

		if (stepper_status.direction == STEPPER_DIR_CCW) {
			actual_flow = -(int16_t) flow_magnitude; // Negative for reverse/draw
		} else {
			actual_flow = (int16_t) flow_magnitude; // Positive for forward/delivery
		}
	} else {
		// Motor is stopped - report 0
		actual_flow = 0;
	}

	ProcImg_SetActualFlowRate(actual_flow);

	// Update PumpState (0x2400) — use state-machine-driven pump_state directly
	// Fault override ensures PUMPSTATE_FAULT is always reported regardless of pump_state
	uint8_t pump_state =
			motor_ctrl.fault_active ? PUMPSTATE_FAULT : motor_ctrl.pump_state;
	ProcImg_SetPumpState(pump_state);

	// Update ErrorRegister (0x1001) if fault
	if (motor_ctrl.fault_active) {
		ProcImg_SetErrorRegister(0x01);  // Generic error
	} else {
		ProcImg_SetErrorRegister(0x00);
	}

	// Update DoseStatus (0x2303 - for debug/calibration)
	uint8_t dose_status;
	if (motor_ctrl.dose_tracker.dose_active) {
		dose_status = DOSE_STATUS_RUNNING;
	} else if (motor_ctrl.dose_tracker.delivered_ml
			>= motor_ctrl.dose_tracker.target_volume_ml
			&& motor_ctrl.dose_tracker.target_volume_ml > 0) {
		dose_status = DOSE_STATUS_COMPLETE;
	} else {
		dose_status = DOSE_STATUS_IDLE;
	}
	ProcImg_SetDoseStatus(dose_status);

	// Update DoseDelivered (0x2304 - already updated in CheckDoseProgress, refresh here)
	ProcImg_SetDoseDelivered(motor_ctrl.dose_tracker.delivered_ml);

}


/* Diagnostic functions --------------------*/

/** Throttle timestamps for periodic diagnostics */
static uint32_t last_status_print = 0;
static uint32_t last_motor_diag_print = 0;
static uint32_t last_timer_diag_print = 0;

/**
 * @brief Print system status periodically (every 5 s)
 * @note Gated by DBG_MOTOR_ENABLE.
 */
static void MotorControl_PrintStatus(void) {
	DBG_BLOCK(MOTOR) {
		uint32_t current_time = HAL_GetTick();

		if (current_time - last_status_print < 5000) {
			return;
		}
		last_status_print = current_time;

		DBG_PRINT_V(MOTOR, "STATUS: NMT=%d, Motor=%s, Time=%lu", MY_NMT_STATE,
				MotorControl_IsOperational() ? "READY" : "DISABLED",
				current_time);
	}
}

/**
 * @brief Print TIM7 (MCO stack tick) health check every 5 s
 * @note  Gated by DBG_CAN_ENABLE. Validates gTimCnt increments at 1 kHz.
 */
static void MotorControl_PrintTimerStats(void) {
	DBG_BLOCK(CAN) {
		extern volatile uint16_t gTimCnt; /* From mcohw_STM32FDHAL.c */
		static uint16_t last_gTimCnt = 0;
		uint32_t current_time = HAL_GetTick();

		if (current_time - last_timer_diag_print < 5000) {
			return;
		}
		last_timer_diag_print = current_time;

		uint16_t current_gTimCnt = gTimCnt;
		uint16_t delta = current_gTimCnt - last_gTimCnt;

		DBG_STATE(CAN,
				"TIM7 Diagnostics: gTimCnt=%u, Delta=%u (expected ~5000)",
				current_gTimCnt, delta);

		if (delta < 4500 || delta > 5500) {
			DBG_ERROR(CAN,
					"  WARNING: Timer tick rate incorrect! Expected ~5000, got %u",
					delta);
		} else {
			DBG_STATE(CAN, "  Timer tick rate OK (within 10%% tolerance)");
		}

		last_gTimCnt = current_gTimCnt;
	}
}

/**
 * @brief Print motor diagnostics while running (every 1 s)
 * @note  Gated by DBG_MOTOR_ENABLE.
 *        Accesses TIM1 registers, GPIO pins, and stepper API.
 */
static void MotorControl_PrintDiagnostics(void) {
	DBG_BLOCK(MOTOR) {
		/* Only print when motor is running */
		if (!Stepper_IsMoving()) {
			return;
		}

		uint32_t current_time = HAL_GetTick();

		if (current_time - last_motor_diag_print < 1000) {
			return;
		}
		last_motor_diag_print = current_time;

		/* Get stepper status */
		Stepper_Status_t status;
		Stepper_GetStatus(&status);

		/* Timer values and speed */
		DBG_PRINT_V(MOTOR, "RPM=%u, ARR=%lu, PSC=%lu, Position=%ld",
				Stepper_GetSpeedRPM(), (unsigned long )Stepper_GetTimerARR(),
				(unsigned long )Stepper_GetTimerPSC(), status.current_position);

		/* GPIO states (MS pins for microstepping, SPREAD, ENABLE, DIR) */
		DBG_HW(MOTOR, "GPIO: MS1=%d MS2=%d SPREAD=%d DIR=%d EN=%d",
				HAL_GPIO_ReadPin(GPIOA, MS1_Pin),
				HAL_GPIO_ReadPin(GPIOA, MS2_Pin),
				HAL_GPIO_ReadPin(GPIOA, SPREAD_Pin),
				HAL_GPIO_ReadPin(GPIOA, DIR_Pin),
				HAL_GPIO_ReadPin(GPIOA, ENABLE_Pin));

		/* Microstep mode */
		const char *microstep_str;
		switch (status.microstep) {
		case MICROSTEP_1:
			microstep_str = "full-step(UART)";
			break;
		case MICROSTEP_2:
			microstep_str = "2-ustep(UART)";
			break;
		case MICROSTEP_8:
			microstep_str = "8-ustep(MS1=0,MS2=0)";
			break;
		case MICROSTEP_16:
			microstep_str = "16-ustep(MS1=1,MS2=1)";
			break;
		case MICROSTEP_32:
			microstep_str = "32-ustep(MS1=1,MS2=0)";
			break;
		case MICROSTEP_64:
			microstep_str = "64-ustep(MS1=0,MS2=1)";
			break;
		default:
			microstep_str = "UNKNOWN";
			break;
		}
		(void) microstep_str; /* Suppress unused warning when debug macros disabled */
		DBG_PRINT_V(MOTOR, "Microstep=%s, Dir=%s, Enabled=%d", microstep_str,
				status.direction == STEPPER_DIR_CW ? "CW" : "CCW",
				status.enabled);

		/* Step rate diagnostics — measured vs expected */
		Stepper_PrintStepRateDiagnostics();
	}
}

/**
 * @brief Public wrapper — runs all periodic diagnostics with a shared 1 s gate
 * @note  Each sub-function still keeps its own internal throttle (1 s or 5 s),
 *        so the gate here simply avoids calling into them every loop iteration.
 *        Called from the main while(1) loop.
 */
void MotorControl_RunDiagnostics(void) {
	static uint32_t last_diag_check = 0;
	uint32_t now = HAL_GetTick();

	if (now - last_diag_check < 1000) {
		return;  // Too soon — skip this cycle
	}
	last_diag_check = now;

	MotorControl_PrintDiagnostics();
	MotorControl_PrintTimerStats();
	MotorControl_PrintStatus();
}

/* MCO Event Handler ---------------------------------------------------------*/

/**
 * @brief Handle events from MCO CANopen stack callbacks
 * @param event  Pointer to event struct fired by mco_events dispatcher
 * @note  This replaces the direct coupling where user_STM32.c called
 *        MotorControl_EmergencyStop/Init/Reset directly.
 */
static void MotorControl_HandleMCOEvent(const MCO_Event_t *event) {
	switch (event->type) {

	case MCO_EVENT_FATAL_ERROR:
		// Previously: MCOUSER_FatalError called MotorControl_EmergencyStop()
		MotorControl_EmergencyStop();
		break;

	case MCO_EVENT_COMM_RESET:
		// Previously: MCOUSER_ResetCommunication called MotorControl_Init()
		if (event->init_result) {
			if (!MotorControl_Init()) {
				gMCOConfig.error_register |= 0x20;
			}
		}
		break;

	case MCO_EVENT_NMT_CHANGE:
		// Previously: MCOUSER_NMTChange had switch/case for PREOP/STOP
		switch (event->nmt_state) {
		case NMTSTATE_PREOP:
		case NMTSTATE_STOP:
			MotorControl_Reset();
			break;
		case NMTSTATE_OP:
			// No action — motor control checks NMT state each cycle
			break;
		default:
			break;
		}
		break;

	case MCO_EVENT_HEARTBEAT_LOST:
		// Previously: MCOUSER_HeartbeatLost called MotorControl_EmergencyStop()
		MotorControl_EmergencyStop();
		break;

	default:
		break;
	}
}

/**
 * @brief Register motor control as MCO event listener
 * @note  Call from main() BEFORE MCOUSER_ResetCommunication() so the
 *        COMM_RESET event handler is in place for the first stack init.
 */
void MotorControl_RegisterMCOEvents(void) {
	MCO_Events_Register(MotorControl_HandleMCOEvent);
}

/* End of motor_control.c */
