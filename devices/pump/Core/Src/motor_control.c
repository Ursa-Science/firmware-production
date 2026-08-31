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
#include "tmc2209_uart.h"
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

// 17HS19-2004S1 NEMA 17, DIRECT DRIVE (no gearbox): timer pulses per pump
// revolution. Derived from the single microstep knob (TMC_MICROSTEP in
// tmc2209_uart.h). At the default 1/8: 200 × 8 = 1600 pulses/rev — numerically
// identical to the WPX-1's 200 motor steps × 8 gearbox. Each timer pulse = 1
// microstep. Used only for the native steps/s ↔ RPM conversion below (a pure
// mechanical relation — NO ml/calibration math lives in the pump any more;
// the MIK owns all ml arithmetic and tubing calibration).
#define STEPS_PER_REV           (STEPPER_STEPS_PER_REV * TMC_MICROSTEP)

// Speed ceiling is enforced by stepRate_to_rpm() clamping to
// STEPPER_MAX_SPEED_RPM (stepper.h) — no separate limit here.

// CiA 402 ControlWord bits (0x6040) - DS402 Standard Mapping
#define CONTROLWORD_SWITCH_ON       (1 << 0)  // Bit 0: Switch On (enable power stage)
#define CONTROLWORD_ENABLE_VOLTAGE  (1 << 1)  // Bit 1: Enable Voltage
#define CONTROLWORD_QUICKSTOP       (1 << 2)  // Bit 2: Quick Stop (ACTIVE LOW - 0=stop active, 1=normal)
#define CONTROLWORD_ENABLE_OP       (1 << 3)  // Bit 3: Enable Operation (permit motion)
#define CONTROLWORD_LAST_KNOWN_RATE (1 << 4)  // Bit 4: Use last known StepRate (manufacturer-specific)
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

// CiA 301 ErrorRegister (0x1001) bits
#define ERREG_GENERIC           0x01
#define ERREG_CURRENT           0x02
#define ERREG_TEMPERATURE       0x08
#define ERREG_COMMUNICATION     0x10
// Bits this module owns in 0x1001. The MCO stack sets others itself (e.g.
// 0x20 on init failure) — fault reset must clear only ours.
#define ERREG_MOTOR_BITS        (ERREG_GENERIC | ERREG_CURRENT \
		| ERREG_TEMPERATURE | ERREG_COMMUNICATION)

// CiA 301/402 EMCY error codes
#define EMCY_CODE_RESET         0x0000  // error cleared / no error
#define EMCY_CODE_GENERIC       0x1000
#define EMCY_CODE_CURRENT       0x2310  // continuous over-current, output side
#define EMCY_CODE_TEMPERATURE   0x4210  // device temperature
#define EMCY_CODE_MOTOR_BLOCKED 0x7121

// Fault source (EMCY manufacturer byte 5; becomes OD 0x2503 in Phase 2)
#define FAULT_SRC_NONE          0x00
#define FAULT_SRC_DIAG_SHORT    0x01  // TMC DIAG latched, DRV_STATUS shows short
#define FAULT_SRC_DIAG_OT       0x02  // TMC DIAG latched, DRV_STATUS shows overtemp
#define FAULT_SRC_STEPPER       0x03  // stepper layer error (position overflow)
#define FAULT_SRC_TMC_INIT      0x04  // reserved: boot-time init failure (Phase 2)
#define FAULT_SRC_TMC_COMM      0x05  // TMC DIAG latched but DRV_STATUS unreadable
#define FAULT_SRC_DIAG_UNKNOWN  0x06  // TMC DIAG latched, no cause flag set

/* Private typedef -----------------------------------------------------------*/

typedef struct {
	bool initialized;
	uint32_t last_update;
	uint16_t last_control_word;
	int16_t target_step_rate;  // signed µsteps/sec (sign = direction), from 0x2600

	// State machine (prevents duplicate command execution in RPDO mode)
	MotorState_t current_state;

	// State tracking
	uint8_t pump_state;
	bool fault_active;
	uint8_t error_register;  // cause-coded 0x1001 value, set/cleared at fault entry/exit

	// Stale-ControlWord guard: a step-counted move that auto-stopped will NOT
	// re-trigger RUNNING on a lingering 0x000F. Set true on TARGET_REACHED,
	// cleared by a ControlWord edge (a fresh run command). NOT keyed off
	// TargetSteps — RPDO1 is synchronous so the stack re-applies the buffered
	// TargetSteps every SYNC, which could never signal "fresh."
	bool run_consumed;

	// Pause/resume (workflow D): a HALT during a step-counted move consumed
	// TargetSteps to 0 at latch, so resume must NOT re-read the OD (that would
	// start an unbounded jog). Captured from Stepper_GetStepsRemaining() at
	// pause; on resume the move restarts with exactly this count. 0 = no paused
	// counted move (fresh start or continuous jog).
	// NOTE: capture is at pause-initiation; any decel steps physically emitted
	// while the motor ramps down are not re-counted, so at HIGH step rates a
	// paused/resumed dose can slightly over-deliver (~ramp length). Exact at the
	// low dosing rates that use the instant-stop bypass. See PUMP_DOSE_TRANSFER_PLAN.
	uint32_t paused_steps_remaining;

	// Delivered-volume fidelity (Q1): the value published to StepsRemaining
	// (0x2603 / TPDO2). While RUNNING it tracks the live stepper countdown; when
	// motion ends it is LATCHED to the true steps-not-delivered at the stopping
	// instant and HELD until the next run arms. This decouples the reported value
	// from the stepper's own counter, which Stepper_Stop()/Stepper_Disable() zero
	// on abort/disable — without this latch an aborted partial dose would report
	// StepsRemaining=0, i.e. full delivery. Captured BEFORE the stepper zeroes at
	// every terminal path (quick-stop, disable, fault, e-stop); 0 on natural
	// completion; reset to the new target when a fresh counted move arms.
	uint32_t reported_steps_remaining;

} MotorControl_t;

/* Private variables ---------------------------------------------------------*/
static MotorControl_t motor_ctrl = { 0 };

/* TMC2209 DIAG pulses seen but not level-confirmed (transients, not faults).
 * Debug-log statistic only — repeated transients suggest supply/EMI trouble. */
static uint32_t diag_transient_count = 0;

// CiA 402 Compliance: last known StepRate, persisted across state transitions so
// the motor resumes at the correct rate after halt/quick-stop when the master
// sets CONTROLWORD_LAST_KNOWN_RATE (bit 4).
//   Bit 4 = 0 (normal): StepRate from RPDO is used directly (zero means stop)
//   Bit 4 = 1 (last known): this stored value is used instead of the RPDO value
static int16_t last_known_step_rate = 0;

// one-shot log flags (prevents UART ring-buffer flooding)
// Each flag is reset at its own semantic point — NOT a blanket "reset all".
// See LogOnceFlags_t members for ownership comments.
typedef struct {
	bool run_blocking; // CheckTransitionBlocking (run consumed, stale ControlWord)
	bool holding; // HandleQuickStopRecovery (quickstop complete, bit still active)
	bool waiting;          // HandleQuickStopRecovery (motor still decelerating)
	bool decel_blocking; // CheckTransitionBlocking (stepper still decelerating)
} LogOnceFlags_t;

static LogOnceFlags_t log_once = { 0 };

/* Private function prototypes -----------------------------------------------*/
static uint16_t stepRate_to_rpm(uint16_t steps_per_sec);
static uint16_t rpm_to_stepRate(uint16_t rpm);
static void MotorControl_ProcessControlWord(uint16_t control_word);
static bool HandleFaultReset(uint16_t rising_edges);
static bool HandleQuickStop(uint16_t falling_edges, uint16_t control_word);
static bool HandleQuickStopRecovery(uint16_t control_word);
static MotorState_t DetermineTargetState(uint16_t control_word);
static bool CheckTransitionBlocking(MotorState_t target_state,
		uint16_t control_word);
static void ExecuteExitActions(MotorState_t target_state);
static void ExecuteEntryActions(MotorState_t target_state);
static void MotorControl_EnterFault(bool diag_latched);
static uint16_t MotorControl_GenerateStatusWord(void);
static void MotorControl_UpdateProcessImage(void);
static void MotorControl_ProcessStepperEvents(void);
static void MotorControl_PrintStatus(void);
static void MotorControl_PrintTimerStats(void);
static void MotorControl_PrintDiagnostics(void);

/* Fault entry ----------------------------------------------------------------*/

/**
 * @brief Enter FAULT state with cause classification, dual 0x1001 update, EMCY
 * @param diag_latched  true = TMC DIAG confirmed latched high (classify via
 *                      DRV_STATUS); false = stepper-layer error
 * @note  0x1001 has TWO consumers: TPDO2 is served from the process image
 *        (published in UpdateProcessImage), but SDO reads (mco.c) and the
 *        EMCY frame's ER byte (mcop.c) are served from
 *        gMCOConfig.error_register. The Phase 0a bench proved they disagree
 *        unless BOTH are written — so both are written here.
 * @note  Caller keeps responsibility for stopping/de-energizing the stepper;
 *        this function only classifies and reports.
 */
static void MotorControl_EnterFault(bool diag_latched) {
	uint16_t emcy_code = EMCY_CODE_GENERIC;
	uint8_t error_reg = ERREG_GENERIC;
	uint8_t source = FAULT_SRC_STEPPER;
	uint32_t drv_status = 0;

	if (diag_latched) {
		/* Classify from the chip's own fault flags. Real DIAG causes are
		 * latched by the TMC until the driver is disabled, so DRV_STATUS
		 * still shows them here. Safe in main-loop context: USART2 is
		 * TMC-only since logging moved to RTT. */
		if (TMC2209_ReadDrvStatus(&drv_status) == TMC2209_OK) {
			if (drv_status & (DRV_STATUS_S2GA | DRV_STATUS_S2GB
					| DRV_STATUS_S2VSA | DRV_STATUS_S2VSB)) {
				source = FAULT_SRC_DIAG_SHORT;
				emcy_code = EMCY_CODE_CURRENT;
				error_reg |= ERREG_CURRENT;
			} else if (drv_status & (DRV_STATUS_OT | DRV_STATUS_OTPW)) {
				source = FAULT_SRC_DIAG_OT;
				emcy_code = EMCY_CODE_TEMPERATURE;
				error_reg |= ERREG_TEMPERATURE;
			} else {
				source = FAULT_SRC_DIAG_UNKNOWN;
			}
		} else {
			source = FAULT_SRC_TMC_COMM;
			error_reg |= ERREG_COMMUNICATION;
		}
	} else {
		emcy_code = EMCY_CODE_MOTOR_BLOCKED;
	}

	motor_ctrl.current_state = MOTOR_STATE_FAULT;
	motor_ctrl.pump_state = PUMPSTATE_FAULT;
	motor_ctrl.fault_active = true;
	motor_ctrl.error_register = error_reg;
	// Latch true steps-not-delivered at the fault instant. The stepper counter is
	// still valid here — the caller stops/zeroes it AFTER EnterFault returns, and a
	// runaway ISR stop leaves steps_remaining intact. The MIK reads 0x2603 for
	// delivered-at-fault (Q1).
	motor_ctrl.reported_steps_remaining = Stepper_GetStepsRemaining();
	motor_ctrl.paused_steps_remaining = 0;  // any in-flight counted move is void
	gMCOConfig.error_register |= error_reg;

	/* Fault-feedback Phase 2: latch the DRV_STATUS snapshot + source into the
	 * SDO-only diagnostics 0x2502/0x2503 so a master can read the last fault
	 * cause off the bus without catching the EMCY frame live. Cleared on reset. */
	ProcImg_SetLastDrvStatus(drv_status);
	ProcImg_SetFaultSource(source);

	/* Event-driven EMCY — reaches the master even with SYNC off. Payload:
	 * error code + ER (stack inserts it) + DRV_STATUS snapshot (LE) + source. */
	MCOP_PushEMCY(emcy_code, (uint8_t) drv_status, (uint8_t) (drv_status >> 8),
			(uint8_t) (drv_status >> 16), (uint8_t) (drv_status >> 24), source);

	DBG_ERROR(MOTOR, "FAULT entered: source=%u, EMCY=0x%04X, ER=0x%02X, DRV_STATUS=0x%08lX",
			source, emcy_code, error_reg, (unsigned long) drv_status);
}

/* Stepper event handling ----------------------------------------------------*/

/**
 * @brief Process stepper events in main loop context
 * @note  Called from MotorControl_Process() each cycle.
 *        Reads-and-clears event bitfield atomically via Stepper_GetPendingEvents().
 */
static void MotorControl_ProcessStepperEvents(void) {
	uint32_t events = Stepper_GetPendingEvents();
	if (events == 0) {
		return;
	}

	if (events & (STEPPER_EVT_STOPPED | STEPPER_EVT_TARGET_REACHED)) {
		// TARGET_REACHED (step-counted move delivered its exact pulse count) is
		// handled like a normal STOPPED: settle into ENABLED_STOPPED. The extra
		// step is the run-consumed latch — a lingering 0x000F ControlWord must
		// NOT restart the move; only a fresh command (new TargetSteps or a CW
		// change) re-arms RUNNING (see CheckTransitionBlocking).
		if (motor_ctrl.current_state == MOTOR_STATE_RUNNING
				|| motor_ctrl.current_state == MOTOR_STATE_QUICKSTOPPING) {
			DBG_STATE(MOTOR,
					"Stepper %s event → transitioning to ENABLED_STOPPED",
					(events & STEPPER_EVT_TARGET_REACHED) ?
							"TARGET_REACHED" : "STOPPED");
			motor_ctrl.current_state = MOTOR_STATE_ENABLED_STOPPED;
			if (ProcImg_GetControlWord() & CONTROLWORD_HALT) {
				motor_ctrl.pump_state = PUMPSTATE_HALT;
			} else {
				motor_ctrl.pump_state = PUMPSTATE_STOPPED;
			}
		}
		if (events & STEPPER_EVT_TARGET_REACHED) {
			motor_ctrl.run_consumed = true;
			// Counted move delivered its exact pulse count — nothing remains (Q1).
			motor_ctrl.reported_steps_remaining = 0;
		}
	}

	if (events & STEPPER_EVT_AT_SPEED) {
		DBG_PRINT_V(MOTOR, "Stepper reached target speed");
	}

	if (events & STEPPER_EVT_ERROR) {
		DBG_ERROR(MOTOR, "Stepper ERROR event → entering FAULT state");
		MotorControl_EnterFault(false);
	}

	if (events & STEPPER_EVT_DRV_FAULT) {
		/* TMC2209 DIAG rose. Real driver errors (short-to-GND/VS, overtemp)
		 * are LATCHED by the chip and hold DIAG high until the driver is
		 * disabled — so confirm by level before faulting. A low read means a
		 * transient pulse (EMI, charge-pump undervoltage blip, or a stray
		 * power-on pulse): log and count it, but do not stop the pump. */
		if (HAL_GPIO_ReadPin(TMC_DIAG_GPIO_Port, TMC_DIAG_Pin)
				== GPIO_PIN_SET) {
			DBG_ERROR(MOTOR,
					"TMC2209 DIAG latched HIGH (short/overtemp) → FAULT, driver disabled");
			MotorControl_EnterFault(true);

			/* Stop step generation, then de-energize. Disabling (EN high)
			 * also clears the TMC's drv_err latch (datasheet §15.2) → DIAG
			 * falls → the EXTI is naturally re-armed for the next fault. */
			Stepper_Stop();
			Stepper_Disable();
		} else {
			diag_transient_count++;
			DBG_PRINT(MOTOR,
					"TMC2209 DIAG transient pulse ignored (not latched; count=%lu)",
					(unsigned long) diag_transient_count);
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

	// Initialize LED control (timer handles injected to keep the module decoupled)
	if (!LEDControl_Init(&htim2, TIM_CHANNEL_1, &htim3, TIM_CHANNEL_2, &htim4,
			TIM_CHANNEL_1)) {
		return false;
	}

	motor_ctrl.initialized = true;
	motor_ctrl.last_update = HAL_GetTick();
	motor_ctrl.last_control_word = 0x0004; // Bit 2 set = "quick stop not active" baseline
	motor_ctrl.target_step_rate = 0;
	motor_ctrl.current_state = MOTOR_STATE_DISABLED; // Initialize state machine
	motor_ctrl.pump_state = PUMPSTATE_INIT;
	motor_ctrl.fault_active = false;
	motor_ctrl.error_register = 0;
	motor_ctrl.run_consumed = false;
	motor_ctrl.paused_steps_remaining = 0;
	motor_ctrl.reported_steps_remaining = 0;

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

	// Read StepRate from process image (0x2600 - signed µsteps/sec)
	int16_t step_rate = ProcImg_GetStepRate();

	// === CONTROLWORD BIT 4: LAST_KNOWN_RATE Feature ===
	//   Bit 4 = 0 (CLEAR): Normal mode - use StepRate from RPDO directly
	//                      Zero values actually mean zero (stop), not "don't care"
	//   Bit 4 = 1 (SET):   Last known mode - ignore RPDO, use the stored value.
	//                      Useful for HALT/resume without resending the rate.
	// Always update last_known when receiving a non-zero value.
	if (step_rate != 0) {
		last_known_step_rate = step_rate;
	}

	if (control_word & CONTROLWORD_LAST_KNOWN_RATE) {
		// Bit 4 SET: use last known rate (ignore current RPDO value)
		motor_ctrl.target_step_rate = last_known_step_rate;
	} else {
		// Bit 4 CLEAR: normal mode - use RPDO value directly (zero means stop)
		motor_ctrl.target_step_rate = step_rate;
	}

	// Process control word commands
	MotorControl_ProcessControlWord(control_word);

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
		// Stop/disable only if actually moving — MCO error/heartbeat-lost events
		// fire repeatedly and the heavy shutdown would repeat while already stopped.
		if (Stepper_IsMoving()
				|| motor_ctrl.current_state == MOTOR_STATE_RUNNING) {
			// Latch true remaining BEFORE the stepper zeroes it, so the last TPDO2
			// before the node drops to PRE-OP reflects delivered-at-loss (Q1: MIK's
			// "best estimate" on heartbeat loss).
			motor_ctrl.reported_steps_remaining = Stepper_GetStepsRemaining();
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
 * @note  Clears cached StepRate on NMT reset to ensure
 *        clean state after communication reset.
 */
void MotorControl_Reset(void) {
	if (motor_ctrl.initialized) {
		// Stop/disable only when not already DISABLED — MCO NMT_CHANGE events fire
		// repeatedly. The state/variable resets below still execute either way.
		if (motor_ctrl.current_state != MOTOR_STATE_DISABLED) {
			Stepper_Stop();
			Stepper_Disable(); // De-energize coils on NMT reset (EN pin HIGH)
		}
		motor_ctrl.last_control_word = 0x0004; //Bit 2 set = "quick stop not active" baseline
		motor_ctrl.target_step_rate = 0;
		motor_ctrl.pump_state = PUMPSTATE_STOPPED;
		motor_ctrl.fault_active = false;
		motor_ctrl.error_register = 0;
		gMCOConfig.error_register &= (uint8_t) ~ERREG_MOTOR_BITS;
		motor_ctrl.current_state = MOTOR_STATE_DISABLED;
		motor_ctrl.run_consumed = false;
		motor_ctrl.paused_steps_remaining = 0;
		motor_ctrl.reported_steps_remaining = 0;
		last_known_step_rate = 0;

		// Reset all one-shot log flags on NMT reset
		memset(&log_once, 0, sizeof(log_once));
	}
}

/* Private function implementations ------------------------------------------*/

/**
 * @brief Convert a step rate (µsteps/sec magnitude) to the stepper's native RPM
 * @param steps_per_sec  unsigned µsteps/sec (already sign-stripped by caller)
 * @return RPM, rounded to nearest, clamped to STEPPER_MAX_SPEED_RPM
 * @note  Pure mechanical relation: rpm = steps_per_sec × 60 / STEPS_PER_REV.
 *        NO ml or calibration math — the pump is unit-agnostic below the bus.
 *        The rate ultimately drives the (validated) RPM ramp via
 *        Stepper_SetSpeedRPM; a native steps/s→ARR path is deferred so the
 *        proven acceleration profile is not perturbed by this refactor.
 */
static uint16_t stepRate_to_rpm(uint16_t steps_per_sec) {
	if (steps_per_sec == 0)
		return 0;

	uint32_t rpm = ((uint32_t) steps_per_sec * 60u + (STEPS_PER_REV / 2u))
			/ STEPS_PER_REV;

	if (rpm == 0) {
		rpm = 1;  // Non-zero rate must not round to a stopped motor
	}
	if (rpm > STEPPER_MAX_SPEED_RPM) {
		rpm = STEPPER_MAX_SPEED_RPM;
	}
	return (uint16_t) rpm;
}

/**
 * @brief Convert native RPM back to a step rate (µsteps/sec magnitude)
 * @param rpm  revolutions per minute
 * @return µsteps/sec, rounded to nearest
 * @note  Inverse of stepRate_to_rpm; used to report ActualStepRate (0x2602).
 */
static uint16_t rpm_to_stepRate(uint16_t rpm) {
	if (rpm == 0)
		return 0;

	return (uint16_t) (((uint32_t) rpm * STEPS_PER_REV + 30u) / 60u);
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

		// De-energize: this transition enters SWITCH_ON_DISABLED directly,
		// bypassing the DISABLED entry actions — without an explicit disable
		// the driver stays energized after a stepper-origin fault (DIAG
		// faults disable at entry; bench 2026-08-12: post-reset StatusWord
		// 0x0250, bit 4 set). CiA 402: SWITCH_ON_DISABLED = power stage off.
		if (Stepper_IsMoving()) {
			Stepper_Stop();
		}
		Stepper_Disable();

		motor_ctrl.fault_active = false;
		motor_ctrl.current_state = MOTOR_STATE_DISABLED;
		motor_ctrl.pump_state = PUMPSTATE_STOPPED;

		// Clear 0x1001 in BOTH homes (see MotorControl_EnterFault) — but only
		// the motor-owned bits; the stack owns others (e.g. 0x20 init error).
		motor_ctrl.error_register = 0;
		gMCOConfig.error_register &= (uint8_t) ~ERREG_MOTOR_BITS;
		motor_ctrl.run_consumed = false;
		motor_ctrl.paused_steps_remaining = 0;
		motor_ctrl.reported_steps_remaining = 0;  // fault cleared — no active dose

		// Clear the Phase 2 fault diagnostics (0x2502/0x2503) to their idle values
		ProcImg_SetFaultSource(FAULT_SRC_NONE);
		ProcImg_SetLastDrvStatus(0);

		// CiA 301 "error reset / no error" — master sees recovery event-driven
		MCOP_PushEMCY(EMCY_CODE_RESET, 0, 0, 0, 0, FAULT_SRC_NONE);
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
		// Latch true steps-not-delivered BEFORE Stepper_Stop() zeroes the counter,
		// so the MIK reads delivered-at-abort from 0x2603 instead of a false 0 (Q1).
		motor_ctrl.reported_steps_remaining = Stepper_GetStepsRemaining();
		Stepper_Stop();
		Stepper_Disable(); // QUICK_STOP: Full de-energization (EN=HIGH, coils off)
		motor_ctrl.current_state = MOTOR_STATE_QUICKSTOPPING;
		motor_ctrl.pump_state = PUMPSTATE_STOPPING;
		// Abort any step-counted move; the stepper cleared its own steps_remaining
		// on Stepper_Stop(), so just drop the completion latch + any paused count.
		motor_ctrl.run_consumed = false;
		motor_ctrl.paused_steps_remaining = 0;
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
 * @brief Check if state transition should be blocked (deceleration or run-consumed)
 * @param target_state   Desired target state
 * @param control_word   Current control word (for run-consumed blocking log)
 * @return true if transition is blocked (caller should return)
 * @note Uses log_once.decel_blocking and log_once.run_blocking
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

	// Run-consumed blocking: after a step-counted move auto-stops, block RUNNING
	// on a lingering enable ControlWord until a fresh command re-arms it
	// (cleared in MotorControl_ProcessControlWord).
	if (motor_ctrl.run_consumed) {
		if (!log_once.run_blocking) {
			DBG_STATE(MOTOR,
					"Blocking RUNNING transition - run consumed (stale ControlWord 0x%04X)",
					control_word);
			log_once.run_blocking = true;
		}
		return true;
	}

	return false;
}

/**
 * @brief Execute exit actions when leaving current state
 * @param target_state  State being transitioned TO (used to detect a HALT/pause)
 */
static void ExecuteExitActions(MotorState_t target_state) {
	switch (motor_ctrl.current_state) {
	case MOTOR_STATE_RUNNING:
		// Pause/resume (workflow D): if this exit is a HALT/pause (power stays
		// on → ENABLED_STOPPED) mid step-counted move, capture the remaining
		// count BEFORE decel so resume can restart it (TargetSteps was consumed
		// to 0 at latch). A continuous jog has steps_remaining==0, so nothing is
		// captured. Aborts (→QUICKSTOPPING/DISABLED) skip this and clear it.
		if (target_state == MOTOR_STATE_ENABLED_STOPPED) {
			motor_ctrl.paused_steps_remaining = Stepper_GetStepsRemaining();
			// Hold the reported remaining at the paused count so 0x2603 reflects
			// what's left mid-dose (Q1). NOTE: this is the decel-start count; the
			// physical over-delivery of the ramp-down steps on resume (Q7) is NOT
			// addressed here — that is the gated Phase B.
			motor_ctrl.reported_steps_remaining = motor_ctrl.paused_steps_remaining;
			if (motor_ctrl.paused_steps_remaining > 0) {
				DBG_STATE(MOTOR, "HALT mid-move - %lu steps held for resume",
						(unsigned long) motor_ctrl.paused_steps_remaining);
			}
		}
		// Exiting RUNNING - graceful deceleration stop
		DBG_STATE(MOTOR,
				"ACTION: Stopping motor with deceleration (exit RUNNING state)");
		Stepper_NormalStop();
		motor_ctrl.pump_state = PUMPSTATE_STOPPING;
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
 * @note Handles TargetSteps latching / resume, direction+rate config, and the
 *       Stepper_StartSteps() call (count 0 = continuous jog).
 */
static void ExecuteEntryActions(MotorState_t target_state) {
	switch (target_state) {
	case MOTOR_STATE_DISABLED:
		// Entering DISABLED - disable driver
		DBG_STATE(MOTOR, "ACTION: Disabling motor driver");
		// Latch true steps-not-delivered BEFORE the stepper is zeroed, so a
		// disable mid-dose reports delivered-at-abort on 0x2603, not a false 0 (Q1).
		// (Already-stopped entries read the frozen/paused count or 0 — both correct.)
		motor_ctrl.reported_steps_remaining = Stepper_GetStepsRemaining();
		if (Stepper_IsMoving()) {
			Stepper_Stop();
		}
		Stepper_Disable();
		// De-energizing ABANDONS any in-flight step-counted move (consistent with
		// QUICKSTOP). Drop the paused count + completion latch so a later re-enable
		// starts fresh from a new TargetSteps rather than resuming a stale move.
		motor_ctrl.paused_steps_remaining = 0;
		motor_ctrl.run_consumed = false;
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
		// Entering RUNNING - configure direction/rate, then start the move.
		DBG_STATE(MOTOR, "ACTION: Starting motor");

		// Direction + rate come from the signed StepRate (0x2600); the MIK owns
		// all ml↔steps math. Sign = direction (dose CW / draw CCW).
		Stepper_Direction_t direction;
		uint16_t rate_magnitude;
		if (motor_ctrl.target_step_rate < 0) {
			direction = STEPPER_DIR_CCW;
			rate_magnitude = (uint16_t) (-motor_ctrl.target_step_rate);
		} else {
			direction = STEPPER_DIR_CW;
			rate_magnitude = (uint16_t) motor_ctrl.target_step_rate;
		}

		Stepper_SetDirection(direction);
		uint16_t target_rpm = stepRate_to_rpm(rate_magnitude);
		Stepper_SetSpeedRPM(target_rpm);

		// Decide the move length:
		//   - resume of a paused counted move → the held remaining count
		//   - else fresh: TargetSteps (0x2601) from the OD; >0 = counted move
		//     (consume-on-latch: zero it so it can't re-arm), 0 = continuous jog.
		uint32_t steps;
		if (motor_ctrl.paused_steps_remaining > 0) {
			steps = motor_ctrl.paused_steps_remaining;
			motor_ctrl.paused_steps_remaining = 0;
			DBG_STATE(MOTOR, "RESUME - continuing held count %lu steps",
					(unsigned long) steps);
		} else {
			steps = ProcImg_GetTargetSteps();
			if (steps > 0) {
				// Zero the OD copy so an event-driven master can't leave a stale
				// TargetSteps armed. NOTE: RPDO1 is synchronous, so the stack
				// re-copies the buffered frame back on the next SYNC — the real
				// re-arm guard is run_consumed (a ControlWord edge), not this
				// write. Harmless either way; kept for the event-driven case.
				ProcImg_SetTargetSteps(0);
				DBG_STATE(MOTOR, "NEW COUNTED MOVE - %lu steps",
						(unsigned long) steps);
			} else {
				DBG_STATE(MOTOR, "CONTINUOUS jog (TargetSteps=0)");
			}
		}

		// Seed the reported remaining for this move (0 for a continuous jog).
		// UpdateProcessImage then tracks it live from the stepper counter while
		// RUNNING; this seed just avoids a one-cycle stale value at arm (Q1).
		motor_ctrl.reported_steps_remaining = steps;

		DBG_STATE(MOTOR,
				"STARTUP DIAG: rate=%d steps/s, mag=%u, dir=%s, rpm=%u, steps=%lu, ARR=%lu",
				motor_ctrl.target_step_rate, rate_magnitude,
				(direction == STEPPER_DIR_CW) ? "CW" : "CCW", target_rpm,
				(unsigned long) steps, (unsigned long) Stepper_GetTimerARR());

		// Enable driver if not already enabled (handles DISABLED→RUNNING case)
		Stepper_Enable();
		Stepper_SetOutputProtection(false);

		// Start motor. Stepper_StartSteps(0) == continuous jog; >0 delivers
		// exactly that many pulses then auto-stops (STEPPER_EVT_TARGET_REACHED).
		motor_ctrl.pump_state = PUMPSTATE_STARTING;
		Stepper_StartSteps(steps);
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
					"Bits: EN=%d START=%d QSTOP=%d RESET=%d | Rate=%d steps/s",
					(control_word & CONTROLWORD_SWITCH_ON) ? 1 : 0,
					(control_word & CONTROLWORD_ENABLE_OP) ? 1 : 0,
					(control_word & CONTROLWORD_QUICKSTOP) ? 1 : 0,
					(control_word & CONTROLWORD_FAULT_RESET) ? 1 : 0,
					motor_ctrl.target_step_rate);
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

	// Clear the run-consumed latch only on a ControlWord CHANGE (a fresh
	// enable/run edge). It deliberately does NOT key off TargetSteps: RPDO1 is a
	// SYNCHRONOUS PDO, so the stack re-copies the last received frame into the
	// process image on EVERY SYNC (mcop.c PDO_RXCOPY) — TargetSteps therefore
	// reads as the commanded N continuously, and could never signal "fresh." The
	// master re-arms a completed step-counted move with a ControlWord edge
	// (e.g. 0x0007→0x000F); a lingering 0x000F alone will not restart it.
	if (motor_ctrl.run_consumed
			&& control_word != motor_ctrl.last_control_word) {
		motor_ctrl.run_consumed = false;
		log_once.run_blocking = false;
	}

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
 * @brief Update process image with current status
 */
static void MotorControl_UpdateProcessImage(void) {
	// Generate StatusWord (0x6041)
	uint16_t status_word = MotorControl_GenerateStatusWord();
	ProcImg_SetStatusWord(status_word);

	// ActualStepRate (0x2602 - signed µsteps/sec with direction). Reported from
	// the stepper's own RPM back to steps/s (inverse of the start conversion).
	int16_t actual_rate;
	if (Stepper_IsMoving()) {
		uint16_t rate_magnitude = rpm_to_stepRate(Stepper_GetSpeedRPM());

		Stepper_Status_t stepper_status;
		Stepper_GetStatus(&stepper_status);

		if (stepper_status.direction == STEPPER_DIR_CCW) {
			actual_rate = -(int16_t) rate_magnitude; // Negative for reverse/draw
		} else {
			actual_rate = (int16_t) rate_magnitude; // Positive for forward/delivery
		}
	} else {
		actual_rate = 0;  // Motor stopped
	}
	ProcImg_SetActualStepRate(actual_rate);

	// Update PumpState (0x2400) — use state-machine-driven pump_state directly
	// Fault override ensures PUMPSTATE_FAULT is always reported regardless of pump_state
	uint8_t pump_state =
			motor_ctrl.fault_active ? PUMPSTATE_FAULT : motor_ctrl.pump_state;
	ProcImg_SetPumpState(pump_state);

	// Update ErrorRegister (0x1001) — cause-coded value maintained at fault
	// entry/exit (MotorControl_EnterFault / HandleFaultReset)
	if (motor_ctrl.fault_active) {
		ProcImg_SetErrorRegister(motor_ctrl.error_register);
	} else {
		ProcImg_SetErrorRegister(0x00);
	}

	// StepsRemaining (0x2603 - TPDO2): the MIK derives delivered volume as
	// (TargetSteps − remaining) / steps_per_ml, so this MUST equal the true
	// steps-not-delivered at rest. While RUNNING, track the live stepper countdown;
	// otherwise publish the value latched at the stopping instant (Q1). Publishing
	// the raw live counter when stopped would report 0 after an abort — because
	// Stepper_Stop()/Stepper_Disable() zero the counter — i.e. a partial dose would
	// look fully delivered. The terminal paths (quick-stop, disable, fault, e-stop)
	// capture the true remaining BEFORE the stepper is zeroed; completion sets 0.
	if (motor_ctrl.current_state == MOTOR_STATE_RUNNING) {
		motor_ctrl.reported_steps_remaining = Stepper_GetStepsRemaining();
	}
	ProcImg_SetStepsRemaining(motor_ctrl.reported_steps_remaining);
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

		/* GPIO states (MS pins for microstepping, DIAG fault input, ENABLE, DIR) */
		DBG_HW(MOTOR, "GPIO: MS1=%d MS2=%d DIAG=%d DIR=%d EN=%d",
				HAL_GPIO_ReadPin(GPIOA, MS1_Pin),
				HAL_GPIO_ReadPin(GPIOA, MS2_Pin),
				HAL_GPIO_ReadPin(TMC_DIAG_GPIO_Port, TMC_DIAG_Pin),
				HAL_GPIO_ReadPin(GPIOA, DIR_Pin),
				HAL_GPIO_ReadPin(GPIOA, ENABLE_Pin));

		/* Microstep mode (CHOPCONF.MRES, from TMC_MICROSTEP) */
		DBG_PRINT_V(MOTOR, "Microstep=1/%u (UART), Dir=%s, Enabled=%d",
				(unsigned) status.microstep,
				status.direction == STEPPER_DIR_CW ? "CW" : "CCW",
				status.enabled);

		/* Step rate diagnostics — measured vs expected */
		Stepper_PrintStepRateDiagnostics();
	}
}

/**
 * @brief Periodic TMC2209 health poll — chip-reported current scale + faults
 * @note  Reads DRV_STATUS over the TMC UART every 5 s while the motor is
 *        moving. Gated by DBG_TMC_ENABLE via DBG_BLOCK(TMC): with the flag
 *        0 the whole poll (including the UART transaction) compiles out, so
 *        the TMC bus stays quiet. Runs in main-loop context; safe since
 *        logging moved to RTT (nothing else touches USART2 at runtime).
 */
static void MotorControl_PrintTMCHealth(void) {
	DBG_BLOCK(TMC) {
		static uint32_t last_tmc_poll = 0;
		uint32_t now = HAL_GetTick();

		if (!Stepper_IsMoving() || (now - last_tmc_poll < 5000)) {
			return;
		}
		last_tmc_poll = now;

		uint32_t ds;
		TMC2209_Status_t st = TMC2209_ReadDrvStatus(&ds);
		if (st != TMC2209_OK) {
			DBG_ERROR(TMC, "DRV_STATUS read failed (status=%d)", (int) st);
			return;
		}

		/* CS_ACTUAL → peak mA: I_pk = (CS+1)/32 * 2.5 A (BTT V1.3, vsense=0,
		 * R_sense 110 mΩ — same formula as the IRUN table in tmc2209_uart.h) */
		uint32_t cs = (ds & DRV_STATUS_CS_MASK) >> 16;
		uint32_t ma_peak = ((cs + 1u) * 2500u) / 32u;

		DBG_PRINT(TMC, "DRV_STATUS=0x%08lX CS_actual=%lu (%lu mA pk)%s%s",
				(unsigned long) ds, (unsigned long) cs, (unsigned long) ma_peak,
				(ds & DRV_STATUS_STST) ? " standstill" : "",
				(ds & DRV_STATUS_T120) ? " >120C" : "");

		if (ds & DRV_STATUS_OT) {
			DBG_ERROR(TMC, "OVERTEMP SHUTDOWN — driver stage disabled!");
		} else if (ds & DRV_STATUS_OTPW) {
			DBG_ERROR(TMC, "Overtemp pre-warning (>~120C) — reduce IRUN or add cooling");
		}
		if (ds & (DRV_STATUS_S2GA | DRV_STATUS_S2GB)) {
			DBG_ERROR(TMC, "Short to GND: phase%s%s",
					(ds & DRV_STATUS_S2GA) ? " A" : "",
					(ds & DRV_STATUS_S2GB) ? " B" : "");
		}
		if (ds & (DRV_STATUS_S2VSA | DRV_STATUS_S2VSB)) {
			DBG_ERROR(TMC, "Short to supply: phase%s%s",
					(ds & DRV_STATUS_S2VSA) ? " A" : "",
					(ds & DRV_STATUS_S2VSB) ? " B" : "");
		}
		if (ds & (DRV_STATUS_OLA | DRV_STATUS_OLB)) {
			DBG_ERROR(TMC, "Open load: phase%s%s (check motor wiring)",
					(ds & DRV_STATUS_OLA) ? " A" : "",
					(ds & DRV_STATUS_OLB) ? " B" : "");
		}
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
	MotorControl_PrintTMCHealth();
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
