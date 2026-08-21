/**
 ******************************************************************************
 * @file    stepper.c
 * @brief   TMC2209 Stepper Motor Driver with PWM Step Generation
 * @author  CANopen Motor Control Project
 * @date    2025
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "stepper.h"
#include "tmc2209_uart.h" /* TMC_MICROSTEP — the one microstep knob */
#include "log.h"
#include <stdlib.h>
#include <stdio.h>
#include <math.h>

/* Private typedef -----------------------------------------------------------*/
typedef struct {
	Stepper_State_t state;
	Stepper_Direction_t direction;
	uint16_t speed_rpm;           // Direct RPM control
	int32_t current_position;
	uint32_t current_arr;
	uint32_t target_arr;
	bool enabled;

	PWM_Control_t pwm_control;

	// DEFERRED STOP: Flag to signal main loop to complete stop sequence
	// Set by ISR when deceleration completes, cleared by Stepper_Poll() in main loop
	bool stop_pending;

	// AVR446 CONSTANT ACCELERATION: Per-step ramp state
	// Written/read by TIM1 ISR only (no race condition)
	uint32_t accel_step_n; // Step number in acceleration sequence (n in c_n formula)
	int32_t accel_remainder;  // Carried integer division remainder for accuracy

	// STEP-COUNTED MOVE: run an exact number of pulses then auto-stop.
	// step_target_active=false → continuous jog (fields ignored, legacy path).
	// steps_remaining is decremented by the ISR; hitting 0 with an active
	// target forces the exact-cutoff deferred stop. Set by StartSteps (main)
	// before the ISR is armed; touched by the ISR thereafter → volatile.
	volatile bool step_target_active;
	volatile uint32_t steps_remaining;

	// Deferred-stop timing: ISR PWM-kill time vs. main-loop completion time
	uint32_t phase1_timestamp;
	uint32_t phase2_timestamp;
} Stepper_Control_t;

/* Private define ------------------------------------------------------------*/
// Timer clock = 80MHz
#define TIMER_CLOCK_HZ      80000000UL
#define TIMER_PRESCALER     79  // Gives 1MHz timer frequency
#define STEPPER_POSITION_LIMIT  2000000000L // Max position limit (2 billion steps)
// Calculate ARR value for desired step frequency
// Step frequency = TIMER_FREQ / (ARR + 1)
#define TIMER_FREQ_HZ       (TIMER_CLOCK_HZ / (TIMER_PRESCALER + 1))
#define STEPS_PER_SEC(rpm, microstep)  ((rpm * STEPPER_STEPS_PER_REV * microstep) / 60)
#define TIMER_ARR(rpm, microstep)      ((TIMER_FREQ_HZ / STEPS_PER_SEC(rpm, microstep)) - 1)

// Default speed at init before the master sets a flow rate (in RPM)
#define SPEED_NORMAL_RPM    30

// AVR446 constant-acceleration ramp: linear RPM/sec at all speeds. (A
// proportional ramp made RPM jumps grow near target speed → lock-up at 100 ml/min.)
// c_n = c_{n-1} - (2 * c_{n-1}) / (4*n + 1), once per step interrupt.
// Reference: Atmel app note AVR446 (doc8017)
#define ACCEL_RATE_RPM_PER_SEC  100  // Acceleration rate in RPM/sec (tunable)
#define DECEL_RATE_RPM_PER_SEC  80   // Deceleration rate in RPM/sec (gentler than accel)

// LOW SPEED RAMP BYPASS THRESHOLD
// At low speeds (high ARR values), ramps cause issues:
// - Motor can start/stop instantly at low step rates (no inertia problem)
// - Ramping with fixed step size takes forever at high ARR values
// - This can lock the motor or cause extremely slow ramp times

#define LOW_SPEED_ARR_THRESHOLD 700

// ABSOLUTE MINIMUM START ARR FLOOR
// Clamp start ARR so motor ALWAYS begins from a safe low speed.
// ARR=2000 → step rate 500 µsteps/s → pump shaft ~18.75 RPM (1600 µsteps/rev)
// → intended as a safe pull-in speed under pump load. All speeds above that
// start here, then ramp up via the AVR446 acceleration.
// (Corrected 2026-08-05: the old comment said "~9.4 RPM (1600 half-steps/rev)"
// — wrong on both counts; 500/1600 rev/s = 18.75 RPM, and they are not half
// steps. NOT yet re-validated as a pull-in speed for the direct-drive NEMA 17.)
#define ACCEL_START_ARR_MIN     2000


/* Private variables ---------------------------------------------------------*/
static Stepper_Control_t stepper = { 0 };
static bool initialized = false;
static volatile uint32_t stepper_pending_events = 0; // ISR-written event bitfield

// Step rate measurement for diagnostics
// VOLATILE: Written by TIM1 ISR (Stepper_ProcessTimerUpdate), read by main loop
// (Stepper_PrintStepRateDiagnostics)
static volatile uint32_t step_rate_counter = 0; // Counts steps in current measurement window
static volatile uint32_t step_rate_start_time = 0; // Start time of measurement window
static volatile uint32_t measured_steps_per_sec = 0; // Measured step rate
static volatile uint32_t expected_steps_per_sec = 0; // Expected step rate based on ARR

// Idle-check counters
// Allows Stepper_ResetIdleCounters() to clear them on motor start/stop transitions,
static uint32_t idle_check_count = 0;
static uint32_t idle_anomaly_count = 0;
static uint32_t idle_last_cnt_value = 0;
static bool idle_first_check = true;

/* Private function prototypes -----------------------------------------------*/
static void Stepper_StartPWM(void);
static void Stepper_StopPWM(void);
static void Stepper_UpdatePWM(void);

/* StopPWM Helper Function ---------------------------------------------------*/
static void StopPWM_ConfigureGPIOSafe(void);

/* Start sequence helper function --------------------------------------------*/
static void Stepper_ConfigureAcceleration(void);
static void Stepper_BeginMotion(void);

/* Idle-check counter reset helper ------------------------------------*/
static void Stepper_ResetIdleCounters(void);

/* Exported functions --------------------------------------------------------*/

/**
 * @brief Initialize stepper motor driver
 * @note Stops any running timer operation FIRST to prevent
 *       hardware/software state mismatch when called during NMT RESET_COMMUNICATION.
 *       Without this, TIM1 could continue generating PWM while software state is reset,
 *       causing the motor to become unresponsive on next start.
 */
bool Stepper_Init(void) {
	// Stop any running motor operation FIRST before resetting state
	// This prevents hardware/software state mismatch when Stepper_Init() is called
	// during NMT RESET_COMMUNICATION (as opposed to NMT RESET_NODE which does full HW reset)
	if (initialized) {
		Stepper_Stop(); // Ensure timer hardware is stopped before resetting software state
	}

	// Reset diagnostic counters that persist across re-init
	// These static variables are NOT cleared by simply resetting the stepper struct
	step_rate_counter = 0;
	step_rate_start_time = 0;
	measured_steps_per_sec = 0;
	expected_steps_per_sec = 0;

	// Reset idle-check counters to prevent stale data from previous run
	Stepper_ResetIdleCounters();

	// Initialize control structure
	stepper.state = STEPPER_IDLE;
	stepper.direction = STEPPER_DIR_CW;
	stepper.speed_rpm = SPEED_NORMAL_RPM;
	stepper.current_position = 0;
	stepper.enabled = false;
	stepper.accel_step_n = 0;
	stepper.accel_remainder = 0;
	stepper.step_target_active = false;
	stepper.steps_remaining = 0;

	// Initialize PWM control
	stepper.pwm_control.arr_value = TIMER_ARR(SPEED_NORMAL_RPM, TMC_MICROSTEP);
	stepper.pwm_control.duty_percent = PWM_DUTY_DEFAULT;

	// DEFERRED STOP: Initialize stop_pending flag
	stepper.stop_pending = false;

	stepper_pending_events = 0;  // Clear any stale ISR events

	// Set default GPIO states
	HAL_GPIO_WritePin(STEPPER_ENABLE_PORT, STEPPER_ENABLE_PIN, GPIO_PIN_SET); // Disabled (active low)
	HAL_GPIO_WritePin(STEPPER_DIR_PORT, STEPPER_DIR_PIN, GPIO_PIN_RESET);  // CW

	/* TMC2209: MS1=0, MS2=0 → UART slave address 0x00. With
	 * GCONF.MSTEP_REG_SELECT=1 the MS pins are address-only; resolution comes
	 * from CHOPCONF.MRES, set from TMC_MICROSTEP at TMC2209_Init(). */
	HAL_GPIO_WritePin(STEPPER_MS1_PORT, STEPPER_MS1_PIN, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(STEPPER_MS2_PORT, STEPPER_MS2_PIN, GPIO_PIN_RESET);

	// Configure timer for PWM generation
	__HAL_TIM_SET_PRESCALER(&htim1, TIMER_PRESCALER);

	// Force prescaler shadow register update immediately.
	// __HAL_TIM_SET_PRESCALER writes to the preload register; the actual PSC
	// shadow register only updates on the next Update Event. Without this UG
	// event, the first PWM period after init uses the OLD prescaler value
	// (from CubeMX MX_TIM1_Init), causing a single wrong-frequency pulse.
	TIM1->EGR = TIM_EGR_UG;

	// Set initial speed from default RPM
	uint32_t init_arr = TIMER_ARR(SPEED_NORMAL_RPM, TMC_MICROSTEP);
	stepper.current_arr = init_arr;
	stepper.target_arr = init_arr;
	stepper.pwm_control.arr_value = init_arr;

	initialized = true;

	// Reset idempotent guard in Stepper_SetOutputProtection() to known baseline.
	// The function-local static tracks last-written state to avoid flooding UART;
	// after re-init we must force it to "not protected" so the first real call
	// from the idle watchdog will execute and write the BDTR register.
	Stepper_SetOutputProtection(false);

	return true;
}


/**
 * @brief Enable motor driver
 */
void Stepper_Enable(void) {
	if (!initialized)
		return;

	HAL_GPIO_WritePin(STEPPER_ENABLE_PORT, STEPPER_ENABLE_PIN, GPIO_PIN_RESET); // Active low
	stepper.enabled = true;
}

/**
 * @brief Disable motor driver
 */
void Stepper_Disable(void) {
	if (!initialized)
		return;

	// Stop any ongoing movement
	Stepper_StopPWM();
	stepper.state = STEPPER_IDLE;
	stepper.step_target_active = false;  // cancel any step-counted move
	stepper.steps_remaining = 0;

	// Disable driver
	HAL_GPIO_WritePin(STEPPER_ENABLE_PORT, STEPPER_ENABLE_PIN, GPIO_PIN_SET); // Active low
	stepper.enabled = false;

}

/**
 * @brief Emergency stop - immediate halt
 * @note Disables PWM/interrupt FIRST — the timer must not keep stepping
 *       during the rest of the shutdown.
 */
void Stepper_Stop(void) {
	if (!initialized)
		return;

	// Already stopped — skip. MCO events call Stepper_Stop repeatedly; the full
	// shutdown + logging every time would starve the main loop.
	if (stepper.state == STEPPER_IDLE && !Stepper_IsMoving())
		return;

	DBG_PRINT(STEPPER, "Stop called - state=%d, enabled=%d",
			stepper.state, stepper.enabled);
	DBG_HW(STEPPER, "BEFORE: TIM1 CR1=0x%04lX, DIER=0x%04lX, CCER=0x%04lX",
           TIM1->CR1, TIM1->DIER, TIM1->CCER);

	Stepper_StopPWM();

	DBG_PRINT_V(STEPPER, "PWM stopped");

	stepper.state = STEPPER_IDLE;
	stepper.step_target_active = false;  // cancel any step-counted move
	stepper.steps_remaining = 0;

	DBG_PRINT_V(STEPPER, "State cleared to IDLE");

	DBG_HW(STEPPER, "AFTER: TIM1 CR1=0x%04lX, DIER=0x%04lX, CCER=0x%04lX",
           TIM1->CR1, TIM1->DIER, TIM1->CCER);
	DBG_PRINT_V(STEPPER, "STEP pin state: %s",
			HAL_GPIO_ReadPin(STEPPER_STEP_PORT, STEPPER_STEP_PIN) ? "HIGH" : "LOW");

	// One-shot post-stop verification (don't wait for the periodic idle audit)
	Stepper_SuppressIdleInterrupts();
	Stepper_SetOutputProtection(true);
	Stepper_ResetIdleCounters(); // Fresh counters for this idle period
	Stepper_CheckIdleState();

	DBG_PRINT(STEPPER, "Stop complete");
}

/**
 * @brief Normal stop - graceful deceleration
 * @note Uses same acceleration logic in reverse: increases ARR to slow down
 *       Motor will stop when ARR reaches target
 * @note LOW SPEED BYPASS: At low speeds (high ARR), skip deceleration and stop immediately.
 *       Low speeds have no inertia issues, and ramping takes forever at high ARR values.
 */
void Stepper_NormalStop(void) {
	if (!initialized)
		return;

	// Only decelerate if we're actually running
	if (stepper.state != STEPPER_RUNNING) {
		return;
	}

	// A graceful stop cancels any step-counted move — the exact-cutoff logic no
	// longer owns the stop, so let the normal decel path end it. (Pause/resume
	// that PRESERVES steps_remaining is a motor_control-layer concern, not here.)
	stepper.step_target_active = false;

	DBG_PRINT(STEPPER, "NormalStop starting from ARR=%lu", stepper.current_arr);

	// LOW SPEED BYPASS: Skip deceleration at low speeds (high ARR values)
	// At low speeds, motor has no inertia issues and can stop instantly
	if (stepper.current_arr > LOW_SPEED_ARR_THRESHOLD) {
		DBG_PRINT(STEPPER,
				"Low speed detected (ARR=%lu > %d), using hybrid stop",
				stepper.current_arr, LOW_SPEED_ARR_THRESHOLD);
		// Deferred stop: kill PWM output now; Stepper_Poll() finishes the shutdown
		HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
		stepper.phase1_timestamp = HAL_GetTick();
		stepper.stop_pending = true;
		stepper.state = STEPPER_IDLE;
		stepper_pending_events |= STEPPER_EVT_STOPPED;
		return;
	}

	// Set deceleration target: 3x current ARR = very slow speed
	// This mirrors the acceleration startup which starts at 3x target
	stepper.target_arr = stepper.current_arr * 3;

	// Enter deceleration state
	stepper.state = STEPPER_DECELERATING;
	// AVR446: Scale step counter for deceleration rate difference
	stepper.accel_step_n = stepper.accel_step_n * ACCEL_RATE_RPM_PER_SEC
			/ DECEL_RATE_RPM_PER_SEC;
	stepper.accel_remainder = 0;
}

/**
 * @brief Set motor direction
 */
void Stepper_SetDirection(Stepper_Direction_t direction) {
	if (!initialized)
		return;

	stepper.direction = direction;
	// TMC2209 DIR polarity: CW (forward/delivery) = LOW, CCW (reverse/draw) = HIGH
	// (inverted from A4988 due to internal motor phase routing difference)
	HAL_GPIO_WritePin(STEPPER_DIR_PORT, STEPPER_DIR_PIN,
			direction == STEPPER_DIR_CW ? GPIO_PIN_RESET : GPIO_PIN_SET);
}

/**
 * @brief Set motor speed directly in RPM
 */
void Stepper_SetSpeedRPM(uint16_t rpm) {
	if (!initialized)
		return;

	// Clamp to safe range
	if (rpm > STEPPER_MAX_SPEED_RPM) {
		rpm = STEPPER_MAX_SPEED_RPM;
	}

	// Store RPM value
	stepper.speed_rpm = rpm;

	// Calculate and update ARR directly
	if (rpm == 0) {
		stepper.target_arr = 0xFFFF;  // Stopped
	} else {
		stepper.target_arr = TIMER_ARR(rpm, TMC_MICROSTEP);
	}

	// If not running, update immediately
	if (stepper.state != STEPPER_RUNNING) {
		stepper.current_arr = stepper.target_arr;
		stepper.pwm_control.arr_value = stepper.current_arr;
	}
	// If running, acceleration will handle the speed change
}

/**
 * @brief Get current motor speed in RPM
 */
uint16_t Stepper_GetSpeedRPM(void) {
	if (!initialized)
		return 0;
	return stepper.speed_rpm;
}

/**
 * @brief Common start sequence for jog and step-counted moves
 * @note  Caller must set the run mode (step_target_active / steps_remaining)
 *        FIRST and have verified STEPPER_IDLE. This is the exact body the
 *        validated StartJog used to inline.
 */
static void Stepper_BeginMotion(void) {
	// Clear any stale stop_pending flag from a previous deferred stop.
	// Race: if Stepper_Poll() hasn't run yet after a NormalStop, a leftover
	// stop_pending would immediately re-stop on the next Poll() cycle (seen as
	// rapid start-stop-start oscillation / CW-CCW hunting).
	stepper.stop_pending = false;

	// Auto-enable motor if not enabled
	if (!stepper.enabled) {
		Stepper_Enable();
	}

	stepper.state = STEPPER_RUNNING;

	// Configure acceleration profile (ARR calculation + low-speed bypass logic)
	Stepper_ConfigureAcceleration();
	Stepper_UpdatePWM();
	// Start PWM
	Stepper_StartPWM();
}

/**
 * @brief Start continuous movement (jog mode) — runs until stopped elsewhere
 */
void Stepper_StartJog(void) {
	DBG_PRINT(STEPPER, "StartJog called - state=%d, enabled=%d", stepper.state,
			stepper.enabled);

	if (!initialized || stepper.state != STEPPER_IDLE) {
		DBG_PRINT(STEPPER,
				"StartJog rejected - init=%d, state=%d (must be IDLE)",
				initialized, stepper.state);
		return;
	}

	stepper.step_target_active = false;  // continuous — no pulse target
	stepper.steps_remaining = 0;
	Stepper_BeginMotion();

	DBG_PRINT(STEPPER, "StartJog started - target_arr=%lu, speed=%u RPM",
			stepper.target_arr, Stepper_GetSpeedRPM());
}

/**
 * @brief Start a step-counted move: deliver EXACTLY `count` pulses then auto-stop
 * @note  count==0 falls through to continuous jog. The ISR decrements
 *        steps_remaining; at 0 it forces the exact-cutoff deferred stop and
 *        fires STEPPER_EVT_TARGET_REACHED. Set direction/speed before calling.
 */
void Stepper_StartSteps(uint32_t count) {
	if (count == 0) {
		Stepper_StartJog();  // 0 = run continuously
		return;
	}

	DBG_PRINT(STEPPER, "StartSteps called - count=%lu, state=%d, enabled=%d",
			count, stepper.state, stepper.enabled);

	if (!initialized || stepper.state != STEPPER_IDLE) {
		DBG_PRINT(STEPPER,
				"StartSteps rejected - init=%d, state=%d (must be IDLE)",
				initialized, stepper.state);
		return;
	}

	stepper.steps_remaining = count;
	stepper.step_target_active = true;
	Stepper_BeginMotion();

	DBG_PRINT(STEPPER,
			"StartSteps started - count=%lu, target_arr=%lu, speed=%u RPM",
			count, stepper.target_arr, Stepper_GetSpeedRPM());
}

/**
 * @brief Get current position
 */
int32_t Stepper_GetPosition(void) {
	return stepper.current_position;
}

/**
 * @brief Get pulses left in the active step-counted move
 * @retval steps_remaining (0 = idle or continuous jog)
 * @note   Single aligned 32-bit read → atomic on Cortex-M4; the ISR only
 *         decrements it, so a lock-free read is safe. Feeds TPDO2
 *         StepsRemaining for MIK dose-progress display.
 */
uint32_t Stepper_GetStepsRemaining(void) {
	return stepper.steps_remaining;
}

/**
 * @brief Get stepper status
 */
void Stepper_GetStatus(Stepper_Status_t *status) {
	if (!status)
		return;

	status->state = stepper.state;
	status->direction = stepper.direction;
	status->microstep = TMC_MICROSTEP;
	status->current_position = stepper.current_position;
	status->enabled = stepper.enabled;
	status->error = (stepper.state == STEPPER_ERROR);
}

/**
 * @brief Check if motor is physically moving
 * @note  Hardware register check: true only when the TIM1 counter is enabled
 *        AND PWM output is active.
 */
bool Stepper_IsMoving(void) {
	return (TIM1->CR1 & TIM_CR1_CEN) && (TIM1->CCER & TIM_CCER_CC1E);
}

/**
 * @brief Clear error condition
 */
void Stepper_ClearError(void) {
	if (stepper.state == STEPPER_ERROR) {
		stepper.state = STEPPER_IDLE;
	}
}

/**
 * @brief Process stepper motor updates
 * @note Call this from HAL_TIM_PeriodElapsedCallback when TIM1 triggers
 */
void Stepper_ProcessTimerUpdate(void) {
	// Process both RUNNING and DECELERATING states
	if (!initialized
			|| (stepper.state != STEPPER_RUNNING
					&& stepper.state != STEPPER_DECELERATING)) {
		return;
	}

	// Step rate counter for main-loop diagnostics (every PWM pulse = 1 microstep)
	step_rate_counter++;

	// Compute the next position (±1)
	int32_t delta = (stepper.direction == STEPPER_DIR_CW) ? +1 : -1;
	int32_t next_pos = stepper.current_position + delta;

	// Position runaway guard. Deferred stop: only the fast PWM kill happens here —
	// a full shutdown in the ISR would block FDCAN/SysTick for 50-100+ µs.
	if (labs(next_pos) >= STEPPER_POSITION_LIMIT) {
		HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
		stepper.phase1_timestamp = HAL_GetTick();
		stepper.stop_pending = true; // Stepper_Poll() finishes the shutdown
		stepper.state = STEPPER_ERROR;
		stepper.step_target_active = false;  // abort any counted move
		stepper_pending_events |= STEPPER_EVT_ERROR;
		return;
	}

	// Commit the move
	stepper.current_position = next_pos;

	// --- Step-counted move: decrement, brake within range, exact cutoff ------
	if (stepper.step_target_active) {
		if (stepper.steps_remaining > 0) {
			stepper.steps_remaining--;
		}

		if (stepper.steps_remaining == 0) {
			// Deliver EXACTLY the commanded pulse count. Deferred stop: the ISR
			// kills PWM output now; Stepper_Poll() finishes the full shutdown.
			HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
			stepper.phase1_timestamp = HAL_GetTick();
			stepper.stop_pending = true;
			stepper.state = STEPPER_IDLE;
			stepper.step_target_active = false;
			stepper_pending_events |= STEPPER_EVT_TARGET_REACHED;
			return;
		}

		// Begin a gentle decel once within braking distance. accel_step_n is the
		// step count the accel ramp consumed; braking takes ~the same. At low
		// speed accel_step_n==0, so this never fires early and the move holds
		// constant speed to the exact cutoff above (matching the validated
		// low-speed instant-stop). The cutoff — not the ramp — ends the move.
		if (stepper.state == STEPPER_RUNNING
				&& stepper.steps_remaining <= stepper.accel_step_n) {
			stepper.target_arr = stepper.current_arr * 3;
			stepper.state = STEPPER_DECELERATING;
			stepper.accel_step_n = stepper.accel_step_n
					* ACCEL_RATE_RPM_PER_SEC / DECEL_RATE_RPM_PER_SEC;
			stepper.accel_remainder = 0;
		}
	}

	if (stepper.state == STEPPER_RUNNING
			&& stepper.current_arr > stepper.target_arr) {
		// ---- ACCELERATING: decrease ARR (speed up) ----
		stepper.accel_step_n++;
		uint32_t n = stepper.accel_step_n;
		int32_t denom = (int32_t) (4 * n + 1);
		int32_t num = 2 * (int32_t) stepper.current_arr
				+ stepper.accel_remainder;
		int32_t delta = num / denom;
		stepper.accel_remainder = num - delta * denom;

		// When AVR446 natural delta is 0, SKIP the ARR update entirely.
		// The remainder has already been accumulated correctly above and will
		// produce delta >= 1 after a few more steps.
		if (delta >= 1) {
			// Apply acceleration step
			if ((uint32_t) delta >= stepper.current_arr - stepper.target_arr) {
				// Would overshoot — clamp to target
				stepper.current_arr = stepper.target_arr;
			} else {
				stepper.current_arr -= (uint32_t) delta;
			}

			// Update timer hardware
			__HAL_TIM_SET_AUTORELOAD(&htim1, stepper.current_arr);
			stepper.pwm_control.arr_value = stepper.current_arr;
			Stepper_UpdatePWM();

			// Fire AT_SPEED event when acceleration reaches target
			if (stepper.current_arr == stepper.target_arr) {
				stepper_pending_events |= STEPPER_EVT_AT_SPEED;
			}
		}
		// else: delta == 0 → remainder still accumulating; skip the update this step

	} else if (stepper.state == STEPPER_DECELERATING
			&& stepper.current_arr < stepper.target_arr) {
		// ---- DECELERATING: increase ARR (slow down) ----
		// AVR446 decel: same formula but ARR increases and n decrements
		uint32_t n = stepper.accel_step_n;
		int32_t denom = (int32_t) (4 * n + 1);
		int32_t num = 2 * (int32_t) stepper.current_arr
				+ stepper.accel_remainder;
		int32_t delta = num / denom;
		stepper.accel_remainder = num - delta * denom;

		// When AVR446 natural delta is 0, SKIP the ARR update entirely.
		// Same reasoning as acceleration: at low ARR (high speed), forcing delta=1
		// produces disproportionately large frequency changes. Let the remainder
		// accumulate naturally and produce delta >= 1 after a few steps.
		if (delta >= 1) {
			// Apply deceleration step (ARR increases = slower)
			stepper.current_arr += (uint32_t) delta;
		}
		// else: delta == 0 → remainder still accumulating; skip the update this step

		// Decrement step counter (mirrors accel ramp in reverse)
		if (n > 0) {
			stepper.accel_step_n--;
		}

		// Check if deceleration is complete (reached target or step counter
		// exhausted). For a step-counted move the exact cutoff above owns the
		// stop — skip here so the ramp only SLOWS the motor and never ends the
		// move early (which would short the delivered pulse count).
		if ((stepper.current_arr >= stepper.target_arr
				|| stepper.accel_step_n == 0)
				&& !stepper.step_target_active) {
			stepper.current_arr = stepper.target_arr;

			// Deferred stop: kill PWM output now; Stepper_Poll() finishes the shutdown
			HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
			stepper.phase1_timestamp = HAL_GetTick();
			stepper.stop_pending = true;
			stepper.state = STEPPER_IDLE;
			stepper_pending_events |= STEPPER_EVT_STOPPED;
			return;
		}

		// Update timer hardware
		__HAL_TIM_SET_AUTORELOAD(&htim1, stepper.current_arr);
		stepper.pwm_control.arr_value = stepper.current_arr;
		Stepper_UpdatePWM();
	}
	// Jog mode runs until stopped elsewhere (NormalStop / Stop / fault)
}

/* Private functions ---------------------------------------------------------*/

/**
 * @brief Start PWM generation
 * @note Restores PA8 to TIM1_CH1 AF mode first (StopPWM leaves it in GPIO
 *       mode for physical isolation). NVIC enable comes AFTER peripheral
 *       configuration is complete.
 */
static void Stepper_StartPWM(void) {
	DBG_PRINT(TIMER, "Restoring PA8 to TIM1_CH1 AF mode");
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };
	GPIO_InitStruct.Pin = STEPPER_STEP_PIN;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;      // Alternate Function Push-Pull
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.Alternate = GPIO_AF6_TIM1;        // TIM1_CH1 on PA8 is AF6
	HAL_GPIO_Init(STEPPER_STEP_PORT, &GPIO_InitStruct);

	// Reset counter
	__HAL_TIM_SET_COUNTER(&htim1, 0);

	// Enable Update interrupt for TIM1 at peripheral level
	__HAL_TIM_ENABLE_IT(&htim1, TIM_IT_UPDATE);

	// Start PWM generation on channel 1
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);

	NVIC_EnableIRQ(TIM1_UP_TIM16_IRQn);

	DBG_PRINT(TIMER, "NVIC enabled: TIM1_UP_TIM16_IRQn");
	DBG_PRINT(TIMER, "PA8 reconnected to TIM1_CH1");
}

/*============================================================================*/
/*                      Start Sequence Helper Function Implementation         */
/*============================================================================*/

/**
 * @brief Configure the AVR446 acceleration profile for motor startup
 * @note c_0 (initial timer period) = 0.676 * f * sqrt(2 / alpha), where
 *       alpha = acceleration in steps/s² and f = timer frequency. Called
 *       ONCE at motor start (not in ISR), so sqrt() is safe.
 * @note Caller sets the duty cycle (CCR) afterwards.
 */
static void Stepper_ConfigureAcceleration(void) {
	// Calculate final ARR value from RPM speed setting
	uint16_t rpm = stepper.speed_rpm > 0 ? stepper.speed_rpm : SPEED_NORMAL_RPM;
	uint32_t final_arr = TIMER_ARR(rpm, TMC_MICROSTEP);
	stepper.target_arr = final_arr;

	// LOW SPEED BYPASS: Skip acceleration ramp at low speeds (high ARR values)
	// At low speeds, motor has no inertia issues and can start instantly
	if (final_arr > LOW_SPEED_ARR_THRESHOLD) {
		stepper.current_arr = final_arr; // Start at target immediately - no ramp
		stepper.accel_step_n = 0;
		stepper.accel_remainder = 0;
	} else {
		// AVR446: Compute c_0 (initial timer period for first step)
		// alpha = acceleration in steps/sec²
		// c_0 = 0.676 * TIMER_FREQ_HZ * sqrt(2.0 / alpha)
		// This uses floating-point but is called only once at motor start, not in ISR.
		double alpha = (double) ACCEL_RATE_RPM_PER_SEC
				* (double) STEPPER_STEPS_PER_REV * (double) TMC_MICROSTEP
				/ 60.0;
		uint32_t c_0 = (uint32_t) (0.676 * (double) TIMER_FREQ_HZ
				* sqrt(2.0 / alpha));

		// Clamp c_0 DOWN to ACCEL_START_ARR_MIN, then pre-compute the equivalent
		// step number n so the per-step formula produces correct deltas from there.
		// From AVR446, c_n ≈ c_0 / sqrt(n+1), so n = (c_0 / c_clamped)² - 1.
		// (The upward clamp below cannot fire under current parameters, c_0 >> 2000;
		// kept for safety.)

		stepper.accel_remainder = 0;

		if (c_0 > ACCEL_START_ARR_MIN) {
			// c_0 is too slow (too high ARR) — would start in resonance zone.
			// Clamp DOWN to ACCEL_START_ARR_MIN and pre-compute step counter.
			double ratio = (double) c_0 / (double) ACCEL_START_ARR_MIN;
			uint32_t pre_n = (uint32_t) (ratio * ratio) - 1;
			if (pre_n < 1)
				pre_n = 1;  // Safety: ensure at least step 1

			stepper.current_arr = ACCEL_START_ARR_MIN;
			stepper.accel_step_n = pre_n;

			DBG_PRINT(STEPPER,
					"c_0=%lu clamped to %d, pre-computed n=%lu (skipped resonance zone)",
					c_0, ACCEL_START_ARR_MIN, pre_n);
		} else if (c_0 < ACCEL_START_ARR_MIN) {
			// c_0 is too fast — clamp UP to the safe pull-in speed
			stepper.current_arr = ACCEL_START_ARR_MIN;
			stepper.accel_step_n = 0;
		} else {
			// c_0 == ACCEL_START_ARR_MIN — use as-is
			stepper.current_arr = c_0;
			stepper.accel_step_n = 0;
		}
	}

	// Apply initial speed to timer hardware
	__HAL_TIM_SET_AUTORELOAD(&htim1, stepper.current_arr);
	stepper.pwm_control.arr_value = stepper.current_arr;
}

/*============================================================================*/
/*                      StopPWM Helper Function Implementations               */
/*============================================================================*/


/**
 * @brief Switch STEP pin to GPIO mode and force LOW for physical isolation
 * @note Disconnects pin from timer peripheral so no internal PWM reaches the driver
 */
static void StopPWM_ConfigureGPIOSafe(void) {
	// Switch pin to GPIO output mode (NOT AF mode)
	// Even if timer generates internal PWM, it won't reach physical pin
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };
	GPIO_InitStruct.Pin = STEPPER_STEP_PIN;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(STEPPER_STEP_PORT, &GPIO_InitStruct);

	// Force STEP pin LOW
	HAL_GPIO_WritePin(STEPPER_STEP_PORT, STEPPER_STEP_PIN, GPIO_PIN_RESET);
}


/*============================================================================*/

/**
 * @brief Stop PWM generation with essential timer shutdown sequence
 */
static void Stepper_StopPWM(void) {
	DBG_ENTER(TIMER, "Stepper_StopPWM");

	// 1. Disable interrupt delivery (NVIC + peripheral)
	NVIC_DisableIRQ(TIM1_UP_TIM16_IRQn);
	TIM1->DIER = 0;

	// 2. Stop PWM output on CH1 (only channel used)
	HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);

	// 3. Disable timer base
	__HAL_TIM_DISABLE(&htim1);

	// 4. BDTR safety: force outputs LOW when MOE=0
	TIM1->BDTR &= ~(TIM_BDTR_AOE | TIM_BDTR_MOE);
	TIM1->BDTR |= TIM_BDTR_OSSI;

	// 5. Physical pin isolation (GPIO mode, force LOW)
	StopPWM_ConfigureGPIOSafe();

	// 6. Reset counter
	__HAL_TIM_SET_COUNTER(&htim1, 0);
}

/**
 * @brief Update PWM parameters based on current state
 */
static void Stepper_UpdatePWM(void) {
	uint32_t arr = __HAL_TIM_GET_AUTORELOAD(&htim1);
	uint32_t ccr = (arr * stepper.pwm_control.duty_percent) / 100;
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, ccr);
}


/**
 * @brief Complete deferred stops; run step-rate measurement windows
 * @note Call from the main loop (MotorControl_Process). The ISR stops PWM
 *       output immediately (a flag-only defer left ms of spurious pulses);
 *       this function finishes the full shutdown (timer base, interrupts,
 *       GPIO isolation) from main-loop context.
 */
void Stepper_Poll(void) {
	if (!initialized)
		return;

	// Step-rate measurement: the ISR only increments step_rate_counter; the
	// timed window runs here where HAL_GetTick() is safe.
	if (stepper.state == STEPPER_RUNNING
			|| stepper.state == STEPPER_DECELERATING) {
		uint32_t now = HAL_GetTick();
		if (step_rate_start_time == 0) {
			// Start a new measurement window
			step_rate_start_time = now;
			step_rate_counter = 0;
		} else if (now - step_rate_start_time >= 1000) {
			// 1-second window elapsed — capture measurement
			measured_steps_per_sec = step_rate_counter;
			expected_steps_per_sec = TIMER_FREQ_HZ / (stepper.current_arr + 1);
			// Reset for next window
			step_rate_counter = 0;
			step_rate_start_time = now;
		}
	} else {
		// Motor not running — reset measurement state
		if (step_rate_start_time != 0) {
			step_rate_start_time = 0;
			step_rate_counter = 0;
		}
	}

	if (stepper.stop_pending) {
		stepper.stop_pending = false;

		stepper.phase2_timestamp = HAL_GetTick();
		DBG_PRINT_V(STEPPER, "Deferred stop: ISR→Poll latency %lu ms",
				stepper.phase2_timestamp - stepper.phase1_timestamp);

		// PWM output already stopped by the ISR; finish timer base stop,
		// interrupt disable, GPIO isolation, counter reset
		Stepper_StopPWM();

		// One-shot post-stop verification
		Stepper_SuppressIdleInterrupts();
		Stepper_SetOutputProtection(true);
		Stepper_ResetIdleCounters();
		Stepper_CheckIdleState();

		DBG_PRINT(STEPPER, "Deferred stop completed - full shutdown done");
	}
}

/**
 * @brief Print step rate diagnostics
 * @note Call periodically when motor is running to see measured vs expected rate
 */
void Stepper_PrintStepRateDiagnostics(void) {
	DBG_BLOCK(STEPPER) {
	
		if (measured_steps_per_sec == 0 && expected_steps_per_sec == 0) {
			DBG_PRINT(STEPPER,
					"STEP_RATE: No measurement yet (run motor for >1 second)");
		return;
		}

		// Calculate actual RPM from measured step rate
		// RPM = (steps_per_sec * 60) / (steps_per_rev * microstep)
		uint32_t actual_rpm = (measured_steps_per_sec * 60)
				/ (STEPPER_STEPS_PER_REV * TMC_MICROSTEP);
		uint32_t expected_rpm = (expected_steps_per_sec * 60)
				/ (STEPPER_STEPS_PER_REV * TMC_MICROSTEP);

		// Calculate ratio
		uint32_t ratio_x100 = 0;
		if (measured_steps_per_sec > 0) {
			ratio_x100 = (expected_steps_per_sec * 100)
					/ measured_steps_per_sec;
		}

		DBG_PRINT(STEPPER, "");
		DBG_PRINT(STEPPER, "=== STEP RATE DIAGNOSTICS ===");
		DBG_PRINT(STEPPER, "  Measured Steps/sec: %lu", measured_steps_per_sec);
		DBG_PRINT(STEPPER, "  Expected Steps/sec: %lu", expected_steps_per_sec);
		DBG_PRINT(STEPPER, "  Measured RPM: %lu", actual_rpm);
		DBG_PRINT(STEPPER, "  Expected RPM: %lu", expected_rpm);
		DBG_PRINT(STEPPER, "  Commanded RPM: %u", Stepper_GetSpeedRPM());
		DBG_PRINT(STEPPER, "  Ratio (expected/measured): %lu.%02lu x",
				ratio_x100 / 100, ratio_x100 % 100);
		DBG_PRINT(STEPPER, "  Current ARR: %lu", stepper.current_arr);
		DBG_PRINT(STEPPER, "  Target ARR: %lu", stepper.target_arr);
		DBG_PRINT(STEPPER, "  Microstep Mode: 1/%u", (unsigned) TMC_MICROSTEP);
		// Analysis
		if (ratio_x100 >= 95 && ratio_x100 <= 105) {
			DBG_PRINT(STEPPER,
					"  ANALYSIS: Step rate matches expected - TIMER IS CORRECT");
			DBG_PRINT(STEPPER,
					"            If motor speed is still wrong, issue is MECHANICAL or MICROSTEPPING");
		/* NOTE: this ratio compares COMMANDED vs MEASURED timer pulse rate.
		 * It says nothing about the driver's CHOPCONF.MRES — the TMC2209
		 * consumes one pulse per microstep either way. A non-unity ratio here
		 * is a timer/ISR problem, not a microstepping problem. (The old labels
		 * claimed otherwise and would send you chasing MRES for a timer bug.) */
		} else if (ratio_x100 > 700 && ratio_x100 < 900) {
			DBG_PRINT(STEPPER,
					"  ANALYSIS: ~8x fewer pulses than commanded - TIMER/ISR fault");
		} else if (ratio_x100 > 150 && ratio_x100 < 250) {
			DBG_PRINT(STEPPER,
					"  ANALYSIS: ~2x fewer pulses than commanded - TIMER/ISR fault");
		} else if (ratio_x100 > 350 && ratio_x100 < 450) {
			DBG_PRINT(STEPPER,
					"  ANALYSIS: ~4x fewer pulses than commanded - TIMER/ISR fault");
		} else {
			DBG_PRINT(STEPPER,
					"  ANALYSIS: Unexpected ratio - INVESTIGATE TIMER OR INTERRUPT");
		}
		DBG_PRINT(STEPPER, "=== END STEP RATE DIAGNOSTICS ===");
		DBG_PRINT(STEPPER, "");
	}
}

/**
 * @brief Check IDLE state integrity (diagnostic function)
 * @note Monitors hardware state when motor should be stopped to detect source
 *       of occasional pulses during IDLE state (motor stopped, driver enabled)
 * @return true if IDLE state is valid, false if anomaly detected
 */
bool Stepper_CheckIdleState(void) {

	idle_check_count++;
	bool anomaly_detected = false;

	// Check 1: Timer base should be disabled (CR1.CEN = 0)
	if (TIM1->CR1 & TIM_CR1_CEN) {
		DBG_PRINT(IDLE, "ANOMALY: Timer base ENABLED (CR1=0x%04lX) @ %lu ms",
				TIM1->CR1, HAL_GetTick());
		anomaly_detected = true;
		idle_anomaly_count++;
	}

	// Check 2: PWM output should be disabled (CCER.CC1E = 0)
	if (TIM1->CCER & TIM_CCER_CC1E) {
		DBG_PRINT(IDLE, "ANOMALY: PWM output ENABLED (CCER=0x%04lX) @ %lu ms",
				TIM1->CCER, HAL_GetTick());
		anomaly_detected = true;
		idle_anomaly_count++;
	}

	// Check 3: Update interrupt should be disabled (DIER.UIE = 0)
	if (TIM1->DIER & TIM_DIER_UIE) {
		DBG_PRINT(IDLE,
				"ANOMALY: Update interrupt ENABLED (DIER=0x%04lX) @ %lu ms",
				TIM1->DIER, HAL_GetTick());
		anomaly_detected = true;
		idle_anomaly_count++;
	}

	// Check 4: STEP pin should be LOW
	if (HAL_GPIO_ReadPin(STEPPER_STEP_PORT, STEPPER_STEP_PIN)
			!= GPIO_PIN_RESET) {
		DBG_PRINT(IDLE, "ANOMALY: STEP pin HIGH @ %lu ms", HAL_GetTick());
		anomaly_detected = true;
		idle_anomaly_count++;
	}

	// Check 5: Counter should not be incrementing
	uint32_t current_cnt = TIM1->CNT;
	if (!idle_first_check && current_cnt != idle_last_cnt_value) {
		DBG_PRINT(IDLE,
				"ANOMALY: Counter incrementing (was %lu, now %lu) @ %lu ms",
				idle_last_cnt_value, current_cnt, HAL_GetTick());
		anomaly_detected = true;
		idle_anomaly_count++;
	}
	idle_last_cnt_value = current_cnt;
	idle_first_check = false;

	// Periodic summary every 1000 checks (~1 second at 1ms main loop rate)
	DBG_BLOCK(IDLE) {
		if (idle_check_count % 1000 == 0) {
			DBG_PRINT(IDLE,
					"Summary: %lu checks, %lu anomalies (%lu.%02lu%% fail rate)",
					idle_check_count, idle_anomaly_count,
					(idle_anomaly_count * 100) / idle_check_count,
					((idle_anomaly_count * 10000) / idle_check_count) % 100);
		}
	}

	return !anomaly_detected;
}

/**
 * @brief Suppress unauthorized TIM1 Update Interrupt during IDLE
 * @note Detects and disables UIE if enabled when motor is stopped.
 *       Centralizes TIM1->DIER access that was previously in motor_control.c
 *       (TIM1_IDLE_Watchdog).
 */
void Stepper_SuppressIdleInterrupts(void) {
	if (TIM1->DIER & 0x0001) {
		DBG_ERROR(IDLE, "VIOLATION: UIE enabled during IDLE - forcing disable");
		TIM1->DIER &= ~0x0001;
	}
}

/**
 * @brief Configure TIM1 output protection for IDLE or RUNNING mode
 * @param idle true = IDLE mode (disable AOE+MOE, set OSSI for safe outputs)
 *             false = RUNNING mode (enable AOE for active PWM generation)
 */
void Stepper_SetOutputProtection(bool idle) {
	// IDEMPOTENT GUARD: Skip redundant register writes AND log messages
	// This function is called every main-loop iteration (~1000 Hz) from the
	// idle watchdog in motor_control.c.
	static bool current_protection_state = false;
	if (idle == current_protection_state)
		return;
	current_protection_state = idle;

	if (idle) {
		// IDLE: Clear AOE+MOE, set OSSI to force outputs LOW when MOE=0
		TIM1->BDTR &= ~(TIM_BDTR_AOE | TIM_BDTR_MOE);
		TIM1->BDTR |= TIM_BDTR_OSSI;
		DBG_HW(TIMER, "Output protection ON (AOE+MOE cleared, OSSI set)");
	} else {
		// RUNNING: Enable AOE for active PWM generation
		TIM1->BDTR |= TIM_BDTR_AOE;
		DBG_HW(TIMER, "Output protection OFF (AOE enabled for RUNNING)");
	}
}

uint32_t Stepper_GetTimerARR(void) {
	return TIM1->ARR;
}

uint32_t Stepper_GetTimerPSC(void) {
	return TIM1->PSC;
}

/**
 * @brief Read-and-clear pending stepper event flags
 * @retval Bitmask of STEPPER_EVT_* flags that were pending
 * @note  __disable_irq()/__enable_irq() makes the read-and-clear atomic
 *        against the TIM1 ISR writing the bitfield.
 */
uint32_t Stepper_GetPendingEvents(void) {
	__disable_irq();
	uint32_t events = stepper_pending_events;
	stepper_pending_events = 0;
	__enable_irq();
	return events;
}

/**
 * @brief Signal a TMC2209 driver fault (DIAG pin rose) — ISR-safe
 * @note  Flag-set only; all policy (level confirmation, FAULT entry,
 *        driver disable) lives in MotorControl_Process().
 */
void Stepper_NotifyDriverFault(void) {
	stepper_pending_events |= STEPPER_EVT_DRV_FAULT;
}

/**
 * @brief EXTI rising-edge callback — TMC2209 DIAG on PA5 (TMC_DIAG_Pin)
 * @note  DIAG drives HIGH on latched driver error (short-to-GND/VS,
 *        overtemp), charge-pump undervoltage, and at power-on reset (the
 *        power-on pulse is discarded by main() clearing pending flags
 *        before arming the NVIC line). If another EXTI pin is ever added,
 *        extend the pin check here — HAL has ONE rising callback for all
 *        lines.
 */
void HAL_GPIO_EXTI_Rising_Callback(uint16_t GPIO_Pin) {
	if (GPIO_Pin == TMC_DIAG_Pin) {
		Stepper_NotifyDriverFault();
	}
}

/*============================================================================*/
/*                      Idle Counter Reset Helper                      */
/*============================================================================*/

/**
 * @brief Reset idle-check diagnostic counters to zero
 * @note Called at motor start/stop transitions so counters reflect
 *       only the current idle period, preventing uint32_t overflow
 */
static void Stepper_ResetIdleCounters(void) {
	idle_check_count = 0;
	idle_anomaly_count = 0;
	idle_last_cnt_value = 0;
	idle_first_check = true;
}
