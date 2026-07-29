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
#include "log.h"
#include <stdlib.h>
#include <stdio.h>
#include <math.h>

/* Private typedef -----------------------------------------------------------*/
typedef struct {
	Stepper_State_t state;
	Stepper_Direction_t direction;
	Stepper_Microstep_t microstep;
	uint16_t speed_rpm;           // Direct RPM control
	int32_t current_position;
	uint32_t steps_remaining;
	uint32_t current_arr;
	uint32_t target_arr;
	uint32_t accel_counter;
	uint32_t step_counter;
	bool enabled;
	bool continuous_mode;
	uint32_t timeout_start;

	PWM_Control_t pwm_control;

	// DEFERRED STOP: Flag to signal main loop to complete stop sequence
	// Set by ISR when deceleration completes, cleared by Stepper_Poll() in main loop
	bool stop_pending;

	// AVR446 CONSTANT ACCELERATION: Per-step ramp state
	// Written/read by TIM1 ISR only (no race condition)
	uint32_t accel_step_n; // Step number in acceleration sequence (n in c_n formula)
	int32_t accel_remainder;  // Carried integer division remainder for accuracy

	// Timing measurements for hybrid deferred stop
	uint32_t phase1_timestamp;   // When ISR executed Phase 1 (HAL_TIM_PWM_Stop)
	uint32_t phase2_timestamp; // When main loop executed Phase 2 (Stepper_StopPWM)
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

// Speed presets (in RPM)
#define SPEED_SLOW_RPM      1
#define SPEED_NORMAL_RPM    30
#define SPEED_FAST_RPM      100
#define SPEED_MAX_RPM       180

// AVR446 CONSTANT ACCELERATION PARAMETERS (Fix 15 — replaces proportional ramp)
// Previous proportional ramp (current_arr / DIVISOR) produced constant % speed change,
// meaning absolute RPM jumps INCREASED near target speed → lock-up at 100ml/min.
// AVR446 algorithm produces true linear velocity ramp: constant RPM/sec at all speeds.
// Formula: c_n = c_{n-1} - (2 * c_{n-1}) / (4*n + 1), evaluated once per step interrupt.
// Reference: Atmel/Microchip Application Note AVR446 (doc8017)
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
// ARR=2000 → step rate 500 Hz → motor shaft ~9.4 RPM (1600 half-steps/rev)
// → safe pull-in under any pump load. All speeds above ~18 RPM start from
// the same safe speed, then ramp up via the proportional acceleration.
#define ACCEL_START_ARR_MIN     2000


/* Private variables ---------------------------------------------------------*/
static Stepper_Control_t stepper = { 0 };
static bool initialized = false;
static volatile uint32_t stepper_pending_events = 0; // ISR-written event bitfield

// Step rate measurement for diagnostics
// VOLATILE: Written by TIM1 ISR (Stepper_ProcessTimerUpdate), read by main loop
// (Stepper_GetStepRate, Stepper_PrintStepRateDiagnostics)
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
static void Stepper_ConfigureMicrostepPins(Stepper_Microstep_t microstep);
static void Stepper_UpdatePWM(void);

/* StopPWM Helper Function ---------------------------------------------------*/
static void StopPWM_ConfigureGPIOSafe(void);

/* Start sequence helper function --------------------------------------------*/
static void Stepper_ConfigureAcceleration(void);

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
	stepper.microstep = MICROSTEP_1; // TMC2209: CHOPCONF.MRES=8 (full step) via UART write, INTPOL=1 (256 internal interpolation)
	stepper.speed_rpm = SPEED_NORMAL_RPM;
	stepper.current_position = 0;
	stepper.steps_remaining = 0;
	stepper.enabled = false;
	stepper.continuous_mode = false;
	stepper.accel_counter = 0;
	stepper.step_counter = 0;
	stepper.accel_step_n = 0;
	stepper.accel_remainder = 0;

	// Initialize PWM control
	stepper.pwm_control.arr_value = TIMER_ARR(SPEED_NORMAL_RPM, MICROSTEP_1);
	stepper.pwm_control.duty_percent = PWM_DUTY_DEFAULT;

	// DEFERRED STOP: Initialize stop_pending flag
	stepper.stop_pending = false;

	stepper_pending_events = 0;  // Clear any stale ISR events

	// Set default GPIO states
	HAL_GPIO_WritePin(STEPPER_ENABLE_PORT, STEPPER_ENABLE_PIN, GPIO_PIN_SET); // Disabled (active low)
	HAL_GPIO_WritePin(STEPPER_DIR_PORT, STEPPER_DIR_PIN, GPIO_PIN_RESET);  // CW

	/* TMC2209: MS1=0, MS2=0 → UART slave address 0x00.
	 * MSTEP_REG_SELECT=1 in GCONF: MS pins are address-only, CHOPCONF.MRES
	 * controls microstepping. UART writes verified via IFCNT.
	 * Firmware uses MICROSTEP_1 (full step) for all step rate calculations. */
	Stepper_ConfigureMicrostepPins(MICROSTEP_1);

	// Configure timer for PWM generation
	__HAL_TIM_SET_PRESCALER(&htim1, TIMER_PRESCALER);

	// Force prescaler shadow register update immediately.
	// __HAL_TIM_SET_PRESCALER writes to the preload register; the actual PSC
	// shadow register only updates on the next Update Event. Without this UG
	// event, the first PWM period after init uses the OLD prescaler value
	// (from CubeMX MX_TIM1_Init), causing a single wrong-frequency pulse.
	TIM1->EGR = TIM_EGR_UG;

	// Set initial speed from default RPM
	uint32_t init_arr = TIMER_ARR(SPEED_NORMAL_RPM, MICROSTEP_1);
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
	stepper.steps_remaining = 0;
	stepper.continuous_mode = false;

	// Disable driver
	HAL_GPIO_WritePin(STEPPER_ENABLE_PORT, STEPPER_ENABLE_PIN, GPIO_PIN_SET); // Active low
	stepper.enabled = false;

}

/**
 * @brief Emergency stop - immediate halt
 * @note CRITICAL: Disables PWM/interrupt FIRST to prevent race condition
 *       Timer interrupt must be disabled before any delays/operations
 */
void Stepper_Stop(void) {
	if (!initialized)
		return;

	// Audit 5 Fix 1: IDLE guard — skip full nuclear shutdown when already stopped
	// Prevents main-loop starvation from repeated StopPWM + UART flood when
	// MotorControl_Reset/EmergencyStop call Stepper_Stop on every MCO event.
	if (stepper.state == STEPPER_IDLE && !Stepper_IsMoving())
		return;

	DBG_PRINT(STEPPER, "Stop called - state=%d, continuous=%d, enabled=%d",
			stepper.state, stepper.continuous_mode, stepper.enabled);
	DBG_HW(STEPPER, "BEFORE: TIM1 CR1=0x%04lX, DIER=0x%04lX, CCER=0x%04lX",
           TIM1->CR1, TIM1->DIER, TIM1->CCER);

	// 1. STOP PWM/INTERRUPT IMMEDIATELY - prevents race condition
	//    Timer interrupt was continuing to generate steps during any delays
	Stepper_StopPWM();

	DBG_PRINT_V(STEPPER, "PWM stopped");

	// 2. Clear movement state
	stepper.state = STEPPER_IDLE;
	stepper.steps_remaining = 0;
	stepper.continuous_mode = false;
	stepper.accel_counter = 0;
	stepper.step_counter = 0;

	DBG_PRINT_V(STEPPER, "State cleared to IDLE");

	DBG_HW(STEPPER, "AFTER: TIM1 CR1=0x%04lX, DIER=0x%04lX, CCER=0x%04lX",
           TIM1->CR1, TIM1->DIER, TIM1->CCER);
	DBG_PRINT_V(STEPPER, "STEP pin state: %s",
			HAL_GPIO_ReadPin(STEPPER_STEP_PORT, STEPPER_STEP_PIN) ? "HIGH" : "LOW");

	// One-shot post-stop verification
	// Catches any stop-sequence failure immediately rather than waiting for periodic audit
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

	DBG_PRINT(STEPPER, "NormalStop starting from ARR=%lu", stepper.current_arr);

	// LOW SPEED BYPASS: Skip deceleration at low speeds (high ARR values)
	// At low speeds, motor has no inertia issues and can stop instantly
	if (stepper.current_arr > LOW_SPEED_ARR_THRESHOLD) {
		DBG_PRINT(STEPPER,
				"Low speed detected (ARR=%lu > %d), using hybrid stop",
				stepper.current_arr, LOW_SPEED_ARR_THRESHOLD);
		// HYBRID DEFERRED STOP PATTERN (fixes spurious pulse timing window):
		// ISR immediately stops PWM OUTPUT (fast, safe, prevents spurious pulses)
		HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1); // IMMEDIATELY stop PWM output

		// Record timestamp when (PWM stop) executed in ISR
		stepper.phase1_timestamp = HAL_GetTick();

		// Signal main loop to complete full shutdown sequence
		stepper.stop_pending = true;
		stepper.state = STEPPER_IDLE;
		stepper.continuous_mode = false;

		// Notify motor_control that motor has stopped (polled via Stepper_GetPendingEvents)
		stepper_pending_events |= STEPPER_EVT_STOPPED;
		return;
	}

	// Set deceleration target: 3x current ARR = very slow speed
	// This mirrors the acceleration startup which starts at 3x target
	stepper.target_arr = stepper.current_arr * 3;

	// Enter deceleration state
	stepper.state = STEPPER_DECELERATING;
	stepper.continuous_mode = false;  // Ensure we stop after deceleration
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
 * @brief Set microstep resolution
 */
bool Stepper_SetMicrostep(Stepper_Microstep_t microstep) {
	if (!initialized)
		return false;

	// Validate microstep value
	// TMC2209: MICROSTEP_1/2 via UART CHOPCONF, others via MS1/MS2 pins
	if (microstep != MICROSTEP_1 && microstep != MICROSTEP_2
			&& microstep != MICROSTEP_8 && microstep != MICROSTEP_16
			&& microstep != MICROSTEP_32 && microstep != MICROSTEP_64) {
		return false;
	}

	stepper.microstep = microstep;
	Stepper_ConfigureMicrostepPins(microstep);

	// Update timer speed for new microstep setting
	if (stepper.speed_rpm > 0) {
		uint32_t arr = TIMER_ARR(stepper.speed_rpm, stepper.microstep);
		if (stepper.state == STEPPER_RUNNING) {
			stepper.target_arr = arr;
		} else {
			stepper.current_arr = arr;
			stepper.target_arr = arr;
			stepper.pwm_control.arr_value = arr;
		}
	}

	return true;
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
		stepper.target_arr = TIMER_ARR(rpm, stepper.microstep);
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
 * @brief Run specified number of steps
 */
bool Stepper_RunSteps(uint32_t steps) {
	if (!initialized || !stepper.enabled || stepper.state != STEPPER_IDLE) {
		return false;
	}

	if (steps == 0) {
		return true;
	}

	// Set up movement with acceleration
	stepper.steps_remaining = steps;
	stepper.continuous_mode = false;
	stepper.state = STEPPER_RUNNING;
	stepper.timeout_start = HAL_GetTick();
	stepper.accel_counter = 0;
	stepper.step_counter = 0;

	// Configure acceleration profile (ARR calculation + low-speed bypass logic)
	Stepper_ConfigureAcceleration();
	Stepper_UpdatePWM();

	// Start PWM
	Stepper_StartPWM();

	return true;
}

/**
 * @brief Move to absolute position
 */
bool Stepper_MoveToPosition(int32_t position) {
	if (!initialized || !stepper.enabled || stepper.state != STEPPER_IDLE) {
		return false;
	}

	int32_t delta = position - stepper.current_position;
	if (delta == 0) {
		return true;
	}

	// Set direction based on delta
	if (delta > 0) {
		Stepper_SetDirection(STEPPER_DIR_CW);
	} else {
		Stepper_SetDirection(STEPPER_DIR_CCW);
		delta = -delta;
	}

	return Stepper_RunSteps((uint32_t) delta);
}

/**
 * @brief Start continuous movement (jog mode)
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

	// Clear any stale stop_pending flag from previous deferred stop
	// Race condition: If Stepper_Poll() hasn't run yet after a previous NormalStop,
	// stop_pending=true would cause immediate re-stop on next Poll() cycle after StartJog.
	// This manifests as rapid start-stop-start oscillation visible as CW/CCW hunting.
	stepper.stop_pending = false;

	// Auto-enable motor if not enabled
	if (!stepper.enabled) {
		DBG_PRINT(STEPPER, "StartJog enabling motor driver");
		Stepper_Enable();
	}

	stepper.continuous_mode = true;
	stepper.state = STEPPER_RUNNING;
	stepper.timeout_start = HAL_GetTick();
	stepper.accel_counter = 0;
	stepper.step_counter = 0;

	// Configure acceleration profile (ARR calculation + low-speed bypass logic)
	Stepper_ConfigureAcceleration();
	Stepper_UpdatePWM();
	// Start PWM
	Stepper_StartPWM();

	DBG_PRINT(STEPPER, "StartJog started - target_arr=%lu, speed=%u RPM",
			stepper.target_arr, Stepper_GetSpeedRPM());
}

/**
 * @brief Reset position counter to zero
 */
void Stepper_ResetPosition(void) {
	if (!initialized)
		return;
	stepper.current_position = 0;
}

/**
 * @brief Get current position
 */
int32_t Stepper_GetPosition(void) {
	return stepper.current_position;
}

/**
 * @brief Get stepper status
 */
void Stepper_GetStatus(Stepper_Status_t *status) {
	if (!status)
		return;

	status->state = stepper.state;
	status->direction = stepper.direction;
	status->microstep = stepper.microstep;
	status->current_position = stepper.current_position;
	status->target_steps = stepper.steps_remaining;
	status->enabled = stepper.enabled;
	status->error = (stepper.state == STEPPER_ERROR);
}

/**
 * @brief Check if motor is physically moving
 * @note  hardware register check'
 *        Returns true only when TIM1 counter is enabled AND PWM output is active.
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

	// Step counter for position tracking (every PWM pulse = 1 step in full-step mode)
	stepper.step_counter++;
	step_rate_counter++;

	// Compute what the next position (±1)
	int32_t delta = (stepper.direction == STEPPER_DIR_CW) ? +1 : -1;
	int32_t next_pos = stepper.current_position + delta;

	// Overflow guard: catch both + and – runaway
	// Hybrid deferred stop — immediate PWM kill in ISR, full shutdown in main loop.
	// Previous code called Stepper_Stop() → Stepper_StopPWM() (50-100+ µs nuclear shutdown)
	// which blocked FDCAN, USART2, and SysTick interrupts.
	if (labs(next_pos) >= STEPPER_POSITION_LIMIT) {
		HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1); // Phase 1: immediate PWM kill
		stepper.phase1_timestamp = HAL_GetTick();
		stepper.stop_pending = true;    // Phase 2: main loop does full shutdown
		stepper.state = STEPPER_ERROR;             // signal via TPDO
		// Fire ERROR event (polled via Stepper_GetPendingEvents)
		stepper_pending_events |= STEPPER_EVT_ERROR;
		return;
	}

	// Commit the move
	stepper.current_position = next_pos;

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

			// CFire AT_SPEED event when acceleration reaches target
			if (stepper.current_arr == stepper.target_arr) {
				stepper_pending_events |= STEPPER_EVT_AT_SPEED;
			}
		}
		// else: delta == 0 → remainder accumulating, skip this step (Fix 17)

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
		// else: delta == 0 → remainder accumulating, skip this step (Fix 17)

		// Decrement step counter (mirrors accel ramp in reverse)
		if (n > 0) {
			stepper.accel_step_n--;
		}

		// Check if deceleration is complete (reached target or step counter exhausted)
		if (stepper.current_arr >= stepper.target_arr
				|| stepper.accel_step_n == 0) {
			stepper.current_arr = stepper.target_arr;

			// HYBRID DEFERRED STOP PATTERN (fixes spurious pulse timing window):
			// 1. ISR immediately stops PWM OUTPUT (fast, safe, prevents spurious pulses)
			// 2. Main loop completes full timer shutdown (safe, non-ISR context)
			HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1); // IMMEDIATELY stop PWM output

			// PHASE 1 DIAGNOSTIC: Record timestamp when Phase 1 completes
			stepper.phase1_timestamp = HAL_GetTick();

			// Signal main loop to complete full shutdown sequence
			stepper.stop_pending = true;
			stepper.state = STEPPER_IDLE;

			// Notify motor_control that motor has stopped (polled via Stepper_GetPendingEvents)
			stepper_pending_events |= STEPPER_EVT_STOPPED;
			return; // Exit early - full shutdown completed by Stepper_Poll()
		}

		// Update timer hardware
		__HAL_TIM_SET_AUTORELOAD(&htim1, stepper.current_arr);
		stepper.pwm_control.arr_value = stepper.current_arr;
		Stepper_UpdatePWM();

	// Non-continuous mode: decrement remaining steps
		// Hybrid deferred stop — immediate PWM kill in ISR, full shutdown in main loop.
	// Previous code called Stepper_Stop() → Stepper_StopPWM() (50-100+ µs nuclear shutdown)
	// which blocked FDCAN, USART2, and SysTick interrupts.
	if (!stepper.continuous_mode) {
		if (stepper.steps_remaining > 0) {
			stepper.steps_remaining--;
			if (stepper.steps_remaining == 0) {
				HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1); // Phase 1: immediate PWM kill
				stepper.phase1_timestamp = HAL_GetTick();
				stepper.stop_pending = true; // Phase 2: main loop does full shutdown
				stepper.state = STEPPER_IDLE;
					// Notify motor_control that motor has stopped (polled via Stepper_GetPendingEvents)
				stepper_pending_events |= STEPPER_EVT_STOPPED;
				return;
			}
		}
	}
	// continuous/jog mode simply runs until stopped elsewhere
	}
}

/* Phase 1 - PWM Control Functions */

/**
 * @brief Set PWM duty cycle
 */
void Stepper_SetDutyCycle(uint32_t duty_percent) {
	if (!initialized)
		return;

	// Clamp duty cycle to valid range
	if (duty_percent < PWM_DUTY_MIN)
		duty_percent = PWM_DUTY_MIN;
	if (duty_percent > PWM_DUTY_MAX)
		duty_percent = PWM_DUTY_MAX;

	stepper.pwm_control.duty_percent = duty_percent;

	// Update timer compare value
	uint32_t arr = __HAL_TIM_GET_AUTORELOAD(&htim1);
	uint32_t ccr = (arr * duty_percent) / 100;
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, ccr);
}

/**
 * @brief Get current PWM control parameters
 */
void Stepper_GetPWMControl(PWM_Control_t *control) {
	if (!control || !initialized)
		return;

	control->arr_value = stepper.pwm_control.arr_value;
	control->duty_percent = stepper.pwm_control.duty_percent;
}


/* Private functions ---------------------------------------------------------*/

/**
 * @brief Start PWM generation
	 * @note NVIC-level interrupt enable added after peripheral enable
 *       to ensure ARM core can deliver timer interrupts
	 * @note Restores PA8 to TIM1_CH1 AF mode before starting PWM
	 *       switches to GPIO mode for physical isolation during IDLE)
 */
static void Stepper_StartPWM(void) {
	DBG_PRINT(TIMER, "Phase 3.2: Restoring PA8 to TIM1_CH1 AF mode");
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

		// Enable interrupt at NVIC level (ARM core interrupt controller)
		// This allows the ARM core to actually deliver TIM1 interrupts
		// Must be done AFTER peripheral configuration is complete
	NVIC_EnableIRQ(TIM1_UP_TIM16_IRQn);

	DBG_PRINT(TIMER, "NVIC enabled: TIM1_UP_TIM16_IRQn");
	DBG_PRINT(TIMER, "Phase 3.2: PA8 reconnected to TIM1_CH1");
}

/*============================================================================*/
/*                      Start Sequence Helper Function Implementation         */
/*============================================================================*/

/**
 * @brief Configure acceleration profile for motor startup (AVR446 algorithm)
	 * @note AVR446 constant-acceleration ramp:
 *       Computes c_0 (initial timer period) from desired acceleration rate using
 *       the kinematic formula: c_0 = 0.676 * f * sqrt(2 / alpha)
 *       where alpha = acceleration in steps/sec² and f = timer frequency (Hz).
 *       This is called ONCE at motor start (not in ISR), so sqrt() is safe.
 * 
 * This function handles:
 * 1. Calculate final_arr (target speed) from speed_rpm
 * 2. Set target_arr
 * 3. Apply LOW SPEED BYPASS: At high ARR values (low speeds), skip ramp
 * 4. Compute AVR446 c_0 as starting ARR, clamped to ACCEL_START_ARR_MIN
 * 5. Initialize per-step ramp state (accel_step_n, accel_remainder)
 * 6. Update timer ARR register and pwm_control
 * 
 * @note Caller is responsible for setting duty cycle (CCR) after this function
 */
static void Stepper_ConfigureAcceleration(void) {
	// Calculate final ARR value from RPM speed setting
	uint16_t rpm = stepper.speed_rpm > 0 ? stepper.speed_rpm : SPEED_NORMAL_RPM;
	uint32_t final_arr = TIMER_ARR(rpm, stepper.microstep);
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
				* (double) STEPPER_STEPS_PER_REV * (double) stepper.microstep
				/ 60.0;
		uint32_t c_0 = (uint32_t) (0.676 * (double) TIMER_FREQ_HZ
				* sqrt(2.0 / alpha));

			// BIDIRECTIONAL c_0 CLAMP + STEP COUNTER PRE-COMPUTATION
			// Clamp c_0 DOWN to ACCEL_START_ARR_MIN (2000 → 500 Hz = F-Low per datasheet),
		// then pre-compute the equivalent step number n so the AVR446 per-step formula
		// (c_n = c_{n-1} - 2*c_{n-1}/(4n+1)) produces correct deltas from that point.
		//
		// Math: From AVR446, c_n ≈ c_0 / sqrt(n+1) for the approximate ramp profile.
		//   If c_clamped = c_0 / sqrt(n+1), then n = (c_0 / c_clamped)² - 1
		//   For c_0=13089, c_clamped=2000: n = (13089/2000)² - 1 ≈ 42
		//
		// The old Fix 11 clamp (if c_0 < ACCEL_START_ARR_MIN) is preserved for safety,
		// but will not fire under current parameters since c_0 >> 2000.

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
					"Fix16: c_0=%lu clamped to %d, pre-computed n=%lu (skipped resonance zone)",
					c_0, ACCEL_START_ARR_MIN, pre_n);
		} else if (c_0 < ACCEL_START_ARR_MIN) {
			// Fix 11 (preserved): c_0 is too fast — clamp UP to safe pull-in speed
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
 * @brief Post-shutdown diagnostic dump for Stepper_StopPWM()
 * @note  Gated by DBG_TIMER_ENABLE.
 *        Call from main-loop context after StopPWM if register-level verification needed.
 */
void Stepper_StopPWM_Diagnostic(void) {
	DBG_BLOCK(TIMER) {
		DBG_PRINT(TIMER, "=== STOP_PWM DIAGNOSTIC ===");
		DBG_PRINT(TIMER, "  SR=0x%04lX, CNT=%lu, DIER=0x%04lX, BDTR=0x%04lX",
				TIM1->SR, TIM1->CNT, TIM1->DIER, TIM1->BDTR);
		DBG_PRINT(TIMER, "  CR1=0x%04lX, CCER=0x%04lX", TIM1->CR1, TIM1->CCER);
		DBG_PRINT(TIMER, "  NVIC_EN=%lu, NVIC_PEND=%lu",
				NVIC_GetEnableIRQ(TIM1_UP_TIM16_IRQn),
				NVIC_GetPendingIRQ(TIM1_UP_TIM16_IRQn));

		// PA8 GPIO mode check (bits [17:16] of MODER for pin 8)
		uint32_t gpio_mode = (GPIOA->MODER >> (8 * 2)) & 0x03;
		DBG_PRINT(TIMER, "  PA8_mode=%lu (expect 1=GPIO output)", gpio_mode);

		// Verify critical post-shutdown invariants
		if (TIM1->DIER != 0x0000) {
			DBG_ERROR(TIMER, "VERIFICATION FAILED: DIER=0x%04lX (expect 0)",
					TIM1->DIER);
		}
		if (TIM1->CR1 & TIM_CR1_CEN) {
			DBG_ERROR(TIMER,
					"VERIFICATION FAILED: Timer still enabled (CEN=1)");
		}
		if (!(TIM1->BDTR & TIM_BDTR_OSSI)) {
			DBG_ERROR(TIMER, "VERIFICATION FAILED: OSSI not set");
		}
		if (TIM1->CCER & TIM_CCER_CC1E) {
			DBG_ERROR(TIMER,
					"VERIFICATION FAILED: PWM output still enabled (CC1E=1)");
		}
		DBG_PRINT(TIMER, "=== END STOP_PWM DIAGNOSTIC ===");
	}
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
 * @brief Configure microstep pins for TMC2209
 * @note  TMC2209 MS1/MS2 mapping (different from A4988!):
 *        MS1=0, MS2=0 →  8 µsteps (UART addr 0x00)
 *        MS1=1, MS2=0 → 32 µsteps (UART addr 0x01)
 *        MS1=0, MS2=1 → 64 µsteps (UART addr 0x02)
 *        MS1=1, MS2=1 → 16 µsteps (UART addr 0x03)
 *        PA5 (SPREAD) not connected on TMC2209 — chopper mode set via UART only.
 */
static void Stepper_ConfigureMicrostepPins(Stepper_Microstep_t microstep) {
	switch (microstep) {
	case MICROSTEP_1:
		// UART-configured: CHOPCONF.mres=8 sets full step via register.
		// Fall through — MS1=0, MS2=0 for UART slave address 0x00.
	case MICROSTEP_2:
		// UART-configured: CHOPCONF.mres=7 sets 2 µsteps via register.
		// MS1=0, MS2=0 → UART slave address 0x00 (with MSTEP_REG_SELECT=1,
		// MS pins no longer control microstep resolution).
		HAL_GPIO_WritePin(STEPPER_MS1_PORT, STEPPER_MS1_PIN, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(STEPPER_MS2_PORT, STEPPER_MS2_PIN, GPIO_PIN_RESET);
		break;

	case MICROSTEP_8:
		// MS1=0, MS2=0 → 8 µsteps, UART addr 0x00
		HAL_GPIO_WritePin(STEPPER_MS1_PORT, STEPPER_MS1_PIN, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(STEPPER_MS2_PORT, STEPPER_MS2_PIN, GPIO_PIN_RESET);
		break;

	case MICROSTEP_32:
		// MS1=1, MS2=0 → 32 µsteps, UART addr 0x01
		HAL_GPIO_WritePin(STEPPER_MS1_PORT, STEPPER_MS1_PIN, GPIO_PIN_SET);
		HAL_GPIO_WritePin(STEPPER_MS2_PORT, STEPPER_MS2_PIN, GPIO_PIN_RESET);
		break;

	case MICROSTEP_64:
		// MS1=0, MS2=1 → 64 µsteps, UART addr 0x02
		HAL_GPIO_WritePin(STEPPER_MS1_PORT, STEPPER_MS1_PIN, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(STEPPER_MS2_PORT, STEPPER_MS2_PIN, GPIO_PIN_SET);
		break;

	case MICROSTEP_16:
		// MS1=1, MS2=1 → 16 µsteps, UART addr 0x03
		HAL_GPIO_WritePin(STEPPER_MS1_PORT, STEPPER_MS1_PIN, GPIO_PIN_SET);
		HAL_GPIO_WritePin(STEPPER_MS2_PORT, STEPPER_MS2_PIN, GPIO_PIN_SET);
		break;
	}

	// NOTE: PA5 (SPREAD) is not connected on TMC2209 — no SPREAD pin exists.
	// Chopper mode (SpreadCycle vs StealthChop) is set via UART GCONF register (Phase 2).
	// TMC2209 defaults to StealthChop without UART configuration.
}

/**
 * @brief Poll for deferred stop completion
 * @note Call this from main loop (MotorControl_Process) to complete deferred stop
 *       This implements the HYBRID DEFERRED STOP PATTERN to prevent spurious pulses
 * 
 * HYBRID PATTERN (prevents timing window for spurious pulses):
 * - Phase 1 (ISR): Immediately stops PWM OUTPUT via HAL_TIM_PWM_Stop()
 * - Phase 2 (Main Loop - THIS FUNCTION): Completes full nuclear timer shutdown
 * 
 * Why hybrid approach needed:
 * - Pure deferred stop (ISR sets flag only) left timing window where timer hardware
 *   continued generating PWM pulses until main loop processed flag (several ms)
 * - During this window, spurious pulses reached motor driver causing unwanted movement
 * - Solution: ISR stops PWM output immediately (safe, fast), main loop completes
 *   full shutdown sequence (timer base, interrupts, GPIO safety, etc.)
 */
void Stepper_Poll(void) {
	if (!initialized)
		return;

	// Fix 20: Main-loop step rate measurement (moved from ISR)
	// ISR only increments step_rate_counter; we do the time-windowed
	// measurement here using HAL_GetTick() safely from main-loop context.
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

		// PHASE 2 DIAGNOSTIC: Record timestamp when Phase 2 starts
		stepper.phase2_timestamp = HAL_GetTick();

		// Log Phase 1 → Phase 2 latency (verbose level - timing measurement)
		DBG_PRINT_V(STEPPER, "Phase1→Phase2 latency: %lu ms",
				stepper.phase2_timestamp - stepper.phase1_timestamp);

		// Complete full nuclear shutdown (PWM output already stopped by ISR)
		// This call still needed for: timer base stop, interrupt disable,
		// GPIO safety/verification, counter reset
		Stepper_StopPWM();

		// Complete state cleanup
		stepper.steps_remaining = 0;
		stepper.accel_counter = 0;
		stepper.step_counter = 0;

		// One-shot post-stop verification ("set once, verify once")
		// Catches any deferred-stop-sequence failure immediately
		Stepper_SuppressIdleInterrupts();
		Stepper_SetOutputProtection(true);
		Stepper_ResetIdleCounters(); //Fresh counters for this idle period
		Stepper_CheckIdleState();

		DBG_PRINT(STEPPER,
				"Hybrid deferred stop completed - full shutdown done");
	}
}

/**
 * @brief Get step rate diagnostics
 * @param measured Pointer to store measured steps per second
 * @param expected Pointer to store expected steps per second
 * @return true if valid measurement available
 */
bool Stepper_GetStepRate(uint32_t *measured, uint32_t *expected) {
	if (!measured || !expected)
		return false;

	*measured = measured_steps_per_sec;
	*expected = expected_steps_per_sec;

	// Return true if we have valid measurements (non-zero)
	return (measured_steps_per_sec > 0 || expected_steps_per_sec > 0);
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
				/ (STEPPER_STEPS_PER_REV * stepper.microstep);
		uint32_t expected_rpm = (expected_steps_per_sec * 60)
				/ (STEPPER_STEPS_PER_REV * stepper.microstep);

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
		DBG_PRINT(STEPPER, "  Microstep Mode: %d", stepper.microstep);
		// Analysis
		if (ratio_x100 >= 95 && ratio_x100 <= 105) {
			DBG_PRINT(STEPPER,
					"  ANALYSIS: Step rate matches expected - TIMER IS CORRECT");
			DBG_PRINT(STEPPER,
					"            If motor speed is still wrong, issue is MECHANICAL or MICROSTEPPING");
		} else if (ratio_x100 > 700 && ratio_x100 < 900) {
			DBG_PRINT(STEPPER,
					"  ANALYSIS: ~8x slower - LIKELY 1/8 MICROSTEPPING ACTIVE");
		} else if (ratio_x100 > 150 && ratio_x100 < 250) {
			DBG_PRINT(STEPPER,
					"  ANALYSIS: ~2x slower - LIKELY HALF STEPPING ACTIVE");
		} else if (ratio_x100 > 350 && ratio_x100 < 450) {
			DBG_PRINT(STEPPER,
					"  ANALYSIS: ~4x slower - LIKELY QUARTER STEPPING ACTIVE");
		} else {
			DBG_PRINT(STEPPER,
					"  ANALYSIS: Unexpected ratio - INVESTIGATE TIMER OR INTERRUPT");
		}
		DBG_PRINT(STEPPER, "=== END STEP RATE DIAGNOSTICS ===");
		DBG_PRINT(STEPPER, "");
	}
}

/**
 * @brief Reset step rate measurement counters
 * @note Call when starting a new measurement
 */
void Stepper_ResetStepRateMeasurement(void) {
	step_rate_counter = 0;
	step_rate_start_time = 0;
	measured_steps_per_sec = 0;
	expected_steps_per_sec = 0;
}

/**
 * @brief Check timer and GPIO configuration
	 * @note Entire body guarded by #if DEBUG_MASTER_ENABLE so the function
 */
void Stepper_DiagnosticCheck(void) {
#if DEBUG_MASTER_ENABLE
	DBG_BLOCK(STEPPER) {
		DBG_PRINT(STEPPER, "");
		DBG_PRINT(STEPPER, "=== STEPPER DIAGNOSTIC CHECK ===");

		// Check GPIO states
		DBG_PRINT(STEPPER, "GPIO States:");
		DBG_PRINT(STEPPER, "  ENABLE: %s (should be LOW when enabled)",
				HAL_GPIO_ReadPin(STEPPER_ENABLE_PORT, STEPPER_ENABLE_PIN) ? "HIGH" : "LOW");
		DBG_PRINT(STEPPER, "  DIR: %s",
				HAL_GPIO_ReadPin(STEPPER_DIR_PORT, STEPPER_DIR_PIN) ? "HIGH" : "LOW");
		DBG_PRINT(STEPPER, "  STEP: %s",
				HAL_GPIO_ReadPin(STEPPER_STEP_PORT, STEPPER_STEP_PIN) ? "HIGH" : "LOW");
		DBG_PRINT(STEPPER, "  MS1: %s",
				HAL_GPIO_ReadPin(STEPPER_MS1_PORT, STEPPER_MS1_PIN) ? "HIGH" : "LOW");
		DBG_PRINT(STEPPER, "  MS2: %s",
				HAL_GPIO_ReadPin(STEPPER_MS2_PORT, STEPPER_MS2_PIN) ? "HIGH" : "LOW");
		DBG_PRINT(STEPPER, "  SPREAD: %s",
				HAL_GPIO_ReadPin(STEPPER_SPREAD_PORT, STEPPER_SPREAD_PIN) ?
						"HIGH (SpreadCycle)" : "LOW (StealthChop)");

		// Check timer configuration
		DBG_PRINT(STEPPER, "");
		DBG_PRINT(STEPPER, "Timer Configuration:");
		DBG_PRINT(STEPPER, "  TIM1 CR1: 0x%04lX", TIM1->CR1);
		DBG_PRINT(STEPPER, "  TIM1 PSC: %lu", TIM1->PSC);
		DBG_PRINT(STEPPER, "  TIM1 ARR: %lu", TIM1->ARR);
		DBG_PRINT(STEPPER, "  TIM1 CCR1: %lu", TIM1->CCR1);
		DBG_PRINT(STEPPER, "  TIM1 CCMR1: 0x%04lX", TIM1->CCMR1);
		DBG_PRINT(STEPPER, "  TIM1 CCER: 0x%04lX", TIM1->CCER);
		DBG_PRINT(STEPPER, "  TIM1 CNT: %lu", TIM1->CNT);
		DBG_PRINT(STEPPER, "  TIM1 DIER: 0x%04lX", TIM1->DIER);

		// Check if timer is running
		if (TIM1->CR1 & TIM_CR1_CEN) {
			DBG_PRINT(STEPPER, "  Timer is RUNNING");
		} else {
			DBG_PRINT(STEPPER, "  Timer is STOPPED");
		}

		// Check if PWM output is enabled
		if (TIM1->CCER & TIM_CCER_CC1E) {
			DBG_PRINT(STEPPER, "  PWM Channel 1 is ENABLED");
		} else {
			DBG_PRINT(STEPPER, "  PWM Channel 1 is DISABLED");
		}

		// PWM control status
		DBG_PRINT(STEPPER, "");
		DBG_PRINT(STEPPER, "PWM Control Status:");
		DBG_PRINT(STEPPER, "  Duty Cycle: %lu%%",
				stepper.pwm_control.duty_percent);
		DBG_PRINT(STEPPER, "  ARR Value: %lu", stepper.pwm_control.arr_value);

		// Check GPIO mode for step pin
		DBG_PRINT(STEPPER, "");
		DBG_PRINT(STEPPER, "Step Pin Configuration:");
		uint32_t step_pin_pos = 0;
		uint32_t step_pin_mask = STEPPER_STEP_PIN;
		while (step_pin_mask >>= 1)
			step_pin_pos++;

		uint32_t moder = STEPPER_STEP_PORT->MODER;
		uint32_t mode = (moder >> (step_pin_pos * 2)) & 0x03;

		const char *mode_str;
		switch (mode) {
		case 0:
			mode_str = "INPUT";
			break;
		case 1:
			mode_str = "OUTPUT";
			break;
		case 2:
			mode_str = "ALTERNATE FUNCTION";
			break;
		case 3:
			mode_str = "ANALOG";
			break;
		default:
			mode_str = "UNKNOWN";
			break;
		}
		DBG_PRINT(STEPPER, "  GPIO Mode: %s", mode_str);

		if (mode == 2) {
			// Check alternate function
			uint32_t afr_reg = (step_pin_pos < 8) ?
			STEPPER_STEP_PORT->AFR[0] :
													STEPPER_STEP_PORT->AFR[1];
			uint32_t afr_pos =
					(step_pin_pos < 8) ? step_pin_pos : (step_pin_pos - 8);
			uint32_t af = (afr_reg >> (afr_pos * 4)) & 0x0F;
			DBG_PRINT(STEPPER, "  Alternate Function: AF%lu", af);
		}

		DBG_PRINT(STEPPER, "");
		DBG_PRINT(STEPPER, "Interrupt Status:");
		DBG_PRINT(STEPPER, "  TIM1_UP_TIM16_IRQn enabled: %s",
				NVIC_GetEnableIRQ(TIM1_UP_TIM16_IRQn) ? "YES" : "NO");
		DBG_PRINT(STEPPER, "  TIM1 Update interrupt enabled: %s",
				(TIM1->DIER & TIM_DIER_UIE) ? "YES" : "NO");
		DBG_PRINT(STEPPER, "=== END DIAGNOSTIC CHECK ===");
		DBG_PRINT(STEPPER, "");
	}
#endif /* DEBUG_MASTER_ENABLE */
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

/**
 * @brief Check if TIM1 Update Interrupt is enabled (diagnostic)
 * @retval true if UIE bit is set in TIM1->DIER
 */
bool Stepper_IsTimerInterruptEnabled(void) {
	return (TIM1->DIER & 0x0001) != 0;
}

uint32_t Stepper_GetTimerARR(void) {
	return TIM1->ARR;
}

uint32_t Stepper_GetTimerPSC(void) {
	return TIM1->PSC;
}

/**
 * @brief Read-and-clear pending stepper event flags (CT-03)
 * @retval Bitmask of STEPPER_EVT_* flags that were pending
 * @note  Uses __disable_irq()/__enable_irq() for atomic read-and-clear
 *        to prevent race with TIM1 ISR writing the bitfield.
 */
uint32_t Stepper_GetPendingEvents(void) {
	__disable_irq();
	uint32_t events = stepper_pending_events;
	stepper_pending_events = 0;
	__enable_irq();
	return events;
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
