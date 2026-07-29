/**
 ******************************************************************************
 * @file    stepper.h
 * @brief   TMC2209 Stepper Motor Driver Interface
 * @author  CANopen Motor Control Project
 * @date    2025
 ******************************************************************************
 */

#ifndef STEPPER_H
#define STEPPER_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include <stdbool.h>
#include "main.h"

/* Exported types ------------------------------------------------------------*/

/**
 * @brief Stepper Motor State (internal ramp state)
 * @note  This enum is retained for ISR ramp logic and diagnostic
 *        reporting via Stepper_GetStatus().
 */
typedef enum {
	STEPPER_IDLE = 0, STEPPER_RUNNING, STEPPER_DECELERATING,
	STEPPER_ERROR
} Stepper_State_t;

/* Stepper event bitmask defines
 * ISR sets bits in a volatile bitfield; main loop reads-and-clears via
 * Stepper_GetPendingEvents().  No function pointer call from ISR context. */
#define STEPPER_EVT_STOPPED   (1U << 0)  // Deceleration/stop complete, motor is idle
#define STEPPER_EVT_AT_SPEED  (1U << 1)  // Acceleration reached target speed
#define STEPPER_EVT_ERROR     (1U << 2)  // Hardware fault detected (e.g. position overflow)

/**
 * @brief Stepper Motor Direction
 */
typedef enum {
	STEPPER_DIR_CW = 0, STEPPER_DIR_CCW = 1
} Stepper_Direction_t;

/**
 * @brief Microstep Resolution
 * @note  TMC2209 MS1/MS2 pin mapping (standalone, no UART):
 *          MS1=0, MS2=0 →  8 µsteps  (UART addr 0x00)
 *          MS1=1, MS2=0 → 32 µsteps  (UART addr 0x01)
 *          MS1=0, MS2=1 → 64 µsteps  (UART addr 0x02)
 *          MS1=1, MS2=1 → 16 µsteps  (UART addr 0x03)
 *        MicroPlyer internally interpolates to 256 regardless.
 *
 *        Phase 2 UART: MICROSTEP_1 (full step) via CHOPCONF.mres=8.
 *        With MSTEP_REG_SELECT=1 in GCONF, MS pins become UART address only.
 *        MS1=0, MS2=0 kept for UART slave address 0x00.
 *        INTPOL=1: MicroPlyer interpolates full-step → 256 internally.
 */
typedef enum {
	MICROSTEP_1 = 1,     // UART-configured: CHOPCONF.mres=8 full step (Phase 2)
	MICROSTEP_2 = 2,        // UART-configured: CHOPCONF.mres=7
	MICROSTEP_8 = 8,        // MS1=0, MS2=0 — TMC2209 default, lowest via pins
	MICROSTEP_16 = 16,       // MS1=1, MS2=1
	MICROSTEP_32 = 32,       // MS1=1, MS2=0
	MICROSTEP_64 = 64        // MS1=0, MS2=1
} Stepper_Microstep_t;


/**
 * @brief PWM Control Parameters
 * @note  TMC2209 motor current is set via IRUN/IHOLD UART registers.
 *        PWM duty cycle controls step pulse width, NOT motor current.
 */
typedef struct {
	uint32_t arr_value;          // Timer auto-reload value
	uint32_t duty_percent;       // Duty cycle percentage (0-100)
} PWM_Control_t;

/**
 * @brief Stepper Motor Status
 */
typedef struct {
	Stepper_State_t state;
	Stepper_Direction_t direction;
	Stepper_Microstep_t microstep;
	int32_t current_position;
	uint32_t target_steps;
	bool enabled;
	bool error;
} Stepper_Status_t;

/* Exported constants --------------------------------------------------------*/
#define STEPPER_STEPS_PER_REV       1600  // WPX-1 with 1:8 gear ratio (200 motor steps × 8)
#define STEPPER_MAX_SPEED_RPM       120    // Maximum safe RPM (tested 12/30/2025)
#define STEPPER_TIMEOUT_MS          5000  // 5 second timeout (was 30s)

/* PWM and Current Control Constants */
#define PWM_DUTY_MIN                4    // Minimum duty cycle %
#define PWM_DUTY_MAX                50   // Maximum duty cycle %
#define PWM_DUTY_DEFAULT            8    // Default duty cycle % (Fix 20: reverted to 8% — old working code proved 8% sufficient; 50% created excess EMI/noise on custom PCB)
/* Exported functions --------------------------------------------------------*/
/**
 * @brief Initialize stepper motor driver
 * @retval true if successful
 */
bool Stepper_Init(void);

/**
 * @brief Enable motor driver
 */
void Stepper_Enable(void);

/**
 * @brief Disable motor driver
 */
void Stepper_Disable(void);

/**
 * @brief Emergency stop - immediate halt
 */
void Stepper_Stop(void);

/**
 * @brief Normal stop - graceful deceleration
 * @note Decelerates motor to stop rather than instant halt
 */
void Stepper_NormalStop(void);

/**
 * @brief Poll for deferred stop completion
 * @note Call from main loop to execute ISR-deferred stop operations
 */
void Stepper_Poll(void);

/**
 * @brief Set motor direction
 * @param direction CW or CCW
 */
void Stepper_SetDirection(Stepper_Direction_t direction);

/**
 * @brief Set microstep resolution
 * @param microstep Resolution (1, 2, 8, 16, 32, or 64) — TMC2209 supported values
 * @retval true if valid setting
 */
bool Stepper_SetMicrostep(Stepper_Microstep_t microstep);


/**
 * @brief Set motor speed directly in RPM
 * @param rpm Speed in revolutions per minute (0-1200)
 */
void Stepper_SetSpeedRPM(uint16_t rpm);

/**
 * @brief Get current motor speed in RPM
 * @retval Current speed in RPM
 */
uint16_t Stepper_GetSpeedRPM(void);

/**
 * @brief Run specified number of steps
 * @param steps Number of steps to run
 * @retval true if command accepted
 */
bool Stepper_RunSteps(uint32_t steps);

/**
 * @brief Move to absolute position
 * @param position Target position in steps
 * @retval true if command accepted
 */
bool Stepper_MoveToPosition(int32_t position);

/**
 * @brief Start continuous movement (jog mode)
 */
void Stepper_StartJog(void);

/**
 * @brief Reset position counter to zero
 */
void Stepper_ResetPosition(void);

/**
 * @brief Get current position
 * @retval Current position in steps
 */
int32_t Stepper_GetPosition(void);

/**
 * @brief Read-and-clear pending stepper event flags (CT-03)
 * @retval Bitmask of STEPPER_EVT_* flags that were pending
 * @note  Atomically reads and clears the ISR-written bitfield using
 *        __disable_irq()/__enable_irq() to prevent read-modify-write races.
 *        Call from main loop (MotorControl_Process) each cycle.
 */
uint32_t Stepper_GetPendingEvents(void);

/**
 * @brief Get stepper status
 * @param status Pointer to status structure to fill
 */
void Stepper_GetStatus(Stepper_Status_t *status);

/**
 * @brief Check if motor is moving
 * @retval true if motor is moving
 */
bool Stepper_IsMoving(void);

/**
 * @brief Clear error condition
 */
void Stepper_ClearError(void);

/**
 * @brief Timer interrupt callback for step generation
 * @note Called from TIM1 interrupt handler
 */
void Stepper_ProcessTimerUpdate(void);

/**
 * @brief Set PWM duty cycle
 * @param duty_percent Duty cycle percentage (0-100)
 */
void Stepper_SetDutyCycle(uint32_t duty_percent);

/**
 * @brief Get current PWM control parameters
 * @param control Pointer to PWM control structure to fill
 */
void Stepper_GetPWMControl(PWM_Control_t *control);

void Stepper_DiagnosticCheck(void);

/**
 * @brief Post-shutdown diagnostic for Stepper_StopPWM()
 * @note Output gated by DBG_BLOCK(TIMER). Call after Stepper_Stop() to
 *       verify TIM1 registers reached safe state. Always compiled;
 *       prints nothing when DBG_TIMER_ENABLE == 0.
 */
void Stepper_StopPWM_Diagnostic(void);

/**
 * @brief Get step rate measurements for diagnostics
 * @param measured Pointer to store measured steps per second
 * @return true if valid measurement available
 */
bool Stepper_GetStepRate(uint32_t *measured, uint32_t *expected);

/**
 * @brief Print step rate diagnostics to UART
 * @note Call periodically when motor is running to see measured vs expected rate
 */
void Stepper_PrintStepRateDiagnostics(void);

/**
 * @brief Reset step rate measurement counters
 * @note Call when starting a new measurement
 */
void Stepper_ResetStepRateMeasurement(void);

/**
 * @brief Check IDLE state integrity (diagnostic function)
 * @note Call periodically from main loop when motor should be stopped
 * @return true if IDLE state is valid, false if anomaly detected
 */
bool Stepper_CheckIdleState(void);

/**
 * @brief Suppress unauthorized TIM1 Update Interrupt during IDLE
 * @note Detects and disables UIE if enabled when motor is stopped.
 *       Replaces direct TIM1->DIER access from motor_control.c.
 */
void Stepper_SuppressIdleInterrupts(void);

/**
 * @brief Configure TIM1 output protection for IDLE or RUNNING mode
 * @param idle true = IDLE mode (disable AOE+MOE, set OSSI for safe outputs)
 *             false = RUNNING mode (enable AOE for active PWM generation)
 * @note Replaces direct TIM1->BDTR access from motor_control.c.
 */
void Stepper_SetOutputProtection(bool idle);

/**
 * @brief Check if TIM1 Update Interrupt is enabled (diagnostic)
 * @retval true if UIE bit is set in TIM1->DIER
 */
bool Stepper_IsTimerInterruptEnabled(void);

/**
 * @brief Get current TIM1 Auto-Reload Register value
 * @retval Current ARR value (step period in timer ticks)
 * @note TC-01 fix: Keeps all TIM1 register access within stepper module
 */
uint32_t Stepper_GetTimerARR(void);

/**
 * @brief Get current TIM1 Prescaler Register value
 * @retval Current PSC value
 * @note TC-01 fix: Keeps all TIM1 register access within stepper module
 */
uint32_t Stepper_GetTimerPSC(void);

/* GPIO pin definitions ------------------------------------------------------*/
#ifndef STEPPER_ENABLE_PIN
#define STEPPER_ENABLE_PIN      ENABLE_Pin
#define STEPPER_ENABLE_PORT     GPIOA
#endif

#ifndef STEPPER_DIR_PIN
#define STEPPER_DIR_PIN         DIR_Pin
#define STEPPER_DIR_PORT        GPIOA
#endif

#ifndef STEPPER_STEP_PIN
#define STEPPER_STEP_PIN        STEP_Pin
#define STEPPER_STEP_PORT       GPIOA
#endif

#ifndef STEPPER_MS1_PIN
#define STEPPER_MS1_PIN         MS1_Pin
#define STEPPER_MS1_PORT        GPIOA
#endif

#ifndef STEPPER_MS2_PIN
#define STEPPER_MS2_PIN         MS2_Pin
#define STEPPER_MS2_PORT        GPIOA
#endif

#ifndef STEPPER_SPREAD_PIN
#define STEPPER_SPREAD_PIN      SPREAD_Pin     // TMC2209: no physical SPREAD pin — PA5 unconnected
#define STEPPER_SPREAD_PORT     SPREAD_GPIO_Port
#endif
/* Timer definition ----------------------------------------------------------*/
#ifndef STEPPER_TIMER
#define STEPPER_TIMER           htim1
extern TIM_HandleTypeDef htim1;
#endif

#ifdef __cplusplus
}
#endif

#endif /* STEPPER_H */
