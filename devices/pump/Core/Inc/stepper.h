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
#define STEPPER_EVT_DRV_FAULT (1U << 3)  // TMC2209 DIAG rose (driver error; confirm by level)

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
 *        CURRENT CONFIG (NEMA 17 direct drive): MICROSTEP_8 via CHOPCONF.mres=5.
 *        With MSTEP_REG_SELECT=1 in GCONF, MS pins become UART address only.
 *        MS1=0, MS2=0 kept for UART slave address 0x00 — which is also the
 *        pin-mode encoding for 1/8, so if the UART config is ever lost the chip
 *        falls back to the SAME resolution and the step math still holds.
 *        INTPOL=1: MicroPlyer interpolates 1/8 → 256 internally.
 */
typedef enum {
	MICROSTEP_1 = 1,     // UART-configured: CHOPCONF.mres=8 full step
	MICROSTEP_2 = 2,        // UART-configured: CHOPCONF.mres=7
	MICROSTEP_8 = 8,        // CHOPCONF.mres=5; also MS1=0/MS2=0 in pin mode
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
	bool enabled;
	bool error;
} Stepper_Status_t;

/* Exported constants --------------------------------------------------------*/
/* 17HS19-2004S1 NEMA 17, direct drive (no gearbox): 1.8°/step = 200 full steps
 * per pump revolution. Always used as STEPPER_STEPS_PER_REV × microstep, so with
 * MICROSTEP_8 the product is 200 × 8 = 1600 pulses/rev — numerically identical to
 * the WPX-1's 200 × 8 gearbox. Every ARR/ramp constant below keeps its meaning. */
#define STEPPER_STEPS_PER_REV       200
#define STEPPER_MAX_SPEED_RPM       120    // Maximum safe RPM — carried over from the
                                           // WPX-1 (tested 12/30/2025); NOT yet
                                           // re-established for the NEMA 17 (Phase 5)

/* PWM step pulse width */
#define PWM_DUTY_DEFAULT            8    // Duty cycle % (Fix 20: reverted to 8% — old working code proved 8% sufficient; 50% created excess EMI/noise on custom PCB)
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
 * @brief Set motor speed directly in RPM
 * @param rpm Speed in revolutions per minute (clamped to STEPPER_MAX_SPEED_RPM)
 */
void Stepper_SetSpeedRPM(uint16_t rpm);

/**
 * @brief Get current motor speed in RPM
 * @retval Current speed in RPM
 */
uint16_t Stepper_GetSpeedRPM(void);

/**
 * @brief Start continuous movement (jog mode)
 */
void Stepper_StartJog(void);

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
 * @brief Signal a TMC2209 driver fault (DIAG pin rose)
 * @note  ISR-safe: only sets STEPPER_EVT_DRV_FAULT in the pending-event
 *        bitmask. Called from the PA5 EXTI callback, and from main() when
 *        DIAG is found already latched high at monitor-arm time (the edge
 *        predates the EXTI, so the interrupt alone would never fire).
 *        MotorControl_Process() confirms by level before entering FAULT.
 */
void Stepper_NotifyDriverFault(void);

/**
 * @brief Print step rate diagnostics to UART
 * @note Call periodically when motor is running to see measured vs expected rate
 */
void Stepper_PrintStepRateDiagnostics(void);

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

/* Timer definition ----------------------------------------------------------*/
#ifndef STEPPER_TIMER
#define STEPPER_TIMER           htim1
extern TIM_HandleTypeDef htim1;
#endif

#ifdef __cplusplus
}
#endif

#endif /* STEPPER_H */
