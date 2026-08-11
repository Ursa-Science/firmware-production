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

/* Microstep resolution is a SINGLE compile-time knob: TMC_MICROSTEP in
 * tmc2209_uart.h. It sets CHOPCONF.MRES at init and feeds every step/ARR
 * calculation here plus the dosing math in motor_control.c. MS1/MS2 are the
 * TMC UART slave address pins (held 0/0 = addr 0x00) and never select
 * resolution on this board. */

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
	uint16_t microstep;          // µsteps per full step (TMC_MICROSTEP)
	int32_t current_position;
	bool enabled;
	bool error;
} Stepper_Status_t;

/* Exported constants --------------------------------------------------------*/
/* 17HS19-2004S1 NEMA 17, direct drive (no gearbox): 1.8°/step = 200 full steps
 * per pump revolution. Always used as STEPPER_STEPS_PER_REV × TMC_MICROSTEP, so
 * at the default 1/8 the product is 200 × 8 = 1600 pulses/rev — numerically
 * identical to the WPX-1's 200 × 8 gearbox. Every ARR/ramp constant below keeps
 * its meaning at that default. */
#define STEPPER_STEPS_PER_REV       200
#define STEPPER_MAX_SPEED_RPM       120    // Maximum safe RPM — carried over from the
                                           // WPX-1 (tested 12/30/2025); NOT yet
                                           // re-established for the NEMA 17

/* PWM step pulse width */
#define PWM_DUTY_DEFAULT            8    // Step pulse duty cycle %; 8% proven sufficient — higher duty caused EMI/noise on the custom PCB
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
 * @brief Read-and-clear pending stepper event flags
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
 * @note Keeps all TIM1 register access within the stepper module
 */
uint32_t Stepper_GetTimerARR(void);

/**
 * @brief Get current TIM1 Prescaler Register value
 * @retval Current PSC value
 * @note Keeps all TIM1 register access within the stepper module
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
