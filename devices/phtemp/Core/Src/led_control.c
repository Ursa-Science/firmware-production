/**
 ******************************************************************************
 * @file    led_control.c
 * @brief   LED Control via CANopen (0x2000) - Timer-based PWM for Tri-LED
 * @author  CANopen Motor Control Project
 * @date    2026
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "led_control.h"
#include "procimg_api.h"
#include "mcohw_LEDs.h"   // MCOTGT_SetLEDOverride() — LED ownership handoff with the MCO stack

/* Private defines -----------------------------------------------------------*/
#define BLINK_PERIOD_MS         300   // Explicit attention blink (300ms = 150ms ON + 150ms OFF, ~3Hz)
#define CAN_ACTIVITY_PULSE_MS   50    // Duration of CAN activity pulse

/* Private typedef -----------------------------------------------------------*/

/** @brief Single LED timer assignment (handle + channel) */
typedef struct {
	TIM_HandleTypeDef *htim;
	uint32_t channel;
} LEDTimer_t;

/** @brief All three LED timer assignments (injected via Init) */
typedef struct {
	LEDTimer_t red;
	LEDTimer_t blue;
	LEDTimer_t green;
} LEDTimers_t;

typedef struct {
	bool initialized;
	LED_Mode_t current_mode;
	uint32_t blink_timer;
	bool blink_state;
	uint32_t can_activity_timer;
	bool can_activity_pending;
} LEDControl_t;

/* Private variables ---------------------------------------------------------*/
static LEDControl_t led_ctrl = { 0 };
static LEDTimers_t led_timers = { 0 };

/* Private function prototypes -----------------------------------------------*/
static void LEDControl_SetRed(bool state);
static void LEDControl_SetGreen(bool state);
static void LEDControl_SetBlue(bool state);
static void LEDControl_ProcessNormalMode(void);
static void LEDControl_ProcessBlinkMode(void);

/* Public functions ----------------------------------------------------------*/

/**
 * @brief Initialize LED control with injected PWM timer handles
 */
bool LEDControl_Init(TIM_HandleTypeDef *red_htim, uint32_t red_ch,
		TIM_HandleTypeDef *blue_htim, uint32_t blue_ch,
		TIM_HandleTypeDef *green_htim, uint32_t green_ch) {
	TIM_OC_InitTypeDef sConfigOC = { 0 };

	// Store injected timer handles
	led_timers.red.htim = red_htim;
	led_timers.red.channel = red_ch;
	led_timers.blue.htim = blue_htim;
	led_timers.blue.channel = blue_ch;
	led_timers.green.htim = green_htim;
	led_timers.green.channel = green_ch;

	// Stop any running PWM on LED channels
	HAL_TIM_PWM_Stop(led_timers.red.htim, led_timers.red.channel);
	HAL_TIM_PWM_Stop(led_timers.blue.htim, led_timers.blue.channel);
	HAL_TIM_PWM_Stop(led_timers.green.htim, led_timers.green.channel);

	// Configure PWM mode for all three LED timers
	// NOTE: LEDs are common anode - inverted logic (0=ON, Period=OFF)
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	sConfigOC.Pulse = 0;

	// Configure and start Red LED
	if (HAL_TIM_PWM_ConfigChannel(led_timers.red.htim, &sConfigOC,
			led_timers.red.channel) != HAL_OK
			|| HAL_TIM_PWM_Start(led_timers.red.htim, led_timers.red.channel)
					!= HAL_OK) {
		return false;
	}
	// Set CCR to Period for OFF state (common anode)
	__HAL_TIM_SET_COMPARE(led_timers.red.htim, led_timers.red.channel,
			led_timers.red.htim->Init.Period);

	// Configure and start Blue LED (always OFF)
	if (HAL_TIM_PWM_ConfigChannel(led_timers.blue.htim, &sConfigOC,
			led_timers.blue.channel) != HAL_OK
			|| HAL_TIM_PWM_Start(led_timers.blue.htim, led_timers.blue.channel)
					!= HAL_OK) {
		return false;
	}
	// Set CCR to Period for OFF state (common anode)
	__HAL_TIM_SET_COMPARE(led_timers.blue.htim, led_timers.blue.channel,
			led_timers.blue.htim->Init.Period);

	// Configure and start Green LED (always OFF)
	if (HAL_TIM_PWM_ConfigChannel(led_timers.green.htim, &sConfigOC,
			led_timers.green.channel) != HAL_OK
			|| HAL_TIM_PWM_Start(led_timers.green.htim,
					led_timers.green.channel) != HAL_OK) {
		return false;
	}
	// Set CCR to Period for OFF state (common anode)
	__HAL_TIM_SET_COMPARE(led_timers.green.htim, led_timers.green.channel,
			led_timers.green.htim->Init.Period);

	// Initialize control state
	led_ctrl.initialized = true;
	led_ctrl.current_mode = LED_MODE_NORMAL;
	led_ctrl.blink_timer = HAL_GetTick();
	led_ctrl.blink_state = false;
	led_ctrl.can_activity_timer = 0;
	led_ctrl.can_activity_pending = false;

	return true;
}

/**
 * @brief Process LED control (call from main loop or motor control)
 */
void LEDControl_Process(void) {
	if (!led_ctrl.initialized)
		return;

	// Read LED mode from process image (0x2000)
	uint8_t led_mode = ProcImg_GetLEDControl();

	// Clamp to valid range
	if (led_mode > LED_MODE_OFF) {
		led_mode = LED_MODE_OFF;
	}

	led_ctrl.current_mode = (LED_Mode_t) led_mode;

	// Claim/release LED ownership from the MCO stack. In any override mode the app
	// drives all three channels below; in Normal mode the stack regains the RUN/ERR LEDs.
	MCOTGT_SetLEDOverride(led_ctrl.current_mode != LED_MODE_NORMAL);

	// Process based on current mode.
	// NOTE: in any override mode (BLINK/ON/OFF) the app takes full ownership of all
	// three LED channels and forces Green+Blue OFF so the RED indication is isolated and
	// cannot be masked by the MCO stack's RUN-green. The MCOTGT_SetLEDOverride() call above
	// suppresses the stack's RUN/ERR writes (gLEDAppOverride token in mcohw_LEDs.h), so there
	// is no contention on the shared LED pins.
	switch (led_ctrl.current_mode) {
	case LED_MODE_NORMAL:
		// Stack owns RUN/ERR LEDs in Normal mode; app only pulses red on CAN activity.
		LEDControl_ProcessNormalMode();
		break;

	case LED_MODE_BLINK:
		LEDControl_ProcessBlinkMode(); // forces Green+Blue OFF, flashes Red
		break;

	case LED_MODE_ON:
		LEDControl_SetGreen(false);
		LEDControl_SetBlue(false);
		LEDControl_SetRed(true);
		break;

	case LED_MODE_OFF:
	default:
		LEDControl_SetGreen(false);
		LEDControl_SetBlue(false);
		LEDControl_SetRed(false);
		break;
	}
}

/**
 * @brief Set LED mode directly (bypasses process image)
 */
void LEDControl_SetMode(LED_Mode_t mode) {
	if (!led_ctrl.initialized)
		return;

	led_ctrl.current_mode = mode;

	// Update process image
	ProcImg_SetLEDControl((uint8_t) mode);
}

/**
 * @brief Indicate CAN activity (for Normal mode)
 */
void LEDControl_CANActivity(void) {
	if (!led_ctrl.initialized)
		return;

	led_ctrl.can_activity_pending = true;
	led_ctrl.can_activity_timer = HAL_GetTick();
}

/* Private functions ---------------------------------------------------------*/

/**
 * @brief Set Red LED state via PWM timer
 * @param state true = ON, false = OFF
 * @note Common-anode: CCR=0 is ON, CCR=Period is OFF
 */
static void LEDControl_SetRed(bool state) {
	if (state) {
		// RED ON: CCR = 0 (common anode inverted)
		__HAL_TIM_SET_COMPARE(led_timers.red.htim, led_timers.red.channel, 0);
	} else {
		// RED OFF: CCR = Period (common anode inverted)
		__HAL_TIM_SET_COMPARE(led_timers.red.htim, led_timers.red.channel,
				led_timers.red.htim->Init.Period);
	}
	led_timers.red.htim->Instance->EGR |= TIM_EGR_UG; // Force update event for immediate change
}

/**
 * @brief Set Green LED state via PWM timer
 * @param state true = ON, false = OFF
 * @note Common-anode: CCR=0 is ON, CCR=Period is OFF. Used to force Green OFF in
 *       override modes so it cannot mask the red indication.
 */
static void LEDControl_SetGreen(bool state) {
	if (state) {
		__HAL_TIM_SET_COMPARE(led_timers.green.htim, led_timers.green.channel,
				0);
	} else {
		__HAL_TIM_SET_COMPARE(led_timers.green.htim, led_timers.green.channel,
				led_timers.green.htim->Init.Period);
	}
	led_timers.green.htim->Instance->EGR |= TIM_EGR_UG; // Force update event for immediate change
}

/**
 * @brief Set Blue LED state via PWM timer
 * @param state true = ON, false = OFF
 * @note Common-anode: CCR=0 is ON, CCR=Period is OFF.
 */
static void LEDControl_SetBlue(bool state) {
	if (state) {
		__HAL_TIM_SET_COMPARE(led_timers.blue.htim, led_timers.blue.channel, 0);
	} else {
		__HAL_TIM_SET_COMPARE(led_timers.blue.htim, led_timers.blue.channel,
				led_timers.blue.htim->Init.Period);
	}
	led_timers.blue.htim->Instance->EGR |= TIM_EGR_UG; // Force update event for immediate change
}

/**
 * @brief Process Normal mode (CAN activity indicator)
 */
static void LEDControl_ProcessNormalMode(void) {
	if (led_ctrl.can_activity_pending) {
		// LED ON during CAN activity pulse
		LEDControl_SetRed(true);

		// Check if pulse duration has elapsed
		uint32_t elapsed = HAL_GetTick() - led_ctrl.can_activity_timer;
		if (elapsed >= CAN_ACTIVITY_PULSE_MS) {
			led_ctrl.can_activity_pending = false;
			LEDControl_SetRed(false);
		}
	} else {
		// LED OFF when no CAN activity
		LEDControl_SetRed(false);
	}
}

/**
 * @brief Process Blink mode
 */
static void LEDControl_ProcessBlinkMode(void) {
	// Hold Green+Blue OFF every pass so the red flash is isolated (not masked by the
	// stack's RUN-green). Cheap CCR writes; the stack's LED tick is gated off in this mode.
	LEDControl_SetGreen(false);
	LEDControl_SetBlue(false);

	uint32_t current_time = HAL_GetTick();
	uint32_t elapsed = current_time - led_ctrl.blink_timer;

	// Toggle Red every half period (BLINK_PERIOD_MS total = half ON + half OFF)
	if (elapsed >= (BLINK_PERIOD_MS / 2)) {
		led_ctrl.blink_state = !led_ctrl.blink_state;
		LEDControl_SetRed(led_ctrl.blink_state);
		led_ctrl.blink_timer = current_time;
	}
}
