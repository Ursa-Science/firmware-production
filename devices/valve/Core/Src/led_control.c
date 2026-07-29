/**************************************************************************
 MODULE:    LED_CONTROL
 CONTAINS:  Valve status + operator-override LED control

 Normal mode: drives the Blue channel (TIM3_CH2 / PA4) for valve-specific
 status; Red (TIM2) and Green (TIM4) are owned by the MCO stack for CiA 303-3
 NMT-state (RUN/ERR) indication.

 OD-override mode (0x2000 = On/Off/Blink): the app takes ownership of all three
 channels via MCOTGT_SetLEDOverride(), forces Green+Blue OFF and drives RED
 (solid or ~3 Hz blink) — the fleet-consistent operator indication shared with
 the pump/pH-temp modules.

 Common-anode active-low: CCR = 0 → LED ON, CCR = Period → LED OFF.
 ***************************************************************************/

#include "led_control.h"
#include "valve_control.h"
#include "procimg_api.h"
#include "log.h"
#include "main.h"
#include "mcohw_LEDs.h"   // MCOTGT_SetLEDOverride() — LED ownership handoff with the MCO stack

/**************************************************************************
 LOCAL DEFINES
 ***************************************************************************/
/* Blink half-periods in ms */
#define BLINK_SLOW_MS     1000   /* 0.5 Hz — valve in motion (Normal mode)            */
#define BLINK_OVERRIDE_MS  150   /* ~3 Hz  — OD override blink (RED; matches pump/pHTemp) */
#define BLINK_FAST_MS      250   /* 2 Hz   — fault (Normal mode)                       */

/**************************************************************************
 EXTERN TIMER HANDLES (defined in main.c, CubeMX-generated)
 ***************************************************************************/
extern TIM_HandleTypeDef htim2; /* Red   — MCO stack owns after init */
extern TIM_HandleTypeDef htim3; /* Blue  — we own                    */
extern TIM_HandleTypeDef htim4; /* Green — MCO stack owns after init */

/**************************************************************************
 LOCAL FUNCTION PROTOTYPES
 ***************************************************************************/
static void LED_SetBlue(uint8_t on);
static void LED_SetRed(uint8_t on);
static void LED_SetGreen(uint8_t on);
static uint8_t LED_BlinkPhase(uint32_t now, uint32_t half_period_ms);

/**************************************************************************
 GLOBAL FUNCTIONS
 ***************************************************************************/

void LEDControl_Init(void) {
	/*
	 * Start all three PWM channels.  TIM2 and TIM4 are needed so the
	 * MCO stack macros (LED_RUN_ON/OFF, LED_ERR_ON/OFF) can set CCR.
	 * We set them to OFF (CCR = Period, active-low) and never touch
	 * them again — the stack takes over from here.
	 */
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1); /* Red   on PA0 */
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2); /* Blue  on PA4 */
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1); /* Green on PB6 */

	/* Red & green OFF — MCO stack will set its own pattern shortly */
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, htim2.Init.Period);
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, htim4.Init.Period);

	/* Blue OFF */
	LED_SetBlue(0);

	DBG_STATE(LED, "Init complete, blue off, red/green → MCO stack");
}

void LEDControl_Process(void) {
	uint8_t mode = ProcImg_GetLEDControl();
	uint32_t now = HAL_GetTick();

	/*
	 * Claim/release LED ownership from the MCO stack. In any override mode
	 * (ON/OFF/BLINK) the app owns all three channels and forces Green+Blue OFF
	 * so the RED operator indication is isolated and cannot be co-lit / masked
	 * by the stack's RUN-green or ERR-red on the shared common-anode emitter. The
	 * flag suppresses the stack's RUN/ERR writes (gLEDAppOverride in mcohw_LEDs.h).
	 * RED is used here for fleet consistency with the pump/pH-temp modules. In
	 * Normal mode the stack regains the Red/Green RUN/ERR LEDs and the app drives
	 * Blue per valve state (unchanged — valve-specific status).
	 */
	MCOTGT_SetLEDOverride(mode != LED_MODE_NORMAL);

	switch (mode) {
	case LED_MODE_ON:
		LED_SetGreen(0);
		LED_SetBlue(0);
		LED_SetRed(1); /* solid RED */
		break;

	case LED_MODE_OFF:
		LED_SetGreen(0);
		LED_SetBlue(0);
		LED_SetRed(0); /* all off */
		break;

	case LED_MODE_BLINK: {
		LED_SetGreen(0);
		LED_SetBlue(0);
		uint8_t phase = LED_BlinkPhase(now, BLINK_OVERRIDE_MS);
		LED_SetRed(phase); /* RED flash ~3 Hz */
		break;
	}

	case LED_MODE_NORMAL:
	default: {
		/* Map blue LED to valve state */
		ValveState_FSM_t state = ValveControl_GetState();
		uint8_t position = ValveControl_GetPosition();

		switch (state) {
		case VALVE_STATE_DISABLED:
			LED_SetBlue(0);
			break;

		case VALVE_STATE_IDLE:
			if (position == VALVE_POS_OPEN) {
				LED_SetBlue(1); /* Solid blue = valve open */
			} else {
				LED_SetBlue(0); /* Off = valve closed / unknown */
			}
			break;

		case VALVE_STATE_OPENING:
		case VALVE_STATE_CLOSING: {
			uint8_t phase = LED_BlinkPhase(now, BLINK_SLOW_MS);
			LED_SetBlue(phase); /* Slow blink = in motion */
			break;
		}

		case VALVE_STATE_FAULT: {
			uint8_t phase = LED_BlinkPhase(now, BLINK_FAST_MS);
			LED_SetBlue(phase); /* Fast blink = fault */
			break;
		}
		}
		break;
	}
	}
}

/**************************************************************************
 LOCAL FUNCTIONS
 ***************************************************************************/

/**
 * @brief  Set the blue LED (TIM3_CH2 / PA4).
 *         Active-low: on=1 → CCR=0 (sink current), on=0 → CCR=Period.
 */
static void LED_SetBlue(uint8_t on) {
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, on ? 0 : htim3.Init.Period);
}

/**
 * @brief  Set the red LED (TIM2_CH1 / PA0).  Normally the MCO stack's ERR LED;
 *         driven here in OD-override modes as the operator indication (solid or
 *         blinking RED), for fleet consistency with the pump/pH-temp modules.
 *         Active-low: on=1 → CCR=0, on=0 → CCR=Period.
 */
static void LED_SetRed(uint8_t on) {
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, on ? 0 : htim2.Init.Period);
	htim2.Instance->EGR |= TIM_EGR_UG; /* match the stack's macro: immediate update */
}

/**
 * @brief  Set the green LED (TIM4_CH1 / PB6).  Normally the MCO stack's RUN LED;
 *         driven here only in OD-override modes to force it OFF so the RED
 *         operator indication is isolated.
 *         Active-low: on=1 → CCR=0, on=0 → CCR=Period.
 */
static void LED_SetGreen(uint8_t on) {
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, on ? 0 : htim4.Init.Period);
	htim4.Instance->EGR |= TIM_EGR_UG; /* match the stack's macro: immediate update */
}

/**
 * @brief  Return 0 or 1 blink phase based on elapsed time and half-period.
 */
static uint8_t LED_BlinkPhase(uint32_t now, uint32_t half_period_ms) {
	return ((now / half_period_ms) & 1) ? 1 : 0;
}

/**************************************************************************
 END-OF-FILE
 ***************************************************************************/
