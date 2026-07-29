/**************************************************************************
 MODULE:    VALVE_DRIVER
 CONTAINS:  Low-level relay GPIO control implementation
 PA10 (Relay_Signal) — push-pull, active HIGH
 HIGH = relay energized = valve open
 LOW  = relay de-energized = valve closed
 ***************************************************************************/

#include "valve_driver.h"
#include "main.h"
#include "log.h"

/**************************************************************************
 LOCAL VARIABLES
 ***************************************************************************/
static bool relay_state; /* Current relay electrical state */

/**************************************************************************
 GLOBAL FUNCTIONS
 ***************************************************************************/

void ValveDriver_Init(void) {
	/* GPIO already configured by CubeMX (MX_GPIO_Init).
	 * CubeMX sets initial output level to GPIO_PIN_RESET (LOW = closed).
	 * Just track our state. */
	relay_state = false;
	HAL_GPIO_WritePin(Relay_Signal_GPIO_Port, Relay_Signal_Pin, GPIO_PIN_RESET);

	DBG_STATE(VALVE, "Relay init: OFF (closed)");
}

void ValveDriver_SetRelay(bool on) {
	if (on != relay_state) {
		relay_state = on;
		HAL_GPIO_WritePin(Relay_Signal_GPIO_Port, Relay_Signal_Pin,
				on ? GPIO_PIN_SET : GPIO_PIN_RESET);

		DBG_STATE(VALVE, "Relay %s", on ? "ON (open)" : "OFF (closed)");
	}
}

bool ValveDriver_GetRelayState(void) {
	return relay_state;
}

/**************************************************************************
 END-OF-FILE
 ***************************************************************************/
