/**************************************************************************
 MODULE:    VALVE_DRIVER
 CONTAINS:  Low-level relay GPIO control for valve actuator
 PA10 (Relay_Signal) — active HIGH = valve energized (open)
 ***************************************************************************/

#ifndef _VALVE_DRIVER_H
#define _VALVE_DRIVER_H

#include <stdint.h>
#include <stdbool.h>

/**************************************************************************
 GLOBAL FUNCTIONS
 ***************************************************************************/

/**
 * @brief  Initialize the valve driver (GPIO already configured by CubeMX)
 *         Sets relay to de-energized (closed) state.
 */
void ValveDriver_Init(void);

/**
 * @brief  Set the relay state
 * @param  on  true = relay energized (valve open), false = de-energized (valve closed)
 */
void ValveDriver_SetRelay(bool on);

/**
 * @brief  Get current relay state
 * @retval true if relay is energized (valve open)
 */
bool ValveDriver_GetRelayState(void);

#endif // _VALVE_DRIVER_H
