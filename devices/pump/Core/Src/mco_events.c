/**
 ******************************************************************************
 * @file    mco_events.c
 * @brief   Event dispatcher — decouples MCO callbacks from motor control
 ******************************************************************************
 */

#include "mco_events.h"
#include <stddef.h>

/* Private variables ---------------------------------------------------------*/
static MCO_EventCallback_t listener = NULL;

/* Public functions ----------------------------------------------------------*/

void MCO_Events_Init(void) {
	listener = NULL;
}

void MCO_Events_Register(MCO_EventCallback_t cb) {
	listener = cb;
}

void MCO_Events_Fire(const MCO_Event_t *event) {
	if (listener != NULL && event != NULL) {
		listener(event);
	}
}
