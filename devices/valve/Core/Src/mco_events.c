/**************************************************************************
 MODULE:    MCO_EVENTS
 CONTAINS:  MCO callback event dispatcher implementation
 Simple observer pattern — modules register callbacks,
 MCO_Target files fire events via MCO_Events_Fire().
 ***************************************************************************/

#include "mco_events.h"
#include <string.h>

/**************************************************************************
 LOCAL VARIABLES
 ***************************************************************************/
static MCO_EventCallback_t event_listeners[MCO_EVENT_COUNT][MCO_EVENTS_MAX_LISTENERS];
static uint8_t listener_count[MCO_EVENT_COUNT];

/**************************************************************************
 GLOBAL FUNCTIONS
 ***************************************************************************/

void MCO_Events_Init(void) {
	memset(event_listeners, 0, sizeof(event_listeners));
	memset(listener_count, 0, sizeof(listener_count));
}

int MCO_Events_Register(MCO_EventType_t type, MCO_EventCallback_t callback) {
	if (type >= MCO_EVENT_COUNT || callback == 0) {
		return -1;
	}

	if (listener_count[type] >= MCO_EVENTS_MAX_LISTENERS) {
		return -1;
	}

	event_listeners[type][listener_count[type]] = callback;
	listener_count[type]++;

	return 0;
}

void MCO_Events_Fire(const MCO_Event_t *event) {
	if (event == 0 || event->type >= MCO_EVENT_COUNT) {
		return;
	}

	uint8_t count = listener_count[event->type];
	for (uint8_t i = 0; i < count; i++) {
		if (event_listeners[event->type][i] != 0) {
			event_listeners[event->type][i](event);
		}
	}
}

/**************************************************************************
 END-OF-FILE
 ***************************************************************************/
