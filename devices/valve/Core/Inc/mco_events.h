/**************************************************************************
 MODULE:    MCO_EVENTS
 CONTAINS:  MCO callback event dispatcher — types and API
 Allows valve_control and other modules to register for MCO
 stack events (NMT change, heartbeat lost, fatal error, etc.)
 without modifying the MCO_Target callback files directly.
 ***************************************************************************/

#ifndef _MCO_EVENTS_H
#define _MCO_EVENTS_H

#include <stdint.h>

/**************************************************************************
 DEFINES: Event types
 ***************************************************************************/
typedef enum {
	MCO_EVENT_NMT_CHANGE = 0,    // NMT state changed
	MCO_EVENT_HEARTBEAT_LOST,    // Consumer heartbeat lost
	MCO_EVENT_FATAL_ERROR,       // Fatal error occurred
	MCO_EVENT_COMM_RESET,        // Communication reset completed
	MCO_EVENT_COUNT              // Number of event types
} MCO_EventType_t;

/**************************************************************************
 DEFINES: Event data structure
 ***************************************************************************/
typedef struct {
	MCO_EventType_t type;
	union {
		uint8_t nmt_state;      // For MCO_EVENT_NMT_CHANGE
		uint8_t node_id;        // For MCO_EVENT_HEARTBEAT_LOST
		uint16_t error_code;     // For MCO_EVENT_FATAL_ERROR
		uint8_t init_result;    // For MCO_EVENT_COMM_RESET
	};
} MCO_Event_t;

/**************************************************************************
 DEFINES: Callback function type
 ***************************************************************************/
typedef void (*MCO_EventCallback_t)(const MCO_Event_t *event);

/**************************************************************************
 DEFINES: Maximum listeners per event type
 ***************************************************************************/
#define MCO_EVENTS_MAX_LISTENERS  4

/**************************************************************************
 GLOBAL FUNCTIONS
 ***************************************************************************/

/**
 * @brief  Initialize the event dispatcher (clear all registrations)
 */
void MCO_Events_Init(void);

/**
 * @brief  Register a callback for a specific event type
 * @param  type     The event type to listen for
 * @param  callback Function to call when event fires
 * @retval 0 on success, -1 if no slots available
 */
int MCO_Events_Register(MCO_EventType_t type, MCO_EventCallback_t callback);

/**
 * @brief  Fire an event, calling all registered listeners
 * @param  event    Pointer to event data
 */
void MCO_Events_Fire(const MCO_Event_t *event);

#endif // _MCO_EVENTS_H
