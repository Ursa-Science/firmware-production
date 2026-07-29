/**
 ******************************************************************************
 * @file    mco_events.h
 * @brief   Event dispatcher — decouples MCO callbacks from motor control
 * @note    MCO callback files (user_STM32.c,
 *          user_cbdata.c) fire events here; motor_control.c registers a
 *          listener to handle them.
 ******************************************************************************
 */

#ifndef MCO_EVENTS_H
#define MCO_EVENTS_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* Event types ---------------------------------------------------------------*/
typedef enum {
	MCO_EVENT_FATAL_ERROR,      // MCOUSER_FatalError — fatal/warning from stack
	MCO_EVENT_COMM_RESET,    // MCOUSER_ResetCommunication — stack (re)init done
	MCO_EVENT_NMT_CHANGE,       // MCOUSER_NMTChange — NMT state transition
	MCO_EVENT_HEARTBEAT_LOST    // MCOUSER_HeartbeatLost — consumer node timeout
} MCO_EventType_t;

/* Event payload (tagged union) ----------------------------------------------*/
typedef struct {
	MCO_EventType_t type;
	union {
		uint16_t error_code;    // FATAL_ERROR: EmSA error code
		uint8_t init_result;   // COMM_RESET:  1 = stack init OK, 0 = failed
		uint8_t nmt_state;     // NMT_CHANGE:  new NMT state value
		uint8_t node_id;       // HEARTBEAT_LOST: node that timed out
	};
} MCO_Event_t;

/* Callback signature --------------------------------------------------------*/
typedef void (*MCO_EventCallback_t)(const MCO_Event_t *event);

/* Public API ----------------------------------------------------------------*/

/**
 * @brief  Initialize the event dispatcher (clears listener list)
 * @note   Call once from main() before any MCO stack calls.
 */
void MCO_Events_Init(void);

/**
 * @brief  Register a listener callback
 * @param  cb  Function to call when any MCO event fires
 * @note   Currently supports a single listener (motor_control.c).
 *         Extend to an array if more consumers are needed later.
 */
void MCO_Events_Register(MCO_EventCallback_t cb);

/**
 * @brief  Fire an event — calls all registered listeners synchronously
 * @param  event  Pointer to populated event struct
 */
void MCO_Events_Fire(const MCO_Event_t *event);

#ifdef __cplusplus
}
#endif

#endif /* MCO_EVENTS_H */
