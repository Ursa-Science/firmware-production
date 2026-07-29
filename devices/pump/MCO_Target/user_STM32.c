/**************************************************************************
MODULE:    USER
CONTAINS:  MicroCANopen special events implementation
COPYRIGHT: Embedded Systems Academy (EmSA) 2002-2024
           All rights reserved. esacademy.com
DISCLAIM:  Read and understand our disclaimer before using this code!
           www.esacademy.com/disclaim.htm
           This software was written in accordance to the guidelines at
           www.esacademy.com/software/softwarestyleguide.pdf
LICENSE:   THIS IS THE COMMERCIAL VERSION OF MICRO CANOPEN PLUS
           ONLY USERS WHO PURCHASED A LICENSE MAY USE THIS SOFTWARE
           See file license_commercial_plus.txt or
           www.microcanopen.com/license_commercial_plus.txt
VERSION:   7.17, EmSA 04-MAR-24
           $LastChangedDate: 2024-03-04 12:55:21 +0100 (Mon, 04 Mar 2024) $
           $LastChangedRevision: 5559 $
***************************************************************************/

#include "mcop_inc.h"
#include "stackinit.h"
#include "mco_events.h"

/**************************************************************************
 GLOBAL VARIABLES
 ***************************************************************************/

// LED ownership gate (see MCO_Target/mcohw_LEDs.h). 0 = MCO stack owns the
// RUN/ERR LEDs (default); non-0 = the application (led_control.c, OD 0x2000)
// owns the LEDs and the stack's LED writes are suppressed.
volatile uint8_t gLEDAppOverride = 0;

/**************************************************************************
GLOBAL FUNCTIONS
***************************************************************************/

/**************************************************************************
 DOES:    Sets/clears the application's LED ownership of the shared tri-LED.
 While active, the MCO stack's CiA-303-3 RUN/ERR indicator writes are
 suppressed so they cannot mask the application's indication.
 RETURNS: nothing
 **************************************************************************/
void MCOTGT_SetLEDOverride(uint8_t active // 0 = release to stack, non-0 = application owns the LED
		) {
	gLEDAppOverride = (active) ? 1 : 0;
}

/**************************************************************************
DOES:    Call-back function for occurance of a fatal error.
         Stops operation and displays blnking error pattern on LED
**************************************************************************/
void MCOUSER_FatalError (
  uint16_t ErrCode
  )
{
uint16_t timeout;

	// Fire event — motor control handles emergency stop
	MCO_Event_t ev = { .type = MCO_EVENT_FATAL_ERROR, .error_code = ErrCode };
	MCO_Events_Fire(&ev);

  // Remember last error for stack
  gMCOConfig.last_fatal = ErrCode;

#if USE_LEDS
  gMCOConfig.LEDErr = LED_ON;
#endif

#if USE_EMCY
  (void)MCOP_PushEMCY(EMCY_INTERN_SW,(uint8_t)(ErrCode >> 8),(uint8_t)ErrCode,0,0,0
#if defined(USE_CANOPEN_FD) && (USE_CANOPEN_FD==1)
    ,
    0,     // dev_num - logical device number
    COFD_BASE_SPEC >> 20, // spec_num, CiA specification number
    ERST_STATE_OCC | ERST_CLASS_NREC | ERST_PRIO(0),   // status - priority=0, non-recoverable, error occurred
    0,     // time_lo - timestamp bits 0-31, not supported
    0      // time_hi - timestamp bits 32-47, not supported
#endif // USE_CANOPEN_FD
  );
#endif // USE_EMCY

  // Wait 10ms
  timeout = MCOHW_GetTime() + 10;
  while (!MCOHW_IsTimeExpired(timeout))
  { // Wait until timeout
  }

  if (ErrCode >= ERR_FATAL)
  { // Fatal error, should abort/reset
    MCOUSER_ResetApplication();
  }

  // Warning only, simply timeout, then continue
}


/**************************************************************************
DOES:    Call-back function for reset application.
         Starts the watchdog and waits until watchdog causes a reset.
**************************************************************************/
void MCOUSER_ResetApplication (void)
{
#ifdef __SIMULATION__
  // if reset callback function has been passed from the simnodehandler
  // then call it
  // reset function does not return
  if (mpReset != 0) mpReset();
#else
#ifdef USE_WATCHDOG_FOR_RESET
  // ensure LSI RC oscillator is turned on
  RCC->CSR |= RCC_LSE_ON;
  // clear watchdog reset flag
  RCC->CSR |= RCC_CSR_RMVF;
  IWDG->KR  = 0x5555;
  IWDG->PR  = 0x00000000;
  IWDG->RLR = 0x00000000;
  IWDG->KR  = 0xAAAA;
  IWDG->KR  = 0xCCCC;
#else
  NVIC_SystemReset();
#endif
  // wait for reset
  while (1);
#endif
}


/**************************************************************************
DOES:    This function both resets and initializes both the CAN interface
         and the CANopen protocol stack. It is called from within the
         CANopen protocol stack if an NMT master message was received with
         the command "Reset Communication".
NOTE:    This function normally calls MCO_DefaultResetCommunication().
RETURNS: FALSE, if initialization was not successful
         TRUE, if initialization was successful
**************************************************************************/
uint8_t MCOUSER_ResetCommunication (void)
{
  uint8_t result;

  // Use default stack initialization
#if !defined(USE_CANOPEN_FD) || (defined(USE_CANOPEN_FD) && (USE_CANOPEN_FD==0))
  result = MCO_DefaultResetCommunication(NODEID,CAN_BITRATE,DEFAULT_HEARTBEAT);
#else
  result = MCO_DefaultResetCommunication(NODEID,CAN_BITRATE,CAN_BRS_BITRATE,DEFAULT_HEARTBEAT);
#endif

	// Fire event — motor control handles its own initialization
	MCO_Event_t ev = { .type = MCO_EVENT_COMM_RESET, .init_result = result };
	MCO_Events_Fire(&ev);

  return result;
}


#if USECB_NMTCHANGE
/**************************************************************************
DOES:    Called when the NMT state of the stack changes
RETURNS: nothing
**************************************************************************/
void MCOUSER_NMTChange
  (
  uint8_t nmtstate    // new nmt state of stack
  )
{
	// Fire event — motor control decides what to do per NMT state
	MCO_Event_t ev = { .type = MCO_EVENT_NMT_CHANGE, .nmt_state = nmtstate };
	MCO_Events_Fire(&ev);
}
#endif


#if USECB_ODSERIAL
/**************************************************************************
DOES:    This function is called upon read requests to Object Dictionary
         entry [1018h,4] - serial number
RETURNS: The 32bit serial number
**************************************************************************/
uint32_t MCOUSER_GetSerial (
  void
  )
{
  // replace with code to read serial number, for example from
  // non-volatile memory
  return 0x12345678;
}
#endif // USECB_ODSERIAL


/**************************************************************************
DOES:    Call-back function, heartbeat lost, timeout occured.
         Gets called when a heartbeat timeout occured for a node.
RETURNS: Nothing
**************************************************************************/
void MCOUSER_HeartbeatLost (
  uint8_t node_id
  )
{
	// Fire event — motor control handles emergency stop
	MCO_Event_t ev = { .type = MCO_EVENT_HEARTBEAT_LOST, .node_id = node_id };
	MCO_Events_Fire(&ev);

  // Add code to react on the loss of heartbeat,
  // if node is essential, switch to pre-operational mode
  MCO_HandleNMTRequest(NMTMSG_PREOP);
}


#if USECB_TIMEOFDAY
/**************************************************************************
DOES:    This function is called if the message with the time object has
         been received. This example implementtion calculates the current
         time in hours, minutes, seconds.
RETURNS: nothing
**************************************************************************/
// Global variables holding clock info
uint32_t hours;
uint32_t minutes;
uint32_t seconds;

void MCOUSER_TimeOfDay (
  uint32_t millis, // Milliseconds since midnight
  uint16_t days  // Number of days since January 1st, 1984
  )
{
  if (millis < (1000UL * 60 * 60 * 24))
  { // less Milliseconds as one day has
    // calculate hours, minutes & seconds since midnight
    hours = millis / (1000UL * 60 * 60);
    minutes = (millis - (hours * 1000UL * 60 * 60)) / (1000UL * 60);
    seconds = (millis - (hours * 1000UL * 60 * 60) - (minutes * 1000UL * 60)) / 1000;
  }
}
#endif // USECB_TIMEOFDAY

/**************************************************************************
END-OF-FILE
***************************************************************************/
