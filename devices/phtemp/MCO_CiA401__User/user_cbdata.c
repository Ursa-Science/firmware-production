/**************************************************************************
MODULE:    USER_CBDATA
CONTAINS:  Default functions for user call-backs accessing process data
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
#include "mco_events.h"   /* MCO_EVENT_HEARTBEAT_RESTORED from MCOUSER_EMCY */


#ifdef MCOUSER_MINMAX
// SET THIS DEFINE MANUALLY TO ENABLE A CUSTOM MIN/MAX CHECK
// Also make sure to set USECB_SDO_WR_PI
// User example: Min Max control of SDO acces
uint8_t MEM_CONST gProcMin[PROCIMG_SIZE] = PIMGMINS;
uint8_t MEM_CONST gProcMax[PROCIMG_SIZE] = PIMGMAXS;
#endif


#if USECB_RPDORECEIVE
/**************************************************************************
DOES:    This function is called after an RPDO has been received and stored
         into the Process Image.
RETURNS: nothing
**************************************************************************/
void MCOUSER_RPDOReceived (
  uint16_t RPDONr, // RPDO Number
  uint16_t offset, // Offset to RPDO data in Process Image
  uint8_t  len     // Length of RPDO
  )
{
}
#endif // USECB_RPDORECEIVE


#if USECB_ODDATARECEIVED
/**************************************************************************
DOES:    This function is called after Object Dictionary data was received
         (works for SDO/USDO and PDO).
RETURNS: nothing
**************************************************************************/
void MCOUSER_ODData (
  uint8_t client_nid,     // node ID from where data arrived (0 if unknown)
  uint16_t idx,           // Index
  uint8_t subidx,         // Subindex
  uint8_t MEM_PROC *pDat, // pointer to data
  uint16_t len            // length of data
  )
{
}
#endif // USECB_ODDATARECEIVED


#if USECB_TPDORDY
/**************************************************************************
DOES:    This function is called before a TPDO is sent. For triggering
         modes that are outside of the application's doing (Event Timer,
         SYNC), it is called before the sent data is retrieved from the
         Process Image. This allows the application to update the TPDO
         data if necessary.
NOTE:    This function is also called before a change-of-state or
         application-triggered TPDO is sent, but updating the Process Image
         will not have any effect on the TPDO data in this case.
RETURNS: TRUE to allow the PDO to be sent, FALSE to stop PDO transmission
**************************************************************************/
uint8_t MCOUSER_TPDOReady (
  uint16_t TPDONr,      // TPDO Number
  uint8_t  TPDOTrigger  // Trigger for this TPDO's transmission:
                          // 0: Event Timer
                          // 1: SYNC
                          // 2: SYNC+COS
                          // 3: COS or application trigger
  )
{
  // always transmit if event timer or SYNC is being used
  if (TPDOTrigger < 2) return TRUE;

  // customize for application-specific TPDO send conditions
  return TRUE;
}
#endif // USECB_TPDORDY


#if USECB_SYNCRECEIVE
/**************************************************************************
DOES:    This function is called with every SYNC message received.
         VERSION for SYNC messages WITHOUT counter value.
 It allows the application to now apply all sync-triggered TPDO
 data to be applied to the application.
RETURNS: nothing
**************************************************************************/
void MCOUSER_SYNCReceived (
  void
  )
{
	// Motor control processing removed from SYNC callback to prevent blocking
	// Main loop handles all processing - callbacks must return immediately
	// The main loop runs MotorControl_Process() continuously at ~10kHz,
	// providing <100us response time without blocking CANopen stack timing
}

/**************************************************************************
DOES:    This function is called with every SYNC message received.
         VERSION for SYNC messages WITH counter value.
         It allows the application to now apply all sync-triggered TPDO
         data to be applied to the application.
RETURNS: nothing
**************************************************************************/
void MCOUSER_SYNCCNTReceived (
  uint8_t counter_value
  )
{
}
#endif // USECB_SYNCRECEIVE


#if defined(USECB_EMCY) && USECB_EMCY
/**************************************************************************
DOES:    Process pending or clearing Emergency events (set or release).
USE:     Use this to implement an active error list and to keep track
         of application specific error codes.
RETURNS: 0 - No objection from application to transmit EMCY message
         !=0  - Application requests, that EMCY message is NOT generated
**************************************************************************/
uint8_t MCOUSER_EMCY (
  uint8_t  ev_clr, // set to TRUE if this is to clear a previous EMCY event
  uint16_t emcy_code, // 16 bit error code
  uint8_t  em_1, // 5 byte manufacturer specific error code
  uint8_t  em_2,
  uint8_t  em_3,
  uint8_t  em_4,
  uint8_t  em_5
#if defined(USE_CANOPEN_FD) && (USE_CANOPEN_FD==1)
  ,
  uint8_t  dev_num,  // logical device number
  uint16_t spec_num, // CiA specification number
  uint8_t  status,   // status
  uint32_t time_lo,  // timestamp bits 0-31
  uint16_t time_hi   // timestamp bits 32-47
#endif // USE_CANOPEN_FD
  )
{
  /* The stack calls this with ev_clr set when a previously lost heartbeat
   * consumer sees the master again. Hand it to sensor_control so the stack's
   * latched ErrorRegister bit 0 can be cleared (the stack sets it on 0x8130
   * and never clears it itself). Return 0 so the stack still sends its own
   * recovery EMCY. */
  if (ev_clr && (emcy_code == EMCY_HB_ERR))
  {
    MCO_Event_t ev = { .type = MCO_EVENT_HEARTBEAT_RESTORED, .node_id = em_1 };
    MCO_Events_Fire(&ev);
  }
  (void)em_2; (void)em_3; (void)em_4; (void)em_5;
  return 0;
}
#endif // USECB_EMCY


#if USECB_SDO_RD_PI
/**************************************************************************
DOES:    This function is called before an SDO or USDO read request is
         executed reading from the process image. The application can
         use this function to either update the data or to deny access
         (by returning an SDO or USDO Abort code).
RETURNS: 0, if access is granted, data can be copied and returned or
         CANopen SDO or USDO Abort Code - in which case the (U)SDO 
         transfer is aborted
**************************************************************************/
uint32_t MCOUSER_SDORdPI (
  uint8_t client_nid,   // node ID from where the request came (0 if unknown)
  uint16_t index,       // Index of Object Dictionary entry
  uint8_t subindex,     // Subindex of Object Dictionary entry
  uint16_t offset,      // Offset to data in process image
  uint16_t len          // Length of data
  )
{
  return 0;
}
#endif // USECB_SDO_RD_PI


#if USECB_SDO_RD_AFTER
/**************************************************************************
DOES:    This function is called after an SDO or USDO read request was
         executed. The application can use this to clear the data or
         mark it as read.
RETURNS: Nothing
**************************************************************************/
void MCOUSER_SDORdAft (
  uint8_t client_nid,   // node ID from where the request came (0 if unknown)
  uint16_t index,       // Index of Object Dictionary entry
  uint8_t subindex,     // Subindex of Object Dictionary entry
  uint16_t offset,      // Offset to data in process image
  uint16_t len          // Length of data
  )
{
}
#endif // USECB_SDO_RD_AFTER


#if USECB_SDO_WR_PI
/**************************************************************************
DOES:    This function is called before an SDO or USDO write request is
         executed writing to the process image. The application can use
         this function to check the data (e.g. range check) BEFORE it
         gets written to the process image.
RETURNS: 0, if access is granted, data can be copied to process image or
         CANopen SDO or USDO Abort Code - in which case the (U)SDO 
         transfer is aborted
**************************************************************************/
uint32_t MCOUSER_SDOWrPI (
  uint8_t client_nid,   // node ID from where the request came (0 if unknown)
  uint16_t index,       // Index of Object Dictionary entry
  uint8_t subindex,     // Subindex of Object Dictionary entry
  uint16_t offset,      // Offset to data in process image
  uint8_t *pDat,        // Pointer to data received
  uint16_t len          // Length of data
  )
{
#ifdef MCOUSER_MINMAX
uint16_t dat;
uint16_t comp;

  if ((index == 0x2030) && (subindex != 0x00))
  { // MinMax test entry for uint16_t
    // Get current data
    dat = pDat[1];
    dat <<= 8;
    dat += pDat[0];
    // Get comparison minimum data
    comp = gProcMin[offset+1];
    comp <<= 8;
    comp += gProcMin[offset];
    if (dat < comp)
    {
      return SDO_ABORT_VALUE_LOW;
    }
    // Get comparison maximum data
    comp = gProcMax[offset+1];
    comp <<= 8;
    comp += gProcMax[offset];
    if (dat > comp)
    {
      return SDO_ABORT_VALUE_HIGH;
    }
  }
#endif // MCOUSER_MINMAX
  return 0;
}
#endif // USECB_SDO_WR_PI


#if USECB_SDO_WR_AFTER
/**************************************************************************
DOES:    This function is called after an SDO or USDO write request was
         executed. Data is now in the process image and can be processed.
RETURNS: Nothing
**************************************************************************/
void MCOUSER_SDOWrAft (
  uint8_t client_nid,   // node ID from where the request came (0 if unknown)
  uint16_t index,       // Index of Object Dictionary entry
  uint8_t subindex,     // Subindex of Object Dictionary entry
  uint16_t offset,      // Offset to data in process image
  uint16_t len          // Length of data
  )
{
	// Motor control processing removed from SDO callback - CRITICAL FIX
	// This callback is executed DURING SDO message processing and must return
	// immediately to allow the SDO response to be transmitted. Any delay here
	// blocks the SDO response, causing commands to appear unresponsive.
	//
	// Main loop handles all processing - it runs MotorControl_Process()
	// continuously at ~10kHz, providing <100us response time without blocking
	// the CANopen stack's critical timing for SDO responses.
	//
	// Data written via SDO is already in the process image (gProcImg) and will
	// be detected by MotorControl_Process() on the very next main loop iteration.
}
#endif // USECB_SDO_WR_AFTER



#if USECB_APPSDO_READ
/*******************************************************************************
DOES:    Call Back for custom, application-specific segmented OD read entries.
         2026-09-22: the EmSA demo handler on [2222h,23h/24h] (alternating test
         strings + a simulated file) was REMOVED. 0x2222 was LastCalibrationDate
         until the dumb-module OD regen and no longer exists; the demo made it
         answer SDO reads with a phantom object. Nothing is handled here now.
RETURNS: 0x00 - OD entry not handled by this function (stack aborts with
                "object does not exist")
*******************************************************************************/
uint8_t MCOUSER_AppSDOReadInit (
  uint8_t sdoserver_client_nid,
  uint16_t idx,
  uint8_t subidx,
  uint32_t MEM_FAR *totalsize,
  uint32_t MEM_FAR *size,
  uint8_t * MEM_FAR *pDat,
  uint8_t MEM_FAR *type
  )
{
  (void)sdoserver_client_nid; (void)idx; (void)subidx;
  (void)totalsize; (void)size; (void)pDat; (void)type;
  return 0;
}


/*******************************************************************************
DOES:    End-of-transfer hook for custom segmented reads. Nothing is handled.
RETURNS: Nothing
*******************************************************************************/
void MCOUSER_AppSDOReadComplete (
  uint8_t sdoserver_client_nid,
  uint16_t idx,
  uint8_t subidx,
  uint32_t MEM_FAR *size
  )
{
  (void)sdoserver_client_nid; (void)idx; (void)subidx;
  *size = 0;
}
#endif // USECB_APPSDO_READ


#if USECB_APPSDO_WRITE
/*******************************************************************************
DOES:    Call Back for custom, application-specific segmented OD write entries.
         Demo handler on [2222h] removed (see MCOUSER_AppSDOReadInit).
RETURNS: 0x00 - OD entry not handled by this function
*******************************************************************************/
uint8_t MCOUSER_AppSDOWriteInit (
  uint8_t sdoserver_client_nid,
  uint16_t idx,
  uint8_t subidx,
  uint32_t MEM_FAR *totalsize,
  uint32_t MEM_FAR *size,
  uint8_t * MEM_FAR *pDat,
  uint8_t MEM_FAR *type
  )
{
  (void)sdoserver_client_nid; (void)idx; (void)subidx;
  (void)totalsize; (void)size; (void)pDat; (void)type;
  return 0;
}


/*******************************************************************************
DOES:    End-of-block hook for custom segmented writes. Nothing is handled.
RETURNS: 0x00 - OD entry not handled by this function
*******************************************************************************/
uint8_t MCOUSER_AppSDOWriteComplete (
  uint8_t sdoserver_client_nid,
  uint16_t idx,
  uint8_t subidx,
  uint32_t size,
  uint32_t more
  )
{
  (void)sdoserver_client_nid; (void)idx; (void)subidx; (void)size; (void)more;
  return 0x00;
}
#endif // USECB_APPSDO_WRITE


/**************************************************************************
END-OF-FILE
***************************************************************************/
