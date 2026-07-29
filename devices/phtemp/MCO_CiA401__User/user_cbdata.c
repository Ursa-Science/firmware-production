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



#if USECB_APPSDO_READ || USECB_APPSDO_WRITE
#include <string.h>
char MEM_CONST od_2222_23_rd_buf1[] = "012345678901234567890123456789";
char MEM_CONST od_2222_23_rd_buf2[] = "Test of custom entry 2222h,23h 0123456789";
uint8_t od_2222_23_wr_buf[64];

#define RW_BUFSIZE      20              // maximum size for single r/w buffer
#define FSSIMU_PACKETS  10              // maximum number of packets for the multi-buffer access
#define FSSIMU_MAXSIZE  (FSSIMU_PACKETS*RW_BUFSIZE) // maximum size for the multi-buffer access

// od_2222_24_rw_buf is the read/write buffer for entry [2222h,24h]. That's the
// buffer the stack "sees".
// od_2222_24_fssimu_buf simulates "some data source/sink in the background,"
// such as a file system. The stack never accesses this buffer directly. Instead,
// the call-back copies data back and forth in maximum chunks of RW_BUFSIZE.
// These MEM_CPY calls simulate file read/write.
uint8_t od_2222_24_rw_buf[RW_BUFSIZE];
uint8_t od_2222_24_fssimu_buf[FSSIMU_PACKETS][RW_BUFSIZE];

uint16_t bufcnt;
volatile uint32_t lenw = 0;
volatile uint32_t lenwc = 0;
uint8_t zero = 0;
#endif // USECB_APPSDO_READ || USECB_APPSDO_WRITE

#if USECB_APPSDO_READ
/*******************************************************************************
DOES:    Call Back function to allow implementation of custom, application
         specific OD Read entries
         Here: Alternating between 2 different strings
RETURNS: 0x00 - OD entry not handled by this function
         0x01 - OD entry handled by this function
         0x05 - Abort with "attempting to read a write-only object"
         0x06 - Abort with "entry does not exist"
         0x08 - Abort with "data type doesn't match" (CANopen FD only)
*******************************************************************************/
uint8_t MCOUSER_AppSDOReadInit (
  uint8_t sdoserver_client_nid, // CANopen: The SDO server number on which
                                  // the request came in.
                                  // CANopen FD: The USDO client node ID
                                  // from which the request came in.
  uint16_t idx, // Index of OD entry
  uint8_t subidx, // Subindex of OD entry
  uint32_t MEM_FAR *totalsize, // RETURN: total size of data, only set if >*size
  uint32_t MEM_FAR *size, // RETURN: size of data buffer
  uint8_t * MEM_FAR *pDat, // RETURN: pointer to data buffer
  uint8_t MEM_FAR *type // RETURN: data type (CANopen FD only)
  )
{
  static uint16_t lenr;

  if ((idx == 0x2222) && (subidx == 0x23))
  { // handle this access, read alternating strings in single-buffer transfer
    if (lenr != sizeof(od_2222_23_rd_buf1)-1)
    {
      lenr = sizeof(od_2222_23_rd_buf1)-1;
      *size = lenr;
      *pDat = (uint8_t *)&od_2222_23_rd_buf1[0];
    }
    else
    {
      lenr = sizeof(od_2222_23_rd_buf2)-1;
      *size = sizeof(od_2222_23_rd_buf2)-1;
      *pDat = (uint8_t *)&od_2222_23_rd_buf2[0];
    }
  }
  else if ((idx == 0x2222) && (subidx == 0x24))
  { // handle this access, multi-buffer transfer
    *pDat = (uint8_t *)&od_2222_24_rw_buf[0];
    *totalsize = lenw;
    // either transmit full r/w buffer length, or partial buffer if data length is smaller
    *size = (lenw > sizeof(od_2222_24_rw_buf)) ? sizeof(od_2222_24_rw_buf) : lenw;
    // keep track of how many bytes have been transmitted
    lenwc = *totalsize - *size;
    // Simulate file system read by copying from simulation buffer to single r/w buffer
    bufcnt = 0;
    MEM_CPY(&od_2222_24_rw_buf[0], &od_2222_24_fssimu_buf[bufcnt][0], sizeof(od_2222_24_rw_buf));
    bufcnt++;
  }
#if defined(USE_CANOPEN_FD) && (USE_CANOPEN_FD==1)
  else if ((idx == 0x1031) && (subidx == 0x04))
  { // handle access to Active Error History - Error History Domain, tbd., this default handler always returns 0 bytes
    *size = 0;
    *type = TYPE_DOMAIN;
  }
#endif
  else
  {
    return 0;
  }

  return 1;
}


/*******************************************************************************
DOES:    Call Back function to allow implementation of custom, application
         specific OD Read entries, called at end of transfer with the option
         to add more data.
RETURNS: Nothing
*******************************************************************************/
void MCOUSER_AppSDOReadComplete (
  uint8_t sdoserver_client_nid, // CANopen: The SDO server number on which
                                  // the request came in.
                                  // CANopen FD: The USDO client node ID
                                  // from which the request came in.
  uint16_t idx, // Index of OD entry
  uint8_t subidx, // Subindex of OD entry
  uint32_t MEM_FAR *size // RETURN: size of next block of data, 0 for no further data
  )
{
  if ((idx == 0x2222) && (subidx == 0x23))
  { // handle this access, single-buffer transfer finished
    *size = 0;
  }
  else if ((idx == 0x2222) && (subidx == 0x24))
  { // handle this access, multi-buffer transfer
    // either transmit full r/w buffer length, or partial buffer if it's the last one
    *size = (lenwc > sizeof(od_2222_24_rw_buf)) ? sizeof(od_2222_24_rw_buf) : lenwc;
    // keep track of how many bytes have been transmitted
    lenwc -= *size;
    // Simulate file system read by copying from simulation buffer to single r/w buffer
    MEM_CPY(&od_2222_24_rw_buf[0], &od_2222_24_fssimu_buf[bufcnt][0], sizeof(od_2222_24_rw_buf));
    bufcnt++;
  }
#if defined(USE_CANOPEN_FD) && (USE_CANOPEN_FD==1)
  else if ((idx == 0x1031) && (subidx == 0x04))
  { // handle access to Active Error History - Error History Domain, tbd., this default handler always returns 0 bytes
    *size = 0;
  }
#endif

  return;
}
#endif // USECB_APPSDO_READ


#if USECB_APPSDO_WRITE
/*******************************************************************************
DOES:    Call Back function to allow implementation of custom, application
         specific OD Read entries
         Here: Simply receive data
RETURNS: 0x00 - OD entry not handled by this function
         0x01 - OD entry handled by this function
         0x04 - Abort with "attempting to write a read-only object"
         0x06 - Abort with "entry does not exist"
         0x08 - Abort with "data type doesn't match" (CANopen FD only)
*******************************************************************************/
uint8_t MCOUSER_AppSDOWriteInit (
  uint8_t sdoserver_client_nid, // CANopen: The SDO server number on which
                                  // the request came in.
                                  // CANopen FD: The USDO client node ID
                                  // from which the request came in.
  uint16_t idx, // Index of OD entry
  uint8_t subidx, // Subindex of OD entry
  uint32_t MEM_FAR *totalsize, // RETURN: total maximum size of data, only set if >*size
  uint32_t MEM_FAR *size, // Data size, if known. RETURN: max size of data buffer
  uint8_t * MEM_FAR *pDat, // RETURN: pointer to data buffer
  uint8_t MEM_FAR *type // RETURN: data type (CANopen FD only)
  )
{
  if ((idx == 0x2222) && (subidx == 0x23))
  { // handle this access, single-buffer transfer
    *size = sizeof(od_2222_23_wr_buf);
    *pDat = (uint8_t *)&od_2222_23_wr_buf[0];
    return 1;
  }
  else if ((idx == 0x2222) && (subidx == 0x24))
  { // handle this access, multi-buffer transfer
    *totalsize = sizeof(od_2222_24_fssimu_buf);
    *size = sizeof(od_2222_24_rw_buf);
    *pDat = (uint8_t *)&od_2222_24_rw_buf[0];
    lenwc = 0;
    bufcnt = 0;
    return 1;
  }
  return 0;
}


/*******************************************************************************
DOES:    Call Back function to allow implementation of custom, application
         specific OD Write entries, call at end of transfer of a block. For
         multiple blocks per transfer, the same buffer is used for all blocks.
RETURNS: 0x00 - OD entry not handled by this function
         0x01 - OD entry handled by this function
         0x04 - Abort with "attempting to write a read-only object"
*******************************************************************************/
uint8_t MCOUSER_AppSDOWriteComplete (
  uint8_t sdoserver_client_nid, // CANopen: The SDO server number on which
                                  // the request came in.
                                  // CANopen FD: The USDO client node ID
                                  // from which the request came in.
  uint16_t idx, // Index of OD entry
  uint8_t subidx, // Subindex of OD entry
  uint32_t size, // Number of bytes written (of last block)
  uint32_t more // number of bytes still to come (of total transfer)
  )
{
  if ((idx == 0x2222) && (subidx == 0x23))
  { // handle this access, all should be done because of single-buffer transfer
    // Here enter code to retrieve data from buffer
    // Data length: size, more == 0
    if (more != 0)
    { // this should never happen
      for (;;); // wait here for break
    }

    return 0x01;
  }
  else if ((idx == 0x2222) && (subidx == 0x24))
  { // handle this access, multi-buffer transfer
    // simulate file system write by storing data from the single r/w buffer into the simulation buffer array
    MEM_CPY(&(od_2222_24_fssimu_buf[bufcnt][0]), &(od_2222_24_rw_buf[0]), size);
    bufcnt++;
    // keep track of how many bytes have been transferred
    lenwc += size;
    if (more == 0)
    { // this is the last transfer, all received
      lenw = lenwc; // save new length of the entry, for read access
    }

    return 0x01;
  }

  return 0x00;
}
#endif // USECB_APPSDO_WRITE


/**************************************************************************
END-OF-FILE
***************************************************************************/
