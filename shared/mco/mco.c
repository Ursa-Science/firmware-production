/**************************************************************************
MODULE:    MCO
CONTAINS:  MicroCANopen implementation
COPYRIGHT: Embedded Systems Academy (EmSA) 2002-2024
           All rights reserved. esacademy.com
DISCLAIM:  Read and understand our disclaimer before using this code!
           www.esacademy.com/disclaim.htm
           This software was written in accordance to the guidelines at
           www.esacademy.com/software/softwarestyleguide.pdf
LICENSE:   THIS IS THE COMMERCIAL PLUS VERSION OF MICROCANOPEN
           ONLY USERS WHO PURCHASED A LICENSE MAY USE THIS SOFTWARE
           See file license_commercial_plus.txt
VERSION:   7.17, EmSA 04-MAR-24
           $LastChangedDate: 2024-03-04 12:55:21 +0100 (Mon, 04 Mar 2024) $
           $LastChangedRevision: 5559 $
***************************************************************************/

#include "mcop_inc.h"

#if USE_XOD_ACCESS
#include "xod.h"
#endif

#if MGR_MONITOR_ALL_NODES
#include "mcop_mgr_inc.h"
#endif


/**************************************************************************
GLOBAL VARIABLES
***************************************************************************/

// this structure holds all node specific configuration
MCO_CONFIG MEM_FAR gMCOConfig;

#if (INDEX_FOR_DIAGNOSTICS != 0)
// MCO diagnostics record
MCO_DIAGNOSTICS MEM_FAR gMCODiag;
#endif

#if USE_EMCY
extern EMCY_CONFIG MEM_FAR gEF;
 #ifndef EMCY_INHIBIT_TIME
  // Set default emergency inhibit time
  #define EMCY_INHIBIT_TIME 0
 #endif
#endif

#if ! MGR_MONITOR_ALL_NODES
 #if (NR_OF_HB_CONSUMER > 0)
// from MCOP, HB consumer values
extern HBCONS_CONFIG MEM_FAR gHBCons[NR_OF_HB_CONSUMER];
static uint8_t MEM_FAR mHBchn = 0; // current channel checked
 #endif // (NR_OF_HB_CONSUMER > 0)
#endif

#if NR_OF_TPDOS > 0
// this structure holds all the TPDO configuration data for the TPDOs
TPDO_CONFIG MEM_FAR gTPDOConfig[NR_OF_TPDOS];
#endif

// this is the next TPDO to be checked in MCO_ProcessStack
uint16_t MEM_FAR gTPDONr = NR_OF_TPDOS;

#if NR_OF_RPDOS > 0
// this structure holds all the RPDO configuration data for the RPDOs
RPDO_CONFIG MEM_FAR gRPDOConfig[NR_OF_RPDOS];
#endif

// this structure holds the current receive message
static CAN_MSG gRxCAN;

// this structure holds the CAN message for SDO responses or aborts
CAN_MSG gTxSDO;


/**************************************************************************
LOCAL VARIABLES
***************************************************************************/


/**************************************************************************
LOCAL FUNCTIONS
***************************************************************************/

#if USE_LEDS
/**************************************************************************
DOES:    This function switches the CANopen Err and Run LEDs
         as specified by DR-303-3
         It must be called every 200ms
GLOBALS: Uses global macros LED_xxx_ON and LED_xxx_OFF.
         Uses module variables mLEDtoggle and mLEDcnt for the blinking.
**************************************************************************/
// NOT static, could be used outside this module to keep LEDs functioning
void MCO_SwitchLEDs  (
  void
  )
{
  // For blinking or flickering
  gMCOConfig.LEDtoggle = ~gMCOConfig.LEDtoggle;

  switch(gMCOConfig.LEDRun) // Run LED
  {
    case LED_OFF:
      LED_RUN_OFF;
      break;
    case LED_ON:
      LED_RUN_ON;
      break;
    case LED_BLINK:
      if (gMCOConfig.LEDtoggle == 0)
      {
        LED_RUN_ON;
      }
      else
      {
        LED_RUN_OFF;
      }
      break;
    case LED_FLASH1:
      gMCOConfig.LEDcntR++;
      if (gMCOConfig.LEDcntR >= 6)
      {
        gMCOConfig.LEDcntR = 0;
        LED_RUN_ON;
      }
      else
      {
        LED_RUN_OFF;
      }
      break;
    case LED_FLASH2:
      gMCOConfig.LEDcntR++;
      if (gMCOConfig.LEDcntR >= 8)
      {
        gMCOConfig.LEDcntR = 0;
      }
      if ((gMCOConfig.LEDcntR == 0) || (gMCOConfig.LEDcntR == 2))
      {
        LED_RUN_ON;
      }
      else
      {
        LED_RUN_OFF;
      }
      break;
    case LED_FLASH3:
      gMCOConfig.LEDcntR++;
      if (gMCOConfig.LEDcntR >= 10)
      {
        gMCOConfig.LEDcntR = 0;
      }
      if ((gMCOConfig.LEDcntR == 0) || (gMCOConfig.LEDcntR == 2) || (gMCOConfig.LEDcntR == 4))
      {
        LED_RUN_ON;
      }
      else
      {
        LED_RUN_OFF;
      }
      break;
    case LED_FLASH4:
      gMCOConfig.LEDcntR++;
      if (gMCOConfig.LEDcntR >= 12)
      {
        gMCOConfig.LEDcntR = 0;
      }
      if ((gMCOConfig.LEDcntR == 0) || (gMCOConfig.LEDcntR == 2) ||
          (gMCOConfig.LEDcntR == 4) || (gMCOConfig.LEDcntR == 6)
         )
      {
        LED_RUN_ON;
      }
      else
      {
        LED_RUN_OFF;
      }
      break;
    default:
      break;
  }

  switch(gMCOConfig.LEDErr) // Error LED
  {
    case LED_OFF:
      LED_ERR_OFF;
      break;
    case LED_ON:
      LED_ERR_ON;
      break;
    case LED_BLINK: // flicker when called every 50ms
      if (gMCOConfig.LEDtoggle == 0)
      {
        LED_ERR_ON;
      }
      else
      {
        LED_ERR_OFF;
      }
      break;
    case LED_FLASH1:
      gMCOConfig.LEDcntE++;
      if (gMCOConfig.LEDcntE >= 6)
      {
        gMCOConfig.LEDcntE = 0;
        LED_ERR_ON;
      }
      else
      {
        LED_ERR_OFF;
      }
      break;
    case LED_FLASH2:
      gMCOConfig.LEDcntE++;
      if (gMCOConfig.LEDcntE >= 8)
      {
        gMCOConfig.LEDcntE = 0;
      }
      if ((gMCOConfig.LEDcntE == 0) || (gMCOConfig.LEDcntE == 2))
      {
        LED_ERR_ON;
      }
      else
      {
        LED_ERR_OFF;
      }
      break;
    case LED_FLASH3:
      gMCOConfig.LEDcntE++;
      if (gMCOConfig.LEDcntE >= 10)
      {
        gMCOConfig.LEDcntE = 0;
      }
      if ((gMCOConfig.LEDcntE == 0) || (gMCOConfig.LEDcntE == 2) || (gMCOConfig.LEDcntE == 4))
      {
        LED_ERR_ON;
      }
      else
      {
        LED_ERR_OFF;
      }
      break;
    case LED_FLASH4:
      gMCOConfig.LEDcntE++;
      if (gMCOConfig.LEDcntE >= 12)
      {
        gMCOConfig.LEDcntE = 0;
      }
      if ((gMCOConfig.LEDcntE == 0) || (gMCOConfig.LEDcntE == 2) ||
          (gMCOConfig.LEDcntE == 4) || (gMCOConfig.LEDcntE == 6)
         )
      {
        LED_ERR_ON;
      }
      else
      {
        LED_ERR_OFF;
      }
      break;
    default:
      break;
  }
}
#endif // USE_LEDS


#if !defined(USE_CANOPEN_FD) || (defined(USE_CANOPEN_FD) && (USE_CANOPEN_FD==0))
/**************************************************************************
DOES:    Handle an incoimg SDO request.
RETURNS: returns 1 if SDO access success, returns 0 if SDO abort generated
**************************************************************************/
static uint8_t MCO_HandleSDORequest (
   uint8_t *pData  // pointer to 8 data bytes with SDO data
  )
{
  // command byte of SDO request
  uint8_t cmd;
  // index of SDO request
  uint16_t index;
  // subindex of SDO request
  uint8_t subindex;
  // search result of Search_OD
  uint16_t found;
  // buffer for read of error data
  uint8_t buf[4];
  uint32_t buf32;
  uint8_t len;
  uint16_t offset;
  // pointer to an entry in gODProcTable
  OD_PROCESS_DATA_ENTRY MEM_CONST *pOD;
#if USECB_ODSERIAL
  // buffer for serial number
  uint32_t serial;
#endif
#if USE_EXTENDED_SDO
  uint8_t sdoserv;
#endif
#if USECB_SDO_RD_PI || USECB_SDO_WR_PI
  uint32_t sdoreturn;
#endif

  // Copy Multiplexor into response, before custom call back
  // index low
  gTxSDO.BUF[1] = pData[1];
  // index high
  gTxSDO.BUF[2] = pData[2];
  // subindex
  gTxSDO.BUF[3] = pData[3];
  // ensure all bytes of data are erased (also unused)
  gTxSDO.BUF[4] = 0;
  gTxSDO.BUF[5] = 0;
  gTxSDO.BUF[6] = 0;
  gTxSDO.BUF[7] = 0;

#if USECB_SDOREQ
  switch (MCOUSER_SDORequest(pData))
  {
  case 0:  // MCOUSER_SDORequest replied with an ABORT
    return 0;
  case 1:  // MCOUSER_SDORequest sent a response
    return 1;
  default: // MCOUSER_SDORequest did not do anything
    break;
  }
#endif // USECB_SDOREQ

#if USE_EXTENDED_SDO
  sdoserv = SDOSERVER(gTxSDO.ID);
  switch(XSDO_HandleExtended(( uint8_t *)pData,&(gTxSDO),sdoserv))
  {
    case 1:
      // SDO response was sent
      return 1;
    case 2:
      // SDO abort was sent
      XSDO_Abort(sdoserv);
      return 0;
    default:
      break;
  }
#endif // USE_EXTENDED_SDO

  // init variables
  // upper 3 bits are the command
  cmd = *pData & 0xE0;
  // get high byte of index
  index = pData[2];
  // add low byte of index
  index = (index << 8) + pData[1];
  // subindex
  subindex = pData[3];

  buf[0] = 0;
  buf[1] = 0;
  buf[2] = 0;
  buf[3] = 0;

#if USE_BLOCKED_SDO
  // Translate block read into regular read
  if ((cmd == 0xA0) && (pData[5] != 0))
  {
    cmd = 0x40;
  }
#endif

  // is it a read or write command?
  if ((cmd == 0x40) || (cmd == 0x20))
  {
    // Response for conformance test, data type of 1000h entry
    // this version for cctt V3.01
    if ((index == 0x1000) && (subindex == 0xFF))
    {
      MCO_SendSDOAbort(SDO_ABORT_UNKNOWNSUB);
      return 0;
    }
#if USECB_ODSERIAL
    else if ((index == 0x1018) && (subindex == 4))
    {
      // read command?
      if (cmd == 0x40)
      {
        // Get serial number from call-back function
        serial = MCOUSER_GetSerial();
        return MCO_ReplyWith((uint8_t *)&(serial),4);
      }
      MCO_SendSDOAbort(SDO_ABORT_READONLY);
      return 0;
    }
#endif // USECB_ODSERIAL

#if NR_OF_SDOSERVER > 0
    if ((index >= 0x1200) && (index < 0x1200+NR_OF_SDOSERVER))
    {
      return XSDO_HandleSDOServerParam(index,pData);
    }
#endif

#if NR_OF_SDO_CLIENTS > 0
    if ((index >= 0x1280) && (index < 0x1280+NR_OF_SDO_CLIENTS))
    {
      return XSDO_HandleSDOClientParam(index,pData);
    }
#endif

#if (NR_OF_RPDOS > 0) || (NR_OF_TPDOS > 0)
  #if (NR_OF_RPDOS > 0)
    if ((index >= 0x1400) && (index <= 0x15FF))
    { // RPDO
      return SDO_HandlePDOComParam(1,index,pData);
    }
    #if USE_DYNAMIC_PDO_MAPPING
    if ((index >= 0x1600) && (index <= 0x17FF))
    { // RPDO
      return SDO_HandlePDOMapParam(1,index,pData);
    }
    #endif
  #endif
  #if (NR_OF_TPDOS > 0)
    if ((index >= 0x1800) && (index <= 0x19FF))
    { // TPDO
      return SDO_HandlePDOComParam(0,index,pData);
    }
    #if USE_DYNAMIC_PDO_MAPPING
    if ((index >= 0x1A00) && (index <= 0x1BFF))
    { // TPDO
      return SDO_HandlePDOMapParam(0,index,pData);
    }
    #endif
  #endif
#endif // (NR_OF_RPDOS > 0) || (NR_OF_TPDOS > 0)

    // access to [1017,00] - heartbeat time
    if ((index == 0x1017) && (subindex == 0x00))
    {
      // read command
      if (cmd == 0x40)
      {
        // expedited, 2 bytes of data
        gTxSDO.BUF[0] = 0x4B;
        GEN_WR16(PIACC_NONE,&(gTxSDO.BUF[4]),gMCOConfig.heartbeat_time);
        if (!MCOHW_PushMessage(&gTxSDO))
        {
          MCOUSER_FatalError(ERROFL_SDO);
        }
        return 1;
      }
      // expedited write command with 2 bytes of data
      if (*pData == 0x2B)
      {
        if (pData[5] > 0x7F)
        { // maximum supported is 0x7FFF
          MCO_SendSDOAbort(SDO_ABORT_VALUE_HIGH);
          return 0;
        }
#if USE_CiA447
        // Support Profile specific write support,
        // write is only allowed, if PROFILE_GetSDOFromNode() is <= 1
        if (PROFILE_GetSDOFromNode() > 1)
        {
          MCO_SendSDOAbort(SDO_ABORT_NOTRANSFERCTRL);
          return 0;
        }
#endif
        buf32 = GEN_RD16(PIACC_NONE,&(pData[4]));
        if (buf32 != gMCOConfig.heartbeat_time)
        { // only do something, if new value is unequal to previous
          gMCOConfig.heartbeat_time = buf32;
          // reset heartbeat time for immediate transmission or current time plus new heartbeat time?
          // Current 3.01 conformance test 9.4 (state 04) requires this to be current time plus new heartbeat time
          // gMCOConfig.heartbeat_timestamp = MCOHW_GetTime();
          gMCOConfig.heartbeat_timestamp = MCOHW_GetTime() + gMCOConfig.heartbeat_time;
        }
        // write response
        gTxSDO.BUF[0] = 0x60;
        if (!MCOHW_PushMessage(&gTxSDO))
        {
          MCOUSER_FatalError(ERROFL_SDO);
        }
        return 1;
      }
      if (*pData == 0x21)
      {
        // Needed to pass conformance test: Abort code different
        // for segmented write command with 2 bytes of data
        MCO_SendSDOAbort(SDO_ABORT_UNKNOWN_COMMAND);
      }
      else if ((*pData == 0x23) || (*pData == 0x2F))
      {
        // expedited write command with 4 bytes of data
        // Needed to pass conformance test: Abort code different for this case
        MCO_SendSDOAbort(SDO_ABORT_TYPEMISMATCH);
      }
      else
      {
        // MCO_SendSDOAbort(SDO_ABORT_UNSUPPORTED);
        // Conformance test does not accept above error code
        MCO_SendSDOAbort(SDO_ABORT_GENERAL);
      }
      return 0;
    }

    // access to [1018h] with extd conform compatibility
    else if ( (index == 0x1018) && (subindex > 0) && (cmd == 0x40) && (pData[4] == 0x41) && (pData[5] == 0x53) && (pData[6] == 0x45) && (pData[7] == 0xFF) )
    {
      buf[0] = 0x41; buf[1] = 0x53; buf[2] = 0x45; buf[3] = 0xFF;
      if (subindex == 0x02)
      {
        buf[0] = 0x4D; buf[1] = 0x43; buf[2] = 0x4F; buf[3] = 0x50;
      }
      else if (subindex == 0x03)
      {
      buf[3] = 0x07; buf[2] = 0x17;
#ifdef REV
      buf[1] = (uint8_t) REV; buf[0] = (uint8_t) (REV >> 8);
#else
      buf[1] = 0; buf[0] = 0;
#endif
      }
      return MCO_ReplyWith(buf,4);
    }

#ifdef REBOOT_FLAG_ADR
    // Supporting the ESAcademy CANopen Bootloader
    if ((index == 0x1F51) && (subindex == 0x01))
    {
      // read command
      if (cmd == 0x40)
      {
        // expedited, 1 byte of data
        gTxSDO.BUF[0] = 0x4F;
        gTxSDO.BUF[4] = 1;   // return "application running"
        if (!MCOHW_PushMessage(&gTxSDO))
        {
          MCOUSER_FatalError(ERROFL_SDO);
        }
        return 1;
      }

      // expedited write command with 1 byte of data
      if (*pData == 0x2F)
      {
        // Write of 0 (stop application)?
        if (pData[4] == 0x00)
        { // Set signal to bootloader
          REBOOT_FLAG = REBOOT_BOOTLOAD_VAL;
          // write response
          gTxSDO.BUF[0] = 0x60;
          if (!MCOHW_PushMessage(&gTxSDO))
          {
            MCOUSER_FatalError(ERROFL_SDO);
          }
          return 1;
        }
      }
      MCO_SendSDOAbort(SDO_ABORT_UNSUPPORTED);
      return 0;
    }
#endif // REBOOT_FLAG_ADR


#if USE_SYNC
    // dynamic access to [1005]
    // access to [1005,00] - SYNC COB-ID
    if ((index == 0x1005) && (subindex == 0x00))
    {
      // read command
      if (cmd == 0x40)
      {
        // expedited, 4 bytes of data
        gTxSDO.BUF[0] = 0x43;
#if CAN_ID_SIZE == 32
        GEN_WR32(PIACC_NONE, &(gTxSDO.BUF[4]), gMCOConfig.SYNC_id);
#else
        GEN_WR16(PIACC_NONE, &(gTxSDO.BUF[4]), (gMCOConfig.SYNC_id & ~COBID_OPT_MASK));
        GEN_WR16(PIACC_NONE, &(gTxSDO.BUF[6]), (gMCOConfig.SYNC_id & COBID_OPT_MASK));
#endif
        if (!MCOHW_PushMessage(&gTxSDO))
        {
          MCOUSER_FatalError(ERROFL_SDO);
        }
        return 1;
      }
      // write access
      if (*pData == (1<<5) + 3)
      { // expedited write command with 4 bytes of data
        // Retrieve new COB-ID
        // conformance test: verify range
        if (((pData[6] > 0) || IS_CANID_RESTRICTED((((uint16_t)pData[5]) << 8) + pData[4])))
        { // illegal value
          MCO_SendSDOAbort(SDO_ABORT_VALUE_HIGH);
          return 0;
        }
#if CAN_ID_SIZE == 32
        gMCOConfig.SYNC_id =  GEN_RD32(PIACC_NONE, &(pData[4]));
#else
        gMCOConfig.SYNC_id = pData[7] & 0xE0; // get cfg bits
        gMCOConfig.SYNC_id <<= 8;
        gMCOConfig.SYNC_id |= (GEN_RD16(PIACC_NONE, &(pData[4])) & ~COBID_OPT_MASK);
#endif
        (void)MCO_ApplySystemEntry(index, subindex, gMCOConfig.SYNC_id);
        // Set new CAN receive filter
        MCOHW_SetCANFilter(gMCOConfig.SYNC_id);
        // write response
        gTxSDO.BUF[0] = 0x60;
        if (!MCOHW_PushMessage(&gTxSDO))
        {
          MCOUSER_FatalError(ERROFL_SDO);
        }
        return 1;
      }
      // if we reach here, wrong write length
      // MCO_SendSDOAbort(SDO_ABORT_UNSUPPORTED);
      // error above not accepted by conformance test
      MCO_SendSDOAbort(SDO_ABORT_GENERAL);
      return 0;
    }
    // access to [1019,00] - SYNC counter overflow
    if ((index == 0x1019) && (subindex == 0x00))
    {
      // read command
      if (cmd == 0x40)
      {
        // expedited, 1 byte of data
        gTxSDO.BUF[0] = 0x4F;
        gTxSDO.BUF[4] = gMCOConfig.SYNC_cntovr;
        if (!MCOHW_PushMessage(&gTxSDO))
        {
          MCOUSER_FatalError(ERROFL_SDO);
        }
        return 1;
      }
      // expedited write command with 1 byte of data
      if (*pData == 0x2F)
      {
        if ((gMCOConfig.SYNC_id & COBID_RTR) != 0) // here RTR bit controls producer enabled
        { // SYNC producer is enabled, no change allowed in this state
          MCO_SendSDOAbort(SDO_ABORT_NOTRANSFERCTRL);
          return 0;
        }
        else
        {
          gMCOConfig.SYNC_cntovr = pData[4];
          (void)MCO_ApplySystemEntry(index, subindex, gMCOConfig.SYNC_cntovr);
          // write response
          gTxSDO.BUF[0] = 0x60;
          if (!MCOHW_PushMessage(&gTxSDO))
          {
            MCOUSER_FatalError(ERROFL_SDO);
          }
          return 1;
        }
      }
      // if we reach here, wrong write length
      // MCO_SendSDOAbort(SDO_ABORT_UNSUPPORTED);
      // error above not accepted by conformance test
      MCO_SendSDOAbort(SDO_ABORT_GENERAL);
      return 0;
    }
#if USE_SYNC_PRODUCER
    // dynamic access to [1006]
    // access to [1006,00] - SYNC cycle time
    if ((index == 0x1006) && (subindex == 0x00))
    {
      // read command
      if (cmd == 0x40)
      {
        // expedited, 4 bytes of data
        gTxSDO.BUF[0] = 0x43;
        GEN_WR32(PIACC_NONE, &(gTxSDO.BUF[4]),gMCOConfig.SYNC_cycle);
        if (!MCOHW_PushMessage(&gTxSDO))
        {
          MCOUSER_FatalError(ERROFL_SDO);
        }
        return 1;
      }
      // write access
      if (*pData == (1<<5) + 3)
      { // expedited write command with 4 bytes of data
        gMCOConfig.SYNC_cycle = GEN_RD32(PIACC_NONE, &(pData[4]));
        (void)MCO_ApplySystemEntry(index, subindex, gMCOConfig.SYNC_cycle);
        // write response
        gTxSDO.BUF[0] = 0x60;
        if (!MCOHW_PushMessage(&gTxSDO))
        {
          MCOUSER_FatalError(ERROFL_SDO);
        }
        return 1;
      }
      // if we reach here, wrong write length
      // MCO_SendSDOAbort(SDO_ABORT_UNSUPPORTED);
      // error above not accepted by conformance test
      MCO_SendSDOAbort(SDO_ABORT_GENERAL);
      return 0;
    }
#endif // USE_SYNC_PRODUCER
#endif // USE_SYNC


#if USE_EMCY
    // dynamic access to [1014]
    // access to [1014,00] - EMCY COB-ID
    if ((index == 0x1014) && (subindex == 0x00))
    {
      // read command
      if (cmd == 0x40)
      {
        // expedited, 4 bytes of data
        gTxSDO.BUF[0] = 0x43;
        gTxSDO.BUF[4] = (uint8_t) 0x80 + MY_NODE_ID;
        if (!MCOHW_PushMessage(&gTxSDO))
        {
          MCOUSER_FatalError(ERROFL_SDO);
        }
        return 1;
      }
      // fail on write access
      MCO_SendSDOAbort(SDO_ABORT_UNSUPPORTED);
      return 0;
    }
    // access to [1015,00] - EMCY Inhibit Time
    if ((index == 0x1015) && (subindex == 0x00))
    {
      // read command
      if (cmd == 0x40)
      {
        // expedited, 2 bytes of data
        buf32 = gEF.emcy_inhibit * 10; // internally stored in ms, reported in 100us
        gTxSDO.BUF[0] = 0x4B;
        gTxSDO.BUF[4] = (uint8_t) buf32;
        gTxSDO.BUF[5] = (uint8_t) (buf32 >> 8);
        if (!MCOHW_PushMessage(&gTxSDO))
        {
          MCOUSER_FatalError(ERROFL_SDO);
        }
        return 1;
      }
      // expedited write command with 2 bytes of data
      if (*pData == 0x2B)
      {
        buf32 = pData[5];
        buf32 = (buf32 << 8) + pData[4];
        gEF.emcy_inhibit = (buf32 + 9) / 10; // passed in 100us, saved in ms
        // reset timestamp for immediate transmission
        gEF.emcy_timestamp = MCOHW_GetTime();
        // write response
        gTxSDO.BUF[0] = 0x60;
        if (!MCOHW_PushMessage(&gTxSDO))
        {
          MCOUSER_FatalError(ERROFL_SDO);
        }
        return 1;
      }
      // expedited write command with 4 bytes of data or 1 byte of data
      // Needed to pass conformance test: Abort code different for this case
      else if ((*pData == 0x23) || (*pData == 0x2F))
      {
        MCO_SendSDOAbort(SDO_ABORT_TYPEMISMATCH);
      }
      MCO_SendSDOAbort(SDO_ABORT_UNSUPPORTED);
      return 0;
    }
#if ERROR_FIELD_SIZE > 0
    // access to [1003,xx] - Error Field (History)
    if (index == 0x1003)
    {
      // read command
      if (cmd == 0x40)
      {
        if (subindex > ERROR_FIELD_SIZE)
        {
          MCO_SendSDOAbort(SDO_ABORT_UNKNOWNSUB);
          return 0;
        }
        buf32 = MCOP_ErrField_Get(subindex, TRUE, NULL);
        if (buf32 != 0xFFFFFFFF)
        {
          if (subindex == 0)
          { // expedited, 1 byte of data
            gTxSDO.BUF[0] = 0x4F;
            gTxSDO.BUF[4] = (uint8_t)buf32;
          }
          else
          { // expedited, 4 bytes of data
            gTxSDO.BUF[0] = 0x43;
            gTxSDO.BUF[4] = (uint8_t) buf32;
            gTxSDO.BUF[5] = (uint8_t)(buf32 >> 8);
            gTxSDO.BUF[6] = (uint8_t)(buf32 >> 16);
            gTxSDO.BUF[7] = (uint8_t)(buf32 >> 24);
          }
          if (!MCOHW_PushMessage(&gTxSDO))
          {
            MCOUSER_FatalError(ERROFL_SDO);
          }
          return 1;
        }
        else
        {
          MCO_SendSDOAbort(SDO_ABORT_UNSUPPORTED);
          return 0;
        }
      }
      else if (pData[0] == 0x2F)
      { // write command
        if (subindex == 0)
        { // flush error history
          if (pData[4] != 0)
          { // only write of 0 allowed
            MCO_SendSDOAbort(SDO_ABORT_VALUE_HIGH);
            return 0;
          }
          MCOP_ErrField_Flush();
          // write response
          gTxSDO.BUF[0] = 0x60;
          if (!MCOHW_PushMessage(&gTxSDO))
          {
            MCOUSER_FatalError(ERROFL_SDO);
          }
          return 1;
        }
        MCO_SendSDOAbort(SDO_ABORT_UNSUPPORTED);
        return 0;
      }
      // if we reach here, fail
      // for conformance test produce generic error
      MCO_SendSDOAbort(SDO_ABORT_GENERAL);
      // MCO_SendSDOAbort(SDO_ABORT_UNSUPPORTED);
      return 0;
    }
#endif // ERROR_FIELD_SIZE > 0
#endif // USE_EMCY

#if (NR_OF_HB_CONSUMER > 0)
#if DYNAMIC_HB_CONSUMER
    // dynamic read/write accesses
    // access to [1016,xx] - heartbeat consumer time
    if (index == 0x1016)
    {
      if (cmd == 0x40)
      { // read command
        if (subindex == 0)
        { // expedited response, 1 byte data
          gTxSDO.BUF[0] = 0x4F;
          gTxSDO.BUF[4] = NR_OF_HB_CONSUMER;
          if (!MCOHW_PushMessage(&gTxSDO))
          {
            MCOUSER_FatalError(ERROFL_SDO);
          }
          return 1;
        }
        else if (subindex > NR_OF_HB_CONSUMER)
        {
          MCO_SendSDOAbort(SDO_ABORT_UNKNOWNSUB);
          return 0;
        }
        else
        { // regular access
          subindex--; // now can directly be used as array pointer
          // expedited, 4 bytes of data
          gTxSDO.BUF[0] = 0x43;
#if ! MGR_MONITOR_ALL_NODES
          gTxSDO.BUF[4] = (uint8_t) gHBCons[subindex].time;
          gTxSDO.BUF[5] = (uint8_t) (gHBCons[subindex].time >> 8);
          gTxSDO.BUF[6] = (uint8_t) gHBCons[subindex].can_id;
          gTxSDO.BUF[7] = (uint8_t) 0;
#else
          gTxSDO.BUF[4] = (uint8_t) gNodeList[subindex].hb_time;
          gTxSDO.BUF[5] = (uint8_t) (gNodeList[subindex].hb_time >> 8);
          gTxSDO.BUF[6] = (uint8_t) subindex+1;
#endif // MGR_MONITOR_ALL_NODES
          if (!MCOHW_PushMessage(&gTxSDO))
          {
            MCOUSER_FatalError(ERROFL_SDO);
          }
          return 1;
        }
      }
      // expedited write command with 4 bytes of data
#if ! MGR_MONITOR_ALL_NODES
      if ( (*pData == 0x23) &&
           (pData[6] != MY_NODE_ID) && // not our own node ID
           (pData[6] < 128) && // not an illegal node ID
           // removed to work with PCOMPDS V1.23
           // re-added to work with Conformance Test
           (!MCOP_IsHBMonitored(subindex,pData[6]))
         )
      { // only if requested node ID is unequal to own node id and not yet used
        MCOP_InitHBConsumer(subindex,pData[6],((uint16_t)(pData[5])<< 8) + pData[4]);
        // write response
        gTxSDO.BUF[0] = 0x60;
        if (!MCOHW_PushMessage(&gTxSDO))
        {
          MCOUSER_FatalError(ERROFL_SDO);
        }
        return 1;
      }
#else
      if (*pData == 0x23)
      { // write access
        MCO_SendSDOAbort(SDO_ABORT_READONLY);
        return 0;
      }
#endif // MGR_MONITOR_ALL_NODES
      // if we reach here probably conformance test tries something...
      if (subindex == 0)
      {
        MCO_SendSDOAbort(SDO_ABORT_READONLY);
      }
      else
      {
        if (index == 0x1016)
        { // Abort code required by Conformance Test HB 04
          MCO_SendSDOAbort(SDO_ABORT_PARAMETER);
        }
        else
        { // Abort code required by Conformance Test SDO 09
          MCO_SendSDOAbort(SDO_ABORT_TYPEMISMATCH);
        }
      }
      return 0;
    }
#endif // DYNAMIC_HB_CONSUMER
#endif // (NR_OF_HB_CONSUMER > 0)

#if USE_STORE_PARAMETERS
    // access to [1010,xx] or [1011,xx] - Store Parameters, Restore
    if ( ((index == 0x1010) || (index == 0x1011)) && (subindex <= NROF_STORE_PARAMETERS) )
    {
      // read command
      if (cmd == 0x40)
      {
        if (subindex == 0)
        {
          // expedited, 1 byte of data
          gTxSDO.BUF[0] = 0x4F;
          gTxSDO.BUF[4] = NROF_STORE_PARAMETERS;
        }
        else
        { // access supported
          // expedited, 4 bytes of data
          gTxSDO.BUF[0] = 0x43;
          gTxSDO.BUF[4] = 1;
        }
        if (!MCOHW_PushMessage(&gTxSDO))
        {
          MCOUSER_FatalError(ERROFL_SDO);
        }
        return 1;
      }
      // expedited write command with 4 bytes of data
      if (*pData == 0x23)
      {
        if(
// ===== BEGIN COMPATIBILITY SECTION ==================
// This section allows to use both "save" resp. "evas"
// as well as "load" resp. "daol". For strict CiA301
// behaviour, disable this section. There are different
// interpretations and implementations in common
// CANopen tools, so allowing both options is the safe
// choice.
           ( (index == 0x1010) &&
             (pData[4] == 'e') && (pData[5] == 'v') &&
             (pData[6] == 'a') && (pData[7] == 's')
           )
           ||
           ( (index == 0x1011) &&
             (pData[4] == 'd') && (pData[5] == 'a') &&
             (pData[6] == 'o') && (pData[7] == 'l')
           )
           ||
// ===== END COMPATIBILITY SECTION ====================
           ( (index == 0x1010) &&
             (pData[4] == 's') && (pData[5] == 'a') &&
             (pData[6] == 'v') && (pData[7] == 'e')
           )
           ||
           ( (index == 0x1011) &&
             (pData[4] == 'l') && (pData[5] == 'o') &&
             (pData[6] == 'a') && (pData[7] == 'd')
           )
          )
        { // write of "save" to 0x1010 or "load" to 0x1011
          if (MCOSP_StoreParameters(index,subindex))
          { // Parameters stored OK.
            // write response
            gTxSDO.BUF[0] = 0x60;
            if (!MCOHW_PushMessage(&gTxSDO))
            {
              MCOUSER_FatalError(ERROFL_SDO);
            }
            return 1;
          }
          else
          {
            MCO_SendSDOAbort(SDO_ABORT_TRANSFER);
            return 0;
          }
        }
        else
        { // wrong code
          MCO_SendSDOAbort(SDO_ABORT_TRANSFER);
          return 0;
        }
      }
      else
      { // wrong size or access
        MCO_SendSDOAbort(SDO_ABORT_TYPEMISMATCH);
        return 0;
      }
    }
#endif // USE_STORE_PARAMETERS

#if (INDEX_FOR_DIAGNOSTICS != 0)
    if (index == INDEX_FOR_DIAGNOSTICS)
    {
      // read command
      if (cmd == 0x40)
      { // default response bytes
        if (subindex == 0)
        { // return 22
          gTxSDO.BUF[0] = 0x4F;
          gTxSDO.BUF[4] = 22;
        }
        else if (subindex <= 7)
        { // expedited, 4 bytes
          gTxSDO.BUF[0] = 0x43;
          switch(subindex)
          {
            case 1: // Identify
              gTxSDO.BUF[4] = 0x00;
              gTxSDO.BUF[5] = 0x45;
              gTxSDO.BUF[6] = 0x53;
              gTxSDO.BUF[7] = 0x41;
              break;
            case 2: // Version
#ifdef REV
              gTxSDO.BUF[4] = (uint8_t) REV;
              gTxSDO.BUF[5] = (uint8_t) (REV >> 8);
#else
              gTxSDO.BUF[4] = 0;
              gTxSDO.BUF[5] = 0;
#endif
              gTxSDO.BUF[6] = 0x15;
              gTxSDO.BUF[7] = 6;
              break;
            case 3: // Functionality
#if USE_EVENT_TIME == 1
                gTxSDO.BUF[4] |= 0x10;
#endif
#if USE_INHIBIT_TIME == 1
                gTxSDO.BUF[4] |= 0x20;
#endif
#if USE_SYNC == 1
                gTxSDO.BUF[4] |= 0x40;
#endif
#if USE_EXTENDED_SDO == 1
                gTxSDO.BUF[5] |= 0x01;
#endif
#if USE_BLOCKED_SDO == 1
                gTxSDO.BUF[5] |= 0x02;
#endif
              break;
            case 4: // Status
              break;
#if (TXFIFOSIZE > 0)
            case 5: // TxFIFO
              break;
#endif
#if (RXFIFOSIZE > 0)
            case 6: // RxFIFO
              break;
#endif
#if (MGRFIFOSIZE > 0)
            case 7: // MgrRxFIFO
              break;
#endif
          }
        }
        else if (subindex <= 22)
        { // expedited, 2 bytes
          gTxSDO.BUF[0] = 0x4B;
          switch(subindex)
          {
            case 8: // ProcTickPerSecCur
              gTxSDO.BUF[4] = (uint8_t) gMCODiag.ProcTickPerSecCur;
              gTxSDO.BUF[5] = (uint8_t) (gMCODiag.ProcTickPerSecCur >> 8);
              break;
            case 9: // ProcTickPerSecMin
              gTxSDO.BUF[4] = (uint8_t) gMCODiag.ProcTickPerSecMin;
              gTxSDO.BUF[5] = (uint8_t) (gMCODiag.ProcTickPerSecMin >> 8);
              break;
            case 10: // ProcTickPerSecMax
              gTxSDO.BUF[4] = (uint8_t) gMCODiag.ProcTickPerSecMax;
              gTxSDO.BUF[5] = (uint8_t) (gMCODiag.ProcTickPerSecMax >> 8);
              break;
            case 11: // ProcTickBurstMax
              gTxSDO.BUF[4] = (uint8_t) gMCODiag.ProcTickBurstMax;
              gTxSDO.BUF[5] = (uint8_t) (gMCODiag.ProcTickBurstMax >> 8);
              break;
            case 12: // ProcRxPerSecCur
              gTxSDO.BUF[4] = (uint8_t) gMCODiag.ProcRxPerSecCur;
              gTxSDO.BUF[5] = (uint8_t) (gMCODiag.ProcRxPerSecCur >> 8);
              break;
            case 13: // ProcRxPerSecMin
              gTxSDO.BUF[4] = (uint8_t) gMCODiag.ProcRxPerSecMin;
              gTxSDO.BUF[5] = (uint8_t) (gMCODiag.ProcRxPerSecMin >> 8);
              break;
            case 14: // ProcRxPerSecMax
              gTxSDO.BUF[4] = (uint8_t) gMCODiag.ProcRxPerSecMax;
              gTxSDO.BUF[5] = (uint8_t) (gMCODiag.ProcRxPerSecMax >> 8);
              break;
#if MGR_MONITOR_ALL_NODES
            case 16: // ProcMgrTickPerSecCur
              gTxSDO.BUF[4] = (uint8_t) gMCODiag.ProcMgrTickPerSecCur;
              gTxSDO.BUF[5] = (uint8_t) (gMCODiag.ProcMgrTickPerSecCur >> 8);
              break;
            case 17: // ProcMgrTickPerSecMin
              gTxSDO.BUF[4] = (uint8_t) gMCODiag.ProcMgrTickPerSecMin;
              gTxSDO.BUF[5] = (uint8_t) (gMCODiag.ProcMgrTickPerSecMin >> 8);
              break;
            case 18: // ProcMgrTickPerSecMax
              gTxSDO.BUF[4] = (uint8_t) gMCODiag.ProcMgrTickPerSecMax;
              gTxSDO.BUF[5] = (uint8_t) (gMCODiag.ProcMgrTickPerSecMax >> 8);
              break;
            case 19: // ProcMgrTickBurstMax
              gTxSDO.BUF[4] = (uint8_t) gMCODiag.ProcMgrTickBurstMax;
              gTxSDO.BUF[5] = (uint8_t) (gMCODiag.ProcMgrTickBurstMax >> 8);
              break;
            case 20: // ProcMgrRxPerSecCur
              gTxSDO.BUF[4] = (uint8_t) gMCODiag.ProcMgrRxPerSecCur;
              gTxSDO.BUF[5] = (uint8_t) (gMCODiag.ProcMgrRxPerSecCur >> 8);
              break;
            case 21: // ProcMgrRxPerSecMin
              gTxSDO.BUF[4] = (uint8_t) gMCODiag.ProcMgrRxPerSecMin;
              gTxSDO.BUF[5] = (uint8_t) (gMCODiag.ProcMgrRxPerSecMin >> 8);
              break;
            case 22: // ProcMgrRxPerSecMax
              gTxSDO.BUF[4] = (uint8_t) gMCODiag.ProcMgrRxPerSecMax;
              gTxSDO.BUF[5] = (uint8_t) (gMCODiag.ProcMgrRxPerSecMax >> 8);
              break;
#endif
            default: // not used, return zero
              MCO_SendSDOAbort(SDO_ABORT_UNKNOWNSUB);
              return 0;
          }
        }
        else
        { // unknown subindex
          MCO_SendSDOAbort(SDO_ABORT_UNKNOWNSUB);
          return 0;
        }
        // gTxSDO all set with current data
        if (!MCOHW_PushMessage(&gTxSDO))
        {
          MCOUSER_FatalError(ERROFL_SDO);
        }
        return 1;
      }
      else
      { // write cmd or unsupported command
        MCO_SendSDOAbort(SDO_ABORT_GENERAL);
        return 0;
      }
    }
#endif // INDEX_FOR_DIAGNOSTICS

    // deal with access to process image area
    found = MCO_SearchODProcTable(index,subindex);
    // entry found?
    if (found != 0xFFFF)
    {
      pOD = OD_ProcTablePtr(found);
      offset = pOD->off_hi;
      offset <<= 8;
      offset +=  pOD->off_lo;
      // read command?
      if (cmd == 0x40)
      {
        // read allowed?
        if ((pOD->len & ODRD) != 0) // Check if RD bit is set
        {
#if USECB_SDO_RD_PI
          // Application call back, SDO read access to process image
          sdoreturn = MCOUSER_SDORdPI(0,index,subindex,offset,pOD->len & 0x07);
          if (sdoreturn != 0)
          { // access not granted
            MCO_SendSDOAbort(sdoreturn);
            return 0;
          }
#endif // USECB_SDO_RD_PI

          PI_READ(PIACC_SDO,offset,buf,pOD->len & 0x07);

#if USECB_SDO_RD_AFTER
          MCOUSER_SDORdAft(0,index,subindex,offset,pOD->len & 0x07);
#endif // USECB_SDO_RD_AFTER

          return MCO_ReplyWith(buf,(pOD->len & 0x07));
        }
        // read not allowed
        else
        {
          MCO_SendSDOAbort(SDO_ABORT_UNSUPPORTED);
          return 0;
        }
      }
      // write command?
      else
      {
        // is WR bit set? - then write allowed
        if ((pOD->len & ODWR) != 0)
        {
          // for writes: Bits 2 and 3 of *pData are number of bytes without data
          len = 4 - ((*pData & 0x0C) >> 2);
          // is length ok?
          if (len != (pOD->len & 0x07))
          {
            MCO_SendSDOAbort(SDO_ABORT_TYPEMISMATCH);
            return 0;
          }

#if USECB_SDO_WR_PI
          // Application call back, SDO write access to process image
          sdoreturn = MCOUSER_SDOWrPI(0,index,subindex,offset,&(gRxCAN.BUF[4]),len);
          if (sdoreturn != 0)
          { // access not granted
            MCO_SendSDOAbort(sdoreturn);
            return 0;
          }
#endif // USECB_SDO_WR_PI

          // retrieve data from SDO write request and copy into process image
          PI_WRITE(PIACC_SDO,offset,&gRxCAN.BUF[4],len);

#if USE_CiA447
          MCOUSER_NodeSpecificSDOWrite(PROFILE_GetSDOFromNode(),index,subindex,offset,len);
#endif

#if USECB_SDO_WR_AFTER
          MCOUSER_SDOWrAft(0,index,subindex,offset,len);
#endif // USECB_SDO_WR_AFTER

#if USECB_ODDATARECEIVED
          RTOS_LOCK_PI(PIACC_APP,PISECT_ALL);
          MCOUSER_ODData(0,index,subindex,&(gProcImg[offset]),len);
          RTOS_UNLOCK_PI(PIACC_APP,PISECT_ALL);
#endif // USECB_ODDATARECEIVED

          // write response
          gTxSDO.BUF[0] = 0x60;
          if (!MCOHW_PushMessage(&gTxSDO))
          {
            MCOUSER_FatalError(ERROFL_SDO);
          }
          return 1;
        }
        // write not allowed
        else
        {
          MCO_SendSDOAbort(SDO_ABORT_UNSUPPORTED);
          return 0;
        }
      }
    }

    // search table with constants
    found = MCO_SearchOD(index,subindex);
    // entry found?
    if (found < 0xFFFF)
    {
      // read command?
      if (cmd == 0x40)
      {
        MEM_CPY_FAR(&(gTxSDO.BUF[0]),OD_SDOResponseTablePtr(found<<3),8);
        if (!MCOHW_PushMessage(&gTxSDO))
        {
          MCOUSER_FatalError(ERROFL_SDO);
        }
        return 1;
      }
      // write command
      MCO_SendSDOAbort(SDO_ABORT_READONLY);
      return 0;
    }
    // Error and status byte
    if ((index == 0x1001) && (subindex == 0x00))
    {
      // read command
      if (cmd == 0x40)
      {
        // expedited, 1 byte of data
        gTxSDO.BUF[0] = 0x4F;
        gTxSDO.BUF[4] = gMCOConfig.error_register;
        if (!MCOHW_PushMessage(&gTxSDO))
        {
          MCOUSER_FatalError(ERROFL_SDO);
        }
        return 1;
      }
      // write command
      MCO_SendSDOAbort(SDO_ABORT_READONLY);
      return 0;
    }

    // Requested OD entry not found
    // can not use more specific error code here, as we don't know if index or subindex failed
    MCO_SendSDOAbort(SDO_ABORT_GENERAL);
    return 0;
  }
  // ignore abort received - all other produce an error
  if (cmd != 0x80)
  {
    MCO_SendSDOAbort(SDO_ABORT_UNKNOWN_COMMAND);
    return 0;
  }
#if USE_EXTENDED_SDO
  else
  { // Inform extended handling about the abort
    XSDO_Abort(sdoserv);
  }
#endif // USE_EXTENDED_SDO
  return 1;
}
#endif // !USE_CANOPEN_FD


#if NR_OF_RPDOS > 0
/**************************************************************************
DOES:    Called when going into the operational mode.
         Inits all RPDO Filters
RETURNS: nothing
**************************************************************************/
static void MCO_PrepareRPDOs (
    void
  )
{
uint16_t i;

  i = 0;
  // prepare all RPDO filters for reception
  while (i < gMCOConfig.nrRPDOs)
  {
    if ((gMCOConfig.error_code & 0x80) == 0)
    { // RPDO filters not yet set
      if ((gRPDOConfig[i].CANID & COBID_DISABLED) == 0)
      {
        if (!MCOHW_SetCANFilter(~COBID_OPT_MASK & gRPDOConfig[i].CANID))
        {
          MCOUSER_FatalError(ERRFT_RXFLTP);
        }
      }
    }
    i++;
  }
  gMCOConfig.error_code |= 0x80; // Signal that RPDO filters are now set
}
#endif // NR_OF_RPDOS > 0


#if NR_OF_TPDOS > 0
/**************************************************************************
DOES:    Called when going into the operational mode.
         Prepares all TPDOs for operational.
RETURNS: nothing
**************************************************************************/
static void MCO_PrepareTPDOs (
    void
  )
{
uint16_t i;

  i = 0;
  // prepare all TPDOs for transmission
  while (i < gMCOConfig.nrTPDOs)
  {
    // Copy current process data
    PDO_TXCOPY(i,( uint8_t *)&(gTPDOConfig[i].CANmsg.BUF[0]));
#if USE_EVENT_TIME
    // Reset event timer for immediate transmission
    gTPDOConfig[i].event_timestamp = MCOHW_GetTime() - 2;
#endif
#if USE_INHIBIT_TIME
    gTPDOConfig[i].inhibit_status = INHITIM_RUNNING_TRIGGERED; // Mark as ready for transmission
    // Reset inhibit timer for immediate transmission
    gTPDOConfig[i].inhibit_timestamp = MCOHW_GetTime() - 2;
#endif
#if USE_SYNC
    gTPDOConfig[i].SYNCcnt = gTPDOConfig[i].TType;
    if (gTPDOConfig[i].TType == 0)
    { // set to 241 for very first call
      gTPDOConfig[i].TType = 241;
    }
    if (gTPDOConfig[i].SYNCmatch != 0)
    { // SYNC with counter is used, ensure mode is enabled
      if (gMCOConfig.SYNC_cntovr == 0)
      {
        gMCOConfig.SYNC_cntovr = 1;
      }
    }
#endif
    i++;
  }
  // ensure that MCO_ProcessStack starts with TPDO1
  gTPDONr = NR_OF_TPDOS;
}
#endif // NR_OF_TPDOS > 0




#if NR_OF_RPDOS > 0
/**************************************************************************
DOES:    Handles Receive PDOs
RETURNS: FALSE, if RPDO not processed
         TRUE, if RPDO processed
**************************************************************************/
static uint8_t MCO_HandleRPDO (
  CAN_MSG *pRPDO
  )
{
uint8_t retval = FALSE;
uint16_t i; // loop variable
#if USECB_ODDATARECEIVED
uint16_t map; // offset into SDOResponseTable, RPDO mapping
uint16_t off; // offset into Process Image to RPDO data
uint8_t MEM_CONST *pSDO; // pointer into SDO response table
uint8_t cnt;
#endif // USECB_ODDATARECEIVED


  if (MY_NMT_STATE == NMTSTATE_OP)
  { // node is operational

#if USE_PROFILE_RPDO
    pRPDO->ID = PROFILE_ExtHandleRPDO(pRPDO->ID);
#endif

    i = 0;
    // loop through RPDOs
    while (i < gMCOConfig.nrRPDOs)
    {
      // is this one of our RPDOs and is not disabled
      if ((pRPDO->ID == (~COBID_OPT_MASK & gRPDOConfig[i].CANID)) &&
        ((gRPDOConfig[i].CANID & COBID_DISABLED) == 0)
         )
      {
#if USE_EMCY
// Only supported with MicroCANopen Plus
        // if (gRxCAN.LEN != gRPDOConfig[i].len)
        // For backwards compatibility, allow PDOs that are too long
        // Only produce emergency for PDOs that are too short
        if (gRxCAN.LEN < gRPDOConfig[i].len)
        { // Length of CAN message does not match PDO len
#if ERROR_FIELD_SIZE > 0
          MCOP_ErrField_AddUpdate(MAKE_ERRCODE32(EMCY_PDO_LEN,gRPDOConfig[i].PDONr,gRPDOConfig[i].len)
#if defined(USE_CANOPEN_FD) && USE_CANOPEN_FD    // CANopen FD
            ,
            ERST_STATE_OCC
#endif
          );
#endif
          if (!(ARRAY16_GETBIT(gEF.active_rpdo,i)))
          { // This was not yet reported
            // Set active bit for this PDO
            ARRAY16_SETBIT(gEF.active_rpdo,i);
#if defined(USECB_EMCY) && USECB_EMCY
            if (0 == MCOUSER_EMCY(FALSE, EMCY_PDO_LEN, (uint8_t)(gRPDOConfig[i].PDONr & 0xFF), (uint8_t)((gRPDOConfig[i].PDONr >> 8) & 0xFF), gRPDOConfig[i].len, gRxCAN.LEN, 0
#if defined(USE_CANOPEN_FD) && USE_CANOPEN_FD    // CANopen FD
              ,
              0,     // dev_num - logical device number
              COFD_BASE_SPEC >> 20, // spec_num, CiA specification number
              ERST_STATE_OCC | ERST_PRIO(4),  // status - priority=4, recoverable, error occurred
              0,     // time_lo - timestamp bits 0-31, not supported
              0      // time_hi - timestamp bits 32-47, not supported
#endif // CANopen FD
            ))
#endif // USECB_EMCY
          {
              (void)MCOP_PushEMCY(EMCY_PDO_LEN, (uint8_t)(gRPDOConfig[i].PDONr & 0xFF), (uint8_t)((gRPDOConfig[i].PDONr >> 8) & 0xFF), gRPDOConfig[i].len, gRxCAN.LEN, 0
#if defined(USE_CANOPEN_FD) && USE_CANOPEN_FD    // CANopen FD
                ,
                0,     // dev_num - logical device number
                COFD_BASE_SPEC >> 20, // spec_num, CiA specification number
               ERST_STATE_OCC | ERST_PRIO(4),  // status - priority=4, recoverable, error occurred
                0,     // time_lo - timestamp bits 0-31, not supported
                0      // time_hi - timestamp bits 32-47, not supported
#endif // CANopen FD
              );
            }
          }
        }
        else if (gRxCAN.LEN == LAYER_MSG_LENGTH(gRPDOConfig[i].len))
        { // length matches, check for previous err
#if ERROR_FIELD_SIZE > 0
          MCOP_ErrField_Remove(MAKE_ERRCODE32(EMCY_PDO_LEN, gRPDOConfig[i].PDONr, gRPDOConfig[i].len));
#endif
          if (ARRAY16_GETBIT(gEF.active_rpdo,i))
          { // This was previously reported as error
            // Clear active bit for this PDO
            ARRAY16_CLRBIT(gEF.active_rpdo,i);
#if defined(USECB_EMCY) && USECB_EMCY
            if (0 == MCOUSER_EMCY(TRUE, EMCY_PDO_LEN, (uint8_t)(gRPDOConfig[i].PDONr & 0xFF), (uint8_t)((gRPDOConfig[i].PDONr >> 8) & 0xFF), gRPDOConfig[i].len, gRxCAN.LEN, 0
#if defined(USE_CANOPEN_FD) && USE_CANOPEN_FD    // CANopen FD
              ,
              0,     // dev_num - logical device number
              COFD_BASE_SPEC >> 20, // spec_num, CiA specification number
              ERST_STATE_OCC | ERST_PRIO(4),  // status - priority=4, recoverable, error occurred
              0,     // time_lo - timestamp bits 0-31, not supported
              0      // time_hi - timestamp bits 32-47, not supported
#endif // CANopen FD
            ))
#endif // USECB_EMCY
            {
              if (MCOP_IsNoEMCYactive())
              { // only call, if no further EMCY is pending
                (void)MCOP_PushEMCY(EMCY_NO_ERROR, 0, 0, 0, 0, 0
#if defined(USE_CANOPEN_FD) && USE_CANOPEN_FD    // CANopen FD
                  ,
                  0,     // dev_num - logical device number
                  COFD_BASE_SPEC >> 20, // spec_num, CiA specification number
                  ERST_STATE_OCC | ERST_PRIO(4),  // status - priority=4, recoverable, error occurred
                  0,     // time_lo - timestamp bits 0-31, not supported
                  0      // time_hi - timestamp bits 32-47, not supported
#endif // CANopen FD
                );
              }
            }
          }
        }
#endif // USE_EMCY

        // For backwards compatibility, allow PDOs that are too long
        if (gRxCAN.LEN >= gRPDOConfig[i].len)
        {
          if (gRPDOConfig[i].TType >= 254)
          { // This PDO is not synced
#if USECB_ODDATARECEIVED
            // Process RPDO mapping
#if USE_DYNAMIC_PDO_MAPPING
#error USECB_ODDATARECEIVED currently not available with USE_DYNAMIC_PDO_MAPPING
#else
            // copy data from RPDO to process image
            PDO_RXCOPY(i,&(pRPDO->BUF[0]));

            map = gRPDOConfig[i].map; // offset to mapping entries
            off = 0; // start with offset zero
            pSDO = OD_SDOResponseTablePtr(0);
            cnt = 1;
            while((pSDO[map+3] == cnt) && (cnt <= 8))
            { // while Subindex is not zero
              RTOS_LOCK_PI(PIACC_APP,PISECT_PDO);
              MCOUSER_ODData(0,(((uint16_t)(pSDO[map+7]))<<8)+pSDO[map+6],pSDO[map+5],&(pRPDO->BUF[0+off]),pSDO[map+4]>>3);
              RTOS_UNLOCK_PI(PIACC_APP,PISECT_PDO);
              off += (pSDO[map+4]>>3); // next mapped OD entry
              map += 8; // next mapping entry
              cnt++;
            }
#endif // USE_DYNAMIC_PDO_MAPPING

#else // USECB_ODDATARECEIVED
            // copy data from RPDO to process image
            PDO_RXCOPY(i,&(pRPDO->BUF[0]));

#endif // USECB_ODDATARECEIVED

#if USECB_RPDORECEIVE
            MCOUSER_RPDOReceived(gRPDOConfig[i].PDONr,gRPDOConfig[i].offset,gRPDOConfig[i].len);
#endif // USECB_RPDORECEIVE

          }
#if USE_SYNC
          else if (gRPDOConfig[i].TType <= 240)
          { // This PDO is synced
            // copy data from CAN message to RPDO buffer
            MEM_CPY_FAR(&(gRPDOConfig[i].BUF[0]),&(pRPDO->BUF[0]),gRPDOConfig[i].len);
          }
#endif // USE_SYNC
          // exit the loop
          retval = TRUE;
          break;
        }
      }

      i++;
    }
  } // for all RPDOs
  return retval; // not a RPDO for us
}
#endif // NR_OF_RPDOS > 0


#if NR_OF_TPDOS > 0
/**************************************************************************
DOES:    Handles Transmit PDOs, checks only one TPDO with each call
RETURNS: FALSE, if no TPDO was sent
         TRUE, if TPDO was sent
**************************************************************************/
static uint8_t MCO_HandleTPDO (
  uint16_t TPDONr // Number of TPDO to check, as gTPDOConfig[] array index
  )
{
uint8_t retval = FALSE;

  // is the TPDO 'TPDONr' in use?
  if ((gTPDOConfig[TPDONr].CANmsg.ID != 0) && 
      ((gTPDOConfig[TPDONr].CANmsg.ID & COBID_DISABLED) == 0)
#if USE_SYNC
      && (gTPDOConfig[TPDONr].TType >= 254) // Not a synced PDO
#endif
     )
  {
#if USE_EVENT_TIME
    // does TPDO use event timer and event timer is expired? if so we need to transmit now
    if ((gTPDOConfig[TPDONr].event_time != 0) &&
        (MCOHW_IsTimeExpired(gTPDOConfig[TPDONr].event_timestamp)) )
    {
#if USECB_TPDORDY
      if (MCOUSER_TPDOReady(gTPDOConfig[TPDONr].PDONr,0))
      {
#endif
        // get data from process image and transmit
        PDO_TXCOPY(TPDONr,( uint8_t *)&(gTPDOConfig[TPDONr].CANmsg.BUF[0]));
        MCO_TransmitPDO(TPDONr);
        retval = TRUE;
#if USECB_TPDORDY
      }
#endif
    }
#endif // USE_EVENT_TIME

#if USE_INHIBIT_TIME
    // is the inihibit timer currently running?
    if (gTPDOConfig[TPDONr].inhibit_status != INHITIM_EXPIRED)
    {
      // has the inhibit time expired?
      if (MCOHW_IsTimeExpired(gTPDOConfig[TPDONr].inhibit_timestamp))
      {
        // is there a new transmit message already waiting?
        if (gTPDOConfig[TPDONr].inhibit_status == INHITIM_RUNNING_TRIGGERED)
        {
#if USECB_TPDORDY
          if (MCOUSER_TPDOReady(gTPDOConfig[TPDONr].PDONr,3))
          {
#endif
            // transmit now
            MCO_TransmitPDO(TPDONr);
            retval = TRUE;
#if USECB_TPDORDY
          }
#endif
        }
        // no new message waiting, but timer expired
        else
        {
          gTPDOConfig[TPDONr].inhibit_status = INHITIM_EXPIRED;
        }
      }
    }

    // do change-of-state detection unless we are in the inhibit time and have already triggered
    if ( (gTPDOConfig[TPDONr].inhibit_status != INHITIM_RUNNING_TRIGGERED) &&
        (gTPDOConfig[TPDONr].inhibit_time != 0) &&
        ((gTPDOConfig[TPDONr].inhibit_time < gTPDOConfig[TPDONr].event_time) || (gTPDOConfig[TPDONr].event_time == 0))
      )
    {
      // has application data changed?
      if (PDO_TXCOMP(TPDONr,&(gTPDOConfig[TPDONr].CANmsg.BUF[0])) != 0)
      {
        // has inhibit time expired?
        if (gTPDOConfig[TPDONr].inhibit_status == INHITIM_EXPIRED)
        {
#if USECB_TPDORDY
          if (MCOUSER_TPDOReady(gTPDOConfig[TPDONr].PDONr,3))
          {
#endif
            // Copy application data
            PDO_TXCOPY(TPDONr,&(gTPDOConfig[TPDONr].CANmsg.BUF[0]));
            // transmit now
            MCO_TransmitPDO(TPDONr);
            retval = TRUE;
#if USECB_TPDORDY
          }
#endif
        }
        else
        {
          // Copy application data
          PDO_TXCOPY(TPDONr,&(gTPDOConfig[TPDONr].CANmsg.BUF[0]));
          // wait for inhibit time to expire 
          gTPDOConfig[TPDONr].inhibit_status = INHITIM_RUNNING_TRIGGERED;
        }
      }
    }
#endif // USE_INHIBIT_TIME
  } // PDO active (CAN_ID != 0)
  return retval;
}
#endif // NR_OF_TPDOS > 0


#if (NR_OF_RPDOS > 0)
#if USECB_ODDATARECEIVED
/**************************************************************************
DOES:    Searches RPDO mapping parameters in SDO reponse table to determine 
         offset to first mapping parameter.
RETURNS: 0, in none found, else offset into SDO response table.
         TRUE, if TPDO was sent
**************************************************************************/
static uint16_t MCO_GetRPDOMappingOffset (
  uint16_t PDO_NR // start at 1
  )
{
  uint8_t MEM_CONST *pSDO; // pointer into SDO response table
  uint16_t map = 0;
  
  pSDO = OD_SDOResponseTablePtr(0);
  // find first entry with RPDO mapping 0x16xx where xx is PDO_NR-1
  while  (!( (pSDO[0] == 0x4F     ) &&
             (pSDO[1] == (uint8_t) (PDO_NR-1) ) &&
             (pSDO[2] == (uint8_t) ((0x1600 + (PDO_NR-1)) >> 8))
           )
         )
  {
    map += 8; // next record
    pSDO += 8;
    if (*pSDO == 0xFF)
    { // End of table, no mapping found
      MCOUSER_FatalError(ERRFT_RPMAP);
    }
  }
  if (map != 0)
  {
    // go to next entry at subindex one
    map += 8; // next record
  }
  
  return map;
}
#endif // USECB_ODDATARECEIVED
#endif

  
/**************************************************************************
PUBLIC FUNCTIONS
***************************************************************************/

/**************************************************************************
DOES:    Initializes the MicroCANopen stack
         It must be called from within MCOUSER_ResetCommunication
RETURNS: TRUE, if init OK, else FALSE (also when unconfigured and in LSS)
**************************************************************************/
uint8_t MCO_Init (
  uint16_t Bitrate,  // CAN bitrate in kbit (1000,800,500,250,125,50,25 or 10)
#if defined(USE_CANOPEN_FD) && (USE_CANOPEN_FD==1)
  uint16_t BRSBitrate,
#endif
  uint8_t Node_ID,    // CANopen node ID (1-126)
  uint16_t Heartbeat  // Heartbeat time in ms (0 for none)
  )
{
  uint32_t i;
  uint8_t retval;
  
  retval = TRUE;

  if (Bitrate == 0)
  {
#if defined(CAN_BITRATE_DCF)
    Bitrate = CAN_BITRATE_DCF; // default baud rate from EDS, if available
#else
    Bitrate = 125; // last-resort-default baud rate
#endif
  }

#if defined(USE_CANOPEN_FD) && (USE_CANOPEN_FD==1)
  if (BRSBitrate == 0)
  {
    BRSBitrate = Bitrate;
  }
#endif

  // Init the global variables
  // MY_NODE_ID is gMCOConfig.Node_ID
  if (Node_ID != 0)
  {
    MY_NODE_ID = Node_ID;
  }
  // else use previously assigned ID (for example LSS)
  gMCOConfig.error_code = 0;
  gMCOConfig.Bitrate = Bitrate;
  gMCOConfig.heartbeat_time = Heartbeat;
  gMCOConfig.error_register = 0;
  gMCOConfig.last_rxtime = MCOHW_GetTime();
#if USE_LEDS
  // Initialize LED blink control 200ms timer
  gMCOConfig.LED_timestamp = MCOHW_GetTime() + 200;
  gMCOConfig.LEDtoggle = 0;
  gMCOConfig.LEDcntR = 0;
  gMCOConfig.LEDcntE = 0;
  gMCOConfig.LEDRun = LED_OFF;
  gMCOConfig.LEDErr = LED_ON;
#endif // USE_LEDS
#if USE_SYNC
  gMCOConfig.SYNC_id = 0x80; // Default SYNC ID, enabled, no producer
  gMCOConfig.SYNC_cntovr = 0; // Default: disable counter usage
#endif // USE_SYNC
#if NR_OF_TPDOS > 0
  gMCOConfig.nrTPDOs = 0;
#endif
#if NR_OF_RPDOS > 0
  gMCOConfig.nrRPDOs = 0;
#endif

#if USE_EMCY
  // Init emergency inhibit time
  gEF.emcy_inhibit = (EMCY_INHIBIT_TIME + 9) / 10;
#if NR_OF_RPDOS > 0
  for (i = 0; i < (sizeof(gEF.active_rpdo) / sizeof(gEF.active_rpdo[0])); i++)
  {
    gEF.active_rpdo[i] = 0;
  }
#endif
#if MGR_MONITOR_ALL_NODES || (NR_OF_HB_CONSUMER > 0)
  for (i = 0; i < (sizeof(gEF.active_hbcons)/sizeof(gEF.active_hbcons[0])); i++)
  {
    gEF.active_hbcons[i] = 0;
  }
#endif
  gEF.active_sys = 0;
#if ERROR_FIELD_SIZE > 0
  MCOP_ErrField_Flush();
#endif
#endif

  // Check essential parameters
  if (! (IS_NODE_ID_VALID(MY_NODE_ID) 
#if USE_LSS_SERVER
  // with LSS slave processing enabled, we have to accept node id 0
      || (MY_NODE_ID==0)
#endif
    ))
  {
    MCOUSER_FatalError(ERRFT_PARA);
    retval = FALSE;
  }

  if (retval == TRUE)
  {
    // Initialize the CAN interface
    if ( ! MCOHW_Init(Bitrate
#if defined(USE_CANOPEN_FD) && (USE_CANOPEN_FD==1)
    , BRSBitrate
#endif
    ))
    {
      MCOUSER_FatalError(ERRFT_INIT);
      retval = FALSE;
    }
  }

#if USE_LSS_SERVER
  // Set receive filter for LSS master message
  if ((retval == TRUE) && !MCOHW_SetCANFilter(LSS_MANAGER_ID))
  {
    MCOUSER_FatalError(ERRFT_RXFLTN);
    retval = FALSE;
  }
#endif

  if (MY_NODE_ID != 0)
  {
    // continue with variable initialization
    gMCOConfig.heartbeat_msg.ID = 0x700+MY_NODE_ID;
    gMCOConfig.heartbeat_msg.LEN = 1;
    // current NMT state of this node = bootup
    MY_NMT_STATE = 0;

    // Init SDO Response/Abort message
    gTxSDO.ID = 0x0580+MY_NODE_ID;
    gTxSDO.LEN = 8;

#if ! MGR_MONITOR_ALL_NODES
#if (NR_OF_HB_CONSUMER > 0)
    // init heartbeat consumption
    for (i = 0; i < NR_OF_HB_CONSUMER; i++)
    {
      gHBCons[i].status = HBCONS_OFF; // disable consumption
    }
#endif // (NR_OF_HB_CONSUMER > 0)
#endif // MGR_MONITOR_ALL_NODES
     
#if NR_OF_TPDOS > 0
    i = 0;
    // init TPDOs
    while (i < NR_OF_TPDOS)
    {
      gTPDOConfig[i].CANmsg.ID = COBID_DISABLED; // Disable by default
      gTPDOConfig[i].PDONr = 0;
#if USE_SYNC
      gTPDOConfig[i].SYNCmatch = 0;
#endif
      i++;
    }
#endif

#if NR_OF_RPDOS > 0
		i = 0;
		// init RPDOs
		while (i < NR_OF_RPDOS) {
			gRPDOConfig[i].CANID = COBID_DISABLED; // Disable by default
			gRPDOConfig[i].PDONr = 0;
			gRPDOConfig[i].TType = 255; // Default to asynchronous, overridden by OD
			i++;
		}
#endif

#if USECB_TIMEOFDAY
    // filter for CAN ID 0x100
    if ((retval == TRUE) && !MCOHW_SetCANFilter(0x100))
    {
      MCOUSER_FatalError(ERRFT_RXFLTN);
      retval = FALSE;
    }
#endif

    // filter for nmt master message
    if ((retval == TRUE) && !MCOHW_SetCANFilter(0))
    {
      MCOUSER_FatalError(ERRFT_RXFLTN);
      retval = FALSE;
    }
#if USE_SYNC
    // for SYNC message
    if ((retval == TRUE) && !MCOHW_SetCANFilter(gMCOConfig.SYNC_id))
    {
      MCOUSER_FatalError(ERRFT_RXFLTN);
      retval = FALSE;
    }
#endif

#if defined(USE_CANOPEN_FD) && (USE_CANOPEN_FD==1)
    // for receiving USDO requests
    if (MCOHW_SetCANFilterRange(0x601,0x67F) != 1)
    {
      MCOUSER_FatalError(ERRFT_RXFLTS);
    }
#else
#if USE_SDOMESH
    // for receiving meshed SDO requests
    for (i=1;i<=16;i++)
    { // for all 16 node IDs
      if (i != MY_NODE_ID)
      { // only proceed if node id is not own node id
        if ((retval == TRUE) && !MCOHW_SetCANFilter(CAN_ID_SDOREQUEST(i, MY_NODE_ID)))
        {
          MCOUSER_FatalError(ERRFT_RXFLTS);
          retval = FALSE;
        }
      }
    }
#else // USE_SDOMESH
#if USE_CiA447
    // for receiving CiA447 SDO requests
    for (i=1;i<=16;i++)
    { // for all 16 node IDs
      if (i != MY_NODE_ID)
      { // only proceed if node id is not own node id
        if ((retval == TRUE) && !MCOHW_SetCANFilter(CAN_ID_SDOREQUEST(i, MY_NODE_ID)))
        {
          MCOUSER_FatalError(ERRFT_RXFLTS);
          retval = FALSE;
        }
      }
    }
#endif // USE_CiA447
    // for standard CANopen SDO requests
    if ((retval == TRUE) && !MCOHW_SetCANFilter(0x600+MY_NODE_ID))
    {
      MCOUSER_FatalError(ERRFT_RXFLTS);
      retval = FALSE;
    }
#endif // USE_SDOMESH
#endif // USE_CANOPEN_FD

#if USE_NODE_GUARDING
    // for Node Guarding requests
    if ((retval == TRUE) && !MCOHW_SetCANFilter(COBID_RTR | 0x700 | MY_NODE_ID))
    {
      MCOUSER_FatalError(ERRFT_RXFLTN);
      retval = FALSE;
    }
    gMCOConfig.NGtoggle = 0;
#endif

#if USE_SLEEP
    for (i = 0; i <= 16; i++)
    { // for wakeup/sleep requests
      if ((retval == TRUE) && !MCOHW_SetCANFilter(0x690+i))
      {
        MCOUSER_FatalError(ERRFT_RXFLTS);
        retval = FALSE;
      }
    }
#endif
  } // if (MY_NODE_ID != 0)
  else
  { // invalid node ID or LSS mode
    retval = FALSE;
  }

#if (INDEX_FOR_DIAGNOSTICS != 0)
  // Init diagnostic data
  gMCODiag.Status = 0;
#if (TXFIFOSIZE > 0)
  gMCODiag.TxFIFOStatus = 0;
#endif
#if (RXFIFOSIZE > 0)
  gMCODiag.RxFIFOStatus = 0;
#endif
#if (MGRFIFOSIZE > 0)
  gMCODiag.RxMgrFIFOStatus = 0;
#endif
  gMCODiag.ProcTickPerSecMin = 0xFFFF;
  gMCODiag.ProcTickPerSecMax = 0;
  gMCODiag.ProcTickBurstMax = 0;
  gMCODiag.ProcRxPerSecMin = 0xFFFF;
  gMCODiag.ProcRxPerSecMax = 0;
#if MGR_MONITOR_ALL_NODES
  gMCODiag.ProcMgrTickPerSecMin = 0xFFFF;
  gMCODiag.ProcMgrTickPerSecMax = 0;
  gMCODiag.ProcMgrTickBurstMax = 0;
  gMCODiag.ProcMgrRxPerSecMin = 0xFFFF;
  gMCODiag.ProcMgrRxPerSecMax = 0;
#endif
  gMCODiag.TickCnt = 0;
  gMCODiag.RxCnt = 0;
  gMCODiag.BurstCnt = 0;
#if MGR_MONITOR_ALL_NODES
  gMCODiag.MgrTickCnt = 0;
  gMCODiag.MgrRxCnt = 0;
  gMCODiag.MgrBurstCnt = 0;
#endif
#endif // Diag record

  if (retval == TRUE)
  {
    // signal to MCO_ProcessStack: we just initialized
    gTPDONr = 0xFFFF;
  }

  return retval;
}


/**************************************************************************
DOES:    Applies a PDO parameter
RETURNS: 0xFFFFFFFF if access success, else SDO abort code
**************************************************************************/
static uint8_t MCO_ApplyPDOparam(
  uint8_t   PDOType,  // 0 for TPDO com, 1 for RPDO com
                      // 2 for TPDO map, 3 for RPDO map
  uint16_t  PDONr,    // PDO Nr starting at 1
  uint8_t   subidx,   // Sub index
  uint32_t  dat       // data to apply
  )
{
uint32_t ret_val = SDO_ABORT_GENERAL;
uint16_t lp;
uint8_t dlen;
#if ! USE_DYNAMIC_PDO_MAPPING
uint16_t offset; // into process image
OD_PROCESS_DATA_ENTRY MEM_CONST* pOD; // pointer to an entry in gODProcTable
#if USE_EXTENDED_SDO && !USE_GENOD_PTR
uint8_t para8;
uint32_t para32;
uint8_t* pdat8;
OD_GENERIC_DATA_ENTRY MEM_CONST* pGOD; // pointer to an entry in gODGenericTable
#endif // USE_EXTENDED_SDO && !USE_GENOD_PTR
#endif
static uint8_t nr_mapped; // number of mapped items in PDO
  
  // calculate real PDONr offset
  if ( (PDOType == 0) || (PDOType == 2) )
  { // TPDO, find the PDONr in array
#if (NR_OF_TPDOS > 0)
    lp = 0;
    while ( (lp <= gMCOConfig.nrTPDOs) && (gTPDOConfig[lp].PDONr != PDONr) )
    {
      lp++;
      if (lp >= gMCOConfig.nrTPDOs) 
      { // not in list
        if (gMCOConfig.nrTPDOs < NR_OF_TPDOS) 
        { // not found, try to add
          gMCOConfig.nrTPDOs++;
          lp = gMCOConfig.nrTPDOs - 1;
          gTPDOConfig[lp].PDONr = PDONr;
        }
        else
        {
          lp = 0xFFFF;
        }
      }
    }
#endif
  }
  else if ( (PDOType == 1) || (PDOType == 3) )
  { // RPDO, find the PDONr in array
#if (NR_OF_RPDOS > 0)
    lp = 0;
    while ( (lp <= gMCOConfig.nrRPDOs) && (gRPDOConfig[lp].PDONr != PDONr) )
    {
      lp++;
      if (lp >= gMCOConfig.nrRPDOs) 
      { // not in list
        if (gMCOConfig.nrRPDOs < NR_OF_RPDOS) 
        { // not found, try to add
          gMCOConfig.nrRPDOs++;
          lp = gMCOConfig.nrRPDOs - 1;
          gRPDOConfig[lp].PDONr = PDONr;
        }
        else
        {
          lp = 0xFFFF;
        }
      }
    }
#endif
  }
  else
  { // invalid PDO Type
    lp = 0xFFFF;
  }
  
  if (lp < 0xFFFF)
  {
    // PDO found
    if (PDOType == 0)
    { // TPDO com
#if (NR_OF_TPDOS > 0)
      if (subidx == 0)
      { // new access, re-init essentials
        gTPDOConfig[lp].CANmsg.ID = COBID_DISABLED;
        gTPDOConfig[lp].TType = 255;
#if USE_INHIBIT_TIME
        gTPDOConfig[lp].inhibit_time = 0;
#endif
#if USE_EVENT_TIME
        gTPDOConfig[lp].event_time = 0;
#endif
#if USE_SYNC
        gTPDOConfig[lp].SYNCmatch = 0;
#endif
        ret_val = 0xFFFFFFFFul;
      }
      else if (subidx == 1)
      { // CAN ID
        uint32_t flags = dat & 0xE0000000ul;
        dat &= ~0xE0000000ul;
        if ((dat == 0) && (gTPDOConfig[lp].PDONr <= 4))
        { // apply default
          dat = MY_NODE_ID + (gTPDOConfig[lp].PDONr * 0x100) + 0x00000080;
        }
        gTPDOConfig[lp].CANmsg.ID = (COBID_TYPE) dat;
#if CAN_ID_SIZE == 32
        gTPDOConfig[lp].CANmsg.ID |= (COBID_TYPE) (flags);
#else
        // adapt to 16bit
        gTPDOConfig[lp].CANmsg.ID |= (COBID_TYPE) (flags >> 16);
#endif
#if defined(USE_CANOPEN_FD) && (USE_CANOPEN_FD==1)
        // Default is to NOT enforce classical CAN
        gTPDOConfig[lp].CANmsg.ID &= ~COBID_FORCE_CL;
#endif
        ret_val = 0xFFFFFFFFul;
      }
      else if (subidx == 2)
      { // Transmission Type
        gTPDOConfig[lp].TType = (uint8_t) dat;
        ret_val = 0xFFFFFFFFul;
      }
#if USE_INHIBIT_TIME
      else if (subidx == 3)
      { // Inhibit Time
        gTPDOConfig[lp].inhibit_time = (uint16_t) ((dat + 9) / 10);
        ret_val = 0xFFFFFFFFul;
      }
#endif
#if USE_EVENT_TIME
      else if (subidx == 5)
      { // Event Time
        if (dat > 32000)
        {
          dat = 32000;
        }
        gTPDOConfig[lp].event_time = (uint16_t) dat;
        ret_val = 0xFFFFFFFFul;
      }
#endif
#if USE_SYNC
      else if (subidx == 6)
      { // SYNC start value
        gTPDOConfig[lp].SYNCmatch = (uint8_t) dat;
        ret_val = 0xFFFFFFFFul;
      }
#endif
      else
      { // wrong subindex
        ret_val = SDO_ABORT_UNKNOWNSUB;
      }

#endif
    }
    else if (PDOType == 1)
    { // RPDO com
#if (NR_OF_RPDOS > 0)
      if (subidx == 0)
      { // new access, re-init essentials
        if (!(IS_CANID_RESTRICTED(gRPDOConfig[lp].CANID & ~COBID_OPT_MASK)))
        { // CAN ID already in use, try to clr filter
          MCOHW_ClearCANFilter(gRPDOConfig[lp].CANID);
        }
        gRPDOConfig[lp].CANID = COBID_DISABLED;
        gRPDOConfig[lp].TType = 255;
      }
      if (subidx == 1)
      { // CAN ID
        uint32_t flags = dat & 0xA0000000ul;  // mask out ID and RTR flag - not used
        dat &= ~0xE0000000ul;
        if ((dat == 0) && (gRPDOConfig[lp].PDONr <= 4))
        { // apply default
          dat = MY_NODE_ID + (gRPDOConfig[lp].PDONr * 0x100) + 0x00000100;
        }
        gRPDOConfig[lp].CANID = (COBID_TYPE)dat;
#if CAN_ID_SIZE == 32
        gRPDOConfig[lp].CANID |= (COBID_TYPE) (flags);
#else
        // adapt to 16bit
        gRPDOConfig[lp].CANID |= (COBID_TYPE) (flags >> 16);
#endif
        // Set CAN receive filter for this, if not disabled and not 0 and not previously set
        if (!(flags & 0x80000000ul) && (dat != 0) && ((gMCOConfig.error_code & 0x80) == 0))
        {
          if (!MCOHW_SetCANFilter(~COBID_OPT_MASK & gRPDOConfig[lp].CANID))
          { // no more filter available
            MCOUSER_FatalError(ERRFT_RXFLTN);
          }
        }
        ret_val = 0xFFFFFFFFul;
      }
      else if (subidx == 2)
      { // Transmission Type
        gRPDOConfig[lp].TType = (uint8_t) dat;
        ret_val = 0xFFFFFFFFul;
      }
      else
      { // wrong subindex
        ret_val = SDO_ABORT_UNKNOWNSUB;
      }
#endif
    }
    else if (PDOType == 2)
    { // TPDO map
#if (NR_OF_TPDOS > 0)
      // This version requires that all mapping parameters are in 
      // SDO Reply table, consecutively and all mapped items are
      // consecutively in the process image.
      // This is default for CANopen Architect generated files
      if (subidx == 0)
      { // set marker to nr of mapped entries
        nr_mapped = dat;
        gTPDOConfig[lp].CANmsg.LEN = 0;
#if USE_DYNAMIC_PDO_MAPPING
        XPDO_ResetPDOMapEntry(0, PDONr);
#endif
        ret_val = 0xFFFFFFFFul;
      }
      else if ((subidx == 1) && (nr_mapped != 0) )
      { // first mapping entry 16bit idx, 8bit sub, 8bit len (in bits)
        // find offset
        dlen = ((dat >> 3) & 0x1F);
#if USE_DYNAMIC_PDO_MAPPING
        if ((gTPDOConfig[lp].CANmsg.LEN + dlen) <= CAN_MAX_DATA_SIZE)
        {
          if (XPDO_SetPDOMapEntry (0,gTPDOConfig[lp].PDONr,subidx,dat >> 16,(dat >> 8) & 0xFF,dlen))
          {
            gTPDOConfig[lp].CANmsg.LEN += dlen;
            ret_val = 0xFFFFFFFFul;
          }
        }
#else
        offset = MCO_SearchODProcTable(dat >> 16,(dat >> 8) & 0xFF);
        if (offset != 0xFFFF)
        { // location found
          pOD = OD_ProcTablePtr(offset);
          gTPDOConfig[lp].offset = pOD->off_hi;
          gTPDOConfig[lp].offset <<= 8;
          gTPDOConfig[lp].offset += pOD->off_lo;
          if ((gTPDOConfig[lp].CANmsg.LEN + dlen) <= CAN_MAX_DATA_SIZE)
          {
            gTPDOConfig[lp].CANmsg.LEN += dlen;
            ret_val = 0xFFFFFFFFul;
          }
        }
#if USE_EXTENDED_SDO
        else
        {
          offset = XSDO_SearchODGenTable(dat >> 16, (dat >> 8) & 0xFF, &para8, &para32, &pdat8);
          if (offset != 0xFF)
          { // location found
            pGOD = OD_GenericTablePtr(offset);
#if USE_GENOD_PTR == 1
            gTPDOConfig[lp].offset = (INSIDE_PI(pGOD->pDat)) ? (uint16_t)OFFSET_PI(pGOD->pDat) : INVALID_PI_OFFSET;
#else
            gTPDOConfig[lp].offset = pGOD->off_hi;
            gTPDOConfig[lp].offset <<= 8;
            gTPDOConfig[lp].offset += pGOD->off_lo;
#endif
            if ((gTPDOConfig[lp].CANmsg.LEN + dlen) <= CAN_MAX_DATA_SIZE)
            {
              gTPDOConfig[lp].CANmsg.LEN += dlen;
              ret_val = 0xFFFFFFFFul;
            }
          }
        }
#endif // USE_EXTENDED_SDO
#endif
      }
      else if (subidx <= nr_mapped)
      { // increase length
        dlen = ((dat >> 3) & 0x1F);
        if ((gTPDOConfig[lp].CANmsg.LEN + dlen) <= CAN_MAX_DATA_SIZE)
        {
#if USE_DYNAMIC_PDO_MAPPING
          XPDO_SetPDOMapEntry(0, gTPDOConfig[lp].PDONr, subidx, dat >> 16, (dat >> 8) & 0xFF, dlen);
#endif
          gTPDOConfig[lp].CANmsg.LEN += dlen;
          ret_val = 0xFFFFFFFFul;
        }
      }
      else if (subidx <= CAN_MAX_DATA_SIZE)
      { // ignore entry, but do not produce an error
        ret_val = 0xFFFFFFFFul;
      }
      else
      { // wrong subindex
        ret_val = SDO_ABORT_UNKNOWNSUB;
      }
      // now check if we reached end
      if (subidx == nr_mapped)
      { // done with this PDO, clr marker
#if USE_DYNAMIC_PDO_MAPPING
        XPDO_SetPDOMapEntry(0,gTPDOConfig[lp].PDONr,0,0,0,nr_mapped);
        XPDO_UpdatePDOMapping(0,gTPDOConfig[lp].PDONr);
#endif
        nr_mapped = 0;
      }
#endif
    }
    else if (PDOType == 3)
    { // RPDO map
#if (NR_OF_RPDOS > 0)
      // This version requires that all mapping parameters are in 
      // SDO Reply table, consecutively and all maped items are
      // consecutively in the process image.
      // This is default for CANopen Architect generate dfiles
      if (subidx == 0)
      { // set marker to nr of mapped entries
        nr_mapped = dat;
        gRPDOConfig[lp].len = 0;
#if USE_DYNAMIC_PDO_MAPPING
        XPDO_ResetPDOMapEntry(1, PDONr);
#endif
        ret_val = 0xFFFFFFFFul;
      }
      else if ((subidx == 1) && (nr_mapped != 0) )
      { // first mapping entry 16bit idx, 8bit sub, 8bit len (in bits)
        dlen = ((dat >> 3) & 0x1F);
#if USE_DYNAMIC_PDO_MAPPING
        if ((gRPDOConfig[lp].len + dlen) <= CAN_MAX_DATA_SIZE)
        {
          if (XPDO_SetPDOMapEntry(0, gRPDOConfig[lp].PDONr, subidx, dat >> 16, (dat >> 8) & 0xFF, dlen))
          {
            gRPDOConfig[lp].len += dlen;
            ret_val = 0xFFFFFFFFul;
          }
        }
#else
        // find offset
        offset = MCO_SearchODProcTable(dat >> 16,(dat >> 8) & 0xFF);
        if (offset != 0xFFFF)
        { // location found
          pOD = OD_ProcTablePtr(offset);
          gRPDOConfig[lp].offset = pOD->off_hi;
          gRPDOConfig[lp].offset <<= 8;
          gRPDOConfig[lp].offset += pOD->off_lo;
          if ((gRPDOConfig[lp].len + dlen) <= CAN_MAX_DATA_SIZE)
          {
            gRPDOConfig[lp].len += dlen;
            ret_val = 0xFFFFFFFFul;
          }
        }
#if USE_EXTENDED_SDO
        else
        {
          offset = XSDO_SearchODGenTable(dat >> 16, (dat >> 8) & 0xFF, &para8, &para32, &pdat8);
          if (offset != 0xFF)
          { // location found
            pGOD = OD_GenericTablePtr(offset);
#if USE_GENOD_PTR == 1
            gRPDOConfig[lp].offset = (INSIDE_PI(pGOD->pDat)) ? (uint16_t)OFFSET_PI(pGOD->pDat) : INVALID_PI_OFFSET;
#else
            gRPDOConfig[lp].offset = pGOD->off_hi;
            gRPDOConfig[lp].offset <<= 8;
            gRPDOConfig[lp].offset += pGOD->off_lo;
#endif
            if ((gRPDOConfig[lp].len + dlen) <= CAN_MAX_DATA_SIZE)
            {
              gRPDOConfig[lp].len += dlen;
              ret_val = 0xFFFFFFFFul;
            }
          }
        }
#endif // USE_EXTENDED_SDO
#if USECB_ODDATARECEIVED
        gRPDOConfig[lp].map = MCO_GetRPDOMappingOffset(gRPDOConfig[lp].PDONr);
#endif // USECB_ODDATARECEIVED
#endif
      }
      else if (subidx <= nr_mapped)
      { // increase length
        dlen = ((dat >> 3) & 0x1F);
        if ((gRPDOConfig[lp].len + dlen) <= CAN_MAX_DATA_SIZE)
        {
#if USE_DYNAMIC_PDO_MAPPING
          XPDO_SetPDOMapEntry(1, gRPDOConfig[lp].PDONr, subidx, dat >> 16, (dat >> 8) & 0xFF, dlen);
#endif
          gRPDOConfig[lp].len += dlen;
          ret_val = 0xFFFFFFFFul;
        }
      }
      else if (subidx <= CAN_MAX_DATA_SIZE)
      { // ignore entry, but do not produce an error
        ret_val = 0xFFFFFFFFul;
      }
      else
      { // wrong subindex
        ret_val = SDO_ABORT_UNKNOWNSUB;
      }
      // now check if we reached end
      if (subidx == nr_mapped)
      { // done with this PDO, clr marker
#if USE_DYNAMIC_PDO_MAPPING
        XPDO_SetPDOMapEntry(1,gRPDOConfig[lp].PDONr,0,0,0,nr_mapped);
        XPDO_UpdatePDOMapping(1,gRPDOConfig[lp].PDONr);
#endif
        nr_mapped = 0;
      }
#endif
    }
  }
  
  return ret_val;
}


/**************************************************************************
DOES:    Retrieves the current value of a PDO communication parameter
NOTE:    Mapping parameters are always taken from OD
RETURNS: 0xFFFFFFFF if access success, else SDO abort code
**************************************************************************/
static uint8_t MCO_GetPDOparam(
  uint8_t   PDOType,  // 0 for TPDO com, 1 for RPDO com
  uint16_t  PDONr,    // PDO Nr starting at 1
  uint8_t   subidx,   // Sub index
  uint8_t   *plen,    // length of data
  uint32_t  *pdat     // data 
  )
{
uint32_t ret_val = SDO_ABORT_GENERAL;
uint16_t lp;
  
  // calculate real PDONr offset
  if ( (PDOType == 0) || (PDOType == 2) )
  { // TPDO, find the PDONr in array
#if (NR_OF_TPDOS > 0)
    lp = 0;
    while ( (lp <= gMCOConfig.nrTPDOs) && (gTPDOConfig[lp].PDONr != PDONr) )
    {
      lp++;
      if (lp >= gMCOConfig.nrTPDOs) 
      { // not in list
        if (gMCOConfig.nrTPDOs < NR_OF_TPDOS) 
        { // not found, try to add
          gMCOConfig.nrTPDOs++;
          lp = gMCOConfig.nrTPDOs - 1;
          gTPDOConfig[lp].PDONr = PDONr;
        }
        else
        {
          lp = 0xFFFF;
        }
      }
    }
#endif
  }
  else if ( (PDOType == 1) || (PDOType == 3) )
  { // RPDO, find the PDONr in array
#if (NR_OF_RPDOS > 0)
    lp = 0;
    while ( (lp <= gMCOConfig.nrRPDOs) && (gRPDOConfig[lp].PDONr != PDONr) )
    {
      lp++;
      if (lp >= gMCOConfig.nrRPDOs) 
      { // not in list
        if (gMCOConfig.nrRPDOs < NR_OF_RPDOS) 
        { // not found, try to add
          gMCOConfig.nrRPDOs++;
          lp = gMCOConfig.nrRPDOs - 1;
          gRPDOConfig[lp].PDONr = PDONr;
        }
        else
        {
          lp = 0xFFFF;
        }
      }
    }
#endif
  }
  else
  { // invalid PDO Type
    lp = 0xFFFF;
  }
  
  if (lp < 0xFFFF)
  {
    // PDO found
    if (PDOType == 0)
    { // TPDO com
#if (NR_OF_TPDOS > 0)
      if (subidx == 0)
      { 
        *pdat = 6;
        *plen = 1;
        ret_val = 0xFFFFFFFFul;
      }
      else if (subidx == 1)
      { // CAN ID
        *pdat = gTPDOConfig[lp].CANmsg.ID;
#if CAN_ID_SIZE == 16
        // adapt to 32bit
        *pdat = ((*pdat & 0xF000) << 16) | (*pdat & 0x07FF);
#endif
        *plen = 4;
        ret_val = 0xFFFFFFFFul;
      }
      else if (subidx == 2)
      { // Transmission Type
        *pdat = gTPDOConfig[lp].TType;
        *plen = 1;
        ret_val = 0xFFFFFFFFul;
      }
#if USE_INHIBIT_TIME
      else if (subidx == 3)
      { // Inhibit Time
        *pdat = gTPDOConfig[lp].inhibit_time * 10;
        *plen = 2;
        ret_val = 0xFFFFFFFFul;
      }
#endif
#if USE_EVENT_TIME
      else if (subidx == 5)
      { // Event Time
        *pdat = gTPDOConfig[lp].event_time;
        *plen = 2;
        ret_val = 0xFFFFFFFFul;
      }
#endif
#if USE_SYNC
      else if (subidx == 6)
      { // SYNC start value
        *pdat = gTPDOConfig[lp].SYNCmatch;
        *plen = 1;
        ret_val = 0xFFFFFFFFul;
      }
#endif
      else
      { // wrong subindex
        ret_val = SDO_ABORT_UNKNOWNSUB;
      }
#endif
    }
    else if (PDOType == 1)
    { // RPDO com
#if (NR_OF_RPDOS > 0)
      if (subidx == 0)
      { // new access, re-init essentials
        *pdat = 2;
        *plen = 1;
      }
      if (subidx == 1)
      { // CAN ID
        *pdat = gRPDOConfig[lp].CANID;
#if CAN_ID_SIZE == 16
        // adapt to 32bit
        *pdat = ((*pdat & 0xF000) << 16) | (*pdat & 0x07FF);
#endif
        *plen = 4;
        ret_val = 0xFFFFFFFFul;
      }
      else if (subidx == 2)
      { // Transmission Type
        *pdat = gRPDOConfig[lp].TType;
        *plen = 1;
        ret_val = 0xFFFFFFFFul;
      }
      else
      { // wrong subindex
        ret_val = SDO_ABORT_UNKNOWNSUB;
      }
#endif
    }
#if USE_DYNAMIC_PDO_MAPPING
    else if (PDOType == 2)
    { // TPDO map
      // STILL TO DO !!! ???
    }
#if (NR_OF_TPDOS > 0)
#endif
#if (NR_OF_RPDOS > 0)
    else if (PDOType == 3)
    { // RPDO map
      // STILL TO DO !!! ???
    }
#endif
#endif
  }
  
  return ret_val;
}


/**************************************************************************
DOES:    Checks if this OD entry is a system entry and if it is
         applies the data to the system variable
RETURNS: 0xFFFFFFFF if access success, else SDO abort code
**************************************************************************/
uint32_t MCO_ApplySystemEntry(
  uint16_t index,
  uint8_t subindex,
  uint32_t dat
  )
{
uint32_t ret_val = SDO_ABORT_NOT_EXISTS;
  
  if (index == 0x1017)
  {
    if (subindex == 0)
    { // HB Producer
      if (dat > 0x7FFF)
      { // maximum time supported by MCOP
        dat = 0x7FFF;
      }
      gMCOConfig.heartbeat_time = dat;
      ret_val = 0xFFFFFFFFul;
    }
    else
    { // wrong subindex
      ret_val = SDO_ABORT_UNKNOWNSUB;
    }
  }
  else if ((index >= 0x1800) && (index <= 0x19FF))
  { // TPDO communication parameter
    ret_val = MCO_ApplyPDOparam(0,(index & 0x1FF)+1,subindex,dat);
  }
  else if ((index >= 0x1400) && (index <= 0x15FF))
  { // RPDO communication parameter
    ret_val = MCO_ApplyPDOparam(1,(index & 0x1FF)+1,subindex,dat);
  }
  else if ((index >= 0x1A00) && (index <= 0x1BFF))
  { // TPDO mapping parameter
    ret_val = MCO_ApplyPDOparam(2,(index & 0x1FF)+1,subindex,dat);
  }
  else if ((index >= 0x1600) && (index <= 0x17FF))
  { // RPDO mapping parameter
    ret_val = MCO_ApplyPDOparam(3,(index & 0x1FF)+1,subindex,dat);
  }
#if (NR_OF_HB_CONSUMER > 0)
  else if (index == 0x1016)
  {
    if ((subindex > 0) && (subindex <= NR_OF_HB_CONSUMER))
    { // HB Consumer
#if ! MGR_MONITOR_ALL_NODES
      MCOP_InitHBConsumer(subindex,(uint8_t)(dat >> 16),(uint16_t)dat);
      ret_val = 0xFFFFFFFFul;
#else
      if (subindex == (uint8_t)(dat >> 16))
      {
        MGR_InitHBConsumer(subindex,(uint16_t)dat);
        ret_val = 0xFFFFFFFFul;
      }
      else
      {
        ret_val = SDO_ABORT_PARAMETER;
      }
#endif
    }
    else
    { // wrong subindex
      ret_val = SDO_ABORT_UNKNOWNSUB;
    }
  }
#endif
#if USE_EMCY
  else if (index == 0x1015)
  { // emergency inhibit time
    if (subindex == 0)
    {
      gEF.emcy_inhibit = (dat + 9) / 10;
      ret_val = 0xFFFFFFFFul;
    }
    else
    { // wrong subindex
      ret_val = SDO_ABORT_UNKNOWNSUB;
    }
  }
#endif
#if USE_SYNC
  else if (index == 0x1005)
  { // SYNC ID
    if (subindex == 0)
    {
      if (dat >= 1)
      { // set CAN rx for SYNC message
        if (gMCOConfig.SYNC_id != (COBID_TYPE) dat)
        { // set new CAN rx filter, if different from previous
          MCOHW_ClearCANFilter(gMCOConfig.SYNC_id);
          if (!MCOHW_SetCANFilter((COBID_TYPE) dat))
          {
            MCOUSER_FatalError(ERRFT_RXFLTN);
          }
        }
#if CAN_ID_SIZE == 32
        gMCOConfig.SYNC_id = dat;
#else
        gMCOConfig.SYNC_id = (dat & 0xF0000000ul) >> 16;
        gMCOConfig.SYNC_id += (COBID_TYPE)dat;
#endif
#if USE_SYNC_PRODUCER
        if ((gMCOConfig.SYNC_id & COBID_RTR) != 0)
        { // here RTR bit enables producer
          MCOHW_SetSyncProducer(gMCOConfig.SYNC_id,gMCOConfig.SYNC_cycle,gMCOConfig.SYNC_cntovr);
        }
#endif
        ret_val = 0xFFFFFFFFul;
      }
    }
    else
    { // wrong subindex
      ret_val = SDO_ABORT_UNKNOWNSUB;
    }
  }
  else if (index == 0x1019)
  { // SYNC counter overflow
    if (subindex == 0)
    {
      gMCOConfig.SYNC_cntovr = dat;
#if USE_SYNC_PRODUCER
      MCOHW_SetSyncProducer(gMCOConfig.SYNC_id, gMCOConfig.SYNC_cycle, gMCOConfig.SYNC_cntovr);
#endif
      ret_val = 0xFFFFFFFFul;
    }
    else
    { // wrong subindex
      ret_val = SDO_ABORT_UNKNOWNSUB;
    }
  }
#if USE_SYNC_PRODUCER
  else if (index == 0x1006)
  { // SYNC Communication cycle period, enables producer
    if (subindex == 0)
    { // re-init producer
      gMCOConfig.SYNC_cycle = dat;
      MCOHW_SetSyncProducer(gMCOConfig.SYNC_id,gMCOConfig.SYNC_cycle,gMCOConfig.SYNC_cntovr);
      ret_val = 0xFFFFFFFFul;
    }
    else
    { // wrong subindex
      ret_val = SDO_ABORT_UNKNOWNSUB;
    }
  }
#endif // USE_SYNC_PRODUCER
#endif // USE_SYNC
  return ret_val;
}


/**************************************************************************
DOES:    Checks if this OD entry is a system entry and if it is
         returns the current value of the system variable
RETURNS: 0xFFFFFFFF if access success, else SDO abort code
**************************************************************************/
uint32_t MCO_GetSystemEntry(
  uint16_t index,
  uint8_t subindex,
  uint8_t *plen,
  uint32_t *pdat
  )
{
uint32_t ret_val = SDO_ABORT_NOT_EXISTS;
  
  if (index == 0x1017)
  {
    if (subindex == 0)
    { // HB Producer
      *pdat = gMCOConfig.heartbeat_time;
      *plen = 2;
      ret_val = 0xFFFFFFFFul;
    }
    else
    { // wrong subindex
      ret_val = SDO_ABORT_UNKNOWNSUB;
    }
  }
  else if ((index >= 0x1800) && (index <= 0x19FF))
  { // TPDO communication parameter
    ret_val = MCO_GetPDOparam(0,(index & 0x1FF)+1,subindex,plen,pdat);
  }
  else if ((index >= 0x1400) && (index <= 0x15FF))
  { // RPDO communication parameter
    ret_val = MCO_GetPDOparam(1,(index & 0x1FF)+1,subindex,plen,pdat);
  }
#if USE_DYNAMIC_PDO_MAPPING
  else if ((index >= 0x1A00) && (index <= 0x1BFF))
  { // TPDO mapping parameter
    ret_val = MCO_GetPDOparam(2,(index & 0x1FF)+1,subindex,plen,pdat);
  }
  else if ((index >= 0x1600) && (index <= 0x17FF))
  { // RPDO mapping parameter
    ret_val = MCO_GetPDOparam(3,(index & 0x1FF)+1,subindex,plen,pdat);
  }
#else
  // these parameters are in regular OD, no internal system copy
#endif
#if USECB_ODSERIAL
  else if ((index == 0x1018) && (subindex == 4))
  {
    *pdat = MCOUSER_GetSerial();
    *plen = 4;
  }
#endif // USECB_ODSERIAL
#if (NR_OF_HB_CONSUMER > 0)
  else if (index == 0x1016)
  {
    if (subindex == 0)
    {
      *pdat = NR_OF_HB_CONSUMER;
      *plen = 1;
      ret_val = 0xFFFFFFFFul;
    }
    else if ((subindex > 0) && (subindex <= NR_OF_HB_CONSUMER))
    { // HB Consumer
#if ! MGR_MONITOR_ALL_NODES
      *pdat = (gHBCons[subindex-1].can_id & 0x7F);
      *pdat <<= 8;
      *pdat += gHBCons[subindex-1].time;
#else
      *pdat = subindex;
      *pdat <<= 8;
      *pdat += gNodeList[subindex-1].hb_time;
#endif
      *plen = 4;
      ret_val = 0xFFFFFFFFul;
    }
    else
    { // wrong subindex
      ret_val = SDO_ABORT_UNKNOWNSUB;
    }
  }
#endif
#if USE_EMCY
  else if (index == 0x1015)
  { // emergency inhibit time
    if (subindex == 0)
    {
      *pdat = gEF.emcy_inhibit * 10;
      *plen = 2;
      ret_val = 0xFFFFFFFFul;
    }
    else
    { // wrong subindex
      ret_val = SDO_ABORT_UNKNOWNSUB;
    }
  }
#endif
#if (NR_OF_EMCY_CONSUMER > 0)
  else if (index == 0x1028)
  {
    if (subindex == 0)
    {
      *pdat = NR_OF_EMCY_CONSUMER;
      *plen = 1;
      ret_val = 0xFFFFFFFFul;
    }
    else if ((subindex > 0) && (subindex <= NR_OF_EMCY_CONSUMER))
    { // EMCY Consumer
      // IN THIS IMPLEMENTATION LOCKED TO HB CONSUMER
#if ! MGR_MONITOR_ALL_NODES
      *pdat = (gHBCons[subindex-1].can_id & 0x7F) + 0x00000080ul;
#else
      *pdat = subindex +0x00000080ul;
#endif
      *plen = 4;
      ret_val = 0xFFFFFFFFul;
    }
    else
    { // wrong subindex
      ret_val = SDO_ABORT_UNKNOWNSUB;
    }
  }
#endif
#if USE_SYNC
  else if (index == 0x1005)
  { // SYNC ID
    if (subindex == 0)
    {
      *pdat = gMCOConfig.SYNC_id;
      *plen = 4;
      ret_val = 0xFFFFFFFFul;
    }
    else
    { // wrong subindex
      ret_val = SDO_ABORT_UNKNOWNSUB;
    }
  }
  else if (index == 0x1019)
  { // SYNC counter overflow
    if (subindex == 0)
    {
      *pdat = gMCOConfig.SYNC_cntovr;
      *plen = 1;
      ret_val = 0xFFFFFFFFul;
    }
    else
    { // wrong subindex
      ret_val = SDO_ABORT_UNKNOWNSUB;
    }
  }
#endif // USE_SYNC
  return ret_val;
}


/**************************************************************************
DOES:    (Re-)Initializes CANopen system entries by reading them from the
         Object Dictionary currently in use.
RETURNS: nothing
**************************************************************************/
void MCO_UpdateSystemFromOD (void)
{
  uint8_t MEM_CONST *pSDOTable; // Ptr to SDOReply table
  OD_PROCESS_DATA_ENTRY MEM_CONST *pODTable; // Ptr to OD table
  uint8_t *pPI; // Ptr to Process Image
  uint16_t idx;
  uint16_t off;
  uint32_t val32;
  uint8_t len;
  
#if (USE_XOD_ACCESS == 1)
  pSDOTable = gOD.p_SDOResponseBase;
  pODTable = gOD.p_ProcBase;
#else
  pSDOTable = &(gSDOResponseTable[0]);
  pODTable = &(gODProcTable[0]);
#endif
  pPI = &(gProcImg[0]);
    
  while ((pSDOTable != NULL) && (pSDOTable[0] != 0xFF) && (pSDOTable[1] != 0xFF))
  { // loop through SDO response table
    idx = GEN_RD16(PIACC_NONE, (void *)&(pSDOTable[1]));
    MCO_ApplySystemEntry(idx,pSDOTable[3],GEN_RD32(PIACC_NONE, (void *)&(pSDOTable[4])));
    pSDOTable += 8;
  }
  while ((pODTable != NULL) && (pODTable->idx_hi != 0xFF) && (pODTable->idx_lo != 0xFF))
  {
    idx = GEN_RD16(PIACC_NONE, (void *)&(pODTable->idx_lo));
    off = GEN_RD16(PIACC_NONE, (void *)&(pODTable->off_lo));
    len = pODTable->len & 0x07u;
    switch (len)
    {
      case 0:
      default:
        len = 0;
        break;
      case 1:
        val32 = (uint32_t)GEN_RD8(PIACC_SDO, &(pPI[off]));
        break;
      case 2:
        val32 = (uint32_t)GEN_RD16(PIACC_SDO, &(pPI[off]));
        break;
      case 3:
        val32 = GEN_RD24(PIACC_SDO, &(pPI[off]));
        break;
      case 4:
        val32 = GEN_RD32(PIACC_SDO, &(pPI[off]));
        break;
    }
    if (len > 0)
    {
      MCO_ApplySystemEntry(idx, pODTable->subidx, val32);
    }
    pODTable++;
  }
  gMCOConfig.error_code |= 0x80; // Signal that RPDO filters are now set
}


#if !defined(USE_CANOPEN_FD) || (defined(USE_CANOPEN_FD) && (USE_CANOPEN_FD==0))
/**************************************************************************
DOES:    Common exit routine for SDO_Handler.
         Send SDO response with variable length (1-4 bytes).
         Assumes that gTxCAN.ID, LEN and BUF[1-3] are already set.
         Requires data to be in little endian format.
RETURNS: 1 if response transmitted
**************************************************************************/
uint8_t MCO_ReplyWith (
  uint8_t *pDat,  // pointer to sdo data
  uint8_t len     // number of bytes of data in SDO
  )
{
  uint8_t k; // for loop counter

  // expedited, len of data
  gTxSDO.BUF[0] = 0x43 | ((4-len) << 2);
  // copy data
  for (k = 4; k < 8; k++)
  {
    if (k < len+4)
    {
      gTxSDO.BUF[k] = *pDat;
      pDat++;
    }
    else
    { // required by conformance test, fill unused with zero
      gTxSDO.BUF[k] = 0;
    }
  }

  // transmit message
  if (!MCOHW_PushMessage(&gTxSDO))
  {
    // failed to transmit
    MCOUSER_FatalError(ERROFL_SDO);
  }

  // transmitted ok
  return 1;
}


/**************************************************************************
DOES:    Generates an SDO Abort Response
RETURNS: nothing
**************************************************************************/
void MCO_SendSDOAbort (
  uint32_t ErrorCode  // 4 byte SDO abort error code
  )
{
uint8_t i;

  // construct message data
  gTxSDO.BUF[0] = 0x80;
  for (i=0;i<4;i++)
  {
    gTxSDO.BUF[4+i] = (uint8_t)ErrorCode;
    ErrorCode >>= 8;
  }

  // transmit message
  if (!MCOHW_PushMessage(&gTxSDO))
  {
    // failed to transmit
    MCOUSER_FatalError(ERROFL_SDO);
  }
}
#endif // !USE_CANOPEN_FD


#if NR_OF_TPDOS > 0
#if USE_INHIBIT_TIME > 0
/**************************************************************************
DOES:    Called by application when a TPDO should be transmitted.
         Can be called after a write to the process image to avoid lengthy
         auto-detection of a COS (Change Of State)
RETURNS: nothing
**************************************************************************/
void MCO_TriggerTPDO (
  uint16_t TPDONr  // TPDO number to transmit (range 1 to 512)
  )
{
uint16_t found_rec = 0;

  // first guess: PDONr matches offset
  if ( (TPDONr < NR_OF_TPDOS) &&
       (gTPDOConfig[TPDONr-1].PDONr == TPDONr)
     )
  { // found the TPDO
    found_rec = TPDONr-1;
  }
  else
  {
    while ((found_rec < gMCOConfig.nrTPDOs) && (gTPDOConfig[found_rec].PDONr != TPDONr))
    {
      found_rec++;
    }
  }
  // found_rec is gMCOConfig.nrTPDOs if the TPDO was not found
  if (found_rec < gMCOConfig.nrTPDOs)
  { // Update TPDO data and mark for transmission
    // Copy current process data
    PDO_TXCOPY(found_rec,( uint8_t *)&(gTPDOConfig[found_rec].CANmsg.BUF[0]));
    // Mark for ASAP transmission
    if (gTPDOConfig[found_rec].inhibit_status == INHITIM_EXPIRED)
    { // inhibit timer is currently NOT running
      gTPDOConfig[found_rec].inhibit_timestamp = MCOHW_GetTime() - 1;
    }
    gTPDOConfig[found_rec].inhibit_status = INHITIM_RUNNING_TRIGGERED;
    // now it is marked for transmission, and timer is corrected
  }
}
#endif // USE_INHIBIT_TIME > 0


/**************************************************************************
DOES:    Called when a TPDO needs to be transmitted
RETURNS: nothing
**************************************************************************/
void MCO_TransmitPDO (
  uint16_t PDONr  // TPDO number to transmit, as gTPDOConfig[] index
  )
{
#if USE_INHIBIT_TIME
  // new inhibit timer started
  gTPDOConfig[PDONr].inhibit_status = INHITIM_RUNNING_NO_TRIGGER;
  gTPDOConfig[PDONr].inhibit_timestamp = MCOHW_GetTime() + gTPDOConfig[PDONr].inhibit_time;
#endif
#if USE_EVENT_TIME
  gTPDOConfig[PDONr].event_timestamp = MCOHW_GetTime() + gTPDOConfig[PDONr].event_time;
#endif
#if USE_SYNC
  gTPDOConfig[PDONr].SYNCcnt = gTPDOConfig[PDONr].TType;
#endif

  if (!MCOHW_PushMessage(&gTPDOConfig[PDONr].CANmsg))
  {
    MCOUSER_FatalError(ERROFL_PDO);
  }
}
#endif // NR_OF_TPDOS > 0


/**************************************************************************
DOES:    Handles the NMT Master Message, switching NMT Slave state
RETURNS: nothing
**************************************************************************/
void MCO_HandleNMTRequest (
  uint8_t NMTReq
  )
{
#if USE_EMCY
uint8_t cmd_is_good = TRUE;
#endif

  DEBUG_PRINT(MCO_LOG_INFO,("CANopen Event: NMT message received: 0x%2.2x\n",NMTReq));

  switch (NMTReq)
  {
    // start node
    case NMTMSG_OP:
      if (MY_NMT_STATE != NMTSTATE_OP)
      { // only if not already operational
        // set new state
        MY_NMT_STATE = NMTSTATE_OP;
#if USE_LEDS
        gMCOConfig.LEDRun = LED_ON;
#endif
#if USECB_NMTCHANGE
        // Call back to user / application
        MCOUSER_NMTChange(MY_NMT_STATE);
#endif
#if NR_OF_TPDOS > 0
        MCO_PrepareTPDOs();
#endif
#if NR_OF_RPDOS > 0
        MCO_PrepareRPDOs();
#endif
      }
      break;

    // stop node
    case NMTMSG_STOP:
      // set new state
      MY_NMT_STATE = NMTSTATE_STOP;
#if USE_LEDS
      gMCOConfig.LEDRun = LED_FLASH1;
#endif
#if USECB_NMTCHANGE
      // Call back to user / application
      MCOUSER_NMTChange(MY_NMT_STATE);
#endif
      break;

    // enter pre-operational
    case NMTMSG_PREOP:
      // set new state
      MY_NMT_STATE = NMTSTATE_PREOP;
#if USE_LEDS
      gMCOConfig.LEDRun = LED_BLINK;
#endif
      // Call back to user / application
#if USECB_NMTCHANGE
      // Call back to user / application
      MCOUSER_NMTChange(MY_NMT_STATE);
#endif
      break;

    // application reset
    case NMTMSG_RESETAPP:
      MCOUSER_ResetApplication();
      break;

    // node reset communication
    case NMTMSG_RESETCOM:
      (void)MCOUSER_ResetCommunication();
      break;

    // unknown command
    default:
#if USE_EMCY
      cmd_is_good = FALSE;
      if ((gEF.active_sys & EMCYSBIT_PROT) == 0)
      { // this was not yet recorded
        gEF.active_sys |= EMCYSBIT_PROT;
#if ERROR_FIELD_SIZE > 0
        MCOP_ErrField_AddUpdate(MAKE_ERRCODE32(EMCY_PROT_ERR,NMTReq,0)
#if defined(USE_CANOPEN_FD) && USE_CANOPEN_FD    // CANopen FD
          ,
          ERST_STATE_OCC
#endif
        );
#endif
#if defined(USECB_EMCY) && USECB_EMCY
        if (0 == MCOUSER_EMCY(FALSE, EMCY_PROT_ERR, NMTReq, 0, 0, 0, 0
#if defined(USE_CANOPEN_FD) && USE_CANOPEN_FD    // CANopen FD
          ,
          0,     // dev_num - logical device number
          COFD_BASE_SPEC >> 20, // spec_num, CiA specification number
          ERST_STATE_OCC | ERST_PRIO(7),  // status - priority=7, recoverable, error occurred
          0,     // time_lo - timestamp bits 0-31, not supported
          0      // time_hi - timestamp bits 32-47, not supported
#endif // CANopen FD
        ))
#endif // USECB_EMCY
        {
          (void)MCOP_PushEMCY(EMCY_PROT_ERR, NMTReq, 0, 0, 0, 0
#if defined(USE_CANOPEN_FD) && USE_CANOPEN_FD    // CANopen FD
            ,
            0,     // dev_num - logical device number
            COFD_BASE_SPEC >> 20, // spec_num, CiA specification number
            ERST_STATE_OCC | ERST_PRIO(7),  // status - priority=7, recoverable, error occurred
            0,     // time_lo - timestamp bits 0-31, not supported
            0      // time_hi - timestamp bits 32-47, not supported
#endif // CANopen FD
          );
        }
      }
#endif // USE_EMCY
      break;
  }

#if USE_EMCY
  if (cmd_is_good)
  { // check if we previously had an error
    if ((gEF.active_sys & EMCYSBIT_PROT) != 0)
    { // this is active, clear it
      gEF.active_sys &= ~EMCYSBIT_PROT;
#if ERROR_FIELD_SIZE > 0
      MCOP_ErrField_Remove(MAKE_ERRCODE32(EMCY_PROT_ERR,NMTReq,0));
#endif
#if defined(USECB_EMCY) && USECB_EMCY
      if (0 == MCOUSER_EMCY(TRUE, EMCY_PROT_ERR, NMTReq, 0, 0, 0, 0
#if defined(USE_CANOPEN_FD) && USE_CANOPEN_FD    // CANopen FD
        ,
        0,     // dev_num - logical device number
        COFD_BASE_SPEC >> 20, // spec_num, CiA specification number
        ERST_STATE_OCC | ERST_PRIO(7),  // status - priority=7, recoverable, error occurred
        0,     // time_lo - timestamp bits 0-31, not supported
        0      // time_hi - timestamp bits 32-47, not supported
#endif // CANopen FD
      ))
#endif // USECB_EMCY
      {
        (void)MCOP_PushEMCY(EMCY_NO_ERROR, 0, 0, 0, 0, 0
#if defined(USE_CANOPEN_FD) && USE_CANOPEN_FD    // CANopen FD
          ,
          0,     // dev_num - logical device number
          COFD_BASE_SPEC >> 20, // spec_num, CiA specification number
          ERST_STATE_OCC | ERST_PRIO(7),  // status - priority=7, recoverable, error occurred
          0,     // time_lo - timestamp bits 0-31, not supported
          0      // time_hi - timestamp bits 32-47, not supported
#endif // CANopen FD
        );
      }
    }
  }
#endif // USE_EMCY
}


#if NR_OF_RPDOS > 0
/**************************************************************************
DOES:    This function initializes a receive PDO. Once initialized, the
         MicroCANopen stack automatically updates the data at offset.
NOTE:    For data consistency, the application should not read the data
         while function MCO_ProcessStack() executes.
RETURNS: nothing
**************************************************************************/
void MCO_InitRPDO (
  uint16_t PDO_NR,      // RPDO number (1-512)
  uint32_t CAN_ID,      // CAN identifier to be used (set to 0 to use default)
  uint8_t len,          // Number of data bytes in RPDO
  uint16_t offset       // Offset to data location in process image
  )
{
uint16_t lp;

#if CHECK_PARAMETERS
  // check PDO range and check node id range 1 - 127
  // 29-bit COB-IDs are not supported
  if (((PDO_NR < 1)     || (PDO_NR > 512))     ||
      (!IS_NODE_ID_VALID(MY_NODE_ID)) ||
      (len > CAN_MAX_DATA_SIZE)
     )
  {
    MCOUSER_FatalError(ERRFT_IPP);
  }
  // is size of process image exceeded?
  if (offset >= PROCIMG_SIZE_MAX)
  {
    MCOUSER_FatalError(ERRFT_PIR);
  }
#endif

  // check, if this PDO was initialized before
  lp = 0;
  while (lp < gMCOConfig.nrRPDOs)
  {
    if (gRPDOConfig[lp].PDONr == PDO_NR)
    { // found
      break;
    }
    lp++;
  }

  // is this a new entry?
  if (lp >= gMCOConfig.nrRPDOs)
  { // not yet in list, new entry
    gMCOConfig.nrRPDOs++;
    if (gMCOConfig.nrRPDOs > NR_OF_RPDOS)
    { // error, all PDOs used
      gMCOConfig.nrRPDOs--;
      MCOUSER_FatalError(ERRFT_RPDOR);
      return;
    }
  }

  // if we reach here, lp is record to use
  gRPDOConfig[lp].PDONr = PDO_NR;

  // initialize PDO
  gRPDOConfig[lp].len = len;
  gRPDOConfig[lp].offset = offset;
  if (IS_CANID_RESTRICTED(CAN_ID & ~COBID_OPT_MASK))
  { // ID not usable for PDO
    if (((CAN_ID & ~COBID_OPT_MASK) == 0) && (PDO_NR <= 4))
    { // if ID is zero and PDO <= 4 then use default CAN ID
      gRPDOConfig[lp].CANID = 0x200 + (0x100 * ((uint16_t)(PDO_NR-1))) + MY_NODE_ID;
    }
    else
    { // disable PDO
      gRPDOConfig[lp].CANID = COBID_DISABLED;
    }
  }
  else
  { // use CAN ID passed
    gRPDOConfig[lp].CANID = (COBID_TYPE) (CAN_ID & ~COBID_OPT_MASK);
  }

  if (CAN_ID & 0x80000000UL) // This is the way the auto-generated COA function call flags disabled PDOs
  { // PDO is disabled
    gRPDOConfig[lp].CANID |= COBID_DISABLED;
  }

#if USECB_ODDATARECEIVED
  gRPDOConfig[lp].map = MCO_GetRPDOMappingOffset(PDO_NR);
#endif // USECB_ODDATARECEIVED

  gRPDOConfig[lp].TType = 255;
  gMCOConfig.error_code &= 0x7F; // Signal that RPDO filter are not yet set

#if USE_DYNAMIC_PDO_MAPPING
  XPDO_ResetPDOMapEntry(1,PDO_NR);
#endif
}


/**************************************************************************
DOES:    This function changes the COBID of a RPDO
RETURNS: nothing
**************************************************************************/
void MCO_ChangeRPDOID (
  uint16_t PDO_NR,      // RPDO number (1-512)
  uint32_t CAN_ID       // CAN identifier to be used
  )
{
uint16_t lp;

#if CHECK_PARAMETERS
  // check PDO range and check node id range 1 - 127
  // 29-bit COB-IDs are not supported
  if (((PDO_NR < 1)     || (PDO_NR > 512))     ||
      (!IS_NODE_ID_VALID(MY_NODE_ID)) ||
      (CAN_ID & 0x20000000UL) || (CAN_ID == 0)
     )
  {
    MCOUSER_FatalError(ERRFT_IPP);
  }
#endif

  // find matching PDOnr
  lp = 0;
  while ((lp < NR_OF_RPDOS) && (gRPDOConfig[lp].PDONr != PDO_NR))
  {
    lp++;
  }

  if ((lp < NR_OF_RPDOS) && (gRPDOConfig[lp].PDONr == PDO_NR))
  {
    // delete old CAN filter
    if ((gRPDOConfig[lp].CANID & COBID_DISABLED) == 0)
    { // PDO is enabled
      MCOHW_ClearCANFilter(gRPDOConfig[lp].CANID);
    }
    // set new CAN ID
    gRPDOConfig[lp].CANID = (COBID_TYPE) (CAN_ID & 0x1FFFFFFFUL);
    // set new filter and enabled/disabled status
    if (CAN_ID & 0x80000000UL)
    { // PDO is disabled
      gRPDOConfig[lp].CANID |= COBID_DISABLED;
    }
    else
    {
      MCOHW_SetCANFilter(gRPDOConfig[lp].CANID);
    }
    DEBUG_PRINT(MCO_LOG_INFO,("RPRO_Nr %d init to CAN ID 0x%8.8lX\n",PDO_NR,(long unsigned int)gRPDOConfig[lp].CANID));
  }

}
#endif // NR_OF_RPDOS > 0


#if NR_OF_TPDOS > 0
/**************************************************************************
DOES:    This function initializes a transmit PDO. Once initialized, the
         MicroCANopen stack automatically handles transmitting the PDO.
         The application can directly change the data at any time.
NOTE:    For data consistency, the application should not write to the data
         while function MCO_ProcessStack executes.
RETURNS: nothing
**************************************************************************/
void MCO_InitTPDO
  (
  uint16_t PDO_NR,       // TPDO number (1-512)
  uint32_t CAN_ID,       // CAN identifier to be used (set to 0 to use default)
  uint16_t event_time,   // Transmitted every event_tim ms
  uint16_t inhibit_time, // Inhibit time in ms for change-of-state transmit
                           // (set to 0 if ONLY event_tim should be used)
  uint8_t len,           // Number of data bytes in TPDO
  uint16_t offset        // Offset to data location in process image
  )
{
uint16_t lp;
uint16_t found;

#if CHECK_PARAMETERS
  // check PDO range, node id, len range 0-8 and event time or inhibit time set
  // 29-bit COB-IDs are not supported
  if (((PDO_NR < 1)     || (PDO_NR > 512))     ||
      (!IS_NODE_ID_VALID(MY_NODE_ID)) ||
      (len > CAN_MAX_DATA_SIZE)
     )
  {
    MCOUSER_FatalError(ERRFT_IPP);
  }
  // is size of process image exceeded?
  if (offset >= PROCIMG_SIZE_MAX)
  {
    MCOUSER_FatalError(ERRFT_PIR);
  }
  // is PDO number
#endif

  // check, if this PDO was initialized before
  lp = 0;
  while (lp < gMCOConfig.nrTPDOs)
  {
    if (gTPDOConfig[lp].PDONr == PDO_NR)
    { // found
      break;
    }
    lp++;
  }

  // is this a new entry?
  if (lp >= gMCOConfig.nrTPDOs)
  { // not yet in list, new entry
    gMCOConfig.nrTPDOs++;
    if (gMCOConfig.nrTPDOs > NR_OF_TPDOS)
    { // error, all PDOs used
      gMCOConfig.nrTPDOs--;
      MCOUSER_FatalError(ERRFT_TPDOR);
      return;
    }
  }

  // if we reach here, gMCOConfig.nrTPDOs points to next record
  gTPDOConfig[lp].PDONr = PDO_NR;

  // initialize PDO
  gTPDOConfig[lp].CANmsg.LEN = len;
  gTPDOConfig[lp].offset = offset;

  if (IS_CANID_RESTRICTED(CAN_ID & ~COBID_OPT_MASK))
  { // ID not usable for PDO
    if (((CAN_ID & ~COBID_OPT_MASK) == 0) && (PDO_NR <= 4))
    { // if ID is zero, use default
      gTPDOConfig[lp].CANmsg.ID = 0x180 + (0x100 * (uint16_t)(PDO_NR-1)) + MY_NODE_ID;
    }
    else
    { // else disable PDO
      gTPDOConfig[lp].CANmsg.ID = COBID_DISABLED;
    }
  }
  else
  { // use COB ID
    gTPDOConfig[lp].CANmsg.ID = (COBID_TYPE) (CAN_ID & ~COBID_OPT_MASK);
  }

#if defined(USE_CANOPEN_FD) && (USE_CANOPEN_FD==1)
  // Default is to NOT enforce classical CAN
  gTPDOConfig[lp].CANmsg.ID &= ~COBID_FORCE_CL;
#else
  if (CAN_ID & 0x40000000UL) // This is the way the auto-generated COA function call flags RTR PDOs
  { // PDO doesn't support RTR
    gTPDOConfig[lp].CANmsg.ID |= COBID_RTR;
  }
#endif

  if (CAN_ID & 0x80000000UL) // This is the way the auto-generated COA function call flags disabled PDOs
  { // PDO is disabled
    gTPDOConfig[lp].CANmsg.ID |= COBID_DISABLED;
  }

#if USE_EVENT_TIME
  gTPDOConfig[lp].event_time = event_time;
#endif
#if USE_INHIBIT_TIME
  gTPDOConfig[lp].inhibit_time = inhibit_time;
#endif

  // handle transmission type, take from OD, if found
  found = MCO_SearchOD(0x1800+PDO_NR-1,2);
  if (found < 0xFFFF)
  {
    gTPDOConfig[lp].TType = *OD_SDOResponseTablePtr((found<<3)+4);
  }
  else
  { // set default
    gTPDOConfig[lp].TType = 255;
  }

#if USE_DYNAMIC_PDO_MAPPING
  XPDO_ResetPDOMapEntry(0,PDO_NR);
#endif
}


/**************************************************************************
DOES:    This function initializes a transmit PDO. Once initialized, the
         MicroCANopen stack automatically handles transmitting the PDO.
         The application can directly change the data at any time.
NOTE:    For data consistency, the application should not write to the data
         while function MCO_ProcessStack executes.
         This is an extended version of MCO_InitTPDO() that includes the
         transmission type. MCO_InitTPDO() is still available for backward-
         compatibility.
RETURNS: nothing
**************************************************************************/
void MCO_InitTPDOFull
  (
  uint16_t PDO_NR,       // TPDO number (1-512)
  uint32_t CAN_ID,       // CAN identifier to be used (set to 0 to use default)
  uint16_t event_time,   // Transmitted every event_tim ms
  uint16_t inhibit_time, // Inhibit time in ms for change-of-state transmit
                           // (set to 0 if ONLY event_tim should be used)
  uint8_t trans_type,    // Transmission type of the TPDO
  uint8_t len,           // Number of data bytes in TPDO
  uint16_t offset        // Offset to data location in process image
  )
{
uint16_t lp;

  MCO_InitTPDO(PDO_NR,CAN_ID,event_time,inhibit_time,len,offset);

  // verify that this PDO is initialized
  lp = 0;
  while (lp < gMCOConfig.nrTPDOs)
  {
    if (gTPDOConfig[lp].PDONr == PDO_NR)
    { // found
      gTPDOConfig[lp].TType = trans_type;
      break;
    }
    lp++;
  }

  // not in list, error
  if (lp >= gMCOConfig.nrTPDOs)
  {
    MCOUSER_FatalError(ERRFT_TPDOR);
  }
}


/**************************************************************************
DOES:    This function changes the COBID of a TPDO
RETURNS: nothing
**************************************************************************/
void MCO_ChangeTPDOID (
  uint16_t PDO_NR,      // RPDO number (1-512)
  uint32_t CAN_ID       // CAN identifier to be used
  )
{
uint16_t lp;

#if CHECK_PARAMETERS
  // check PDO range and check node id range 1 - 127
  // 29-bit COB-IDs are not supported
  if (((PDO_NR < 1)     || (PDO_NR > 512))     ||
      (!IS_NODE_ID_VALID(MY_NODE_ID)) ||
      (CAN_ID & 0x20000000UL) || (CAN_ID == 0)
     )
  {
    MCOUSER_FatalError(ERRFT_IPP);
  }
#endif

  // find matching PDOnr
  lp = 0;
  while ((lp < NR_OF_TPDOS) && (gTPDOConfig[lp].PDONr != PDO_NR))
  {
    lp++;
  }

  if ((lp < NR_OF_TPDOS) && (gTPDOConfig[lp].PDONr == PDO_NR))
  {
    // set new CAN ID
    gTPDOConfig[lp].CANmsg.ID = (COBID_TYPE) CAN_ID;
    // set new filter and enabled/disabled status
    if (CAN_ID & 0x80000000UL)
    { // PDO is disabled
      gTPDOConfig[lp].CANmsg.ID |= COBID_DISABLED;
    }
    DEBUG_PRINT(MCO_LOG_INFO,("TPDO_Nr %d set to CAN ID 0x%8.8lX\n",PDO_NR,CAN_ID));
  }

}
#endif // NR_OF_TPDOS > 0


/**************************************************************************
DOES:    This function processes the next CAN message from the CAN receive
         queue. When using an RTOS, this can be turned into a task
         triggered by a CAN receive event.
RETURNS: FALSE, if no message was processed,
         TRUE, if a CAN message received was processed
**************************************************************************/
uint8_t MCO_ProcessStackRx (
  void
  )
{
uint8_t retval = FALSE;
#if USECB_TIMEOFDAY
uint32_t millis;
uint16_t days;
#endif
#if !USE_CANOPEN_FD && (NR_OF_SDO_CLIENTS > 0)
uint8_t channel;
#endif // USE_CANOPEN_FD

  // work on next incoming messages
  // if message received
  if (MCOHW_PullMessage(&gRxCAN))
  {

#if (INDEX_FOR_DIAGNOSTICS != 0)
    gMCODiag.RxCnt++;
#endif

    // used to detect idle times on network
    gMCOConfig.last_rxtime = MCOHW_GetTime();

#if USECB_TIMEOFDAY
    if ((gRxCAN.ID == 0x100) &&
      ((MY_NMT_STATE == NMTSTATE_PREOP) || (MY_NMT_STATE == NMTSTATE_OP))
     )
    { // time stamp received, needs to be processed
      millis = gRxCAN.BUF[3];
      millis <<= 8;
      millis |= gRxCAN.BUF[2];
      millis <<= 8;
      millis |= gRxCAN.BUF[1];
      millis <<= 8;
      millis |= gRxCAN.BUF[0];
      days = gRxCAN.BUF[5];
      days <<= 8;
      days |= gRxCAN.BUF[4];
      MCOUSER_TimeOfDay(millis,days);
      retval = TRUE;
    }
#endif

    // is it an NMT master message?
    if (!retval && (gRxCAN.ID == NMT_MASTER_ID))
    {
      // nmt message is for this node or all nodes
      if ((gRxCAN.BUF[1] == MY_NODE_ID) || (gRxCAN.BUF[1] == 0))
      {
        // if we have a node ID assigned or this is a NMT reset command then process it
        if ((MY_NODE_ID != 0) || (gRxCAN.BUF[0] == NMTMSG_RESETAPP) || (gRxCAN.BUF[0] == NMTMSG_RESETCOM))
        {        
          MCO_HandleNMTRequest(gRxCAN.BUF[0]);
          retval = TRUE;
        }
      } // NMT message addressed to this node
    } // NMT master message received

#if USE_LSS_SERVER
    if ( !retval && (gRxCAN.ID == LSS_MANAGER_ID) )
    { // in LSS slave mode only process this message, ignore the rest
      LSS_HandleMsg(gRxCAN.LEN,&(gRxCAN.BUF[0]));
      retval = TRUE;
    }
#endif

#if USE_SYNC
    // Handle SYNC
    if (!retval && (gRxCAN.ID == gMCOConfig.SYNC_id))
    { // SYNC received
      if (MCOP_HandleSYNC(gRxCAN.LEN,gRxCAN.BUF[0]) != 0)
      { // SYNC PDOs processed
        retval = TRUE;
      }
    }
#endif // USE_SYNC

#if NR_OF_RPDOS > 0
    // Handle RPDO
    if (!retval && (MCO_HandleRPDO(&gRxCAN) == 1))
    { // RPDO processed
      retval = TRUE;
    }
#endif // NR_OF_RPDOS > 0

    // (U)SDO Request, handle if node is not stopped...
    if (!retval && (MY_NMT_STATE != NMTSTATE_STOP))
    {
#if defined(USE_CANOPEN_FD) && (USE_CANOPEN_FD==1)
      if (IS_CAN_ID_USDOREQUEST(gRxCAN.ID))
      {
        // USDO Request for us, handle
        if (USDO_HandleUSDORequest(&gRxCAN))
        {
          retval = TRUE;
        }
      }
#else
      if (IS_CAN_ID_SDOREQUEST(gRxCAN.ID))
      {
        // SDO Request for us, handle
        // set response ID
        gTxSDO.ID = CAN_ID_SDORESPONSE_FROM_RXID(gRxCAN.ID);
#if USE_CiA447
        PROFILE_SetSDOFromNode(0);
        if (gRxCAN.ID != 0x600 + MY_NODE_ID)
        {
          PROFILE_SetSDOFromNode((uint8_t)((gRxCAN.ID & 0x0030) >> 4) + (((gRxCAN.ID & 0x0700) - 0x0200) >> 6) + 1);
        }
#endif
        if (MCO_HandleSDORequest(&gRxCAN.BUF[0]))
        {
          retval = TRUE;
        }
      }
#endif
    }

#if USE_NODE_GUARDING
    if (!retval && (MCOP_HandleGuarding(gRxCAN.ID) == 1))
    {
      retval = TRUE;
    }
#endif

#if ! MGR_MONITOR_ALL_NODES
#if (NR_OF_HB_CONSUMER > 0)
    // Check if message received was a Heartbeat monitored
    if (!retval && MCOP_ConsumeHB(&gRxCAN))
    {
      retval = TRUE;
    }
#endif// (NR_OF_HB_CONSUMER > 0)
#endif // MGR_MONITOR_ALL_NODES

#if (NR_OF_SDO_CLIENTS > 0)
#if defined(USE_CANOPEN_FD) && USE_CANOPEN_FD    // CANopen FD
    if (IS_CAN_ID_USDORESPONSE(gRxCAN.ID))
    { // This is a USDO response
      if (USDOCLNT_HandleUSDOClientResponse(&gRxCAN))
      {
        retval = TRUE;
      }
    }
#else
    // Find the matching SDO channel
    for (channel = 0; channel < NR_OF_SDO_CLIENTS; channel++)
    { // only check initialized channels for matching SDO response message
      if ( (gSDOClientList[channel].status != SDOCL_FREE) &&
           (gRxCAN.ID == (gSDOClientList[channel].canid_response)) )
      {
        SDOCLNT_HandleSDOClientResponse(&(gSDOClientList[channel]), &(gRxCAN.BUF[0]));
        retval = TRUE;
        break;
      }
    }
#endif  // USE_CANOPEN_FD
#endif // (NR_OF_SDO_CLIENTS > 0)

#if USE_SLEEP
    if (!retval && (gRxCAN.ID >= 0x690) && (gRxCAN.ID <= 0x690+16))
    { // sleep/wakeup requst
      MCOUSER_Sleep(gRxCAN.ID-0x690,gRxCAN.BUF[0],gRxCAN.BUF[1]);
      retval = TRUE;
    }
#endif // USE SLEEP

#if USECB_RX_UNKNOWN
    if (!retval && MCOUSER_UnknownReceived(&gRxCAN))
    {
      retval = TRUE;
    }
#endif

  } // Message received

  return retval; // no message in receive queue
}


/**************************************************************************
DOES:    This function executes all sub functions required to keep the
         CANopen stack operating. It should be called frequently. When used
         in an RTOS it should be called repeatedly every RTOS time tick
         until it returns zero.
RETURNS: FALSE, if there was nothing to process
         TRUE, if functions were
**************************************************************************/
uint8_t MCO_ProcessStackTick (
  void
  )
{
uint8_t retval = NOT_SET;

  // check if this is right after boot-up
  // was set by MCO_Init
  if ((gTPDONr >= 0xFFFE) && (MY_NMT_STATE != NMTSTATE_LSS))
  { // first call or wait for bootup to be transmited
    if (gTPDONr > 0xFFFE)
    {
      // init heartbeat time
      gMCOConfig.heartbeat_timestamp = MCOHW_GetTime() + 1000;
      // send boot-up message
      if (!MCOHW_PushMessage(&gMCOConfig.heartbeat_msg))
      {
        MCOUSER_FatalError(ERROFL_HBT);
      }
      gTPDONr--;
#if USECB_NMTCHANGE
      MCOUSER_NMTChange(NMTSTATE_BOOT);
#endif
      retval = TRUE;
    }
    else
    { // Now wait for bootup to go out
      // Additional MCOHW_GetStatus() error check removed.
      // Performing error checks is application specific.
      if ((MCOHW_GetStatus() & HW_TXBSY) == 0)
      { // Bootup went out, now continue
        gMCOConfig.heartbeat_timestamp = MCOHW_GetTime() + gMCOConfig.heartbeat_time;
#if AUTOSTART
        // going into operational state
        MY_NMT_STATE = NMTSTATE_OP;
#if USE_LEDS
        gMCOConfig.LEDRun = LED_ON;
        gMCOConfig.LEDErr = LED_OFF;
#endif
#if NR_OF_TPDOS > 0
        MCO_PrepareTPDOs();
#endif
#if NR_OF_RPDOS > 0
        MCO_PrepareRPDOs();
#endif
#if USECB_NMTCHANGE
        MCOUSER_NMTChange(NMTSTATE_OP);
#endif
#else // AUTOSTART
        // going into pre-operational state
        MY_NMT_STATE = NMTSTATE_PREOP;
#if USE_LEDS
        gMCOConfig.LEDRun = LED_BLINK;
        gMCOConfig.LEDErr = LED_OFF;
#endif
#if USECB_NMTCHANGE
        MCOUSER_NMTChange(NMTSTATE_PREOP);
#endif
#endif// AUTOSTART

#if USE_EMCY
        // Only supported by MicroCANopen Plus
        // set time stamp to expired
        gEF.emcy_timestamp = MCOHW_GetTime() - 1;
        // send EMCY clear message
#if defined(USECB_EMCY) && USECB_EMCY
        if (0 == MCOUSER_EMCY(FALSE, EMCY_NO_ERROR, 0, 0, 0, 0, 0
#if defined(USE_CANOPEN_FD) && USE_CANOPEN_FD    // CANopen FD
          ,
          0,     // dev_num - logical device number
          COFD_BASE_SPEC >> 20, // spec_num, CiA specification number
          0,  // status - priority=0, recoverable, error removed
          0,     // time_lo - timestamp bits 0-31, not supported
          0      // time_hi - timestamp bits 32-47, not supported
#endif // CANopen FD
        ))
#endif // USECB_EMCY
        {
          (void)MCOP_PushEMCY(EMCY_NO_ERROR, 0, 0, 0, 0, 0
#if defined(USE_CANOPEN_FD) && USE_CANOPEN_FD    // CANopen FD
            ,
            0,     // dev_num - logical device number
            COFD_BASE_SPEC >> 20, // spec_num, CiA specification number
            0,  // status - priority=0, recoverable, error removed
            0,  // time_lo - timestamp bits 0-31, not supported
            0   // time_hi - timestamp bits 32-47, not supported
#endif // CANopen FD
          );
        }
#endif // USE_EMCY

        // return TPDONr value to default
        gTPDONr = NR_OF_TPDOS;

        retval = TRUE;
      }
    }
  }

#if USE_LSS_SERVER
  else
  {
    { // work on LSS
      LSS_DoLSS();
    }
  }
#endif

#if USE_LEDS
  if ((MCOHW_IsTimeExpired(gMCOConfig.LED_timestamp)))
  { // LED time did expire
    gMCOConfig.LED_timestamp = MCOHW_GetTime() + 200;
    MCO_SwitchLEDs();
  }
#endif // USE_LEDS

#if USE_EMCY
  // Check if an EMCY waits for transmission
  if ((retval == NOT_SET) && (MCOHW_IsTimeExpired(gEF.emcy_timestamp)))
  { // Timer overrun protection, ensure that it remains expired
    gEF.emcy_timestamp = MCOHW_GetTime() - 1;
    if (gEF.emcy_msg.ID != 0)
    { // An emergency is due for transmission
      if (MCOHW_PushMessage(&(gEF.emcy_msg)))
      {
        gEF.emcy_msg.ID = 0; // Mark as transmitted
        // now reset the inhibit time
        gEF.emcy_timestamp = MCOHW_GetTime() + gEF.emcy_inhibit;
        retval = TRUE;
      }
    }
  }
#endif

#if NR_OF_TPDOS > 0
  // is the node operational?
  if ((retval == NOT_SET) && (MY_NMT_STATE == NMTSTATE_OP))
  {
    // check next TPDO for transmission
    gTPDONr++;
    if (gTPDONr >= gMCOConfig.nrTPDOs)
    {
      gTPDONr = 0;
    }
    if (MCO_HandleTPDO(gTPDONr) == 1)
    { // TPDO was generated
      retval = TRUE;
    }
  } // if node is operational
#endif // NR_OF_TPDOS > 0

  // Check if we are in the middle of a block read transfer
#if !defined(USE_CANOPEN_FD) || (defined(USE_CANOPEN_FD) && (USE_CANOPEN_FD==0))
#if USE_BLOCKED_SDO
  if ((retval == NOT_SET) && (XSDO_BlkRdProgress()))
  { // yes, message generated
    retval = TRUE;
  }
#endif //USE_BLOCKED_SDO
#else
  if ((retval == NOT_SET) && (USDO_CheckProgress()))
  { // yes, message generated
    retval = TRUE;
  }
#endif // !USE_CANOPEN_FD

  // do we produce a heartbeat?
  if ((retval == NOT_SET) && (gMCOConfig.heartbeat_time != 0) &&
      ((MY_NMT_STATE == NMTSTATE_PREOP) || (MY_NMT_STATE == NMTSTATE_OP) || (MY_NMT_STATE == NMTSTATE_STOP))
     )
  {
    // has heartbeat time passed?
    if (MCOHW_IsTimeExpired(gMCOConfig.heartbeat_timestamp))
    {
      // transmit heartbeat message
      if (!MCOHW_PushMessage(&gMCOConfig.heartbeat_msg))
      {
        MCOUSER_FatalError(ERROFL_HBT);
      }
      // get new heartbeat time for next transmission
      gMCOConfig.heartbeat_timestamp = MCOHW_GetTime() + gMCOConfig.heartbeat_time;
      retval = TRUE;
    }
  }

#if (NR_OF_SDO_CLIENTS > 0)
  // Check if an SDO client has segmentation handling or a timeout
  if ((retval == NOT_SET) && (
#if USE_CANOPEN_FD
    USDOCLNT_USDOHandleClient()
#else
    SDOCLNT_SDOHandleClient()
#endif // USE_CANOPEN_FD
    ))
  {
    retval = TRUE;
  }
#endif // (NR_OF_SDO_CLIENTS > 0)

#if ! MGR_MONITOR_ALL_NODES
#if (NR_OF_HB_CONSUMER > 0)
  // Check Heartbeat monitors
  mHBchn++;
  if (mHBchn > NR_OF_HB_CONSUMER)
  {
    mHBchn = 1;
  }
  if(MCOP_ProcessHBCheck(mHBchn) == HBCONS_LOST)
  { // timeout occured
    retval = TRUE;
  }
#endif
#endif

#if (INDEX_FOR_DIAGNOSTICS != 0)
  // diagnostics: how often does this get executed per second
  if (gMCODiag.BurstCnt == 0)
  { // first call, we start diagnostics now
    RTOS_LOCK; // receive counter might be incremented in some interrupt
    gMCODiag.RxCnt = 0;
    RTOS_UNLOCK;
    gMCODiag.TickCnt = 0;
    gMCODiag.BurstCnt = 1;
    gMCODiag.NextSecond = MCOHW_GetTime() + 1000;
  }
  else
  {
    gMCODiag.TickCnt++;
    if(MCOHW_IsTimeExpired(gMCODiag.NextSecond))
    { // once per second
      // work on current tick counter
      gMCODiag.ProcTickPerSecCur = gMCODiag.TickCnt;
      // check min, max
      if (gMCODiag.ProcTickPerSecCur < gMCODiag.ProcTickPerSecMin)
      {
        gMCODiag.ProcTickPerSecMin = gMCODiag.ProcTickPerSecCur;
      }
      if (gMCODiag.ProcTickPerSecCur > gMCODiag.ProcTickPerSecMax)
      {
        gMCODiag.ProcTickPerSecMax = gMCODiag.ProcTickPerSecCur;
      }
      // reset counter
      gMCODiag.TickCnt = 0;

      // work on current receive counter
      RTOS_LOCK; // receive counter might be incremented in some interrupt
      gMCODiag.ProcRxPerSecCur = gMCODiag.RxCnt;
      RTOS_UNLOCK;
      // check min (!= 0), max
      if ((gMCODiag.ProcRxPerSecCur < gMCODiag.ProcRxPerSecMin) && (gMCODiag.ProcRxPerSecCur != 0))
      {
        gMCODiag.ProcRxPerSecMin = gMCODiag.ProcRxPerSecCur;
      }
      if (gMCODiag.ProcRxPerSecCur > gMCODiag.ProcRxPerSecMax)
      {
        gMCODiag.ProcRxPerSecMax = gMCODiag.ProcRxPerSecCur;
      }
      // reset counter
      RTOS_LOCK; // receive counter might be incremented in some interrupt
      gMCODiag.RxCnt = 0;
      RTOS_UNLOCK;
    }

    // diagnostics: what is the longest burst of back to back calls executing something
    if (retval == TRUE)
    { // there was something to do
      gMCODiag.BurstCnt++;
    }
    else
    { // there was nothing to do
      if (gMCODiag.BurstCnt > gMCODiag.ProcTickBurstMax)
      {
        gMCODiag.ProcTickBurstMax = gMCODiag.BurstCnt;
      }
      gMCODiag.BurstCnt = 1;
    }
  }
#endif

  if (retval == NOT_SET)
  {
    retval = FALSE;
  }
  return retval;
}


/**************************************************************************
DOES:    This function implements the main MicroCANopen protocol stack.
         It must be called frequently to ensure proper operation of the
         communication stack.
         When using an RTOS this function should not be called, instead
         MCO_ProcessStackRx() and MCO_ProcessStackTick() should be used.
         Typically it is called from the while(1) loop in main.
RETURNS: 0 if nothing was done, 1 if a CAN message was sent or received
**************************************************************************/
uint8_t MCO_ProcessStack (
  void
  )
{
  if (MCO_ProcessStackRx() > 0)
  {
    return 1;
  }
  return MCO_ProcessStackTick();
}


/**************************************************************************
DOES:    This function executes once on power up of the system.
         It performs initilaizations only done once.
RETURNS: nothing
**************************************************************************/
void MCO_DefaultPowerUp(
  void
)
{
#if USE_STORE_PARAMETERS
  NVOL_Init();
#endif
#if USE_LSS_SERVER
  LSS_Init();
#endif
}


/**************************************************************************
DOES:    Default stack and communication initialization function for most
         common use cases.
NOTE:    It is called from MCOUSER_ResetCommunication() as default if it
         doesn't implement custom initialization.
RETURNS: FALSE, if initialization was not successful
         TRUE, if initialization was successful
**************************************************************************/
uint8_t MCO_DefaultResetCommunication (
  uint8_t nodeID,       // CANopen node ID (1-126 or 0 if not yet set), ignored for simulation
  uint16_t nomBitrate,  // CAN/CAN FD nominal bitrate in kbit
#if defined(USE_CANOPEN_FD) && (USE_CANOPEN_FD==1)
  uint16_t dataBitrate, // CAN FD data bitrate in kbit
#endif
  uint16_t heartbeat    // Default heartbeat time in ms (0 for none), if
                        // not set through readable entry 1017h in OD.
)
{
// Used for functions that are only called once at first power up
static uint8_t firstPowerUp = TRUE;
// To determine bitrates and node ID to be used
uint16_t can_bps = nomBitrate;
#if defined(USE_CANOPEN_FD) && USE_CANOPEN_FD
uint16_t can_data_bps = dataBitrate;
#endif
uint8_t node_id = nodeID;
uint8_t ret_val = FALSE;
#ifdef __SIMULATION__
SIMCONFIGURATION simconfiguration;
#endif

  if (firstPowerUp)
  {
    MCO_DefaultPowerUp();
    firstPowerUp = FALSE;
  }

  DEBUG_PRINT(MCO_LOG_INFO,("\nReset Communication of MicroCANopen Plus simulation V%d.%d\n\n", _MCOPVERSION_, _MCOSUBPVERSION_));

#ifdef __SIMULATION__
  // Retrieve desired Node ID from simulation handler
  SimDriver_GetConfiguration(&simconfiguration);

#if USE_LSS_SERVER
  LSS_LoadConfiguration(&can_bps,&node_id);
  if (!(LSS_IS_NID_SET(node_id)))
  { // no node id stored in NVOL memory
    if ((simconfiguration.nodeid > 0) && (simconfiguration.nodeid < 128))
    { // simulation system has node id
      node_id = simconfiguration.nodeid;
      DEBUG_PRINT(MCO_LOG_INFO,("Node ID - assigned by simulation system: %d\n", node_id));
    }
    else
    { // no node id in simulation system, force LSS mode
      node_id = 0;
      DEBUG_PRINT(MCO_LOG_INFO,("Node ID - none assigned by simulation system\n"));
    }
  }
  else
  {// node id taken from NVOL memory
    DEBUG_PRINT(MCO_LOG_INFO,("Node ID - assigned by NVOL of simulation system: %d\n", node_id));
  }
#else // USE_LSS_SERVER
  node_id = simconfiguration.nodeid;
  DEBUG_PRINT(MCO_LOG_INFO,("Node ID - assigned by simulation system: %d\n", node_id));
#endif

#else // __SIMULATION__

#if USE_LSS_SERVER
  LSS_LoadConfiguration(&can_bps, &node_id);
  if (!(LSS_IS_NID_SET(node_id)))
  { // no node id stored in NVOL memory, take from parameters
    node_id = nodeID;
    can_bps = nomBitrate;
    DEBUG_PRINT(MCO_LOG_INFO, ("No node ID stored in NVOL memory, take from parameters: %d, %d kbps\n", node_id, can_bps));
  }
#else
 // not using LSS, not using simulation
 #if USE_XOD_ACCESS
  // extract node id and can bitrate from binary configuration
  OD_GetLayerSettings((uint8_t*)&(gBinaryConfiguration[0]), &node_id, &can_bps);
 #else
  node_id = nodeID;
  DEBUG_PRINT(MCO_LOG_INFO, ("Node ID set to %d\n", node_id));
#endif
#endif

#endif // __SIMULATION__
  
  // Initialize communication and if successful, other function blocks of the stack
  if (MCO_Init(
    can_bps,
#if defined(USE_CANOPEN_FD) && USE_CANOPEN_FD
    can_data_bps,
#endif
    node_id,
    heartbeat))
  {
    ret_val = TRUE;

#if USE_XOD_ACCESS
    // Activate alternate, loaded configuration
    if (OD_SwitchObjectDictionary(&gBinaryConfiguration[0], gBinaryConfigurationSize))
    {
      DEBUG_PRINT(MCO_LOG_INFO,("OD switched to loaded binary configuration.\n"));
    }
    else
    {
      DEBUG_PRINT(MCO_LOG_INFO,("OD NOT switched, no loaded binary configuration.\n"));
    }
#endif

    // Initialize system entries from values stored in OD
    // NOTE: This replaces the legacy CANopen Architect generated initialization macros in stackinit.h
    //       such as INITPDOS_CALLS, INITHBCONSUMER_CALLS, INITEMCYCONSUMER_CALLS.
    MCO_UpdateSystemFromOD();

#if USE_STORE_PARAMETERS
    MCOSP_GetStoredParameters();
#endif

    // configure heartbeat producer time using loaded configuration
    if (gMCOConfig.heartbeat_time > 0)
    {
      // reset heartbeat time for immediate transmission or current time plus new heartbeat time?
      // Current 3.01 conformance test 9.4 (state 04) requires this to be current time plus new heartbeat time
      // gMCOConfig.heartbeat_timestamp = MCOHW_GetTime();
      gMCOConfig.heartbeat_timestamp = MCOHW_GetTime() + gMCOConfig.heartbeat_time;
      DEBUG_PRINT(MCO_LOG_INFO,("Heartbeat producer time set to %d ms\n", gMCOConfig.heartbeat_time));
    }

#if MGR_MONITOR_ALL_NODES
    MCOHWMGR_SetCANFilter();
    MGR_ResetNodeList();
#endif

#if (NR_OF_SDO_CLIENTS > 0)
    // Set up receive filters for (U)SDO responses
    if (MCOHW_SetCANFilterRange(0x581, 0x5FF) != 1)
    {
      ret_val = FALSE;
    }

  #if defined(USE_CANOPEN_FD) && USE_CANOPEN_FD
    USDOCLNT_ResetChannels();
  #else
    SDOCLNT_ResetChannels();
  #endif
#endif

  #if USE_LSS_SERVER
    DEBUG_PRINT(MCO_LOG_INFO,("Node started in LSS mode with node ID 0x%02X", node_id));
  #else
    DEBUG_PRINT(MCO_LOG_INFO,("Node started with node ID 0x%02X", node_id));
  #endif
#if defined(USE_CANOPEN_FD) && USE_CANOPEN_FD
    DEBUG_PRINT(MCO_LOG_INFO,(" using (ignored) CAN FD bitrates (nominal/data) %d/%d kbps\n", can_bps, can_data_bps));
#else
    DEBUG_PRINT(MCO_LOG_INFO,(" using (ignored) CAN bitrate %d kbps\n", can_bps));
#endif
  }
  else
  {
    DEBUG_PRINT(MCO_LOG_INFO,("ERROR: Node initialization failed, or LSS-only.\n"));
  }

#if USE_LSS_MANAGER
#if defined(USE_CANOPEN_FD) && USE_CANOPEN_FD
  if (FALSE == LSSM_Init(NODE_ID_MIN, 1))
#else
  if (FALSE == LSSM_Init(NODE_ID_MIN))
#endif
  {
    ret_val = FALSE;
  }
#endif

  return ret_val;
}


/**************************************************************************
DOES:    Search the SDO Response table for a specifc index and subindex.
RETURNS: 0xFFFF if not found, otherwise the number of the record found
         (staring at zero)
NOTE:    Used from xpdo.c
**************************************************************************/
uint16_t MCO_SearchOD (
  uint16_t index,  // Index of OD entry searched
  uint8_t subindex // Subindex of OD entry searched
  )
{
  uint16_t i;
  uint8_t i_hi, hi;
  uint8_t i_lo, lo;
  uint8_t MEM_CONST *p;
  uint8_t MEM_CONST *r;
  uint16_t retval = 0xFFFF;

  i = 0;
  i_hi = (uint8_t) (index >> 8);
  i_lo = (uint8_t) index;
  r = OD_SDOResponseTablePtr(0);
  while (i < 0xFFFF)
  {
    p = r;
    // set r to next record in table
    r += 8;
    // skip command byte
    p++;
    lo = *p;
    p++;
    hi = *p;
    // if index in table is 0xFFFF, then this is the end of the table
    if ((lo == 0xFF) && (hi == 0xFF))
    {
      break;
    }
    else if (lo == i_lo)
    {
      if (hi == i_hi)
      {
        p++;
        // entry found?
        if (*p == subindex)
        {
          retval = i;
          break;
        }
      }
    }
    i++;
  }
  // not found
  return retval;
}


/**************************************************************************
DOES:    Search the gODProcTable from user_xxxx.c for a specific index
         and subindex.
RETURNS: 0xFFFF if not found, otherwise the number of the record found
         (starting at zero)
NOTE:    Used from xpdo.c
**************************************************************************/
uint16_t MCO_SearchODProcTable(
  uint16_t index,   // Index of OD entry searched
  uint8_t subindex  // Subindex of OD entry searched
)
{
  uint16_t j = 0;
  uint16_t compare;
  // pointer to od records
  OD_PROCESS_DATA_ENTRY MEM_CONST* pOD;
  uint16_t retval = 0xFFFF;

  // initialize pointer
  pOD = OD_ProcTablePtr(0);
  // loop until maximum table size
  while (j < 0xFFFF)
  {
    compare = pOD->idx_hi;
    compare <<= 8;
    compare += pOD->idx_lo;
    // end of table reached?
    if (compare == 0xFFFF)
    {
      break;
    }
    // index found?
    else if (compare == index)
    {
      // subindex found?
      if (pOD->subidx == subindex)
      {
#if !defined(USE_CANOPEN_FD) || (defined(USE_CANOPEN_FD) && (USE_CANOPEN_FD==0))
        // disregard entry if is FD-only but we are not processing FD
        if (!((pOD->len & COFD) == COFD))
#endif
        {
          retval = j;
          break;
        }
      }
    }
    // increment by SIZEOF(OD_PROCESS_DATA_ENTRY)
    pOD++;
    j++;
  }
  // not found
  return retval;
}


/**************************************************************************
DOES:    Generic read functions for data in little-endian format.
         Supporting PI data with different access levels. If the access
         level is not PIACC_NONE, PI_XXXX macros are used. In this case,
         ptr must point to data within the process image!
RETURNS: Read data in the respective format.
**************************************************************************/
uint32_t MCO_GenRd32(
  int pi_acc,   // PI access level
  void *ptr     // Pointer to data
)
{
  uint8_t pidata32[4];
  uint32_t data32;
  uint8_t *dptr;

  if (PIACC_NONE == pi_acc)
  {
    dptr = (uint8_t *)ptr;
  }
  else
  {
    PI_READ(pi_acc, ((uint8_t*)ptr - gProcImg), pidata32, 4);
    dptr = pidata32;
  }

  // use endianness and misaligned-safe little-endian read to generic data
  data32 =
    ((uint32_t)dptr[3] << 24) |
    ((uint32_t)dptr[2] << 16) |
    ((uint32_t)dptr[1] << 8) |
    ((uint32_t)dptr[0]);

  return (data32);
}

uint32_t MCO_GenRd24(
  int pi_acc,   // PI access level
  void *ptr     // Pointer to data
)
{
  uint8_t pidata24[3];
  uint32_t data24;
  uint8_t *dptr;

  if (PIACC_NONE == pi_acc)
  {
    dptr = (uint8_t *)ptr;
  }
  else
  {
    PI_READ(pi_acc, ((uint8_t *)ptr - gProcImg), pidata24, 3);
    dptr = pidata24;
  }

  // use endianness and misaligned-safe little-endian read to generic data
  data24 =
    ((uint32_t)dptr[2] << 16) |
    ((uint32_t)dptr[1] <<  8) |
    ((uint32_t)dptr[0]);

  return (data24);
}

uint16_t MCO_GenRd16(
  int pi_acc,   // PI access level
  void *ptr     // Pointer to data
)
{
  uint8_t pidata16[2];
  uint16_t data16;
  uint8_t *dptr;

  if (PIACC_NONE == pi_acc)
  {
    dptr = (uint8_t *)ptr;
  }
  else
  {
    PI_READ(pi_acc, ((uint8_t *)ptr - gProcImg), pidata16, 2);
    dptr = pidata16;
  }

  // use endianness and misaligned-safe little-endian read to generic data
  data16 =
    ((uint32_t)dptr[1] << 8) |
    ((uint32_t)dptr[0]);

  return (data16);
}

uint8_t MCO_GenRd8(
  int pi_acc,   // PI access level
  void *ptr     // Pointer to data
)
{
  uint8_t pidata8;
  uint8_t data8;
  uint8_t *dptr;

  if (PIACC_NONE == pi_acc)
  {
    dptr = (uint8_t *)ptr;
  }
  else
  {
    PI_READ(pi_acc, ((uint8_t *)ptr - gProcImg), &pidata8, 1);
    dptr = (uint8_t *)&pidata8;
  }

  data8 = (*((uint8_t *)dptr) & 0xFF);

  return (data8);
}


/**************************************************************************
DOES:    Generic write functions for data in little-endian format.
         Supporting PI data with different access levels. If the access
         level is not PIACC_NONE, PI_XXXX macros are used. In this case,
         ptr must point to data within the process image!
RETURNS: Read data in the respective format.
**************************************************************************/
void MCO_GenWr32(
  int pi_acc,   // PI access level
  void *ptr,    // Pointer to data
  uint32_t val  // Value
)
{
  uint8_t pidata32[4];
  uint8_t *dptr;

  if (PIACC_NONE == pi_acc)
  {
    dptr = (uint8_t *)ptr;
  }
  else
  {
    dptr = pidata32;
  }

  dptr[0] = (uint8_t)(val & 0xFF);
  dptr[1] = (uint8_t)((val >> 8) & 0xFF);
  dptr[2] = (uint8_t)((val >> 16) & 0xFF);
  dptr[3] = (uint8_t)((val >> 24) & 0xFF);

  if (PIACC_NONE != pi_acc)
  {
    PI_WRITE(pi_acc, ((uint8_t *)ptr - gProcImg), dptr, 4);
  }

  return;
}

void MCO_GenWr24(
  int pi_acc,   // PI access level
  void *ptr,    // Pointer to data
  uint32_t val  // Value
)
{
  uint8_t pidata24[3];
  uint8_t *dptr;

  if (PIACC_NONE == pi_acc)
  {
    dptr = (uint8_t *)ptr;
  }
  else
  {
    dptr = pidata24;
  }

  dptr[0] = (uint8_t)(val & 0xFF);
  dptr[1] = (uint8_t)((val >> 8) & 0xFF);
  dptr[2] = (uint8_t)((val >> 16) & 0xFF);

  if (PIACC_NONE != pi_acc)
  {
    PI_WRITE(pi_acc, ((uint8_t *)ptr - gProcImg), dptr, 3);
  }

  return;
}

void MCO_GenWr16(
  int pi_acc,   // PI access level
  void *ptr,    // Pointer to data
  uint16_t val  // Value
)
{
  uint8_t pidata16[2];
  uint8_t *dptr;

  if (PIACC_NONE == pi_acc)
  {
    dptr = (uint8_t *)ptr;
  }
  else
  {
    dptr = pidata16;
  }

  dptr[0] = (uint8_t)(val & 0xFF);
  dptr[1] = (uint8_t)((val >> 8) & 0xFF);

  if (PIACC_NONE != pi_acc)
  {
    PI_WRITE(pi_acc, ((uint8_t *)ptr - gProcImg), dptr, 2);
  }

  return;
}

void MCO_GenWr8(
  int pi_acc,   // PI access level
  void *ptr,    // Pointer to data
  uint8_t val   // Value
)
{
  uint8_t pidata8;
  uint8_t *dptr;

  if (PIACC_NONE == pi_acc)
  {
    dptr = (uint8_t *)ptr;
  }
  else
  {
    dptr = &pidata8;
  }

  *dptr = val;

  if (PIACC_NONE != pi_acc)
  {
    PI_WRITE(pi_acc, ((uint8_t *)ptr - gProcImg), dptr, 1);
  }

  return;
}


/*----------------------- END OF FILE ----------------------------------*/
