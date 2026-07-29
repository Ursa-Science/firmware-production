/**************************************************************************
MODULE:    LSSSRV
CONTAINS:  MicroCANopen Plus - Support for Layer Setting Services Server
COPYRIGHT: Embedded Systems Academy (EmSA) 2002-2024
           All rights reserved. esacademy.com
DISCLAIM:  Read and understand our disclaimer before using this code!
           www.esacademy.com/disclaim.htm
           This software was written in accordance to the guidelines at
           www.esacademy.com/software/softwarestyleguide.pdf
LICENSE:   THIS IS THE COMMERCIAL VERSION OF MICRO CANOPEN PLUS
           ONLY USERS WHO PURCHASED A LICENSE MAY USE THIS SOFTWARE
           See file license_commercial_plus.txt
VERSION:   7.17, EmSA 04-MAR-24
           $LastChangedDate: 2024-03-04 12:55:21 +0100 (Mon, 04 Mar 2024) $
           $LastChangedRevision: 5559 $
DEFINES:   USE_MICROLSS and USE_LSS_SERVER must both be defined.
***************************************************************************/

#include <string.h>
#include "mcop_inc.h"
#include "stackinit.h"

#if USE_STORE_PARAMETERS
// usage areas of the nvol memory
static uint16_t nvol_offsets[5];

// External function for NVOL storage of LSS info
extern void MCOSP_GetNVOLUsage (uint16_t pLoc[5]);

// Local unconditional function prototypes
#if !defined(NLSSLOADCONF)
static void LSS_GetBitrateFromLSSBitrate(uint8_t, uint16_t*);
#endif
#endif


#if USE_LSS_SERVER
// Only use all functions if project is configured for LSS Consumer support

#if ! USE_MICROLSS
#error "USE_LSS_SERVER and USE_MICROLSS must be defined to use this module"
#endif

#if MLSS_ONLY
#error "MLSS_ONLY option (fastscan-only) is not supported by this module"
#endif


/**************************************************************************
EXTERNAL GLOBAL VARIABLES
**************************************************************************/

// This structure holds all node specific configuration
extern MCO_CONFIG MEM_FAR gMCOConfig;

// Process Image
extern uint8_t MEM_PROC gProcImg[];


/**************************************************************************
PRIVATE VARIABLES
**************************************************************************/

typedef enum {
  LSSID_IND_VENDORID = 0,
  LSSID_IND_PRODCODE,
  LSSID_IND_REVNUM,
  LSSID_IND_SERNUM
} LSSID_INDEX;

static struct {
  uint32_t lss_id[4];        // contains the 1018h Object, the LSSID
  uint8_t  active;           // TRUE if node is in LSS mode
  uint8_t  operation_mode;   // Node is in operation mode. If not=>configuration
  uint8_t  lss_state;        // In MicroLSS mode, shows which 32-bit part we
                             // are currently working on
  uint8_t  new_node_id;      // New configured node ID
  uint8_t  old_node_id;      // Original (pre-configured) node ID
  uint8_t  new_node_bps;     // New configured bitrate
  uint8_t  node_id_set;      // Flag to indicate if node ID is configured
  uint8_t  confbt_mode;      // Node is in "Configure bit timing" mode
#if !(USE_CANOPEN_FD)
  uint8_t  match_vid;        // Match of Vendor ID from "Switch Mode Selective" command
  uint8_t  match_pid;        // Match of Product Code from "Switch Mode Selective" command
  uint8_t  match_rev;        // Match of Revision Number from "Switch Mode Selective" command
  uint8_t  idr_match_vid;    // Match of Vendor ID from "Identify Remote Slave" command
  uint8_t  idr_match_pid;    // Match of Product Code from "Identify Remote Slave" command
  uint32_t idr_rev_lo;       // Low boundary of revision from "Identify Remote Slave" command
  uint8_t  idr_match_rev_lo; // Match of Revision Number low boundary from "Identify Remote Slave" command
  uint32_t idr_rev_hi;       // High boundary of revision from "Identify Remote Slave" command
  uint8_t  idr_match_rev_hi; // Match of Revision Number high boundary from "Identify Remote Slave" command
  uint32_t idr_ser_lo;       // Low boundary of serial from "Identify Remote Slave" command
  uint8_t  idr_match_ser_lo; // Match of Serial Number low boundary from "Identify Remote Slave" command
#endif
  uint16_t actbt_sw_delay;   // Time in ms after which LSS switches the bitrate, and after which it is ready again
  uint16_t actbt_delay;      // Timestamp to switch or be ready to receive/transmit again
  uint8_t  actbt_waitswitch; // If true, we wait until we can switch the bitrate
  uint8_t  actbt_waitready;  // If true, we have swiched the bitrate and wait until we can send/receive again
} MEM_FAR mLSS;


// This structure holds the current transmit message
static CAN_MSG MEM_BUF mTxCAN;


/*******************************************************************************
PRIVATE FUNCTIONS
*******************************************************************************/

static uint32_t LSS_GetDword (uint8_t *pDat);
static void LSS_PutDword (uint32_t lvalue,uint8_t *pDat);
static void LSS_ResetSwitchMode(void);
static void LSS_InitResponse (uint8_t);
static uint8_t LSS_GetLSSBitrateFromBitrate (uint16_t);
static uint8_t LSS_SwitchModeGlobal (uint8_t *pDat);
static void LSS_ConfigureNodeID (uint8_t *pDat);
static void LSS_ConfigureBitTiming (uint8_t *pDat);
static void LSS_ActivateBitTiming (uint8_t *pDat);
static void LSS_StoreConfiguration(void);
static void LSS_InquireIdentity (uint8_t *pDat);
static void LSS_InquireNodeID (void);
static void LSS_ResetSwitchMode(void);
#if !(USE_CANOPEN_FD)
static void LSS_IdentifyRemoteSlaves (uint8_t *pDat);
static void LSS_ResetInquireRemoteSlave(void);
static void LSS_IdentifyNonconfigRemoteSlaves(void);
static void LSS_SwitchModeSelective (uint8_t *pDat);
#endif
#if USE_29BIT_LSSFEEDBACK == 1
static void LSS_FeedBackResponse (uint8_t BitChecked,uint8_t LSSNext);
#endif
#if defined(LSS_REQID_NIBBLER)
static uint32_t LSS_NibbleExtract(uint32_t from[4], uint8_t nibble);
static void LSS_NibbleFeedBack(uint8_t nibble);
static uint8_t LSS_IDBitwiseCompare(uint32_t value1[4], uint32_t value2[4], uint8_t bits_to_compare);
static void LSS_SwitchStateSelectiveFD(uint8_t* pDat);
#endif


/****************************************************************
DOES:    Extracts dword in CANopen byte order from memory location
RETURNS: uint32_t
*****************************************************************/
static uint32_t LSS_GetDword (
  uint8_t *pDat
  )
{
  return GEN_RD32(PIACC_NONE,pDat);
}


/****************************************************************
DOES:    Inserts dword in CANopen byte order into memory location
RETURNS: uint32_t
*****************************************************************/
static void LSS_PutDword (
  uint32_t lvalue,
  uint8_t *pDat
  )
{
  GEN_WR32(PIACC_NONE,pDat,lvalue);
  return;
}


/****************************************************************
DOES:    Reset the control flags for the Switch Mode Selective commands
RETURNS:
*****************************************************************/
static void LSS_ResetSwitchMode(void)
{
#if !(USE_CANOPEN_FD)
  mLSS.match_vid = FALSE;
  mLSS.match_pid = FALSE;
  mLSS.match_rev = FALSE;
#endif
  // go back to operation, even in case we were in config mode
  mLSS.operation_mode = LSS_MODE_OPERATION;

  return;
}

/****************************************************************
DOES:    Initializes CAN buffer for LSS response, sets byte 0
RETURNS:
*****************************************************************/
static void LSS_InitResponse (
  uint8_t data0
  )
{
  uint8_t i;

  mTxCAN.ID      = LSS_SERVER_ID;
  mTxCAN.LEN     = 8;
  mTxCAN.BUF[0]  = data0;
  for (i = 1; i < 8; i++)
  {
    mTxCAN.BUF[i] = 0;
  }

  return;
}


/****************************************************************
DOES:    From real bitrate value in kbps, get LSS bitrate number.
RETURNS: LSS bitrate number.
*****************************************************************/
static uint8_t LSS_GetLSSBitrateFromBitrate (
  uint16_t bitrate
  )
{
uint8_t lss_bitrate = LSS_BPS_NOTSET;
  
  switch(bitrate)
  {
    case 1000:
      lss_bitrate = LSS_BPS_1000;
      break;
    case 800:
      lss_bitrate = LSS_BPS_800;
      break;
    case 500:
      lss_bitrate = LSS_BPS_500;
      break;
    case 250:
      lss_bitrate = LSS_BPS_250;
      break;
    case 125:
      lss_bitrate = LSS_BPS_125;
      break;
    case 50:
      lss_bitrate = LSS_BPS_50;
      break;
    case 20:
      lss_bitrate = LSS_BPS_20;
      break;
    case 10:
      lss_bitrate = LSS_BPS_10;
      break;
  }

  return lss_bitrate;
}


#if !(USE_CANOPEN_FD)
/****************************************************************
DOES:    Reset the control flags for the Identify Remote Slave commands
RETURNS:
*****************************************************************/
static void LSS_ResetInquireRemoteSlave(void)
{
  mLSS.idr_match_vid     = FALSE;
  mLSS.idr_match_pid     = FALSE;
  mLSS.idr_rev_lo        = 0xFFFFFFFFUL;
  mLSS.idr_match_rev_lo  = FALSE;
  mLSS.idr_rev_hi        = 0xFFFFFFFFUL;
  mLSS.idr_match_rev_hi  = FALSE;
  mLSS.idr_ser_lo        = 0xFFFFFFFFUL;
  mLSS.idr_match_ser_lo  = FALSE;
}
#endif


#if USE_29BIT_LSSFEEDBACK == 1
/****************************************************************
DOES:    Initializes CAN buffer for LSS response with feedback
RETURNS:
*****************************************************************/
static void LSS_FeedBackResponse (
  uint8_t BitChecked, // bit currently checked
  uint8_t LSSNext // ID step checked
  )
{
uint32_t feedback = (LSS_SERVER_ID << 18);


  if ((BitChecked == 0x80) || (BitChecked == 0))
  { // respond with hi word
    feedback += (((mLSS.lss_id[LSSNext]) & 0xFFFF0000UL) >> 16);
    // add information to feedback which portion of feedback this is (0-7)
    feedback += (LSSNext << 1) << 16;
  }
  else
  { // respond with low word, set toggle
    feedback += ((mLSS.lss_id[LSSNext]) & 0x0000FFFFUL);
    // add information to feedback which portion of feedback this is (0-7)
    feedback += ((LSSNext << 1) + 1) << 16;
  }
  DEBUG_PRINT(MCO_LOG_DEBUG,(" [LSSfeedback:%d,%08xh]\n",(feedback>>16)&0x07,feedback & 0x0000FFFFUL));
  MCOHW_Push29Message(feedback);
}
#endif


/****************************************************************
DOES:    LSS Switch Mode Global Command
GLOBALS: Sets mLSS.active status flag to FALSE if end-of-LSS
RETURNS:
*****************************************************************/
static uint8_t LSS_SwitchModeGlobal (
  uint8_t *pDat
  )
{
uint8_t ret_val;

  ret_val = FALSE;

#if !(USE_CANOPEN_FD)
  LSS_ResetInquireRemoteSlave();
#endif

  if (pDat[1] == LSS_MODE_CONFIG)
  { // configuration mode
    mLSS.operation_mode = LSS_MODE_CONFIG;

    if (MY_NMT_STATE != NMTSTATE_LSS)
    {
      MY_NMT_STATE = NMTSTATE_LSS;
      mLSS.active = TRUE;
    }
  }
  else if ( (pDat[1] == LSS_MODE_OPERATION) ||
            ((pDat[1] == LSS_MODE_REACT) && (mLSS.operation_mode == LSS_MODE_PASSIVE))
          )
  { // configuration mode
    LSS_ResetSwitchMode();

    mLSS.operation_mode = LSS_MODE_OPERATION;

    // If a node is configured, a switch back into operation mode
    // means the node leaves LSS and initializes into CANopen NMT,
    // if no bit timing configuration is requested.
    if (mLSS.node_id_set && !mLSS.confbt_mode)
    {
      // Set module-internal LSS status to leave mLSS. The next call
      // of LSS_Do_LSS() will then re-initialize the node with LSS
      // parameters.
      mLSS.active = FALSE;

      ret_val = TRUE;
    }

  }
  else if (pDat[1] == LSS_MODE_PASSIVE)
  { // passive mode
    if (mLSS.operation_mode == LSS_MODE_CONFIG)
    { // only allow switch to passive when in config mode
      mLSS.operation_mode = LSS_MODE_PASSIVE;
      
      ret_val = TRUE;
    }
  }
  else
  { // unknown mode value
    // ??? 
  }
  return ret_val;
}


/****************************************************************
DOES:    LSS Configure Node ID Command
RETURNS: -
*****************************************************************/
static void LSS_ConfigureNodeID (
  uint8_t *pDat
  )
{
uint8_t node_id;

  // This command is only accepted in configuration mode but
  // outside of "configure bit timing"!
  if ((mLSS.operation_mode == LSS_MODE_CONFIG) && !(mLSS.confbt_mode))
  {
    // Prepare answer
    LSS_InitResponse(LSS_CONF_NID);

    node_id = pDat[1];  // Byte 1 in message is node id

    if (LSS_IS_NID_SET(node_id))
    {
      mLSS.new_node_id = pDat[1];

      mLSS.node_id_set = TRUE;

      // Memorize this as old node ID for inquiry command
      mLSS.old_node_id = node_id;

      if (mLSS.new_node_bps == LSS_BPS_NOTSET) // bitrate not set? Default to the current one.
      {
        mLSS.new_node_bps = LSS_GetLSSBitrateFromBitrate(gMCOConfig.Bitrate);
      }

#if USE_CANOPEN_FD
      // In CiA 1305 Server now immediately restarts using this cfg
      mLSS.operation_mode = LSS_MODE_OPERATION;
      // Set module-internal LSS status to leave mLSS. The next call
      // of LSS_Do_LSS() will then re-initialize the node with LSS
      // parameters.
      mLSS.active = FALSE;
#endif

      DEBUG_PRINT(MCO_LOG_INFO,("\nNode ID assigned by LSS: %d\n", mLSS.new_node_id));
    }
    else
    {
      mTxCAN.BUF[1] |= 1;  // Node ID out of range
      DEBUG_PRINT(MCO_LOG_WARNING,("\nNode ID assigned by LSS out of range: %d\n", mLSS.new_node_id));
    }

#if USE_CANOPEN_FD
    if (pDat[2] != 0)
    { // net ID not supported by this module
      mTxCAN.BUF[1] |= 2;  // Net ID out of range
      mTxCAN.BUF[2] = 0x01;  // Net not supported
    }
    mTxCAN.LEN = 3;
    mTxCAN.ID |= COBID_FORCE_CL;
#endif

    // Sending message
    if (!MCOHW_PushMessage(&mTxCAN))
    {
      MCOUSER_FatalError(0x0602);
    }
  }

  return;
}


/****************************************************************
DOES:    LSS Configure Bit Timing Command
RETURNS: -
*****************************************************************/
static void LSS_ConfigureBitTiming (
  uint8_t *pDat
  )
{
  // This command is only accepted in configuration mode
  if (mLSS.operation_mode == LSS_MODE_CONFIG)
  {
    LSS_InitResponse(LSS_CONF_BIT);

    if (CAN_BITRATE_SUPPORTED & (1U << pDat[2]))
    { // => bit timing supported
      mLSS.new_node_bps = pDat[2];  // Set new bitrate
      mLSS.confbt_mode  = TRUE;     // Configure bit timing mode is active
    }
    else
    { // => bit timing not supported
      mTxCAN.BUF[1] = 1;
      mLSS.confbt_mode  = FALSE;    // Configure bit timing mode is not active
    }

    mLSS.actbt_waitswitch = FALSE;
    mLSS.actbt_waitready  = FALSE;

#if USE_CANOPEN_FD
    mTxCAN.LEN = 3;
    mTxCAN.ID |= COBID_FORCE_CL;
#endif
    // Sending message
    if (!MCOHW_PushMessage(&mTxCAN))
    {
      MCOUSER_FatalError(0x0602);
    }
  }

  return;
}


/****************************************************************
DOES:    LSS Activate Bit Timing Command
RETURNS: -
*****************************************************************/
static void LSS_ActivateBitTiming (
  uint8_t *pDat
  )
{
  // This command is only accepted in configure bit timing mode
  if (mLSS.confbt_mode)
  {
    pDat++;   // Point to switch delay LSB
    mLSS.actbt_sw_delay  = *pDat++;
    mLSS.actbt_sw_delay |= (*pDat << 8);

    mLSS.actbt_waitswitch = TRUE;
    mLSS.actbt_waitready  = FALSE;

    // Calculate the timestamp to switch
    mLSS.actbt_delay = MCOHW_GetTime() + mLSS.actbt_sw_delay;
  }

  return;
}


/****************************************************************
DOES:    LSS Store Configuration Command
RETURNS: -
*****************************************************************/
static void LSS_StoreConfiguration(void)
{
#if USE_STORE_PARAMETERS
uint8_t lss_chk;
#endif

  // This command is only accepted in configuration mode
  if (mLSS.operation_mode == LSS_MODE_CONFIG)
  {
    LSS_InitResponse(LSS_STOR_CONF);

#if USE_STORE_PARAMETERS

    // Get offsets
    MCOSP_GetNVOLUsage(nvol_offsets);
    // Build checksum
    lss_chk = 0;
    lss_chk -= mLSS.new_node_id;
    lss_chk -= mLSS.new_node_bps;
    lss_chk -= NVOL_LSSENA_VAL;

    // Write data
    NVOL_WriteByte(nvol_offsets[0]+NVOL_LSSNID,mLSS.new_node_id);
    NVOL_WriteByte(nvol_offsets[0]+NVOL_LSSBPS,mLSS.new_node_bps);
    NVOL_WriteByte(nvol_offsets[0]+NVOL_LSSENA,NVOL_LSSENA_VAL);
    NVOL_WriteByte(nvol_offsets[0]+NVOL_LSSCHK,lss_chk);
    NVOL_WriteComplete();

    // Verify
    if ( (NVOL_ReadByte(nvol_offsets[0]+NVOL_LSSNID) != mLSS.new_node_id)  ||
         (NVOL_ReadByte(nvol_offsets[0]+NVOL_LSSBPS) != mLSS.new_node_bps) ||
         (NVOL_ReadByte(nvol_offsets[0]+NVOL_LSSENA) != NVOL_LSSENA_VAL)   ||
         (NVOL_ReadByte(nvol_offsets[0]+NVOL_LSSCHK) != lss_chk)
       )
    {
      // Storage media access error
      mTxCAN.BUF[1] = 0x02;
    }

#else // USE_STORE_PARAMETERS

    // If NVOL configuration storage not supported, respond
    // with "not supported"
    mTxCAN.BUF[1] = 0x01;

#endif // USE_STORE_PARAMETERS

#if USE_CANOPEN_FD
    mTxCAN.LEN = 3;
    mTxCAN.ID |= COBID_FORCE_CL;
#endif
    // Sending response
    if (!MCOHW_PushMessage(&mTxCAN))
    {
      MCOUSER_FatalError(0x0602);
    }
  }

  return;
}


/****************************************************************
DOES:    LSS Inquire Identity Commands
RETURNS: -
*****************************************************************/
static void LSS_InquireIdentity (
  uint8_t *pDat
  )
{
#if !(USE_CANOPEN_FD)
    uint32_t lvalue;  // dword for response
#endif

  // These commands are only accepted in configuration mode but
  // outside of "configure bit timing"!
  if ((mLSS.operation_mode == LSS_MODE_CONFIG) && !(mLSS.confbt_mode))
  {
      LSS_InitResponse(*pDat);

#if USE_CANOPEN_FD
      LSS_PutDword(0x5F, &mTxCAN.BUF[0]);
      LSS_PutDword(mLSS.lss_id[LSSID_IND_VENDORID], &mTxCAN.BUF[4]);
      LSS_PutDword(mLSS.lss_id[LSSID_IND_PRODCODE], &mTxCAN.BUF[8]);
      LSS_PutDword(mLSS.lss_id[LSSID_IND_REVNUM], &mTxCAN.BUF[12]);
      LSS_PutDword(mLSS.lss_id[LSSID_IND_SERNUM], &mTxCAN.BUF[16]);
      mTxCAN.LEN = 20;
#else
      switch (*pDat)
    {
      case LSS_INQ_VID:
        lvalue = mLSS.lss_id[LSSID_IND_VENDORID];
        break;
      case LSS_INQ_PID:
        lvalue = mLSS.lss_id[LSSID_IND_PRODCODE];
        break;
      case LSS_INQ_REV:
        lvalue = mLSS.lss_id[LSSID_IND_REVNUM];
        break;
      case LSS_INQ_SER:
        lvalue = mLSS.lss_id[LSSID_IND_SERNUM];
        break;
      default:
        lvalue = 0L;
        break;
    }

    // store 32-bit value in CAN message bytes 1-4
    LSS_PutDword(lvalue,&mTxCAN.BUF[1]);
#endif
    // Sending message
    if (!MCOHW_PushMessage(&mTxCAN))
    {
      MCOUSER_FatalError(0x0602);
    }
  }

  return;
}


/****************************************************************
DOES:    LSS Inquire Node ID Command
RETURNS: -
*****************************************************************/
static void LSS_InquireNodeID (void)
{
  // This command is only accepted in configuration mode but
  // outside of "configure bit timing"!
  if ((mLSS.operation_mode == LSS_MODE_CONFIG) && !(mLSS.confbt_mode))
  {
    LSS_InitResponse(LSS_INQ_NID);

#if USE_CANOPEN_FD
    mTxCAN.BUF[1] = mLSS.old_node_id;
    mTxCAN.BUF[2] = 0; // No net id support in this module
    mTxCAN.BUF[3] = mLSS.new_node_id;
    mTxCAN.BUF[4] = 0; // No net id support in this module
    mTxCAN.LEN = 5;
    mTxCAN.ID |= COBID_FORCE_CL;
#else
    // Respond with the original (not LSS-configured) Node ID
    mTxCAN.BUF[1] = mLSS.old_node_id;
#endif

    // Sending message
    if (!MCOHW_PushMessage(&mTxCAN))
    {
      MCOUSER_FatalError(0x0602);
    }
  }

  return;
}


#if !(USE_CANOPEN_FD)
/****************************************************************
DOES:    LSS Identify Remote Slaves Commands
RETURNS: -
*****************************************************************/
static void LSS_IdentifyRemoteSlaves (
  uint8_t *pDat
  )
{
// Values received in MicroLSS Master Message
uint32_t IDNumber;    // current LSS_ID Subindex IDnumber (32bit)
uint8_t  BitChecked;  // current bit requested 0..31 (0x80 for init/restart)
uint8_t  LSSSub;      // current LSS_ID subindex 0..3 (vendor,product,rev,serial)
uint8_t  LSSNext;     // next state for the MicroLSS states

uint32_t mask;        // compare mask
uint8_t found;        // return value

  // These commands are accepted in either operation and configuration mode
  // BUT outside of "configure bit timing"!
  // AND ignoring LSS master fastscan messages, if we already have a node id
  if (!mLSS.confbt_mode)
  {
    // Initialize variables
    mask = 0;
    found = 0;

    // extract 32-bit value from CAN message bytes 1-4
    IDNumber = LSS_GetDword(pDat+1);
    BitChecked = pDat[5]; // 0..31
    LSSSub = pDat[6]; // 0..3
    LSSNext = pDat[7];

    switch (*pDat)
    {
      case LSS_MICROLSS:
        if (!(mLSS.node_id_set))
        { // only proceed, if we do NOT have a node ID
          found = 0;
          if (BitChecked & 0x80)
          { // MicroLSS initialization
            found = 1;
            mLSS.lss_state = 0;
#if USE_29BIT_LSSFEEDBACK == 1
            LSS_FeedBackResponse(BitChecked,LSSNext);
#endif
          }
          else if (LSSSub == mLSS.lss_state)
          { // we are still "on go" for the next 32bit value
            mask = 0xFFFFFFFF << BitChecked;
            if ((mLSS.lss_id[LSSSub] & mask) == (IDNumber & mask))
            { // match
#if USE_29BIT_LSSFEEDBACK == 1
              if ((BitChecked == 0x10) || (BitChecked == 0))
              {
                if (LSSNext < 4)
                {
                  LSS_FeedBackResponse(BitChecked,LSSNext);
                }
              }
#endif
              found = 1;
              // Set new state as commanded by MicroLSS master
              mLSS.lss_state = LSSNext;
              if (BitChecked == 0)
              { // 32bit scan completed with success
                if (LSSSub == 3)
                { // All done and matched, scan completed, NODE IDENTIFIED
                  // Switch node to configuration mode now!
                  mLSS.operation_mode = LSS_MODE_CONFIG;
                  mLSS.active = TRUE;
                  MY_NMT_STATE = NMTSTATE_LSS;
                }
              }
            }
          }

          if (found == 1)
          { // Send a response as long as found
            // Send confirmation
            LSS_InitResponse(LSS_ID_SLAVE);
            // Sending message
            if (!MCOHW_PushMessage(&mTxCAN))
            {
              MCOUSER_FatalError(0x0602);
            }
          }
          
        }
        break;

      case LSS_REQID_VID:
        if (!(mLSS.node_id_set))
        { // only proceed, if we do NOT have a node ID
          if (IDNumber == OD_VENDOR_ID)
          {
            mLSS.idr_match_vid = TRUE;
          }
          else
          {
            LSS_ResetInquireRemoteSlave();
          }
        }
        break;

      case LSS_REQID_PID:
        if ( mLSS.idr_match_vid  &&
            (IDNumber == OD_PRODUCT_CODE) )
        {
          mLSS.idr_match_pid = TRUE;
        }
        else
        {
          LSS_ResetInquireRemoteSlave();
        }
        break;

      case LSS_REQID_REV_LO:
        if ( mLSS.idr_match_vid && mLSS.idr_match_pid &&
            (IDNumber <= OD_REVISION) )
        {
          mLSS.idr_match_rev_lo = TRUE;
        }
        else
        {
          LSS_ResetInquireRemoteSlave();
        }
        break;

      case LSS_REQID_REV_HI:
        if ( mLSS.idr_match_vid     && mLSS.idr_match_pid &&
             mLSS.idr_match_rev_lo  &&
            ((IDNumber > OD_REVISION) || (IDNumber == OD_REVISION)) )
        {
          mLSS.idr_match_rev_hi = TRUE;
        }
        else
        {
          LSS_ResetInquireRemoteSlave();
        }
        break;

      case LSS_REQID_SER_LO:
        if ( mLSS.idr_match_vid     && mLSS.idr_match_pid    &&
             mLSS.idr_match_rev_lo  && mLSS.idr_match_rev_hi &&
            (IDNumber <= mLSS.lss_id[LSSID_IND_SERNUM]) )
        {
          mLSS.idr_match_ser_lo = TRUE;
        }
        else
        {
          LSS_ResetInquireRemoteSlave();
        }
        break;

      case LSS_REQID_SER_HI:
        if ( mLSS.idr_match_vid     && mLSS.idr_match_pid    &&
             mLSS.idr_match_rev_lo  && mLSS.idr_match_rev_hi &&
             mLSS.idr_match_ser_lo  &&
            (IDNumber >= mLSS.lss_id[LSSID_IND_SERNUM]) )
        {
          // Send confirmation
          LSS_InitResponse(LSS_ID_SLAVE);
          // Sending message
          if (!MCOHW_PushMessage(&mTxCAN))
          {
            MCOUSER_FatalError(0x0602);
          }
        }
        else
        {
          LSS_ResetInquireRemoteSlave();
        }
        break;

      default:
        break;
    }
  }
}


/****************************************************************
DOES:    LSS Identify Non-configured Remote Slaves
RETURNS: -
*****************************************************************/
static void LSS_IdentifyNonconfigRemoteSlaves(void)
{
  // This command is accepted in either operation and configuration mode
  // if we are still unconfigured but outside of "configure bit timing"!
  if (!mLSS.node_id_set && !mLSS.confbt_mode)
  {
    // Send confirmation
    LSS_InitResponse(LSS_ID_NCONF_SLAVE);
    // Sending message
    if (!MCOHW_PushMessage(&mTxCAN))
    {
      MCOUSER_FatalError(0x0602);
    }
    // Force operation mode, to deal with faulty LSS master implementation or when
    // the switch global message is not received for whatever reason.
    // Not part of CiA 305.
    LSS_ResetSwitchMode();
  }

  return;
}


/****************************************************************
DOES:    LSS Switch Mode Selective Commands
RETURNS: -
*****************************************************************/
static void LSS_SwitchModeSelective (
  uint8_t *pDat
  )
{
  uint32_t lvalue;  // dword to compare
  uint8_t  command; // Command Specifier Byte

  // These commands are only accepted in operation mode but
  // outside of "configure bit timing"!
  if ((mLSS.operation_mode == LSS_MODE_CONFIG) || (mLSS.confbt_mode))
  {
    LSS_ResetSwitchMode();
  }
  else
  {
    command = *pDat++;

    // extract 32-bit value from CAN message bytes 1-4
    lvalue = LSS_GetDword(pDat);

    switch (command)
    {
      case LSS_SWMOD_VID:
        if (lvalue == mLSS.lss_id[LSSID_IND_VENDORID])
        {
          mLSS.match_vid = TRUE;
        }
        else
        {
          LSS_ResetSwitchMode();
        }
        break;

      case LSS_SWMOD_PID:
        if ( mLSS.match_vid  &&
            (lvalue == mLSS.lss_id[LSSID_IND_PRODCODE]) )
        {
          mLSS.match_pid = TRUE;
        }
        else
        {
          LSS_ResetSwitchMode();
        }
        break;

      case LSS_SWMOD_REV:
        if ( mLSS.match_vid && mLSS.match_pid &&
            (lvalue == mLSS.lss_id[LSSID_IND_REVNUM]) )
        {
          mLSS.match_rev = TRUE;
        }
        else
        {
          LSS_ResetSwitchMode();
        }
        break;

      case LSS_SWMOD_SER:
        if ( mLSS.match_vid && mLSS.match_pid && mLSS.match_rev &&
            (lvalue == mLSS.lss_id[LSSID_IND_SERNUM]) )
        {
          // Send confirmation
          LSS_InitResponse(LSS_SWMOD_RESP);
          // Sending message
          if (!MCOHW_PushMessage(&mTxCAN))
          {
            MCOUSER_FatalError(0x0602);
          }

          // This node is in configuration mode now!
          mLSS.operation_mode = LSS_MODE_CONFIG;

          if (MY_NMT_STATE != NMTSTATE_LSS)
          {
            MY_NMT_STATE = NMTSTATE_LSS;
            mLSS.active = TRUE;
          }
        }
        else
        {
          LSS_ResetSwitchMode();
        }

      default:
        break;
    }
  }

  return;
}
#endif


#if defined(LSS_REQID_NIBBLER)
/****************************************************************
DOES:    Returns a nibble from uint32_t 1018h_ID_[4]
         Most significant nibbles first
RETURNS: Requested nibble
*****************************************************************/
static uint32_t LSS_NibbleExtract(
    uint32_t from[4], // 128bit LSS ID
    uint8_t nibble    // nibble from 0 to 31
)
{
    uint32_t dat, shift;

    dat = from[nibble >> 3]; // select 32bit from LSS ID
    shift = ((31 - nibble) & 0x07) * 4; // select

    return (dat >> shift) & 0x0F;
}


/****************************************************************
DOES:    Initializes CAN buffer for LSS response with feedback
RETURNS:
*****************************************************************/
static void LSS_NibbleFeedBack (
  uint8_t nibble // nibble from 0 to 31
  )
{
uint32_t feedback = 0;
CAN_MSG can_fb;

  if (nibble < 32)
  {
#if !(USE_29BIT_LSSFEEDBACK)
    feedback = LSS_NibbleExtract(mLSS.lss_id,nibble);
    DEBUG_PRINT(MCO_LOG_DEBUG,(" [LSSfeedback:%02lxh]\n", feedback & 0x0000000FUL));
    // for this feedback, enforce classical transmission
    can_fb.ID = COBID_FORCE_CL + LSS_FBMIN_ID + (feedback & 0x0F);
    can_fb.LEN = 0;
    MCOHW_PushMessage(&can_fb);
#endif
  }
  else if (nibble == 32)
  {
    DEBUG_PRINT(MCO_LOG_DEBUG,(" [LSSselected]\n"));
    can_fb.ID = LSS_SERVER_ID;
    can_fb.LEN = 20;
    LSS_PutDword(0x64,&(can_fb.BUF[0]));
    LSS_PutDword(mLSS.lss_id[LSSID_IND_VENDORID],&(can_fb.BUF[4]));
    LSS_PutDword(mLSS.lss_id[LSSID_IND_PRODCODE],&(can_fb.BUF[8]));
    LSS_PutDword(mLSS.lss_id[LSSID_IND_REVNUM],&(can_fb.BUF[12]));
    LSS_PutDword(mLSS.lss_id[LSSID_IND_SERNUM],&(can_fb.BUF[16]));
    MCOHW_PushMessage(&can_fb);
  }
}


/****************************************************************
DOES:    Comparing a number of bits from 2 uint32_t[4]
RETURNS: TRUE on match
*****************************************************************/
static uint8_t LSS_IDBitwiseCompare(
    uint32_t value1[4], 
    uint32_t value2[4], 
    uint8_t bits_to_compare // 1 to 128
    ) 
{
    uint8_t ret_val = TRUE;
    uint8_t lp = 0;
    uint32_t mask;
    uint32_t bits;

    bits = bits_to_compare;
    while ((ret_val) && (lp < 4))
    { // loop through 4 LSS ID values of 32bit
        // Create a mask with the specified number of bits
        if (bits >= 32) 
        { // use all bits
            mask = 0xFFFFFFFF;
        }
        else 
        { // only use 'bits' most significant bits
            mask = ((1 << bits) - 1) << (32 - bits);
        }
        // Now compare
        if ((value1[lp] & mask) != (value2[lp] & mask))
        { // no match
            ret_val = FALSE;
        }
        if (bits > 32)
        {
            bits -= 32;
            lp++;
        }
        else
        {
            lp = 4;
        }
    }
    return ret_val;
}


/****************************************************************
DOES:    LSS Identify using LSS Nibble by Nibble
         "Switch state selective FD"
RETURNS: -
*****************************************************************/
static void LSS_SwitchStateSelectiveFD (
  uint8_t *pDat // 20 data bytes of LSS Master message
  )
{
uint8_t RT_request;   // Real-time request in ms
uint8_t nibble_chk;   // nibble number to match, 0 to 31, 32 for complete
uint32_t buf32[4];    // Buffer to hold LSS ID received in request
uint8_t lp;
uint8_t match;        // TRUE, while we are a match

  RT_request = pDat[1];
  
  if ( (!mLSS.confbt_mode) && (RT_request > MY_REALTIME_DELAY+9) )
  { // not in bitrate configuration mode, real-time requirements met
    nibble_chk = pDat[2];
    for (lp = 0; lp < 4; lp++) 
    {
      buf32[lp] = LSS_GetDword(&(pDat[(lp + 1) * 4]));
    }
    match = LSS_IDBitwiseCompare(&(mLSS.lss_id[0]), &(buf32[0]),nibble_chk*4);
    if (match) 
    { // match!
      if (nibble_chk == 32)
      { // reached the end, we are it!
        // Switch node to configuration mode now!
        mLSS.operation_mode = LSS_MODE_CONFIG;
        mLSS.active = TRUE;
        MY_NMT_STATE = NMTSTATE_LSS;
      }
#if USE_29BIT_LSSFEEDBACK == 1
      else
      {
        switch (nibble_chk)
        {
        case 0:
        case 8:
        case 16:
        case 24:
          LSS_FeedBackResponse(0,nibble_chk>>3);
          break;
        case 4:
        case 12:
        case 20:
        case 28:
          LSS_FeedBackResponse(1,nibble_chk>>3);
          break;
        default:
          break;
        }
      }
#endif
      // we are matched, provide feedback
      LSS_NibbleFeedBack(nibble_chk); // send next nibble
    }
  }
}
#endif


/*******************************************************************************
GLOBAL FUNCTIONS
*******************************************************************************/

/****************************************************************
DOES:    Process all LSS messages.
RETURNS:
*****************************************************************/
void LSS_HandleMsg (
  uint8_t Len,
  uint8_t *pDat
  )
{
  // After "Activate Bit Timing Parameter" command, don't execute
  // any commands for the time of 2*mLSS.actbt_sw_delay (ms) => LSS_Do_LSS()
  // Allow LSS commands only in pure-LSS and stopped mode
  if ( !(mLSS.actbt_waitswitch) && !(mLSS.actbt_waitready) )
  {
#if ! (USE_CANOPEN_FD)
    if (Len == 8)   // must be 8 bytes long!
    // Only in classical CANopen
#endif
    {      
      switch (*pDat)
      {
        case LSS_SWMOD_GLOB:
          LSS_SwitchModeGlobal(pDat);
          break;

#if ! (USE_CANOPEN_FD)
        case LSS_SWMOD_VID:
        case LSS_SWMOD_PID:
        case LSS_SWMOD_REV:
        case LSS_SWMOD_SER:
          LSS_SwitchModeSelective(pDat);
          break;
#endif

        case LSS_CONF_NID:
            LSS_ConfigureNodeID(pDat);
          break;

        case LSS_CONF_BIT:
            LSS_ConfigureBitTiming(pDat);
          break;
        case LSS_ACT_BIT:
            LSS_ActivateBitTiming(pDat);
          break;
        case LSS_STOR_CONF:
            LSS_StoreConfiguration();
          break;

#if USE_CANOPEN_FD
        case LSS_INQ_VID:
            LSS_InquireIdentity(pDat);
            break;
#else
        case LSS_INQ_VID:
        case LSS_INQ_PID:
        case LSS_INQ_REV:
        case LSS_INQ_SER:
            LSS_InquireIdentity(pDat);
          break;

        case LSS_REQID_VID:
        case LSS_REQID_PID:
        case LSS_REQID_REV_LO:
        case LSS_REQID_REV_HI:
        case LSS_REQID_SER_LO:
        case LSS_REQID_SER_HI:
            LSS_IdentifyRemoteSlaves(pDat);
            break;

        case LSS_MICROLSS:
            LSS_IdentifyRemoteSlaves(pDat);
            break;

        case LSS_REQID_NCONF:
            LSS_IdentifyNonconfigRemoteSlaves();
            break;
#endif

        case LSS_INQ_NID:
            LSS_InquireNodeID();
          break;

#ifdef LSS_REQID_NIBBLER
        case LSS_REQID_NIBBLER: // only if this device is unconfigured
          if (!(LSS_IS_NID_SET(MY_NODE_ID)))
          {
            LSS_SwitchStateSelectiveFD(pDat);
          }
        break;
        case (LSS_REQID_NIBBLER+1):
          LSS_SwitchStateSelectiveFD(pDat); // in all cases
        break;
#endif

        default:
          break;
      }
    }
  }

  return;
}


/****************************************************************
DOES:    Check and update LSS state machine
RETURNS: FALSE: LSS not currently active
         TRUE:  Otherwise (LSS is still in process)
*****************************************************************/
uint8_t LSS_DoLSS(void)
{

#if USE_LEDS
  if (!(LSS_IS_NID_SET(MY_NODE_ID)))
  {
      if (MY_NMT_STATE != NMTSTATE_STOP)
      { // in stopped mode, still show blinking pattern for stopped,
        // else if LEDs are used, toggle all 50ms
        if (MCOHW_IsTimeExpired(gMCOConfig.LED_timestamp))
        {
          if (gMCOConfig.LED_timestamp & 1)
          {
            LED_RUN_ON;
            LED_ERR_OFF;
            gMCOConfig.LED_timestamp = (MCOHW_GetTime() + 50) & 0xFFFE;
          }
          else
          {
            LED_RUN_OFF;
            LED_ERR_ON;
            gMCOConfig.LED_timestamp = (MCOHW_GetTime() + 50) | 0x0001;
          }
        }
      }
  }
#endif //USE_LEDS
 
  // Has LSS just been finished?
  if ((mLSS.active == FALSE) && (MY_NMT_STATE == NMTSTATE_LSS))
  {
#if USE_LEDS
    LED_RUN_OFF;
    LED_ERR_OFF;
#endif // USE_LEDS

    // Re-init
    // apply new node ID NOW
    MY_NODE_ID = mLSS.new_node_id;
    MCOUSER_ResetCommunication();
    mLSS.active = FALSE;
  }

  // After "Activate Bit Timing" command, wait until we can actually
  // switch the bitrate
  if (mLSS.actbt_waitswitch)
  {
    if (MCOHW_IsTimeExpired(mLSS.actbt_delay))
    {
      // Calculate the timestamp to be ready to receive/transmit again
      mLSS.actbt_delay = MCOHW_GetTime() + mLSS.actbt_sw_delay;

      // Set bitrate for CANopen stack
      LSS_GetBitrateFromLSSBitrate(mLSS.new_node_bps,&gMCOConfig.Bitrate);

      // Reset CAN controller, set new bitrate
#if defined(USE_CANOPEN_FD) && (USE_CANOPEN_FD==1)
      MCOHW_Init(gMCOConfig.Bitrate, gMCOConfig.BRSBitrate);
#else
      MCOHW_Init(gMCOConfig.Bitrate);
#endif // not CANopen FD

      // Set receive filter for LSS master message
      if (!MCOHW_SetCANFilter(LSS_MANAGER_ID))
      {
        MCOUSER_FatalError(0x0601);
      }

       // We are no longer in "configure bit timing" mode
      mLSS.confbt_mode      = FALSE;

      mLSS.actbt_waitswitch = FALSE;
      mLSS.actbt_waitready  = TRUE;
    }
  }

  // After the bitrate has changed, wait until we are ready to receive/
  // transmit again
  if (mLSS.actbt_waitready)
  {
    if (MCOHW_IsTimeExpired(mLSS.actbt_delay))
    {
      mLSS.actbt_waitready = FALSE;
    }
  }

  // Sanity check
  if (!(LSS_IS_NID_SET(MY_NODE_ID)))
  { // if we do not have a node id, keep LSS active
    mLSS.active = TRUE;
  }

  return mLSS.active;
}


/****************************************************************
DOES:    Gets the LSS ID that LSS_Init() determined.
         Call after LSS_Init().
RETURNS: LSS ID in passed array - has to be writable.
*****************************************************************/
void LSS_GetLSSID(
  uint32_t lssid[4]
)
{
LSSID_INDEX lp;
  
  for(lp = LSSID_IND_VENDORID; lp <= LSSID_IND_SERNUM; lp++)
  {
    lssid[lp] = mLSS.lss_id[lp];
  }
}


/****************************************************************
DOES:    Sets the LSS ID. Use, if LSS ID is not hard coded and
         does not come from process image locations.
         Call after LSS_Init() and before LSS processing starts.
RETURNS: nothing
*****************************************************************/
void LSS_SetLSSID(
  uint32_t lssid[4]
)
{
LSSID_INDEX lp;
  
  for(lp = LSSID_IND_VENDORID; lp <= LSSID_IND_SERNUM; lp++)
  {
    mLSS.lss_id[lp] = lssid[lp];
  }
}


/****************************************************************
DOES:    Initialize LSS mechanism (variables etc.)
GLOBALS: Sets mLSS.active status flag to TRUE
RETURNS: -
*****************************************************************/
void LSS_Init(
  void
  )
{
uint8_t i;
static uint32_t len; 
static uint8_t *dat;

  LSS_ResetSwitchMode();
#if !(USE_CANOPEN_FD)
  LSS_ResetInquireRemoteSlave();
#endif

  // Init LSS ID with hard coded values
  mLSS.lss_id[LSSID_IND_VENDORID] = OD_VENDOR_ID;
  mLSS.lss_id[LSSID_IND_PRODCODE] = OD_PRODUCT_CODE;
  mLSS.lss_id[LSSID_IND_REVNUM] = OD_REVISION;
#ifdef OD_SERIAL
  mLSS.lss_id[LSSID_IND_SERNUM] = OD_SERIAL;
#endif
  // Overwrite defaults with data from dynamic Object Dictionary, if available
  for (i=LSSID_IND_VENDORID; i<=LSSID_IND_SERNUM; i++)
  { // if data is in process image, take it from there
    if (OD_FindODDataEntry(1,0x1018,i+1,&len,&dat))
    {
      mLSS.lss_id[i] = GEN_RD32(PIACC_NONE,dat);
    }
  }
#if USECB_ODSERIAL
  // Overwrite serial number, if custom
  mLSS.lss_id[LSSID_IND_SERNUM] = MCOUSER_GetSerial();
#endif
  
  // Init LSS state machine
  mLSS.lss_state = 0;

  // After reset, the node is in operation mode
  mLSS.operation_mode = LSS_MODE_OPERATION;

  // After reset, the node is not in "configure bit timing" mode
  mLSS.confbt_mode    = FALSE;

  // Pre-set node ID to default values
  mLSS.new_node_id    = LSS_NID_NONE;

  // Memorize the old node ID for inquiry command
  mLSS.old_node_id    = LSS_NID_NONE;

  // currently no node ID set
  mLSS.node_id_set = FALSE;

  return;
}

#endif // #if USE_LSS_SERVER


// Allow use of the functions below that apply to NVOL configuration of node ID and bitrate
// also when LSS is not otherwise enabled.

#if !defined(NLSSLOADCONF) && USE_STORE_PARAMETERS
/****************************************************************
DOES:    From LSS bitrate number, get real bitrate in kbps.
RETURNS: Real bitrate in *bitrate, but only set if valid.
*****************************************************************/
static void LSS_GetBitrateFromLSSBitrate(
  uint8_t  lss_bitrate,
  uint16_t* bitrate
)
{
  switch (lss_bitrate)
  {
  case LSS_BPS_NOTSET:
  default:
    // don't change bitrate, return the current value
    break;
  case LSS_BPS_1000:
    *bitrate = 1000;
    break;
  case LSS_BPS_800:
    *bitrate = 800;
    break;
  case LSS_BPS_500:
    *bitrate = 500;
    break;
  case LSS_BPS_250:
    *bitrate = 250;
    break;
  case LSS_BPS_125:
    *bitrate = 125;
    break;
  case LSS_BPS_50:
    *bitrate = 50;
    break;
  case LSS_BPS_20:
    *bitrate = 20;
    break;
  case LSS_BPS_10:
    *bitrate = 10;
    break;
  }

  return;
}


/****************************************************************
DOES:    LSS Load Configuration Command
RETURNS: -
*****************************************************************/
void LSS_LoadConfiguration (
  uint16_t *Bitrate,  // returns CAN bitrate in kbit
  uint8_t *Node_ID    // returns CANopen node ID (0-127)
  )
{
#if USE_STORE_PARAMETERS
uint8_t cfg[4];
#endif

  // only read configuration if node id is unknown, otherwise return current config
  if (MY_NODE_ID != 0)
  { 
    *Bitrate = gMCOConfig.Bitrate;
    *Node_ID = MY_NODE_ID;
  }
  else
  {
#if USE_STORE_PARAMETERS
    // NVOL_Init() must have been called before reaching here

    // Get offsets
    MCOSP_GetNVOLUsage(nvol_offsets);
    
    // Read record
    cfg[NVOL_LSSNID] = NVOL_ReadByte(nvol_offsets[0]+NVOL_LSSNID);
    cfg[NVOL_LSSBPS] = NVOL_ReadByte(nvol_offsets[0]+NVOL_LSSBPS);
    cfg[NVOL_LSSENA] = NVOL_ReadByte(nvol_offsets[0]+NVOL_LSSENA);
    cfg[NVOL_LSSCHK] = NVOL_ReadByte(nvol_offsets[0]+NVOL_LSSCHK);
    
    // Record OK?
    if (LSS_CheckConfiguration(cfg))
    { // data ok

      if (cfg[NVOL_LSSENA] == NVOL_LSSENA_VAL)
      {
        *Node_ID = cfg[NVOL_LSSNID];
      }
      else
      {
        *Node_ID = 0;
      }
      LSS_GetBitrateFromLSSBitrate(cfg[NVOL_LSSBPS],Bitrate);
#if USE_LSS_SERVER
      mLSS.new_node_bps = cfg[NVOL_LSSBPS]; // defaults to the currently configured bitrate
#endif
    }
    else
    {
      *Bitrate = 125;
      *Node_ID = 0;
#if USE_LSS_SERVER
      mLSS.new_node_bps = LSS_BPS_NOTSET; // defaults to the currently set bitrate
#endif
    }
#else // USE_STORE_PARAMETERS
    *Bitrate = 0;
    *Node_ID = 0;
#endif // USE_STORE_PARAMETERS
  }

#if USE_LSS_SERVER
  if (!(LSS_IS_NID_SET(*Node_ID)))
  { // No node ID, remain in LSS mode
    mLSS.node_id_set    = FALSE;
    MY_NMT_STATE = NMTSTATE_LSS;
    mLSS.active = TRUE;
  }
  else
  { // The Node ID is set
    mLSS.new_node_id = *Node_ID;
    mLSS.node_id_set    = TRUE;
  }
#endif

  return;
}


/****************************************************************
DOES:    Verifies a 4-byte configuration record
         from NVOL_LSSNID to NVOL_LSSCHK for plausible values
RETURNS: TRUE, if configuration is valid
*****************************************************************/
uint8_t LSS_CheckConfiguration (
  uint8_t cfg[4]
)
{
uint8_t chk;
uint8_t ret_val = FALSE;
  
  // Build checksum
  chk = 0;
  chk -= cfg[NVOL_LSSNID];
  chk -= cfg[NVOL_LSSBPS];
  chk -= cfg[NVOL_LSSENA];
  
  if (chk == cfg[NVOL_LSSCHK])
  { // Checksum OK
    if ( ( ((cfg[NVOL_LSSNID] > 0) && (cfg[NVOL_LSSNID] < 128)) || (cfg[NVOL_LSSNID] == 0xFF) ) && 
         ((cfg[NVOL_LSSBPS] <= LSS_BPS_10) || (cfg[NVOL_LSSBPS] == 0xFF)) &&
         ((cfg[NVOL_LSSENA] == NVOL_LSSENA_VAL) || (cfg[NVOL_LSSENA] == NVOL_DOLSS_VAL))
       )
    { // Plausibility checks ok
      ret_val = TRUE;
    }
  }

return ret_val;
}
#endif


/*******************************************************************************
END OF FILE
*******************************************************************************/
