/**************************************************************************
MODULE:    MCOP
CONTAINS:  MicroCANopen Plus implementation
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

// this structure holds the CAN message for SDO responses or aborts
extern CAN_MSG gTxSDO;


/**************************************************************************
GLOBAL/MODULE VARIABLES
***************************************************************************/

#if ! MGR_MONITOR_ALL_NODES
#if (NR_OF_HB_CONSUMER > 0)
HBCONS_CONFIG MEM_FAR gHBCons[NR_OF_HB_CONSUMER];
#endif // (NR_OF_HB_CONSUMER > 0)
#endif // MGR_MONITOR_ALL_NODES

#if USE_EMCY
EMCY_CONFIG MEM_FAR gEF; // Emergency configuration
#endif


/**************************************************************************
PUBLIC FUNCTIONS
***************************************************************************/

/**************************************************************************
DOES:    This function reads data from the process image and copies it
         to an OUTPUT location
RETURNS: Number of bytes that were copied
**************************************************************************/
uint8_t MCO_ReadProcessData (
  uint8_t MEM_PROC *pDest, // Destination pointer
  uint8_t length, // Number of bytes to copy
  uint16_t offset // Offset of source data in process image
  )
{
  PI_READ(PIACC_APP,offset,pDest,length);
  return length;
}


/**************************************************************************
DOES:    This function writes data from an INPUT location to the process
         image
RETURNS: Number of bytes that were copied
**************************************************************************/
uint8_t MCO_WriteProcessData (
  uint16_t offset, // Offset of destination data in process image
  uint8_t length,  // Number of bytes to copy
  uint8_t MEM_PROC *pSrc // Source pointer
  )
{
  PI_WRITE(PIACC_APP,offset,pSrc,length);
  return length;
}


/**************************************************************************
DOES:    Obtain PI offset and length for short (1-4 bytes) OD entries by
         index/subindex.
RETURNS: 0: Entry exists, results valid
         1: Error - entry not found in PI, or is no short entry
**************************************************************************/
uint8_t MCO_GetODProcPIOffsLenByIndex(
  uint16_t index,    // OD index of entry in PI
  uint8_t subindex,  // OD subindex of entry in PI
  uint16_t* offset,  // out: offset of OD entry in PI
  uint8_t* len       // out: length of OD entry in PI
)
{
  uint8_t ret_stat;
  uint16_t pt_ind;
  uint16_t pi_offs;
  // pointer to an entry in gODProcTable
  OD_PROCESS_DATA_ENTRY MEM_CONST* pOD;

  pt_ind = MCO_SearchODProcTable(index, subindex);
  if (pt_ind != 0xFFFFu)
  {
    pOD = OD_ProcTablePtr(pt_ind);
    pi_offs = pOD->off_hi;
    pi_offs <<= 8;
    pi_offs += pOD->off_lo;
    *offset = pi_offs;
    *len = (pOD->len & 0x07u);
    ret_stat = 0;
  }
  else
  {
    ret_stat = 1;
  }

  return(ret_stat);
}


/**************************************************************************
DOES:    Obtain PI offset and length for any PI-based OD entries by
         index/subindex.
RETURNS: 0: Entry exists, results valid
         1: Error - entry not found in PI, or is no short entry
**************************************************************************/
uint8_t MCO_GetPIOffsLenByIndex(
  uint16_t index,    // OD index of entry in PI
  uint8_t subindex,  // OD subindex of entry in PI
  uint16_t* offset,  // out: offset of OD entry in PI
  uint16_t* len      // out: length of OD entry in PI
)
{
  uint8_t ret_stat = 1;
  uint8_t len8;

  if (0 == MCO_GetODProcPIOffsLenByIndex(index, subindex, offset, &len8))
  {
    *len = (uint16_t)len8;
    ret_stat = 0;
  }
#if USE_EXTENDED_SDO
  else
  {
    uint8_t access;
    uint32_t len32;
    uint8_t* p_dat;

    if (0xFF != XSDO_SearchODGenTable(index, subindex, &access, &len32, &p_dat))
    {
      if ((p_dat >= &gProcImg[0]) && (p_dat < (&gProcImg[0] + PROCIMG_SIZE_MAX)))
      {
        *offset = p_dat - &gProcImg[0];
        *len = (uint16_t)len32;
        ret_stat = 0;
      }
    }
  }
#endif // USE_EXTENDED_SDO
  return ret_stat;
}


/**************************************************************************
DOES:    Read OD entries by index/subindex.
         Short entries up to 4 bytes read into variables and obey byte order,
         (potentially) longer entries we read into a byte array.
NOTE:    This function executes a search for the index/subindex selected.
         For better performance data access, use MCO_ReadProcessData()
RETURNS: 0: Entry read ok
         1: Error - entry not found
         2: Error - data length mismatch
         3: Error - data length exceeded maximum
         4: Error - parameter error
**************************************************************************/
uint8_t MCO_ReadValByIndex(
  uint16_t index,     // OD index of entry in PI
  uint8_t subindex,   // OD subindex of entry in PI
  uint8_t len_type,   // 1-4: read into uint8_t..uint32_t, 0: read into byte array
  uint16_t* len_read, // in: if len_type==0 give max. read length
                      // out: must be !=NULL to return read length for larger entries
  void* data          // pointer to write data (variable or byte array)
)
{
  uint8_t found;
  uint8_t val8;
  uint16_t val16;
  uint32_t val32;
  uint16_t pi_offs;
  uint16_t entry_len;
  uint8_t* p_dat;
  int pi_acc;

  if ((len_type > 4) ||
    ((len_type == 0) && (len_read == NULL)))
    return 4; // parameter error

  found = FALSE;

  // search table with constants
  val16 = MCO_SearchOD(index,subindex);
  // entry found?
  if (val16 < 0xFFFFu)
  {
    const uint8_t *sdortptr;

    found = TRUE;
    sdortptr = OD_SDOResponseTablePtr(val16<<3);
    // point to data bytes in the constant SDO reply table
    p_dat = (uint8_t *)sdortptr + 4;
    pi_acc = PIACC_NONE;
    // length of the entry is decoded from SDO command specifier
    entry_len = 4-(((*sdortptr) & 0x0C) >> 2);
  }
  else if (0 == MCO_GetODProcPIOffsLenByIndex(index, subindex, &pi_offs, &val8))
  {
    found = TRUE;
    entry_len = (uint16_t)val8;
    p_dat = &gProcImg[pi_offs];
    pi_acc = PIACC_APP;
  }
#if USE_EXTENDED_SDO
  else
  {
    uint8_t access;

    val8 = XSDO_SearchODGenTable(index, subindex, &access, &val32, &p_dat);
    if (val8 != 0xFFu)
    {
      found = TRUE;
      entry_len = (uint16_t)val32;
#if USE_GENOD_PTR
      pi_acc = PIACC_NONE;
#else
      pi_acc = PIACC_APP;
#endif
    }
  }
#endif // USE_EXTENDED_SDO

  if (!found)
    return 1;

  if (len_type == 0)
  {
    if (entry_len > *len_read)
      return 3; // data length exceeded maximum
  }
  else // len_type is 1..4
  {
    if (entry_len != len_type)
      return 2; // data length mismatch
  }

  switch (len_type)
  {
    case 0:
      if (pi_acc == PIACC_NONE)
      {
        MEM_CPY_FAR(data, p_dat, entry_len);
      }
      else
      {
        PI_READ(pi_acc, (p_dat - gProcImg), data, entry_len);
      }
      *len_read = entry_len;
      break;
    case 1:
      if (pi_acc == PIACC_NONE)
      {
        MEM_CPY_FAR(data, p_dat, 1);
      }
      else
      {
        PI_READ(pi_acc, (p_dat - gProcImg), data, 1);
      }
      break;
    case 2:
      val16 = GEN_RD16(pi_acc, p_dat);
      MEM_CPY_FAR(data, &val16, 2);
      break;
    case 3:
      val32 = GEN_RD24(pi_acc, p_dat);
      MEM_CPY_FAR(data, &val32, 3);
      break;
    case 4:
      val32 = GEN_RD32(pi_acc, p_dat);
      MEM_CPY_FAR(data, &val32, 4);
      break;
  }

  return 0;
}


/**************************************************************************
DOES:    Write OD entries to PI by index/subindex.
         Short entries up to 4 bytes write from variables and obey byte order,
         longer entries we write from a byte array.
NOTE:    This function executes a search for the index/subindex selected.
         For better performance data access, use MCO_WriteProcessData()
RETURNS: 0: Entry write ok
         1: Error - entry not found in PI
         2: Error - data length mismatch
         3: Error - data length exceeded maximum
         4: Error - parameter error
**************************************************************************/
uint8_t MCO_WriteValByIndex(
  uint16_t index,     // OD index of entry in PI
  uint8_t subindex,   // OD subindex of entry in PI
  uint8_t len_type,   // 1-4: write from uint8_t..uint32_t, 0: write from byte array
  uint16_t len_write, // if len_type==0 give write length, otherwise 0
  void* data          // pointer to write data (variable or byte array)
)
{
  uint8_t found;
  uint8_t val8;
  uint16_t val16;
  uint32_t val32 = 0;
  uint16_t pi_offs;
  uint16_t entry_len;
  uint8_t* p_dat;
  int pi_acc;

  if ((len_type > 4) ||
    ((len_type != 0) && (len_write != 0)))
    return 4; // parameter error

  // first check, if this is a system parameter
  if (len_type > 0)
  { // system parameters are 1 to 4 bytes only
    switch (len_type)
    { // get data
    case 1:
      val32 = *(uint8_t*)data & 0x000000FFul;
      break;
    case 2:
      val16 = *(uint16_t*)data & 0x0000FFFFul;
      break;
    default:
      val32 = *(uint32_t*)data;
      break;
    }
    if (MCO_ApplySystemEntry(index,subindex,val32) == 0xFFFFFFFFul)
    { // found and handled
      return 0;
    }
  }

  found = FALSE;

  if (0 == MCO_GetODProcPIOffsLenByIndex(index, subindex, &pi_offs, &val8))
  {
    found = TRUE;

    entry_len = (uint16_t)val8;
    p_dat = &gProcImg[pi_offs];
    pi_acc = PIACC_APP;
  }
#if USE_EXTENDED_SDO
  else
  {
    uint8_t access;

    val8 = XSDO_SearchODGenTable(index, subindex, &access, &val32, &p_dat);
    if (val8 != 0xFFu)
    {
      found = TRUE;
      entry_len = (uint16_t)val32;
#if USE_GENOD_PTR
      pi_acc = PIACC_NONE;
#else
      pi_acc = PIACC_APP;
#endif
    }
  }
#endif // USE_EXTENDED_SDO

  if (!found)
    return 1;

  if (len_type == 0)
  {
    if (len_write > entry_len)
      return 3; // data length exceeded maximum
  }
  else // len_type is 1..4
  {
    if (entry_len != len_type)
      return 2; // data length mismatch
  }

  switch (len_type)
  {
  case 0:
    if (pi_acc == PIACC_NONE)
    {
      MEM_CPY_FAR(p_dat, data, entry_len);
    }
    else
    {
      PI_WRITE(pi_acc, (p_dat - gProcImg), data, entry_len);
    }
    break;
  case 1:
    if (pi_acc == PIACC_NONE)
    {
      MEM_CPY_FAR(p_dat, data, 1);
    }
    else
    {
      PI_READ(pi_acc, (p_dat - gProcImg), data, 1);
    }
    break;
  case 2:
    val16 = *(uint16_t*)data;
    GEN_WR16(pi_acc, p_dat, val16);
    break;
  case 3:
    val32 = *(uint32_t*)data;
    GEN_WR24(pi_acc, p_dat, val32);
    break;
  case 4:
    val32 = *(uint32_t*)data;
    GEN_WR32(pi_acc, p_dat, val32);
    break;
  }

  return 0;
}


#if USE_EMCY
#if ERROR_FIELD_SIZE > 0
/**************************************************************************
DOES:    This function clears all entries of the error history [1003h] for
         CANopen or the active error list [1032h] for CANopen FD.
RETURNS: Nothing
**************************************************************************/
void MCOP_ErrField_Flush (void)
{ // Reset pointer and counter for error field
  gEF.InPtr = 0;
  gEF.NrOfRec = 0;
}


/**************************************************************************
DOES:    This function adds or updates an entry to/in the error history
         [1003h] for CANopen or the active error list [1032h] for
         CANopen FD.
RETURNS: Nothing
**************************************************************************/
void MCOP_ErrField_AddUpdate (
  uint32_t err_value // the 32bit error code used in the last EMCY
#if defined(USE_CANOPEN_FD) && USE_CANOPEN_FD
  ,
  uint8_t  status    // status esp. recoverable/non-recoverable
#endif // CANopen FD
  )
{ // add entry to field, if not already there
uint8_t found = 0xFFu;
uint8_t index;
  
  // first see if error is already in the list
  if (MCOP_ErrField_Find(err_value,&index) > 0)
  {
    found = index;
  }
  
  // if already in the list, make it the newest entry by removing and
  // re-adding it, in CANopen FD also mark it as having occurred several times.
  if (found < 0xFFu)
  {
    (void)MCOP_ErrField_Remove(err_value);
    found = 0xFFu;  // signal to add entry to the list as new
#if defined(USE_CANOPEN_FD) && USE_CANOPEN_FD    // CANopen FD
    status |= ERST_STATE_MOCC; // Error occurred multiple times
#endif
  }
  
  if (found == 0xFFu)  // otherwise, add it to the list
  {
    gEF.Field[gEF.InPtr] = err_value;
#if defined(USE_CANOPEN_FD) && USE_CANOPEN_FD
    gEF.status[gEF.InPtr]   = status;
#endif // CANopen FD
    // increment pointer and counter
    gEF.InPtr++;
    if (gEF.InPtr >= ERROR_FIELD_SIZE)
    { // roll over on end of buffer
      gEF.InPtr = 0;
    }
    gEF.NrOfRec++;
    if (gEF.NrOfRec >= ERROR_FIELD_SIZE)
    { // maximum is ERROR_FIELD_SIZE
      gEF.NrOfRec = ERROR_FIELD_SIZE;
    }
  }
}


/**************************************************************************
DOES:    This function removes an entry from the error history [1003h]
         for CANopen or the active error list [1032h] for CANopen FD.
RETURNS: The subindex value that was removed, 0 if there was no matching
         error found.
**************************************************************************/
uint8_t MCOP_ErrField_Remove (
  uint32_t err_value // the 32-bit error code used in the last EMCY
  )
{
uint8_t i;
uint8_t ret_val = 0;
  
  // if in the list, remove the entry
  if (MCOP_ErrField_Find(err_value,NULL) > 0)
  {
    uint32_t bufcopy[ERROR_FIELD_SIZE];
    uint32_t *p_bufcopy = bufcopy;
    uint8_t cnt = 0;
    
    // Remove old duplicate from the list by rebuilding
    // the buffer while skipping the item to remove.
    for (i=gEF.NrOfRec; i>0; i--)
    {
      uint32_t errval = MCOP_ErrField_Get(i,FALSE,NULL);
      if (errval == err_value)
      {
        ret_val = i;
      }
      else if (errval != 0xFFFFFFFFul)
      {
        *p_bufcopy++ = errval;
        cnt++;
      }
    }
    memcpy(gEF.Field,bufcopy,sizeof(gEF.Field));
    gEF.NrOfRec = cnt;
    gEF.InPtr = cnt;  // We already know the new list is one item shorter, so no roll-over here.
  }
  
  return(ret_val);
}


/**************************************************************************
DOES:    This function retrieves an entry from the error history [1003h]
         for CANopen or the active error list [1032h] for CANopen FD, based
         on the subindex
RETURNS: 32-bit error code for OD at subindex, or raw stored value
**************************************************************************/
uint32_t MCOP_ErrField_Get (
  uint8_t subindex,  // Subindex number of [1003h]/[1032h]
  uint8_t od_val,    // TRUE: Value for OD, FALSE: raw stored value
  uint8_t *ef_index  // if !=NULL, return internal index to entry
  )
{
uint32_t ret_value = 0xFFFFFFFF;
int16_t offset;

  if (subindex == 0)
  { // return number of entries
    ret_value = gEF.NrOfRec;
  }
  else
  { // return error value history (1 returns newest)
    if (subindex <= gEF.NrOfRec)
    { // only continue if subindex is in legal range
      offset = gEF.InPtr - subindex;
      if (offset < 0)
      {
        offset += ERROR_FIELD_SIZE;
      }
      ret_value = gEF.Field[offset];
#if defined(USE_CANOPEN_FD) && USE_CANOPEN_FD
      if (od_val)
      {
        ret_value &= 0x0000FFFFul;
        if (gEF.status[offset] & ERST_CLASS_NREC)
          ret_value |= 0x00010000ul;
        if (gEF.status[offset] & ERST_STATE_MOCC)
          ret_value |= 0x00020000ul;
      }
#endif // CANopen FD
      if (ef_index)
        *ef_index = (uint8_t)offset;
    }
  }
  return ret_value;
}


/**************************************************************************
DOES:    This function finds an entry in the error history [1003h]
         for CANopen or the active error list [1032h] for CANopen FD, based
         on the 32-bit error code used in the EMCY.
RETURNS: The subindex value for the entry or 0 if none found.
**************************************************************************/
uint8_t MCOP_ErrField_Find (
  uint32_t err_value, // the 32bit error code used in the last EMCY
  uint8_t *ef_index   // if !=NULL, return internal index to entry
  )
{
uint8_t i;
uint8_t ret_val = 0;
  
  // see if error is in the list
  for (i=1; i <= gEF.NrOfRec; i++)
  {
    uint32_t errval = MCOP_ErrField_Get(i,FALSE,ef_index);
    if (errval == err_value)
    {
      ret_val = i;
      break;
    }
  }

  return(ret_val);
}
#endif // ERROR_FIELD_SIZE > 0


/**************************************************************************
DOES:    Checks if all EMCY active bits are cleared.
         Used to determine if now an EMCY no error / reset can be sent.
RETURNS: TRUE - No more EMCY pending
         FALSE - Still some EMCY pending
**************************************************************************/
uint8_t MCOP_IsNoEMCYactive (void)
{
  uint8_t ret_val = TRUE;
  uint8_t lp;
  
  if (gEF.active_sys != 0)
  { // system EMCY pending
    ret_val = FALSE;
  }
#if NR_OF_RPDOS > 0
  if (ret_val)
  {
    lp = 0;
    while (lp < (sizeof(gEF.active_rpdo) / sizeof(gEF.active_rpdo[0])))
    {
      if (gEF.active_rpdo[lp] != 0)
      {
        ret_val = FALSE;
        break;
      }
      lp++;
    }
  }
#endif
#if NR_OF_HB_CONSUMER > 0
  if (ret_val)
  {
    lp = 0;
    while (lp < (sizeof(gEF.active_hbcons) / sizeof(gEF.active_hbcons[0])))
    {
      if (gEF.active_hbcons[lp] != 0)
      {
        ret_val = FALSE;
        break;
      }
      lp++;
    }
  }
#endif
  
  return ret_val;
}

/**************************************************************************
DOES:    Transmits an Emergency Message
RETURNS: TRUE - If msg was considered for transmit
         FALSE - If message was not sent due to duplicate
**************************************************************************/
uint8_t MCOP_PushEMCY (
  uint16_t emcy_code, // 16 bit error code
  uint8_t  em_1, // 5 byte manufacturer specific error code
  uint8_t  em_2,
  uint8_t  em_3,
  uint8_t  em_4,
  uint8_t  em_5
#if defined(USE_CANOPEN_FD) && USE_CANOPEN_FD    // CANopen FD
  ,
  uint8_t  dev_num,  // logical device number
  uint16_t spec_num, // CiA specification number
  uint8_t  status,   // status
  uint32_t time_lo,  // timestamp bits 0-31
  uint16_t time_hi   // timestamp bits 32-47
#endif // CANopen FD
  )
{
uint8_t ret_val;

  // Only send EMCYs when we have a valid node ID
  if (MY_NODE_ID == 0)
  {
    ret_val = FALSE;
  }
  else
  {
    if (emcy_code != 0)
    {
      gMCOConfig.error_register |= 1; // set generic error bit
    }

#if defined(USE_CANOPEN_FD) && USE_CANOPEN_FD    // CANopen FD
    gEF.emcy_msg.ID = 0x80 + MY_NODE_ID;
    gEF.emcy_msg.LEN = 20;
    gEF.emcy_msg.BUF[0]  = dev_num;
    gEF.emcy_msg.BUF[1]  = 0;
    GEN_WR16(PIACC_NONE, &gEF.emcy_msg.BUF[2],spec_num);
    GEN_WR16(PIACC_NONE, &gEF.emcy_msg.BUF[4],emcy_code);
    gEF.emcy_msg.BUF[6]  = gMCOConfig.error_register;
    gEF.emcy_msg.BUF[7]  = em_1;
    gEF.emcy_msg.BUF[8]  = em_2;
    gEF.emcy_msg.BUF[9]  = em_3;
    gEF.emcy_msg.BUF[10] = em_4;
    gEF.emcy_msg.BUF[11] = em_5;
    gEF.emcy_msg.BUF[12] = status;
    gEF.emcy_msg.BUF[13] = 0;
    GEN_WR32(PIACC_NONE, &gEF.emcy_msg.BUF[14],time_lo);
    GEN_WR16(PIACC_NONE, &gEF.emcy_msg.BUF[18],time_hi);
#else                                            // CANopen
    gEF.emcy_msg.ID = 0x80 + MY_NODE_ID;
    gEF.emcy_msg.LEN = 8;
    gEF.emcy_msg.BUF[0] = (uint8_t) emcy_code;
    gEF.emcy_msg.BUF[1] = (uint8_t) (emcy_code >> 8);
    gEF.emcy_msg.BUF[2] = gMCOConfig.error_register;
    gEF.emcy_msg.BUF[3] = em_1;
    gEF.emcy_msg.BUF[4] = em_2;
    gEF.emcy_msg.BUF[5] = em_3;
    gEF.emcy_msg.BUF[6] = em_4;
    gEF.emcy_msg.BUF[7] = em_5;
#endif

    ret_val = TRUE;
    
    // Note: Removed Error Field Handling from here, is called in stack "on occurance"

    }
  return ret_val;
}
#endif // USE_EMCY


#if !defined(USE_CANOPEN_FD) || (defined(USE_CANOPEN_FD) && (USE_CANOPEN_FD==0))
#if (NR_OF_RPDOS > 0) || (NR_OF_TPDOS > 0)
/**************************************************************************
DOES: Common exit routine for SDO_Handler.
      Send SDO response with write confirmation.
      Assumes that gTxSDO.ID, LEN and BUF[1-3] are already set
**************************************************************************/
void MCOP_WriteConfirm (
  void
  )
{
uint8_t i;

  // Load SDO Response into transmit buffer
  gTxSDO.BUF[0] = 0x60; // Write response code
  // Clear unused bytes
  for (i = 4; i < 8; i++)
  {
    gTxSDO.BUF[i] = 0;
  }
    
  // Transmit SDO Response message
  if (!MCOHW_PushMessage(&gTxSDO))
  {
    MCOUSER_FatalError(ERROFL_SDO);
  }
}


/**************************************************************************
DOES:    Handles incoming SDO Request for accesses to PDO Communication
         Parameters
RETURNS: 0: Wrong access, SDO Abort sent
         1: Access was made, SDO Response sent
GLOBALS: Various global variables with configuration information
**************************************************************************/
uint8_t SDO_HandlePDOComParam (
  uint8_t  PDOType,  // 0 for TPDO, 1 for RPDO
  uint16_t index,    // OD index
   uint8_t *pData    // pointer to SDO Request message
  )
{
uint16_t lp;
uint16_t PDONr;    // PDONr - 1
uint8_t cmd;       // SDO Request command byte
uint8_t len_req;   // length of SDO write access
COBID_TYPE rdat = 0; // current response data
uint8_t reply[4];  // SDO reply value
#if ((USE_EVENT_TIME || USE_INHIBIT_TIME) && (NR_OF_TPDOS > 0))
uint16_t temp16;
#endif

  cmd = pData[0];
  PDONr = (index & 0x1FF) + 1;
  len_req = 4-((cmd >> 2) & 0x03);

  // calculate real PDONr offset
  if (PDOType == 0)
  { // TPDO, find the PDONr in array
#if (NR_OF_TPDOS > 0)
    lp = 0;
    while (gTPDOConfig[lp].PDONr != PDONr)
    {
      lp++;
      if (lp >= gMCOConfig.nrTPDOs)
      { // not found!
        MCO_SendSDOAbort(SDO_ABORT_NOT_EXISTS);
        return 0;
      }
    }
    // PDO found, set PDONr to index
    PDONr = lp;
#endif
  }
  else
  { // RPDO, find the PDONr in array
#if (NR_OF_RPDOS > 0)
    lp = 0;
    while (gRPDOConfig[lp].PDONr != PDONr)
    {
      lp++;
      if (lp >= gMCOConfig.nrRPDOs)
      { // not found!
        MCO_SendSDOAbort(SDO_ABORT_NOT_EXISTS);
        return 0;
      }
    }
    // PDO found, set PDONr to index
    PDONr = lp;
#endif
  }

  if (pData[3] == 0) // subindex
  { // Nr Of Entries: Read-only, "2" for RPDO, "5" or "6" for TPDO
    if (cmd == 0x40)
    { // Read
      if (PDOType == 0)
      { // TPDO
#if USE_SYNC
        reply[0] = 6;
#else
        reply[0] = 5;
#endif
      }
      else
      { // RPDO
        reply[0] = 2;
      }
      MCO_ReplyWith(reply,1);
      return 1;
    }
    else
    { // Write
      MCO_SendSDOAbort(SDO_ABORT_READONLY);
      return 0;
    }
  }

  if (PDOType == 0)
  { // TPDO
#if NR_OF_TPDOS > 0
    rdat = gTPDOConfig[PDONr].CANmsg.ID;
#endif
  }
  else
  { // RPDO
#if NR_OF_RPDOS > 0
    rdat = gRPDOConfig[PDONr].CANID;
#endif
  }

  if (pData[3] == 1) // subindex
  { // COB ID
    if (cmd == 0x40)
    { // Read
      // Load SDO Response into transmit buffer
      if ((rdat & COBID_DISABLED) == 0)
      {
        gTxSDO.BUF[7] = 0x00;
      }
      else
      {
        gTxSDO.BUF[7] = 0x80; // PDO Enable/Disable bit;
      }
      if (PDOType == 0)
      { // TPDO
        gTxSDO.BUF[7] |= 0x40; // No RTR
      }
      gTxSDO.BUF[6] = 0;
      gTxSDO.BUF[5] = (uint8_t) ((rdat) >> 8) & 0x07; // Bits 8-10 of CAN ID
      gTxSDO.BUF[4] = (uint8_t) rdat; // Bits 0-7 of CAN ID
      gTxSDO.BUF[0] = 0x43; // Expedited, 4 bytes
      if (!MCOHW_PushMessage(&gTxSDO))
      {
        MCOUSER_FatalError(ERROFL_SDO);
      }
      return 1;
    }
    else
    { // Write
#if USE_STATIC_PDO == 1
      MCO_SendSDOAbort(SDO_ABORT_READONLY);
      return 0;
#else
      if (len_req != 4)
      { // length of access is not 4 bytes
        MCO_SendSDOAbort(SDO_ABORT_TYPEMISMATCH);
        return 0;
      }
      // for conformance test also check illegal value zero
      if (IS_CANID_RESTRICTED((((uint16_t)pData[5]) << 8) + pData[4]))
      { // zero not allowed
        MCO_SendSDOAbort(SDO_ABORT_VALUE_RANGE);
        return 0;
      }
      if (PDOType == 0)
      { // TPDO
#if NR_OF_TPDOS > 0
        /* in latest cct do not check RTR
        if ((pData[7] & 0xC0) == 0)
        { // RTR not supported
          MCO_SendSDOAbort(SDO_ABORT_VALUE_RANGE);
          return 0;
        }
        */
        if (((gTPDOConfig[PDONr].CANmsg.ID & COBID_DISABLED) != 0) ||
            ((gTPDOConfig[PDONr].CANmsg.ID & 0x07FF) == ((((uint16_t)(pData[5])) << 8)+ pData[4])) ||
            ((pData[7] & 0x80) != 0)
           )
        { // Only allowed if PDO is disabled, the same, or this access disables it
          // set new CAN ID
          gTPDOConfig[PDONr].CANmsg.ID = pData[4] | (((uint16_t)pData[5]) << 8);
          if ((pData[7] & 0x80) != 0)
          {
            gTPDOConfig[PDONr].CANmsg.ID |= COBID_DISABLED;
          }
          // Reset all possible TPDO trigger
#if USE_EVENT_TIME
          // This assignment split into two lines to work around certain limited C compilers
          temp16 = MCOHW_GetTime();
          gTPDOConfig[PDONr].event_timestamp = temp16;
#endif
#if USE_INHIBIT_TIME
          gTPDOConfig[PDONr].inhibit_status = INHITIM_EXPIRED;
#endif
#if USE_SYNC
          gTPDOConfig[PDONr].SYNCcnt = gTPDOConfig[PDONr].TType;
#endif
          // write completed
          MCOP_WriteConfirm();
          return 1;
        }
#endif // NR_OF_TPDOS
      }
      else
      { // RPDO
#if NR_OF_RPDOS > 0
        if (((gRPDOConfig[PDONr].CANID & COBID_DISABLED) != 0) ||
            ((gRPDOConfig[PDONr].CANID & 0x07FF) == ((((uint16_t)(pData[5])) << 8)+ pData[4])) ||
            ((pData[7] & 0x80) != 0)
           )
        { // Only allowed if PDO is disabled, or this access disables it
          // remove CAN ID filter
          MCOHW_ClearCANFilter(gRPDOConfig[PDONr].CANID);
          // Signal that RPDO filters are NOT set
          gMCOConfig.error_code &= ~0x80;
          // set new CAN ID
          gRPDOConfig[PDONr].CANID = pData[4] | (((uint16_t)pData[5]) << 8);
          if ((pData[7] & 0x80) != 0)
          {
            gRPDOConfig[PDONr].CANID |= COBID_DISABLED;
          }
          else
          {
            // set new filter
            MCOHW_SetCANFilter(gRPDOConfig[PDONr].CANID);
          }
          // write completed
          MCOP_WriteConfirm();
          return 1;
        }
#endif // NR_OF_RPDOS
      }
      MCO_SendSDOAbort(SDO_ABORT_UNSUPPORTED);
      return 0;
#endif // USE_STATIC_PDO
    } // write
  } // subindex 1

  // Now subindex is > 1, only allow writes if PDO is disabled
  if (cmd == 0x23)
  { // It is a write command
#if USE_STATIC_PDO == 1
     MCO_SendSDOAbort(SDO_ABORT_READONLY);
     return 0;
#else
    if (!(rdat & COBID_DISABLED))
    { // PDO is not disabled
      MCO_SendSDOAbort(SDO_ABORT_UNSUPPORTED);
      return 0;
    }
#endif // USE_STATIC_PDO
  }

  // Now handle remaining subindexes
  if (pData[3] == 2) // subindex
  { // Transmission Type
    if (cmd == 0x40)
    { // Read
      if (PDOType == 0)
      { // TPDO
#if NR_OF_TPDOS > 0
        if (gTPDOConfig[PDONr].TType == 241)
        {
          reply[0] = 0;
        }
        else
        {
         reply[0] = gTPDOConfig[PDONr].TType;
        }
#endif
      }
      else
      { // RPDO
#if NR_OF_RPDOS > 0
        reply[0] = gRPDOConfig[PDONr].TType;
#endif
      }
      MCO_ReplyWith(reply,1);
      return 1;
    }

    // Write
#if USE_STATIC_PDO == 1
     MCO_SendSDOAbort(SDO_ABORT_READONLY);
     return 0;
#else

    if (len_req != 1)
    {
      MCO_SendSDOAbort(SDO_ABORT_TYPEMISMATCH);
      return 0;
    }
#if ! USE_SYNC
    if (pData[4] <= 252)
    { // SYNC not supported
      MCO_SendSDOAbort(SDO_ABORT_VALUE_RANGE);
      return 0;
    }
#endif // USE_SYNC

    // RTR is not supported
    if (pData[4] == 253)
    { // RTR not supported
      MCO_SendSDOAbort(SDO_ABORT_VALUE_RANGE);
      return 0;
    }

    // This code version does not support the combination of SYNC with RTR
    if (pData[4] == 252)
    { // SYNC-RTR combination not supported
      MCO_SendSDOAbort(SDO_ABORT_VALUE_RANGE);
      return 0;
    }

    if (PDOType == 0)
    { // TPDO
#if NR_OF_TPDOS > 0
      gTPDOConfig[PDONr].TType = pData[4];
#if USE_SYNC
      gTPDOConfig[PDONr].SYNCcnt = gTPDOConfig[PDONr].TType;
#endif // USE_SYNC
#endif // NR_OF_TPDOS
    }
    else
    { // RPDO
#if NR_OF_RPDOS > 0
      gRPDOConfig[PDONr].TType = pData[4];
#endif
    }
    MCOP_WriteConfirm();
    return 1;
#endif // USE_STATIC_PDO
  }

  if (PDOType != 0)
  { // RPDO
    // No subindex > 2 supported for RPDO
    MCO_SendSDOAbort(SDO_ABORT_UNKNOWNSUB);
    return 0;
  }

#if NR_OF_TPDOS > 0
  if (pData[3] == 3) // subindex
  { // Inhibit Time
    if (cmd == 0x40)
    { // Read
#if USE_INHIBIT_TIME
      rdat = gTPDOConfig[PDONr].inhibit_time * 10;
#else
      rdat = 0;
#endif
      GEN_WR16(PIACC_NONE, &(reply[0]),rdat);
      MCO_ReplyWith(reply,2);
      return 1;
    }
    // Write

#if USE_STATIC_PDO == 1
     MCO_SendSDOAbort(SDO_ABORT_READONLY);
     return 0;
#else

    if (len_req != 2)
    {
      MCO_SendSDOAbort(SDO_ABORT_TYPEMISMATCH);
      return 0;
    }
#if USE_INHIBIT_TIME
    // Set new inhibit time
    // This assignment split into two lines to work around certain limited C compilers
    temp16 = (GEN_RD16(PIACC_NONE, &(pData[4])) + 9)/10;
    gTPDOConfig[PDONr].inhibit_time = temp16;

    // Reset inhibit status
    gTPDOConfig[PDONr].inhibit_status = INHITIM_EXPIRED;
    MCOP_WriteConfirm();
    return 1;
#else
    MCO_SendSDOAbort(SDO_ABORT_TYPEMISMATCH);
    return 0;
#endif

#endif // USE_STATIC_PDO

  }

#if USE_EVENT_TIME
  if (pData[3] == 5) // subindex
  { // Event Time
    if (cmd == 0x40)
    { // Read
      rdat = gTPDOConfig[PDONr].event_time;
      GEN_WR16(PIACC_NONE, &(reply[0]),rdat);
      MCO_ReplyWith(reply,2);
      return 1;
    }
    // Write
#if USE_STATIC_PDO == 1
     MCO_SendSDOAbort(SDO_ABORT_READONLY);
     return 0;
#else

    if (len_req != 2)
    {
      MCO_SendSDOAbort(SDO_ABORT_TYPEMISMATCH);
      return 0;
    }
    // This assignment split into two lines to work around certain limited C compilers
    temp16 = GEN_RD16(PIACC_NONE, &(pData[4]));
    gTPDOConfig[PDONr].event_time = temp16;
    if (gTPDOConfig[PDONr].event_time > 0x7FFF)
    {
      gTPDOConfig[PDONr].event_time = 0x7FFF;
    }
    // This assignment split into two lines to work around certain limited C compilers
    temp16 = MCOHW_GetTime();
    gTPDOConfig[PDONr].event_timestamp = temp16;
    MCOP_WriteConfirm();
    return 1;
#endif // USE_STATIC_PDO
  }
#endif

#if USE_SYNC
  if (pData[3] == 6) // subindex
  { // SYNC start value
    if (cmd == 0x40)
    { // Read
      MCO_ReplyWith(&(gTPDOConfig[PDONr].SYNCmatch),1);
      return 1;
    }
    // Write
#if USE_STATIC_PDO == 1
     MCO_SendSDOAbort(SDO_ABORT_READONLY);
     return 0;
#else

    if (len_req != 1)
    {
      MCO_SendSDOAbort(SDO_ABORT_TYPEMISMATCH);
      return 0;
    }
    gTPDOConfig[PDONr].SYNCmatch = pData[4];
    MCOP_WriteConfirm();
    return 1;
#endif // USE_STATIC_PDO
  }
#endif

#endif // NR_OF_TPDOS > 0

  MCO_SendSDOAbort(SDO_ABORT_UNKNOWNSUB);
  return 0;
}
#endif // (NR_OF_RPDOS > 0) || (NR_OF_TPDOS > 0)
#endif // !USE_CANOPEN_FD


#if USE_SYNC
/**************************************************************************
DOES:    Processes reception of the SYNC message
RETURNS: 0: No messages processed
         Bit 0 set: SYNC TPDOs transmitted
         Bit 1 set: SYNC RPDOs received
**************************************************************************/
uint8_t MCOP_HandleSYNC (
  uint8_t len, // length of SYNC (0 or 1)
  uint8_t scnt  // counter value, if len = 1
  )
{
uint16_t PDONr;
uint8_t retstat;
uint8_t trigger; // TPDO trigger;
#if (NR_OF_RPDOS > 0)
#if USECB_ODDATARECEIVED
uint16_t map; // offset into SDOResponseTable, RPDO mapping
uint16_t off; // offset into Process Image to RPDO data
MEM_CONST uint8_t *pSDO; // Pointer into SDOResponseTable
uint8_t cnt;
#endif // USECB_ODDATARECEIVED
#endif
  
#if (NR_OF_RPDOS > 0) || (NR_OF_TPDOS > 0)
  if (MY_NMT_STATE != NMTSTATE_OP)
  { // node is not in operational state
    return 0;
  }
  retstat = 0;
  
  if ( ( (gMCOConfig.SYNC_cntovr == 0) && (len != 0) ) ||
       ( (gMCOConfig.SYNC_cntovr != 0) && (len == 0) )
     )
  { // length of SYNC received does not match local configuration
#if USE_EMCY
#if ERROR_FIELD_SIZE > 0
    MCOP_ErrField_AddUpdate(MAKE_ERRCODE32(EMCY_SYNC_LEN,gMCOConfig.SYNC_cntovr,len)
#if defined(USE_CANOPEN_FD) && USE_CANOPEN_FD    // CANopen FD
      ,
      ERST_STATE_OCC
#endif
    );
#endif

    if ((gEF.active_sys & EMCYSBIT_SYNCLEN) == 0)
    { // this was not yet recorded
      gEF.active_sys |= EMCYSBIT_SYNCLEN;
#if defined(USECB_EMCY) && USECB_EMCY
      if (0 == MCOUSER_EMCY(FALSE, EMCY_SYNC_LEN, gMCOConfig.SYNC_cntovr, len, scnt, 0, 0
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
        (void)MCOP_PushEMCY(EMCY_SYNC_LEN, gMCOConfig.SYNC_cntovr, len, scnt, 0, 0
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
#endif // USE_EMCY
  }
  else
  { // sync len is as expected
#if USE_EMCY
    if ((gEF.active_sys & EMCYSBIT_SYNCLEN) != 0)
    { // previous error active
      gEF.active_sys &= ~EMCYSBIT_SYNCLEN; // clear active error bit
#if ERROR_FIELD_SIZE > 0
      MCOP_ErrField_Remove(MAKE_ERRCODE32(EMCY_SYNC_LEN,gMCOConfig.SYNC_cntovr,len));
#endif
#if defined(USECB_EMCY) && USECB_EMCY
      if (0 == MCOUSER_EMCY(TRUE, EMCY_SYNC_LEN, gMCOConfig.SYNC_cntovr, len, scnt, 0, 0
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
        { // no further EMCY is pending
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
#endif // USE_EMCY

#if USECB_SYNCRECEIVE
    if (len == 1)
    {
      MCOUSER_SYNCCNTReceived(scnt);
    }
    else
    {
      MCOUSER_SYNCReceived();
    }
#endif

#if (NR_OF_TPDOS > 0)
  for (PDONr = 0; PDONr < gMCOConfig.nrTPDOs; PDONr++)
  {
    if ((gTPDOConfig[PDONr].CANmsg.ID & COBID_DISABLED) == 0)
    { // this TPDO is used, 241 marks special case, first call since switch to operational of type 0
        if ((gTPDOConfig[PDONr].TType == 0) || (gTPDOConfig[PDONr].TType == 241))
      { // Combination COS and SYNC
        // has application data changed?
        if ((PDO_TXCOMP(PDONr,&(gTPDOConfig[PDONr].CANmsg.BUF[0])) != 0) || (gTPDOConfig[PDONr].TType == 241))
        { // ensure type is back to zero, 241 only used on first call
          gTPDOConfig[PDONr].TType = 0;
#if USECB_TPDORDY
          if (MCOUSER_TPDOReady(gTPDOConfig[PDONr].PDONr,2))
          {
#endif
            // Copy application data
            PDO_TXCOPY(PDONr,&(gTPDOConfig[PDONr].CANmsg.BUF[0]));
            // transmit now
            MCO_TransmitPDO(PDONr);
            retstat |= 1;
#if USECB_TPDORDY
          }
#endif
        }
      }
      if ((gTPDOConfig[PDONr].TType >= 1) &&
          (gTPDOConfig[PDONr].TType <= 240)
         )
      { // This PDO is synced
        trigger = FALSE;
        if (gMCOConfig.SYNC_cntovr == 0)
        { // no SYNC counter used
          gTPDOConfig[PDONr].SYNCcnt--;
          if (gTPDOConfig[PDONr].SYNCcnt == 0)
            { // SYNC counter reached zero, transmit PDO
              trigger = TRUE;
            }
          }
          else
          { // SYNC with counter
            if ( (scnt >= gTPDOConfig[PDONr].SYNCmatch) &&
                 ( ((scnt - gTPDOConfig[PDONr].SYNCmatch) % gTPDOConfig[PDONr].TType) == 0 ) )
            {
              trigger = TRUE;
            }
          }
          
          if (trigger)
          {
#if USECB_TPDORDY
          if (MCOUSER_TPDOReady(gTPDOConfig[PDONr].PDONr,1))
          {
#endif
            // Copy application data
            PDO_TXCOPY(PDONr,&(gTPDOConfig[PDONr].CANmsg.BUF[0]));
            // transmit now
            MCO_TransmitPDO(PDONr);
            retstat |= 1;
#if USECB_TPDORDY
          }
#endif
        }
      }
    }
  }
#endif // NR_OF_TPDOS

#if (NR_OF_RPDOS > 0)
  for (PDONr = 0; PDONr < gMCOConfig.nrRPDOs; PDONr++)
  {
    if ((gRPDOConfig[PDONr].CANID & COBID_DISABLED) == 0)
    { // this RPDO is used
      if (gRPDOConfig[PDONr].TType <= 240)
      { // This RPDO is synced
#if USECB_ODDATARECEIVED
          // Process RPDO mapping
#if USE_DYNAMIC_PDO_MAPPING
#error USECB_ODDATARECEIVED currently not available with USE_DYNAMIC_PDO_MAPPING
#else
          map = gRPDOConfig[PDONr].map; // offset to mapping entries
          off = gRPDOConfig[PDONr].offset; // offset to data in process image
          pSDO = OD_SDOResponseTablePtr(0);
          cnt = 1;
          while((pSDO[map+3] == cnt) && (cnt <= 8))
          { // while Subindex is not zero
            RTOS_LOCK_PI(PIACC_APP,PISECT_PDO);
            MCOUSER_ODData(0,(((uint16_t)(pSDO[map+7]))<<8)+pSDO[map+6],pSDO[map+5],&(gRPDOConfig[PDONr].BUF[off]),pSDO[map+4]>>3);
            RTOS_UNLOCK_PI(PIACC_APP,PISECT_PDO);
            off += (pSDO[map+4]>>3); // next mapped OD entry
            map += 8; // next mapping entry
            cnt++;
          }
#endif // USE_DYNAMIC_PDO_MAPPING
#endif // USECB_ODDATARECEIVED

          // copy data from RPDO to process image
          PDO_RXCOPY(PDONr,&(gRPDOConfig[PDONr].BUF[0]));

#if USECB_RPDORECEIVE
          RTOS_LOCK_PI(PIACC_APP, PISECT_PDO);
          MCOUSER_RPDOReceived(gRPDOConfig[PDONr].PDONr,gRPDOConfig[PDONr].offset,gRPDOConfig[PDONr].len);
          RTOS_UNLOCK_PI(PIACC_APP, PISECT_PDO);
#endif // USECB_RPDORECEIVE
        retstat |= 2;
      }
    }
  }
#endif // NR_OF_RPDOS
  } // SYNC cnt config 

  return retstat;
#else   // (NR_OF_RPDOS > 0) || (NR_OF_TPDOS > 0)
  return 0;
#endif  // (NR_OF_RPDOS > 0) || (NR_OF_TPDOS > 0)
}
#endif // USE_SYNC


#if USE_NODE_GUARDING
/**************************************************************************
DOES:    Checks if message received is guarding request
RETURNS: 0: message received is not guarding request
         1: guarding request received, response sent
**************************************************************************/
uint8_t MCOP_HandleGuarding (
  uint16_t can_id
  )
{
  if (can_id == (uint16_t) 0x0700 + MY_NODE_ID)
  { // ID matches, so probably a request
    // transmit response / heartbeat message
    // Merge toggle bit into response
    MY_NMT_STATE += gMCOConfig.NGtoggle;
    if (!MCOHW_PushMessage(&gMCOConfig.heartbeat_msg))
    {
      MCOUSER_FatalError(ERROFL_HBT);
    }
    // Remove toggle bit again
    MY_NMT_STATE &= 0x7F;
    if (gMCOConfig.NGtoggle == 0)
    {
      gMCOConfig.NGtoggle = 0x80;
    }
    else
    {
      gMCOConfig.NGtoggle = 0;
    }
    return 1;
  }
  return 0;
}
#endif


#if ! MGR_MONITOR_ALL_NODES
#if (NR_OF_HB_CONSUMER > 0)
/**************************************************************************
DOES:    Checks if a node ID is already used, needed for conformance test
RETURNS: TRUE, if node_id is already used
**************************************************************************/
uint8_t MCOP_IsHBMonitored (
  uint8_t channel,
  uint8_t node_id
  )
{
uint8_t loop;
uint8_t retval = FALSE;

  if (!(gHBCons[--channel].can_id == 0x700 + (uint16_t) node_id))
  { // the current channel already uses this node ID, so OK to modify

    loop = NR_OF_HB_CONSUMER;
    while (loop > 0)
    {
      loop--;
      if ((uint16_t) node_id + 0x700 == gHBCons[loop].can_id)
      {
        retval = TRUE;
        break;
      }
    }
  }
  return retval;
}


/**************************************************************************
DOES:    Initializes Heartbeat Consumer
GLOBALS: Inits gHBCons[consumer_channel-1]
**************************************************************************/
void MCOP_InitHBConsumer (
  uint8_t consumer_channel, // HB Consumer channel
  uint8_t node_id, // Node ID to monitor, 0 to disable monitor
  uint16_t hb_time // Timeout to use (in ms)
  )
{
#if CHECK_PARAMETERS
  // check ranges
  if ((consumer_channel == 0) || (consumer_channel > NR_OF_HB_CONSUMER))
  {
    MCOUSER_FatalError(ERRFT_HBIP);
  }
#endif

  consumer_channel--; // adapt to range 0 to NR_OF_HB_CONSUMER-1

  if ((node_id == 0) || (hb_time == 0))
  { // disable
    gHBCons[consumer_channel].status = HBCONS_OFF;
    gHBCons[consumer_channel].time = 0;
    gHBCons[consumer_channel].can_id = 0;
  }
  else
  { // enable
    if (hb_time >= 0x8000)
    { // maximum time supported by MCOP
      hb_time = 0x7FFF;
    }
    // (re)set CAN rx filter
    if ((gHBCons[consumer_channel].can_id & 0x7F) != node_id)
    { // new or different filter, apply receive filter
      if (!MCOHW_SetCANFilter(0x700 + node_id))
      {
        MCOUSER_FatalError(ERRFT_RXFLTN);
      }
    }
    gHBCons[consumer_channel].time = hb_time;
    gHBCons[consumer_channel].can_id = 0x700 + node_id;
    gHBCons[consumer_channel].status = HBCONS_INIT;
  }

}

/**************************************************************************
DOES:    Checks if a message received contains a heartbeat to be consumed
GLOBALS: Updates gHBCons[consumer_channel-1]
RETURNS: one, if message received was a heartbeat monitored, else zero
**************************************************************************/
uint8_t MCOP_ConsumeHB (
  CAN_MSG *pRxCAN // CAN message received
  )
{
uint8_t loop;
uint8_t retval = FALSE;

  for (loop = 0; loop < NR_OF_HB_CONSUMER; loop++)
  {
    if ( (gHBCons[loop].status != HBCONS_OFF) && // consumer is not disabled
         (pRxCAN->ID == gHBCons[loop].can_id) // CAN ID matches
       )
    { // Match found
      /* conformance requires that boot up msg is also considered
      if (pRxCAN->BUF[0] != 0)
      { // This is not the bootup message
      */
      gHBCons[loop].status = HBCONS_ACTIVE; // activate consumption
#if USE_EMCY
      if (ARRAY16_GETBIT(gEF.active_hbcons,loop))
      { // this node was previously reported lost
        ARRAY16_CLRBIT(gEF.active_hbcons,loop); // clear active bit
#if ERROR_FIELD_SIZE > 0
        MCOP_ErrField_Remove(MAKE_ERRCODE32(EMCY_HB_ERR,(uint8_t) gHBCons[loop].can_id & 0x07F,0));
#endif // ERROR_FIELD_SIZE
#if defined(USECB_EMCY) && USECB_EMCY
        if (0 == MCOUSER_EMCY(TRUE, EMCY_HB_ERR, (uint8_t) gHBCons[loop].can_id & 0x07F, 0, 0, 0, 0
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
          { // no further EMCY is pending
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
#endif // USE_EMCY
      // calculate expiration timestamp
      gHBCons[loop].timestamp = MCOHW_GetTime() + gHBCons[loop].time;
#if USE_LEDS
        gMCOConfig.LEDErr = LED_OFF; // clear previous error indication
#endif
      /* } */
      retval = TRUE;
      break;
    }
  }
  return retval;
}


/**************************************************************************
DOES:    Checks if a heartbeat consumer timeout occured
RETURNS:
          HBCONS_OFF,    // Disabled
          HBCONS_INIT,   // Initialized, waiting for first 2 heartbeats
          HBCONS_ACTIVE, // Consumer active and OK
          HBCONS_LOST    // Heartbeat lost
**************************************************************************/
HBCONS_STATE MCOP_ProcessHBCheck (
  uint8_t consumer_channel // Range 1 to NR_OF_HB_CONSUMER
  )
{

#if CHECK_PARAMETERS
  // check ranges
  if ((consumer_channel == 0) || (consumer_channel > NR_OF_HB_CONSUMER))
  {
    MCOUSER_FatalError(ERRFT_HBCFLT);
  }
#endif

  consumer_channel--; // adapt to range 0 to NR_OF_HB_CONSUMER-1

  if (gHBCons[consumer_channel].status == HBCONS_ACTIVE)
  { // Heartbeat consumer is active
    if (MCOHW_IsTimeExpired(gHBCons[consumer_channel].timestamp))
    { // active and expired
#if USE_EMCY
      // Set active bit for this channel
      ARRAY16_SETBIT(gEF.active_hbcons,consumer_channel);
#if ERROR_FIELD_SIZE > 0
      MCOP_ErrField_AddUpdate(MAKE_ERRCODE32(EMCY_HB_ERR,(uint8_t) gHBCons[consumer_channel].can_id & 0x07F,0)
#if defined(USE_CANOPEN_FD) && USE_CANOPEN_FD    // CANopen FD
       ,
       ERST_STATE_OCC
#endif
      );
#endif
#if defined(USECB_EMCY) && USECB_EMCY
      if (0 == MCOUSER_EMCY(FALSE, EMCY_HB_ERR, (uint8_t) gHBCons[consumer_channel].can_id & 0x07F, 0, 0, 0, 0
#if defined(USE_CANOPEN_FD) && USE_CANOPEN_FD    // CANopen FD
          ,
          0,     // dev_num - logical device number
          COFD_BASE_SPEC >> 20, // spec_num, CiA specification number
          ERST_STATE_OCC | ERST_PRIO(4),   // status - priority=4, recoverable, error occurred
        0,     // time_lo - timestamp bits 0-31, not supported
        0      // time_hi - timestamp bits 32-47, not supported
#endif // CANopen FD
      ))
#endif // USECB_EMCY
      {
        (void)MCOP_PushEMCY(EMCY_HB_ERR, (uint8_t)gHBCons[consumer_channel].can_id, 0, 0, 0, 0
#if defined(USE_CANOPEN_FD) && USE_CANOPEN_FD    // CANopen FD
          ,
          0,     // dev_num - logical device number
          COFD_BASE_SPEC >> 20, // spec_num, CiA specification number
          ERST_STATE_OCC | ERST_PRIO(4),   // status - priority=4, recoverable, error occurred
          0,     // time_lo - timestamp bits 0-31, not supported
          0      // time_hi - timestamp bits 32-47, not supported
#endif // CANopen FD
        );
      }
#endif // USE_EMCY
#if USE_LEDS
      gMCOConfig.LEDErr = LED_FLASH2;
#endif
      MCOUSER_HeartbeatLost((uint8_t)gHBCons[consumer_channel].can_id);
      // set new state
      MY_NMT_STATE = NMTSTATE_PREOP;
#if USECB_NMTCHANGE
      // Call back to user / application
      MCOUSER_NMTChange(MY_NMT_STATE);
#endif
      // restart HB consumer
      gHBCons[consumer_channel].status = HBCONS_INIT;
    }
  }
  return gHBCons[consumer_channel].status;
}
#endif // (NR_OF_HB_CONSUMER > 0)
#endif // MGR_MONITOR_ALL_NODES



#if USE_SLEEP
/**************************************************************************
Description in mco.h
***************************************************************************/
uint8_t MCOP_TransmitWakeupSleep (
  uint8_t statcmd,
  uint8_t reason
  )
{
CAN_MSG TxMSG;

  if ((MY_NODE_ID > 0) && (MY_NODE_ID <= 16))
  { // Do we have a node ID?
    TxMSG.ID = 0x690+MY_NODE_ID;
    TxMSG.BUF[0] = statcmd;
    TxMSG.BUF[1] = reason;
  }
  else
  { // use generic ID and wakeup only
    TxMSG.ID = 0x690;
    TxMSG.BUF[0] = SLEEP_WAKEUP;
    TxMSG.BUF[1] = SLEEP_REASON_NONE;
  }
  TxMSG.LEN = 8;
  TxMSG.BUF[2] = 0;
  TxMSG.BUF[3] = 0;
  TxMSG.BUF[4] = 0;
  TxMSG.BUF[5] = 0;
  TxMSG.BUF[6] = 0;
  TxMSG.BUF[7] = 0;
  return MCOHW_PushMessage(&TxMSG);
}

#endif // USE_SLEEP

/*----------------------- END OF FILE ----------------------------------*/
