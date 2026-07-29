/**************************************************************************
MODULE:    STORPARA
CONTAINS:  MicroCANopen Plus implementation, Store Parameter Function
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
#if (USE_XOD_ACCESS == 1)
#include "xod.h"
#endif

#ifdef __SIMULATION__
// header files to create dll
#include <windows.h>
#include "mcohwpcsim.h"
#include "simnodehandler.h"
#endif

#if USE_STORE_PARAMETERS

#if ! MGR_MONITOR_ALL_NODES
#if (NR_OF_HB_CONSUMER > 0)
extern HBCONS_CONFIG MEM_FAR gHBCons[NR_OF_HB_CONSUMER];
#endif // (NR_OF_HB_CONSUMER > 0)
#endif // MGR_MONITOR_ALL_NODES

#if USE_EMCY
extern EMCY_CONFIG MEM_FAR gEF;
#endif

// Index definitions
#define STORE_PARAMETERS    0x1010
#define RESTORE_PARAMETERS  0x1011

enum {
    PARAMETERS_ALL = 1,
    PARAMETERS_COMMUNICATION,
    PARAMETERS_APPLICATION,
    PARAMETERS_MANUFACTURER
} store_parameters_config;


/**************************************************************************
PRIVATE FUNCTIONS
***************************************************************************/

/**************************************************************************
DOES:    Counts how many bytes of data are in the process image in the
         specified index range with OD entry type ODWR.
RETURNS: Number of bytes required for storage.
**************************************************************************/
static uint16_t MCOSP_ODCount (
  uint16_t start,
  uint16_t end
  )
{
uint16_t sum; // for counting bytes
uint16_t index = 0;
OD_PROCESS_DATA_ENTRY MEM_CONST *pOD; // pointer into OD array

  sum = 0;
  pOD = OD_ProcTablePtr(0);
  while (index != 0xFFFF)
  {
    index = pOD->idx_hi;
    index <<= 8;
    index |= pOD->idx_lo;
    if ((index >= start) && (index <= end))
    { // entry is in right index range
      if (pOD->len & ODWR)
      { // entry can be written to
        // add to total
        sum += (pOD->len & 0x07);
      }
    }
    pOD++;
  }
  if (sum > 0)
  { // add space for checksum and ID
    sum += 4;
  }
  return sum;
}


/**************************************************************************
DOES:    Checks or retrieves all communication parameters from NVOL
RETURNS: TRUE, if data is present in NVOL
         FALSE, if no data is present in NVOL
**************************************************************************/
static uint8_t MCOSP_GetStoredComParameters (
  uint16_t start, // start offset in NVOL memory
  uint8_t restore // TRUE: restore data, FALSE: only perform chksum test
  )
{
// Note: first uint16_t used for ID "1X", second for checksum
uint16_t offset;
uint16_t loop;
uint16_t chk_sum;
uint8_t MEM_FAR *pDest;
#if (USE_XOD_ACCESS == 1)
uint16_t tmp;
#endif
  
  if ( (NVOL_ReadByte(start) != '2') ||
       (NVOL_ReadByte(start+1) != 'X')
     )
  { // ID does not match, no parameters stored
    return FALSE;
  }

  // Init checksum
  chk_sum = 0;
  offset = start + 4;

#if (USE_XOD_ACCESS == 1)
  // If supported, get XOD binary EDS checksum
  tmp = NVOL_ReadByte(offset+1);
  tmp <<= 8;
  tmp += NVOL_ReadByte(offset);
  // verify the checksum, only continue if it matches
  if (tmp != gOD.chksum)
  { // saved parameters are from another configuration
    return FALSE;
  }
  // continue, add this to checksum
  chk_sum += NVOL_ReadByte(offset);
  offset++;
  chk_sum += NVOL_ReadByte(offset);
  offset++;
#endif

  // HB time
  chk_sum += NVOL_ReadByte(offset);
  offset++;
  chk_sum += NVOL_ReadByte(offset);
  offset++;

#if NR_OF_TPDOS > 0
  // TPDO configuration data
  for (loop = 0; loop < (sizeof(TPDO_CONFIG)*NR_OF_TPDOS); loop++)
  {
    chk_sum += NVOL_ReadByte(offset);
    offset++;
  }
#if USE_DYNAMIC_PDO_MAPPING
  for (loop = 0; loop < sizeof(gTPDOmap); loop++)
  { // gTPDOmap
    chk_sum += NVOL_ReadByte(offset);
    offset++;
  }
  for (loop = 0; loop < sizeof(gTPDODataPtr); loop++)
  { // gTPDODataPtr
    chk_sum += NVOL_ReadByte(offset);
    offset++;
  }
#endif
#endif

#if NR_OF_RPDOS > 0
  // RPDO configuration data
  for (loop = 0; loop < (sizeof(RPDO_CONFIG)*NR_OF_RPDOS); loop++)
  {
    chk_sum += NVOL_ReadByte(offset);
    offset++;
  }
#if USE_DYNAMIC_PDO_MAPPING
  for (loop = 0; loop < sizeof(gRPDOmap); loop++)
  { // gRPDOmap
    chk_sum += NVOL_ReadByte(offset);
    offset++;
  }
  for (loop = 0; loop < sizeof(gRPDODataPtr); loop++)
  { // gRPDODataPtr
    chk_sum += NVOL_ReadByte(offset);
    offset++;
  }
#endif
#endif

#if ! MGR_MONITOR_ALL_NODES
#if (NR_OF_HB_CONSUMER > 0)
  // HB consumer data
  for (loop = 0; loop < (sizeof(HBCONS_CONFIG)*NR_OF_HB_CONSUMER); loop++)
  {
    chk_sum += NVOL_ReadByte(offset);
    offset++;
  }
#endif
#endif // MGR_MONITOR_ALL_NODES

#if USE_SYNC
  chk_sum += NVOL_ReadByte(offset);
  offset++;
  chk_sum += NVOL_ReadByte(offset);
  offset++;
#if CAN_ID_SIZE == 32
  chk_sum += NVOL_ReadByte(offset);
  offset++;
  chk_sum += NVOL_ReadByte(offset);
  offset++;
#endif
  chk_sum += NVOL_ReadByte(offset);
  offset++;
#if USE_SYNC_PRODUCER
  chk_sum += NVOL_ReadByte(offset);
  offset++;
  chk_sum += NVOL_ReadByte(offset);
  offset++;
  chk_sum += NVOL_ReadByte(offset);
  offset++;
  chk_sum += NVOL_ReadByte(offset);
  offset++;
#endif
#endif // USE_SYNC

#if USE_EMCY
  chk_sum += NVOL_ReadByte(offset);
  offset++;
  chk_sum += NVOL_ReadByte(offset);
  offset++;
#endif // USE_EMCY

  if ( (NVOL_ReadByte(start+2) != (chk_sum & 0x00FF)) ||
       (NVOL_ReadByte(start+3) != ((chk_sum>>8) & 0x00FF))
     )
  {
    // Checksum does not match, no parameters stored
    return FALSE;
  }

  if (! restore)
  { // No restore requested, verify only
    return TRUE;
  }

  // ID matches and checksum matches -> retrieve data
  offset = start + 4; // skip ID and chksum

#if (USE_XOD_ACCESS == 1)
  offset += 2;
#endif

  // Get Heartbeat time
  gMCOConfig.heartbeat_time = NVOL_ReadByte(offset+1);
  gMCOConfig.heartbeat_time <<= 8;
  gMCOConfig.heartbeat_time += NVOL_ReadByte(offset);
  offset += 2;

#if NR_OF_TPDOS > 0
  // Get TPDO configuration data
  pDest = (uint8_t MEM_FAR *) &(gTPDOConfig[0]);
  for (loop = 0; loop < (sizeof(TPDO_CONFIG)*NR_OF_TPDOS); loop++)
  { // Get all TPDO Config Parameter from NVOL
    *pDest = NVOL_ReadByte(offset);
    offset++;
    pDest++;
  }
#if USE_DYNAMIC_PDO_MAPPING
  pDest = (uint8_t MEM_FAR *) &(gTPDOmap[0]);
  for (loop = 0; loop < sizeof(gTPDOmap); loop++)
  { // gTPDOmap
    *pDest = NVOL_ReadByte(offset);
    offset++;
    pDest++;
  }
  pDest = (uint8_t MEM_FAR *) &(gTPDODataPtr[0]);
  for (loop = 0; loop < sizeof(gTPDODataPtr); loop++)
  { // gTPDODataPtr
    *pDest = NVOL_ReadByte(offset);
    offset++;
    pDest++;
  }
#endif
  for (loop = 0; loop < NR_OF_TPDOS; loop++)
  { // restore default COB-IDs, keeping the option bits
    if ((gTPDOConfig[loop].CANmsg.ID & ~COBID_OPT_MASK) == COBID_PDO_DEFAULT)
    {
      gTPDOConfig[loop].CANmsg.ID &= COBID_OPT_MASK;
      gTPDOConfig[loop].CANmsg.ID |= (0x180u+(0x100u*loop)+MY_NODE_ID);
    }
  }
#endif

#if NR_OF_RPDOS > 0
  // Save RPDO configuration data
  pDest = (uint8_t MEM_FAR *) &(gRPDOConfig[0]);
  for (loop = 0; loop < (sizeof(RPDO_CONFIG)*NR_OF_RPDOS); loop++)
  { // Get all RPDO Config Parameter from NVOL
    *pDest = NVOL_ReadByte(offset);
    offset++;
    pDest++;
  }
#if USE_DYNAMIC_PDO_MAPPING
  pDest = (uint8_t MEM_FAR *) &(gRPDOmap[0]);
  for (loop = 0; loop < sizeof(gRPDOmap); loop++)
  { // gRPDOmap
    *pDest = NVOL_ReadByte(offset);
    offset++;
    pDest++;
  }
  pDest = (uint8_t MEM_FAR *) &(gRPDODataPtr[0]);
  for (loop = 0; loop < sizeof(gRPDODataPtr); loop++)
  { // gRPDODataPtr
    *pDest = NVOL_ReadByte(offset);
    offset++;
    pDest++;
  }
#endif
  for (loop = 0; loop < NR_OF_RPDOS; loop++)
  { // restore default COB-IDs, keeping the option bits
    if ((gRPDOConfig[loop].CANID & ~COBID_OPT_MASK) == COBID_PDO_DEFAULT)
    {
      gRPDOConfig[loop].CANID &= COBID_OPT_MASK;
      gRPDOConfig[loop].CANID |= (0x200u+(0x100u*loop)+MY_NODE_ID);
    }
  }
#endif

#if ! MGR_MONITOR_ALL_NODES
#if (NR_OF_HB_CONSUMER > 0)
  // Get HB consumer data
  pDest = (uint8_t MEM_FAR *) &(gHBCons[0]);
  for (loop = 0; loop < (sizeof(HBCONS_CONFIG)*NR_OF_HB_CONSUMER); loop++)
  { // Write all HB Config Parameter to NVOL
    *pDest = NVOL_ReadByte(offset);
    offset++;
    pDest++;
  }
#endif
#endif // MGR_MONITOR_ALL_NODES

#if USE_SYNC
  // get CAN ID of SYNC object
  gMCOConfig.SYNC_id = NVOL_ReadByte(offset);
  offset++;
  gMCOConfig.SYNC_id += (((uint16_t) NVOL_ReadByte(offset)) << 8);
  offset++;
#if CAN_ID_SIZE == 32
  gMCOConfig.SYNC_id += (((uint32_t) NVOL_ReadByte(offset)) << 16);
  offset++;
  gMCOConfig.SYNC_id += (((uint32_t) NVOL_ReadByte(offset)) << 24);
  offset++;
#endif
  gMCOConfig.SYNC_cntovr = NVOL_ReadByte(offset);
  offset++;
#if USE_SYNC_PRODUCER
  gMCOConfig.SYNC_cycle = NVOL_ReadByte(offset);
  offset++;
  gMCOConfig.SYNC_cycle += (((uint32_t) NVOL_ReadByte(offset)) << 8);
  offset++;
  gMCOConfig.SYNC_cycle += (((uint32_t) NVOL_ReadByte(offset)) << 16);
  offset++;
  gMCOConfig.SYNC_cycle += (((uint32_t) NVOL_ReadByte(offset)) << 24);
  offset++;
#endif
#endif // USE_SYNC

#if USE_EMCY
  gEF.emcy_inhibit = NVOL_ReadByte(offset);
  offset++;
  gEF.emcy_inhibit += (((uint16_t) NVOL_ReadByte(offset)) << 8);
  offset++;
#endif // USE_EMCY

  return TRUE;
}


/**************************************************************************
DOES:    Stores all communication parameters to NVOL
RETURNS: TRUE, if parameters were stored
         FALSE, if a write/verify occured
**************************************************************************/
static uint8_t MCOSP_StoreComParameters (
  uint16_t start // destination address in NVOL memory
  )
{
// Note: first uint16_t used for ID "1X", second for checksum
uint16_t offset;
uint16_t loop;
uint16_t chk_sum;
uint8_t MEM_FAR *pSrc;

  chk_sum = 0;
  offset = start;
  // erase ID
  NVOL_WriteByte(offset,0xFF);
  NVOL_WriteByte(offset+1,0xFF);
  offset += 4; // skip ID and chk_sum

#if (USE_XOD_ACCESS == 1)
  // If supported, save XOD binary EDS checksum
  NVOL_WriteByte(offset,gOD.chksum & 0x00FF);
  chk_sum += gOD.chksum & 0x00FF;
  offset++;
  NVOL_WriteByte(offset,(gOD.chksum>>8) & 0x00FF);
  chk_sum += (gOD.chksum>>8) & 0x00FF;
  offset++;
#endif
  
  // Save Heartbeat time
  NVOL_WriteByte(offset,gMCOConfig.heartbeat_time & 0x00FF);
  chk_sum += gMCOConfig.heartbeat_time & 0x00FF;
  offset++;
  NVOL_WriteByte(offset,(gMCOConfig.heartbeat_time>>8) & 0x00FF);
  chk_sum += (gMCOConfig.heartbeat_time>>8) & 0x00FF;
  offset++;

#if NR_OF_TPDOS > 0
  // Save TPDO configuration data
  // Check COB-IDs, if they are default, save as default, but leave the option bits intact
  for (loop = 0; loop < NR_OF_TPDOS; loop++)
  {
    if ((gTPDOConfig[loop].CANmsg.ID & ~COBID_OPT_MASK) == (0x180u+(0x100u*loop)+MY_NODE_ID))
    {
      gTPDOConfig[loop].CANmsg.ID = (gTPDOConfig[loop].CANmsg.ID & COBID_OPT_MASK) | COBID_PDO_DEFAULT;
    }
  }
  pSrc = (uint8_t MEM_FAR *) &(gTPDOConfig[0]);
  for (loop = 0; loop < (sizeof(TPDO_CONFIG)*NR_OF_TPDOS); loop++)
  { // Write all TPDO Config Parameter to NVOL
    NVOL_WriteByte(offset,*pSrc);
    chk_sum += *pSrc;
    offset++;
    pSrc++;
  }
#if USE_DYNAMIC_PDO_MAPPING
  pSrc = (uint8_t MEM_FAR *) &(gTPDOmap[0]);
  for (loop = 0; loop < sizeof(gTPDOmap); loop++)
  { // Write gTPDOmap
    NVOL_WriteByte(offset,*pSrc);
    chk_sum += *pSrc;
    offset++;
    pSrc++;
  }
  pSrc = (uint8_t MEM_FAR *) &(gTPDODataPtr[0]);
  for (loop = 0; loop < sizeof(gTPDODataPtr); loop++)
  { // Write gTPDODataPtr
    NVOL_WriteByte(offset,*pSrc);
    chk_sum += *pSrc;
    offset++;
    pSrc++;
  }
#endif
  for (loop = 0; loop < NR_OF_TPDOS; loop++)
  { // restore default COB-IDs, keeping the option bits
    if ((gTPDOConfig[loop].CANmsg.ID & ~COBID_OPT_MASK) == COBID_PDO_DEFAULT)
    {
      gTPDOConfig[loop].CANmsg.ID &= COBID_OPT_MASK;
      gTPDOConfig[loop].CANmsg.ID |= (0x180u+(0x100u*loop)+MY_NODE_ID);
    }
  }
#endif

#if NR_OF_RPDOS > 0
  // Check COB-IDs, if they are default, save as default, but leave the option bits intact
  for (loop = 0; loop < NR_OF_RPDOS; loop++)
  {
    if ((gRPDOConfig[loop].CANID & ~COBID_OPT_MASK) == (0x200u+(0x100u*loop)+MY_NODE_ID))
    {
      gRPDOConfig[loop].CANID = (gRPDOConfig[loop].CANID & COBID_OPT_MASK) | COBID_PDO_DEFAULT;
    }
  }
  // Save RPDO configuration data
  pSrc = (uint8_t MEM_FAR *) &(gRPDOConfig[0]);
  for (loop = 0; loop < (sizeof(RPDO_CONFIG)*NR_OF_RPDOS); loop++)
  { // Write all RPDO Config Parameter to NVOL
    NVOL_WriteByte(offset,*pSrc);
    chk_sum += *pSrc;
    offset++;
    pSrc++;
  }
#if USE_DYNAMIC_PDO_MAPPING
  pSrc = (uint8_t MEM_FAR *) &(gRPDOmap[0]);
  for (loop = 0; loop < sizeof(gRPDOmap); loop++)
  { // Write gRPDOmap
    NVOL_WriteByte(offset,*pSrc);
    chk_sum += *pSrc;
    offset++;
    pSrc++;
  }
  pSrc = (uint8_t MEM_FAR *) &(gRPDODataPtr[0]);
  for (loop = 0; loop < sizeof(gRPDODataPtr); loop++)
  { // Write gRPDODataPtr
    NVOL_WriteByte(offset,*pSrc);
    chk_sum += *pSrc;
    offset++;
    pSrc++;
  }
#endif
  for (loop = 0; loop < NR_OF_RPDOS; loop++)
  { // restore default COB-IDs, keeping the option bits
    if ((gRPDOConfig[loop].CANID & COBID_OPT_MASK) == COBID_PDO_DEFAULT)
    {
      gRPDOConfig[loop].CANID &= COBID_OPT_MASK;
      gRPDOConfig[loop].CANID |= (0x200u+(0x100u*loop)+MY_NODE_ID);
    }
  }
#endif

#if ! MGR_MONITOR_ALL_NODES
#if (NR_OF_HB_CONSUMER > 0)
  // Save HB consumer data
  pSrc = (uint8_t MEM_FAR *) &(gHBCons[0]);
  for (loop = 0; loop < (sizeof(HBCONS_CONFIG)*NR_OF_HB_CONSUMER); loop++)
  { // Write all HB Config Parameter to NVOL
    NVOL_WriteByte(offset,*pSrc);
    chk_sum += *pSrc;
    offset++;
    pSrc++;
  }
#endif
#endif // MGR_MONITOR_ALL_NODES

#if USE_SYNC
  NVOL_WriteByte(offset,gMCOConfig.SYNC_id & 0xFF);
  chk_sum += gMCOConfig.SYNC_id & 0xFF;
  offset++;
  NVOL_WriteByte(offset,(gMCOConfig.SYNC_id>>8) & 0xFF);
  chk_sum += (gMCOConfig.SYNC_id>>8) & 0xFF;
  offset++;
#if CAN_ID_SIZE == 32
  NVOL_WriteByte(offset,(gMCOConfig.SYNC_id>>16) & 0xFF);
  chk_sum += (gMCOConfig.SYNC_id>>16) & 0xFF;
  offset++;
  NVOL_WriteByte(offset,(gMCOConfig.SYNC_id>>24) & 0xFF);
  chk_sum += (gMCOConfig.SYNC_id>>24) & 0xFF;
  offset++;
#endif
  NVOL_WriteByte(offset,gMCOConfig.SYNC_cntovr);
  chk_sum += gMCOConfig.SYNC_cntovr;
  offset++;
#if USE_SYNC_PRODUCER
  NVOL_WriteByte(offset,gMCOConfig.SYNC_cycle & 0xFF);
  chk_sum += gMCOConfig.SYNC_cycle & 0xFF;
  offset++;
  NVOL_WriteByte(offset,(gMCOConfig.SYNC_cycle>>8) & 0xFF);
  chk_sum += (gMCOConfig.SYNC_cycle>>8) & 0xFF;
  offset++;
  NVOL_WriteByte(offset,(gMCOConfig.SYNC_cycle>>16) & 0xFF);
  chk_sum += (gMCOConfig.SYNC_cycle>>16) & 0xFF;
  offset++;
  NVOL_WriteByte(offset,(gMCOConfig.SYNC_cycle>>24) & 0xFF);
  chk_sum += (gMCOConfig.SYNC_cycle>>24) & 0xFF;
  offset++;
#endif
#endif // USE_SYNC

#if USE_EMCY
  NVOL_WriteByte(offset,gEF.emcy_inhibit & 0x00FF);
  chk_sum += gEF.emcy_inhibit & 0x00FF;
  offset++;
  NVOL_WriteByte(offset,(gEF.emcy_inhibit>>8) & 0x00FF);
  chk_sum += (gEF.emcy_inhibit>>8) & 0x00FF;
  offset++;
#endif // USE_EMCY

  // No further consecutive writes here
  NVOL_WriteComplete();

  // Now write chksum + ID
  NVOL_WriteByte(start,'2');
  NVOL_WriteByte(start+1,'X');
  NVOL_WriteByte(start+2,chk_sum & 0x00FF);
  NVOL_WriteByte(start+3,(chk_sum>>8) & 0x00FF);

  // No further consecutive writes after this
  NVOL_WriteComplete();

  // Now verify if valid data was written
  return MCOSP_GetStoredComParameters(start,FALSE);
}


/**************************************************************************
DOES:    Retrieves OD data from NVOL to process image
RETURNS: TRUE, if data is present in NVOL
         FALSE, if no data is present in NVOL
**************************************************************************/
static uint8_t MCOSP_GetStoredODParameters (
  uint16_t start, // destination address in NVOL memory
  uint16_t idx_start, // index range to store
  uint16_t idx_end, // end of index range to store
  uint8_t restore // TRUE: restore data, FALSE: only perform chksum test
  )
{
// Note: first uint16_t used for ID "OD", second for checksum
uint16_t offset;
uint16_t loop;
uint16_t chk_sum;
uint16_t index = 0;
uint16_t len;
uint16_t pioff; // process image offset
uint8_t MEM_PROC *pDest;
OD_PROCESS_DATA_ENTRY MEM_CONST *pOD; // pointer into OD array
#if USE_EXTENDED_SDO
OD_GENERIC_DATA_ENTRY MEM_CONST *pGOD; // pointer into generic OD array
int pi_acc;
#endif // USE_EXTENDED_SDO

  if ( (NVOL_ReadByte(start) != 'O') ||
       (NVOL_ReadByte(start+1) != 'D')
     )
  { // ID does not match, no parameters stored
    return FALSE;
  }

  // Verify checksum
  chk_sum = 0;
  offset = start + 4; // skip ID and chk_sum
  pOD = OD_ProcTablePtr(0);
  while (index != 0xFFFF)
  {
    index = pOD->idx_hi;
    index <<= 8;
    index += pOD->idx_lo;
    if ((index >= idx_start) && (index <= idx_end))
    { // entry is in right index range
      if (pOD->len & ODWR)
      { // entry can be written to
        for (loop = 0; loop < (pOD->len & 0x07); loop++)
        {
          chk_sum += NVOL_ReadByte(offset);
          offset++;
        }
      }
    }
    pOD++;
  }
#if USE_EXTENDED_SDO
  pGOD = OD_GenericTablePtr(0);
  index = 0;
  while (index != 0xFFFF)
  {
    index = pGOD->idx_hi;
    index <<= 8;
    index += pGOD->idx_lo;
    len = pGOD->len_hi;
    len <<= 8;
    len += pGOD->len_lo;
    if ((index >= idx_start) && (index <= idx_end))
    { // entry is in right index range
      if (pGOD->access & ODWR)
      { // entry can be written to
        for (loop = 0; loop < len; loop++)
        {
          chk_sum += NVOL_ReadByte(offset);
          offset++;
        }
      }
    }
    pGOD++;
  }
#endif // USE_EXTENDED_SDO
  // Checksum calculated, now verify
  if ( (NVOL_ReadByte(start+2) != (chk_sum & 0x00FF)) ||
       (NVOL_ReadByte(start+3) != ((chk_sum>>8) & 0x00FF))
     )
  { // Checksum does not match, no parameters stored
    return FALSE;
  }

  if (! restore)
  { // No restore requested, verify only
    return TRUE;
  }

  // ID matches and checksum matches -> retrieve data
  offset = start + 4; // skip ID and chksum
  pOD = OD_ProcTablePtr(0);
  index = 0;
  while (index != 0xFFFF)
  {
    index = pOD->idx_hi;
    index <<= 8;
    index += pOD->idx_lo;
    if ((index >= idx_start) && (index <= idx_end))
    { // entry is in right index range
      if (pOD->len & ODWR)
      { // entry can be written to
        pioff = pOD->off_hi;
        pioff <<= 8;
        pioff += pOD->off_lo;
        pDest = &(gProcImg[pioff]);
        for (loop = 0; loop < (pOD->len & 0x07); loop++)
        {
          GEN_WR8(PIACC_SDO, pDest, NVOL_ReadByte(offset));
          offset++;
          pDest++;
        }
      }
    }
    pOD++;
  }
#if USE_EXTENDED_SDO
  pGOD = OD_GenericTablePtr(0);
  index = 0;
  while (index != 0xFFFF)
  {
    index = pGOD->idx_hi;
    index <<= 8;
    index += pGOD->idx_lo;
    if ((index >= idx_start) && (index <= idx_end))
    { // entry is in right index range
      if (pGOD->access & ODWR)
      { // entry can be written to
#if USE_GENOD_PTR
        pDest = pGOD->pDat;
        pi_acc = PIACC_NONE;
#else
        pioff = pGOD->off_hi;
        pioff <<= 8;
        pioff += pGOD->off_lo;
        pDest = &(gProcImg[pioff]);
        pi_acc = PIACC_SDO;
#endif
        len = pGOD->len_hi;
        len <<= 8;
        len += pGOD->len_lo;
        for (loop = 0; loop < len; loop++)
        {
          GEN_WR8(pi_acc, pDest, NVOL_ReadByte(offset));
          offset++;
          pDest++;
        }
      }
    }
    pGOD++;
  }
#endif // USE_EXTENDED_SDO

  return TRUE;
}


/**************************************************************************
DOES:    Stores data from OD to NVOL
RETURNS: TRUE, if data is present in NVOL
         FALSE, if no data is present in NVOL
**************************************************************************/
static uint8_t MCOSP_StoreODParameters (
  uint16_t start, // destination address in NVOL memory
  uint16_t idx_start, // index range to store
  uint16_t idx_end // end of index range to store
  )
{
// Note: first uint16_t used for ID "OD", second for checksum
uint16_t offset;
uint16_t loop;
uint16_t chk_sum;
uint8_t MEM_PROC *pSrc;
OD_PROCESS_DATA_ENTRY MEM_CONST *pOD; // pointer into OD array
#if USE_EXTENDED_SDO
OD_GENERIC_DATA_ENTRY MEM_CONST *pGOD; // pointer into generic OD array
uint16_t index = 0;
uint16_t len;
uint16_t pioff; // process image offset
uint8_t byte;
int pi_acc;
#endif // USE_EXTENDED_SDO

  (void)pi_acc;

  chk_sum = 0;
  offset = start;
  // erase ID
  NVOL_WriteByte(offset,0xFF);
  NVOL_WriteByte(offset+1,0xFF);
  offset += 4; // skip ID and chk_sum

  pOD = OD_ProcTablePtr(0);
  while (index != 0xFFFF)
  {
    index = pOD->idx_hi;
    index <<= 8;
    index += pOD->idx_lo;
    if ((index >= idx_start) && (index <= idx_end))
    { // entry is in right index range
      if (pOD->len & ODWR)
      { // entry can be written to
        // store entry
        pioff = pOD->off_hi;
        pioff <<= 8;
        pioff += pOD->off_lo;
        pSrc = &(gProcImg[pioff]);
        for (loop = 0; loop < (pOD->len & 0x07); loop++)
        {
          byte = GEN_RD8(PIACC_SDO, pSrc);
          NVOL_WriteByte(offset,byte);
          chk_sum += byte;
          offset++;
          pSrc++;
        }
      }
    }
    pOD++;
  }
#if USE_EXTENDED_SDO
  pGOD = OD_GenericTablePtr(0);
  index = 0;
  while (index != 0xFFFF)
  {
    index = pGOD->idx_hi;
    index <<= 8;
    index += pGOD->idx_lo;
    if ((index >= idx_start) && (index <= idx_end))
    { // entry is in right index range
      if (pGOD->access & ODWR)
      { // entry can be written to
        // store entry
#if USE_GENOD_PTR
        pSrc = pGOD->pDat;
        pi_acc = PIACC_NONE;
#else
        pioff = pGOD->off_hi;
        pioff <<= 8;
        pioff += pGOD->off_lo;
        pSrc = &(gProcImg[pioff]);
        pi_acc = PIACC_SDO;
#endif
        len = pGOD->len_hi;
        len <<= 8;
        len += pGOD->len_lo;
        for (loop = 0; loop < len; loop++)
        {
          byte = GEN_RD8(PIACC_SDO, pSrc);
          NVOL_WriteByte(offset,byte);
          chk_sum += byte;
          offset++;
          pSrc++;
        }
      }
    }
    pGOD++;
  }
#endif // USE_EXTENDED_SDO
  // No further consecutive writes after this
  NVOL_WriteComplete();

  // Now write chksum + ID
  NVOL_WriteByte(start,'O');
  NVOL_WriteByte(start+1,'D');
  NVOL_WriteByte(start+2,chk_sum & 0x00FF);
  NVOL_WriteByte(start+3,(chk_sum>>8) & 0x00FF);

  // No further consecutive writes after this
  NVOL_WriteComplete();

  // Verify if valid data was stored
  return MCOSP_GetStoredODParameters(start,idx_start,idx_end,FALSE);
}


/**************************************************************************
PUBLIC FUNCTIONS
***************************************************************************/

/**************************************************************************
DOES:    Determines offset values for the the different regions
         in non-volatile memory.
         1. LSS + XOD binary eds checksum
         2. Save Parameters 1XXX
         3. Save Parameters 6XXX
         4. Save Parameters 2XXX
         5. First free space
RETURNS: Info on start offsets is copied into the string (5 values)
**************************************************************************/
void MCOSP_GetNVOLUsage (
  uint16_t pLoc[5] // info is written into this string
  )
{
uint16_t pos;

  // NVOL_Init() must have been called before reaching here

  pos = NVOL_STORE_START;
  *pLoc = pos; // start of LSS
  pLoc++; // pointer to next section: 1XXX
  pos += 4; // size of LSS record is 4 bytes
  // Now start of section 1XXXX
  *pLoc = pos;
  pLoc++; // pointer to next section: 2XXX
  pos += 4; // 2 byte ID, 2 byte checksum
#if (USE_XOD_ACCESS == 1)
  // If supported, save XOD binary EDS checksum
  pos += 2; // 2 byte XOD binary EDS checksum
#endif
  pos += 2; // 2 byte HB producer time
#if NR_OF_TPDOS > 0
  pos += (sizeof(TPDO_CONFIG) * NR_OF_TPDOS);
#if USE_DYNAMIC_PDO_MAPPING
  pos += (sizeof(gTPDOmap));
  pos += (sizeof(gTPDODataPtr));
#endif
#endif
#if NR_OF_RPDOS > 0
  pos += (sizeof(RPDO_CONFIG) * NR_OF_RPDOS);
#if USE_DYNAMIC_PDO_MAPPING
  pos += (sizeof(gRPDOmap));
  pos += (sizeof(gRPDODataPtr));
#endif
#endif
#if ! MGR_MONITOR_ALL_NODES
#if (NR_OF_HB_CONSUMER > 0)
  pos += (sizeof(HBCONS_CONFIG) * NR_OF_HB_CONSUMER);
#endif
#endif // MGR_MONITOR_ALL_NODES
#if USE_SYNC
  pos += 2;
#if CAN_ID_SIZE == 32
  pos += 2;
#endif
  pos += 1; // 1019
#if USE_SYNC_PRODUCER
  pos += 4; // 1006
#endif
#endif
#if USE_EMCY
  pos += 2;
#endif // USE_EMCY

  // Now reached end of section, next section 6XXX
  *pLoc = pos;
  pLoc++; // pointer to next section: 6XXX
  pos += MCOSP_ODCount(0x6000,0x9FFF);
  // Now reached end of section, next section 2XXX
  *pLoc = pos;
  pLoc++; // pointer to end
  pos += MCOSP_ODCount(0x2000,0x5FFF);
  // Now reached end of required NVOL
  *pLoc = pos;

  DEBUG_PRINT(MCO_LOG_INFO,("Store parameters size check: %d\n", pos));

  if (pos > (NVOL_STORE_START + NVOL_STORE_SIZE))
  { // NVOL storage size TOO small!
    MCOUSER_FatalError(ERRFT_NVMEM);
  }
}


/**************************************************************************
DOES:    Implements the Store Parameters and Restore Parameters
         functionality.
RETURNS: TRUE, if data was restored
         FALSE. if no valid data was found
**************************************************************************/
uint8_t MCOSP_StoreParameters (
  uint16_t idx, // set to 0x1010 for store, to 0x1011 for restore
  uint8_t sub // subindex
  )
{
uint16_t location[5]; // start offsets and sizes of areas in NVOL
uint8_t ret_value;

  // Get info about current NVOL usage
  MCOSP_GetNVOLUsage(&(location[0]));

  if (idx == STORE_PARAMETERS)
  { // Save Parameters
    DEBUG_PRINT(MCO_LOG_INFO,("Storing: "));
    ret_value = TRUE;
    if ((sub == PARAMETERS_ALL) || (sub == PARAMETERS_COMMUNICATION))
    { // Save Configuration Parameters 0x1XXX
      DEBUG_PRINT(MCO_LOG_INFO,("  Communication parameters\n"));
      ret_value &= MCOSP_StoreComParameters(location[1]);
      if (ret_value)
        gMCOConfig.saved_flags |= 0x02;
      else
        gMCOConfig.saved_flags &= ~0x02;
    }
    if ((sub == PARAMETERS_ALL) || (sub == PARAMETERS_APPLICATION))
    { // Save Application Parameters 0x6XXX
      DEBUG_PRINT(MCO_LOG_INFO,("  Profile parameters\n"));
      ret_value &= MCOSP_StoreODParameters(location[2],0x6000,0x9FFF);
      if (ret_value)
        gMCOConfig.saved_flags |= 0x04;
      else
        gMCOConfig.saved_flags &= ~0x04;
    }
    if ((sub == PARAMETERS_ALL) || (sub == PARAMETERS_MANUFACTURER))
    { // Save Manufacturer Parameters 0x2XXX
      DEBUG_PRINT(MCO_LOG_INFO,("  Manufacturer parameters\n"));
      ret_value &= MCOSP_StoreODParameters(location[3],0x2000,0x5FFF);
      if (ret_value)
        gMCOConfig.saved_flags |= 0x08;
      else
        gMCOConfig.saved_flags &= ~0x08;
    }
    // If all sections are saved, set the bit for "all saved"
    if (0x0E == (gMCOConfig.saved_flags & 0x0E))
      gMCOConfig.saved_flags |= 0x01;
    else
      gMCOConfig.saved_flags &= ~0x01;
    return ret_value;
  }
  if (idx == RESTORE_PARAMETERS)
  { // Restore Parameters
    DEBUG_PRINT(MCO_LOG_INFO,("Restoring: "));
    if ((sub == PARAMETERS_ALL) || (sub == PARAMETERS_COMMUNICATION))
    { // Restore Configuration Parameters 0x1XXX
      DEBUG_PRINT(MCO_LOG_INFO,("  Communication parameters\n"));
      // Destroy ID and Chksum
      NVOL_WriteByte(location[1],0xFF);
      NVOL_WriteByte(location[1]+2,NVOL_ReadByte(location[1]+2)+1);
      // No further consecutive writes after this
      NVOL_WriteComplete();
      gMCOConfig.saved_flags &= ~0x03;
    }
    if ((sub == PARAMETERS_ALL) || (sub == PARAMETERS_APPLICATION))
    { // Restore Application Parameters 0x6XXX
      DEBUG_PRINT(MCO_LOG_INFO,("  Profile parameters\n"));
      NVOL_WriteByte(location[2],0xFF);
      NVOL_WriteByte(location[2]+2,NVOL_ReadByte(location[2]+2)+1);
      // No further consecutive writes after this
      NVOL_WriteComplete();
      gMCOConfig.saved_flags &= ~0x05;
    }
    if ((sub == PARAMETERS_ALL) || (sub == PARAMETERS_MANUFACTURER))
    { // Restore Manufacturer Parameters 0x2XXX
      DEBUG_PRINT(MCO_LOG_INFO,("  Manufacturer parameters\n"));
      NVOL_WriteByte(location[3],0xFF);
      NVOL_WriteByte(location[3]+2,NVOL_ReadByte(location[3]+2)+1);
      // No further consecutive writes after this
      NVOL_WriteComplete();
      gMCOConfig.saved_flags &= ~0x09;
    }
  }
  return TRUE;
}


/**************************************************************************
DOES:    Checks the non-volatile memory for saved parameters and retrives
         them all
RETURNS: Nothing.
**************************************************************************/
void MCOSP_GetStoredParameters (
  void
  )
{
uint16_t location[5]; // start offsets and sizes of areas in NVOL
uint8_t result;

  DEBUG_PRINT(MCO_LOG_INFO,("Retrieving stored parameters: "));

  // Get info about current NVOL usage
  MCOSP_GetNVOLUsage(&(location[0]));

  // Try to retrieve communication parameters
  result = MCOSP_GetStoredComParameters(location[1],TRUE);
  if (result)
    gMCOConfig.saved_flags |= 0x02;
  else
    gMCOConfig.saved_flags &= ~0x02;
  DEBUG_PRINT(MCO_LOG_INFO,("  Communication parameters - "));
  if (result)
    DEBUG_PRINT(MCO_LOG_INFO,("OK\n"));
  else
    DEBUG_PRINT(MCO_LOG_INFO,("NONE\n"));

  // Try to retrieve device profile parameters
  result = MCOSP_GetStoredODParameters(location[2],0x6000,0x9FFF,TRUE);
  if (result)
    gMCOConfig.saved_flags |= 0x04;
  else
    gMCOConfig.saved_flags &= ~0x04;
  DEBUG_PRINT(MCO_LOG_INFO,("  Profile parameters - "));
  if (result)
    DEBUG_PRINT(MCO_LOG_INFO,("OK\n"));
  else
    DEBUG_PRINT(MCO_LOG_INFO,("NONE\n"));

  // Try to retrieve manufacturer parameters
  result = MCOSP_GetStoredODParameters(location[3],0x2000,0x5FFF,TRUE);
  if (result)
    gMCOConfig.saved_flags |= 0x08;
  else
    gMCOConfig.saved_flags &= ~0x08;
  DEBUG_PRINT(MCO_LOG_INFO,("  Manufacturer parameters - "));
  if (result)
    DEBUG_PRINT(MCO_LOG_INFO,("OK\n"));
  else
    DEBUG_PRINT(MCO_LOG_INFO,("NONE\n"));

  // If all sections are saved, set the bit for "all saved"
  if (0x0E == (gMCOConfig.saved_flags & 0x0E))
    gMCOConfig.saved_flags |= 0x01;
  else
    gMCOConfig.saved_flags &= ~0x01;

  (void)result; // Ignoring result
}
#endif // USE_STORE_PARAMETERS


/*----------------------- END OF FILE ----------------------------------*/
