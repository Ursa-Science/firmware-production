/**************************************************************************
MODULE:    XSDO
CONTAINS:  MicroCANopen Plus, Extended SDO implementation
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

#include <stddef.h>

#if !defined(USE_CANOPEN_FD) || (defined(USE_CANOPEN_FD) && (USE_CANOPEN_FD==0))

#if USE_EXTENDED_SDO

#if ((NR_OF_SDOSERVER == 0) || (NR_OF_SDOSERVER > 127))
#error Illegal value for NR_OF_SDOSERVER
#endif

/**************************************************************************
GLOBAL VARIABLES
***************************************************************************/ 

// Fatal Error return value
#define TXLOST_SDO 0x0770

// states for state machine
#define STAT_NONE    0
#define STAT_READ    1
#define STAT_WRITE   2
#define STAT_WRITECB 4

#define BLK_STAT_WRITE  8   // sdo block transfer in progress (download/write)
#define BLK_STAT_READ   16  // sdo block transfer in progress (upload/read)
#define BLK_STAT_WRCONF 32  // sdo block transfer completed
#define BLK_STAT_RDGO   64  // sdo block transfer in progress (upload/read)
#define BLK_STAT_RDCONF 128 // sdo block transfer completed

// SDO status info
static struct
{
  uint32_t size; // Current data length
  uint32_t bufsize; // Max data length of buffer
  uint32_t bufcnt; // Count in buffer
  uint16_t idx; // Index
  uint8_t *pBuf; // Buffer base pointer
  uint8_t *pDat; // Running data pointer
#if USE_BLOCKED_SDO
  uint16_t sgCANID; // CAN ID used in block read transfers
  uint16_t sgdelay; // Time delay for back to back messages
  uint8_t sgcount; // Segment counter
  uint8_t sgunused; // Unused bytes of segment
  uint8_t sgblks; // number of segments in a block
#endif
  uint8_t sub; // Subindex
  uint8_t state; // State machine
  uint8_t tog; // Toggle bit
  uint8_t calb; // TRUE if entry is not table based 
} MEM_FAR mXSDO[NR_OF_SDOSERVER];

#if USE_BLOCKED_SDO
// Current SDO server used, must be from 0 to NR_OF_SDOSERVER-1
static uint8_t MEM_FAR mSDObr = 0;
#endif


/**************************************************************************
LOCAL FUNCTIONS
***************************************************************************/

/**************************************************************************
DOES: Common exit routine for SDO_Handler. 
      Send SDO response with write confirmation.
      Assumes that ID, LEN and BUF[1-3] are already set
**************************************************************************/
static void XSDO_WriteConfirm (
  CAN_MSG *pTxCAN
  )
{
uint8_t i;

  // Load SDO Response into transmit buffer
  pTxCAN->BUF[0] = 0x60; // Write response code
  // Clear unused bytes
  for (i = 4; i < 8; i++)
  {
    pTxCAN->BUF[i] = 0;
  }
    
  // Transmit SDO Response message
  if (!MCOHW_PushMessage(pTxCAN))
  {
    MCOUSER_FatalError(TXLOST_SDO);
  }
}

/**************************************************************************
DOES: Common exit routine for SDO_Handler, segmented write
      Send SDO response with write confirmation for segmented transfer
**************************************************************************/
static void XSDO_WriteSegConfirm (
  CAN_MSG *pTxCAN,
  uint8_t sdoserv
  )
{
uint8_t i;

  // Load SDO Response into transmit buffer
  pTxCAN->BUF[0] = mXSDO[sdoserv].tog & 0x10; // Copy toggle bit
  pTxCAN->BUF[0] += 0x20;
  
  // Clear unused bytes
  for (i = 1; i < 8; i++)
  {
    pTxCAN->BUF[i] = 0;
  }
    
  // Transmit SDO Response message
  if (!MCOHW_PushMessage(pTxCAN))
  {
    MCOUSER_FatalError(TXLOST_SDO);
  }

  mXSDO[sdoserv].tog = ~mXSDO[sdoserv].tog; // Toggle the toggle bit
}


/*******************************************************************************
DOES:    Writes the next segment of segmented SDO transfer to the 
         destination buffer.
RETURNS: TRUE if no error occured during the segment
         FALSE if a major error occured and the transfer needs to be aborted
*******************************************************************************/
static uint8_t XSDO_WriteNextSegment (
  uint8_t last, // Set to 1 if this is the last segment
  uint8_t len, // length of segment (0-7)
  uint8_t *pDat, // pointer to 'len' data bytes
  uint8_t sdoserv // SDO server from 0 to NR_OF_SDOSERVERS-1
  )
{
#if USECB_APPSDO_WRITE
uint8_t res;
#endif
uint8_t ret_val = TRUE;

  // Only lock/unlock if PI offsets are used for long entries
#if !USE_GENOD_PTR
  // Only lock/unlock PI if this is not a user-handled long SDO transfer
  if (!mXSDO[sdoserv].calb)
  {
    RTOS_LOCK_PI(PIACC_SDO, PISECT_ALL);
  }
#endif

  while ((len > 0) && ret_val)
  { // Process byte-by-byte of segment
    if (mXSDO[sdoserv].size == 0)
    { // end of buffer reached
      ret_val = FALSE;
    }
    else
    {
      // Copy data
      *mXSDO[sdoserv].pDat = *pDat;
      // Increment pointers
      pDat++;
      mXSDO[sdoserv].pDat++;
      // Decrement local and overall length counter
      len--;
      mXSDO[sdoserv].size--;

#if USECB_APPSDO_WRITE
      if (mXSDO[sdoserv].pBuf != 0)
      { // Entry handled by application
        // Check for destination buffer overrun
        mXSDO[sdoserv].bufcnt++;
        if ((mXSDO[sdoserv].bufcnt >= mXSDO[sdoserv].bufsize) || (mXSDO[sdoserv].size == 0))
        { // reached end of destination buffer or end of transfer
          // Application call back for custom OD entries
          res = MCOUSER_AppSDOWriteComplete(sdoserv, mXSDO[sdoserv].idx, mXSDO[sdoserv].sub,
            mXSDO[sdoserv].pDat - mXSDO[sdoserv].pBuf, mXSDO[sdoserv].size);
          if (res > 1)
            ret_val = FALSE;
          else
          {
            // Restore buffer pointer
            mXSDO[sdoserv].pDat = mXSDO[sdoserv].pBuf;
            // Reset counter
            mXSDO[sdoserv].bufcnt = 0;
          }
        }
      }
#endif
    }
  }

  // Only lock/unlock if PI offsets are used for long entries
#if !USE_GENOD_PTR
  // Only lock/unlock PI if this is not a user-handled long SDO transfer
  if (!mXSDO[sdoserv].calb)
  {
    RTOS_UNLOCK_PI(PIACC_SDO, PISECT_ALL);
  }
#endif

  return ret_val;
}


/*******************************************************************************
DOES:    Reads the next segment of segmented SDO transfer from the
         source buffer.
*******************************************************************************/
static void XSDO_ReadNextSegment (
  uint8_t *pDat, // pointer to SDO data bytes
  uint8_t sdoserv // sdo server from 0 to NR_OF_SDOSERVERS-1
  )
{
uint8_t *p1st;
uint8_t len;

  // Init variables
  p1st = pDat; // remember pointer to first byte
  len = 7;
  pDat++; // start at byte 1 not zero

  // Only lock/unlock PI if this is not a user-handled long SDO transfer
  if (!mXSDO[sdoserv].calb)
  {
    RTOS_LOCK_PI(PIACC_SDO, PISECT_ALL);
  }

  while ((len > 0) && (mXSDO[sdoserv].size > 0))
  {
    // Copy data
    *pDat = *mXSDO[sdoserv].pDat;
    // Increment pointers
    pDat++;
    mXSDO[sdoserv].pDat++;
    // Decrement local and overall length counter

    len--;
    mXSDO[sdoserv].size--;
  }

#if USECB_APPSDO_READ
  if (mXSDO[sdoserv].size == 0)
  {
    // Application call back for custom OD entries
    MCOUSER_AppSDOReadComplete(sdoserv+1,mXSDO[sdoserv].idx,mXSDO[sdoserv].sub,&mXSDO[sdoserv].size);
    // Reset data buffer pointer
    if (mXSDO[sdoserv].size > 0)
    {
      mXSDO[sdoserv].pDat = mXSDO[sdoserv].pBuf;
    }
    while ((len > 0) && (mXSDO[sdoserv].size > 0))
    {
      // Copy data
      *pDat = *mXSDO[sdoserv].pDat;
      // Increment pointers
      pDat++;
      mXSDO[sdoserv].pDat++;
      // Decrement local and overall length counter

      len--;
      mXSDO[sdoserv].size--;
    }
  }
#endif

  // Only lock/unlock PI if this is not a user-handled long SDO transfer
  if (!mXSDO[sdoserv].calb)
  {
    RTOS_UNLOCK_PI(PIACC_SDO, PISECT_ALL);
  }

#if USE_BLOCKED_SDO
  if (mXSDO[sdoserv].state == BLK_STAT_RDGO)
  { // we are in blocked mode
    mXSDO[sdoserv].sgcount++;
    *p1st = mXSDO[sdoserv].sgcount; // next seq. number
    if (mXSDO[sdoserv].size == 0)
    { // end of all data reached
      *p1st |= 0x80; // set "last segment" bit
      mXSDO[sdoserv].state = BLK_STAT_RDCONF;
      // remember number of unused bytes in last msg
      mXSDO[sdoserv].sgunused = len;
    }
    else if (mXSDO[sdoserv].sgblks == mXSDO[sdoserv].sgcount)
    { // end of block is reached, wait for confirmation
      mXSDO[sdoserv].state = BLK_STAT_RDCONF;
    }
  }
  else
#endif
  {
    // Now calculate contents of 1st byte
    *p1st = ((mXSDO[sdoserv].tog & 1) << 4) + (len << 1);
    if (mXSDO[sdoserv].size == 0)
    { // end of all segmented data reached
      *p1st |= 0x01; // set "last segment" bit
      mXSDO[sdoserv].state = STAT_NONE; // transfer completed
    }
  }
}


/**************************************************************************
DOES:    Process segmented SDO Requests to generic OD entries
RETURNS: 0x01 all OK
         0x02 toggle error
         0x03 received data too big
         0x04 do a SDO_ABORT_READONLY
         0x05 do a SDO_ABORT_WRITEONLY
         0x06 do a SDO_ABORT_NOT_EXISTS
         0x07 do a SDO_ABORT_DATATOBIG
         0x20 pTxData contains SDO response to send
         0xF0 command specifier error
**************************************************************************/
static uint8_t XSDO_HandleSegmented (
  uint16_t index, // Current index (if known)
  uint8_t *pRxData, // SDO Request
  uint8_t *pTxData, // SDO Response
  uint8_t SDOsrv // SDO server number from 0 to NR_OF_SDOSERVERS-1
  )
{
uint8_t access; // Access type of OD entry
uint8_t ret_val = 0; 
uint32_t len;
#if USECB_APPSDO_READ || USECB_APPSDO_WRITE
uint32_t MEM_FAR totalsize = 0;
uint8_t MEM_FAR dummytype = 0;
#endif // USECB_APPSDO_READ || USECB_APPSDO_WRITE

#if USE_BLOCKED_SDO
uint8_t segsize;
uint8_t last;
uint32_t blksize;

  // Check if SDO Block Download (Write) Transfer in progress 
  if (mXSDO[SDOsrv].state == BLK_STAT_WRITE)
  { 
    if ((*pRxData & 0x7F) != mXSDO[SDOsrv].sgcount) 
    { // Not the correct sequence number
      if ((*pRxData & 0x7F) > mXSDO[SDOsrv].sgcount)
      { // Lost a message, abort with illegal sequence number
        mXSDO[SDOsrv].state = STAT_NONE;
        return 8;
      }
      // Possible duplicate: do not process, do not increment count
      return 15; // just ignore message
    }
    if ( (mXSDO[SDOsrv].sgcount >= BLK_MAX_SIZE) || 
         ((*pRxData & 0x80) == 0x80)
       )
    { // End of block reached
      if (mXSDO[SDOsrv].size < 7)
      {// this is the very last piece of data, get remaining size
        segsize = mXSDO[SDOsrv].size; 
        last = TRUE;
      }
      else
      {
        segsize = 7;
        last = FALSE; 
      }
      if ( XSDO_WriteNextSegment(last, segsize, &(pRxData[1]), SDOsrv) != TRUE )
      {
        // received data too big!
        return 0xF0; // Abort transfer
      }
      // Calculate remaining data size
      blksize = (mXSDO[SDOsrv].size+6) / 7;
      if (blksize > BLK_MAX_SIZE)
      {
        blksize = BLK_MAX_SIZE;
      }
      if (blksize == 0)
      { // this was the last block, now wait for confirmation
        mXSDO[SDOsrv].state = BLK_STAT_WRCONF;
      }
      // Send confirmation for block
      pTxData[0] = (5 << 5) + 2; // Server Command specifier
      pTxData[1] = mXSDO[SDOsrv].sgcount; // last received segment
      pTxData[2] = blksize; // remaining blocks
      pTxData[3] = 0;
      pTxData[4] = 0;
      pTxData[5] = 0;
      pTxData[6] = 0;
      pTxData[7] = 0;
      mXSDO[SDOsrv].sgcount = 1; // prepare for next block
      return 0x20; // Send SDO response
    }
    else
    { // not yet the last segment
      if ( XSDO_WriteNextSegment(FALSE, 7, &(pRxData[1]), SDOsrv) != TRUE )
      {
        // received data too big!
        return 0xF0; // Abort transfer
      }
    }

    // Prepare for next segment in block
    mXSDO[SDOsrv].sgcount++;
    return 15; // all OK
  }

  // Check if SDO Block Upload (Read) Transfer to start
  if (mXSDO[SDOsrv].state == BLK_STAT_READ)
  { 
    // Check if command specifier is right
    if ((pRxData[0] & 0xE3) != 0xA3)
    {
      mXSDO[SDOsrv].state = STAT_NONE;
      return 0xF0; // Report general error
    }
    mXSDO[SDOsrv].state = BLK_STAT_RDGO;
    mXSDO[SDOsrv].sgdelay = MCOHW_GetTime();
    mXSDO[SDOsrv].sgcount = 0; 
    return 15; // all OK, no response
  }

  // Check if SDO Block Upload (Read) Transfer completed
  if (mXSDO[SDOsrv].state == BLK_STAT_RDCONF)
  {
    if ((*pRxData & 0xE3) == 0xA2) 
    { // confirmation for block received
#if USECB_APPSDO_READ
      if (mXSDO[SDOsrv].size == 0)
      { // this is not the last block
        // Application call back for custom OD entries
        MCOUSER_AppSDOReadComplete(SDOsrv+1,mXSDO[SDOsrv].idx,mXSDO[SDOsrv].sub,&mXSDO[SDOsrv].size);
        // Reset data buffer pointer
        if (mXSDO[SDOsrv].size > 0)
        {
          mXSDO[SDOsrv].pDat = mXSDO[SDOsrv].pBuf;
        }
      }
#endif

      if (mXSDO[SDOsrv].size > 0)
      { // this is not the last block
        
        // continue with next block
        mXSDO[SDOsrv].sgcount = 0;
        mXSDO[SDOsrv].state = BLK_STAT_RDGO;

        return 15; // Nothing more to do here
      }
      else
      { // this is the very last block
        // Send confirmation for entire block transfer
        pTxData[0] = (6 << 5) + (mXSDO[SDOsrv].sgunused << 2) + 1; // Server Command specifier
        pTxData[1] = 0; // optional CRC
        pTxData[2] = 0; // optional CRC
        pTxData[3] = 0;
        pTxData[4] = 0;
        pTxData[5] = 0;
        pTxData[6] = 0;
        pTxData[7] = 0;

        return 0x20; // Send response
      }
    }
    if ((*pRxData & 0xE3) == 0xA1) 
    { // confirmation for ENTIRE block received
      mXSDO[SDOsrv].state = STAT_NONE;
      return 15; // Nothing more to do here
    }
    return 0xF0; // Report general error
  }

  // Check if SDO Block Download (Write) Transfer completed
  if (mXSDO[SDOsrv].state == BLK_STAT_WRCONF)
  {
    if (mXSDO[SDOsrv].sgunused == 0xFF)
    { // Data sent was too big
      return 0xF0; // Report general error
    }
    if ((*pRxData & 0xC1) == 0xC1) 
    { // this is the confirmation request
      // Send confirmation for block
      pTxData[0] = (5 << 5) + 1; // Server Command specifier
      pTxData[1] = 0;
      pTxData[2] = 0;
      pTxData[3] = 0;
      pTxData[4] = 0;
      pTxData[5] = 0;
      pTxData[6] = 0;
      pTxData[7] = 0;
      mXSDO[SDOsrv].state = STAT_NONE;
#if USECB_ODDATARECEIVED
      RTOS_LOCK_PI(PIACC_APP,PISECT_ALL);
      MCOUSER_ODData(0,mXSDO[SDOsrv].idx,mXSDO[SDOsrv].sub,mXSDO[SDOsrv].pBuf,mXSDO[SDOsrv].bufcnt);
      RTOS_UNLOCK_PI(PIACC_APP,PISECT_ALL);
#endif // USECB_ODDATARECEIVED
      return 0x20; // Send SDO response
    }
    return 0xF0; // Report general error
  }

  // Check if this is a new block download (write) request
  if ((pRxData[0] & 0xE0) == (6 << 5))
  { // This is SDO block download, init transfer
    
    // Calculate index 
    index = pRxData[2]; 
    index = (index << 8) + pRxData[1]; 

#if USECB_APPSDO_WRITE
    // Application call back for custom OD entries
    ret_val = MCOUSER_AppSDOWriteInit(SDOsrv+1,index,pRxData[3],&totalsize,&mXSDO[SDOsrv].bufsize,&mXSDO[SDOsrv].pDat,&dummytype);
    if (ret_val > 0x01)
    { // abort request
      return ret_val;
    }
    else if (ret_val == 1)
    { // mark as write and call back
      access = ODWR;
      mXSDO[SDOsrv].calb = TRUE;
      // Save base pointer for buffer
      mXSDO[SDOsrv].pBuf = mXSDO[SDOsrv].pDat;
      // If not set in call-back, set size of transfer to buffer size
      if (totalsize == 0)
      {
        mXSDO[SDOsrv].size = mXSDO[SDOsrv].bufsize;
      }
      else
      {
        mXSDO[SDOsrv].size = totalsize;
      }
      mXSDO[SDOsrv].bufcnt = 0;
    }
    else
    { // mark as not used by user call back
      mXSDO[SDOsrv].pBuf = 0;
    }
#endif // USECB_APPSDO_WRITE
    
    if (ret_val == 0)
    { // until now no return value assigned
      ret_val = XSDO_SearchODGenTable(index,pRxData[3],(uint8_t *)&access,&(mXSDO[SDOsrv].size),&(mXSDO[SDOsrv].pDat));
    }

    if (ret_val != 0xFF)
    { // Index and Subindex match

      if (!(access & ODWR) || (mXSDO[SDOsrv].size < 4))
      { // Write access not allowed!
        return 0x04;
      }

      // Verify data size available
      len = pRxData[4+3];
      len <<= 8;
      len += pRxData[4+2];
      len <<= 8;
      len += pRxData[4+1];
      len <<= 8;
      len += pRxData[4+0];

      if (mXSDO[SDOsrv].size < len)
      { // Data block requested is too big
        return 7;
      }

      // Total size to transfer
      mXSDO[SDOsrv].size = len;

      // calculate number of blocks for next transfer
      blksize = (len+6) / 7;
      if (blksize > BLK_MAX_SIZE)
      {
        blksize = BLK_MAX_SIZE;
      }

      // Initate block writing
      mXSDO[SDOsrv].sgcount = 1;
      mXSDO[SDOsrv].state = BLK_STAT_WRITE;
      mXSDO[SDOsrv].idx = index;
      mXSDO[SDOsrv].sub = pRxData[3];

      // Prepare answer
      pTxData[0] = 5 << 5; // Server Command specifier
      pTxData[4] = (uint8_t) blksize; // Number of blocks expected
      pTxData[5] = 0;
      pTxData[6] = 0;
      pTxData[7] = 0;
      return 0x20; // Send SDO response
    }

    // When this is reached, something went wrong
    mXSDO[SDOsrv].state = STAT_NONE;
    return 0x03;
  }

  // Check if this is an new upload (read) request
  if ((pRxData[0] & 0xE0) == (5 << 5))
  { // This is SDO upload, init segmented transfer
    // Calculate index 
    index = pRxData[2]; 
    index = (index << 8) + pRxData[1]; 

#if USECB_APPSDO_READ
    // Application call back for custom OD entries
    ret_val = MCOUSER_AppSDOReadInit(SDOsrv+1,index,pRxData[3],&totalsize,&mXSDO[SDOsrv].size,&mXSDO[SDOsrv].pDat,&dummytype);
    if (ret_val > 0x01)
    { // abort request
      return ret_val;
    }
    else if (ret_val != 0)
    { // mark as read and call back
      access = ODRD;
      mXSDO[SDOsrv].calb = TRUE;
      mXSDO[SDOsrv].pBuf = mXSDO[SDOsrv].pDat;
    }
#endif // USECB_APPSDO_READ

    if (ret_val == 0)
    { // until now no return value assigned
      ret_val = XSDO_SearchODGenTable(index,pRxData[3],(uint8_t *)&access,&(mXSDO[SDOsrv].size),&(mXSDO[SDOsrv].pDat));
    }

    if (ret_val != 0xFF)
    { // Index and Subindex match

      if (!(access & ODRD))
      { // Read access not allowed!
        return 0x05;
      }

      mXSDO[SDOsrv].state = BLK_STAT_READ;
      mXSDO[SDOsrv].sgcount = 0;

      mXSDO[SDOsrv].idx = index;
      mXSDO[SDOsrv].sub = pRxData[3];

      mXSDO[SDOsrv].sgblks = pRxData[4]; // blksize to use 

      pTxData[0] = (6 << 5) + 2; // no crc, size indicated
#if USECB_APPSDO_READ
      if (totalsize)
      {
        pTxData[4] = (uint8_t) ((totalsize      ) & 0xFFul);
        pTxData[5] = (uint8_t) ((totalsize >>  8) & 0xFFul);
        pTxData[6] = (uint8_t) ((totalsize >> 16) & 0xFFul);
        pTxData[7] = (uint8_t) ((totalsize >> 24) & 0xFFul);
      }
      else      
#endif // USECB_APPSDO_READ
      {
        pTxData[4] = (uint8_t) ((mXSDO[SDOsrv].size      ) & 0xFFul);
        pTxData[5] = (uint8_t) ((mXSDO[SDOsrv].size >>  8) & 0xFFul);
        pTxData[6] = (uint8_t) ((mXSDO[SDOsrv].size >> 16) & 0xFFul);
        pTxData[7] = (uint8_t) ((mXSDO[SDOsrv].size >> 24) & 0xFFul);
      }

      return 0x20; // Send SDO response
    }
  }
#endif // USE_BLOCKED_SDO

  // Check if SDO Segmented Download (Write) Transfer in progress 
  if ((mXSDO[SDOsrv].state == STAT_WRITE) || (mXSDO[SDOsrv].state == STAT_WRITECB))
  { 
    // Check if command specifier is right
    if (pRxData[0] & 0xE0)
    {
      mXSDO[SDOsrv].state = STAT_NONE;
      return 0xF0; // Report general error
    }
    if (((*pRxData & 0x10) >> 4) != (mXSDO[SDOsrv].tog & 0x01))
    {
      mXSDO[SDOsrv].state = STAT_NONE;
      return 0x02; // Report toggle error
    }

#if USE_XSDOCB_WRITE
    if (mXSDO[SDOsrv].state == STAT_WRITECB)
    {
      if ( MCOUSER_XSDOWriteSegment( *pRxData & 0x01, 
                                 7 - ((*pRxData & 0x0E) >> 1), 
                                 &(pRxData[1]) ) != TRUE
         )
      {
        // received data too big
        mXSDO[SDOsrv].state = STAT_NONE;
        return 0x03; // Report access error
      }
    }
    else
#endif // USE_XSDOCB_WRITE

    if ( XSDO_WriteNextSegment( *pRxData & 0x01, 
                                 7 - ((*pRxData & 0x0E) >> 1), 
                                 &(pRxData[1]),
                                 SDOsrv ) != TRUE
       )
    {
      // received data too big
      mXSDO[SDOsrv].state = STAT_NONE;
      return 0x03; // Report access error
    }
  
    if (*pRxData & 0x01)
    { // last segment
      mXSDO[SDOsrv].state = STAT_NONE; // transfer completed
#if USECB_ODDATARECEIVED
      RTOS_LOCK_PI(PIACC_APP,PISECT_ALL);
      MCOUSER_ODData(0,mXSDO[SDOsrv].idx,mXSDO[SDOsrv].sub,mXSDO[SDOsrv].pBuf,mXSDO[SDOsrv].bufcnt);
      RTOS_UNLOCK_PI(PIACC_APP,PISECT_ALL);
#endif // USECB_ODDATARECEIVED
    }

    return 1; // all OK
  }
  
  // Check if SDO Segmented Upload (Read) Transfer in progress 
  if (mXSDO[SDOsrv].state == STAT_READ)
  { 
    // Check if command specifier is right
    if ((pRxData[0] & 0xE0) != 0x60)
    {
      mXSDO[SDOsrv].state = STAT_NONE;
      return 0xF0; // Report general error
    }
    if (((*pRxData & 0x10) >> 4) != (mXSDO[SDOsrv].tog & 0x01))
    {
      mXSDO[SDOsrv].state = STAT_NONE;
      return 0x02; // Report toggle error
    }
    XSDO_ReadNextSegment(pTxData,SDOsrv);

    mXSDO[SDOsrv].tog = ~mXSDO[SDOsrv].tog; // Toggle the toggle bit
    return 0x20; // Transmit pTXData as response
  }

  // Check if this is a new download (write) request
  if ((pRxData[0] & 0xF0) == 0x20) 
  { // This is SDO download, segmented or expedited transfer
#if USECB_APPSDO_WRITE
    // Application call back for custom OD entries
    ret_val = MCOUSER_AppSDOWriteInit(SDOsrv+1,index,pRxData[3],&totalsize,&mXSDO[SDOsrv].bufsize,&mXSDO[SDOsrv].pDat,&dummytype);
    if (ret_val > 0x01)
    { // abort request
      return ret_val;
    }
    else if (ret_val == 1)
    { // mark as write
      access = ODWR;
      // Save base pointer for buffer
      mXSDO[SDOsrv].pBuf = mXSDO[SDOsrv].pDat;
      // If not set in call-back, set size of transfer to buffer size
      if (totalsize == 0)
      {
        mXSDO[SDOsrv].size = mXSDO[SDOsrv].bufsize;
      }
      else
      {
        mXSDO[SDOsrv].size = totalsize;
      }
      mXSDO[SDOsrv].bufcnt = 0;
    }
    else
    { // mark as not used by user call back
      mXSDO[SDOsrv].pBuf = 0;
    }
#endif // USECB_APPSDO_WRITE

    if (ret_val == 0)
    { // until now no return value assigned
      ret_val = XSDO_SearchODGenTable(index,pRxData[3],(uint8_t *)&access,&(mXSDO[SDOsrv].size),&(mXSDO[SDOsrv].pDat));
    }

    if (ret_val != 0xFF)
    { // Index and Subindex match

      if (!(access & ODWR) || (mXSDO[SDOsrv].size < 4))
      { // Write access not allowed!
        return 0x04;
      }

      if (pRxData[0] & 0x02)
      { // expedited transfer
        RTOS_LOCK_PI(PIACC_SDO, PISECT_ALL);
        mXSDO[SDOsrv].pDat[0] = pRxData[4];
        mXSDO[SDOsrv].pDat[1] = pRxData[5];
        mXSDO[SDOsrv].pDat[2] = pRxData[6];
        mXSDO[SDOsrv].pDat[3] = pRxData[7];
        RTOS_UNLOCK_PI(PIACC_SDO, PISECT_ALL);
        // all completed
#if USECB_APPSDO_WRITE
        ret_val = MCOUSER_AppSDOWriteComplete(SDOsrv+1, index, pRxData[3], mXSDO[SDOsrv].size, 0);
        if (ret_val <= 1)
          ret_val = 0x10;
#else
        ret_val = 0x10;
#endif
        return ret_val;
      }

      // Verify data size available
      len = pRxData[4+3];
      len <<= 8;
      len += pRxData[4+2];
      len <<= 8;
      len += pRxData[4+1];
      len <<= 8;
      len += pRxData[4+0];

      if (mXSDO[SDOsrv].size < len)
      { // Data block requested is too big
        return 7;
      }

      // Total size to transfer
      mXSDO[SDOsrv].size = len;

      // Initate segmented writing
      mXSDO[SDOsrv].state = STAT_WRITE;
      mXSDO[SDOsrv].tog = 0; // Init toggle bit
      mXSDO[SDOsrv].calb = FALSE;
      mXSDO[SDOsrv].idx = index;
      mXSDO[SDOsrv].sub = pRxData[3];

      return 0x10;
    }

#if USE_XSDOCB_WRITE
    else if (MCOUSER_XSDOInitWrite(index,pRxData[3],mXSDO[SDOsrv].size) == TRUE)
    {
      // Initate segmented writing
      mXSDO[SDOsrv].state = STAT_WRITECB;
      mXSDO[SDOsrv].tog = 0; // Init toggle bit
      mXSDO[SDOsrv].calb = TRUE;
      mXSDO[SDOsrv].idx = index;
      mXSDO[SDOsrv].sub = pRxData[3];

      return 0x10;
    }
#endif // USE_XSDOCBWRITE

    // When this is reached, nothing in generic OD
    return 0;
  }

  // Check if this is an new upload (read) request
  if (pRxData[0] == 0x40)
  { // This is SDO upload, init segmented transfer

#if USECB_APPSDO_READ
    // Application call back for custom OD entries
    ret_val = MCOUSER_AppSDOReadInit(SDOsrv+1,index,pRxData[3],&totalsize,&mXSDO[SDOsrv].size,&mXSDO[SDOsrv].pDat,&dummytype);
    if (ret_val > 0x01)
    { // abort request
      return ret_val;
    }
    else if (ret_val != 0)
    {
      mXSDO[SDOsrv].pBuf = mXSDO[SDOsrv].pDat;
    }
    access = ODRD;
#endif // USECB_APPSDO_READ

    if (ret_val == 0)
    { // until now no return value assigned
      ret_val = XSDO_SearchODGenTable(index,pRxData[3],(uint8_t *)&access,&(mXSDO[SDOsrv].size),&(mXSDO[SDOsrv].pDat));
    }
    
    if (ret_val != 0xFF)
    { // Index and Subindex match

      if (!(access & ODRD))
      { // Read access not allowed!
        return 0x05;
      }

      mXSDO[SDOsrv].state = STAT_READ;
      mXSDO[SDOsrv].tog = 0;
      mXSDO[SDOsrv].calb = FALSE;
      mXSDO[SDOsrv].idx = index;
      mXSDO[SDOsrv].sub = pRxData[3];

      pTxData[0] = 0x41; // size indicated
#if USECB_APPSDO_READ
      if (totalsize)
      {
        pTxData[4] = (uint8_t) ((totalsize      ) & 0xFFul);
        pTxData[5] = (uint8_t) ((totalsize >>  8) & 0xFFul);
        pTxData[6] = (uint8_t) ((totalsize >> 16) & 0xFFul);
        pTxData[7] = (uint8_t) ((totalsize >> 24) & 0xFFul);
      }
      else      
#endif // USECB_APPSDO_READ
      {
        pTxData[4] = (uint8_t) ((mXSDO[SDOsrv].size      ) & 0xFFul);
        pTxData[5] = (uint8_t) ((mXSDO[SDOsrv].size >>  8) & 0xFFul);
        pTxData[6] = (uint8_t) ((mXSDO[SDOsrv].size >> 16) & 0xFFul);
        pTxData[7] = (uint8_t) ((mXSDO[SDOsrv].size >> 24) & 0xFFul);
      }
      
/* Original version without indicated size
      pTxData[0] = 0x40;
      pTxData[4] = 0;
      pTxData[5] = 0;
      pTxData[6] = 0;
      pTxData[7] = 0;
*/

      return 0x20; // Send SDO response
    }
  }

  // Nothing was done here
  return 0;
}  

#endif // USE_EXTENDED_SDO


/**************************************************************************
PUBLIC FUNCTIONS
***************************************************************************/

#if NR_OF_SDOSERVER > 0
/**************************************************************************
DOES:    Handles incoming SDO Request for accesses to SDO Server
Parameters
RETURNS: 0: Wrong access, SDO Abort sent
1: Access was made, SDO Response sent
GLOBALS: Various global variables with configuration information
**************************************************************************/
uint8_t XSDO_HandleSDOServerParam(
  uint16_t index,    // OD index
  uint8_t *pData    // pointer to SDO Request message
)
{
  uint8_t reply[4] = { 0,0,0,0 };  // SDO reply value
  uint32_t cobid;

  if (pData[3] == 0) // subindex 0
  { // Nr Of Entries: "2"
    if (pData[0] == 0x40)
    { // Read command
      reply[0] = 2;
      MCO_ReplyWith(reply, 1);
      return 1;
    }
    else
    { // Write
      MCO_SendSDOAbort(SDO_ABORT_READONLY);
      return 0;
    }
  }
  else if (pData[3] == 1) // subindex 1
  { // COB ID client
    if (index == 0x1200)
    {
      cobid = 0x600 + MY_NODE_ID;
    }
    else
    {
      if ((index & 0x7F) == MY_NODE_ID)
      { // disable this entry where own node ID matches index
        cobid = 0x80000000ul;
      }
      else
      {
        cobid = CAN_ID_SDOREQUEST((index & 0x7F), MY_NODE_ID);
      }
    }
    if (pData[0] == 0x40)
    { // Read command
      reply[0] = (uint8_t)cobid;
      reply[1] = (uint8_t)(cobid >> 8);
      reply[2] = (uint8_t)(cobid >> 16);
      reply[3] = (uint8_t)(cobid >> 24);
      MCO_ReplyWith(reply, 4);
      return 1;
    }
    else
    { // Write
      MCO_SendSDOAbort(SDO_ABORT_READONLY);
      return 0;
    }
  }
  else if (pData[3] == 2) // subindex 2
  { // COB ID server
    if (index == 0x1200)
    {
      cobid = 0x580 + MY_NODE_ID;
    }
    else
    {
      if ((index & 0x7F) == MY_NODE_ID)
      { // disable this entry where own node ID matches index
        cobid = 0x80000000ul;
      }
      else
      {
        cobid = CAN_ID_SDORESPONSE((index & 0x7F), MY_NODE_ID);
      }
    }
    if (pData[0] == 0x40)
    { // Read command
      reply[0] = (uint8_t)cobid;
      reply[1] = (uint8_t)(cobid >> 8);
      reply[2] = (uint8_t)(cobid >> 16);
      reply[3] = (uint8_t)(cobid >> 24);
      MCO_ReplyWith(reply, 4);
      return 1;
    }
    else
    { // Write
      MCO_SendSDOAbort(SDO_ABORT_READONLY);
      return 0;
    }
  }

  MCO_SendSDOAbort(SDO_ABORT_UNKNOWNSUB);
  return 0;
}
#endif


#if NR_OF_SDO_CLIENTS > 0
/**************************************************************************
DOES:    Handles incoming SDO Request for accesses to SDO Server
Parameters
RETURNS: 0: Wrong access, SDO Abort sent
1: Access was made, SDO Response sent
GLOBALS: Various global variables with configuration information
**************************************************************************/
uint8_t XSDO_HandleSDOClientParam(
  uint16_t index,    // OD index
  uint8_t *pData    // pointer to SDO Request message
)
{
  uint8_t reply[4] = { 0,0,0,0 };  // SDO reply value
  uint32_t cobid;

  if (pData[3] == 0) // subindex 0
  { // Nr Of Entries: "3"
    if (pData[0] == 0x40)
    { // Read command
      reply[0] = 3;
      MCO_ReplyWith(reply, 1);
      return 1;
    }
    else
    { // Write
      MCO_SendSDOAbort(SDO_ABORT_READONLY);
      return 0;
    }
  }
  else if (pData[3] == 1) // subindex 1
  { // COB ID client
    if (index == 0x1200)
    {
      if (IS_NODE_CiA301_CLIENT())
      {
        cobid = 0x600 + MY_NODE_ID;
      }
      else
      { // disable this entry
        cobid = 0x80000000ul;
      }
    }
    else
    {
      if ((index & 0x7F) == MY_NODE_ID)
      { // disable this entry where own node ID matches index
        cobid = 0x80000000ul;
      }
      else
      {
        cobid = CAN_ID_SDOREQUEST(MY_NODE_ID, (index & 0x7F));
      }
    }
    if (pData[0] == 0x40)
    { // Read command
      reply[0] = (uint8_t)cobid;
      reply[1] = (uint8_t)(cobid >> 8);
      reply[2] = (uint8_t)(cobid >> 16);
      reply[3] = (uint8_t)(cobid >> 24);
      MCO_ReplyWith(reply, 4);
      return 1;
    }
    else
    { // Write
      MCO_SendSDOAbort(SDO_ABORT_READONLY);
      return 0;
    }
  }
  else if (pData[3] == 2) // subindex 2
  { // COB ID server
    if (index == 0x1200)
    {
      if (IS_NODE_CiA301_CLIENT())
      {
        cobid = 0x580 + MY_NODE_ID;
      }
      else
      { // disable this entry
        cobid = 0x80000000ul;
      }
    }
    else
    {
      if ((index & 0x7F) == MY_NODE_ID)
      { // disable this entry where own node ID matches index
        cobid = 0x80000000ul;
      }
      else
      {
        cobid = CAN_ID_SDORESPONSE(MY_NODE_ID, (index & 0x7F));
      }
    }
    if (pData[0] == 0x40)
    { // Read command
      reply[0] = (uint8_t)cobid;
      reply[1] = (uint8_t)(cobid >> 8);
      reply[2] = (uint8_t)(cobid >> 16);
      reply[3] = (uint8_t)(cobid >> 24);
      MCO_ReplyWith(reply, 4);
      return 1;
    }
    else
    { // Write
      MCO_SendSDOAbort(SDO_ABORT_READONLY);
      return 0;
    }
  }
  else if (pData[3] == 3) // subindex 3
  { // Node ID of server
    if (pData[0] == 0x40)
    { // Read command
      reply[0] = (uint8_t)(index & 0x7F) + 1;
      reply[1] = 0;
      reply[2] = 0;
      reply[3] = 0;
      MCO_ReplyWith(reply, 1);
      return 1;
    }
    else
    { // Write
      MCO_SendSDOAbort(SDO_ABORT_READONLY);
      return 0;
    }
  }

  MCO_SendSDOAbort(SDO_ABORT_UNKNOWNSUB);
  return 0;
}
#endif


#if USE_EXTENDED_SDO

void XSDO_Abort (
  uint8_t SDOServer // Number of SDO Server (0 to NR_OF_SDOSERVER-1)
  )
{
  // Reset state machine
  mXSDO[SDOServer].state = STAT_NONE;
}


/**************************************************************************
DOES:    Process SDO Segmented Requests to generic OD entries
RETURNS: 0x00 Nothing was done
         0x01 OK, handled, response generated
         0x02 Abort, SDO Abort was generated
**************************************************************************/
uint8_t XSDO_HandleExtended (
  uint8_t *pReqBUF, // Pointer to 8 data bytes with SDO data from request
  CAN_MSG *pResCAN, // Pointer to SDO response
  uint8_t SDOServer // Number of SDO Server (< NR_OF_SDOSERVER)
  )
{
uint16_t index;   // Index of SDO request
uint8_t  ret_val; // Return value

#if (NR_OF_SDOSERVER > 1)
  if (SDOServer >= NR_OF_SDOSERVER)
  { // Fatal error from caller, paramter out of range
    MCOUSER_FatalError(0x3001);
  }
#endif

  // Copy Multiplexor into response
  pResCAN->BUF[1] = pReqBUF[1]; // index lo
  pResCAN->BUF[2] = pReqBUF[2]; // index hi
  pResCAN->BUF[3] = pReqBUF[3]; // subindex

  // Check for abort
  if (*pReqBUF == 0x80)
  { // Abort code received
    // reset state machine for segmented transfers
    mXSDO[SDOServer].state = STAT_NONE;
    return 0; // simply ignore the abort received
  }

  // conformance check on sdo commands
  // only apply when outside a block transfer
  if (mXSDO[SDOServer].state != BLK_STAT_WRITE)
  {
    if (((*pReqBUF & 0xE0) == 0xE0) 
#if ! USE_BLOCKED_SDO
        || (*pReqBUF == 0xA0)
#endif
       )
    {
      // reset state machine for segmented transfers
      mXSDO[SDOServer].state = STAT_NONE;
      MCO_SendSDOAbort(SDO_ABORT_UNKNOWN_COMMAND);
      return 2;
    }
  }

  // Get requested index
  index = pReqBUF[2]; 
  index = (index << 8) + pReqBUF[1]; 

#if USE_BLOCKED_SDO
  // Save copy of response CANID, needed for block read transfers
  mXSDO[SDOServer].sgCANID = pResCAN->ID;
#endif

  ret_val = XSDO_HandleSegmented(index,pReqBUF,(uint8_t *)&(pResCAN->BUF[0]),SDOServer);
  switch (ret_val)
  {
  case 0: // Nothing found
    break;
  case 1: 
    XSDO_WriteSegConfirm(pResCAN,SDOServer);
    return 1;
  case 2:
    MCO_SendSDOAbort(SDO_ABORT_TOGGLE);
    return 2;
  case 3:
    MCO_SendSDOAbort(SDO_ABORT_TRANSFER);
    return 2;
  case 4:
    MCO_SendSDOAbort(SDO_ABORT_READONLY);
    return 2;
  case 5:
    MCO_SendSDOAbort(SDO_ABORT_WRITEONLY);
    return 2;
  case 6:
    MCO_SendSDOAbort(SDO_ABORT_NOT_EXISTS);
  return 2;
#if USE_BLOCKED_SDO
  case 7:
    MCO_SendSDOAbort(SDO_ABORT_DATATOBIG);
    return 2;
  case 8:
    MCO_SendSDOAbort(SDO_ABORT_INVALID_SEQ);
    return 2;
  case 9:
    MCO_SendSDOAbort(SDO_ABORT_CRC);
    return 2;
  case 15: // in middle of block transfer, no response
    return 1;
#endif
  case 0x10: 
    XSDO_WriteConfirm(pResCAN);
    return 1;
  case 0x20: 
  case 0x21: 
    // Transmit SDO Response message
    if (!MCOHW_PushMessage(pResCAN))
    {
      MCOUSER_FatalError(TXLOST_SDO);
    }
    return 1;
  default:
    MCO_SendSDOAbort(SDO_ABORT_GENERAL);
    return 2;
  }

  // Nothing done here
  return 0;
}


#if USE_BLOCKED_SDO
/**************************************************************************
DOES:    Called from ProcessStackTick
         Checks if we are in middle of Block Read transfer
RETURNS: FALSE, nothing done
         TRUE, transfer in progress, message generated
**************************************************************************/
uint8_t XSDO_BlkRdProgress (
  void
  )
{
CAN_MSG BlkTx;

#if NR_OF_SDOSERVER > 1
  // With each call check one SDO server
  mSDObr++;
  if (mSDObr >= NR_OF_SDOSERVER)
  {
    mSDObr = 0;
  }
#endif

  if ( (mXSDO[mSDObr].state == BLK_STAT_RDGO) &&
       (MY_NMT_STATE != NMTSTATE_STOP)
     )
  {
    if (MCOHW_IsTimeExpired(mXSDO[mSDObr].sgdelay))
    { // only work on this with a delay
      // Reset timer
      mXSDO[mSDObr].sgdelay = MCOHW_GetTime() + BLK_B2B_TIMEOUT;
      // work on next segment piece
      XSDO_ReadNextSegment(&(BlkTx.BUF[0]),mSDObr);
      // Set CAN ID and length for next message
      BlkTx.ID = mXSDO[mSDObr].sgCANID;
      BlkTx.LEN = 8;
      // Transmit SDO Response message
      if (!MCOHW_PushMessage(&BlkTx))
      {
        MCOUSER_FatalError(TXLOST_SDO);
      }
      return 1;
    }
  }
  return 0;
}
#endif // USE_BLOCK_SDO

#endif // USE_EXTENDED_SDO

#endif // not CANopen FD


/**************************************************************************
DOES:    Checks if for a certain index and subindex a data entry exists
in the Object Dictionary, does NOT check 1xxxh entries
RETURNS: TRUE, if entry was found, then pDat contains pointer to data
and pLen the length of the data
**************************************************************************/
uint8_t OD_FindODDataEntry(
  uint8_t mode,   // Bit 0 set: Search Process Image (up to 4 byte data)
  // Bit 1 set: Search Generic Data (any size, any location)
  // Bit 2 set: Search Constant table (up to 4 byte data)
  uint16_t idx,   // Index of Object Dictionary entry to find
  uint8_t sub,    // Subindex of Object Dictionary entry to find
  uint32_t* pLen, // When found, contains length of entry
  uint8_t** pDat  // When found, contains pointer to data of entry
)
{
  uint16_t found;
  // pointer to an entry in gODProcTable
  OD_PROCESS_DATA_ENTRY MEM_CONST* pOD;
  // pointer to const OD entry
  uint8_t MEM_CONST* pDatConst;
  // return value required for search of generic table, here unused
  uint8_t acc;
  // return value: pointer to const OD entry
  static uint8_t mBufDat;
  // return value: length of entry
  static uint32_t mBufLen;

  if (mode & 0x01)
  { // Data available in process image?
    found = MCO_SearchODProcTable(idx, sub);
    // entry found?
    if (found != 0xFFFF)
    {
      // initialize pointer
      pOD = OD_ProcTablePtr(0);
      pOD += (uint32_t)(found);
      *pDat = &(gProcImg[GEN_RD16(PIACC_NONE, (uint8_t*)&(pOD->off_lo))]);
      *pLen = (uint32_t)(pOD->len & 0x07);
      return TRUE;
    }
  }

#if USE_EXTENDED_SDO
  if (mode & 0x02)
  { // Data available in generic data table?
    found = XSDO_SearchODGenTable(idx, sub, &acc, pLen, pDat);
    // entry found?
    if (found != 0xFF)
    {
      return TRUE;
    }
  }
#endif // USE_EXTENDED_SDO

  if (mode & 0x04)
  { // Data available in SDO response table?
    found = MCO_SearchOD(idx, sub);
    // entry found?
    if (found != 0xFFFF)
    {
      pDatConst = OD_SDOResponseTablePtr(0);
      pDatConst += (found * 8); // point to SDOREPLY entry
      // use buffer to return value
      mBufLen = 4 - ((*pDatConst >> 2) & 0x03);
      pDatConst += 4; // point to data
      mBufDat = *pDatConst;
      // return the buffers
      *pDat = (uint8_t*)&mBufDat;
      *pLen = mBufLen;
      return TRUE;
    }
  }

  // not found
  return FALSE;
}


/**************************************************************************
DOES:    Find the Process Image offset to an OD entry.
When USE_XOD_ACCESS is set, use this to find an offset before
using the PI_READ or PI_WRITE macros.
RETURNS: 0xFFFF if not found, else the offset
**************************************************************************/
uint16_t OD_GetPIEntryOffset(
  uint16_t index,
  uint8_t subindex
)
{
  ptrdiff_t offset = -1;
  uint32_t len;
  uint8_t* pdat;

  if (OD_FindODDataEntry(3, index, subindex, &len, &pdat))
  { // entry found, now check if result is in process image
    // calculate offset into process image
    offset = (pdat - &(gProcImg[0]));
  }

  if (offset >= PROCIMG_SIZE_MAX)
  { // found but not within process image
#ifdef DEBUG
    MCOUSER_FatalError(ERRFT_PIR);
#endif  
    return (0xFFFFU);
  }
  else if (offset < 0)
  {
    return (0xFFFFU);
  }
  else
  { // found and offset in range
    return ((uint16_t)offset);
  }
}


#if USE_EXTENDED_SDO

/**************************************************************************
DOES:    Search the OD table with generic OD entries for
         a specifc index and subindex.
RETURNS: 0xFF if OD entry NOT found, else the record number
         access is set to access type
         len is set to length of entry
         pDat is set to first byte of data
**************************************************************************/
uint8_t XSDO_SearchODGenTable(
  uint16_t index,     // Index of OD entry searched
  uint8_t  subindex,  // Subindex of OD entry searched 
  uint8_t* access,
  uint32_t* len,
  uint8_t** pDat
)
{
  uint8_t j = 0;
  uint16_t compare;
  OD_GENERIC_DATA_ENTRY MEM_CONST* pGOD;

  while (j < 0xFF) // Loop until maximum table size
  {
    pGOD = OD_GenericTablePtr(j);
    compare = pGOD->idx_hi;
    compare <<= 8;
    compare += pGOD->idx_lo;
    if (compare == 0xFFFF) // End of table reached
    {
      break;
    }
    if (compare == index) // Index found
    {
      if (pGOD->subidx == subindex) // Subindex found
      {
        // disregard entry if is FD-only
        if (((pGOD->access & COFD) != COFD))
        {
          *access = pGOD->access;
          *len = pGOD->len_hi;
          *len <<= 8;
          *len += pGOD->len_lo;
#if USE_GENOD_PTR
          * pDat = pGOD->pDat;
#else
          compare = pGOD->off_hi;
          compare <<= 8;
          compare += pGOD->off_lo;
          *pDat = (uint8_t*)&(gProcImg[compare]);
#endif
          return j;
        }
      }
    }
    j++;
  } // while search loop
  return 0xFF;
}

#endif // USE_EXTENDED_SDO

/*----------------------- END OF FILE ----------------------------------*/
