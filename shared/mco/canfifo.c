/**************************************************************************
MODULE:    CANFIFO
CONTAINS:  MicroCANopen Plus implementation, CAN SW Filter and I/O FIFOs
COPYRIGHT: Embedded Systems Academy (EmSA) 2002-2024
           All rights reserved. esacademy.com
DISCLAIM:  Read and understand our disclaimer before using this code!
           www.esacademy.com/disclaim.htm
           This software was written in accordance to the guidelines at
           www.esacademy.com/software/softwarestyleguide.pdf
LICENSE:   THIS FILE IS PART OF MICROCANOPEN PLUS
           ONLY USERS WHO PURCHASED A LICENSE MAY USE THIS SOFTWARE
VERSION:   7.17, EmSA 04-MAR-24
           $LastChangedDate: 2024-03-04 12:55:21 +0100 (Mon, 04 Mar 2024) $
           $LastChangedRevision: 5559 $
***************************************************************************/ 

#include "mcop_inc.h"

#if USE_CANFIFO

#if (TXFIFOSIZE != 0) && (TXFIFOSIZE != 4) && (TXFIFOSIZE != 8) && (TXFIFOSIZE != 16) && (TXFIFOSIZE != 32) && (TXFIFOSIZE != 64)
#error TXFIFOSIZE must be 0 (deactivated), 4, 8, 16, 32 or 64
#endif
#if (RXFIFOSIZE != 0) && (RXFIFOSIZE != 4) && (RXFIFOSIZE != 8) && (RXFIFOSIZE != 16) && (RXFIFOSIZE != 32) && (RXFIFOSIZE != 64) && (RXFIFOSIZE != 128)
#error RXFIFOSIZE must be 0 (deactivated), 4, 8, 16, 32, 64 or 128
#endif
#if (MGRFIFOSIZE != 0) && (MGRFIFOSIZE != 4) && (MGRFIFOSIZE != 8) && (MGRFIFOSIZE != 16) && (MGRFIFOSIZE != 32) && (MGRFIFOSIZE != 64)
#error MGRFIFOSIZE must be 0 (deactivated), 4, 8, 16, 32 or 64

#endif


/**************************************************************************
LOCAL VARIABLES
***************************************************************************/ 

typedef struct
{
#if USE_CANSWFILTER
  uint32_t FilterList[64]; // 2048 bits, one for each 11bit CAN ID
#endif
#if (TXFIFOSIZE > 0)
  uint32_t TxIn;
  uint32_t TxOut;
  CAN_MSG TxFifo[TXFIFOSIZE];
#endif
#if (RXFIFOSIZE > 0)
  uint32_t RxIn;
  uint32_t RxOut;
  CAN_MSG RxFifo[RXFIFOSIZE];
#endif
#if MGR_MONITOR_ALL_NODES && (MGRFIFOSIZE > 0)
  uint32_t MGRIn;
  uint32_t MGROut;
  CAN_MSG MGRFifo[MGRFIFOSIZE];
#endif
#if (INDEX_FOR_DIAGNOSTICS != 0)
 #if (TXFIFOSIZE > 0)
  uint8_t TxCur; // diagnostic, current fill level
 #endif
 #if (RXFIFOSIZE > 0)
  uint8_t RxCur; // diagnostic, current fill level
 #endif
 #if MGR_MONITOR_ALL_NODES && (MGRFIFOSIZE > 0)
  uint8_t MgrCur; // diagnostic, current fill level
 #endif
#endif
} CANFIFOINFO;

// Module variable with all FIFO information
static CANFIFOINFO MEM_BUF mCF;


/**************************************************************************
PUBLIC FUNCTIONS
***************************************************************************/ 

#if USE_CANSWFILTER
/**************************************************************************
DOES: Initializes the CAN SW receive filtering variables
      Default: No CAN message is received.
***************************************************************************/ 
void CANSWFILTER_Init (
  void
  )
{
uint16_t loop;

  for (loop = 0; loop < 64; loop++)
  {
    mCF.FilterList[loop] = 0;
  }
}


/**************************************************************************
DOES: Setting a single CAN receive filter
***************************************************************************/ 
uint8_t CANSWFILTER_Set (
  COBID_TYPE CANID     // CAN-ID to be received by filter 
  )
{
  // In the array of 2048 bits, set the bit Nr CANID
  mCF.FilterList[(CANID & 0x07E0) >> 5] |= (1UL << (CANID & 0x001F));

  if (CANID <= 0x7FF)
  {
    return TRUE;
  }
  return FALSE;
}


/**************************************************************************
DOES: Clearing a previously set CAN receive filter
***************************************************************************/ 
uint8_t CANSWFILTER_Clear (
  COBID_TYPE CANID     // CAN-ID to be no longer received
  )
{
  // In the array of 2048 bits, set the bit Nr CANID
  mCF.FilterList[(CANID & 0x07E0) >> 5] &= ~(1UL << (CANID & 0x001F));

  if (CANID <= 0x7FF)
  {
    return TRUE;
  }
  return FALSE;
}


/**************************************************************************
DOES: Checks if a CAN filter is set for an ID
***************************************************************************/ 
uint8_t CANSWFILTER_Match (
  COBID_TYPE CANID     // CAN-ID requested
  )
{
  // In the array of 2048 bits, return the bit Nr CANID
  return (0x01 & ((mCF.FilterList[(CANID & 0x07E0) >> 5]) >> (CANID & 0x001F)));
}
#endif // USE_CANSWFILTER


#if (TXFIFOSIZE > 0)
/**************************************************************************
DOES: Flushes / clears the TXFIFO, all data stored in FIFO is lost
***************************************************************************/ 
void CANTXFIFO_Flush (
  void
  )
{
  mCF.TxIn = 0;
  mCF.TxOut = 0;
#if (INDEX_FOR_DIAGNOSTICS != 0)
  mCF.TxCur = 0;
  gMCODiag.TxFIFOStatus = TXFIFOSIZE;
#endif
}


/**************************************************************************
DOES: Returns number of elements in TXFIFO
***************************************************************************/ 
uint8_t CANTXFIFO_FillStatus (
  void
  )
{
int8_t ret_val;
  
  ret_val = (int8_t)mCF.TxIn - (int8_t)mCF.TxOut;
  
  return ((uint8_t)(ret_val & 0x7F));
}


/**************************************************************************
DOES:    Returns a CAN message pointer to the next free location in FIFO.
         Application can then copy a CAN message to the location given by 
         the pointer and MUST call CANTXFIFO_InDone() when done.
RETURNS: CAN message pointer into FIFO
         NULL if FIFO is full
***************************************************************************/ 
CAN_MSG MEM_BUF *CANTXFIFO_GetInPtr (
  void
  )
{
uint8_t ovr; // check if FIFO is full

  ovr = mCF.TxIn + 1;
  ovr &= (TXFIFOSIZE-1);

  if (ovr != mCF.TxOut)
  {// FIFO is not full
    return &(mCF.TxFifo[mCF.TxIn]);
  }
  gMCOConfig.HWStatus |= HW_TXOR; // signal overrun
#if (INDEX_FOR_DIAGNOSTICS != 0)
  gMCODiag.TxFIFOStatus &= 0x00FFFFFFul;
  gMCODiag.TxFIFOStatus |= 0x81000000ul;
  if ((gMCODiag.TxFIFOStatus & 0x00FF0000ul) < 0x00FE0000ul)
  {
    gMCODiag.TxFIFOStatus += 0x00010000ul;
  }
#endif
  return 0;
}


/**************************************************************************
DOES:    Must be called by application after data was copied into the FIFO,
         this increments the internal IN pointer to the next free location 
         in the FIFO.
RETURNS: nothing
***************************************************************************/ 
void CANTXFIFO_InDone (
  void
  )
{
  // Increment IN pointer
  mCF.TxIn = ((mCF.TxIn+1) & (TXFIFOSIZE-1));
#if (INDEX_FOR_DIAGNOSTICS != 0)
  mCF.TxCur++;
  if (mCF.TxCur > ((gMCODiag.TxFIFOStatus & 0x0000FF00ul) >> 8))
  {
    gMCODiag.TxFIFOStatus &= 0xFFFF00FF;
    gMCODiag.TxFIFOStatus |= (((uint32_t) mCF.TxCur) << 8);
  }
#endif
}


/**************************************************************************
DOES:    Returns a CAN message pointer to the next OUT message in the FIFO.
         Application can then copy the CAN message from the location given 
         by the pointer to the desired destination and MUST call 
         CANTXFIFO_OutDone() when done.
RETURNS: CAN message pointer into FIFO      
         NULL if FIFO is empty
***************************************************************************/ 
CAN_MSG MEM_BUF *CANTXFIFO_GetOutPtr (
  void
  )
{
  if (mCF.TxIn != mCF.TxOut)
  { // message available in FIFO
    return &(mCF.TxFifo[mCF.TxOut]);
  }
  return 0;
}


/**************************************************************************
DOES:    Must be called by application after data was copied from the FIFO,
         this increments the internal OUT pointer to the next location 
         in the FIFO.
RETURNS: nothing
***************************************************************************/ 
void CANTXFIFO_OutDone (
  void
  )
{
  mCF.TxOut = ((mCF.TxOut+1) & (TXFIFOSIZE-1));
#if (INDEX_FOR_DIAGNOSTICS != 0)
  mCF.TxCur--;
#endif
}
#endif // (TXFIFOSIZE > 0)


#if (RXFIFOSIZE > 0)
/**************************************************************************
DOES: Flushes / clears the RXFIFO, all data stored in FIFO is lost
***************************************************************************/ 
void CANRXFIFO_Flush (
  void
  )
{
  mCF.RxIn = 0;
  mCF.RxOut = 0;
#if (INDEX_FOR_DIAGNOSTICS != 0)
  mCF.RxCur = 0;
  gMCODiag.RxFIFOStatus = RXFIFOSIZE;
#endif
}


/**************************************************************************
DOES:    Returns a CAN message pointer to the next free location in FIFO.
         Application can then copy a CAN message to the location given by 
         the pointer and MUST call CANRXFIFO_InDone() when done.
RETURNS: CAN message pointer into FIFO      
         NULL if FIFO is full
***************************************************************************/ 
CAN_MSG MEM_BUF *CANRXFIFO_GetInPtr (
  void
  )
{
uint8_t ovr; // check if FIFO is full

  ovr = mCF.RxIn + 1;
  ovr &= (RXFIFOSIZE-1);

  if (ovr != mCF.RxOut)
  {// FIFO is not full
    return &(mCF.RxFifo[mCF.RxIn]);
  }
  gMCOConfig.HWStatus |= HW_RXOR; // signal overrun
#if (INDEX_FOR_DIAGNOSTICS != 0)
  gMCODiag.RxFIFOStatus &= 0x00FFFFFFul;
  gMCODiag.RxFIFOStatus |= 0x81000000ul;
  if ((gMCODiag.RxFIFOStatus & 0x00FF0000ul) < 0x00FE0000ul)
  {
    gMCODiag.RxFIFOStatus += 0x00010000ul;
  }
#endif
  return 0;
}


/**************************************************************************
DOES:    Must be called by application after data was copied into the FIFO,
         this increments the internal IN pointer to the next free location 
         in the FIFO.
RETURNS: nothing
***************************************************************************/ 
void CANRXFIFO_InDone (
  void
  )
{
  // Increment IN pointer
  mCF.RxIn = ((mCF.RxIn+1) & (RXFIFOSIZE-1));
#if (INDEX_FOR_DIAGNOSTICS != 0)
  mCF.RxCur++;
  if (mCF.RxCur > ((gMCODiag.RxFIFOStatus & 0x0000FF00ul) >> 8))
  {
    gMCODiag.RxFIFOStatus &= 0xFFFF00FF;
    gMCODiag.RxFIFOStatus |= (((uint32_t) mCF.RxCur) << 8);
  }
#endif
}


/**************************************************************************
DOES:    Returns a CAN message pointer to the next OUT message in the FIFO.
         Application can then copy the CAN message from the location given 
         by the pointer to the desired destination and MUST call 
         CANRXFIFO_OutDone() when done.
RETURNS: CAN message pointer into FIFO      
         NULL if FIFO is empty
***************************************************************************/ 
CAN_MSG MEM_BUF *CANRXFIFO_GetOutPtr (
  void
  )
{
  if (mCF.RxIn != mCF.RxOut)
  { // message available in FIFO
    return &(mCF.RxFifo[mCF.RxOut]);
  }
  return 0;
}


/**************************************************************************
DOES:    Must be called by application after data was copied from the FIFO,
         this increments the internal OUT pointer to the next location 
         in the FIFO.
RETURNS: nothing
***************************************************************************/ 
void CANRXFIFO_OutDone (
  void
  )
{
  mCF.RxOut = ((mCF.RxOut+1) & (RXFIFOSIZE-1));
#if (INDEX_FOR_DIAGNOSTICS != 0)
  mCF.RxCur--;
#endif
}
#endif // (RXFIFOSIZE > 0)


#if MGR_MONITOR_ALL_NODES && (MGRFIFOSIZE > 0)
/**************************************************************************
DOES: Flushes / clears the MGRFIFO, all data stored in FIFO is lost
***************************************************************************/ 
void CANMGRFIFO_Flush (
  void
  )
{
  mCF.MGRIn = 0;
  mCF.MGROut = 0;
#if (INDEX_FOR_DIAGNOSTICS != 0)
  mCF.MgrCur = 0;
  gMCODiag.RxMgrFIFOStatus = MGRFIFOSIZE;
#endif
}


/**************************************************************************
DOES:    Returns a CAN message pointer to the next free location in FIFO.
         Application can then copy a CAN message to the location given by 
         the pointer and MUST call CANMGRFIFO_InDone() when done.
RETURNS: CAN message pointer into FIFO      
         NULL if FIFO is full
***************************************************************************/ 
MEM_BUF CAN_MSG *CANMGRFIFO_GetInPtr (
  void
  )
{
uint8_t ovr; // check if FIFO is full

  ovr = mCF.MGRIn + 1;
  ovr &= (MGRFIFOSIZE-1);

  if (ovr != mCF.MGROut)
  {// FIFO is not full
    return &(mCF.MGRFifo[mCF.MGRIn]);
  }
  gMCOConfig.HWStatus |= HW_RXOR; // signal overrun
#if (INDEX_FOR_DIAGNOSTICS != 0)
  gMCODiag.RxMgrFIFOStatus &= 0x00FFFFFFul;
  gMCODiag.RxMgrFIFOStatus |= 0x81000000ul;
  if ((gMCODiag.RxMgrFIFOStatus & 0x00FF0000ul) < 0x00FE0000ul)
  {
    gMCODiag.RxMgrFIFOStatus += 0x00010000ul;
  }
#endif
  return 0;
}


/**************************************************************************
DOES:    Must be called by application after data was copied into the FIFO,
         this increments the internal IN pointer to the next free location 
         in the FIFO.
RETURNS: nothing
***************************************************************************/ 
void CANMGRFIFO_InDone (
  void
  )
{
  // Increment IN pointer
  mCF.MGRIn = ((mCF.MGRIn+1) & (MGRFIFOSIZE-1));
#if (INDEX_FOR_DIAGNOSTICS != 0)
  mCF.MgrCur++;
  if (mCF.MgrCur > ((gMCODiag.RxMgrFIFOStatus & 0x0000FF00ul) >> 8))
  {
    gMCODiag.RxMgrFIFOStatus &= 0xFFFF00FF;
    gMCODiag.RxMgrFIFOStatus |= (((uint32_t) mCF.MgrCur) << 8);
  }
#endif
}


/**************************************************************************
DOES:    Returns a CAN message pointer to the next OUT message in the FIFO.
         Application can then copy the CAN message from the location given 
         by the pointer to the desired destination and MUST call 
         CANMGRFIFO_OutDone() when done.
RETURNS: CAN message pointer into FIFO      
         NULL if FIFO is empty
***************************************************************************/ 
MEM_BUF CAN_MSG *CANMGRFIFO_GetOutPtr (
  void
  )
{
  if (mCF.MGRIn != mCF.MGROut)
  { // message available in FIFO
    return &(mCF.MGRFifo[mCF.MGROut]);
  }
  return 0;
}


/**************************************************************************
DOES:    Must be called by application after data was copied from the FIFO,
         this increments the internal OUT pointer to the next location 
         in the FIFO.
RETURNS: nothing
***************************************************************************/ 
void CANMGRFIFO_OutDone (
  void
  )
{
  mCF.MGROut = ((mCF.MGROut+1) & (MGRFIFOSIZE-1));
#if (INDEX_FOR_DIAGNOSTICS != 0)
  mCF.MgrCur--;
#endif
}
#endif // MGR_MONITOR_ALL_NODES && (MGRFIFOSIZE > 0)

#endif // USE_CANFIFO
/**************************************************************************
END OF FILE
**************************************************************************/

