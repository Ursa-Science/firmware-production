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

#ifndef _CANFIFO_H
#define _CANFIFO_H

#include "mco.h"

#ifdef __cplusplus
extern "C" {
#endif


/**************************************************************************
FIFO FILL SIZE MACROS FOR COMPARISONS
***************************************************************************/ 
#define TXFIFO_EMPTY      (CANTXFIFO_FillStatus() == 0)
#define TXFIFO_HALFFULL   (CANTXFIFO_FillStatus() >= (TXFIFOSIZE/2))
#define TXFIFO_3QUARTFULL (CANTXFIFO_FillStatus() >= (TXFIFOSIZE*3/4))

  
/**************************************************************************
PUBLIC FUNCTIONS
***************************************************************************/ 

/**************************************************************************
DOES:    Initializes the CAN SW receive filtering variables
         Default: No CAN message is received.
***************************************************************************/ 
void CANSWFILTER_Init (
  void
  );


/**************************************************************************
DOES:    Setting a single CAN receive filter
RETURNS: TRUE if filter set, else FALSE
***************************************************************************/ 
uint8_t CANSWFILTER_Set (
  COBID_TYPE CANID     // CAN-ID to be received by filter 
  );


/**************************************************************************
DOES:    Clearing a previously set CAN receive filter
RETURNS: TRUE if filter cleared, else FALSE
***************************************************************************/ 
uint8_t CANSWFILTER_Clear (
  COBID_TYPE CANID     // CAN-ID to be no longer received
  );


/**************************************************************************
DOES:    Checks if a CAN filter is set for an ID
RETURNS: TRUE if matched
***************************************************************************/ 
uint8_t CANSWFILTER_Match (
  COBID_TYPE CANID     // CAN-ID requested
  );


/**************************************************************************
DOES:    Flushes / clears the TXFIFO, all data stored in FIFO is lost
***************************************************************************/ 
void CANTXFIFO_Flush (
  void
  );


/**************************************************************************
DOES:    Check fill status of the TXFIFO
RETURNS: Number of elements in TXFIFO
***************************************************************************/ 
uint8_t CANTXFIFO_FillStatus (
  void
  );

  
/**************************************************************************
DOES:    Returns a CAN message pointer to the next free location in FIFO.
         Application can then copy a CAN message to the location given by 
         the pointer and MUST call CANTXFIFO_InDone() when done.
RETURNS: CAN message pointer into FIFO      
         NULL if FIFO is full
***************************************************************************/ 
CAN_MSG MEM_BUF *CANTXFIFO_GetInPtr (
  void
  );


/**************************************************************************
DOES:    Must be called by application after data was copied into the FIFO,
         this increments the internal IN pointer to the next free location 
         in the FIFO.
RETURNS: nothing
***************************************************************************/ 
void CANTXFIFO_InDone (
  void
  );


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
  );


/**************************************************************************
DOES:    Must be called by application after data was copied from the FIFO,
         this increments the internal OUT pointer to the next location 
         in the FIFO.
RETURNS: nothing
***************************************************************************/ 
void CANTXFIFO_OutDone (
  void
  );


/**************************************************************************
DOES:    Flushes / clears the RXFIFO, all data stored in FIFO is lost
***************************************************************************/ 
void CANRXFIFO_Flush (
  void
  );


/**************************************************************************
DOES:    Returns a CAN message pointer to the next free location in FIFO.
         Application can then copy a CAN message to the location given by 
         the pointer and MUST call CANRXFIFO_InDone() when done.
RETURNS: CAN message pointer into FIFO      
         NULL if FIFO is full
***************************************************************************/ 
CAN_MSG MEM_BUF *CANRXFIFO_GetInPtr (
  void
  );


/**************************************************************************
DOES: 	 Must be called by application after data was copied into the FIFO,
         this increments the internal IN pointer to the next free location 
		     in the FIFO.
RETURNS: nothing
***************************************************************************/ 
void CANRXFIFO_InDone (
  void
  );


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
  );


/**************************************************************************
DOES:    Must be called by application after data was copied from the FIFO,
         this increments the internal OUT pointer to the next location 
         in the FIFO.
RETURNS: nothing
***************************************************************************/ 
void CANRXFIFO_OutDone (
  void
  );


/**************************************************************************
DOES:    Flushes / clears the MGRFIFO, all data stored in FIFO is lost
***************************************************************************/ 
void CANMGRFIFO_Flush (
  void
  );


/**************************************************************************
DOES:    Returns a CAN message pointer to the next free location in FIFO.
         Application can then copy a CAN message to the location given by 
         the pointer and MUST call CANMGRFIFO_InDone() when done.
RETURNS: CAN message pointer into FIFO      
         NULL if FIFO is full
***************************************************************************/ 
CAN_MSG MEM_BUF *CANMGRFIFO_GetInPtr (
  void
  );


/**************************************************************************
DOES:    Must be called by application after data was copied into the FIFO,
         this increments the internal IN pointer to the next free location 
         in the FIFO.
RETURNS: nothing
***************************************************************************/ 
void CANMGRFIFO_InDone (
  void
  );


/**************************************************************************
DOES:    Returns a CAN message pointer to the next OUT message in the FIFO.
         Application can then copy the CAN message from the location given 
         by the pointer to the desired destination and MUST call 
         CANMGRFIFO_OutDone() when done.
RETURNS: CAN message pointer into FIFO      
         NULL if FIFO is empty
***************************************************************************/ 
CAN_MSG MEM_BUF *CANMGRFIFO_GetOutPtr (
  void
  );


/**************************************************************************
DOES:     Must be called by application after data was copied from the FIFO,
         this increments the internal OUT pointer to the next location 
         in the FIFO.
RETURNS: nothing
***************************************************************************/ 
void CANMGRFIFO_OutDone (
  void
  );

#ifdef __cplusplus
}
#endif

#endif // _CANFIFO_H
/**************************************************************************
END OF FILE
**************************************************************************/
