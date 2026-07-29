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
VERSION:   7.17, EmSA 04-MAR-24
           $LastChangedDate: 2024-03-04 12:55:21 +0100 (Mon, 04 Mar 2024) $
           $LastChangedRevision: 5559 $
***************************************************************************/ 

#ifndef _XSDO_H
#define _XSDO_H

#ifdef __cplusplus
extern "C" {
#endif

#include "mco.h"

/**************************************************************************
PUBLIC FUNCTIONS
***************************************************************************/

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
  uint32_t *pLen, // When found, contains length of entry
  uint8_t **pDat  // When found, contains pointer to data of entry
);


/**************************************************************************
DOES:    Find the Process Image offset to an OD entry.
When USE_XOD_ACCESS is set, use this to find an offset before
using the PI_READ or PI_WRITE macros.
RETURNS: 0xFFFF if not found, else the offset
**************************************************************************/
uint16_t OD_GetPIEntryOffset(
  uint16_t index,
  uint8_t subindex
);


#if USE_EXTENDED_SDO

void XSDO_Abort (
  uint8_t SDOServer // Number of SDO Server (1 to NR_OF_SDOSERVER)
  );


/**************************************************************************
DOES:    Process SDO Segmented Requests to generic OD entries
RETURNS: 0x00 Nothing was done
         0x01 OK, handled, response generated
         0x02 Abort, SDO Abort was generated
**************************************************************************/
uint8_t XSDO_HandleExtended (
  uint8_t *pReqBUF, // Pointer to 8 data bytes with SDO data from request
  CAN_MSG *pResCAN, // Pointer to SDO response
  uint8_t SDOServer // Number of SDO Server (1 to NR_OF_SDOSERVER)
  );


/**************************************************************************
DOES:    Called from ProcessStackTick
         Checks if we are in middle of Block Read transfer
RETURNS: FALSE, nothing done
         TRUE, transfer in progress, message generated
**************************************************************************/
uint8_t XSDO_BlkRdProgress (
  void
  );

#endif // USE_EXTENDED_SDO


#if NR_OF_SDOSERVER > 0
/**************************************************************************
DOES:    Internal Funtion: Handles incoming SDO Request for accesses to 
         SDO Server Parameters
RETURNS: 0: Wrong access, SDO Abort sent
         1: Access was made, SDO Response sent
GLOBALS: Various global variables with configuration information
**************************************************************************/
uint8_t XSDO_HandleSDOServerParam (
   uint16_t index,    // OD index
   uint8_t *pData    // pointer to SDO Request message
  );
#endif

#if NR_OF_SDO_CLIENTS > 0
/**************************************************************************
DOES:    Internal Funtion: Handles incoming SDO Request for accesses to 
         SDO Client Parameters
RETURNS: 0: Wrong access, SDO Abort sent
         1: Access was made, SDO Response sent
GLOBALS: Various global variables with configuration information
**************************************************************************/
uint8_t XSDO_HandleSDOClientParam (
   uint16_t index,    // OD index
   uint8_t *pData    // pointer to SDO Request message
  );
#endif

#ifdef __cplusplus
}
#endif

#endif // _XSDO_H
/**************************************************************************
END OF FILE
**************************************************************************/
