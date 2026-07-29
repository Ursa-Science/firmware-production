/**************************************************************************
MODULE:    PROFILE_CiA447
CONTAINS:  MicroCANopen application profile specific extensions
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


#ifndef _PROFILE_H
#define _PROFILE_H

#ifdef __cplusplus
extern "C" {
#endif


/**************************************************************************
Functions to store and retrieve the node ID from which the last SDO request
or RPDO was received
**************************************************************************/
void PROFILE_SetSDOFromNode (uint8_t node_id);
uint8_t PROFILE_GetSDOFromNode (void);
void PROFILE_SetRPDOFromNode (uint8_t node_id);
uint8_t PROFILE_GetRPDOFromNode (void);


/**************************************************************************
DOES:    Checks if an ID is a 447 PDO
RETURNS: TRUE if yes, else FALSE
**************************************************************************/
uint8_t IS_CAN_ID_ANY_PDO (COBID_TYPE CANID);


/**************************************************************************
DOES:    Returns the node ID of the node transmitting a PDO
RETURNS: 1 to 16, or 127 if not known
**************************************************************************/
uint8_t GET_NODE_ID_FROM_PDO (COBID_TYPE CANID);


#if USE_PROFILE_RPDO
/**************************************************************************
DOES:    This function checks if this RPDO CAN ID is a duplicate/mirror of 
         multiple same RPDOs coming from different devices.
         CiA447: supporting same virtual device PDO coming from multiple
         nodes.
RETURNS: RPDO CAN ID of the main RPDO that can be used to handle this RPDO.
**************************************************************************/
uint32_t PROFILE_ExtHandleRPDO(
  uint32_t RPDO_canid
  );
#endif

#ifdef __cplusplus
}
#endif

#endif // _PROFILE_H
/**************************************************************************
END OF FILE
**************************************************************************/
