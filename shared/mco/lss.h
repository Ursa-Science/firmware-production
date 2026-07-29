/**************************************************************************
MODULE:    LSS
CONTAINS:  MicroCANopen Plus - Support for Layer Setting Services Consumer
COPYRIGHT: Embedded Systems Academy (EmSA) 2002-2024
           All rights reserved. esacademy.com
           This software was written in accordance to the guidelines at
           www.esacademy.com/software/softwarestyleguide.pdf
DISCLAIM:  Read and understand our disclaimer before using this code!
           www.esacademy.com/disclaim.htm
LICENSE:   THIS IS THE COMMERCIAL PLUS VERSION OF MICROCANOPEN
           ONLY USERS WHO PURCHASED A LICENSE MAY USE THIS SOFTWARE
           See file license_commercial_plus.txt
VERSION:   7.17, EmSA 04-MAR-24
           $LastChangedDate: 2024-03-04 12:55:21 +0100 (Mon, 04 Mar 2024) $
           $LastChangedRevision: 5559 $
***************************************************************************/

#ifndef _LSS_H
#define _LSS_H

#ifdef __cplusplus
extern "C" {
#endif

#include "mco.h"

#if USE_LSS_SERVER
 // If enabled, LSS Fast Scan is implemented.
 // Due to merging LSS and LSS fast-scan slave modules, just follow the LSS setting.
 #define USE_MICROLSS (USE_LSS_SERVER)
#endif

// No node ID assigned
#define LSS_NID_NONE 0xFF

// Is node ID in legal range
#define LSS_IS_NID_SET(node_id) ((node_id > 0) && (node_id < 0x80))

// LSS CAN bps values
#define LSS_BPS_10     8
#define LSS_BPS_20     7
#define LSS_BPS_50     6
#define LSS_BPS_NOTSET 5
#define LSS_BPS_125    4
#define LSS_BPS_250    3
#define LSS_BPS_500    2
#define LSS_BPS_800    1
#define LSS_BPS_1000   0

// Doing LSS in stopped state
#define NMTSTATE_LSS 0xF0

// LSS modes
#define LSS_MODE_OPERATION  0
#define LSS_MODE_CONFIG     1
#define LSS_MODE_PASSIVE    2
#define LSS_MODE_REACT      3

// LSS Command Specifiers
#define LSS_SWMOD_GLOB      4
#define LSS_SWMOD_VID      64
#define LSS_SWMOD_PID      65
#define LSS_SWMOD_REV      66
#define LSS_SWMOD_SER      67
#define LSS_SWMOD_RESP     68
#define LSS_CONF_NID       17
#define LSS_CONF_BIT       19
#define LSS_ACT_BIT        21
#define LSS_STOR_CONF      23
#define LSS_INQ_VID        90
#define LSS_INQ_PID        91
#define LSS_INQ_REV        92
#define LSS_INQ_SER        93
#define LSS_INQ_NID        94
#define LSS_REQID_VID      70
#define LSS_REQID_PID      71
#define LSS_REQID_REV_LO   72
#define LSS_REQID_REV_HI   73
#define LSS_REQID_SER_LO   74
#define LSS_REQID_SER_HI   75
#define LSS_REQID_NCONF    76
#define LSS_ID_SLAVE       79
#define LSS_ID_NCONF_SLAVE 80
#define LSS_MICROLSS       81

#if defined(USE_CANOPEN_FD) && USE_CANOPEN_FD
 // With CANopen FD, enforce use of feedback nibbles
 // Set to the CiA 1305 command specifier for switch state selective
 #ifndef LSS_REQID_NIBBLER
  #define LSS_REQID_NIBBLER 0x64
 #endif
#endif

// Offsets into NVOL memory
#define NVOL_LSSENA 0
#define NVOL_LSSNID 1
#define NVOL_LSSBPS 2
#define NVOL_LSSCHK 3

// Use LSS values
#define NVOL_LSSENA_VAL 0x5A
// Execute LSS, do not use values
#define NVOL_DOLSS_VAL  0x4C

/*************************************************************************
Function Prototypes
*************************************************************************/

/****************************************************************
DOES:    Process all LSS messages.
RETURNS:
*****************************************************************/
void LSS_HandleMsg (
  uint8_t Len,
  uint8_t *pDat
  );

/****************************************************************
DOES:    Check and update LSS status
RETURNS: FALSE: LSS is finished for this node
         TRUE:  Otherwise (LSS is still in process)
*****************************************************************/
uint8_t LSS_DoLSS (
  void
  );


/****************************************************************
DOES:    Gets the LSS ID that LSS_Init() determined.
         Call after LSS_Init().
RETURNS: LSS ID in passed array - has to be writable.
*****************************************************************/
void LSS_GetLSSID(
  uint32_t lssid[4]
);


/****************************************************************
DOES:    Sets the LSS ID. Use, if LSS ID is not hard coded and
         does not come from process image locations.
         Call after LSS_Init() and before LSS processing starts.
RETURNS: nothing
*****************************************************************/
void LSS_SetLSSID(
  uint32_t lssid[4]
);


/****************************************************************
DOES:    Initialize LSS mechanism (variables etc.)
GLOBALS: Sets mLSS.active status flag to TRUE
RETURNS: -
*****************************************************************/
void LSS_Init(
  void
);


#if USE_29BIT_LSSFEEDBACK == 1
/****************************************************************
DOES:    Sends an LSS response feedback msg, this transmits
         a 29bit CAN ID message with DLC = 0
RETURNS: -
*****************************************************************/
void MCOHW_Push29Message (
  uint32_t canid // CAN ID to use
  );
#endif


/****************************************************************
DOES:    LSS Load Configuration Command
RETURNS: Retrieves the current configuration stored in memory.
         Node ID is set to zero, if no configuration found
*****************************************************************/
void LSS_LoadConfiguration (
  uint16_t *Bitrate,  // returns CAN bitrate in kbit (1000,800,500,250,125,50,25 or 10)
  uint8_t *Node_ID    // returns CANopen node ID (0-127)
  );

/****************************************************************
DOES:    Verifies a 4-byte configuration record
         from NVOL_LSSNID to NVOL_LSSCHK for plausible values
RETURNS: TRUE, if configuration is valid
*****************************************************************/
uint8_t LSS_CheckConfiguration (
  uint8_t cfg[4]
  );

#endif  // if _LSS_H

#ifdef __cplusplus
}
#endif

/*******************************************************************************
END OF FILE
*******************************************************************************/
