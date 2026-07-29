/**************************************************************************
MODULE:    MCOINST
CONTAINS:  Compatibility with MCO v8+ driver API include
COPYRIGHT: Embedded Systems Academy, Inc. (EmSA) 2002-2024
           All rights reserved. esacademy.com
DISCLAIM:  Read and understand our disclaimer before using this code!
           www.esacademy.com/disclaim.htm
           This software was written in accordance to the guidelines at
           www.esacademy.com/software/softwarestyleguide.pdf
LICENSE:   This file may be freely distributed.
VERSION:   7.17, EmSA 04-MAR-24
           $LastChangedDate: 2024-03-04 12:55:21 +0100 (Mon, 04 Mar 2024) $
           $LastChangedRevision: 5559 $
***************************************************************************/


#ifndef _MCOINST_H
#define _MCOINST_H

#ifdef __cplusplus
extern "C" {
#endif

#include <string.h>
#include "mcohw.h"
#include "canfifo.h"
#include "mcop_inc.h"


#define NUM_CAN_PORTS  1  // Set to 1 to use FDCAN1, set to 2 to use FDCAN1 or FDCAN2

#if NUM_CAN_PORTS==1
#define CAN_LIST_INITIALIZER {FDCAN1}
#elif NUM_CAN_PORTS==2
#define CAN_LIST_INITIALIZER {FDCAN1, FDCAN2}
#elif NUM_CAN_PORTS==3
#error This architecture supports a maximum of two CAN ports.
#endif

// The number from the list of initializers to use
#define CAN_PORT_INDEX 0  // Set to 0 to use FDCAN1, set to 1 to use FDCAN2

#define OPT_SINGLE_HANDLE_PARAM void
#define OPT_FIRST_HANDLE_PARAM 
#define OPT_SINGLE_HANDLE_CALL_PARAM 
#define OPT_FIRST_HANDLE_CALL_PARAM 


#define MCO_PROT_LEGACY    0x01u
#define MCO_PROT_FD        0x02u
#define MCO_PROT_COMANAGER 0x10u

#if defined(MGR_MONITOR_ALL_NODES) && (MGR_MONITOR_ALL_NODES==1)
#define USE_CANOPEN_MANAGER 1
#else
#define USE_CANOPEN_MANAGER 0
#endif

#if defined(USE_CANOPEN_FD) && (USE_CANOPEN_FD==1)
#define PROCESS_CO_FD 1
#else
#define PROCESS_CO_FD 0
#endif
#if !defined(USE_CANOPEN_FD) || (defined(USE_CANOPEN_FD) && (USE_CANOPEN_FD==0))
#define PROCESS_CO_LEGACY 1
#else
#define PROCESS_CO_LEGACY 0
#endif

#define MCO_PROT_BIT_SET(mco_handle,testbits)  ( \
  (testbits==MCO_PROT_LEGACY) ? PROCESS_CO_LEGACY : \
  (testbits==MCO_PROT_FD) ? PROCESS_CO_FD : \
  (testbits==MCO_PROT_COMANAGER) ? USE_CANOPEN_MANAGER : \
  0)


#define HW_STATUS_USED  gMCOConfig.HWStatus
#define CONFIG_USED  gMCOConfig

// Empty definition of ´handle´
#define handle (void)0

#endif //  _MCOINST_H
/**************************************************************************
END OF FILE
**************************************************************************/

