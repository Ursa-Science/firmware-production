/**************************************************************************
MODULE:    MCO_TYPES.H
CONTAINS:  Data types used by MicroCANopen
COPYRIGHT: Embedded Systems Academy (EmSA) 2002-2024
           All rights reserved. esacademy.com
           This software was written in accordance to the guidelines at
           www.esacademy.com/software/softwarestyleguide.pdf
DISCLAIM:  Read and understand our disclaimer before using this code!
           www.esacademy.com/disclaim.htm
           This software was written in accordance to the guidelines at
           www.esacademy.com/software/softwarestyleguide.pdf
LICENSE:   THIS IS THE COMMERCIAL VERSION OF MICRO CANOPEN PLUS
           ONLY USERS WHO PURCHASED A LICENSE MAY USE THIS SOFTWARE
           See file license_commercial_plus.txt or
           www.microcanopen.com/license_commercial_plus.txt
VERSION:   7.17, EmSA 04-MAR-24
           $LastChangedDate: 2024-03-04 12:55:21 +0100 (Mon, 04 Mar 2024) $
           $LastChangedRevision: 5559 $
***************************************************************************/ 

#ifndef _MCO_TYPES_H
#define _MCO_TYPES_H

#ifdef __cplusplus
extern "C" {
#endif


/**************************************************************************
DEFINES: MEMORY TYPE OPTIMIZATION
**************************************************************************/

// CONST Object Dictionary Data
#define MEM_CONST const

// Process data
#define MEM_PROC

// buffers
#define MEM_BUF

// non-process data
#define MEM_FAR


/**************************************************************************
DEFINES: TRUE AND FALSE
**************************************************************************/
#ifndef TRUE
#define TRUE  (1==1)
#endif
#ifndef FALSE
#define FALSE (!TRUE)
#endif
#ifndef NOT_SET
#define NOT_SET 2
#endif


/**************************************************************************
TYPEDEF: CANOPEN DATA TYPES
**************************************************************************/
#if !defined(UINTMAX_C)  // if not stdint.h is already used in the project
typedef unsigned char uint8_t;
typedef unsigned short uint16_t;
typedef unsigned int uint32_t;
typedef unsigned long long int uint64_t;
typedef char int8_t;
typedef short int16_t;
typedef int int32_t;
typedef long long int int64_t;
#endif // !defined(UINTMAX_C)


/**************************************************************************
TYPEDEF: CAN IDENTIFIER TYPE
         Plain CANopen does not use 29-bit IDs, use 16 here for memory
         optimization.
**************************************************************************/
#define CAN_ID_SIZE 16


#ifdef __cplusplus
}
#endif

#endif  // _MCO_TYPES_H
