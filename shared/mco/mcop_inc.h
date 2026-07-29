/**************************************************************************
MODULE:    MCOP_INC.h
CONTAINS:  MicroCANopen Plus, all includes
COPYRIGHT: Embedded Systems Academy (EmSA) 2002-2024
           All rights reserved. esacademy.com
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

#ifndef _MCOP_INC_H
#define _MCOP_INC_H

#ifdef __cplusplus
extern "C" {
#endif

#include "nodecfg.h"
#include "mco_types.h"
#include "mcohw_cfg.h"
#include "mcohw.h"
#include "mco.h"
#include "mcohw_LEDs.h"
#include "canfifo.h"
#include "mcop.h"
#include "xsdo.h"
#include "profile.h"
#include "lss.h"
#include "svninfo.h"

#include "procimg.h"

#if (USE_DYNAMIC_PDO_MAPPING == 1)
  #include "xpdo.h"
#endif

#if (USE_XOD_ACCESS == 1)
  #include "xod.h"
#endif

#if (USE_CANOPEN_FD == 1)
  #include "usdo.h"
#endif

#if defined(NR_OF_SDO_CLIENTS) && (NR_OF_SDO_CLIENTS>0)
  #if USE_CANOPEN_FD
    #include "usdoclnt.h"
  #endif
  #if !USE_CANOPEN_FD || (defined(USE_CANOPEN_DUALMODE) && (USE_CANOPEN_DUALMODE==1))
    #include "sdoclnt.h"
  #endif
#endif // defined(NR_OF_SDO_CLIENTS) && (NR_OF_SDO_CLIENTS>0)

#if (USE_REMOTE_ACCESS == 1)
  #include "mcohw_com.h"
  #include "raccess.h"
  #include "raserial.h"
  #include "racrc.h"
#endif

#if USE_LEDS
#include "mcohw_LEDs.h"
#endif

#if USE_EXTERNAL_MEMORY
#include "extmem.h"
#endif

#ifdef __SIMULATION__
  #include "mcohwPCSIM.h"
  #include "simdriver.h"
  #include "simnodehandler.h"
#endif

#ifdef __cplusplus
}
#endif

#endif // _MCOP_INC_H
/**************************************************************************
END OF FILE
**************************************************************************/
