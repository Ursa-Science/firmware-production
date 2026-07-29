/**************************************************************************
MODULE:    MCOHW_CFG.H
CONTAINS:  MicroCANopen hardware configuration
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

#ifndef _MCOHW_CFG_H
#define _MCOHW_CFG_H

#include <stdio.h>

/**************************************************************************
DEBUG PRINT DEFINITIONS
**************************************************************************/
#ifdef __SIMULATION__
  #define DEBUG_PRINT(loglevel,x) do  \
  { \
    if (loglevel==MCO_LOG_ERROR) \
    { \
      SimDriver_printf("ERROR: "); \
    } \
    else if (loglevel==MCO_LOG_WARNING) \
    { \
      SimDriver_printf("WARNING: "); \
    } \
    SimDriver_printf x; \
  } while(0)
#else // __SIMULATION__
#ifdef DEBUG
#define DEBUG_PRINT(loglevel,x) printf x
#else
#define DEBUG_PRINT(loglevel,x) do {} while (0)
#endif
#endif // __SIMULATION__


#if defined(USE_CUBEMX)
#include "main.h"
#define MCOHW_NO_TIMER_INIT 1
#endif

#if defined(USE_CMSIS_OS)
#include "cmsis_os.h"
extern osThreadId_t mcopTaskRxHandle;
extern osEventFlagsId_t mcopEventRxHandle;
extern uint8_t canRxFlag;
#endif // defined(USE_CMSIS_OS)


// Use simplified derivative definitions
#if defined(STM32F042x6) || defined(STM32F098xx) || \
    defined (STM32F030) || defined (STM32F031) || defined (STM32F051) || \
    defined (STM32F072) || defined (STM32F042) || defined (STM32F091) || \
    defined (STM32F072xB) || defined(STM32F070xB) || defined (STM32F070x6) || defined (STM32F030xC)
#if !defined(STM32F0)
#define STM32F0
#endif
#elif defined (STM32F303xC) || defined (STM32F334x8) || \
      defined (STM32F302x8) || defined (STM32F303xE)
#if !defined(STM32F3)
#define STM32F3
#endif
#elif defined (STM32F756xx) || defined (STM32F746xx) || defined (STM32F745xx) || defined (STM32F767xx) || \
      defined (STM32F769xx) || defined (STM32F777xx) || defined (STM32F779xx) || defined (STM32F722xx) || \
      defined (STM32F723xx) || defined (STM32F732xx) || defined (STM32F733xx)
#if !defined(STM32F7)
#define STM32F7
#endif
#elif defined (STM32L471xx) || defined (STM32L475xx) || defined (STM32L476xx) || defined (STM32L485xx) || defined (STM32L486xx) || \
      defined (STM32L496xx) || defined (STM32L4A6xx) || \
      defined (STM32L4R5xx) || defined (STM32L4R7xx) || defined (STM32L4R9xx) || defined (STM32L4S5xx) || defined (STM32L4S7xx) || defined (STM32L4S9xx)
#if !defined(STM32L4)
#define STM32L4
#endif
#elif defined(STM32G071xx) || defined(STM32G081xx) || defined(STM32G070xx) || defined(STM32G030xx) || defined(STM32G031xx) || \
      defined(STM32G041xx) || defined(STM32G0B0xx) || defined(STM32G0B1xx) || defined(STM32G0C1xx) || defined(STM32G050xx) || \
      defined(STM32G051xx) || defined(STM32G061xx)
#if !defined(STM32G0)
#define STM32G0
#endif
#elif defined(STM32G474xx) || defined(STM32G484xx) || defined(STM32G491xx) || defined(STM32G4A1xx) || defined(STM32G473xx) || \
      defined(STM32G483xx) || defined(STM32G471xx) || defined(STM32G431xx) || defined(STM32G441xx) || defined(STM32GBK1CB)
#if !defined(STM32G4)
#define STM32G4
#endif
#elif defined (STM32H573xx) || defined (STM32H563xx)  || defined (STM32H562xx) || defined (STM32H503xx)
#if !defined(STM32H5)
#define STM32H5
#endif
#elif defined (STM32H743xx) || defined (STM32H753xx)  || defined (STM32H750xx) || defined (STM32H742xx) || \
      defined (STM32H745xx) || defined (STM32H755xx)  || defined (STM32H747xx) || defined (STM32H757xx)
#if !defined(STM32H7)
#define STM32H7
#endif
#elif defined (STM32U575xx) || defined (STM32U585xx)  || defined (STM32U595xx) || defined (STM32U599xx) || \
      defined (STM32U5A5xx) || defined (STM32U5A9xx)  || defined (STM32U535xx) || defined (STM32U545xx)
#if !defined(STM32U5)
#define STM32U5
#endif
#else
#error The part is not yet supported by this driver
#endif

#if defined(STM32F0)
#include "stm32f0xx.h"
#include "stm32f0xx_hal.h"

#elif defined(STM32F1)
#include "stm32f10x.h"
#include "stm32f10x_hal.h"

#elif defined(STM32F3)
#include "stm32f3xx.h"
#include "stm32f3xx_hal.h"

#elif defined(STM32F4)
#include "stm32f4xx.h"
#include "stm32f4xx_hal.h"

#elif defined(STM32F7)
#include "stm32f7xx.h"
#include "stm32f7xx_hal.h"

#elif defined(STM32L4)
#include "stm32l4xx.h"
#include "stm32l4xx_hal.h"

#elif defined(STM32G0)
#include "stm32g0xx.h"
#include "stm32g0xx_hal.h"

#elif defined(STM32G4)
#include "stm32g4xx.h"
#include "stm32g4xx_hal.h"

#elif defined(STM32H5)
#include "stm32h5xx.h"
#include "stm32h5xx_hal.h"

#elif defined(STM32H7)
#include "stm32h7xx.h"
#include "stm32h7xx_hal.h"

#elif defined(STM32U5)
#include "stm32u5xx.h"
#include "stm32u5xx_hal.h"

#endif

#ifdef __ICCARM__
#define __INTRINSICS_INCLUDED
#include "arm_comm.h"
#endif

// CANopen Data Types
#include "mco_types.h"


// Function prototypes for HAL support
#if defined(USE_FDHAL)
void MCOHW_CAN_MspInit(FDCAN_HandleTypeDef *hcan);
void MCOHW_CAN_MspDeInit(FDCAN_HandleTypeDef *hcan);
#else
void MCOHW_CAN_MspInit(FDCAN_HandleTypeDef *hcan);
void MCOHW_CAN_MspDeInit(FDCAN_HandleTypeDef *hcan);
#endif
// Global variables for HAL support
extern TIM_HandleTypeDef  *gMcopTimHandle_p;


/**************************************************************************
DEFINES: clock-dependent configurations
**************************************************************************/

/* Definition for TIMx clock resources */
#define TIMx                           TIM7
#define TIMx_CLK_ENABLE()              __HAL_RCC_TIM7_CLK_ENABLE()
/* Definition for TIMx's NVIC */
#define TIMx_IRQn                      TIM7_IRQn
#define TIMx_IRQHandler                TIM7_IRQHandler


#if defined(STM32F0)

#ifdef  USE_CAN1
#define CANx                       CAN
#define CAN_CLK                    RCC_APB1Periph_CAN
#define CAN_GPIO_CLK               RCC_AHBPeriph_GPIOB
#define CAN_GPIO_PORT              GPIOB
#define CAN_RX_PIN                 GPIO_PIN_8
#define CAN_TX_PIN                 GPIO_PIN_9
#define CAN_AF_PORT                GPIO_AF4_CAN
#define CANx_RX_IRQn               CEC_CAN_IRQn
#define CANx_RX_IRQHandler         CEC_CAN_IRQHandler
#define CANx_SCE_IRQHandler
#define CAN_CLK_ENABLE()           __HAL_RCC_CAN1_CLK_ENABLE()
#define CAN_GPIO_CLK_ENABLE()      __HAL_RCC_GPIOB_CLK_ENABLE()
#define CAN_FORCE_RESET()          __HAL_RCC_CAN1_FORCE_RESET()
#define CAN_RELEASE_RESET()        __HAL_RCC_CAN1_RELEASE_RESET()
#else
#error This family only supports CAN1
#endif  /* USE_CAN1 */

#elif defined(STM32F3)

#ifdef  USE_CAN1
#define CANx                       CAN
#define CAN_CLK                    RCC_APB1Periph_CAN
#define CAN_GPIO_CLK               RCC_AHBPeriph_GPIOB
#define CAN_GPIO_PORT              GPIOB
#define CAN_RX_PIN                 GPIO_PIN_8
#define CAN_TX_PIN                 GPIO_PIN_9
#define CAN_AF_PORT                GPIO_AF9_CAN
#define CANx_RX_IRQn               CAN_RX0_IRQn
#define CANx_RX_IRQHandler         CAN_RX0_IRQHandler
#define CANx_SCE_IRQHandler        CAN_SCE_IRQHandler
#define CAN_CLK_ENABLE()           __HAL_RCC_CAN1_CLK_ENABLE()
#define CAN_GPIO_CLK_ENABLE()      __HAL_RCC_GPIOB_CLK_ENABLE()
#define CAN_FORCE_RESET()          __HAL_RCC_CAN1_FORCE_RESET()
#define CAN_RELEASE_RESET()        __HAL_RCC_CAN1_RELEASE_RESET()
#else
#error This family only supports CAN1
#endif  /* USE_CAN1 */

#elif defined(STM32F7)

#ifdef  USE_CAN1
#define CANx                       CAN1
#define CAN_CLK                    RCC_APB1Periph_CAN
#define CAN_GPIO_CLK               RCC_AHBPeriph_GPIOD
#define CAN_GPIO_PORT              GPIOD
#define CAN_RX_PIN                 GPIO_PIN_0
#define CAN_TX_PIN                 GPIO_PIN_1
#define CAN_AF_PORT                GPIO_AF9_CAN1
#define CANx_RX_IRQn               CAN1_RX0_IRQn
#define CANx_RX_IRQHandler         CAN1_RX0_IRQHandler
#define CANx_TX_IRQn               CAN1_TX_IRQn
#define CANx_TX_IRQHandler         CAN1_TX_IRQHandler
#define CANx_SCE_IRQn              CAN1_SCE_IRQn
#define CANx_SCE_IRQHandler        CAN1_SCE_IRQHandler
#define CAN_CLK_ENABLE()           __HAL_RCC_CAN1_CLK_ENABLE()
#define CAN_GPIO_CLK_ENABLE()      __HAL_RCC_GPIOD_CLK_ENABLE()
#define CAN_FORCE_RESET()          __HAL_RCC_CAN1_FORCE_RESET()
#define CAN_RELEASE_RESET()        __HAL_RCC_CAN1_RELEASE_RESET()
#elif defined(USE_CAN2)
#error Use of CAN2 not defined yet
#elif defined(USE_CAN3)
#error Use of CAN3 not defined yet
#else
#error This family only supports CAN1-3
#endif  /* USE_CAN1 */

#elif defined(STM32L4)

#ifdef  USE_CAN1
#define CANx                       CAN1
#define CAN_CLK                    RCC_APB1Periph_CAN
#define CAN_GPIO_CLK               RCC_AHBPeriph_GPIOD
#define CAN_GPIO_PORT              GPIOD
#define CAN_RX_PIN                 GPIO_PIN_0
#define CAN_TX_PIN                 GPIO_PIN_1
#define CAN_AF_PORT                GPIO_AF9_CAN1
#define CANx_RX_IRQn               CAN1_RX0_IRQn
#define CANx_RX_IRQHandler         CAN1_RX0_IRQHandler
#define CANx_TX_IRQn               CAN1_TX_IRQn
#define CANx_TX_IRQHandler         CAN1_TX_IRQHandler
#define CANx_SCE_IRQn              CAN1_SCE_IRQn
#define CANx_SCE_IRQHandler        CAN1_SCE_IRQHandler
#define CAN_CLK_ENABLE()           __HAL_RCC_CAN1_CLK_ENABLE()
#define CAN_GPIO_CLK_ENABLE()      __HAL_RCC_GPIOD_CLK_ENABLE()
#define CAN_FORCE_RESET()          __HAL_RCC_CAN1_FORCE_RESET()
#define CAN_RELEASE_RESET()        __HAL_RCC_CAN1_RELEASE_RESET()
#elif defined(USE_CAN2)
#error Use of CAN2 not defined yet
#else
#error This family only supports CAN1-2
#endif  /* USE_CAN1 */

#elif defined(STM32G0)

#define FDCAN_TYPE_G
#define FDCANx(canindex)                    ((canindex==2) ? FDCAN3 : ((canindex==1) ? FDCAN2 : FDCAN1))
#define FDCANx_CLK_ENABLE(canindex)         __HAL_RCC_FDCAN_CLK_ENABLE()
#define FDCANx_RX_GPIO_CLK_ENABLE(canindex) { if (canindex==1) __HAL_RCC_GPIOB_CLK_ENABLE(); else __HAL_RCC_GPIOA_CLK_ENABLE(); }
#define FDCANx_TX_GPIO_CLK_ENABLE(canindex) { if (canindex==1) __HAL_RCC_GPIOB_CLK_ENABLE(); else __HAL_RCC_GPIOA_CLK_ENABLE(); }
#define FDCANx_FORCE_RESET(canindex)        __HAL_RCC_FDCAN_FORCE_RESET()
#define FDCANx_RELEASE_RESET(canindex)      __HAL_RCC_FDCAN_RELEASE_RESET()
#define FDCANx_TX_PIN(canindex)             ((canindex==2) ? GPIO_PIN_15 : ((canindex==1) ? GPIO_PIN_13 : GPIO_PIN_12))
#define FDCANx_TX_GPIO_PORT(canindex)       ((canindex==1) ? GPIOB : GPIOA)
#define FDCANx_TX_AF(canindex)              ((canindex==2) ? GPIO_AF11_FDCAN3 : ((canindex==1) ? GPIO_AF9_FDCAN2 : GPIO_AF9_FDCAN1))
#define FDCANx_RX_PIN(canindex)             ((canindex==2) ? GPIO_PIN_8 : ((canindex==1) ? GPIO_PIN_12 : GPIO_PIN_11))
#define FDCANx_RX_GPIO_PORT(canindex)       ((canindex==1) ? GPIOB : GPIOA)
#define FDCANx_RX_AF(canindex)              ((canindex==2) ? GPIO_AF11_FDCAN3 : ((canindex==1) ? GPIO_AF9_FDCAN2 : GPIO_AF9_FDCAN1))
#define FDCANx_IRQn(canindex)               ((canindex==2) ? FDCAN3_IT0_IRQn : ((canindex==1) ? FDCAN2_IT0_IRQn : FDCAN1_IT0_IRQn))
#define FDCANx_IRQHandler(canindex)         ((canindex==2) ? FDCAN3_IT0_IRQHandler : ((canindex==1) ? FDCAN2_IT0_IRQHandler : FDCAN1_IT0_IRQHandler))

#elif defined(STM32G4)
#define FDCAN_TYPE_G

// Define FDCAN instance
#define FDCANx(canindex)                    FDCAN1

// Enable FDCAN clock
#define FDCANx_CLK_ENABLE(canindex)         __HAL_RCC_FDCAN_CLK_ENABLE()

// Enable GPIO clocks for FDCAN TX and RX
#define FDCANx_RX_GPIO_CLK_ENABLE(canindex) __HAL_RCC_GPIOA_CLK_ENABLE()
#define FDCANx_TX_GPIO_CLK_ENABLE(canindex) __HAL_RCC_GPIOA_CLK_ENABLE()

// Reset and release FDCAN peripheral
#define FDCANx_FORCE_RESET(canindex)        __HAL_RCC_FDCAN_FORCE_RESET()
#define FDCANx_RELEASE_RESET(canindex)      __HAL_RCC_FDCAN_RELEASE_RESET()

// Define FDCAN TX and RX pins and ports
#define FDCANx_TX_PIN(canindex)             GPIO_PIN_12
#define FDCANx_TX_GPIO_PORT(canindex)       GPIOA
#define FDCANx_TX_AF(canindex)              GPIO_AF9_FDCAN1

#define FDCANx_RX_PIN(canindex)             GPIO_PIN_11
#define FDCANx_RX_GPIO_PORT(canindex)       GPIOA
#define FDCANx_RX_AF(canindex)              GPIO_AF9_FDCAN1

// Define FDCAN interrupt
#define FDCANx_IRQn(canindex)               FDCAN1_IT0_IRQn
#define FDCANx_IRQHandler(canindex)         FDCAN1_IT0_IRQHandler

//#define FDCAN_TYPE_G
//#define FDCANx(canindex)                    ((canindex==2) ? FDCAN3 : ((canindex==1) ? FDCAN2 : FDCAN1))
//#define FDCANx_CLK_ENABLE(canindex)         __HAL_RCC_FDCAN_CLK_ENABLE()
//#define FDCANx_RX_GPIO_CLK_ENABLE(canindex) { if (canindex==1) __HAL_RCC_GPIOB_CLK_ENABLE(); else __HAL_RCC_GPIOA_CLK_ENABLE(); }
//#define FDCANx_TX_GPIO_CLK_ENABLE(canindex) { if (canindex==1) __HAL_RCC_GPIOB_CLK_ENABLE(); else __HAL_RCC_GPIOA_CLK_ENABLE(); }
//#define FDCANx_FORCE_RESET(canindex)        __HAL_RCC_FDCAN_FORCE_RESET()
//#define FDCANx_RELEASE_RESET(canindex)      __HAL_RCC_FDCAN_RELEASE_RESET()
//#define FDCANx_TX_PIN(canindex)             ((canindex==2) ? GPIO_PIN_15 : ((canindex==1) ? GPIO_PIN_13 : GPIO_PIN_12))
//#define FDCANx_TX_GPIO_PORT(canindex)       ((canindex==1) ? GPIOB : GPIOA)
//#define FDCANx_TX_AF(canindex) ((canindex == 1) ? GPIO_AF9_FDCAN2 : GPIO_AF9_FDCAN1)

//#define FDCANx_TX_AF(canindex)              ((canindex==2) ? GPIO_AF11_FDCAN3 : ((canindex==1) ? GPIO_AF9_FDCAN2 : GPIO_AF9_FDCAN1))
//#define FDCANx_RX_PIN(canindex)             ((canindex==2) ? GPIO_PIN_8 : ((canindex==1) ? GPIO_PIN_12 : GPIO_PIN_11))
//#define FDCANx_RX_GPIO_PORT(canindex)       ((canindex==1) ? GPIOB : GPIOA)
//#define FDCANx_RX_AF(canindex)              ((canindex==2) ? GPIO_AF11_FDCAN3 : ((canindex==1) ? GPIO_AF9_FDCAN2 : GPIO_AF9_FDCAN1))
//#define FDCANx_IRQn(canindex)               ((canindex==2) ? FDCAN3_IT0_IRQn : ((canindex==1) ? FDCAN2_IT0_IRQn : FDCAN1_IT0_IRQn))
//#define FDCANx_IRQHandler(canindex)         ((canindex==2) ? FDCAN3_IT0_IRQHandler : ((canindex==1) ? FDCAN2_IT0_IRQHandler : FDCAN1_IT0_IRQHandler))

#elif defined(STM32H5)

#define FDCAN_TYPE_H
#define FDCANx(canindex)                    ((canindex==0) ? FDCAN1 : FDCAN2)
#define FDCANx_CLK_ENABLE(canindex)         __HAL_RCC_FDCAN_CLK_ENABLE()
#define FDCANx_RX_GPIO_CLK_ENABLE(canindex) { if (canindex==0) __HAL_RCC_GPIOH_CLK_ENABLE(); else __HAL_RCC_GPIOB_CLK_ENABLE(); }
#define FDCANx_TX_GPIO_CLK_ENABLE(canindex) { if (canindex==0) __HAL_RCC_GPIOH_CLK_ENABLE(); else __HAL_RCC_GPIOB_CLK_ENABLE(); }
#define FDCANx_FORCE_RESET(canindex)        __HAL_RCC_FDCAN_FORCE_RESET()
#define FDCANx_RELEASE_RESET(canindex)      __HAL_RCC_FDCAN_RELEASE_RESET()
#define FDCANx_TX_PIN(canindex)             ((canindex==0) ? GPIO_PIN_13 : GPIO_PIN_13)
#define FDCANx_TX_GPIO_PORT(canindex)       ((canindex==0) ? GPIOH : GPIOB)
#define FDCANx_TX_AF(canindex)              ((canindex==0) ? GPIO_AF9_FDCAN1 : GPIO_AF9_FDCAN2)
#define FDCANx_RX_PIN(canindex)             ((canindex==0) ? GPIO_PIN_14 : GPIO_PIN_5)
#define FDCANx_RX_GPIO_PORT(canindex)       ((canindex==0) ? GPIOH : GPIOB)
#define FDCANx_RX_AF(canindex)              ((canindex==0) ? GPIO_AF9_FDCAN1 : GPIO_AF9_FDCAN2)
#define FDCANx_IRQn(canindex)               ((canindex==0) ? FDCAN1_IT0_IRQn : FDCAN2_IT0_IRQn)
#define FDCANx_IRQHandler(canindex)         ((canindex==0) ? FDCAN1_IT0_IRQHandler : FDCAN2_IT0_IRQHandler)

#elif defined(STM32H7)

#define FDCAN_TYPE_H
#define FDCAN_TYPE_H7
#define FDCANx(canindex)                    ((canindex==0) ? FDCAN1 : FDCAN2)
#define FDCANx_CLK_ENABLE(canindex)         __HAL_RCC_FDCAN_CLK_ENABLE()
#define FDCANx_RX_GPIO_CLK_ENABLE(canindex) { if (canindex==0) __HAL_RCC_GPIOH_CLK_ENABLE(); else __HAL_RCC_GPIOB_CLK_ENABLE(); }
#define FDCANx_TX_GPIO_CLK_ENABLE(canindex) { if (canindex==0) __HAL_RCC_GPIOH_CLK_ENABLE(); else __HAL_RCC_GPIOB_CLK_ENABLE(); }
#define FDCANx_FORCE_RESET(canindex)        __HAL_RCC_FDCAN_FORCE_RESET()
#define FDCANx_RELEASE_RESET(canindex)      __HAL_RCC_FDCAN_RELEASE_RESET()
#define FDCANx_TX_PIN(canindex)             ((canindex==0) ? GPIO_PIN_13 : GPIO_PIN_13)
#define FDCANx_TX_GPIO_PORT(canindex)       ((canindex==0) ? GPIOH : GPIOB)
#define FDCANx_TX_AF(canindex)              ((canindex==0) ? GPIO_AF9_FDCAN1 : GPIO_AF9_FDCAN2)
#define FDCANx_RX_PIN(canindex)             ((canindex==0) ? GPIO_PIN_14 : GPIO_PIN_5)
#define FDCANx_RX_GPIO_PORT(canindex)       ((canindex==0) ? GPIOH : GPIOB)
#define FDCANx_RX_AF(canindex)              ((canindex==0) ? GPIO_AF9_FDCAN1 : GPIO_AF9_FDCAN2)
#define FDCANx_IRQn(canindex)               ((canindex==0) ? FDCAN1_IT0_IRQn : FDCAN2_IT0_IRQn)
#define FDCANx_IRQHandler(canindex)         ((canindex==0) ? FDCAN1_IT0_IRQHandler : FDCAN2_IT0_IRQHandler)

#elif defined(STM32U5)

#define FDCAN_TYPE_U
#define FDCANx(canindex)                    FDCAN1
#define FDCANx_CLK_ENABLE(canindex)         __HAL_RCC_FDCAN1_CLK_ENABLE()
#define FDCANx_RX_GPIO_CLK_ENABLE(canindex) __HAL_RCC_GPIOH_CLK_ENABLE()
#define FDCANx_TX_GPIO_CLK_ENABLE(canindex) __HAL_RCC_GPIOH_CLK_ENABLE()
#define FDCANx_FORCE_RESET(canindex)        __HAL_RCC_FDCAN1_FORCE_RESET()
#define FDCANx_RELEASE_RESET(canindex)      __HAL_RCC_FDCAN1_RELEASE_RESET()
#define FDCANx_TX_PIN(canindex)             GPIO_PIN_13
#define FDCANx_TX_GPIO_PORT(canindex)       GPIOH
#define FDCANx_TX_AF(canindex)              GPIO_AF9_FDCAN1
#define FDCANx_RX_PIN(canindex)             GPIO_PIN_14
#define FDCANx_RX_GPIO_PORT(canindex)       GPIOH
#define FDCANx_RX_AF(canindex)              GPIO_AF9_FDCAN1
#define FDCANx_IRQn(canindex)               FDCAN1_IT0_IRQn
#define FDCANx_IRQHandler(canindex)         FDCAN1_IT0_IRQHandler

#endif


#if (PCLK == 32)
// CAN bus timing for PCLK = 32MHz
// Values in order are:
//               (SJW-1),      (BS2-1),      (BS1-1),   PRESCALER - 1
#define CAN_1000 ((3UL << 24) | (3UL << 20) | (10UL << 16) | (  2 - 1))  // 1000kbps, 75% sample point
#define CAN_800  ((3UL << 24) | (3UL << 20) | (14UL << 16) | (  2 - 1)) // 800kbps, 80% sample point
#define CAN_500  ((1UL << 24) | (1UL << 20) | (12UL << 16) | (  4 - 1)) // 500kbps, 85.7% sample point
#define CAN_250  ((1UL << 24) | (1UL << 20) | (12UL << 16) | (  8 - 1)) // 250kbps, 87.5% sample point
#define CAN_125  ((1UL << 24) | (1UL << 20) | (12UL << 16) | ( 16 - 1)) // 125kbps, 87.5% sample point
#define CAN_50   ((1UL << 24) | (1UL << 20) | (12UL << 16) | ( 40 - 1)) // 50kbps, 87,5% sample point
// Not all CAN bitrates possible with default clock - need to adjust for low bitrates

#elif (PCLK == 36)
// CAN bus timing for PCLK = 36MHz
// Values in order are:
//               (SJW-1),      (BS2-1),      (BS1-1),   PRESCALER - 1
#define CAN_1000 ((1UL << 24) | (2UL << 20) | ( 4UL << 16) | (  4 - 1))  // 1000kbps, 67% sample point
#define CAN_500  ((0UL << 24) | (0UL << 20) | ( 5UL << 16) | (  9 - 1)) // 500kbps, 87.5% sample point
#define CAN_250  ((1UL << 24) | (1UL << 20) | (12UL << 16) | (  9 - 1)) // 250kbps, 87.5% sample point
#define CAN_125  ((1UL << 24) | (1UL << 20) | (12UL << 16) | ( 18 - 1)) // 125kbps, 87.5% sample point
#define CAN_50   ((1UL << 24) | (1UL << 20) | (12UL << 16) | ( 45 - 1)) // 50kbps, 87,5% sample point
#define CAN_20   ((1UL << 24) | (0UL << 20) | ( 5UL << 16) | (225 - 1)) // 20kbps, 87,5% sample point
#define CAN_10   ((1UL << 24) | (0UL << 20) | ( 5UL << 16) | (450 - 1)) // 10kbps, 87,5% sample point

#elif (PCLK == 40) || (PCLK == 80)
// CAN FD controller, no simple bit rate definitions possible anymore

#elif (PCLK == 42)
// CAN bus timing for PCLK = 42MHz
// Values in order are:
//               (SJW-1),      (BS2-1),      (BS1-1),   PRESCALER - 1
#define CAN_1000 ((2UL << 24) | (2UL << 20) | ( 9UL << 16) | (  3 - 1))  // 1000kbps, 78.6% sample point
#define CAN_500  ((1UL << 24) | (1UL << 20) | (10UL << 16) | (  6 - 1)) // 500kbps, 85.7% sample point
#define CAN_250  ((0UL << 24) | (0UL << 20) | ( 5UL << 16) | ( 14 - 1)) // 250kbps, 87.5% sample point
#define CAN_125  ((1UL << 24) | (1UL << 20) | (12UL << 16) | ( 21 - 1)) // 125kbps, 87.5% sample point
#define CAN_50   ((1UL << 24) | (1UL << 20) | (11UL << 16) | ( 56 - 1)) // 50kbps, 86,7% sample point
// Not all CAN bitrates possible with default clock - need to adjust for low bitrates and/or 800 kbps

#elif (PCLK == 45)
// CAN bus timing for PCLK = 45MHz
// Values in order are:
//               (SJW-1),      (BS2-1),      (BS1-1),   PRESCALER - 1
#define CAN_1000 ((2UL << 24) | (2UL << 20) | ( 10UL << 16) | (  3 - 1))  // 1000kbps, 80.0% sample point
#define CAN_500  0
#define CAN_250  0
#define CAN_125  0
#define CAN_50   0

#elif (PCLK == 48)
// CAN bus timing for PCLK = 48MHz
// Values in order are:
//               (SJW-1),      (BS2-1),      (BS1-1),   PRESCALER - 1
#define CAN_1000 ((3UL << 24) | (3UL << 20) | (10UL << 16) | (  3 - 1))  // 1000kbps, 75% sample point
#define CAN_800  ((3UL << 24) | (3UL << 20) | (14UL << 16) | (  3 - 1)) // 800kbps, 80% sample point
#define CAN_500  ((1UL << 24) | (1UL << 20) | (12UL << 16) | (  6 - 1)) // 500kbps, 85.7% sample point
#define CAN_250  ((1UL << 24) | (1UL  << 20) | (12UL << 16) | ( 12 - 1)) // 250kbps, 87.5% sample point
#define CAN_125  ((1UL << 24) | (1UL << 20) | (12UL << 16) | ( 24 - 1)) // 125kbps, 87.5% sample point
#define CAN_50   ((1UL << 24) | (1UL << 20) | (12UL << 16) | ( 60 - 1)) // 50kbps, 87,5% sample point
// Not all CAN bitrates possible with default clock - need to adjust for low bitrates

#else
#error PCLK not supported
#endif



/**************************************************************************
DEFINES: Address of unused RAM for NVOL simulation
**************************************************************************/
#if !defined(SRAM_BASE)
#if defined(D1_DTCMRAM_BASE)
#define SRAM_BASE D1_DTCMRAM_BASE
#else
#error Define SRAM_BASE to use with TEMPORARY NVOL simulation. Never use NVOL simulation in production code!
#endif
#endif

#if !defined(SRAM_SIZE)
#define SRAM_SIZE  0x1800 // Depends on used derivative, unfortunately, HAL doesn't have a macro for this
#endif

#define NVOL_SIM_RAM_START (SRAM_BASE+SRAM_SIZE-NVOL_STORE_SIZE)


/**************************************************************************
DEFINES: Timer tick (ms)
**************************************************************************/
#define TIMER_TICK  1


/**************************************************************************
MACROS: Memory copy and compare to use
**************************************************************************/
#include "string.h"

#define MEM_CPY_FAR(d,s,l) memcpy((void *)d,(void *)s,l)
#define MEM_CPY memcpy
#define MEM_CMP memcmp


/**************************************************************************
MACROS: RTOS SLEEP FUNCTION
**************************************************************************/

// If used in RTOS, allow other tasks to run in blocking function calls
// Only define RTOS_SLEEP when an RTOS is used!
// #define RTOS_SLEEP Sleep(3);
#define RTOS_LOCK_PI(access,area)
#define RTOS_UNLOCK_PI(access,area)
#define RESOURCE_LOCK(resource)
#define RESOURCE_UNLOCK(resource)

// The maximum real-time delay this system has, in milliseconds
// (from receiving a CAN request to transmitting a CAN response)
#define MY_REALTIME_DELAY 10


/**************************************************************************
DEFINES: BASIC CAN COMMUNICATION
Modify these for your application
**************************************************************************/

// Default CAN bitrate — driven by CAN_BITRATE_xxxK in main.h
#if defined(CAN_BITRATE_1000K)
  #define CAN_BITRATE 1000
#elif defined(CAN_BITRATE_800K)
  #define CAN_BITRATE 800
#elif defined(CAN_BITRATE_500K)
#define CAN_BITRATE 500
#elif defined(CAN_BITRATE_250K)
  #define CAN_BITRATE 250
#elif defined(CAN_BITRATE_125K)
  #define CAN_BITRATE 125
#elif defined(CAN_BITRATE_50K)
  #define CAN_BITRATE 50
#elif defined(CAN_BITRATE_20K)
  #define CAN_BITRATE 20
#else
  #define CAN_BITRATE 1000  // fallback default
#endif
#define CAN_BRS_BITRATE 2000  // CAN FD data bitrate, make ==CAN_BITRATE to disable BRS

// Use CAN SW Filtering
#if !defined(USE_CANSWFILTER)
#define USE_CANSWFILTER 1
#endif

// use 29-bit CAN message for LSS feedback
#define USE_29BIT_LSSFEEDBACK 0


/**************************************************************************
DEFINES: CAN HARDWARE DRIVER DEFINITIONS
**************************************************************************/

// if defined a SW FIFO for CAN receive and transmit is used
#if !defined(USE_CANFIFO)
#define USE_CANFIFO 1
#endif

// Tx FIFO depth (must be 0, 4, 8, 16 or 32)
#define TXFIFOSIZE 32

// Rx FIFO depth (must be 0, 4, 8, 16 or 32)
#define RXFIFOSIZE 8

// Manager Rx FIFO depth (must be 0, 4, 8, 16 or 32)
#define MGRFIFOSIZE 0


/**************************************************************************
DEFINES: BOOTLOADER SUPPORT
**************************************************************************/
// If enabled, supports the ESAcademy CANopen bootloader, copy values
// from bootloader configuration
//#define REBOOT_FLAG_ADR     0x40007FFCUL
//#define REBOOT_FLAG         *(volatile uint32_t *)REBOOT_FLAG_ADR
//#define REBOOT_BOOTLOAD_VAL 0x21654387


#endif // _MCOHW_CFG_H

/*----------------------- END OF FILE ----------------------------------*/
