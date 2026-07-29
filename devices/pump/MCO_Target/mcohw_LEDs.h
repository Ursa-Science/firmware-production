/**************************************************************************
MODULE:    MCOHW_LED
CONTAINS:  Macro definition for LED switching
COPYRIGHT: Embedded Systems Academy (EmSA) 2002-2024
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
#ifndef _LED_H
#define _LED_H

#include "stm32g4xx_hal.h" // Include HAL header for GPIO/TIM definitions

// External timer handles for tri-LED PWM control (defined in main.c)
extern TIM_HandleTypeDef htim2;  // Red LED - TIM2 CH1
extern TIM_HandleTypeDef htim3;  // Blue LED - TIM3 CH2
extern TIM_HandleTypeDef htim4;  // Green LED - TIM4 CH1

/* -------------------------------------------------------------------------
 * LED ownership gate (resource-arbitration token)
 * -------------------------------------------------------------------------
 * The tri-LED pins are shared between the MCO stack's CiA-303-3 RUN/ERR
 * indicators (driven via the LED_xxx_ON/OFF macros below) and the application
 * LED module (led_control.c, controlled over OD 0x2000).
 *
 * When the application takes manual control of the LED it asserts this flag via
 * MCOTGT_SetLEDOverride(1); the stack's macro writes then become no-ops so they
 * cannot contend with / mask the application's indication. On release (flag = 0)
 * the next stack LED tick repaints RUN/ERR automatically.
 *
 * The LED hardware layer intentionally stays ignorant of *why* it is suppressed
 * (no dependency on application object-dictionary indices) — it only checks
 * ownership. Direction of dependency is app -> target, as it should be.
 */
extern volatile uint8_t gLEDAppOverride; // 0 = stack owns LEDs, non-0 = app owns LEDs
void MCOTGT_SetLEDOverride(uint8_t active); // set/clear app LED ownership (defined in user_STM32.c)

#define MCO_LED_STACK_OWNS  (gLEDAppOverride == 0)

#if USE_LEDS

#ifdef __SIMULATION__

#define LED_RUN_ON SimDriver_UpdateLEDState(SIMDRV_RUNLED, 1)
#define LED_RUN_OFF SimDriver_UpdateLEDState(SIMDRV_RUNLED, 0)
#define LED_ERR_ON SimDriver_UpdateLEDState(SIMDRV_ERRLED, 1)
#define LED_ERR_OFF SimDriver_UpdateLEDState(SIMDRV_ERRLED, 0)

#else

#ifdef USE_CUBEMX

// Timer-based PWM LED control for tri-LED (common anode)
// Common-anode inverted logic: CCR=0 for ON, CCR=Period for OFF

// LED_RUN = Green LED (TIM4 CH1) — suppressed while the app owns the LED (see MCO_LED_STACK_OWNS)
#define LED_RUN_ON  { if (MCO_LED_STACK_OWNS) { __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 0); TIM4->EGR |= TIM_EGR_UG; } }
#define LED_RUN_OFF { if (MCO_LED_STACK_OWNS) { __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, htim4.Init.Period); TIM4->EGR |= TIM_EGR_UG; } }

// LED_ERR = Red LED (TIM2 CH1) — suppressed while the app owns the LED (see MCO_LED_STACK_OWNS)
#define LED_ERR_ON  { if (MCO_LED_STACK_OWNS) { __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0); TIM2->EGR |= TIM_EGR_UG; } }
#define LED_ERR_OFF { if (MCO_LED_STACK_OWNS) { __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, htim2.Init.Period); TIM2->EGR |= TIM_EGR_UG; } }

#else
#error No functional LED switch macros defined!
#define LED_RUN_ON  {;}
#define LED_RUN_OFF {;}
#define LED_ERR_ON  {;}
#define LED_ERR_OFF {;}
#endif // USE_CUBEMX

#endif // __SIMULATION__

#endif // USE_LEDS

//#endif // _LED_H

//#ifndef _LED_H
//#define _LED_H

//#if USE_LEDS

//#ifdef __SIMULATION__

//#define LED_RUN_ON SimDriver_UpdateLEDState(SIMDRV_RUNLED, 1)
//#define LED_RUN_OFF SimDriver_UpdateLEDState(SIMDRV_RUNLED, 0)
//#define LED_ERR_ON SimDriver_UpdateLEDState(SIMDRV_ERRLED, 1)
//#define LED_ERR_OFF SimDriver_UpdateLEDState(SIMDRV_ERRLED, 0)

//#else

//#ifdef USE_CUBEMX
//#define LED_RUN_ON  {HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_SET);}
//#define LED_RUN_OFF {HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_RESET);}
//#define LED_ERR_ON  {HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_SET);}
//#define LED_ERR_OFF {HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_RESET);}
//#else
//#error No functional LED switch macros defined!
//#define LED_RUN_ON  {;}
//#define LED_RUN_OFF {;}
//#define LED_ERR_ON  {;}
//#define LED_ERR_OFF {;}
//#endif

//#endif // __SIMULATION__

//#endif // USE_LEDS

#endif // _LED_H
