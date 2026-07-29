/**************************************************************************
 MODULE:    LED_CONTROL
 CONTAINS:  Blue status LED control (TIM3_CH2 / PA4)

 The tri-colour LED is common-anode (active-low):
 Red   = TIM2_CH1 (PA0) — driven by MCO stack (LED_ERR, CiA 303-3)
 Green = TIM4_CH1 (PB6) — driven by MCO stack (LED_RUN, CiA 303-3)
 Blue  = TIM3_CH2 (PA4) — driven HERE for valve-specific status

 LED modes (OD 0x2000 values):
 0 = Normal  — blue LED maps to valve state
 1 = Blink   — blue LED blinks at 1 Hz
 2 = On      — blue LED solid on
 3 = Off     — blue LED off

 Normal-mode valve-state mapping (blue channel only):
 DISABLED        — off
 IDLE / CLOSED   — off
 IDLE / OPEN     — solid on
 OPENING/CLOSING — slow blink (0.5 Hz)
 FAULT           — fast blink (2 Hz)
 ***************************************************************************/

#ifndef _LED_CONTROL_H
#define _LED_CONTROL_H

#include <stdint.h>

/**************************************************************************
 DEFINES: LED mode values (match OD 0x2000)
 ***************************************************************************/
#define LED_MODE_NORMAL  0
#define LED_MODE_BLINK   1
#define LED_MODE_ON      2
#define LED_MODE_OFF     3

/**************************************************************************
 GLOBAL FUNCTIONS
 ***************************************************************************/

/**
 * @brief  Initialise LED control
 *         Starts PWM on all three channels and sets initial OFF state.
 *         TIM2/TIM4 are handed off to the MCO stack after start.
 */
void LEDControl_Init(void);

/**
 * @brief  Process blue status LED — call from main loop AFTER MCO_ProcessStack
 *         Reads OD 0x2000 and valve state, updates TIM3 duty cycle only.
 */
void LEDControl_Process(void);

#endif // _LED_CONTROL_H
