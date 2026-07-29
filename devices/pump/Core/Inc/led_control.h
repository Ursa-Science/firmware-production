/**
 ******************************************************************************
 * @file    led_control.h
 * @brief   LED Control via CANopen (0x2000)
 * @author  CANopen Motor Control Project
 * @date    2026
 ******************************************************************************
 */

#ifndef LED_CONTROL_H
#define LED_CONTROL_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include <stdbool.h>
#include "main.h"

/* Exported types ------------------------------------------------------------*/

/**
 * @brief LED Control Modes (0x2000)
 */
typedef enum {
	LED_MODE_NORMAL = 0, // CANopen heartbeat indicator (flicker on CAN activity)
	LED_MODE_BLINK = 1, // Standard blink (500ms)
	LED_MODE_ON = 2, // Solid ON
	LED_MODE_OFF = 3 // Solid OFF
} LED_Mode_t;

/* Exported functions --------------------------------------------------------*/

/**
 * @brief Initialize LED control with injected timer handles
 * @param red_htim   Timer handle for Red LED PWM
 * @param red_ch     Timer channel for Red LED (e.g. TIM_CHANNEL_1)
 * @param blue_htim  Timer handle for Blue LED PWM
 * @param blue_ch    Timer channel for Blue LED (e.g. TIM_CHANNEL_2)
 * @param green_htim Timer handle for Green LED PWM
 * @param green_ch   Timer channel for Green LED (e.g. TIM_CHANNEL_1)
 * @retval true if successful
 */
bool LEDControl_Init(TIM_HandleTypeDef *red_htim, uint32_t red_ch,
		TIM_HandleTypeDef *blue_htim, uint32_t blue_ch,
		TIM_HandleTypeDef *green_htim, uint32_t green_ch);

/**
 * @brief Process LED control (call from main loop)
 * @note Reads mode from CANopen process image (0x2000)
 */
void LEDControl_Process(void);

/**
 * @brief Set LED mode directly (bypasses process image)
 * @param mode LED mode
 */
void LEDControl_SetMode(LED_Mode_t mode);

/**
 * @brief Indicate CAN activity (for Normal mode)
 * @note Call this when CAN messages are received/transmitted
 */
void LEDControl_CANActivity(void);

#ifdef __cplusplus
}
#endif

#endif /* LED_CONTROL_H */
