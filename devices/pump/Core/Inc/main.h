/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32g4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
/* TMC2209 module DIAG output → MCU fault input (EXTI5, rising edge).
 * History: this pin was mislabeled "MS3" (A4988 era) then "SPREAD" — both
 * fictions. The PCB netlist (U8 pad 17 → net PA5) routes the BTT module's
 * DIAG here. DIAG drives HIGH on latched driver error (short-to-GND/VS,
 * overtemp), charge-pump undervoltage, and briefly at power-on reset. */
#define TMC_DIAG_Pin GPIO_PIN_5
#define TMC_DIAG_GPIO_Port GPIOA
#define MS2_Pin GPIO_PIN_6
#define MS2_GPIO_Port GPIOA
#define MS1_Pin GPIO_PIN_7
#define MS1_GPIO_Port GPIOA
#define STEP_Pin GPIO_PIN_8
#define STEP_GPIO_Port GPIOA
#define DIR_Pin GPIO_PIN_9
#define DIR_GPIO_Port GPIOA
#define ENABLE_Pin GPIO_PIN_10
#define ENABLE_GPIO_Port GPIOA

/* USER CODE BEGIN Private defines */

/*
 * FDCAN Nominal Bitrate Selection
 * --------------------------------
 * Uncomment ONE of the following to set the CAN bus bitrate.
 * All timing derived from 80 MHz FDCAN kernel clock (PLLQ).
 * This define lives in main.h (not main.c) so that both
 * MX_FDCAN1_Init and the MCO stack (mcohw_cfg.h) see it.
 */
// #define CAN_BITRATE_1000K
// #define CAN_BITRATE_800K
//#define CAN_BITRATE_500K
#define CAN_BITRATE_250K
// #define CAN_BITRATE_125K
// #define CAN_BITRATE_50K
// #define CAN_BITRATE_20K

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
