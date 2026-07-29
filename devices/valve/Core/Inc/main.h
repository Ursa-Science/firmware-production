/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#define Relay_Signal_Pin GPIO_PIN_10
#define Relay_Signal_GPIO_Port GPIOA

/* USER CODE BEGIN Private defines */

/* ── FDCAN Bitrate Selector ──────────────────────────────────────────
 * Uncomment ONE line to select CAN bus bitrate.
 * This define is read by mcohw_cfg.h to set CAN_BITRATE for the MCO
 * stack, and by MX_FDCAN1_Init() for the CubeMX HAL call.
 * ─────────────────────────────────────────────────────────────────── */
// #define CAN_BITRATE_1000K
//#define CAN_BITRATE_500K
// #define CAN_BITRATE_800K
#define CAN_BITRATE_250K
// #define CAN_BITRATE_125K
// #define CAN_BITRATE_50K
// #define CAN_BITRATE_20K

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
