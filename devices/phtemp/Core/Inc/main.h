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
#define DQ_Pin GPIO_PIN_6
#define DQ_GPIO_Port GPIOA

/* USER CODE BEGIN Private defines */
/* -- CAN bus bitrate ---------------------------------------------------
 * Set by the build, NOT here: CMake passes exactly one CAN_BITRATE_xxxK from
 * the URSA_CAN_BITRATE cache variable (default 250; see CMAKE_GUIDE.md 3.10).
 * The symbol is read by MX_FDCAN1_Init() (main.c) and by mcohw_cfg.h, which
 * turns it into CAN_BITRATE for the MCO stack. Timing assumes the 80 MHz
 * FDCAN kernel clock (PLLQ).
 *
 * CANopen Architect's bitrate selector has no effect on this firmware: it only
 * sets CAN_BITRATE_DCF in stackinit.h (unused here) and BaudRate= in the DCF.
 *
 * The fallback below only serves builds that bypass CMake (IDE indexers).
 * --------------------------------------------------------------------- */
#if !defined(CAN_BITRATE_1000K) && !defined(CAN_BITRATE_800K) && \
    !defined(CAN_BITRATE_500K)  && !defined(CAN_BITRATE_250K) && \
    !defined(CAN_BITRATE_125K)  && !defined(CAN_BITRATE_50K)  && \
    !defined(CAN_BITRATE_20K)
#define CAN_BITRATE_250K
#endif

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
