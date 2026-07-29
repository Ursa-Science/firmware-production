/**
 ******************************************************************************
 * @file    temp_sensor.h
 * @brief   DS18B20 1-Wire temperature sensor driver (non-blocking)
 * @note    Bit-bang 1-Wire on DQ pin (PB0, open-drain).
 *          Uses DWT cycle counter for microsecond-precision timing.
 *
 *          Non-blocking usage pattern:
 *            Temp_Init()
 *            loop {
 *                Temp_Process()  // handles state machine internally
 *            }
 *
 *          Internally cycles through:
 *            IDLE → CONVERTING (750ms) → READING → IDLE → ...
 *
 * @date    2026
 ******************************************************************************
 */

#ifndef TEMP_SENSOR_H
#define TEMP_SENSOR_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include "stm32g4xx_hal.h"

/*============================================================================*/
/*                            TYPES                                           */
/*============================================================================*/

/** DS18B20 driver states */
typedef enum {
	TEMP_STATE_IDLE = 0, /**< Not started or between cycles          */
	TEMP_STATE_CONVERTING, /**< Convert T issued, waiting 750ms        */
	TEMP_STATE_READING, /**< Reading scratchpad (transient state)    */
	TEMP_STATE_READY, /**< Valid reading available                 */
	TEMP_STATE_ERROR /**< No presence, CRC fail, or timeout      */
} Temp_State_t;

/*============================================================================*/
/*                          PUBLIC API                                        */
/*============================================================================*/

/**
 * @brief  Initialize DS18B20 driver
 * @param  port  GPIO port for DQ pin (e.g. GPIOB)
 * @param  pin   GPIO pin mask for DQ (e.g. GPIO_PIN_0)
 * @retval HAL_OK on success, HAL_ERROR if no device present
 */
HAL_StatusTypeDef Temp_Init(GPIO_TypeDef *port, uint16_t pin);

/**
 * @brief  Non-blocking process function — call from main loop
 * @note   Manages the full conversion cycle:
 *         IDLE → start conversion → wait 750ms → read scratchpad → READY
 *         Automatically restarts conversion after each successful read.
 */
void Temp_Process(void);

/**
 * @brief  Start a temperature conversion (manual trigger)
 * @retval HAL_OK if conversion started, HAL_ERROR if bus error
 * @note   Normally not needed — Temp_Process() auto-cycles.
 *         Use this for explicit control if needed.
 */
HAL_StatusTypeDef Temp_StartConversion(void);

/**
 * @brief  Check if conversion is complete (750ms elapsed)
 * @retval true if conversion done or not converting
 */
bool Temp_IsConversionDone(void);

/**
 * @brief  Read scratchpad and parse temperature
 * @retval HAL_OK on success, HAL_ERROR on CRC mismatch or bus error
 * @note   Only valid after Temp_IsConversionDone() returns true.
 */
HAL_StatusTypeDef Temp_ReadResult(void);

/**
 * @brief  Get current driver state
 * @retval Temp_State_t
 */
Temp_State_t Temp_GetState(void);

/**
 * @brief  Get last valid temperature reading
 * @retval Temperature in °C × 10 (e.g. 235 = 23.5°C)
 * @note   Returns default 250 (25.0°C) if no valid reading yet.
 */
int16_t Temp_GetValue(void);

/**
 * @brief  Get raw temperature (full 12-bit DS18B20 resolution)
 * @retval Raw 16-bit register value (LSB = 0.0625°C)
 */
int16_t Temp_GetRawValue(void);

/**
 * @brief  Get signal quality indicator
 * @retval 0-100:
 *         100 = device present + CRC valid
 *          50 = device present + CRC mismatch (reading used but flagged)
 *           0 = no device presence detected
 */
uint8_t Temp_GetSignalQuality(void);

/**
 * @brief  Set temperature offset correction
 * @param  offset  Offset in °C × 10 (e.g. -5 = -0.5°C correction)
 * @note   Applied to all subsequent readings.
 *         Written by CANopen master via object 0x2210.
 */
void Temp_SetOffset(int16_t offset);

/**
 * @brief  Clear error state and return to IDLE
 * @note   Next Temp_Process() call will attempt a new conversion.
 */
void Temp_ClearError(void);

#ifdef __cplusplus
}
#endif

#endif /* TEMP_SENSOR_H */
