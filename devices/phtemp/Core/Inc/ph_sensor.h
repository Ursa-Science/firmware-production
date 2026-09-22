/**
 ******************************************************************************
 * @file    ph_sensor.h
 * @brief   MCP3221 ADC electrode driver — raw millivolts only (non-blocking)
 * @note    Reads the MCP3221A5T 12-bit ADC via I2C and reports the electrode
 *          voltage in millivolts plus a rolling-window signal-quality metric.
 *
 *          "Dumb module" contract (see docs/PHTEMP_REFACTOR_PLAN.md): there is
 *          NO pH/Nernst/calibration on-module. The reported value is the
 *          ADC-referred voltage in **mV × 10** (0-20480, Vref 2.048 V, 0.1 mV
 *          resolution — same ×10 convention as Temperature) AFTER the MikroE
 *          pH-2 Click analog front-end (gain G + Vref/2 bias, ~10240 at
 *          pH 7). The MIK owns the Nernst equation, temperature compensation,
 *          multi-point calibration, and all persistence.
 *
 *          Non-blocking usage pattern:
 *            pH_Init(&hi2c1)
 *            loop {
 *                pH_Process()    // handles sampling internally
 *            }
 *
 *          Internally cycles through:
 *            IDLE -> SAMPLING (100ms interval, 4-sample avg) -> READY -> ...
 *
 * @date    2026
 ******************************************************************************
 */

#ifndef PH_SENSOR_H
#define PH_SENSOR_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include "stm32g4xx_hal.h"

/*============================================================================*/
/*                            CONSTANTS                                       */
/*============================================================================*/

/** MCP3221A5T I2C address (7-bit: 0x4D, shifted for HAL) */
#define PH_MCP3221_ADDR         (0x4D << 1)

/** ADC parameters */
#define PH_ADC_RESOLUTION       4096u       /**< 12-bit ADC */
#define PH_ADC_MASK             0x0FFFu     /**< 12-bit mask */
#define PH_VREF_MV              2048u       /**< MCP1501T-20E precision reference (mV); 1 count = 0.5 mV */

/** Sampling parameters */
#define PH_SAMPLE_INTERVAL_MS   100u        /**< 10 Hz sampling rate */
#define PH_AVG_COUNT            4u          /**< Samples per averaged reading */
#define PH_QUALITY_WINDOW       8u          /**< Rolling window for quality */

/** Error recovery */
#define PH_MAX_I2C_RETRIES      3u          /**< Consecutive I2C failures -> ERROR */

/*============================================================================*/
/*                            TYPES                                           */
/*============================================================================*/

/** Electrode ADC driver states */
typedef enum {
	PH_STATE_IDLE = 0, /**< Not started or between cycles          */
	PH_STATE_SAMPLING, /**< Accumulating samples (4 per average)   */
	PH_STATE_READY, /**< Valid millivolt reading available       */
	PH_STATE_ERROR /**< I2C failure or ADC not responding      */
} pH_State_t;

/*============================================================================*/
/*                          PUBLIC API                                        */
/*============================================================================*/

/**
 * @brief  Initialize electrode ADC driver
 * @param  hi2c  Pointer to I2C handle (I2C1)
 * @retval HAL_OK on success, HAL_ERROR if MCP3221 not responding
 */
HAL_StatusTypeDef pH_Init(I2C_HandleTypeDef *hi2c);

/**
 * @brief  Non-blocking process function — call from main loop
 * @note   Manages the full sampling cycle:
 *         IDLE -> read ADC every 100ms -> average 4 samples -> mV×10 -> READY
 *         Automatically restarts sampling after each averaged reading.
 */
void pH_Process(void);

/**
 * @brief  Get current driver state
 * @retval pH_State_t
 */
pH_State_t pH_GetState(void);

/**
 * @brief  Get raw ADC value from last reading
 * @retval 12-bit ADC value (0-4095)
 */
uint16_t pH_GetRawADC(void);

/**
 * @brief  Get electrode voltage from last reading
 * @retval Voltage in mV × 10 (e.g. 16500 = 1650.0 mV), ADC-referred (0-20480)
 * @note   This is the primary output (OD 0x6003). The MIK converts mV -> pH.
 *         0.1 mV resolution; the 4-sample average's fractional counts are kept.
 */
uint16_t pH_GetMillivoltsX10(void);

/**
 * @brief  Get signal quality indicator
 * @retval 0-100%:
 *         100 = ADC stable (stddev < 5 counts in rolling window)
 *         50  = ADC noisy (stddev 5-20 counts)
 *         0   = ADC not responding or wildly unstable
 */
uint8_t pH_GetSignalQuality(void);

/**
 * @brief  Clear error state and return to IDLE
 * @note   Next pH_Process() call will attempt new sampling.
 */
void pH_ClearError(void);

#ifdef __cplusplus
}
#endif

#endif /* PH_SENSOR_H */
