/**
 ******************************************************************************
 * @file    ph_sensor.h
 * @brief   MCP3221 ADC + Nernst pH sensor driver (non-blocking)
 * @note    Reads MCP3221A5T 12-bit ADC via I2C, applies temperature-
 *          compensated Nernst equation, multi-point calibration, and
 *          rolling-window signal quality.
 *
 *          Non-blocking usage pattern:
 *            pH_Init(&hi2c1)
 *            loop {
 *                pH_Process()    // handles sampling + Nernst internally
 *            }
 *
 *          Internally cycles through:
 *            IDLE → SAMPLING (100ms interval, 4-sample avg) → READY → ...
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
#define PH_VREF                 2.048f      /**< MCP1501T-20E precision reference (V) */

/** pH defaults */
#define PH_NEUTRAL_VOLTAGE      1.024f      /**< Voltage at pH 7 (Vref/2) */
#define PH_DEFAULT_SLOPE        1.0f        /**< Uncalibrated slope */
#define PH_DEFAULT_OFFSET       0.0f        /**< Uncalibrated offset */
#define PH_DEFAULT_TEMP         25.0f       /**< Default temperature (°C) */

/** Sampling parameters */
#define PH_SAMPLE_INTERVAL_MS   100u        /**< 10 Hz sampling rate */
#define PH_AVG_COUNT            4u          /**< Samples per averaged reading */
#define PH_QUALITY_WINDOW       8u          /**< Rolling window for quality */

/** Nernst equation constants */
#define PH_R_CONSTANT           8.314f      /**< Universal gas constant (J/mol·K) */
#define PH_F_CONSTANT           96485.0f    /**< Faraday constant (C/mol) */
#define PH_LN10                 2.302585f   /**< ln(10) */

/** MCP4017T-103E/LT I2C digital rheostat (PH Module v1.0 gain control) */
#define PH_MCP4017_ADDR         (0x2E << 1) /**< HAL-format I2C address */
#define PH_MCP4017_STEPS        128u        /**< 7-bit: 0–127 wiper positions */
#define PH_MCP4017_RAB          10000u      /**< Full-scale resistance (10kΩ) */
#define PH_MCP4017_RW           100u        /**< Wiper resistance (100Ω typical) */
#define PH_MCP4017_DEFAULT_WIPER 64u        /**< Midscale default (≈5kΩ, G≈0.5) */
#define PH_R_INPUT              10000.0f    /**< R4 = 10kΩ (fixed input resistor) */

/** Calibration command values (from 0x2200 CalibrationCommand) */
#define PH_CAL_CMD_NONE         0x00u       /**< No command */
#define PH_CAL_CMD_OFFSET       0x01u       /**< Offset cal (zero-gain method) */
#define PH_CAL_CMD_PH7          0x02u       /**< Single-point pH 7.00 */
#define PH_CAL_CMD_PH4          0x03u       /**< Buffer pH 4.01 */
#define PH_CAL_CMD_PH10         0x04u       /**< Buffer pH 10.01 */
#define PH_CAL_CMD_3POINT       0x05u       /**< Three-point piecewise cal (pH4+7+10) */
#define PH_CAL_CMD_RESET        0xFFu       /**< Reset to factory defaults */

/** Calibration status values (written to 0x2201) */
#define PH_CAL_STATUS_IDLE      0x00u
#define PH_CAL_STATUS_BUSY      0x01u
#define PH_CAL_STATUS_OK        0x02u
#define PH_CAL_STATUS_FAIL      0x03u

/** pH value limits */
#define PH_VALUE_MIN            0u          /**< pH × 100 minimum */
#define PH_VALUE_MAX            1400u       /**< pH × 100 maximum (14.00) */

/** Error recovery */
#define PH_MAX_I2C_RETRIES      3u          /**< Consecutive I2C failures → ERROR */

/** Calibration settling gate: max mV delta between cal samples and main-loop
 *  voltage. If pH_Calibrate()'s fresh ADC average differs from ph_ctx.voltage
 *  by more than this, the probe is in transition and calibration is rejected.
 *  50 mV ≈ 100 ADC counts — well above noise (±2.5 mV), well below any real
 *  probe transition (CAN trace showed 350 mV jump on emulator switch). */
#define PH_CAL_SETTLE_MV       50u

/** Electrode status bitfield (0x2220 pHElectrodeStatus) — 1 = OK, 0 = fault */
#define ELEC_CONNECTED          (1u << 0)   /**< Probe voltage in valid range     */
#define ELEC_SIGNAL_OK          (1u << 1)   /**< Signal quality >= 50%            */
#define ELEC_CALIBRATED         (1u << 2)   /**< Calibration active (not NONE)    */
#define ELEC_SLOPE_OK           (1u << 3)   /**< Cal segments symmetric (±30%)    */
#define ELEC_OFFSET_OK          (1u << 4)   /**< Neutral voltage near Vref/2      */
#define ELEC_RESPONDING         (1u << 5)   /**< I2C communication OK             */

/*============================================================================*/
/*                            TYPES                                           */
/*============================================================================*/

/** pH sensor driver states */
typedef enum {
	PH_STATE_IDLE = 0, /**< Not started or between cycles          */
	PH_STATE_SAMPLING, /**< Accumulating samples (4 per average)   */
	PH_STATE_READY, /**< Valid pH reading available              */
	PH_STATE_CALIBRATING, /**< Calibration in progress                */
	PH_STATE_ERROR /**< I2C failure or ADC not responding      */
} pH_State_t;

/** Active calibration mode — determines which correction path is used */
typedef enum {
	PH_CAL_MODE_NONE = 0, /**< Factory defaults (no calibration)     */
	PH_CAL_MODE_1POINT, /**< Single-point (pH 7 offset only)       */
	PH_CAL_MODE_2POINT, /**< Two-point (single slope + offset)     */
	PH_CAL_MODE_3POINT /**< Three-point piecewise (two segments)  */
} pH_CalMode_t;

/*============================================================================*/
/*                          PUBLIC API                                        */
/*============================================================================*/

/**
 * @brief  Initialize pH sensor driver
 * @param  hi2c  Pointer to I2C handle (I2C1)
 * @retval HAL_OK on success, HAL_ERROR if MCP3221 not responding
 */
HAL_StatusTypeDef pH_Init(I2C_HandleTypeDef *hi2c);

/**
 * @brief  Non-blocking process function — call from main loop
 * @note   Manages the full sampling cycle:
 *         IDLE → read ADC every 100ms → average 4 samples → compute pH → READY
 *         Automatically restarts sampling after each averaged reading.
 */
void pH_Process(void);

/**
 * @brief  Get current driver state
 * @retval pH_State_t
 */
pH_State_t pH_GetState(void);

/**
 * @brief  Get last valid pH reading
 * @retval pH value × 100 (e.g. 700 = pH 7.00)
 * @note   Returns 700 (pH 7.00) if no valid reading yet.
 *         Clamped to 0.00–14.00 range.
 */
uint16_t pH_GetValue(void);

/**
 * @brief  Get raw ADC value from last reading
 * @retval 12-bit ADC value (0–4095)
 */
uint16_t pH_GetRawADC(void);

/**
 * @brief  Get voltage from last reading
 * @retval Voltage in mV (e.g. 1650 = 1.650V)
 */
uint16_t pH_GetMillivolts(void);

/**
 * @brief  Get signal quality indicator
 * @retval 0–100%:
 *         100 = ADC stable (stddev < 5 counts in rolling window)
 *         50  = ADC noisy (stddev 5–20 counts)
 *         0   = ADC not responding or wildly unstable
 */
uint8_t pH_GetSignalQuality(void);

/**
 * @brief  Set temperature for Nernst compensation
 * @param  temp_c10  Temperature in °C × 10 (e.g. 235 = 23.5°C)
 * @note   Feed from DS18B20 via Temp_GetValue().
 *         Default 25.0°C if never set.
 */
void pH_SetTemperature(int16_t temp_c10);

/**
 * @brief  Execute calibration command
 * @param  command  One of PH_CAL_CMD_* constants
 * @retval HAL_OK if calibration started/completed, HAL_ERROR on failure
 * @note   Commands:
 *         OFFSET: Short BNC → sets neutral voltage to current reading
 *         PH7:    Probe in pH 7 buffer → single-point calibration
 *         PH4:    Probe in pH 4 buffer → stores ref, two-point if pH7 exists
 *         PH10:   Probe in pH 10 buffer → stores ref, two-point if pH7 exists
 *         3POINT: Compute piecewise cal from stored pH4+7+10 references
 *         RESET:  Restore factory defaults
 */
HAL_StatusTypeDef pH_Calibrate(uint8_t command);

/**
 * @brief  Get active calibration mode
 * @retval pH_CalMode_t (NONE, 1POINT, 2POINT, or 3POINT)
 */
pH_CalMode_t pH_GetCalMode(void);

/**
 * @brief  Get electrode status bitfield (for OD 0x2220)
 * @retval Bitfield: ELEC_CONNECTED | ELEC_SIGNAL_OK | ELEC_CALIBRATED |
 *         ELEC_SLOPE_OK | ELEC_OFFSET_OK | ELEC_RESPONDING
 * @note   All bits 1 = healthy (0x3F). Disconnected probe = 0x00.
 */
uint8_t pH_GetElectrodeStatus(void);

/**
 * @brief  Clear error state and return to IDLE
 * @note   Next pH_Process() call will attempt new sampling.
 */
void pH_ClearError(void);

#ifdef __cplusplus
}
#endif

#endif /* PH_SENSOR_H */
