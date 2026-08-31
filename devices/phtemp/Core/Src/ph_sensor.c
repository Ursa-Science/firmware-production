/**
 ******************************************************************************
 * @file    ph_sensor.c
 * @brief   MCP3221 ADC electrode driver — raw millivolts only (non-blocking)
 * @note    Ported from ExampleCode/Src/ph.c and MikroE pH 2 Click driver.
 *
 *          MCP3221A5T: 12-bit I2C ADC, continuous conversion, 2-byte read.
 *          Data format: [byte0: 0000_D11..D8] [byte1: D7..D0]
 *            -> raw = ((byte0 << 8) | byte1) & 0x0FFF
 *
 *          Reports the ADC-referred electrode voltage in millivolts. There is
 *          NO pH/Nernst/calibration on-module — the MIK owns all of that (see
 *          docs/PHTEMP_REFACTOR_PLAN.md). This driver only acquires and
 *          averages the ADC and scores signal stability.
 *
 *          Sampling: 10 Hz (100ms), 4-sample average, 8-sample quality window.
 *
 * @date    2026
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "ph_sensor.h"
#include "log.h"
#include <string.h>

/* Private defines -----------------------------------------------------------*/

/** I2C timeout for single ADC read (ms) */
#define I2C_TIMEOUT_MS          50u

/** Signal quality thresholds (in ADC counts, applied to stddev) */
#define QUALITY_GOOD_STDDEV     5u      /**< stddev < 5 -> 100% */
#define QUALITY_FAIR_STDDEV     20u     /**< stddev 5-20 -> proportional 50-99% */

/** Squared thresholds for variance comparison (avoids sqrtf / libm dependency) */
#define QUALITY_GOOD_VAR        (QUALITY_GOOD_STDDEV * QUALITY_GOOD_STDDEV)   /* 25 */
#define QUALITY_FAIR_VAR        (QUALITY_FAIR_STDDEV * QUALITY_FAIR_STDDEV)   /* 400 */

/* Private types -------------------------------------------------------------*/

/** Driver context */
typedef struct {
	I2C_HandleTypeDef *hi2c;
	pH_State_t state;

	/* Sampling state */
	uint32_t last_sample_ms; /**< HAL_GetTick at last ADC read */
	uint16_t sample_buf[PH_AVG_COUNT]; /**< Raw ADC samples for averaging */
	uint8_t sample_idx; /**< Next slot in sample_buf */

	/* Averaged result */
	uint16_t raw_adc; /**< Last averaged raw ADC value */
	float voltage; /**< Last averaged voltage (V) */

	/* Signal quality (rolling window of averaged readings) */
	uint16_t quality_buf[PH_QUALITY_WINDOW]; /**< Averaged ADC values */
	uint8_t quality_idx; /**< Next slot in quality_buf */
	uint8_t quality_count; /**< Number of valid entries (up to WINDOW) */
	uint8_t signal_quality; /**< 0-100% */

	/* Error tracking */
	uint8_t i2c_error_count; /**< Consecutive I2C failures */
} pH_Context_t;

/* Private variables ---------------------------------------------------------*/
static pH_Context_t ph_ctx;

/* Private function prototypes -----------------------------------------------*/
static HAL_StatusTypeDef pH_ReadADC(uint16_t *raw);
static float pH_ADCToVoltage(uint16_t raw);
static void pH_UpdateSignalQuality(uint16_t adc_avg);

/* Exported functions --------------------------------------------------------*/

HAL_StatusTypeDef pH_Init(I2C_HandleTypeDef *hi2c) {
	memset(&ph_ctx, 0, sizeof(ph_ctx));

	ph_ctx.hi2c = hi2c;
	ph_ctx.state = PH_STATE_IDLE;

	/* Verify MCP3221 is present on I2C bus */
	HAL_StatusTypeDef status = HAL_I2C_IsDeviceReady(hi2c, PH_MCP3221_ADDR, 3,
			100);

	if (status != HAL_OK) {
		DBG_ERROR(PH, "Init: MCP3221 not found at 0x%02X", PH_MCP3221_ADDR);
		ph_ctx.state = PH_STATE_ERROR;
		return HAL_ERROR;
	}

	/* Do one test read to confirm data path */
	uint16_t test_raw;
	status = pH_ReadADC(&test_raw);
	if (status != HAL_OK) {
		DBG_ERROR(PH, "Init: MCP3221 test read failed");
		ph_ctx.state = PH_STATE_ERROR;
		return HAL_ERROR;
	}

	DBG_PRINT(PH, "Init: MCP3221 OK, test ADC=%u (%.3fV)", test_raw,
			pH_ADCToVoltage(test_raw));

	return HAL_OK;
}

void pH_Process(void) {
	if (ph_ctx.state == PH_STATE_ERROR) {
		return; /* Don't sample during error */
	}

	if (ph_ctx.hi2c == NULL) {
		return; /* Not initialized */
	}

	uint32_t now = HAL_GetTick();

	/* ── 10 Hz sampling gate ──────────────────────────────────── */
	if (now - ph_ctx.last_sample_ms < PH_SAMPLE_INTERVAL_MS) {
		return;
	}
	ph_ctx.last_sample_ms = now;

	/* ── Read one ADC sample ──────────────────────────────────── */
	uint16_t raw;
	HAL_StatusTypeDef status = pH_ReadADC(&raw);

	if (status != HAL_OK) {
		ph_ctx.i2c_error_count++;
		DBG_ERROR(PH, "I2C read fail (%u/%u)", ph_ctx.i2c_error_count,
				PH_MAX_I2C_RETRIES);

		if (ph_ctx.i2c_error_count >= PH_MAX_I2C_RETRIES) {
			ph_ctx.state = PH_STATE_ERROR;
			ph_ctx.signal_quality = 0;
			DBG_ERROR(PH, "State -> ERROR (I2C failures)");
		}
		return;
	}

	/* I2C success — reset error counter */
	ph_ctx.i2c_error_count = 0;

	/* ── Accumulate sample for averaging ──────────────────────── */
	if (ph_ctx.state == PH_STATE_IDLE) {
		ph_ctx.state = PH_STATE_SAMPLING;
		ph_ctx.sample_idx = 0;
	}

	ph_ctx.sample_buf[ph_ctx.sample_idx] = raw;
	ph_ctx.sample_idx++;

	/* ── Check if we have enough samples for an average ───────── */
	if (ph_ctx.sample_idx >= PH_AVG_COUNT) {
		/* Compute average */
		uint32_t sum = 0;
		for (uint8_t i = 0; i < PH_AVG_COUNT; i++) {
			sum += ph_ctx.sample_buf[i];
		}
		ph_ctx.raw_adc = (uint16_t) (sum / PH_AVG_COUNT);

		/* Convert to voltage (millivolts served by pH_GetMillivolts) */
		ph_ctx.voltage = pH_ADCToVoltage(ph_ctx.raw_adc);

		/* Update signal quality rolling window */
		pH_UpdateSignalQuality(ph_ctx.raw_adc);

		/* Transition to READY (or stay READY) */
		if (ph_ctx.state == PH_STATE_SAMPLING) {
			ph_ctx.state = PH_STATE_READY;
		}

		/* Reset for next averaging cycle */
		ph_ctx.sample_idx = 0;

		DBG_PRINT_V(PH, "ADC=%u V=%.3f mV=%u q=%u%%", ph_ctx.raw_adc,
				ph_ctx.voltage, (unsigned )(ph_ctx.voltage * 1000.0f + 0.5f),
				ph_ctx.signal_quality);
	}
}

pH_State_t pH_GetState(void) {
	return ph_ctx.state;
}

uint16_t pH_GetRawADC(void) {
	return ph_ctx.raw_adc;
}

uint16_t pH_GetMillivolts(void) {
	return (uint16_t) (ph_ctx.voltage * 1000.0f + 0.5f);
}

uint8_t pH_GetSignalQuality(void) {
	return ph_ctx.signal_quality;
}

void pH_ClearError(void) {
	if (ph_ctx.state == PH_STATE_ERROR) {
		ph_ctx.i2c_error_count = 0;
		ph_ctx.state = PH_STATE_IDLE;
		ph_ctx.sample_idx = 0;
		DBG_PRINT(PH, "Error cleared, state -> IDLE");
	}
}

/* Private functions ---------------------------------------------------------*/

/**
 * @brief  Read raw 12-bit value from MCP3221
 * @param  raw  Pointer to store result (0-4095)
 * @retval HAL_OK or HAL_ERROR
 */
static HAL_StatusTypeDef pH_ReadADC(uint16_t *raw) {
	uint8_t data[2];
	HAL_StatusTypeDef status;

	status = HAL_I2C_Master_Receive(ph_ctx.hi2c, PH_MCP3221_ADDR, data, 2,
	I2C_TIMEOUT_MS);

	if (status != HAL_OK) {
		return HAL_ERROR;
	}

	/*
	 * MCP3221 data format (MSB first):
	 *   byte0: 0000_D11_D10_D9_D8
	 *   byte1: D7_D6_D5_D4_D3_D2_D1_D0
	 *
	 * 12-bit result in lower 12 bits of 16-bit word.
	 * Ref: MikroE pH 2 Click driver (no right-shift needed).
	 */
	*raw = (((uint16_t) data[0] << 8) | data[1]) & PH_ADC_MASK;

	return HAL_OK;
}

/**
 * @brief  Convert 12-bit ADC value to voltage
 * @param  raw  ADC value (0-4095)
 * @retval Voltage in volts
 */
static float pH_ADCToVoltage(uint16_t raw) {
	return ((float) raw / (float) PH_ADC_RESOLUTION) * PH_VREF;
}

/**
 * @brief  Update rolling-window signal quality metric
 * @param  adc_avg  Latest averaged ADC reading
 * @note   Quality based on standard deviation of the rolling window.
 *         Lower stddev = more stable = higher quality.
 */
static void pH_UpdateSignalQuality(uint16_t adc_avg) {
	/* Add to rolling window */
	ph_ctx.quality_buf[ph_ctx.quality_idx] = adc_avg;
	ph_ctx.quality_idx = (ph_ctx.quality_idx + 1) % PH_QUALITY_WINDOW;
	if (ph_ctx.quality_count < PH_QUALITY_WINDOW) {
		ph_ctx.quality_count++;
	}

	/* Need at least 2 entries for meaningful stddev */
	if (ph_ctx.quality_count < 2) {
		ph_ctx.signal_quality = 50; /* Unknown — assume fair */
		return;
	}

	/* Compute mean */
	uint32_t sum = 0;
	for (uint8_t i = 0; i < ph_ctx.quality_count; i++) {
		sum += ph_ctx.quality_buf[i];
	}
	float mean = (float) sum / (float) ph_ctx.quality_count;

	/* Compute variance — compare against squared thresholds (avoids sqrtf) */
	float var_sum = 0.0f;
	for (uint8_t i = 0; i < ph_ctx.quality_count; i++) {
		float diff = (float) ph_ctx.quality_buf[i] - mean;
		var_sum += diff * diff;
	}
	float variance = var_sum / (float) ph_ctx.quality_count;

	/* Map variance to quality percentage (using squared thresholds) */
	if (variance < (float) QUALITY_GOOD_VAR) {
		ph_ctx.signal_quality = 100;
	} else if (variance < (float) QUALITY_FAIR_VAR) {
		/* Linear interpolation on variance: GOOD_VAR->99%, FAIR_VAR->50% */
		float range = (float) (QUALITY_FAIR_VAR - QUALITY_GOOD_VAR);
		float ratio = (variance - (float) QUALITY_GOOD_VAR) / range;
		ph_ctx.signal_quality = (uint8_t) (99.0f - (ratio * 49.0f));
	} else {
		/* Very noisy — scale down further, minimum 10% while responding */
		ph_ctx.signal_quality = 10;
	}
}
