/**
 ******************************************************************************
 * @file    ph_sensor.c
 * @brief   MCP3221 ADC + Nernst pH sensor driver (non-blocking)
 * @note    Ported from ExampleCode/Src/ph.c and MikroE pH 2 Click driver.
 *
 *          MCP3221A5T: 12-bit I2C ADC, continuous conversion, 2-byte read.
 *          Data format: [byte0: 0000_D11..D8] [byte1: D7..D0]
 *          → raw = ((byte0 << 8) | byte1) & 0x0FFF
 *
 *          Nernst equation (temperature-compensated):
 *            slope_mV = (R × T_K × ln10) / F × 1000
 *            pH = 7.0 − ((V − V_neutral) × 1000 / slope_mV)
 *            pH_corrected = (pH_raw × cal_slope) + cal_offset
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
#define QUALITY_GOOD_STDDEV     5u      /**< stddev < 5 → 100% */
#define QUALITY_FAIR_STDDEV     20u     /**< stddev 5–20 → proportional 50–99% */

/** Squared thresholds for variance comparison (avoids sqrtf / libm dependency) */
#define QUALITY_GOOD_VAR        (QUALITY_GOOD_STDDEV * QUALITY_GOOD_STDDEV)   /* 25 */
#define QUALITY_FAIR_VAR        (QUALITY_FAIR_STDDEV * QUALITY_FAIR_STDDEV)   /* 400 */

/** Inline absolute-value for float — avoids fabsf / libm dependency */
#define PH_ABSF(x)  ((x) >= 0.0f ? (x) : -(x))

/** Known buffer pH values (× 100 for integer math) */
#define PH_BUFFER_4_VALUE       401u    /**< pH 4.01 */
#define PH_BUFFER_7_VALUE       700u    /**< pH 7.00 */
#define PH_BUFFER_10_VALUE      1001u   /**< pH 10.01 */

/* Private types -------------------------------------------------------------*/

/** Calibration data — persistent across resets (lives in RAM for now) */
typedef struct {
	float neutral_voltage; /**< Voltage at pH 7 (V), default Vref/2 */

	/* Active calibration mode */
	pH_CalMode_t mode; /**< Which correction path is active */

	/* Single-segment calibration (used for 1-point and 2-point) */
	float cal_slope; /**< Calibration slope multiplier */
	float cal_offset; /**< Calibration offset (pH units) */

	/* Piecewise calibration segments (used for 3-point) */
	float acid_slope; /**< Slope for pH < 7 (from pH 4–7 span) */
	float acid_offset; /**< Offset for pH < 7 */
	float alka_slope; /**< Slope for pH >= 7 (from pH 7–10 span) */
	float alka_offset; /**< Offset for pH >= 7 */

	/* Multi-point reference storage */
	bool has_ph4; /**< pH 4 buffer reference captured */
	float ph4_voltage; /**< Measured voltage at pH 4.01 (V) */
	float ph4_raw_ph; /**< Raw pH reading at pH 4.01 buffer */

	bool has_ph7; /**< pH 7 buffer reference captured */
	float ph7_voltage; /**< Measured voltage at pH 7.00 (V) */
	float ph7_raw_ph; /**< Raw pH reading at pH 7.00 buffer */

	bool has_ph10; /**< pH 10 buffer reference captured */
	float ph10_voltage; /**< Measured voltage at pH 10.01 (V) */
	float ph10_raw_ph; /**< Raw pH reading at pH 10.01 buffer */
} pH_Cal_t;

/** Driver context */
typedef struct {
	I2C_HandleTypeDef *hi2c;
	pH_State_t state;

	/* Temperature for Nernst compensation */
	float temperature_c; /**< °C, fed from DS18B20 */

	/* Sampling state */
	uint32_t last_sample_ms; /**< HAL_GetTick at last ADC read */
	uint16_t sample_buf[PH_AVG_COUNT]; /**< Raw ADC samples for averaging */
	uint8_t sample_idx; /**< Next slot in sample_buf */

	/* Averaged result */
	uint16_t raw_adc; /**< Last averaged raw ADC value */
	float voltage; /**< Last averaged voltage (V) */
	float ph_float; /**< Last computed pH (float) */
	uint16_t ph_value; /**< pH × 100, clamped 0–1400 */

	/* Signal quality (rolling window of averaged readings) */
	uint16_t quality_buf[PH_QUALITY_WINDOW]; /**< Averaged ADC values */
	uint8_t quality_idx; /**< Next slot in quality_buf */
	uint8_t quality_count; /**< Number of valid entries (up to WINDOW) */
	uint8_t signal_quality; /**< 0–100% */

	/* Error tracking */
	uint8_t i2c_error_count; /**< Consecutive I2C failures */

	/* Calibration */
	pH_Cal_t cal;
} pH_Context_t;

/* Private variables ---------------------------------------------------------*/
static pH_Context_t ph_ctx;

/* Private function prototypes -----------------------------------------------*/
static HAL_StatusTypeDef pH_ReadADC(uint16_t *raw);
static float pH_ADCToVoltage(uint16_t raw);
static float pH_NernstRaw(float voltage);
static float pH_NernstCalculate(float voltage);
static void pH_UpdateSignalQuality(uint16_t adc_avg);
static void pH_ResetCalibration(void);

/* Exported functions --------------------------------------------------------*/

HAL_StatusTypeDef pH_Init(I2C_HandleTypeDef *hi2c) {
	memset(&ph_ctx, 0, sizeof(ph_ctx));

	ph_ctx.hi2c = hi2c;
	ph_ctx.state = PH_STATE_IDLE;
	ph_ctx.temperature_c = PH_DEFAULT_TEMP;
	ph_ctx.ph_value = 700; /* Default pH 7.00 */
	ph_ctx.ph_float = 7.0f;

	/* Initialize calibration to defaults */
	pH_ResetCalibration();

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
	if (ph_ctx.state == PH_STATE_ERROR
			|| ph_ctx.state == PH_STATE_CALIBRATING) {
		return; /* Don't sample during error or calibration */
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

		/* Convert to voltage and pH */
		ph_ctx.voltage = pH_ADCToVoltage(ph_ctx.raw_adc);
		ph_ctx.ph_float = pH_NernstCalculate(ph_ctx.voltage);

		/* Clamp and convert to × 100 integer */
		if (ph_ctx.ph_float < 0.0f) {
			ph_ctx.ph_value = PH_VALUE_MIN;
		} else if (ph_ctx.ph_float > 14.0f) {
			ph_ctx.ph_value = PH_VALUE_MAX;
		} else {
			ph_ctx.ph_value = (uint16_t) (ph_ctx.ph_float * 100.0f + 0.5f);
		}

		/* Update signal quality rolling window */
		pH_UpdateSignalQuality(ph_ctx.raw_adc);

		/* Transition to READY (or stay READY) */
		if (ph_ctx.state == PH_STATE_SAMPLING) {
			ph_ctx.state = PH_STATE_READY;
		}

		/* Reset for next averaging cycle */
		ph_ctx.sample_idx = 0;

		DBG_PRINT_V(PH, "ADC=%u V=%.3f mV=%u pH=%.2f q=%u%%", ph_ctx.raw_adc,
				ph_ctx.voltage, (unsigned )(ph_ctx.voltage * 1000.0f + 0.5f),
				ph_ctx.ph_float, ph_ctx.signal_quality);
	}
}

pH_State_t pH_GetState(void) {
	return ph_ctx.state;
}

uint16_t pH_GetValue(void) {
	return ph_ctx.ph_value;
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

void pH_SetTemperature(int16_t temp_c10) {
	ph_ctx.temperature_c = (float) temp_c10 / 10.0f;
}

HAL_StatusTypeDef pH_Calibrate(uint8_t command) {
	HAL_StatusTypeDef status = HAL_OK;

	DBG_PRINT(CAL, "pH_Calibrate: cmd=0x%02X", command);

	switch (command) {

	case PH_CAL_CMD_OFFSET: {
		/*
		 * Offset calibration: BNC connector shorted (no probe).
		 * The ADC should read Vref/2.  We store the actual reading
		 * as the new neutral voltage so any op-amp offset is absorbed.
		 */
		ph_ctx.state = PH_STATE_CALIBRATING;

		/* Take a fresh average (blocking — during calibration is OK) */
		uint32_t sum = 0;
		for (uint8_t i = 0; i < PH_AVG_COUNT; i++) {
			uint16_t raw;
			status = pH_ReadADC(&raw);
			if (status != HAL_OK) {
				DBG_ERROR(CAL, "Offset cal: ADC read failed");
				ph_ctx.state = PH_STATE_ERROR;
				return HAL_ERROR;
			}
			sum += raw;
			HAL_Delay(50); /* Short delay between samples */
		}
		uint16_t avg_raw = (uint16_t) (sum / PH_AVG_COUNT);
		ph_ctx.cal.neutral_voltage = pH_ADCToVoltage(avg_raw);

		DBG_PRINT(CAL, "Offset cal done: Vneutral=%.3fV mV=%u (ADC=%u)",
				ph_ctx.cal.neutral_voltage,
				(unsigned )(ph_ctx.cal.neutral_voltage * 1000.0f + 0.5f),
				avg_raw);

		ph_ctx.state = PH_STATE_READY;
		break;
	}

	case PH_CAL_CMD_PH7: {
		/*
		 * Single-point pH 7.00 calibration.
		 * Adjust offset so current reading maps to pH 7.00.
		 */
		ph_ctx.state = PH_STATE_CALIBRATING;

		uint32_t sum = 0;
		for (uint8_t i = 0; i < PH_AVG_COUNT; i++) {
			uint16_t raw;
			status = pH_ReadADC(&raw);
			if (status != HAL_OK) {
				DBG_ERROR(CAL, "pH7 cal: ADC read failed");
				ph_ctx.state = PH_STATE_ERROR;
				return HAL_ERROR;
			}
			sum += raw;
			HAL_Delay(50);
		}
		uint16_t avg_raw = (uint16_t) (sum / PH_AVG_COUNT);
		float voltage = pH_ADCToVoltage(avg_raw);

		/* Compute raw Nernst pH (uncorrected — consistent scale for all refs) */
		float raw_ph = pH_NernstRaw(voltage);

		/* Set offset so this reading = 7.00, keep slope at 1.0 */
		ph_ctx.cal.cal_offset = 7.0f - raw_ph;
		ph_ctx.cal.cal_slope = 1.0f;
		ph_ctx.cal.mode = PH_CAL_MODE_1POINT;

		/* Store reference for potential two-point / three-point */
		ph_ctx.cal.has_ph7 = true;
		ph_ctx.cal.ph7_voltage = voltage;
		ph_ctx.cal.ph7_raw_ph = raw_ph;

		DBG_PRINT(CAL, "pH7 cal done: V=%.3f mV=%u raw_pH=%.2f offset=%.3f",
				voltage, (unsigned )(voltage * 1000.0f + 0.5f), raw_ph,
				ph_ctx.cal.cal_offset);

		ph_ctx.state = PH_STATE_READY;
		break;
	}

	case PH_CAL_CMD_PH4: {
		/*
		 * pH 4.01 buffer calibration point.
		 * Stores measured values. If pH 7 cal exists, computes two-point
		 * slope+offset between pH 4 and pH 7.
		 */
		ph_ctx.state = PH_STATE_CALIBRATING;

		uint32_t sum = 0;
		for (uint8_t i = 0; i < PH_AVG_COUNT; i++) {
			uint16_t raw;
			status = pH_ReadADC(&raw);
			if (status != HAL_OK) {
				DBG_ERROR(CAL, "pH4 cal: ADC read failed");
				ph_ctx.state = PH_STATE_ERROR;
				return HAL_ERROR;
			}
			sum += raw;
			HAL_Delay(50);
		}
		uint16_t avg_raw = (uint16_t) (sum / PH_AVG_COUNT);
		float voltage = pH_ADCToVoltage(avg_raw);
		float raw_ph = pH_NernstRaw(voltage);

		ph_ctx.cal.has_ph4 = true;
		ph_ctx.cal.ph4_voltage = voltage;
		ph_ctx.cal.ph4_raw_ph = raw_ph;

		DBG_PRINT(CAL, "pH4 ref stored: V=%.3f mV=%u raw_pH=%.2f", voltage,
				(unsigned )(voltage * 1000.0f + 0.5f), raw_ph);

		/* Two-point calibration if pH 7 reference exists */
		if (ph_ctx.cal.has_ph7) {
			float known1 = 4.01f;
			float meas1 = ph_ctx.cal.ph4_raw_ph;
			float known2 = 7.00f;
			float meas2 = ph_ctx.cal.ph7_raw_ph;

			if (PH_ABSF(meas2 - meas1) > 0.01f) {
				ph_ctx.cal.cal_slope = (known2 - known1) / (meas2 - meas1);
				ph_ctx.cal.cal_offset = known1 - (meas1 * ph_ctx.cal.cal_slope);
				ph_ctx.cal.mode = PH_CAL_MODE_2POINT;
				DBG_PRINT(CAL, "Two-point (pH4+pH7): slope=%.4f offset=%.3f",
						ph_ctx.cal.cal_slope, ph_ctx.cal.cal_offset);
			} else {
				DBG_ERROR(CAL, "Two-point failed: pH4/pH7 too close");
			}
		}

		ph_ctx.state = PH_STATE_READY;
		break;
	}

	case PH_CAL_CMD_PH10: {
		/*
		 * pH 10.01 buffer calibration point.
		 * If pH 7 cal exists, computes two-point slope+offset between pH 7 and pH 10.
		 */
		ph_ctx.state = PH_STATE_CALIBRATING;

		uint32_t sum = 0;
		for (uint8_t i = 0; i < PH_AVG_COUNT; i++) {
			uint16_t raw;
			status = pH_ReadADC(&raw);
			if (status != HAL_OK) {
				DBG_ERROR(CAL, "pH10 cal: ADC read failed");
				ph_ctx.state = PH_STATE_ERROR;
				return HAL_ERROR;
			}
			sum += raw;
			HAL_Delay(50);
		}
		uint16_t avg_raw = (uint16_t) (sum / PH_AVG_COUNT);
		float voltage = pH_ADCToVoltage(avg_raw);
		float raw_ph = pH_NernstRaw(voltage);

		ph_ctx.cal.has_ph10 = true;
		ph_ctx.cal.ph10_voltage = voltage;
		ph_ctx.cal.ph10_raw_ph = raw_ph;

		DBG_PRINT(CAL, "pH10 ref stored: V=%.3f mV=%u raw_pH=%.2f", voltage,
				(unsigned )(voltage * 1000.0f + 0.5f), raw_ph);

		/* Two-point calibration with pH 7 reference */
		if (ph_ctx.cal.has_ph7) {
			float known1 = 7.00f;
			float meas1 = ph_ctx.cal.ph7_raw_ph;
			float known2 = 10.01f;
			float meas2 = ph_ctx.cal.ph10_raw_ph;

			if (PH_ABSF(meas2 - meas1) > 0.01f) {
				ph_ctx.cal.cal_slope = (known2 - known1) / (meas2 - meas1);
				ph_ctx.cal.cal_offset = known1 - (meas1 * ph_ctx.cal.cal_slope);
				ph_ctx.cal.mode = PH_CAL_MODE_2POINT;
				DBG_PRINT(CAL, "Two-point (pH7+pH10): slope=%.4f offset=%.3f",
						ph_ctx.cal.cal_slope, ph_ctx.cal.cal_offset);
			} else {
				DBG_ERROR(CAL, "Two-point failed: pH7/pH10 too close");
			}
		}

		ph_ctx.state = PH_STATE_READY;
		break;
	}

	case PH_CAL_CMD_3POINT: {
		/*
		 * Three-point piecewise calibration.
		 * Requires all three buffer references (pH 4, 7, 10) to be stored.
		 * Computes two linear segments:
		 *   Acidic  (pH < 7): slope/offset from pH 4.01 <-> pH 7.00
		 *   Alkaline (pH >= 7): slope/offset from pH 7.00 <-> pH 10.01
		 */
		if (!ph_ctx.cal.has_ph4 || !ph_ctx.cal.has_ph7
				|| !ph_ctx.cal.has_ph10) {
			DBG_ERROR(CAL, "3-point cal: missing refs (ph4=%d ph7=%d ph10=%d)",
					ph_ctx.cal.has_ph4, ph_ctx.cal.has_ph7,
					ph_ctx.cal.has_ph10);
			status = HAL_ERROR;
			break;
		}

		/* Acidic segment: pH 4.01 <-> pH 7.00 */
		float acid_meas_span = ph_ctx.cal.ph7_raw_ph - ph_ctx.cal.ph4_raw_ph;
		if (PH_ABSF(acid_meas_span) < 0.01f) {
			DBG_ERROR(CAL, "3-point cal: pH4/pH7 too close (span=%.3f)",
					acid_meas_span);
			status = HAL_ERROR;
			break;
		}
		ph_ctx.cal.acid_slope = (7.00f - 4.01f) / acid_meas_span;
		ph_ctx.cal.acid_offset = 4.01f
				- (ph_ctx.cal.ph4_raw_ph * ph_ctx.cal.acid_slope);

		/* Alkaline segment: pH 7.00 <-> pH 10.01 */
		float alka_meas_span = ph_ctx.cal.ph10_raw_ph - ph_ctx.cal.ph7_raw_ph;
		if (PH_ABSF(alka_meas_span) < 0.01f) {
			DBG_ERROR(CAL, "3-point cal: pH7/pH10 too close (span=%.3f)",
					alka_meas_span);
			status = HAL_ERROR;
			break;
		}
		ph_ctx.cal.alka_slope = (10.01f - 7.00f) / alka_meas_span;
		ph_ctx.cal.alka_offset = 7.00f
				- (ph_ctx.cal.ph7_raw_ph * ph_ctx.cal.alka_slope);

		/* Activate piecewise mode */
		ph_ctx.cal.mode = PH_CAL_MODE_3POINT;

		/* Keep single-segment fields at identity for SDO readback compat */
		ph_ctx.cal.cal_slope = 1.0f;
		ph_ctx.cal.cal_offset = 0.0f;

		DBG_PRINT(CAL,
				"3-point cal done: acid[s=%.4f o=%.3f] alka[s=%.4f o=%.3f]",
				ph_ctx.cal.acid_slope, ph_ctx.cal.acid_offset,
				ph_ctx.cal.alka_slope, ph_ctx.cal.alka_offset);

		ph_ctx.state = PH_STATE_READY;
		break;
	}

	case PH_CAL_CMD_RESET:
		pH_ResetCalibration();
		DBG_PRINT(CAL, "Calibration reset to factory defaults");
		if (ph_ctx.state != PH_STATE_ERROR) {
			ph_ctx.state = PH_STATE_READY;
		}
		break;

	default:
		DBG_ERROR(CAL, "Unknown cal command: 0x%02X", command);
		status = HAL_ERROR;
		break;
	}

	return status;
}

pH_CalMode_t pH_GetCalMode(void) {
	return ph_ctx.cal.mode;
}

uint8_t pH_GetElectrodeStatus(void) {
	uint8_t status = 0;

	/* Bit 0: Probe connected — voltage not railed high or low */
	uint16_t mv = (uint16_t) (ph_ctx.voltage * 1000.0f + 0.5f);
	if (mv > 100 && mv < 3200) {
		status |= ELEC_CONNECTED;
	}

	/* Bit 1: Signal stable — quality >= 50% */
	if (ph_ctx.signal_quality >= 50) {
		status |= ELEC_SIGNAL_OK;
	}

	/* Bit 2: Calibrated — any cal mode active */
	if (ph_ctx.cal.mode != PH_CAL_MODE_NONE) {
		status |= ELEC_CALIBRATED;
	}

	/*
	 * Bit 3: Slope healthy.
	 *
	 * The analog front-end (MikroE pH 2 Click) amplifies the electrode
	 * signal before the ADC, so cal slopes encode the hardware gain and
	 * are NOT near 1.0.  Absolute slope ≈ 1/G (typically 0.3–0.5).
	 *
	 * Instead of checking an absolute range we verify:
	 *   (a) slopes are positive and non-degenerate  (> 0.05)
	 *   (b) for 3-point: the two segments are symmetric — ratio of
	 *       acid_slope / alka_slope is within 0.70–1.30, catching
	 *       asymmetric electrode degradation.
	 */
	if (ph_ctx.cal.mode == PH_CAL_MODE_3POINT) {
		if (ph_ctx.cal.acid_slope > 0.05f && ph_ctx.cal.alka_slope > 0.05f) {
			float ratio = ph_ctx.cal.acid_slope / ph_ctx.cal.alka_slope;
			if (ratio >= 0.70f && ratio <= 1.30f) {
				status |= ELEC_SLOPE_OK;
			}
		}
	} else if (ph_ctx.cal.mode == PH_CAL_MODE_2POINT) {
		/* Single-segment: just verify slope is positive and non-degenerate */
		if (ph_ctx.cal.cal_slope > 0.05f) {
			status |= ELEC_SLOPE_OK;
		}
	} else {
		/* 1-point or NONE: slope is 1.0 by default, always OK */
		status |= ELEC_SLOPE_OK;
	}

	/* Bit 4: Offset OK — neutral voltage near Vref/2 (±0.3V) */
	float offset_diff = ph_ctx.cal.neutral_voltage - PH_NEUTRAL_VOLTAGE;
	if (offset_diff < 0.0f)
		offset_diff = -offset_diff;
	if (offset_diff <= 0.3f) {
		status |= ELEC_OFFSET_OK;
	}

	/* Bit 5: I2C responding — not in error state */
	if (ph_ctx.state != PH_STATE_ERROR) {
		status |= ELEC_RESPONDING;
	}

	return status;
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
 * @param  raw  Pointer to store result (0–4095)
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
 * @param  raw  ADC value (0–4095)
 * @retval Voltage in volts
 */
static float pH_ADCToVoltage(uint16_t raw) {
	return ((float) raw / (float) PH_ADC_RESOLUTION) * PH_VREF;
}

/**
 * @brief  Raw Nernst equation: voltage → uncorrected pH (no slope/offset cal)
 * @param  voltage  Measured voltage (V)
 * @retval Raw Nernst pH (temperature-compensated, but NO calibration correction)
 * @note   Used by calibration handlers to store consistent reference values.
 *         All stored raw_ph values are on the same scale regardless of which
 *         calibration mode was active when the reference was captured.
 */
static float pH_NernstRaw(float voltage) {
	float temp_k = ph_ctx.temperature_c + 273.15f;
	float slope_mv = (PH_R_CONSTANT * temp_k * PH_LN10) / PH_F_CONSTANT
			* 1000.0f;
	float delta_mv = (voltage - ph_ctx.cal.neutral_voltage) * 1000.0f;
	return 7.0f - (delta_mv / slope_mv);
}

/**
 * @brief  Nernst equation: voltage → pH with temp compensation + calibration
 * @param  voltage  Measured voltage (V)
 * @retval Corrected pH value
 */
static float pH_NernstCalculate(float voltage) {
	/* Get uncorrected Nernst pH */
	float ph_raw = pH_NernstRaw(voltage);

	/* Apply calibration correction based on active mode */
	float ph_corrected;

	if (ph_ctx.cal.mode == PH_CAL_MODE_3POINT) {
		/* Piecewise: select segment based on raw pH relative to 7.0 */
		if (ph_raw < 7.0f) {
			/* Acidic segment (pH 4–7) */
			ph_corrected = (ph_raw * ph_ctx.cal.acid_slope)
					+ ph_ctx.cal.acid_offset;
		} else {
			/* Alkaline segment (pH 7–10) */
			ph_corrected = (ph_raw * ph_ctx.cal.alka_slope)
					+ ph_ctx.cal.alka_offset;
		}
	} else {
		/* Single-segment: 1-point or 2-point (existing behavior) */
		ph_corrected = (ph_raw * ph_ctx.cal.cal_slope) + ph_ctx.cal.cal_offset;
	}

	return ph_corrected;
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
		/* Linear interpolation on variance: GOOD_VAR→99%, FAIR_VAR→50% */
		float range = (float) (QUALITY_FAIR_VAR - QUALITY_GOOD_VAR);
		float ratio = (variance - (float) QUALITY_GOOD_VAR) / range;
		ph_ctx.signal_quality = (uint8_t) (99.0f - (ratio * 49.0f));
	} else {
		/* Very noisy — scale down further, minimum 10% while responding */
		ph_ctx.signal_quality = 10;
	}
}

/**
 * @brief  Reset calibration to factory defaults
 */
static void pH_ResetCalibration(void) {
	ph_ctx.cal.neutral_voltage = PH_NEUTRAL_VOLTAGE;
	ph_ctx.cal.mode = PH_CAL_MODE_NONE;
	ph_ctx.cal.cal_slope = PH_DEFAULT_SLOPE;
	ph_ctx.cal.cal_offset = PH_DEFAULT_OFFSET;
	ph_ctx.cal.acid_slope = PH_DEFAULT_SLOPE;
	ph_ctx.cal.acid_offset = PH_DEFAULT_OFFSET;
	ph_ctx.cal.alka_slope = PH_DEFAULT_SLOPE;
	ph_ctx.cal.alka_offset = PH_DEFAULT_OFFSET;
	ph_ctx.cal.has_ph4 = false;
	ph_ctx.cal.has_ph7 = false;
	ph_ctx.cal.has_ph10 = false;
	ph_ctx.cal.ph4_voltage = 0.0f;
	ph_ctx.cal.ph4_raw_ph = 0.0f;
	ph_ctx.cal.ph7_voltage = 0.0f;
	ph_ctx.cal.ph7_raw_ph = 0.0f;
	ph_ctx.cal.ph10_voltage = 0.0f;
	ph_ctx.cal.ph10_raw_ph = 0.0f;
}
