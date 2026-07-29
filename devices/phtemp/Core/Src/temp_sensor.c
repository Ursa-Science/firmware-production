/**
 ******************************************************************************
 * @file    temp_sensor.c
 * @brief   DS18B20 1-Wire temperature sensor driver (non-blocking)
 * @note    Bit-bang 1-Wire protocol on DQ pin (PB0, open-drain).
 *          Uses DWT cycle counter for microsecond-precision timing.
 *
 *          The DS18B20 is operated in "skip ROM" mode (single device on bus).
 *          12-bit resolution (default): 750ms conversion time, 0.0625°C LSB.
 *
 *          1-Wire Timing (from DS18B20 datasheet):
 *            Reset:   480µs low, 480µs release, presence 60-240µs low
 *            Write 0: 60µs low, 10µs release
 *            Write 1: 6µs low, 64µs release
 *            Read:    6µs low, 9µs release, sample, 55µs wait
 *
 * @date    2026
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "temp_sensor.h"
#include "log.h"
#include <string.h>

/* Private defines -----------------------------------------------------------*/

/** 1-Wire ROM commands */
#define OW_CMD_SKIP_ROM         0xCC

/** DS18B20 function commands */
#define DS18B20_CMD_CONVERT_T   0x44
#define DS18B20_CMD_READ_SCRATCH 0xBE

/** Scratchpad size (9 bytes: 2 temp + 3 config + 1 reserved + 1 reserved + 1 config + 1 CRC) */
#define DS18B20_SCRATCHPAD_SIZE 9

/** Conversion time for 12-bit resolution (ms) */
#define DS18B20_CONV_TIME_MS    750

/** Maximum time to wait for conversion before declaring timeout (ms) */
#define DS18B20_CONV_TIMEOUT_MS 1000

/** Number of consecutive errors before entering ERROR state */
#define DS18B20_MAX_RETRIES     3

/** Default temperature when no reading available (25.0°C × 10) */
#define TEMP_DEFAULT_VALUE      250

/* Private types -------------------------------------------------------------*/

typedef struct {
	/* GPIO */
	GPIO_TypeDef *port;
	uint16_t pin;

	/* State machine */
	Temp_State_t state;
	uint32_t conv_start_ms; /**< HAL_GetTick() when Convert T was issued */
	uint8_t error_count; /**< Consecutive error counter               */

	/* Readings */
	int16_t raw_value; /**< Raw 16-bit DS18B20 register             */
	int16_t value; /**< Temperature in °C × 10 (with offset)    */
	int16_t offset; /**< User offset in °C × 10 (from 0x2210)    */
	uint8_t signal_quality; /**< 0-100%                                  */

	/* Init flag */
	bool initialized;
} TempSensor_t;

/* Private variables ---------------------------------------------------------*/
static TempSensor_t ts;

/* Private function prototypes -----------------------------------------------*/
static void delay_us(uint32_t us);
static void dwt_init(void);
static void ow_pin_low(void);
static void ow_pin_release(void);
static uint8_t ow_pin_read(void);
static bool ow_reset(void);
static void ow_write_byte(uint8_t data);
static uint8_t ow_read_byte(void);
static uint8_t ds18b20_crc8(const uint8_t *data, uint8_t len);

/* ─── DWT Microsecond Delay ─────────────────────────────────────────────── */

/**
 * @brief  Initialize DWT cycle counter for µs delays
 * @note   CoreDebug->DEMCR and DWT->CYCCNT must be enabled.
 *         Safe to call multiple times.
 */
static void dwt_init(void) {
	CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
	DWT->CYCCNT = 0;
	DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

/**
 * @brief  Blocking microsecond delay using DWT cycle counter
 * @param  us  Microseconds to wait
 * @note   At 80 MHz, 1 µs = 80 cycles.
 */
static void delay_us(uint32_t us) {
	uint32_t start = DWT->CYCCNT;
	uint32_t cycles = us * (SystemCoreClock / 1000000u);
	while ((DWT->CYCCNT - start) < cycles) {
		/* spin */
	}
}

/* ─── 1-Wire Primitives ─────────────────────────────────────────────────── */

/** Drive DQ low (open-drain: set output low) */
static void ow_pin_low(void) {
	ts.port->BSRR = (uint32_t) ts.pin << 16u; /* Reset bit */
}

/** Release DQ (open-drain: set output high → pulled up by external resistor) */
static void ow_pin_release(void) {
	ts.port->BSRR = ts.pin; /* Set bit */
}

/** Read DQ pin state */
static uint8_t ow_pin_read(void) {
	return (ts.port->IDR & ts.pin) ? 1u : 0u;
}

/**
 * @brief  1-Wire reset pulse + presence detect
 * @retval true if device present, false if no presence detected
 * @note   Timing from datasheet: 480µs low, release, sample presence 60-240µs
 *         Total slot: ~960µs. Interrupts disabled during critical window.
 */
static bool ow_reset(void) {
	uint8_t presence;

	uint32_t primask = __get_PRIMASK();
	__disable_irq();

	ow_pin_low();
	delay_us(480);
	ow_pin_release();
	delay_us(70); /* Wait for device to pull low (15-60µs response) */
	presence = !ow_pin_read(); /* Low = device present */
	delay_us(410); /* Complete the reset time slot */

	__set_PRIMASK(primask);

	return presence;
}

/**
 * @brief  Write one byte to the 1-Wire bus (LSB first)
 * @param  data  Byte to write
 * @note   Write-0: 60µs low, 10µs recovery
 *         Write-1: 6µs low, 64µs recovery
 *         Interrupts disabled per bit for timing accuracy.
 */
static void ow_write_byte(uint8_t data) {
	for (uint8_t i = 0; i < 8; i++) {
		uint32_t primask = __get_PRIMASK();
		__disable_irq();

		if (data & (1u << i)) {
			/* Write 1 */
			ow_pin_low();
			delay_us(6);
			ow_pin_release();
			delay_us(64);
		} else {
			/* Write 0 */
			ow_pin_low();
			delay_us(60);
			ow_pin_release();
			delay_us(10);
		}

		__set_PRIMASK(primask);
	}
}

/**
 * @brief  Read one byte from the 1-Wire bus (LSB first)
 * @retval Byte read from device
 * @note   Read slot: 6µs low, release, sample at ~9µs, 55µs wait
 *         Interrupts disabled per bit for timing accuracy.
 */
static uint8_t ow_read_byte(void) {
	uint8_t data = 0;

	for (uint8_t i = 0; i < 8; i++) {
		uint32_t primask = __get_PRIMASK();
		__disable_irq();

		ow_pin_low();
		delay_us(6);
		ow_pin_release();
		delay_us(9);

		if (ow_pin_read()) {
			data |= (1u << i);
		}

		delay_us(55);

		__set_PRIMASK(primask);
	}

	return data;
}

/* ─── CRC8 (Dallas/Maxim polynomial: X^8 + X^5 + X^4 + 1) ─────────────── */

/**
 * @brief  Compute DS18B20 CRC8 over a data buffer
 * @param  data  Pointer to data bytes
 * @param  len   Number of bytes
 * @retval CRC8 value (should be 0 when computed over data+CRC byte)
 */
static uint8_t ds18b20_crc8(const uint8_t *data, uint8_t len) {
	uint8_t crc = 0;

	for (uint8_t i = 0; i < len; i++) {
		uint8_t byte = data[i];
		for (uint8_t j = 0; j < 8; j++) {
			uint8_t mix = (crc ^ byte) & 0x01;
			crc >>= 1;
			if (mix) {
				crc ^= 0x8C; /* Reversed polynomial */
			}
			byte >>= 1;
		}
	}

	return crc;
}

/* ─── Public API ─────────────────────────────────────────────────────────── */

HAL_StatusTypeDef Temp_Init(GPIO_TypeDef *port, uint16_t pin) {
	memset(&ts, 0, sizeof(ts));
	ts.port = port;
	ts.pin = pin;
	ts.state = TEMP_STATE_IDLE;
	ts.value = TEMP_DEFAULT_VALUE;
	ts.raw_value = 0;
	ts.signal_quality = 0;
	ts.offset = 0;
	ts.error_count = 0;
	ts.initialized = false;

	/* Enable DWT cycle counter for µs delays */
	dwt_init();

	/* Release bus */
	ow_pin_release();
	delay_us(500);

	/* Verify device presence */
	if (!ow_reset()) {
		DBG_ERROR(TEMP, "Init: no DS18B20 presence detected on DQ");
		ts.state = TEMP_STATE_ERROR;
		ts.signal_quality = 0;
		return HAL_ERROR;
	}

	ts.initialized = true;
	ts.signal_quality = 100;
	DBG_PRINT(TEMP, "Init: DS18B20 detected on DQ");
	return HAL_OK;
}

void Temp_Process(void) {
	if (!ts.initialized) {
		return;
	}

	switch (ts.state) {
	case TEMP_STATE_IDLE:
	case TEMP_STATE_READY:
		/* Auto-start next conversion */
		if (Temp_StartConversion() != HAL_OK) {
			ts.error_count++;
			if (ts.error_count >= DS18B20_MAX_RETRIES) {
				ts.state = TEMP_STATE_ERROR;
				ts.signal_quality = 0;
				DBG_ERROR(TEMP,
						"Process: %u consecutive errors, entering ERROR",
						ts.error_count);
			}
		}
		break;

	case TEMP_STATE_CONVERTING:
		if (Temp_IsConversionDone()) {
			if (Temp_ReadResult() == HAL_OK) {
				ts.error_count = 0; /* Reset on success */
			} else {
				ts.error_count++;
				if (ts.error_count >= DS18B20_MAX_RETRIES) {
					ts.state = TEMP_STATE_ERROR;
					ts.signal_quality = 0;
					DBG_ERROR(TEMP,
							"Process: read failed %u times, entering ERROR",
							ts.error_count);
				} else {
					/* Retry: go back to IDLE for next Process() cycle */
					ts.state = TEMP_STATE_IDLE;
				}
			}
		} else {
			/* Check for conversion timeout */
			uint32_t elapsed = HAL_GetTick() - ts.conv_start_ms;
			if (elapsed > DS18B20_CONV_TIMEOUT_MS) {
				DBG_ERROR(TEMP, "Process: conversion timeout (%lums)", elapsed);
				ts.error_count++;
				if (ts.error_count >= DS18B20_MAX_RETRIES) {
					ts.state = TEMP_STATE_ERROR;
					ts.signal_quality = 0;
				} else {
					ts.state = TEMP_STATE_IDLE;
				}
			}
		}
		break;

	case TEMP_STATE_READING:
		/* Transient state — should not persist between Process() calls */
		ts.state = TEMP_STATE_IDLE;
		break;

	case TEMP_STATE_ERROR:
		/* Stay in error until cleared externally */
		break;
	}
}

HAL_StatusTypeDef Temp_StartConversion(void) {
	if (!ts.initialized) {
		return HAL_ERROR;
	}

	/* 1-Wire: Reset → Skip ROM → Convert T */
	if (!ow_reset()) {
		DBG_PRINT_V(TEMP, "StartConversion: no presence");
		return HAL_ERROR;
	}

	ow_write_byte(OW_CMD_SKIP_ROM);
	ow_write_byte(DS18B20_CMD_CONVERT_T);

	ts.conv_start_ms = HAL_GetTick();
	ts.state = TEMP_STATE_CONVERTING;

	return HAL_OK;
}

bool Temp_IsConversionDone(void) {
	if (ts.state != TEMP_STATE_CONVERTING) {
		return true; /* Not converting — trivially "done" */
	}

	uint32_t elapsed = HAL_GetTick() - ts.conv_start_ms;
	return (elapsed >= DS18B20_CONV_TIME_MS);
}

HAL_StatusTypeDef Temp_ReadResult(void) {
	if (!ts.initialized) {
		return HAL_ERROR;
	}

	ts.state = TEMP_STATE_READING;

	/* 1-Wire: Reset → Skip ROM → Read Scratchpad */
	if (!ow_reset()) {
		DBG_PRINT_V(TEMP, "ReadResult: no presence");
		ts.state = TEMP_STATE_IDLE;
		return HAL_ERROR;
	}

	ow_write_byte(OW_CMD_SKIP_ROM);
	ow_write_byte(DS18B20_CMD_READ_SCRATCH);

	/* Read all 9 bytes of scratchpad */
	uint8_t scratch[DS18B20_SCRATCHPAD_SIZE];
	for (uint8_t i = 0; i < DS18B20_SCRATCHPAD_SIZE; i++) {
		scratch[i] = ow_read_byte();
	}

	/* CRC8 validation (computed over all 9 bytes should yield 0) */
	uint8_t crc = ds18b20_crc8(scratch, DS18B20_SCRATCHPAD_SIZE);

	/* Parse temperature: bytes 0-1, 16-bit signed, LSB = 0.0625°C */
	int16_t raw = (int16_t) ((uint16_t) scratch[1] << 8) | scratch[0];

	if (crc != 0) {
		/* CRC mismatch — use reading but flag as degraded */
		DBG_PRINT(TEMP, "ReadResult: CRC mismatch (0x%02X), raw=%d", crc, raw);
		ts.signal_quality = 50;
	} else {
		ts.signal_quality = 100;
	}

	ts.raw_value = raw;

	/* Convert raw to °C × 10:
	 * raw LSB = 0.0625°C
	 * °C × 10 = raw * 0.625 = raw * 10 / 16
	 * Use integer arithmetic: (raw * 10 + 8) / 16  (with rounding) */
	int16_t temp_x10 = (int16_t) (((int32_t) raw * 10 + 8) / 16);

	/* Apply user offset */
	temp_x10 += ts.offset;

	ts.value = temp_x10;
	ts.state = TEMP_STATE_READY;

	DBG_PRINT_V(TEMP, "ReadResult: raw=%d, temp=%d.%d C (offset=%d)", raw,
			temp_x10 / 10, (temp_x10 >= 0 ? temp_x10 : -temp_x10) % 10,
			ts.offset);

	return HAL_OK;
}

Temp_State_t Temp_GetState(void) {
	return ts.state;
}

int16_t Temp_GetValue(void) {
	return ts.value;
}

int16_t Temp_GetRawValue(void) {
	return ts.raw_value;
}

uint8_t Temp_GetSignalQuality(void) {
	return ts.signal_quality;
}

void Temp_SetOffset(int16_t offset) {
	ts.offset = offset;
	DBG_PRINT(TEMP, "SetOffset: %d (%d.%d C)", offset, offset / 10,
			(offset >= 0 ? offset : -offset) % 10);
}

void Temp_ClearError(void) {
	if (ts.state == TEMP_STATE_ERROR) {
		ts.state = TEMP_STATE_IDLE;
		ts.error_count = 0;
		ts.signal_quality = 0;
		DBG_PRINT(TEMP, "ClearError: state -> IDLE");
	}
}
