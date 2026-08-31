/**
 ******************************************************************************
 * @file    procimg_api.h
 * @brief   Typed getter/setter API for MCO process image access
 * @note    Isolates gProcImg[] byte layout in one place so that
 *          sensor_control.c (and any future consumers) are agnostic to MCO
 *          internals.  All functions are static inline — zero call overhead.
 *
 *          "Dumb module" scope (docs/PHTEMP_REFACTOR_PLAN.md): no pH/cal
 *          accessors. The module exchanges only ControlWord/StatusWord, raw
 *          electrode millivolts, temperature, sensor health, and TPDO deltas.
 *
 ******************************************************************************
 */

#ifndef PROCIMG_API_H
#define PROCIMG_API_H

#include <stdint.h>
#include <string.h>
#include "procimg.h"       /* P604000_ControlWord, P604100_StatusWord, etc. */
#include "mcop_inc.h"      /* MEM_PROC, gProcImg[] declaration */

/* The MCO stack owns gProcImg[]; we reference it here so callers don't need to. */
extern uint8_t MEM_PROC gProcImg[];

/* Transitional shim: OD object 0x2400 is renamed pHDeltaThreshold ->
 * MillivoltDeltaThreshold at the Step-2 EDS regen (docs/PHTEMP_EDS_REGEN_CHANGELIST.md).
 * This lets the millivolt-semantics symbol resolve BEFORE the regen (aliasing the
 * old generated name) and AFTER (using the regenerated one). Remove this block
 * once the regenerated pimg.h defines P240000_MillivoltDeltaThreshold directly. */
#ifndef P240000_MillivoltDeltaThreshold
#define P240000_MillivoltDeltaThreshold P240000_pHDeltaThreshold
#endif

/* ========================================================================== */
/* GETTERS (RPDO / SDO → application)                                         */
/* ========================================================================== */

/** @brief Read ControlWord [0x6040] — uint16_t */
static inline uint16_t ProcImg_GetControlWord(void) {
	uint16_t val;
	memcpy(&val, &gProcImg[P604000_ControlWord], 2);
	return val;
}

/** @brief Read LEDControl [0x2000] — uint8_t */
static inline uint8_t ProcImg_GetLEDControl(void) {
	return gProcImg[P200000_LEDControl];
}

/** @brief Read MillivoltDeltaThreshold [0x2400] — uint16_t (mV delta) */
static inline uint16_t ProcImg_GetMillivoltDeltaThreshold(void) {
	uint16_t val;
	memcpy(&val, &gProcImg[P240000_MillivoltDeltaThreshold], 2);
	return val;
}

/** @brief Read TempDeltaThreshold [0x2401] — int16_t (°C × 10 delta) */
static inline int16_t ProcImg_GetTempDeltaThreshold(void) {
	int16_t val;
	memcpy(&val, &gProcImg[P240100_TempDeltaTheshold], 2);
	return val;
}

/** @brief Read StatusDeltaThreshold [0x2402] — uint8_t (bit-change delta) */
static inline uint8_t ProcImg_GetStatusDeltaThreshold(void) {
	return gProcImg[P240200_StatusDeltaThreshold];
}

/* ========================================================================== */
/* SETTERS (application → TPDO / SDO)                                         */
/* ========================================================================== */

/** @brief Write StatusWord [0x6041] — uint16_t */
static inline void ProcImg_SetStatusWord(uint16_t sw) {
	memcpy(&gProcImg[P604100_StatusWord], &sw, 2);
}

/** @brief Write Temperature [0x6010] — int16_t (°C × 10, e.g. 235 = 23.5°C) */
static inline void ProcImg_SetTemperature(int16_t temp) {
	memcpy(&gProcImg[P601000_Temperature], &temp, 2);
}

/** @brief Write pHMillivolts [0x6003] — uint16_t (mV, e.g. 1650 = 1.650V) */
static inline void ProcImg_SetpHMillivolts(uint16_t mv) {
	memcpy(&gProcImg[P600300_pHMillivolts], &mv, 2);
}

/** @brief Write SensorStatus [0x2300] — uint8_t (bitfield) */
static inline void ProcImg_SetSensorStatus(uint8_t status) {
	gProcImg[P230000_SensorStatus] = status;
}

/** @brief Write pHSignalQuality [0x6001] — uint8_t (0-100%) */
static inline void ProcImg_SetpHSignalQuality(uint8_t quality) {
	gProcImg[P600100_pHSignalQuality] = quality;
}

/** @brief Write TempSignalQuality [0x6011] — uint8_t (0-100%) */
static inline void ProcImg_SetTempSignalQuality(uint8_t quality) {
	gProcImg[P601100_TempSignalQuality] = quality;
}

/** @brief Write pHSensorStatus [0x6002] — uint8_t */
static inline void ProcImg_SetpHSensorStatus(uint8_t status) {
	gProcImg[P600200_pHSensorStatus] = status;
}

/** @brief Write TempSensorStatus [0x6012] — uint8_t */
static inline void ProcImg_SetTempSensorStatus(uint8_t status) {
	gProcImg[P601200_TempSensorStatus] = status;
}

/** @brief Write ErrorRegister [0x1001] — uint8_t */
static inline void ProcImg_SetErrorRegister(uint8_t err) {
	gProcImg[P100100_Error_Register] = err;
}

/** @brief Write LEDControl [0x2000] — uint8_t */
static inline void ProcImg_SetLEDControl(uint8_t mode) {
	gProcImg[P200000_LEDControl] = mode;
}

#endif /* PROCIMG_API_H */
