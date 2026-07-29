/**
 ******************************************************************************
 * @file    procimg_api.h
 * @brief   Typed getter/setter API for MCO process image access
 * @note    Isolates gProcImg[] byte layout in one place so that
 *          sensor_control.c (and any future consumers) are agnostic to MCO
 *          internals.  All functions are static inline — zero call overhead.
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

/** @brief Read pHCalibrationCommand [0x2200] — uint8_t (WO from master) */
static inline uint8_t ProcImg_GetCalibrationCommand(void) {
	return gProcImg[P220000_pHCalibrationCommand];
}

/** @brief Read CalibrationBuffer4 [0x2202] — int16_t (mV at pH 4.01) */
static inline int16_t ProcImg_GetCalibrationBuffer4(void) {
	int16_t val;
	memcpy(&val, &gProcImg[P220200_CalibrationBuffer4], 2);
	return val;
}

/** @brief Read CalibrationBuffer7 [0x2203] — int16_t (mV at pH 7.00) */
static inline int16_t ProcImg_GetCalibrationBuffer7(void) {
	int16_t val;
	memcpy(&val, &gProcImg[P220300_CalibrationBuffer7], 2);
	return val;
}

/** @brief Read CalibrationBuffer10 [0x2204] — int16_t (mV at pH 10.01) */
static inline int16_t ProcImg_GetCalibrationBuffer10(void) {
	int16_t val;
	memcpy(&val, &gProcImg[P220400_CalibrationBuffer10], 2);
	return val;
}

/** @brief Read TempOffset [0x2210] — int16_t (°C × 10 correction) */
static inline int16_t ProcImg_GetTempOffset(void) {
	int16_t val;
	memcpy(&val, &gProcImg[P221000_TempOffset], 2);
	return val;
}

/** @brief Read pHDeltaThreshold [0x2400] — uint16_t (pH × 100 delta) */
static inline uint16_t ProcImg_GetpHDeltaThreshold(void) {
	uint16_t val;
	memcpy(&val, &gProcImg[P240000_pHDeltaThreshold], 2);
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

/** @brief Write pHValue [0x6000] — uint16_t (pH × 100, e.g. 700 = pH 7.00) */
static inline void ProcImg_SetpHValue(uint16_t ph) {
	memcpy(&gProcImg[P600000_pHValue], &ph, 2);
}

/** @brief Write Temperature [0x6010] — int16_t (°C × 10, e.g. 235 = 23.5°C) */
static inline void ProcImg_SetTemperature(int16_t temp) {
	memcpy(&gProcImg[P601000_Temperature], &temp, 2);
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

/** @brief Write ErrorRegister [0x1001] — uint8_t */
static inline void ProcImg_SetErrorRegister(uint8_t err) {
	gProcImg[P100100_Error_Register] = err;
}

/** @brief Write pHCalibrationStatus [0x2201] — uint8_t */
static inline void ProcImg_SetCalibrationStatus(uint8_t status) {
	gProcImg[P220100_pHCalibrationStatus] = status;
}

/** @brief Write pHSensorStatus [0x6002] — uint8_t */
static inline void ProcImg_SetpHSensorStatus(uint8_t status) {
	gProcImg[P600200_pHSensorStatus] = status;
}

/** @brief Write pHMillivolts [0x6003] — uint16_t (mV, e.g. 1650 = 1.650V) */
static inline void ProcImg_SetpHMillivolts(uint16_t mv) {
	memcpy(&gProcImg[P600300_pHMillivolts], &mv, 2);
}

/** @brief Read pHMillivolts [0x6003] — uint16_t */
static inline uint16_t ProcImg_GetpHMillivolts(void) {
	uint16_t val;
	memcpy(&val, &gProcImg[P600300_pHMillivolts], 2);
	return val;
}

/** @brief Write pHCalibrationMode [0x2205] — uint8_t (0=NONE,1=1PT,2=2PT,3=3PT) */
static inline void ProcImg_SetCalibrationMode(uint8_t mode) {
	gProcImg[P220500_pHCalibrationMode] = mode;
}

/** @brief Read pHCalibrationMode [0x2205] — uint8_t */
static inline uint8_t ProcImg_GetCalibrationMode(void) {
	return gProcImg[P220500_pHCalibrationMode];
}

/** @brief Write TempSensorStatus [0x6012] — uint8_t */
static inline void ProcImg_SetTempSensorStatus(uint8_t status) {
	gProcImg[P601200_TempSensorStatus] = status;
}

/** @brief Write pHElectrodeStatus [0x2220] — uint8_t */
static inline void ProcImg_SetElectrodeStatus(uint8_t status) {
	gProcImg[P222000_pHElectrodeStatus] = status;
}

/** @brief Write ElectrodeAge [0x2221] — uint16_t (hours) */
static inline void ProcImg_SetElectrodeAge(uint16_t hours) {
	memcpy(&gProcImg[P222100_ElectrodeAge], &hours, 2);
}

/** @brief Write LastCalibrationDate [0x2222] — uint32_t (Unix timestamp) */
static inline void ProcImg_SetLastCalibrationDate(uint32_t timestamp) {
	memcpy(&gProcImg[P222200_LastCalibrationDate], &timestamp, 4);
}

/** @brief Write LEDControl [0x2000] — uint8_t */
static inline void ProcImg_SetLEDControl(uint8_t mode) {
	gProcImg[P200000_LEDControl] = mode;
}

/* ========================================================================== */
/* CALIBRATION BUFFER SETTERS (for firmware-side writes during calibration)    */
/* ========================================================================== */

/** @brief Write CalibrationBuffer4 [0x2202] — int16_t */
static inline void ProcImg_SetCalibrationBuffer4(int16_t mv) {
	memcpy(&gProcImg[P220200_CalibrationBuffer4], &mv, 2);
}

/** @brief Write CalibrationBuffer7 [0x2203] — int16_t */
static inline void ProcImg_SetCalibrationBuffer7(int16_t mv) {
	memcpy(&gProcImg[P220300_CalibrationBuffer7], &mv, 2);
}

/** @brief Write CalibrationBuffer10 [0x2204] — int16_t */
static inline void ProcImg_SetCalibrationBuffer10(int16_t mv) {
	memcpy(&gProcImg[P220400_CalibrationBuffer10], &mv, 2);
}

#endif /* PROCIMG_API_H */
