/**
 ******************************************************************************
 * @file    procimg_api.h
 * @brief   Typed getter/setter API for MCO process image access
 * @note    Isolates gProcImg[] byte layout in one place so that
 *          motor_control.c (and any future consumers) are agnostic to MCO
 *          internals.  All functions are static inline — zero call overhead.
 ******************************************************************************
 */

#ifndef PROCIMG_API_H
#define PROCIMG_API_H

#include <stdint.h>
#include <string.h>
#include "procimg.h"       /* P604000_ControlWord, P604100_StatusWord, … */
#include "mcop_inc.h"      /* MEM_PROC, gProcImg[] declaration */

/* The MCO stack owns gProcImg[]; we reference it here so callers don't need to. */
extern uint8_t MEM_PROC gProcImg[];

/* ---- GETTERS (RPDO / SDO → application) --------------------------------- */

/** @brief Read ControlWord [0x6040] — uint16_t */
static inline uint16_t ProcImg_GetControlWord(void) {
	uint16_t val;
	memcpy(&val, &gProcImg[P604000_ControlWord], 2);
	return val;
}

/** @brief Read TargetFlowRate [0x6042] — int16_t (signed for bidirection) */
static inline int16_t ProcImg_GetTargetFlowRate(void) {
	int16_t val;
	memcpy(&val, &gProcImg[P604200_TargetFlowRate], 2);
	return val;
}

/** @brief Read DoseVolume [0x2300] — uint32_t (ml × 1000) */
static inline uint32_t ProcImg_GetDoseVolume(void) {
	uint32_t val;
	memcpy(&val, &gProcImg[P230000_DoseVolume], 4);
	return val;
}

/** @brief Read DoseFlowRate [0x2301] — int16_t (ml/min, signed) */
static inline int16_t ProcImg_GetDoseFlowRate(void) {
	int16_t val;
	memcpy(&val, &gProcImg[P230100_DoseFlowRate], 2);
	return val;
}

/** @brief Read DoseTimeout [0x2305] — uint16_t (seconds) */
static inline uint16_t ProcImg_GetDoseTimeout(void) {
	uint16_t val;
	memcpy(&val, &gProcImg[P230500_DoseTimeout], 2);
	return val;
}

/* ---- SETTERS (application → TPDO / SDO) --------------------------------- */

/** @brief Write StatusWord [0x6041] — uint16_t */
static inline void ProcImg_SetStatusWord(uint16_t sw) {
	memcpy(&gProcImg[P604100_StatusWord], &sw, 2);
}

/** @brief Write ActualFlowRate [0x6043] — int16_t */
static inline void ProcImg_SetActualFlowRate(int16_t flow) {
	memcpy(&gProcImg[P604300_ActualFlowRate], &flow, 2);
}

/** @brief Write PumpState [0x2400] — uint8_t */
static inline void ProcImg_SetPumpState(uint8_t state) {
	gProcImg[P240000_PumpState] = state;
}

/** @brief Write ErrorRegister [0x1001] — uint8_t */
static inline void ProcImg_SetErrorRegister(uint8_t err) {
	gProcImg[P100100_Error_Register] = err;
}

/** @brief Write DoseStatus [0x2303] — uint8_t */
static inline void ProcImg_SetDoseStatus(uint8_t status) {
	gProcImg[P230300_DoseStatus] = status;
}

/** @brief Write DoseDelivered [0x2304] — uint32_t (ml × 1000) */
static inline void ProcImg_SetDoseDelivered(uint32_t delivered) {
	memcpy(&gProcImg[P230400_DoseDelivered], &delivered, 4);
}

/** @brief Write DoseVolume [0x2300] — uint32_t (ml × 1000) */
static inline void ProcImg_SetDoseVolume(uint32_t volume) {
	memcpy(&gProcImg[P230000_DoseVolume], &volume, 4);
}

/* ---- CALIBRATION -------------------------------------------------------- */

/** @brief Read FlowCorrectionFactor [0x2200] — Real32 (IEEE 754 float) */
static inline float ProcImg_GetFlowCorrectionFactor(void) {
	float val;
	memcpy(&val, &gProcImg[P220000_FlowCorrectionFactor], 4);
	return val;
}

/* ---- LED CONTROL (TC-04) ------------------------------------------------ */

/** @brief Read LEDControl [0x2000] — uint8_t */
static inline uint8_t ProcImg_GetLEDControl(void) {
	return gProcImg[P200000_LEDControl];
}

/** @brief Write LEDControl [0x2000] — uint8_t */
static inline void ProcImg_SetLEDControl(uint8_t mode) {
	gProcImg[P200000_LEDControl] = mode;
}

#endif /* PROCIMG_API_H */
