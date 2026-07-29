/**************************************************************************
 MODULE:    PROCIMG_API
 CONTAINS:  Typed inline accessors for the MCO process image (gProcImg[]).
 All offsets taken from auto-generated pimg.h.
 Little-endian byte order (ARM Cortex-M4, matches CAN byte order).
 NOTE:      This file provides a clean API so application code never
 accesses gProcImg[] directly with magic numbers.
 ***************************************************************************/

#ifndef _PROCIMG_API_H
#define _PROCIMG_API_H

#include <stdint.h>
#include "pimg.h"

/* External reference to the MCO process image */
extern uint8_t gProcImg[];

/**************************************************************************
 * CiA 408 Application Objects — PDO-mapped
 **************************************************************************/

/* ControlWord [0x6040,00] — RPDO1 (master → valve), UINT16
 * Offset: P604000_ControlWord = 0x00 */
static inline uint16_t ProcImg_GetControlWord(void) {
	return (uint16_t) gProcImg[P604000_ControlWord]
			| ((uint16_t) gProcImg[P604000_ControlWord + 1] << 8);
}

/* StatusWord [0x6041,00] — TPDO1 (valve → master), UINT16
 * Offset: P604100_StatusWord = 0x04 */
static inline uint16_t ProcImg_GetStatusWord(void) {
	return (uint16_t) gProcImg[P604100_StatusWord]
			| ((uint16_t) gProcImg[P604100_StatusWord + 1] << 8);
}

static inline void ProcImg_SetStatusWord(uint16_t val) {
	gProcImg[P604100_StatusWord] = (uint8_t) (val & 0xFF);
	gProcImg[P604100_StatusWord + 1] = (uint8_t) ((val >> 8) & 0xFF);
}

/* ValveState [0x6042,00] — TPDO1 (valve → master), UINT8
 * Offset: P604200_ValveState = 0x06 */
static inline uint8_t ProcImg_GetValveState(void) {
	return gProcImg[P604200_ValveState];
}

static inline void ProcImg_SetValveState(uint8_t val) {
	gProcImg[P604200_ValveState] = val;
}

/**************************************************************************
 * Error Register [0x1001,00] — TPDO2 (valve → master), UINT8
 * Offset: P100100_Error_Register = 0x08
 **************************************************************************/
static inline uint8_t ProcImg_GetErrorRegister(void) {
	return gProcImg[P100100_Error_Register];
}

static inline void ProcImg_SetErrorRegister(uint8_t val) {
	gProcImg[P100100_Error_Register] = val;
}

/**************************************************************************
 * Manufacturer-Specific Objects — SDO access
 **************************************************************************/

/* LEDControl [0x2000,00] — UINT8
 * Offset: P200000_LEDControl = 0x3E */
static inline uint8_t ProcImg_GetLEDControl(void) {
	return gProcImg[P200000_LEDControl];
}

static inline void ProcImg_SetLEDControl(uint8_t val) {
	gProcImg[P200000_LEDControl] = val;
}

/* FailSafePosition [0x2100,00] — UINT8
 * Offset: P210000_FailSafePosition = 0x3F
 * Values: 0=AS_IS, 1=OPEN, 2=CLOSED */
static inline uint8_t ProcImg_GetFailSafePosition(void) {
	return gProcImg[P210000_FailSafePosition];
}

static inline void ProcImg_SetFailSafePosition(uint8_t val) {
	gProcImg[P210000_FailSafePosition] = val;
}

/* ManualOverride [0x2101,00] — Boolean (UINT8)
 * Offset: P210100_ManaualOverride = 0x40 */
static inline uint8_t ProcImg_GetManualOverride(void) {
	return gProcImg[P210100_ManaualOverride];
}

/* MotionTimeout [0x2300,00] — UINT16 (seconds)
 * Offset: P230000_MotionTimeout = 0x41 */
static inline uint16_t ProcImg_GetMotionTimeout(void) {
	return (uint16_t) gProcImg[P230000_MotionTimeout]
			| ((uint16_t) gProcImg[P230000_MotionTimeout + 1] << 8);
}

static inline void ProcImg_SetMotionTimeout(uint16_t val) {
	gProcImg[P230000_MotionTimeout] = (uint8_t) (val & 0xFF);
	gProcImg[P230000_MotionTimeout + 1] = (uint8_t) ((val >> 8) & 0xFF);
}

#endif // _PROCIMG_API_H
