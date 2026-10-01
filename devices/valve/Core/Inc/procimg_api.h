/**************************************************************************
 MODULE:    PROCIMG_API
 CONTAINS:  Typed inline accessors for the MCO process image (gProcImg[]).
 All offsets taken from auto-generated pimg.h.
 Little-endian byte order (ARM Cortex-M4, matches CAN byte order).
 NOTE:      This file provides a clean API so application code never
 accesses gProcImg[] directly with magic numbers.
 Only objects the firmware actually reads or writes have accessors;
 0x2100 FailSafePosition / 0x2101 ManualOverride / 0x2300 MotionTimeout
 were dropped 2026-09-30 (VALVE_REFACTOR_PLAN.md) so this file builds
 against both the current OD and the regenerated 21-object OD.
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
 * Offset: P604000_ControlWord */
static inline uint16_t ProcImg_GetControlWord(void) {
	return (uint16_t) gProcImg[P604000_ControlWord]
			| ((uint16_t) gProcImg[P604000_ControlWord + 1] << 8);
}

/* Written by the application only from ValveControl_ForceClosed(): RPDO1 is
 * event-driven and only lands in the process image in NMT OP, so the buffered
 * ControlWord survives PRE-OP. Zeroing it (together with the FSM's edge
 * detector) is what stops a stale Open re-firing when the master returns. */
static inline void ProcImg_SetControlWord(uint16_t val) {
	gProcImg[P604000_ControlWord] = (uint8_t) (val & 0xFF);
	gProcImg[P604000_ControlWord + 1] = (uint8_t) ((val >> 8) & 0xFF);
}

/* StatusWord [0x6041,00] — TPDO1 (valve → master), UINT16
 * Offset: P604100_StatusWord */
static inline uint16_t ProcImg_GetStatusWord(void) {
	return (uint16_t) gProcImg[P604100_StatusWord]
			| ((uint16_t) gProcImg[P604100_StatusWord + 1] << 8);
}

static inline void ProcImg_SetStatusWord(uint16_t val) {
	gProcImg[P604100_StatusWord] = (uint8_t) (val & 0xFF);
	gProcImg[P604100_StatusWord + 1] = (uint8_t) ((val >> 8) & 0xFF);
}

/* ValveState [0x6042,00] — TPDO1 (valve → master), UINT8
 * Offset: P604200_ValveState */
static inline uint8_t ProcImg_GetValveState(void) {
	return gProcImg[P604200_ValveState];
}

static inline void ProcImg_SetValveState(uint8_t val) {
	gProcImg[P604200_ValveState] = val;
}

/**************************************************************************
 * Error Register [0x1001,00] — TPDO2 (valve → master), UINT8
 * Offset: P100100_Error_Register
 * NOTE: this is only ONE of the two homes of 0x1001. SDO reads and the EMCY
 * frame's error-register byte are served from gMCOConfig.error_register
 * (mco.c). valve_control writes both and mirrors the merged register here.
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
 * Offset: P200000_LEDControl */
static inline uint8_t ProcImg_GetLEDControl(void) {
	return gProcImg[P200000_LEDControl];
}

static inline void ProcImg_SetLEDControl(uint8_t val) {
	gProcImg[P200000_LEDControl] = val;
}

#endif // _PROCIMG_API_H
