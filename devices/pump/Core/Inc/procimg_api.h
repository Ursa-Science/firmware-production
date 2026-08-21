/**
 ******************************************************************************
 * @file    procimg_api.h
 * @brief   Typed getter/setter API for MCO process image access
 * @note    Isolates gProcImg[] byte layout in one place so that
 *          motor_control.c (and any future consumers) are agnostic to MCO
 *          internals.  All functions are static inline — zero call overhead.
 *
 *          Step-counter OD (dose engine stripped, 2026-08): the pump is a
 *          "dumb" CiA 402 stepper — it speaks native microsteps and
 *          microsteps/sec only; ALL ml math + tubing calibration lives in the
 *          MIK. RPDO1 = ControlWord + StepRate + TargetSteps (8 B atomic);
 *          TPDO1 = StatusWord + ActualStepRate; TPDO2 = PumpState +
 *          ErrorRegister + StepsRemaining.
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

/** @brief Read StepRate [0x2600] — int16_t signed µsteps/sec (sign = direction) */
static inline int16_t ProcImg_GetStepRate(void) {
	int16_t val;
	memcpy(&val, &gProcImg[P260000_StepRate], 2);
	return val;
}

/** @brief Read TargetSteps [0x2601] — uint32_t (µsteps to run then auto-stop; 0 = jog) */
static inline uint32_t ProcImg_GetTargetSteps(void) {
	uint32_t val;
	memcpy(&val, &gProcImg[P260100_TargetSteps], 4);
	return val;
}

/* ---- SETTERS (application → TPDO / SDO) --------------------------------- */

/** @brief Write StatusWord [0x6041] — uint16_t */
static inline void ProcImg_SetStatusWord(uint16_t sw) {
	memcpy(&gProcImg[P604100_StatusWord], &sw, 2);
}

/** @brief Write ActualStepRate [0x2602] — int16_t signed µsteps/sec */
static inline void ProcImg_SetActualStepRate(int16_t rate) {
	memcpy(&gProcImg[P260200_ActualStepRate], &rate, 2);
}

/** @brief Write StepsRemaining [0x2603] — uint32_t (countdown to auto-stop) */
static inline void ProcImg_SetStepsRemaining(uint32_t remaining) {
	memcpy(&gProcImg[P260300_StepsRemaining], &remaining, 4);
}

/** @brief Write TargetSteps [0x2601] — uint32_t (consume-on-latch: zero after arming) */
static inline void ProcImg_SetTargetSteps(uint32_t steps) {
	memcpy(&gProcImg[P260100_TargetSteps], &steps, 4);
}

/** @brief Write PumpState [0x2400] — uint8_t */
static inline void ProcImg_SetPumpState(uint8_t state) {
	gProcImg[P240000_PumpState] = state;
}

/** @brief Write ErrorRegister [0x1001] — uint8_t */
static inline void ProcImg_SetErrorRegister(uint8_t err) {
	gProcImg[P100100_Error_Register] = err;
}

/* ---- FAULT FEEDBACK (0x2500-0x2503, SDO-only diagnostics) -------------- */

/** @brief Write TMCInitStatus [0x2500] — uint8_t (TMC2209_Status_t at boot) */
static inline void ProcImg_SetTMCInitStatus(uint8_t status) {
	gProcImg[P250000_TMCInitStatus] = status;
}

/** @brief Write TMCInitFailedStep [0x2501] — uint8_t (TMC2209_InitStep_t) */
static inline void ProcImg_SetTMCInitFailedStep(uint8_t step) {
	gProcImg[P250100_TMCInitFailedStep] = step;
}

/** @brief Write LastDrvStatus [0x2502] — uint32_t (DRV_STATUS snapshot at fault) */
static inline void ProcImg_SetLastDrvStatus(uint32_t drv_status) {
	memcpy(&gProcImg[P250200_LastDrvStatus], &drv_status, 4);
}

/** @brief Write FaultSource [0x2503] — uint8_t (FAULT_SRC_* classification) */
static inline void ProcImg_SetFaultSource(uint8_t source) {
	gProcImg[P250300_FaultSource] = source;
}

/* ---- LED CONTROL -------------------------------------------------------- */

/** @brief Read LEDControl [0x2000] — uint8_t */
static inline uint8_t ProcImg_GetLEDControl(void) {
	return gProcImg[P200000_LEDControl];
}

/** @brief Write LEDControl [0x2000] — uint8_t */
static inline void ProcImg_SetLEDControl(uint8_t mode) {
	gProcImg[P200000_LEDControl] = mode;
}

#endif /* PROCIMG_API_H */
