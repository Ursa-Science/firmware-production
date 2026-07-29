/**
 ******************************************************************************
 * @file    sensor_control.h
 * @brief   Sensor Control Bridge for MCO Stack Integration (CiA 404)
 * @note    Mirrors motor_control.h from the pump module.
 *          Bridges pH/temperature sensor drivers with the CANopen stack
 *          via process image and MCO event dispatcher.
 * @date    2026
 ******************************************************************************
 */

#ifndef SENSOR_CONTROL_H
#define SENSOR_CONTROL_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include <stdbool.h>
#include "main.h"

/* Exported types ------------------------------------------------------------*/

/**
 * @brief Sensor control state machine states (CiA 404)
 */
typedef enum {
	SENSOR_STATE_DISABLED = 0, /**< NMT not operational, sensors idle       */
	SENSOR_STATE_WARMING_UP, /**< Post-boot sensor stabilization          */
	SENSOR_STATE_RUNNING, /**< Active measurement + TPDO transmission  */
	SENSOR_STATE_FAULT /**< Sensor error detected                   */
} SensorState_t;

/* StatusWord bit definitions (0x6041) — CiA 404 */
#define SW_PH_READY         (1u << 0)   /**< pH sensor has valid reading       */
#define SW_TEMP_READY       (1u << 1)   /**< Temperature sensor has valid rdg  */
#define SW_CALIBRATING      (1u << 2)   /**< Calibration in progress           */
#define SW_FAULT            (1u << 3)   /**< General fault active              */
#define SW_PH_FAULT         (1u << 4)   /**< pH sensor fault                   */
#define SW_TEMP_FAULT       (1u << 5)   /**< Temperature sensor fault          */
#define SW_WARMING_UP       (1u << 6)   /**< Post-boot warmup in progress      */
#define SW_REMOTE           (1u << 9)   /**< NMT Operational (remote ready)    */

/* ControlWord bit definitions (0x6040) */
#define CW_START_PH_CAL     (1u << 0)   /**< Trigger pH calibration            */
#define CW_START_TEMP_CAL   (1u << 1)   /**< Trigger temperature offset cal    */
#define CW_FAULT_RESET      (1u << 7)   /**< Clear fault, return to DISABLED   */

/* Warmup timeout */
#define WARMUP_TIME_MS      2000u       /**< Post-boot stabilization period    */

/* Exported functions --------------------------------------------------------*/

/**
 * @brief Initialize sensor control bridge
 * @retval true if successful
 * @note  Called from MCO_EVENT_COMM_RESET handler (same as pump's MotorControl_Init)
 */
bool SensorControl_Init(void);

/**
 * @brief Process sensor control (call from MCO main loop)
 * @note  Reads sensors, updates process image, checks delta thresholds.
 *        Must be called every main loop iteration.
 */
void SensorControl_Process(void);

/**
 * @brief Check if sensor control is operational
 * @retval true if sensors are actively measuring
 */
bool SensorControl_IsOperational(void);

/**
 * @brief Emergency stop from MCO error handling
 * @note  Puts sensors into safe state, clears outputs
 */
void SensorControl_EmergencyStop(void);

/**
 * @brief Reset sensor control on NMT state change
 * @note  Called when NMT transitions to PRE-OP or STOP
 */
void SensorControl_Reset(void);

/**
 * @brief Register sensor control as MCO event listener
 * @note  Call from main() before MCOUSER_ResetCommunication()
 */
void SensorControl_RegisterMCOEvents(void);

/**
 * @brief Run periodic diagnostics (throttled internally)
 * @note  Call from the main while(1) loop.
 *        Prints sensor values, state, timing at 1 Hz.
 */
void SensorControl_RunDiagnostics(void);

/**
 * @brief Get current sensor control state
 * @retval Current SensorState_t
 */
SensorState_t SensorControl_GetState(void);

#ifdef __cplusplus
}
#endif

#endif /* SENSOR_CONTROL_H */
