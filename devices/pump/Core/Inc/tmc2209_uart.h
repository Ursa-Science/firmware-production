/**
 ******************************************************************************
 * @file    tmc2209_uart.h
 * @brief   TMC2209 UART Interface — Register Configuration via USART2
 * @author  CANopen Motor Control Project
 * @date    2026
 ******************************************************************************
 *
 * Two-wire full-duplex UART on USART2 to TMC2209 PDN_UART.
 * PA2 (TX) drives through R7 (1kΩ) to PDN_UART; PA3 (RX) connects
 * directly to PDN_UART. R12 (5k) provides pull-up to 3V3.
 * All TMC2209 access uses direct register-level USART2 writes —
 * no HAL UART calls, no mode switching.
 *
 * Hardware:                  3V3
 *                             |
 *                          R12 (5k)
 *                             |
 *           PA2 (TX) ---[R7 1kΩ]---+--- PDN_UART
 *           PA3 (RX) --------------+
 *
 * After the first valid UART datagram, the TMC2209 switches PDN_UART from
 * standby-control mode to UART mode. Subsequent HIGH idle (debug traffic)
 * does NOT trigger standby. (TMC2209 Datasheet §5.3)
 *
 * Protocol: 8-byte datagrams, CRC8 (polynomial 0x07, LSB-first)
 *   Write: [0x05] [addr] [reg|0x80] [d3] [d2] [d1] [d0] [CRC]
 *   Read:  [0x05] [addr] [reg]      [CRC]  → reply: 8 bytes
 *
 ******************************************************************************
 */

#ifndef TMC2209_UART_H
#define TMC2209_UART_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

/* -------------------------------------------------------------------------- */
/*  TMC2209 UART Slave Address                                                */
/* -------------------------------------------------------------------------- */
/* Determined by MS1/MS2 pin state at power-up:                               */
/*   MS1=0, MS2=0 → addr 0x00    MS1=1, MS2=0 → addr 0x01                   */
/*   MS1=0, MS2=1 → addr 0x02    MS1=1, MS2=1 → addr 0x03                   */
#define TMC2209_SLAVE_ADDR      0x00  /* MS1=0, MS2=0 */
#define TMC2209_SYNC_BYTE       0x05

/* -------------------------------------------------------------------------- */
/*  TMC2209 Register Addresses                                                */
/* -------------------------------------------------------------------------- */
/* General */
#define TMC2209_GCONF           0x00
#define TMC2209_GSTAT           0x01
#define TMC2209_IFCNT           0x02  /* Interface transmission counter */

/* Velocity-dependent control */
#define TMC2209_IHOLD_IRUN      0x10
#define TMC2209_TPOWERDOWN      0x11
#define TMC2209_TSTEP           0x12  /* Measured time between steps */
#define TMC2209_TPWMTHRS        0x13  /* StealthChop/SpreadCycle velocity threshold */
#define TMC2209_TCOOLTHRS       0x14  /* CoolStep & StallGuard enable velocity */
#define TMC2209_VACTUAL         0x22

/* StallGuard */
#define TMC2209_SGTHRS          0x40  /* StallGuard threshold (0-255) */
#define TMC2209_SG_RESULT       0x41  /* StallGuard result (0-510) */
#define TMC2209_COOLCONF        0x42

/* Chopper control */
#define TMC2209_CHOPCONF        0x6C
#define TMC2209_DRV_STATUS      0x6F

/* -------------------------------------------------------------------------- */
/*  GCONF Register Bits (0x00)                                                */
/* -------------------------------------------------------------------------- */
#define GCONF_I_SCALE_ANALOG    (1 << 0)  /* 1=use VREF pin for current scale */
#define GCONF_INTERNAL_RSENSE   (1 << 1)  /* 1=use internal sense resistors   */
#define GCONF_EN_SPREADCYCLE    (1 << 2)  /* 1=SpreadCycle, 0=StealthChop     */
#define GCONF_SHAFT             (1 << 3)  /* 1=invert motor direction          */
#define GCONF_INDEX_OTPW        (1 << 4)
#define GCONF_INDEX_STEP        (1 << 5)
#define GCONF_PDN_DISABLE       (1 << 6)  /* 1=disable PDN standby function   */
#define GCONF_MSTEP_REG_SELECT  (1 << 7)  /* 1=microstep set by CHOPCONF reg  */
#define GCONF_MULTISTEP_FILT    (1 << 8)  /* 1=filter step input pulses       */

/* -------------------------------------------------------------------------- */
/*  CHOPCONF Register Fields (0x6C)                                           */
/* -------------------------------------------------------------------------- */
#define CHOPCONF_TOFF_SHIFT     0   /* Off-time (bits 0-3)        */
#define CHOPCONF_TOFF_MASK      0x0F
#define CHOPCONF_HSTRT_SHIFT    4   /* Hysteresis start (bits 4-6) */
#define CHOPCONF_HSTRT_MASK     0x07
#define CHOPCONF_HEND_SHIFT     7   /* Hysteresis end (bits 7-10)  */
#define CHOPCONF_HEND_MASK      0x0F
#define CHOPCONF_TBL_SHIFT      15  /* Blank time (bits 15-16)     */
#define CHOPCONF_TBL_MASK       0x03
#define CHOPCONF_MRES_SHIFT     24  /* Microstep resolution (bits 24-27) */
#define CHOPCONF_MRES_MASK      0x0F
#define CHOPCONF_INTPOL         (1 << 28)  /* MicroPlyer interpolation  */
#define CHOPCONF_DEDGE          (1 << 29)  /* Step on both edges        */
#define CHOPCONF_DISS2G         (1 << 30)  /* Disable short-to-GND prot */
#define CHOPCONF_DISS2VS        (1UL << 31) /* Disable short-to-VS prot */

/* CHOPCONF.mres values */
#define MRES_256                0
#define MRES_128                1
#define MRES_64                 2
#define MRES_32                 3
#define MRES_16                 4
#define MRES_8                  5
#define MRES_4                  6
#define MRES_2                  7   /* Half-step (2 microsteps)     */
#define MRES_FULLSTEP           8   /* Full step                    */

/* -------------------------------------------------------------------------- */
/*  IHOLD_IRUN Register Fields (0x10)                                         */
/* -------------------------------------------------------------------------- */
#define IHOLD_SHIFT             0   /* Standstill current (bits 0-4)   */
#define IHOLD_MASK              0x1F
#define IRUN_SHIFT              8   /* Run current (bits 8-12)         */
#define IRUN_MASK               0x1F
#define IHOLDDELAY_SHIFT        16  /* Power-down delay (bits 16-19)   */
#define IHOLDDELAY_MASK         0x0F

/* -------------------------------------------------------------------------- */
/*  DRV_STATUS Register Bits (0x6F) — Read-only                               */
/* -------------------------------------------------------------------------- */
#define DRV_STATUS_STST         (1UL << 31) /* Standstill indicator     */
#define DRV_STATUS_OLB          (1 << 30)   /* Open load phase B        */
#define DRV_STATUS_OLA          (1 << 29)   /* Open load phase A        */
#define DRV_STATUS_S2VSB        (1 << 28)   /* Short to supply phase B  */
#define DRV_STATUS_S2VSA        (1 << 27)   /* Short to supply phase A  */
#define DRV_STATUS_S2GB         (1 << 26)   /* Short to GND phase B     */
#define DRV_STATUS_S2GA         (1 << 25)   /* Short to GND phase A     */
#define DRV_STATUS_OT           (1 << 24)   /* Overtemperature           */
#define DRV_STATUS_OTPW         (1 << 23)   /* Overtemp pre-warning      */
#define DRV_STATUS_CS_MASK      0x001F0000  /* Current scale (bits 16-20)*/
#define DRV_STATUS_SG_MASK      0x000003FF  /* StallGuard result (bits 0-9) */

/* -------------------------------------------------------------------------- */
/*  Return Codes                                                              */
/* -------------------------------------------------------------------------- */
typedef enum {
	TMC2209_OK = 0, TMC2209_ERR_UART_TX, /* UART transmit failed / timeout      */
	TMC2209_ERR_UART_RX, /* UART receive failed / timeout       */
	TMC2209_ERR_CRC, /* CRC mismatch on read reply          */
	TMC2209_ERR_VERIFY, /* IFCNT or readback verification fail */
	TMC2209_ERR_NOT_INIT, /* Init not called                     */
	TMC2209_ERR_SMOKE_TEST /* Smoke test read failed at init      */
} TMC2209_Status_t;

/* -------------------------------------------------------------------------- */
/*  Init Step Tracking (for diagnostics)                                      */
/* -------------------------------------------------------------------------- */
typedef enum {
	TMC2209_INIT_STEP_NONE = 0,
	TMC2209_INIT_STEP_SMOKE_TEST,
	TMC2209_INIT_STEP_GCONF_WRITE,
	TMC2209_INIT_STEP_CHOPCONF_WRITE,
	TMC2209_INIT_STEP_IHOLD_WRITE,
	TMC2209_INIT_STEP_GCONF_READBACK,
	TMC2209_INIT_STEP_CHOPCONF_READBACK,
	TMC2209_INIT_STEP_COMPLETE
} TMC2209_InitStep_t;

/* -------------------------------------------------------------------------- */
/*  Init Diagnostics — captured during TMC2209_Init(), printed after          */
/* -------------------------------------------------------------------------- */
#define TMC_DIAG_MAX_ATTEMPTS  4   /* one per swept address (0-3) */
#define TMC_DIAG_MAX_RX_BYTES  8

/* Per-attempt diagnostic data captured by ReadRegisterDiag during smoke test */
typedef struct {
	bool echo_drain_ok; /* Did TransmitWithEchoDrain succeed?  */
	uint8_t rx_count; /* Number of RX bytes received         */
	uint8_t rx_bytes[TMC_DIAG_MAX_RX_BYTES]; /* Raw RX data            */
	uint32_t isr_at_rx_fail; /* USART2 ISR at point of RX failure   */
	TMC2209_Status_t error; /* Error code for this attempt         */
} TMC2209_DiagAttempt_t;

typedef struct {
	/* USART2 register snapshots */
	uint32_t brr; /* BRR register (baud rate divisor)    */
	uint32_t cr1; /* CR1 at init start                   */
	uint32_t cr2; /* CR2 at init start                   */
	uint32_t cr3; /* CR3 at init start                   */
	uint32_t isr_at_entry; /* ISR before any TMC traffic          */

	/* Auto-baud preamble / first GCONF write */
	bool preamble_tx_ok; /* Did TransmitDirect succeed?         */
	uint32_t isr_after_preamble; /* ISR after preamble write            */

	/* Smoke test per-attempt diagnostics */
	TMC2209_DiagAttempt_t attempt[TMC_DIAG_MAX_ATTEMPTS];

	/* Init mode */
	bool blind_fallback; /* true = RX dead, used blind writes   */

	/* MS1/MS2 pin state at init time */
	uint8_t ms1_level; /* GPIO read of MS1 pin (0 or 1)       */
	uint8_t ms2_level; /* GPIO read of MS2 pin (0 or 1)       */

	/* GPIO state */
	uint32_t gpioa_idr; /* GPIOA IDR at init start (pin levels)*/
	uint32_t gpioa_moder; /* GPIOA MODER (pin modes)             */
	uint32_t gpioa_afrl; /* GPIOA AFRL (AF selection 0-7)       */

	/* GCONF read-back after writes (verified path only; RX must work) */
	bool gconf_readback_ok; /* true = GCONF read back successfully  */
	uint32_t gconf_readback; /* actual GCONF value read after writes */

	/* Push-pull RX isolation probe (runs only when smoke read failed) */
	bool pp_echo_ok; /* true = >=1 byte echoed under push-pull */
	uint8_t pp_echo_count; /* bytes echoed back (0-4) at clean levels */

	/* GPIO bit-bang loopback (PA2 out → PA3 in, bypasses USART) */
	bool bb_tracks; /* true = PA3 tracked PA2 (pin+wire good)  */
	uint8_t bb_pa3_high; /* PA3 IDR while PA2 driven HIGH (expect 1) */
	uint8_t bb_pa3_low; /* PA3 IDR while PA2 driven LOW  (expect 0) */

	/* Address sweep (half-duplex): which UART address the TMC replied on */
	bool addr_found; /* true = a 0-3 address responded          */
	uint8_t resolved_addr; /* the responding TMC UART address (0-3)   */
} TMC2209_Diag_t;

/* -------------------------------------------------------------------------- */
/*  Public API                                                                */
/* -------------------------------------------------------------------------- */

/**
 * @brief  Initialize TMC2209 via UART on USART2
 * @note   Call BEFORE Log_Init(). Uses direct register-level USART2 TX
 *         to write TMC2209 registers. Does NOT modify USART2 config or
 *         HAL handle.
 *         Configures: SpreadCycle, 2-microstep via register, run current,
 *         and disables PDN standby.
 * @retval TMC2209_OK on success, error code on failure
 */
TMC2209_Status_t TMC2209_Init(void);

/**
 * @brief  Write a 32-bit value to a TMC2209 register
 * @param  reg     Register address (0x00–0x7F)
 * @param  value   32-bit value to write
 * @retval TMC2209_OK on success
 */
TMC2209_Status_t TMC2209_WriteRegister(uint8_t reg, uint32_t value);

/**
 * @brief  Read a 32-bit value from a TMC2209 register
 * @param  reg     Register address (0x00–0x7F)
 * @param  value   Pointer to store 32-bit result
 * @retval TMC2209_OK on success
 */
TMC2209_Status_t TMC2209_ReadRegister(uint8_t reg, uint32_t *value);

/**
 * @brief  Read StallGuard result (runtime diagnostic)
 * @param  sg_result  Pointer to store SG_RESULT value (0-510)
 * @retval TMC2209_OK on success
 */
TMC2209_Status_t TMC2209_ReadStallGuard(uint16_t *sg_result);

/**
 * @brief  Read DRV_STATUS register (runtime diagnostic)
 * @param  status  Pointer to store raw DRV_STATUS value
 * @retval TMC2209_OK on success
 */
TMC2209_Status_t TMC2209_ReadDrvStatus(uint32_t *status);

/**
 * @brief  Get which init step failed (for boot diagnostics)
 * @retval TMC2209_InitStep_t indicating the failed step,
 *         TMC2209_INIT_STEP_COMPLETE on success, TMC2209_INIT_STEP_NONE if not called.
 */
TMC2209_InitStep_t TMC2209_GetInitFailedStep(void);

/**
 * @brief  Get pointer to init diagnostic data (captured during TMC2209_Init)
 * @retval Pointer to static TMC2209_Diag_t struct (valid after TMC2209_Init returns)
 */
const TMC2209_Diag_t* TMC2209_GetDiagnostics(void);

#ifdef __cplusplus
}
#endif

#endif /* TMC2209_UART_H */
