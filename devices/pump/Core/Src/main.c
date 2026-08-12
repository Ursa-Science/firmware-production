/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "mcop_inc.h"
#include "log.h"
#include "SEGGER_RTT.h"
#include "motor_control.h"
#include "mco_events.h"
#include "stepper.h"
#include "tmc2209_uart.h"
#include <string.h>
#include <stdbool.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define FIRMWARE_VERSION "2.0.0"

// CAN bitrate selector is in main.h (see CAN_BITRATE_xxxK)

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
FDCAN_HandleTypeDef hfdcan1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim7;

UART_HandleTypeDef huart2;

IWDG_HandleTypeDef hiwdg;

/* USER CODE BEGIN PV */
// External declaration for process image
extern uint8_t MEM_PROC gProcImg[];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_FDCAN1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM7_Init(void);
static void MX_IWDG_Init(void);
/* USER CODE BEGIN PFP */
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#ifdef __GNUC__
  /* With GCC, small printf (option LD Linker->Libraries->Small printf
     set to 'Yes') calls __io_putchar() */
  #define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
  #define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */

/**
 * @brief  Printf retarget — routes to SEGGER RTT via Log_PutChar.
 *         Safe at any time (RTT self-initialises on first write), so early
 *         init banners no longer need a blocking UART fallback and printf
 *         never touches USART2 (which belongs to the TMC2209).
 */
PUTCHAR_PROTOTYPE
{
	if (ch == '\n') {
		Log_PutChar('\r');
	}
	Log_PutChar((uint8_t) ch);
	return ch;
}

/**
 * @brief  One-shot boot diagnostics — clock tree, FDCAN registers, TIM7
 * @note   Called once during init while output is still blocking.
 *         Gated by DBG_CAN_ENABLE.  Kept in main.c because it reads
 *         RCC / FDCAN1 / TIM7 peripheral handles that are local here.
 */
static void PrintBootDiagnostics(void)
{
	DBG_BLOCK(CAN) {
		/* ── Clock configuration ── */
		DBG_PRINT(CAN, "--- Clock Configuration ---");
		DBG_HW(CAN, "HSE_VALUE (expected): %lu Hz", (unsigned long)HSE_VALUE);
		DBG_HW(CAN, "SystemCoreClock: %lu Hz", (unsigned long )SystemCoreClock);

		uint32_t clksrc = RCC->CFGR & RCC_CFGR_SWS;
		switch (clksrc) {
		case RCC_CFGR_SWS_HSI:
			DBG_ERROR(CAN, "Clock Source: HSI (16 MHz) - HSE may have failed!");
			break;
		case RCC_CFGR_SWS_HSE:
			DBG_STATE(CAN, "Clock Source: HSE direct (24 MHz)");
			break;
		case RCC_CFGR_SWS_PLL:
			DBG_STATE(CAN, "Clock Source: PLL (expected 80 MHz)");
			break;
		default:
			DBG_ERROR(CAN, "Clock Source: Unknown (0x%lx)",
					(unsigned long )clksrc);
			break;
		}

		if (RCC->CR & RCC_CR_HSERDY) {
			DBG_STATE(CAN, "HSE Status: READY");
		} else {
			DBG_ERROR(CAN, "HSE Status: NOT READY - CLOCK FAILURE!");
		}

		if (RCC->CR & RCC_CR_HSEBYP) {
			DBG_STATE(CAN,
					"HSE Mode: BYPASS (external oscillator) - CORRECT for ECS-1612MV");
		} else {
			DBG_ERROR(CAN,
					"HSE Mode: Crystal - WRONG for ECS-1612MV oscillator!");
		}

		uint32_t pllm = ((RCC->PLLCFGR & RCC_PLLCFGR_PLLM)
				>> RCC_PLLCFGR_PLLM_Pos) + 1;
		uint32_t plln = (RCC->PLLCFGR & RCC_PLLCFGR_PLLN)
				>> RCC_PLLCFGR_PLLN_Pos;
		uint32_t pllr = (((RCC->PLLCFGR & RCC_PLLCFGR_PLLR)
				>> RCC_PLLCFGR_PLLR_Pos) + 1) * 2;
		(void) pllm;
		(void) plln;
		(void) pllr;
		DBG_HW(CAN, "PLL: HSE / %lu * %lu / %lu = %lu MHz",
				(unsigned long )pllm, (unsigned long )plln,
				(unsigned long )pllr,
				(unsigned long)(HSE_VALUE / pllm * plln / pllr / 1000000));

		uint32_t fdcansel = (RCC->CCIPR & RCC_CCIPR_FDCANSEL)
				>> RCC_CCIPR_FDCANSEL_Pos;
		switch (fdcansel) {
		case 0:
			DBG_STATE(CAN, "FDCAN Clock Source: HSE (24 MHz)");
			break;
		case 1:
			DBG_STATE(CAN, "FDCAN Clock Source: PLLQ (expected 80 MHz)");
			break;
		case 2:
			DBG_STATE(CAN, "FDCAN Clock Source: PCLK1");
			break;
		default:
			DBG_ERROR(CAN, "FDCAN Clock Source: Unknown");
			break;
		}
	}
}

/**
 * @brief  Post-MCO-init FDCAN register dump — bitrate, mode, error counters
 * @note   Called once after MCOUSER_ResetCommunication().  Gated by DBG_CAN_ENABLE.
 */
static void PrintFDCANDiagnostics(void) {
	DBG_BLOCK(CAN) {
		DBG_PRINT(CAN, "--- FDCAN Configuration (after MCO init) ---");

		uint32_t fdcansel_after = (RCC->CCIPR & RCC_CCIPR_FDCANSEL)
				>> RCC_CCIPR_FDCANSEL_Pos;
		switch (fdcansel_after) {
		case 0:
			DBG_ERROR(CAN, "FDCAN Clock Source (after init): HSE - WRONG!");
			break;
		case 1:
			DBG_STATE(CAN,
					"FDCAN Clock Source (after init): PLLQ (80 MHz) - CORRECT!");
			break;
		case 2:
			DBG_ERROR(CAN, "FDCAN Clock Source (after init): PCLK1 - WRONG!");
			break;
		default:
			DBG_ERROR(CAN, "FDCAN Clock Source (after init): Unknown");
			break;
		}

		DBG_HW(CAN, "FDCAN1->NBTP: 0x%08lX", (unsigned long)FDCAN1->NBTP);

		uint32_t nbrp =
				((FDCAN1->NBTP & FDCAN_NBTP_NBRP) >> FDCAN_NBTP_NBRP_Pos) + 1;
		uint32_t ntseg1 = ((FDCAN1->NBTP & FDCAN_NBTP_NTSEG1)
				>> FDCAN_NBTP_NTSEG1_Pos) + 1;
		uint32_t ntseg2 = ((FDCAN1->NBTP & FDCAN_NBTP_NTSEG2)
				>> FDCAN_NBTP_NTSEG2_Pos) + 1;
		uint32_t nsjw =
				((FDCAN1->NBTP & FDCAN_NBTP_NSJW) >> FDCAN_NBTP_NSJW_Pos) + 1;
		uint32_t tq = 1 + ntseg1 + ntseg2;
		(void) nbrp;
		(void) ntseg1;
		(void) ntseg2;
		(void) nsjw;
		(void) tq;

		DBG_HW(CAN, "Prescaler: %lu, Seg1: %lu, Seg2: %lu, SJW: %lu",
				(unsigned long )nbrp, (unsigned long )ntseg1,
				(unsigned long )ntseg2, (unsigned long )nsjw);
		DBG_HW(CAN, "Time Quanta: %lu, Sample Point: %lu.%lu%%",
				(unsigned long )tq, (unsigned long )((1 + ntseg1) * 100 / tq),
				(unsigned long )(((1 + ntseg1) * 1000 / tq) % 10));
		DBG_STATE(CAN,
				"Calculated Bitrate: %lu kb/s (assuming 80 MHz FDCAN clock)",
				(unsigned long )(80000000UL / (nbrp * tq) / 1000));

		uint32_t cccr = FDCAN1->CCCR;
		(void) cccr;
		DBG_HW(CAN, "FDCAN1->CCCR: 0x%08lX", (unsigned long )cccr);
		DBG_HW(CAN,
				"  INIT=%d, CCE=%d, ASM=%d, CSA=%d, CSR=%d, MON=%d, DAR=%d, TEST=%d",
				(int )((cccr >> 0) & 1), (int )((cccr >> 1) & 1),
				(int )((cccr >> 2) & 1), (int )((cccr >> 3) & 1),
				(int )((cccr >> 4) & 1), (int )((cccr >> 5) & 1),
				(int )((cccr >> 6) & 1), (int )((cccr >> 7) & 1));
		if (cccr & (1 << 5)) {
			DBG_ERROR(CAN,
					"  MON=1 means BUS MONITORING MODE (listen-only, cannot transmit!)");
		}
		if (cccr & (1 << 7)) {
			uint32_t test = FDCAN1->TEST;
			(void) test;
			DBG_HW(CAN, "FDCAN1->TEST: 0x%08lX (LBCK=%d)", (unsigned long )test,
					(int )((test >> 4) & 1));
			if (test & (1 << 4)) {
				DBG_ERROR(CAN,
						"  LBCK=1 means LOOPBACK MODE (internal loopback, TX not connected to bus!)");
			}
		}

		uint32_t ecr = FDCAN1->ECR;
		uint32_t psr = FDCAN1->PSR;
		(void) ecr;
		(void) psr;
		uint8_t tec = (ecr >> 0) & 0xFF;
		uint8_t rec = (ecr >> 8) & 0x7F;
		uint8_t lec = (psr >> 0) & 0x07;
		(void) tec;
		(void) rec;
		(void) lec;
		DBG_HW(CAN, "Initial Error Counters: TEC=%u, REC=%u", tec, rec);
		DBG_HW(CAN, "Protocol Status (PSR): 0x%08lX, LEC=%u",
				(unsigned long )psr, lec);
		const char *lec_str[] =
				{ "No Error", "Stuff Error", "Form Error", "ACK Error",
						"Bit1 Error", "Bit0 Error", "CRC Error", "Reserved" };
		(void) lec_str;
		DBG_STATE(CAN, "Last Error Code: %s", lec_str[lec]);
	}
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM1_Init();
  MX_USART2_UART_Init();
  MX_FDCAN1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM7_Init();
  MX_IWDG_Init();
  /* USER CODE BEGIN 2 */

	/* Configure TMC2209 via direct register-level USART2 TX/RX BEFORE any
	 * debug output.  Does NOT modify USART2 config or HAL handle.
	 * Writes GCONF (SpreadCycle, MSTEP_REG_SELECT, PDN standby off),
	 * CHOPCONF (microstep resolution) and IHOLD_IRUN (run/hold current),
	 * each IFCNT-confirmed, then reads GCONF/CHOPCONF back.
	 * TMC2209_OK means the CHIP confirmed the config, not merely that
	 * datagrams were transmitted. */
	TMC2209_Status_t tmc_status = TMC2209_Init();

	/* NOTE: init status is NOT stored in the process image — 0x2500/0x2501 do
	 * not exist in the OD yet (fault-feedback Phase 2 adds them). The writes
	 * that used to sit here targeted gProcImg[0x67/0x68], PAST the end of the
	 * process image (PIMGEND=0x66) — an out-of-bounds clobber on every boot. */

	/* (The old 0x0F/ESC(B terminal-reset hack is gone: logging now goes out
	 * over RTT/SWD, so the console never sees TMC2209 datagram bytes and
	 * USART2 carries TMC traffic only.) */
	Log_Init(&huart2);

	/* ── IWDG reset detection ── */
	if (__HAL_RCC_GET_FLAG(RCC_FLAG_IWDGRST)) {
		printf("*** IWDG RESET DETECTED — previous main loop stall ***\r\n");
	}
	__HAL_RCC_CLEAR_RESET_FLAGS();
	printf("IWDG enabled, ~2048ms timeout\r\n");

	DBG_BLOCK(GENERAL) {
		DBG_PRINT(GENERAL, "=== CANopen Motor Control Node ===");
		DBG_PRINT(GENERAL, "Firmware: %s", FIRMWARE_VERSION);
		if (tmc_status == TMC2209_OK) {
			const TMC2209_Diag_t *drb = TMC2209_GetDiagnostics();
			DBG_PRINT(GENERAL,
					"TMC2209 init: OK — CONFIG VERIFIED BY CHIP (addr=0x%02X, %s)",
					drb->resolved_addr,
					(drb->config_baud_div == 1) ?
							"debug baud" : "fallback ~9600 baud");
			DBG_PRINT(GENERAL, "  GCONF    readback=0x%08lX MATCH",
					(unsigned long )drb->gconf_readback);
			DBG_PRINT(GENERAL, "  CHOPCONF readback=0x%08lX MATCH",
					(unsigned long )drb->chopconf_readback);

			/* Chopper mode reported from the READ-BACK, not from the macro —
			 * this is the ground truth the chip is actually running. */
			DBG_PRINT(GENERAL, "  Chopper: %s%s",
					(drb->gconf_readback & GCONF_EN_SPREADCYCLE) ?
							"SpreadCycle" : "StealthChop",
					(TMC_CHOPPER_MODE == TMC_CHOPPER_HYBRID) ?
							" (hybrid: SpreadCycle above TPWMTHRS)" : "");
			DBG_PRINT(GENERAL,
					"  Current: IRUN=%u (%u mA pk) IHOLD=%u (%u mA pk) %s",
					TMC_IRUN_SETTING,
					(unsigned )((TMC_IRUN_SETTING + 1u) * 2500u / 32u),
					TMC_IHOLD_SETTING,
					(unsigned )((TMC_IHOLD_SETTING + 1u) * 2500u / 32u),
					drb->ihold_write_ok ?
							"IFCNT-confirmed" : "*** UNCONFIRMED ***");
			DBG_PRINT(GENERAL, "  TPOWERDOWN/TPWMTHRS: %s",
					drb->timing_write_ok ?
							"IFCNT-confirmed" : "*** UNCONFIRMED ***");

			/* DRV_STATUS at standstill: OT/OTPW and the short flags are
			 * meaningful here; OLA/OLB (open load) are NOT — they only mean
			 * something while the motor is turning. */
			if (drb->drv_status_ok) {
				uint32_t ds = drb->drv_status;
				DBG_PRINT(GENERAL,
						"  DRV_STATUS=0x%08lX CS_actual=%lu%s%s%s%s",
						(unsigned long )ds,
						(unsigned long )((ds & DRV_STATUS_CS_MASK) >> 16),
						(ds & DRV_STATUS_OT) ? " OVERTEMP!" : "",
						(ds & DRV_STATUS_OTPW) ? " otpw(pre-warn)" : "",
						((ds & (DRV_STATUS_S2GA | DRV_STATUS_S2GB)) ?
								" SHORT-TO-GND!" : ""),
						((ds & (DRV_STATUS_S2VSA | DRV_STATUS_S2VSB)) ?
								" SHORT-TO-VS!" : ""));
			} else {
				DBG_PRINT(GENERAL, "  DRV_STATUS unavailable");
			}
		} else {
			static const char *step_names[] = { [TMC2209_INIT_STEP_NONE
					] = "NONE", [TMC2209_INIT_STEP_SMOKE_TEST] = "SMOKE_TEST",
					[TMC2209_INIT_STEP_GCONF_WRITE] = "GCONF_WRITE",
					[TMC2209_INIT_STEP_CHOPCONF_WRITE] = "CHOPCONF_WRITE",
					[TMC2209_INIT_STEP_IHOLD_WRITE] = "IHOLD_WRITE",
					[TMC2209_INIT_STEP_GCONF_READBACK] = "GCONF_READBACK",
					[TMC2209_INIT_STEP_CHOPCONF_READBACK] = "CHOPCONF_READBACK",
					[TMC2209_INIT_STEP_COMPLETE] = "COMPLETE", };
			const TMC2209_Diag_t *dfb = TMC2209_GetDiagnostics();
			TMC2209_InitStep_t step = TMC2209_GetInitFailedStep();
			const char *sname =
					(step <= TMC2209_INIT_STEP_COMPLETE) ?
							step_names[step] : "UNKNOWN";
			DBG_ERROR(GENERAL,
					"TMC2209 init: *** CONFIG NOT VERIFIED *** failed at step %s (err=%d)",
					sname, (int )tmc_status);
			if (!dfb->addr_found) {
				DBG_ERROR(GENERAL,
						"  No TMC2209 answered an IFCNT read on addr 0-3 — RX path is dead or chip not in UART mode");
			} else {
				DBG_ERROR(GENERAL,
						"  TMC2209 answered on addr=0x%02X but the config could not be confirmed",
						dfb->resolved_addr);
			}
			if (dfb->blind_fallback) {
				DBG_ERROR(GENERAL,
						"  Config was written UNVERIFIED as a last resort. Motor will run, but:");
				DBG_ERROR(GENERAL,
						"    - SpreadCycle / microstep resolution are NOT confirmed");
				DBG_ERROR(GENERAL,
						"    - IRUN/IHOLD are NOT confirmed: if GCONF was lost, I_SCALE_ANALOG");
				DBG_ERROR(GENERAL,
						"      defaults to 1 and phase current comes from the module VREF pot");
			}
		}

		/* ── TMC2209 UART diagnostic dump (IFCNT-verified init) ── */
		{
			const TMC2209_Diag_t *d = TMC2209_GetDiagnostics();

			DBG_PRINT(GENERAL, "--- TMC2209 UART Diagnostics ---");

			/* MS1/MS2 nominally set the slave address; the real address is
			 * discovered by IFCNT read, so a mismatch here is informative
			 * (it means the MS pins do not reach the driver module). */
			uint8_t expected_addr = (d->ms2_level << 1) | d->ms1_level;
			DBG_PRINT(GENERAL, "MS1=%u MS2=%u -> addr=0x%02X (pin-implied)",
					d->ms1_level, d->ms2_level, expected_addr);
			if (d->addr_found) {
				DBG_PRINT(GENERAL, "Addr probe: TMC replied on 0x%02X -> %s",
						d->resolved_addr,
						(d->resolved_addr == expected_addr) ?
								"matches MS pins" :
								"DIFFERS from MS pins (MS routing suspect)");
			} else {
				DBG_PRINT(GENERAL,
						"Addr probe: no reply on addr 0-3 (RX path dead)");
			}

			DBG_PRINT(GENERAL, "Preamble TX: %s",
					d->preamble_tx_ok ? "OK" : "FAIL");

			DBG_PRINT(GENERAL, "USART2: BRR=0x%04lX CR1=0x%08lX CR3=0x%08lX",
					(unsigned long )d->brr, (unsigned long )d->cr1,
					(unsigned long )d->cr3);
			DBG_PRINT(GENERAL, "  ISR@entry=0x%08lX ISR@postPreamble=0x%08lX",
					(unsigned long )d->isr_at_entry,
					(unsigned long )d->isr_after_preamble);
			DBG_PRINT(GENERAL, "  GPIOA IDR=0x%08lX MODER=0x%08lX",
					(unsigned long )d->gpioa_idr,
					(unsigned long )d->gpioa_moder);
			if (d->config_verified) {
				DBG_PRINT(GENERAL,
						"  MODE: IFCNT-verified writes + read-back (RX OK, BRR x%u)",
						d->config_baud_div);
			} else {
				DBG_PRINT(GENERAL,
						"  MODE: UNVERIFIED blind writes — verification failed on every baud");
			}
			DBG_PRINT(GENERAL, "--- end TMC2209 diag ---");
		}
	}
	PrintBootDiagnostics();

	// Start stack timer
	gMcopTimHandle_p = &htim7;
	(void) HAL_TIM_Base_Start_IT(gMcopTimHandle_p);

	DBG_BLOCK(CAN) {
		extern volatile uint16_t gTimCnt;
		DBG_STATE(CAN, "TIM7 started: Prescaler=%lu, Period=%lu",
				(unsigned long)htim7.Init.Prescaler + 1,
				(unsigned long)htim7.Init.Period + 1);
		DBG_STATE(CAN, "Expected tick rate: %lu kHz, Initial gTimCnt=%u",
				(unsigned long)(80000000UL / (htim7.Init.Prescaler + 1) / (htim7.Init.Period + 1) / 1000),
				gTimCnt);
	}

	// Initialize event dispatcher and register motor control listener
	// MUST happen before MCOUSER_ResetCommunication so the COMM_RESET
	// event handler is in place for the first stack init.
	MCO_Events_Init();
	MotorControl_RegisterMCOEvents();

	// MCO stack will initialize Timer 2 and motor control (via COMM_RESET event)
	MCOUSER_ResetCommunication();

	PrintFDCANDiagnostics();

	/* ── Arm the TMC2209 DIAG fault monitor (PA5 EXTI) ──
	 * Armed last, after TMC init and motor-control init, with pending flags
	 * cleared first: the TMC pulses DIAG at power-on reset (datasheet Fig 15.1)
	 * and that pulse must not fault a healthy boot. If DIAG is ALREADY latched
	 * high here, a real driver error exists at power-up — inject the event so
	 * the normal fault path handles it instead of it going unnoticed (the
	 * rising edge already happened, so the EXTI alone would never fire). */
	__HAL_GPIO_EXTI_CLEAR_IT(TMC_DIAG_Pin);
	HAL_NVIC_ClearPendingIRQ(EXTI9_5_IRQn);
	HAL_NVIC_SetPriority(EXTI9_5_IRQn, 6, 0);
	HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
	if (HAL_GPIO_ReadPin(TMC_DIAG_GPIO_Port, TMC_DIAG_Pin) == GPIO_PIN_SET) {
		DBG_ERROR(GENERAL,
				"TMC DIAG fault monitor armed: PA5=HIGH — driver fault latched at boot!");
		Stepper_NotifyDriverFault();
	} else {
		DBG_PRINT(GENERAL, "TMC DIAG fault monitor armed (PA5=LOW, healthy)");
	}

	


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		// Process CANopen stack
		MCO_ProcessStack();
		// Process motor control
		MotorControl_Process();
		MotorControl_RunDiagnostics();

		HAL_IWDG_Refresh(&hiwdg); // Kick IWDG watchdog — must execute within ~2048ms

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
	 * NOTE: Using RCC_HSE_BYPASS because the ECS-1612MV is an external HCMOS
	 * oscillator (active clock source), NOT a passive crystal.
  */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE
			| RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV3;
  RCC_OscInitStruct.PLL.PLLN = 20;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief FDCAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_FDCAN1_Init(void)
{

  /* USER CODE BEGIN FDCAN1_Init 0 */

  /* USER CODE END FDCAN1_Init 0 */

  /* USER CODE BEGIN FDCAN1_Init 1 */

  /* USER CODE END FDCAN1_Init 1 */
  hfdcan1.Instance = FDCAN1;
  hfdcan1.Init.ClockDivider = FDCAN_CLOCK_DIV1;
  hfdcan1.Init.FrameFormat = FDCAN_FRAME_CLASSIC;
  hfdcan1.Init.Mode = FDCAN_MODE_NORMAL;
	hfdcan1.Init.AutoRetransmission = ENABLE;
  hfdcan1.Init.TransmitPause = DISABLE;
  hfdcan1.Init.ProtocolException = DISABLE;

	/* --- Nominal bit timing (selected by CAN_BITRATE_xxxK define) --- */
#if defined(CAN_BITRATE_1000K)
  // 1000 kb/s @ 80 MHz: 80M / (5 × 16 TQ) = 1000 kb/s, 87.5% sample point
  hfdcan1.Init.NominalPrescaler = 5;
  hfdcan1.Init.NominalSyncJumpWidth = 2;
  hfdcan1.Init.NominalTimeSeg1 = 13;
  hfdcan1.Init.NominalTimeSeg2 = 2;
#elif defined(CAN_BITRATE_800K)
  // 800 kb/s @ 80 MHz: 80M / (4 × 25 TQ) = 800 kb/s, 88.0% sample point
  hfdcan1.Init.NominalPrescaler = 4;
  hfdcan1.Init.NominalSyncJumpWidth = 3;
  hfdcan1.Init.NominalTimeSeg1 = 21;
  hfdcan1.Init.NominalTimeSeg2 = 3;
#elif defined(CAN_BITRATE_500K)
  // 500 kb/s @ 80 MHz: 80M / (10 × 16 TQ) = 500 kb/s, 87.5% sample point
  hfdcan1.Init.NominalPrescaler = 10;
  hfdcan1.Init.NominalSyncJumpWidth = 2;
  hfdcan1.Init.NominalTimeSeg1 = 13;
  hfdcan1.Init.NominalTimeSeg2 = 2;
#elif defined(CAN_BITRATE_250K)
  // 250 kb/s @ 80 MHz: 80M / (20 × 16 TQ) = 250 kb/s, 87.5% sample point
  hfdcan1.Init.NominalPrescaler = 20;
  hfdcan1.Init.NominalSyncJumpWidth = 2;
  hfdcan1.Init.NominalTimeSeg1 = 13;
  hfdcan1.Init.NominalTimeSeg2 = 2;
#elif defined(CAN_BITRATE_125K)
  // 125 kb/s @ 80 MHz: 80M / (40 × 16 TQ) = 125 kb/s, 87.5% sample point
  hfdcan1.Init.NominalPrescaler = 40;
  hfdcan1.Init.NominalSyncJumpWidth = 2;
  hfdcan1.Init.NominalTimeSeg1 = 13;
  hfdcan1.Init.NominalTimeSeg2 = 2;
#elif defined(CAN_BITRATE_50K)
  // 50 kb/s @ 80 MHz: 80M / (100 × 16 TQ) = 50 kb/s, 87.5% sample point
  hfdcan1.Init.NominalPrescaler = 100;
  hfdcan1.Init.NominalSyncJumpWidth = 2;
  hfdcan1.Init.NominalTimeSeg1 = 13;
  hfdcan1.Init.NominalTimeSeg2 = 2;
#elif defined(CAN_BITRATE_20K)
  // 20 kb/s @ 80 MHz: 80M / (250 × 16 TQ) = 20 kb/s, 87.5% sample point
  hfdcan1.Init.NominalPrescaler = 250;
  hfdcan1.Init.NominalSyncJumpWidth = 2;
  hfdcan1.Init.NominalTimeSeg1 = 13;
  hfdcan1.Init.NominalTimeSeg2 = 2;
#else
  #error "No CAN bitrate defined. Define one of: CAN_BITRATE_1000K, CAN_BITRATE_800K, CAN_BITRATE_500K, CAN_BITRATE_250K, CAN_BITRATE_125K, CAN_BITRATE_50K, CAN_BITRATE_20K"
#endif

	// Data phase (not used in Classic CAN, but required by HAL)
	hfdcan1.Init.DataPrescaler = 5;
  hfdcan1.Init.DataSyncJumpWidth = 2;
  hfdcan1.Init.DataTimeSeg1 = 5;
  hfdcan1.Init.DataTimeSeg2 = 2;
  hfdcan1.Init.StdFiltersNbr = 0;
  hfdcan1.Init.ExtFiltersNbr = 0;
  hfdcan1.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
  if (HAL_FDCAN_Init(&hfdcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN FDCAN1_Init 2 */

  /* USER CODE END FDCAN1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
	htim1.Init.Prescaler = 79;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
	htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.BreakAFMode = TIM_BREAK_AFMODE_INPUT;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.Break2AFMode = TIM_BREAK_AFMODE_INPUT;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4294967295;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

}

/**
  * @brief TIM7 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM7_Init(void)
{

  /* USER CODE BEGIN TIM7_Init 0 */

  /* USER CODE END TIM7_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM7_Init 1 */

  /* USER CODE END TIM7_Init 1 */
  htim7.Instance = TIM7;
  htim7.Init.Prescaler = 800-1;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim7.Init.Period = 100 - 1; // 80 MHz / 800 / 100 = 1 kHz = 1ms tick for MCO stack
  htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM7_Init 2 */

  /* USER CODE END TIM7_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart2, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart2, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
	// ENABLE_Pin deliberately separate from the batch write.
	// TMC2209 EN is ACTIVE LOW: GPIO_PIN_RESET = driver ENABLED (draws IHOLD).
	// Boot with ENABLE_Pin HIGH (disabled) so coils are de-energized until
	// firmware explicitly enables the driver via Stepper_Enable().
	HAL_GPIO_WritePin(GPIOA, MS2_Pin | MS1_Pin | DIR_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA, ENABLE_Pin, GPIO_PIN_SET); // HIGH = driver disabled at boot
	/*Configure GPIO pins : MS2_Pin MS1_Pin */
	GPIO_InitStruct.Pin = MS2_Pin | MS1_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_PULLDOWN;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/* TMC_DIAG_Pin (PA5): the TMC2209 module's DIAG output — INPUT, never an
	 * output (the old SPREAD/MS3 output config fought the module's push-pull
	 * DIAG driver on every fault). Rising-edge EXTI; pulldown so the socketed
	 * module being absent cannot float the line. The EXTI NVIC line is NOT
	 * enabled here — it is armed in main() after TMC/motor init, with pending
	 * flags cleared, so the chip's power-on DIAG pulse is discarded. */
	GPIO_InitStruct.Pin = TMC_DIAG_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
	GPIO_InitStruct.Pull = GPIO_PULLDOWN;
	HAL_GPIO_Init(TMC_DIAG_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : DIR_Pin ENABLE_Pin */
	GPIO_InitStruct.Pin = DIR_Pin | ENABLE_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pin : PB7 */
	GPIO_InitStruct.Pin = GPIO_PIN_7;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/**
 * @brief IWDG Initialization Function
 * @note  Uses LSI (~32 kHz) clock. Prescaler=32, Reload=2047 → ~2048ms timeout.
 *        Window=4095 (disabled — any refresh accepted).
 *        IWDG cannot be stopped once started; main loop MUST refresh within timeout.
 *        Error_Handler() infinite loop will trigger IWDG reset after ~2s (desired).
 * @param None
 * @retval None
 */
static void MX_IWDG_Init(void) {
	hiwdg.Instance = IWDG;
	hiwdg.Init.Prescaler = IWDG_PRESCALER_32;
	hiwdg.Init.Window = 4095;
	hiwdg.Init.Reload = 2047;
	if (HAL_IWDG_Init(&hiwdg) != HAL_OK) {
		Error_Handler();
	}
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
	/* Diagnostic output + red LED blink before halting.
	 * This function may be called very early (before Log_Init / LEDControl_Init),
	 * so we use only RTT and direct GPIO — no PWM, no UART. */

	/* ── RTT diagnostic (before IRQ disable) ── */
	/* RTT self-initialises on first write, so this is safe at any point;
	 * the message stays readable in the RAM buffer even after the halt
	 * (probe-rs/RTT viewer, or GDB symbol _SEGGER_RTT). */
	SEGGER_RTT_WriteString(0,
			"\r\n[FATAL] Error_Handler() entered — system halted\r\n");

  __disable_irq();

	/* ── IRQs off: reconfigure LED pins as plain GPIO ── */
	/* Common-anode tri-LED: GPIO LOW = LED ON, GPIO HIGH = LED OFF.
	 * Red = PA0 (was TIM2_CH1), Blue = PA4 (was TIM3_CH2), Green = PB6 (was TIM4_CH1). */
	{
		GPIO_InitTypeDef gpio = { 0 };

		/* Turn off Blue (PA4) and Green (PB6) — drive HIGH for common-anode OFF */
		gpio.Pin = GPIO_PIN_4;
		gpio.Mode = GPIO_MODE_OUTPUT_PP;
		gpio.Pull = GPIO_NOPULL;
		gpio.Speed = GPIO_SPEED_FREQ_LOW;
		HAL_GPIO_Init(GPIOA, &gpio);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET); /* Blue OFF */

		gpio.Pin = GPIO_PIN_6;
		HAL_GPIO_Init(GPIOB, &gpio);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET); /* Green OFF */

		/* Reconfigure Red (PA0) as push-pull output for software toggle */
		gpio.Pin = GPIO_PIN_0;
		HAL_GPIO_Init(GPIOA, &gpio);
	}

	/* ── Blink red LED forever (crude software delay) ── */
	/* HAL_Delay / HAL_GetTick won't work (SysTick ISR disabled).
	 * Use a volatile spin loop calibrated for ~200 ms at 80 MHz. */
  while (1)
  {
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET); /* Red ON  */
		for (volatile uint32_t i = 0; i < 800000; i++) {
			__NOP();
		}

		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET); /* Red OFF */
		for (volatile uint32_t i = 0; i < 800000; i++) {
			__NOP();
		}
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
