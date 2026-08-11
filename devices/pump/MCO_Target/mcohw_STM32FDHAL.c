/**************************************************************************
MODULE:    MCOHW_STM
CONTAINS:  Driver implementation for ST STM32 derivatives with
           CAN FD interface. Compiled and Tested with Keil Tools www.keil.com
COPYRIGHT: Embedded Systems Academy (EmSA) 2002-2024
           All rights reserved. esacademy.com
DISCLAIM:  Read and understand our disclaimer before using this code!
           www.esacademy.com/disclaim.htm
           This software was written in accordance to the guidelines at
           www.esacademy.com/software/softwarestyleguide.pdf
LICENSE:   THIS IS THE COMMERCIAL VERSION OF MICRO CANOPEN PLUS
           ONLY USERS WHO PURCHASED A LICENSE MAY USE THIS SOFTWARE
VERSION:   7.17, EmSA 04-MAR-24
           $LastChangedDate: 2024-03-04 12:55:21 +0100 (Mon, 04 Mar 2024) $
           $LastChangedRevision: 5559 $
***************************************************************************/

// With MCO stack version up to 7, this is a also pulls in other needed
// include files and defines macros for compatibility with single-
// instance environments.
// With MCO stack version 8 and up, this also includes "mchohw_cancfg.h"
// to define macros for compatibility with multi-instance environments.
#include "mcoinst.h"

#if !defined(USE_FDHAL)
#error "This driver only supports the STM32 CAN FD HAL, for parts without CAN FD use mcohw_STM32HAL.c instead"
#endif

// Filters to use for standard receive. The last ones are reserved for receive-all
// filter for RXFIFO1 which is used in manager applications.
#if defined(FDCAN_TYPE_G) || defined(FDCAN_TYPE_U) || (defined(FDCAN_TYPE_H) && !defined(FDCAN_TYPE_H7))
  #define RXFILTER_NUM(canindex,use_fd) 28    // FDCAN controllers in STM32Gx and STM32Ux devices have less message RAM
#else
  #define RXFILTER_NUM(canindex,use_fd) 128   // FDCAN controllers in STM32Hx devices have more message RAM
#endif // defined(FDCAN_TYPE_G)
#define FIRST_RXFILTER_NUM(canindex,use_fd) 0
#define LAST_RXFILTER_NUM(canindex,use_fd) (RXFILTER_NUM(canindex,use_fd)-3)
#define RXFIFO1_RXFILTER_NUM(canindex,use_fd) (RXFILTER_NUM(canindex,use_fd)-2)
#if defined(FDCAN_TYPE_H7)
  // Settings for FIFO elements and message RAM. When using multiple instances, the
  // combined FIFO, buffers and filters may not exceed the maximum, see reference manual.
  #define RXFIFO0_NUM(canindex,use_fd) (use_fd ? 50 : 64)
  #define RXFIFO1_NUM(canindex,use_fd) (use_fd ? 50 : 64)
  #define TXFIFO_NUM(canindex,use_fd) 32
  #define MSG_RAM_OFFSET(canindex,use_fd) 0
#endif // defined(FDCAN_TYPE_H)

// The errors/notifications that we handle in this driver
#define FDCAN_NOTIFICATIONS ( FDCAN_IT_RX_FIFO0_NEW_MESSAGE | \
                              FDCAN_IT_RX_FIFO1_NEW_MESSAGE | \
														  FDCAN_IT_RX_FIFO0_MESSAGE_LOST | \
														  FDCAN_IT_RX_FIFO0_FULL | \
														  FDCAN_IT_RX_FIFO1_MESSAGE_LOST | \
														  FDCAN_IT_RX_FIFO1_FULL | \
														  FDCAN_IT_TX_EVT_FIFO_FULL | \
														  FDCAN_IT_RAM_ACCESS_FAILURE | \
														  FDCAN_IT_ARB_PROTOCOL_ERROR  | \
														  FDCAN_IT_DATA_PROTOCOL_ERROR  | \
														  FDCAN_IT_ERROR_PASSIVE | \
														  FDCAN_IT_ERROR_WARNING | \
														  FDCAN_IT_BUS_OFF )


// Bit time settings for ~87.5% sample point (compatible with Peak adapters)
// The macros are REGISTER values — the stack adds 1 before writing to HAL Init.
// FDCAN clock = 80 MHz (24 MHz HSE, PLL: /3 × 20 / 2 = 80 MHz)
// Baud = FDCAN_Clock / ((BRP+1) × TQ), where TQ = 1 + (Seg1+1) + (Seg2+1)
//
// Most rates use TQ=16 (Seg1=12, Seg2=1) → 87.5% sample point:
//   1000 kb/s: 80M / ( 5 × 16) = 1000k   BRP=4
//    500 kb/s: 80M / (10 × 16) =  500k   BRP=9
//    250 kb/s: 80M / (20 × 16) =  250k   BRP=19
//    125 kb/s: 80M / (40 × 16) =  125k   BRP=39
//     50 kb/s: 80M / (100× 16) =   50k   BRP=99
//     20 kb/s: 80M / (250× 16) =   20k   BRP=249
//
// 800 kb/s uses TQ=25 (Seg1=20, Seg2=2) → 88.0% sample point:
//    800 kb/s: 80M / ( 4 × 25) =  800k   BRP=3
#define CAN_BRP(portnum,bitratekpbs) (\
		(bitratekpbs==1000) ?   4 : \
		(bitratekpbs== 800) ?   3 : \
		(bitratekpbs== 500) ?   9 : \
		(bitratekpbs== 250) ?  19 : \
		(bitratekpbs== 125) ?  39 : \
		(bitratekpbs==  50) ?  99 : \
		(bitratekpbs==  20) ? 249 : 0)
#define CANA_SEG1(portnum,bitratekpbs) (\
		(bitratekpbs== 800) ? 20 : 12)
#define CANA_SEG2(portnum,bitratekpbs) (\
		(bitratekpbs== 800) ?  2 :  1)
#define CANA_SJW(portnum,bitratekpbs)      CANA_SEG2(portnum,bitratekpbs)
#define CANA_TQNUM(portnum,bitratekpbs)    (1+CANA_SEG1(portnum,bitratekpbs)+1+CANA_SEG2(portnum,bitratekpbs)+1)
#if PROCESS_CO_FD
#define CANFDA_SEG1(portnum,bitratekpbs) (\
		(bitratekpbs== 1000) ?  30 : \
		(bitratekpbs==  800) ?  38 : \
		(bitratekpbs==  500) ?  62 : \
		(bitratekpbs==  250) ? 126 : 62 )
#define CANFDA_SEG2(portnum,bitratekpbs) (\
		(bitratekpbs== 1000) ?   7 : \
		(bitratekpbs==  800) ?   9 : \
		(bitratekpbs==  500) ?  15 : \
		(bitratekpbs==  250) ?  31 : 15)
#define CANFDA_SJW(portnum,bitratekpbs)    CANFDA_SEG2(portnum,bitratekpbs)
#define CANFDA_TQNUM(portnum,bitratekpbs)  (1+CANFDA_SEG1(portnum,bitratekpbs)+1+CANFDA_SEG2(portnum,bitratekpbs)+1)
#define CANFDD_SEG1(portnum,dbitratekpbs) (\
		(dbitratekpbs== 1000) ?  30 : \
		(dbitratekpbs== 2000) ?  13 : \
		(dbitratekpbs== 4000) ?   5 : \
		(dbitratekpbs== 5000) ?   4 : 28)
#define CANFDD_SEG2(portnum,dbitratekpbs) (\
		(dbitratekpbs== 1000) ?   7 : \
		(dbitratekpbs== 2000) ?   4 : \
		(dbitratekpbs== 4000) ?   2 : \
		(dbitratekpbs== 5000) ?   1 : 4)
#define CANFDD_SJW(portnum,dbitratekpbs)   CANFDD_SEG2(portnum,dbitratekpbs)
#define CANFDD_TQNUM(portnum,dbitratekpbs) (1+CANFDD_SEG1(portnum,dbitratekpbs)+1+CANFDD_SEG2(portnum,dbitratekpbs)+1)
#endif // PROCESS_CO_FD


/**************************************************************************
GLOBAL VARIABLES
***************************************************************************/

// Global timer/counter variable, incremented every millisecond
uint16_t volatile gTimCnt = 0;

// Flags to indicate that at least one message is available in the slave or manager receive FIFO
uint8_t volatile gCanRxPending[NUM_CAN_PORTS];
uint8_t volatile gCanMgrRxPending[NUM_CAN_PORTS];

//TIM_HandleTypeDef   gMcopTimHandle;
//TIM_HandleTypeDef  *gMcopTimHandle_p=&gMcopTimHandle;
extern TIM_HandleTypeDef htim7;
TIM_HandleTypeDef *gMcopTimHandle_p = &htim7;

// Array of pointers to CAN controller handles, index is the logical (driver-supported) CAN port number starting with 0 for the first
FDCAN_HandleTypeDef gCANHandle[NUM_CAN_PORTS];

void MCOHW_TimerTick(void);


/**************************************************************************
LOCAL VARIABLES
***************************************************************************/

static uint8_t mNextFilterNum[NUM_CAN_PORTS];

// Array of pointers to CAN controller registers, index is the logical (driver-supported) CAN port number
static FDCAN_GlobalTypeDef *mCAN[NUM_CAN_PORTS] = CAN_LIST_INITIALIZER;

#if MCO_NUM_INST_MAX
// keeps track of stack handles for CAN controllers, index is the enumerated CAN port number
static MCO_HANDLE mMCO_HANDLEs[NUM_CAN_PORTS];
#endif // MCO_NUM_INST_MAX


#if USE_SYNC_PRODUCER
struct
{
  uint32_t nexttim; // next timer valuze when SYNC gets produced
  uint8_t cnt; // current counter
  CAN_MSG syncMsg;   // persistent buffer for SYNC message
} mSYNCprod[NUM_CAN_PORTS];
#endif


/**************************************************************************
LOCAL FUNCTIONS
***************************************************************************/

static uint8_t getCanIndexFromCanHandle(
  FDCAN_HandleTypeDef* hfdcan
  )
{
  uint8_t i;

  for (i=0; i<NUM_CAN_PORTS; i++)
  {
    if (&gCANHandle[i] == hfdcan)
    {
      break;
    }
  }
  return i;
}

// Helper functions to convert between CAN message length and DLC coding
static uint8_t getLenFromDlc(uint8_t dlc)
{
	const uint8_t length[16] = { 0, 1, 2, 3, 4, 5, 6, 7, 8, 12, 16, 20, 24, 32, 48, 64 };
	
  if (dlc>15)
    dlc = 0;
	return length[dlc];
}	

static uint8_t getDlcFromLen(uint8_t len)
{
	uint8_t dlc;
	
  if      (len <=  8) dlc = len;
  else if (len <= 12) dlc =   9;
  else if (len <= 16) dlc =  10;
  else if (len <= 20) dlc =  11;
  else if (len <= 24) dlc =  12;
  else if (len <= 32) dlc =  13;
  else if (len <= 48) dlc =  14;
  else                dlc =  15;

  return dlc;
}	


#if USE_SYNC_PRODUCER
/**************************************************************************
DOES:    Handles SYNC transmission
RETURNS: TRUE, if SYNC message was produced
         FALSE, if SYNC was not produced
**************************************************************************/
void MCOHW_ProcessSYNCtx(
  OPT_SINGLE_HANDLE_PARAM // Stack handle (only in multi-instance environment)
)
{
  // Handle SYNC production
  if ((MY_NMT_STATE == NMTSTATE_OP) &&
    ((CONFIG_USED.SYNC_id & COBID_RTR) != 0) && //here RTR bit enables producer
    (CONFIG_USED.SYNC_cycle > 1000)
    )
  { // only while we are operational
    if ((mSYNCprod[CAN_PORT_INDEX].cnt == CONFIG_USED.SYNC_cntovr + 1) || (MCOHW_IsTimeExpired(mSYNCprod[CAN_PORT_INDEX].nexttim)))
    { // first time trigger or timer expired
      mSYNCprod[CAN_PORT_INDEX].syncMsg.ID = CONFIG_USED.SYNC_id & 0x000007FFL;
      if (CONFIG_USED.SYNC_cntovr == 0)
      { // no counter used
        mSYNCprod[CAN_PORT_INDEX].syncMsg.LEN = 0;
      }
      else
      { // counter used
        if (mSYNCprod[CAN_PORT_INDEX].cnt > CONFIG_USED.SYNC_cntovr)
        {
          mSYNCprod[CAN_PORT_INDEX].cnt = 1;
        }
        mSYNCprod[CAN_PORT_INDEX].syncMsg.LEN = 1;
        mSYNCprod[CAN_PORT_INDEX].syncMsg.BUF[0] = mSYNCprod[CAN_PORT_INDEX].cnt;
        // update counter
        mSYNCprod[CAN_PORT_INDEX].cnt++;
        if (mSYNCprod[CAN_PORT_INDEX].cnt > CONFIG_USED.SYNC_cntovr)
        {
          mSYNCprod[CAN_PORT_INDEX].cnt = 1;
        }
      }
      MCOHW_PushMessage(OPT_FIRST_HANDLE_CALL_PARAM &mSYNCprod[CAN_PORT_INDEX].syncMsg);
      // set new timer trigger
      mSYNCprod[CAN_PORT_INDEX].nexttim = MCOHW_GetTime() + (CONFIG_USED.SYNC_cycle / 1000);
    }
  }
  else
  { // when not operational, keep resetting producer
    mSYNCprod[CAN_PORT_INDEX].cnt = CONFIG_USED.SYNC_cntovr + 1;
    mSYNCprod[CAN_PORT_INDEX].cnt = 1;
  }

  return;
}
#endif


/**************************************************************************
PUBLIC FUNCTIONS
***************************************************************************/

/**************************************************************************
DOES:    This function returns the global status variable.
CHANGES: The status can be changed anytime by this module, for example from
         within an interrupt service routine or by any of the other
         functions in this module.
BITS:    0: INIT - set to 1 after a completed initialization
                   left 0 if not yet inited or init failed
         1: CERR - set to 1 if a CAN bit or frame error occured
         2: ERPA - set to 1 if a CAN "error passive" occured
         3: RXOR - set to 1 if a receive queue overrun occured
         4: TXOR - set to 1 if a transmit queue overrun occured
         5: Reserved
         6: TXBSY - set to 1 if Transmit queue is not empty
         7: BOFF - set to 1 if a CAN "bus off" error occured
**************************************************************************/
uint8_t MCOHW_GetStatus
  (
  OPT_SINGLE_HANDLE_PARAM // Stack handle (only in multi-instance environment)
  )
{
  uint8_t ret_val = 0;

  // are we initialized?
  if ((HW_STATUS_USED & HW_INIT) != 0)
  {
		// transmit fifo or buffer in use?
		if (gCANHandle[CAN_PORT_INDEX].Instance->TXBRP != 0x00000000UL)
		{
			// Busy transmitting
			HW_STATUS_USED |= HW_TXBSY;
		}
		else
		{
			// all Tx buffers empty
			HW_STATUS_USED &= ~HW_TXBSY;
		}

		ret_val = HW_STATUS_USED;

		// if no rx data overrun
		if (((gCANHandle[CAN_PORT_INDEX].Instance->RXF0S & FDCAN_RXF0S_RF0L) == 0UL) && ((gCANHandle[CAN_PORT_INDEX].Instance->RXF1S & FDCAN_RXF1S_RF1L) == 0UL))
		{
			// clear overrun flag
			HW_STATUS_USED &= ~HW_RXOR;
		}
		if ((gCANHandle[CAN_PORT_INDEX].Instance->PSR & FDCAN_PSR_EP) == 0UL)
		{
			// clear error passive flag
			HW_STATUS_USED &= ~HW_ERPA;
		}
		if ((gCANHandle[CAN_PORT_INDEX].Instance->PSR & FDCAN_PSR_LEC) == 0UL)
		{
			// clear misc. error flag
			HW_STATUS_USED &= ~HW_CERR;
		}
		// if not bus off
		if ((gCANHandle[CAN_PORT_INDEX].Instance->PSR & FDCAN_PSR_BO) == 0UL)
		{
			// clear bus off flag
			HW_STATUS_USED &= ~HW_BOFF;
		}

#if USE_LEDS
		if (HW_STATUS_USED & (HW_BOFF | HW_ERPA | HW_CERR))
		{
			if (HW_STATUS_USED & (HW_BOFF))
			{
				CONFIG_USED.LEDErr = LED_ON;
			}
			else
			{
				CONFIG_USED.LEDErr = LED_FLASH1;
			}
		}
		else
		{
			CONFIG_USED.LEDErr = LED_OFF;
		}
#endif // USE_LEDS
	}
	
  return ret_val;
}


/**************************************************************************
DOES:    This function implements a CAN receive queue. With each
         function call a message is pulled from the queue.
RETURNS: 1 Message was pulled from receive queue
         0 Queue empty, no message received
**************************************************************************/
uint8_t MCOHW_PullMessage
  (
  OPT_FIRST_HANDLE_PARAM // Stack handle (only in multi-instance environment)
  MEM_FAR CAN_MSG *pReceiveBuf
  )
{
	FDCAN_RxHeaderTypeDef RxHeader;
	uint8_t RxData[CAN_MAX_DATA_SIZE];
  uint8_t useFD = FALSE;

  (void)useFD; // avoid 'unused' compiler warning
#if PROCESS_CO_FD
  if (MCO_PROT_BIT_SET(handle, MCO_PROT_FD))
  {
    useFD = TRUE;
  }
#endif

  // are we initialized?
  if ((HW_STATUS_USED & HW_INIT) != 0)
  {
    if (HAL_FDCAN_GetRxMessage(&gCANHandle[CAN_PORT_INDEX], FDCAN_RX_FIFO0, &RxHeader, RxData) == HAL_OK)
		{
			/* Retrieve Rx messages from RX FIFO0 */
			// if 11-bit id, no RTR
			if ((RxHeader.IdType == FDCAN_STANDARD_ID) && (RxHeader.RxFrameType == FDCAN_DATA_FRAME))
			{
        if ( (useFD &&  (RxHeader.FDFormat)) ||
            (!useFD && !(RxHeader.FDFormat)) )
        {
          uint8_t i;
          // Message needs to be received
          // copy message
          pReceiveBuf->ID = RxHeader.Identifier;
#if defined(LEGACY_HAL_DATALENGTH)
              pReceiveBuf->LEN = getLenFromDlc((uint8_t)(RxHeader.DataLength & 0x000F0000UL) >> 16));
#else
              pReceiveBuf->LEN = getLenFromDlc((uint8_t)(RxHeader.DataLength));
#endif
          for (i=0; i<pReceiveBuf->LEN; i++)
          {
            pReceiveBuf->BUF[i] = RxData[i];
          }
				
          return TRUE; // msg received
        }
			}
		}
		else
		{
			gCanRxPending[CAN_PORT_INDEX] = FALSE;
		}
	}

  return FALSE; // no msg rcvd
}


#if USE_CANOPEN_MANAGER
/**************************************************************************
DOES:    This function is used by the manager to poll messages that are
         needed by the manager
RETURNS: TRUE or FALSE, if no message was received
**************************************************************************/
uint8_t MCOHWMGR_PullMessage
  (
  OPT_FIRST_HANDLE_PARAM // Stack handle (only in multi-instance environment)
  MEM_FAR CAN_MSG *pReceiveBuf // buffer to witch a received message is copied
  )
{
	FDCAN_RxHeaderTypeDef RxHeader;
	uint8_t RxData[CAN_MAX_DATA_SIZE];
	uint32_t canid;
  uint8_t useFD = FALSE;

  (void)useFD; // avoid 'unused' compiler warning
#if PROCESS_CO_FD
  if (MCO_PROT_BIT_SET(handle, MCO_PROT_FD))
  {
    useFD = TRUE;
  }
#endif

  // are we initialized?
  if ((HW_STATUS_USED & HW_INIT) != 0)
  {
    if (HAL_FDCAN_GetRxMessage(&gCANHandle[CAN_PORT_INDEX], FDCAN_RX_FIFO1, &RxHeader, RxData) == HAL_OK)
		{
			/* Retrieve Rx messages from RX FIFO1 */
			// if 11-bit id, no RTR
			if ((RxHeader.IdType == FDCAN_STANDARD_ID) && (RxHeader.RxFrameType == FDCAN_DATA_FRAME))
			{
        canid = RxHeader.Identifier;

        // only work on manager when operational or LSS response received
        if (
#if USE_MLSS_MANAGER || USE_LSS_MANAGER
            IS_CANID_LSS_RESPONSE(canid) ||
#endif
               ( ( (useFD && (RxHeader.FDFormat)) || (!useFD && !(RxHeader.FDFormat)) ) &&
                 (MY_NMT_STATE == NMTSTATE_OP)
               )
            )
        {
          uint8_t i;

          if (
#if defined(MCO_NUM_INST_MAX) && MCO_NUM_INST_MAX
            IS_CAN_ID_EMERGENCY(handle, canid)
            || IS_CAN_ID_HEARTBEAT(handle, canid)
#if USE_MLSS_MANAGER || USE_LSS_MANAGER
            || IS_CANID_LSS_RESPONSE(handle, canid) // LSS Response
#endif
#if PROCESS_CO_LEGACY
            || IS_CAN_ID_SDORESPONSE(handle, canid)
#endif
#if PROCESS_CO_FD
            || IS_CAN_ID_USDORESPONSE(handle, canid)
#endif
#else
            IS_CAN_ID_EMERGENCY(canid)
            || IS_CAN_ID_HEARTBEAT(canid)
#if USE_MLSS_MANAGER || USE_LSS_MANAGER
            || IS_CANID_LSS_RESPONSE(canid) // LSS Response
#endif
#if PROCESS_CO_LEGACY
            || IS_CAN_ID_SDORESPONSE(canid)
#endif
#if PROCESS_CO_FD
            || IS_CAN_ID_USDORESPONSE(canid)
#endif
#endif
          )
          {
          // This is a message for the CANopen Manager

            // Message needs to be received
            // copy message
            pReceiveBuf->ID = canid;
#if defined(LEGACY_HAL_DATALENGTH)
            pReceiveBuf->LEN = getLenFromDlc((uint8_t)(RxHeader.DataLength & 0x000F0000UL) >> 16));
#else
            pReceiveBuf->LEN = getLenFromDlc((uint8_t)(RxHeader.DataLength));
#endif
            for (i=0; i<pReceiveBuf->LEN; i++)
            {
              pReceiveBuf->BUF[i] = RxData[i];
            }

            return TRUE; // msg received
          }
        }
			}
		}
		else
		{
			gCanMgrRxPending[CAN_PORT_INDEX] = FALSE;
		}
	}

  return FALSE; // no msg rcvd
}
#endif // USE_CANOPEN_MANAGER


/**************************************************************************
DOES:    Adding a CAN message to the transmit queue
RETURNS: TRUE or FALSE if queue overrun
***************************************************************************/
uint8_t MCOHW_PushMessage
  (
  OPT_FIRST_HANDLE_PARAM // Stack handle (only in multi-instance environment)
  MEM_FAR CAN_MSG *pTransmitBuf // Data structure with message to be send
  )
{
  static FDCAN_TxHeaderTypeDef TxHeader;
  uint8_t useFD = FALSE;

  (void)useFD; // avoid 'unused' compiler warning
#if PROCESS_CO_FD
  if (MCO_PROT_BIT_SET(handle, MCO_PROT_FD))
  {
    useFD = TRUE;
  }
#endif

  // are we initialized?
  if ((HW_STATUS_USED & HW_INIT) != 0)
  {
		/* Prepare Tx Header */
		TxHeader.Identifier = pTransmitBuf->ID;
		TxHeader.IdType = FDCAN_STANDARD_ID;
		TxHeader.TxFrameType = FDCAN_DATA_FRAME;
//		TxHeader.DataLength = (uint32_t)(getDlcFromLen(pTransmitBuf->LEN) << 16);
		TxHeader.DataLength = (uint32_t)(getDlcFromLen(pTransmitBuf->LEN));
		TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
#if defined(FDCAN_BRS_DISABLED)
		TxHeader.BitRateSwitch = FDCAN_BRS_OFF;
#else
		TxHeader.BitRateSwitch = FDCAN_BRS_ON;
#endif
#if PROCESS_CO_FD
		if (useFD)
		{
			TxHeader.FDFormat = FDCAN_FD_CAN;
		}
		else
#else
		{
			TxHeader.FDFormat = FDCAN_CLASSIC_CAN;
		}
#endif
		TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
		TxHeader.MessageMarker = 0;

		/*##-3- Start the Transmission process ###############################*/
					/* Start the Transmission process */
		if (HAL_FDCAN_AddMessageToTxFifoQ(&gCANHandle[CAN_PORT_INDEX], &TxHeader, &pTransmitBuf->BUF[0]) == HAL_OK)
		{
			return TRUE;
		}
		else
		{
			// Overrun occurred
			// Signal overrun to status variable
			HW_STATUS_USED |= HW_TXOR;
			return FALSE;
		}
	}
	else
	{
		return FALSE;
	}
}


/** @defgroup HAL_MSP_Private_Functions
  * @{
  */

/**
  * @brief TIM MSP Initialization
  *        This function configures the hardware resources:
  *           - Peripheral's clock enable
  *           - Peripheral's GPIO Configuration
  * @param htim: TIM handle pointer
  * @retval None
  */
#if defined(USE_CUBEMX)
__weak
#endif
void HAL_TIM_Base_MspInit(TIM_HandleTypeDef *htim)
{
  // is this the timer used in the stack?
  if (htim == gMcopTimHandle_p)
  {
    /*##-1- Enable peripherals and GPIO Clocks #################################*/
    /* TIMx Peripheral clock enable */
    TIMx_CLK_ENABLE();

    /*##-2- Configure the NVIC for TIMx ########################################*/
    /* Set the TIMx priority */
    HAL_NVIC_SetPriority(TIMx_IRQn, 3, 0);

    /* Enable the TIMx global Interrupt */
    HAL_NVIC_EnableIRQ(TIMx_IRQn);
  }
}


/**
  * @brief  Initializes the FDCAN MSP.
  * @param  hfdcan: pointer to an FDCAN_HandleTypeDef structure that contains
  *         the configuration information for the specified FDCAN.
  * @retval None
  */
#if defined(USE_CUBEMX)
__weak
#endif
void HAL_FDCAN_MspInit(FDCAN_HandleTypeDef* hfdcan)
{
  GPIO_InitTypeDef  GPIO_InitStruct;
  RCC_PeriphCLKInitTypeDef RCC_PeriphClkInit;
  uint8_t i;
#if MCO_NUM_INST_MAX
  MCO_HANDLE handle;
#endif // MCO_NUM_INST_MAX

  i = getCanIndexFromCanHandle(hfdcan);
  if (i<NUM_CAN_PORTS)
  {
#if MCO_NUM_INST_MAX
    // Get handle for this CAN port
    handle = mMCO_HANDLEs[i];
#endif // MCO_NUM_INST_MAX

    /*##-1- Enable peripherals and GPIO Clocks #################################*/
    /* Enable GPIO TX/RX clock */
    FDCANx_TX_GPIO_CLK_ENABLE(i);
    FDCANx_RX_GPIO_CLK_ENABLE(i);

    /* Select PLL1Q as source of FDCANx clock */
#if defined(FDCAN_TYPE_U)
    RCC_PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_FDCAN1;
    RCC_PeriphClkInit.Fdcan1ClockSelection = RCC_FDCAN1CLKSOURCE_PLL1;
#elif defined(FDCAN_TYPE_H)
    RCC_PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_FDCAN;
    RCC_PeriphClkInit.FdcanClockSelection = RCC_FDCANCLKSOURCE_PLL1Q;
#else
    RCC_PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_FDCAN;
    RCC_PeriphClkInit.FdcanClockSelection = RCC_FDCANCLKSOURCE_PLL;
#endif // defined(FDCAN_TYPE_U)

    HAL_RCCEx_PeriphCLKConfig(&RCC_PeriphClkInit);

    /* Enable FDCANx clock */
    FDCANx_CLK_ENABLE(i);

    /*##-2- Configure peripheral GPIO ##########################################*/
    /* FDCANx TX GPIO pin configuration  */
    GPIO_InitStruct.Pin       = FDCANx_TX_PIN(i);
    GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull      = GPIO_PULLUP;
    GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = FDCANx_TX_AF(i);
    HAL_GPIO_Init(FDCANx_TX_GPIO_PORT(i), &GPIO_InitStruct);

    /* FDCANx RX GPIO pin configuration  */
    GPIO_InitStruct.Pin       = FDCANx_RX_PIN(i);
    GPIO_InitStruct.Alternate = FDCANx_RX_AF(i);
    HAL_GPIO_Init(FDCANx_RX_GPIO_PORT(i), &GPIO_InitStruct);

    MCOHW_CAN_MspInit(hfdcan);
  }
}
void MCOHW_CAN_MspInit(FDCAN_HandleTypeDef *hfdcan)
{
  uint8_t i;
#if MCO_NUM_INST_MAX
  MCO_HANDLE handle;
#endif // MCO_NUM_INST_MAX

  i = getCanIndexFromCanHandle(hfdcan);
  if (i<NUM_CAN_PORTS)
  {
#if MCO_NUM_INST_MAX
    // Get handle for this CAN port
    handle = mMCO_HANDLEs[i];
#endif // MCO_NUM_INST_MAX

    /*##-3- Configure the NVIC #################################################*/
    /* NVIC for FDCANx */
    HAL_NVIC_SetPriority(FDCANx_IRQn(i), 0, 1);
    HAL_NVIC_EnableIRQ(FDCANx_IRQn(i));
  }
}

/**
  * @brief  DeInitializes the FDCAN MSP.
  * @param  hfdcan: pointer to an FDCAN_HandleTypeDef structure that contains
  *         the configuration information for the specified FDCAN.
  * @retval None
  */
#if defined(USE_CUBEMX)
__weak
#endif
void HAL_FDCAN_MspDeInit(FDCAN_HandleTypeDef* hfdcan)
{
  uint8_t i;
#if MCO_NUM_INST_MAX
  MCO_HANDLE handle;
#endif // MCO_NUM_INST_MAX

  i = getCanIndexFromCanHandle(hfdcan);
  if (i<NUM_CAN_PORTS)
  {
#if MCO_NUM_INST_MAX
    // Get handle for this CAN port
    handle = mMCO_HANDLEs[i];
#endif // MCO_NUM_INST_MAX

    MCOHW_CAN_MspDeInit(hfdcan);

    /*##-2- Disable peripherals and GPIO Clocks ################################*/
    /* Configure FDCANx Tx as alternate function  */
    HAL_GPIO_DeInit(FDCANx_TX_GPIO_PORT(i), FDCANx_TX_PIN(i));

    /* Configure FDCANx Rx as alternate function  */
    HAL_GPIO_DeInit(FDCANx_RX_GPIO_PORT(i), FDCANx_RX_PIN(i));
  }
}

void MCOHW_CAN_MspDeInit(FDCAN_HandleTypeDef *hfdcan)
{
  uint8_t i;
#if MCO_NUM_INST_MAX
  MCO_HANDLE handle;
#endif // MCO_NUM_INST_MAX

  i = getCanIndexFromCanHandle(hfdcan);
  if (i<NUM_CAN_PORTS)
  {
#if MCO_NUM_INST_MAX
    // Get handle for this CAN port
    handle = mMCO_HANDLEs[i];
#endif // MCO_NUM_INST_MAX

    /*##-1- Reset peripherals ##################################################*/
    FDCANx_FORCE_RESET();
    FDCANx_RELEASE_RESET();

    /*##-3- Disable the NVIC for FDCANx ########################################*/
    HAL_NVIC_DisableIRQ(FDCANx_IRQn(i));
  }
}


/* NOTE: HAL_TIM_PeriodElapsedCallback lives in stm32g4xx_it.c
 * to decouple this CAN hardware driver from stepper.h.
 * The callback routes both the MCO stack timer (TIM7 → MCOHW_TimerTick)
 * and the stepper timer (TIM1 → Stepper_ProcessTimerUpdate).
 */


/**************************************************************************
DOES:    Timer interrupt handler (1ms)
**************************************************************************/
void MCOHW_TimerTick(void)
{
  uint8_t i;
#if MCO_NUM_INST_MAX
  MCO_HANDLE handle;
#endif // MCO_NUM_INST_MAX

  for (i=0; i<NUM_CAN_PORTS; i++)
  {
#if MCO_NUM_INST_MAX
    // Get handle for this CAN port
    handle = mMCO_HANDLEs[i];
#endif // MCO_NUM_INST_MAX
    // are we initialized?
    if ((HW_STATUS_USED & HW_INIT) != 0)
    {
#if USE_SYNC_PRODUCER
#if MCO_NUM_INST_MAX
      MCOHW_ProcessSYNCtx(handle);
#else
      MCOHW_ProcessSYNCtx();
#endif // MCO_NUM_INST_MAX
#endif // USE_SYNC_PRODUCER
    }
  }

  gTimCnt++; // increment global timer counter
}


/**************************************************************************
DOES:    CAN receive interrupt handler
**************************************************************************/
/**
  * @brief  Rx FIFO 0 callback.
  * @param  hfdcan: pointer to an FDCAN_HandleTypeDef structure that contains
  *         the configuration information for the specified FDCAN.
  * @param  RxFifo0ITs: indicates which Rx FIFO 0 interrupts are signalled.
  *                     This parameter can be any combination of @arg FDCAN_Rx_Fifo0_Interrupts.
  * @retval None
  */
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *CanHandle, uint32_t RxFifo0ITs)
{
  uint8_t i;
#if defined(USE_CMSIS_OS)
  // Signal receive to handling task
  canRxFlag = 1;
#endif // defined(USE_CMSIS_OS)

#if MCO_NUM_INST_MAX
  MCO_HANDLE handle;
#endif // MCO_NUM_INST_MAX

  i = getCanIndexFromCanHandle(CanHandle);
  if (i<NUM_CAN_PORTS)
  {
#if MCO_NUM_INST_MAX
    // Get handle for this CAN port
    handle = mMCO_HANDLEs[i];
#endif // MCO_NUM_INST_MAX

    // are we initialized?
    if ((HW_STATUS_USED & HW_INIT) != 0)
    {
      if((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != RESET)
      {
        gCanRxPending[i] = TRUE;
      }
    }
  }
}


/**
  * @brief  Rx FIFO 1 callback.
  * @param  hfdcan: pointer to an FDCAN_HandleTypeDef structure that contains
  *         the configuration information for the specified FDCAN.
  * @param  RxFifo0ITs: indicates which Rx FIFO 1 interrupts are signalled.
  *                     This parameter can be any combination of @arg FDCAN_Rx_Fifo1_Interrupts.
  * @retval None
  */
void HAL_FDCAN_RxFifo1Callback(FDCAN_HandleTypeDef *CanHandle, uint32_t RxFifo1ITs)
{
  uint8_t i;
#if MCO_NUM_INST_MAX
  MCO_HANDLE handle;
#endif // MCO_NUM_INST_MAX

  i = getCanIndexFromCanHandle(CanHandle);
  if (i<NUM_CAN_PORTS)
  {
#if MCO_NUM_INST_MAX
    // Get handle for this CAN port
    handle = mMCO_HANDLEs[i];
#endif // MCO_NUM_INST_MAX

    // do we manage the CAN controller and are we initialized?
    if ((i<NUM_CAN_PORTS) && ((HW_STATUS_USED & HW_INIT) != 0))
    {
      if((RxFifo1ITs & FDCAN_IT_RX_FIFO1_NEW_MESSAGE) != RESET)
      {
        gCanMgrRxPending[i] = TRUE;
      }
    }
  }
}


/**************************************************************************
DOES:    CAN FD error interrupt handler
**************************************************************************/
void HAL_FDCAN_ErrorStatusCallback(FDCAN_HandleTypeDef *CanHandle, uint32_t ErrorStatusITs)
{
  uint8_t i;
#if MCO_NUM_INST_MAX
  MCO_HANDLE handle;
#endif // MCO_NUM_INST_MAX

  i = getCanIndexFromCanHandle(CanHandle);
  if (i<NUM_CAN_PORTS)
  {
#if MCO_NUM_INST_MAX
    // Get handle for this CAN port
    handle = mMCO_HANDLEs[i];
#endif // MCO_NUM_INST_MAX

    // do we manage the CAN controller and are we initialized?
    if ((i<NUM_CAN_PORTS) && ((HW_STATUS_USED & HW_INIT) != 0))
    {
    // INSERT APPLICATION SPECIFIC CODE AS NEEDED

#if USE_LEDS
      CONFIG_USED.LEDErr = LED_FLASH1;
#endif

      if (ErrorStatusITs & FDCAN_IT_ERROR_PASSIVE)  /* Error Passive */
      {
        HW_STATUS_USED |= HW_ERPA;
      }
      if (ErrorStatusITs & FDCAN_IT_BUS_OFF)  /* Bus_Off Status  */
      {
        HW_STATUS_USED |= HW_BOFF;
#if USE_LEDS
        CONFIG_USED.LEDErr = LED_ON;
#endif
				// Bus-off recovery — restart FDCAN protocol engine
				// ISO 11898-1: controller must see 128×11 recessive bits before rejoining.
				// HAL_FDCAN_Stop + HAL_FDCAN_Start resets the protocol engine and allows
				// the automatic recovery sequence to begin. Without this, bus-off is
				// permanent and the node becomes deaf (NMT RESET NODE unresponsive).
				HAL_FDCAN_Stop(CanHandle);
				if (HAL_FDCAN_Start(CanHandle) == HAL_OK) {
					HAL_FDCAN_ActivateNotification(CanHandle,
							FDCAN_NOTIFICATIONS, 0);
				}
      }
      if (ErrorStatusITs & (
          FDCAN_IT_RAM_ACCESS_FAILURE |   /* Message RAM Access Failure */
          FDCAN_IT_ARB_PROTOCOL_ERROR |   /* Protocol Error in Arbitration Phase */
          FDCAN_IT_DATA_PROTOCOL_ERROR )) /* Protocol Error in Data Phase */
      {
        HW_STATUS_USED |= HW_CERR;
      }
      if (ErrorStatusITs & FDCAN_IT_TX_EVT_FIFO_FULL) /* Tx Event FIFO Full */
      {
        HW_STATUS_USED |= HW_TXOR;
      }
      if (CanHandle->ErrorCode & (
          FDCAN_IT_RX_FIFO0_MESSAGE_LOST | /* Rx FIFO 0 Message Lost */
          FDCAN_IT_RX_FIFO0_FULL         | /* Rx FIFO 0 Full */
          FDCAN_IT_RX_FIFO1_MESSAGE_LOST | /* Rx FIFO 1 Message Lost */
          FDCAN_IT_RX_FIFO1_FULL ))        /* Rx FIFO 1 Full */
      {
        HW_STATUS_USED |= HW_RXOR;
      }

      // Reset errors
      CanHandle->ErrorCode = HAL_FDCAN_ERROR_NONE;

      /*##-2- Start the Reception process and enable reception interrupt #########*/
      if (HAL_FDCAN_ActivateNotification(CanHandle, FDCAN_NOTIFICATIONS, 0) != HAL_OK)
      {
        /* Reception Error */
        (void)0; // status may be busy, so just ignore here
      }
    }
  }
}


/**************************************************************************
DOES:    FDCAN HAL run-time error handler
**************************************************************************/
void HAL_FDCAN_ErrorCallback(FDCAN_HandleTypeDef *CanHandle)
{
	// should never happen, customize if needed
	// CanHandle->ErrorCode contains the reason for the HAL error
}


/**************************************************************************
DOES:    This function implements the initialization of the CAN interface.
RETURNS: 1 if init is completed
         0 if init failed, bit INIT of MCOHW_GetStatus stays 0
**************************************************************************/
uint8_t MCOHW_Init
  (
  OPT_FIRST_HANDLE_PARAM // Stack handle (only in multi-instance environment)
  uint16_t bitrate
#if PROCESS_CO_FD
  ,
  uint16_t brs_bitrate
#endif
  )
{
  uint32_t uwPrescalerValue = 0;
  uint8_t useFD; 

  (void)useFD; // avoid 'unused' compiler warning
  (void)uwPrescalerValue;
  
  // Stack not initialized
  HW_STATUS_USED &= ~HW_INIT;

  // Check if logical CAN port number is supported by driver configuration
  if (CAN_PORT_INDEX >= NUM_CAN_PORTS)
    return FALSE;
  
#if PROCESS_CO_FD
  useFD = (bitrate!=brs_bitrate); // not supporting using CAN FD without bit rate switch right now
#else
  useFD = FALSE;
#endif

#if PROCESS_CO_FD
  if (useFD)
  {
    if (
      (bitrate !=  250) &&
      (bitrate !=  500) &&
      (bitrate !=  800) &&
      (bitrate != 1000) )
    return 0; // unsupported bitrate
    if (
      (brs_bitrate !=  1000) &&
      (brs_bitrate !=  2000) &&
      (brs_bitrate !=  4000) &&
      (brs_bitrate !=  5000) &&
      (brs_bitrate !=  8000) &&
      (brs_bitrate != 10000) )
    return 0; // unsupported data bitrate
  }
  else
#endif
  {
    if (
      (bitrate != 20) && (bitrate != 50) &&
      (bitrate !=  125) &&
      (bitrate !=  250) &&
      (bitrate !=  500) &&
      (bitrate != 800)
				&&
      (bitrate != 1000) )
    return 0; // unsupported bitrate
  }

#if MCO_NUM_INST_MAX
  // Save handle for this CAN port
  mMCO_HANDLEs[CAN_PORT_INDEX] = handle;
#endif // MCO_NUM_INST_MAX

// CAN I/O CONFIGURATION
// see HAL_FDCAN_MspInit()
	
// CAN CONFIGURATION

  /*##-1- Configure the CAN peripheral #######################################*/
  gCANHandle[CAN_PORT_INDEX].Instance = mCAN[CAN_PORT_INDEX];
#if PROCESS_CO_FD
	if (useFD)
	{
    gCANHandle[CAN_PORT_INDEX].Init.FrameFormat = FDCAN_FRAME_FD_BRS;
    gCANHandle[CAN_PORT_INDEX].Init.NominalPrescaler = 1; // all bitrate settings for 40 MHz clock, no prescaler
    gCANHandle[CAN_PORT_INDEX].Init.NominalSyncJumpWidth = 1 + CANFDA_SJW(CAN_PORT_INDEX,bitrate); 
    gCANHandle[CAN_PORT_INDEX].Init.NominalTimeSeg1 = 1 + CANFDA_SEG1(CAN_PORT_INDEX,bitrate); /* NominalTimeSeg1 = Propagation_segment + Phase_segment_1 */
    gCANHandle[CAN_PORT_INDEX].Init.NominalTimeSeg2 = 1 + CANFDA_SEG2(CAN_PORT_INDEX,bitrate);
    gCANHandle[CAN_PORT_INDEX].Init.DataPrescaler = 1; // all bitrate settings for 40 MHz clock, no prescaler
    gCANHandle[CAN_PORT_INDEX].Init.DataSyncJumpWidth = 1 + CANFDD_SJW(CAN_PORT_INDEX,brs_bitrate); 
    gCANHandle[CAN_PORT_INDEX].Init.DataTimeSeg1 = 1 + CANFDD_SEG1(CAN_PORT_INDEX,brs_bitrate);
    gCANHandle[CAN_PORT_INDEX].Init.DataTimeSeg2 = 1 + CANFDD_SEG2(CAN_PORT_INDEX,brs_bitrate);
#if defined(FDCAN_TYPE_H7)
    gCANHandle[CAN_PORT_INDEX].Init.RxFifo0ElmtSize = FDCAN_DATA_BYTES_64;
    gCANHandle[CAN_PORT_INDEX].Init.RxFifo1ElmtSize = FDCAN_DATA_BYTES_64;
    gCANHandle[CAN_PORT_INDEX].Init.RxBufferSize = FDCAN_DATA_BYTES_64;
    gCANHandle[CAN_PORT_INDEX].Init.TxElmtSize = FDCAN_DATA_BYTES_64;
#endif // defined(FDCAN_TYPE_H)
	}
	else
#endif
	{
    gCANHandle[CAN_PORT_INDEX].Init.FrameFormat = FDCAN_FRAME_CLASSIC;
    gCANHandle[CAN_PORT_INDEX].Init.NominalPrescaler = 1 + CAN_BRP(CAN_PORT_INDEX,bitrate);
    gCANHandle[CAN_PORT_INDEX].Init.NominalSyncJumpWidth = 1 + CANA_SJW(CAN_PORT_INDEX,bitrate); 
    gCANHandle[CAN_PORT_INDEX].Init.NominalTimeSeg1 = 1 + CANA_SEG1(CAN_PORT_INDEX,bitrate); /* NominalTimeSeg1 = Propagation_segment + Phase_segment_1 */
    gCANHandle[CAN_PORT_INDEX].Init.NominalTimeSeg2 = 1 + CANA_SEG2(CAN_PORT_INDEX,bitrate);
#if defined(FDCAN_TYPE_H7)
    gCANHandle[CAN_PORT_INDEX].Init.RxFifo0ElmtSize = FDCAN_DATA_BYTES_8;
    gCANHandle[CAN_PORT_INDEX].Init.RxFifo1ElmtSize = FDCAN_DATA_BYTES_8;
    gCANHandle[CAN_PORT_INDEX].Init.RxBufferSize = FDCAN_DATA_BYTES_8;
    gCANHandle[CAN_PORT_INDEX].Init.TxElmtSize = FDCAN_DATA_BYTES_8;
#endif // defined(FDCAN_TYPE_H)
	}
#if defined(FDCAN_TYPE_G) || defined(FDCAN_TYPE_U)
  gCANHandle[CAN_PORT_INDEX].Init.ClockDivider = FDCAN_CLOCK_DIV1;
#endif // defined(FDCAN_TYPE_G)
  gCANHandle[CAN_PORT_INDEX].Init.Mode = FDCAN_MODE_NORMAL;
  gCANHandle[CAN_PORT_INDEX].Init.AutoRetransmission = ENABLE;
  gCANHandle[CAN_PORT_INDEX].Init.TransmitPause = DISABLE;
  gCANHandle[CAN_PORT_INDEX].Init.ProtocolException = ENABLE;
  gCANHandle[CAN_PORT_INDEX].Init.StdFiltersNbr = RXFILTER_NUM(CAN_PORT_INDEX,useFD);
  gCANHandle[CAN_PORT_INDEX].Init.ExtFiltersNbr = 0;
#if defined(FDCAN_TYPE_H7)
  gCANHandle[CAN_PORT_INDEX].Init.MessageRAMOffset = MSG_RAM_OFFSET(CAN_PORT_INDEX,useFD);
  gCANHandle[CAN_PORT_INDEX].Init.RxFifo0ElmtsNbr = RXFIFO0_NUM(CAN_PORT_INDEX,useFD);
  gCANHandle[CAN_PORT_INDEX].Init.RxFifo1ElmtsNbr = RXFIFO1_NUM(CAN_PORT_INDEX,useFD);
  gCANHandle[CAN_PORT_INDEX].Init.RxBuffersNbr = 0;
  gCANHandle[CAN_PORT_INDEX].Init.TxEventsNbr = 0;
  gCANHandle[CAN_PORT_INDEX].Init.TxBuffersNbr = 0;
  gCANHandle[CAN_PORT_INDEX].Init.TxFifoQueueElmtsNbr = TXFIFO_NUM(CAN_PORT_INDEX,useFD);
#endif // defined(FDCAN_TYPE_H)
  gCANHandle[CAN_PORT_INDEX].Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
  if (HAL_FDCAN_Init(&gCANHandle[CAN_PORT_INDEX]) != HAL_OK)
  {
    /* Initialization Error */
    return 0;
  }

#if PROCESS_CO_FD
	if (useFD)
	{
    /* Configure and enable Tx Delay Compensation : TdcOffset = DataTimeSeg1*DataPrescaler */
    HAL_FDCAN_ConfigTxDelayCompensation(&gCANHandle[CAN_PORT_INDEX], CANFDD_SEG1(CAN_PORT_INDEX,brs_bitrate), 0);
    HAL_FDCAN_EnableTxDelayCompensation(&gCANHandle[CAN_PORT_INDEX]);
	}
#endif

	// Only accept messages through filter, reject all others and also reject all remote frames.
	if (HAL_FDCAN_ConfigGlobalFilter(&gCANHandle[CAN_PORT_INDEX], FDCAN_REJECT, FDCAN_REJECT, FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE) != HAL_OK)
  {
    /* Initialization Error */
    return 0;
  }

  /*##-2- Start the Reception process and enable reception interrupt #########*/
  /* Start the FDCAN module */
  if (HAL_FDCAN_Start(&gCANHandle[CAN_PORT_INDEX]) != HAL_OK)
  {
    /* Reception Error */
    return 0;
  }
  if (HAL_FDCAN_ActivateNotification(&gCANHandle[CAN_PORT_INDEX], FDCAN_NOTIFICATIONS, 0) != HAL_OK)
  {
    /* Configuration Error */
    return 0;
  }

#if !defined(MCOHW_NO_TIMER_INIT)
  // TIMER INITIALISATION

   /* Compute the prescaler value to have TIMx counter clock equal to 10000 Hz */
  uwPrescalerValue = (uint32_t)(SystemCoreClock / (2*10000)) - 1;

  /* Set TIMx instance */
  gMcopTimHandle.Instance = TIMx;

    /* Initialize TIMx peripheral as follows:
       + Period = 10 - 1
       + Prescaler = (SystemCoreClock/10000) - 1
       + ClockDivision = 0
       + Counter direction = Up
  */
  gMcopTimHandle.Init.Period            = 10 - 1;
  gMcopTimHandle.Init.Prescaler         = uwPrescalerValue;
  gMcopTimHandle.Init.ClockDivision     = 0;
  gMcopTimHandle.Init.CounterMode       = TIM_COUNTERMODE_UP;
  gMcopTimHandle.Init.RepetitionCounter = 0;
  if (HAL_TIM_Base_Init(&gMcopTimHandle) != HAL_OK)
  {
    /* Initialization Error */
    return 0;
  }

  /*##-2- Start the TIMx Base generation in interrupt mode ####################*/
  /* Start Channel1 */
  if (HAL_TIM_Base_Start_IT(&gMcopTimHandle) != HAL_OK)
  {
    /* Starting Error */
    return 0;
  }
#endif // !defined(MCOHW_NO_TIMER_INIT)

  // Initialize the number of the first filter to use
  mNextFilterNum[CAN_PORT_INDEX] = FIRST_RXFILTER_NUM(CAN_PORT_INDEX,useFD);
  // Initialize other status variables
  gCanRxPending[CAN_PORT_INDEX] = FALSE;
  gCanMgrRxPending[CAN_PORT_INDEX] = FALSE;
  
#if (USE_CANSWFILTER > 0)
  // Init CAN receive SW filter
  CANSWFILTER_Init(OPT_SINGLE_HANDLE_CALL_PARAM);
#endif

#if (USE_CANFIFO > 0)
#if (TXFIFOSIZE > 0)
  // Init Tx FIFO
  CANTXFIFO_Flush(OPT_SINGLE_HANDLE_CALL_PARAM);
#endif  

#if (RXFIFOSIZE > 0)
  // Init RxFIFO
  CANRXFIFO_Flush(OPT_SINGLE_HANDLE_CALL_PARAM);
#endif  

#if USE_CANOPEN_MANAGER && (MGRFIFOSIZE > 0)
  // Init MGRFIFO
  CANMGRFIFO_Flush(OPT_SINGLE_HANDLE_CALL_PARAM);
#endif  
#endif

  // Init HW status variable
  HW_STATUS_USED |= HW_INIT;

  return 1;
}


/**************************************************************************
DOES:    This function implements the initialization of a CAN ID hardware
         filter as supported by many CAN controllers.
RETURNS: 1 if filter was set
         2 if this HW does not support filters
           (in this case HW will receive EVERY CAN message)
         0 if no more filter is available
**************************************************************************/
uint8_t MCOHW_SetCANFilter
  (
  OPT_FIRST_HANDLE_PARAM // Stack handle (only in multi-instance environment)
  COBID_TYPE CANID
  )
{
  uint8_t useFD = FALSE;
  FDCAN_FilterTypeDef sFilterConfig;

  (void)useFD; // avoid 'unused' compiler warning
#if PROCESS_CO_FD
  if (MCO_PROT_BIT_SET(handle, MCO_PROT_FD))
  {
    useFD = TRUE;
  }
#endif

	if (mNextFilterNum[CAN_PORT_INDEX] > LAST_RXFILTER_NUM(CAN_PORT_INDEX,UseFD))
		return 0;
	
  sFilterConfig.IdType = FDCAN_STANDARD_ID;
  sFilterConfig.FilterIndex = mNextFilterNum[CAN_PORT_INDEX];
  sFilterConfig.FilterType = FDCAN_FILTER_MASK;
  sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
  sFilterConfig.FilterID1 = CANID;
  sFilterConfig.FilterID2 = 0x7FF;
  if (HAL_FDCAN_ConfigFilter(&gCANHandle[CAN_PORT_INDEX], &sFilterConfig) != HAL_OK)
  {
    /* Filter configuration Error */
    return 0;
  }
	mNextFilterNum[CAN_PORT_INDEX]++;
	
	return 1;
}


/**************************************************************************
DOES:    This function implements the deletion of a previously set CAN ID
         hardware filter as supported by many CAN controllers.
RETURNS: 1 if filter was deleted
         0 if filter could not be deleted
**************************************************************************/
uint8_t MCOHW_ClearCANFilter
  (
  OPT_FIRST_HANDLE_PARAM // Stack handle (only in multi-instance environment)
  COBID_TYPE CANID
  )
{
  // clearing of hardware filter by ID is not supported, as a consequence,
	// the stack may have to ignore some messages, e.g. previous RPDOs
	return 0;
}


/**************************************************************************
DOES:    This function implements the initialization of a CAN ID hardware
         filter as supported by many CAN controllers.
         Allows a range of identifiers to be received
RETURNS: 1 if filter was set
         2 if this HW does not support filters
           (in this case HW will receive EVERY CAN message)
         0 if no more filter is available
**************************************************************************/
uint8_t MCOHW_SetCANFilterRange (
  OPT_FIRST_HANDLE_PARAM // Stack handle (only in multi-instance environment)
  COBID_TYPE CANID_Start,  // start of identifier range
  COBID_TYPE CANID_End     // end of identifier range
  )
{
  uint8_t useFD = FALSE;
  FDCAN_FilterTypeDef sFilterConfig;

  (void)useFD; // avoid 'unused' compiler warning
#if PROCESS_CO_FD
  if (MCO_PROT_BIT_SET(handle, MCO_PROT_FD))
  {
    useFD = TRUE;
  }
#endif

	if (mNextFilterNum[CAN_PORT_INDEX] > LAST_RXFILTER_NUM(CAN_PORT_INDEX,UseFD))
		return 0;
	
  sFilterConfig.IdType = FDCAN_STANDARD_ID;
  sFilterConfig.FilterIndex = mNextFilterNum[CAN_PORT_INDEX];
  sFilterConfig.FilterType = FDCAN_FILTER_RANGE;
  sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
  sFilterConfig.FilterID1 = CANID_Start;
  sFilterConfig.FilterID2 = CANID_End;
  if (HAL_FDCAN_ConfigFilter(&gCANHandle[CAN_PORT_INDEX], &sFilterConfig) != HAL_OK)
  {
    /* Filter configuration Error */
    return 0;
  }
	mNextFilterNum[CAN_PORT_INDEX]++;
	
	return 1;
}


#if USE_CANOPEN_MANAGER
/**************************************************************************
DOES:    This function implements an additional CAN receive filter
         used by the manager. Messages received using this ID are pulled
         by the manager using function MCOHWMGR_PullMessage
         Filter set receives messages from 0x81 to 0xFF and 0x581 to 0x5FF
RETURNS: TRUE or FALSE, if filter was not set
**************************************************************************/
uint8_t MCOHWMGR_SetCANFilter
  (
  OPT_SINGLE_HANDLE_PARAM // Stack handle (only in multi-instance environment)
  )
{
  uint8_t useFD = FALSE;
  FDCAN_FilterTypeDef sFilterConfig;

  (void)useFD; // avoid 'unused' compiler warning
#if PROCESS_CO_FD
  if (MCO_PROT_BIT_SET(handle, MCO_PROT_FD))
  {
    useFD = TRUE;
  }
#endif

  // In this driver no SW filters are used, HW receive-all only
  sFilterConfig.IdType = FDCAN_STANDARD_ID;
  sFilterConfig.FilterIndex = RXFIFO1_RXFILTER_NUM(CAN_PORT_INDEX,UseFD);
  sFilterConfig.FilterType = FDCAN_FILTER_MASK;
  sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO1;
  sFilterConfig.FilterID1 = 0x000;
  sFilterConfig.FilterID2 = 0x000;
  if (HAL_FDCAN_ConfigFilter(&gCANHandle[CAN_PORT_INDEX], &sFilterConfig) != HAL_OK)
  {
    /* Filter configuration Error */
    return FALSE;
  }

  return TRUE;
}


/**************************************************************************
DOES:    This function implements the initialization of a CAN ID hardware
         filter as supported by many CAN controllers.
         Allows a range of identifiers to be received
RETURNS: 1 if filter was set
         2 if this HW does not support filters
           (in this case HW will receive EVERY CAN message)
         0 if no more filter is available
**************************************************************************/
uint8_t MCOHWMGR_SetCANFilterRange(
  OPT_FIRST_HANDLE_PARAM // Stack handle (only in multi-instance environment)
  COBID_TYPE CANID_Start,                                  // start of identifier range
  COBID_TYPE CANID_End                                     // end of identifier range
  )
{
  // dummy, no dedicated filters used
  return 2;
}
#endif // USE_CANOPEN_MANAGER

#if USE_SYNC_PRODUCER
/**************************************************************************
DOES:    Activates the SYNC producer.
NOTE:    Sync signal is only produced while MY_NMT_STATE is NMTSTATE_OP.
RETURNS: Nothing.
**************************************************************************/
void MCOHW_SetSyncProducer(
  OPT_FIRST_HANDLE_PARAM // Stack handle (only in multi-instance environment)
  COBID_TYPE canid, // CAN ID used for SYNC production
  uint32_t cycle,   // SYNC cycle time in microseconds
  uint8_t cntovr    // SYNC overflow counter, zero for none used
)
{
  // Init SYNC production
  CONFIG_USED.SYNC_cycle = cycle;
  CONFIG_USED.SYNC_id = canid;
  CONFIG_USED.SYNC_cntovr = cntovr;
  mSYNCprod[CAN_PORT_INDEX].cnt = cntovr + 1;
}
#endif


/**************************************************************************
DOES:    This function reads a 1 millisecond timer tick. The timer tick
         must be a uint16_t and must be incremented once per millisecond.
RETURNS: 1 millisecond timer tick
**************************************************************************/
uint16_t MCOHW_GetTime
  (
  void
  )
{
  return gTimCnt;
}


/**************************************************************************
DOES:    This function compares a uint16_t timestamp to the internal
         timer tick and returns 1 if the timestamp expired/passed.
RETURNS: 1 if timestamp expired/passed
         0 if timestamp is not yet reached
NOTES:   The maximum timer runtime measurable is 0x8000 (about 32 seconds).
         For the usage in MicroCANopen that is sufficient.
**************************************************************************/
uint8_t MCOHW_IsTimeExpired
  (
  uint16_t timestamp
  )
{
  uint16_t time_now;

  time_now = gTimCnt;
  timestamp--;
  if (time_now > timestamp)
  {
    if ((time_now - timestamp) < 0x8000)
      return 1;
    else
      return 0;
  }
  else
  {
    if ((timestamp - time_now) >= 0x8000)
      return 1;
    else
      return 0;
  }
}


#if USE_SLEEP
/**************************************************************************
DOES:    Sets the processor into sleep or power down mode.
         Called when a sleep request was received and confirmed.
         Wakeup MUST be through a reset.
**************************************************************************/
void MCOHW_Sleep
  (
  void
  )
{ // Here: simplified implementation, waiting for next CAN message
  while (CANRXFIFO_GetOutPtr() != 0)
  {
  }
  // Do a full reset now
  MCOUSER_ResetApplication();
}
#endif
/*----------------------- END OF FILE ----------------------------------*/
