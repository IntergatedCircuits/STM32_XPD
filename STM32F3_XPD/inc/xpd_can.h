/**
  ******************************************************************************
  * @file    xpd_can.h
  * @author  Benedek Kupper
  * @version V0.1
  * @date    2015-12-30
  * @brief   STM32 eXtensible Peripheral Drivers CAN Module
  *
  *  This file is part of STM32_XPD.
  *
  *  STM32_XPD is free software: you can redistribute it and/or modify
  *  it under the terms of the GNU General Public License as published by
  *  the Free Software Foundation, either version 3 of the License, or
  *  (at your option) any later version.
  *
  *  STM32_XPD is distributed in the hope that it will be useful,
  *  but WITHOUT ANY WARRANTY; without even the implied warranty of
  *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  *  GNU General Public License for more details.
  *
  *  You should have received a copy of the GNU General Public License
  *  along with STM32_XPD.  If not, see <http://www.gnu.org/licenses/>.
  */
#ifndef XPD_CAN_H_
#define XPD_CAN_H_

#include "xpd_common.h"
#include "xpd_config.h"

#if defined(CAN2)
#define __DUAL_CAN_DEVICE
#define CAN_FILTERBANK_NUMBER   28
#define CAN_MASTER              CAN1
#elif defined(CAN_FM1R_FBM13)
#define CAN_FILTERBANK_NUMBER   14
#define CAN_MASTER              CAN
#endif

#ifdef CAN_MASTER

/** @defgroup CAN
 * @{ */

/** @defgroup CAN_Exported_Types CAN Exported Types
 * @{ */

/** @brief CAN operating modes */
typedef enum
{
    CAN_MODE_NORMAL         = 0, /*!< CAN Rx and Tx are connected to the GPIO mux */
    CAN_MODE_LOOPBACK       = 1, /*!< CAN Rx is connected to Tx  */
    CAN_MODE_SILENT         = 2, /*!< CAN Tx is connected to Rx  */
    CAN_MODE_SILENTLOOPBACK = 3, /*!< CAN Tx and Rx are exclusively connected  */
}CAN_ModeType;

/** @brief CAN setup structure */
typedef struct
{
    struct {
        uint16_t Prescaler;   /*!< CAN bus sampler clock prescaler from PCLKx. Permitted values: @arg 1 .. 1024 */
        uint8_t  BS1;         /*!< Bit segment 1 of bit timing. Permitted values: @arg 1 .. 16 */
        uint8_t  BS2;         /*!< Bit segment 2 of bit timing. Permitted values: @arg 1 .. 8 */
        uint8_t  SJW;         /*!< Synchronization jump width. Permitted values: @arg 1 .. 4 */
    } Timing;                 /*   Bit timing configuration */
    union {
        struct {
            CAN_ModeType    Mode : 2;   /*!< CAN test mode selection */
            FunctionalState TXFP : 1;   /*!< Transmit FIFO Priority on request age instead of ID priority */
            FunctionalState RFLM : 1;   /*!< Receive FIFO Locked Mode keeps older messages in full FIFO */
            FunctionalState NART : 1;   /*!< Non Automatic ReTransmission */
            FunctionalState AWUM : 1;   /*!< Automatic Wake-Up on message detection */
            FunctionalState ABOM : 1;   /*!< Automatic Bus-Off Recovery */
            FunctionalState TTCM : 1;   /*!< Time-Triggered Communication Mode */
        } Individual;
        uint8_t All;                    /*!< Set to 0 for default features */
    }Features;                          /*   Features configuration */
} CAN_InitType;


/** @brief CAN Identifier types */
typedef enum
{
    CAN_IDTYPE_STD_DATA   = 0,   /*!< Standard (11 bit Id) data */
    CAN_IDTYPE_EXT_DATA   = 4,   /*!< Extended (29 bit Id) data */
    CAN_IDTYPE_STD_RTR    = 2,   /*!< Standard (11 bit Id) remote transmit request */
    CAN_IDTYPE_EXT_RTR    = 6    /*!< Extended (29 bit Id) remote transmit request */
} CAN_IdType;

/** @brief CAN Identifier structure */
typedef struct
{
    uint32_t   Value;           /*!< Identifier field numeric value */
    CAN_IdType Type;            /*!< ID and data type (Std/Ext, Data/RTR) */
} CAN_IdentifierFieldType;

/** @brief CAN Frame structure */
typedef struct
{
	union {
		uint8_t  Byte[8];          /*!< Data in byte format */
		uint32_t Word[2];          /*!< Data in word format */
	} Data;                        /*   Data field */
	CAN_IdentifierFieldType Id;    /*!< Frame Identifier */
    uint8_t                 DLC;   /*!< Data Length Code */
	uint8_t                 Index; /*!< This field has different use based on its direction:
	                                 @arg Received frames: Filter Match Index,
	                                      for pairing with accepter filters' @ref CAN_FilterType::MatchIndex
	                                 @arg Transmitted frames: Mailbox Index */
} CAN_FrameType;

/** @brief CAN Error types */
typedef enum
{
	CAN_ERROR_NONE         = 0x00, /*!< No error */
	CAN_ERROR_STUFF        = 0x10, /*!< Stuff error */
	CAN_ERROR_FORM         = 0x20, /*!< Form error */
	CAN_ERROR_ACK          = 0x30, /*!< Acknowledge error */
	CAN_ERROR_BITRECESSIVE = 0x40, /*!< Bit recessive error */
	CAN_ERROR_BITDOMINANT  = 0x50, /*!< Bit dominant error */
	CAN_ERROR_CRC          = 0x60, /*!< CRC error */
    CAN_ERROR_ERRORWARNING = 0x01, /*!< Error warning state */
    CAN_ERROR_ERRORPASSIVE = 0x02, /*!< Error passive state */
    CAN_ERROR_BUSOFF       = 0x04, /*!< Bus off state */
} CAN_ErrorType;

/** @brief CAN Filter types */
typedef enum
{
    CAN_FILTER_MATCH        = 0x01, /*!< Filter accepts messages with identical Identifier */
    CAN_FILTER_MASK         = 0x00, /*!< Filter accepts messages with identical Identifier type
                                         and matching Identifier field values on the mask-selected bits */
    CAN_FILTER_MASK_ANYTYPE = 0x08, /*!< Filter accepts messages with matching Identifier field values on the mask-selected bits */
} CAN_FilterModeType;

/** @brief CAN Filter structure */
typedef struct
{
    uint8_t                 MatchIndex;    /*!< This value is set by @ref XPD_CAN_FilterInit function,
                                                the user can use the value for matching the filter with the received frames' FMI */
    CAN_FilterModeType      Mode;          /*!< Filter mode */
    uint32_t                Mask;          /*!< Identifier mask which selects which Identifier field bits are checked */
    CAN_IdentifierFieldType Pattern;       /*!< Identifier pattern which is expected by the filter */
} CAN_FilterType;

/** @brief CAN Handle structure */
typedef struct
{
	CAN_TypeDef *Inst;                     /*!< The address of the peripheral instance used by the handle */
#ifdef CAN_BB
	CAN_BitBand_TypeDef *Inst_BB;          /*!< The address of the peripheral instance in the bit-band region */
#endif
    XPD_CtrlFnType ClockCtrl;              /*!< Function pointer for RCC clock control */
	struct {
	    XPD_HandleCallbackType DepInit;    /*!< Callback to initialize module dependencies (GPIOs, IRQs) */
	    XPD_HandleCallbackType DepDeinit;  /*!< Callback to restore module dependencies (GPIOs, IRQs) */
	    XPD_HandleCallbackType Transmit;   /*!< Frame transmission successful callback */
	    XPD_HandleCallbackType Receive[2]; /*!< Frame reception successful callback for each FIFO */
	    XPD_HandleCallbackType Error;      /*!< Error detection callback */
	} Callbacks;                           /*   Handle Callbacks */
	CAN_FrameType* RxFrame[2];             /*!< [Internal] Pointers to where the received frames will be stored */
    volatile uint8_t State;                /*!< [Internal] CAN interrupt-controlled communication state */
} CAN_HandleType;

/** @} */

/** @defgroup CAN_Exported_Macros CAN Exported Macros
 * @{ */

/**
 * @brief  CAN Handle initializer macro
 * @param  INSTANCE: specifies the CAN peripheral instance.
 * @param  INIT_FN: specifies the dependency initialization function to call back.
 * @param  DEINIT_FN: specifies the dependency deinitialization function to call back.
 */
#define         NEW_CAN_HANDLE(INSTANCE,INIT_FN,DEINIT_FN)      \
    {.Inst      = (INSTANCE),                                   \
     .Callbacks = {(INIT_FN),(DEINIT_FN),NULL,{NULL,NULL},NULL},\
     .ClockCtrl = XPD_##INSTANCE##_ClockCtrl}

/**
 * @brief  Enable the specified CAN interrupt.
 * @param  HANDLE: specifies the CAN Handle.
 * @param  IT_NAME: specifies the interrupt to enable.
 *         This parameter can be one of the following values:
 *            @arg TME:     Transmit Mailbox Empty Interrupt
 *            @arg FMP0:    FIFO 0 Message Pending Interrupt
 *            @arg FF0:     FIFO 0 Full Interrupt
 *            @arg FOV0:    FIFO 0 Overrun Interrupt
 *            @arg FMP1:    FIFO 1 Message Pending Interrupt
 *            @arg FF1:     FIFO 1 Full Interrupt
 *            @arg FOV1:    FIFO 1 Overrun Interrupt
 *            @arg EWG:     Error Warning Interrupt
 *            @arg EPV:     Error Passive Interrupt
 *            @arg BOF:     Bus-Off Interrupt
 *            @arg LEC:     Last Error Code Interrupt
 *            @arg ERR:     Error Interrupt
 *            @arg WKU:     Wakeup Interrupt
 *            @arg SLK:     Sleep Interrupt
 */
#define         XPD_CAN_EnableIT(HANDLE,IT_NAME)          	\
    (CAN_REG_BIT((HANDLE),IER,IT_NAME##IE) = 1)

/**
 * @brief  Disable the specified CAN interrupt.
 * @param  HANDLE: specifies the CAN Handle.
 * @param  IT_NAME: specifies the interrupt to disable.
 *         This parameter can be one of the following values:
 *            @arg TME:     Transmit Mailbox Empty Interrupt
 *            @arg FMP0:    FIFO 0 Message Pending Interrupt
 *            @arg FF0:     FIFO 0 Full Interrupt
 *            @arg FOV0:    FIFO 0 Overrun Interrupt
 *            @arg FMP1:    FIFO 1 Message Pending Interrupt
 *            @arg FF1:     FIFO 1 Full Interrupt
 *            @arg FOV1:    FIFO 1 Overrun Interrupt
 *            @arg EWG:     Error Warning Interrupt
 *            @arg EPV:     Error Passive Interrupt
 *            @arg BOF:     Bus-Off Interrupt
 *            @arg LEC:     Last Error Code Interrupt
 *            @arg ERR:     Error Interrupt
 *            @arg WKU:     Wakeup Interrupt
 *            @arg SLK:     Sleep Interrupt
 */
#define         XPD_CAN_DisableIT(HANDLE,IT_NAME)           \
    (CAN_REG_BIT((HANDLE),IER,IT_NAME##IE) = 0)

/**
 * @brief  Clear the specified CAN receive flag.
 * @param  HANDLE: specifies the CAN Handle.
 * @param  FIFO: specifies the CAN receive FIFO.
 * @param  FLAG_NAME: specifies the flag to clear.
 *         This parameter can be one of the following values:
 *            @arg FMP:     FIFO Message Pending
 *            @arg FULL:    FIFO Full
 *            @arg FOV:     FIFO Overrun
 */
#define         XPD_CAN_ClearRxFlag(HANDLE,FIFO,FLAG_NAME)  \
    ((HANDLE)->Inst->RFR[FIFO].w = CAN_RF0R_##FLAG_NAME##0)

/**
 * @brief  Get the specified CAN receive flag.
 * @param  HANDLE: specifies the CAN Handle.
 * @param  FIFO: specifies the CAN receive FIFO.
 * @param  FLAG_NAME: specifies the flag to return.
 *         This parameter can be one of the following values:
 *            @arg FMP:     FIFO Message Pending
 *            @arg FULL:    FIFO Full
 *            @arg FOV:     FIFO Overrun
 * @return The state of the flag.
 */
#define         XPD_CAN_GetRxFlag(HANDLE,FIFO,FLAG_NAME)    \
    (CAN_REG_BIT((HANDLE),RFR[(FIFO)],FLAG_NAME))

/**
 * @brief  Clear the specified CAN transmit flag.
 * @param  HANDLE: specifies the CAN Handle.
 * @param  MB: specifies the mailbox index.
 * @param  FLAG_NAME: specifies the flag to clear.
 *         This parameter can be one of the following values:
 *            @arg RQCP:    Request Completed
 *            @arg TXOK:    Transmission OK
 *            @arg ALST:    Arbitration Lost
 *            @arg TERR:    Transmission Error
 *            @arg ABRQ:    Abort Request
 */
#define         XPD_CAN_ClearTxFlag(HANDLE,MB,FLAG_NAME)    \
        ((HANDLE)->Inst->TSR.w = CAN_TSR_##FLAG_NAME##0 << (8 * (MB)))

/**
 * @brief  Get the specified CAN transmit flag.
 * @param  HANDLE: specifies the CAN Handle.
 * @param  MB: specifies the mailbox index.
 * @param  FLAG_NAME: specifies the flag to return.
 *         This parameter can be one of the following values:
 *            @arg RQCP:    Request Completed
 *            @arg TXOK:    Transmission OK
 *            @arg ALST:    Arbitration Lost
 *            @arg TERR:    Transmission Error
 *            @arg ABRQ:    Abort Request
 * @return The state of the flag.
 */
#define         XPD_CAN_GetTxFlag(HANDLE,MB,FLAG_NAME)      \
        (((HANDLE)->Inst->TSR.w & (CAN_TSR_##FLAG_NAME##0 << (8 * (MB)))) != 0 ? 1 : 0)

/**
 * @brief  Clear the specified CAN error flag.
 * @param  HANDLE: specifies the CAN Handle.
 * @param  FLAG_NAME: specifies the flag to clear.
 *         This parameter can be one of the following values:
 *            @arg EWG:     Error Warning
 *            @arg EPV:     Error Passive
 *            @arg BOF:     Bus Off
 */
#define         XPD_CAN_ClearErrorFlag(HANDLE,FLAG_NAME)    \
    ((HANDLE)->Inst->ESR.w = CAN_ESR_##FLAG_NAME)

/**
 * @brief  Get the specified CAN error flag.
 * @param  HANDLE: specifies the CAN Handle.
 * @param  FLAG_NAME: specifies the flag to return.
 *         This parameter can be one of the following values:
 *            @arg EWG:     Error Warning
 *            @arg EPV:     Error Passive
 *            @arg BOF:     Bus Off
 * @return The state of the flag.
 */
#define         XPD_CAN_GetErrorFlag(HANDLE,FLAG_NAME)      \
    (CAN_REG_BIT((HANDLE),ESR,FLAG_NAME))

/**
 * @brief  Clear the specified CAN state flag.
 * @param  HANDLE: specifies the CAN Handle.
 * @param  FLAG_NAME: specifies the flag to clear.
 *         This parameter can be one of the following values:
 *            @arg WKUI:    Wakeup Interrupt
 *            @arg SLAKI:   Sleep Acknowledge Interrupt
 *            @arg ERRI:    Error Interrupt
 *            @arg SLAK:    Sleep Acknowledge
 *            @arg INAK:    Initialization Acknowledge
 */
#define         XPD_CAN_ClearFlag(HANDLE,FLAG_NAME)         \
    ((HANDLE)->Inst->MSR.w = CAN_MSR_##FLAG_NAME)

/**
 * @brief  Get the specified CAN state flag.
 * @param  HANDLE: specifies the CAN Handle.
 * @param  FLAG_NAME: specifies the flag to return.
 *         This parameter can be one of the following values:
 *            @arg WKUI:    Wakeup Interrupt
 *            @arg SLAKI:   Sleep Acknowledge Interrupt
 *            @arg ERRI:    Error Interrupt
 *            @arg SLAK:    Sleep Acknowledge
 *            @arg INAK:    Initialization Acknowledge
 * @return The state of the flag.
 */
#define         XPD_CAN_GetFlag(HANDLE,FLAG_NAME)           \
    (CAN_REG_BIT((HANDLE),MSR,FLAG_NAME))


#ifdef CAN_BB
#define CAN_REG_BIT(HANDLE, REG_NAME, BIT_NAME) ((HANDLE)->Inst_BB->REG_NAME.BIT_NAME)
#define CAN_MASTER_REG_BIT(REG_NAME, BIT_NAME)  (CAN_BB(CAN_MASTER)->REG_NAME.BIT_NAME)
#else
#define CAN_REG_BIT(HANDLE, REG_NAME, BIT_NAME) ((HANDLE)->Inst->REG_NAME.b.BIT_NAME)
#define CAN_MASTER_REG_BIT(REG_NAME, BIT_NAME)  (CAN_MASTER->REG_NAME.b.BIT_NAME)
#endif

/** @} */

/** @addtogroup CAN_Exported_Functions
 * @{ */

/** @addtogroup CAN_Exported_Functions_IRQ
 * @{ */
void            XPD_CAN_TX_IRQHandler       (CAN_HandleType * hcan);
void            XPD_CAN_RX0_IRQHandler      (CAN_HandleType * hcan);
void            XPD_CAN_RX1_IRQHandler      (CAN_HandleType * hcan);
void            XPD_CAN_SCE_IRQHandler      (CAN_HandleType * hcan);
/** @} */

/** @addtogroup CAN_Exported_Functions_State
 * @{ */
XPD_ReturnType  XPD_CAN_Init                (CAN_HandleType * hcan, CAN_InitType * Config);
XPD_ReturnType  XPD_CAN_Deinit              (CAN_HandleType * hcan);

XPD_ReturnType  XPD_CAN_Sleep               (CAN_HandleType * hcan);
XPD_ReturnType  XPD_CAN_WakeUp              (CAN_HandleType * hcan);

CAN_ErrorType   XPD_CAN_GetError            (CAN_HandleType * hcan);
/** @} */

/** @addtogroup CAN_Exported_Functions_Filter
 * @{ */
XPD_ReturnType  XPD_CAN_FilterInit          (CAN_HandleType * hcan, uint8_t FIFONumber, CAN_FilterType * Filter);
XPD_ReturnType  XPD_CAN_FilterDeinit        (CAN_HandleType * hcan, uint8_t FIFONumber, uint8_t * MatchIndex);
XPD_ReturnType  XPD_CAN_FilterIndexUpdate   (CAN_HandleType * hcan, uint8_t FIFONumber, uint8_t * MatchIndex);
void            XPD_CAN_FilterBankReset     (CAN_HandleType * hcan);
#ifdef __DUAL_CAN_DEVICE
XPD_ReturnType  XPD_CAN_FilterBankSizeConfig(CAN_HandleType* hcan, uint8_t NewSize);
#endif
/** @} */

/** @addtogroup CAN_Exported_Functions_Transmit
 * @{ */
XPD_ReturnType  XPD_CAN_Transmit            (CAN_HandleType * hcan, CAN_FrameType * Frame, uint32_t Timeout);
XPD_ReturnType  XPD_CAN_Transmit_IT         (CAN_HandleType * hcan, CAN_FrameType * Frame);
/** @} */

/** @addtogroup CAN_Exported_Functions_Receive
 * @{ */
XPD_ReturnType  XPD_CAN_Receive             (CAN_HandleType * hcan, CAN_FrameType * Frame, uint8_t FIFONumber, uint32_t Timeout);
XPD_ReturnType  XPD_CAN_Receive_IT          (CAN_HandleType * hcan, CAN_FrameType * Frame, uint8_t FIFONumber);
/** @} */

/** @} */

/** @} */

#define XPD_CAN_API
#include "xpd_rcc_gen.h"
#undef XPD_CAN_API

#endif /* CAN_MASTER */

#endif /* XPD_CAN_H_ */
