/**
  ******************************************************************************
  * @file    xpd_can.h
  * @author  Benedek Kupper
  * @version 0.3
  * @date    2018-01-28
  * @brief   STM32 eXtensible Peripheral Drivers CAN Module
  *
  * Copyright (c) 2018 Benedek Kupper
  *
  * Licensed under the Apache License, Version 2.0 (the "License");
  * you may not use this file except in compliance with the License.
  * You may obtain a copy of the License at
  *
  *     http://www.apache.org/licenses/LICENSE-2.0
  *
  * Unless required by applicable law or agreed to in writing, software
  * distributed under the License is distributed on an "AS IS" BASIS,
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  */
#ifndef __XPD_CAN_H_
#define __XPD_CAN_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <xpd_common.h>
#include <xpd_rcc.h>

#if defined(CAN) || defined(CAN1)

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

/** @brief CAN bit timing configuration */
typedef struct
{
    uint16_t Prescaler; /*!< CAN bus sampler clock prescaler from PCLKx. Permitted values: @arg 1 .. 1024 */
    uint8_t  BS1;       /*!< Bit segment 1 of bit timing. Permitted values: @arg 1 .. 16 */
    uint8_t  BS2;       /*!< Bit segment 2 of bit timing. Permitted values: @arg 1 .. 8 */
    uint8_t  SJW;       /*!< Synchronization jump width. Permitted values: @arg 1 .. 4 */
}CAN_TimingConfigType;

/** @brief CAN setup structure */
typedef struct
{
    CAN_TimingConfigType Timing;
    union {
    struct {
    CAN_ModeType    Mode : 2;             /*!< CAN test mode selection */
    FunctionalState TxFifoMode : 1;       /*!< Transmit mailboxes are sent in FIFO instead of ID priority order */
    FunctionalState RxFifoLocked : 1;     /*!< Receive FIFO Locked Mode keeps older messages in full FIFO */
    FunctionalState NoAutoRetransmit : 1; /*!< Non Automatic ReTransmission */
    FunctionalState AutoWakeup : 1;       /*!< Automatic Wake-Up on message detection */
    FunctionalState AutoBusRecovery : 1;  /*!< Automatic Bus-Off Recovery */
    FunctionalState TTCAN : 1;            /*!< Time-Triggered Communication Mode */
    };
    uint8_t wSettings;            /*!< Set to 0 for default features */
    };
}CAN_InitType;

/** @brief CAN Identifier types */
typedef enum
{
    CAN_IDTYPE_STD_DATA   = 0,   /*!< Standard (11 bit Id) data */
    CAN_IDTYPE_EXT_DATA   = 4,   /*!< Extended (29 bit Id) data */
    CAN_IDTYPE_STD_RTR    = 2,   /*!< Standard (11 bit Id) remote transmit request */
    CAN_IDTYPE_EXT_RTR    = 6    /*!< Extended (29 bit Id) remote transmit request */
}CAN_IdType;

/** @brief CAN Identifier structure */
typedef struct
{
    uint32_t   Value;           /*!< Identifier field numeric value */
    CAN_IdType Type;            /*!< ID and data type (Std/Ext, Data/RTR) */
}CAN_IdentifierFieldType;

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
                                          for pairing with acceptance filter
                                     @arg Transmitted frames: Mailbox Index */
}CAN_FrameType;

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
}CAN_ErrorType;

/** @brief CAN Filter types */
typedef enum
{
    CAN_FILTER_MATCH        = 0x01, /*!< Filter accepts messages with identical frame type and Identifier */
    CAN_FILTER_MASK         = 0x06, /*!< Filter accepts messages with identical frame type
                                         and matching Identifier field values on the mask-selected bits */
    CAN_FILTER_MASK_ANYTYPE = 0x00, /*!< Filter accepts messages with matching Identifier field
                                         values on the mask-selected bits, of all frame types */
}CAN_FilterModeType;

/** @brief CAN Filter structure */
typedef struct
{
    uint32_t                Mask;    /*!< Identifier mask which selects which Identifier field bits are checked */
    CAN_IdentifierFieldType Pattern; /*!< Identifier pattern which is expected by the filter */
    CAN_FilterModeType      Mode;    /*!< Filter mode */
    uint8_t                 FIFO;    /*!< The selected receive FIFO [0 .. 1]*/
}CAN_FilterType;

/** @brief CAN Handle structure */
typedef struct
{
    CAN_TypeDef * Inst;                    /*!< The address of the peripheral instance used by the handle */
#ifdef CAN_BB
    CAN_BitBand_TypeDef * Inst_BB;         /*!< The address of the peripheral instance in the bit-band region */
#endif
    struct {
        XPD_HandleCallbackType DepInit;    /*!< Callback to initialize module dependencies (GPIOs, IRQs) */
        XPD_HandleCallbackType DepDeinit;  /*!< Callback to restore module dependencies (GPIOs, IRQs) */
        XPD_HandleCallbackType Transmit;   /*!< Frame transmission successful callback */
        XPD_HandleCallbackType Receive[2]; /*!< Frame reception successful callback for each FIFO */
        XPD_HandleCallbackType Error;      /*!< Error detection callback */
    } Callbacks;                           /*   Handle Callbacks */
    CAN_FrameType * RxFrame[2];            /*!< [Internal] Pointers to where the received frames will be stored */
    RCC_PositionType CtrlPos;              /*!< Relative position for reset and clock control */
    volatile uint8_t State;                /*!< [Internal] CAN interrupt-controlled communication state */
}CAN_HandleType;

/** @} */

/** @defgroup CAN_Exported_Macros CAN Exported Macros
 * @{ */

#ifdef CAN_BB
/**
 * @brief CAN Instance to handle binder macro
 * @param HANDLE: specifies the peripheral handle.
 * @param INSTANCE: specifies the CAN peripheral instance.
 */
#define         CAN_INST2HANDLE(HANDLE,INSTANCE)                \
    ((HANDLE)->Inst    = (INSTANCE),                            \
     (HANDLE)->Inst_BB = CAN_BB(INSTANCE),                      \
     (HANDLE)->CtrlPos = RCC_POS_##INSTANCE)

/**
 * @brief CAN register bit accessing macro
 * @param HANDLE: specifies the peripheral handle.
 * @param REG_NAME: specifies the register name.
 * @param BIT_NAME: specifies the register bit name.
 */
#define         CAN_REG_BIT(HANDLE, REG_NAME, BIT_NAME)         \
    ((HANDLE)->Inst_BB->REG_NAME.BIT_NAME)

#else
/**
 * @brief CAN Instance to handle binder macro
 * @param HANDLE: specifies the peripheral handle.
 * @param INSTANCE: specifies the CAN peripheral instance.
 */
#define         CAN_INST2HANDLE(HANDLE,INSTANCE)                \
    ((HANDLE)->Inst    = (INSTANCE),                            \
     (HANDLE)->CtrlPos = RCC_POS_##INSTANCE)

/**
 * @brief CAN register bit accessing macro
 * @param HANDLE: specifies the peripheral handle.
 * @param REG_NAME: specifies the register name.
 * @param BIT_NAME: specifies the register bit name.
 */
#define         CAN_REG_BIT(HANDLE, REG_NAME, BIT_NAME)         \
    ((HANDLE)->Inst->REG_NAME.b.BIT_NAME)

#endif /* CAN_BB */

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
#define         CAN_IT_ENABLE(HANDLE, IT_NAME)                  \
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
#define         CAN_IT_DISABLE(HANDLE, IT_NAME)                 \
    (CAN_REG_BIT((HANDLE),IER,IT_NAME##IE) = 0)

/**
 * @brief  Get the specified CAN receive flag.
 * @param  HANDLE: specifies the CAN Handle.
 * @param  FIFO: specifies the CAN receive FIFO.
 * @param  FLAG_NAME: specifies the flag to return.
 *         This parameter can be one of the following values:
 *            @arg FMP:     FIFO Message Pending
 *            @arg FULL:    FIFO Full
 *            @arg FOVR:    FIFO Overrun
 * @return The state of the flag.
 */
#define         CAN_RXFLAG_STATUS(HANDLE, FIFO, FLAG_NAME)      \
    (CAN_REG_BIT((HANDLE),RFR[FIFO],FLAG_NAME))

/**
 * @brief  Clear the specified CAN receive flag.
 * @param  HANDLE: specifies the CAN Handle.
 * @param  FIFO: specifies the CAN receive FIFO.
 * @param  FLAG_NAME: specifies the flag to clear.
 *         This parameter can be one of the following values:
 *            @arg FMP:     FIFO Message Pending
 *            @arg FULL:    FIFO Full
 *            @arg FOVR:    FIFO Overrun
 */
#define         CAN_RXFLAG_CLEAR(HANDLE, FIFO, FLAG_NAME)       \
    ((HANDLE)->Inst->RFR[FIFO].w = CAN_RF0R_##FLAG_NAME##0)

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
#define         CAN_TXFLAG_STATUS(HANDLE, MB, FLAG_NAME)        \
    (((HANDLE)->Inst->TSR.w & (CAN_TSR_##FLAG_NAME##0 << (8 * (MB)))) != 0 ? 1 : 0)

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
#define         CAN_TXFLAG_CLEAR(HANDLE, MB, FLAG_NAME)         \
    ((HANDLE)->Inst->TSR.w = CAN_TSR_##FLAG_NAME##0 << (8 * (MB)))

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
#define         CAN_ERRFLAG_STATUS(HANDLE, FLAG_NAME)           \
    (CAN_REG_BIT((HANDLE),ESR,FLAG_NAME##F))

/**
 * @brief  Clear the specified CAN error flag.
 * @param  HANDLE: specifies the CAN Handle.
 * @param  FLAG_NAME: specifies the flag to clear.
 *         This parameter can be one of the following values:
 *            @arg EWG:     Error Warning
 *            @arg EPV:     Error Passive
 *            @arg BOF:     Bus Off
 */
#define         CAN_ERRFLAG_CLEAR(HANDLE, FLAG_NAME)            \
    ((HANDLE)->Inst->ESR.w = CAN_ESR_##FLAG_NAME##F)

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
#define         CAN_FLAG_STATUS(HANDLE, FLAG_NAME)              \
    (CAN_REG_BIT((HANDLE),MSR,FLAG_NAME))

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
#define         CAN_FLAG_CLEAR(HANDLE, FLAG_NAME)               \
    ((HANDLE)->Inst->MSR.w = CAN_MSR_##FLAG_NAME)

/** @} */

/** @addtogroup CAN_Exported_Functions
 * @{ */

XPD_ReturnType  CAN_eBitrateConfig      (uint32_t ulBitrate, CAN_TimingConfigType * pxTimingConfig);

/** @addtogroup CAN_Exported_Functions_State
 * @{ */
XPD_ReturnType  CAN_eInit               (CAN_HandleType * pxCAN, const CAN_InitType * pxConfig);
void            CAN_vDeinit             (CAN_HandleType * pxCAN);

XPD_ReturnType  CAN_eSleep              (CAN_HandleType * pxCAN);
XPD_ReturnType  CAN_eWakeUp             (CAN_HandleType * pxCAN);

CAN_ErrorType   CAN_eGetError           (CAN_HandleType * pxCAN);

void            CAN_vIRQHandlerSCE      (CAN_HandleType * pxCAN);
/** @} */

/** @addtogroup CAN_Exported_Functions_Filter
 * @{ */
XPD_ReturnType  CAN_eFilterBankConfig   (CAN_HandleType * pxCAN, uint8_t ucNewSize);
XPD_ReturnType  CAN_eFilterConfig       (CAN_HandleType * pxCAN, const CAN_FilterType axFilters[],
                                         uint8_t aucMatchIndexes[], uint8_t ucFilterCount);
/** @} */

/** @addtogroup CAN_Exported_Functions_Transmit
 * @{ */
XPD_ReturnType  CAN_ePost               (CAN_HandleType * pxCAN, CAN_FrameType * pxFrame,
                                         uint32_t ulTimeout);
XPD_ReturnType  CAN_eSend               (CAN_HandleType * pxCAN, CAN_FrameType * pxFrame,
                                         uint32_t ulTimeout);
XPD_ReturnType  CAN_eSend_IT            (CAN_HandleType * pxCAN, CAN_FrameType * pxFrame);

void            CAN_vIRQHandlerTX       (CAN_HandleType * pxCAN);
/** @} */

/** @addtogroup CAN_Exported_Functions_Receive
 * @{ */
XPD_ReturnType  CAN_eReceive            (CAN_HandleType * pxCAN, CAN_FrameType * pxFrame,
                                         uint8_t ucFIFONumber, uint32_t ulTimeout);
XPD_ReturnType  CAN_eReceive_IT         (CAN_HandleType * pxCAN, CAN_FrameType * pxFrame,
                                         uint8_t ucFIFONumber);

void            CAN_vIRQHandlerRX0      (CAN_HandleType * pxCAN);
void            CAN_vIRQHandlerRX1      (CAN_HandleType * pxCAN);
/** @} */

/** @} */

/** @} */

#endif /* defined(CAN) || defined(CAN1) */

#ifdef __cplusplus
}
#endif

#endif /* __XPD_CAN_H_ */
