/**
  ******************************************************************************
  * @file    xpd_can.c
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
#include <xpd_can.h>
#include <xpd_utils.h>

#if defined(CAN) || defined(CAN1)

/** @addtogroup CAN
 * @{ */

#define CAN_STATE_TRANSMIT      0x0F
#define CAN_STATE_RECEIVE       0x30
#define CAN_STATE_RECEIVE0      0x10
#define CAN_STATE_RECEIVE1      0x20

/* Timeout for INAK bit [ms] */
#define INAK_TIMEOUT      (1000)
/* Timeout for SLAK bit [ms] */
#define SLAK_TIMEOUT      (10)

#ifdef __XPD_CAN_ERROR_DETECT
#define CAN_ERROR_INTERRUPTS    (CAN_IER_BOFIE | CAN_IER_EPVIE | CAN_IER_EWGIE | CAN_IER_LECIE)
#else
#define CAN_ERROR_INTERRUPTS     0
#endif
#define CAN_RECEIVE0_INTERRUPTS  CAN_IER_FMPIE0
#define CAN_RECEIVE1_INTERRUPTS  CAN_IER_FMPIE1
#define CAN_TRANSMIT_INTERRUPTS  CAN_IER_TMEIE

#if defined(CAN3)
#define CAN_MASTER(HANDLE)                                      \
    (((HANDLE)->Inst == CAN3) ? CAN3 : CAN1)

#define FILTERBANK_NUMBER(HANDLE)                               \
    (((HANDLE)->Inst == CAN3) ? 14 : 28)

#define FILTERBANK_OFFSET(HANDLE)                               \
    (((HANDLE)->Inst == CAN2) ? CAN1->FMR.b.CAN2SB : 0)

#define FILTERBANK_COUNT(HANDLE)                                \
    (((HANDLE)->Inst == CAN1) ? (CAN1->FMR.b.CAN2SB) :          \
    (((HANDLE)->Inst == CAN2) ? (28 - CAN1->FMR.b.CAN2SB) : 14))

#elif defined(CAN2)
#define CAN_MASTER(HANDLE)              (CAN1)
#define FILTERBANK_NUMBER(HANDLE)       28

#define FILTERBANK_OFFSET(HANDLE)                               \
    (((HANDLE)->Inst == CAN1) ? 0 : CAN1->FMR.b.CAN2SB)

#define FILTERBANK_COUNT(HANDLE)                                \
    (((HANDLE)->Inst == CAN1) ? (CAN1->FMR.b.CAN2SB) : (28 - CAN1->FMR.b.CAN2SB))

#define CAN1_SLAVE_FILTERS_ACTIVE()                                  \
    ((CAN1->FA1R & (~((1 << CAN1->FMR.b.CAN2SB) - 1))) != 0)

#else
#define CAN_MASTER(HANDLE)              (CAN)
#define FILTERBANK_NUMBER(HANDLE)       14
#define FILTERBANK_OFFSET(HANDLE)       0
#define FILTERBANK_COUNT(HANDLE)        (FILTERBANK_NUMBER(HANDLE))

#endif

#ifdef CAN_BB
#define CANx_REG_BIT(REG_NAME, BIT_NAME)  \
    (CANx_BB->REG_NAME.BIT_NAME)
#else
#define CANx_REG_BIT(REG_NAME, BIT_NAME)  \
    (CANx->REG_NAME.b.BIT_NAME)
#endif

#define CAN_INPUT_CLOCK_RATE    \
    (RCC_ulClockFreq_Hz(PCLK1))

/* Filter types */
#define FILTER_SIZE_FLAG_Pos    1
#define FILTER_SIZE_FLAG        2
#define FILTER_MODE_FLAG_Pos    0
#define FILTER_MODE_FLAG        1
#define FMI_INVALID             0xFF

static const uint8_t can_aucFilterTypeSpace[] = {2, 1, 4, 2};

/** @defgroup CAN_Private_Functions CAN Private Functions
 * @{ */

/**
 * @brief Gets an empty transmit mailbox.
 * @param pxCAN: pointer to the CAN handle structure
 * @param pucMb: Set to the lowest number empty mailbox
 * @return BUSY if all mailboxes are full, OK if empty mailbox found
 */
__STATIC_INLINE XPD_ReturnType CAN_prvGetEmptyMailbox(CAN_HandleType * pxCAN, uint8_t * pucMb)
{
    XPD_ReturnType eResult = XPD_BUSY;
    uint32_t ulTSR = pxCAN->Inst->TSR.w;

    /* Check if at least one mailbox is empty */
    if ((ulTSR & CAN_TSR_TME) != 0)
    {
        *pucMb = (ulTSR & CAN_TSR_CODE) >> CAN_TSR_CODE_Pos;
        eResult = XPD_OK;
    }
    return eResult;
}

/**
 * @brief Puts the frame data in an empty transmit mailbox, and requests transmission.
 * @param pxCAN: pointer to the CAN handle structure
 * @param pxFrame: pointer to the frame to transmit
 * @return BUSY if all mailboxes are in use, OK if transmission scheduling was successful
 */
static XPD_ReturnType CAN_prvFrameTransmit(CAN_HandleType * pxCAN, CAN_FrameType * pxFrame)
{
    /* save the mailbox number to the Index field */
    XPD_ReturnType eResult = CAN_prvGetEmptyMailbox(pxCAN, &pxFrame->Index);

    if (eResult == XPD_OK)
    {
        uint8_t ucOffset;

        /* set up the Id */
        if ((pxFrame->Id.Type & CAN_IDTYPE_EXT_DATA) == CAN_IDTYPE_STD_DATA)
        {
            ucOffset = CAN_TI0R_STID_Pos;
        }
        else
        {
            ucOffset = CAN_TI0R_EXID_Pos;
        }
        pxCAN->Inst->sTxMailBox[pxFrame->Index].TIR.w =
                ((pxFrame->Id.Value << ucOffset) | (uint32_t)pxFrame->Id.Type);

        /* set up the DLC */
        pxCAN->Inst->sTxMailBox[pxFrame->Index].TDTR.w =
                ((uint32_t)pxFrame->DLC << CAN_TDT0R_DLC_Pos) & CAN_TDT0R_DLC_Msk;

        /* set up the data field */
        pxCAN->Inst->sTxMailBox[pxFrame->Index].TDLR.w = pxFrame->Data.Word[0];
        pxCAN->Inst->sTxMailBox[pxFrame->Index].TDHR.w = pxFrame->Data.Word[1];

        /* request transmission */
        CAN_REG_BIT(pxCAN,sTxMailBox[pxFrame->Index].TIR,TXRQ) = 1;
    }

    return eResult;
}

/**
 * @brief Gets the data from the receive FIFO to the receive frame pointer of the handle
 *        and flushes the frame from the FIFO.
 * @param pxCAN: pointer to the CAN handle structure
 * @param ucFIFONumber: the selected receive FIFO [0 .. 1]
 */
static void CAN_prvFrameReceive(CAN_HandleType * pxCAN, uint8_t ucFIFONumber)
{
    uint32_t ulRIR  = pxCAN->Inst->sFIFOMailBox[ucFIFONumber].RIR.w;
    uint32_t ulRDTR = pxCAN->Inst->sFIFOMailBox[ucFIFONumber].RDTR.w;

    /* Get the Id */
    pxCAN->RxFrame[ucFIFONumber]->Id.Type = ulRIR & CAN_IDTYPE_EXT_RTR;

    if ((pxCAN->RxFrame[ucFIFONumber]->Id.Type & CAN_IDTYPE_EXT_DATA) == CAN_IDTYPE_STD_DATA)
    {
        pxCAN->RxFrame[ucFIFONumber]->Id.Value = ulRIR >> CAN_RI0R_STID_Pos;
    }
    else
    {
        pxCAN->RxFrame[ucFIFONumber]->Id.Value = ulRIR >> CAN_RI0R_EXID_Pos;
    }

    /* Get the DLC */
    pxCAN->RxFrame[ucFIFONumber]->DLC = ulRDTR & CAN_RDT0R_DLC;
    /* Get the FMI */
    pxCAN->RxFrame[ucFIFONumber]->Index = (ulRDTR & CAN_RDT0R_FMI) >> CAN_RDT0R_FMI_Pos;

    /* Get the data field */
    pxCAN->RxFrame[ucFIFONumber]->Data.Word[0] =
            pxCAN->Inst->sFIFOMailBox[ucFIFONumber].RDLR.w;
    pxCAN->RxFrame[ucFIFONumber]->Data.Word[1] =
            pxCAN->Inst->sFIFOMailBox[ucFIFONumber].RDHR.w;

    /* Release the FIFO */
    CAN_RXFLAG_CLEAR(pxCAN, ucFIFONumber, RFOM);
}

/**
 * @brief Resets the receive filter bank configurations for the CAN peripheral.
 * @param pxCAN: pointer to the CAN handle structure
 */
__STATIC_INLINE void CAN_prvFilterReset(CAN_HandleType * pxCAN)
{
    uint32_t ucFBOffset = FILTERBANK_OFFSET(pxCAN);
    uint32_t ulMask     = (1 << FILTERBANK_COUNT(pxCAN)) - 1;

#ifdef CAN2
#ifdef CAN3
    if (pxCAN->Inst != CAN3)
#endif /* CAN3 */
    {
        RCC_vClockEnable(RCC_POS_CAN1);
    }
#endif /* CAN2 */

    CLEAR_BIT(CAN_MASTER(pxCAN)->FA1R, ulMask << ucFBOffset);
}

/** @} */

/** @defgroup CAN_Exported_Functions CAN Exported Functions
 * @{ */

/**
 * @brief Attempts to calculate a possible bit timing setup based on the desired bitrate.
 * @param ulBitrate: The target bitrate to achieve
 * @param pxConfig: The timing configuration to set
 * @return OK if successful, ERROR if bitrate is not supported
 */
XPD_ReturnType CAN_eBitrateConfig(uint32_t ulBitrate, CAN_TimingConfigType * pxTimingConfig)
{
    XPD_ReturnType eResult = XPD_ERROR;

    /* 1bit: 4TQ <= 1TQ sync + 2-16TQ BS1 + 1-8TQ BS2 <= 25TQ */
    uint32_t ulOvers = CAN_INPUT_CLOCK_RATE / ulBitrate;
    uint32_t ulTQs = 25;
    uint32_t ulPres;

    do
    {
        ulPres = ulOvers / ulTQs;

        /* Division without remainder? */
        if (0 == (ulOvers - (ulTQs * ulPres)))
        {
            ulTQs--;
            pxTimingConfig->Prescaler = ulPres;

            /* Sample point is set at 2/3 of the bit */
            pxTimingConfig->BS1 = (ulTQs * 2) / 3;
            pxTimingConfig->BS2 = ulTQs - pxTimingConfig->BS1;

            /* Simply set to valid value, not part of the bitrate */
            pxTimingConfig->SJW = 4;

            eResult = XPD_OK;
            break;
        }
        ulTQs--;
    }
    while (ulTQs >= 4);

    return eResult;
}

/** @defgroup CAN_Exported_Functions_State CAN State Management Functions
 *  @brief    CAN initialization, state and error management
 *  @details  These functions provide API for CAN controller state management
 *            and reading the error status.
 * @{
 */

/**
 * @brief Initializes the CAN peripheral using the setup configuration.
 * @param pxCAN: pointer to the CAN handle structure
 * @param pxConfig: pointer to CAN setup configuration
 * @return ERROR if input is incorrect, TIMEOUT if initialization failed, OK if success
 */
XPD_ReturnType CAN_eInit(CAN_HandleType * pxCAN, const CAN_InitType * pxConfig)
{
    XPD_ReturnType eResult = XPD_OK;
    uint32_t ulTimeout = INAK_TIMEOUT;

    /* enable peripheral clock */
    RCC_vClockEnable(pxCAN->CtrlPos);

    /* reset operation state */
    pxCAN->State = 0;

    /* Dependencies initialization */
    XPD_SAFE_CALLBACK(pxCAN->Callbacks.DepInit, pxCAN);

    /* exit from sleep mode */
    CAN_REG_BIT(pxCAN, MCR, SLEEP) = 0;

    /* request initialization */
    CAN_REG_BIT(pxCAN, MCR, INRQ) = 1;

    eResult = XPD_eWaitForMatch(&pxCAN->Inst->MSR.w,
            CAN_MSR_INAK, CAN_MSR_INAK, &ulTimeout);

    if(eResult == XPD_OK)
    {
        /* set the features (except for mode) */
        MODIFY_REG(pxCAN->Inst->MCR.w, 0xFC, pxConfig->wSettings);

        /* set the bit timing register (with test mode) */
        pxCAN->Inst->BTR.w = (uint32_t)pxConfig->wSettings << 30;
        pxCAN->Inst->BTR.b.SJW = (uint32_t)pxConfig->Timing.SJW - 1;
        pxCAN->Inst->BTR.b.TS1 = (uint32_t)pxConfig->Timing.BS1 - 1;
        pxCAN->Inst->BTR.b.TS2 = (uint32_t)pxConfig->Timing.BS2 - 1;
        pxCAN->Inst->BTR.b.BRP = (uint32_t)pxConfig->Timing.Prescaler - 1;

        /* request leave initialization */
        CAN_REG_BIT(pxCAN, MCR, INRQ) = 0;

        eResult = XPD_eWaitForMatch(&pxCAN->Inst->MSR.w,
                CAN_MSR_INAK, 0, &ulTimeout);
    }

    return eResult;
}

/**
 * @brief Restores the CAN peripheral to its default inactive state.
 * @param pxCAN: pointer to the CAN handle structure
 */
void CAN_vDeinit(CAN_HandleType * pxCAN)
{
    /* Disable all interrupt requests */
    pxCAN->Inst->IER.w = 0;

    /* Disable filters */
    CAN_prvFilterReset(pxCAN);

    /* Deinitialize peripheral dependencies */
    XPD_SAFE_CALLBACK(pxCAN->Callbacks.DepDeinit, pxCAN);

    /* Disable peripheral clock */
#ifdef CAN2
    /* Master clock is only off when no filters are used */
    if ((pxCAN->Inst != CAN1) || !CAN1_SLAVE_FILTERS_ACTIVE())
#endif
    {
        RCC_vClockDisable(pxCAN->CtrlPos);
    }
}

/**
 * @brief Requests sleep mode for CAN controller.
 * @param pxCAN: pointer to the CAN handle structure
 * @return TIMEOUT if timed out, OK if sleep mode is entered
 */
XPD_ReturnType CAN_eSleep(CAN_HandleType * pxCAN)
{
    uint32_t ulTimeout = SLAK_TIMEOUT;
    CAN_REG_BIT(pxCAN, MCR, INRQ) = 0;
    CAN_REG_BIT(pxCAN, MCR, SLEEP) = 1;

    return XPD_eWaitForMatch(&pxCAN->Inst->MSR.w,
            CAN_MSR_SLAK, CAN_MSR_SLAK, &ulTimeout);
}

/**
 * @brief Requests wake up for CAN controller.
 * @param pxCAN: pointer to the CAN handle structure
 * @return TIMEOUT if timed out, OK if sleep mode is left
 */
XPD_ReturnType CAN_eWakeUp(CAN_HandleType * pxCAN)
{
    uint32_t ulTimeout = SLAK_TIMEOUT;
    CAN_REG_BIT(pxCAN, MCR, SLEEP) = 0;

    return XPD_eWaitForMatch(&pxCAN->Inst->MSR.w,
            CAN_MSR_SLAK, 0, &ulTimeout);
}

/**
 * @brief Gets the error state of the CAN peripheral and clears its last error code.
 * @param pxCAN: pointer to the CAN handle structure
 * @return Current CAN error state
 */
CAN_ErrorType CAN_eGetError(CAN_HandleType * pxCAN)
{
    CAN_ErrorType eErrors = (pxCAN->Inst->ESR.w) &
            (CAN_ESR_LEC | CAN_ESR_BOFF | CAN_ESR_EPVF | CAN_ESR_EWGF);

    /* clear last error code */
    pxCAN->Inst->ESR.b.LEC = 0;

    return eErrors;
}

/**
 * @brief CAN state change and error interrupt handler that provides handle callbacks.
 * @param pxCAN: pointer to the CAN handle structure
 */
void CAN_vIRQHandlerSCE(CAN_HandleType * pxCAN)
{
    /* check if errors are configured for interrupt and present */
    if (    ((pxCAN->Inst->IER.w & (CAN_IER_BOFIE | CAN_IER_EPVIE | CAN_IER_EWGIE | CAN_IER_LECIE)) != 0)
         && ((pxCAN->Inst->ESR.w & (CAN_ESR_LEC | CAN_ESR_BOFF | CAN_ESR_EPVF | CAN_ESR_EWGF)) != 0))
    {
        /* Clear error interrupt flag */
        CAN_FLAG_CLEAR(pxCAN, ERRI);

        /* call error callback function if interrupt is not by state change */
        XPD_SAFE_CALLBACK(pxCAN->Callbacks.Error, pxCAN);
    }
}

/** @} */

/** @defgroup CAN_Exported_Functions_Transmit CAN Transmit Control Functions
 *  @brief    CAN frame transmission control
 *  @details  These functions provide API for CAN frame transmission control.
 *            The transmission scheduling is protected by XPD critical section.
 *            The mailbox number is returned in the input frame's Index field.
 * @{
 */

/**
 * @brief Waits for an empty transmit mailbox and posts the frame in it.
 * @param pxCAN: pointer to the CAN handle structure
 * @param pxFrame: pointer to the frame to transmit
 * @param ulTimeout: available time for getting a mailbox
 * @return TIMEOUT if timed out, OK if frame is posted in a mailbox
 */
XPD_ReturnType CAN_ePost(
        CAN_HandleType *    pxCAN,
        CAN_FrameType *     pxFrame,
        uint32_t            ulTimeout)
{
    XPD_ReturnType eResult = XPD_eWaitForDiff(&pxCAN->Inst->TSR.w,
            CAN_TSR_TME, 0, &ulTimeout);

    if (eResult == XPD_OK)
    {
        /* Empty mailbox available, post frame */
        (void)CAN_prvFrameTransmit(pxCAN, pxFrame);
    }

    return eResult;
}

/**
 * @brief Transmits a frame and waits for completion until times out.
 * @param pxCAN: pointer to the CAN handle structure
 * @param pxFrame: pointer to the frame to transmit
 * @param ulTimeout: available time for getting a mailbox and successfully sending the frame
 * @return BUSY if no empty mailbox was available, TIMEOUT if timed out, OK if frame is sent
 */
XPD_ReturnType CAN_eSend(
        CAN_HandleType *    pxCAN,
        CAN_FrameType *     pxFrame,
        uint32_t            ulTimeout)
{
    XPD_ReturnType eResult = XPD_eWaitForDiff(&pxCAN->Inst->TSR.w,
            CAN_TSR_TME, 0, &ulTimeout);

    /* wait if transmit mailbox was found */
    if (eResult == XPD_OK)
    {
        uint32_t ulTXOK;

        (void)CAN_prvFrameTransmit(pxCAN, pxFrame);

        /* convert to TXOK flag for mailbox */
        ulTXOK = CAN_TSR_TXOK0 << (pxFrame->Index * 8);

        /* wait for TXOK with the remaining time */
        eResult = XPD_eWaitForMatch(&pxCAN->Inst->TSR.w, ulTXOK, ulTXOK, &ulTimeout);

        /* Abort frame if transmission has timed out */
        if (eResult != XPD_OK)
        {
            CAN_TXFLAG_CLEAR(pxCAN, pxFrame->Index, ABRQ);
        }
    }

    return eResult;
}

/**
 * @brief Sets up a frame transmission and produces completion callback using the interrupt stack.
 * @param pxCAN: pointer to the CAN handle structure
 * @param pxFrame: pointer to the frame to transmit
 * @return BUSY if no empty mailbox was available, OK if frame is set for transmission
 */
XPD_ReturnType CAN_eSend_IT(
        CAN_HandleType *    pxCAN,
        CAN_FrameType *     pxFrame)
{
    XPD_ReturnType eResult;

    eResult = CAN_prvFrameTransmit(pxCAN, pxFrame);

    if (eResult == XPD_OK)
    {
        SET_BIT(pxCAN->Inst->IER.w, CAN_ERROR_INTERRUPTS | CAN_TRANSMIT_INTERRUPTS);

        /* state bit is set for the transmit mailbox */
        SET_BIT(pxCAN->State, 1 << pxFrame->Index);
    }

    return eResult;
}

/**
 * @brief CAN transmit interrupt handler that provides handle callbacks.
 * @param pxCAN: pointer to the CAN handle structure
 */
void CAN_vIRQHandlerTX(CAN_HandleType * pxCAN)
{
    /* check end of transmission */
    if (CAN_REG_BIT(pxCAN,IER,TMEIE) && ((pxCAN->State & CAN_STATE_TRANSMIT) != 0))
    {
        uint32_t ulTxMB;

        /* check all mailboxes for successful interrupt requests */
        for (ulTxMB = 0; ulTxMB < 3; ulTxMB++)
        {
            uint8_t ucMbState = 1 << ulTxMB;
            if (((pxCAN->State & ucMbState) != 0) && CAN_TXFLAG_STATUS(pxCAN, ulTxMB, TXOK))
            {
                CLEAR_BIT(pxCAN->State, ucMbState);

                /* transmission complete callback */
                XPD_SAFE_CALLBACK(pxCAN->Callbacks.Transmit, pxCAN);
            }
        }

        /* if no more transmission requests are pending */
        if ((pxCAN->State & CAN_STATE_TRANSMIT) == 0)
        {
            /* disable transmit interrupt */
            uint32_t ulIEs = CAN_TRANSMIT_INTERRUPTS;

#ifdef __XPD_CAN_ERROR_DETECT
            if ((pxCAN->State & CAN_STATE_RECEIVE) == 0)
            {
                ulIEs |= CAN_ERROR_INTERRUPTS;
            }
#endif
            CLEAR_BIT(pxCAN->Inst->IER.w, ulIEs);
        }
    }
}

/** @} */

/** @defgroup CAN_Exported_Functions_Receive CAN Receive Control Functions
 *  @brief    CAN frame reception control
 *  @details  These functions provide API for CAN frame reception control.
 *            The received data is stored in the provided frame pointers.
 * @{
 */

/**
 * @brief Waits for a received frame and gets it from the FIFO.
 * @param pxCAN: pointer to the CAN handle structure
 * @param pxFrame: pointer to the frame to put the received frame data to
 * @param ucFIFONumber: the selected receive FIFO [0 .. 1]
 * @param ulTimeout: available time for frame reception
 * @return BUSY if FIFO is already in use, TIMEOUT if timed out, OK if frame is received
 */
XPD_ReturnType CAN_eReceive(
        CAN_HandleType *    pxCAN,
        CAN_FrameType *     pxFrame,
        uint8_t             ucFIFONumber,
        uint32_t            ulTimeout)
{
    XPD_ReturnType eResult = XPD_BUSY;
    uint8_t ucRecState = CAN_STATE_RECEIVE0 << ucFIFONumber;

    /* check if FIFO is not in use */
    if ((pxCAN->State & ucRecState) == 0)
    {
        SET_BIT(pxCAN->State, ucRecState);

        /* save receive data target */
        pxCAN->RxFrame[ucFIFONumber] = pxFrame;

        /* wait until at least one frame is in FIFO */
        eResult = XPD_eWaitForDiff(&pxCAN->Inst->RFR[ucFIFONumber].w, CAN_RF0R_FMP0, 0, &ulTimeout);

        if (eResult == XPD_OK)
        {
            /* frame data is extracted */
            CAN_prvFrameReceive(pxCAN, ucFIFONumber);
        }
    }

    return eResult;
}

/**
 * @brief Requests a frame reception using the interrupt stack.
 * @param pxCAN: pointer to the CAN handle structure
 * @param pxFrame: pointer to the frame to put the received frame data to
 * @param ucFIFONumber: the selected receive FIFO [0 .. 1]
 * @return BUSY if the FIFO is already in use, OK otherwise
 */
XPD_ReturnType CAN_eReceive_IT(
        CAN_HandleType *    pxCAN,
        CAN_FrameType *     pxFrame,
        uint8_t             ucFIFONumber)
{
    XPD_ReturnType eResult = XPD_BUSY;
    uint8_t ucRecState = CAN_STATE_RECEIVE0 << ucFIFONumber;

    /* check if FIFO is not in use */
    if ((pxCAN->State & ucRecState) == 0)
    {
        SET_BIT(pxCAN->State, ucRecState);

        /* save receive data target */
        pxCAN->RxFrame[ucFIFONumber] = pxFrame;

        SET_BIT(pxCAN->Inst->IER.w, CAN_ERROR_INTERRUPTS
            | ((ucFIFONumber == 0) ? CAN_RECEIVE0_INTERRUPTS : CAN_RECEIVE1_INTERRUPTS));

        eResult = XPD_OK;
    }
    return eResult;
}

/**
 * @brief CAN receive FIFO 0 interrupt handler that provides handle callbacks.
 * @param pxCAN: pointer to the CAN handle structure
 */
void CAN_vIRQHandlerRX0(CAN_HandleType * pxCAN)
{
    /* check reception completion */
    if (CAN_REG_BIT(pxCAN,IER,FMP0IE) && (CAN_REG_BIT(pxCAN,RFR[0],FMP) != 0))
    {
        /* get the FIFO contents to the requested frame structure */
        CAN_prvFrameReceive(pxCAN, 0);

        /* only clear interrupt requests if they were enabled through XPD API */
        if ((pxCAN->State & CAN_STATE_RECEIVE0) != 0)
        {
            uint32_t ulIEs = CAN_RECEIVE0_INTERRUPTS;

#ifdef __XPD_CAN_ERROR_DETECT
            if ((pxCAN->State & (CAN_STATE_TRANSMIT | CAN_STATE_RECEIVE1)) == 0)
            {
                ulIEs |= CAN_ERROR_INTERRUPTS;
            }
#endif
            CLEAR_BIT(pxCAN->State, CAN_STATE_RECEIVE0);

            CLEAR_BIT(pxCAN->Inst->IER.w, ulIEs);
        }

        /* receive complete callback */
        XPD_SAFE_CALLBACK(pxCAN->Callbacks.Receive[0], pxCAN);
    }
}

/**
 * @brief CAN receive FIFO 1 interrupt handler that provides handle callbacks.
 * @param pxCAN: pointer to the CAN handle structure
 */
void CAN_vIRQHandlerRX1(CAN_HandleType * pxCAN)
{
    /* check reception completion */
    if (CAN_REG_BIT(pxCAN,IER,FMP1IE) && (CAN_REG_BIT(pxCAN,RFR[1],FMP) != 0))
    {
        /* get the FIFO contents to the requested frame structure */
        CAN_prvFrameReceive(pxCAN, 1);

        /* only clear interrupt requests if they were enabled through XPD API */
        if ((pxCAN->State & CAN_STATE_RECEIVE1) != 0)
        {
            uint32_t ulIEs = CAN_RECEIVE1_INTERRUPTS;

#ifdef __XPD_CAN_ERROR_DETECT
            if ((pxCAN->State & (CAN_STATE_TRANSMIT | CAN_STATE_RECEIVE0)) == 0)
            {
                ulIEs |= CAN_ERROR_INTERRUPTS;
            }
#endif
            CLEAR_BIT(pxCAN->State, CAN_STATE_RECEIVE1);

            CLEAR_BIT(pxCAN->Inst->IER.w, ulIEs);
        }

        /* receive complete callback */
        XPD_SAFE_CALLBACK(pxCAN->Callbacks.Receive[1], pxCAN);
    }
}

/** @} */

/** @defgroup CAN_Exported_Functions_Filter CAN Filter Management Functions
 *  @brief    CAN frame reception filters management
 *  @details  These functions provide API for frame reception filtering.
 *            The core features are:
 *            @n - Filter bank utilization maximized
 *            @n - Filter Match Index feature support: received frames can be mapped to filters
 * @{
 */

/**
 * @brief Configures the receive filters for the peripheral and provides each filter
 *        its own Filter Match Index (FMI).
 * @param pxCAN: pointer to the CAN handle structure
 * @param axFilters: filter configuration list (array)
 * @param aucMatchIndexes: array to fill with the FMI of the filters
 * @param ucFilterCount: the number of input filters to configure
 * @return ERROR if filters cannot fit in the filter bank, OK if filters are added
 */
XPD_ReturnType CAN_eFilterConfig(
        CAN_HandleType *        pxCAN,
        const CAN_FilterType    axFilters[],
        uint8_t                 aucMatchIndexes[],
        uint8_t                 ucFilterCount)
{
    XPD_ReturnType eResult = XPD_OK;
    uint8_t ucFilterIndex, ucBase, ucFBDemand = 0, ucCurrentFMI = 0;
    CAN_TypeDef * CANx = CAN_MASTER(pxCAN);
#ifdef CAN_BB
    CAN_BitBand_TypeDef * CANx_BB = CAN_BB(CANx);
#endif

    uint8_t ucFBOffset = FILTERBANK_OFFSET(pxCAN);
    uint8_t ucFBCount  = FILTERBANK_COUNT(pxCAN);
    uint32_t ulMask    = (1 << ucFBCount) - 1;

#ifdef CAN2
#ifdef CAN3
    if (pxCAN->Inst != CAN3)
#endif /* CAN3 */
    {
        RCC_vClockEnable(RCC_POS_CAN1);
    }
#endif /* CAN2 */

    /* Enter filter initialization */
    CANx_REG_BIT(FMR,FINIT) = 1;

    /* Deactivate all filter banks assigned to this peripheral */
    CLEAR_BIT(CANx->FA1R, ulMask << ucFBOffset);

    /* Initially set invalid value to FMI, to indicate missing configuration */
    for (ucFilterIndex = 0; ucFilterIndex < ucFilterCount; ucFilterIndex++)
    {
        aucMatchIndexes[ucFilterIndex] = FMI_INVALID;
    }

    for (ucBase = 0; ucBase < ucFilterCount; ucBase++)
    {
        /* If the current filter is not configured, it has a yet unprocessed type */
        if (aucMatchIndexes[ucBase] == FMI_INVALID)
        {
            uint8_t ucFBIndex; /* Ignore "may be used uninitialized" warning */
            uint8_t ucFBPos = 255;
            uint8_t ucFilterSize;
            union {
                struct {
                uint8_t Mode : 1; /* Set for match, clear for mask mode */
                uint8_t Size : 1; /* Set when the pattern is extended */
                uint8_t FIFO : 1;
                uint8_t : 5;
                };
                uint8_t w;
            }xSelectedType = { .w = 0 }, xCurrentType;
            union {
                union {
                    struct {
                    uint16_t : 3;
                    uint16_t IDE : 1;
                    uint16_t RTR : 1;
                    uint16_t StdId : 11;
                    };
                    uint16_t w;
                }u16[4];
                union {
                    struct {
                    uint32_t : 1;
                    uint32_t RTR : 1;
                    uint32_t IDE : 1;
                    uint32_t ExtId : 29;
                    };
                    uint32_t w;
                }u32[2];
            }xFilterBank;

            xSelectedType.Mode = axFilters[ucBase].Mode;
            xSelectedType.Size = axFilters[ucBase].Pattern.Type >> (CAN_RI0R_IDE_Pos - FILTER_SIZE_FLAG_Pos);
            xSelectedType.FIFO = axFilters[ucBase].FIFO;
            ucFilterSize = can_aucFilterTypeSpace[xSelectedType.w & (FILTER_MODE_FLAG | FILTER_SIZE_FLAG)];
            xCurrentType.w = xSelectedType.w;
            ucFilterIndex = ucBase;

            do
            {
                /* If the xCurrentType of the currently indexed filter matches the base */
                if (xCurrentType.w == xSelectedType.w)
                {
                    /* If the current filter bank is full, set up another one */
                    if (ucFBPos >= ucFilterSize)
                    {
                        ucFBIndex = ucFBDemand + ucFBOffset;

                        /* Check against overrun of used filter banks */
                        if ((ucFBDemand + 1) > ucFBCount)
                        {
                            ucBase = ucFilterCount;
                            eResult = XPD_ERROR;
                            break;
                        }

                        /* Configure new filter bank type */
#ifdef CAN_BB
                        /* Setting filter FIFO assignment */
                        CANx_BB->FFA1R[ucFBIndex] = axFilters[ucFilterIndex].FIFO;

                        /* Setting filter mode */
                        CANx_BB->FM1R[ucFBIndex] = xCurrentType.Mode;

                        /* Setting filter scale */
                        CANx_BB->FS1R[ucFBIndex] = xCurrentType.Size;
#else
                        uint32_t ulFBIMask = 1 << ucFBIndex;

                        /* Setting filter FIFO assignment */
                        if (axFilters[ucFilterIndex].FIFO == 0)
                        {   CLEAR_BIT(CANx->FFA1R, ulFBIMask); }
                        else
                        {   SET_BIT(CANx->FFA1R, ulFBIMask); }

                        /* Setting filter mode */
                        if (xCurrentType.Mode == 0)
                        {   CLEAR_BIT(CANx->FM1R, ulFBIMask); }
                        else
                        {   SET_BIT(CANx->FM1R, ulFBIMask); }

                        /* Setting filter scale */
                        if (xCurrentType.Size == 0)
                        {   CLEAR_BIT(CANx->FS1R, ulFBIMask); }
                        else
                        {   SET_BIT(CANx->FS1R, ulFBIMask); }
#endif /* CAN_BB */

                        ucFBPos = 0;
                        ucFBDemand++;
                    }

                    /* Set the identifier pattern depending on the filter scale */
                    if (xCurrentType.Size == 0)
                    {
                        xFilterBank.u16[ucFBPos].w = axFilters[ucFilterIndex].Pattern.Value << 5;

                        /* IDE and RTR bit order is reverse in this layout */
                        xFilterBank.u16[ucFBPos].IDE =
                                axFilters[ucFilterIndex].Pattern.Type >> CAN_RI0R_IDE_Pos;
                        xFilterBank.u16[ucFBPos].RTR =
                                axFilters[ucFilterIndex].Pattern.Type >> CAN_RI0R_RTR_Pos;

                        /* Set the masking bits */
                        if (xCurrentType.Mode == 0)
                        {
                            xFilterBank.u16[ucFBPos + 1].w = (axFilters[ucFilterIndex].Mask << 5)
                                                           | (axFilters[ucFilterIndex].Mode << 2);
                        }
                    }
                    else
                    {
                        xFilterBank.u32[ucFBPos].w = (axFilters[ucFilterIndex].Pattern.Value << CAN_RI0R_EXID_Pos)
                                                   | (axFilters[ucFilterIndex].Pattern.Type);

                        /* Set the masking bits */
                        if (xCurrentType.Mode == 0)
                        {
                            /* Filter bank size fixes the mask field location */
                            xFilterBank.u16[1].w = (axFilters[ucFilterIndex].Mask << CAN_RI0R_EXID_Pos)
                                                  | axFilters[ucFilterIndex].Mode;
                        }
                    }

                    /* Set the current filter's FMI */
                    aucMatchIndexes[ucFilterIndex] = ucCurrentFMI + ucFBPos;

                    /* If the last element of the bank */
                    if ((ucFBPos + 1) >= ucFilterSize)
                    {
                        ucCurrentFMI += ucFilterSize;

                        /* Set the configured bank in the peripheral */
                        CANx->sFilterRegister[ucFBIndex].FR1 = xFilterBank.u32[0].w;
                        CANx->sFilterRegister[ucFBIndex].FR2 = xFilterBank.u32[1].w;
                    }
                    /* Unused filters after the current one are set to
                     * the copies of the current to avoid receiving unwanted frames */
                    else if (ucFilterSize == 2)
                    {
                        xFilterBank.u32[1].w = xFilterBank.u32[0].w;
                    }
                    else if (ucFBPos == 0)
                    {
                        xFilterBank.u16[1].w = xFilterBank.u16[0].w;
                        xFilterBank.u32[1].w = xFilterBank.u32[0].w;
                    }
                    else {}

                    ucFBPos++;
                }

                /* Advance to the next filter */
                ucFilterIndex++;

                xCurrentType.Mode = axFilters[ucBase].Mode;
                xCurrentType.Size = axFilters[ucBase].Pattern.Type >> (CAN_RI0R_IDE_Pos - FILTER_SIZE_FLAG_Pos);
                xCurrentType.FIFO = axFilters[ucBase].FIFO;
            }
            while (ucFilterIndex < ucFilterCount);

            /* If the last bank was not filled completely */
            if (ucFBPos < ucFilterSize)
            {
                ucCurrentFMI += ucFilterSize;

                /* Set the configured bank in the peripheral */
                CANx->sFilterRegister[ucFBIndex].FR1 = xFilterBank.u32[0].w;
                CANx->sFilterRegister[ucFBIndex].FR2 = xFilterBank.u32[1].w;
            }
        }
    }

    /* Activate all configured filter banks */
    ulMask = (1 << ucFBDemand) - 1;
    SET_BIT(CANx->FA1R, ulMask << ucFBOffset);

    /* Exit filter initialization */
    CANx_REG_BIT(FMR,FINIT) = 0;

    return eResult;
}

/**
 * @brief Sets the filter bank size for the CAN peripheral.
 * @note  This operation resets the filter configuration for the slave CAN controller.
 * @param pxCAN: pointer to the CAN handle structure
 * @param ucNewSize: the new filter bank size for the CAN peripheral [0 .. 28]
 * @return ERROR if size is invalid, or if size change would disable master CAN filters, OK if successful
 */
XPD_ReturnType CAN_eFilterBankConfig(CAN_HandleType * pxCAN, uint8_t ucNewSize)
{
    XPD_ReturnType eResult = XPD_ERROR;

#ifdef CAN3
    if (pxCAN->Inst == CAN3)
    { /* Simply return with error */ }
    else
#endif /* CAN3 */
#ifdef CAN2
    if (ucNewSize <= 28)
    {
        uint32_t ulMasterMask;

        RCC_vClockEnable(RCC_POS_CAN1);

        ulMasterMask = (1 << CAN1->FMR.b.CAN2SB) - 1;

        /* Slave CAN bank size is counted from the end */
        if (pxCAN->Inst != CAN1)
        {
            ucNewSize = 28 - ucNewSize;
        }

        /* Do not allow the master CAN to lose active filters */
        if (((CAN1->FA1R & ulMasterMask) & (0xFFFF << ucNewSize)) == 0)
        {
            CAN1->FMR.b.FINIT = 1;

            /* Deactivate slave CAN filter banks */
            CLEAR_BIT(CAN1->FA1R, ~ulMasterMask);

            /* select the start slave bank */
            CAN1->FMR.b.CAN2SB = ucNewSize;

            CAN1->FMR.b.FINIT = 0;

            eResult = XPD_OK;
        }
    }
#endif /* CAN2 */

    return eResult;
}

/** @} */

/** @} */

/** @} */

#endif /* defined(CAN) || defined(CAN1) */
