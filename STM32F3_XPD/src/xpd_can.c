/**
  ******************************************************************************
  * @file    xpd_can.c
  * @author  Benedek Kupper
  * @version V0.2
  * @date    2017-08-02
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
#include "xpd_can.h"
#include "xpd_utils.h"

#ifdef USE_XPD_CAN

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

#ifdef USE_XPD_CAN_ERROR_DETECT
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
#define CAN_MASTER(HANDLE)              ((HANDLE)->Inst)
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

/* Filter types */
#define STD_MASK    0
#define STD_MATCH   1
#define EXT_MASK    2
#define EXT_MATCH   3

static const uint8_t filterTypeSpace[4] = {2, 4, 1, 2};

/** @defgroup CAN_Private_Functions CAN Private Functions
 * @{ */

/**
 * @brief Gets an empty transmit mailbox.
 * @param hcan: pointer to the CAN handle structure
 * @param mailbox: Set to the lowest number empty mailbox
 * @return BUSY if all mailboxes are full, OK if empty mailbox found
 */
__STATIC_INLINE XPD_ReturnType can_getEmptyMailbox(CAN_HandleType * hcan, uint8_t * mailbox)
{
    XPD_ReturnType result = XPD_BUSY;
    uint32_t reg = hcan->Inst->TSR.w;

    /* Check if at least one mailbox is empty */
    if ((reg & CAN_TSR_TME) != 0)
    {
        *mailbox = (reg & CAN_TSR_CODE) >> CAN_TSR_CODE_Pos;
        result = XPD_OK;
    }
    return result;
}

/**
 * @brief Puts the frame data in an empty transmit mailbox, and requests transmission.
 * @param hcan: pointer to the CAN handle structure
 * @param Frame: pointer to the frame to transmit
 * @return BUSY if all mailboxes are in use, OK if transmission scheduling was successful
 */
static XPD_ReturnType can_frameTransmit(CAN_HandleType * hcan, CAN_FrameType * Frame)
{
    uint8_t transmitmailbox;
    XPD_ReturnType result;

    result = can_getEmptyMailbox(hcan, &transmitmailbox);

    if (result == XPD_OK)
    {
        uint32_t temp = 3;

        /* set up the Id */
        if ((Frame->Id.Type & CAN_IDTYPE_EXT_DATA) == CAN_IDTYPE_STD_DATA)
        {
            temp = 21;
        }
        hcan->Inst->sTxMailBox[transmitmailbox].TIR.w = ((Frame->Id.Value << temp) | (uint32_t)Frame->Id.Type);

        /* set up the DLC */
        hcan->Inst->sTxMailBox[transmitmailbox].TDTR.b.DLC = (uint32_t) Frame->DLC;

        /* set up the data field */
        hcan->Inst->sTxMailBox[transmitmailbox].TDLR.w = Frame->Data.Word[0];
        hcan->Inst->sTxMailBox[transmitmailbox].TDHR.w = Frame->Data.Word[1];

        /* request transmission */
        CAN_REG_BIT(hcan,sTxMailBox[transmitmailbox].TIR,TXRQ) = 1;

        /* save the mailbox number to the Index field */
        Frame->Index = transmitmailbox;
    }

    return result;
}

/**
 * @brief Gets the data from the receive FIFO to the receive frame pointer of the handle
 *        and flushes the frame from the FIFO.
 * @param hcan: pointer to the CAN handle structure
 * @param FIFONumber: the selected receive FIFO [0 .. 1]
 */
static void can_frameReceive(CAN_HandleType * hcan, uint8_t FIFONumber)
{
    if (hcan->RxFrame[FIFONumber] != NULL)
    {
        /* copy the FIFO data to stack */
        CAN_FIFOMailBox_TypeDef rxFIFO = hcan->Inst->sFIFOMailBox[FIFONumber];

        /* Get the Id */
        hcan->RxFrame[FIFONumber]->Id.Type = rxFIFO.RIR.w & CAN_IDTYPE_EXT_RTR;

        if ((hcan->RxFrame[FIFONumber]->Id.Type & CAN_IDTYPE_EXT_DATA) == CAN_IDTYPE_STD_DATA)
        {
            hcan->RxFrame[FIFONumber]->Id.Value = rxFIFO.RIR.w >> 21;
        }
        else
        {
            hcan->RxFrame[FIFONumber]->Id.Value = rxFIFO.RIR.w >> 3;
        }

        /* Get the DLC */
        hcan->RxFrame[FIFONumber]->DLC = rxFIFO.RDTR.b.DLC;
        /* Get the FMI */
        hcan->RxFrame[FIFONumber]->Index = rxFIFO.RDTR.b.FMI;
        /* Get the data field */
        hcan->RxFrame[FIFONumber]->Data.Word[0] = rxFIFO.RDLR.w;
        hcan->RxFrame[FIFONumber]->Data.Word[1] = rxFIFO.RDHR.w;

        /* Release the FIFO */
        XPD_CAN_ClearRxFlag(hcan, FIFONumber, RFOM);
    }
}

/**
 * @brief Resets the receive filter bank configurations for the CAN peripheral.
 * @param hcan: pointer to the CAN handle structure
 */
__STATIC_INLINE void can_filterReset(CAN_HandleType * hcan)
{
    uint32_t fbOffset = FILTERBANK_OFFSET(hcan);
    uint32_t mask     = (1 << FILTERBANK_COUNT(hcan)) - 1;

#ifdef CAN2
#ifdef CAN3
    if (hcan->Inst != CAN3)
#endif /* CAN3 */
    {
        XPD_CAN1_ClockCtrl(ENABLE);
    }
#endif /* CAN2 */

    CLEAR_BIT(CAN_MASTER(hcan)->FA1R, mask << fbOffset);
}

/** @} */

/** @defgroup CAN_Exported_Functions CAN Exported Functions
 * @{ */

/** @defgroup CAN_Exported_Functions_State CAN State Management Functions
 *  @brief    CAN initialization, state and error management
 *  @details  These functions provide API for CAN controller state management
 *            and reading the error status.
 * @{
 */

/**
 * @brief Initializes the CAN peripheral using the setup configuration.
 * @param hcan: pointer to the CAN handle structure
 * @param Config: pointer to CAN setup configuration
 * @return ERROR if input is incorrect, TIMEOUT if initialization failed, OK if success
 */
XPD_ReturnType XPD_CAN_Init(CAN_HandleType * hcan, const CAN_InitType * Config)
{
    XPD_ReturnType result = XPD_OK;
    uint32_t timeout = INAK_TIMEOUT;

    /* enable peripheral clock */
    hcan->ClockCtrl(ENABLE);

    /* Dependencies initialization */
    XPD_SAFE_CALLBACK(hcan->Callbacks.DepInit, hcan);

    /* exit from sleep mode */
    CAN_REG_BIT(hcan, MCR, SLEEP) = 0;

    /* request initialization */
    CAN_REG_BIT(hcan, MCR, INRQ) = 1;

    result = XPD_WaitForMatch(&hcan->Inst->MSR.w, CAN_MSR_INAK, CAN_MSR_INAK, &timeout);
    if(result != XPD_OK)
    {
        return result;
    }

    /* set the features (except for mode) */
    MODIFY_REG(hcan->Inst->MCR.w, 0xFC, Config->wSettings);

    /* set the bit timing register (with test mode) */
    hcan->Inst->BTR.w = (uint32_t)Config->wSettings << 30;
    hcan->Inst->BTR.b.SJW = (uint32_t)Config->Timing.SJW - 1;
    hcan->Inst->BTR.b.TS1 = (uint32_t)Config->Timing.BS1 - 1;
    hcan->Inst->BTR.b.TS2 = (uint32_t)Config->Timing.BS2 - 1;
    hcan->Inst->BTR.b.BRP = (uint32_t)Config->Timing.Prescaler - 1;

    /* request leave initialization */
    CAN_REG_BIT(hcan, MCR, INRQ) = 0;

    result = XPD_WaitForMatch(&hcan->Inst->MSR.w, CAN_MSR_INAK, 0, &timeout);
    if(result != XPD_OK)
    {
        return result;
    }

    /* reset operation state */
    hcan->State = 0;

    /* reset filter banks */
    can_filterReset(hcan);

    return result;
}

/**
 * @brief Restores the CAN peripheral to its default inactive state.
 * @param hcan: pointer to the CAN handle structure
 * @return ERROR if input is incorrect, OK if success
 */
XPD_ReturnType XPD_CAN_Deinit(CAN_HandleType * hcan)
{
    /* Disable all interrupt requests */
    hcan->Inst->IER.w = 0;

    /* Disable filters */
    can_filterReset(hcan);

    /* Deinitialize peripheral dependencies */
    XPD_SAFE_CALLBACK(hcan->Callbacks.DepDeinit, hcan);

    /* Disable peripheral clock */
#ifdef CAN2
    /* Master clock is only off when no filters are used */
    if ((hcan->Inst != CAN1) || !CAN1_SLAVE_FILTERS_ACTIVE())
#endif
    {
        hcan->ClockCtrl(DISABLE);
    }

    return XPD_OK;
}

/**
 * @brief Requests sleep mode for CAN controller.
 * @param hcan: pointer to the CAN handle structure
 * @return TIMEOUT if timed out, OK if sleep mode is entered
 */
XPD_ReturnType XPD_CAN_Sleep(CAN_HandleType * hcan)
{
    uint32_t timeout = SLAK_TIMEOUT;
    CAN_REG_BIT(hcan, MCR, INRQ) = 0;
    CAN_REG_BIT(hcan, MCR, SLEEP) = 1;

    return XPD_WaitForMatch(&hcan->Inst->MSR.w, CAN_MSR_SLAK, CAN_MSR_SLAK, &timeout);
}

/**
 * @brief Requests wake up for CAN controller.
 * @param hcan: pointer to the CAN handle structure
 * @return TIMEOUT if timed out, OK if sleep mode is left
 */
XPD_ReturnType XPD_CAN_WakeUp(CAN_HandleType * hcan)
{
    uint32_t timeout = SLAK_TIMEOUT;
    CAN_REG_BIT(hcan, MCR, SLEEP) = 0;

    return XPD_WaitForMatch(&hcan->Inst->MSR.w, CAN_MSR_SLAK, 0, &timeout);
}

/**
 * @brief Gets the error state of the CAN peripheral and clears its last error code.
 * @param hcan: pointer to the CAN handle structure
 * @return Current CAN error state
 */
CAN_ErrorType XPD_CAN_GetError(CAN_HandleType * hcan)
{
    CAN_ErrorType errors = (hcan->Inst->ESR.w) & (CAN_ESR_LEC | CAN_ESR_BOFF | CAN_ESR_EPVF | CAN_ESR_EWGF);

    /* clear last error code */
    hcan->Inst->ESR.b.LEC = 0;

    return errors;
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
 * @brief Transmits a frame and waits for completion until times out.
 * @param hcan: pointer to the CAN handle structure
 * @param Frame: pointer to the frame to transmit
 * @param Timeout: available time for getting a mailbox and successfully sending the frame
 * @return BUSY if no empty mailbox was available, TIMEOUT if timed out, OK if frame is sent
 */
XPD_ReturnType XPD_CAN_Transmit(CAN_HandleType * hcan, CAN_FrameType * Frame, uint32_t Timeout)
{
    XPD_ReturnType result;

    /* no timeout, no wait for success */
    if (Timeout == 0)
    {
        /* get mailbox if available */
        result = can_frameTransmit(hcan, Frame);

        /* result is busy if there was no free mailbox, timeout if scheduling was successful */
        if (result == XPD_OK)
        {
            result = XPD_TIMEOUT;
        }
    }
    else
    {
        do
        {
            result = XPD_WaitForDiff(&hcan->Inst->TSR.w, CAN_TSR_TME, 0, &Timeout);

            /* if getting free mailbox times out, exit with busy */
            if (result != XPD_OK)
            {
                result = XPD_BUSY;
                break;
            }

            /* get mailbox if available */
            result = can_frameTransmit(hcan, Frame);

        /* until free mailbox was found */
        } while (result != XPD_OK);

        /* wait if transmit mailbox was found */
        if (result == XPD_OK)
        {
            /* convert to txok flag for mailbox */
            uint32_t txok = CAN_TSR_TXOK0 << (Frame->Index * 8);

            /* wait for txok with the remaining time */
            result = XPD_WaitForMatch(&hcan->Inst->TSR.w, txok, txok, &Timeout);
        }
    }

    return result;
}

/**
 * @brief Sets up a frame transmission and produces completion callback using the interrupt stack.
 * @param hcan: pointer to the CAN handle structure
 * @param Frame: pointer to the frame to transmit
 * @return BUSY if no empty mailbox was available, OK if frame is set for transmission
 */
XPD_ReturnType XPD_CAN_Transmit_IT(CAN_HandleType * hcan, CAN_FrameType * Frame)
{
    uint32_t transmitmailbox;
    XPD_ReturnType result;

    result = can_frameTransmit(hcan, Frame);

    if (result == XPD_OK)
    {
        SET_BIT(hcan->Inst->IER.w, CAN_ERROR_INTERRUPTS | CAN_TRANSMIT_INTERRUPTS);

        /* state bit is set for the transmit mailbox */
        SET_BIT(hcan->State, 1 << Frame->Index);
    }

    return result;
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
 * @param hcan: pointer to the CAN handle structure
 * @param Frame: pointer to the frame to put the received frame data to
 * @param FIFONumber: the selected receive FIFO [0 .. 1]
 * @param Timeout: available time for frame reception
 * @return BUSY if FIFO is already in use, TIMEOUT if timed out, OK if frame is received
 */
XPD_ReturnType XPD_CAN_Receive(CAN_HandleType * hcan, CAN_FrameType* Frame, uint8_t FIFONumber, uint32_t Timeout)
{
    XPD_ReturnType result = XPD_BUSY;
    uint8_t recState = CAN_STATE_RECEIVE0 << FIFONumber;

    /* check if FIFO is not in use */
    if ((hcan->State & recState) == 0)
    {
        SET_BIT(hcan->State, recState);

        /* save receive data target */
        hcan->RxFrame[FIFONumber] = Frame;

        /* wait until at least one frame is in FIFO */
        result = XPD_WaitForDiff(&hcan->Inst->RFR[FIFONumber].w, CAN_RF0R_FMP0, 0, &Timeout);

        if (result == XPD_OK)
        {
            /* frame data is extracted */
            can_frameReceive(hcan, FIFONumber);
        }
    }

    return result;
}

/**
 * @brief Requests a frame reception using the interrupt stack.
 * @param hcan: pointer to the CAN handle structure
 * @param Frame: pointer to the frame to put the received frame data to
 * @param FIFONumber: the selected receive FIFO [0 .. 1]
 * @return BUSY if the FIFO is already in use, OK otherwise
 */
XPD_ReturnType XPD_CAN_Receive_IT(CAN_HandleType * hcan, CAN_FrameType * Frame, uint8_t FIFONumber)
{
    XPD_ReturnType result = XPD_BUSY;
    uint8_t recState = CAN_STATE_RECEIVE0 << FIFONumber;

    /* check if FIFO is not in use */
    if ((hcan->State & recState) == 0)
    {
        SET_BIT(hcan->State, recState);

        /* save receive data target */
        hcan->RxFrame[FIFONumber] = Frame;

        SET_BIT(hcan->Inst->IER.w, CAN_ERROR_INTERRUPTS
            | ((FIFONumber == 0) ? CAN_RECEIVE0_INTERRUPTS : CAN_RECEIVE1_INTERRUPTS));

        result = XPD_OK;
    }
    return result;
}

/** @} */

/** @defgroup CAN_Exported_Functions_IRQ CAN Interrupt Handling Functions
 *  @brief    CAN interrupt handlers
 *  @details  These functions provide API for CAN interrupt handling.
 *            These functions shall be called from the interrupt request handlers
 *            of the CAN peripheral that is managed by the specified handle input.
 * @{
 */

/**
 * @brief CAN transmit interrupt handler that provides handle callbacks.
 * @param hcan: pointer to the CAN handle structure
 */
void XPD_CAN_TX_IRQHandler(CAN_HandleType * hcan)
{
    uint32_t temp, i;

    /* check end of transmission */
    if (CAN_REG_BIT(hcan,IER,TMEIE) && ((hcan->State & CAN_STATE_TRANSMIT) != 0))
    {
        /* check all mailboxes for successful interrupt requests */
        for (i = 0; i < 3; i++)
        {
            temp = 1 << i;
            if (((hcan->State & temp) != 0) && XPD_CAN_GetTxFlag(hcan, i, TXOK))
            {
                CLEAR_BIT(hcan->State, temp);

                /* transmission complete callback */
                XPD_SAFE_CALLBACK(hcan->Callbacks.Transmit, hcan);
            }
        }

        /* if no more transmission requests are pending */
        if ((hcan->State & CAN_STATE_TRANSMIT) == 0)
        {
            /* disable transmit interrupt */
            temp = CAN_TRANSMIT_INTERRUPTS;

#ifdef USE_XPD_CAN_ERROR_DETECT
            if ((hcan->State & CAN_STATE_RECEIVE) == 0)
            {
                temp |= CAN_ERROR_INTERRUPTS;
            }
#endif
            CLEAR_BIT(hcan->Inst->IER.w, temp);
        }
    }
}

/**
 * @brief CAN receive FIFO 0 interrupt handler that provides handle callbacks.
 * @param hcan: pointer to the CAN handle structure
 */
void XPD_CAN_RX0_IRQHandler(CAN_HandleType * hcan)
{
    uint32_t temp;

    /* check reception completion */
    if (CAN_REG_BIT(hcan,IER,FMP0IE) && (hcan->Inst->RFR[0].b.FMP != 0))
    {
        /* get the FIFO contents to the requested frame structure */
        can_frameReceive(hcan, 0);

        /* only clear interrupt requests if they were enabled through XPD API */
        if ((hcan->State & CAN_STATE_RECEIVE0) != 0)
        {
            temp = CAN_RECEIVE0_INTERRUPTS;

#ifdef USE_XPD_CAN_ERROR_DETECT
            if ((hcan->State & (CAN_STATE_TRANSMIT | CAN_STATE_RECEIVE1)) == 0)
            {
                temp |= CAN_ERROR_INTERRUPTS;
            }
#endif
            CLEAR_BIT(hcan->State, CAN_STATE_RECEIVE0);

            CLEAR_BIT(hcan->Inst->IER.w, temp);
        }

        /* receive complete callback */
        XPD_SAFE_CALLBACK(hcan->Callbacks.Receive[0], hcan);
    }
}

/**
 * @brief CAN receive FIFO 1 interrupt handler that provides handle callbacks.
 * @param hcan: pointer to the CAN handle structure
 */
void XPD_CAN_RX1_IRQHandler(CAN_HandleType * hcan)
{
    uint32_t temp;

    /* check reception completion */
    if (CAN_REG_BIT(hcan,IER,FMP1IE) && (hcan->Inst->RFR[1].b.FMP != 0))
    {
        /* get the FIFO contents to the requested frame structure */
        can_frameReceive(hcan, 1);

        /* only clear interrupt requests if they were enabled through XPD API */
        if ((hcan->State & CAN_STATE_RECEIVE1) != 0)
        {
            temp = CAN_RECEIVE1_INTERRUPTS;

#ifdef USE_XPD_CAN_ERROR_DETECT
            if ((hcan->State & (CAN_STATE_TRANSMIT | CAN_STATE_RECEIVE0)) == 0)
            {
                temp |= CAN_ERROR_INTERRUPTS;
            }
#endif
            CLEAR_BIT(hcan->State, CAN_STATE_RECEIVE1);

            CLEAR_BIT(hcan->Inst->IER.w, temp);
        }

        /* receive complete callback */
        XPD_SAFE_CALLBACK(hcan->Callbacks.Receive[1], hcan);
    }
}

/**
 * @brief CAN state change and error interrupt handler that provides handle callbacks.
 * @param hcan: pointer to the CAN handle structure
 */
void XPD_CAN_SCE_IRQHandler(CAN_HandleType * hcan)
{
    /* check if errors are configured for interrupt and present */
    if (    ((hcan->Inst->IER.w & (CAN_IER_BOFIE | CAN_IER_EPVIE | CAN_IER_EWGIE | CAN_IER_LECIE)) != 0)
         && ((hcan->Inst->ESR.w & (CAN_ESR_LEC | CAN_ESR_BOFF | CAN_ESR_EPVF | CAN_ESR_EWGF)) != 0))
    {
        /* call error callback function if interrupt is not by state change */
        XPD_SAFE_CALLBACK(hcan->Callbacks.Error, hcan);
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
 * @brief Configures the receive filters for the peripheral and provides each filter its own Filter Match Index (FMI).
 * @param hcan: pointer to the CAN handle structure
 * @param Filters: filter configuration list (array)
 * @param MatchIndexes: array to fill with the FMI of the filters - set to NULL to ignore
 * @param FilterCount: the number of input filters to configure
 * @return ERROR if filters cannot fit in the filter bank, OK if filters are added
 */
XPD_ReturnType XPD_CAN_FilterConfig(CAN_HandleType * hcan, const CAN_FilterType * Filters, uint8_t * MatchIndexes, uint8_t FilterCount)
{
    XPD_ReturnType result = XPD_OK;
    uint8_t fbDemand = 0, currentFMI = 0, remaining, currentType;
    CAN_TypeDef * CANx = CAN_MASTER(hcan);
#ifdef CAN_BB
    CAN_BitBand_TypeDef * CANx_BB = CAN_BB(CANx);
#endif

    uint8_t fbOffset = FILTERBANK_OFFSET(hcan);
    uint8_t fbCount  = FILTERBANK_COUNT(hcan);
    uint32_t mask    = (1 << fbCount) - 1;

#ifdef CAN2
#ifdef CAN3
    if (hcan->Inst != CAN3)
#endif /* CAN3 */
    {
        XPD_CAN1_ClockCtrl(ENABLE);
    }
#endif /* CAN2 */

    /* Enter filter initialization */
    CANx_REG_BIT(FMR,FINIT) = 1;

    /* Deactivate all filter banks assigned to this peripheral */
    CLEAR_BIT(CANx->FA1R, mask << fbOffset);

    /* Configure filters in a type assorted way */
    for (remaining = FilterCount, currentType = 0; remaining > 0; currentType++)
    {
        uint8_t i, fbankIndex, fbankPos = 255;
        union {
            uint16_t u16[4];
            uint32_t u32[2];
            CAN_FilterRegister_TypeDef filterReg;
        }FilterBank;

        /* Find all filters with the selected type in the list */
        for (i = 0; i < FilterCount; i++)
        {
            uint8_t type = (Filters[i].Mode & CAN_FILTER_MATCH)
                    | ((Filters[i].Pattern.Type & CAN_IDTYPE_EXT_DATA) >> 1);

            if (type == currentType)
            {
                /* If the current filter bank is full, set up another one */
                if (fbankPos >= filterTypeSpace[currentType])
                {
                    fbankIndex = fbDemand + fbOffset;

                    /* Check against overrun of used filter banks */
                    if ((fbDemand + 1) > fbCount)
                    {
                        remaining = 0;
                        result = XPD_ERROR;
                        break;
                    }

                    /* Configure new filter bank type */
#ifdef CAN_BB
                    /* Setting filter FIFO assignment */
                    CANx_BB->FFA1R[fbankIndex] = Filters[i].FIFO;

                    /* Setting filter mode */
                    CANx_BB->FM1R[fbankIndex]  = type;

                    /* Setting filter scale */
                    CANx_BB->FS1R[fbankIndex]  = type >> 1;
#else
                    uint32_t fbIndexMask = 1 << fbankIndex;

                    /* Setting filter FIFO assignment */
                    if (Filters[i].FIFO == 0)
                    {   CLEAR_BIT(CANx->FFA1R, fbIndexMask); }
                    else
                    {   SET_BIT(CANx->FFA1R, fbIndexMask); }

                    /* Setting filter mode */
                    if ((type & 1) == 0)
                    {   CLEAR_BIT(CANx->FM1R, fbIndexMask); }
                    else
                    {   SET_BIT(CANx->FM1R, fbIndexMask); }

                    /* Setting filter scale */
                    if ((type & 2) == 0)
                    {   CLEAR_BIT(CANx->FS1R, fbIndexMask); }
                    else
                    {   SET_BIT(CANx->FS1R, fbIndexMask); }
#endif /* CAN_BB */

                    fbankPos = 0;
                    fbDemand++;
                }

                /* Set the identifier pattern depending on the filter scale */
                if ((currentType & 2) == 0)
                {
                    FilterBank.u16[fbankPos] = (Filters[i].Pattern.Value << 5)
                                             | (Filters[i].Pattern.Type  << 2);

                    /* Set the masking bits */
                    if ((currentType & 1) == 0)
                    {
                        FilterBank.u16[fbankPos + 1] = (Filters[i].Mask << 5);

                        /* If mask does not pass any type, set the type bits */
                        if (Filters[i].Mode == CAN_FILTER_MASK)
                        {
                            FilterBank.u16[fbankPos + 1] |= CAN_IDTYPE_EXT_RTR << 2;
                        }
                    }
                }
                else
                {
                    FilterBank.u32[fbankPos] = (Filters[i].Pattern.Value << 3)
                                             | (Filters[i].Pattern.Type);

                    /* Set the masking bits */
                    if ((currentType & 1) == 0)
                    {
                        /* Filter bank size fixes the mask field location */
                        FilterBank.u32[1] = (Filters[i].Mask << 3);

                        /* If mask does not pass any type, set the type bits */
                        if (Filters[i].Mode == CAN_FILTER_MASK)
                        {
                            FilterBank.u32[1] |= CAN_IDTYPE_EXT_RTR;
                        }
                    }
                }

                /* Set the current filter's FMI */
                if (MatchIndexes != NULL)
                {
                    MatchIndexes[i] = currentFMI + fbankPos;
                }

                /* If the last element of the bank */
                if ((fbankPos + 1) >= filterTypeSpace[currentType])
                {
                    currentFMI += filterTypeSpace[currentType];

                    /* Set the configured bank in the peripheral */
                    CANx->sFilterRegister[fbankIndex] = FilterBank.filterReg;
                }
                /* Unused filters after the current one are set to
                 * the copies of the current to avoid receiving unwanted frames */
                else if (filterTypeSpace[type] == 2)
                {
                    FilterBank.u32[1] = FilterBank.u32[0];
                }
                else if (fbankPos == 0)
                {
                    FilterBank.u16[1] = FilterBank.u16[0];
                    FilterBank.u32[1] = FilterBank.u32[0];
                }
                else {}

                fbankPos++;
                remaining--;
            }
        }

        /* If the last bank was not filled completely */
        if (fbankPos < filterTypeSpace[currentType])
        {
            currentFMI += filterTypeSpace[currentType];

            /* Set the configured bank in the peripheral */
            CANx->sFilterRegister[fbankIndex] = FilterBank.filterReg;
        }
    }

    /* Activate all configured filter banks */
    mask = (1 << fbDemand) - 1;
    SET_BIT(CANx->FA1R, mask << fbOffset);

    /* Exit filter initialization */
    CANx_REG_BIT(FMR,FINIT) = 0;

    return result;
}

/**
 * @brief Sets the filter bank size for the CAN peripheral.
 * @note  This operation resets the filter configuration for the slave CAN controller.
 * @param hcan: pointer to the CAN handle structure
 * @param NewSize: the new filter bank size for the CAN peripheral [0 .. 28]
 * @return ERROR if size is invalid, or if size change would disable master CAN filters, OK if successful
 */
XPD_ReturnType XPD_CAN_FilterBankConfig(CAN_HandleType * hcan, uint8_t NewSize)
{
    XPD_ReturnType result = XPD_ERROR;

#ifdef CAN3
    if (hcan->Inst == CAN3)
    { /* Simply return with error */ }
    else
#endif /* CAN3 */
#ifdef CAN2
    if (NewSize <= 28)
    {
        uint32_t masterMask;

        XPD_CAN1_ClockCtrl(ENABLE);

        masterMask = (1 << CAN1->FMR.b.CAN2SB) - 1;

        /* Slave CAN bank size is counted from the end */
        if (hcan->Inst != CAN1)
        {
            NewSize = 28 - NewSize;
        }

        /* Do not allow the master CAN to lose active filters */
        if (((CAN1->FA1R & masterMask) & (0xFFFF << NewSize)) == 0)
        {
            CAN1->FMR.b.FINIT = 1;

            /* Deactivate slave CAN filter banks */
            CLEAR_BIT(CAN1->FA1R, ~masterMask);

            /* select the start slave bank */
            CAN1->FMR.b.CAN2SB = NewSize;

            CAN1->FMR.b.FINIT = 0;

            result = XPD_OK;
        }
    }
#endif /* CAN2 */

    return result;
}

/** @} */

/** @} */

/** @} */

#endif /* USE_XPD_CAN */
