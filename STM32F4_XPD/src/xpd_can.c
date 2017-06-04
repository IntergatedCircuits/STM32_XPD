/**
  ******************************************************************************
  * @file    xpd_can.c
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
#include "xpd_can.h"
#include "xpd_utils.h"

#if defined(USE_XPD_CAN) && defined(CAN_MASTER)

/** @addtogroup CAN
 * @{ */

#ifndef CAN_TSR_CODE_Pos
#define CAN_TSR_CODE_Pos 24U
#endif

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

#ifdef __DUAL_CAN_DEVICE
static boolean_t can_slaveFiltersUnused();
#endif

/** @defgroup CAN_Private_Functions CAN Private Functions
 * @{ */

/**
 * @brief Gets an empty transmit mailbox.
 * @param hcan: pointer to the CAN handle structure
 * @param mailbox: Set to the lowest number empty mailbox
 * @return BUSY if all mailboxes are full, OK if empty mailbox found
 */
static XPD_ReturnType can_getEmptyMailbox(CAN_HandleType * hcan, uint8_t * mailbox)
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
    MODIFY_REG(hcan->Inst->MCR.w, 0xFC, Config->Features.All);

    /* set the bit timing register (with test mode) */
    hcan->Inst->BTR.w = (uint32_t)Config->Features.All << 30;
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
    hcan->State  = 0;

    /* reset filter banks */
    XPD_CAN_FilterBankReset(hcan);

    return result;
}

/**
 * @brief Restores the CAN peripheral to its default inactive state.
 * @param hcan: pointer to the CAN handle structure
 * @return ERROR if input is incorrect, OK if success
 */
XPD_ReturnType XPD_CAN_Deinit(CAN_HandleType * hcan)
{
    /* disable all interrupt requests */
    hcan->Inst->IER.w = 0;

    /* disable filters */
    XPD_CAN_FilterBankReset(hcan);

    /* disable peripheral clock */
#ifdef __DUAL_CAN_DEVICE
    /* master clock is only off when no filters are used */
    if ((hcan->Inst != CAN1) || can_slaveFiltersUnused())
#endif
    {
        hcan->ClockCtrl(DISABLE);
    }

    /* Deinitialize peripheral dependencies */
    XPD_SAFE_CALLBACK(hcan->Callbacks.DepDeinit, hcan);

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
    uint32_t temp;
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
            temp = CAN_TSR_TXOK0 << (temp * 8);

            /* wait for txok with the remaining time */
            result = XPD_WaitForMatch(&hcan->Inst->TSR.w, temp, temp, &Timeout);
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
    XPD_ReturnType result;
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
    else
    {
        result = XPD_BUSY;
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
    XPD_ReturnType result;
    uint8_t recState = CAN_STATE_RECEIVE0 << FIFONumber;

    /* check if FIFO is not in use */
    if ((hcan->State & recState) == 0)
    {
        SET_BIT(hcan->State, recState);

        /* save receive data target */
        hcan->RxFrame[FIFONumber] = Frame;

        SET_BIT(hcan->Inst->IER.w,
            CAN_ERROR_INTERRUPTS | ((FIFONumber == 0) ? CAN_RECEIVE0_INTERRUPTS : CAN_RECEIVE1_INTERRUPTS));

        result = XPD_OK;
    }
    else
    {
        result = XPD_BUSY;
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

            if ((hcan->State & CAN_STATE_RECEIVE) == 0)
            {
                temp |= CAN_ERROR_INTERRUPTS;
            }
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

            if ((hcan->State & (CAN_STATE_TRANSMIT | CAN_STATE_RECEIVE1)) == 0)
            {
                temp |= CAN_ERROR_INTERRUPTS;
            }
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

            if ((hcan->State & (CAN_STATE_TRANSMIT | CAN_STATE_RECEIVE0)) == 0)
            {
                temp |= CAN_ERROR_INTERRUPTS;
            }
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

__STATIC_INLINE void can_filterBankEnable(uint8_t FilterNumber)
{
#ifdef CAN_BB
    CAN_BB(CAN_MASTER)->FA1R[FilterNumber] = 1;
#else
    SET_BIT(CAN_MASTER->FA1R, 1 << (uint32_t)FilterNumber);
#endif
}

__STATIC_INLINE void can_filterBankDisable(uint8_t FilterNumber)
{
#ifdef CAN_BB
    CAN_BB(CAN_MASTER)->FA1R[FilterNumber] = 0;
#else
    CLEAR_BIT(CAN_MASTER->FA1R, 1 << (uint32_t)FilterNumber);
#endif
}

__STATIC_INLINE uint8_t can_getFilterBankOffset(CAN_HandleType * hcan)
{
#ifdef __DUAL_CAN_DEVICE
    if (hcan->Inst == CAN_MASTER)
    {
        return 0;
    }
    else
    {
        return CAN_MASTER->FMR.b.CAN2SB;
    }
#else
    return 0;
#endif
}

__STATIC_INLINE uint8_t can_getFilterBankSize(CAN_HandleType * hcan)
{
#ifdef __DUAL_CAN_DEVICE
    if (hcan->Inst == CAN_MASTER)
    {
        return CAN_MASTER->FMR.b.CAN2SB - 1;
    }
    else
    {
        return CAN_FILTERBANK_NUMBER;
    }
#else
    return CAN_FILTERBANK_NUMBER;
#endif
}

typedef enum
{
    Std_Mask = 0,
    Std_Match = 1,
    Ext_Mask = 2,
    Ext_Match = 3,
    FILTERTYPE_INVALID = 4
}FilterInfoType;

static const uint8_t filterTypeSpace[4] = {2, 4, 1, 2};

static struct
{
    uint8_t type;
    uint8_t configuredFields;
} FilterInfo[CAN_FILTERBANK_NUMBER];

typedef union
{
    uint16_t u16[4];
    uint32_t u32[2];
}FilterRegister;

#ifdef __DUAL_CAN_DEVICE
static boolean_t can_slaveFiltersUnused()
{
    return FilterInfo[CAN_MASTER->FMR.b.CAN2SB].type == FILTERTYPE_INVALID;
}
#endif

/**
 * @brief Resets the receive filter bank configurations for the CAN peripheral.
 * @param hcan: pointer to the CAN handle structure
 */
void XPD_CAN_FilterBankReset(CAN_HandleType * hcan)
{
    uint8_t filterbankoffset, filterbanksize, i;

#ifdef __DUAL_CAN_DEVICE
    XPD_CAN1_ClockCtrl(ENABLE);
#endif

    filterbankoffset = can_getFilterBankOffset(hcan);
    filterbanksize = can_getFilterBankSize(hcan);

    for (i = filterbankoffset; i < filterbanksize; i++)
    {
        /* disable all matched filters */
        can_filterBankDisable(i);
        /* set type value to invalid */
        FilterInfo[i].type = FILTERTYPE_INVALID;
        FilterInfo[i].configuredFields = 0;
    }
}

/**
 * @brief Configures a receive filter and sets its Match Index (FMI).
 * @param hcan: pointer to the CAN handle structure
 * @param FIFONumber: the selected receive FIFO [0 .. 1]
 * @param Filter: pointer to the filter configuration
 * @return ERROR if filter cannot fit in the filter bank, OK if filter is added
 */
XPD_ReturnType XPD_CAN_FilterInit(CAN_HandleType * hcan, uint8_t FIFONumber, CAN_FilterType * Filter)
{
    uint8_t filterbanksize, filterbankoffset, fbank, fplace, i, confFieldMask;
    uint32_t fFifoReg;
    FilterRegister FR;
    uint8_t type = (Filter->Mode & CAN_FILTER_MATCH) | ((Filter->Pattern.Type & CAN_IDTYPE_EXT_DATA) >> 1);
    XPD_ReturnType result = XPD_ERROR;

#ifdef __DUAL_CAN_DEVICE
    XPD_CAN1_ClockCtrl(ENABLE);
#endif

    fFifoReg = CAN_MASTER->FFA1R;
    filterbanksize = can_getFilterBankSize(hcan);
    filterbankoffset = can_getFilterBankOffset(hcan);

    fbank = filterbanksize;

    /* see if the filter can be paired up in an already configured bank */
    if (type != Ext_Mask)
    {
        confFieldMask = (1 << filterTypeSpace[type]) - 1;
        for (fbank = filterbankoffset; fbank < filterbanksize; fbank++)
        {
            if (    (FilterInfo[fbank].type == type)
                 && (FilterInfo[fbank].configuredFields != confFieldMask)
                 && (((fFifoReg >> fbank) & 1) == FIFONumber))
                break;
        }
    }

    /* try to find empty filter bank */
    if (fbank == filterbanksize)
    {
        for (fbank = filterbankoffset; fbank < filterbanksize; fbank++)
        {
            /* type is set to invalid at reset */
            if (FilterInfo[fbank].type == FILTERTYPE_INVALID)
            {
                CAN_MASTER_REG_BIT(FMR,FINIT) = 1;
#ifdef CAN_BB
                CAN_BB(CAN_MASTER)->FFA1R[fbank] = FIFONumber;
                CAN_BB(CAN_MASTER)->FM1R[fbank]  = type; /* mode is either mask or ID match */
                CAN_BB(CAN_MASTER)->FS1R[fbank]  = type >> 1; /* 32 bit scale for ext ID, 16 bit for std ID */
#else
                fFifoReg = 1 << fbank;

                if (FIFONumber == 0)
                    CLEAR_BIT(CAN_MASTER->FFA1R,fFifoReg);
                else
                    SET_BIT(CAN_MASTER->FFA1R,fFifoReg);

                if ((type & 1) == 0)
                    CLEAR_BIT(CAN_MASTER->FM1R,fFifoReg);
                else
                    SET_BIT(CAN_MASTER->FM1R,fFifoReg);

                if ((type & 2) == 0)
                    CLEAR_BIT(CAN_MASTER->FS1R,fFifoReg);
                else
                    SET_BIT(CAN_MASTER->FS1R,fFifoReg);
#endif /* CAN_BB */
                CAN_MASTER_REG_BIT(FMR,FINIT) = 0;

                fFifoReg = CAN_MASTER->FFA1R;
                FilterInfo[fbank].type = type;
                break;
            }
        }
    }

    /* if any filter bank is available */
    if (fbank < filterbanksize)
    {
        /* find the first unused filter field in the selected bank */
        for (fplace = 0; fplace < filterTypeSpace[type]; fplace++)
        {
            if ((FilterInfo[fbank].configuredFields & (1 << fplace)) == 0)
            {
                /* copy the contents of the filter bank to local variable */
                FR.u32[0] = CAN_MASTER->sFilterRegister[fbank].FR1;
                FR.u32[1] = CAN_MASTER->sFilterRegister[fbank].FR2;

                /* fill the selected field with the filter configuration */
                switch (type)
                {
                    case Std_Mask:
                    {
                        FR.u16[fplace] = Filter->Pattern.Value << 5;
                        if (Filter->Pattern.Type & CAN_IDTYPE_STD_RTR)
                        {
                            FR.u16[fplace] |= 0x10;
                        }
                        FR.u16[fplace + 1] = Filter->Mask << 5;
                        if ((Filter->Mode & CAN_FILTER_MASK_ANYTYPE) == 0)
                        {
                            FR.u16[fplace + 1] |= 0x18;
                        }
                        break;
                    }
                    case Std_Match:
                    {
                        FR.u16[fplace] = Filter->Pattern.Value << 5;
                        if (Filter->Pattern.Type & CAN_IDTYPE_STD_RTR)
                        {
                            FR.u16[fplace] |= 0x10;
                        }
                        break;
                    }
                    case Ext_Mask:
                    {
                        FR.u32[fplace] = (Filter->Pattern.Value << 3) | (Filter->Pattern.Type & CAN_IDTYPE_EXT_RTR);
                        FR.u32[fplace + 1] = (Filter->Mask << 3);
                        if ((Filter->Mode & CAN_FILTER_MASK_ANYTYPE) == 0)
                        {
                            FR.u32[fplace + 1] |= Filter->Pattern.Type & CAN_IDTYPE_EXT_RTR;
                        }
                        break;
                    }
                    case Ext_Match:
                    {
                        FR.u32[fplace] = (Filter->Pattern.Value << 3) | (Filter->Pattern.Type & CAN_IDTYPE_EXT_RTR);
                        break;
                    }
                }
                /* save active configuration in context */
                SET_BIT(FilterInfo[fbank].configuredFields, 1 << fplace);

                /* unused filters after the current one are set to the copies of the current */
                for (i = fplace + 1; i < filterTypeSpace[type]; i++)
                {
                    if ((FilterInfo[fbank].configuredFields & (1 << i)) == 0)
                    {
                        if (filterTypeSpace[type] == 4)
                            FR.u16[i] = FR.u16[fplace];
                        else
                            FR.u32[i] = FR.u32[fplace];
                    }
                }
                /* disable filter bank to enable changing settings */
                can_filterBankDisable(fbank);

                /* copy back the configured bank to the peripheral */
                CAN_MASTER->sFilterRegister[fbank].FR1 = FR.u32[0];
                CAN_MASTER->sFilterRegister[fbank].FR2 = FR.u32[1];

                /* enable filter bank */
                can_filterBankEnable(fbank);

                /* set the filter match index in the config structure */
                Filter->MatchIndex = fplace;
                for (i = filterbankoffset; i < fbank; i++)
                {
                    if (((fFifoReg >> fbank) & 1) == FIFONumber)
                        Filter->MatchIndex += filterTypeSpace[FilterInfo[i].type];
                }

                result = XPD_OK;
                break;
            }
        }
    }

    return result;
}

/**
 * @brief Updates a receive Filter Match Index. Necessary after a filter has been removed from the same filter bank.
 * @param hcan: pointer to the CAN handle structure
 * @param FIFONumber: the selected receive FIFO [0 .. 1]
 * @param MatchIndex: pointer to the old Filter Match Index that will be refreshed after the operation
 * @return ERROR if no filter with the input match index was found, OK if successful
 */
XPD_ReturnType XPD_CAN_FilterIndexUpdate(CAN_HandleType * hcan, uint8_t FIFONumber, uint8_t * MatchIndex)
{
    uint8_t filterbankoffset, filterbanksize, fbank, fplace;
    uint8_t offset = 0;
    uint32_t fFifoReg;
    XPD_ReturnType result = XPD_ERROR;

    fFifoReg = CAN_MASTER->FFA1R;
    filterbankoffset = can_getFilterBankOffset(hcan);
    filterbanksize = can_getFilterBankSize(hcan);

    for (fbank = filterbankoffset; fbank < filterbanksize; fbank++)
    {
        if ((FilterInfo[fbank].configuredFields != 0) && (((fFifoReg >> fbank) & 1) == FIFONumber))
        {
            if (*MatchIndex < (offset + filterTypeSpace[FilterInfo[fbank].type]))
            {
                /* due to removable filters, the filter can be copied to preceding locations,
                 * if no preceding filters are configured */
                for (fplace = 0; fplace < (*MatchIndex - offset); fplace++)
                {
                    if ((FilterInfo[fbank].configuredFields & (1 << fplace)) != 0)
                    {
                        break;
                    }
                }
                /* no preceding configured filter is found */
                if (fplace == (*MatchIndex - offset))
                {
                    *MatchIndex = offset;
                    FilterInfo[fbank].configuredFields = (FilterInfo[fbank].configuredFields & ~(1 << fplace)) | 1;
                }
                result = XPD_OK;
                break;
            }
            offset += filterTypeSpace[FilterInfo[fbank].type];
        }
    }

    return result;
}

/**
 * @brief Removes a receive filter by its Match Index (FMI).
 * @param hcan: pointer to the CAN handle structure
 * @param FIFONumber: the selected receive FIFO [0 .. 1]
 * @param MatchIndex: pointer to the reference of the filter
 * @return ERROR if no filter with the input match index was found, OK if successful
 */
XPD_ReturnType XPD_CAN_FilterDeinit(CAN_HandleType * hcan, uint8_t FIFONumber, uint8_t * MatchIndex)
{
    uint8_t filterbankoffset, filterbanksize, fbank, fplace, i;
    uint8_t offset = 0;
    uint32_t fFifoReg, temp;
    FilterRegister FR;
    XPD_ReturnType result = XPD_ERROR;

    fFifoReg = CAN_MASTER->FFA1R;
    filterbankoffset = can_getFilterBankOffset(hcan);
    filterbanksize = can_getFilterBankSize(hcan);

    for (fbank = filterbankoffset; fbank < filterbanksize; fbank++)
    {
        if ((FilterInfo[fbank].configuredFields != 0) && (((fFifoReg >> fbank) & 1) == FIFONumber))
        {
            if (*MatchIndex < (offset + filterTypeSpace[FilterInfo[fbank].type]))
            {
                /* the filter is not active, error */
                temp = 1 << (*MatchIndex - offset);
                if ((FilterInfo[fbank].configuredFields & temp) == 0)
                {
                    break;
                }
                CLEAR_BIT(FilterInfo[fbank].configuredFields, temp);

                /* if it is possible to rearrange filters within the bank */
                if (FilterInfo[fbank].type != Ext_Mask)
                {
                    /* check if any other configured filter is in the bank */
                    for (fplace = 0; fplace < filterTypeSpace[FilterInfo[fbank].type]; fplace++)
                    {
                        if ((FilterInfo[fbank].configuredFields & (1 << fplace)) != 0)
                        {
                            /* copy the contents of the filter bank to local variable */
                            FR.u32[0] = CAN_MASTER->sFilterRegister[fbank].FR1;
                            FR.u32[1] = CAN_MASTER->sFilterRegister[fbank].FR2;

                            if (filterTypeSpace[FilterInfo[fbank].type] == 4)
                            {
                                for (i = 0; i < 4; i++)
                                {
                                    if (i != fplace)
                                        FR.u16[i] = FR.u16[fplace];
                                }
                            }
                            else
                            {
                                FR.u32[1 - fplace] = FR.u32[fplace];
                            }
                            /* disable filter bank to enable changing settings */
                            can_filterBankDisable(fbank);

                            /* copy back the configured bank to the peripheral */
                            CAN_MASTER->sFilterRegister[fbank].FR1 = FR.u32[0];
                            CAN_MASTER->sFilterRegister[fbank].FR2 = FR.u32[1];

                            /* enable filter bank */
                            can_filterBankEnable(fbank);

                            result = XPD_OK;
                            break;
                        }
                    }
                }
                /* filter is ext_mask, or filter bank is empty, disable it */
                if (result != XPD_OK)
                {
                    can_filterBankDisable(fbank);
                }

                result = XPD_OK;
                break;
            }
            offset += filterTypeSpace[FilterInfo[fbank].type];
        }
    }

    return result;
}

#ifdef __DUAL_CAN_DEVICE

/**
 * Sets the filter bank size for the CAN peripheral.
 * @param hcan: pointer to the CAN handle structure
 * @param NewSize: the new filter bank size for the CAN peripheral [0 .. 28]
 * @return ERROR if size is invalid, or if size change would disable master CAN filters, OK if successful
 * @note Operation resets the filter configuration for the slave CAN controller.
 */
XPD_ReturnType XPD_CAN_FilterBankSizeConfig(CAN_HandleType * hcan, uint8_t NewSize)
{
    uint8_t prevBanks, i;
    XPD_ReturnType result = XPD_OK;

    XPD_CAN1_ClockCtrl(ENABLE);

    if(NewSize > CAN_FILTERBANK_NUMBER)
    {
        result = XPD_ERROR;
    }
    else
    {
        /* slave CAN bank size is counted from the end */
        if (hcan->Inst != CAN_MASTER)
        {
            NewSize = 28 - NewSize;
        }

        prevBanks = CAN_MASTER->FMR.b.CAN2SB;

        /* if the master CAN is losing filters, check if there are any configured */
        if (prevBanks > NewSize)
        {
            for (i = NewSize; i < prevBanks; i++)
            {
                if (FilterInfo[i].configuredFields != 0)
                {
                    result = XPD_ERROR;
                    break;
                }
            }
        }

        if (result == XPD_OK)
        {
            /* reset all slave filters, as their indexes get misaligned due to the change */
            for (i = prevBanks; i < CAN_FILTERBANK_NUMBER; i++)
            {
                /* disable all matched filters */
                can_filterBankDisable(i);
                /* set type value to invalid */
                FilterInfo[i].type = FILTERTYPE_INVALID;
                FilterInfo[i].configuredFields = 0;
            }

            CAN_MASTER_REG_BIT(FMR,FINIT) = 1;

            /* select the start slave bank */
            CAN_MASTER->FMR.b.CAN2SB = NewSize;

            CAN_MASTER_REG_BIT(FMR,FINIT) = 0;
        }
    }

    return result;
}
#endif /* __DUAL_CAN_DEVICE */

/** @} */

/** @} */

/** @} */

#endif /* defined(USE_XPD_CAN) && defined(CAN_MASTER) */
