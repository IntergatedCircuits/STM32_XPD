/**
  ******************************************************************************
  * @file    xpd_dma.c
  * @author  Benedek Kupper
  * @version V0.1
  * @date    2016-11-01
  * @brief   STM32 eXtensible Peripheral Drivers DMA Module
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
#include "xpd_dma.h"
#include "xpd_rcc.h"
#include "xpd_utils.h"

/** @addtogroup DMA
 * @{ */

#define DMA_ABORT_TIMEOUT   1000

#define DMA_BASE(CHANNEL)            ((uint32_t)(CHANNEL) & (~(uint32_t)0xFF))
#ifdef DMA2
#define DMA_BASE_OFFSET(CHANNEL)     (((uint32_t)(CHANNEL) < (uint32_t)DMA2) ? 0 : 1)
#else
#define DMA_BASE_OFFSET(CHANNEL)     0
#endif
#define DMA_CHANNEL_NUMBER(CHANNEL)   ((((uint32_t)(CHANNEL) & 0xFF) - 8) / 20)

static const XPD_CtrlFnType dma_clkCtrl[] = {
        XPD_DMA_ClockCtrl,
#ifdef DMA2
        XPD_DMA2_ClockCtrl
#endif
};
static uint8_t dma_users[] = {
        0,
#ifdef DMA2
        0
#endif
};

static void dma_calcBase(DMA_HandleType * hdma)
{
#ifdef DMA_BB
    /* base points to the FEIF bit of the current stream (in BB alias) */
    hdma->Base_BB = DMA_BB(DMA_BASE(hdma->Inst));
    hdma->Base_BB += 16 * DMA_CHANNEL_NUMBER(hdma->Inst);
#else
    hdma->Base = DMA_BASE(hdma->Inst);
    hdma->ChannelOffset = DMA_CHANNEL_NUMBER(hdma->Inst) * 4;
#endif
}

static void dma_setConfig(DMA_HandleType * hdma)
{
    /* DMA Stream data length */
    hdma->Inst->CNDTR = hdma->Transfer.DataCount;

    /* if direction is mem2periph, M0 is source, P is destination */
    if (hdma->Inst->CCR.b.DIR == DMA_MEMORY2PERIPH)
    {
        /* DMA Stream source address */
        hdma->Inst->CMAR = (uint32_t)hdma->Transfer.SourceAddress;

        /* DMA Stream destination address */
        hdma->Inst->CPAR = (uint32_t)hdma->Transfer.DestAddress;
    }
    else
    {
        /* DMA Stream source address */
        hdma->Inst->CPAR = (uint32_t)hdma->Transfer.SourceAddress;

        /* DMA Stream destination address */
        hdma->Inst->CMAR = (uint32_t)hdma->Transfer.DestAddress;
    }

    /* reset error state */
    hdma->Errors = DMA_ERROR_NONE;
}

/** @defgroup DMA_Exported_Functions DMA Exported Functions
 * @{ */

/**
 * @brief Initializes the DMA stream using the setup configuration.
 * @param hdma: pointer to the DMA stream handle structure
 * @param Config: DMA stream setup configuration
 * @return ERROR if input is incorrect, OK if success
 */
XPD_ReturnType XPD_DMA_Init(DMA_HandleType * hdma, DMA_InitType * Config)
{
    /* enable DMA clock */
    {
        uint32_t bo = DMA_BASE_OFFSET(hdma->Inst);

        SET_BIT(dma_users[bo], 1 << DMA_CHANNEL_NUMBER(hdma->Inst));

        dma_clkCtrl[bo](ENABLE);
    }

#ifdef DMA_Channel_BB
    hdma->Inst_BB = DMA_Channel_BB(hdma->Inst);
#endif

    hdma->Inst->CCR.b.PL         = Config->Priority;
    hdma->Inst->CCR.b.DIR        = Config->Direction;
    DMA_REG_BIT(hdma,CCR,CIRC)   = Config->Mode;

    DMA_REG_BIT(hdma,CCR,PINC)   = Config->Peripheral.Increment;
    hdma->Inst->CCR.b.PSIZE      = Config->Peripheral.DataAlignment;

    DMA_REG_BIT(hdma,CCR,MINC)   = Config->Memory.Increment;
    hdma->Inst->CCR.b.MSIZE      = Config->Memory.DataAlignment;

    /* calculate DMA steam Base Address */
    dma_calcBase(hdma);

    return XPD_OK;
}

/**
 * @brief Deinitializes the DMA stream.
 * @param hdma: pointer to the DMA stream handle structure
 * @return ERROR if input is incorrect, OK if success
 */
XPD_ReturnType XPD_DMA_Deinit(DMA_HandleType * hdma)
{
    XPD_DMA_Disable(hdma);

    /* configuration reset */
    hdma->Inst->CCR.w = 0;

    hdma->Inst->CNDTR = 0;
    hdma->Inst->CPAR = 0;

    hdma->Inst->CMAR = 0;

    /* Clear all interrupt flags at correct offset within the register */
    XPD_DMA_ClearFlag(hdma, HT);
    XPD_DMA_ClearFlag(hdma, TC);
    XPD_DMA_ClearFlag(hdma, TE);

    /* disable DMA clock */
    {
        uint32_t bo = DMA_BASE_OFFSET(hdma->Inst);
        CLEAR_BIT(dma_users[bo], 1 << DMA_CHANNEL_NUMBER(hdma->Inst));

        if (dma_users[bo] == 0)
        {
            dma_clkCtrl[bo](DISABLE);
        }
    }

    return XPD_OK;
}

/**
 * @brief Enables the DMA stream.
 * @param hdma: pointer to the DMA stream handle structure
 */
void XPD_DMA_Enable(DMA_HandleType * hdma)
{
    DMA_REG_BIT(hdma, CCR, EN) = 1;
}

/**
 * @brief Disables the DMA stream.
 * @param hdma: pointer to the DMA stream handle structure
 */
void XPD_DMA_Disable(DMA_HandleType * hdma)
{
    DMA_REG_BIT(hdma, CCR, EN) = 0;
}

/**
 * @brief Sets up a DMA transfer and starts it.
 * @param hdma: pointer to the DMA stream handle structure
 */
void XPD_DMA_Start(DMA_HandleType * hdma)
{
    XPD_DMA_Disable(hdma);

    /* Configure the source, destination address and the data length */
    dma_setConfig(hdma);

    XPD_DMA_Enable(hdma);
}

/**
 * @brief Sets up a DMA transfer, starts it and produces completion callback using the interrupt stack.
 * @param hdma: pointer to the DMA stream handle structure
 */
void XPD_DMA_Start_IT(DMA_HandleType * hdma)
{
    XPD_DMA_Disable(hdma);

    /* Configure the source, destination address and the data length */
    dma_setConfig(hdma);

    /* enable interrupts */
#ifdef USE_XPD_DMA_ERROR_DETECT
    SET_BIT(hdma->Inst->CCR.w, (DMA_CCR_TCIE | DMA_CCR_HTIE | DMA_CCR_TEIE));
#else
    SET_BIT(hdma->Inst->CCR.w, (DMA_CCR_TCIE | DMA_CCR_HTIE));
#endif

    XPD_DMA_Enable(hdma);
}

/**
 * @brief Stops a DMA transfer.
 * @param hdma: pointer to the DMA stream handle structure
 * @return TIMEOUT if abort timed out, OK if successful
 */
XPD_ReturnType XPD_DMA_Stop(DMA_HandleType *hdma)
{
    XPD_ReturnType result;

    /* disable the stream */
    XPD_DMA_Disable(hdma);

    /* wait until stream is effectively disabled */
    result = XPD_WaitForMatch(&hdma->Inst->CCR.w, DMA_CCR_EN, 0, DMA_ABORT_TIMEOUT);

    return result;
}

/**
 * @brief Stops a DMA transfer and disables all interrupt sources.
 * @param hdma: pointer to the DMA stream handle structure
 */
void XPD_DMA_Stop_IT(DMA_HandleType *hdma)
{
    /* disable the stream */
    XPD_DMA_Disable(hdma);

    /* disable interrupts */
#ifdef USE_XPD_DMA_ERROR_DETECT
    CLEAR_BIT(hdma->Inst->CCR.w, (DMA_CCR_TCIE | DMA_CCR_HTIE | DMA_CCR_TEIE));
#else
    CLEAR_BIT(hdma->Inst->CCR.w, (DMA_CCR_TCIE | DMA_CCR_HTIE));
#endif
}

/**
 * @brief Polls the status of the DMA transfer.
 * @param hdma: pointer to the DMA stream handle structure
 * @param Operation: the type of operation to check
 * @param Timeout: the timeout in ms for the polling.
 * @return ERROR if there were transfer errors, TIMEOUT if timed out, OK if successful
 */
XPD_ReturnType XPD_DMA_PollStatus(DMA_HandleType * hdma, DMA_OperationType Operation, uint32_t Timeout)
{
    XPD_ReturnType result;
    uint32_t tickstart;
    bool success;

    /* Get tick */
    tickstart = XPD_GetTimer();

    for (success = (Operation == DMA_OPERATION_TRANSFER) ? XPD_DMA_GetFlag(hdma, TC) : XPD_DMA_GetFlag(hdma, HT);
        !success;
         success = (Operation == DMA_OPERATION_TRANSFER) ? XPD_DMA_GetFlag(hdma, TC) : XPD_DMA_GetFlag(hdma, HT))
    {
        if (XPD_DMA_GetFlag(hdma, TE))
        {
            /* Update error code */
            hdma->Errors |= DMA_ERROR_TRANSFER;

            /* Clear the transfer error flag */
            XPD_DMA_ClearFlag(hdma, TE);

            return XPD_ERROR;
        }
        /* Check for the Timeout */
        if ((Timeout != XPD_NO_TIMEOUT) && ((XPD_GetTimer() - tickstart) > Timeout))
        {
            return XPD_TIMEOUT;
        }
    }

    /* Clear the half transfer and transfer complete flags */
    if (Operation == DMA_OPERATION_TRANSFER)
    {
        XPD_DMA_ClearFlag(hdma, TC);
    }
    XPD_DMA_ClearFlag(hdma, HT);

    return XPD_OK;
}

/**
 * @brief Gets the error state of the DMA stream.
 * @param hdma: pointer to the DMA stream handle structure
 * @return Current DMA error state
 */
DMA_ErrorType XPD_DMA_GetError(DMA_HandleType * hdma)
{
    return hdma->Errors;
}

/**
 * @brief DMA stream transfer interrupt handler that provides handle callbacks.
 * @param hdma: pointer to the DMA stream handle structure
 */
void XPD_DMA_IRQHandler(DMA_HandleType * hdma)
{
    /* Half Transfer Complete interrupt management */
    if (XPD_DMA_GetFlag(hdma, HT))
    {
        /* clear the half transfer complete flag */
        XPD_DMA_ClearFlag(hdma, HT);

        /* DMA mode is not CIRCULAR */
        if (DMA_REG_BIT(hdma, CCR, CIRC) == 0)
        {
            XPD_DMA_DisableIT(hdma, HT);
        }

        /* half transfer callback */
        XPD_SAFE_CALLBACK(hdma->Callbacks.HalfComplete,hdma);
    }

    /* Transfer Complete interrupt management */
    if (XPD_DMA_GetFlag(hdma, TC))
    {
        /* clear the transfer complete flag */
        XPD_DMA_ClearFlag(hdma, TC);

        /* DMA mode is not CIRCULAR */
        if (DMA_REG_BIT(hdma, CCR, CIRC) == 0)
        {
            XPD_DMA_DisableIT(hdma, TC);
        }

        /* transfer complete callback */
        XPD_SAFE_CALLBACK(hdma->Callbacks.Complete,hdma);
    }

#ifdef USE_XPD_DMA_ERROR_DETECT
    /* Transfer Error interrupt management */
    if (XPD_DMA_GetFlag(hdma, TE))
    {
        /* clear the transfer error flag */
        XPD_DMA_ClearFlag(hdma, TE);

        hdma->Errors |= DMA_ERROR_TRANSFER;

        /* transfer errors callback */
        XPD_SAFE_CALLBACK(hdma->Callbacks.Error,hdma);
    }
#endif
}

/** @} */

/** @} */
