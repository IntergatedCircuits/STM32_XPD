/**
  ******************************************************************************
  * @file    xpd_dma.c
  * @author  Benedek Kupper
  * @version V0.1
  * @date    2016-01-19
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

#define DMA_BASE(STREAM)            ((uint32_t)(STREAM) & (~(uint32_t)0x3FF))
#define DMA_BASE_OFFSET(STREAM)     (((uint32_t)(STREAM) < (uint32_t)DMA2) ? 0 : 1)
#define DMA_STREAM_NUMBER(STREAM)   ((((uint32_t)(STREAM) & 0xFF) - 16) / 24)

static const uint8_t dma_streamBitOffset[8] = {0, 24, 64, 88, 128, 152, 192, 216};

static const XPD_CtrlFnType dma_clkCtrl[] = {
        XPD_DMA1_ClockCtrl,
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
    /* base points to the FEIF bit of the current stream (in BB alias) */
    hdma->Base_BB = DMA_BB(DMA_BASE(hdma->Inst));
    hdma->Base_BB += dma_streamBitOffset[DMA_STREAM_NUMBER(hdma->Inst)];
}

static void dma_setConfig(DMA_HandleType * hdma, DMA_TransferType * Config)
{
    /* DMA Stream data length */
    hdma->Inst->NDTR = Config->DataCount;

    /* if direction is mem2periph, M0 is source, P is destination */
    if (hdma->Inst->CR.b.DIR == DMA_MEMORY2PERIPH)
    {
        /* DMA Stream source address */
        hdma->Inst->M0AR = (uint32_t)Config->SourceAddress;

        /* DMA Stream destination address */
        hdma->Inst->PAR = (uint32_t)Config->DestAddress;
    }
    else
    {
        /* DMA Stream source address */
        hdma->Inst->PAR = (uint32_t)Config->SourceAddress;

        /* DMA Stream destination address */
        hdma->Inst->M0AR = (uint32_t)Config->DestAddress;
    }

    /* reset error state */
    hdma->Errors = DMA_ERROR_NONE;
}

void dma_getConfig(DMA_HandleType * hdma, DMA_TransferType * Config)
{
    /* DMA Stream data length */
    Config->DataCount = hdma->Inst->NDTR;

    /* if direction is mem2periph, M0 is source, P is destination */
    if (hdma->Inst->CR.b.DIR == DMA_MEMORY2PERIPH)
    {
        /* DMA Stream source address */
        Config->SourceAddress = (void*)hdma->Inst->M0AR;

        /* DMA Stream destination address */
        Config->DestAddress = (void*)hdma->Inst->PAR;
    }
    else
    {
        /* DMA Stream source address */
        Config->SourceAddress = (void*)hdma->Inst->PAR;

        /* DMA Stream destination address */
        Config->DestAddress = (void*)hdma->Inst->M0AR;
    }
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

        SET_BIT(dma_users[bo], 1 << DMA_STREAM_NUMBER(hdma->Inst));

        dma_clkCtrl[bo](ENABLE);
    }

#ifdef DMA_Stream_BB
    hdma->Inst_BB = DMA_Stream_BB(hdma->Inst);
#endif

    DMA_REG_BIT(hdma,CR,CT) = 0;

    hdma->Inst->CR.b.CHSEL      = Config->Channel;
    hdma->Inst->CR.b.PL         = Config->Priority;
    hdma->Inst->CR.b.DIR        = Config->Direction;
    DMA_REG_BIT(hdma,CR,CIRC)   = Config->Mode;
    DMA_REG_BIT(hdma,CR,PFCTRL) = Config->Mode >> 1;
    DMA_REG_BIT(hdma,CR,DBM)    = Config->Mode >> 2;

    DMA_REG_BIT(hdma,CR,PINC)   = Config->Peripheral.Increment;
    hdma->Inst->CR.b.PSIZE      = Config->Peripheral.DataAlignment;

    DMA_REG_BIT(hdma,CR,MINC)   = Config->Memory.Increment;
    hdma->Inst->CR.b.MSIZE      = Config->Memory.DataAlignment;

    /* the memory burst and peripheral burst are not used when the FIFO is disabled */
    if (Config->FIFO.Mode == ENABLE)
    {
        /* set memory burst and peripheral burst */
        hdma->Inst->CR.b.MBURST = Config->Memory.Burst;
        hdma->Inst->CR.b.PBURST = Config->Peripheral.Burst;

        hdma->Inst->FCR.b.FTH   = Config->FIFO.Threshold;
    }
    else
    {
        hdma->Inst->CR.b.MBURST = 0;
        hdma->Inst->CR.b.PBURST = 0;

        hdma->Inst->FCR.b.FTH   = 0;
    }
    DMA_REG_BIT(hdma,FCR,DMDIS) = Config->FIFO.Mode;

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
    DMA_BitBand_TypeDef *regs;

    XPD_DMA_Disable(hdma);

    /* configuration reset */
    hdma->Inst->CR.w = 0;

    hdma->Inst->NDTR = 0;
    hdma->Inst->PAR = 0;

    hdma->Inst->M0AR = 0;
    hdma->Inst->M1AR = 0;

    /* FIFO control reset */
    hdma->Inst->FCR.w = (uint32_t) 0x00000021;

    regs = (DMA_BitBand_TypeDef*)hdma->Base_BB;

    /* Clear all interrupt flags at correct offset within the register */
    regs->LIFCR.CDMEIF0 = 0;
    regs->LIFCR.CFEIF0 = 0;
    regs->LIFCR.CHTIF0 = 0;
    regs->LIFCR.CTCIF0 = 0;
    regs->LIFCR.CTEIF0 = 0;

    /* disable DMA clock */
    {
        uint32_t bo = DMA_BASE_OFFSET(hdma->Inst);
        CLEAR_BIT(dma_users[bo], 1 << DMA_STREAM_NUMBER(hdma->Inst));

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
    DMA_REG_BIT(hdma, CR, EN) = 1;
}

/**
 * @brief Disables the DMA stream.
 * @param hdma: pointer to the DMA stream handle structure
 */
void XPD_DMA_Disable(DMA_HandleType * hdma)
{
    DMA_REG_BIT(hdma, CR, EN) = 0;
}

/**
 * @brief Sets up a DMA transfer and starts it.
 * @param hdma: pointer to the DMA stream handle structure
 * @param Config: pointer to the transfer configuration
 */
void XPD_DMA_Start(DMA_HandleType * hdma, DMA_TransferType * Config)
{
    XPD_DMA_Disable(hdma);

    /* Configure the source, destination address and the data length */
    dma_setConfig(hdma, Config);

    XPD_DMA_Enable(hdma);
}

/**
 * @brief Sets up a DMA transfer, starts it and produces completion callback using the interrupt stack.
 * @param hdma: pointer to the DMA stream handle structure
 * @param Config: pointer to the transfer configuration
 */
void XPD_DMA_Start_IT(DMA_HandleType * hdma, DMA_TransferType * Config)
{
    XPD_DMA_Disable(hdma);

    /* Configure the source, destination address and the data length */
    dma_setConfig(hdma, Config);

    /* enable interrupts */
    SET_BIT(hdma->Inst->CR.w, (DMA_SxCR_TCIE | DMA_SxCR_HTIE | DMA_SxCR_TEIE | DMA_SxCR_DMEIE));
    DMA_REG_BIT(hdma,FCR,FEIE) = 1;

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
    result = XPD_WaitForMatch(&hdma->Inst->CR.w, DMA_SxCR_EN, 0, DMA_ABORT_TIMEOUT);

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
    CLEAR_BIT(hdma->Inst->CR.w, (DMA_SxCR_TCIE | DMA_SxCR_HTIE | DMA_SxCR_TEIE | DMA_SxCR_DMEIE));
    DMA_REG_BIT(hdma,FCR,FEIE) = 0;
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
    DMA_BitBand_TypeDef * flags = (DMA_BitBand_TypeDef*)hdma->Base_BB;
    uint32_t tickstart;
    __IO uint32_t *complete;

    /* get the level transfer complete flag */
    if (Operation == DMA_OPERATION_TRANSFER)
    {
        complete = &flags->LISR.TCIF0;
    }
    else
    {
        complete = &flags->LISR.HTIF0;
    }

    /* Get tick */
    tickstart = XPD_GetTimer();

    while ((*complete) == 0)
    {
        if (flags->LISR.TEIF0 != 0)
        {
            /* Update error code */
            hdma->Errors |= DMA_ERROR_TRANSFER;

            /* Clear the transfer error flag */
            flags->LIFCR.CTEIF0 = 1;

            return XPD_ERROR;
        }
        if (flags->LISR.FEIF0 != 0)
        {
            /* Update error code */
            hdma->Errors |= DMA_ERROR_FIFO;

            /* Clear the FIFO error flag */
            flags->LIFCR.CFEIF0 = 1;

            return XPD_ERROR;
        }
        if (flags->LISR.DMEIF0 != 0)
        {
            /* Update error code */
            hdma->Errors |= DMA_ERROR_DIRECTM;

            /* Clear the Direct Mode error flag */
            flags->LIFCR.CDMEIF0 = 1;

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
        flags->LIFCR.CTCIF0 = 1;
    }
    flags->LIFCR.CHTIF0 = 1;

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
    DMA_BitBand_TypeDef *flags = (DMA_BitBand_TypeDef*)hdma->Base_BB;
    uint32_t errors = 0;

    /* Half Transfer Complete interrupt management */
    if (flags->LISR.HTIF0 != 0)
    {
        /* clear the half transfer complete flag */
        flags->LIFCR.CHTIF0 = 1;

        /* multi buffering mode disabled, DMA mode is not CIRCULAR */
        if ((hdma->Inst->CR.w & (DMA_SxCR_CIRC | DMA_SxCR_DBM)) == 0)
        {
            XPD_DMA_DisableIT(hdma, HT);
        }

        /* half transfer callback */
        XPD_SAFE_CALLBACK(hdma->Callbacks.HalfComplete,hdma);
    }

    /* Transfer Complete interrupt management */
    if (flags->LISR.TCIF0 != 0)
    {
        /* clear the transfer complete flag */
        flags->LIFCR.CTCIF0 = 1;

        /* multi buffering mode disabled, DMA mode is not CIRCULAR */
        if ((hdma->Inst->CR.w & (DMA_SxCR_CIRC | DMA_SxCR_DBM)) == 0)
        {
            XPD_DMA_DisableIT(hdma, TC);
        }

        /* transfer complete callback */
        XPD_SAFE_CALLBACK(hdma->Callbacks.Complete,hdma);
    }

    /* Transfer Error interrupt management */
    if (flags->LISR.TEIF0 != 0)
    {
        /* clear the transfer error flag */
        flags->LIFCR.CTEIF0 = 1;

        errors |= DMA_ERROR_TRANSFER;
    }
    /* FIFO Error interrupt management */
    if (flags->LISR.FEIF0 != 0)
    {
        /* clear the FIFO error flag */
        flags->LIFCR.CFEIF0 = 1;

        errors |= DMA_ERROR_FIFO;
    }
    /* Direct Mode Error interrupt management */
    if (flags->LISR.DMEIF0 != 0)
    {
        /* clear the direct mode error flag */
        flags->LIFCR.CDMEIF0 = 1;

        errors |= DMA_ERROR_DIRECTM;
    }

    if (errors != 0)
    {
        hdma->Errors |= errors;

        /* transfer errors callback */
        XPD_SAFE_CALLBACK(hdma->Callbacks.Error,hdma);
    }
}

/**
 * @brief Gets the memory address register number which is currently used by the DMA stream.
 * @param hdma: pointer to the DMA stream handle structure
 * @return Index of the currently active memory address register
 */
uint32_t XPD_DMA_GetActiveMemory(DMA_HandleType * hdma)
{
    return DMA_REG_BIT(hdma, CR, CT);
}

/**
 * @brief Sets the memory address in the currently inactive register for double buffering transfer.
 * @param hdma: pointer to the DMA stream handle structure
 * @param Address: The new memory address to use
 */
void XPD_DMA_SetSwapMemory(DMA_HandleType * hdma, void * Address)
{
    /* selects the MxAR register which is not used, and sets the new address */
    (&hdma->Inst->M0AR)[1 - XPD_DMA_GetActiveMemory(hdma)] = (uint32_t)Address;
}

/** @} */

/** @} */
