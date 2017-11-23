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

#define DMA_BASE(STREAM)            ((((uint32_t)(STREAM) & 0xFF) < 0x70) ?     \
                        (void*)( (uint32_t)(STREAM) & (~(uint32_t)0x3FF)) :     \
                        (void*)(((uint32_t)(STREAM) & (~(uint32_t)0x3FF)) + 4))
#define DMA_BASE_OFFSET(STREAM)     (((uint32_t)(STREAM) < (uint32_t)DMA2) ? 0 : 1)
#define DMA_STREAM_NUMBER(STREAM)   ((((uint32_t)(STREAM) & 0xFF) - 16) / 24)

static volatile uint8_t dma_users[] = {
        0,
#ifdef DMA2
        0
#endif
};

static void dma_clockEnable(DMA_HandleType * hdma)
{
    uint32_t bo = DMA_BASE_OFFSET(hdma->Inst);

    if (dma_users[bo] == 0)
    {
        XPD_RCC_ClockEnable(RCC_POS_DMA1 + bo);
    }

    SET_BIT(dma_users[bo], 1 << DMA_STREAM_NUMBER(hdma->Inst));
}

static void dma_clockDisable(DMA_HandleType * hdma)
{
    uint32_t bo = DMA_BASE_OFFSET(hdma->Inst);

    CLEAR_BIT(dma_users[bo], 1 << DMA_STREAM_NUMBER(hdma->Inst));

    if (dma_users[bo] == 0)
    {
        XPD_RCC_ClockDisable(RCC_POS_DMA1 + bo);
    }
}

static void dma_calcBase(DMA_HandleType * hdma)
{
    uint8_t streamNumber = DMA_STREAM_NUMBER(hdma->Inst);

    hdma->Base = DMA_BASE(hdma->Inst);
    hdma->StreamOffset = ((streamNumber & 2) * 8) + ((streamNumber & 1) * 6);
}

/** @defgroup DMA_Exported_Functions DMA Exported Functions
 * @{ */

/**
 * @brief Initializes the DMA stream using the setup configuration.
 * @param hdma: pointer to the DMA stream handle structure
 * @param Config: DMA stream setup configuration
 * @return ERROR if input is incorrect, OK if success
 */
XPD_ReturnType XPD_DMA_Init(DMA_HandleType * hdma, const DMA_InitType * Config)
{
    /* enable DMA clock */
    dma_clockEnable(hdma);

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

    hdma->Inst->NDTR = 0;
    hdma->Inst->PAR = 0;

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
    /* Verify that the channel is used */
    if (DMA_REG_BIT(hdma, CR, EN) != 0)
    {
        XPD_DMA_Disable(hdma);

        /* configuration reset */
        hdma->Inst->CR.w = 0;

        hdma->Inst->NDTR = 0;
        hdma->Inst->PAR = 0;

        /* FIFO control reset */
        hdma->Inst->FCR.w = 0x00000021;

        /* Clear all interrupt flags */
        hdma->Base->LIFCR.w = (DMA_LIFCR_CFEIF0 | DMA_LIFCR_CDMEIF0 |
            DMA_LIFCR_CHTIF0 | DMA_LIFCR_CTCIF0 | DMA_LIFCR_CTEIF0)
                    << (uint32_t)hdma->StreamOffset;
    }

    /* disable DMA clock */
    dma_clockDisable(hdma);

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
 * @param PeriphAddress: pointer to the peripheral data register
 * @param MemAddress: pointer to the memory data
 * @param DataCount: the amount of data to be transferred
 * @return BUSY if DMA is in use, OK if success
 */
XPD_ReturnType XPD_DMA_Start(DMA_HandleType * hdma, void * PeriphAddress, void * MemAddress, uint16_t DataCount)
{
    XPD_ReturnType result = XPD_OK;

    /* Enter critical section to ensure single user of DMA */
    XPD_ENTER_CRITICAL(hdma);

    /* If previous user was a different peripheral, check busy state first */
    if ((uint32_t)PeriphAddress != hdma->Inst->PAR)
    {
        result = XPD_DMA_GetStatus(hdma);
    }

    if (result == XPD_OK)
    {
        XPD_DMA_Disable(hdma);

        /* DMA transfer setup */
        hdma->Inst->NDTR = DataCount;
        hdma->Inst->PAR  = (uint32_t)PeriphAddress;
        hdma->Inst->M0AR = (uint32_t)MemAddress;

#ifdef USE_XPD_DMA_ERROR_DETECT
        /* reset error state */
        hdma->Errors = DMA_ERROR_NONE;
#endif

        XPD_DMA_Enable(hdma);
    }
    else
    {
        result = XPD_BUSY;
    }

    XPD_EXIT_CRITICAL(hdma);

    return result;
}

/**
 * @brief Sets up a DMA transfer, starts it and produces completion callback using the interrupt stack.
 * @param hdma: pointer to the DMA stream handle structure
 * @param PeriphAddress: pointer to the peripheral data register
 * @param MemAddress: pointer to the memory data
 * @param DataCount: the amount of data to be transferred
 * @return BUSY if DMA is in use, OK if success
 */
XPD_ReturnType XPD_DMA_Start_IT(DMA_HandleType * hdma, void * PeriphAddress, void * MemAddress, uint16_t DataCount)
{
    XPD_ReturnType result = XPD_DMA_Start(hdma, PeriphAddress, MemAddress, DataCount);

    if (result == XPD_OK)
    {
        /* enable interrupts
         * half transfer interrupt has to be enabled by user if callback is used */
#ifdef USE_XPD_DMA_ERROR_DETECT
        SET_BIT(hdma->Inst->CR.w, DMA_SxCR_TCIE | DMA_SxCR_TEIE | DMA_SxCR_DMEIE);
        DMA_REG_BIT(hdma,FCR,FEIE) = 1;
#else
        XPD_DMA_EnableIT(hdma,TC);
#endif
    }
    return result;
}

/**
 * @brief Stops a DMA transfer.
 * @param hdma: pointer to the DMA stream handle structure
 * @return TIMEOUT if abort timed out, OK if successful
 */
XPD_ReturnType XPD_DMA_Stop(DMA_HandleType *hdma)
{
    XPD_ReturnType result;
    uint32_t timeout = DMA_ABORT_TIMEOUT;

    /* disable the stream */
    XPD_DMA_Disable(hdma);

    /* wait until stream is effectively disabled */
    result = XPD_WaitForMatch(&hdma->Inst->CR.w, DMA_SxCR_EN, 0, &timeout);

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
    CLEAR_BIT(hdma->Inst->CR.w, DMA_SxCR_TCIE | DMA_SxCR_HTIE | DMA_SxCR_TEIE | DMA_SxCR_DMEIE);
#ifdef USE_XPD_DMA_ERROR_DETECT
    DMA_REG_BIT(hdma,FCR,FEIE) = 0;
#endif
}

/**
 * @brief Gets the remaining transfer length of the DMA stream.
 * @param hdma: pointer to the DMA stream handle structure
 * @return The number of transfers left until completion
 */
uint16_t XPD_DMA_GetStatus(DMA_HandleType * hdma)
{
    return DMA_REG_BIT(hdma, CR, EN) * hdma->Inst->NDTR;
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
    uint32_t mask;

    /* Assemble monitoring mask */
    mask = (Operation == DMA_OPERATION_TRANSFER) ?
            DMA_LISR_TCIF1 | DMA_LISR_TEIF1 | DMA_LISR_FEIF1 | DMA_LISR_DMEIF1
          : DMA_LISR_HTIF1 | DMA_LISR_TEIF1 | DMA_LISR_FEIF1 | DMA_LISR_DMEIF1;
    mask <<= hdma->StreamOffset;

    /* Wait until any flags get active */
    result = XPD_WaitForDiff(&hdma->Base->LISR.w, mask, 0, &Timeout);
    if (result == XPD_OK)
    {
#ifdef USE_XPD_DMA_ERROR_DETECT
        if (XPD_DMA_GetFlag(hdma, TE))
        {
            /* Update error code */
            hdma->Errors |= DMA_ERROR_TRANSFER;

            /* Clear the transfer error flag */
            XPD_DMA_ClearFlag(hdma, TE);

            result = XPD_ERROR;
        }
        if (XPD_DMA_GetFlag(hdma, FE))
        {
            /* Update error code */
            hdma->Errors |= DMA_ERROR_FIFO;

            /* Clear the FIFO error flag */
            XPD_DMA_ClearFlag(hdma, FE);

            return XPD_ERROR;
        }
        if (XPD_DMA_GetFlag(hdma, DME))
        {
            /* Update error code */
            hdma->Errors |= DMA_ERROR_DIRECTM;

            /* Clear the Direct Mode error flag */
            XPD_DMA_ClearFlag(hdma, DME);

            return XPD_ERROR;
        }
        else
#endif
        {
            /* Clear the half transfer and transfer complete flags */
            if (Operation == DMA_OPERATION_TRANSFER)
            {
                XPD_DMA_ClearFlag(hdma, TC);
            }
            XPD_DMA_ClearFlag(hdma, HT);
        }
    }

    return result;
}

/**
 * @brief DMA stream transfer interrupt handler that provides handle callbacks.
 * @param hdma: pointer to the DMA stream handle structure
 */
void XPD_DMA_IRQHandler(DMA_HandleType * hdma)
{
    /* Half Transfer Complete interrupt management */
    if ((DMA_REG_BIT(hdma,CR,HTIE) != 0) && (XPD_DMA_GetFlag(hdma, HT) != 0))
    {
        /* clear the half transfer complete flag */
        XPD_DMA_ClearFlag(hdma, HT);

        /* half transfer callback */
        XPD_SAFE_CALLBACK(hdma->Callbacks.HalfComplete, hdma);
    }

    /* Transfer Complete interrupt management */
    if (XPD_DMA_GetFlag(hdma, TC) != 0)
    {
        /* clear the transfer complete flag */
        XPD_DMA_ClearFlag(hdma, TC);

        /* DMA mode is not CIRCULAR */
        if (XPD_DMA_CircularMode(hdma) == 0)
        {
            CLEAR_BIT(hdma->Inst->CR.w, DMA_SxCR_TCIE | DMA_SxCR_HTIE | DMA_SxCR_TEIE | DMA_SxCR_DMEIE);
#ifdef USE_XPD_DMA_ERROR_DETECT
            DMA_REG_BIT(hdma,FCR,FEIE) = 0;
#endif
        }

        /* transfer complete callback */
        XPD_SAFE_CALLBACK(hdma->Callbacks.Complete, hdma);
    }

#ifdef USE_XPD_DMA_ERROR_DETECT
    /* Transfer Error interrupt management */
    if (XPD_DMA_GetFlag(hdma, TE) != 0)
    {
        /* clear the transfer error flag */
        XPD_DMA_ClearFlag(hdma, TE);

        hdma->Errors |= DMA_ERROR_TRANSFER;
    }
    /* FIFO Error interrupt management */
    if (XPD_DMA_GetFlag(hdma, FE) != 0)
    {
        /* clear the FIFO error flag */
        XPD_DMA_ClearFlag(hdma, FE);

        hdma->Errors |= DMA_ERROR_FIFO;
    }
    /* Direct Mode Error interrupt management */
    if (XPD_DMA_GetFlag(hdma, DME) != 0)
    {
        /* clear the direct mode error flag */
        XPD_DMA_ClearFlag(hdma, DME);

        hdma->Errors |= DMA_ERROR_DIRECTM;
    }

    if (hdma->Errors != 0)
    {
        /* transfer errors callback */
        XPD_SAFE_CALLBACK(hdma->Callbacks.Error, hdma);
    }
#endif
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
