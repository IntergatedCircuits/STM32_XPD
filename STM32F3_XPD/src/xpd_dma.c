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

#define DMA_BASE(CHANNEL)            ((DMA_TypeDef*)((uint32_t)(CHANNEL) & (~(uint32_t)0xFF)))
#ifdef DMA2
#define DMA_BASE_OFFSET(CHANNEL)     (((uint32_t)(CHANNEL) < (uint32_t)DMA2) ? 0 : 1)
#else
#define DMA_BASE_OFFSET(CHANNEL)     0
#endif
#define DMA_CHANNEL_NUMBER(CHANNEL)   ((((uint32_t)(CHANNEL) & 0xFF) - 8) / 20)

static uint8_t dma_users[] = {
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

    SET_BIT(dma_users[bo], 1 << DMA_CHANNEL_NUMBER(hdma->Inst));
}

static void dma_clockDisable(DMA_HandleType * hdma)
{
    uint32_t bo = DMA_BASE_OFFSET(hdma->Inst);

    CLEAR_BIT(dma_users[bo], 1 << DMA_CHANNEL_NUMBER(hdma->Inst));

    if (dma_users[bo] == 0)
    {
        XPD_RCC_ClockDisable(RCC_POS_DMA1 + bo);
    }
}

static void dma_calcBase(DMA_HandleType * hdma)
{
    hdma->Base = DMA_BASE(hdma->Inst);
    hdma->ChannelOffset = DMA_CHANNEL_NUMBER(hdma->Inst) * 4;
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

    hdma->Inst->CCR.b.PL         = Config->Priority;
    DMA_REG_BIT(hdma,CCR,DIR)    = Config->Direction;
    DMA_REG_BIT(hdma,CCR,CIRC)   = Config->Mode;
    DMA_REG_BIT(hdma,CCR,MEM2MEM)= Config->Direction >> 1;

    DMA_REG_BIT(hdma,CCR,PINC)   = Config->Peripheral.Increment;
    hdma->Inst->CCR.b.PSIZE      = Config->Peripheral.DataAlignment;

    DMA_REG_BIT(hdma,CCR,MINC)   = Config->Memory.Increment;
    hdma->Inst->CCR.b.MSIZE      = Config->Memory.DataAlignment;

    hdma->Inst->CNDTR = 0;
    hdma->Inst->CPAR  = 0;

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
    if (DMA_REG_BIT(hdma, CCR, EN) != 0)
    {
        XPD_DMA_Disable(hdma);

        /* configuration reset */
        hdma->Inst->CCR.w = 0;

        hdma->Inst->CNDTR = 0;
        hdma->Inst->CPAR = 0;

        /* Clear all interrupt flags */
        hdma->Base->IFCR.w =
           (DMA_IFCR_CHTIF1 | DMA_IFCR_CTCIF1 | DMA_IFCR_CTEIF1)
                    << (uint32_t)hdma->ChannelOffset;
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
    if ((uint32_t)PeriphAddress != hdma->Inst->CPAR)
    {
        result = XPD_DMA_GetStatus(hdma);
    }

    if (result == XPD_OK)
    {
        XPD_DMA_Disable(hdma);

        /* DMA transfer setup */
        hdma->Inst->CNDTR = DataCount;
        hdma->Inst->CPAR  = (uint32_t)PeriphAddress;
        hdma->Inst->CMAR  = (uint32_t)MemAddress;
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
        SET_BIT(hdma->Inst->CCR.w, DMA_CCR_TCIE | DMA_CCR_TEIE);
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
    result = XPD_WaitForMatch(&hdma->Inst->CCR.w, DMA_CCR_EN, 0, &timeout);

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
    CLEAR_BIT(hdma->Inst->CCR.w, DMA_CCR_TCIE | DMA_CCR_HTIE | DMA_CCR_TEIE);
}

/**
 * @brief Gets the remaining transfer length of the DMA stream.
 * @param hdma: pointer to the DMA stream handle structure
 * @return The number of transfers left until completion
 */
uint16_t XPD_DMA_GetStatus(DMA_HandleType * hdma)
{
    return DMA_REG_BIT(hdma, CCR, EN) * hdma->Inst->CNDTR;
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
            DMA_ISR_TCIF1 | DMA_ISR_TEIF1
          : DMA_ISR_HTIF1 | DMA_ISR_TEIF1;
    mask <<= hdma->ChannelOffset;

    /* Wait until any flags get active */
    result = XPD_WaitForDiff(&hdma->Base->ISR.w, mask, 0, &Timeout);
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
    if ((DMA_REG_BIT(hdma,CCR,HTIE) != 0) && (XPD_DMA_GetFlag(hdma, HT) != 0))
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
            CLEAR_BIT(hdma->Inst->CCR.w, DMA_CCR_TCIE | DMA_CCR_HTIE | DMA_CCR_TEIE);
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

        /* transfer errors callback */
        XPD_SAFE_CALLBACK(hdma->Callbacks.Error, hdma);
    }
#endif
}

/** @} */

/** @} */
