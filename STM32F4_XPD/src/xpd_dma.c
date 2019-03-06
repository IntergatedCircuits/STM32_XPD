/**
  ******************************************************************************
  * @file    xpd_dma.c
  * @author  Benedek Kupper
  * @version 0.2
  * @date    2018-01-28
  * @brief   STM32 eXtensible Peripheral Drivers DMA Module
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
#include <xpd_dma.h>
#include <xpd_rcc.h>
#include <xpd_utils.h>

/** @addtogroup DMA
 * @{ */

#define DMA_ABORT_TIMEOUT   1000

#define DMA_BASE(STREAM)            ((((uint32_t)(STREAM) & 0xFF) < 0x70) ?     \
                        (void*)( (uint32_t)(STREAM) & (~(uint32_t)0x3FF)) :     \
                        (void*)(((uint32_t)(STREAM) & (~(uint32_t)0x3FF)) + 4))
#define DMA_BASE_OFFSET(STREAM)     (((uint32_t)(STREAM) < (uint32_t)DMA2) ? 0 : 1)
#define DMA_STREAM_NUMBER(STREAM)   ((((uint32_t)(STREAM) & 0xFF) - 16) / 24)

static uint8_t dma_aucUsers[] = {
        0,
#ifdef DMA2
        0
#endif
};

static void DMA_prvClockEnable(DMA_HandleType * pxDMA)
{
    uint32_t ulBO = DMA_BASE_OFFSET(pxDMA->Inst);

    if (dma_aucUsers[ulBO] == 0)
    {
        RCC_vClockEnable(RCC_POS_DMA1 + ulBO);
    }

    SET_BIT(dma_aucUsers[ulBO], 1 << DMA_STREAM_NUMBER(pxDMA->Inst));
}

static void DMA_prvClockDisable(DMA_HandleType * pxDMA)
{
    uint32_t ulBO = DMA_BASE_OFFSET(pxDMA->Inst);

    CLEAR_BIT(dma_aucUsers[ulBO], 1 << DMA_STREAM_NUMBER(pxDMA->Inst));

    if (dma_aucUsers[ulBO] == 0)
    {
        RCC_vClockDisable(RCC_POS_DMA1 + ulBO);
    }
}

/*
 * @brief Enables the DMA stream.
 * @param pxDMA: pointer to the DMA stream handle structure
 */
__STATIC_INLINE void DMA_prvEnable(DMA_HandleType * pxDMA)
{
    DMA_REG_BIT(pxDMA, CR, EN) = 1;
}

/*
 * @brief Disables the DMA stream.
 * @param pxDMA: pointer to the DMA stream handle structure
 */
__STATIC_INLINE void DMA_prvDisable(DMA_HandleType * pxDMA)
{
    DMA_REG_BIT(pxDMA, CR, EN) = 0;
}

__STATIC_INLINE void DMA_prvCalcBase(DMA_HandleType * pxDMA)
{
    uint8_t ucStream = DMA_STREAM_NUMBER(pxDMA->Inst);

    pxDMA->Base = DMA_BASE(pxDMA->Inst);
    pxDMA->StreamOffset = ((ucStream & 2) * 8) + ((ucStream & 1) * 6);
}

/** @defgroup DMA_Exported_Functions DMA Exported Functions
 * @{ */

/**
 * @brief Initializes the DMA stream using the setup configuration.
 * @param pxDMA: pointer to the DMA stream handle structure
 * @param pxConfig: DMA stream setup configuration
 */
void DMA_vInit(DMA_HandleType * pxDMA, const DMA_InitType * pxConfig)
{
    uint32_t ulCR;

    /* enable DMA clock */
    DMA_prvClockEnable(pxDMA);

    /* The memory and peripheral burst are forced to 0 when the FIFO is disabled */
    ulCR = pxConfig->w & (DMA_SxCR_PFCTRL | DMA_SxCR_DIR | DMA_SxCR_MINC | DMA_SxCR_MSIZE
            | DMA_SxCR_PINC | DMA_SxCR_PSIZE| DMA_SxCR_PINCOS | DMA_SxCR_PL
            | DMA_SxCR_PBURST | DMA_SxCR_MBURST | DMA_SxCR_CHSEL);

    /* Set the misplaced bits separately */
    if (pxConfig->Mode == DMA_MODE_CIRCULAR)
    {
        ulCR |= DMA_SxCR_CIRC;
    }
    else if (pxConfig->Mode == DMA_MODE_DBUFFER)
    {
        ulCR |= DMA_SxCR_DBM | DMA_SxCR_CIRC;
    }
    else {}

    pxDMA->Inst->CR.w = ulCR;

    /* FIFO configuration */
    if (pxConfig->FifoThreshold > 0)
    {
        pxDMA->Inst->FCR.w = DMA_SxFCR_DMDIS | ((pxConfig->FifoThreshold - 1) << DMA_SxFCR_FTH_Pos);
    }
    else
    {
        pxDMA->Inst->FCR.w = 0;
    }

    pxDMA->Inst->NDTR = 0;
    pxDMA->Inst->PAR = 0;

    /* calculate DMA steam Base Address */
    DMA_prvCalcBase(pxDMA);
}

/**
 * @brief Deinitializes the DMA stream.
 * @param pxDMA: pointer to the DMA stream handle structure
 */
void DMA_vDeinit(DMA_HandleType * pxDMA)
{
    DMA_prvDisable(pxDMA);

    /* configuration reset */
    pxDMA->Inst->CR.w = 0;
    pxDMA->Inst->FCR.w = 0;

    pxDMA->Inst->NDTR = 0;
    pxDMA->Inst->PAR = 0;

    /* Clear all interrupt flags */
    pxDMA->Base->LIFCR.w = (DMA_LIFCR_CFEIF0 | DMA_LIFCR_CDMEIF0 |
        DMA_LIFCR_CHTIF0 | DMA_LIFCR_CTCIF0 | DMA_LIFCR_CTEIF0)
                << (uint32_t)pxDMA->StreamOffset;

    /* disable DMA clock */
    DMA_prvClockDisable(pxDMA);
}

/**
 * @brief Sets up a DMA transfer and starts it.
 * @param pxDMA: pointer to the DMA stream handle structure
 * @param pvPeriphAddress: pointer to the peripheral data register
 * @param pvMemAddress: pointer to the memory data
 * @param usDataCount: the amount of data to be transferred
 * @return BUSY if DMA is in use, OK if success
 */
XPD_ReturnType DMA_eStart(
        DMA_HandleType *    pxDMA,
        void *              pvPeriphAddress,
        void *              pvMemAddress,
        uint16_t            usDataCount)
{
    XPD_ReturnType eResult = XPD_OK;

    /* Enter critical section to ensure single user of DMA */
    XPD_ENTER_CRITICAL(pxDMA);

    /* If previous user was a different peripheral, check busy state first */
    if ((uint32_t)pvPeriphAddress != pxDMA->Inst->PAR)
    {
        eResult = DMA_usGetStatus(pxDMA);
    }

    if (eResult == XPD_OK)
    {
        DMA_prvDisable(pxDMA);

        /* DMA transfer setup */
        pxDMA->Inst->NDTR = usDataCount;
        pxDMA->Inst->PAR  = (uint32_t)pvPeriphAddress;
        pxDMA->Inst->M0AR = (uint32_t)pvMemAddress;
#ifdef __XPD_DMA_ERROR_DETECT
        /* reset error state */
        pxDMA->Errors = DMA_ERROR_NONE;
#endif

        DMA_prvEnable(pxDMA);
    }
    else
    {
        eResult = XPD_BUSY;
    }

    XPD_EXIT_CRITICAL(pxDMA);

    return eResult;
}

/**
 * @brief Sets up a DMA transfer, starts it and produces completion callback using the interrupt stack.
 * @param pxDMA: pointer to the DMA stream handle structure
 * @param pvPeriphAddress: pointer to the peripheral data register
 * @param pvMemAddress: pointer to the memory data
 * @param usDataCount: the amount of data to be transferred
 * @return BUSY if DMA is in use, OK if success
 */
XPD_ReturnType DMA_eStart_IT(
        DMA_HandleType *    pxDMA,
        void *              pvPeriphAddress,
        void *              pvMemAddress,
        uint16_t            usDataCount)
{
    XPD_ReturnType eResult = DMA_eStart(pxDMA,
            pvPeriphAddress, pvMemAddress, usDataCount);

    if (eResult == XPD_OK)
    {
        /* enable interrupts
         * half transfer interrupt has to be enabled by user if callback is used */
#ifdef __XPD_DMA_ERROR_DETECT
        SET_BIT(pxDMA->Inst->CR.w, DMA_SxCR_TCIE | DMA_SxCR_TEIE | DMA_SxCR_DMEIE);
        DMA_REG_BIT(pxDMA,FCR,FEIE) = 1;
#else
        DMA_IT_ENABLE(pxDMA,TC);
#endif
    }
    return eResult;
}

/**
 * @brief Stops a DMA transfer.
 * @param pxDMA: pointer to the DMA stream handle structure
 */
void DMA_vStop(DMA_HandleType *pxDMA)
{
    uint32_t ulTimeout = DMA_ABORT_TIMEOUT;

    /* disable the stream */
    DMA_prvDisable(pxDMA);

    /* wait until stream is effectively disabled */
    XPD_eWaitForMatch(&pxDMA->Inst->CR.w, DMA_SxCR_EN, 0, &ulTimeout);
}

/**
 * @brief Stops a DMA transfer and disables all interrupt sources.
 * @param pxDMA: pointer to the DMA stream handle structure
 */
void DMA_vStop_IT(DMA_HandleType *pxDMA)
{
    /* disable the stream */
    DMA_prvDisable(pxDMA);

    /* disable interrupts */
    CLEAR_BIT(pxDMA->Inst->CR.w,
        DMA_SxCR_TCIE | DMA_SxCR_HTIE | DMA_SxCR_TEIE | DMA_SxCR_DMEIE);
#ifdef __XPD_DMA_ERROR_DETECT
    DMA_REG_BIT(pxDMA,FCR,FEIE) = 0;
#endif
}

/**
 * @brief Gets the remaining transfer length of the DMA stream.
 * @param pxDMA: pointer to the DMA stream handle structure
 * @return The number of transfers left until completion
 */
uint16_t DMA_usGetStatus(DMA_HandleType * pxDMA)
{
    return DMA_REG_BIT(pxDMA, CR, EN) * pxDMA->Inst->NDTR;
}

/**
 * @brief Polls the status of the DMA transfer.
 * @param pxDMA: pointer to the DMA stream handle structure
 * @param eOperation: the type of operation to check
 * @param ulTimeout: the timeout in ms for the polling.
 * @return ERROR if there were transfer errors, TIMEOUT if timed out, OK if successful
 */
XPD_ReturnType DMA_ePollStatus(
        DMA_HandleType *    pxDMA,
        DMA_OperationType   eOperation,
        uint32_t            ulTimeout)
{
    XPD_ReturnType eResult;
    uint32_t ulSMask;

    /* Assemble monitoring mask */
    ulSMask = (eOperation == DMA_OPERATION_TRANSFER) ?
            DMA_LISR_TCIF1 | DMA_LISR_TEIF1 | DMA_LISR_FEIF1 | DMA_LISR_DMEIF1
          : DMA_LISR_HTIF1 | DMA_LISR_TEIF1 | DMA_LISR_FEIF1 | DMA_LISR_DMEIF1;
    ulSMask <<= pxDMA->StreamOffset;

    /* Wait until any flags get active */
    eResult = XPD_eWaitForDiff(&pxDMA->Base->LISR.w, ulSMask, 0, &ulTimeout);

    if (eResult == XPD_OK)
    {
#ifdef __XPD_DMA_ERROR_DETECT
        if (DMA_FLAG_STATUS(pxDMA, TE))
        {
            /* Update error code */
            pxDMA->Errors |= DMA_ERROR_TRANSFER;

            /* Clear the transfer error flag */
            DMA_FLAG_CLEAR(pxDMA, TE);

            eResult = XPD_ERROR;
        }
        if (DMA_FLAG_STATUS(pxDMA, FE))
        {
            /* Update error code */
            pxDMA->Errors |= DMA_ERROR_FIFO;

            /* Clear the FIFO error flag */
            DMA_FLAG_CLEAR(pxDMA, FE);

            return XPD_ERROR;
        }
        if (DMA_FLAG_STATUS(pxDMA, DME))
        {
            /* Update error code */
            pxDMA->Errors |= DMA_ERROR_DIRECTM;

            /* Clear the Direct Mode error flag */
            DMA_FLAG_CLEAR(pxDMA, DME);

            return XPD_ERROR;
        }
        else
#endif
        {
            /* Clear the half transfer and transfer complete flags */
            if (eOperation == DMA_OPERATION_TRANSFER)
            {
                DMA_FLAG_CLEAR(pxDMA, TC);
            }
            DMA_FLAG_CLEAR(pxDMA, HT);
        }
    }

    return eResult;
}

/**
 * @brief DMA stream transfer interrupt handler that provides handle callbacks.
 * @param pxDMA: pointer to the DMA stream handle structure
 */
void DMA_vIRQHandler(DMA_HandleType * pxDMA)
{
    /* Half Transfer Complete interrupt management */
    if ((DMA_REG_BIT(pxDMA,CR,HTIE) != 0) && (DMA_FLAG_STATUS(pxDMA, HT) != 0))
    {
        /* clear the half transfer complete flag */
        DMA_FLAG_CLEAR(pxDMA, HT);

        /* half transfer callback */
        XPD_SAFE_CALLBACK(pxDMA->Callbacks.HalfComplete, pxDMA);
    }

    /* Transfer Complete interrupt management */
    if (DMA_FLAG_STATUS(pxDMA, TC) != 0)
    {
        /* clear the transfer complete flag */
        DMA_FLAG_CLEAR(pxDMA, TC);

        /* DMA mode is not CIRCULAR */
        if (DMA_eCircularMode(pxDMA) == 0)
        {
            CLEAR_BIT(pxDMA->Inst->CR.w,
                DMA_SxCR_TCIE | DMA_SxCR_HTIE | DMA_SxCR_TEIE | DMA_SxCR_DMEIE);
#ifdef __XPD_DMA_ERROR_DETECT
            DMA_REG_BIT(pxDMA,FCR,FEIE) = 0;
#endif
        }

        /* transfer complete callback */
        XPD_SAFE_CALLBACK(pxDMA->Callbacks.Complete, pxDMA);
    }

#ifdef __XPD_DMA_ERROR_DETECT
    /* Transfer Error interrupt management */
    if (DMA_FLAG_STATUS(pxDMA, TE) != 0)
    {
        /* clear the transfer error flag */
        DMA_FLAG_CLEAR(pxDMA, TE);

        pxDMA->Errors |= DMA_ERROR_TRANSFER;
    }
    /* FIFO Error interrupt management */
    if (DMA_FLAG_STATUS(pxDMA, FE) != 0)
    {
        /* clear the FIFO error flag */
        DMA_FLAG_CLEAR(pxDMA, FE);

        pxDMA->Errors |= DMA_ERROR_FIFO;
    }
    /* Direct Mode Error interrupt management */
    if (DMA_FLAG_STATUS(pxDMA, DME) != 0)
    {
        /* clear the direct mode error flag */
        DMA_FLAG_CLEAR(pxDMA, DME);

        pxDMA->Errors |= DMA_ERROR_DIRECTM;
    }

    if (pxDMA->Errors != 0)
    {
        /* transfer errors callback */
        XPD_SAFE_CALLBACK(pxDMA->Callbacks.Error, pxDMA);
    }
#endif
}

/**
 * @brief Gets the memory address register number which is currently used by the DMA stream.
 * @param pxDMA: pointer to the DMA stream handle structure
 * @return Index of the currently active memory address register
 */
uint32_t DMA_ulActiveMemory(DMA_HandleType * pxDMA)
{
    return DMA_REG_BIT(pxDMA, CR, CT);
}

/**
 * @brief Sets the memory address in the currently inactive register for double buffering transfer.
 * @param pxDMA: pointer to the DMA stream handle structure
 * @param pvAddress: The new memory address to use
 */
void DMA_vSetSwapMemory(DMA_HandleType * pxDMA, void * pvAddress)
{
    /* selects the MxAR register which is not used, and sets the new address */
    (&pxDMA->Inst->M0AR)[1 - DMA_ulActiveMemory(pxDMA)] = (uint32_t)pvAddress;
}

/** @} */

/** @} */
