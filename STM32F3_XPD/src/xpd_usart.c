/**
  ******************************************************************************
  * @file    xpd_usart.c
  * @author  Benedek Kupper
  * @version 0.2
  * @date    2018-01-28
  * @brief   STM32 eXtensible Peripheral Drivers USART Module
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

#include <xpd_usart.h>
#include <xpd_utils.h>

/** @addtogroup USART
 * @{ */

/** @addtogroup USART_Common
 * @{ */

/* clock is enabled */
#define USART_SYNCHRONOUS_MODE(HANDLE) (USART_REG_BIT(HANDLE, CR2, CLKEN) != 0)

/* half-duplex */
#define USART_HALF_DUPLEX_MODE(HANDLE) (USART_REG_BIT(HANDLE, CR3, HDSEL) != 0)

#define USART_APPLY_CONFIG(HANDLE, REG, MASK, CONFIG)                               \
    do{ uint32_t ulCR1 = (HANDLE)->Inst->CR1.w; uint32_t uartEn = ulCR1 & USART_CR1_UE; \
    if (uartEn != 0) { pxUSART->Inst->CR1.w = ulCR1 & ~USART_CR1_UE; }                 \
    MODIFY_REG((HANDLE)->Inst->REG.w, (MASK), (CONFIG));                            \
    pxUSART->Inst->CR1.w |= uartEn;} while (0)

#if defined(__XPD_USART_ERROR_DETECT) || defined(__XPD_DMA_ERROR_DETECT)
#define USART_RESET_ERRORS(HANDLE)    ((HANDLE)->Errors = USART_ERROR_NONE)
#else
#define USART_RESET_ERRORS(HANDLE)    ((void)0)
#endif

#if (__USART_PERIPHERAL_VERSION > 1)
#define USART_STATR(HANDLE)         ((HANDLE)->Inst->ISR.w)
#define USART_STATF(FLAG_NAME)      (USART_ISR_##FLAG_NAME)
#define USART_RXDR(HANDLE)          ((HANDLE)->Inst->RDR)
#define USART_TXDR(HANDLE)          ((HANDLE)->Inst->TDR)

#define USART_INVERSION_MASK        (USART_CR2_TXINV | USART_CR2_RXINV | USART_CR2_DATAINV | USART_CR2_MSBFIRST | USART_CR2_SWAP)
#define USART_BAUDRATEMODE_MASK     (USART_CR2_ABREN | USART_CR2_ABRMODE)
#else
#define USART_STATR(HANDLE)         ((HANDLE)->Inst->SR.w)
#define USART_STATF(FLAG_NAME)      (USART_SR_##FLAG_NAME)
#define USART_RXDR(HANDLE)          ((HANDLE)->Inst->DR)
#define USART_TXDR(HANDLE)          ((HANDLE)->Inst->DR)

#define USART_INVERSION_MASK        0
#define USART_BAUDRATEMODE_MASK     0
#endif

#ifdef __XPD_DMA_ERROR_DETECT
static void USART_prvDmaErrorRedirect(void *pxDMA)
{
    USART_HandleType * pxUSART = (USART_HandleType*) ((DMA_HandleType*) pxDMA)->Owner;

    pxUSART->Errors |= USART_ERROR_DMA;

    XPD_SAFE_CALLBACK(pxUSART->Callbacks.Error, pxUSART);
}
#endif

static void USART_prvDmaTransmitRedirect(void *pxDMA)
{
    USART_HandleType * pxUSART = (USART_HandleType*) ((DMA_HandleType*) pxDMA)->Owner;

    /* DMA normal mode */
    if (DMA_eCircularMode((DMA_HandleType*)pxDMA) == 0)
    {
        /* Disable Tx DMA Request */
        USART_REG_BIT(pxUSART, CR3, DMAT) = 0;

        /* Update stream status */
        pxUSART->TxStream.buffer += pxUSART->TxStream.length * pxUSART->TxStream.size;
        pxUSART->TxStream.length = 0;
    }

    /* If the completion of the character sending isn't waited for,
     * provide callback now */
    if (USART_REG_BIT(pxUSART, CR1, TCIE) == 0)
    {
        XPD_SAFE_CALLBACK(pxUSART->Callbacks.Transmit, pxUSART);
    }
}

static void USART_prvDmaReceiveRedirect(void *pxDMA)
{
    USART_HandleType * pxUSART = (USART_HandleType*) ((DMA_HandleType*) pxDMA)->Owner;

    /* DMA normal mode */
    if (DMA_eCircularMode((DMA_HandleType*)pxDMA) == 0)
    {
        /* Disable Rx DMA Request */
        USART_REG_BIT(pxUSART, CR3, DMAR) = 0;

        /* Update stream status */
        pxUSART->RxStream.buffer += pxUSART->RxStream.length * pxUSART->RxStream.size;
        pxUSART->RxStream.length = 0;
    }
    XPD_SAFE_CALLBACK(pxUSART->Callbacks.Receive, pxUSART);
}

/* Calculates and configures the baudrate */
static void USART_prvSetBaudrate(USART_HandleType * pxUSART, uint32_t ulBaudrate)
{
#if (__USART_PERIPHERAL_VERSION > 1)
    if (USART_REG_BIT(pxUSART, CR1, OVER8) == 0)
    {
        pxUSART->Inst->BRR.w = (USART_ulClockFreq_Hz(pxUSART) + (ulBaudrate / 2)) / ulBaudrate;
    }
    else
    {
        uint32_t ulTmp = ((USART_ulClockFreq_Hz(pxUSART) * 2) + (ulBaudrate / 2)) / ulBaudrate;
        pxUSART->Inst->BRR.w = (ulTmp & USART_BRR_DIV_MANTISSA) | ((ulTmp & USART_BRR_DIV_FRACTION) >> 1);
    }
#else
    uint32_t ulOver8   = USART_REG_BIT(pxUSART, CR1, OVER8);
    uint32_t ulDiv     = USART_ulClockFreq_Hz(pxUSART) * 25 / ((4 >> ulOver8) * ulBaudrate);
    uint32_t ulDivMant = ulDiv / 100;
    uint32_t ulDivFraq = ((ulDiv - (ulDivMant * 100)) * (16 >> ulOver8) + 50) / 100;

    /* Mantissa is filled with the USARTDIV integer part + fractional overflow */
    pxUSART->Inst->BRR.b.DIV_Mantissa = ulDivMant + (ulDivFraq >> (4 - ulOver8));
    /* Fraction gets the fractional part (4 - over8 bits) */
    pxUSART->Inst->BRR.b.DIV_Fraction = ulDivFraq & (0xf >> ulOver8);
#endif
}

/* Enables the USART peripheral */
__STATIC_INLINE void USART_prvEnable(USART_HandleType * pxUSART)
{
    USART_REG_BIT(pxUSART,CR1,UE) = 1;
}

/* Disables the USART peripheral */
__STATIC_INLINE void USART_prvDisable(USART_HandleType * pxUSART)
{
    USART_REG_BIT(pxUSART,CR1,UE) = 0;
}

#ifdef IS_UART_WAKEUP_FROMSTOP_INSTANCE
static void USART_prvWaitIdle(USART_HandleType * pxUSART)
{
    uint32_t ulTimeout = 1000;
    uint32_t ulFlags = (USART_ISR_TEACK | USART_ISR_REACK) &
            ((pxUSART->Inst->CR1.w & (USART_CR1_TE | USART_CR1_RE)) <<
                    (USART_ISR_TEACK_Pos - USART_CR1_TE_Pos));

    (void) XPD_eWaitForMatch(&pxUSART->Inst->ISR.w, ulFlags, ulFlags, &ulTimeout);
}
#endif

static void USART_prvPreinit(USART_HandleType * pxUSART, uint8_t ucDataSize, USART_ParityType eParity)
{
    uint8_t ucFrameSize = ucDataSize + ((eParity != USART_PARITY_NONE) ? 1 : 0);

    /* enable clock */
    RCC_vClockEnable(pxUSART->CtrlPos);

    pxUSART->Inst->CR1.w = (eParity << USART_CR1_PS_Pos) & (USART_CR1_PS | USART_CR1_PCE);
    pxUSART->Inst->CR2.w = 0;
    pxUSART->Inst->CR3.w = 0;

    /* Dependencies initialization */
    XPD_SAFE_CALLBACK(pxUSART->Callbacks.DepInit, pxUSART);

#ifdef USART_CR1_M1
    USART_REG_BIT(pxUSART, CR1, M0)     = (ucFrameSize > 8) ? 1 : 0;
    USART_REG_BIT(pxUSART, CR1, M1)     = (ucFrameSize < 8) ? 1 : 0;
#else
    USART_REG_BIT(pxUSART, CR1, M)      = (ucFrameSize > 8) ? 1 : 0;
#endif

    /* Set data transfer size */
    switch (pxUSART->Inst->CR1.w & (USART_CR1_M | USART_CR1_PCE))
    {
        /* data size = 9, no parity -> mask = 0x1FF */
#ifdef USART_CR1_M0
        case USART_CR1_M0:
#else
        case USART_CR1_M:
#endif
            pxUSART->TxStream.size = pxUSART->RxStream.size = 2;
            break;

#ifdef USART_CR1_M1
        /* data size = 7 including parity -> mask = 0x3F */
        case USART_CR1_M1 | USART_CR1_PCE:
        /* data size = 8 including parity -> mask = 0x7F */
        case USART_CR1_M1:
#endif
        /* data size = 8 including parity -> mask = 0x7F */
        case USART_CR1_PCE:
        /* data size = 8, or = 9 with parity -> mask = 0xFF */
        default:
            pxUSART->TxStream.size = pxUSART->RxStream.size = 1;
            break;
    }
    pxUSART->TxStream.length = pxUSART->RxStream.length = 0;
}

/** @defgroup USART_Common_Exported_Functions USART Common Exported Functions
 * @{ */

/**
 * @brief Restores the USART peripheral to its default inactive state
 * @param pxUSART: pointer to the USART handle structure
 * @return ERROR if input is incorrect, OK if success
 */
void USART_vDeinit(USART_HandleType * pxUSART)
{
    USART_prvDisable(pxUSART);

    /* Deinitialize peripheral dependencies */
    XPD_SAFE_CALLBACK(pxUSART->Callbacks.DepDeinit, pxUSART);

    /* disable clock */
    RCC_vClockDisable(pxUSART->CtrlPos);
}

/**
 * @brief Sets the communication direction in half-duplex mode.
 * @param pxUSART: pointer to the USART handle structure
 * @param eDirection: desired communication direction(s)
 */
void USART_vSetDirection(USART_HandleType * pxUSART, USART_DirectionType eDirection)
{
    uint32_t ulCR1 = pxUSART->Inst->CR1.w;

    if ((ulCR1 & (USART_CR1_TE | USART_CR1_RE)) != eDirection)
    {
        CLEAR_BIT(ulCR1, USART_CR1_UE | USART_CR1_TE | USART_CR1_RE);
        pxUSART->Inst->CR1.w = ulCR1;
        SET_BIT(ulCR1, USART_CR1_UE | eDirection);
        pxUSART->Inst->CR1.w = ulCR1;
    }
}

/**
 * @brief Determines the current status of USART peripheral.
 * @param pxUSART: pointer to the USART handle structure
 * @return BUSY if a transmission is in progress, OK if USART is ready for new transfer
 */
XPD_ReturnType USART_eGetStatus(USART_HandleType * pxUSART)
{
    return ((USART_STATR(pxUSART) & (USART_STATF(RXNE) | USART_STATF(TC))) != 0)
            ? XPD_OK : XPD_BUSY;
}

/**
 * @brief Polls the status of the USART transfer.
 * @param pxUSART: pointer to the USART handle structure
 * @param ulTimeout: the timeout in ms for the polling.
 * @return ERROR if there were transfer errors, TIMEOUT if timed out, OK if successful
 */
XPD_ReturnType USART_ePollStatus(USART_HandleType * pxUSART, uint32_t ulTimeout)
{
#ifdef __XPD_USART_ERROR_DETECT
    /* wait until not busy, or until an error is present */
    XPD_ReturnType eResult = XPD_eWaitForDiff(&USART_STATR(pxUSART),
            USART_STATF(RXNE) | USART_STATF(TC) | USART_STATF(PE) |
            USART_STATF(FE)   | USART_STATF(NE) | USART_STATF(ORE),
            0, &ulTimeout);

    /* Error checks */
    if (USART_FLAG_STATUS(pxUSART, PE))
    {
        /* Update error code */
        pxUSART->Errors |= USART_ERROR_PARITY;

        /* Clear the transfer error flag */
        USART_FLAG_CLEAR(pxUSART, PE);

        eResult = XPD_ERROR;
    }
    if (USART_FLAG_STATUS(pxUSART, NE))
    {
        /* Update error code */
        pxUSART->Errors |= USART_ERROR_NOISE;

        /* Clear the transfer error flag */
        USART_FLAG_CLEAR(pxUSART, NE);

        eResult = XPD_ERROR;
    }
    if (USART_FLAG_STATUS(pxUSART, FE))
    {
        /* Update error code */
        pxUSART->Errors |= USART_ERROR_FRAME;

        /* Clear the transfer error flag */
        USART_FLAG_CLEAR(pxUSART, FE);

        eResult = XPD_ERROR;
    }
    if (USART_FLAG_STATUS(pxUSART, ORE))
    {
        /* Update error code */
        pxUSART->Errors |= USART_ERROR_OVERRUN;

        /* Clear the transfer error flag */
        USART_FLAG_CLEAR(pxUSART, ORE);

        eResult = XPD_ERROR;
    }
#else
    XPD_ReturnType eResult = XPD_eWaitForDiff(&USART_STATR(pxUSART),
            USART_STATF(RXNE) | USART_STATF(TC), 0, &ulTimeout);
#endif
    return eResult;
}

/**
 * @brief Transmits data over USART.
 * @param pxUSART: pointer to the USART handle structure
 * @param pvTxData: pointer to the data buffer
 * @param usLength: amount of data transfers
 * @param ulTimeout: available time for successful transmission in ms
 * @return TIMEOUT if transmission or reception times out,
 *         OK if transfer is completed
 */
XPD_ReturnType USART_eTransmit(
        USART_HandleType *  pxUSART,
        void *              pvTxData,
        uint16_t            usLength,
        uint32_t            ulTimeout)
{
    XPD_ReturnType eResult;

    /* save stream info */
    pxUSART->TxStream.buffer = pvTxData;
    pxUSART->TxStream.length = usLength;
    USART_RESET_ERRORS(pxUSART);

    while (pxUSART->TxStream.length > 0)
    {
        /* wait for empty transmit buffer */
        eResult = XPD_eWaitForDiff(&USART_STATR(pxUSART), USART_STATF(TXE), 0, &ulTimeout);
        if (eResult != XPD_OK)
        {
            return eResult;
        }

        XPD_vWriteFromStream((uint32_t*)&USART_TXDR(pxUSART), &pxUSART->TxStream);
    }
    /* wait for the last transmission to finish */
    eResult = XPD_eWaitForDiff(&USART_STATR(pxUSART), USART_STATF(TC), 0, &ulTimeout);

    /* Clear overrun flag because received data is not read */
    USART_FLAG_CLEAR(pxUSART, ORE);

    return eResult;
}

/**
 * @brief Transmits data over USART and waits for the completion of the transfer.
 * @param pxUSART: pointer to the USART handle structure
 * @param pvTxData: pointer to the data buffer
 * @param usLength: amount of data transfers
 * @param ulTimeout: available time for successful transmission in ms
 * @return TIMEOUT if transmission or reception times out,
 *         OK if transfer is completed
 */
XPD_ReturnType USART_eSend(
        USART_HandleType *  pxUSART,
        void *              pvTxData,
        uint16_t            usLength,
        uint32_t            ulTimeout)
{
    XPD_ReturnType eResult = USART_eTransmit(pxUSART, pvTxData, usLength, ulTimeout);

    if (eResult == XPD_OK)
    {
        /* wait for the last transmission to finish */
        eResult = XPD_eWaitForDiff(&USART_STATR(pxUSART), USART_STATF(TC), 0, &ulTimeout);

        /* Clear overrun flag because received data is not read */
        USART_FLAG_CLEAR(pxUSART, ORE);
    }

    return eResult;
}

/**
 * @brief Receives data over USART.
 * @param pxUSART: pointer to the USART handle structure
 * @param pvRxData: pointer to the data buffer
 * @param usLength: amount of data transfers
 * @param ulTimeout: available time for successful transmission in ms
 * @return TIMEOUT if transmission or reception times out,
 *         ERROR if a reception error occurs,
 *         OK if transfer is completed
 */
XPD_ReturnType USART_eReceive(
        USART_HandleType *  pxUSART,
        void *              pvRxData,
        uint16_t            usLength,
        uint32_t            ulTimeout)
{
    XPD_ReturnType eResult;

    /* save stream info */
    pxUSART->RxStream.buffer = pvRxData;
    pxUSART->RxStream.length = usLength;
    USART_RESET_ERRORS(pxUSART);

    while (pxUSART->RxStream.length > 0)
    {
        /* Send data to generate clock */
        if (USART_SYNCHRONOUS_MODE(pxUSART) && (USART_FLAG_STATUS(pxUSART, TXE) != 0))
        {
            USART_TXDR(pxUSART) = 0;
        }

        /* Wait for not empty receive buffer */
        eResult = XPD_eWaitForDiff(&USART_STATR(pxUSART), USART_STATF(RXNE), 0, &ulTimeout);
        if (eResult != XPD_OK)
        {
            return eResult;
        }

        XPD_vReadToStream((const uint32_t*)&USART_RXDR(pxUSART), &pxUSART->RxStream);
    }
    return eResult;
}

/**
 * @brief Simultaneously transmits and receives data over USART (full duplex communication).
 * @param pxUSART: pointer to the USART handle structure
 * @param pvTxData: pointer to the transmitted data buffer
 * @param pvRxData: pointer to the received data buffer
 * @param usLength: amount of data transfers
 * @param ulTimeout: available time for successful transfer in ms
 * @return TIMEOUT if transmission or reception times out,
 *         ERROR if a reception error occurs,
 *         OK if transfer is completed
 */
XPD_ReturnType USART_eTransmitReceive(
        USART_HandleType *  pxUSART,
        void *              pvTxData,
        void *              pvRxData,
        uint16_t            usLength,
        uint32_t            ulTimeout)
{
    XPD_ReturnType eResult;

    /* save stream info */
    pxUSART->TxStream.buffer = pvTxData;
    pxUSART->TxStream.length = usLength;
    pxUSART->RxStream.buffer = pvRxData;
    pxUSART->RxStream.length = usLength;
    USART_RESET_ERRORS(pxUSART);

    while ((pxUSART->TxStream.length + pxUSART->RxStream.length) > 0)
    {
        /* wait for buffer change */
        eResult = XPD_eWaitForDiff(&USART_STATR(pxUSART),
                USART_STATF(TXE) | USART_STATF(RXNE), 0, &ulTimeout);
        if (eResult != XPD_OK)
        {
            return eResult;
        }

        /* Time to transmit */
        if ((pxUSART->TxStream.length > 0) && (USART_FLAG_STATUS(pxUSART, TXE)))
        {
            XPD_vWriteFromStream((uint32_t*)&USART_TXDR(pxUSART), &pxUSART->TxStream);
        }

        /* Time to receive */
        if ((pxUSART->RxStream.length > 0) && (USART_FLAG_STATUS(pxUSART, RXNE)))
        {
            XPD_vReadToStream((const uint32_t*)&USART_RXDR(pxUSART), &pxUSART->RxStream);
        }
    }

    return eResult;
}

/**
 * @brief Simultaneously transmits and receives data over USART (full duplex communication).
 * @param pxUSART: pointer to the USART handle structure
 * @param pvTxData: pointer to the transmitted data buffer
 * @param pvRxData: pointer to the received data buffer
 * @param usLength: amount of data transfers
 * @param ulTimeout: available time for successful transfer in ms
 * @return TIMEOUT if transmission or reception times out,
 *         ERROR if a reception error occurs,
 *         OK if transfer is completed
 */
XPD_ReturnType USART_eSendReceive(
        USART_HandleType *  pxUSART,
        void *              pvTxData,
        void *              pvRxData,
        uint16_t            usLength,
        uint32_t            ulTimeout)
{
    XPD_ReturnType eResult = USART_eTransmitReceive(pxUSART,
            pvTxData, pvRxData, usLength, ulTimeout);

    if (eResult == XPD_OK)
    {
        /* wait for the last transmission to finish */
        eResult = XPD_eWaitForDiff(&USART_STATR(pxUSART),
                USART_STATF(TC), 0, &ulTimeout);
    }

    return eResult;
}

/**
 * @brief Starts interrupt-driven data transmission over USART.
 * @param pxUSART: pointer to the USART handle structure
 * @param pvTxData: pointer to the data buffer
 * @param usLength: amount of data transfers
 */
void USART_vTransmit_IT(
        USART_HandleType *  pxUSART,
        void *              pvTxData,
        uint16_t            usLength)
{
    /* save stream info */
    pxUSART->TxStream.buffer = pvTxData;
    pxUSART->TxStream.length = usLength;
    USART_RESET_ERRORS(pxUSART);

    /* Enable TXE interrupt */
    USART_IT_ENABLE(pxUSART, TXE);
}

/**
 * @brief Starts interrupt-driven data transmission over USART.
 * @param pxUSART: pointer to the USART handle structure
 * @param pvTxData: pointer to the data buffer
 * @param usLength: amount of data transfers
 */
void USART_vSend_IT(
        USART_HandleType *  pxUSART,
        void *              pvTxData,
        uint16_t            usLength)
{
    /* save stream info */
    pxUSART->TxStream.buffer = pvTxData;
    pxUSART->TxStream.length = usLength;
    USART_RESET_ERRORS(pxUSART);

    /* Enable TXE and TC interrupts */
    USART_IT_ENABLE(pxUSART, TXE);
    USART_IT_ENABLE(pxUSART, TC);
}

/**
 * @brief Starts interrupt-driven data reception over USART.
 * @note  In synchronous mode the user has to ensure data transmission in order to generate clock.
 * @param pxUSART: pointer to the USART handle structure
 * @param pvTxData: pointer to the data buffer
 * @param usLength: amount of data transfers
 */
void USART_vReceive_IT(
        USART_HandleType *  pxUSART,
        void *              pvRxData,
        uint16_t            usLength)
{
    /* save stream info */
    pxUSART->RxStream.buffer = pvRxData;
    pxUSART->RxStream.length = usLength;
    USART_RESET_ERRORS(pxUSART);

#ifdef __XPD_USART_ERROR_DETECT
    USART_IT_ENABLE(pxUSART, PE);
    USART_IT_ENABLE(pxUSART, E);
#endif
    /* Enable RXNE interrupt */
    USART_IT_ENABLE(pxUSART, RXNE);
}

/**
 * @brief Starts interrupt-driven full-duplex data transfer over USART.
 * @param pxUSART: pointer to the USART handle structure
 * @param pvTxData: pointer to the transmitted data buffer
 * @param pvRxData: pointer to the received data buffer
 * @param usLength: amount of data transfers
 */
void USART_vTransmitReceive_IT(
        USART_HandleType *  pxUSART,
        void *              pvTxData,
        void *              pvRxData,
        uint16_t            usLength)
{
    /* save stream info */
    pxUSART->TxStream.buffer = pvTxData;
    pxUSART->TxStream.length = usLength;
    pxUSART->RxStream.buffer = pvRxData;
    pxUSART->RxStream.length = usLength;
    USART_RESET_ERRORS(pxUSART);

#ifdef __XPD_USART_ERROR_DETECT
    USART_IT_ENABLE(pxUSART, PE);
    USART_IT_ENABLE(pxUSART, E);
#endif
    /* Enable TXE, RXNE interrupt */
    USART_IT_ENABLE(pxUSART, RXNE);
    USART_IT_ENABLE(pxUSART, TXE);
}

/**
 * @brief Starts interrupt-driven full-duplex data transfer over USART.
 * @param pxUSART: pointer to the USART handle structure
 * @param pvTxData: pointer to the transmitted data buffer
 * @param pvRxData: pointer to the received data buffer
 * @param usLength: amount of data transfers
 */
void USART_vSendReceive_IT(
        USART_HandleType *  pxUSART,
        void *              pvTxData,
        void *              pvRxData,
        uint16_t            usLength)
{
    /* save stream info */
    pxUSART->TxStream.buffer = pvTxData;
    pxUSART->TxStream.length = usLength;
    pxUSART->RxStream.buffer = pvRxData;
    pxUSART->RxStream.length = usLength;
    USART_RESET_ERRORS(pxUSART);

#ifdef __XPD_USART_ERROR_DETECT
    USART_IT_ENABLE(pxUSART, PE);
    USART_IT_ENABLE(pxUSART, E);
#endif
    /* Enable TXE, TC, RXNE interrupt */
    USART_IT_ENABLE(pxUSART, RXNE);
    USART_IT_ENABLE(pxUSART, TXE);
    USART_IT_ENABLE(pxUSART, TC);
}

/**
 * @brief USART transfer interrupt handler that provides handle callbacks.
 * @param pxUSART: pointer to the USART handle structure
 */
void USART_vIRQHandler(USART_HandleType * pxUSART)
{
    uint32_t ulSR  = USART_STATR(pxUSART);
    uint32_t ulCR1 = pxUSART->Inst->CR1.w;
    uint32_t ulCR2 = pxUSART->Inst->CR2.w;
    uint32_t ulCR3 = pxUSART->Inst->CR3.w;

#ifdef __XPD_USART_ERROR_DETECT
    /* parity error */
    if (((ulSR & USART_STATF(PE)) != 0) && ((ulCR1 & USART_CR1_PEIE) != 0))
    {
        pxUSART->Errors |= USART_ERROR_PARITY;
        USART_FLAG_CLEAR(pxUSART, PE);
    }
    /* channel errors */
    if ((ulCR3 & USART_CR3_EIE) != 0)
    {
        if ((ulSR & USART_STATF(NE)) != 0)
        {
            pxUSART->Errors |= USART_ERROR_NOISE;
            USART_FLAG_CLEAR(pxUSART, NE);
        }
        if ((ulSR & USART_STATF(FE)) != 0)
        {
            pxUSART->Errors |= USART_ERROR_FRAME;
            USART_FLAG_CLEAR(pxUSART, FE);
        }
        if ((ulSR & USART_STATF(ORE)) != 0)
        {
            pxUSART->Errors |= USART_ERROR_OVERRUN;
            USART_FLAG_CLEAR(pxUSART, ORE);
        }
    }
    if (pxUSART->Errors != USART_ERROR_NONE)
    {
        XPD_SAFE_CALLBACK(pxUSART->Callbacks.Error, pxUSART);
    }
#endif

    /* successful reception */
    if (((ulSR & USART_STATF(RXNE)) != 0) && ((ulCR1 & USART_CR1_RXNEIE) != 0))
    {
        XPD_vReadToStream((const uint32_t*)&USART_RXDR(pxUSART), &pxUSART->RxStream);

        /* End of reception */
        if (pxUSART->RxStream.length == 0)
        {
            USART_IT_DISABLE(pxUSART, RXNE);
#ifdef __XPD_USART_ERROR_DETECT
            USART_IT_DISABLE(pxUSART, PE);
            USART_IT_DISABLE(pxUSART, E);
#endif

            /* reception finished callback */
            XPD_SAFE_CALLBACK(pxUSART->Callbacks.Receive, pxUSART);
        }
    }

    /* successful transmission */
    if (((ulSR & USART_STATF(TXE)) != 0) && ((ulCR1 & USART_CR1_TXEIE) != 0))
    {
        XPD_vWriteFromStream((uint32_t*)&USART_TXDR(pxUSART), &pxUSART->TxStream);

        /* last transmission, disable TXE */
        if (pxUSART->TxStream.length == 0)
        {
            USART_IT_DISABLE(pxUSART, TXE);

            /* callback if transmit completion isn't waited for */
            if ((ulCR1 & USART_CR1_TCIE) != 0)
            {
                XPD_SAFE_CALLBACK(pxUSART->Callbacks.Transmit, pxUSART);
            }
        }
    }

    /* last element in data stream is sent successfully */
    else if (((ulSR & USART_STATF(TC)) != 0) && ((ulCR1 & USART_CR1_TCIE) != 0))
    {
        USART_FLAG_CLEAR(pxUSART, TC);

        /* data is successfully sent */
        if (pxUSART->TxStream.length == 0)
        {
            USART_IT_DISABLE(pxUSART, TC);

            /* transmission finished callback */
            XPD_SAFE_CALLBACK(pxUSART->Callbacks.Transmit, pxUSART);
        }
    }

    /* IDLE detected */
    if (((ulSR & USART_STATF(IDLE)) != 0) && ((ulCR1 & USART_CR1_IDLEIE) != 0))
    {
        USART_FLAG_CLEAR(pxUSART, IDLE);

        XPD_SAFE_CALLBACK(pxUSART->Callbacks.Idle, pxUSART);
    }

    /* LIN break detected */
    if (((ulSR & USART_STATF(LBD)) != 0) && ((ulCR2 & USART_CR2_LBDIE) != 0))
    {
        USART_FLAG_CLEAR(pxUSART, LBD);

        XPD_SAFE_CALLBACK(pxUSART->Callbacks.Break, pxUSART);
    }

    /* CTS detected */
    if (((ulSR & USART_STATF(CTS)) != 0) && ((ulCR3 & USART_CR3_CTSIE) != 0))
    {
        USART_FLAG_CLEAR(pxUSART, CTS);

        XPD_SAFE_CALLBACK(pxUSART->Callbacks.ClearToSend, pxUSART);
    }

#if (__USART_PERIPHERAL_VERSION > 2)
    /* UART wakeup from Stop mode interrupt occurred */
    if(((ulSR & USART_ISR_WUF) != 0) && ((ulCR3 & USART_CR3_WUFIE) != 0))
    {
        USART_FLAG_CLEAR(pxUSART, WU);
    }
#endif
}

/**
 * @brief Starts DMA-managed data transmission over USART.
 * @param pxUSART: pointer to the USART handle structure
 * @param pvTxData: pointer to the data buffer
 * @param usLength: amount of data transfers
 * @return BUSY if DMA is in use, OK if transfer is started
 */
XPD_ReturnType USART_eTransmit_DMA(
        USART_HandleType *  pxUSART,
        void *              pvTxData,
        uint16_t            usLength)
{
    XPD_ReturnType eResult;

    /* save stream info */
    pxUSART->TxStream.buffer = pvTxData;
    pxUSART->TxStream.length = usLength;

    /* Set up DMA for transfer */
    eResult = DMA_eStart_IT(pxUSART->DMA.Transmit,
            (void*)&USART_TXDR(pxUSART), pvTxData, usLength);

    if (eResult == XPD_OK)
    {
        /* Set the callback owner */
        pxUSART->DMA.Transmit->Owner = pxUSART;

        /* Set the DMA transfer callbacks */
        pxUSART->DMA.Transmit->Callbacks.Complete     = USART_prvDmaTransmitRedirect;
#ifdef __XPD_DMA_ERROR_DETECT
        pxUSART->DMA.Transmit->Callbacks.Error        = USART_prvDmaErrorRedirect;
#endif
        USART_RESET_ERRORS(pxUSART);

        USART_FLAG_CLEAR(pxUSART, TC);

        USART_REG_BIT(pxUSART, CR3, DMAT) = 1;
    }
    return eResult;
}

/**
 * @brief Starts DMA-managed data transmission over USART.
 * @param pxUSART: pointer to the USART handle structure
 * @param pvTxData: pointer to the data buffer
 * @param usLength: amount of data transfers
 * @return BUSY if DMA is in use, OK if transfer is started
 */
XPD_ReturnType USART_eSend_DMA(
        USART_HandleType *  pxUSART,
        void *              pvTxData,
        uint16_t            usLength)
{
    XPD_ReturnType eResult = USART_eTransmit_DMA(pxUSART, pvTxData, usLength);

    if (eResult == XPD_OK)
    {
        USART_IT_ENABLE(pxUSART, TC);
    }

    return eResult;
}

/**
 * @brief Starts DMA-managed data reception over USART.
 * @note  In synchronous mode the user has to ensure data transmission in order to generate clock.
 * @param pxUSART: pointer to the USART handle structure
 * @param pvRxData: pointer to the data buffer
 * @param usLength: amount of data transfers
 * @return BUSY if DMA is in use, OK if transfer is started
 */
XPD_ReturnType USART_eReceive_DMA(
        USART_HandleType *  pxUSART,
        void *              pvRxData,
        uint16_t            usLength)
{
    XPD_ReturnType eResult;

    /* save stream info */
    pxUSART->RxStream.buffer = pvRxData;
    pxUSART->RxStream.length = usLength;

    /* Set up DMA for transfer */
    eResult = DMA_eStart_IT(pxUSART->DMA.Receive,
            (void*)&USART_RXDR(pxUSART), pvRxData, usLength);

    if (eResult == XPD_OK)
    {
        /* Set the callback owner */
        pxUSART->DMA.Receive->Owner = pxUSART;

        /* Set the DMA transfer callbacks */
        pxUSART->DMA.Receive->Callbacks.Complete     = USART_prvDmaReceiveRedirect;
#ifdef __XPD_DMA_ERROR_DETECT
        pxUSART->DMA.Receive->Callbacks.Error        = USART_prvDmaErrorRedirect;
#endif
        USART_RESET_ERRORS(pxUSART);

        USART_FLAG_CLEAR(pxUSART, ORE);

        USART_REG_BIT(pxUSART, CR3, DMAR) = 1;
    }
    return eResult;
}

/**
 * @brief Starts DMA-managed full-duplex data transfer over USART.
 * @param pxUSART: pointer to the USART handle structure
 * @param pvTxData: pointer to the transmitted data buffer
 * @param pvRxData: pointer to the received data buffer
 * @param usLength: amount of data transfers
 * @return BUSY if DMA is in use, OK if transfer is started
 */
XPD_ReturnType USART_eTransmitReceive_DMA(
        USART_HandleType *  pxUSART,
        void *              pvTxData,
        void *              pvRxData,
        uint16_t            usLength)
{
    XPD_ReturnType eResult;

    /* save stream info */
    pxUSART->TxStream.buffer = pvTxData;
    pxUSART->TxStream.length = usLength;
    pxUSART->RxStream.buffer = pvRxData;
    pxUSART->RxStream.length = usLength;

    /* Set up DMAs for transfers */
    eResult = DMA_eStart_IT(pxUSART->DMA.Receive,
            (void*)&USART_RXDR(pxUSART), pvRxData, usLength);

    if (eResult == XPD_OK)
    {
        eResult = DMA_eStart_IT(pxUSART->DMA.Transmit,
                (void*)&USART_TXDR(pxUSART), pvTxData, usLength);

        /* If one DMA allocation failed, reset the other and exit */
        if (eResult != XPD_OK)
        {
            DMA_vStop_IT(pxUSART->DMA.Receive);
            return eResult;
        }

        /* Set the callback owner */
        pxUSART->DMA.Transmit->Owner = pxUSART;
        pxUSART->DMA.Receive->Owner  = pxUSART;

        /* Set the DMA transfer callbacks */
        pxUSART->DMA.Transmit->Callbacks.Complete     = USART_prvDmaTransmitRedirect;
        pxUSART->DMA.Receive->Callbacks.Complete      = USART_prvDmaReceiveRedirect;
#ifdef __XPD_DMA_ERROR_DETECT
        pxUSART->DMA.Transmit->Callbacks.Error        = USART_prvDmaErrorRedirect;
        pxUSART->DMA.Receive->Callbacks.Error         = USART_prvDmaErrorRedirect;
#endif
        USART_RESET_ERRORS(pxUSART);

        USART_FLAG_CLEAR(pxUSART, ORE);
        USART_FLAG_CLEAR(pxUSART, TC);

        USART_REG_BIT(pxUSART, CR3, DMAR) = 1;
        USART_REG_BIT(pxUSART, CR3, DMAT) = 1;
    }
    return eResult;
}

/**
 * @brief Starts DMA-managed full-duplex data transfer over USART.
 * @param pxUSART: pointer to the USART handle structure
 * @param pvTxData: pointer to the transmitted data buffer
 * @param pvRxData: pointer to the received data buffer
 * @param usLength: amount of data transfers
 * @return BUSY if DMA is in use, OK if transfer is started
 */
XPD_ReturnType USART_eSendReceive_DMA(
        USART_HandleType *  pxUSART,
        void *              pvTxData,
        void *              pvRxData,
        uint16_t            usLength)
{
    XPD_ReturnType eResult = USART_eTransmitReceive_DMA(pxUSART, pvTxData, pvRxData, usLength);

    if (eResult == XPD_OK)
    {
        USART_IT_ENABLE(pxUSART, TC);
    }

    return eResult;
}

/**
 * @brief Stops all ongoing DMA-managed transfers over USART.
 * @param pxUSART: pointer to the USART handle structure
 */
void USART_vStop_DMA(USART_HandleType * pxUSART)
{
    /* Transmit DMA disable */
    if (USART_REG_BIT(pxUSART,CR3,DMAT) != 0)
    {
        uint16_t remaining;
        USART_REG_BIT(pxUSART,CR3,DMAT) = 0;

        /* Read remaining transfer count */
        remaining = DMA_usGetStatus(pxUSART->DMA.Transmit);

        /* Update transfer context */
        pxUSART->TxStream.buffer += (pxUSART->TxStream.length - remaining)
                * pxUSART->TxStream.size;
        pxUSART->TxStream.length = remaining;

        DMA_vStop_IT(pxUSART->DMA.Transmit);
    }
    /* Receive DMA disable */
    if (USART_REG_BIT(pxUSART,CR3,DMAR) != 0)
    {
        uint16_t remaining;
        USART_REG_BIT(pxUSART,CR3,DMAR) = 0;

        /* Read remaining transfer count */
        remaining = DMA_usGetStatus(pxUSART->DMA.Receive);

        /* Update transfer context */
        pxUSART->RxStream.buffer += (pxUSART->RxStream.length - remaining)
                * pxUSART->RxStream.size;
        pxUSART->RxStream.length = remaining;

        DMA_vStop_IT(pxUSART->DMA.Receive);
    }
}

#ifdef USART_CR3_OVRDIS
/**
 * @brief Sets the overrun detection configuration for the USART (enabled by default)
 * @param pxUSART: pointer to the USART handle structure
 */
void USART_vOverrunEnable(USART_HandleType * pxUSART)
{
    uint32_t ulCR1 = pxUSART->Inst->CR1.w;
    uint32_t ulUE = ulCR1 & USART_CR1_UE;

    if (ulUE != 0)
    {
        pxUSART->Inst->CR1.w = ulCR1 & ~USART_CR1_UE;
    }
    USART_REG_BIT(pxUSART,CR3,OVRDIS) = 0;
    pxUSART->Inst->CR1.w |= ulUE;
}

/**
 * @brief Sets the overrun detection configuration for the USART (enabled by default)
 * @param pxUSART: pointer to the USART handle structure
 */
void USART_vOverrunDisable(USART_HandleType * pxUSART)
{
    uint32_t ulCR1 = pxUSART->Inst->CR1.w;
    uint32_t ulUE = ulCR1 & USART_CR1_UE;

    if (ulUE != 0)
    {
        pxUSART->Inst->CR1.w = ulCR1 & ~USART_CR1_UE;
    }
    USART_REG_BIT(pxUSART,CR3,OVRDIS) = 1;
    pxUSART->Inst->CR1.w |= ulUE;
}
#endif /* USART_CR3_OVRDIS */

/** @} */

/** @} */

/** @addtogroup UART
 * @{ */

/** @defgroup UART_Exported_Functions UART Exported Functions
 * @{ */

/**
 * @brief Initializes the USART peripheral in asynchronous mode using the setup configuration
 * @param pxUSART: pointer to the USART handle structure
 * @param pxConfig: UART setup configuration
 */
void USART_vInitAsync(USART_HandleType * pxUSART, const UART_InitType * pxConfig)
{
    USART_prvPreinit(pxUSART, pxConfig->DataSize, pxConfig->Parity);

    pxUSART->Inst->CR2.b.STOP           = pxConfig->StopBits;
    USART_REG_BIT(pxUSART, CR3, HDSEL)  = pxConfig->HalfDuplex;
#ifdef IS_LPUART_INSTANCE
    if (!IS_LPUART_INSTANCE(pxUSART->Inst))
    {
        USART_REG_BIT(pxUSART, CR1, OVER8)  = pxConfig->OverSampling8;
        USART_REG_BIT(pxUSART, CR3, ONEBIT) = pxConfig->SingleSample;
    }
#endif

    /* configure hardware flow control */
    SET_BIT(pxUSART->Inst->CR3.w, (USART_CR3_RTSE | USART_CR3_CTSE) &
            ((uint32_t)pxConfig->FlowControl << USART_CR3_RTSE_Pos));

#if (USART_INVERSION_MASK != 0)
    /* set inversions */
    SET_BIT(pxUSART->Inst->CR2.w, USART_INVERSION_MASK &
            ((uint32_t)pxConfig->Inversions << USART_CR2_SWAP_Pos));
#endif

#if (USART_BAUDRATEMODE_MASK != 0)
    /* set the baudrate detection strategy for the UART */
    SET_BIT(pxUSART->Inst->CR2.w, USART_BAUDRATEMODE_MASK &
            ((uint32_t)pxConfig->BaudrateMode << USART_CR2_ABREN_Pos));
#endif

    /* baudrate configuration */
    USART_prvSetBaudrate(pxUSART, pxConfig->Baudrate);

    /* enable USART with the selected directions */
    SET_BIT(pxUSART->Inst->CR1.w, USART_CR1_UE |
            ((USART_CR1_TE | USART_CR1_RE) & pxConfig->Directions));

#ifdef IS_UART_WAKEUP_FROMSTOP_INSTANCE
    if (IS_UART_WAKEUP_FROMSTOP_INSTANCE(pxUSART->Inst))
    {
        USART_prvWaitIdle(pxUSART);
    }
#endif
}

/** @} */

/** @} */

/** @addtogroup USRT
 * @{ */

/** @defgroup USRT_Exported_Functions USRT Exported Functions
 * @{ */

/**
 * @brief Initializes the USART peripheral in synchronous mode using the setup configuration
 * @param pxUSART: pointer to the USART handle structure
 * @param pxConfig: USRT setup configuration
 */
void USART_vInitSync(USART_HandleType * pxUSART, const USRT_InitType * pxConfig)
{
    USART_prvPreinit(pxUSART, pxConfig->DataSize, pxConfig->Parity);

    /* configure clock settings and stop mode */
    USART_REG_BIT(pxUSART, CR1, OVER8)  = ENABLE;
    USART_REG_BIT(pxUSART, CR2, CLKEN)  = ENABLE;
    USART_REG_BIT(pxUSART, CR2, CPOL)   = pxConfig->Clock.Polarity;
    USART_REG_BIT(pxUSART, CR2, CPHA)   = pxConfig->Clock.Phase;
    USART_REG_BIT(pxUSART, CR2, LBCL)   = pxConfig->Clock.LastBit;
    pxUSART->Inst->CR2.b.STOP           = pxConfig->StopBits;
    USART_REG_BIT(pxUSART, CR3, ONEBIT) = pxConfig->SingleSample;

#if (USART_INVERSION_MASK != 0)
    /* set inversions */
    SET_BIT(pxUSART->Inst->CR2.w, USART_INVERSION_MASK &
            ((uint32_t)pxConfig->Inversions << USART_CR2_SWAP_Pos));
#endif

    /* baudrate configuration */
    USART_prvSetBaudrate(pxUSART, pxConfig->Baudrate);

    /* enable USART with both directions */
    SET_BIT(pxUSART->Inst->CR1.w, USART_CR1_UE |
            USART_CR1_TE | USART_CR1_RE);

#ifdef IS_UART_WAKEUP_FROMSTOP_INSTANCE
    if (IS_UART_WAKEUP_FROMSTOP_INSTANCE(pxUSART->Inst))
    {
        USART_prvWaitIdle(pxUSART);
    }
#endif
}

/** @} */

/** @} */

/** @addtogroup LIN
 * @{ */

/** @defgroup LIN_Exported_Functions LIN Exported Functions
 * @{ */

/**
 * @brief Sets the UART peripheral in LIN protocol mode
 * @param pxUSART: pointer to the USART handle structure
 * @param pxConfig: LIN setup configuration
 */
void USART_vInitLIN(USART_HandleType * pxUSART, const LIN_InitType * pxConfig)
{
    USART_prvPreinit(pxUSART, 8, USART_PARITY_NONE);

    /* Set LIN mode, break length */
    pxUSART->Inst->CR2.w = USART_CR2_LINEN | (pxConfig->BreakSize > 10) ? USART_CR2_LBDL : 0;

#if (USART_INVERSION_MASK != 0)
    /* set inversions */
    SET_BIT(pxUSART->Inst->CR2.w, USART_INVERSION_MASK &
            ((uint32_t)pxConfig->Inversions << USART_CR2_SWAP_Pos));
#endif

#if (USART_BAUDRATEMODE_MASK != 0)
    /* set the baudrate detection strategy for the UART */
    SET_BIT(pxUSART->Inst->CR2.w, USART_BAUDRATEMODE_MASK &
            ((uint32_t)pxConfig->BaudrateMode << USART_CR2_ABREN_Pos));
#endif

    /* baudrate configuration */
    USART_prvSetBaudrate(pxUSART, pxConfig->Baudrate);

    /* enable USART with both directions */
    SET_BIT(pxUSART->Inst->CR1.w, USART_CR1_UE |
            USART_CR1_TE | USART_CR1_RE);

#ifdef IS_UART_WAKEUP_FROMSTOP_INSTANCE
    if (IS_UART_WAKEUP_FROMSTOP_INSTANCE(pxUSART->Inst))
    {
        USART_prvWaitIdle(pxUSART);
    }
#endif
}

/**
 * @brief Sends a BREAK frame
 * @param pxUSART: pointer to the USART handle structure
 */
void USART_vSendBreak(USART_HandleType * pxUSART)
{
#if (__USART_PERIPHERAL_VERSION > 1)
    pxUSART->Inst->RQR.w = USART_RQR_SBKRQ;
#else
    USART_REG_BIT(pxUSART, CR1, SBK) = 1;
#endif
}

/** @} */

/** @} */

/** @addtogroup MSUART
 * @{ */

/** @defgroup MSUART_Exported_Functions MultiSlave UART Exported Functions
 * @{ */

/**
 * @brief Sets the UART peripheral in MultiProcessor slave mode
 * @param pxUSART: pointer to the USART handle structure
 * @param pxConfig: MultiSlave UART setup configuration
 */
void USART_InitMultiSlave(USART_HandleType * pxUSART, const MSUART_InitType * pxConfig)
{
    USART_prvPreinit(pxUSART, pxConfig->DataSize, pxConfig->Parity);

    pxUSART->Inst->CR2.b.STOP           = pxConfig->StopBits;
    USART_REG_BIT(pxUSART, CR1, OVER8)  = pxConfig->OverSampling8;
    USART_REG_BIT(pxUSART, CR3, ONEBIT) = pxConfig->SingleSample;
    USART_REG_BIT(pxUSART, CR3, HDSEL)  = pxConfig->HalfDuplex;

    /* configure multislave settings */
    USART_REG_BIT(pxUSART, CR1, WAKE)  = pxConfig->UnmuteMethod;
#if (__USART_PERIPHERAL_VERSION > 2)
    pxUSART->Inst->CR3.b.WUS           = pxConfig->WakeUpMethod;
#endif

    if ((pxConfig->UnmuteMethod == MSUART_UNMUTE_ADDRESSED)
#if (__USART_PERIPHERAL_VERSION > 2)
     || (pxConfig->WakeUpMethod == MSUART_WAKEUP_ADDRESSED)
#endif
     )
    {
        pxUSART->Inst->CR2.b.ADD           = pxConfig->Address;
#ifdef USART_CR2_ADDM7
        USART_REG_BIT(pxUSART, CR2, ADDM7) = (pxConfig->AddressLength == 7) ? 1 : 0;
#endif
    }

#if (USART_INVERSION_MASK != 0)
    /* set inversions */
    SET_BIT(pxUSART->Inst->CR2.w, USART_INVERSION_MASK &
            ((uint32_t)pxConfig->Inversions << USART_CR2_SWAP_Pos));
#endif

#if (USART_BAUDRATEMODE_MASK != 0)
    /* set the baudrate detection strategy for the UART */
    SET_BIT(pxUSART->Inst->CR2.w, USART_BAUDRATEMODE_MASK &
            ((uint32_t)pxConfig->BaudrateMode << USART_CR2_ABREN_Pos));
#endif

    /* baudrate configuration */
    USART_prvSetBaudrate(pxUSART, pxConfig->Baudrate);

    /* enable USART with the selected directions */
    SET_BIT(pxUSART->Inst->CR1.w, USART_CR1_UE |
            ((USART_CR1_TE | USART_CR1_RE) & pxConfig->Directions));

#ifdef IS_UART_WAKEUP_FROMSTOP_INSTANCE
    if (IS_UART_WAKEUP_FROMSTOP_INSTANCE(pxUSART->Inst))
    {
        USART_prvWaitIdle(pxUSART);
    }
#endif
}

/** @} */

/** @} */

/** @addtogroup SmartCard
 * @{ */

/** @defgroup SmartCard_Exported_Functions SmartCard Exported Functions
 * @{ */

/**
 * @brief Sets the UART peripheral in SmartCard mode
 * @param pxUSART: pointer to the USART handle structure
 * @param pxConfig: SmartCard setup configuration
 */
void USART_vInitSmartCard(USART_HandleType * pxUSART, const SmartCard_InitType * pxConfig)
{
    USART_prvPreinit(pxUSART, 8, USART_PARITY_EVEN | pxConfig->Parity);

    /* Set Smartcard features, output clock */
    USART_REG_BIT(pxUSART, CR3, SCEN)   = ENABLE;
    pxUSART->Inst->CR2.b.STOP           = USART_STOPBITS_1p5;
    USART_REG_BIT(pxUSART, CR3, NACK)   = pxConfig->NACK;
    pxUSART->Inst->GTPR.b.GT            = pxConfig->GuardTime;
    USART_REG_BIT(pxUSART, CR3, ONEBIT) = pxConfig->SingleSample;

    /* Optional clock */
    USART_REG_BIT(pxUSART, CR2, CLKEN)  = ENABLE;
    USART_REG_BIT(pxUSART, CR2, CPOL)   = pxConfig->Clock.Polarity;
    USART_REG_BIT(pxUSART, CR2, CPHA)   = pxConfig->Clock.Phase;
    pxUSART->Inst->GTPR.b.PSC           = pxConfig->Clock.Divider / 2;

#if (__USART_PERIPHERAL_VERSION > 1)
    pxUSART->Inst->CR3.b.SCARCNT        = pxConfig->AutoRetries;
    pxUSART->Inst->RTOR.b.BLEN          = pxConfig->BlockLength;

    if (pxConfig->RxTimeout > 0)
    {
        USART_REG_BIT(pxUSART, CR2, RTOEN) = ENABLE;
        pxUSART->Inst->RTOR.b.RTO          = pxConfig->RxTimeout;
    }
    else
    {
        USART_REG_BIT(pxUSART, CR2, RTOEN) = DISABLE;
    }
#endif

#if (USART_INVERSION_MASK != 0)
    /* set inversions */
    SET_BIT(pxUSART->Inst->CR2.w, USART_INVERSION_MASK &
            ((uint32_t)pxConfig->Inversions << USART_CR2_SWAP_Pos));
#endif

    /* baudrate configuration */
    USART_prvSetBaudrate(pxUSART, pxConfig->Baudrate);

    /* enable USART with transmit direction by default */
    SET_BIT(pxUSART->Inst->CR1.w, USART_CR1_UE |
            USART_CR1_TE);

#ifdef IS_UART_WAKEUP_FROMSTOP_INSTANCE
    if (IS_UART_WAKEUP_FROMSTOP_INSTANCE(pxUSART->Inst))
    {
        USART_prvWaitIdle(pxUSART);
    }
#endif
}

/** @} */

/** @} */

/** @addtogroup IrDA
 * @{ */

/** @defgroup IrDA_Exported_Functions IrDA Exported Functions
 * @{ */

/**
 * @brief Sets the UART peripheral in IrDA mode
 * @param pxUSART: pointer to the USART handle structure
 * @param pxConfig: IrDA setup configuration
 */
void USART_vInitIrDA(USART_HandleType * pxUSART, const IrDA_InitType * pxConfig)
{
    USART_prvPreinit(pxUSART, pxConfig->DataSize, pxConfig->Parity);

    /* IrDA features configuration */
    pxUSART->Inst->GTPR.b.PSC           = pxConfig->Prescaler;
    pxUSART->Inst->CR2.b.STOP           = 0;
    USART_REG_BIT(pxUSART, CR3, IRLP)   = pxConfig->LowPowerMode;
    USART_REG_BIT(pxUSART, CR3, IREN)   = ENABLE;

#if (USART_INVERSION_MASK != 0)
    /* set inversions */
    SET_BIT(pxUSART->Inst->CR2.w, USART_INVERSION_MASK &
            ((uint32_t)pxConfig->Inversions << USART_CR2_SWAP_Pos));
#endif

    /* baudrate configuration */
    USART_prvSetBaudrate(pxUSART, pxConfig->Baudrate);

    /* enable USART with transmit direction by default */
    SET_BIT(pxUSART->Inst->CR1.w, USART_CR1_UE |
            USART_CR1_TE);

#ifdef IS_UART_WAKEUP_FROMSTOP_INSTANCE
    if (IS_UART_WAKEUP_FROMSTOP_INSTANCE(pxUSART->Inst))
    {
        USART_prvWaitIdle(pxUSART);
    }
#endif
}

/** @} */

/** @} */

#ifdef USART_CR3_DEM
/** @addtogroup RS485
 * @{ */

/** @defgroup RS485_Exported_Functions RS485 Exported Functions
 * @{ */

/**
 * @brief Sets the UART peripheral in RS485 mode
 * @param pxUSART: pointer to the USART handle structure
 * @param pxConfig: RS485 setup configuration
 */
void USART_vInitRS485(USART_HandleType * pxUSART, const RS485_InitType * pxConfig)
{
    USART_prvPreinit(pxUSART, pxConfig->DataSize, pxConfig->Parity);

    pxUSART->Inst->CR2.b.STOP           = pxConfig->StopBits;
    USART_REG_BIT(pxUSART, CR3, HDSEL)  = pxConfig->HalfDuplex;
    USART_REG_BIT(pxUSART, CR1, OVER8)  = pxConfig->OverSampling8;
    USART_REG_BIT(pxUSART, CR3, ONEBIT) = pxConfig->SingleSample;

    /* configure DE signal */
    USART_REG_BIT(pxUSART, CR3, DEM)    = ENABLE;
    USART_REG_BIT(pxUSART, CR3, DEP)    = pxConfig->DE.Polarity;
    pxUSART->Inst->CR1.b.DEAT           = pxConfig->DE.AssertionTime;
    pxUSART->Inst->CR1.b.DEDT           = pxConfig->DE.DeassertionTime;

#if (USART_INVERSION_MASK != 0)
    /* set inversions */
    SET_BIT(pxUSART->Inst->CR2.w, USART_INVERSION_MASK &
            ((uint32_t)pxConfig->Inversions << USART_CR2_SWAP_Pos));
#endif

#if (USART_BAUDRATEMODE_MASK != 0)
    /* set the baudrate detection strategy for the UART */
    SET_BIT(pxUSART->Inst->CR2.w, USART_BAUDRATEMODE_MASK &
            ((uint32_t)pxConfig->BaudrateMode << USART_CR2_ABREN_Pos));
#endif

    /* baudrate configuration */
    USART_prvSetBaudrate(pxUSART, pxConfig->Baudrate);

    /* enable USART with the selected directions */
    SET_BIT(pxUSART->Inst->CR1.w, USART_CR1_UE |
            ((USART_CR1_TE | USART_CR1_RE) & pxConfig->Directions));

#ifdef IS_UART_WAKEUP_FROMSTOP_INSTANCE
    if (IS_UART_WAKEUP_FROMSTOP_INSTANCE(pxUSART->Inst))
    {
        USART_prvWaitIdle(pxUSART);
    }
#endif
}

/** @} */

/** @} */
#endif

/** @} */
