/**
  ******************************************************************************
  * @file    xpd_spi.c
  * @author  Benedek Kupper
  * @version 0.2
  * @date    2018-01-28
  * @brief   STM32 eXtensible Peripheral Drivers SPI Module
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
#include <xpd_spi.h>
#include <xpd_utils.h>

/** @addtogroup SPI
 * @{ */

#define SPI_BUSY_TIMEOUT    1000

#define SPI_MASTER_RXONLY(HANDLE) \
    (((HANDLE)->Inst->CR1.w & (SPI_CR1_MSTR | SPI_CR1_BIDIMODE | SPI_CR1_RXONLY)) > SPI_CR1_MSTR)

#define SPI_MASTER_FULL_DUPLEX(HANDLE) \
    (((HANDLE)->Inst->CR1.w & (SPI_CR1_MSTR | SPI_CR1_BIDIMODE | SPI_CR1_RXONLY)) == SPI_CR1_MSTR)

#ifndef SPI_CR2_DS_Pos
#define SPI_CR2_DS_Pos 8
#endif

#define SPI_DATASIZE_16(HANDLE) \
    (((HANDLE)->Inst->CR2.w & SPI_CR2_DS_3) >> (SPI_CR2_DS_Pos + 3))

#if defined(__XPD_SPI_ERROR_DETECT) || defined(__XPD_DMA_ERROR_DETECT)
#define SPI_RESET_ERRORS(HANDLE)    ((HANDLE)->Errors = SPI_ERROR_NONE)
#else
#define SPI_RESET_ERRORS(HANDLE)    ((void)0)
#endif

#define SPI_REG_BY_SIZE(REG, SIZE) \
    ((SIZE == 1) ? *((__IO uint8_t *)REG) : *((__IO uint16_t *)REG))

#ifdef __XPD_SPI_ERROR_DETECT
/* Reinitialize CRC calculation by OFF-ON switch */
__STATIC_INLINE void SPI_prvInitCRC(SPI_HandleType * pxSPI)
{
    SPI_REG_BIT(pxSPI, CR1, CRCEN) = 0;
    SPI_REG_BIT(pxSPI, CR1, CRCEN) = 1;
}

/* Receive CRC bytes on SPI and check for error */
static XPD_ReturnType SPI_prvReceiveCRC(SPI_HandleType * pxSPI, uint32_t * ulTimeout)
{
    XPD_ReturnType eResult;

#ifdef SPI_SR_FRLVL
    /* Set FIFO threshold according to CRC size */
    SPI_REG_BIT(pxSPI, CR2, FRXTH) = 2 - pxSPI->CRCSize;
#endif

    /* wait for not empty receive buffer */
    eResult = XPD_eWaitForDiff(&pxSPI->Inst->SR.w, SPI_SR_RXNE, 0, ulTimeout);

    if (eResult == XPD_OK)
    {
        /* read CRC data from data register */
        (void) SPI_REG_BY_SIZE(&pxSPI->Inst->DR, pxSPI->RxStream.size);
    }

#ifdef SPI_SR_FRLVL
    /* Reset Rx FIFO threshold */
    SPI_REG_BIT(pxSPI, CR2, FRXTH) = 2 - pxSPI->RxStream.size;
#endif

    /* Check if CRC error occurred */
    if (SPI_FLAG_STATUS(pxSPI, CRCERR) != 0)
    {
        pxSPI->Errors |= SPI_ERROR_CRC;
        SPI_FLAG_CLEAR(pxSPI, CRCERR);

        eResult = XPD_ERROR;
    }
    return eResult;
}
#endif

static void SPI_prvDmaTransmitRedirect(void * pxDMA)
{
    SPI_HandleType * pxSPI = (SPI_HandleType*) ((DMA_HandleType*) pxDMA)->Owner;

    if (DMA_eCircularMode((DMA_HandleType*)pxDMA) == 0)
    {
        /* Disable Tx DMA Request */
        SPI_REG_BIT(pxSPI, CR2, TXDMAEN) = 0;

        /* Clear overrun flag in 2 Lines communication mode because received data is not read */
        if (SPI_REG_BIT(pxSPI, CR1, BIDIMODE) == 0)
        {
            /* Nothing to receive */
            if (pxSPI->RxStream.length == 0)
            {
                /* Empty previously received data from data register  */
                while (SPI_FLAG_STATUS(pxSPI, RXNE) != 0)
                {
                    (void) SPI_REG_BY_SIZE(&pxSPI->Inst->DR, pxSPI->RxStream.size);
                }
            }
            SPI_FLAG_CLEAR(pxSPI, OVR);
        }

        /* Update stream status */
        pxSPI->TxStream.buffer += pxSPI->TxStream.length * pxSPI->TxStream.size;
        pxSPI->TxStream.length = 0;
    }

    XPD_SAFE_CALLBACK(pxSPI->Callbacks.Transmit, pxSPI);
}

static void SPI_prvDmaReceiveRedirect(void * pxDMA)
{
    SPI_HandleType * pxSPI = (SPI_HandleType*) ((DMA_HandleType*) pxDMA)->Owner;

    if (DMA_eCircularMode((DMA_HandleType*)pxDMA) == 0)
    {
        /* Disable Rx DMA Request */
        SPI_REG_BIT(pxSPI, CR2, RXDMAEN) = 0;

        /* Update stream status */
        pxSPI->RxStream.buffer += pxSPI->RxStream.length * pxSPI->RxStream.size;
        pxSPI->RxStream.length = 0;

#ifdef __XPD_SPI_ERROR_DETECT
        /* CRC handling */
        if (pxSPI->CRCSize > 0)
        {
            /* CRC reception is processed by SPI interrupt */
#ifdef SPI_SR_FRLVL
            /* Set FIFO threshold according to CRC size */
            SPI_REG_BIT(pxSPI, CR2, FRXTH) = 2 - pxSPI->CRCSize;
#endif
            /* Enable RXNE and ERR interrupt */
            SET_BIT(pxSPI->Inst->CR2.w, SPI_CR2_RXNEIE | SPI_CR2_ERRIE);

            /* callback is provided in interrupt handler in CRC mode */
            return;
        }
#endif
    }
    XPD_SAFE_CALLBACK(pxSPI->Callbacks.Receive, pxSPI);
}

static void SPI_prvDmaTransmitReceiveRedirect(void * pxDMA)
{
    SPI_HandleType * pxSPI = (SPI_HandleType*) ((DMA_HandleType*) pxDMA)->Owner;

    if (DMA_eCircularMode((DMA_HandleType*)pxDMA) == 0)
    {
        /* Disable DMA Requests */
        CLEAR_BIT(pxSPI->Inst->CR2.w, SPI_CR2_RXDMAEN | SPI_CR2_TXDMAEN);

        /* Update stream status */
        pxSPI->RxStream.buffer += pxSPI->RxStream.length * pxSPI->RxStream.size;
        pxSPI->RxStream.length = 0;
        pxSPI->TxStream.buffer += pxSPI->TxStream.length * pxSPI->TxStream.size;
        pxSPI->TxStream.length = 0;

#ifdef __XPD_SPI_ERROR_DETECT
        /* CRC handling */
        if (pxSPI->CRCSize > 0)
        {
            /* CRC reception is processed by SPI interrupt */
#ifdef SPI_SR_FRLVL
            /* Set FIFO threshold according to CRC size */
            SPI_REG_BIT(pxSPI, CR2, FRXTH) = 2 - pxSPI->CRCSize;
#endif
            /* Enable RXNE and ERR interrupt */
            SET_BIT(pxSPI->Inst->CR2.w, SPI_CR2_RXNEIE | SPI_CR2_ERRIE);

            /* receive callback is provided in interrupt handler in CRC mode */
            XPD_SAFE_CALLBACK(pxSPI->Callbacks.Transmit, pxSPI);
            return;
        }
#endif
    }
    XPD_SAFE_CALLBACK(pxSPI->Callbacks.Transmit, pxSPI);
    XPD_SAFE_CALLBACK(pxSPI->Callbacks.Receive, pxSPI);
}

#ifdef __XPD_DMA_ERROR_DETECT
static void SPI_prvDmaErrorRedirect(void * pxDMA)
{
    SPI_HandleType * pxSPI = (SPI_HandleType*) ((DMA_HandleType*) pxDMA)->Owner;

    pxSPI->Errors |= SPI_ERROR_DMA;

    XPD_SAFE_CALLBACK(pxSPI->Callbacks.Error, pxSPI);
}
#endif

/**
 * @brief Enables the SPI peripheral.
 * @param pxSPI: pointer to the SPI handle structure
 */
void SPI_prvEnable(SPI_HandleType * pxSPI)
{
    SPI_REG_BIT(pxSPI, CR1, SPE) = 1;
}

/**
 * @brief Disables the SPI peripheral.
 * @param pxSPI: pointer to the SPI handle structure
 */
void SPI_prvDisable(SPI_HandleType * pxSPI)
{
    SPI_REG_BIT(pxSPI, CR1, SPE) = 0;
}

/** @defgroup SPI_Exported_Functions SPI Exported Functions
 * @{ */

/**
 * @brief Initializes the SPI peripheral using the setup configuration.
 * @param pxSPI: pointer to the SPI handle structure
 * @param pxConfig: SPI setup configuration
 */
void SPI_vInit(SPI_HandleType * pxSPI, const SPI_InitType * pxConfig)
{
    /* enable clock */
    RCC_vClockEnable(pxSPI->CtrlPos);

    SPI_prvDisable(pxSPI);

    /* Set bits related to Channel and NSS behavior */
    MODIFY_REG(pxSPI->Inst->CR1.w, SPI_CR1_BIDIMODE | SPI_CR1_RXONLY | SPI_CR1_SSM, pxConfig->Channel | pxConfig->NSS);

    /* Set both bits according to master mode */
    SPI_REG_BIT(pxSPI, CR1, MSTR)     = pxConfig->Mode;
    SPI_REG_BIT(pxSPI, CR1, SSI)      = pxConfig->Mode;

    SPI_REG_BIT(pxSPI, CR1, CPOL)     = pxConfig->Clock.Polarity;
    SPI_REG_BIT(pxSPI, CR1, CPHA)     = pxConfig->Clock.Phase;
    SPI_REG_BIT(pxSPI, CR1, LSBFIRST) = pxConfig->Format;

    pxSPI->Inst->CR1.b.BR = pxConfig->Clock.Prescaler - 1;

#ifdef __XPD_SPI_ERROR_DETECT
    /* Disable CRC by setting 0 length */
    if (pxConfig->CRC_Length == 0)
    {
#ifdef SPI_CR1_CRCL
        SPI_REG_BIT(pxSPI, CR1, CRCL) = 0;
#endif
        SPI_REG_BIT(pxSPI, CR1, CRCEN) = 0;
        pxSPI->CRCSize = 0;
    }
    else
    {
#ifdef SPI_CR1_CRCL
        /* Set 8 bit CRC when not 16 bit is requested, and data fits in 1 byte */
        if ((pxConfig->CRC_Length < 16) && (pxConfig->DataSize <= 8))
        {
            pxSPI->CRCSize = 1;
        }
        /* Set 16 bit CRC when requested, or when data size requires it */
        else
        {
            pxSPI->CRCSize = 2;
        }
        SPI_REG_BIT(pxSPI, CR1, CRCL) = pxSPI->CRCSize - 1;
#else
        /* CRC size equals data size */
        pxSPI->CRCSize = (pxConfig->DataSize > 8) ? 2 : 1;
#endif
        /* Set up CRC configuration */
        SPI_REG_BIT(pxSPI, CR1, CRCEN) = 1;
        pxSPI->Inst->CRCPR = pxConfig->CRC_Polynomial;
    }
#endif

#ifdef SPI_SR_FRLVL
    /* Rx FIFO threshold is set to quarter full when data size fits in 1 byte */
    SPI_REG_BIT(pxSPI, CR2, FRXTH) = (uint32_t)(pxConfig->DataSize <= 8);
#endif
    SPI_REG_BIT(pxSPI, CR2, FRF)   = pxConfig->TI_Mode;

#if defined(SPI_CR2_DS)
    /* Configure data bit size */
    pxSPI->Inst->CR2.b.DS = pxConfig->DataSize - 1;
#elif defined(SPI_CR1_DFF)
    /* Configure data frame format */
    SPI_REG_BIT(pxSPI, CR1, DFF)   = (uint32_t)(pxConfig->DataSize > 8);
#endif

#ifdef SPI_I2SCFGR_I2SMOD
    /* Activate the SPI mode (Make sure that I2SMOD bit in I2SCFGR register is reset) */
    SPI_REG_BIT(pxSPI, I2SCFGR, I2SMOD) = 0;
#endif

    /* Initialize handle variables */
    pxSPI->TxStream.length = pxSPI->RxStream.length = 0;
    pxSPI->TxStream.size   = pxSPI->RxStream.size   = (pxConfig->DataSize <= 8) ? 1 : 2;

    /* Dependencies initialization */
    XPD_SAFE_CALLBACK(pxSPI->Callbacks.DepInit, pxSPI);
}

/**
 * @brief Restores the SPI peripheral to its default inactive state.
 * @param pxSPI: pointer to the SPI handle structure
 */
void SPI_vDeinit(SPI_HandleType * pxSPI)
{
    SPI_prvDisable(pxSPI);

    /* Deinitialize peripheral dependencies */
    XPD_SAFE_CALLBACK(pxSPI->Callbacks.DepDeinit, pxSPI);

    /* Disable clock */
    RCC_vClockDisable(pxSPI->CtrlPos);
}

/**
 * @brief Determines the current status of SPI peripheral.
 * @param pxSPI: pointer to the SPI handle structure
 * @return BUSY if a transfer is in progress, OK if SPI is ready for new transfer
 */
XPD_ReturnType SPI_eGetStatus(SPI_HandleType * pxSPI)
{
    return (SPI_FLAG_STATUS(pxSPI, BSY) != 0) ? XPD_BUSY : XPD_OK;
}

/**
 * @brief Polls the status of the SPI transfer.
 * @param pxSPI: pointer to the SPI handle structure
 * @param ulTimeout: the timeout in ms for the polling.
 * @return ERROR if there were transfer errors, TIMEOUT if timed out, OK if successful
 */
XPD_ReturnType SPI_ePollStatus(SPI_HandleType * pxSPI, uint32_t ulTimeout)
{
#ifdef __XPD_SPI_ERROR_DETECT
    /* wait until not busy, or until an error is present */
    XPD_ReturnType eResult = XPD_eWaitForDiff(&pxSPI->Inst->SR.w,
            SPI_SR_BSY | SPI_SR_OVR | SPI_SR_MODF | SPI_SR_CRCERR | SPI_SR_FRE,
            SPI_SR_BSY, &ulTimeout);

    /* Error checks */
    if (SPI_FLAG_STATUS(pxSPI, OVR))
    {
        /* Update error code */
        pxSPI->Errors |= SPI_ERROR_OVERRUN;

        /* Clear the transfer error flag */
        SPI_FLAG_CLEAR(pxSPI, OVR);

        eResult = XPD_ERROR;
    }
    if (SPI_FLAG_STATUS(pxSPI, MODF))
    {
        /* Update error code */
        pxSPI->Errors |= SPI_ERROR_MODE;

        /* Clear the transfer error flag */
        SPI_FLAG_CLEAR(pxSPI, MODF);

        eResult = XPD_ERROR;
    }
    if (SPI_FLAG_STATUS(pxSPI, CRCERR))
    {
        /* Update error code */
        pxSPI->Errors |= SPI_ERROR_CRC;

        /* Clear the transfer error flag */
        SPI_FLAG_CLEAR(pxSPI, CRCERR);

        eResult = XPD_ERROR;
    }
    if (SPI_FLAG_STATUS(pxSPI, FRE))
    {
        /* Update error code */
        pxSPI->Errors |= SPI_ERROR_FRAME;

        /* Clear the transfer error flag */
        SPI_FLAG_CLEAR(pxSPI, FRE);

        eResult = XPD_ERROR;
    }
#else
    XPD_ReturnType eResult = XPD_eWaitForMatch(&pxSPI->Inst->SR.w, SPI_SR_BSY, 0, &ulTimeout);
#endif

    return eResult;
}

/**
 * @brief Transmits data over SPI.
 * @param pxSPI: pointer to the SPI handle structure
 * @param pvTxData: pointer to the data buffer
 * @param usLength: amount of data transfers
 * @param ulTimeout: available time for successful transmission in ms
 * @return TIMEOUT if transmission or reception times out,
 *         OK if transfer is completed
 */
XPD_ReturnType SPI_eSend(
        SPI_HandleType *    pxSPI,
        void *              pvTxData,
        uint16_t            usLength,
        uint32_t            ulTimeout)
{
    XPD_ReturnType eResult;
    bool eDuplex = true;

    /* save stream info */
    pxSPI->TxStream.buffer = pvTxData;
    pxSPI->TxStream.length = usLength;
    SPI_RESET_ERRORS(pxSPI);

    /* Configure communication direction : 1Line */
    if (SPI_REG_BIT(pxSPI, CR1, BIDIMODE) != 0)
    {
        SPI_REG_BIT(pxSPI, CR1, BIDIOE) = 1;
        eDuplex = false;
    }

#ifdef __XPD_SPI_ERROR_DETECT
    /* Reset CRC Calculation */
    if (pxSPI->CRCSize > 0)
    {
        SPI_prvInitCRC(pxSPI);
    }
#endif

    /* Check if the SPI is already enabled */
    SPI_prvEnable(pxSPI);

    while (pxSPI->TxStream.length > 0)
    {
        /* wait for empty transmit buffer */
        eResult = XPD_eWaitForDiff(&pxSPI->Inst->SR.w, SPI_SR_TXE, 0, &ulTimeout);
        if (eResult != XPD_OK)
        {
            return eResult;
        }

        XPD_vWriteFromStream((uint32_t*)&pxSPI->Inst->DR, &pxSPI->TxStream);
    }
    /* wait for the last transmission to finish */
    eResult = XPD_eWaitForDiff(&pxSPI->Inst->SR.w, SPI_SR_TXE, 0, &ulTimeout);

#ifdef __XPD_SPI_ERROR_DETECT
    /* Enable CRC Transmission */
    if (pxSPI->CRCSize > 0)
    {
        SPI_REG_BIT(pxSPI, CR1, CRCNEXT) = 1;
    }
#endif

    /* Control the BSY flag */
    eResult = XPD_eWaitForMatch(&pxSPI->Inst->SR.w, SPI_SR_BSY, 0, &ulTimeout);

    /* Clear overrun flag in 2 Lines communication mode because received data is not read */
    if (eDuplex)
    {
        /* Empty previously received data from data register  */
        while (SPI_FLAG_STATUS(pxSPI, RXNE) != 0)
        {
            (void) SPI_REG_BY_SIZE(&pxSPI->Inst->DR, pxSPI->RxStream.size);
        }
        SPI_FLAG_CLEAR(pxSPI, OVR);
    }

    return eResult;
}

/**
 * @brief Receives data over SPI.
 * @param pxSPI: pointer to the SPI handle structure
 * @param pvRxData: pointer to the data buffer
 * @param usLength: amount of data transfers
 * @param ulTimeout: available time for successful transmission in ms
 * @return TIMEOUT if transmission or reception times out,
 *         ERROR if a reception error occurs,
 *         OK if transfer is completed
 */
XPD_ReturnType SPI_eReceive(
        SPI_HandleType *    pxSPI,
        void *              pvRxData,
        uint16_t            usLength,
        uint32_t            ulTimeout)
{
    XPD_ReturnType eResult;

    /* If master mode, and full duplex communication */
    if (SPI_MASTER_FULL_DUPLEX(pxSPI))
    {
        /* the receive process is not supported in 2Lines direction master mode */
        /* in this case we call the TransmitReceive process                     */
        return SPI_eSendReceive(pxSPI, pvRxData, pvRxData, usLength, ulTimeout);
    }

    /* save stream info */
    pxSPI->RxStream.buffer = pvRxData;
    pxSPI->RxStream.length = usLength;
    SPI_RESET_ERRORS(pxSPI);

#ifdef __XPD_SPI_ERROR_DETECT
    /* Reset CRC Calculation */
    if (pxSPI->CRCSize > 0)
    {
        SPI_prvInitCRC(pxSPI);
    }
#endif

    /* Configure communication direction 1Line and enabled SPI if needed */
    if (SPI_REG_BIT(pxSPI, CR1, BIDIMODE) != 0)
    {
        SPI_REG_BIT(pxSPI, CR1, BIDIOE) = 0;
    }

    /* Check if the SPI is already enabled */
    SPI_prvEnable(pxSPI);

    while (pxSPI->RxStream.length > 0)
    {
        /* Wait for not empty receive buffer */
        eResult = XPD_eWaitForDiff(&pxSPI->Inst->SR.w, SPI_SR_RXNE, 0, &ulTimeout);
        if (eResult != XPD_OK)
        {
            return eResult;
        }

        XPD_vReadToStream((const uint32_t*)&pxSPI->Inst->DR, &pxSPI->RxStream);

#ifdef __XPD_SPI_ERROR_DETECT
        /* This is done to handle the CRCNEXT before the last data */
        if ((pxSPI->CRCSize > 0) && (pxSPI->RxStream.length == 1))
        {
            SPI_REG_BIT(pxSPI, CR1, CRCNEXT) = 1;
        }
#endif
    }

#ifdef __XPD_SPI_ERROR_DETECT
    /* Handle the CRC Reception */
    if (pxSPI->CRCSize > 0)
    {
        eResult = SPI_prvReceiveCRC(pxSPI, &ulTimeout);
    }
#endif

    /* If master mode, and either simplex, or half duplex communication */
    if (SPI_MASTER_RXONLY(pxSPI))
    {
        /* Disable SPI peripheral */
        SPI_prvDisable(pxSPI);
    }
    return eResult;
}

/**
 * @brief Simultaneously transmits and receives data over SPI (full duplex communication).
 * @param pxSPI: pointer to the SPI handle structure
 * @param pvTxData: pointer to the transmitted data buffer
 * @param pvRxData: pointer to the received data buffer
 * @param usLength: amount of data transfers
 * @param ulTimeout: available time for successful transfer in ms
 * @return TIMEOUT if transmission or reception times out,
 *         ERROR if a reception error occurs,
 *         OK if transfer is completed
 */
XPD_ReturnType SPI_eSendReceive(
        SPI_HandleType *    pxSPI,
        void *              pvTxData,
        void *              pvRxData,
        uint16_t            usLength,
        uint32_t            ulTimeout)
{
    XPD_ReturnType eResult;

    /* save stream info */
    pxSPI->TxStream.buffer = pvTxData;
    pxSPI->TxStream.length = usLength;
    pxSPI->RxStream.buffer = pvRxData;
    pxSPI->RxStream.length = usLength;
    SPI_RESET_ERRORS(pxSPI);

#ifdef __XPD_SPI_ERROR_DETECT
    /* Reset CRC Calculation */
    if (pxSPI->CRCSize > 0)
    {
        SPI_prvInitCRC(pxSPI);
    }
#endif

    /* Check if the SPI is already enabled */
    SPI_prvEnable(pxSPI);

    while ((pxSPI->TxStream.length + pxSPI->RxStream.length) > 0)
    {
        /* wait for buffer change */
        eResult = XPD_eWaitForDiff(&pxSPI->Inst->SR.w, SPI_SR_TXE | SPI_SR_RXNE, 0, &ulTimeout);
        if (eResult != XPD_OK)
        {
            return eResult;
        }

        /* Time to transmit */
        if ((pxSPI->TxStream.length > 0) && (SPI_FLAG_STATUS(pxSPI, TXE)))
        {
            XPD_vWriteFromStream((uint32_t*)&pxSPI->Inst->DR, &pxSPI->TxStream);

#ifdef __XPD_SPI_ERROR_DETECT
            /* Enable CRC Transmission */
            if ((pxSPI->CRCSize > 0) && (pxSPI->TxStream.length == 0))
            {
                SPI_REG_BIT(pxSPI, CR1, CRCNEXT) = 1;
            }
#endif
        }

        /* Time to receive */
        if ((pxSPI->RxStream.length > 0) && (SPI_FLAG_STATUS(pxSPI, RXNE)))
        {
            XPD_vReadToStream((const uint32_t*)&pxSPI->Inst->DR, &pxSPI->RxStream);
        }
    }

#ifdef __XPD_SPI_ERROR_DETECT
    /* Handle the CRC Reception */
    if (pxSPI->CRCSize > 0)
    {
        eResult = SPI_prvReceiveCRC(pxSPI, &ulTimeout);
    }
#endif
    return eResult;
}

/**
 * @brief Starts interrupt-driven data transmission over SPI.
 * @note  The Transmit callback will be called when the last data is written to buffer,
 *        the user has to wait for the end of that transfer by SPI_ePollStatus() before
 *        switching ChipSelect or starting a new reception.
 * @param pxSPI: pointer to the SPI handle structure
 * @param pvTxData: pointer to the data buffer
 * @param usLength: amount of data transfers
 */
void SPI_vTransmit_IT(
        SPI_HandleType *    pxSPI,
        void *              pvTxData,
        uint16_t            usLength)
{
    /* save stream info */
    pxSPI->TxStream.buffer = pvTxData;
    pxSPI->TxStream.length = usLength;
    SPI_RESET_ERRORS(pxSPI);

    /* Configure communication direction : 1Line */
    if (SPI_REG_BIT(pxSPI, CR1, BIDIMODE) != 0)
    {
        SPI_REG_BIT(pxSPI, CR1, BIDIOE) = 1;
    }

#ifdef __XPD_SPI_ERROR_DETECT
    /* Reset CRC Calculation */
    if (pxSPI->CRCSize > 0)
    {
        SPI_prvInitCRC(pxSPI);
    }
#endif
    /* Enable TXE and ERR interrupt */
    SPI_IT_ENABLE(pxSPI, TXE);

    /* Check if the SPI is already enabled */
    SPI_prvEnable(pxSPI);
}

/**
 * @brief Starts interrupt-driven data reception over SPI.
 * @param pxSPI: pointer to the SPI handle structure
 * @param pvRxData: pointer to the data buffer
 * @param usLength: amount of data transfers
 */
void SPI_vReceive_IT(
        SPI_HandleType *    pxSPI,
        void *              pvRxData,
        uint16_t            usLength)
{
    /* If master mode, and full duplex communication */
    if (SPI_MASTER_FULL_DUPLEX(pxSPI))
    {
        /* the receive process is not supported in 2Lines direction master mode */
        /* in this case we call the TransmitReceive process                     */
        SPI_vTransmitReceive_IT(pxSPI, pvRxData, pvRxData, usLength);
    }

    /* save stream info */
    pxSPI->RxStream.buffer = pvRxData;
    pxSPI->RxStream.length = usLength;
    SPI_RESET_ERRORS(pxSPI);

    /* Configure communication direction 1Line and enabled SPI if needed */
    if (SPI_REG_BIT(pxSPI, CR1, BIDIMODE) != 0)
    {
        SPI_REG_BIT(pxSPI, CR1, BIDIOE) = 0;
    }

#ifdef __XPD_SPI_ERROR_DETECT
    /* Reset CRC Calculation */
    if (pxSPI->CRCSize > 0)
    {
        SPI_prvInitCRC(pxSPI);
    }
    /* Enable RXNE and ERR interrupt */
    SET_BIT(pxSPI->Inst->CR2.w, SPI_CR2_RXNEIE | SPI_CR2_ERRIE);
#else
    /* Enable RXNE interrupt */
    SPI_IT_ENABLE(pxSPI, RXNE);
#endif

    /* Check if the SPI is already enabled */
    SPI_prvEnable(pxSPI);
}

/**
 * @brief Starts interrupt-driven full-duplex data transfer over SPI.
 * @note  Only the Receive callback is called at the end of the communication,
 *        the Transmit callback will be called when the last data is written to buffer.
 * @param pxSPI: pointer to the SPI handle structure
 * @param pvTxData: pointer to the transmitted data buffer
 * @param pvRxData: pointer to the received data buffer
 * @param usLength: amount of data transfers
 */
void SPI_vTransmitReceive_IT(
        SPI_HandleType *    pxSPI,
        void *              pvTxData,
        void *              pvRxData,
        uint16_t            usLength)
{
    /* save stream info */
    pxSPI->TxStream.buffer = pvTxData;
    pxSPI->TxStream.length = usLength;
    pxSPI->RxStream.buffer = pvRxData;
    pxSPI->RxStream.length = usLength;
    SPI_RESET_ERRORS(pxSPI);

#ifdef __XPD_SPI_ERROR_DETECT
    /* Reset CRC Calculation */
    if (pxSPI->CRCSize > 0)
    {
        SPI_prvInitCRC(pxSPI);
    }
    /* Enable TXE, RXNE and ERR interrupt */
    SET_BIT(pxSPI->Inst->CR2.w, SPI_CR2_TXEIE | SPI_CR2_RXNEIE | SPI_CR2_ERRIE);
#else
    /* Enable TXE, RXNE interrupt */
    SET_BIT(pxSPI->Inst->CR2.w, SPI_CR2_TXEIE | SPI_CR2_RXNEIE);
#endif

    /* Check if the SPI is already enabled */
    SPI_prvEnable(pxSPI);
}

/**
 * @brief SPI transfer interrupt handler that provides handle callbacks.
 * @param pxSPI: pointer to the SPI handle structure
 */
void SPI_vIRQHandler(SPI_HandleType * pxSPI)
{
    uint32_t ulCR2 = pxSPI->Inst->CR2.w;
    uint32_t ulSR = pxSPI->Inst->SR.w;

    /* Successful reception */
    if (((ulSR & (SPI_SR_RXNE | SPI_SR_OVR)) == SPI_SR_RXNE) && ((ulCR2 & SPI_CR2_RXNEIE) != 0))
    {
#ifdef __XPD_SPI_ERROR_DETECT
        /* If CRC is used */
        if (pxSPI->CRCSize > 0)
        {
            /* Avoid reading CRC to buffer */
            if (pxSPI->RxStream.length > 0)
            {
                /* Read data from FIFO */
                XPD_vReadToStream((const uint32_t*)&pxSPI->Inst->DR, &pxSPI->RxStream);

                /* This is done to handle the CRCNEXT before the last data */
                if (pxSPI->RxStream.length == 1)
                {
                    SPI_REG_BIT(pxSPI, CR1, CRCNEXT) = 1;
                }

#ifdef SPI_SR_FRLVL
                /* End of data reception */
                else if (pxSPI->RxStream.length == 0)
                {
                    /* Set FIFO threshold according to CRC size */
                    SPI_REG_BIT(pxSPI, CR2, FRXTH) = 2 - pxSPI->CRCSize;
                }
#endif
            }
            else
            {
                /* read CRC data from data register */
                (void) SPI_REG_BY_SIZE(&pxSPI->Inst->DR, pxSPI->RxStream.size);
#ifdef SPI_SR_FRLVL
                /* Reset Rx FIFO threshold */
                SPI_REG_BIT(pxSPI, CR2, FRXTH) = 2 - pxSPI->RxStream.size;
#endif

                /* Check if CRC error occurred */
                if ((ulSR & SPI_SR_CRCERR) != 0)
                {
                    pxSPI->Errors |= SPI_ERROR_CRC;
                    SPI_FLAG_CLEAR(pxSPI, CRCERR);

                    /* error callback */
                    XPD_SAFE_CALLBACK(pxSPI->Callbacks.Error, pxSPI);
                }

                /* If master mode, and either simplex, or half duplex communication */
                if (SPI_MASTER_RXONLY(pxSPI))
                {
                    /* Disable SPI peripheral */
                    SPI_prvDisable(pxSPI);
                }

                /* Disable RXNE and ERR interrupt */
                CLEAR_BIT(pxSPI->Inst->CR2.w, SPI_CR2_RXNEIE | SPI_CR2_ERRIE);

                /* reception finished callback */
                XPD_SAFE_CALLBACK(pxSPI->Callbacks.Receive, pxSPI);
            }
        }
        else
#endif
        {
            XPD_vReadToStream((const uint32_t*)&pxSPI->Inst->DR, &pxSPI->RxStream);

            /* End of reception */
            if (pxSPI->RxStream.length == 0)
            {
                /* If master mode, and either simplex, or half duplex communication */
                if (SPI_MASTER_RXONLY(pxSPI))
                {
                    /* Disable SPI peripheral */
                    SPI_prvDisable(pxSPI);
                }

                /* Disable RXNE and ERR interrupt */
                CLEAR_BIT(pxSPI->Inst->CR2.w, SPI_CR2_RXNEIE | SPI_CR2_ERRIE);

                /* reception finished callback */
                XPD_SAFE_CALLBACK(pxSPI->Callbacks.Receive, pxSPI);
            }
        }
    }

    /* Successful transmission */
    if (((ulSR & SPI_SR_TXE) != 0) && ((ulCR2 & SPI_CR2_TXEIE) != 0))
    {
        XPD_vWriteFromStream((uint32_t*)&pxSPI->Inst->DR, &pxSPI->TxStream);

        if (pxSPI->TxStream.length == 0)
        {
#ifdef __XPD_SPI_ERROR_DETECT
            /* Enable CRC Transmission */
            if (pxSPI->CRCSize > 0)
            {
                SPI_REG_BIT(pxSPI, CR1, CRCNEXT) = 1;
            }
#endif
            SPI_IT_DISABLE(pxSPI, TXE);

            /* Clear overrun flag in 2 Lines communication mode because received is not read */
            if (SPI_REG_BIT(pxSPI, CR1, BIDIMODE) == 0)
            {
                /* Nothing to receive */
                if (pxSPI->RxStream.length == 0)
                {
                    /* Empty previously received data from data register  */
                    while (SPI_FLAG_STATUS(pxSPI, RXNE) != 0)
                    {
                        (void) SPI_REG_BY_SIZE(&pxSPI->Inst->DR, pxSPI->RxStream.size);
                    }
                }
                SPI_FLAG_CLEAR(pxSPI, OVR);
            }

            XPD_SAFE_CALLBACK(pxSPI->Callbacks.Transmit, pxSPI);
        }
    }

#ifdef __XPD_SPI_ERROR_DETECT
    /* Transfer error occurred */
    if (((ulSR & (SPI_SR_MODF | SPI_SR_OVR | SPI_SR_FRE)) != 0) && ((ulCR2 & SPI_CR2_ERRIE) != 0))
    {
        if ((ulSR & SPI_SR_OVR) != 0)
        {
            pxSPI->Errors |= SPI_ERROR_OVERRUN;
            SPI_FLAG_CLEAR(pxSPI, OVR);
        }
        if ((ulSR & SPI_SR_MODF) != 0)
        {
            pxSPI->Errors |= SPI_ERROR_MODE;
            SPI_FLAG_CLEAR(pxSPI, MODF);
        }
        if ((ulSR & SPI_SR_FRE) != 0)
        {
            pxSPI->Errors |= SPI_ERROR_FRAME;
            SPI_FLAG_CLEAR(pxSPI, FRE);
        }
        /* Clear interrupt enable bits */
        CLEAR_BIT(pxSPI->Inst->CR2.w, SPI_CR2_TXEIE | SPI_CR2_RXNEIE | SPI_CR2_ERRIE);

        XPD_SAFE_CALLBACK(pxSPI->Callbacks.Error, pxSPI);
    }
#endif
}

/**
 * @brief Starts DMA-managed data transmission over SPI.
 * @note  The Transmit callback will be called when the last data is written to buffer,
 *        the user has to wait for the end of that transfer by SPI_ePollStatus() before
 *        switching ChipSelect or starting a new reception.
 * @param pxSPI: pointer to the SPI handle structure
 * @param pvTxData: pointer to the data buffer
 * @param usLength: amount of data transfers
 * @return BUSY if DMA is in use, OK if transfer is started
 */
XPD_ReturnType SPI_eTransmit_DMA(
        SPI_HandleType *    pxSPI,
        void *              pvTxData,
        uint16_t            usLength)
{
    XPD_ReturnType eResult;

    /* save stream info */
    pxSPI->TxStream.buffer = pvTxData;
    pxSPI->TxStream.length = usLength;

    /* Set up DMA for transfer */
    eResult = DMA_eStart_IT(pxSPI->DMA.Transmit,
            (void*)&pxSPI->Inst->DR, pvTxData, usLength);

    if (eResult == XPD_OK)
    {
        /* Set the callback owner */
        pxSPI->DMA.Transmit->Owner = pxSPI;

        /* Set the DMA transfer callbacks */
        pxSPI->DMA.Transmit->Callbacks.Complete     = SPI_prvDmaTransmitRedirect;
#ifdef __XPD_DMA_ERROR_DETECT
        pxSPI->DMA.Transmit->Callbacks.Error        = SPI_prvDmaErrorRedirect;
#endif
        SPI_RESET_ERRORS(pxSPI);

#ifdef __XPD_SPI_ERROR_DETECT
        /* Reset CRC Calculation */
        if (pxSPI->CRCSize > 0)
        {
            SPI_prvInitCRC(pxSPI);
        }
#endif

        /* Configure communication direction : 1Line */
        if (SPI_REG_BIT(pxSPI, CR1, BIDIMODE) != 0)
        {
            SPI_REG_BIT(pxSPI, CR1, BIDIOE) = 1;
        }

        /* Enable Tx DMA Request */
        SPI_REG_BIT(pxSPI, CR2, TXDMAEN) = 1;

        /* Check if the SPI is already enabled */
        SPI_prvEnable(pxSPI);
    }
    return eResult;
}

/**
 * @brief Starts DMA-managed data reception over SPI.
 * @param pxSPI: pointer to the SPI handle structure
 * @param pvRxData: pointer to the data buffer
 * @param usLength: amount of data transfers
 * @return BUSY if DMA is in use, OK if transfer is started
 */
XPD_ReturnType SPI_eReceive_DMA(
        SPI_HandleType *    pxSPI,
        void *              pvRxData,
        uint16_t            usLength)
{
    XPD_ReturnType eResult;

    /* If master mode, and full duplex communication */
    if (SPI_MASTER_FULL_DUPLEX(pxSPI))
    {
        /* the receive process is not supported in 2Lines direction master mode
         * in this case we call the TransmitReceive process */
        eResult = SPI_eSendReceive_DMA(pxSPI, NULL, pvRxData, usLength);
    }
    else
    {
        /* save stream info */
        pxSPI->RxStream.buffer = pvRxData;
        pxSPI->RxStream.length = usLength;

        /* Set up DMA for transfer */
        eResult = DMA_eStart_IT(pxSPI->DMA.Receive,
                (void*)&pxSPI->Inst->DR, pvRxData, usLength);

        if (eResult == XPD_OK)
        {
            /* Set the callback owner */
            pxSPI->DMA.Receive->Owner = pxSPI;

            /* Set the DMA transfer callbacks */
            pxSPI->DMA.Receive->Callbacks.Complete     = SPI_prvDmaReceiveRedirect;
#ifdef __XPD_DMA_ERROR_DETECT
            pxSPI->DMA.Receive->Callbacks.Error        = SPI_prvDmaErrorRedirect;
#endif
            SPI_RESET_ERRORS(pxSPI);

#ifdef __XPD_SPI_ERROR_DETECT
            /* Reset CRC Calculation */
            if (pxSPI->CRCSize > 0)
            {
                SPI_prvInitCRC(pxSPI);
            }
#endif

            /* Configure communication direction : 1Line */
            if (SPI_REG_BIT(pxSPI, CR1, BIDIMODE) != 0)
            {
                SPI_REG_BIT(pxSPI, CR1, BIDIOE) = 0;
            }

            /* Enable Rx DMA Request */
            SPI_REG_BIT(pxSPI, CR2, RXDMAEN) = 1;

            /* Check if the SPI is already enabled */
            SPI_prvEnable(pxSPI);
        }
    }
    return eResult;
}

/**
 * @brief Starts DMA-managed full-duplex data transfer over SPI.
 * @param pxSPI: pointer to the SPI handle structure
 * @param pvTxData: pointer to the transmitted data buffer
 * @param pvRxData: pointer to the received data buffer
 * @param usLength: amount of data transfers
 * @return BUSY if DMA is in use, OK if transfer is started
 */
XPD_ReturnType SPI_eSendReceive_DMA(
        SPI_HandleType *    pxSPI,
        void *              pvTxData,
        void *              pvRxData,
        uint16_t            usLength)
{
    XPD_ReturnType eResult;

    /* save stream info */
    pxSPI->TxStream.buffer = pvTxData;
    pxSPI->TxStream.length = usLength;
    pxSPI->RxStream.buffer = pvRxData;
    pxSPI->RxStream.length = usLength;

    /* In case there is no actual data transmission, send dummy from receive buffer */
    if (pvTxData == NULL)
    {
        pvTxData = pvRxData;
    }

    /* Set up DMAs for transfers */
    eResult = DMA_eStart_IT(pxSPI->DMA.Receive,
            (void*)&pxSPI->Inst->DR, pvRxData, usLength);

    if (eResult == XPD_OK)
    {
#ifdef __XPD_DMA_ERROR_DETECT
        eResult = DMA_eStart_IT(pxSPI->DMA.Transmit, (void*) &pxSPI->Inst->DR, pvTxData, usLength);
#else
        eResult = DMA_eStart(pxSPI->DMA.Transmit,
                (void*)&pxSPI->Inst->DR, pvTxData, usLength);
#endif

        /* If one DMA allocation failed, reset the other and exit */
        if (eResult != XPD_OK)
        {
            DMA_vStop_IT(pxSPI->DMA.Receive);
            return eResult;
        }

        /* Set the callback owner */
        pxSPI->DMA.Receive->Owner = pxSPI;

        /* Set the DMA transfer callbacks */
        if (pxSPI->TxStream.buffer == NULL)
        {
            pxSPI->DMA.Receive->Callbacks.Complete  = SPI_prvDmaReceiveRedirect;
        }
        else
        {
            pxSPI->DMA.Receive->Callbacks.Complete  = SPI_prvDmaTransmitReceiveRedirect;
        }
#ifdef __XPD_DMA_ERROR_DETECT
        /* Set the DMA error callback */
        pxSPI->DMA.Receive->Callbacks.Error         = SPI_prvDmaErrorRedirect;
#endif

        /* Set the callback owner */
        pxSPI->DMA.Transmit->Owner = pxSPI;

#ifdef __XPD_DMA_ERROR_DETECT
        /* Set the SPI Tx DMA transfer complete callback as NULL because the communication closing
        is performed in DMA reception complete callback  */
        pxSPI->DMA.Transmit->Callbacks.Complete     = NULL;
        pxSPI->DMA.Transmit->Callbacks.Error        = SPI_prvDmaErrorRedirect;
#endif
        SPI_RESET_ERRORS(pxSPI);

#ifdef __XPD_SPI_ERROR_DETECT
        /* Reset CRC Calculation */
        if (pxSPI->CRCSize > 0)
        {
            SPI_prvInitCRC(pxSPI);
        }
#endif

        /* Enable DMA Requests */
        SET_BIT(pxSPI->Inst->CR2.w, SPI_CR2_RXDMAEN | SPI_CR2_TXDMAEN);

        /* Check if the SPI is already enabled */
        SPI_prvEnable(pxSPI);
    }
    return eResult;
}

/**
 * @brief Stops all ongoing DMA-managed transfers over SPI.
 * @param pxSPI: pointer to the SPI handle structure
 */
void SPI_vStop_DMA(SPI_HandleType * pxSPI)
{
    /* Transmit DMA disable */
    if (SPI_REG_BIT(pxSPI, CR2, TXDMAEN) != 0)
    {
        uint16_t usRemaining;
        SPI_REG_BIT(pxSPI, CR2, TXDMAEN) = 0;

        /* Read remaining transfer count */
        usRemaining = DMA_usGetStatus(pxSPI->DMA.Transmit);

        /* Update transfer context */
        pxSPI->TxStream.buffer += (pxSPI->TxStream.length - usRemaining)
                * pxSPI->TxStream.size;
        pxSPI->TxStream.length = usRemaining;

        DMA_vStop_IT(pxSPI->DMA.Transmit);
    }
    /* Receive DMA disable */
    if (SPI_REG_BIT(pxSPI, CR2, RXDMAEN) != 0)
    {
        uint16_t usRemaining;
        SPI_REG_BIT(pxSPI, CR2, RXDMAEN) = 0;

        /* Read remaining transfer count */
        usRemaining = DMA_usGetStatus(pxSPI->DMA.Receive);

        /* Update transfer context */
        pxSPI->RxStream.buffer += (pxSPI->RxStream.length - usRemaining)
                * pxSPI->RxStream.size;
        pxSPI->RxStream.length = usRemaining;

        DMA_vStop_IT(pxSPI->DMA.Receive);
    }
}

/** @} */

/** @} */
