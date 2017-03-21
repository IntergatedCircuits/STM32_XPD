/**
  ******************************************************************************
  * @file    xpd_spi.c
  * @author  Benedek Kupper
  * @version V0.1
  * @date    2017-01-04
  * @brief   STM32 eXtensible Peripheral Drivers SPI Module
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
#include "xpd_spi.h"
#include "xpd_utils.h"

#if defined(USE_XPD_SPI)

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

#if defined(USE_XPD_SPI_ERROR_DETECT) || defined(USE_XPD_DMA_ERROR_DETECT)
#define SPI_RESET_ERRORS(HANDLE)    ((HANDLE)->Errors = SPI_ERROR_NONE)
#else
#define SPI_RESET_ERRORS(HANDLE)    ((void)0)
#endif

#define SPI_REG_BY_SIZE(REG, SIZE) \
    ((SIZE == 1) ? *((__IO uint8_t *)REG) : *((__IO uint16_t *)REG))

#ifdef USE_XPD_SPI_ERROR_DETECT
/* Reinitialize CRC calculation by OFF-ON switch */
__STATIC_INLINE void spi_initCRC(SPI_HandleType * hspi)
{
    SPI_REG_BIT(hspi, CR1, CRCEN) = 0;
    SPI_REG_BIT(hspi, CR1, CRCEN) = 1;
}

/* Receive CRC bytes on SPI and check for error */
static XPD_ReturnType spi_receiveCRC(SPI_HandleType * hspi, uint32_t * timeout)
{
    XPD_ReturnType result;

#ifdef SPI_SR_FRLVL
    /* Set FIFO threshold according to CRC size */
    SPI_REG_BIT(hspi, CR2, FRXTH) = 2 - hspi->CRCSize;
#endif

    /* wait for not empty receive buffer */
    result = XPD_WaitForDiff(&hspi->Inst->SR.w, SPI_SR_RXNE, 0, timeout);

    if (result == XPD_OK)
    {
        /* read CRC data from data register */
        uint16_t temp = SPI_REG_BY_SIZE(&hspi->Inst->DR, hspi->RxStream.size);
    }

#ifdef SPI_SR_FRLVL
    /* Reset Rx FIFO threshold */
    SPI_REG_BIT(hspi, CR2, FRXTH) = 2 - hspi->RxStream.size;
#endif

    /* Check if CRC error occurred */
    if (XPD_SPI_GetFlag(hspi, CRCERR) != 0)
    {
        hspi->Errors |= SPI_ERROR_CRC;
        XPD_SPI_ClearFlag(hspi, CRCERR);

        result = XPD_ERROR;
    }
    return result;
}
#endif

static void spi_dmaTransmitRedirect(void * hdma)
{
    SPI_HandleType * hspi = (SPI_HandleType*) ((DMA_HandleType*) hdma)->Owner;

    if (XPD_DMA_CircularMode((DMA_HandleType*)hdma) == 0)
    {
        /* Disable Tx DMA Request */
        SPI_REG_BIT(hspi, CR2, TXDMAEN) = 0;

        /* Clear overrun flag in 2 Lines communication mode because received data is not read */
        if (SPI_REG_BIT(hspi, CR1, BIDIMODE) == 0)
        {
            while (XPD_SPI_GetFlag(hspi, RXNE) != 0)
            {
                /* empty received data from data register */
                uint16_t temp = SPI_REG_BY_SIZE(&hspi->Inst->DR, hspi->RxStream.size);
            }
            XPD_SPI_ClearFlag(hspi, OVR);
        }

        /* Update stream status */
        hspi->TxStream.buffer += hspi->TxStream.length * hspi->TxStream.size;
        hspi->TxStream.length = 0;
    }

    XPD_SAFE_CALLBACK(hspi->Callbacks.Transmit, hspi);
}

static void spi_dmaReceiveRedirect(void * hdma)
{
    SPI_HandleType * hspi = (SPI_HandleType*) ((DMA_HandleType*) hdma)->Owner;

    if (XPD_DMA_CircularMode((DMA_HandleType*)hdma) == 0)
    {
        /* Disable DMA Requests */
        CLEAR_BIT(hspi->Inst->CR2.w, SPI_CR2_RXDMAEN | SPI_CR2_TXDMAEN);

        /* Update stream status */
        hspi->RxStream.buffer += hspi->RxStream.length * hspi->RxStream.size;
        hspi->RxStream.length = 0;

#ifdef USE_XPD_SPI_ERROR_DETECT
        /* CRC handling */
        if (hspi->CRCSize > 0)
        {
            /* CRC reception is processed by SPI interrupt */
#ifdef SPI_SR_FRLVL
            /* Set FIFO threshold according to CRC size */
            SPI_REG_BIT(hspi, CR2, FRXTH) = 2 - hspi->CRCSize;
#endif
            /* Enable RXNE and ERR interrupt */
            SET_BIT(hspi->Inst->CR2.w, SPI_CR2_RXNEIE | SPI_CR2_ERRIE);

            /* callback is provided in interrupt handler in CRC mode */
            return;
        }
#endif
    }
    XPD_SAFE_CALLBACK(hspi->Callbacks.Receive, hspi);
}

static void spi_dmaTransmitReceiveRedirect(void * hdma)
{
    SPI_HandleType * hspi = (SPI_HandleType*) ((DMA_HandleType*) hdma)->Owner;

    if (XPD_DMA_CircularMode((DMA_HandleType*)hdma) == 0)
    {
        /* Disable DMA Requests */
        CLEAR_BIT(hspi->Inst->CR2.w, SPI_CR2_RXDMAEN | SPI_CR2_TXDMAEN);

        /* Update stream status */
        hspi->RxStream.buffer += hspi->RxStream.length * hspi->RxStream.size;
        hspi->RxStream.length = 0;
        hspi->TxStream.buffer += hspi->TxStream.length * hspi->TxStream.size;
        hspi->TxStream.length = 0;

#ifdef USE_XPD_SPI_ERROR_DETECT
        /* CRC handling */
        if (hspi->CRCSize > 0)
        {
            /* CRC reception is processed by SPI interrupt */
#ifdef SPI_SR_FRLVL
            /* Set FIFO threshold according to CRC size */
            SPI_REG_BIT(hspi, CR2, FRXTH) = 2 - hspi->CRCSize;
#endif
            /* Enable RXNE and ERR interrupt */
            SET_BIT(hspi->Inst->CR2.w, SPI_CR2_RXNEIE | SPI_CR2_ERRIE);

            /* receive callback is provided in interrupt handler in CRC mode */
            XPD_SAFE_CALLBACK(hspi->Callbacks.Transmit, hspi);
            return;
        }
#endif
    }
    XPD_SAFE_CALLBACK(hspi->Callbacks.Transmit, hspi);
    XPD_SAFE_CALLBACK(hspi->Callbacks.Receive, hspi);
}

#ifdef USE_XPD_DMA_ERROR_DETECT
static void spi_dmaErrorRedirect(void * hdma)
{
    SPI_HandleType * hspi = (SPI_HandleType*) ((DMA_HandleType*) hdma)->Owner;

    hspi->Errors |= SPI_ERROR_DMA;

    XPD_SAFE_CALLBACK(hspi->Callbacks.Error, hspi);
}
#endif

static XPD_ReturnType spi_waitFinished(SPI_HandleType * hspi, uint32_t * timeout)
{
    XPD_ReturnType result;

    /* While SPI is still busy with previous transfer, new one is not started */
    result = XPD_WaitForMatch(&hspi->Inst->SR.w, SPI_SR_BSY, 0, timeout);

    while (XPD_SPI_GetFlag(hspi, RXNE) != 0)
    {
        /* empty received data from data register */
        uint16_t temp = SPI_REG_BY_SIZE(&hspi->Inst->DR, hspi->RxStream.size);
    }
    return result;
}

/** @defgroup SPI_Exported_Functions SPI Exported Functions
 * @{ */

/**
 * @brief Initializes the SPI peripheral using the setup configuration.
 * @param hspi: pointer to the SPI handle structure
 * @param Config: SPI setup configuration
 * @return ERROR if input is incorrect, OK if success
 */
XPD_ReturnType XPD_SPI_Init(SPI_HandleType * hspi, SPI_InitType * Config)
{
    /* enable clock */
    XPD_SAFE_CALLBACK(hspi->ClockCtrl, ENABLE);

#ifdef SPI_BB
    hspi->Inst_BB = SPI_BB(hspi->Inst);
#endif

    /* Dependencies initialization */
    XPD_SAFE_CALLBACK(hspi->Callbacks.DepInit, hspi);

    XPD_SPI_Disable(hspi);

    /* Set bits related to Channel and NSS behavior */
    MODIFY_REG(hspi->Inst->CR1.w, SPI_CR1_BIDIMODE | SPI_CR1_RXONLY | SPI_CR1_SSM, Config->Channel | Config->NSS);

    /* Set both bits according to master mode */
    SPI_REG_BIT(hspi, CR1, MSTR)     = Config->Mode;
    SPI_REG_BIT(hspi, CR1, SSI)      = Config->Mode;

    SPI_REG_BIT(hspi, CR1, CPOL)     = Config->Clock.Polarity;
    SPI_REG_BIT(hspi, CR1, CPHA)     = Config->Clock.Phase;
    SPI_REG_BIT(hspi, CR1, LSBFIRST) = Config->Format;

    hspi->Inst->CR1.b.BR = Config->Clock.Prescaler - 1;

#ifdef USE_XPD_SPI_ERROR_DETECT
    /* Disable CRC by setting 0 length */
    if (Config->CRC_Length == 0)
    {
#ifdef SPI_CR1_CRCL
        SPI_REG_BIT(hspi, CR1, CRCL) = 0;
#endif
        SPI_REG_BIT(hspi, CR1, CRCEN) = 0;
        hspi->CRCSize = 0;
    }
    else
    {
#ifdef SPI_CR1_CRCL
        /* Set 8 bit CRC when not 16 bit is requested, and data fits in 1 byte */
        if ((Config->CRC_Length < 16) && (Config->DataSize <= 8))
        {
            hspi->CRCSize = 1;
        }
        /* Set 16 bit CRC when requested, or when data size requires it */
        else
        {
            hspi->CRCSize = 2;
        }
        SPI_REG_BIT(hspi, CR1, CRCL) = hspi->CRCSize - 1;
#else
        /* CRC size equals data size */
        hspi->CRCSize = (Config->DataSize > 8) ? 2 : 1;
#endif
        /* Set up CRC configuration */
        SPI_REG_BIT(hspi, CR1, CRCEN) = 1;
        hspi->Inst->CRCPR = Config->CRC_Polynomial;
    }
#endif

#ifdef SPI_SR_FRLVL
    /* Rx FIFO threshold is set to quarter full when data size fits in 1 byte */
    SPI_REG_BIT(hspi, CR2, FRXTH) = (uint32_t)(Config->DataSize <= 8);
#endif
    SPI_REG_BIT(hspi, CR2, FRF)   = Config->TI_Mode;

#if defined(SPI_CR2_DS)
    /* Configure data bit size */
    hspi->Inst->CR2.b.DS = Config->DataSize - 1;
#elif defined(SPI_CR1_DFF)
    /* Configure data frame format */
    SPI_REG_BIT(hspi, CR1, DFF)   = (uint32_t)(Config->DataSize > 8);
#endif

    /* Activate the SPI mode (Make sure that I2SMOD bit in I2SCFGR register is reset) */
    SPI_REG_BIT(hspi, I2SCFGR, I2SMOD) = 0;

    /* Initialize handle variables */
    hspi->TxStream.length = hspi->RxStream.length = 0;
    hspi->TxStream.size   = hspi->RxStream.size   = (Config->DataSize <= 8) ? 1 : 2;

    return XPD_OK;
}

/**
 * @brief Restores the SPI peripheral to its default inactive state.
 * @param hspi: pointer to the SPI handle structure
 * @return ERROR if input is incorrect, OK if success
 */
XPD_ReturnType XPD_SPI_Deinit(SPI_HandleType * hspi)
{
    XPD_SPI_Disable(hspi);

    /* Disable clock */
    XPD_SAFE_CALLBACK(hspi->ClockCtrl, DISABLE);

    /* Deinitialize peripheral dependencies */
    XPD_SAFE_CALLBACK(hspi->Callbacks.DepDeinit, hspi);

    return XPD_OK;
}

/**
 * @brief Enables the SPI peripheral.
 * @param hspi: pointer to the SPI handle structure
 */
void XPD_SPI_Enable(SPI_HandleType * hspi)
{
    SPI_REG_BIT(hspi, CR1, SPE) = 1;
}

/**
 * @brief Disables the SPI peripheral.
 * @param hspi: pointer to the SPI handle structure
 */
void XPD_SPI_Disable(SPI_HandleType * hspi)
{
    SPI_REG_BIT(hspi, CR1, SPE) = 0;
}

/**
 * @brief Determines the current status of SPI peripheral.
 * @param hspi: pointer to the SPI handle structure
 * @return BUSY if a transfer is in progress, OK if SPI is ready for new transfer
 */
XPD_ReturnType XPD_SPI_GetStatus(SPI_HandleType * hspi)
{
    return (XPD_SPI_GetFlag(hspi, BSY) != 0) ? XPD_BUSY : XPD_OK;
}

/**
 * @brief Polls the status of the SPI transfer.
 * @param hspi: pointer to the SPI handle structure
 * @param Timeout: the timeout in ms for the polling.
 * @return ERROR if there were transfer errors, TIMEOUT if timed out, OK if successful
 */
XPD_ReturnType XPD_SPI_PollStatus(SPI_HandleType * hspi, uint32_t Timeout)
{
#ifdef USE_XPD_SPI_ERROR_DETECT
    /* wait until not busy, or until an error is present */
    XPD_ReturnType result = XPD_WaitForDiff(&hspi->Inst->SR.w,
            SPI_SR_BSY | SPI_SR_OVR | SPI_SR_MODF | SPI_SR_CRCERR | SPI_SR_FRE,
            SPI_SR_BSY, &Timeout);

    /* Error checks */
    if (XPD_SPI_GetFlag(hspi, OVR))
    {
        /* Update error code */
        hspi->Errors |= SPI_ERROR_OVR;

        /* Clear the transfer error flag */
        XPD_SPI_ClearFlag(hspi, OVR);

        result = XPD_ERROR;
    }
    if (XPD_SPI_GetFlag(hspi, MODF))
    {
        /* Update error code */
        hspi->Errors |= SPI_ERROR_MODF;

        /* Clear the transfer error flag */
        XPD_SPI_ClearFlag(hspi, MODF);

        result = XPD_ERROR;
    }
    if (XPD_SPI_GetFlag(hspi, CRCERR))
    {
        /* Update error code */
        hspi->Errors |= SPI_ERROR_CRC;

        /* Clear the transfer error flag */
        XPD_SPI_ClearFlag(hspi, CRCERR);

        result = XPD_ERROR;
    }
    if (XPD_SPI_GetFlag(hspi, FRE))
    {
        /* Update error code */
        hspi->Errors |= SPI_ERROR_FRE;

        /* Clear the transfer error flag */
        XPD_SPI_ClearFlag(hspi, FRE);

        result = XPD_ERROR;
    }
#else
    XPD_ReturnType result = XPD_WaitForMatch(&hspi->Inst->SR.w, SPI_SR_BSY, 0, &Timeout);
#endif

    return result;
}

/**
 * @brief Transmits data over SPI.
 * @param hspi: pointer to the SPI handle structure
 * @param TxData: pointer to the data buffer
 * @param Length: amount of data transfers
 * @param Timeout: available time for successful transmission in ms
 * @return TIMEOUT if transmission or reception times out,
 *         OK if transfer is completed
 */
XPD_ReturnType XPD_SPI_Transmit(SPI_HandleType * hspi, void * TxData, uint16_t Length, uint32_t Timeout)
{
    XPD_ReturnType result = XPD_OK;

    if (Length > 0)
    {
        boolean_t duplex = TRUE;

        /* While SPI is still busy with previous transfer, new one is not started */
        result = spi_waitFinished(hspi, &Timeout);
        if (result != XPD_OK)
        {
            return result;
        }

        /* save stream info */
        hspi->TxStream.buffer = TxData;
        hspi->TxStream.length = Length;
        SPI_RESET_ERRORS(hspi);

        /* Configure communication direction : 1Line */
        if (SPI_REG_BIT(hspi, CR1, BIDIMODE) != 0)
        {
            SPI_REG_BIT(hspi, CR1, BIDIOE) = 1;
            duplex = FALSE;
        }

#ifdef USE_XPD_SPI_ERROR_DETECT
        /* Reset CRC Calculation */
        if (hspi->CRCSize > 0)
        {
            spi_initCRC(hspi);
        }
#endif

        /* Check if the SPI is already enabled */
        XPD_SPI_Enable(hspi);

        while (hspi->TxStream.length > 0)
        {
            /* wait for empty transmit buffer */
            result = XPD_WaitForDiff(&hspi->Inst->SR.w, SPI_SR_TXE, 0, &Timeout);
            if (result != XPD_OK)
            {
                return result;
            }

            XPD_WriteFromStream(&hspi->Inst->DR, &hspi->TxStream);
        }
        /* wait for the last transmission to finish */
        result = XPD_WaitForDiff(&hspi->Inst->SR.w, SPI_SR_TXE, 0, &Timeout);

#ifdef USE_XPD_SPI_ERROR_DETECT
        /* Enable CRC Transmission */
        if (hspi->CRCSize > 0)
        {
            SPI_REG_BIT(hspi, CR1, CRCNEXT) = 1;
        }
#endif

        /* Control the BSY flag */
        result = XPD_WaitForMatch(&hspi->Inst->SR.w, SPI_SR_BSY, 0, &Timeout);

        /* Clear overrun flag in 2 Lines communication mode because received data is not read */
        if (duplex)
        {
            while (XPD_SPI_GetFlag(hspi, RXNE) != 0)
            {
                /* empty received data from data register */
                uint16_t temp = SPI_REG_BY_SIZE(&hspi->Inst->DR, hspi->RxStream.size);
            }
            XPD_SPI_ClearFlag(hspi, OVR);
        }
    }

    return result;
}

/**
 * @brief Receives data over SPI.
 * @param hspi: pointer to the SPI handle structure
 * @param RxData: pointer to the data buffer
 * @param Length: amount of data transfers
 * @param Timeout: available time for successful transmission in ms
 * @return TIMEOUT if transmission or reception times out,
 *         ERROR if a reception error occurs,
 *         OK if transfer is completed
 */
XPD_ReturnType XPD_SPI_Receive(SPI_HandleType * hspi, void * RxData, uint16_t Length, uint32_t Timeout)
{
    XPD_ReturnType result = XPD_OK;

    if (Length > 0)
    {
        /* If master mode, and full duplex communication */
        if (SPI_MASTER_FULL_DUPLEX(hspi))
        {
            /* the receive process is not supported in 2Lines direction master mode */
            /* in this case we call the TransmitReceive process                     */
            return XPD_SPI_TransmitReceive(hspi, RxData, RxData, Length, Timeout);
        }

        /* While SPI is still busy with previous transfer, new one is not started */
        result = spi_waitFinished(hspi, &Timeout);
        if (result != XPD_OK)
        {
            return result;
        }

        /* save stream info */
        hspi->RxStream.buffer = RxData;
        hspi->RxStream.length = Length;
        SPI_RESET_ERRORS(hspi);

#ifdef USE_XPD_SPI_ERROR_DETECT
        /* Reset CRC Calculation */
        if (hspi->CRCSize > 0)
        {
            spi_initCRC(hspi);
        }
#endif

        /* Configure communication direction 1Line and enabled SPI if needed */
        if (SPI_REG_BIT(hspi, CR1, BIDIMODE) != 0)
        {
            SPI_REG_BIT(hspi, CR1, BIDIOE) = 0;
        }

        /* Check if the SPI is already enabled */
        XPD_SPI_Enable(hspi);

        while (hspi->RxStream.length > 0)
        {
            /* Wait for not empty receive buffer */
            result = XPD_WaitForDiff(&hspi->Inst->SR.w, SPI_SR_RXNE, 0, &Timeout);
            if (result != XPD_OK)
            {
                return result;
            }

            XPD_ReadToStream(&hspi->Inst->DR, &hspi->RxStream);

#ifdef USE_XPD_SPI_ERROR_DETECT
            /* This is done to handle the CRCNEXT before the last data */
            if ((hspi->CRCSize > 0) && (hspi->RxStream.length == 1))
            {
                SPI_REG_BIT(hspi, CR1, CRCNEXT) = 1;
            }
#endif
        }

#ifdef USE_XPD_SPI_ERROR_DETECT
        /* Handle the CRC Reception */
        if (hspi->CRCSize > 0)
        {
            result = spi_receiveCRC(hspi, &Timeout);
        }
#endif

        /* If master mode, and either simplex, or half duplex communication */
        if (SPI_MASTER_RXONLY(hspi))
        {
            /* Disable SPI peripheral */
            XPD_SPI_Disable(hspi);
        }
    }
    return result;
}

/**
 * @brief Simultaneously transmits and receives data over SPI (full duplex communication).
 * @param hspi: pointer to the SPI handle structure
 * @param TxData: pointer to the transmitted data buffer
 * @param RxData: pointer to the received data buffer
 * @param Length: amount of data transfers
 * @param Timeout: available time for successful transfer in ms
 * @return TIMEOUT if transmission or reception times out,
 *         ERROR if a reception error occurs,
 *         OK if transfer is completed
 */
XPD_ReturnType XPD_SPI_TransmitReceive(SPI_HandleType * hspi, void * TxData, void * RxData,
        uint16_t Length, uint32_t Timeout)
{
    XPD_ReturnType result = XPD_OK;

    if (Length > 0)
    {
        boolean_t duplex = TRUE, exclMaster = FALSE;

        /* While SPI is still busy with previous transfer, new one is not started */
        result = spi_waitFinished(hspi, &Timeout);
        if (result != XPD_OK)
        {
            return result;
        }

        /* save stream info */
        hspi->TxStream.buffer = TxData;
        hspi->TxStream.length = Length;
        hspi->RxStream.buffer = RxData;
        hspi->RxStream.length = Length;
        SPI_RESET_ERRORS(hspi);

#ifdef USE_XPD_SPI_ERROR_DETECT
        /* Reset CRC Calculation */
        if (hspi->CRCSize > 0)
        {
            spi_initCRC(hspi);
        }
#endif

        /* Check if the SPI is already enabled */
        XPD_SPI_Enable(hspi);

        while ((hspi->TxStream.length + hspi->RxStream.length) > 0)
        {
            /* wait for empty transmit buffer */
            result = XPD_WaitForDiff(&hspi->Inst->SR.w, SPI_SR_TXE | SPI_SR_RXNE, 0, &Timeout);
            if (result != XPD_OK)
            {
                return result;
            }

            /* Time to transmit */
            if ((hspi->TxStream.length > 0) && (XPD_SPI_GetFlag(hspi, TXE)))
            {
                XPD_WriteFromStream(&hspi->Inst->DR, &hspi->TxStream);

#ifdef USE_XPD_SPI_ERROR_DETECT
                /* Enable CRC Transmission */
                if ((hspi->CRCSize > 0) && (hspi->TxStream.length == 0))
                {
                    SPI_REG_BIT(hspi, CR1, CRCNEXT) = 1;
                }
#endif
            }

            /* Time to receive */
            if ((hspi->RxStream.length > 0) && (XPD_SPI_GetFlag(hspi, RXNE)))
            {
                XPD_ReadToStream(&hspi->Inst->DR, &hspi->RxStream);
            }
        }

#ifdef USE_XPD_SPI_ERROR_DETECT
        /* Handle the CRC Reception */
        if (hspi->CRCSize > 0)
        {
            result = spi_receiveCRC(hspi, &Timeout);
        }
#endif
    }
    return result;
}

/**
 * @brief Starts interrupt-driven data transmission over SPI.
 * @note  The function indefinitely waits for the end of the previous transfer (BSY flag)
 * @param hspi: pointer to the SPI handle structure
 * @param TxData: pointer to the data buffer
 * @param Length: amount of data transfers
 */
void XPD_SPI_Transmit_IT(SPI_HandleType * hspi, void * TxData, uint16_t Length)
{
    if (Length > 0)
    {
        uint32_t timeout = SPI_BUSY_TIMEOUT;
        /* While SPI is still busy with previous transfer, new one is not started */
        (void)spi_waitFinished(hspi, &timeout);

        /* save stream info */
        hspi->TxStream.buffer = TxData;
        hspi->TxStream.length = Length;
        SPI_RESET_ERRORS(hspi);

        /* Configure communication direction : 1Line */
        if (SPI_REG_BIT(hspi, CR1, BIDIMODE) != 0)
        {
            SPI_REG_BIT(hspi, CR1, BIDIOE) = 1;
        }

#ifdef USE_XPD_SPI_ERROR_DETECT
        /* Reset CRC Calculation */
        if (hspi->CRCSize > 0)
        {
            spi_initCRC(hspi);
        }
#endif
        /* Enable TXE and ERR interrupt */
        XPD_SPI_EnableIT(hspi, TXE);

        /* Check if the SPI is already enabled */
        XPD_SPI_Enable(hspi);
    }
}

/**
 * @brief Starts interrupt-driven data reception over SPI.
 * @note  The function indefinitely waits for the end of the previous transfer (BSY flag)
 * @param hspi: pointer to the SPI handle structure
 * @param TxData: pointer to the data buffer
 * @param Length: amount of data transfers
 */
void XPD_SPI_Receive_IT(SPI_HandleType * hspi, void * RxData, uint16_t Length)
{
    if (Length > 0)
    {
        uint32_t timeout = SPI_BUSY_TIMEOUT;

        /* If master mode, and full duplex communication */
        if (SPI_MASTER_FULL_DUPLEX(hspi))
        {
            /* the receive process is not supported in 2Lines direction master mode */
            /* in this case we call the TransmitReceive process                     */
            XPD_SPI_TransmitReceive_IT(hspi, RxData, RxData, Length);
        }

        /* While SPI is still busy with previous transfer, new one is not started */
        (void)spi_waitFinished(hspi, &timeout);

        /* save stream info */
        hspi->RxStream.buffer = RxData;
        hspi->RxStream.length = Length;
        SPI_RESET_ERRORS(hspi);

        /* Configure communication direction 1Line and enabled SPI if needed */
        if (SPI_REG_BIT(hspi, CR1, BIDIMODE) != 0)
        {
            SPI_REG_BIT(hspi, CR1, BIDIOE) = 0;
        }

#ifdef USE_XPD_SPI_ERROR_DETECT
        /* Reset CRC Calculation */
        if (hspi->CRCSize > 0)
        {
            spi_initCRC(hspi);
        }
        /* Enable RXNE and ERR interrupt */
        SET_BIT(hspi->Inst->CR2.w, SPI_CR2_RXNEIE | SPI_CR2_ERRIE);
#else
        /* Enable RXNE interrupt */
        XPD_SPI_EnableIT(hspi, RXNE);
#endif

        /* Check if the SPI is already enabled */
        XPD_SPI_Enable(hspi);
    }
}

/**
 * @brief Starts interrupt-driven full-duplex data transfer over SPI.
 * @note  The function indefinitely waits for the end of the previous transfer (BSY flag)
 * @param hspi: pointer to the SPI handle structure
 * @param TxData: pointer to the transmitted data buffer
 * @param RxData: pointer to the received data buffer
 * @param Length: amount of data transfers
 */
void XPD_SPI_TransmitReceive_IT(SPI_HandleType * hspi, void * TxData, void * RxData, uint16_t Length)
{
    if (Length > 0)
    {
        uint32_t timeout = SPI_BUSY_TIMEOUT;
        /* While SPI is still busy with previous transfer, new one is not started */
        (void)spi_waitFinished(hspi, &timeout);

        /* save stream info */
        hspi->TxStream.buffer = TxData;
        hspi->TxStream.length = Length;
        hspi->RxStream.buffer = RxData;
        hspi->RxStream.length = Length;
        SPI_RESET_ERRORS(hspi);

#ifdef USE_XPD_SPI_ERROR_DETECT
        /* Reset CRC Calculation */
        if (hspi->CRCSize > 0)
        {
            spi_initCRC(hspi);
        }
        /* Enable TXE, RXNE and ERR interrupt */
        SET_BIT(hspi->Inst->CR2.w, SPI_CR2_TXEIE | SPI_CR2_RXNEIE | SPI_CR2_ERRIE);
#else
        /* Enable TXE, RXNE interrupt */
        SET_BIT(hspi->Inst->CR2.w, SPI_CR2_TXEIE | SPI_CR2_RXNEIE);
#endif

        /* Check if the SPI is already enabled */
        XPD_SPI_Enable(hspi);
    }
}

/**
 * @brief SPI transfer interrupt handler that provides handle callbacks.
 * @param hspi: pointer to the SPI handle structure
 */
void XPD_SPI_IRQHandler(SPI_HandleType * hspi)
{
    uint32_t cr2 = hspi->Inst->CR2.w;
    uint32_t sr = hspi->Inst->SR.w;

    /* Successful reception */
    if (((sr & (SPI_SR_RXNE | SPI_SR_OVR)) == SPI_SR_RXNE) && ((cr2 & SPI_CR2_RXNEIE) != 0))
    {
#ifdef USE_XPD_SPI_ERROR_DETECT
        /* If CRC is used */
        if (hspi->CRCSize > 0)
        {
            /* Avoid reading CRC to buffer */
            if (hspi->RxStream.length > 0)
            {
                /* Read data from FIFO */
                XPD_ReadToStream(&hspi->Inst->DR, &hspi->RxStream);

                /* This is done to handle the CRCNEXT before the last data */
                if (hspi->RxStream.length == 1)
                {
                    SPI_REG_BIT(hspi, CR1, CRCNEXT) = 1;
                }

#ifdef SPI_SR_FRLVL
                /* End of data reception */
                else if (hspi->RxStream.length == 0)
                {
                    /* Set FIFO threshold according to CRC size */
                    SPI_REG_BIT(hspi, CR2, FRXTH) = 2 - hspi->CRCSize;
                }
#endif
            }
            else
            {
                /* read CRC data from data register */
                uint16_t temp = SPI_REG_BY_SIZE(&hspi->Inst->DR, hspi->RxStream.size);
#ifdef SPI_SR_FRLVL
                /* Reset Rx FIFO threshold */
                SPI_REG_BIT(hspi, CR2, FRXTH) = 2 - hspi->RxStream.size;
#endif

                /* Check if CRC error occurred */
                if ((sr & SPI_SR_CRCERR) != 0)
                {
                    hspi->Errors |= SPI_ERROR_CRC;
                    XPD_SPI_ClearFlag(hspi, CRCERR);

                    /* error callback */
                    XPD_SAFE_CALLBACK(hspi->Callbacks.Error, hspi);
                }

                /* If master mode, and either simplex, or half duplex communication */
                if (SPI_MASTER_RXONLY(hspi))
                {
                    /* Disable SPI peripheral */
                    XPD_SPI_Disable(hspi);
                }

                /* Disable RXNE and ERR interrupt */
                CLEAR_BIT(hspi->Inst->CR2.w, SPI_CR2_RXNEIE | SPI_CR2_ERRIE);

                /* reception finished callback */
                XPD_SAFE_CALLBACK(hspi->Callbacks.Receive, hspi);
            }
        }
        else
#endif
        {
            XPD_ReadToStream(&hspi->Inst->DR, &hspi->RxStream);

            /* End of reception */
            if (hspi->RxStream.length == 0)
            {
                /* If master mode, and either simplex, or half duplex communication */
                if (SPI_MASTER_RXONLY(hspi))
                {
                    /* Disable SPI peripheral */
                    XPD_SPI_Disable(hspi);
                }

                /* Disable RXNE and ERR interrupt */
                CLEAR_BIT(hspi->Inst->CR2.w, SPI_CR2_RXNEIE | SPI_CR2_ERRIE);

                /* reception finished callback */
                XPD_SAFE_CALLBACK(hspi->Callbacks.Receive, hspi);
            }
        }
    }

    /* Successful transmission */
    if (((sr & SPI_SR_TXE) != 0) && ((cr2 & SPI_CR2_TXEIE) != 0))
    {
        XPD_WriteFromStream(&hspi->Inst->DR, &hspi->TxStream);

        if (hspi->TxStream.length == 0)
        {
#ifdef USE_XPD_SPI_ERROR_DETECT
            /* Enable CRC Transmission */
            if (hspi->CRCSize > 0)
            {
                SPI_REG_BIT(hspi, CR1, CRCNEXT) = 1;
            }
#endif
            XPD_SPI_DisableIT(hspi, TXE);

            /* Clear overrun flag in 2 Lines communication mode because received is not read */
            if (SPI_REG_BIT(hspi, CR1, BIDIMODE) == 0)
            {
                while (XPD_SPI_GetFlag(hspi, RXNE) != 0)
                {
                    /* empty received data from data register */
                    uint16_t temp = SPI_REG_BY_SIZE(&hspi->Inst->DR, hspi->RxStream.size);
                }
                XPD_SPI_ClearFlag(hspi, OVR);
            }

            XPD_SAFE_CALLBACK(hspi->Callbacks.Transmit, hspi);
        }
    }

#ifdef USE_XPD_SPI_ERROR_DETECT
    /* Transfer error occurred */
    if (((sr & (SPI_SR_MODF | SPI_SR_OVR | SPI_SR_FRE)) != 0) && ((cr2 & SPI_CR2_ERRIE) != 0))
    {
        if ((sr & SPI_SR_OVR) != 0)
        {
            hspi->Errors |= SPI_ERROR_OVR;
            XPD_SPI_ClearFlag(hspi, OVR);
        }
        if ((sr & SPI_SR_MODF) != 0)
        {
            hspi->Errors |= SPI_ERROR_MODF;
            XPD_SPI_ClearFlag(hspi, MODF);
        }
        if ((sr & SPI_SR_FRE) != 0)
        {
            hspi->Errors |= SPI_ERROR_FRE;
            XPD_SPI_ClearFlag(hspi, FRE);
        }
        /* Clear interrupt enable bits */
        CLEAR_BIT(hspi->Inst->CR2.w, SPI_CR2_TXEIE | SPI_CR2_RXNEIE | SPI_CR2_ERRIE);

        XPD_SAFE_CALLBACK(hspi->Callbacks.Error, hspi);
    }
#endif
}

/**
 * @brief Starts DMA-managed data transmission over SPI.
 * @note  The function indefinitely waits for the end of the previous transfer (BSY flag)
 * @param hspi: pointer to the SPI handle structure
 * @param TxData: pointer to the data buffer
 * @param Length: amount of data transfers
 * @return BUSY if DMA is in use, OK if transfer is started
 */
XPD_ReturnType XPD_SPI_Transmit_DMA(SPI_HandleType * hspi, void * TxData, uint16_t Length)
{
    XPD_ReturnType result = XPD_OK;

    if (Length > 0)
    {
        uint32_t timeout = SPI_BUSY_TIMEOUT;
        /* While SPI is still busy with previous transfer, new one is not started */
        (void)spi_waitFinished(hspi, &timeout);

        /* save stream info */
        hspi->TxStream.buffer = TxData;
        hspi->TxStream.length = Length;

        /* Set up DMA for transfer */
        result = XPD_DMA_Start_IT(hspi->DMA.Transmit, (void*) &hspi->Inst->DR, TxData, Length);

        if (result == XPD_OK)
        {
            /* Set the callback owner */
            hspi->DMA.Transmit->Owner = hspi;

            /* Set the DMA transfer callbacks */
            hspi->DMA.Transmit->Callbacks.Complete     = spi_dmaTransmitRedirect;
            hspi->DMA.Transmit->Callbacks.HalfComplete = NULL;
#ifdef USE_XPD_DMA_ERROR_DETECT
            hspi->DMA.Transmit->Callbacks.Error        = spi_dmaErrorRedirect;
#endif
            SPI_RESET_ERRORS(hspi);

#ifdef USE_XPD_SPI_ERROR_DETECT
            /* Reset CRC Calculation */
            if (hspi->CRCSize > 0)
            {
                spi_initCRC(hspi);
            }
#endif

            /* Configure communication direction : 1Line */
            if (SPI_REG_BIT(hspi, CR1, BIDIMODE) != 0)
            {
                SPI_REG_BIT(hspi, CR1, BIDIOE) = 1;
            }

            /* Enable Tx DMA Request */
            SPI_REG_BIT(hspi, CR2, TXDMAEN) = 1;

            /* Check if the SPI is already enabled */
            XPD_SPI_Enable(hspi);
        }
    }
    return result;
}

/**
 * @brief Starts DMA-managed data reception over SPI.
 * @note  The function indefinitely waits for the end of the previous transfer (BSY flag)
 * @param hspi: pointer to the SPI handle structure
 * @param RxData: pointer to the data buffer
 * @param Length: amount of data transfers
 * @return BUSY if DMA is in use, OK if transfer is started
 */
XPD_ReturnType XPD_SPI_Receive_DMA(SPI_HandleType * hspi, void * RxData, uint16_t Length)
{
    XPD_ReturnType result = XPD_OK;

    /* If master mode, and full duplex communication */
    if (SPI_MASTER_FULL_DUPLEX(hspi))
    {
        /* the receive process is not supported in 2Lines direction master mode
         * in this case we call the TransmitReceive process */
        result = XPD_SPI_TransmitReceive_DMA(hspi, NULL, RxData, Length);
    }

    else if (Length > 0)
    {
        uint32_t timeout = SPI_BUSY_TIMEOUT;
        /* While SPI is still busy with previous transfer, new one is not started */
        (void)spi_waitFinished(hspi, &timeout);

        /* save stream info */
        hspi->RxStream.buffer = RxData;
        hspi->RxStream.length = Length;

        /* Set up DMA for transfer */
        result = XPD_DMA_Start_IT(hspi->DMA.Receive, (void*) &hspi->Inst->DR, RxData, Length);

        if (result == XPD_OK)
        {
            /* Set the callback owner */
            hspi->DMA.Receive->Owner = hspi;

            /* Set the DMA transfer callbacks */
            hspi->DMA.Receive->Callbacks.Complete     = spi_dmaReceiveRedirect;
            hspi->DMA.Receive->Callbacks.HalfComplete = NULL;
#ifdef USE_XPD_DMA_ERROR_DETECT
            hspi->DMA.Receive->Callbacks.Error        = spi_dmaErrorRedirect;
#endif
            SPI_RESET_ERRORS(hspi);

#ifdef USE_XPD_SPI_ERROR_DETECT
            /* Reset CRC Calculation */
            if (hspi->CRCSize > 0)
            {
                spi_initCRC(hspi);
            }
#endif

            /* Configure communication direction : 1Line */
            if (SPI_REG_BIT(hspi, CR1, BIDIMODE) != 0)
            {
                SPI_REG_BIT(hspi, CR1, BIDIOE) = 0;
            }

            /* Enable Rx DMA Request */
            SPI_REG_BIT(hspi, CR2, RXDMAEN) = 1;

            /* Check if the SPI is already enabled */
            XPD_SPI_Enable(hspi);
        }
    }
    return result;
}

/**
 * @brief Starts DMA-managed full-duplex data transfer over SPI.
 * @note  The function indefinitely waits for the end of the previous transfer (BSY flag)
 * @param hspi: pointer to the SPI handle structure
 * @param TxData: pointer to the transmitted data buffer
 * @param RxData: pointer to the received data buffer
 * @param Length: amount of data transfers
 * @return BUSY if DMA is in use, OK if transfer is started
 */
XPD_ReturnType XPD_SPI_TransmitReceive_DMA(SPI_HandleType * hspi, void * TxData, void * RxData, uint16_t Length)
{
    XPD_ReturnType result = XPD_OK;

    if (Length > 0)
    {
        uint32_t timeout = SPI_BUSY_TIMEOUT;
        /* While SPI is still busy with previous transfer, new one is not started */
        (void)spi_waitFinished(hspi, &timeout);

        /* save stream info */
        hspi->TxStream.buffer = TxData;
        hspi->TxStream.length = Length;
        hspi->RxStream.buffer = RxData;
        hspi->RxStream.length = Length;

        /* In case there is no actual data transmission, send dummy from receive buffer */
        if (TxData == NULL)
        {
            TxData = RxData;
        }

        /* Set up DMAs for transfers */
        result = XPD_DMA_Start_IT(hspi->DMA.Receive, (void*) &hspi->Inst->DR, RxData, Length);

        if (result == XPD_OK)
        {
            result = XPD_DMA_Start_IT(hspi->DMA.Transmit, (void*) &hspi->Inst->DR, TxData, Length);

            /* If one DMA allocation failed, reset the other and exit */
            if (result != XPD_OK)
            {
                XPD_DMA_Stop_IT(hspi->DMA.Receive);
                return result;
            }

            /* Set the callback owner */
            hspi->DMA.Receive->Owner = hspi;

            /* Set the DMA transfer callbacks */
            if (hspi->TxStream.buffer == NULL)
            {
                hspi->DMA.Receive->Callbacks.Complete  = spi_dmaReceiveRedirect;
            }
            else
            {
                hspi->DMA.Receive->Callbacks.Complete  = spi_dmaTransmitReceiveRedirect;
            }
            hspi->DMA.Receive->Callbacks.HalfComplete  = NULL;
#ifdef USE_XPD_DMA_ERROR_DETECT
            /* Set the DMA error callback */
            hspi->DMA.Receive->Callbacks.Error         = spi_dmaErrorRedirect;
#endif

            /* Set the callback owner */
            hspi->DMA.Transmit->Owner = hspi;

            /* Set the SPI Tx DMA transfer complete callback as NULL because the communication closing
            is performed in DMA reception complete callback  */
            hspi->DMA.Transmit->Callbacks.Complete     = NULL;
            hspi->DMA.Transmit->Callbacks.HalfComplete = NULL;
#ifdef USE_XPD_DMA_ERROR_DETECT
            hspi->DMA.Transmit->Callbacks.Error        = spi_dmaErrorRedirect;
#endif
            SPI_RESET_ERRORS(hspi);

#ifdef USE_XPD_SPI_ERROR_DETECT
            /* Reset CRC Calculation */
            if (hspi->CRCSize > 0)
            {
                spi_initCRC(hspi);
            }
#endif

            /* Enable DMA Requests */
            SET_BIT(hspi->Inst->CR2.w, SPI_CR2_RXDMAEN | SPI_CR2_TXDMAEN);

            /* Check if the SPI is already enabled */
            XPD_SPI_Enable(hspi);
        }
    }
    return result;
}

/**
 * @brief Stops all ongoing DMA-managed transfers over SPI.
 * @param hspi: pointer to the SPI handle structure
 */
void XPD_SPI_Stop_DMA(SPI_HandleType * hspi)
{
    /* Transmit DMA disable */
    if (SPI_REG_BIT(hspi, CR2, TXDMAEN) != 0)
    {
        SPI_REG_BIT(hspi, CR2, TXDMAEN) = 0;

        XPD_DMA_Stop_IT(hspi->DMA.Transmit);
    }
    /* Receive DMA disable */
    if (SPI_REG_BIT(hspi, CR2, RXDMAEN) != 0)
    {
        SPI_REG_BIT(hspi, CR2, RXDMAEN) = 0;

        XPD_DMA_Stop_IT(hspi->DMA.Receive);
    }
}

/** @} */

/** @} */

#endif /* USE_XPD_SPI */
