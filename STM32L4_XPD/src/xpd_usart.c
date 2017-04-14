/**
  ******************************************************************************
  * @file    xpd_usart.c
  * @author  Benedek Kupper
  * @version V0.1
  * @date    2016-05-01
  * @brief   STM32 eXtensible Peripheral Drivers USART Module
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

#include "xpd_usart.h"
#include "xpd_utils.h"

#if defined(USE_XPD_USART)

/** @addtogroup USART
 * @{ */

/** @addtogroup USART_Common
 * @{ */

/* clock is enabled */
#define USART_SYNCHRONOUS_MODE(HANDLE) (USART_REG_BIT(HANDLE, CR2, CLKEN) != 0)

/* half-duplex */
#define USART_HALF_DUPLEX_MODE(HANDLE) (USART_REG_BIT(HANDLE, CR3, HDSEL) != 0)

#define USART_APPLY_CONFIG(HANDLE, REG, MASK, CONFIG)                               \
    do{ uint32_t cr1 = (HANDLE)->Inst->CR1.w; uint32_t uartEn = cr1 & USART_CR1_UE; \
    if (uartEn != 0) { husart->Inst->CR1.w = cr1 & ~USART_CR1_UE; }                 \
    MODIFY_REG((HANDLE)->Inst->REG.w, (MASK), (CONFIG));                            \
    husart->Inst->CR1.w |= uartEn;} while (0)

#if defined(USE_XPD_USART_ERROR_DETECT) || defined(USE_XPD_DMA_ERROR_DETECT)
#define USART_RESET_ERRORS(HANDLE)    ((HANDLE)->Errors = USART_ERROR_NONE)
#else
#define USART_RESET_ERRORS(HANDLE)    ((void)0)
#endif

#if (USART_PERIPHERAL_VERSION > 1)
#define USART_STATR(HANDLE)         ((HANDLE)->Inst->ISR.w)
#define USART_STATF(FLAG_NAME)      (USART_ISR_##FLAG_NAME)
#define USART_RXDR(HANDLE)          ((HANDLE)->Inst->RDR)
#define USART_TXDR(HANDLE)          ((HANDLE)->Inst->TDR)
#else
#define USART_STATR(HANDLE)         ((HANDLE)->Inst->SR.w)
#define USART_STATF(FLAG_NAME)      (USART_SR_##FLAG_NAME)
#define USART_RXDR(HANDLE)          ((HANDLE)->Inst->DR)
#define USART_TXDR(HANDLE)          ((HANDLE)->Inst->DR)
#endif

#ifdef USE_XPD_DMA_ERROR_DETECT
static void usart_dmaErrorRedirect(void *hdma)
{
    USART_HandleType * husart = (USART_HandleType*) ((DMA_HandleType*) hdma)->Owner;

    husart->Errors |= USART_ERROR_DMA;

    XPD_SAFE_CALLBACK(husart->Callbacks.Error, husart);
}
#endif

static void usart_dmaTransmitRedirect(void *hdma)
{
    USART_HandleType * husart = (USART_HandleType*) ((DMA_HandleType*) hdma)->Owner;

    /* DMA normal mode */
    if (XPD_DMA_CircularMode((DMA_HandleType*)hdma) == 0)
    {
        /* Disable Tx DMA Request */
        USART_REG_BIT(husart, CR3, DMAT) = 0;

        /* Transmit complete interrupt will provide callback */
        XPD_USART_EnableIT(husart, TC);

        /* Update stream status */
        husart->TxStream.buffer += husart->TxStream.length * husart->TxStream.size;
        husart->TxStream.length = 0;
    }
    /* DMA circular mode */
    else
    {
        XPD_SAFE_CALLBACK(husart->Callbacks.Transmit, husart);
    }
}

static void usart_dmaReceiveRedirect(void *hdma)
{
    USART_HandleType * husart = (USART_HandleType*) ((DMA_HandleType*) hdma)->Owner;

    /* DMA normal mode */
    if (XPD_DMA_CircularMode((DMA_HandleType*)hdma) == 0)
    {
        /* Disable Rx DMA Request */
        USART_REG_BIT(husart, CR3, DMAR) = 0;

        /* Update stream status */
        husart->RxStream.buffer += husart->RxStream.length * husart->RxStream.size;
        husart->RxStream.length = 0;
    }
    XPD_SAFE_CALLBACK(husart->Callbacks.Receive, husart);
}

static void usart_baudrateConfig(USART_HandleType * husart, uint32_t baudrate)
{
#if (USART_PERIPHERAL_VERSION > 1)
    if (USART_REG_BIT(husart, CR1, OVER8) == 0)
    {
        husart->Inst->BRR.w = (XPD_USART_GetClockFreq(husart) + (baudrate / 2)) / baudrate;
    }
    else
    {
        uint32_t tmp = ((XPD_USART_GetClockFreq(husart) * 2) + (baudrate / 2)) / baudrate;
        husart->Inst->BRR.w = (tmp & USART_BRR_DIV_MANTISSA) | ((tmp & USART_BRR_DIV_FRACTION) >> 1);
    }
#else
    uint32_t over8   = USART_REG_BIT(husart, CR1, OVER8);
    uint32_t div     = XPD_USART_GetClockFreq(husart) * 25 / ((2 << over8) * baudrate);
    uint32_t divmant = div / 100;
    uint32_t divfraq = ((div - (divmant * 100)) * (8 << over8) + 50) / 100;

    husart->Inst->BRR.b.DIV_Fraction = divfraq & ((16 >> over8) - 1);
    husart->Inst->BRR.b.DIV_Mantissa = divmant + (divfraq >> (4 - over8));

#endif
}

static void usart_setDirection(USART_HandleType * husart, uint32_t direction)
{
    if ((USART_HALF_DUPLEX_MODE(husart)) && ((husart->Inst->CR1.w & (USART_CR1_TE | USART_CR1_RE)) != direction))
    {
        uint32_t cr1 = husart->Inst->CR1.w;
        CLEAR_BIT(cr1, USART_CR1_UE | USART_CR1_TE | USART_CR1_RE);
        husart->Inst->CR1.w = cr1;
        SET_BIT(cr1, USART_CR1_UE | direction);
        husart->Inst->CR1.w = cr1;
    }
}

static XPD_ReturnType usart_init1(USART_HandleType * husart, const USART_InitType * Common)
{
    uint8_t framesize;

    /* enable clock */
    XPD_SAFE_CALLBACK(husart->ClockCtrl, ENABLE);

#ifdef USART_BB
    husart->Inst_BB = USART_BB(husart->Inst);
#endif

    husart->Inst->CR1.w = 0;
    husart->Inst->CR2.w = 0;
    husart->Inst->CR3.w = 0;

    /* configure frame format and mode */
#ifdef USE_XPD_USART_ERROR_DETECT
    framesize = Common->DataSize + (uint8_t)(Common->Parity != USART_PARITY_NONE);

    USART_REG_BIT(husart, CR1, PS)     = Common->Parity;
    USART_REG_BIT(husart, CR1, PCE)    = Common->Parity >> 1;
#else
    framesize = Common->DataSize;
#endif
    husart->Inst->CR2.b.STOP           = Common->StopBits;
#ifdef USART_CR1_M1
    USART_REG_BIT(husart, CR1, M0)     = (uint32_t)(framesize > 8);
    USART_REG_BIT(husart, CR1, M1)     = (uint32_t)(framesize < 8);
#else
    USART_REG_BIT(husart, CR1, M)      = (uint32_t)(framesize > 8);
#endif
    USART_REG_BIT(husart, CR3, ONEBIT) = Common->SingleSample;

    /* Set data transfer size */
    switch (husart->Inst->CR1.w & (USART_CR1_M | USART_CR1_PCE))
    {
        /* data size = 9, no parity -> mask = 0x1FF */
#ifdef USART_CR1_M0
        case USART_CR1_M0:
#else
        case USART_CR1_M:
#endif
            husart->TxStream.size = husart->RxStream.size = 2;
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
            husart->TxStream.size = husart->RxStream.size = 1;
            break;
    }
    husart->TxStream.length = husart->RxStream.length = 0;

#ifdef USE_XPD_USART_ERROR_DETECT
    /* Reception through DMA will stop if a reception error occurs */
    husart->haltOnError = Common->HaltRxOnError;
#ifdef USART_CR3_DDRE
    USART_REG_BIT(husart, CR3, DDRE) = Common->HaltRxOnError;
#endif
#endif

    return XPD_OK;
}

static XPD_ReturnType usart_init2(USART_HandleType * husart, const USART_InitType * Common)
{
    XPD_ReturnType result = XPD_OK;

    /* baudrate configuration */
    usart_baudrateConfig(husart, Common->BaudRate);

    USART_REG_BIT(husart, CR1, TE) = Common->Transmitter;
    USART_REG_BIT(husart, CR1, RE) = Common->Receiver;

    XPD_USART_Enable(husart);

    /* Dependencies initialization */
    XPD_SAFE_CALLBACK(husart->Callbacks.DepInit, husart);

#ifdef IS_UART_WAKEUP_FROMSTOP_INSTANCE
    if (IS_UART_WAKEUP_FROMSTOP_INSTANCE(husart->Inst))
    {
        uint32_t timeout = 1000;
        uint32_t flags = (Common->Transmitter * USART_ISR_TEACK) | (Common->Receiver * USART_ISR_REACK);

        result = XPD_WaitForMatch(&husart->Inst->ISR.w, flags, flags, &timeout);
    }
#endif
    return result;
}

/** @defgroup USART_Common_Exported_Functions USART Common Exported Functions
 * @{ */

/**
 * @brief Restores the USART peripheral to its default inactive state
 * @param husart: pointer to the USART handle structure
 * @return ERROR if input is incorrect, OK if success
 */
XPD_ReturnType XPD_USART_Deinit(USART_HandleType * husart)
{
    XPD_USART_Disable(husart);

    /* Deinitialize peripheral dependencies */
    XPD_SAFE_CALLBACK(husart->Callbacks.DepDeinit, husart);

    /* disable clock */
    XPD_SAFE_CALLBACK(husart->ClockCtrl, DISABLE);

	return XPD_OK;
}

/**
 * @brief Enables the USART peripheral.
 * @param husart: pointer to the USART handle structure
 */
void XPD_USART_Enable(USART_HandleType * husart)
{
    USART_REG_BIT(husart,CR1,UE) = 1;
}

/**
 * @brief Disables the USART peripheral.
 * @param husart: pointer to the USART handle structure
 */
void XPD_USART_Disable(USART_HandleType * husart)
{
    USART_REG_BIT(husart,CR1,UE) = 0;
}

/**
 * @brief Determines the current status of USART peripheral.
 * @param husart: pointer to the USART handle structure
 * @return BUSY if a transmission is in progress, OK if USART is ready for new transfer
 */
XPD_ReturnType XPD_USART_GetStatus(USART_HandleType * husart)
{
    return ((USART_STATR(husart) & (USART_STATF(RXNE) | USART_STATF(TC))) != 0)
            ? XPD_OK : XPD_BUSY;
}

/**
 * @brief Polls the status of the USART transfer.
 * @param husart: pointer to the USART handle structure
 * @param Timeout: the timeout in ms for the polling.
 * @return ERROR if there were transfer errors, TIMEOUT if timed out, OK if successful
 */
XPD_ReturnType XPD_USART_PollStatus(USART_HandleType * husart, uint32_t Timeout)
{
#ifdef USE_XPD_USART_ERROR_DETECT
    /* wait until not busy, or until an error is present */
    XPD_ReturnType result = XPD_WaitForDiff(&USART_STATR(husart),
            USART_STATF(RXNE) | USART_STATF(TC) | USART_STATF(PE) |
            USART_STATF(FE)   | USART_STATF(NE) | USART_STATF(ORE),
            0, &Timeout);

    /* Error checks */
    if (XPD_USART_GetFlag(husart, PE))
    {
        /* Update error code */
        husart->Errors |= USART_ERROR_PARITY;

        /* Clear the transfer error flag */
        XPD_USART_ClearFlag(husart, PE);

        result = XPD_ERROR;
    }
    if (XPD_USART_GetFlag(husart, NE))
    {
        /* Update error code */
        husart->Errors |= USART_ERROR_NOISE;

        /* Clear the transfer error flag */
        XPD_USART_ClearFlag(husart, NE);

        result = XPD_ERROR;
    }
    if (XPD_USART_GetFlag(husart, FE))
    {
        /* Update error code */
        husart->Errors |= USART_ERROR_FRAME;

        /* Clear the transfer error flag */
        XPD_USART_ClearFlag(husart, FE);

        result = XPD_ERROR;
    }
    if (XPD_USART_GetFlag(husart, ORE))
    {
        /* Update error code */
        husart->Errors |= USART_ERROR_OVERRUN;

        /* Clear the transfer error flag */
        XPD_USART_ClearFlag(husart, ORE);

        result = XPD_ERROR;
    }
#else
    XPD_ReturnType result = XPD_WaitForDiff(&USART_STATR(husart),
            USART_STATF(RXNE) | USART_STATF(TC), 0, &Timeout);
#endif
    return result;
}

/**
 * @brief Transmits data over USART.
 * @param husart: pointer to the USART handle structure
 * @param TxData: pointer to the data buffer
 * @param Length: amount of data transfers
 * @param Timeout: available time for successful transmission in ms
 * @return TIMEOUT if transmission or reception times out,
 *         OK if transfer is completed
 */
XPD_ReturnType XPD_USART_Transmit(USART_HandleType * husart, void * TxData, uint16_t Length, uint32_t Timeout)
{
    XPD_ReturnType result;

    /* save stream info */
    husart->TxStream.buffer = TxData;
    husart->TxStream.length = Length;
    USART_RESET_ERRORS(husart);

    /* The direction has to be set in half-duplex mode */
    usart_setDirection(husart, USART_CR1_TE);

    while (husart->TxStream.length > 0)
    {
        /* wait for empty transmit buffer */
        result = XPD_WaitForDiff(&USART_STATR(husart), USART_STATF(TXE), 0, &Timeout);
        if (result != XPD_OK)
        {
            return result;
        }

        XPD_WriteFromStream((__IO uint32_t*)&USART_TXDR(husart), &husart->TxStream);
    }
    /* wait for the last transmission to finish */
    result = XPD_WaitForDiff(&USART_STATR(husart), USART_STATF(TC), 0, &Timeout);

    /* Clear overrun flag because received data is not read */
    XPD_USART_ClearFlag(husart, ORE);

    return result;
}

/**
 * @brief Receives data over USART.
 * @param husart: pointer to the USART handle structure
 * @param RxData: pointer to the data buffer
 * @param Length: amount of data transfers
 * @param Timeout: available time for successful transmission in ms
 * @return TIMEOUT if transmission or reception times out,
 *         ERROR if a reception error occurs,
 *         OK if transfer is completed
 */
XPD_ReturnType XPD_USART_Receive(USART_HandleType * husart, void * RxData, uint16_t Length, uint32_t Timeout)
{
    XPD_ReturnType result;

    /* save stream info */
    husart->RxStream.buffer = RxData;
    husart->RxStream.length = Length;
    USART_RESET_ERRORS(husart);

    /* The direction has to be set in half-duplex mode */
    usart_setDirection(husart, USART_CR1_RE);

    while (husart->RxStream.length > 0)
    {
        /* Send data to generate clock */
        if (USART_SYNCHRONOUS_MODE(husart) && (XPD_USART_GetFlag(husart, TXE) != 0))
        {
            USART_TXDR(husart) = 0;
        }

        /* Wait for not empty receive buffer */
        result = XPD_WaitForDiff(&USART_STATR(husart), USART_STATF(RXNE), 0, &Timeout);
        if (result != XPD_OK)
        {
            return result;
        }

        XPD_ReadToStream((__IO uint32_t*)&USART_RXDR(husart), &husart->RxStream);
    }
    return result;
}

/**
 * @brief Simultaneously transmits and receives data over USART (full duplex communication).
 * @param husart: pointer to the USART handle structure
 * @param TxData: pointer to the transmitted data buffer
 * @param RxData: pointer to the received data buffer
 * @param Length: amount of data transfers
 * @param Timeout: available time for successful transfer in ms
 * @return TIMEOUT if transmission or reception times out,
 *         ERROR if a reception error occurs,
 *         OK if transfer is completed
 */
XPD_ReturnType XPD_USART_TransmitReceive(USART_HandleType * husart, void * TxData, void * RxData, uint16_t Length, uint32_t Timeout)
{
    XPD_ReturnType result;

    /* save stream info */
    husart->TxStream.buffer = TxData;
    husart->TxStream.length = Length;
    husart->RxStream.buffer = RxData;
    husart->RxStream.length = Length;
    USART_RESET_ERRORS(husart);

    while ((husart->TxStream.length + husart->RxStream.length) > 0)
    {
        /* wait for buffer change */
        result = XPD_WaitForDiff(&USART_STATR(husart), USART_STATF(TXE) | USART_STATF(RXNE), 0, &Timeout);
        if (result != XPD_OK)
        {
            return result;
        }

        /* Time to transmit */
        if ((husart->TxStream.length > 0) && (XPD_USART_GetFlag(husart, TXE)))
        {
            XPD_WriteFromStream((__IO uint32_t*)&USART_TXDR(husart), &husart->TxStream);
        }

        /* Time to receive */
        if ((husart->RxStream.length > 0) && (XPD_USART_GetFlag(husart, RXNE)))
        {
            XPD_ReadToStream((__IO uint32_t*)&USART_RXDR(husart), &husart->RxStream);
        }
    }
    return result;
}

/**
 * @brief Starts interrupt-driven data transmission over USART.
 * @param husart: pointer to the USART handle structure
 * @param TxData: pointer to the data buffer
 * @param Length: amount of data transfers
 */
void XPD_USART_Transmit_IT(USART_HandleType * husart, void * TxData, uint16_t Length)
{
    /* save stream info */
    husart->TxStream.buffer = TxData;
    husart->TxStream.length = Length;
    USART_RESET_ERRORS(husart);

    /* The direction has to be set in half-duplex mode */
    usart_setDirection(husart, USART_CR1_TE);

    /* Enable TXE interrupt */
    XPD_USART_EnableIT(husart, TXE);
}

/**
 * @brief Starts interrupt-driven data reception over USART.
 * @note  In synchronous mode the user has to ensure data transmission in order to generate clock.
 * @param husart: pointer to the USART handle structure
 * @param TxData: pointer to the data buffer
 * @param Length: amount of data transfers
 */
void XPD_USART_Receive_IT(USART_HandleType * husart, void * RxData, uint16_t Length)
{
    /* save stream info */
    husart->RxStream.buffer = RxData;
    husart->RxStream.length = Length;
    USART_RESET_ERRORS(husart);

#ifdef USE_XPD_USART_ERROR_DETECT
    XPD_USART_EnableIT(husart, PE);
    XPD_USART_EnableIT(husart, E);
#endif
    /* Enable RXNE interrupt */
    XPD_USART_EnableIT(husart, RXNE);

    /* The direction has to be set in half-duplex mode */
    usart_setDirection(husart, USART_CR1_RE);
}

/**
 * @brief Starts interrupt-driven full-duplex data transfer over USART.
 * @param husart: pointer to the USART handle structure
 * @param TxData: pointer to the transmitted data buffer
 * @param RxData: pointer to the received data buffer
 * @param Length: amount of data transfers
 */
void XPD_USART_TransmitReceive_IT(USART_HandleType * husart, void * TxData, void * RxData, uint16_t Length)
{
    /* save stream info */
    husart->TxStream.buffer = TxData;
    husart->TxStream.length = Length;
    husart->RxStream.buffer = RxData;
    husart->RxStream.length = Length;
    USART_RESET_ERRORS(husart);

#ifdef USE_XPD_USART_ERROR_DETECT
    XPD_USART_EnableIT(husart, PE);
    XPD_USART_EnableIT(husart, E);
#endif
    /* Enable TXE, RXNE interrupt */
    XPD_USART_EnableIT(husart, RXNE);
    XPD_USART_EnableIT(husart, TXE);
}

/**
 * @brief USART transfer interrupt handler that provides handle callbacks.
 * @param husart: pointer to the USART handle structure
 */
void XPD_USART_IRQHandler(USART_HandleType * husart)
{
    uint32_t sr  = USART_STATR(husart);
    uint32_t cr1 = husart->Inst->CR1.w;
    uint32_t cr2 = husart->Inst->CR2.w;
    uint32_t cr3 = husart->Inst->CR3.w;

#ifdef USE_XPD_USART_ERROR_DETECT
    /* Stop DMA reception on error when configured */
    if (((sr & (USART_STATF(NE) | USART_STATF(FE) | USART_STATF(PE))) != 0)
        && (husart->haltOnError))
    {
        USART_REG_BIT(husart, CR3, DMAR) = 0;
    }
    /* parity error */
    if (((sr & USART_STATF(PE)) != 0) && ((cr1 & USART_CR1_PEIE) != 0))
    {
        husart->Errors |= USART_ERROR_PARITY;
        XPD_USART_ClearFlag(husart, PE);
    }
    /* channel errors */
    if ((cr3 & USART_CR3_EIE) != 0)
    {
        if ((sr & USART_STATF(NE)) != 0)
        {
            husart->Errors |= USART_ERROR_NOISE;
            XPD_USART_ClearFlag(husart, NE);
        }
        if ((sr & USART_STATF(FE)) != 0)
        {
            husart->Errors |= USART_ERROR_FRAME;
            XPD_USART_ClearFlag(husart, FE);
        }
        if ((sr & USART_STATF(ORE)) != 0)
        {
            husart->Errors |= USART_ERROR_OVERRUN;
            XPD_USART_ClearFlag(husart, ORE);
        }
    }
    if (husart->Errors != USART_ERROR_NONE)
    {
        XPD_SAFE_CALLBACK(husart->Callbacks.Error, husart);
    }
#endif

    /* successful reception */
    if (((sr & USART_STATF(RXNE)) != 0) && ((cr1 & USART_CR1_RXNEIE) != 0))
    {
        XPD_ReadToStream((__IO uint32_t*)&USART_RXDR(husart), &husart->RxStream);

        /* End of reception */
        if (husart->RxStream.length == 0)
        {
            XPD_USART_DisableIT(husart, RXNE);
#ifdef USE_XPD_USART_ERROR_DETECT
            XPD_USART_DisableIT(husart, PE);
            XPD_USART_DisableIT(husart, E);
#endif

            /* reception finished callback */
            XPD_SAFE_CALLBACK(husart->Callbacks.Receive, husart);
        }
    }

    /* successful transmission */
    if (((sr & USART_STATF(TXE)) != 0) && ((cr1 & USART_CR1_TXEIE) != 0))
    {
        XPD_WriteFromStream(&USART_TXDR(husart), &husart->TxStream);

        /* last transmission, disable TXE, enable transmit complete interrupt */
        if (husart->TxStream.length == 0)
        {
            XPD_USART_DisableIT(husart, TXE);
            XPD_USART_EnableIT(husart, TC);
        }
    }

    /* last element in data stream is transmitted successfully */
    else if (((sr & USART_STATF(TC)) != 0) && ((cr1 & USART_CR1_TCIE) != 0))
    {
        XPD_USART_ClearFlag(husart, TC);
        XPD_USART_DisableIT(husart, TC);

        /* transmission finished callback */
        XPD_SAFE_CALLBACK(husart->Callbacks.Transmit, husart);
    }

    /* IDLE detected */
    if (((sr & USART_STATF(IDLE)) != 0) && ((cr1 & USART_CR1_IDLEIE) != 0))
    {
        XPD_USART_ClearFlag(husart, IDLE);

        XPD_SAFE_CALLBACK(husart->Callbacks.Idle, husart);
    }

    /* LIN break detected */
    if (((sr & USART_STATF(LBD)) != 0) && ((cr2 & USART_CR2_LBDIE) != 0))
    {
        XPD_USART_ClearFlag(husart, LBD);

        XPD_SAFE_CALLBACK(husart->Callbacks.Break, husart);
    }

    /* CTS detected */
    if (((sr & USART_STATF(CTS)) != 0) && ((cr3 & USART_CR3_CTSIE) != 0))
    {
        XPD_USART_ClearFlag(husart, CTS);

        XPD_SAFE_CALLBACK(husart->Callbacks.ClearToSend, husart);
    }

#if (USART_PERIPHERAL_VERSION > 2)
    /* UART wakeup from Stop mode interrupt occurred */
    if(((sr & USART_ISR_WUF) != 0) && ((cr3 & USART_CR3_WUFIE) != 0))
    {
        XPD_USART_ClearFlag(husart, WU);
    }
#endif
}

/**
 * @brief Starts DMA-managed data transmission over USART.
 * @param husart: pointer to the USART handle structure
 * @param TxData: pointer to the data buffer
 * @param Length: amount of data transfers
 * @return BUSY if DMA is in use, OK if transfer is started
 */
XPD_ReturnType XPD_USART_Transmit_DMA(USART_HandleType * husart, void * TxData, uint16_t Length)
{
    XPD_ReturnType result;

    /* save stream info */
    husart->TxStream.buffer = TxData;
    husart->TxStream.length = Length;

    /* Set up DMA for transfer */
    result = XPD_DMA_Start_IT(husart->DMA.Transmit, (void*) &USART_TXDR(husart), TxData, Length);

    if (result == XPD_OK)
    {
        /* Set the callback owner */
        husart->DMA.Transmit->Owner = husart;

        /* Set the DMA transfer callbacks */
        husart->DMA.Transmit->Callbacks.Complete     = usart_dmaTransmitRedirect;
#ifdef USE_XPD_DMA_ERROR_DETECT
        husart->DMA.Transmit->Callbacks.Error        = usart_dmaErrorRedirect;
#endif
        USART_RESET_ERRORS(husart);

        /* The direction has to be set in half-duplex mode */
        usart_setDirection(husart, USART_CR1_TE);

        XPD_USART_ClearFlag(husart, TC);

        USART_REG_BIT(husart, CR3, DMAT) = 1;
    }
    return result;
}

/**
 * @brief Starts DMA-managed data reception over USART.
 * @note  In synchronous mode the user has to ensure data transmission in order to generate clock.
 * @param husart: pointer to the USART handle structure
 * @param RxData: pointer to the data buffer
 * @param Length: amount of data transfers
 * @return BUSY if DMA is in use, OK if transfer is started
 */
XPD_ReturnType XPD_USART_Receive_DMA(USART_HandleType * husart, void * RxData, uint16_t Length)
{
    XPD_ReturnType result;

    /* save stream info */
    husart->RxStream.buffer = RxData;
    husart->RxStream.length = Length;

    /* Set up DMA for transfer */
    result = XPD_DMA_Start_IT(husart->DMA.Receive, (void*) &USART_RXDR(husart), RxData, Length);

    if (result == XPD_OK)
    {
        /* Set the callback owner */
        husart->DMA.Receive->Owner = husart;

        /* Set the DMA transfer callbacks */
        husart->DMA.Receive->Callbacks.Complete     = usart_dmaReceiveRedirect;
#ifdef USE_XPD_DMA_ERROR_DETECT
        husart->DMA.Receive->Callbacks.Error        = usart_dmaErrorRedirect;
#endif
        USART_RESET_ERRORS(husart);

        XPD_USART_ClearFlag(husart, ORE);

        USART_REG_BIT(husart, CR3, DMAR) = 1;

        /* The direction has to be set in half-duplex mode */
        usart_setDirection(husart, USART_CR1_RE);
    }
    return result;
}

/**
 * @brief Starts DMA-managed full-duplex data transfer over USART.
 * @param husart: pointer to the USART handle structure
 * @param TxData: pointer to the transmitted data buffer
 * @param RxData: pointer to the received data buffer
 * @param Length: amount of data transfers
 * @return BUSY if DMA is in use, OK if transfer is started
 */
XPD_ReturnType XPD_USART_TransmitReceive_DMA(USART_HandleType * husart, void * TxData, void * RxData, uint16_t Length)
{
    XPD_ReturnType result;

    /* save stream info */
    husart->TxStream.buffer = TxData;
    husart->TxStream.length = Length;
    husart->RxStream.buffer = RxData;
    husart->RxStream.length = Length;

    /* Set up DMAs for transfers */
    result = XPD_DMA_Start_IT(husart->DMA.Receive, (void*) &USART_RXDR(husart), RxData, Length);

    if (result == XPD_OK)
    {
        result = XPD_DMA_Start_IT(husart->DMA.Transmit, (void*) &USART_TXDR(husart), TxData, Length);

        /* If one DMA allocation failed, reset the other and exit */
        if (result != XPD_OK)
        {
            XPD_DMA_Stop_IT(husart->DMA.Receive);
            return result;
        }

        /* Set the callback owner */
        husart->DMA.Transmit->Owner = husart;
        husart->DMA.Receive->Owner  = husart;

        /* Set the DMA transfer callbacks */
        husart->DMA.Transmit->Callbacks.Complete     = usart_dmaTransmitRedirect;
        husart->DMA.Receive->Callbacks.Complete      = usart_dmaReceiveRedirect;
#ifdef USE_XPD_DMA_ERROR_DETECT
        husart->DMA.Transmit->Callbacks.Error        = usart_dmaErrorRedirect;
        husart->DMA.Receive->Callbacks.Error         = usart_dmaErrorRedirect;
#endif
        USART_RESET_ERRORS(husart);

        XPD_USART_ClearFlag(husart, ORE);
        XPD_USART_ClearFlag(husart, TC);

        USART_REG_BIT(husart, CR3, DMAR) = 1;
        USART_REG_BIT(husart, CR3, DMAT) = 1;
    }
    return result;
}

/**
 * @brief Stops all ongoing DMA-managed transfers over USART.
 * @param husart: pointer to the USART handle structure
 */
void XPD_USART_Stop_DMA(USART_HandleType * husart)
{
    /* Transmit DMA disable */
    if (USART_REG_BIT(husart,CR3,DMAT) != 0)
    {
        USART_REG_BIT(husart,CR3,DMAT) = 0;

        XPD_DMA_Stop_IT(husart->DMA.Transmit);
    }
    /* Receive DMA disable */
    if (USART_REG_BIT(husart,CR3,DMAR) != 0)
    {
        USART_REG_BIT(husart,CR3,DMAR) = 0;

        XPD_DMA_Stop_IT(husart->DMA.Receive);
    }
}

#if (USART_PERIPHERAL_VERSION > 1)
/**
 * @brief Sets the selected inversion configuration for the USART
 * @param husart: pointer to the USART handle structure
 * @param Inversions: the selected logical levels and order to invert
 */
void XPD_USART_InversionConfig(USART_HandleType * husart, USART_InversionType Inversions)
{
    USART_APPLY_CONFIG(husart, CR2,
            USART_CR2_TXINV | USART_CR2_RXINV | USART_CR2_DATAINV | USART_CR2_MSBFIRST | USART_CR2_SWAP,
            Inversions);
}

/**
 * @brief Sets the overrun detection configuration for the USART (enabled by default)
 * @param husart: pointer to the USART handle structure
 * @param Mode: the selected detection strategy
 */
void XPD_USART_OverrunConfig(USART_HandleType * husart, FunctionalState Mode)
{
    USART_APPLY_CONFIG(husart, CR3, USART_CR3_OVRDIS, (1 - Mode) << USART_CR3_OVRDIS_Pos);
}
#endif

/** @} */

/** @} */

/** @addtogroup UART
 * @{ */

/** @defgroup UART_Exported_Functions UART Exported Functions
 * @{ */

/**
 * @brief Initializes the USART peripheral in asynchronous mode using the setup configuration
 * @param husart: pointer to the USART handle structure
 * @param Common: General peripheral setup configuration
 * @param Config: UART setup configuration
 * @return ERROR if input is incorrect, OK if success
 */
XPD_ReturnType XPD_UART_Init(USART_HandleType * husart,
        const USART_InitType * Common, const UART_InitType * Config)
{
    XPD_ReturnType result = usart_init1(husart, Common);

    {
        /* configure UART specific settings */
        USART_REG_BIT(husart, CR1, OVER8) = Config->OverSampling8;
        USART_REG_BIT(husart, CR3, HDSEL) = Config->HalfDuplex;

        /* configure hardware flow control */
        husart->Inst->CR3.w |= (USART_CR3_RTSE | USART_CR3_CTSE) & ((uint32_t)Config->FlowControl << USART_CR3_RTSE_Pos);

        result = usart_init2(husart, Common);
    }

    return result;
}

#if (USART_PERIPHERAL_VERSION > 1)
/**
 * @brief Sets the baudrate detection strategy for the UART
 * @param husart: pointer to the USART handle structure
 * @param Mode: the selected detection strategy
 */
void XPD_UART_BaudrateModeConfig(USART_HandleType * husart, UART_BaudrateModeType Mode)
{
    USART_APPLY_CONFIG(husart, CR2, USART_CR2_ABREN | USART_CR2_ABRMODE, Mode);
}
#endif

/** @} */

/** @} */

/** @addtogroup USRT
 * @{ */

/** @defgroup USRT_Exported_Functions USRT Exported Functions
 * @{ */

/**
 * @brief Initializes the USART peripheral in synchronous mode using the setup configuration
 * @param husart: pointer to the USART handle structure
 * @param Common: General peripheral setup configuration
 * @param Config: USRT setup configuration
 * @return ERROR if input is incorrect, OK if success
 */
XPD_ReturnType XPD_USRT_Init(USART_HandleType * husart,
        const USART_InitType * Common, const USRT_InitType * Config)
{
    XPD_ReturnType result = usart_init1(husart, Common);

    {
        /* configure clock settings and stop mode */
        USART_REG_BIT(husart, CR1, OVER8) = ENABLE;
        USART_REG_BIT(husart, CR2, CLKEN) = ENABLE;
        USART_REG_BIT(husart, CR2, CPOL)  = Config->Polarity;
        USART_REG_BIT(husart, CR2, CPHA)  = Config->Phase;
        USART_REG_BIT(husart, CR2, LBCL)  = Config->LastBit;
        USART_REG_BIT(husart, CR1, TE)    = ENABLE;

        result = usart_init2(husart, Common);
    }

    return result;
}

/** @} */

/** @} */

/** @addtogroup LIN
 * @{ */

/** @defgroup LIN_Exported_Functions LIN Exported Functions
 * @{ */

/**
 * @brief Sets the UART peripheral in LIN protocol mode
 * @param husart: pointer to the USART handle structure
 * @param Common: General peripheral setup configuration
 * @param BreakSize: bit size of the break frame for detection [10..11]
 * @return ERROR if input is incorrect, OK if success
 */
XPD_ReturnType XPD_LIN_Init(USART_HandleType * husart, const USART_InitType * Common, uint8_t BreakSize)
{
    XPD_ReturnType result = usart_init1(husart, Common);

    {
        uint32_t bdl = (BreakSize > 10) ? USART_CR2_LBDL : 0;

        /* Set LIN mode, stop bits to 1, break length */
        husart->Inst->CR2.w = (husart->Inst->CR2.w & ~(USART_CR2_LBDL | USART_CR2_STOP)) | USART_CR2_LINEN | bdl;

        result = usart_init2(husart, Common);
    }

    return result;
}

/**
 * @brief Sends a BREAK frame
 * @param husart: pointer to the USART handle structure
 */
void XPD_LIN_SendBreak(USART_HandleType * husart)
{
#if (USART_PERIPHERAL_VERSION > 1)
    husart->Inst->RQR.w = USART_RQR_SBKRQ;
#else
    USART_REG_BIT(husart, CR1, SBK) = 1;
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
 * @param husart: pointer to the USART handle structure
 * @param Common: General peripheral setup configuration
 * @param Config: MultiSlave UART setup configuration
 * @return ERROR if input is incorrect, OK if success
 */
XPD_ReturnType XPD_MSUART_Init(USART_HandleType * husart,
        const USART_InitType * Common, const MSUART_InitType * Config)
{
    XPD_ReturnType result = usart_init1(husart, Common);

    {
        /* configure UART specific settings */
        USART_REG_BIT(husart, CR1, OVER8) = Config->OverSampling8;
        USART_REG_BIT(husart, CR3, HDSEL) = Config->HalfDuplex;

        /* configure multislave settings */
        USART_REG_BIT(husart, CR1, WAKE)  = Config->UnmuteMethod;
#if (USART_PERIPHERAL_VERSION > 2)
        husart->Inst->CR3.b.WUS           = Config->WakeUpMethod;
#endif

        if ((Config->UnmuteMethod == MSUART_UNMUTE_ADDRESSED)
#if (USART_PERIPHERAL_VERSION > 2)
         || (Config->WakeUpMethod == MSUART_WAKEUP_ADDRESSED)
#endif
         )
        {
            husart->Inst->CR2.b.ADD           = Config->Address;
#ifdef USART_CR2_ADDM7
            USART_REG_BIT(husart, CR2, ADDM7) = (uint32_t)(Config->Address == 7);
#endif
        }
        result = usart_init2(husart, Common);
    }

    return result;
}

/**
 * @brief Sets the Mute state for the UART
 * @param husart: pointer to the USART handle structure
 * @param NewState: Mute state to set
 */
void XPD_MSUART_MuteCtrl(USART_HandleType * husart, FunctionalState NewState)
{
#if (USART_PERIPHERAL_VERSION > 1)
    USART_REG_BIT(husart, CR1, MME) = NewState;

    if (NewState != DISABLE)
    {
        /* Send actual mute request */
        husart->Inst->RQR.w = USART_RQR_MMRQ;
    }
#else
    USART_REG_BIT(husart, CR1, RWU) = NewState;
#endif
}

#if (USART_PERIPHERAL_VERSION > 2)
/**
 * @brief Sets the Stop mode for the UART
 * @note  The UART is able to wake up the MCU from Stop 1 mode as long as UART clock is HSI or LSE.
 * @note  There shall be no ongoing transfer in the UART when Stop mode is entered.
 * @param husart: pointer to the USART handle structure
 * @param NewState: Mute state to set
 */
void XPD_MSUART_StopModeCtrl(USART_HandleType * husart, FunctionalState NewState)
{
    /* When enabling Stop mode: */
    if (NewState != DISABLE)
    {
        /* Enable RXNE interrupt when it is the wake up source */
        if(husart->Inst->CR3.b.WUS == MSUART_WAKEUP_DATA_RECEIVED)
        {
            XPD_USART_EnableIT(husart, RXNE);
        }
        /* Otherwise enable dedicated interrupt */
        else
        {
            XPD_USART_EnableIT(husart, WU);
        }
    }
    USART_REG_BIT(husart, CR1, UESM) = NewState;
}
#endif

/** @} */

/** @} */

/** @addtogroup SmartCard
 * @{ */

/** @defgroup SmartCard_Exported_Functions SmartCard Exported Functions
 * @{ */

/**
 * @brief Sets the UART peripheral in SmartCard mode
 * @param husart: pointer to the USART handle structure
 * @param Common: General peripheral setup configuration
 * @param Config: SmartCard setup configuration
 * @return ERROR if input is incorrect, OK if success
 */
XPD_ReturnType XPD_SmartCard_Init(USART_HandleType * husart,
        const USART_InitType * Common, const SmartCard_InitType * Config)
{
    XPD_ReturnType result = usart_init1(husart, Common);

    {
        /* Set Smartcard features, output clock */
        USART_REG_BIT(husart, CR3, SCEN) = ENABLE;
        USART_REG_BIT(husart, CR3, NACK) = Config->NACK;
        husart->Inst->GTPR.b.GT          = Config->GuardTime;
        husart->Inst->GTPR.b.PSC         = Config->Clock.Divider / 2;
        USART_REG_BIT(husart, CR2, CPOL) = Config->Clock.Polarity;
        USART_REG_BIT(husart, CR2, CPHA) = Config->Clock.Phase;

#if (USART_PERIPHERAL_VERSION > 1)
        husart->Inst->CR3.b.SCARCNT      = Config->AutoRetries;
        husart->Inst->RTOR.b.BLEN        = Config->BlockLength;

        if (Config->RxTimeout > 0)
        {
            USART_REG_BIT(husart, CR2, RTOEN) = ENABLE;
            husart->Inst->RTOR.b.RTO          = Config->RxTimeout;
        }
        else
        {
            USART_REG_BIT(husart, CR2, RTOEN) = DISABLE;
        }
#endif

        result = usart_init2(husart, Common);
    }

    return result;
}

/** @} */

/** @} */

/** @addtogroup IrDA
 * @{ */

/** @defgroup IrDA_Exported_Functions IrDA Exported Functions
 * @{ */

/**
 * @brief Sets the UART peripheral in IrDA mode
 * @param husart: pointer to the USART handle structure
 * @param Common: General peripheral setup configuration
 * @param Config: IrDA setup configuration
 * @return ERROR if input is incorrect, OK if success
 */
XPD_ReturnType XPD_IrDA_Init(USART_HandleType * husart,
        const USART_InitType * Common, const IrDA_InitType * Config)
{
    XPD_ReturnType result = usart_init1(husart, Common);

    {
        /* IrDA features configuration */
        husart->Inst->GTPR.b.PSC         = Config->Prescaler;
        husart->Inst->CR2.b.STOP         = 0;
        USART_REG_BIT(husart, CR3, IRLP) = Config->LowPowerMode;
        USART_REG_BIT(husart, CR3, IREN) = ENABLE;

        result = usart_init2(husart, Common);
    }

    return result;
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
 * @param husart: pointer to the USART handle structure
 * @param Common: General peripheral setup configuration
 * @param Config: RS485 setup configuration
 * @return ERROR if input is incorrect, OK if success
 */
XPD_ReturnType XPD_RS485_Init(USART_HandleType * husart,
        const USART_InitType * Common, const RS485_InitType * Config)
{
    XPD_ReturnType result = usart_init1(husart, Common);

    {
        /* configure UART specific settings */
        USART_REG_BIT(husart, CR1, OVER8) = Config->OverSampling8;
        USART_REG_BIT(husart, CR3, HDSEL) = Config->HalfDuplex;

        /* configure DE signal */
        USART_REG_BIT(husart, CR3, DEM)   = ENABLE;
        USART_REG_BIT(husart, CR3, DEP)   = Config->DE.Polarity;
        husart->Inst->CR1.b.DEAT          = Config->DE.AssertionTime;
        husart->Inst->CR1.b.DEDT          = Config->DE.DeassertionTime;

        result = usart_init2(husart, Common);
    }

    return result;
}

/** @} */

/** @} */
#endif

/** @} */

#endif /* USE_XPD_USART */
