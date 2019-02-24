/**
  ******************************************************************************
  * @file    xpd_i2c.c
  * @author  Benedek Kupper
  * @version 0.1
  * @date    2018-06-21
  * @brief   STM32 eXtensible Peripheral Drivers I2C Module
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
#include <xpd_i2c.h>
#include <xpd_utils.h>

/** @addtogroup I2C
 * @{ */

typedef enum
{
    I2C_AUTOSTOP             = I2C_CR2_AUTOEND,
    I2C_STOP                 = I2C_CR2_STOP,
    I2C_START_WRITE          = I2C_CR2_START,
    I2C_START_WRITE_AUTOSTOP = I2C_CR2_START | I2C_CR2_AUTOEND,
    I2C_START_READ           = I2C_CR2_START | I2C_CR2_RD_WRN,
    I2C_START_READ_AUTOSTOP  = I2C_CR2_START | I2C_CR2_AUTOEND | I2C_CR2_RD_WRN,
}I2C_RequestType;

#define I2C_RESET_ERRORS(HANDLE)    ((HANDLE)->Errors = 0)

#ifdef __XPD_I2C_ERROR_DETECT
#define I2C_ERR_ITS                 (I2C_CR1_ERRIE)
#define I2C_MASTER_RX_ITS           (I2C_CR1_RXIE | I2C_CR1_TCIE | I2C_CR1_STOPIE | I2C_CR1_ERRIE)
#define I2C_MASTER_TX_ITS           (I2C_CR1_TXIE | I2C_CR1_TCIE | I2C_CR1_STOPIE | I2C_CR1_ERRIE)
#else
#define I2C_ERR_ITS                 (0)
#define I2C_MASTER_RX_ITS           (I2C_CR1_RXIE | I2C_CR1_TCIE | I2C_CR1_STOPIE)
#define I2C_MASTER_TX_ITS           (I2C_CR1_TXIE | I2C_CR1_TCIE | I2C_CR1_STOPIE)
#endif
#define I2C_SLAVE_ITS               (I2C_CR1_ADDRIE)

#define I2C_NBYTES_MASK             (I2C_CR2_NBYTES_Msk >> I2C_CR2_NBYTES_Pos)

#define I2C_ERROR_FLAGS             (I2C_ISR_BERR | I2C_ISR_ARLO | I2C_ISR_OVR |\
                                     I2C_ISR_TIMEOUT)

/* Returns the correct status flag to wait for, depending on the transfer setup */
static uint32_t I2C_prvMasterEndFlag(I2C_HandleType* pxI2C)
{
    return (I2C_REG_BIT(pxI2C, CR2, RELOAD) != 0) ? I2C_ISR_TCR : I2C_ISR_STOPF;
}

/* Enables the peripheral */
__STATIC_INLINE void I2C_prvEnable(I2C_HandleType* pxI2C)
{
    I2C_REG_BIT(pxI2C, CR1, PE) = 1;
}

/* Disables the peripheral */
__STATIC_INLINE void I2C_prvDisable(I2C_HandleType* pxI2C)
{
    I2C_REG_BIT(pxI2C, CR1, PE) = 0;
}

static void I2C_prvSetTiming(I2C_HandleType * pxI2C, uint32_t ulBusFreq_Hz)
{
    uint32_t ulClkFreq_Hz = I2C_ulClockFreq_Hz(pxI2C);
    uint32_t ulScl, ulOvers, ulPresc = 0, ulTSetUp = 1, ulDiv, ulMulL;

    /* Standard mode */
    if (ulBusFreq_Hz <= 100000)
    {
        /* Thigh = Tlow */
        ulDiv = 2;
        ulMulL = 1;
    }
    else /* Fast mode */
    {
        /* Thigh = Tlow / 2 */
        ulDiv = 3;
        ulMulL = 2;
    }

    ulOvers = (ulClkFreq_Hz / ulBusFreq_Hz - 4) / ulDiv;
    do
    {
        ulPresc++;
        ulScl = ulOvers / ulPresc;
    }
    while (ulScl > 0x40);

    pxI2C->Inst->TIMINGR.b.PRESC  = ulPresc - 1;
    pxI2C->Inst->TIMINGR.b.SCLL   = (ulScl * ulMulL) - 1;

#if 0 /* TODO proper implementation */
    {
        uint32_t ulOffset = 16, ulScaler = 8, ulLimit = 15;

        if (ulScl > (ulOffset + (ulLimit * ulScaler)))
        {
            ulTSetUp = ulLimit;
            ulScl -= ulTSetUp;
        }
        else if (ulScl >= (ulOffset + (2 * ulScaler)))
        {
            ulTSetUp = (ulScl - ulOffset) / ulScaler;
            ulScl -= ulTSetUp;
        }
    }
#endif

    pxI2C->Inst->TIMINGR.b.SCLH   = ulScl - 1;
    pxI2C->Inst->TIMINGR.b.SCLDEL = ulTSetUp - 1;
    pxI2C->Inst->TIMINGR.b.SDADEL = 0;
}

/* Clears the TXIS flag and subsequently the TXDR */
static void I2C_prvFlushTx(I2C_HandleType* pxI2C)
{
    /* If a pending TXIS flag is set, write a dummy byte to TXDR to clear it */
    if (I2C_FLAG_STATUS(pxI2C, TXIS) != 0)
    {
        pxI2C->Inst->TXDR = 0;

        /* Afterwards flush the dummy TX data from the register */
        I2C_FLAG_CLEAR(pxI2C, TXE);
    }
}

/* Waits for a data flag or a NACK flag, also handles the NACK */
static XPD_ReturnType I2C_prvWaitForFlagOrNACK(I2C_HandleType* pxI2C, uint32_t ulFlag, uint32_t* pulTimeout)
{
    XPD_ReturnType eResult = XPD_eWaitForDiff(&pxI2C->Inst->ISR.w,
            ulFlag | I2C_ISR_NACKF, 0, pulTimeout);

    if (eResult == XPD_OK)
    {
        /* Check which flag was set */
        if (I2C_FLAG_STATUS(pxI2C, NACK) != 0)
        {
            pxI2C->Errors = I2C_ERROR_NACK;
            eResult = XPD_ERROR;

            /* After a NACK, STOP is either sent by the master, or auto-generated in master mode */
            if (XPD_OK == XPD_eWaitForDiff(&pxI2C->Inst->ISR.w, I2C_ISR_STOPF, 0, pulTimeout))
            {
                /* Clear NACKF and STOPF */
                pxI2C->Inst->ICR.w = I2C_ICR_NACKCF | I2C_ICR_STOPCF;

                /* Flush TX register */
                I2C_prvFlushTx(pxI2C);

                /* Clear master transfer */
                I2C_prvClearTransfer(pxI2C);
            }
        }
    }
    return eResult;
}

/* Feeds a ready stream to the TX register */
static XPD_ReturnType I2C_prvWriteStream(I2C_HandleType* pxI2C, uint32_t ulEndFlag, uint32_t* pulTimeout)
{
    XPD_ReturnType eResult;

    while (pxI2C->Stream.size > 0)
    {
        /* Wait until TXIS flag is set */
        eResult = I2C_prvWaitForFlagOrNACK(pxI2C, I2C_ISR_TXIS, pulTimeout);
        if (eResult != XPD_OK)
        {
            return eResult;
        }

        /* Write data to TXDR */
        pxI2C->Inst->TXDR = *(uint8_t*)pxI2C->Stream.buffer++;
        pxI2C->Stream.size--;
    }

    /* Finally wait for transfer complete / STOP */
    eResult = XPD_eWaitForMatch(&pxI2C->Inst->ISR.w, ulEndFlag, ulEndFlag, pulTimeout);

    return eResult;
}

/* Fetches the RX register to the pending stream */
static XPD_ReturnType I2C_prvReadStream(I2C_HandleType* pxI2C, uint32_t ulEndFlag, uint32_t* pulTimeout)
{
    XPD_ReturnType eResult;

    while (pxI2C->Stream.size > 0)
    {
        /* Wait until RXNE flag is set */
        eResult = I2C_prvWaitForFlagOrNACK(pxI2C, I2C_ISR_RXNE, pulTimeout);
        if (eResult != XPD_OK)
        {
            return eResult;
        }

        /* Read data from RXDR */
        *(uint8_t*)pxI2C->Stream.buffer++ = pxI2C->Inst->RXDR;
        pxI2C->Stream.size--;
    }

    /* Finally wait for transfer complete / STOP */
    eResult = XPD_eWaitForMatch(&pxI2C->Inst->ISR.w, ulEndFlag, ulEndFlag, pulTimeout);

    return eResult;
}

/* Clears all master mode transfer control bits */
static void I2C_prvClearTransfer(I2C_HandleType* pxI2C)
{
    CLEAR_BIT(pxI2C->Inst->CR2.w, I2C_CR2_NBYTES | I2C_CR2_RELOAD | I2C_CR2_RD_WRN);
}

/* Configures the peripheral for a master data transfer */
static void I2C_prvSetTransfer(I2C_HandleType* pxI2C, I2C_RequestType eRequest, uint16_t usLength)
{
    pxI2C->Stream.size = usLength;
    /* If size is too large for complete transfer, change from autoend to reload mode */
    if (pxI2C->Stream.size > I2C_NBYTES_MASK)
    {
        eRequest ^= I2C_CR2_AUTOEND | I2C_CR2_RELOAD;
        pxI2C->Stream.size = I2C_NBYTES_MASK;
    }
    pxI2C->Stream.length = usLength - pxI2C->Stream.size;
    pxI2C->Inst->CR2.w = (pxI2C->Inst->CR2.w &
        ~(I2C_CR2_NBYTES | I2C_CR2_RELOAD | I2C_CR2_AUTOEND |
          I2C_CR2_RD_WRN | I2C_CR2_START | I2C_CR2_STOP)) |
          (pxI2C->Stream.size << I2C_CR2_NBYTES_Pos) | eRequest;
}

/* Sets the handle context to transfer the command */
static void I2C_prvMasterSetCmdStage(I2C_HandleType * pxI2C)
{
    /* Set stream context to command */
    pxI2C->Stream.buffer = &pxI2C->Transfers.Master->Cmd;
    I2C_prvSetTransfer(pxI2C, I2C_START_WRITE, pxI2C->Transfers.Master->CmdSize);
}

/* Sets the handle context to transfer the data */
static void I2C_prvMasterSetDataStage(I2C_HandleType * pxI2C)
{
    I2C_RequestType eRequest;

    if (pxI2C->Transfers.Master->Direction == I2C_DIRECTION_WRITE)
    {
        eRequest = I2C_START_WRITE_AUTOSTOP;
    }
    else
    {
        eRequest = I2C_START_READ_AUTOSTOP;
    }

    /* Set stream context to data */
    pxI2C->Stream.buffer = pxI2C->Transfers.Master->Data;
    I2C_prvSetTransfer(pxI2C, eRequest, pxI2C->Transfers.Master->Length);
}

/* Master mode EV signals interrupt handler */
static void I2C_prvMasterIRQHandler_IT(I2C_HandleType * pxI2C)
{
    /* Interrupt enable and status bits are at the same position */
    uint32_t ulCR1 = pxI2C->Inst->CR1.w;
    uint32_t ulISR = pxI2C->Inst->ISR.w;
    uint32_t ulIT = ulCR1 & ulISR;

    /* Transmit register empty */
    if ((ulIT & I2C_ISR_TXIS) != 0)
    {
        /* Write data to TXDR */
        pxI2C->Inst->TXDR = *(uint8_t*)pxI2C->Stream.buffer++;
        pxI2C->Stream.size--;
    }
    /* Receive register empty */
    else if ((ulIT & I2C_ISR_RXNE) != 0)
    {
        /* Read data from RXDR */
        *(uint8_t*)pxI2C->Stream.buffer++ = pxI2C->Inst->RXDR;
        pxI2C->Stream.size--;
    }
    /* Transfer complete */
    else if ((ulIT & I2C_ISR_TC) != 0)
    {
        /* Move on from CMD to DATA stage */
        I2C_prvMasterSetDataStage(pxI2C);

        /* Change direction if necessary */
        if (pxI2C->Transfers.Master->Direction == I2C_DIRECTION_READ)
        {
            pxI2C->Inst->CR1.w = ulCR1 ^ (I2C_CR1_RXIE | I2C_CR1_TXIE);
        }
    }
    /* Transfer reload complete */
    else if (((ulISR & I2C_ISR_TCR) != 0) && ((ulCR1 & I2C_CR1_TCIE) != 0))
    {
        /* Request next NBYTES data transfer */
        if (pxI2C->Stream.length > 0)
        {
            I2C_prvSetTransfer(pxI2C, I2C_AUTOSTOP, pxI2C->Stream.length);
        }
    }
    /* STOP */
    if ((ulIT & I2C_ISR_STOPF) != 0)
    {
        I2C_FLAG_CLEAR(pxI2C, STOP);

        /* Clear master transfer */
        I2C_prvClearTransfer(pxI2C);

        /* Disable interrupts as transfer is over */
        pxI2C->Inst->CR1.w = ulCR1 & ~(I2C_MASTER_TX_ITS | I2C_MASTER_RX_ITS);

        if ((ulCR1 & I2C_CR1_ADDRIE) != 0)
        {
            /* TODO Switch to slave IRQHandler if address is listened to */
        }
        else
        {
            pxI2C->IRQHandler = NULL;
        }

        /* Check if NACK error triggered it */
        if ((ulISR & I2C_ISR_NACKF) != 0)
        {
            I2C_FLAG_CLEAR(pxI2C, NACK);

            pxI2C->Errors |= I2C_ERROR_NACK;

            /* Flush TX register */
            I2C_prvFlushTx(pxI2C);

            XPD_SAFE_CALLBACK(pxI2C->Callbacks.Error, pxI2C);
        }
        else
        {
            XPD_SAFE_CALLBACK(pxI2C->Callbacks.MasterComplete, pxI2C);
        }
    }
}

/** @defgroup I2C_Exported_Functions I2C Exported Functions
 * @{ */

/**
 * @brief Initializes the I2C peripheral using the setup configuration
 * @param pxI2C: pointer to the I2C handle structure
 * @param pxConfig: I2C master setup configuration
 */
void I2C_vInit(I2C_HandleType * pxI2C, const I2C_InitType * pxConfig)
{
    /* Enable clock */
    RCC_vClockEnable(pxI2C->CtrlPos);

    /* disable peripheral */
    pxI2C->Inst->CR1.w = 0;

    I2C_prvSetTiming(pxI2C, pxConfig->BusFreq_Hz);

    /* Slave configuration */
    I2C_REG_BIT(pxI2C, CR1, GCEN) = pxConfig->GeneralCall;
    I2C_REG_BIT(pxI2C, CR1, NOSTRETCH) = pxConfig->NoStretch;

    /* Set Address 1 */
    if (pxConfig->OwnAddress1.wValue == 0)
    {
        pxI2C->Inst->OAR1.w = 0;
    }
    else
    {
        pxI2C->Inst->OAR1.w = pxConfig->AddressingMode | pxConfig->OwnAddress1.wValue;
    }

    if (pxConfig->AddressingMode == I2C_ADDRESS_10BIT)
    {
        /* Master addressing mode */
        pxI2C->Inst->CR2.w = I2C_CR2_ADD10;
    }
    else
    {
        pxI2C->Inst->CR2.w = 0;

        /* Slave Address 2 is only used with 7 bit addressing */
        if (pxConfig->OwnAddress2.wValue == 0)
        {
            pxI2C->Inst->OAR2.w = 0;
        }
        else
        {
            pxI2C->Inst->OAR2.w = I2C_OAR2_OA2EN | pxConfig->OwnAddress2.wValue;
        }
    }

    pxI2C->IRQHandler = NULL;
    I2C_prvEnable(pxI2C);

    /* Dependencies initialization */
    XPD_SAFE_CALLBACK(pxI2C->Callbacks.DepInit, pxI2C);
}

/**
 * @brief Restores the I2C peripheral to its default inactive state
 * @param pxI2C: pointer to the I2C handle structure
 */
void I2C_vDeinit(I2C_HandleType * pxI2C)
{
    I2C_prvDisable(pxI2C);

	/* Deinitialize peripheral dependencies */
    XPD_SAFE_CALLBACK(pxI2C->Callbacks.DepDeinit, pxI2C);

    /* Disable clock */
    RCC_vClockDisable(pxI2C->CtrlPos);
}

/**
 * @brief Determines the current status of I2C bus.
 * @param pxI2C: pointer to the I2C handle structure
 * @return BUSY if a transfer is in progress, OK if I2C bus is idle
 */
XPD_ReturnType I2C_eGetStatus(I2C_HandleType * pxI2C)
{
    return (I2C_FLAG_STATUS(pxI2C, BUSY) != 0) ? XPD_BUSY : XPD_OK;
}

/**
 * @brief Polls the status of the I2C bus.
 * @param pxI2C: pointer to the I2C handle structure
 * @param ulTimeout: the timeout in ms for the polling.
 * @return TIMEOUT if timed out, OK if successful
 */
XPD_ReturnType I2C_ePollStatus(I2C_HandleType * pxI2C, uint32_t ulTimeout)
{
    XPD_ReturnType eResult = XPD_eWaitForMatch(&pxI2C->Inst->ISR.w, I2C_ISR_BUSY, 0, &ulTimeout);

    return eResult;
}

/**
 * @brief Performs an I2C transfer as master.
 * @param pxI2C: pointer to the I2C handle structure
 * @param pxTransfer: transfer context reference (its Direction field determines the data transfer's direction)
 * @return TIMEOUT if timed out, ERROR if the transfer was NACK-ed prematurely, OK if successful
 */
XPD_ReturnType I2C_eMasterTransfer(I2C_HandleType * pxI2C, const I2C_TransferType * pxTransfer, uint32_t ulTimeout)
{
    XPD_ReturnType eResult = XPD_OK;
    XPD_ReturnType (*pxTransferStream)(I2C_HandleType*, uint32_t, uint32_t*);
    I2C_RequestType eRequest;

    I2C_RESET_ERRORS(pxI2C);

    /* Initialize: set transfer, slave address */
    pxI2C->Transfers.Master = (void*)pxTransfer;
    pxI2C->Inst->CR2.b.SADD = pxTransfer->SlaveAddress_10bit;

    if (pxTransfer->CmdSize > 0)
    {
        I2C_prvMasterSetCmdStage(pxI2C);

        eResult = I2C_prvWriteStream(pxI2C, I2C_ISR_TC, &ulTimeout);
    }

    if (eResult == XPD_OK)
    {
        if (pxTransfer->Direction == I2C_DIRECTION_WRITE)
        {
            eRequest = I2C_START_WRITE_AUTOSTOP;
            pxTransferStream = I2C_prvWriteStream;
        }
        else
        {
            eRequest = I2C_START_READ_AUTOSTOP;
            pxTransferStream = I2C_prvReadStream;
        }

        /* Set stream context to data */
        pxI2C->Stream.buffer = pxI2C->Transfers.Master->Data;
        pxI2C->Stream.length = pxI2C->Transfers.Master->Length;

        do
        {
            /* Send (re)START and data with auto-end */
            I2C_prvSetTransfer(pxI2C, eRequest, pxI2C->Stream.length);
            eRequest = I2C_AUTOSTOP;

            eResult = pxTransferStream(pxI2C, I2C_prvMasterEndFlag(pxI2C), &ulTimeout);
        }
        while ((eResult == XPD_OK) && (pxI2C->Stream.length > 0));

        /* If the transfer is over, clear it */
        if (I2C_FLAG_STATUS(pxI2C, STOP) != 0)
        {
            I2C_FLAG_CLEAR(pxI2C, STOP);

            I2C_prvClearTransfer(pxI2C);
        }
    }

    return eResult;
}

/**
 * @brief Performs an I2C transfer as master using the EV interrupt stack.
 * @param pxI2C: pointer to the I2C handle structure
 * @param pxTransfer: transfer context reference (its Direction field determines the data transfer's direction)
 */
void I2C_vMasterTransfer_IT(I2C_HandleType * pxI2C, const I2C_TransferType * pxTransfer)
{
    uint32_t ulITs = I2C_MASTER_TX_ITS;
    I2C_RESET_ERRORS(pxI2C);

    /* Initialize: set transfer, slave address */
    pxI2C->Transfers.Master = (void*)pxTransfer;
    pxI2C->Inst->CR2.b.SADD = pxTransfer->SlaveAddress_10bit;

    if (pxTransfer->CmdSize > 0)
    {
        I2C_prvMasterSetCmdStage(pxI2C);
    }
    else
    {
        I2C_prvMasterSetDataStage(pxI2C);

        if (pxTransfer->Direction == I2C_DIRECTION_READ)
        {
            ulITs = I2C_MASTER_RX_ITS;
        }
    }

    /* Enable related interrupts */
    pxI2C->IRQHandler = (XPD_HandleCallbackType)I2C_prvMasterIRQHandler_IT;
    SET_BIT(pxI2C->Inst->CR1.w, ulITs);
}

/**
 * @brief I2C event interrupt handler that provides handle callbacks.
 * @param pxI2C: pointer to the I2C handle structure
 */
void I2C_vIRQHandler_EV(I2C_HandleType * pxI2C)
{
    XPD_HandleCallbackType pxHandler = pxI2C->IRQHandler;

    XPD_SAFE_CALLBACK(pxHandler, pxI2C);
}

/**
 * @brief I2C error interrupt handler that provides handle error callback.
 * @param pxI2C: pointer to the I2C handle structure
 */
void I2C_vIRQHandler_ER(I2C_HandleType * pxI2C)
{
    if (I2C_REG_BIT(pxI2C, CR1, ERRIE) != 0)
    {
        uint32_t ulISR = pxI2C->Inst->ISR.w;
        I2C_ErrorType prev = pxI2C->Errors;

        if ((ulISR & I2C_ISR_BERR) != 0)
        {
            I2C_FLAG_CLEAR(pxI2C, BERR);
            pxI2C->Errors |= I2C_ERROR_BUS;
        }
        if ((ulISR & I2C_ISR_OVR) != 0)
        {
            I2C_FLAG_CLEAR(pxI2C, OVR);
            pxI2C->Errors |= I2C_ERROR_OVERRUN;
        }
        if ((ulISR & I2C_ISR_ARLO) != 0)
        {
            I2C_FLAG_CLEAR(pxI2C, ARLO);
            pxI2C->Errors |= I2C_ERROR_ARBIT;

            if (I2C_REG_BIT(pxI2C, CR1, ADDRIE) != 0)
            {
                /* TODO Switch to slave IRQHandler if address is listened to */
            }
        }

        if (pxI2C->Errors != prev)
        {
            XPD_SAFE_CALLBACK(pxI2C->Callbacks.Error, pxI2C);
        }
    }
}

/** @} */

/** @} */
