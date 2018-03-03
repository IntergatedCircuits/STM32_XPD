/**
  ******************************************************************************
  * @file    xpd_flash.c
  * @author  Benedek Kupper
  * @version 0.3
  * @date    2018-01-28
  * @brief   STM32 eXtensible Peripheral Drivers Flash Module
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
#include <xpd_flash.h>
#include <xpd_utils.h>

/** @addtogroup FLASH
 * @{ */

#define FLASH_TIMEOUT_MS        50000 /* 50 s */

#ifdef FLASH_CR_MER2
#define FLASH_OPERATIONS        \
    (FLASH_OPERATION_ERASE_BLOCK | FLASH_OPERATION_ERASE_BANK1 \
   | FLASH_OPERATION_ERASE_BANK2 | FLASH_OPERATION_PROGRAM)
#else
#define FLASH_OPERATIONS        \
    (FLASH_OPERATION_ERASE_BLOCK | FLASH_OPERATION_ERASE_BANK1 | FLASH_OPERATION_PROGRAM)
#endif

#define FLASH_BANK1_ADDRESS     ((void*)FLASH_BASE)
/* Bank2 start address is at half of the range */
#define FLASH_BANK2_ADDRESS     ((void*)(FLASH_BASE + (DEVICE_FLASH_SIZE_kB << (10 - 1))))

#define FLASH_PAGE_SIZE         0x800

#define FLASH_BLOCK_SIZE_KB     (FLASH_PAGE_SIZE >> 10)

#ifdef FLASH_ECCR_ECCC2
#define FLASH_FAST_PROGRAM_SIZE (64 * sizeof(uint64_t))
#define FLASH_FAST_PROGRAM_MASK 0x01FF
#else
#define FLASH_FAST_PROGRAM_SIZE (32 * sizeof(uint64_t))
#define FLASH_FAST_PROGRAM_MASK 0x00FF
#endif

/* The size of a single programming operation */
#define FLASH_MEMSTREAM_SIZE    (sizeof(uint64_t))

/* Keys for FLASH operations unlocking */
#ifndef FLASH_KEY1
#define FLASH_KEY1              0x45670123
#endif
#ifndef FLASH_KEY2
#define FLASH_KEY2              0xCDEF89AB
#endif

/* Internal variable for context storage */
static struct
{
    void * Address;
    DataStreamType MemStream;
    uint8_t BkpACR;
    volatile FLASH_ErrorType Errors;
} flash_xHandle =
{
    .Errors         = FLASH_ERROR_NONE,
    .MemStream.size = FLASH_MEMSTREAM_SIZE,
};

FLASH_CallbacksType FLASH_xCallbacks = { NULL, NULL, NULL };

/* Programs a double word from the stream to the flash */
static void FLASH_prvWriteDWord(uint32_t * pulAddr, DataStreamType * pxStream)
{
    *pulAddr = *((uint32_t*)pxStream->buffer);
    pulAddr++;
    pxStream->buffer += sizeof(uint32_t);
    *pulAddr = *((uint32_t*)pxStream->buffer);
    pxStream->buffer += sizeof(uint32_t);
    pxStream->length -= 2;
}

/* Fast programming: if HCLK > 8MHz, in FLASH_FAST_PROGRAM_SIZE alignment */

/* Erase the next scheduled block */
static void FLASH_prvBlockErase(void)
{
#ifdef FLASH_CR_MER2
    /* If the dual bank is enabled on the device */
#if   defined (FLASH_OPTR_DBANK)
    if (FLASH_REG_BIT(OPTR,DBANK) != 0)
#elif defined (FLASH_OPTR_DUALBANK)
    if (FLASH_REG_BIT(OPTR,DUALBANK) != 0)
#endif
    {
        /* Select the bank number of the page to erase */
        if ((uint32_t)flash_xHandle.Address < (FLASH_BASE + (DEVICE_FLASH_SIZE_kB << (10 - 1))))
        {
            /* Bank1 */
            FLASH_REG_BIT(CR,BKER) = 0;
        }
        else
        {
            /* Bank2 */
            FLASH_REG_BIT(CR,BKER) = 1;
        }
    }
#endif

    /* Set the page address to erase */
    FLASH->CR.b.PNB = (uint32_t)flash_xHandle.Address >> 11;
    FLASH_REG_BIT(CR,STRT) = 1;
}

/* Read errors to context, and clear them in register */
static FLASH_ErrorType FLASH_prvCheckErrors(void)
{
    /* Read error flags */
    flash_xHandle.Errors = FLASH->SR.w & (FLASH_SR_WRPERR
            | FLASH_SR_RDERR   | FLASH_SR_RDERR
            | FLASH_SR_PGAERR  | FLASH_SR_PGSERR
            | FLASH_SR_FASTERR | FLASH_SR_MISERR
            | FLASH_SR_PROGERR | FLASH_SR_SIZERR
            | FLASH_SR_OPTVERR);

    /* Clear error flags */
    FLASH->SR.w = flash_xHandle.Errors;
    return flash_xHandle.Errors;
}

/* Disable the flash caches before erase */
static void FLASH_prvDisableCaches(void)
{
    /* Store backup of current cache settings, disable all */
    flash_xHandle.BkpACR = (FLASH->ACR.w & (FLASH_ACR_ICEN | FLASH_ACR_DCEN)) >> 8;
    CLEAR_BIT(FLASH->ACR.w, FLASH_ACR_ICEN | FLASH_ACR_DCEN);
}

/* Flush the flash caches after erase */
static void FLASH_prvFlushCaches(void)
{
    uint32_t ulEnMask = flash_xHandle.BkpACR << 8;
    uint32_t ulRstMask = ulEnMask << (FLASH_ACR_ICRST_Pos - FLASH_ACR_ICEN_Pos);

    /* Set and clear DCRST and ICRST for previously enabled caches */
    SET_BIT(FLASH->ACR.w, ulRstMask);
    CLEAR_BIT(FLASH->ACR.w, ulRstMask);

    /* Set and clear DCEN and ICEN for previously enabled caches */
    SET_BIT(FLASH->ACR.w, ulEnMask);
}

/** @addtogroup FLASH_Exported_Functions
 * @{ */

/**
 * @brief Unlock the FLASH control register access.
 */
void FLASH_vUnlock(void)
{
    if (FLASH_REG_BIT(CR,LOCK) != 0)
    {
        /* Authorize the FLASH Registers access */
        FLASH->KEYR = FLASH_KEY1;
        FLASH->KEYR = FLASH_KEY2;

        /* Clear any previous request bits */
        CLEAR_BIT(FLASH->CR.w, FLASH_OPERATIONS);
    }
}

/**
 * @brief Lock the FLASH control register access.
 */
void FLASH_vLock(void)
{
    /* Set the LOCK Bit to lock the FLASH Registers access */
    FLASH_REG_BIT(CR,LOCK) = 1;
}

/**
 * @brief Polls the status of the ongoing FLASH operation.
 * @param ulTimeout: the timeout in ms for the polling.
 * @return ERROR    if there were errors,
 *         TIMEOUT  if timed out,
 *         OK       if successful
 */
XPD_ReturnType FLASH_ePollStatus(uint32_t ulTimeout)
{
    /* Wait for the FLASH operation to complete by polling on BUSY flag to be reset.
     Even if the FLASH operation fails, the BUSY flag will be reset and an error
     flag will be set */
    XPD_ReturnType eResult = XPD_eWaitForDiff(&FLASH->SR.w,
            FLASH_SR_BSY, FLASH_SR_BSY, &ulTimeout);

    /* Check FLASH End of Operation flag  */
    if (FLASH_FLAG_STATUS(EOP) != 0)
    {
        /* Clear FLASH End of Operation pending bit */
        FLASH_FLAG_CLEAR(EOP);
    }

    if (FLASH_prvCheckErrors() != FLASH_ERROR_NONE)
    {
        eResult = XPD_ERROR;
    }

    return eResult;
}

/**
 * @brief Programs the input data to the specified flash address.
 * @note  The FLASH interface should be unlocked beforehand, and locked afterwards.
 * @note  If flash memory is not erased before programming,
 *        only 0 value is allowed to be written. Otherwise an error flag is set.
 * @param pvAddress: the start flash address to write
 * @param pucData: input data to program
 * @param usLength: amount of bytes to program
 * @return BUSY     if another operation is already ongoing,
 *         ERROR    if there were errors,
 *         TIMEOUT  if timed out,
 *         OK       if successful
 */
XPD_ReturnType FLASH_eProgram(void * pvAddress, const uint8_t * pucData, uint16_t usLength)
{
    /* Wait for last operation to be completed */
    XPD_ReturnType eResult = FLASH_ePollStatus(FLASH_TIMEOUT_MS);

    if (eResult == XPD_OK)
    {
        flash_xHandle.MemStream.buffer = (void*)pucData;
        flash_xHandle.MemStream.length = usLength / flash_xHandle.MemStream.size;
        flash_xHandle.Address = pvAddress;

        /* Disable caches before start */
        FLASH_prvDisableCaches();

        /* Enable flash programming */
        FLASH_REG_BIT(CR,PG) = 1;

        while (flash_xHandle.MemStream.length > 0)
        {
            FLASH_prvWriteDWord(flash_xHandle.Address, &flash_xHandle.MemStream);

            /* Wait for last operation to be completed */
            eResult = FLASH_ePollStatus(FLASH_TIMEOUT_MS);

            /* In case of error, stop flash programming */
            if (eResult != XPD_OK)
            {
                break;
            }

            /* Increment flash address */
            flash_xHandle.Address += flash_xHandle.MemStream.size;
        }

        /* Disable flash programming */
        FLASH_REG_BIT(CR,PG) = 0;

        /* Flush the caches to be sure of the data consistency */
        FLASH_prvFlushCaches();
    }

    return eResult;
}

/**
 * @brief Programs the input data to the specified flash address in interrupt (non-blocking) mode.
 * @note  The FLASH interface should be unlocked beforehand, and locked afterwards.
 * @note  If flash memory is not erased before programming,
 *        only 0 value is allowed to be written. Otherwise an error flag is set.
 * @param pvAddress: the start flash address to write
 * @param pucData: input data to program
 * @param usLength: amount of bytes to program
 * @return BUSY     if another operation is already ongoing,
 *         OK       if programming started
 */
XPD_ReturnType FLASH_eProgram_IT(void * pvAddress, const uint8_t * pucData, uint16_t usLength)
{
    XPD_ReturnType eResult = XPD_BUSY;

    /* Only start if no ongoing operations are present */
    if (FLASH_REG_BIT(SR,BSY) == 0)
    {
        flash_xHandle.MemStream.buffer = (void*)pucData;
        flash_xHandle.MemStream.length = usLength / flash_xHandle.MemStream.size;
        flash_xHandle.Address = pvAddress;

        /* Disable caches before start */
        FLASH_prvDisableCaches();

        /* Enable interrupts */
        SET_BIT(FLASH->CR.w, FLASH_CR_EOPIE | FLASH_CR_ERRIE);

        /* Enable flash programming */
        FLASH_REG_BIT(CR,PG) = 1;

        FLASH_prvWriteDWord(flash_xHandle.Address, &flash_xHandle.MemStream);

        eResult = XPD_OK;
    }
    return eResult;
}

/**
 * @brief Performs a mass erase on a flash memory bank.
 * @note  The FLASH interface should be unlocked beforehand, and locked afterwards.
 * @param Bank: The selected memory bank (1 or 2)
 * @return BUSY     if another operation is already ongoing,
 *         ERROR    if there were errors,
 *         TIMEOUT  if timed out,
 *         OK       if successful
 */
XPD_ReturnType FLASH_eEraseBank(uint8_t ucBank)
{
    /* Wait for last operation to be completed */
    XPD_ReturnType eResult = FLASH_ePollStatus(FLASH_TIMEOUT_MS);

    if (eResult == XPD_OK)
    {
        /* Disable caches before start */
        FLASH_prvDisableCaches();

#ifdef FLASH_CR_MER2
        if (ucBank == 2)
        {
#if   defined (FLASH_OPTR_DBANK)
            if (FLASH_REG_BIT(OPTR,DBANK) == 0)
            {
                /* Bank2 not available on this device */
                eResult = XPD_ERROR;
            }
            else
#elif defined (FLASH_OPTR_DUALBANK)
            if (FLASH_REG_BIT(OPTR,DUALBANK) == 0)
            {
                /* Bank2 not available on this device */
                eResult = XPD_ERROR;
            }
            else
#endif
            {
                /* Bank2 is selected */
                FLASH_REG_BIT(CR,MER2) = 1;
                FLASH_REG_BIT(CR,STRT) = 1;

                /* Bank2 start address is at half of the range */
                flash_xHandle.Address = FLASH_BANK2_ADDRESS;
                eResult = FLASH_ePollStatus(FLASH_TIMEOUT_MS);

                FLASH_REG_BIT(CR,MER2) = 0;
            }
        }
        else
#endif
        {
            /* Bank1 */
            FLASH_REG_BIT(CR,MER1) = 1;
            FLASH_REG_BIT(CR,STRT) = 1;

            flash_xHandle.Address = FLASH_BANK1_ADDRESS;
            eResult = FLASH_ePollStatus(FLASH_TIMEOUT_MS);

            FLASH_REG_BIT(CR,MER1) = 0;
        }

        /* Flush the caches to be sure of the data consistency */
        FLASH_prvFlushCaches();
    }

    return eResult;
}

/**
 * @brief Performs a mass erase on a flash memory bank in interrupt (non-blocking) mode.
 * @note  The FLASH interface should be unlocked beforehand, and locked afterwards.
 * @param Bank: The selected memory bank (1 or 2)
 * @return BUSY     if another operation is already ongoing,
 *         OK       if mass erase started
 */
XPD_ReturnType FLASH_eEraseBank_IT(uint8_t ucBank)
{
    XPD_ReturnType eResult = XPD_BUSY;

    /* Only start if no ongoing operations are present */
    if (FLASH_REG_BIT(SR,BSY) == 0)
    {
        /* Disable caches before start */
        FLASH_prvDisableCaches();

        /* Enable interrupts */
        SET_BIT(FLASH->CR.w, FLASH_CR_EOPIE | FLASH_CR_ERRIE);

#ifdef FLASH_CR_MER2
        if (ucBank == 2)
        {
#if   defined (FLASH_OPTR_DBANK)
            if (FLASH_REG_BIT(OPTR,DBANK) == 0)
            {
                /* Bank2 not available on this device */
                eResult = XPD_ERROR;
            }
            else
#elif defined (FLASH_OPTR_DUALBANK)
            if (FLASH_REG_BIT(OPTR,DUALBANK) == 0)
            {
                /* Bank2 not available on this device */
                eResult = XPD_ERROR;
            }
            else
#endif
            {
                /* Bank2 is selected */
                FLASH_REG_BIT(CR,MER2) = 1;
                FLASH_REG_BIT(CR,STRT) = 1;

                /* Bank2 start address is at half of the range */
                flash_xHandle.Address = FLASH_BANK2_ADDRESS;
            }
        }
        else
#endif
        {
            /* Bank1 */
            FLASH_REG_BIT(CR,MER1) = 1;
            FLASH_REG_BIT(CR,STRT) = 1;

            flash_xHandle.Address = FLASH_BANK1_ADDRESS;
        }
        eResult = XPD_OK;
    }

    return eResult;
}

/**
 * @brief Performs consecutive page erases in flash.
 * @note  The FLASH interface should be unlocked beforehand, and locked afterwards.
 * @param pvAddress: start address of first page to be erased
 * @param usKBytes: amount of flash memory to erase in kilobytes (one page is 2 kB)
 * @return BUSY     if another operation is already ongoing,
 *         ERROR    if there were errors,
 *         TIMEOUT  if timed out,
 *         OK       if successful
 */
XPD_ReturnType FLASH_eErase(void * pvAddress, uint16_t usKBytes)
{
    /* Wait for last operation to be completed */
    XPD_ReturnType eResult = FLASH_ePollStatus(FLASH_TIMEOUT_MS);

    if (eResult == XPD_OK)
    {
        flash_xHandle.Address = pvAddress;
        flash_xHandle.MemStream.length = usKBytes;

        /* Disable caches before start */
        FLASH_prvDisableCaches();

        /* Set the page erase control bit */
        FLASH_REG_BIT(CR,PER) = 1;

        /* Keep erasing blocks until at least the requested amount */
        while (flash_xHandle.MemStream.length > 0)
        {
            FLASH_prvBlockErase();

            /* Wait for last operation to be completed */
            eResult = FLASH_ePollStatus(FLASH_TIMEOUT_MS);

            /* In case of error or length underflow, stop flash programming */
            if ((eResult != XPD_OK) || (flash_xHandle.MemStream.length < FLASH_BLOCK_SIZE_KB))
            {
                break;
            }

            /* Increase flash address */
            flash_xHandle.Address += FLASH_BLOCK_SIZE_KB << 10;
            flash_xHandle.MemStream.length -= FLASH_BLOCK_SIZE_KB;
        }

        /* If the erase operation is completed, disable the PER Bit */
        FLASH_REG_BIT(CR,PER) = 0;

        /* Flush the caches to be sure of the data consistency */
        FLASH_prvFlushCaches();
    }

    return eResult;
}

/**
 * @brief Performs consecutive page erases in flash in interrupt (non-blocking) mode.
 * @note  The FLASH interface should be unlocked beforehand, and locked afterwards.
 * @param pvAddress: start address of first page to be erased
 * @param usKBytes: amount of flash memory to erase in kilobytes (one page is 2 kB)
 * @return BUSY     if another operation is already ongoing,
 *         OK       if page erase started
 */
XPD_ReturnType FLASH_eErase_IT(void * pvAddress, uint16_t usKBytes)
{
    XPD_ReturnType eResult = XPD_BUSY;

    /* Only start if no ongoing operations are present */
    if (FLASH_REG_BIT(SR,BSY) == 0)
    {
        flash_xHandle.Address = pvAddress;
        flash_xHandle.MemStream.length = usKBytes;

        /* Disable caches before start */
        FLASH_prvDisableCaches();

        /* Enable interrupts */
        SET_BIT(FLASH->CR.w, FLASH_CR_EOPIE | FLASH_CR_ERRIE);

        /* Set the page erase control bit */
        FLASH_REG_BIT(CR,PER) = 1;

        FLASH_prvBlockErase();

        eResult = XPD_OK;
    }

    return eResult;
}

/**
 * @brief FLASH interrupt handler that manages consecutive block erasing
 *        and programming, and provides completion and error callbacks.
 */
void FLASH_vIRQHandler(void)
{
    /* Check FLASH error flags */
    if (FLASH_prvCheckErrors() != FLASH_ERROR_NONE)
    {
        /* Stop programming */
        flash_xHandle.MemStream.length = 0;
        CLEAR_BIT(FLASH->CR.w, FLASH_OPERATIONS);

        /* Error callback */
        XPD_SAFE_CALLBACK(FLASH_xCallbacks.Error,);
    }

    /* Check FLASH End of Operation flag  */
    if (FLASH_FLAG_STATUS(EOP) != 0)
    {
        FLASH_OperationType eOperation = (FLASH->CR.w & FLASH_OPERATIONS);

        /* Clear FLASH End of Operation pending bit */
        FLASH_FLAG_CLEAR(EOP);

        switch (eOperation)
        {
            case FLASH_OPERATION_PROGRAM:
            {
                /* Check if there are still data to program */
                if (flash_xHandle.MemStream.length > 0)
                {
                    /* Continue with programming consecutive pages */
                    flash_xHandle.Address += flash_xHandle.MemStream.size;

                    FLASH_prvWriteDWord(flash_xHandle.Address, &flash_xHandle.MemStream);
                }
                else
                {
                    /* If the operation completed, disable its control bit */
                    FLASH_REG_BIT(CR,PG) = 0;

                    /* provide completion callback */
                    XPD_SAFE_CALLBACK(FLASH_xCallbacks.ProgramComplete,);
                }
                break;
            }

            case FLASH_OPERATION_ERASE_BLOCK:
            {
                if (flash_xHandle.MemStream.length > FLASH_BLOCK_SIZE_KB)
                {
                    /* Continue with erasing consecutive blocks */
                    flash_xHandle.Address += FLASH_BLOCK_SIZE_KB << 10;
                    flash_xHandle.MemStream.length -= FLASH_BLOCK_SIZE_KB;
                    FLASH_prvBlockErase();
                }
                else
                {
                    /* If the operation completed, disable its control bit */
                    FLASH_REG_BIT(CR,PER) = 0;

                    /* provide completion callback */
                    XPD_SAFE_CALLBACK(FLASH_xCallbacks.EraseComplete,);
                }
                break;
            }

            /* FLASH_OPERATION_ERASE_BANK */
            default:
            {
                /* If the operation completed, disable its control bit */
                CLEAR_BIT(FLASH->CR.w, eOperation);

                /* provide completion callback */
                XPD_SAFE_CALLBACK(FLASH_xCallbacks.EraseComplete,);
                break;
            }
        }
    }

    /* No more pending operations, disable further interrupts */
    if ((FLASH->CR.w & FLASH_OPERATIONS) == FLASH_OPERATION_NONE)
    {
        CLEAR_BIT(FLASH->CR.w, FLASH_CR_EOPIE | FLASH_CR_ERRIE);

        /* Flush the caches to be sure of the data consistency */
        FLASH_prvFlushCaches();
    }
}

/**
 * @brief Provides the errors which have occurred during the last operation.
 * @return The occurred errors
 */
FLASH_ErrorType FLASH_eGetError(void)
{
    return flash_xHandle.Errors;
}

/**
 * @brief Provides the current address where the last flash operation was attempted.
 * @return The current address
 */
void * FLASH_pvGetAddress(void)
{
    return flash_xHandle.Address;
}

/** @} */

/** @} */
