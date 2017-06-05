/**
  ******************************************************************************
  * @file    xpd_flash.c
  * @author  Benedek Kupper
  * @version V0.2
  * @date    2016-06-05
  * @brief   STM32 eXtensible Peripheral Drivers Flash Module
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
#include "xpd_flash.h"
#include "xpd_utils.h"

#ifdef USE_XPD_FLASH

/** @addtogroup FLASH
 * @{ */

#define FLASH_TIMEOUT_MS        50000 /* 50 s */

#define hflash                  (&xpd_flashHandle)

#define FLASH_GETOPERATION()    (FLASH->CR.w & (FLASH_CR_PER | FLASH_CR_MER | FLASH_CR_PG))

#define FLASH_PAGE_SIZE         0x800
#define FLASH_BLOCK_SIZE(ADDR)  (FLASH_PAGE_SIZE)

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
    DataStreamType  MemStream;
    volatile FLASH_ErrorType Errors;
} xpd_flashHandle =
{
        .Errors         = FLASH_ERROR_NONE,
        .MemStream.size = sizeof(uint16_t),
};

/* Erase the next scheduled block */
static void flash_blockErase(void)
{
    /* Proceed to erase the page */
    FLASH_REG_BIT(CR,PER) = 1;

    FLASH->AR = (uint32_t)hflash->Address;

    FLASH_REG_BIT(CR,STRT) = 1;
}

/* Read errors to context, and clear them in register */
void flash_checkErrors(void)
{
    /* Read error flags */
    hflash->Errors = FLASH->SR.w & (FLASH_SR_PGERR | FLASH_SR_WRPERR);

    /* Clear error flags */
    FLASH->SR.w = hflash->Errors;
}

/** @addtogroup FLASH_Exported_Functions
 * @{ */

/**
 * @brief Unlock the FLASH control register access.
 */
void XPD_FLASH_Unlock(void)
{
    if (FLASH_REG_BIT(CR,LOCK) != 0)
    {
        /* Authorize the FLASH Registers access */
        FLASH->KEYR = FLASH_KEY1;
        FLASH->KEYR = FLASH_KEY2;
    }
    CLEAR_BIT(FLASH->CR.w, FLASH_CR_PER | FLASH_CR_MER | FLASH_CR_PG);
}

/**
 * @brief Lock the FLASH control register access.
 */
void XPD_FLASH_Lock(void)
{
    /* Set the LOCK Bit to lock the FLASH Registers access */
    FLASH_REG_BIT(CR,LOCK) = 1;
}

/**
 * @brief Polls the status of the ongoing FLASH operation.
 * @param Timeout: the timeout in ms for the polling.
 * @return ERROR    if there were errors,
 *         TIMEOUT  if timed out,
 *         OK       if successful
 */
XPD_ReturnType XPD_FLASH_PollStatus(uint32_t Timeout)
{
    /* Wait for the FLASH operation to complete by polling on BUSY flag to be reset.
     Even if the FLASH operation fails, the BUSY flag will be reset and an error
     flag will be set */
    XPD_ReturnType result = XPD_WaitForDiff(&FLASH->SR.w,
            FLASH_SR_BSY, FLASH_SR_BSY, &Timeout);

    /* Check FLASH End of Operation flag  */
    if (XPD_FLASH_GetFlag(EOP) != 0)
    {
        /* Clear FLASH End of Operation pending bit */
        XPD_FLASH_ClearFlag(EOP);
    }

    flash_checkErrors();
    if (hflash->Errors != FLASH_ERROR_NONE)
    {
        result = XPD_ERROR;
    }

    return result;
}

/**
 * @brief Programs the input data to the specified flash address.
 * @note  The FLASH interface should be unlocked beforehand, and locked afterwards.
 * @note  If flash memory is not erased before programming, only 1 -> 0 bit transitions
 *        will be written.
 * @param Address: the start flash address to write
 * @param Data: input data to program
 * @param Length: amount of bytes to program
 * @return BUSY     if another operation is already ongoing,
 *         ERROR    if there were errors,
 *         TIMEOUT  if timed out,
 *         OK       if successful
 */
XPD_ReturnType XPD_FLASH_Program(void * Address, const void * Data, uint16_t Length)
{
    /* Wait for last operation to be completed */
    XPD_ReturnType result = XPD_FLASH_PollStatus(FLASH_TIMEOUT_MS);

    if (result == XPD_OK)
    {
        hflash->MemStream.buffer = (void*)Data;
        hflash->MemStream.length = Length / hflash->MemStream.size;
        hflash->Address = Address;

        while (hflash->MemStream.length > 0)
        {
            /* Enable flash programming */
            FLASH_REG_BIT(CR,PG) = 1;

            XPD_WriteFromStream(hflash->Address, &hflash->MemStream);

            /* Wait for last operation to be completed */
            result = XPD_FLASH_PollStatus(FLASH_TIMEOUT_MS);

            /* Disable flash programming */
            FLASH_REG_BIT(CR,PG) = 0;

            /* In case of error, stop flash programming */
            if (result != XPD_OK)
            {
                break;
            }

            /* Increment flash address */
            hflash->Address += hflash->MemStream.size;
        }

    }

    return result;
}

/**
 * @brief Programs the input data to the specified flash address in interrupt (non-blocking) mode.
 * @note  The FLASH interface should be unlocked beforehand, and locked afterwards.
 * @note  If flash memory is not erased before programming, only 1 -> 0 bit transitions
 *        will be written.
 * @param Address: the start flash address to write
 * @param Data: input data to program
 * @param Length: amount of bytes to program
 * @return BUSY     if another operation is already ongoing,
 *         OK       if programming started
 */
XPD_ReturnType XPD_FLASH_Program_IT(void * Address, const void * Data, uint16_t Length)
{
    XPD_ReturnType result = XPD_BUSY;

    /* Only start if no ongoing operations are present */
    if (FLASH_GETOPERATION() == FLASH_OPERATION_NONE)
    {
        hflash->MemStream.buffer = (void*)Data;
        hflash->MemStream.length = Length / hflash->MemStream.size;
        hflash->Address = Address;

        /* Enable interrupts */
        SET_BIT(FLASH->CR.w, FLASH_CR_EOPIE | FLASH_CR_ERRIE);

        /* Enable flash programming */
        FLASH_REG_BIT(CR,PG) = 1;

        XPD_WriteFromStream(hflash->Address, &hflash->MemStream);

        result = XPD_OK;
    }
    return result;
}

/**
 * @brief Performs a mass erase on a flash memory bank.
 * @note  The FLASH interface should be unlocked beforehand, and locked afterwards.
 * @param Bank: The selected memory bank (for compatibility only)
 * @return BUSY     if another operation is already ongoing,
 *         ERROR    if there were errors,
 *         TIMEOUT  if timed out,
 *         OK       if successful
 */
XPD_ReturnType XPD_FLASH_EraseBank(uint8_t Bank)
{
    /* Wait for last operation to be completed */
    XPD_ReturnType result = XPD_FLASH_PollStatus(FLASH_TIMEOUT_MS);

    if (result == XPD_OK)
    {
        /* Only bank1 is present */
        FLASH_REG_BIT(CR,MER) = 1;
        FLASH_REG_BIT(CR,STRT) = 1;

        result = XPD_FLASH_PollStatus(FLASH_TIMEOUT_MS);

        FLASH_REG_BIT(CR,MER) = 0;
    }

    return result;
}

/**
 * @brief Performs a mass erase on a flash memory bank in interrupt (non-blocking) mode.
 * @note  The FLASH interface should be unlocked beforehand, and locked afterwards.
 * @param Bank: The selected memory bank (for compatibility only)
 * @return BUSY     if another operation is already ongoing,
 *         OK       if mass erase started
 */
XPD_ReturnType XPD_FLASH_EraseBank_IT(uint8_t Bank)
{
    XPD_ReturnType result = XPD_BUSY;

    /* Only start if no ongoing operations are present */
    if (FLASH_GETOPERATION() == FLASH_OPERATION_NONE)
    {
        /* Enable interrupts */
        SET_BIT(FLASH->CR.w, FLASH_CR_EOPIE | FLASH_CR_ERRIE);

        /* Only bank1 is present */
        FLASH_REG_BIT(CR,MER)  = 1;
        FLASH_REG_BIT(CR,STRT) = 1;

        result = XPD_OK;
    }

    return result;
}

/**
 * @brief Performs consecutive page erases in flash.
 * @note  The FLASH interface should be unlocked beforehand, and locked afterwards.
 * @param Address: start address of first page to be erased
 * @param kBytes: amount of flash memory to erase in kilobytes (one page is 2 kB)
 * @return BUSY     if another operation is already ongoing,
 *         ERROR    if there were errors,
 *         TIMEOUT  if timed out,
 *         OK       if successful
 */
XPD_ReturnType XPD_FLASH_Erase(void * Address, uint16_t kBytes)
{
    /* Wait for last operation to be completed */
    XPD_ReturnType result = XPD_FLASH_PollStatus(FLASH_TIMEOUT_MS);

    if (result == XPD_OK)
    {
        hflash->Address = Address;
        hflash->MemStream.length = kBytes;

        while (hflash->MemStream.length > 0)
        {
            uint32_t blocksize = FLASH_BLOCK_SIZE(hflash->Address);
            flash_blockErase();

            /* Wait for last operation to be completed */
            result = XPD_FLASH_PollStatus(FLASH_TIMEOUT_MS);

            /* If the erase operation is completed, disable the PER Bit */
            FLASH_REG_BIT(CR,PER) = 0;

            /* In case of error, stop flash programming */
            if (result != XPD_OK)
            {
                break;
            }

            /* Increase flash address */
            hflash->Address += blocksize;
            hflash->MemStream.length -= blocksize >> 10;
        }
    }

    return result;
}

/**
 * @brief Performs consecutive page erases in flash in interrupt (non-blocking) mode.
 * @note  The FLASH interface should be unlocked beforehand, and locked afterwards.
 * @param Address: start address of first page to be erased
 * @param kBytes: amount of flash memory to erase in kilobytes (one page is 2 kB)
 * @return BUSY     if another operation is already ongoing,
 *         OK       if page erase started
 */
XPD_ReturnType XPD_FLASH_Erase_IT(void * Address, uint16_t kBytes)
{
    XPD_ReturnType result = XPD_BUSY;

    /* Only start if no ongoing operations are present */
    if (FLASH_GETOPERATION() == FLASH_OPERATION_NONE)
    {
        hflash->Address = Address;
        hflash->MemStream.length = kBytes;

        flash_blockErase();

        /* Enable interrupts */
        SET_BIT(FLASH->CR.w, FLASH_CR_EOPIE | FLASH_CR_ERRIE);

        result = XPD_OK;
    }

    return result;
}

/**
 * @brief FLASH interrupt handler that manages consecutive block erasing
 *        and programming, and provides completion and error callbacks.
 */
void XPD_FLASH_IRQHandler(void)
{
    /* Check FLASH error flags */
    flash_checkErrors();
    if (hflash->Errors != FLASH_ERROR_NONE)
    {
        /* Stop programming */
        hflash->MemStream.length = 0;
        CLEAR_BIT(FLASH->CR.w, FLASH_CR_MER | FLASH_CR_PER | FLASH_CR_PG);

        /* Error callback */
        XPD_SAFE_CALLBACK(XPD_FLASH_Callbacks.Error,);
    }

    /* Check FLASH End of Operation flag  */
    if (XPD_FLASH_GetFlag(EOP) != 0)
    {
        FLASH_OperationType operation = FLASH_GETOPERATION();

        /* Clear FLASH End of Operation pending bit */
        XPD_FLASH_ClearFlag(EOP);

        /* If the operation completed, disable its control bit */
        CLEAR_BIT(FLASH->CR.w, operation);

        switch (operation)
        {
            case FLASH_OPERATION_ERASE_BLOCK:
            {
                uint32_t blocksize = FLASH_BLOCK_SIZE(hflash->Address);
                hflash->MemStream.length -= blocksize >> 10;

                if (hflash->MemStream.length > 0)
                {
                    /* Continue with erasing consecutive blocks */
                    hflash->Address += blocksize;
                    flash_blockErase();
                }
                else
                {
                    /* provide completion callback */
                    XPD_SAFE_CALLBACK(XPD_FLASH_Callbacks.EraseComplete,);
                }
                break;
            }

            case FLASH_OPERATION_ERASE_BANK:
            {
                /* provide completion callback */
                XPD_SAFE_CALLBACK(XPD_FLASH_Callbacks.EraseComplete,);
                break;
            }

            case FLASH_OPERATION_PROGRAM:
            {
                /* Check if there are still data to program */
                if (hflash->MemStream.length > 0)
                {
                    /* Continue with programming consecutive pages */
                    hflash->Address += hflash->MemStream.size;

                    FLASH_REG_BIT(CR,PG) = 1;

                    XPD_WriteFromStream(hflash->Address, &hflash->MemStream);
                }
                else
                {
                    /* provide completion callback */
                    XPD_SAFE_CALLBACK(XPD_FLASH_Callbacks.ProgramComplete,);
                }
                break;
            }

            default: break;
        }
    }

    /* No more pending operations, disable further interrupts */
    if (FLASH_GETOPERATION() == FLASH_OPERATION_NONE)
    {
        CLEAR_BIT(FLASH->CR.w, FLASH_CR_EOPIE | FLASH_CR_ERRIE);
    }
}

/**
 * @brief Provides the errors which have occurred during the last operation.
 * @return The occurred errors
 */
FLASH_ErrorType XPD_FLASH_GetError(void)
{
    return hflash->Errors;
}

/** @} */

XPD_FLASH_CallbacksType XPD_FLASH_Callbacks = { NULL, NULL, NULL };

/** @} */

#endif /* USE_XPD_FLASH */
