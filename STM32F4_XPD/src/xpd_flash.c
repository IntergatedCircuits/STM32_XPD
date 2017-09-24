/**
  ******************************************************************************
  * @file    xpd_flash.c
  * @author  Benedek Kupper
  * @version V0.2
  * @date    2017-09-24
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

#ifdef FLASH_CR_MER1
#define FLASH_OPERATIONS        \
    (FLASH_OPERATION_ERASE_BLOCK | FLASH_OPERATION_ERASE_BANK1 \
   | FLASH_OPERATION_ERASE_BANK2 | FLASH_OPERATION_PROGRAM)
#else
#define FLASH_OPERATIONS        \
    (FLASH_OPERATION_ERASE_BLOCK | FLASH_OPERATION_ERASE_BANK1 | FLASH_OPERATION_PROGRAM)
#endif

#define FLASH_BLOCK_SIZE_KB     (xpd_flashHandle.SectorSize)

/* The size of a single programming operation */
#if defined(FLASH_CR_PSIZE)
/* is configurable, adjust based on VDD voltage level */
#if   (VDD_VALUE > 2700)
#define FLASH_PSIZE_CONFIG()    (FLASH->CR.b.PSIZE = 2)
#define FLASH_MEMSTREAM_SIZE    (sizeof(uint32_t))
#elif (VDD_VALUE > 2100)
#define FLASH_PSIZE_CONFIG()    (FLASH->CR.b.PSIZE = 1)
#define FLASH_MEMSTREAM_SIZE    (sizeof(uint16_t))
#else
#define FLASH_PSIZE_CONFIG()    (FLASH->CR.b.PSIZE = 0)
#define FLASH_MEMSTREAM_SIZE    (sizeof(uint8_t))
#endif /* VDD_VALUE */
#else
#define FLASH_PSIZE_CONFIG()    ((void)0)
#define FLASH_MEMSTREAM_SIZE    (sizeof(uint16_t))
#endif

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
    uint16_t SectorSize;
    uint8_t BkpACR;
    volatile FLASH_ErrorType Errors;
} xpd_flashHandle =
{
    .Errors         = FLASH_ERROR_NONE,
    .MemStream.size = FLASH_MEMSTREAM_SIZE,
};

/* Erase the next scheduled block */
static void flash_blockErase(void)
{
    uint32_t sector = 0;
    uint32_t addr   =  (uint32_t)hflash->Address & 0x000FFFFF;

#ifdef FLASH_CR_MER1
    /* Check that the address is in Bank2 */
    if ((((uint32_t)hflash->Address) >= (FLASH_BASE + (DEVICE_FLASH_SIZE_KB / 2)))
#ifdef FLASH_OPTCR_DB1M
        && (!((DEVICE_FLASH_SIZE_KB == 1024) && (FLASH_REG_BIT(OPTCR,DB1M) == 0)))
#endif
        )
    {
        /* Sector offset is 12, address offset by half */
        sector = 12;
        addr  &= ~(DEVICE_FLASH_SIZE_KB / 2);
    }
#endif

    if (addr < 0x10000)
    {
        /* Sector 0-3:  16kB */
        hflash->SectorSize = 16;

        sector += addr >> 14;
    }
    else
    {
        /* Sector 4  :  64kB
         * Sector 5- : 128kB */
        addr >>= 17;
        hflash->SectorSize = (addr == 0) ? 64 : 128;

        sector += 4 + addr;
    }

    /* Proceed to erase the sector */
    FLASH->CR.b.SNB = sector;
    FLASH_REG_BIT(CR,STRT) = 1;
}

/* Read errors to context, and clear them in register */
static FLASH_ErrorType flash_checkErrors(void)
{
    /* Read error flags */
    hflash->Errors = FLASH->SR.w & (FLASH_SR_WRPERR
    | FLASH_SR_PGAERR | FLASH_SR_PGSERR
    | FLASH_SR_PGPERR
#ifdef FLASH_SR_RDERR
    | FLASH_SR_RDERR
#endif
#ifdef FLASH_SR_SOP
    | FLASH_SR_SOP
#endif
    );

    /* Clear error flags */
    FLASH->SR.w = hflash->Errors;
    return hflash->Errors;
}

/* Disable the flash caches before erase */
static void flash_disableCaches(void)
{
    /* Store backup of current cache settings, disable all */
    hflash->BkpACR = (FLASH->ACR.w & (FLASH_ACR_ICEN | FLASH_ACR_DCEN)) >> 8;
    CLEAR_BIT(FLASH->ACR.w, FLASH_ACR_ICEN | FLASH_ACR_DCEN);
}

/* Flush the flash caches after erase */
static void flash_flushCaches(void)
{
    uint32_t enMask = hflash->BkpACR << 8;
    uint32_t rstMask = enMask << (FLASH_ACR_ICRST_Pos - FLASH_ACR_ICEN_Pos);

    /* Set and clear DCRST and ICRST for previously enabled caches */
    SET_BIT(FLASH->ACR.w, rstMask);
    CLEAR_BIT(FLASH->ACR.w, rstMask);

    /* Set and clear DCEN and ICEN for previously enabled caches */
    SET_BIT(FLASH->ACR.w, enMask);
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

        /* Clear any previous request bits */
        CLEAR_BIT(FLASH->CR.w, FLASH_OPERATIONS);

        /* Also set the default programming size
         * (based on power voltage level) */
        FLASH_PSIZE_CONFIG();
    }
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

    if (flash_checkErrors() != FLASH_ERROR_NONE)
    {
        result = XPD_ERROR;
    }

    return result;
}

/**
 * @brief Programs the input data to the specified flash address.
 * @note  The FLASH interface should be unlocked beforehand, and locked afterwards.
 * @note  If flash memory is not erased before programming,
          only 1 -> 0 bit transitions will be written.
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

        /* Disable caches before start */
        flash_disableCaches();

        /* Enable flash programming */
        FLASH_REG_BIT(CR,PG) = 1;

        while (hflash->MemStream.length > 0)
        {
            XPD_WriteFromStream(hflash->Address, &hflash->MemStream);

            /* Wait for last operation to be completed */
            result = XPD_FLASH_PollStatus(FLASH_TIMEOUT_MS);

            /* In case of error, stop flash programming */
            if (result != XPD_OK)
            {
                break;
            }

            /* Increment flash address */
            hflash->Address += hflash->MemStream.size;
        }

        /* Disable flash programming */
        FLASH_REG_BIT(CR,PG) = 0;

        /* Flush the caches to be sure of the data consistency */
        flash_flushCaches();
    }

    return result;
}

/**
 * @brief Programs the input data to the specified flash address in interrupt (non-blocking) mode.
 * @note  The FLASH interface should be unlocked beforehand, and locked afterwards.
 * @note  If flash memory is not erased before programming,
          only 1 -> 0 bit transitions will be written.
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
    if (FLASH_REG_BIT(SR,BSY) == 0)
    {
        hflash->MemStream.buffer = (void*)Data;
        hflash->MemStream.length = Length / hflash->MemStream.size;
        hflash->Address = Address;

        /* Disable caches before start */
        flash_disableCaches();

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
 * @param Bank: The selected memory bank (1 or 2)
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
        /* Disable caches before start */
        flash_disableCaches();

#ifdef FLASH_CR_MER1
        if (Bank == 2)
        {
#ifdef FLASH_OPTCR_DB1M
            /* Check if device flash size is 1MB, bank2 is only used when DB1M is set */
            if ((DEVICE_FLASH_SIZE_KB == 1024) && (FLASH_REG_BIT(OPTCR,DB1M) == 0))
            {
                result = XPD_ERROR;
            }
            else
#endif
            {
                /* Bank2 is selected */
                FLASH_REG_BIT(CR,MER1) = 1;
                FLASH_REG_BIT(CR,STRT) = 1;

                /* Bank2 start address is at half of the range */
                hflash->Address = (void*)(FLASH_BASE + (DEVICE_FLASH_SIZE_KB << (10 - 1)));
                result = XPD_FLASH_PollStatus(FLASH_TIMEOUT_MS);

                FLASH_REG_BIT(CR,MER1) = 0;
            }
        }
        else
#endif
        {
            /* Bank1 is selected */
            FLASH_REG_BIT(CR,MER)  = 1;
            FLASH_REG_BIT(CR,STRT) = 1;

            hflash->Address = (void*)FLASH_BASE;
            result = XPD_FLASH_PollStatus(FLASH_TIMEOUT_MS);

            FLASH_REG_BIT(CR,MER)  = 0;
        }

        /* Flush the caches to be sure of the data consistency */
        flash_flushCaches();
    }

    return result;
}

/**
 * @brief Performs a mass erase on a flash memory bank in interrupt (non-blocking) mode.
 * @note  The FLASH interface should be unlocked beforehand, and locked afterwards.
 * @param Bank: The selected memory bank (1 or 2)
 * @return BUSY     if another operation is already ongoing,
 *         OK       if mass erase started
 */
XPD_ReturnType XPD_FLASH_EraseBank_IT(uint8_t Bank)
{
    XPD_ReturnType result = XPD_BUSY;

    /* Only start if no ongoing operations are present */
    if (FLASH_REG_BIT(SR,BSY) == 0)
    {
        /* Disable caches before start */
        flash_disableCaches();

        /* Enable interrupts */
        SET_BIT(FLASH->CR.w, FLASH_CR_EOPIE | FLASH_CR_ERRIE);

#ifdef FLASH_CR_MER1
        if (Bank == 2)
        {
#ifdef FLASH_OPTCR_DB1M
            /* Check if device flash size is 1MB, bank2 is only used when DB1M is set */
            if ((DEVICE_FLASH_SIZE_KB == 1024) && (FLASH_REG_BIT(OPTCR,DB1M) == 0))
            {
                result = XPD_ERROR;
            }
            else
#endif
            {
                /* Bank2 is selected */
                FLASH_REG_BIT(CR,MER1) = 1;
                FLASH_REG_BIT(CR,STRT) = 1;

                /* Bank2 start address is at half of the range */
                hflash->Address = (void*)(FLASH_BASE + (DEVICE_FLASH_SIZE_KB << (10 - 1)));
            }
        }
        else
#endif
        {
            /* Bank1 is selected */
            FLASH_REG_BIT(CR,MER)  = 1;
            FLASH_REG_BIT(CR,STRT) = 1;

            hflash->Address = (void*)FLASH_BASE;
        }
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

        /* Disable caches before start */
        flash_disableCaches();

        /* Set the sector erase control bit */
        FLASH_REG_BIT(CR,SER) = 1;

        /* Keep erasing blocks until at least the requested amount */
        while (hflash->MemStream.length > 0)
        {
            flash_blockErase();

            /* Wait for last operation to be completed */
            result = XPD_FLASH_PollStatus(FLASH_TIMEOUT_MS);

            /* In case of error or length underflow, stop flash programming */
            if ((result != XPD_OK) || (hflash->MemStream.length < FLASH_BLOCK_SIZE_KB))
            {
                break;
            }

            /* Increase flash address */
            hflash->Address += FLASH_BLOCK_SIZE_KB << 10;
            hflash->MemStream.length -= FLASH_BLOCK_SIZE_KB;
        }

        /* If the erase operation is completed, disable the SER Bit */
        FLASH_REG_BIT(CR,SER) = 0;

        /* Flush the caches to be sure of the data consistency */
        flash_flushCaches();
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
    if (FLASH_REG_BIT(SR,BSY) == 0)
    {
        hflash->Address = Address;
        hflash->MemStream.length = kBytes;

        /* Disable caches before start */
        flash_disableCaches();

        /* Enable interrupts */
        SET_BIT(FLASH->CR.w, FLASH_CR_EOPIE | FLASH_CR_ERRIE);

        /* Set the sector erase control bit */
        FLASH_REG_BIT(CR,SER) = 1;

        flash_blockErase();

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
    if (flash_checkErrors() != FLASH_ERROR_NONE)
    {
        /* Stop programming */
        hflash->MemStream.length = 0;
        CLEAR_BIT(FLASH->CR.w, FLASH_OPERATIONS);

        /* Error callback */
        XPD_SAFE_CALLBACK(XPD_FLASH_Callbacks.Error,);
    }

    /* Check FLASH End of Operation flag  */
    if (XPD_FLASH_GetFlag(EOP) != 0)
    {
        FLASH_OperationType operation = (FLASH->CR.w & FLASH_OPERATIONS);

        /* Clear FLASH End of Operation pending bit */
        XPD_FLASH_ClearFlag(EOP);

        switch (operation)
        {
            case FLASH_OPERATION_PROGRAM:
            {
                /* Check if there are still data to program */
                if (hflash->MemStream.length > 0)
                {
                    /* Continue with programming consecutive pages */
                    hflash->Address += hflash->MemStream.size;

                    XPD_WriteFromStream(hflash->Address, &hflash->MemStream);
                }
                else
                {
                    /* If the operation completed, disable its control bit */
                    FLASH_REG_BIT(CR,PG) = 0;

                    /* provide completion callback */
                    XPD_SAFE_CALLBACK(XPD_FLASH_Callbacks.ProgramComplete,);
                }
                break;
            }

            case FLASH_OPERATION_ERASE_BLOCK:
            {
                if (hflash->MemStream.length > FLASH_BLOCK_SIZE_KB)
                {
                    /* Continue with erasing consecutive blocks */
                    hflash->Address += FLASH_BLOCK_SIZE_KB << 10;
                    hflash->MemStream.length -= FLASH_BLOCK_SIZE_KB;
                    flash_blockErase();
                }
                else
                {
                    /* If the operation completed, disable its control bit */
                    FLASH_REG_BIT(CR,SER) = 0;

                    /* provide completion callback */
                    XPD_SAFE_CALLBACK(XPD_FLASH_Callbacks.EraseComplete,);
                }
                break;
            }

            /* FLASH_OPERATION_ERASE_BANK */
            default:
            {
                /* If the operation completed, disable its control bit */
                CLEAR_BIT(FLASH->CR.w, operation);

                /* provide completion callback */
                XPD_SAFE_CALLBACK(XPD_FLASH_Callbacks.EraseComplete,);
                break;
            }
        }
    }

    /* No more pending operations, disable further interrupts */
    if ((FLASH->CR.w & FLASH_OPERATIONS) == FLASH_OPERATION_NONE)
    {
        CLEAR_BIT(FLASH->CR.w, FLASH_CR_EOPIE | FLASH_CR_ERRIE);

        /* Flush the caches to be sure of the data consistency */
        flash_flushCaches();
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

/**
 * @brief Provides the current address where the last flash operation was attempted.
 * @return The current address
 */
uint32_t XPD_FLASH_GetAddress(void)
{
    return (uint32_t)hflash->Address;
}

/** @} */

XPD_FLASH_CallbacksType XPD_FLASH_Callbacks = { NULL, NULL, NULL };

/** @} */

#endif /* USE_XPD_FLASH */
