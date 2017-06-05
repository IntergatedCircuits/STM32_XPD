/**
  ******************************************************************************
  * @file    usbd_dfu_flash.c
  * @author  Benedek Kupper
  * @version V0.1
  * @date    2017-06-05
  * @brief   STM32 eXtensible Peripheral Drivers USB Firmware Upgrade Project
  *
  *  @verbatim
  *
  *          ===================================================================
  *                               USB Device Firmware Upgrade
  *          ===================================================================
  *           This project implements a USB bootloader using USB DFU interface
  *           and the embedded flash memory of the device.
  *  @endverbatim
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
#include <usbd_dfu_flash.h>
#include <xpd_user.h>

#define FLASH_ERASE_TIME    (uint16_t)50
#define FLASH_PROGRAM_TIME  (uint16_t)50

/* USB Device Core handle declaration */
USBD_HandleTypeDef hUsbDeviceFS;

void FlashIf_Init(void);
void FlashIf_DeInit(void);
void FlashIf_Erase(uint32_t Add);
void* FlashIf_Write(uint8_t *src, uint8_t *dest, uint32_t Len);
void* FlashIf_Read(uint8_t *src, uint8_t *dest, uint32_t Len);
void FlashIf_GetStatus(uint32_t Add, uint8_t Cmd, uint8_t *buffer);

const USBD_DFU_MediaTypeDef USBD_DFU_Flash_fops = {
    (uint8_t *)FLASH_DESC_STR,
    FlashIf_Init,  FlashIf_DeInit, FlashIf_Erase,
    FlashIf_Write, FlashIf_Read,   FlashIf_GetStatus
};

/**
 * @brief  Initializes Memory.
 */
void FlashIf_Init(void)
{
    /* Unlock the internal flash */
    XPD_FLASH_Unlock();
}

/**
 * @brief  De-Initializes Memory.
 */
void FlashIf_DeInit(void)
{
    /* Lock the internal flash */
    XPD_FLASH_Lock();
}

/**
 * @brief  Erases sector.
 * @param  Add: Address of sector to be erased.
 */
void FlashIf_Erase(uint32_t Add)
{
    if (Add == USBD_DFU_APP_DEFAULT_ADD)
    {
        XPD_FLASH_Erase((void*)Add, DEVICE_FLASH_SIZE_KB - ((Add - FLASH_BASE) >> 10));
    }
}

/**
 * @brief  Writes Data into Memory.
 * @param  dest: Pointer to the destination buffer.
 * @param  src: Pointer to the source buffer. Address to be written to.
 * @param  Len: Number of data to be written (in bytes).
 */
void* FlashIf_Write(uint8_t *dest, uint8_t *src, uint32_t Len)
{
    if ((Len & 3) != 0)
    {
        Len = Len & (~1);
    }
    XPD_FLASH_Program(dest, src, Len);
    return dest;
}

/**
 * @brief  Reads Data into Memory.
 * @param  dest: Pointer to the destination buffer.
 * @param  src: Pointer to the source buffer. Address to be written to.
 * @param  Len: Number of data to be read (in bytes).
 */
void* FlashIf_Read(uint8_t *dest, uint8_t *src, uint32_t Len)
{
    while (Len-- > 0)
    {
        *dest++ = *src++;
    }
    return dest;
}

/**
 * @brief  Gets Memory Status.
 * @param  Add: Address to be read from.
 * @param  Cmd: Number of data to be read (in bytes).
 */
void FlashIf_GetStatus(uint32_t Add, uint8_t Cmd, uint8_t *buffer)
{
    switch (Cmd)
    {
        case DFU_MEDIA_PROGRAM:
            buffer[1] = (uint8_t) FLASH_PROGRAM_TIME;
            buffer[2] = (uint8_t) (FLASH_PROGRAM_TIME << 8);
            buffer[3] = 0;
            break;

        case DFU_MEDIA_ERASE:
        default:
            buffer[1] = (uint8_t) FLASH_ERASE_TIME;
            buffer[2] = (uint8_t) (FLASH_ERASE_TIME << 8);
            buffer[3] = 0;
            break;
    }
}
