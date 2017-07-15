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

#include "../BSP_STM32F3-Discovery/xpd_bsp.h"

/* USB Device Core handle declaration */
USBD_HandleTypeDef hUsbDeviceFS;


#define FLASH_ERASE_TIME      50
#define FLASH_PROGRAM_TIME    50

/*
 * Device memory:   128 kB
 * Layout:          64 pages of 2 kBytes
 * Bootloader size: 24 kB */
#define FLASH_WRITE_ADDRESS   (FLASH_BASE + 0x5000)

#define FLASH_DESC_STR        "@Internal Flash /0x08000000/10*2Ka,54*2Kg"


void FlashIf_Erase(uint32_t Add);
void FlashIf_Write(uint8_t *dest, uint8_t *src, uint32_t Len);
void FlashIf_Read(uint8_t *dest, uint8_t *src, uint32_t Len);
void FlashIf_GetStatus(uint32_t Add, uint8_t Cmd, uint8_t *buffer);

const USBD_DFU_MediaTypeDef USBD_DFU_Flash_fops = {
    (uint8_t *)FLASH_DESC_STR,
    (void*)FLASH_WRITE_ADDRESS,
    XPD_FLASH_Unlock,
    XPD_FLASH_Lock,
    FlashIf_Erase,
    FlashIf_Write,
    FlashIf_Read,
    FlashIf_GetStatus
};

/**
 * @brief  Erases flash block.
 * @param  Add: Address of block to be erased.
 */
void FlashIf_Erase(uint32_t Add)
{
    /* Erase flash memory from the start address
     * As length is not provided, only delete one block */
    XPD_FLASH_Erase((void*)Add, 1);
}

/**
 * @brief  Writes Data into Memory.
 * @param  dest: Pointer to the destination buffer.
 * @param  src: Pointer to the source buffer. Address to be written to.
 * @param  Len: Number of data to be written (in bytes).
 */
void FlashIf_Write(uint8_t *dest, uint8_t *src, uint32_t Len)
{
    XPD_FLASH_Program(dest, src, Len);
}

/**
 * @brief  Reads Data into Memory.
 * @param  dest: Pointer to the destination buffer.
 * @param  src: Pointer to the source buffer. Address to be written to.
 * @param  Len: Number of data to be read (in bytes).
 */
void FlashIf_Read(uint8_t *dest, uint8_t *src, uint32_t Len)
{
    while (Len-- > 0)
    {
        *dest++ = *src++;
    }
}

/**
 * @brief  Gets memory operation duration.
 * @param  Add: Address to be read from.
 * @param  Cmd: Type of operation.
 * @param  buffer: Response data - duration of operation.
 */
void FlashIf_GetStatus(uint32_t Add, uint8_t Cmd, uint8_t *buffer)
{
    switch (Cmd)
    {
        case DFU_MEDIA_PROGRAM:
            buffer[1] = (uint8_t) FLASH_PROGRAM_TIME;
            buffer[2] = (uint8_t)(FLASH_PROGRAM_TIME << 8);
            buffer[3] = 0;
            break;

        case DFU_MEDIA_ERASE:
        default:
            buffer[1] = (uint8_t) FLASH_ERASE_TIME;
            buffer[2] = (uint8_t)(FLASH_ERASE_TIME << 8);
            buffer[3] = 0;
            break;
    }
}
