/**
  ******************************************************************************
  * @file    usbd_dfu_flash.h
  * @author  Benedek Kupper
  * @version V0.1
  * @date    2017-06-05
  * @brief   STM32 eXtensible Peripheral Drivers USB Firmware Upgrade Project
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
#ifndef __USBD_DFU_FLASH_H_
#define __USBD_DFU_FLASH_H_
#include <usbd_dfu.h>

extern const USBD_DFU_MediaTypeDef USBD_DFU_Flash_fops;

extern USBD_HandleTypeDef hUsbDeviceFS;

#endif /* __USBD_DFU_FLASH_H_ */
