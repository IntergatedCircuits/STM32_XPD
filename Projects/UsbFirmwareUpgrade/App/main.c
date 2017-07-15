/**
  ******************************************************************************
  * @file    main.c
  * @author  Benedek Kupper
  * @version V0.1
  * @date    2016-05-27
  * @brief   STM32 CAN to USB Virtual COM Port Project
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

#include <usbd_core.h>
#include <usbd_desc.h>
#include <usbd_dfu_flash.h>
#include "../BSP_STM32F3-Discovery/xpd_bsp.h"


int main(void)
{
    ClockConfiguration();

    /* Init Device Library, Add Supported Class and Start the library */
    USBD_Init(&hUsbDeviceFS, (void*)&DFU_Desc, DEVICE_FS);

    USBD_RegisterClass(&hUsbDeviceFS, (void*)&USBD_DFU);

    USBD_DFU_RegisterMedia(&hUsbDeviceFS, &USBD_DFU_Flash_fops);

    USBD_Start(&hUsbDeviceFS);

    while(1)
    {
    }
}
