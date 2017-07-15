/**
  ******************************************************************************
  * @file    main.c
  * @author  Benedek Kupper
  * @version V0.1
  * @date    2017-04-16
  * @brief   STM32 eXtensible Peripheral Drivers USB Virtual COM Port Project
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
#include <xpd_bsp.h>

#include <usbd_core.h>
#include <usbd_desc.h>
#include <usbd_cdc_if.h>


int main(void)
{
    ClockConfiguration();

    /* Init Device Library, Add Supported Class and Start the library */
    USBD_Init(&hUsbDeviceFS, (void*)&CDC_Desc, DEVICE_FS);

    USBD_RegisterClass(&hUsbDeviceFS, (void*)&USBD_CDC);

    USBD_CDC_RegisterInterface(&hUsbDeviceFS, &USBD_Interface_fops_FS);

    USBD_Start(&hUsbDeviceFS);

    while(1)
    {
    }
}
