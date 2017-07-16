/**
  ******************************************************************************
  * @file    usbd_conf.c
  * @author  Benedek Kupper
  * @version V0.2
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
#include "usbd_conf.h"
#include "usbd_dfu.h"
#include "usbd_core.h"

#include "xpd_bsp.h"

#ifdef USB_OTG_HS
static int usbResetCallback(void * user)
{
    USB_SpeedType speed = ((USB_HandleType *)(((USBD_HandleTypeDef *)user)->pData))->Speed;

    /* Reset Device */
    USBD_LL_Reset(user);

    return USBD_LL_SetSpeed(user,
            (speed == USB_SPEED_FULL) ? USBD_SPEED_FULL : USBD_SPEED_HIGH);
}
#endif

/*******************************************************************************
 LL Driver Interface (USB Device Library --> XPD)
 *******************************************************************************/

/**
 * @brief  Initializes the Low Level portion of the Device driver.
 * @param  pdev: Device handle
 * @retval USBD Status
 */
USBD_StatusTypeDef USBD_LL_Init(USBD_HandleTypeDef *pdev)
{
#ifdef USB_OTG_HS
    if (pdev->id == DEVICE_FS)
#endif
    {
        /* USB init setup */
        const USB_InitType init = {
            .Speed = USB_SPEED_FULL,
        };

        /* Link driver to user */
        pdev->pData = &usbHandle;

        /* Link the stack to the driver */
        usbHandle.User = pdev;

        /* Set direct USBD API callbacks */
        usbHandle.Callbacks.SetupStage       = USBD_LL_SetupStage;
        usbHandle.Callbacks.DataOutStage     = USBD_LL_DataOutStage;
        usbHandle.Callbacks.DataInStage      = USBD_LL_DataInStage;
#ifdef USB_OTG_HS
        usbHandle.Callbacks.Reset            = usbResetCallback;
#else
        usbHandle.Callbacks.Reset            = USBD_LL_Reset;
#endif
        usbHandle.Callbacks.Suspend          = USBD_LL_Suspend;
        usbHandle.Callbacks.Resume           = USBD_LL_Resume;

        XPD_USB_Init(&usbHandle, &init);

        /* USB device only supports full speed */
        USBD_LL_SetSpeed(pdev, USBD_SPEED_FULL);
    }

    return USBD_OK;
}

/**
 * @brief  static single allocation.
 * @param  size: size of allocated memory
 * @retval None
 */
void *USBD_static_malloc(uint32_t size)
{
    static uint32_t mem[(sizeof(USBD_DFU_HandleTypeDef) / 4) + 1];
    return mem;
}
