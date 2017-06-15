/**
  ******************************************************************************
  * @file    usbd_conf.c
  * @author  Benedek Kupper
  * @version V0.2
  * @date    2017-05-15
  * @brief   STM32 eXtensible Peripheral Drivers USB Device Module
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
#include "usbd_cdc.h"
#include "usbd_core.h"

#include "xpd_user.h"

static int usbSuspendCallback(void * user)
{
    /* Inform USB library that core enters in suspend Mode */
    int retval = USBD_LL_Suspend(user);

    if (((USB_HandleType *)(((USBD_HandleTypeDef *)user)->pData))->LowPowerMode == ENABLE)
    {
        XPD_USB_PHY_ClockCtrl(&usbHandle, DISABLE);

        /* Set SLEEPDEEP bit and SleepOnExit of Cortex System Control Register */
        SET_BIT(SCB->SCR.w, SCB_SCR_SLEEPDEEP_Msk | SCB_SCR_SLEEPONEXIT_Msk);
    }
    return retval;
}

static int usbResumeCallback(void * user)
{
    if (((USB_HandleType *)(((USBD_HandleTypeDef *)user)->pData))->LowPowerMode == ENABLE)
    {
        CLEAR_BIT(SCB->SCR.w, SCB_SCR_SLEEPDEEP_Msk | SCB_SCR_SLEEPONEXIT_Msk);
        /* Reconfigure system clocks */
        ClockConfiguration();
    }
    return USBD_LL_Resume(user);
}

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
    if (pdev->id == DEVICE_FS)
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
        usbHandle.Callbacks.SOF              = USBD_LL_SOF;
#ifdef USB_OTG_HS
        usbHandle.Callbacks.Reset            = usbResetCallback;
#else
        usbHandle.Callbacks.Reset            = USBD_LL_Reset;
#endif
        usbHandle.Callbacks.Suspend          = usbSuspendCallback;
        usbHandle.Callbacks.Resume           = usbResumeCallback;
        usbHandle.Callbacks.Connected        = USBD_LL_DevConnected;
        usbHandle.Callbacks.Disconnected     = USBD_LL_DevDisconnected;

        XPD_USB_Init(&usbHandle, &init);

        /* Endpoints for CDC device (bulk EPs are set to double-buffered) */
        XPD_USB_EP_BufferInit(pdev->pData, CDC_IN_EP,  CDC_DATA_FS_MAX_PACKET_SIZE * 2);
        XPD_USB_EP_BufferInit(pdev->pData, CDC_OUT_EP, CDC_DATA_FS_MAX_PACKET_SIZE * 2);

#if (CDC_AT_COMMAND_SUPPORT == 1)
        XPD_USB_EP_BufferInit(pdev->pData, CDC_CMD_EP, CDC_CMD_PACKET_SIZE);
#endif

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
    static uint32_t mem[(sizeof(USBD_CDC_HandleTypeDef) / 4) + 1];
    return mem;
}
