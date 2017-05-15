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

int usbSuspendCallback(void * user)
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

int usbResumeCallback(void * user)
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
int usbResetCallback(void * user)
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

    /* Control endpoints */
    XPD_USB_EP_BufferInit(pdev->pData, 0x00,  0x40);
    XPD_USB_EP_BufferInit(pdev->pData, 0x80,  0x40);

    /* Endpoints for CDC device (bulk EPs are set to double-buffered) */
    XPD_USB_EP_BufferInit(pdev->pData, CDC_IN_EP,  CDC_DATA_FS_MAX_PACKET_SIZE * 2);
    XPD_USB_EP_BufferInit(pdev->pData, CDC_OUT_EP, CDC_DATA_FS_MAX_PACKET_SIZE * 2);
    XPD_USB_EP_BufferInit(pdev->pData, CDC_CMD_EP, CDC_CMD_PACKET_SIZE);

    /* USB device only supports full speed */
    USBD_LL_SetSpeed(pdev, USBD_SPEED_FULL);

    return USBD_OK;
}

/**
 * @brief  De-Initializes the Low Level portion of the Device driver.
 * @param  pdev: Device handle
 * @retval USBD Status
 */
USBD_StatusTypeDef USBD_LL_DeInit(USBD_HandleTypeDef *pdev)
{
    XPD_USB_Deinit(pdev->pData);
    return USBD_OK;
}

/**
 * @brief  Starts the Low Level portion of the Device driver.
 * @param  pdev: Device handle
 * @retval USBD Status
 */
USBD_StatusTypeDef USBD_LL_Start(USBD_HandleTypeDef *pdev)
{
    XPD_USB_Start(pdev->pData);
    return USBD_OK;
}

/**
 * @brief  Stops the Low Level portion of the Device driver.
 * @param  pdev: Device handle
 * @retval USBD Status
 */
USBD_StatusTypeDef USBD_LL_Stop(USBD_HandleTypeDef *pdev)
{
    XPD_USB_Stop(pdev->pData);
    return USBD_OK;
}

/**
 * @brief  Opens an endpoint of the Low Level Driver.
 * @param  pdev: Device handle
 * @param  ep_addr: Endpoint Number
 * @param  ep_type: Endpoint Type
 * @param  ep_mps: Endpoint Max Packet Size
 * @retval USBD Status
 */
USBD_StatusTypeDef USBD_LL_OpenEP(USBD_HandleTypeDef *pdev, uint8_t ep_addr,
        uint8_t ep_type, uint16_t ep_mps)
{
    XPD_USB_EP_Open(pdev->pData, ep_addr, (USB_EndPointType)ep_type, ep_mps);
    return USBD_OK;
}

/**
 * @brief  Closes an endpoint of the Low Level Driver.
 * @param  pdev: Device handle
 * @param  ep_addr: Endpoint Number
 * @retval USBD Status
 */
USBD_StatusTypeDef USBD_LL_CloseEP(USBD_HandleTypeDef *pdev, uint8_t ep_addr)
{
    XPD_USB_EP_Close(pdev->pData, ep_addr);
    return USBD_OK;
}

/**
 * @brief  Flushes an endpoint of the Low Level Driver.
 * @param  pdev: Device handle
 * @param  ep_addr: Endpoint Number
 * @retval USBD Status
 */
USBD_StatusTypeDef USBD_LL_FlushEP(USBD_HandleTypeDef *pdev, uint8_t ep_addr)
{
    XPD_USB_EP_Flush(pdev->pData, ep_addr);
    return USBD_OK;
}

/**
 * @brief  Sets a Stall condition on an endpoint of the Low Level Driver.
 * @param  pdev: Device handle
 * @param  ep_addr: Endpoint Number
 * @retval USBD Status
 */
USBD_StatusTypeDef USBD_LL_StallEP(USBD_HandleTypeDef *pdev, uint8_t ep_addr)
{
    XPD_USB_EP_SetStall(pdev->pData, ep_addr);
    return USBD_OK;
}

/**
 * @brief  Clears a Stall condition on an endpoint of the Low Level Driver.
 * @param  pdev: Device handle
 * @param  ep_addr: Endpoint Number
 * @retval USBD Status
 */
USBD_StatusTypeDef USBD_LL_ClearStallEP(USBD_HandleTypeDef *pdev,
        uint8_t ep_addr)
{
    XPD_USB_EP_ClearStall(pdev->pData, ep_addr);
    return USBD_OK;
}

/**
 * @brief  Returns Stall condition.
 * @param  pdev: Device handle
 * @param  ep_addr: Endpoint Number
 * @retval Stall (1: Yes, 0: No)
 */
uint8_t USBD_LL_IsStallEP(USBD_HandleTypeDef *pdev, uint8_t ep_addr)
{
    USB_HandleType *husb = pdev->pData;

    if (ep_addr > 0x7F)
    {
        return husb->EP.IN[ep_addr & 0x7F].Stalled;
    }
    else
    {
        return husb->EP.OUT[ep_addr].Stalled;
    }
}

/**
 * @brief  Assigns a USB address to the device.
 * @param  pdev: Device handle
 * @param  ep_addr: Endpoint Number
 * @retval USBD Status
 */
USBD_StatusTypeDef USBD_LL_SetUSBAddress(USBD_HandleTypeDef *pdev,
        uint8_t dev_addr)
{
    XPD_USB_SetAddress(pdev->pData, dev_addr);
    return USBD_OK;
}

/**
 * @brief  Transmits data over an endpoint.
 * @param  pdev: Device handle
 * @param  ep_addr: Endpoint Number
 * @param  pbuf: Pointer to data to be sent
 * @param  size: Data size
 * @retval USBD Status
 */
USBD_StatusTypeDef USBD_LL_Transmit(USBD_HandleTypeDef *pdev, uint8_t ep_addr,
        uint8_t *pbuf, uint16_t size)
{
    XPD_USB_EP_Transmit(pdev->pData, ep_addr, pbuf, size);
    return USBD_OK;
}

/**
 * @brief  Prepares an endpoint for reception.
 * @param  pdev: Device handle
 * @param  ep_addr: Endpoint Number
 * @param  pbuf: Pointer to data to be received
 * @param  size: Data size
 * @retval USBD Status
 */
USBD_StatusTypeDef USBD_LL_PrepareReceive(USBD_HandleTypeDef *pdev,
        uint8_t ep_addr, uint8_t *pbuf, uint16_t size)
{
    XPD_USB_EP_Receive(pdev->pData, ep_addr, pbuf, size);
    return USBD_OK;
}

/**
 * @brief  Returns the last transfered packet size.
 * @param  pdev: Device handle
 * @param  ep_addr: Endpoint Number
 * @retval Recived Data Size
 */
uint32_t USBD_LL_GetRxDataSize(USBD_HandleTypeDef *pdev, uint8_t ep_addr)
{
    return XPD_USB_EP_GetRxCount(pdev->pData, ep_addr);
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

/**
 * @brief  Dummy memory free
 * @param  *p pointer to allocated  memory address
 * @retval None
 */
void USBD_static_free(void *p)
{

}
