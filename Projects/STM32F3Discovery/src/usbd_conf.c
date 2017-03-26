/**
  ******************************************************************************
  * @file    usbd_conf.c
  * @author  Benedek Kupper
  * @version V0.1
  * @date    2017-03-22
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
#include "xpd_nvic.h"
#include "xpd_rcc.h"
#include "xpd_gpio.h"

#define USB_DISCONNECT_PORT                 GPIOA
#define USB_DISCONNECT_PIN                  9
#define USE_USB_INTERRUPT_REMAPPED

USB_HandleType husb = NEW_USB_HANDLE(USB);

void usb_setConnectionState(FunctionalState state)
{
#ifdef USB_DISCONNECT_PIN
    XPD_GPIO_WritePin(USB_DISCONNECT_PORT, USB_DISCONNECT_PIN, state);
#endif
}

#if defined (USE_USB_INTERRUPT_REMAPPED)
void USB_HP_IRQHandler(void)
{
    XPD_USB_IRQHandler(&husb);
}
void USB_LP_IRQHandler(void)
{
    XPD_USB_IRQHandler(&husb);
}
#else
void USB_HP_CAN_TX_IRQHandler(void)
{
    XPD_USB_IRQHandler(&husb);
}
void USB_LP_CAN_RX0_IRQHandler(void)
{
    XPD_USB_IRQHandler(&husb);
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
    GPIO_InitType gpioInit;

    /* Configure USB DM and DP pins */
    gpioInit.AlternateMap = GPIO_USB_AF14;
    gpioInit.Mode = GPIO_MODE_ALTERNATE;
    gpioInit.Output.Speed = VERY_HIGH;
    gpioInit.Output.Type  = GPIO_OUTPUT_PUSHPULL;
    gpioInit.Pull = GPIO_PULL_FLOAT;
    XPD_GPIO_InitPin(GPIOA, 11, &gpioInit);
    XPD_GPIO_InitPin(GPIOA, 12, &gpioInit);

#ifdef USB_DISCONNECT_PIN
    gpioInit.Mode = GPIO_MODE_OUTPUT;
    XPD_GPIO_InitPin(USB_DISCONNECT_PORT, USB_DISCONNECT_PIN, &gpioInit);
#endif

    /* USB clock configuration - must be operated from 48 MHz */
    {
        uint32_t i = XPD_RCC_GetOscFreq(PLL);
        switch (XPD_RCC_GetOscFreq(PLL))
        {
            case 72000000:
                XPD_USB_ClockConfig(USB_CLOCKSOURCE_PLL_DIV1p5);
                break;

            case 48000000:
                XPD_USB_ClockConfig(USB_CLOCKSOURCE_PLL);
                break;

            default: /* disable USB clock when no acceptable configuration is available */
                return USBD_FAIL;
        }
    }

#if defined (USE_USB_INTERRUPT_REMAPPED)
    /*USB interrupt remapping enable */
    XPD_USB_ITRemap(ENABLE);
#endif

#if defined (USE_USB_INTERRUPT_REMAPPED)
    /* Set USB Remapped FS Interrupt priority */
    XPD_NVIC_SetPriorityConfig(USB_HP_IRQn, 2, 0);
    XPD_NVIC_EnableIRQ(USB_HP_IRQn);

    /* Set USB Remapped FS Interrupt priority */
    XPD_NVIC_SetPriorityConfig(USB_LP_IRQn, 2, 0);
    XPD_NVIC_EnableIRQ(USB_LP_IRQn);
#else
    /* Set USB Default FS Interrupt priority */
    XPD_NVIC_SetPriorityConfig(USB_LP_CAN_RX0_IRQn, 2, 0);
    XPD_NVIC_EnableIRQ(USB_LP_CAN_RX0_IRQn);

    /* Set USB Default FS Interrupt priority */
    XPD_NVIC_SetPriorityConfig(USB_HP_CAN_TX_IRQn, 2, 0);
    XPD_NVIC_EnableIRQ(USB_HP_CAN_TX_IRQn);
#endif

    /* Link driver to user */
    pdev->pData = &husb;

    /* USB init setup */
    {
        USB_InitType init;
        init.SOF = ENABLE;
        init.LowPowerMode = DISABLE;
        init.LinkPowerMgmt = DISABLE;

        /* Link the stack to the driver */
        husb.User = pdev;

        husb.Callbacks.SetupStage       = USBD_LL_SetupStage;
        husb.Callbacks.DataOutStage     = USBD_LL_DataOutStage;
        husb.Callbacks.DataInStage      = USBD_LL_DataInStage;
        husb.Callbacks.SOF              = USBD_LL_SOF;
        husb.Callbacks.Reset            = USBD_LL_Reset;
        husb.Callbacks.Suspend          = USBD_LL_Suspend;
        husb.Callbacks.Resume           = USBD_LL_Resume;
        husb.Callbacks.Connected        = USBD_LL_DevConnected;
        husb.Callbacks.Disconnected     = USBD_LL_DevDisconnected;
        husb.Callbacks.ISOOUTIncomplete = USBD_LL_IsoOUTIncomplete;
        husb.Callbacks.ISOINIncomplete  = USBD_LL_IsoINIncomplete;

        XPD_USB_Init(pdev->pData, &init);
    }

    /* SETUP endpoints */
    XPD_USB_PMAConfig(pdev->pData, 0x00, 0x18, DISABLE);
    XPD_USB_PMAConfig(pdev->pData, 0x80, 0x58, DISABLE);

    /* Endpoints for CDC device */
    XPD_USB_PMAConfig(pdev->pData, CDC_IN_EP, 0xC0, DISABLE);
    XPD_USB_PMAConfig(pdev->pData, CDC_OUT_EP, 0x110, DISABLE);
    XPD_USB_PMAConfig(pdev->pData, CDC_CMD_EP, 0x100, DISABLE);

    pdev->dev_speed = USBD_SPEED_FULL;

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
    XPD_USB_EP_Open(pdev->pData, ep_addr, ep_type, ep_mps);

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
