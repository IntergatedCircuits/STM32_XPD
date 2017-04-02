/**
  ******************************************************************************
 * @file    xpd_usb.c
 * @author  Benedek Kupper
 * @version V0.1
 * @date    2017-02-07
 * @brief   STM32 eXtensible Peripheral Drivers Universal Serial Bus Module
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

#include "xpd_usb.h"
#include "xpd_rcc.h"

#if defined(USE_XPD_USB) && defined(USB)

/** @addtogroup USB
 * @{ */

#ifdef USB_LPMCSR_LMPEN
typedef struct {
    struct {
        __IO uint16_t TX_ADDR;
        __IO uint16_t TX_COUNT;
        __IO uint16_t RX_ADDR;
        __IO uint16_t RX_COUNT;
    }BDT[8];
}USB_BufferDescriptorTable_TypeDef;
#else
typedef struct {
    struct {
        __IO uint32_t TX_ADDR;
        __IO uint32_t TX_COUNT;
        __IO uint32_t RX_ADDR;
        __IO uint32_t RX_COUNT;
    }BDT[8];
}USB_BufferDescriptorTable_TypeDef;
#endif

#define USB_EP_BDT(EP_NUM) (((USB_BufferDescriptorTable_TypeDef *)(USB_PMAADDR + USB->BTABLE))->BDT[EP_NUM])

#define USB_TOGGLE(EP_NUM, BIT_NAME) \
    (USB->EP[EP_NUM].w = (USB->EP[EP_NUM].w & USB_EPREG_MASK) | USB_EP_CTR_RX | USB_EP_CTR_TX | USB_EP_ ## BIT_NAME)

/* if the DTOG bit is not 1, write a 1 to it to toggle back to 0 */
#define USB_TOGGLE_CLEAR(EP_NUM, BIT_NAME) \
do { if (USB_REG_BIT(EP[EP_NUM],BIT_NAME) != 0) USB_TOGGLE(EP_NUM, BIT_NAME); }while(0)

#define USB_EPRX_TX_DTOG1    (USB_EPRX_DTOG1 | USB_EPTX_DTOG1)
#define USB_EPRX_TX_DTOG2    (USB_EPRX_DTOG2 | USB_EPTX_DTOG2)
#define USB_EPRX_TX_DTOGMASK (USB_EPRX_DTOGMASK | USB_EPTX_DTOGMASK)
#define USB_EP_RX_TX_STALL   (USB_EP_RX_STALL | USB_EP_TX_STALL)

#define USB_SET_STATUS(EP_NUM, DIRECTION, STATUS) \
    do { __IO uint16_t epreg = USB->EP[EP_NUM].w & USB_EP##DIRECTION##_DTOGMASK; \
    if (USB_EP_##DIRECTION##_##STATUS & USB_EP##DIRECTION##_DTOG1) epreg ^= USB_EP##DIRECTION##_DTOG1; \
    if (USB_EP_##DIRECTION##_##STATUS & USB_EP##DIRECTION##_DTOG2) epreg ^= USB_EP##DIRECTION##_DTOG2; \
    USB->EP[EP_NUM].w = epreg | USB_EP_CTR_RX | USB_EP_CTR_TX; } while(0)

#define EP_IS_IN(HANDLE,ENDPOINT)   (((uint32_t)(ENDPOINT)) < ((uint32_t)(&(HANDLE)->EP.OUT[0])))
#define EP_IS_OUT(HANDLE,ENDPOINT)  (!EP_IS_IN(HANDLE,ENDPOINT))

#define USB_GET_EP_AT(HANDLE, NUMBER)  \
    (((NUMBER) > 0x7F) ? (&(HANDLE)->EP.IN[(NUMBER) &= 0x7F]) : (&(HANDLE)->EP.OUT[NUMBER]))

static const uint16_t usb_epTypeRemap[4] = {USB_EP_CONTROL, USB_EP_ISOCHRONOUS, USB_EP_BULK, USB_EP_INTERRUPT};

static void usb_writePMA(uint8_t * sourceBuf, uint16_t pmaAddress, uint16_t dataCount)
{
    /* PMA stores data in 16 bit elements */
    uint16_t i = (dataCount + 1) / 2;
#ifdef USB_LPMCSR_LMPEN
    uint16_t * dest = (uint16_t *)(USB_PMAADDR + pmaAddress);
#else
    uint32_t * dest = (uint32_t *)(USB_PMAADDR + pmaAddress * 2);
#endif

    while (i--)
    {
        *dest = (uint16_t)(sourceBuf[0]) | ((uint16_t)(sourceBuf[1]) << 8);
        dest++;
        sourceBuf += 2;
    }
}

static void usb_readPMA(uint8_t * destBuf, uint16_t pmaAddress, uint16_t dataCount)
{
    uint16_t i = (dataCount + 1) / 2;
#ifdef USB_LPMCSR_LMPEN
    uint16_t * source = (uint16_t *)(USB_PMAADDR + pmaAddress);
#else
    uint32_t * source = (uint32_t *)(USB_PMAADDR + pmaAddress * 2);
#endif
    while (i--)
    {
        *(uint16_t*) destBuf = *source;
        source++;
        destBuf += 2;
    }
}

/* Setting RX_COUNT requires special conversion */
static uint16_t usb_epConvertRxCount(uint16_t count)
{
    uint16_t blocks;
    if(count > 62)
    {
        blocks = count >> 5;
        if ((count & 0x1f) == 0)
        {
            blocks--;
        }
        blocks = (uint16_t)((blocks << 10) | 0x8000);
    }
    else
    {
        blocks = count >> 1;
        if((count & 0x1) != 0)
        {
            blocks++;
        }
        blocks = (uint16_t)(blocks << 10);
    }
    return blocks;
}

/** @defgroup USB_Exported_Functions USB Exported Functions
 * @{ */
 
/**
 * @brief Initializes the USB peripheral using the setup configuration
 * @param husb: pointer to the USB handle structure
 * @param Config: USB setup configuration
 * @return ERROR if input is incorrect, OK if success
 */
XPD_ReturnType XPD_USB_Init(USB_HandleType * husb, const USB_InitType * Config)
{
    uint32_t i = 0;

    /* Enable peripheral clock */
    XPD_USB_ClockCtrl(ENABLE);

    /* Init endpoints structures (USB FS has 8 endpoints) */
    for (i = 0; i < (sizeof(USB->EP) / sizeof(USB->EP[0])); i++)
    {
        /* Control type until ep is activated */
        husb->EP.IN[i].Type             = USB_EP_TYPE_CONTROL;
        husb->EP.OUT[i].Type              = USB_EP_TYPE_CONTROL;
        husb->EP.IN[i].MaxPacketSize    = husb->EP.OUT[i].MaxPacketSize     = 0;
        husb->EP.IN[i].Transfer.buffer  = husb->EP.OUT[i].Transfer.buffer   = 0;
        husb->EP.IN[i].Transfer.length  = husb->EP.OUT[i].Transfer.length   = 0;
    }

    /* Initialize peripheral device */
    /* FRES = 1 */
    USB->CNTR.w = USB_CNTR_FRES;

    /* FRES = 0 */
    USB->CNTR.w = 0;

    /*Clear pending interrupts */
    USB->ISTR.w = 0;

    /* Set Btable Address */
    USB->BTABLE = 0;

    /*Set interrupt mask */
    USB->CNTR.w = USB_CNTR_CTRM | USB_CNTR_WKUPM | USB_CNTR_SUSPM
                | USB_CNTR_ERRM | USB_CNTR_ESOFM | USB_CNTR_RESETM
                | USB_CNTR_SOFM;

#ifdef USB_LPMCSR_LMPEN
    USB->LPMCSR.b.LMPEN = Config->LinkPowerMgmt;
#endif

    husb->DeviceAddress = 0;

	return XPD_OK;
}

/**
 * @brief Restores the USB peripheral to its default inactive state
 * @param husb: pointer to the USB handle structure
 * @return ERROR if input is incorrect, OK if success
 */
XPD_ReturnType XPD_USB_Deinit(USB_HandleType * husb)
{
    /* Stop Device */
    XPD_USB_Stop(husb);

    XPD_USB_ClockCtrl(DISABLE);

	return XPD_OK;
}

/**
 * @brief Starts the USB device operation
 * @param husb: pointer to the USB handle structure
 */
void XPD_USB_Start(USB_HandleType * husb)
{
    /* activate DP line pull up */
#ifdef USB_BCDR_DPPU
    USB_REG_BIT(BCDR, DPPU) = 1;
#else
    XPD_SAFE_CALLBACK(husb->Callbacks.ConnectionStateCtrl, ENABLE);
#endif
}

/**
 * @brief Stops the USB device operation
 * @param husb: pointer to the USB handle structure
 */
void XPD_USB_Stop(USB_HandleType * husb)
{
    /* disable all interrupts and force USB reset */
    USB->CNTR.w = USB_CNTR_FRES;

    /* clear interrupt status register */
    USB->ISTR.w = 0;

    /* switch-off device */
    USB_REG_BIT(CNTR, PDWN) = 1;
}

/**
 * @brief Connects the device to the USB host
 * @param husb: pointer to the USB handle structure
 */
void XPD_USB_DevConnect(USB_HandleType * husb)
{
    /* activate DP line pull up */
#ifdef USB_BCDR_DPPU
    USB_REG_BIT(BCDR, DPPU) = 1;
#else
    XPD_SAFE_CALLBACK(husb->Callbacks.ConnectionStateCtrl, ENABLE);
#endif
}

/**
 * @brief Disconnects the device from the USB host
 * @param husb: pointer to the USB handle structure
 */
void XPD_USB_DevDisconnect(USB_HandleType * husb)
{
    /* deactivate DP line pull up */
#ifdef USB_BCDR_DPPU
    USB_REG_BIT(BCDR, DPPU) = 0;
#else
    XPD_SAFE_CALLBACK(husb->Callbacks.ConnectionStateCtrl, DISABLE);
#endif

}

/**
 * @brief Activates remote wake-up signaling
 * @param husb: pointer to the USB handle structure
 */
void XPD_USB_ActivateRemoteWakeup(USB_HandleType * husb)
{
#ifdef USB_CNTR_L1RESUME
    if (USB->LPMCSR.b.LMPEN != 0)
    {
        USB_REG_BIT(CNTR, L1RESUME) = 1;
    }
    else
#endif
    {
        USB_REG_BIT(CNTR, RESUME) = 1;
    }
}

/**
 * @brief Deactivates remote wake-up signaling
 * @param husb: pointer to the USB handle structure
 */
void XPD_USB_DeActivateRemoteWakeup(USB_HandleType * husb)
{
#ifdef USB_CNTR_L1RESUME
    if (USB->LPMCSR.b.LMPEN != 0)
    {
        USB_REG_BIT(CNTR, L1RESUME) = 0;
    }
    else
#endif
    {
        USB_REG_BIT(CNTR, RESUME) = 0;
    }
}

/**
 * @brief Sets the USB device address
 * @param husb: pointer to the USB handle structure
 * @param Address: new device address
 */
void XPD_USB_SetAddress(USB_HandleType * husb, uint8_t Address)
{
    if (Address == 0)
    {
        /* set device address and enable function */
        USB->DADDR.w = USB_DADDR_EF;
    }
    else
    {
        /* USB Address will be applied later */
        husb->DeviceAddress = Address;
    }
}

/**
 * @brief Opens an endpoint
 * @param husb: pointer to the USB handle structure
 * @param EpNumber: endpoint number
 * @param Type: endpoint type
 * @param MaxPacketSize: endpoint maximum data packet size
 */
void XPD_USB_EP_Open(USB_HandleType * husb, uint8_t EpNumber, USB_EP_PacketType Type, uint16_t MaxPacketSize)
{
    USB_EndPointType * ep = USB_GET_EP_AT(husb, EpNumber);

    ep->MaxPacketSize = MaxPacketSize;
    ep->Type = Type;

    /* Configure EP */
    USB->EP[EpNumber].w = (USB->EP[EpNumber].w & USB_EP_T_MASK)
            | usb_epTypeRemap[ep->Type];

    USB->EP[EpNumber].w = (USB->EP[EpNumber].w & USB_EPREG_MASK)
            | USB_EP_CTR_RX | USB_EP_CTR_TX | EpNumber;

    if (ep->DoubleBuffer == DISABLE)
    {
        if (EP_IS_IN(husb,ep))
        {
            /*Set the endpoint Transmit buffer address */
            USB_EP_BDT(EpNumber).TX_ADDR = ep->PMAAddress0 & 0xFFFE;
            USB_TOGGLE_CLEAR(EpNumber, DTOG_TX);

            /* Configure NAK status for the Endpoint */
            USB_SET_STATUS(EpNumber, TX, NAK);
        }
        else
        {
            /*Set the endpoint Receive buffer address */
            USB_EP_BDT(EpNumber).RX_ADDR = ep->PMAAddress0 & 0xFFFE;
            /*Set the endpoint Receive buffer counter*/
            USB_EP_BDT(EpNumber).RX_COUNT = usb_epConvertRxCount(ep->MaxPacketSize);
            USB_TOGGLE_CLEAR(EpNumber, DTOG_RX);

            /* Configure VALID status for the Endpoint */
            USB_SET_STATUS(EpNumber, RX, VALID);
        }
    }
    /* Double buffer */
    else
    {
        /*Set the endpoint as double buffered */
        USB->EP[EpNumber].w =  (USB->EP[EpNumber].w & USB_EPREG_MASK) | USB_EP_CTR_RX | USB_EP_CTR_TX | USB_EP_KIND;

        /*Set buffer address for double buffered mode*/
        USB_EP_BDT(EpNumber).TX_ADDR = ep->PMAAddress0 & 0xFFFE;
        USB_EP_BDT(EpNumber).RX_ADDR = ep->PMAAddress1 & 0xFFFE;

        if (EP_IS_OUT(husb,ep))
        {
            /* Clear the data toggle bits for the endpoint IN/OUT */
            USB_TOGGLE_CLEAR(EpNumber, DTOG_RX);
            USB_TOGGLE_CLEAR(EpNumber, DTOG_TX);

            /* Reset value of the data toggle bits for the endpoint out */
            USB_TOGGLE(EpNumber, DTOG_TX);

            USB_SET_STATUS(EpNumber, RX, VALID);
            USB_SET_STATUS(EpNumber, TX, DIS);
        }
        else
        {
            /* Clear the data toggle bits for the endpoint IN/OUT */
            USB_TOGGLE_CLEAR(EpNumber, DTOG_RX);
            USB_TOGGLE_CLEAR(EpNumber, DTOG_TX);
            USB_TOGGLE(EpNumber, DTOG_RX);

            /* Configure DISABLE status for the Endpoint */
            USB_SET_STATUS(EpNumber, TX, DIS);
            USB_SET_STATUS(EpNumber, RX, DIS);
        }
    }
}

/**
 * @brief Closes an active endpoint
 * @param husb: pointer to the USB handle structure
 * @param EpNumber: endpoint number
 */
void XPD_USB_EP_Close(USB_HandleType * husb, uint8_t EpNumber)
{
    USB_EndPointType * ep = USB_GET_EP_AT(husb, EpNumber);

    if (ep->DoubleBuffer == DISABLE)
    {
        if (EP_IS_IN(husb,ep))
        {
            USB_TOGGLE_CLEAR(EpNumber, DTOG_TX);
            /* Configure DISABLE status for the Endpoint*/
            USB_SET_STATUS(EpNumber, TX, DIS);
        }
        else
        {
            USB_TOGGLE_CLEAR(EpNumber, DTOG_RX);
            /* Configure DISABLE status for the Endpoint*/
            USB_SET_STATUS(EpNumber, RX, DIS);
        }
    }
    /* Double buffer */
    else
    {
        /* Clear the data toggle bits for the endpoint IN/OUT*/
        USB_TOGGLE_CLEAR(EpNumber, DTOG_RX);
        USB_TOGGLE_CLEAR(EpNumber, DTOG_TX);

        if (EP_IS_OUT(husb,ep))
        {
            USB_TOGGLE(EpNumber, DTOG_TX);
        }
        else
        {
            USB_TOGGLE(EpNumber, DTOG_RX);
        }

        /* Configure DISABLE status for the Endpoint*/
        USB_SET_STATUS(EpNumber, TX, DIS);
        USB_SET_STATUS(EpNumber, RX, DIS);
    }
}

/**
 * @brief Initiates data reception on the OUT endpoint
 * @param husb: pointer to the USB handle structure
 * @param EpNumber: endpoint number
 * @param Data: pointer to the data buffer
 * @param Length: amount of data bytes to transfer
 */
void XPD_USB_EP_Receive(USB_HandleType * husb, uint8_t EpNumber, uint8_t * Data, uint16_t Length)
{
    USB_EndPointType * ep = &husb->EP.OUT[EpNumber];

    /* setup transfer */
    ep->Transfer.buffer = Data;
    ep->Transfer.size = 0;

    /* Multi packet transfer*/
    if (Length > ep->MaxPacketSize)
    {
        ep->Transfer.length = Length - ep->MaxPacketSize;
        Length = ep->MaxPacketSize;
    }
    else
    {
        ep->Transfer.length = 0;
    }

    /* configure and validate Rx endpoint */
    if (ep->DoubleBuffer == DISABLE)
    {
        /*Set RX buffer count*/
        USB_EP_BDT(EpNumber).RX_COUNT = usb_epConvertRxCount(Length);
    }
    else
    {
        /*Set the Double buffer counter */
        USB_EP_BDT(EpNumber).TX_COUNT = Length;
    }

    USB_SET_STATUS(EpNumber, RX, VALID);
}

/**
 * @brief  Returns the OUT endpoint's received data size
 * @param  husb: pointer to the USB handle structure
 * @param  EpNumber: endpoint number
 * @return The number of received bytes
 */
uint16_t XPD_USB_EP_GetRxCount(USB_HandleType * husb, uint8_t EpNumber)
{
  return husb->EP.OUT[EpNumber].Transfer.size;
}

/**
 * @brief Initiates data transmission on the IN endpoint
 * @param husb: pointer to the USB handle structure
 * @param EpNumber: endpoint number
 * @param Data: pointer to the data buffer
 * @param Length: amount of data bytes to transfer
 */
void XPD_USB_EP_Transmit(USB_HandleType * husb, uint8_t EpNumber, uint8_t * Data, uint16_t Length)
{
    USB_EndPointType * ep = &husb->EP.IN[EpNumber & 0x7F];
    uint16_t pmabuffer = 0;

    /*setup and start the Xfer */
    ep->Transfer.buffer = Data;
    ep->Transfer.size = 0;

    /* Multi packet transfer*/
    if (Length > ep->MaxPacketSize)
    {
        ep->Transfer.length = Length - ep->MaxPacketSize;
        Length = ep->MaxPacketSize;
    }
    else
    {
        ep->Transfer.length = 0;
    }

    /* configure and validate Tx endpoint */
    if (ep->DoubleBuffer == DISABLE)
    {
        usb_writePMA(ep->Transfer.buffer, ep->PMAAddress0, Length);
        USB_EP_BDT(EpNumber).TX_COUNT = Length;
    }
    else
    {
        /*Write the data to the USB endpoint*/
        if (USB->EP[EpNumber].w & USB_EP_DTOG_TX)
        {
            USB_EP_BDT(EpNumber).TX_COUNT = Length;
            pmabuffer = ep->PMAAddress1;
        }
        else
        {
            USB_EP_BDT(EpNumber).TX_COUNT = Length;
            pmabuffer = ep->PMAAddress0;
        }
        usb_writePMA(ep->Transfer.buffer, pmabuffer, Length);
        USB_TOGGLE(EpNumber, DTOG_RX);
    }

    USB_SET_STATUS(EpNumber, TX, VALID);
}

/**
 * @brief Set a STALL condition on an endpoint
 * @param husb: pointer to the USB handle structure
 * @param EpNumber: endpoint number
 */
void XPD_USB_EP_SetStall(USB_HandleType * husb, uint8_t EpNumber)
{
    USB_EndPointType * ep = USB_GET_EP_AT(husb, EpNumber);

    if (EpNumber == 0)
    {
        /* This macro sets STALL status for RX & TX*/
        USB_SET_STATUS(EpNumber, RX_TX, STALL);
    }
    else
    {
        if (EP_IS_IN(husb,ep))
        {
            USB_SET_STATUS(EpNumber, TX, STALL);
        }
        else
        {
            USB_SET_STATUS(EpNumber, RX, STALL);
        }
    }
    ep->Stalled = TRUE;
}

/**
 * @brief Clear a STALL condition on an endpoint
 * @param husb: pointer to the USB handle structure
 * @param EpNumber: endpoint number
 */
void XPD_USB_EP_ClearStall(USB_HandleType * husb, uint8_t EpNumber)
{
    USB_EndPointType * ep = USB_GET_EP_AT(husb, EpNumber);

    if (EP_IS_IN(husb,ep))
    {
        USB_TOGGLE_CLEAR(EpNumber, DTOG_TX);
        USB_SET_STATUS(EpNumber, TX, VALID);
    }
    else
    {
        USB_TOGGLE_CLEAR(EpNumber, DTOG_RX);
        USB_SET_STATUS(EpNumber, RX, VALID);
    }
    ep->Stalled = FALSE;
}

/**
 * @brief USB interrupt handler that provides event-driven peripheral management and handle callbacks.
 * @param husb: pointer to the USB handle structure
 */
void XPD_USB_IRQHandler(USB_HandleType * husb)
{
    uint32_t istr;
    USB_EndPointType * ep;
    uint16_t epReg;
    uint8_t EpNumber;

    /* loop while Endpoint interrupts are present */
    for (istr = USB->ISTR.w; (istr & USB_ISTR_CTR) != 0; istr = USB->ISTR.w)
    {
        /* Read highest priority endpoint number */
        EpNumber = (uint8_t)(istr & USB_ISTR_EP_ID);

        /* Control endpoint interrupt */
        if (EpNumber == 0)
        {
            /* DIR bit indicates IN data */
            if ((istr & USB_ISTR_DIR) == 0)
            {
                /* DIR = 0 implies that (EP_CTR_TX = 1) always */
                USB->EP[0].w &= USB_EPREG_MASK & (~USB_EP_CTR_TX);
                ep = &husb->EP.IN[0];

                ep->Transfer.size = USB_EP_BDT(EpNumber).TX_COUNT & 0x3FF;
                ep->Transfer.buffer += ep->Transfer.size;

                /* IN packet successfully sent */
                XPD_SAFE_CALLBACK(husb->Callbacks.DataInStage, husb->User, 0, ep->Transfer.buffer);

                /* Set device address if valid */
                if ((husb->DeviceAddress > 0) && (ep->Transfer.length == 0))
                {
                    USB->DADDR.w = (husb->DeviceAddress | USB_DADDR_EF);
                    husb->DeviceAddress = 0;
                }
            }
            else
            {
                /* DIR bit indicates SETUP or OUT data */
                ep = &husb->EP.OUT[0];
                epReg = USB->EP[0].w;

                ep->Transfer.size = USB_EP_BDT(EpNumber).RX_COUNT & 0x3FF;

                /* Get SETUP Packet */
                if ((epReg & USB_EP_SETUP) != 0)
                {
                    usb_readPMA((uint8_t*) husb->Setup, ep->PMAAddress0, ep->Transfer.size);

                    /* SETUP bit kept frozen while CTR_RX = 1*/
                    USB->EP[0].w &= USB_EPREG_MASK & (~USB_EP_CTR_RX);

                    /* Process SETUP Packet */
                    XPD_SAFE_CALLBACK(husb->Callbacks.SetupStage, husb->User, (uint8_t *)husb->Setup);
                }

                /* Get Control Data OUT Packet */
                else if ((epReg & USB_EP_CTR_RX) != 0)
                {
                    USB->EP[0].w &= USB_EPREG_MASK & (~USB_EP_CTR_RX);

                    if (ep->Transfer.size > 0)
                    {
                        usb_readPMA(ep->Transfer.buffer, ep->PMAAddress0, ep->Transfer.size);
                        ep->Transfer.buffer += ep->Transfer.size;
                    }

                    /* Process Control Data OUT Packet*/
                    XPD_SAFE_CALLBACK(husb->Callbacks.DataOutStage, husb->User, 0, ep->Transfer.buffer);

                    /* Setup new OUT transfer */
                    USB_EP_BDT(0).RX_COUNT = usb_epConvertRxCount(ep->MaxPacketSize);
                    USB_SET_STATUS(0, RX, VALID);
                }
            }
        }
        else
        {
            /* Non control endpoints interrupt processing */
            epReg = USB->EP[EpNumber].w;

            /* OUT data received */
            if ((epReg & USB_EP_CTR_RX) != 0)
            {
                uint16_t count;
                ep = &husb->EP.OUT[EpNumber];

                /* clear flag */
                USB->EP[EpNumber].w &= USB_EPREG_MASK & (~USB_EP_CTR_RX);

                if (ep->DoubleBuffer == DISABLE)
                {
                    count = USB_EP_BDT(EpNumber).RX_COUNT & 0x3FF;
                    if (count > 0)
                    {
                        usb_readPMA(ep->Transfer.buffer, ep->PMAAddress0, count);
                    }
                }
                /* Double buffering */
                else
                {
                    if (epReg & USB_EP_DTOG_RX)
                    {
                        /* read from endpoint 0 buffer */
                        count = USB_EP_BDT(EpNumber).TX_COUNT & 0x3FF;
                        if (count > 0)
                        {
                            usb_readPMA(ep->Transfer.buffer, ep->PMAAddress0, count);
                        }
                    }
                    else
                    {
                        /* read from endpoint 1 buffer */
                        count = USB_EP_BDT(EpNumber).RX_COUNT & 0x3FF;
                        if (count > 0)
                        {
                            usb_readPMA(ep->Transfer.buffer, ep->PMAAddress1, count);
                        }
                    }
                    /* Switch the reception buffer */
                    USB_TOGGLE(EpNumber, DTOG_TX);
                }

                ep->Transfer.size += count;
                ep->Transfer.buffer += count;

                /* If the last packet of the data */
                if ((ep->Transfer.length == 0) || (count < ep->MaxPacketSize))
                {
                    /* Reception finished */
                    XPD_SAFE_CALLBACK(husb->Callbacks.DataOutStage, husb->User, EpNumber, ep->Transfer.buffer);
                }
                else
                {
                    /* Continue data reception */
                    XPD_USB_EP_Receive(husb, EpNumber, ep->Transfer.buffer, ep->Transfer.length);
                }
            }

            /* IN data sent */
            if ((epReg & USB_EP_CTR_TX) != 0)
            {
                ep = &husb->EP.IN[EpNumber];

                /* clear flag */
                USB->EP[EpNumber].w &= USB_EPREG_MASK & (~USB_EP_CTR_TX);

                if (ep->DoubleBuffer == DISABLE)
                {
                    ep->Transfer.size = USB_EP_BDT(EpNumber).TX_COUNT & 0x3FF;
                    if (ep->Transfer.size > 0)
                    {
                        usb_writePMA(ep->Transfer.buffer, ep->PMAAddress0, ep->Transfer.size);
                    }
                }
                /* Double buffering */
                else
                {
                    if (epReg & USB_EP_DTOG_TX)
                    {
                        /* write to endpoint 0 buffer */
                        ep->Transfer.size = USB_EP_BDT(EpNumber).TX_COUNT & 0x3FF;
                        if (ep->Transfer.size > 0)
                        {
                            usb_writePMA(ep->Transfer.buffer, ep->PMAAddress0, ep->Transfer.size);
                        }
                    }
                    else
                    {
                        /* write to endpoint 1 buffer */
                        ep->Transfer.size = USB_EP_BDT(EpNumber).RX_COUNT & 0x3FF;
                        if (ep->Transfer.size > 0)
                        {
                            usb_writePMA(ep->Transfer.buffer, ep->PMAAddress1, ep->Transfer.size);
                        }
                    }
                    /* Switch the transmission buffer */
                    USB_TOGGLE(EpNumber, DTOG_RX);
                }

                ep->Transfer.size = USB_EP_BDT(EpNumber).TX_COUNT & 0x3FF;
                ep->Transfer.buffer += ep->Transfer.size;

                /* If the last packet of the data */
                if (ep->Transfer.length == 0)
                {
                    /* Transmission complete */
                    XPD_SAFE_CALLBACK(husb->Callbacks.DataInStage, husb->User, EpNumber, ep->Transfer.buffer);
                }
                else
                {
                    /* Continue data transmission */
                    XPD_USB_EP_Transmit(husb, EpNumber, ep->Transfer.buffer, ep->Transfer.length);
                }
            }
        }
    }

    if ((istr & USB_ISTR_RESET) != 0)
    {
        XPD_USB_ClearFlag(husb, RESET);
        XPD_SAFE_CALLBACK(husb->Callbacks.Reset, husb->User);
        XPD_USB_SetAddress(husb, 0);
    }

    if ((istr & USB_ISTR_WKUP) != 0)
    {
        /*Set interrupt mask*/
        USB->CNTR.w = USB_CNTR_CTRM | USB_CNTR_WKUPM | USB_CNTR_SUSPM | USB_CNTR_ERRM | USB_CNTR_ESOFM | USB_CNTR_RESETM;

        XPD_SAFE_CALLBACK(husb->Callbacks.Resume, husb->User);

        XPD_USB_ClearFlag(husb, WKUP);
    }

    if ((istr & USB_ISTR_SUSP) != 0)
    {
        /* clear of the ISTR bit must be done after setting of CNTR_FSUSP */
        XPD_USB_ClearFlag(husb, SUSP);

        /* Force low-power mode in the macrocell */
        USB->CNTR.w |= USB_CNTR_FSUSP | USB_CNTR_LPMODE;

        if (XPD_USB_GetFlag(husb, WKUP) == 0)
        {
            XPD_SAFE_CALLBACK(husb->Callbacks.Suspend, husb->User);
        }
    }

    if ((istr & USB_ISTR_SOF) != 0)
    {
        XPD_USB_ClearFlag(husb, SOF);
        XPD_SAFE_CALLBACK(husb->Callbacks.SOF, husb->User);
    }

    if ((istr & USB_ISTR_PMAOVR) != 0)
    {
        XPD_USB_ClearFlag(husb, PMAOVR);
    }

    if ((istr & USB_ISTR_ERR) != 0)
    {
        XPD_USB_ClearFlag(husb, ERR);
    }

    if ((istr & USB_ISTR_ESOF) != 0)
    {
        XPD_USB_ClearFlag(husb, ESOF);
    }
}

/**
 * @brief Configure Packet Memory Address for EndPoint
 * @param husb: pointer to the USB handle structure
 * @param EpNumber: endpoint number
 * @param DoubleBuffer: specifies if the endpoint is double-buffered
 * @param PmaAdress: Endpoint data address in the PMA
 *        - In case of single buffer endpoint it is a 16-bit value providing the PMA address
 *        - In case of double buffer endpoint it is a 32-bit value providing the endpoint buffer 0 address
 *          in the LSB part of 32-bit value and endpoint buffer 1 address in the MSB part of 32-bit value.
 */
void XPD_USB_PMAConfig(USB_HandleType * husb, uint8_t EpNumber, uint32_t PmaAdress, FunctionalState DoubleBuffer)
{
    USB_EndPointType * ep = USB_GET_EP_AT(husb, EpNumber);

    /*Double buffer setting for Endpoint */
    ep->DoubleBuffer = DoubleBuffer;

    /*Configure the PMAs */
    ep->PMAAddress0 = PmaAdress & 0xFFFF;
    ep->PMAAddress1 = PmaAdress >> 16;
}

/** @} */

/** @} */

#endif /* USE_XPD_USB */
