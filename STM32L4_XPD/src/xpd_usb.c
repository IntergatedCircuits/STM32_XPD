/**
  ******************************************************************************
  * @file    xpd_usb.c
  * @author  Benedek Kupper
  * @version V0.1
  * @date    2017-04-23
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
#include "xpd_utils.h"

#if   defined(USE_XPD_USB) && defined(USB)

/** @addtogroup USB
 * @{ */

#define USB_MAX_PACKET_SIZE     64
#define USB_ENDPOINT_COUNT      (sizeof(USB->EP) / sizeof(USB->EP[0])) /* 8 */

#ifdef USB_LPMCSR_LMPEN
typedef struct {
    __IO uint16_t TX_ADDR;
    __IO uint16_t TX_COUNT;
    __IO uint16_t RX_ADDR;
    __IO uint16_t RX_COUNT;
}USB_BufferDescriptor_TypeDef;

typedef uint16_t USB_PacketAddressType;
#else
typedef struct {
    __IO uint32_t TX_ADDR;
    __IO uint32_t TX_COUNT;
    __IO uint32_t RX_ADDR;
    __IO uint32_t RX_COUNT;
}USB_BufferDescriptor_TypeDef;

typedef uint32_t USB_PacketAddressType;
#endif

#define USB_EP_BDT ((USB_BufferDescriptor_TypeDef *)(USB_PMAADDR + USB->BTABLE))

#define USB_TOGGLE(EP_ID, BIT_NAME) \
    (USB->EP[EP_ID].w = (USB->EP[EP_ID].w & USB_EPREG_MASK) | USB_EP_CTR_RX | USB_EP_CTR_TX | USB_EP_ ## BIT_NAME)

/* if the DTOG bit is not 1, write a 1 to it to toggle back to 0 */
#define USB_TOGGLE_CLEAR(EP_ID, BIT_NAME) \
do { if (USB_REG_BIT(husb,EP[EP_ID],BIT_NAME) != 0) USB_TOGGLE(EP_ID, BIT_NAME); }while(0)

#define USB_EPRX_TX_DTOG1    (USB_EPRX_DTOG1 | USB_EPTX_DTOG1)
#define USB_EPRX_TX_DTOG2    (USB_EPRX_DTOG2 | USB_EPTX_DTOG2)
#define USB_EPRX_TX_DTOGMASK (USB_EPRX_DTOGMASK | USB_EPTX_DTOGMASK)
#define USB_EP_RX_TX_STALL   (USB_EP_RX_STALL | USB_EP_TX_STALL)

#define USB_EP_SET_STATUS(EP_ID, DIRECTION, STATUS) \
    do { __IO uint16_t epreg = USB->EP[EP_ID].w & USB_EP##DIRECTION##_DTOGMASK; \
    if (USB_EP_##DIRECTION##_##STATUS & USB_EP##DIRECTION##_DTOG1) epreg ^= USB_EP##DIRECTION##_DTOG1; \
    if (USB_EP_##DIRECTION##_##STATUS & USB_EP##DIRECTION##_DTOG2) epreg ^= USB_EP##DIRECTION##_DTOG2; \
    USB->EP[EP_ID].w = epreg | USB_EP_CTR_RX | USB_EP_CTR_TX; } while(0)

#define USB_CLEAR_EP_FLAG(EP_ID, FLAG_NAME)     \
        (USB->EP[EP_ID].w &= USB_EPREG_MASK & (~USB_EP_ ## FLAG_NAME))

#define EP_IS_IN(HANDLE,ENDPOINT)   (((uint32_t)(ENDPOINT)) < ((uint32_t)(&(HANDLE)->EP.OUT[0])))
#define EP_IS_OUT(HANDLE,ENDPOINT)  (!EP_IS_IN(HANDLE,ENDPOINT))

#define USB_GET_EP_AT(HANDLE, NUMBER)  \
    (((NUMBER) > 0x7F) ? (&((HANDLE)->EP.IN[(NUMBER) &= 0x7F])) : (&((HANDLE)->EP.OUT[NUMBER])))

#define USB_PMA_ALLOCATION(SIZE)    (SIZE)

static const uint16_t usb_epTypeRemap[4] = {
    USB_EP_CONTROL,
    USB_EP_ISOCHRONOUS,
    USB_EP_BULK,
    USB_EP_INTERRUPT
};

/* Writes user data to USB endpoint packet memory */
static void usb_writePMA(uint8_t * sourceBuf, uint16_t pmaAddress, uint16_t dataCount)
{
    USB_PacketAddressType * dest = (USB_PacketAddressType *)USB_PMAADDR + (pmaAddress / 2);

    /* Check if input data is aligned */
    if ((((uint32_t)sourceBuf) & 1) == 0)
    {
        uint16_t i;

        for (i = 0; i < ((dataCount + 1) / 2); i++)
        {
            dest[i] = ((uint16_t*)sourceBuf)[i];
        }
    }
    else
    {
        dataCount = (dataCount + 1) / 2;
        while (dataCount--)
        {
            *dest = ((uint16_t)(sourceBuf[1]) << 8) | (uint16_t)(sourceBuf[0]);
            dest++;
            sourceBuf += 2;
        }
    }
}

/* Reads USB endpoint data from packet memory */
static void usb_readPMA(uint8_t * destBuf, uint16_t pmaAddress, uint16_t dataCount)
{
    USB_PacketAddressType * source = (USB_PacketAddressType *)USB_PMAADDR + (pmaAddress / 2);

    /* Check if input data is aligned */
    if ((((uint32_t)destBuf) & 1) == 0)
    {
        uint16_t i;
        for (i = 0; i < ((dataCount + 1) / 2); i++)
        {
            ((uint16_t*)destBuf)[i] = source[i];
        }
    }
    else
    {
        dataCount = (dataCount + 1) / 2;
        while (dataCount--)
        {
            *destBuf = *source;
            destBuf++;
            *destBuf = (*source) >> 8;
            destBuf++;
            source++;
        }
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

/* Handle OUT EP transfer */
static void usb_epReceive(USB_HandleType * husb, USB_EndPointHandleType * ep, uint16_t Length)
{
    /* Multi packet transfer */
    if (Length > ep->MaxPacketSize)
    {
        ep->Transfer.length = Length - ep->MaxPacketSize;
        Length = usb_epConvertRxCount(ep->MaxPacketSize);
    }
    else
    {
        ep->Transfer.length = 0;
        Length = usb_epConvertRxCount(Length);
    }

    /* Double buffering */
    if ((ep->DoubleBuffer == ENABLE) && ((USB->EP[ep->RegId].w & USB_EP_DTOG_RX) != 0))
    {
        /* Set endpoint buffer 0 count */
        USB_EP_BDT[ep->RegId].TX_COUNT = Length;
    }
    else
    {
        /*Set RX buffer count */
        USB_EP_BDT[ep->RegId].RX_COUNT = Length;
    }

    USB_EP_SET_STATUS(ep->RegId, RX, VALID);
}

/* Handle IN EP transfer */
static void usb_epTransmit(USB_HandleType * husb, USB_EndPointHandleType * ep, uint16_t Length)
{
    uint16_t pmaAddress = husb->BdtSize + ep->PacketAddress;

    /* Multi packet transfer */
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
        usb_writePMA(ep->Transfer.buffer, pmaAddress, Length);
        USB_EP_BDT[ep->RegId].TX_COUNT = Length;
    }
    else
    {
        /* Write the data to the USB endpoint */
        if ((USB->EP[ep->RegId].w & USB_EP_DTOG_TX) != 0)
        {
            USB_EP_BDT[ep->RegId].RX_COUNT = Length;
            pmaAddress += ep->MaxPacketSize;
        }
        else
        {
            USB_EP_BDT[ep->RegId].TX_COUNT = Length;
        }
        usb_writePMA(ep->Transfer.buffer, pmaAddress, Length);

        /* Switch the transmission buffer by toggling SW_BUF flag */
        USB_TOGGLE(ep->RegId, DTOG_RX);
    }

    USB_EP_SET_STATUS(ep->RegId, TX, VALID);
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
    uint32_t i;

    /* Enable peripheral clock */
    XPD_USB_ClockCtrl(ENABLE);

    /* Init endpoints structures (USB FS has 8 endpoints) */
    for (i = 0; i < USB_ENDPOINT_COUNT; i++)
    {
        /* Control type until ep is activated */
        husb->EP.IN[i].Type            = husb->EP.OUT[i].Type            = USB_EP_TYPE_CONTROL;
        husb->EP.IN[i].MaxPacketSize   = husb->EP.OUT[i].MaxPacketSize   = 0;
        husb->EP.IN[i].Stalled         = husb->EP.OUT[i].Stalled         = FALSE;
        husb->EP.IN[i].RegId           = husb->EP.OUT[i].RegId           = 0;
        husb->EP.IN[i].DoubleBuffer    = husb->EP.OUT[i].DoubleBuffer    = DISABLE;
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

    /* Reserve place for EP0 descriptor and packets */
    husb->BdtSize = sizeof(USB_BufferDescriptor_TypeDef);
    husb->EP.OUT[0].PacketAddress = 0;
    husb->EP.IN [0].PacketAddress = USB_PMA_ALLOCATION(USB_MAX_PACKET_SIZE);
    husb->PmaOffset = USB_PMA_ALLOCATION(2 * USB_MAX_PACKET_SIZE);

    /*Set interrupt mask */
    i = USB_CNTR_CTRM | USB_CNTR_WKUPM | USB_CNTR_SUSPM | USB_CNTR_RESETM;

    if (Config->SOF == ENABLE)
    {
        i |= USB_CNTR_SOFM;
    }

#ifdef USB_LPMCSR_LMPEN
    /* Set Link Power Management feature (L1 sleep mode support) */
    if (Config->LinkPowerMgmt == ENABLE)
    {
        SET_BIT(USB->LPMCSR.w, USB_LPMCSR_LMPEN | USB_LPMCSR_LPMACK);
        i |= USB_CNTR_L1REQM;
    }
    else
    {
        CLEAR_BIT(USB->LPMCSR.w, USB_LPMCSR_LMPEN | USB_LPMCSR_LPMACK);
    }
#endif
    husb->DeviceAddress = 0;
    husb->LowPowerMode  = Config->LowPowerMode;

    /* Initialize dependencies (pins, IRQ lines) */
    XPD_SAFE_CALLBACK(husb->Callbacks.DepInit, husb);

    /* Apply interrupts selection */
    USB->CNTR.w = i;

    return XPD_OK;
}

/**
 * @brief Restores the USB peripheral to its default inactive state
 * @param husb: pointer to the USB handle structure
 * @return ERROR if input is incorrect, OK if success
 */
XPD_ReturnType XPD_USB_Deinit(USB_HandleType * husb)
{
    /* Stop device if not done so far */
    XPD_USB_Stop(husb);

    /* disable all interrupts and force USB reset */
    USB->CNTR.w = USB_CNTR_FRES;

    /* clear interrupt status register */
    USB->ISTR.w = 0;

    /* switch-off device */
    USB_REG_BIT(husb,CNTR,PDWN) = 1;

    /* Deinitialize dependencies */
    XPD_SAFE_CALLBACK(husb->Callbacks.DepDeinit, husb);

    /* Peripheral clock disabled */
    XPD_USB_ClockCtrl(DISABLE);

    return XPD_OK;
}

/**
 * @brief Starts the USB device operation
 * @param husb: pointer to the USB handle structure
 */
void XPD_USB_Start(USB_HandleType * husb)
{
    /* Activate DP line pull up */
#ifdef USB_BCDR_DPPU
    USB_REG_BIT(husb,BCDR, DPPU) = 1;
#else
    XPD_SAFE_CALLBACK(husb->Callbacks.ConnectionStateCtrl, ENABLE);
#endif
    /* Set Link State to connected */
    husb->LinkState = USB_LPM_L0;
}

/**
 * @brief Disconnects the device from the USB host
 * @param husb: pointer to the USB handle structure
 */
void XPD_USB_Stop(USB_HandleType * husb)
{
    /* Deactivate DP line pull up */
#ifdef USB_BCDR_DPPU
    USB_REG_BIT(husb,BCDR, DPPU) = 0;
#else
    XPD_SAFE_CALLBACK(husb->Callbacks.ConnectionStateCtrl, DISABLE);
#endif
    /* Set Link State to disconnected */
    husb->LinkState = USB_LPM_L3;
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
        /* USB Address will be applied in IRQHandler */
        husb->DeviceAddress = Address;
    }
}

/**
 * @brief Configure peripheral RAM allocation for endpoints (except default EP0) after device initialization
 *        and before starting the USB operation.
 * @param husb: pointer to the USB handle structure
 * @param EpAddress: endpoint number
 * @param BufferSize: size of buffer to be allocated in peripheral RAM
 *        This value is limited to FS_MAX_PACKET_SIZE = 64 for Control and Interrupt endpoints
 *        Bulk endpoints can be configured to double-buffered by doubling the configured buffer size
 *        Isochronous endpoints are mandatory double-buffered, therefore it shall be set to between 65 and 128
 */
void XPD_USB_EP_BufferInit(USB_HandleType * husb, uint8_t EpAddress, uint16_t BufferSize)
{
    USB_EndPointHandleType * ep = USB_GET_EP_AT(husb, EpAddress);

    /* EP0 is managed internally */
    if (EpAddress > 0)
    {
        /* Buffer size must align to 16 bit */
        BufferSize += BufferSize & 1;

        /* Double buffering is mandatory for isochronous endpoints,
         * optional for bulk, unavailable for others */
        ep->DoubleBuffer  = (BufferSize > USB_MAX_PACKET_SIZE) ? ENABLE : DISABLE;
        ep->PacketAddress = husb->PmaOffset;

        husb->PmaOffset += USB_PMA_ALLOCATION(BufferSize);

        /* Incrementing of BufferDescriptorTable size is done on a worst case basis
         * If an EPnR can manage both an IN and OUT endpoint, this offset can decrease */
        husb->BdtSize += sizeof(USB_BufferDescriptor_TypeDef);
    }
}

/**
 * @brief Opens an endpoint
 * @param husb: pointer to the USB handle structure
 * @param EpAddress: endpoint number
 * @param Type: endpoint type
 * @param MaxPacketSize: endpoint maximum data packet size
 */
void XPD_USB_EP_Open(USB_HandleType * husb, uint8_t EpAddress, USB_EndPointType Type, uint16_t MaxPacketSize)
{
    USB_EndPointHandleType * ep = USB_GET_EP_AT(husb, EpAddress);
    uint16_t pmaAddress = husb->BdtSize + ep->PacketAddress;

    ep->MaxPacketSize = MaxPacketSize;
    ep->Type          = Type;

    /* Skip EP0, it is mandatory and has been configured at init */
    if (EpAddress > 0)
    {
        USB_EndPointHandleType * pair = (EP_IS_IN(husb,ep) ?
                &husb->EP.OUT[EpAddress] : &husb->EP.IN[EpAddress]);

        /* Ensuring that double buffering is not enabled for control or interrupt EPs */
        if ((ep->Type == USB_EP_CONTROL) || (ep->Type == USB_EP_INTERRUPT))
        {
            ep->DoubleBuffer = DISABLE;
        }

        /* If IN-OUT endpoints with the same address and type are both single buffer,
         * one EPnR can manage both */
        if (  (ep->DoubleBuffer == DISABLE) && (pair->Type == ep->Type) &&
            (pair->DoubleBuffer == DISABLE) && (pair->RegId > 0))
        {
            ep->RegId = pair->RegId;
        }
        else
        {
            uint8_t id;

            /* Find a disabled (free) EP register */
            for (id = 1; id < USB_ENDPOINT_COUNT; id++)
            {
                if ((USB->EP[id].w & (USB_EPTX_STAT | USB_EPRX_STAT)) == 0)
                {
                    /* Save EPnR ID */
                    ep->RegId = id;
                    break;
                }
            }
            /* If the control reaches here, the Endpoint allocation failed!
             * USBD library does not have any handling mechanism for this problem,
             * therefore it is ignored on XPD level as well */
        }
    }

    /* Configure EP */
    USB->EP[ep->RegId].w = (USB->EP[ep->RegId].w & USB_EP_T_MASK)
            | usb_epTypeRemap[ep->Type];

    USB->EP[ep->RegId].w = (USB->EP[ep->RegId].w & USB_EPREG_MASK)
            | USB_EP_CTR_RX | USB_EP_CTR_TX | EpAddress;

    /* Double buffer */
    if (ep->DoubleBuffer == ENABLE)
    {
        /* Set the endpoint as double buffered */
        USB->EP[ep->RegId].w = (USB->EP[ep->RegId].w & USB_EPREG_MASK)
            | USB_EP_CTR_RX | USB_EP_CTR_TX | USB_EP_KIND;

        /* Clear the data toggle bits for the endpoint IN/OUT */
        USB_TOGGLE_CLEAR(ep->RegId, DTOG_RX);
        USB_TOGGLE_CLEAR(ep->RegId, DTOG_TX);

        if (EP_IS_IN(husb,ep))
        {
            /* Set buffer address for double buffered mode */
            USB_EP_BDT[ep->RegId].RX_ADDR  = pmaAddress + ep->MaxPacketSize;
            USB_EP_BDT[ep->RegId].RX_COUNT = 0;

            /* Set SW_BUF flag */
            USB_TOGGLE(ep->RegId, DTOG_RX);

            /* Disable unused direction */
            USB_EP_SET_STATUS(ep->RegId, RX, DIS);
        }
        else
        {
            /* Set buffer address for double buffered mode */
            USB_EP_BDT[ep->RegId].TX_ADDR  = pmaAddress + ep->MaxPacketSize;

            /* Set SW_BUF flag */
            USB_TOGGLE(ep->RegId, DTOG_TX);

            /* Disable unused direction */
            USB_EP_SET_STATUS(ep->RegId, TX, DIS);
        }
    }

    if (EP_IS_IN(husb,ep))
    {
        /*Set the endpoint Transmit buffer address */
        USB_EP_BDT[ep->RegId].TX_ADDR  = pmaAddress;
        USB_EP_BDT[ep->RegId].TX_COUNT = 0;

        USB_TOGGLE_CLEAR(ep->RegId, DTOG_TX);

        /* Configure NAK status for the Endpoint */
        USB_EP_SET_STATUS(ep->RegId, TX, NAK);
    }
    else
    {
        /* Set the endpoint Receive buffer address and counter */
        USB_EP_BDT[ep->RegId].RX_ADDR  = pmaAddress;
        USB_EP_BDT[ep->RegId].RX_COUNT = usb_epConvertRxCount(MaxPacketSize);

        USB_TOGGLE_CLEAR(ep->RegId, DTOG_RX);

        /* Configure VALID status for the Endpoint */
        USB_EP_SET_STATUS(ep->RegId, RX, VALID);
    }
}

/**
 * @brief Closes an active endpoint
 * @param husb: pointer to the USB handle structure
 * @param EpAddress: endpoint number
 */
void XPD_USB_EP_Close(USB_HandleType * husb, uint8_t EpAddress)
{
    USB_EndPointHandleType * ep = USB_GET_EP_AT(husb, EpAddress);

    /* Only previously opened EPs shall be closed */
    if ((EpAddress == 0) || (ep->RegId > 0))
    {
        if (EP_IS_IN(husb,ep))
        {
            /* Configure DISABLE status for the Endpoint*/
            USB_TOGGLE_CLEAR(ep->RegId, DTOG_TX);
            USB_EP_SET_STATUS(ep->RegId, TX, DIS);

            if (ep->DoubleBuffer == ENABLE)
            {
                /* Disable other half of EPnR as well */
                USB_TOGGLE_CLEAR(ep->RegId, DTOG_RX);
                USB_TOGGLE(ep->RegId, DTOG_RX);
                USB_EP_SET_STATUS(ep->RegId, RX, DIS);
            }
        }
        else
        {
            /* Configure DISABLE status for the Endpoint*/
            USB_TOGGLE_CLEAR(ep->RegId, DTOG_RX);
            USB_EP_SET_STATUS(ep->RegId, RX, DIS);

            if (ep->DoubleBuffer == ENABLE)
            {
                /* Disable other half of EPnR as well */
                USB_TOGGLE_CLEAR(ep->RegId, DTOG_TX);
                USB_TOGGLE(ep->RegId, DTOG_TX);
                USB_EP_SET_STATUS(ep->RegId, TX, DIS);
            }
        }

        /* Reset EPnR binding */
        ep->RegId = 0;
    }
}

/**
 * @brief Initiates data reception on the OUT endpoint
 * @param husb: pointer to the USB handle structure
 * @param EpAddress: endpoint number
 * @param Data: pointer to the data buffer
 * @param Length: amount of data bytes to transfer
 */
void XPD_USB_EP_Receive(USB_HandleType * husb, uint8_t EpAddress, uint8_t * Data, uint16_t Length)
{
    USB_EndPointHandleType * ep = &husb->EP.OUT[EpAddress];

    /* setup transfer */
    ep->Transfer.buffer = Data;
    ep->Transfer.size   = 0;

    usb_epReceive(husb, ep, Length);
}

/**
 * @brief  Returns the OUT endpoint's received data size
 * @param  husb: pointer to the USB handle structure
 * @param  EpAddress: endpoint number
 * @return The number of received bytes
 */
uint16_t XPD_USB_EP_GetRxCount(USB_HandleType * husb, uint8_t EpAddress)
{
    return husb->EP.OUT[EpAddress].Transfer.size;
}

/**
 * @brief Initiates data transmission on the IN endpoint
 * @param husb: pointer to the USB handle structure
 * @param EpAddress: endpoint number
 * @param Data: pointer to the data buffer
 * @param Length: amount of data bytes to transfer
 */
void XPD_USB_EP_Transmit(USB_HandleType * husb, uint8_t EpAddress, uint8_t * Data, uint16_t Length)
{
    USB_EndPointHandleType * ep = &husb->EP.IN[EpAddress & 0x7F];

    /* setup and start the transfer */
    ep->Transfer.buffer = Data;
    ep->Transfer.size   = 0;

    usb_epTransmit(husb, ep, Length);
}

/**
 * @brief Set a STALL condition on an endpoint (not supported for Isochronous)
 * @param husb: pointer to the USB handle structure
 * @param EpAddress: endpoint number
 */
void XPD_USB_EP_SetStall(USB_HandleType * husb, uint8_t EpAddress)
{
    USB_EndPointHandleType * ep = USB_GET_EP_AT(husb, EpAddress);

    if (EpAddress == 0)
    {
        /* For control EP0 set both directions */
        USB_EP_SET_STATUS(ep->RegId, RX_TX, STALL);
    }
    else
    {
        if (EP_IS_IN(husb,ep))
        {
            USB_EP_SET_STATUS(ep->RegId, TX, STALL);
        }
        else
        {
            USB_EP_SET_STATUS(ep->RegId, RX, STALL);
        }
    }
    ep->Stalled = TRUE;
}

/**
 * @brief Clear a STALL condition on an endpoint
 * @param husb: pointer to the USB handle structure
 * @param EpAddress: endpoint number
 */
void XPD_USB_EP_ClearStall(USB_HandleType * husb, uint8_t EpAddress)
{
    USB_EndPointHandleType * ep = USB_GET_EP_AT(husb, EpAddress);

    if (EP_IS_IN(husb,ep))
    {
        USB_TOGGLE_CLEAR(ep->RegId, DTOG_TX);
        USB_EP_SET_STATUS(ep->RegId, TX, VALID);
    }
    else
    {
        USB_TOGGLE_CLEAR(ep->RegId, DTOG_RX);
        USB_EP_SET_STATUS(ep->RegId, RX, VALID);
    }
    ep->Stalled = FALSE;
}

/**
 * @brief USB interrupt handler that provides event-driven peripheral management
 *        and handle callbacks.
 * @param husb: pointer to the USB handle structure
 */
void XPD_USB_IRQHandler(USB_HandleType * husb)
{
    uint32_t istr;

    /* loop while Endpoint interrupts are present */
    for (istr = USB->ISTR.w; (istr & USB_ISTR_CTR) != 0; istr = USB->ISTR.w)
    {
        /* Read highest priority endpoint number */
        uint8_t  epId  = (uint8_t)(istr & USB_ISTR_EP_ID);
        uint16_t epReg = USB->EP[epId].w, count;
        USB_EndPointHandleType * ep;

        /* EP0 Control endpoint interrupt - handled in accordance with
         * ST USBD Core library (single packet transfers only) */
        if (epId == 0)
        {
            /* DIR bit indicates IN data */
            if ((istr & USB_ISTR_DIR) == 0)
            {
                ep = &husb->EP.IN[0];

                /* Clear TX complete flag */
                USB_CLEAR_EP_FLAG(0, CTR_TX);

                ep->Transfer.size = USB_EP_BDT[0].TX_COUNT & 0x3FF;
                ep->Transfer.buffer += ep->Transfer.size;

                /* IN packet successfully sent */
                XPD_SAFE_CALLBACK(husb->Callbacks.DataInStage, husb->User, 0,
                        ep->Transfer.buffer);

                /* Set device address if new valid has been received */
                if ((husb->DeviceAddress > 0) && (ep->Transfer.length == 0))
                {
                    USB->DADDR.w = USB_DADDR_EF | husb->DeviceAddress;
                    husb->DeviceAddress = 0;
                }
            }
            else
            {
                /* DIR bit indicates SETUP or OUT data */
                ep = &husb->EP.OUT[0];

                ep->Transfer.size = USB_EP_BDT[0].RX_COUNT & 0x3FF;

                /* Get SETUP Packet */
                if ((epReg & USB_EP_SETUP) != 0)
                {
                    usb_readPMA((uint8_t*) husb->Setup, (husb->BdtSize + ep->PacketAddress),
                            ep->Transfer.size);

                    /* Clear RX complete flag */
                    USB_CLEAR_EP_FLAG(0, CTR_RX);

                    /* Process SETUP Packet */
                    XPD_SAFE_CALLBACK(husb->Callbacks.SetupStage,
                            husb->User, (uint8_t *)husb->Setup);
                }

                /* Get Control Data OUT Packet */
                else if ((epReg & USB_EP_CTR_RX) != 0)
                {
                    /* Clear RX complete flag */
                    USB_CLEAR_EP_FLAG(0, CTR_RX);

                    /* Read only if not Zero Length Packet data is expected */
                    if (ep->Transfer.size > 0)
                    {
                        usb_readPMA(ep->Transfer.buffer, (husb->BdtSize + ep->PacketAddress),
                                ep->Transfer.size);
                        ep->Transfer.buffer += ep->Transfer.size;
                    }

                    /* Process Control Data OUT Packet */
                    XPD_SAFE_CALLBACK(husb->Callbacks.DataOutStage,
                            husb->User, 0, ep->Transfer.buffer);

                    /* Keep EP0 in receiving state */
                    USB_EP_BDT[0].RX_COUNT = usb_epConvertRxCount(ep->MaxPacketSize);
                    USB_EP_SET_STATUS(0, RX, VALID);
                }
            }
        }
        /* EPx Class specific endpoints interrupt processing */
        else
        {
            uint8_t EpAddress = epReg & USB_EPADDR_FIELD;

            /* OUT data received */
            if ((epReg & USB_EP_CTR_RX) != 0)
            {
                uint16_t pmaAddress;
                ep = &husb->EP.OUT[EpAddress];
                pmaAddress = husb->BdtSize + ep->PacketAddress;

                /* Clear RX complete flag */
                USB_CLEAR_EP_FLAG(epId, CTR_RX);

                /* Double buffering */
                if ((ep->DoubleBuffer == ENABLE) && ((epReg & USB_EP_DTOG_RX) != 0))
                {
                    /* read from endpoint 0 buffer */
                    count       = USB_EP_BDT[epId].TX_COUNT & 0x3FF;
                    pmaAddress += ep->MaxPacketSize;
                }
                else
                {
                    /* read from endpoint 1 (Rx) buffer */
                    count       = USB_EP_BDT[epId].RX_COUNT & 0x3FF;
                }

                usb_readPMA(ep->Transfer.buffer, pmaAddress, count);

                /* Switch the reception buffer by toggling SW_BUF flag */
                if (ep->DoubleBuffer == ENABLE)
                {
                    USB_TOGGLE(epId, DTOG_TX);
                }

                ep->Transfer.size   += count;
                ep->Transfer.buffer += count;

                /* If the last packet of the data, transfer is complete
                 * TODO 64 byte single message is not handled */
                if ((ep->Transfer.length == 0) || (count < ep->MaxPacketSize))
                {
                    /* Reception finished */
                    XPD_SAFE_CALLBACK(husb->Callbacks.DataOutStage,
                            husb->User, EpAddress, ep->Transfer.buffer);
                }
                else
                {
                    /* Continue data reception */
                    usb_epReceive(husb, ep, ep->Transfer.length);
                }
            }

            /* IN data sent */
            if ((epReg & USB_EP_CTR_TX) != 0)
            {
                ep = &husb->EP.IN[EpAddress];

                /* Clear TX complete flag */
                USB_CLEAR_EP_FLAG(epId, CTR_TX);

                /* Double buffering */
                if ((ep->DoubleBuffer == ENABLE) && ((epReg & USB_EP_DTOG_TX) == 0))
                {
                    /* written from endpoint 1 buffer */
                    count = USB_EP_BDT[epId].RX_COUNT & 0x3FF;
                }
                else
                {
                    /* written from endpoint 0 (Tx) buffer */
                    count = USB_EP_BDT[epId].TX_COUNT & 0x3FF;
                }

                ep->Transfer.size   += count;
                ep->Transfer.buffer += count;

                /* If the last packet of the data */
                if (ep->Transfer.length == 0)
                {
                    /* Transmission complete */
                    XPD_SAFE_CALLBACK(husb->Callbacks.DataInStage,
                            husb->User, EpAddress, ep->Transfer.buffer);
                }
                else
                {
                    /* Continue data transmission */
                    usb_epTransmit(husb, ep, ep->Transfer.length);
                }
            }
        }
    }

    /* Handle device reset */
    if ((istr & USB_ISTR_RESET) != 0)
    {
        XPD_USB_ClearFlag(husb, RESET);
        XPD_SAFE_CALLBACK(husb->Callbacks.Reset, husb->User);

        /* reset device address, enable addressing */
        USB->DADDR.w = USB_DADDR_EF;
    }

    /* Handle wakeup signal */
    if ((istr & USB_ISTR_WKUP) != 0)
    {
        /* Release low-power mode in the macrocell */
        CLEAR_BIT(USB->CNTR.w, USB_CNTR_FSUSP | USB_CNTR_LPMODE);

        XPD_USB_ClearFlag(husb, WKUP);

        XPD_SAFE_CALLBACK(husb->Callbacks.Resume, husb->User);

        /* LPM state is changed after Resume callback
         * -> possible to determine exited suspend level */
        husb->LinkState = USB_LPM_L0;
    }

#ifdef USB_ISTR_L1REQ
    /* Handle L1 suspend request */
    if ((istr & USB_ISTR_L1REQ) != 0)
    {
        /* Force suspend and low-power mode before going to L1 state */
        SET_BIT(USB->CNTR.w, USB_CNTR_FSUSP | USB_CNTR_LPMODE);

        XPD_USB_ClearFlag(husb, L1REQ);

        /* Set the target Link State */
        husb->LinkState = USB_LPM_L1;
        XPD_SAFE_CALLBACK(husb->Callbacks.Suspend, husb->User);
    }
#endif

    /* Handle suspend request */
    if ((istr & USB_ISTR_SUSP) != 0)
    {
        /* Force low-power mode in the macrocell */
        SET_BIT(USB->CNTR.w, USB_CNTR_FSUSP | USB_CNTR_LPMODE);

        XPD_USB_ClearFlag(husb, SUSP);

        if (XPD_USB_GetFlag(husb, WKUP) == 0)
        {
            /* Set the target Link State */
            husb->LinkState = USB_LPM_L2;
            XPD_SAFE_CALLBACK(husb->Callbacks.Suspend, husb->User);
        }
    }

    /* Handle 1ms periodic Start Of Frame packet */
    if ((istr & USB_ISTR_SOF) != 0)
    {
        XPD_USB_ClearFlag(husb, SOF);
        XPD_SAFE_CALLBACK(husb->Callbacks.SOF, husb->User);
    }
}

/**
 * @brief Activates remote wake-up signaling
 * @param husb: pointer to the USB handle structure
 */
void XPD_USB_ActivateRemoteWakeup(USB_HandleType * husb)
{
#ifdef USB_CNTR_L1RESUME
    if (husb->LinkState == USB_LPM_L1)
    {
        USB_REG_BIT(husb,CNTR,L1RESUME) = 1;
    }
    else
#endif
    {
        USB_REG_BIT(husb,CNTR,RESUME) = 1;
    }
}

/**
 * @brief Deactivates remote wake-up signaling
 * @param husb: pointer to the USB handle structure
 */
void XPD_USB_DeactivateRemoteWakeup(USB_HandleType * husb)
{
#ifdef USB_CNTR_L1RESUME
    CLEAR_BIT(USB->CNTR.w, USB_CNTR_L1RESUME | USB_CNTR_RESUME);
#else
    USB_REG_BIT(husb,CNTR,RESUME) = 0;
#endif
}

#ifdef USB_BCDR_BCDEN
/**
 * @brief Executes Battery Charger Detection algorithm.
 * @note  This function shall be executed before the USB peripheral is connected
 *        (started) with DP pull-up.
 * @param husb: pointer to the USB handle structure
 * @return The determined upstream port type
 */
USB_ChargerType XPD_USB_ChargerDetect(USB_HandleType * husb)
{
    /* USB power is always available before the data lines are contacted */
    USB_ChargerType detection = USB_BCD_NO_DATA_CONTACT;

    /* Enable Battery Charger Detection */
    SET_BIT(USB->BCDR.w, USB_BCDR_BCDEN | USB_BCDR_DCDEN);
    {
        uint32_t timeout = 1000; /* Adjust depending on the speed of plugging in the device */

        /* Data Contact Detect: determines contact of data lines to the bus powering entity */
        if (XPD_OK == XPD_WaitForDiff((void*)&USB->BCDR.w, USB_BCDR_DCDET, 0, &timeout))
        {
            /* Bus electric stabilization */
            XPD_Delay_ms(300);

            /* Primary Detection:
             * distinguishes between a Standard Downstream Port and Charging Ports */
            USB->BCDR.w = (USB->BCDR.w & ~USB_BCDR_DCDEN) | USB_BCDR_PDEN;

            /* Bus electric stabilization */
            XPD_Delay_ms(300);

            /* Check the results of PD */
            switch (USB->BCDR.w & (USB_BCDR_PS2DET | USB_BCDR_PDET))
            {
                case USB_BCDR_PS2DET:
                    /* D- is externally pulled high during Primary Detection
                     * PS2 port or proprietary charger */
                    detection = USB_BCD_PS2_PROPRIETARY_PORT;
                    break;

                case USB_BCDR_PDET:
                    /* Secondary Detection:
                     * distinguishes between Charging Downstream Port and Dedicated Charging Port */
                    USB->BCDR.w = (USB->BCDR.w & ~USB_BCDR_PDEN) | USB_BCDR_SDEN;

                    /* Bus electric stabilization */
                    XPD_Delay_ms(300);

                    if (USB_REG_BIT(husb,BCDR,SDET) != 0)
                    {
                        /* Dedicated Downstream Port DCP */
                        detection = USB_BCD_DEDICATED_CHARGING_PORT;
                    }
                    else
                    {
                        /* Charging Downstream Port CDP */
                        detection = USB_BCD_CHARGING_DOWNSTREAM_PORT;
                    }
                    break;

                default:
                    /* No downstream side support for PD
                     * Standard Downstream Port SDP */
                    detection = USB_BCD_STANDARD_DOWNSTREAM_PORT;
                    break;
            }
        }
    }
    /* Disable Battery Charger Detection */
    CLEAR_BIT(USB->BCDR.w, USB_BCDR_BCDEN | USB_BCDR_DCDEN | USB_BCDR_PDEN | USB_BCDR_SDEN);

    return detection;
}
#endif

/** @} */

/** @} */

#elif defined(USE_XPD_USB) && defined(USB_OTG_FS)

/** @addtogroup USB
 * @{ */

#define USB_EP0_MAX_PACKET_SIZE     64

#define STS_GOUT_NAK                           (1 << USB_OTG_GRXSTSP_PKTSTS_Pos)
#define STS_DATA_UPDT                          (2 << USB_OTG_GRXSTSP_PKTSTS_Pos)
#define STS_XFER_COMP                          (3 << USB_OTG_GRXSTSP_PKTSTS_Pos)
#define STS_SETUP_COMP                         (4 << USB_OTG_GRXSTSP_PKTSTS_Pos)
#define STS_SETUP_UPDT                         (6 << USB_OTG_GRXSTSP_PKTSTS_Pos)

#define EP_IS_IN(HANDLE,ENDPOINT)   (((uint32_t)(ENDPOINT)) < ((uint32_t)(&(HANDLE)->EP.OUT[0])))
#define EP_IS_OUT(HANDLE,ENDPOINT)  (!EP_IS_IN(HANDLE,ENDPOINT))

#define USB_GET_EP_AT(HANDLE, NUMBER)  \
    (((NUMBER) > 0x7F) ? (&(HANDLE)->EP.IN[(NUMBER) &= 0x7F]) : (&(HANDLE)->EP.OUT[NUMBER]))

#ifdef USB_OTG_HS
#define USB_ENDPOINT_COUNT(HANDLE)     \
    ((((uint32_t) husb->Inst) == ((uint32_t)USB_OTG_HS)) ? 6 : 4)
#elif defined(USB_OTG_GCCFG_BCDEN)
#define USB_ENDPOINT_COUNT(HANDLE)     6
#else
#define USB_ENDPOINT_COUNT(HANDLE)     4
#endif

/* Set the status of the DP pull-up resistor */
__STATIC_INLINE void usb_connectionStateCtrl(USB_HandleType * husb, FunctionalState NewState)
{
    USB_REG_BIT(husb,DCTL,SDIS) = ~NewState;
    XPD_Delay_ms(3);
}

/* Set FIFO sizes */
static void usb_setFifoSizes(USB_HandleType * husb)
{
    uint8_t i, epCount_1 = USB_ENDPOINT_COUNT(husb) - 1;
    uint32_t offset = husb->EP.OUT[0].FifoSize;

    /* Global RX FIFO */
    husb->Inst->GRXFSIZ = offset;

    /* EP0 TX FIFO */
    husb->Inst->DIEPTXF0_HNPTXFSIZ.w = ((uint32_t)husb->EP.IN[0].FifoSize << 16) | offset;

    for (i = 0; i < epCount_1; i++)
    {
        /* Increase offset with the previous FIFO size */
        offset += husb->EP.IN[i].FifoSize;

        /* EPx TX FIFOs */
        husb->Inst->DIEPTXF[i].w = (husb->EP.IN[i + 1].FifoSize << 16) | offset;
    }
}

/* Flush an IN FIFO */
__STATIC_INLINE void usb_flushTxFifo(USB_OTG_TypeDef * USBx, uint8_t FifoNumber)
{
    uint32_t timeout = 2;
    USBx->GRSTCTL.w = USB_OTG_GRSTCTL_TXFFLSH | ((uint32_t)FifoNumber << 6);

    XPD_WaitForDiff(&USBx->GRSTCTL.w, USB_OTG_GRSTCTL_TXFFLSH, USB_OTG_GRSTCTL_TXFFLSH, &timeout);
}

/* Flush global OUT FIFO */
__STATIC_INLINE void usb_flushRxFifo(USB_OTG_TypeDef * USBx)
{
    uint32_t timeout = 2;
    USBx->GRSTCTL.w = USB_OTG_GRSTCTL_RXFFLSH;

    XPD_WaitForDiff(&USBx->GRSTCTL.w, USB_OTG_GRSTCTL_RXFFLSH, USB_OTG_GRSTCTL_RXFFLSH, &timeout);
}

/* Push packet data to IN FIFO */
static void usb_writePacket(USB_OTG_TypeDef * USBx, uint8_t FIFOx, uint8_t * Data, uint16_t Length)
{
    int32_t wordCount = (Length + 3) / 4;

    for (; wordCount > 0; wordCount--, Data += 4)
    {
        USBx->DFIFO[FIFOx].DR = *((__packed uint32_t *) Data);
    }
}

/* Pop packet data from OUT FIFO */
static void usb_readPacket(USB_OTG_TypeDef * USBx, uint8_t * Data, uint16_t Length)
{
    uint32_t wordCount = (Length + 3) / 4;

    for (; wordCount > 0; wordCount--, Data += 4)
    {
        *(__packed uint32_t *) Data = USBx->DFIFO[0].DR;

    }
}

/* Set up EP0 to receive control data */
static void usb_EP0_outStart(USB_HandleType * husb)
{
    husb->Inst->OEP[0].DOEPTSIZ.w =
          ( 1      << USB_OTG_DOEPTSIZ_PKTCNT_Pos)
        | ((3 * 8) << USB_OTG_DOEPTSIZ_XFRSIZ_Pos)
        | ( 3      << USB_OTG_DOEPTSIZ_STUPCNT_Pos);

#if (USB_DATA_WORD_ALIGNED == 1) && defined(USB_OTG_GAHBCFG_DMAEN)
    if (husb->DMA == ENABLE)
    {
        husb->Inst->OEP[0].DOEPDMA   = (uint32_t)husb->Setup;
        husb->Inst->OEP[0].DOEPCTL.w = USB_OTG_DOEPCTL_EPENA | USB_OTG_DOEPCTL_USBAEP;
    }
#endif
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
    /* Enable peripheral clock */
#ifdef USB_OTG_HS
    if (((uint32_t)husb->Inst) == ((uint32_t)USB_OTG_HS))
    {
        XPD_OTG_HS_ClockCtrl(ENABLE);

        /* ULPI HS interface */
        if (Config->Speed == USB_SPEED_HIGH)
        {
            XPD_OTG_HS_ULPI_ClockCtrl(ENABLE);
        }

        XPD_OTG_HS_Reset();
    }
    /* OTG_FS does not support Hgh speed ULPI interface */
    else if (Config->Speed == USB_SPEED_HIGH)
    {
        return XPD_ERROR;
    }
    else
#endif
    /* Embedded FS interface */
    {
        XPD_OTG_FS_ClockCtrl(ENABLE);

        XPD_OTG_FS_Reset();

        /* Select FS Embedded PHY */
        USB_REG_BIT(husb,GUSBCFG,PHYSEL) = 1;

        /* Deactivate the power down */
        USB_REG_BIT(husb,GCCFG,PWRDWN) = 1;
    }

    /* Disable the Interrupts */
    USB_REG_BIT(husb,GAHBCFG,GINT) = 0;

#if (USB_DATA_WORD_ALIGNED == 1) && defined(USB_OTG_GAHBCFG_DMAEN)
    /* Set dedicated DMA */
    if (Config->DMA != DISABLE)
    {
        SET_BIT(husb->Inst->GAHBCFG.w, USB_OTG_GAHBCFG_HBSTLEN_2 | USB_OTG_GAHBCFG_DMAEN);
    }
#endif

    /* Set Device Mode */
    MODIFY_REG(husb->Inst->GUSBCFG.w,
            USB_OTG_GUSBCFG_FHMOD | USB_OTG_GUSBCFG_FDMOD,
            USB_OTG_GUSBCFG_FDMOD);

    /* DevInit */
    {
        uint32_t i, epCount = USB_ENDPOINT_COUNT(husb);

        /* Deactivate VBUS Sensing B */
#ifdef USB_OTG_GCCFG_VBDEN
        {
            USB_REG_BIT(husb,GCCFG,VBDEN) = 0;

            /* B-peripheral session valid override enable */
            SET_BIT(husb->Inst->GOTGCTL.w,
                    USB_OTG_GOTGCTL_BVALOEN | USB_OTG_GOTGCTL_BVALOVAL);
        }
#else
        {
            USB_REG_BIT(husb,GCCFG,NOVBUSSENS) = 1;
        }
#endif

        /* Restart the Phy Clock */
        husb->Inst->PCGCCTL.w = 0;

#ifdef USB_OTG_HS
        /* ULPI HS interface */
        if (Config->Speed == USB_SPEED_HIGH)
        {
            /* High speed with ULPI selected */
            husb->Inst->DCFG.b.DSPD = 0;
            husb->Speed = USB_SPEED_HIGH;
        }
        else
#endif
        {
            /* Internal FS Phy */
            husb->Inst->DCFG.b.DSPD = 3;
            husb->Speed = USB_SPEED_FULL;
        }

        /* Flush the FIFOs */
        usb_flushTxFifo(husb->Inst, 0x10);
        usb_flushRxFifo(husb->Inst);

        /* Clear all pending Device Interrupts */
        husb->Inst->DIEPMSK.w  = 0;
        husb->Inst->DOEPMSK.w  = 0;
        husb->Inst->DAINT.w    = ~0;
        husb->Inst->DAINTMSK.w = 0;

        /* Init endpoints structures */
        for (i = 0; i < epCount; i++)
        {
            /* Control type until ep is activated */
            husb->EP.IN[i].Type            = husb->EP.OUT[i].Type            = USB_EP_TYPE_CONTROL;
            husb->EP.IN[i].MaxPacketSize   = husb->EP.OUT[i].MaxPacketSize   = 0;
            husb->EP.IN[i].Stalled         = husb->EP.OUT[i].Stalled         = FALSE;
            husb->EP.IN[i].FifoSize        = husb->EP.OUT[i].FifoSize        = 0;
            husb->EP.IN[i].Transfer.length = husb->EP.OUT[i].Transfer.length = 0;
            husb->EP.IN[i].Transfer.size   = husb->EP.OUT[i].Transfer.size   = 0;

            /* Tx FIFO initialization */
            husb->Inst->DIEPTXF[i].w = 0;

            /* input endpoints */
            if (husb->Inst->IEP[i].DIEPCTL.b.EPENA != 0)
            {
                husb->Inst->IEP[i].DIEPCTL.w = USB_OTG_DIEPCTL_EPDIS | USB_OTG_DIEPCTL_SNAK;
            }
            else
            {
                husb->Inst->IEP[i].DIEPCTL.w = 0;
            }

            husb->Inst->IEP[i].DIEPTSIZ.w = 0;
            husb->Inst->IEP[i].DIEPINT.w  = 0xFF;

            /* output endpoints */
            if (husb->Inst->OEP[i].DOEPCTL.b.EPENA != 0)
            {
                husb->Inst->OEP[i].DOEPCTL.w = (USB_OTG_DOEPCTL_EPDIS | USB_OTG_DOEPCTL_SNAK);
            }
            else
            {
                husb->Inst->OEP[i].DOEPCTL.w = 0;
            }

            husb->Inst->OEP[i].DOEPTSIZ.w = 0;
            husb->Inst->OEP[i].DOEPINT.w  = 0xFF;
        }
        USB_REG_BIT(husb,DIEPMSK,TXFURM) = 0;

#if (USB_DATA_WORD_ALIGNED == 1) && defined(USB_OTG_GAHBCFG_DMAEN)
        if (Config->DMA != DISABLE)
        {
            /*Set threshold parameters */
            husb->Inst->DTHRCTL.w = (USB_OTG_DTHRCTL_TXTHRLEN_6 | USB_OTG_DTHRCTL_RXTHRLEN_6 |
                    USB_OTG_DTHRCTL_RXTHREN |
                    USB_OTG_DTHRCTL_ISOTHREN | USB_OTG_DTHRCTL_NONISOTHREN);
        }
#endif

        /* Clear any pending interrupts */
        husb->Inst->GINTSTS.w = 0xBFFFFFFF;

        /* Enable interrupts matching to the Device mode ONLY */
        i = USB_OTG_GINTMSK_USBSUSPM | USB_OTG_GINTMSK_USBRST |
            USB_OTG_GINTMSK_ENUMDNEM | USB_OTG_GINTMSK_IEPINT |
            USB_OTG_GINTMSK_OEPINT   | USB_OTG_GINTMSK_WUIM;

        /* Enable the common interrupts */
#if (USB_DATA_WORD_ALIGNED == 1) && defined(USB_OTG_GAHBCFG_DMAEN)
        husb->DMA = Config->DMA;

        if (Config->DMA == DISABLE)
#endif
        {
            i |= USB_OTG_GINTMSK_RXFLVLM;
        }
        if (Config->SOF == ENABLE)
        {
            i |= USB_OTG_GINTMSK_SOFM;
        }
#ifdef USB_OTG_GLPMCFG_LPMEN
        /* Set Link Power Management feature (L1 sleep mode support) */
        if (Config->LinkPowerMgmt == ENABLE)
        {
            SET_BIT(husb->Inst->GLPMCFG.w,
                USB_OTG_GLPMCFG_LPMEN | USB_OTG_GLPMCFG_LPMACK | USB_OTG_GLPMCFG_ENBESL);
            i |= USB_OTG_GINTMSK_LPMINTM;
        }
#endif

        husb->DeviceAddress = 0;
        husb->LowPowerMode  = Config->LowPowerMode;
        husb->EP.IN[0].FifoSize  = USB_EP0_MAX_PACKET_SIZE;
        husb->EP.OUT[0].FifoSize = USB_EP0_MAX_PACKET_SIZE;

        /* Initialize dependencies (pins, IRQ lines) */
        XPD_SAFE_CALLBACK(husb->Callbacks.DepInit, husb);

        /* Apply interrupts selection */
        husb->Inst->GINTMSK.w = i;
    }

    return XPD_OK;
}

/**
 * @brief Restores the USB peripheral to its default inactive state
 * @param husb: pointer to the USB handle structure
 * @return ERROR if input is incorrect, OK if success
 */
XPD_ReturnType XPD_USB_Deinit(USB_HandleType * husb)
{
    XPD_USB_Stop(husb);

    /* Deinitialize dependencies */
    XPD_SAFE_CALLBACK(husb->Callbacks.DepDeinit, husb);

    /* Disable peripheral clock */
#ifdef USB_OTG_HS
    if (((uint32_t)husb->Inst) == ((uint32_t)USB_OTG_HS))
    {
        XPD_OTG_HS_ClockCtrl(DISABLE);

        /* ULPI HS interface */
        if (USB_REG_BIT(husb,GUSBCFG,PHYSEL) == 0)
        {
            XPD_OTG_HS_ULPI_ClockCtrl(DISABLE);
        }
    }
    else
#endif
    {
        XPD_OTG_FS_ClockCtrl(DISABLE);
    }

    return XPD_OK;
}

/**
 * @brief Starts the USB device operation
 * @param husb: pointer to the USB handle structure
 */
void XPD_USB_Start(USB_HandleType * husb)
{
    usb_setFifoSizes(husb);

    usb_connectionStateCtrl(husb, ENABLE);

    /* Enable global interrupts */
    USB_REG_BIT(husb,GAHBCFG,GINT) = 1;

    /* Set Link State to connected */
    husb->LinkState = USB_LPM_L0;
}

/**
 * @brief Disconnects the device from the USB host
 * @param husb: pointer to the USB handle structure
 */
void XPD_USB_Stop(USB_HandleType * husb)
{
    uint32_t i;

    /* Disable global interrupts */
    USB_REG_BIT(husb,GAHBCFG,GINT) = 0;

    /* StopDevice */
    /* Clear Pending interrupt */
    for (i = 0; i < 15; i++)
    {
        husb->Inst->IEP[i].DIEPINT.w = 0xFF;
        husb->Inst->OEP[i].DOEPINT.w = 0xFF;
    }
    husb->Inst->DAINT.w = ~0;

    /* Clear interrupt masks */
    husb->Inst->DIEPMSK.w  = 0;
    husb->Inst->DOEPMSK.w  = 0;
    husb->Inst->DAINTMSK.w = 0;

    /* Flush the FIFO */
    usb_flushRxFifo(husb->Inst);
    usb_flushTxFifo(husb->Inst, 0x10);

    usb_connectionStateCtrl(husb, DISABLE);

    /* Set Link State to disconnected */
    husb->LinkState = USB_LPM_L3;
}

/**
 * @brief Sets the USB device address
 * @param husb: pointer to the USB handle structure
 * @param Address: new device address
 */
void XPD_USB_SetAddress(USB_HandleType * husb, uint8_t Address)
{
    husb->Inst->DCFG.b.DAD = Address;
}

/**
 * @brief Configure peripheral FIFO allocation for endpoints after device initialization
 *        and before starting the USB operation.
 * @param husb: pointer to the USB handle structure
 * @param EpAddress: endpoint number
 * @param BufferSize: size of buffer to be allocated in peripheral FIFO
 *        For OUT direction one global FIFO is shared by all endpoints
 */
void XPD_USB_EP_BufferInit(USB_HandleType * husb, uint8_t EpAddress, uint16_t BufferSize)
{
    if (EpAddress < 0x80)
    {
        /* Configure the global Receive FIFO as the largest requested OUT EP size */
        if (husb->EP.OUT[0].FifoSize < BufferSize)
        {
            husb->EP.OUT[0].FifoSize = BufferSize;
        }
    }
    else
    {
        /* Set EPx Tx FIFO size (for use in Start) */
        husb->EP.IN[EpAddress & 0x7F].FifoSize = BufferSize;
    }
}

/**
 * @brief Opens an endpoint
 * @param husb: pointer to the USB handle structure
 * @param EpAddress: endpoint number
 * @param Type: endpoint type
 * @param MaxPacketSize: endpoint maximum data packet size
 */
void XPD_USB_EP_Open(USB_HandleType * husb, uint8_t EpAddress, USB_EndPointType Type, uint16_t MaxPacketSize)
{
    USB_EndPointHandleType * ep = USB_GET_EP_AT(husb, EpAddress);

    ep->MaxPacketSize = MaxPacketSize;
    ep->Type = Type;

    /* Activate Endpoint */
    if (EP_IS_IN(husb,ep))
    {
        SET_BIT(husb->Inst->DAINTMSK.w, 1 << EpAddress);

        /* Check if currently inactive */
        if (husb->Inst->IEP[EpAddress].DIEPCTL.b.USBAEP == 0)
        {
            husb->Inst->IEP[EpAddress].DIEPCTL.b.MPSIZ  = ep->MaxPacketSize;
            husb->Inst->IEP[EpAddress].DIEPCTL.b.EPTYP  = ep->Type;
            husb->Inst->IEP[EpAddress].DIEPCTL.b.TXFNUM = EpAddress;
            husb->Inst->IEP[EpAddress].DIEPCTL.b.SD0PID_SEVNFRM = 1;
            husb->Inst->IEP[EpAddress].DIEPCTL.b.USBAEP = 1;
        }
    }
    else
    {
        SET_BIT(husb->Inst->DAINTMSK.w, 1 << (EpAddress + 16));

        /* Check if currently inactive */
        if (husb->Inst->OEP[EpAddress].DOEPCTL.b.USBAEP == 0)
        {
            husb->Inst->OEP[EpAddress].DOEPCTL.b.MPSIZ  = ep->MaxPacketSize;
            husb->Inst->OEP[EpAddress].DOEPCTL.b.EPTYP  = ep->Type;
            husb->Inst->OEP[EpAddress].DOEPCTL.b.SD0PID_SEVNFRM = 1;
            husb->Inst->OEP[EpAddress].DOEPCTL.b.USBAEP = 1;
        }
    }
}

/**
 * @brief Closes an active endpoint
 * @param husb: pointer to the USB handle structure
 * @param EpAddress: endpoint number
 */
void XPD_USB_EP_Close(USB_HandleType * husb, uint8_t EpAddress)
{
    USB_EndPointHandleType * ep = USB_GET_EP_AT(husb, EpAddress);
    uint32_t timeout = 2;

    /* Deactivate Endpoint */
    if (EP_IS_IN(husb,ep))
    {
        husb->Inst->IEP[EpAddress].DIEPCTL.b.USBAEP = 0;

        /* sets the NAK bit for the IN endpoint */
        husb->Inst->IEP[EpAddress].DIEPCTL.w = USB_OTG_DIEPCTL_SNAK;

        /* Disable IN endpoint */
        husb->Inst->IEP[EpAddress].DIEPCTL.w = USB_OTG_DIEPCTL_EPDIS;

        XPD_WaitForMatch(&husb->Inst->IEP[EpAddress].DIEPINT.w,
                USB_OTG_DIEPINT_EPDISD, USB_OTG_DIEPINT_EPDISD, &timeout);

        /* Flush any data remaining in the TxFIFO */
        usb_flushTxFifo(husb->Inst, 0x10);

        /* Disable endpoint interrupts */
        CLEAR_BIT(husb->Inst->DAINTMSK.w, 1 << EpAddress);
    }
    else
    {
        husb->Inst->OEP[EpAddress].DOEPCTL.b.USBAEP = 0;

        /* sets the NAK bit for the OUT endpoint */
        husb->Inst->OEP[EpAddress].DOEPCTL.w = USB_OTG_DOEPCTL_SNAK;

        /* Disable IN endpoint */
        husb->Inst->OEP[EpAddress].DOEPCTL.w = USB_OTG_DOEPCTL_EPDIS;

        XPD_WaitForMatch(&husb->Inst->OEP[EpAddress].DOEPINT.w,
                USB_OTG_DOEPINT_EPDISD, USB_OTG_DOEPINT_EPDISD, &timeout);

        /* Set the "Clear the Global OUT NAK bit" to disable global OUT NAK mode */
        USB_REG_BIT(husb,DCTL,CGONAK) = 1;

        /* Disable endpoint interrupts */
        CLEAR_BIT(husb->Inst->DAINTMSK.w, 1 << (EpAddress + 16));
    }
}

/**
 * @brief Clears the current data content of an active endpoint
 * @param husb: pointer to the USB handle structure
 * @param EpAddress: endpoint number
 */
void XPD_USB_EP_Flush(USB_HandleType * husb, uint8_t EpAddress)
{
    if ((EpAddress & 0x80) != 0)
    {
        usb_flushTxFifo(husb->Inst, EpAddress & 0x7F);
    }
    else
    {
        usb_flushRxFifo(husb->Inst);
    }
}

/**
 * @brief Initiates data reception on the OUT endpoint
 * @param husb: pointer to the USB handle structure
 * @param EpAddress: endpoint number
 * @param Data: pointer to the data buffer
 * @param Length: amount of data bytes to transfer
 */
void XPD_USB_EP_Receive(USB_HandleType * husb, uint8_t EpAddress, uint8_t * Data, uint16_t Length)
{
    USB_EndPointHandleType * ep = &husb->EP.OUT[EpAddress];

    /* setup transfer */
    ep->Transfer.buffer = Data;
    ep->Transfer.size   = 0;
    ep->Transfer.length = Length;

    /* EP0 is limited to single packet transfers */
    if (EpAddress == 0)
    {
        if (Length > ep->MaxPacketSize)
        {
            ep->Transfer.length = ep->MaxPacketSize;
        }
        Length = 0;
    }

    /* Zero Length Packet */
    if (Length == 0)
    {
        husb->Inst->OEP[EpAddress].DOEPTSIZ.b.PKTCNT = 1;
        husb->Inst->OEP[EpAddress].DOEPTSIZ.b.XFRSIZ = ep->MaxPacketSize;
    }
    else
    {
        /* Program the transfer size and packet count as follows:
         * packet count  = N
         * transfer size = N * maxpacket
         */
        uint16_t pktcnt = (Length + ep->MaxPacketSize - 1) / ep->MaxPacketSize;
        husb->Inst->OEP[EpAddress].DOEPTSIZ.b.PKTCNT = pktcnt;
        husb->Inst->OEP[EpAddress].DOEPTSIZ.b.XFRSIZ = ep->MaxPacketSize * pktcnt;
    }

#if (USB_DATA_WORD_ALIGNED == 1) && defined(USB_OTG_GAHBCFG_DMAEN)
    if (husb->DMA == ENABLE)
    {
        husb->Inst->OEP[EpAddress].DOEPDMA = (uint32_t)Data;
    }
#endif

    /* Set DATA PID parity */
    if (ep->Type == USB_EP_TYPE_ISOCHRONOUS)
    {
        if ((husb->Inst->DSTS.w & (1 << USB_OTG_DSTS_FNSOF_Pos)) == 0)
        {
            husb->Inst->OEP[EpAddress].DOEPCTL.b.SODDFRM = 1;
        }
        else
        {
            husb->Inst->OEP[EpAddress].DOEPCTL.b.SD0PID_SEVNFRM = 1;
        }
    }

    /* EP transfer request */
    SET_BIT(husb->Inst->OEP[EpAddress].DOEPCTL.w,
            USB_OTG_DOEPCTL_CNAK | USB_OTG_DOEPCTL_EPENA);
}

/**
 * @brief  Returns the OUT endpoint's received data size
 * @param  husb: pointer to the USB handle structure
 * @param  EpAddress: endpoint number
 * @return The number of received bytes
 */
uint16_t XPD_USB_EP_GetRxCount(USB_HandleType * husb, uint8_t EpAddress)
{
    return husb->EP.OUT[EpAddress].Transfer.size;
}

/**
 * @brief Initiates data transmission on the IN endpoint
 * @param husb: pointer to the USB handle structure
 * @param EpAddress: endpoint number
 * @param Data: pointer to the data buffer
 * @param Length: amount of data bytes to transfer
 */
void XPD_USB_EP_Transmit(USB_HandleType * husb, uint8_t EpAddress, uint8_t * Data, uint16_t Length)
{
    USB_EndPointHandleType * ep = &husb->EP.IN[EpAddress &= 0x7F];

    /* setup and start the transfer */
    ep->Transfer.buffer = Data;
    ep->Transfer.size   = 0;
    ep->Transfer.length = Length;

    /* EP0 is limited to single packet transfers */
    if (EpAddress == 0)
    {
        if (Length > ep->MaxPacketSize)
        {
            ep->Transfer.length = ep->MaxPacketSize;
        }

        husb->Inst->IEP[EpAddress].DIEPTSIZ.w = ep->Transfer.length
                | (1 << USB_OTG_DIEPTSIZ_PKTCNT_Pos);
    }
    /* Zero Length Packet */
    else if (Length == 0)
    {
        husb->Inst->IEP[EpAddress].DIEPTSIZ.w = 1 << USB_OTG_DIEPTSIZ_PKTCNT_Pos;
    }
    else
    {
        /* Program the transfer size and packet count
         * as follows: transfersize =
         * N * maxpacket + short_packet packet count =
         * N + (short_packet exist ? 1 : 0)
         */
        uint16_t pktcnt = (Length + ep->MaxPacketSize - 1) / ep->MaxPacketSize;
        husb->Inst->IEP[EpAddress].DIEPTSIZ.b.PKTCNT = pktcnt;
        husb->Inst->IEP[EpAddress].DIEPTSIZ.b.XFRSIZ = Length;

        if (ep->Type == USB_EP_TYPE_ISOCHRONOUS)
        {
            husb->Inst->IEP[EpAddress].DIEPTSIZ.b.MULCNT = 1;
        }
    }

    /* Set DATA PID parity */
    if (ep->Type == USB_EP_TYPE_ISOCHRONOUS)
    {
        if ((husb->Inst->DSTS.w & (1 << USB_OTG_DSTS_FNSOF_Pos)) == 0)
        {
            husb->Inst->IEP[EpAddress].DIEPCTL.b.SODDFRM = 1;
        }
        else
        {
            husb->Inst->IEP[EpAddress].DIEPCTL.b.SD0PID_SEVNFRM = 1;
        }
    }

#if (USB_DATA_WORD_ALIGNED == 1) && defined(USB_OTG_GAHBCFG_DMAEN)
    if (husb->DMA == ENABLE)
    {
        husb->Inst->IEP[EpAddress].DIEPDMA = (uint32_t)Data;

        /* EP enable, IN data in FIFO */
        SET_BIT(husb->Inst->IEP[EpAddress].DIEPCTL.w, USB_OTG_DIEPCTL_CNAK | USB_OTG_DIEPCTL_EPENA);
    }
    else
#endif
    {
        if (ep->Type == USB_EP_TYPE_ISOCHRONOUS)
        {
            /* EP enable, IN data in FIFO */
            SET_BIT(husb->Inst->IEP[EpAddress].DIEPCTL.w,
                    USB_OTG_DIEPCTL_CNAK | USB_OTG_DIEPCTL_EPENA);

            usb_writePacket(husb->Inst, EpAddress, Data, Length);
        }
        else
        {
            /* Enable the Tx FIFO Empty Interrupt for this EP */
            if (Length > 0)
            {
                SET_BIT(husb->Inst->DIEPEMPMSK, 1 << EpAddress);
            }

            /* EP enable */
            SET_BIT(husb->Inst->IEP[EpAddress].DIEPCTL.w,
                    USB_OTG_DIEPCTL_CNAK | USB_OTG_DIEPCTL_EPENA);
        }
    }
}

/**
 * @brief Set a STALL condition on an endpoint
 * @param husb: pointer to the USB handle structure
 * @param EpAddress: endpoint number
 */
void XPD_USB_EP_SetStall(USB_HandleType * husb, uint8_t EpAddress)
{
    USB_EndPointHandleType * ep = USB_GET_EP_AT(husb, EpAddress);

    if (EP_IS_IN(husb,ep))
    {
        if (husb->Inst->IEP[EpAddress].DIEPCTL.b.EPENA == 0)
        {
            husb->Inst->IEP[EpAddress].DIEPCTL.b.EPDIS = 0;
        }
        husb->Inst->IEP[EpAddress].DIEPCTL.b.STALL = 1;
    }
    else
    {
        if (husb->Inst->OEP[EpAddress].DOEPCTL.b.EPENA == 0)
        {
            husb->Inst->OEP[EpAddress].DOEPCTL.b.EPDIS = 0;
        }
        husb->Inst->OEP[EpAddress].DOEPCTL.b.STALL = 1;
    }

    if (EpAddress == 0)
    {
        usb_EP0_outStart(husb);
    }
    ep->Stalled = TRUE;
}

/**
 * @brief Clear a STALL condition on an endpoint
 * @param husb: pointer to the USB handle structure
 * @param EpAddress: endpoint number
 */
void XPD_USB_EP_ClearStall(USB_HandleType * husb, uint8_t EpAddress)
{
    USB_EndPointHandleType * ep = USB_GET_EP_AT(husb, EpAddress);

    if (EP_IS_IN(husb,ep))
    {
        husb->Inst->IEP[EpAddress].DIEPCTL.b.STALL = 0;
        if ((ep->Type == USB_EP_TYPE_INTERRUPT) || (ep->Type == USB_EP_TYPE_BULK))
        {
            husb->Inst->IEP[EpAddress].DIEPCTL.b.SD0PID_SEVNFRM = 1;
        }
    }
    else
    {
        husb->Inst->OEP[EpAddress].DOEPCTL.b.STALL = 0;
        if ((ep->Type == USB_EP_TYPE_INTERRUPT) || (ep->Type == USB_EP_TYPE_BULK))
        {
            husb->Inst->OEP[EpAddress].DOEPCTL.b.SD0PID_SEVNFRM = 1;
        }
    }
    ep->Stalled = FALSE;
}

/**
 * @brief USB interrupt handler that provides event-driven peripheral management and handle callbacks.
 * @param husb: pointer to the USB handle structure
 */
void XPD_USB_IRQHandler(USB_HandleType * husb)
{
    uint32_t gints = husb->Inst->GINTSTS.w & husb->Inst->GINTMSK.w;

    if (gints != 0)
    {
        /* Incorrect mode, acknowledge the interrupt */
        if ((gints & USB_OTG_GINTSTS_MMIS) != 0)
        {
            XPD_USB_ClearFlag(husb, MMIS);
        }

        /* OUT endpoint interrupts */
        if ((gints & USB_OTG_GINTSTS_OEPINT) != 0)
        {
            /* Read in the device interrupt bits */
            uint32_t oepint = (husb->Inst->DAINT.w & husb->Inst->DAINTMSK.w) >> 16;
            uint8_t EpAddress;

            /* Handle individual endpoint interrupts */
            for (EpAddress = 0; oepint != 0; EpAddress++, oepint >>= 1)
            {
                if ((oepint & 1) != 0)
                {
                    uint32_t epint = husb->Inst->OEP[EpAddress].DOEPINT.w & husb->Inst->DOEPMSK.w;

                    /* Transfer completed */
                    if ((epint & USB_OTG_DOEPINT_XFRC) != 0)
                    {
                        /* Clear IT flag */
                        husb->Inst->OEP[EpAddress].DOEPINT.w = USB_OTG_DOEPINT_XFRC;

#if (USB_DATA_WORD_ALIGNED == 1) && defined(USB_OTG_GAHBCFG_DMAEN)
                        if (husb->DMA == ENABLE)
                        {
                            husb->EP.OUT[EpAddress].Transfer.size = husb->EP.OUT[EpAddress].MaxPacketSize
                                    - husb->Inst->OEP[EpAddress]->DOEPTSIZ.b.XFRSIZ;
                            husb->EP.OUT[EpAddress].Transfer.buffer += husb->EP.OUT[EpAddress].MaxPacketSize;
                        }
#endif

                        /* Data packet received callback */
                        XPD_SAFE_CALLBACK(husb->Callbacks.DataOutStage, husb->User, EpAddress, husb->EP.OUT[EpAddress].Transfer.buffer);

#if (USB_DATA_WORD_ALIGNED == 1) && defined(USB_OTG_GAHBCFG_DMAEN)
                        if (husb->DMA == ENABLE)
                        {
                            if ((EpAddress == 0) && (husb->EP.OUT[EpAddress].Transfer.length == 0))
                            {
                                /* this is ZLP, so prepare EP0 for next setup */
                                usb_EP0_outStart(husb);
                            }
                        }
#endif
                    }

                    /* Setup request arrived */
                    if ((epint & USB_OTG_DOEPINT_STUP) != 0)
                    {
                        /* Clear IT flag */
                        husb->Inst->OEP[EpAddress].DOEPINT.w = USB_OTG_DOEPINT_STUP;

                        /* Setup packet received callback */
                        XPD_SAFE_CALLBACK(husb->Callbacks.SetupStage, husb->User, (uint8_t *)husb->Setup);
                    }

                    /* Clear irrelevant flags */
                    husb->Inst->OEP[EpAddress].DOEPINT.w =
#ifdef USB_OTG_DOEPINT_OTEPSPR
                    USB_OTG_DOEPINT_OTEPSPR |
#endif
                    USB_OTG_DOEPINT_OTEPDIS;
                }
            }
        }

        /* IN endpoint interrupts */
        if ((gints & USB_OTG_GINTSTS_IEPINT) != 0)
        {
            /* Read in the device interrupt bits */
            uint32_t iepint = (husb->Inst->DAINT.w & husb->Inst->DAINTMSK.w) & 0xFFFF;
            uint8_t EpAddress;

            /* Handle individual endpoint interrupts */
            for (EpAddress = 0; iepint != 0; EpAddress++, iepint >>= 1)
            {
                if ((iepint & 1) != 0)
                {
                    uint32_t epint = husb->Inst->IEP[EpAddress].DIEPINT.w &
                            (husb->Inst->DIEPMSK.w | (((husb->Inst->DIEPEMPMSK >> EpAddress) & 0x1) << 7));

                    /* Transfer completed */
                    if ((epint & USB_OTG_DIEPINT_XFRC) != 0)
                    {
                        CLEAR_BIT(husb->Inst->DIEPEMPMSK, 0x1 << EpAddress);

                        /* Clear IT flag */
                        husb->Inst->IEP[EpAddress].DIEPINT.w = USB_OTG_DIEPINT_XFRC;

#if (USB_DATA_WORD_ALIGNED == 1) && defined(USB_OTG_GAHBCFG_DMAEN)
                        if (husb->DMA == ENABLE)
                        {
                            husb->EP.IN[EpAddress].Transfer.buffer += husb->EP.IN[EpAddress].MaxPacketSize;
                        }
#endif

                        XPD_SAFE_CALLBACK(husb->Callbacks.DataInStage, husb->User, EpAddress, husb->EP.IN[EpAddress].Transfer.buffer);

#if (USB_DATA_WORD_ALIGNED == 1) && defined(USB_OTG_GAHBCFG_DMAEN)
                        if (husb->DMA == ENABLE)
                        {
                            /* this is ZLP, so prepare EP0 for next setup */
                            if ((EpAddress == 0) && (husb->EP.IN[EpAddress].Transfer.length == 0))
                            {
                                usb_EP0_outStart(husb);
                            }
                        }
#endif
                    }

                    /* Fill empty Tx FIFO with available data */
                    if ((epint & USB_OTG_DIEPINT_TXFE) != 0)
                    {
                        USB_EndPointHandleType * ep = &husb->EP.IN[EpAddress];
                        uint32_t count = ep->Transfer.length - ep->Transfer.size;

                        if (count > ep->MaxPacketSize)
                        {
                            count = ep->MaxPacketSize;
                        }

                        while ((ep->Transfer.size < ep->Transfer.length) &&
                               (husb->Inst->IEP[EpAddress].DTXFSTS > ((count + 3) / 4)))
                        {
                            count = ep->Transfer.length - ep->Transfer.size;

                            if (count > ep->MaxPacketSize)
                            {
                                count = ep->MaxPacketSize;
                            }

                            /* Write another packet to FIFO */
                            usb_writePacket(husb->Inst, EpAddress, ep->Transfer.buffer, count);

                            ep->Transfer.buffer += count;
                            ep->Transfer.size   += count;
                        }

                        if (count == 0)
                        {
                            /* Clear empty EP flag */
                            CLEAR_BIT(husb->Inst->DIEPEMPMSK, 0x1 << EpAddress);
                        }
                    }

                    /* Clear irrelevant flags */
                    husb->Inst->IEP[EpAddress].DIEPINT.w =
                            USB_OTG_DIEPINT_TOC    | USB_OTG_DIEPINT_ITTXFE
                          | USB_OTG_DIEPINT_INEPNE | USB_OTG_DIEPINT_EPDISD;
                }
            }
        }

        /* Handle RxQLevel Interrupt */
        if ((gints & USB_OTG_GINTSTS_RXFLVL) != 0)
        {
            uint32_t temp = husb->Inst->GRXSTSP.w,
            EpAddress =  temp & USB_OTG_GRXSTSP_EPNUM,
            count     = (temp & USB_OTG_GRXSTSP_BCNT_Msk) >> USB_OTG_GRXSTSP_BCNT_Pos;
            USB_EndPointHandleType * ep = &husb->EP.OUT[EpAddress];

            /* Disable interrupt for the duration of data transfer to buffer */
            CLEAR_BIT(husb->Inst->GINTMSK.w, USB_OTG_GINTSTS_RXFLVL);

            switch (temp & USB_OTG_GRXSTSP_PKTSTS_Msk)
            {
                case STS_DATA_UPDT:
                    /* Data packet received */
                    usb_readPacket(husb->Inst, ep->Transfer.buffer, count);
                    ep->Transfer.buffer += count;
                    ep->Transfer.size   += count;
                    break;

                case STS_SETUP_UPDT:
                    /* Setup packet received */
                    usb_readPacket(husb->Inst, (uint8_t *) husb->Setup, count);
                    ep->Transfer.size += count;
                    break;

                default:
                    break;
            }

            SET_BIT(husb->Inst->GINTMSK.w, USB_OTG_GINTSTS_RXFLVL);
        }

        /* Handle Reset Interrupt */
        if ((gints & USB_OTG_GINTSTS_USBRST) != 0)
        {
            uint8_t i, epCount = USB_ENDPOINT_COUNT(husb);

            USB_REG_BIT(husb,DCTL,RWUSIG) = 0;
            usb_flushTxFifo(husb->Inst, 0);

            /* Clear EP interrupt flags */
            for (i = 0; i < epCount; i++)
            {
                husb->Inst->IEP[i].DIEPINT.w = 0xFF;
                husb->Inst->OEP[i].DOEPINT.w = 0xFF;
            }

            /* Clear device flags, enable EP0 interrupts */
            husb->Inst->DAINT.w     = ~0;
            husb->Inst->DAINTMSK.w |= 0x10001;

            {
                SET_BIT(husb->Inst->DOEPMSK.w, USB_OTG_DOEPMSK_STUPM
#ifdef USB_OTG_DOEPINT_OTEPSPR
                        | USB_OTG_DOEPMSK_OTEPSPRM
#endif
                        | USB_OTG_DOEPMSK_XFRCM | USB_OTG_DOEPMSK_EPDM);
                SET_BIT(husb->Inst->DIEPMSK.w, USB_OTG_DIEPMSK_TOM
                        | USB_OTG_DIEPMSK_XFRCM | USB_OTG_DIEPMSK_EPDM);
            }

            /* Set Default Address to 0 */
            husb->Inst->DCFG.b.DAD = 0;

            /* setup EP0 to receive SETUP packets */
            usb_EP0_outStart(husb);

            XPD_USB_ClearFlag(husb, USBRST);
        }

        /* Handle Enumeration done Interrupt */
        if ((gints & USB_OTG_GINTSTS_ENUMDNE) != 0)
        {
            /* ActivateSetup */
            /* Set the MPS of the IN EP based on the enumeration speed */
            husb->Inst->IEP[0].DIEPCTL.b.MPSIZ = 0;
            USB_REG_BIT(husb,DCTL,CGINAK)  = 1;
            USB_REG_BIT(husb,GUSBCFG,TRDT) = 0;

#ifdef USB_OTG_HS
            /* High speed enumberated */
            if (husb->Inst->DSTS.b.ENUMSPD == 0)
            {
                husb->Speed = USB_SPEED_HIGH;
                husb->Inst->GUSBCFG.b.TRDT = 9;
            }
            else
#endif
            {
                uint32_t trdt;
                husb->Speed = USB_SPEED_FULL;

                /* Get most suitable value depending on AHB frequency */
                trdt = 224000000 / XPD_RCC_GetClockFreq(HCLK);
                if (trdt < 6)
                {
                    trdt = 6;
                }
                husb->Inst->GUSBCFG.b.TRDT = trdt;
            }

            XPD_USB_ClearFlag(husb, ENUMDNE);

            XPD_SAFE_CALLBACK(husb->Callbacks.Reset, husb->User);
        }

        /* Handle Resume Interrupt */
        if ((gints & USB_OTG_GINTSTS_WKUINT) != 0)
        {
            /* Clear the Remote Wake-up Signaling */
            USB_REG_BIT(husb,DCTL,RWUSIG) = 0;

            XPD_USB_ClearFlag(husb, WKUINT);

            XPD_SAFE_CALLBACK(husb->Callbacks.Resume, husb->User);

            /* LPM state is changed after Resume callback
             * -> possible to determine exited suspend level */
            husb->LinkState = USB_LPM_L0;
        }

#ifdef USB_OTG_GLPMCFG_LPMEN
        /* Handle L1 suspend request */
        if ((gints & USB_OTG_GINTSTS_LPMINT) != 0)
        {
            XPD_USB_ClearFlag(husb, LPMINT);

            /* Set the target Link State */
            husb->LinkState = USB_LPM_L1;
            XPD_SAFE_CALLBACK(husb->Callbacks.Suspend, husb->User);
        }
#endif

        /* Handle Suspend Interrupt */
        if ((gints & USB_OTG_GINTSTS_USBSUSP) != 0)
        {
            XPD_USB_ClearFlag(husb, USBSUSP);

            if (USB_REG_BIT(husb,DSTS,SUSPSTS) != 0)
            {
                /* Set the target Link State */
                husb->LinkState = USB_LPM_L2;
                XPD_SAFE_CALLBACK(husb->Callbacks.Suspend, husb->User);
            }
        }

        /* Handle Connection event Interrupt */
        if ((gints & USB_OTG_GINTSTS_SRQINT) != 0)
        {
            XPD_USB_ClearFlag(husb, SRQINT);
            XPD_SAFE_CALLBACK(husb->Callbacks.Connected, husb->User);
        }

        /* Handle Disconnection event Interrupt */
        if ((gints & USB_OTG_GINTSTS_OTGINT) != 0)
        {
            if ((husb->Inst->GOTGINT.w & USB_OTG_GOTGINT_SEDET) != 0)
            {
                XPD_SAFE_CALLBACK(husb->Callbacks.Disconnected, husb->User);
            }

            /* Clear all flags */
            husb->Inst->GOTGINT.w = ~0;
        }

        /* Handle SOF Interrupt */
        if ((gints & USB_OTG_GINTSTS_SOF) != 0)
        {
            XPD_USB_ClearFlag(husb, SOF);

            XPD_SAFE_CALLBACK(husb->Callbacks.SOF, husb->User);
        }
    }
}

/**
 * @brief Sets the USB PHY clock status.
 * @param husb: pointer to the USB handle structure
 * @param NewState: Enable or disable PHY clock
 */
void XPD_USB_PHY_ClockCtrl(USB_HandleType * husb, FunctionalState NewState)
{
    USB_REG_BIT(husb, PCGCCTL, STOPCLK) = ~NewState;
}

/**
 * @brief Activates remote wake-up signaling
 * @param husb: pointer to the USB handle structure
 */
void XPD_USB_ActivateRemoteWakeup(USB_HandleType * husb)
{
    if (USB_REG_BIT(husb,DSTS,SUSPSTS) != 0)
    {
        /* Activate Remote wakeup signaling */
        USB_REG_BIT(husb,DCTL,RWUSIG) = 1;
    }
}

/**
 * @brief Deactivates remote wake-up signaling
 * @param husb: pointer to the USB handle structure
 */
void XPD_USB_DeActivateRemoteWakeup(USB_HandleType * husb)
{
    /* Deactivate Remote wakeup signaling */
    USB_REG_BIT(husb,DCTL,RWUSIG) = 0;
}

#ifdef USB_OTG_GCCFG_BCDEN
/**
 * @brief Executes Battery Charger Detection algorithm.
 * @note  This function shall be executed before the USB peripheral is started.
 * @param husb: pointer to the USB handle structure
 * @return The determined upstream port type
 */
USB_ChargerType XPD_USB_ChargerDetect(USB_HandleType * husb)
{
    /* USB power is always available before the data lines are contacted */
    USB_ChargerType detection = USB_BCD_NO_DATA_CONTACT;

    /* Enable Battery Charger Detection */
    USB_REG_BIT(husb,GCCFG,BCDEN) = 1;

    /* Check if device is connected */
    if (USB_REG_BIT(husb,DCTL,SDIS) != 0)
    {
        uint32_t timeout = 1000; /* Adjust depending on the speed of plugging in the device */

        USB_REG_BIT(husb,GCCFG,DCDEN) = 1;

        /* Data Contact Detect: determines contact of data lines to the bus powering entity */
        if (XPD_OK == XPD_WaitForDiff((void*)&husb->Inst->GCCFG.w, USB_OTG_GCCFG_DCDET, 0, &timeout))
        {
            /* Bus electric stabilization */
            XPD_Delay_ms(100);

            /* Primary Detection:
             * distinguishes between a Standard Downstream Port and Charging Ports */
            husb->Inst->GCCFG.w = (husb->Inst->GCCFG.w & ~USB_OTG_GCCFG_DCDEN) | USB_OTG_GCCFG_PDEN;

            /* Bus electric stabilization */
            XPD_Delay_ms(100);

            /* Check the results of PD */
            switch (husb->Inst->GCCFG.w & (USB_OTG_GCCFG_PS2DET | USB_OTG_GCCFG_PDET))
            {
                case USB_OTG_GCCFG_PS2DET:
                    /* D- is externally pulled high during Primary Detection
                     * PS2 port or proprietary charger */
                    detection = USB_BCD_PS2_PROPRIETARY_PORT;
                    break;

                case USB_OTG_GCCFG_PDET:
                    /* Secondary Detection:
                     * distinguishes between Charging Downstream Port and Dedicated Charging Port */
                    husb->Inst->GCCFG.w = (husb->Inst->GCCFG.w & ~USB_OTG_GCCFG_PDEN) | USB_OTG_GCCFG_SDEN;

                    /* Bus electric stabilization */
                    XPD_Delay_ms(100);

                    if (USB_REG_BIT(husb,GCCFG,SDET) != 0)
                    {
                        /* Dedicated Downstream Port DCP */
                        detection = USB_BCD_DEDICATED_CHARGING_PORT;
                    }
                    else
                    {
                        /* Charging Downstream Port CDP */
                        detection = USB_BCD_CHARGING_DOWNSTREAM_PORT;
                    }
                    break;

                default:
                    /* No downstream side support for PD
                     * Standard Downstream Port SDP */
                    detection = USB_BCD_STANDARD_DOWNSTREAM_PORT;
                    break;
            }
        }
    }
    /* Disable Battery Charger Detection */
    CLEAR_BIT(husb->Inst->GCCFG.w, USB_OTG_GCCFG_BCDEN | USB_OTG_GCCFG_DCDEN | USB_OTG_GCCFG_PDEN | USB_OTG_GCCFG_SDEN);

    return detection;
}
#endif

/** @} */

/** @} */

#endif /* defined(USB_OTG_FS) && defined(USE_XPD_USB) */
