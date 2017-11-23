/**
  ******************************************************************************
 * @file    xpd_usb.c
 * @author  Benedek Kupper
 * @version V0.2
 * @date    2017-05-04
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

#if defined(USE_XPD_USB) && defined(USB)

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
    XPD_RCC_ClockEnable(RCC_POS_USB);

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
    XPD_RCC_ClockDisable(RCC_POS_USB);

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
                XPD_SAFE_CALLBACK(husb->Callbacks.DataInStage, husb->User, 0x80,
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
                __IO USB_PacketAddressType* pCountReg;
                ep = &husb->EP.IN[EpAddress];

                /* Clear TX complete flag */
                USB_CLEAR_EP_FLAG(epId, CTR_TX);

                /* Double buffering */
                if ((ep->DoubleBuffer == ENABLE) && ((epReg & USB_EP_DTOG_TX) == 0))
                {
                    /* written from endpoint 1 buffer */
                    pCountReg = &USB_EP_BDT[epId].RX_COUNT;
                }
                else
                {
                    /* written from endpoint 0 (Tx) buffer */
                    pCountReg = &USB_EP_BDT[epId].TX_COUNT;
                }
                count = (*pCountReg) & 0x3FF;

                /* Clear register to avoid initial repeated data sending */
                (*pCountReg) = 0;

                ep->Transfer.size   += count;
                ep->Transfer.buffer += count;

                /* If the last packet of the data */
                if (ep->Transfer.length == 0)
                {
                    /* Transmission complete */
                    XPD_SAFE_CALLBACK(husb->Callbacks.DataInStage,
                            husb->User, 0x80 | EpAddress, ep->Transfer.buffer);
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

#endif /* USE_XPD_USB */
