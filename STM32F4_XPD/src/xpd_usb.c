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

#if defined(USB_OTG_FS) && defined(USE_XPD_USB)

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
        XPD_RCC_ClockEnable(RCC_POS_OTG_HS);

        /* ULPI HS interface */
        if (Config->Speed == USB_SPEED_HIGH)
        {
            XPD_RCC_ClockEnable(RCC_POS_OTG_HS_ULPI);
        }

        XPD_RCC_Reset(RCC_POS_OTG_HS);
    }
    /* OTG_FS does not support High speed ULPI interface */
    else if (Config->Speed == USB_SPEED_HIGH)
    {
        return XPD_ERROR;
    }
    else
#endif
    /* Embedded FS interface */
    {
        XPD_RCC_ClockEnable(RCC_POS_OTG_FS);

        XPD_RCC_Reset(RCC_POS_OTG_FS);

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
        XPD_RCC_ClockDisable(RCC_POS_OTG_HS);

        /* ULPI HS interface */
        if (USB_REG_BIT(husb,GUSBCFG,PHYSEL) == 0)
        {
            XPD_RCC_ClockDisable(RCC_POS_OTG_HS_ULPI);
        }
    }
    else
#endif
    {
        XPD_RCC_ClockDisable(RCC_POS_OTG_FS);
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
        usb_flushTxFifo(husb->Inst, EpAddress);

        /* Disable endpoint interrupts */
        CLEAR_BIT(husb->Inst->DAINTMSK.w, 1 << EpAddress);
    }
    else
    {
        husb->Inst->OEP[EpAddress].DOEPCTL.b.USBAEP = 0;

        /* sets the NAK bit for the OUT endpoint */
        husb->Inst->OEP[EpAddress].DOEPCTL.w = USB_OTG_DOEPCTL_SNAK;

        /* Disable OUT endpoint */
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

                        XPD_SAFE_CALLBACK(husb->Callbacks.DataInStage, husb->User,
                                0x80 | EpAddress, husb->EP.IN[EpAddress].Transfer.buffer);

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
