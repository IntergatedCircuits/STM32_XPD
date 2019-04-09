/**
  ******************************************************************************
  * @file    xpd_usb_otg.c
  * @author  Benedek Kupper
  * @version 0.2
  * @date    2018-01-28
  * @brief   STM32 eXtensible Peripheral Drivers USB OTG Module
  *
  * Copyright (c) 2018 Benedek Kupper
  *
  * Licensed under the Apache License, Version 2.0 (the "License");
  * you may not use this file except in compliance with the License.
  * You may obtain a copy of the License at
  *
  *     http://www.apache.org/licenses/LICENSE-2.0
  *
  * Unless required by applicable law or agreed to in writing, software
  * distributed under the License is distributed on an "AS IS" BASIS,
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  */
#include <xpd_usb_otg.h>
#include <xpd_rcc.h>
#include <xpd_utils.h>

#if defined(USB_OTG_FS)

/** @addtogroup USB
 * @{ */

typedef struct {
    union {
        struct {
            __IO uint32_t MPSIZ : 11;                       /*!< Maximum packet size              */
                 uint32_t __RESERVED0 : 4;
            __IO uint32_t USBAEP : 1;                       /*!< USB active endpoint              */
            __IO uint32_t EONUM_DPID : 1;                   /*!< Even/odd frame                   */
            __IO uint32_t NAKSTS : 1;                       /*!< NAK status                       */
            __IO uint32_t EPTYP : 2;                        /*!< Endpoint type                    */
                 uint32_t __RESERVED1 : 1;
            __IO uint32_t STALL : 1;                        /*!< STALL handshake                  */
            __IO uint32_t TXFNUM : 4;                       /*!< TxFIFO number                    */
            __IO uint32_t CNAK : 1;                         /*!< Clear NAK                        */
            __IO uint32_t SNAK : 1;                         /*!< Set NAK */
            __IO uint32_t SD0PID_SEVNFRM : 1;               /*!< Set DATA0 PID                    */
            __IO uint32_t SODDFRM : 1;                      /*!< Set odd frame                    */
            __IO uint32_t EPDIS : 1;                        /*!< Endpoint disable                 */
            __IO uint32_t EPENA : 1;                        /*!< Endpoint enable                  */
        } b;
        __IO uint32_t w;
    } DxEPCTL;                                              /*!< dev Endpoint Control Reg    */
         uint32_t __RESERVED0;
    union {
        struct {
            __IO uint32_t XFRC : 1;                         /*!< Transfer completed interrupt */
            __IO uint32_t EPDISD : 1;                       /*!< Endpoint disabled interrupt */
                 uint32_t __RESERVED0 : 1;
            __IO uint32_t STUP : 1;                         /*!< SETUP phase done */
            __IO uint32_t ITTXFE : 1;                       /*!< IN token received when TxFIFO is empty */
                 uint32_t __RESERVED1 : 1;
            __IO uint32_t INEPNE : 1;                       /*!< IN endpoint NAK effective */
            __IO uint32_t TXFE : 1;                         /*!< Transmit FIFO empty */
            __IO uint32_t TXFIFOUDRN : 1;                   /*!< Transmit Fifo Underrun */
            __IO uint32_t BNA : 1;                          /*!< Buffer not available interrupt */
                 uint32_t __RESERVED2 : 1;
            __IO uint32_t PKTDRPSTS : 1;                    /*!< Packet dropped status */
            __IO uint32_t BERR : 1;                         /*!< Babble error interrupt */
            __IO uint32_t NAK : 1;                          /*!< NAK interrupt */
            __IO uint32_t NYET : 1;                         /*!< NYET interrupt */
                 uint32_t __RESERVED3 : 17;
        } b;
        __IO uint32_t w;
    } DxEPINT;                                              /*!< dev Endpoint Itr Reg       */
         uint32_t __RESERVED1;
    union {
        struct {
            __IO uint32_t XFRSIZ : 19;                      /*!< Transfer size */
            __IO uint32_t PKTCNT : 10;                      /*!< Packet count */
            __IO uint32_t MULCNT : 2;                       /*!< Packet count */
                 uint32_t __RESERVED0 : 1;
        } b;
        __IO uint32_t w;
    } DxEPTSIZ;                                             /*!< Endpoint Txfer Size        */
    __IO uint32_t DxEPDMA;                                  /*!< Endpoint DMA Address Reg   */
    __IO uint32_t DTXFSTS;                                  /*!< Endpoint Tx FIFO Status Reg*/
         uint32_t __RESERVED2;
} USB_OTG_GenEndpointType;

#define USB_OTG_DMA_SUPPORT         (defined(USB_OTG_HS) && 1)

#if (USB_OTG_DMA_SUPPORT != 0)
#define USB_DMA_CONFIG(HANDLE)      USB_REG_BIT((HANDLE),GAHBCFG,DMAEN)
#else
#define USB_DMA_CONFIG(HANDLE)      0
#endif

#define STS_GOUT_NAK                (1 << USB_OTG_GRXSTSP_PKTSTS_Pos)
#define STS_DATA_UPDT               (2 << USB_OTG_GRXSTSP_PKTSTS_Pos)
#define STS_XFER_COMP               (3 << USB_OTG_GRXSTSP_PKTSTS_Pos)
#define STS_SETUP_COMP              (4 << USB_OTG_GRXSTSP_PKTSTS_Pos)
#define STS_SETUP_UPDT              (6 << USB_OTG_GRXSTSP_PKTSTS_Pos)

#define USB_ALL_TX_FIFOS            0x10

#define USB_GET_EP_AT(HANDLE, NUMBER)   (((NUMBER) > 0x7F) ?            \
        (&(HANDLE)->EP.IN[(NUMBER) & 0xF]) :                            \
        (&(HANDLE)->EP.OUT[NUMBER]))

#define USB_IEPR(HANDLE, NUMBER)                                        \
        ((USB_OTG_GenEndpointType*)(&(HANDLE)->Inst->IEP[(NUMBER)&0xF]))

#define USB_OEPR(HANDLE, NUMBER)                                        \
        ((USB_OTG_GenEndpointType*)(&(HANDLE)->Inst->OEP[(NUMBER)]))

#define USB_EPR(HANDLE, NUMBER)         (((NUMBER) > 0x7F) ?            \
        USB_IEPR(HANDLE, NUMBER) : USB_OEPR(HANDLE, NUMBER))


#ifdef USB_OTG_HS

#define IS_USB_OTG_HS(INST)     ((uint32_t)(INST) == USB_OTG_HS_PERIPH_BASE)
#define USB_ENDPOINT_COUNT(HANDLE)  (IS_USB_OTG_HS((HANDLE)->Inst) ?        \
        USB_OTG_HS_MAX_IN_ENDPOINTS : USB_OTG_FS_MAX_IN_ENDPOINTS)

#define USB_TOTAL_FIFO_SIZE(HANDLE) (IS_USB_OTG_HS((HANDLE)->Inst) ?        \
        USB_OTG_HS_TOTAL_FIFO_SIZE : USB_OTG_FS_TOTAL_FIFO_SIZE)
#else
#define IS_USB_OTG_HS(INST)     0
#define USB_ENDPOINT_COUNT(HANDLE)  6

#define USB_TOTAL_FIFO_SIZE(HANDLE) 1280
#endif

/* Set the status of the DP pull-up resistor */
__STATIC_INLINE void USB_prvConnectCtrl(USB_HandleType * pxUSB, FunctionalState NewState)
{
    USB_REG_BIT(pxUSB,DCTL,SDIS) = ~NewState;
}

/* Flush an IN FIFO */
__STATIC_INLINE void USB_prvFlushTxFifo(USB_HandleType * pxUSB, uint8_t FifoNumber)
{
    pxUSB->Inst->GRSTCTL.w = USB_OTG_GRSTCTL_TXFFLSH |
            ((uint32_t)FifoNumber << USB_OTG_GRSTCTL_TXFNUM_Pos);
}

/* Flush global OUT FIFO */
__STATIC_INLINE void USB_prvFlushRxFifo(USB_HandleType * pxUSB)
{
    pxUSB->Inst->GRSTCTL.w = USB_OTG_GRSTCTL_RXFFLSH;
}

/* Clears all endpoint interrupt request flags */
static void USB_prvClearEpInts(USB_HandleType * pxUSB)
{
    uint8_t ucEpNum;
    uint8_t ucEpCount = USB_ENDPOINT_COUNT(pxUSB);

    for (ucEpNum = 0; ucEpNum < ucEpCount; ucEpNum++)
    {
        pxUSB->Inst->IEP[ucEpNum].DIEPINT.w = 0xFF;
        pxUSB->Inst->OEP[ucEpNum].DOEPINT.w = 0xFF;
    }
}

/* Push packet data to IN FIFO */
static void USB_prvWriteFifo(USB_HandleType * pxUSB,
        uint8_t ucFIFOx, uint8_t * pucData, uint16_t usLength)
{
    uint16_t usWordCount;

    /* Disable interrupts while FIFO is being accessed */
    USB_REG_BIT(pxUSB, GAHBCFG, GINT) = 0;

    for (usWordCount = (usLength + 3) / 4; usWordCount > 0; usWordCount--, pucData += 4)
    {
        pxUSB->Inst->DFIFO[ucFIFOx].DR = *((__packed uint32_t *) pucData);
    }

    USB_REG_BIT(pxUSB, GAHBCFG, GINT) = 1;
}

/* Pop packet data from OUT FIFO */
static void USB_prvReadFifo(USB_HandleType * pxUSB,
        uint8_t * pucData, uint16_t usLength)
{
    uint16_t usWordCount;

    /* Disable interrupts while FIFO is being accessed */
    USB_REG_BIT(pxUSB, GAHBCFG, GINT) = 0;

    for (usWordCount = (usLength + 3) / 4; usWordCount > 0; usWordCount--, pucData += 4)
    {
        *(__packed uint32_t *) pucData = pxUSB->Inst->DFIFO[0].DR;
    }

    USB_REG_BIT(pxUSB, GAHBCFG, GINT) = 1;
}

/* Handle IN EP transfer */
static void USB_prvTransmitPacket(USB_HandleType * pxUSB, uint8_t ucEpNum)
{
    USB_EndPointHandleType * pxEP = &pxUSB->EP.IN[ucEpNum];
    uint32_t ulFifoSpace = pxUSB->Inst->IEP[ucEpNum].DTXFSTS * sizeof(uint32_t);
    uint32_t ulEpFlag = 1 << ucEpNum;

    /* If there is enough space in the FIFO for a packet, fill immediately */
    if (ulFifoSpace >= (uint32_t)pxEP->MaxPacketSize)
    {
        uint16_t usPacketLength;

        /* Multi packet transfer */
        if (pxEP->Transfer.Progress > pxEP->MaxPacketSize)
        {
            usPacketLength = pxEP->MaxPacketSize;
        }
        else
        {
            usPacketLength = pxEP->Transfer.Progress;
        }

        /* Write a packet to the FIFO */
        USB_prvWriteFifo(pxUSB, ucEpNum, pxEP->Transfer.Data, usPacketLength);
        pxEP->Transfer.Data += usPacketLength;
        pxEP->Transfer.Progress -= usPacketLength;
    }

    if (ucEpNum == 0)
    {
        /* Interrupt isn't used */
    }
    else if (pxEP->Transfer.Progress == 0)
    {
        /* Disable Tx FIFO interrupts when all data is written */
        CLEAR_BIT(pxUSB->Inst->DIEPEMPMSK, ulEpFlag);
    }
    else
    {
        /* Enable Tx FIFO interrupts when more data is available */
        SET_BIT(pxUSB->Inst->DIEPEMPMSK, ulEpFlag);
    }
}

/* Internal handling of EP transmission */
static void USB_prvEpSend(USB_HandleType * pxUSB, uint8_t ucEpNum)
{
    USB_EndPointHandleType * pxEP = &pxUSB->EP.IN[ucEpNum];
    USB_OTG_GenEndpointType * pxDEP = USB_IEPR(pxUSB, ucEpNum);
    uint16_t usTransferSize = pxEP->Transfer.Progress;

    if (pxEP->Transfer.Progress == 0)
    {
        /* 1 transfer with 0 length */
        pxDEP->DxEPTSIZ.w = 1 << USB_OTG_DIEPTSIZ_PKTCNT_Pos;
    }
    /* EP0 has limited transfer size */
    else if ((ucEpNum == 0) && (pxEP->Transfer.Progress > pxEP->MaxPacketSize))
    {
        pxDEP->DxEPTSIZ.b.PKTCNT = 1;
        pxDEP->DxEPTSIZ.b.XFRSIZ = usTransferSize = pxEP->MaxPacketSize;
    }
    else
    {
        uint16_t usPktCnt = (pxEP->Transfer.Progress + pxEP->MaxPacketSize - 1)
                / pxEP->MaxPacketSize;
        pxDEP->DxEPTSIZ.b.PKTCNT = usPktCnt;
        pxDEP->DxEPTSIZ.b.XFRSIZ = pxEP->Transfer.Progress;

        if (pxEP->Type == USB_EP_TYPE_ISOCHRONOUS)
        {
            pxDEP->DxEPTSIZ.b.MULCNT = 1;

            /* If LSB of SOF frame number is one */
            if ((pxUSB->Inst->DSTS.w & (1 << USB_OTG_DSTS_FNSOF_Pos)) == 0)
            {
                /* Set ODD frame */
                pxDEP->DxEPCTL.b.SODDFRM = 1;
            }
            else
            {
                /* Set DATA0 PID */
                pxDEP->DxEPCTL.b.SD0PID_SEVNFRM = 1;
            }
        }
    }

#if (USB_OTG_DMA_SUPPORT != 0)
    if (USB_DMA_CONFIG(pxUSB) != 0)
    {
        /* Set DMA start address */
        pxDEP->DxEPDMA = (uint32_t)pxEP->Transfer.Data;
        pxEP->Transfer.Data += usTransferSize;
        pxEP->Transfer.Progress -= usTransferSize;
    }
#endif
    /* EP enable */
    SET_BIT(pxDEP->DxEPCTL.w, USB_OTG_DIEPCTL_CNAK | USB_OTG_DIEPCTL_EPENA);

    if ((pxEP->Transfer.Progress > 0) &&
        (USB_DMA_CONFIG(pxUSB) == 0))
    {
        /* Push the nonzero packet to FIFO */
        USB_prvTransmitPacket(pxUSB, ucEpNum);
    }
}

/* Internal handling of EP reception */
static void USB_prvEpReceive(USB_HandleType * pxUSB, uint8_t ucEpNum)
{
    USB_EndPointHandleType * pxEP = &pxUSB->EP.OUT[ucEpNum];
    USB_OTG_GenEndpointType * pxDEP = USB_OEPR(pxUSB, ucEpNum);

    /* Zero Length Packet or EP0 with limited transfer size */
    if ((pxEP->Transfer.Progress == 0) || (ucEpNum == 0))
    {
        pxDEP->DxEPTSIZ.b.PKTCNT = 1;
        pxDEP->DxEPTSIZ.b.XFRSIZ = pxEP->MaxPacketSize;
    }
    else
    {
        uint16_t usPktCnt = (pxEP->Transfer.Progress + pxEP->MaxPacketSize - 1)
                / pxEP->MaxPacketSize;
        pxDEP->DxEPTSIZ.b.PKTCNT = usPktCnt;
        pxDEP->DxEPTSIZ.b.XFRSIZ = pxEP->Transfer.Progress;
    }

#if (USB_OTG_DMA_SUPPORT != 0)
    if (USB_DMA_CONFIG(pxUSB) != 0)
    {
        /* Set DMA start address */
        pxDEP->DxEPDMA = (uint32_t)pxEP->Transfer.Data;
    }
#endif

    /* Set DATA PID parity */
    if (pxEP->Type == USB_EP_TYPE_ISOCHRONOUS)
    {
        /* If LSB of SOF frame number is one */
        if ((pxUSB->Inst->DSTS.w & (1 << USB_OTG_DSTS_FNSOF_Pos)) == 0)
        {
            /* Set ODD frame */
            pxDEP->DxEPCTL.b.SODDFRM = 1;
        }
        else
        {
            /* Set DATA0 PID */
            pxDEP->DxEPCTL.b.SD0PID_SEVNFRM = 1;
        }
    }

    /* EP transfer request */
    SET_BIT(pxDEP->DxEPCTL.w, USB_OTG_DOEPCTL_CNAK | USB_OTG_DOEPCTL_EPENA);
}

/* Set up EP0 to receive control data */
static void USB_prvPrepareSetup(USB_HandleType * pxUSB)
{
    /* Setup: 1 transfer with 8 byte data */
    pxUSB->Inst->OEP[0].DOEPTSIZ.w =
          ( 1      << USB_OTG_DOEPTSIZ_PKTCNT_Pos)
        | ((3 * 8) << USB_OTG_DOEPTSIZ_XFRSIZ_Pos)
        | ( 3      << USB_OTG_DOEPTSIZ_STUPCNT_Pos);

#if (USB_OTG_DMA_SUPPORT != 0)
    if (USB_DMA_CONFIG(pxUSB) != 0)
    {
        pxUSB->Inst->OEP[0].DOEPDMA   = (uint32_t)&pxUSB->Setup;
        pxUSB->Inst->OEP[0].DOEPCTL.w = USB_OTG_DOEPCTL_EPENA | USB_OTG_DOEPCTL_USBAEP;
    }
#endif
}

/* Handle events of a given IN endpoint */
static void USB_prvInEpEventHandler(USB_HandleType * pxUSB, uint8_t ucEpNum)
{
    USB_OTG_GenEndpointType * pxDEP = USB_IEPR(pxUSB, ucEpNum);

    /* TXFE enable bit is extracted from common register */
    uint32_t ulEpTXFE = ((pxUSB->Inst->DIEPEMPMSK >> ucEpNum)
            << USB_OTG_DIEPINT_TXFE_Pos) & USB_OTG_DIEPINT_TXFE;

    uint32_t ulEpFlags = pxDEP->DxEPINT.w &
            (pxUSB->Inst->DIEPMSK.w | ulEpTXFE);

    /* Clear irrelevant flags */
    pxDEP->DxEPINT.w = USB_OTG_DIEPINT_TOC | USB_OTG_DIEPINT_ITTXFE
            | USB_OTG_DIEPINT_INEPNE | USB_OTG_DIEPINT_EPDISD;

    /* Fill empty Tx FIFO with available data */
    if ((ulEpFlags & USB_OTG_DIEPINT_TXFE) != 0)
    {
        USB_prvTransmitPacket(pxUSB, ucEpNum);
    }
    /* Transfer completed */
    if ((ulEpFlags & USB_OTG_DIEPINT_XFRC) != 0)
    {
        USB_EndPointHandleType * pxEP = &pxUSB->EP.IN[ucEpNum];

        /* Clear IT flag */
        pxDEP->DxEPINT.w = USB_OTG_DIEPINT_XFRC;

        if (ucEpNum > 0)
        {
            /* Transmission complete */
            USB_vDataInCallback(pxUSB, pxEP);
        }
        else /* EP0 packetization requires software handling */
        {
            if (pxEP->Transfer.Progress == 0)
            {
                /* Transmission complete */
                USB_vDataInCallback(pxUSB, pxEP);

                if ((USB_DMA_CONFIG(pxUSB) != 0) &&
                    (pxEP->Transfer.Length == 0))
                {
                    /* this is ZLP, so prepare EP0 for next setup */
                    USB_prvPrepareSetup(pxUSB);
                }
            }
            else
            {
                /* Transfer next packet */
                USB_prvEpSend(pxUSB, 0);
            }
        }
    }
}

/* Handle events of a given OUT endpoint */
static void USB_prvOutEpEventHandler(USB_HandleType * pxUSB, uint8_t ucEpNum)
{
    USB_OTG_GenEndpointType * pxDEP = USB_OEPR(pxUSB, ucEpNum);
    uint32_t ulEpFlags = pxDEP->DxEPINT.w & pxUSB->Inst->DOEPMSK.w;

    /* Clear irrelevant flags */
    pxDEP->DxEPINT.w =
#ifdef USB_OTG_DOEPINT_OTEPSPR
            USB_OTG_DOEPINT_OTEPSPR |
#endif
            USB_OTG_DOEPINT_OTEPDIS;

    /* Setup stage complete */
    if ((ulEpFlags & USB_OTG_DOEPINT_STUP) != 0)
    {
        /* Clear IT flag */
        pxDEP->DxEPINT.w = USB_OTG_DOEPINT_STUP;

        /* Process SETUP Packet */
        USB_vSetupCallback(pxUSB);
    }
    /* Transfer completed */
    else if ((ulEpFlags & USB_OTG_DOEPINT_XFRC) != 0)
    {
        USB_EndPointHandleType * pxEP = &pxUSB->EP.OUT[ucEpNum];

        /* Clear IT flag */
        pxDEP->DxEPINT.w = USB_OTG_DOEPINT_XFRC;

        if (USB_DMA_CONFIG(pxUSB) != 0)
        {
            /* XFRSIZ holds the unfilled byte count
             * after the transfer is complete;
             * EP0 transfers are limited to MPS */
            uint16_t usTransferSize =
                    pxEP->MaxPacketSize - pxDEP->DxEPTSIZ.b.XFRSIZ;
            pxEP->Transfer.Length += usTransferSize;
            pxEP->Transfer.Data += usTransferSize;

            if ((ucEpNum + pxEP->Transfer.Length) == 0)
            {
                /* this is ZLP, so prepare EP0 for next setup */
                USB_prvPrepareSetup(pxUSB);
            }
        }

        if ((ucEpNum > 0) || (pxEP->Transfer.Progress == pxEP->Transfer.Length))
        {
            /* Reception finished */
            USB_vDataOutCallback(pxUSB, pxEP);
        }
        else
        {
            /* EP0 packetization requires software handling */
            USB_prvEpReceive(pxUSB, 0);
        }
    }
}

/* Opens EP0 bidirectional dedicated control endpoint. */
static void USB_prvCtrlEpOpen(USB_HandleType * pxUSB)
{
    /* Activate Endpoint 0 interrupts */
    SET_BIT(pxUSB->Inst->DAINTMSK.w,
            (1 << (0 + USB_OTG_DAINTMSK_IEPM_Pos)) |
            (1 << (0 + USB_OTG_DAINTMSK_OEPM_Pos)));

    /* Check if currently inactive */
    if (pxUSB->Inst->IEP[0].DIEPCTL.b.USBAEP == 0)
    {
        pxUSB->Inst->IEP[0].DIEPCTL.b.MPSIZ  = pxUSB->EP.IN[0].MaxPacketSize;
        pxUSB->Inst->IEP[0].DIEPCTL.b.EPTYP  = USB_EP_TYPE_CONTROL;
        pxUSB->Inst->IEP[0].DIEPCTL.b.TXFNUM = 0;
        pxUSB->Inst->IEP[0].DIEPCTL.b.SD0PID_SEVNFRM = 1;
        pxUSB->Inst->IEP[0].DIEPCTL.b.USBAEP = 1;
    }
    /* Check if currently inactive */
    if (pxUSB->Inst->OEP[0].DOEPCTL.b.USBAEP == 0)
    {
        pxUSB->Inst->OEP[0].DOEPCTL.b.MPSIZ  = pxUSB->EP.OUT[0].MaxPacketSize;
        pxUSB->Inst->OEP[0].DOEPCTL.b.EPTYP  = USB_EP_TYPE_CONTROL;
        pxUSB->Inst->OEP[0].DOEPCTL.b.SD0PID_SEVNFRM = 1;
        pxUSB->Inst->OEP[0].DOEPCTL.b.USBAEP = 1;
    }

    /* prepare receive SETUP packet */
    USB_prvPrepareSetup(pxUSB);
}

#if defined(USB_HS_PHYC) && defined(HSE_VALUE_Hz)
#if !defined(USB_HS_PHYC_TUNE_VALUE)
#define USB_HS_PHYC_TUNE_VALUE    0x00000F13
#endif
static void USB_prvPhycInit(void)
{
    RCC_vClockEnable(RCC_POS_USBPHYC);

    /* Enable LDO */
    PHYC_REG_BIT(LDO,DISABLE) = 0;

    if (XPD_OK == XPD_eWaitForMatch(&USB_HS_PHYC->LDO.w,
            USB_HS_PHYC_LDO_STATUS, USB_HS_PHYC_LDO_STATUS, 2))
    {
        /* Control the tuning interface of the High Speed PHY */
        USB_HS_PHYC->TUNE.w |= USB_HS_PHYC_TUNE_VALUE;

        switch (RCC_ulOscFreq_Hz(HSE))
        {
            case 12000000:
                USB_HS_PHYC->PLL.w = (0 << USB_HS_PHYC_PLL_PLLSEL_Pos)
                    | USB_HS_PHYC_PLL_PLLEN;
                break;
            case 12500000:
                USB_HS_PHYC->PLL.w = (2 << USB_HS_PHYC_PLL_PLLSEL_Pos)
                    | USB_HS_PHYC_PLL_PLLEN;
                break;
            case 16000000:
                USB_HS_PHYC->PLL.w = (3 << USB_HS_PHYC_PLL_PLLSEL_Pos)
                    | USB_HS_PHYC_PLL_PLLEN;
                break;
            case 24000000:
                USB_HS_PHYC->PLL.w = (4 << USB_HS_PHYC_PLL_PLLSEL_Pos)
                    | USB_HS_PHYC_PLL_PLLEN;
                break;
            case 25000000:
                USB_HS_PHYC->PLL.w = (5 << USB_HS_PHYC_PLL_PLLSEL_Pos)
                    | USB_HS_PHYC_PLL_PLLEN;
                break;
            case 32000000:
                USB_HS_PHYC->PLL.w = (7 << USB_HS_PHYC_PLL_PLLSEL_Pos)
                    | USB_HS_PHYC_PLL_PLLEN;
                break;
            default:
                break;
        }

        XPD_vDelay_ms(2);
    }
}
#endif

/* Resets the USB OTG core */
static void USB_prvReset(USB_HandleType * pxUSB)
{
    if (USB_REG_BIT(pxUSB,GRSTCTL,AHBIDL) != 0)
    {
        USB_REG_BIT(pxUSB,GRSTCTL,CSRST) = 1;
    }
}

/* Initializes the selected PHY for the USB */
static void USB_prvPhyInit(USB_HandleType * pxUSB, USB_PHYType ePHY)
{
#ifdef USB_OTG_HS
    if (IS_USB_OTG_HS(pxUSB->Inst) && (ePHY != USB_PHY_EMBEDDED_FS))
    {
#if defined(USB_HS_PHYC) && defined(HSE_VALUE_Hz)
        if (ePHY == USB_PHY_EMBEDDED_HS)
        {
            /* Embedded UTMI HS PHY */
            USB_REG_BIT(pxUSB, GCCFG, PWRDWN) = 0;

            CLEAR_BIT(pxUSB->Inst->GUSBCFG.w,
                USB_OTG_GUSBCFG_TSDPS  | USB_OTG_GUSBCFG_ULPIFSLS |
                USB_OTG_GUSBCFG_PHYSEL | USB_OTG_GUSBCFG_ULPI_UTMI_SEL |
                USB_OTG_GUSBCFG_ULPIEVBUSD | USB_OTG_GUSBCFG_ULPIEVBUSI);

            /* Select UTMI Interface */
            USB_REG_BIT(pxUSB, GCCFG, PHYHSEN) = 1;

            USB_prvPhycInit();
        }
        else
#endif
        {
            /* ULPI HS PHY */
            RCC_vClockEnable(RCC_POS_OTG_HS_ULPI);

            USB_REG_BIT(pxUSB, GCCFG, PWRDWN) = 0;

            CLEAR_BIT(pxUSB->Inst->GUSBCFG.w,
                USB_OTG_GUSBCFG_TSDPS  | USB_OTG_GUSBCFG_ULPIFSLS |
                USB_OTG_GUSBCFG_PHYSEL |
                USB_OTG_GUSBCFG_ULPIEVBUSD | USB_OTG_GUSBCFG_ULPIEVBUSI);
        }

        USB_prvReset(pxUSB);
    }
    else
#endif /* USB_OTG_HS */
    {
        /* Select FS Embedded PHY */
        USB_REG_BIT(pxUSB, GUSBCFG, PHYSEL) = 1;

        USB_prvReset(pxUSB);

        pxUSB->Inst->GCCFG.w = USB_OTG_GCCFG_PWRDWN;
    }
}

#ifdef USB_OTG_HS
/* Shuts down the HS PHY */
static void USB_prvHsPhyDeinit(USB_HandleType * pxUSB)
{
#if defined(USB_HS_PHYC) && defined(HSE_VALUE_Hz)
    if (USB_REG_BIT(pxUSB, GCCFG, PHYHSEN) != 0)
    {
        RCC_vClockDisable(RCC_POS_USBPHYC);
    }
    else
#endif
    {
        RCC_vClockDisable(RCC_POS_OTG_HS_ULPI);
    }
}
#endif /* USB_OTG_HS */

/** @defgroup USB_Exported_Functions USB Exported Functions
 * @{ */

/**
 * @brief Initializes the USB OTG peripheral using the setup configuration
 * @param pxUSB: pointer to the USB handle structure
 * @param pxConfig: USB setup configuration
 */
void USB_vDevInit(USB_HandleType * pxUSB, const USB_InitType * pxConfig)
{
    /* Enable peripheral clock */
#ifdef USB_OTG_HS
    if (IS_USB_OTG_HS(pxUSB->Inst))
    {
        RCC_vClockEnable(RCC_POS_OTG_HS);
    }
    else
#endif
    {
        RCC_vClockEnable(RCC_POS_OTG_FS);
    }

    /* Initialize handle variables */
    pxUSB->EP.OUT[0].MaxPacketSize =
    pxUSB->EP.IN [0].MaxPacketSize = USBD_EP0_MAX_PACKET_SIZE;
    pxUSB->EP.OUT[0].Type =
    pxUSB->EP.IN [0].Type = USB_EP_TYPE_CONTROL;
    pxUSB->LinkState = USB_LINK_STATE_OFF;

    /* Disable interrupts */
    USB_REG_BIT(pxUSB, GAHBCFG, GINT) = 0;

    /* Initialize dependencies (pins, IRQ lines) */
    XPD_SAFE_CALLBACK(pxUSB->Callbacks.DepInit, pxUSB);

    /* Initialize selected PHY */
    USB_prvPhyInit(pxUSB, pxConfig->PHY);

#if (USB_OTG_DMA_SUPPORT != 0)
    /* Set dedicated DMA */
    if (pxConfig->DMA != DISABLE)
    {
        SET_BIT(pxUSB->Inst->GAHBCFG.w,
                USB_OTG_GAHBCFG_HBSTLEN_2 | USB_OTG_GAHBCFG_DMAEN);
    }
#endif

    {
        uint8_t ucEpNum;
        uint8_t ucEpCount = USB_ENDPOINT_COUNT(pxUSB);

        /* Set Device Mode */
        MODIFY_REG(pxUSB->Inst->GUSBCFG.w,
                USB_OTG_GUSBCFG_FHMOD | USB_OTG_GUSBCFG_FDMOD,
                USB_OTG_GUSBCFG_FDMOD);

        /* Immediate soft disconnect */
        USB_REG_BIT(pxUSB,DCTL,SDIS) = 1;

        /* VBUS sensing unused */
#ifdef USB_OTG_GCCFG_VBDEN
        {
            USB_REG_BIT(pxUSB,GCCFG,VBDEN) = 0;

            SET_BIT(pxUSB->Inst->GOTGCTL.w,
                    USB_OTG_GOTGCTL_BVALOEN | USB_OTG_GOTGCTL_BVALOVAL);
        }
#else
        {
            USB_REG_BIT(pxUSB,GCCFG,NOVBUSSENS) = 1;
        }
#endif

        /* Restart the Phy Clock */
        pxUSB->Inst->PCGCCTL.w = 0;

#ifdef USB_OTG_HS
        /* HS PHY interfaces */
        if (pxConfig->PHY != USB_PHY_EMBEDDED_FS)
        {
            pxUSB->Inst->DCFG.b.DSPD = 0;
        }
        else
#endif
        {
            /* Internal FS Phy */
            pxUSB->Inst->DCFG.b.DSPD = 3;
        }

        /* Init endpoints */
        for (ucEpNum = 0; ucEpNum < ucEpCount; ucEpNum++)
        {
            USB_vEpClose(pxUSB, ucEpNum);
            USB_vEpClose(pxUSB, 0x80 | ucEpNum);
        }
        USB_REG_BIT(pxUSB,DIEPMSK,TXFURM) = 0;

#if (USB_OTG_DMA_SUPPORT != 0)
        if (USB_DMA_CONFIG(pxUSB) != 0)
        {
            /*Set threshold parameters */
            pxUSB->Inst->DTHRCTL.w = (
                    USB_OTG_DTHRCTL_TXTHRLEN_6  |
                    USB_OTG_DTHRCTL_RXTHRLEN_6  |
                    USB_OTG_DTHRCTL_RXTHREN     |
                    USB_OTG_DTHRCTL_ISOTHREN    |
                    USB_OTG_DTHRCTL_NONISOTHREN  );
        }
#endif

#ifdef USB_OTG_GLPMCFG_LPMEN
        /* Set Link Power Management feature (L1 sleep mode support) */
        if (pxConfig->LPM != DISABLE)
        {
            SET_BIT(pxUSB->Inst->GLPMCFG.w,
                USB_OTG_GLPMCFG_LPMEN | USB_OTG_GLPMCFG_LPMACK | USB_OTG_GLPMCFG_ENBESL);
        }
#endif
    }
}

/**
 * @brief Restores the USB peripheral to its default inactive state
 * @param pxUSB: pointer to the USB handle structure
 * @return ERROR if input is incorrect, OK if success
 */
void USB_vDevDeinit(USB_HandleType * pxUSB)
{
    USB_vDevStop_IT(pxUSB);

    /* Deinitialize dependencies */
    XPD_SAFE_CALLBACK(pxUSB->Callbacks.DepDeinit, pxUSB);

    /* Disable peripheral clock */
#ifdef USB_OTG_HS
    if (IS_USB_OTG_HS(pxUSB->Inst))
    {
        /* Disable any PHY clocking as well */
        USB_prvHsPhyDeinit(pxUSB);

        RCC_vClockDisable(RCC_POS_OTG_HS);
    }
    else
#endif
    {
        RCC_vClockDisable(RCC_POS_OTG_FS);
    }
}

/**
 * @brief Starts the USB device operation
 * @param pxUSB: pointer to the USB handle structure
 */
void USB_vDevStart_IT(USB_HandleType * pxUSB)
{
    uint32_t ulGINTMSK;

    /* Clear any pending interrupts except SRQ */
    pxUSB->Inst->GINTSTS.w  = ~USB_OTG_GINTSTS_SRQINT;
    USB_prvClearEpInts(pxUSB);

    /* Enable interrupts matching to the Device mode ONLY */
    ulGINTMSK = USB_OTG_GINTMSK_USBSUSPM | USB_OTG_GINTMSK_USBRST |
                USB_OTG_GINTMSK_ENUMDNEM | USB_OTG_GINTMSK_IEPINT |
                USB_OTG_GINTMSK_OEPINT   | USB_OTG_GINTMSK_WUIM   |
                USB_OTG_GINTMSK_RXFLVLM;

    /* When DMA is used, Rx data isn't read by IRQHandler */
    if (USB_DMA_CONFIG(pxUSB) != 0)
    {
        CLEAR_BIT(ulGINTMSK, USB_OTG_GINTMSK_RXFLVLM);
    }
#ifdef USB_OTG_GLPMCFG_LPMEN
    /* Set Link Power Management feature (L1 sleep mode support) */
    if (USB_REG_BIT(pxUSB,GLPMCFG,LPMEN) != DISABLE)
    {
        SET_BIT(ulGINTMSK, USB_OTG_GINTMSK_LPMINTM);
    }
#endif

    /* Apply interrupts selection */
    pxUSB->Inst->GINTMSK.w = ulGINTMSK;

    /* Also configure device endpoint interrupts */
    pxUSB->Inst->DIEPMSK.w = USB_OTG_DIEPMSK_XFRCM
            | USB_OTG_DIEPMSK_TOM | USB_OTG_DIEPMSK_EPDM;
    pxUSB->Inst->DOEPMSK.w = USB_OTG_DOEPMSK_XFRCM | USB_OTG_DOEPMSK_STUPM
#ifdef USB_OTG_DOEPMSK_OTEPSPRM
            | USB_OTG_DOEPMSK_OTEPSPRM
#endif
            | USB_OTG_DOEPMSK_EPDM;
    pxUSB->Inst->DAINTMSK.w = 0;

    USB_prvConnectCtrl(pxUSB, ENABLE);

    /* Enable global interrupts */
    USB_REG_BIT(pxUSB, GAHBCFG, GINT) = 1;
}

/**
 * @brief Disconnects the device from the USB host
 * @param pxUSB: pointer to the USB handle structure
 */
void USB_vDevStop_IT(USB_HandleType * pxUSB)
{
    /* Disable global interrupts */
    USB_REG_BIT(pxUSB, GAHBCFG, GINT) = 0;

    /* Clear any pending interrupts except SRQ */
    pxUSB->Inst->GINTSTS.w  = ~USB_OTG_GINTSTS_SRQINT;
    USB_prvClearEpInts(pxUSB);

    /* Clear interrupt masks */
    CLEAR_BIT(pxUSB->Inst->GINTMSK.w,
            USB_OTG_GINTMSK_USBSUSPM | USB_OTG_GINTMSK_USBRST |
            USB_OTG_GINTMSK_ENUMDNEM | USB_OTG_GINTMSK_IEPINT |
            USB_OTG_GINTMSK_OEPINT   | USB_OTG_GINTMSK_WUIM   |
#ifdef USB_OTG_GLPMCFG_LPMEN
            USB_OTG_GINTMSK_LPMINTM |
#endif
            USB_OTG_GINTMSK_RXFLVLM);
    pxUSB->Inst->DIEPMSK.w  = 0;
    pxUSB->Inst->DOEPMSK.w  = 0;
    pxUSB->Inst->DAINTMSK.w = 0;

    /* Flush the FIFOs */
    USB_prvFlushRxFifo(pxUSB);
    USB_prvFlushTxFifo(pxUSB, USB_ALL_TX_FIFOS);

    /* Virtual disconnect */
    USB_prvConnectCtrl(pxUSB, DISABLE);

    /* Set Link State to disconnected */
    pxUSB->LinkState = USB_LINK_STATE_OFF;
}

/**
 * @brief Sets the USB device address
 * @param pxUSB: pointer to the USB handle structure
 * @param ucAddress: new device address
 */
void USB_vSetAddress(USB_HandleType * pxUSB, uint8_t ucAddress)
{
    pxUSB->Inst->DCFG.b.DAD = ucAddress;
}

/**
 * @brief Sets endpoint buffers and opens the default control endpoint.
 * @param pxUSB: pointer to the USB handle structure
 */
void USB_vCtrlEpOpen(USB_HandleType * pxUSB)
{
    /* Allocate FIFO space for all used endpoints based on MPS */
    USB_vAllocateEPs(pxUSB);

    /* Open EP0 */
    USB_prvCtrlEpOpen(pxUSB);
}

/**
 * @brief Opens an endpoint.
 * @param pxUSB: pointer to the USB handle structure
 * @param ucEpAddress: endpoint address
 * @param eType: endpoint type
 * @param usMaxPacketSize: endpoint maximum data packet size
 */
void USB_vEpOpen(
        USB_HandleType *    pxUSB,
        uint8_t             ucEpAddress,
        USB_EndPointType    eType,
        uint16_t            usMaxPacketSize)
{
    USB_OTG_GenEndpointType * pxDEP = USB_EPR(pxUSB, ucEpAddress);
    USB_EndPointHandleType * pxEP = USB_GET_EP_AT(pxUSB, ucEpAddress);
    uint8_t ucEpNum = ucEpAddress & 0xF;

    pxEP->MaxPacketSize = usMaxPacketSize;
    pxEP->Type = eType;

    /* Activate Endpoint interrupts */
    if (ucEpAddress > 0x7F)
    {
        SET_BIT(pxUSB->Inst->DAINTMSK.w,
                1 << (ucEpNum + USB_OTG_DAINTMSK_IEPM_Pos));
    }
    else
    {
        SET_BIT(pxUSB->Inst->DAINTMSK.w,
                1 << (ucEpNum + USB_OTG_DAINTMSK_OEPM_Pos));
    }

    /* Check if currently inactive */
    if (pxDEP->DxEPCTL.b.USBAEP == 0)
    {
        pxDEP->DxEPCTL.b.MPSIZ  = pxEP->MaxPacketSize;
        pxDEP->DxEPCTL.b.EPTYP  = pxEP->Type;

        /* Only valid for IN EP, the field is reserved for OUT EPs */
        pxDEP->DxEPCTL.b.TXFNUM = ucEpNum;

        pxDEP->DxEPCTL.b.SD0PID_SEVNFRM = 1;
        pxDEP->DxEPCTL.b.USBAEP = 1;
    }
}

/**
 * @brief Closes an active endpoint (EP0 shall not be closed).
 * @param pxUSB: pointer to the USB handle structure
 * @param ucEpAddress: endpoint address
 */
void USB_vEpClose(USB_HandleType * pxUSB, uint8_t ucEpAddress)
{
    USB_OTG_GenEndpointType * pxDEP = USB_EPR(pxUSB, ucEpAddress);
    uint8_t ucEpNum = ucEpAddress & 0xF;

    /* Deactivate Endpoint */
    if (ucEpAddress > 0x7F)
    {
        /* Disable endpoint interrupts */
        CLEAR_BIT(pxUSB->Inst->DEACHMSK,
                1 << (ucEpNum + USB_OTG_DEACHINTMSK_IEP1INTM_Pos - 1));
        CLEAR_BIT(pxUSB->Inst->DAINTMSK.w,
                1 << (ucEpNum + USB_OTG_DAINTMSK_IEPM_Pos));

        /* Flush dedicated FIFO */
        USB_prvFlushTxFifo(pxUSB, ucEpNum);
    }
    else
    {
        /* Disable endpoint interrupts */
        CLEAR_BIT(pxUSB->Inst->DEACHMSK,
                1 << (ucEpNum + USB_OTG_DEACHINTMSK_OEP1INTM_Pos - 1));
        CLEAR_BIT(pxUSB->Inst->DAINTMSK.w,
                1 << (ucEpNum + USB_OTG_DAINTMSK_OEPM_Pos));
    }

    /* If a transfer is ongoing, interrupt with NACK */
    if (pxDEP->DxEPCTL.b.EPENA != 0)
    {
        pxDEP->DxEPCTL.w = USB_OTG_DOEPCTL_EPDIS | USB_OTG_DOEPCTL_SNAK;
    }
    else
    {
        pxDEP->DxEPCTL.w = 0;
    }

    pxDEP->DxEPTSIZ.w = 0;
    pxDEP->DxEPINT.w  = 0xFF;
}

/**
 * @brief Set a STALL condition on an endpoint
 * @param pxUSB: pointer to the USB handle structure
 * @param ucEpAddress: endpoint number
 */
void USB_vEpSetStall(USB_HandleType * pxUSB, uint8_t ucEpAddress)
{
    USB_OTG_GenEndpointType * pxDEP = USB_EPR(pxUSB, ucEpAddress);

    if (pxDEP->DxEPCTL.b.EPENA == 0)
    {
        pxDEP->DxEPCTL.b.EPDIS = 0;
    }
    pxDEP->DxEPCTL.b.STALL = 1;

    /* STALL-ed EP must still be able to receive SETUP */
    if (ucEpAddress == 0)
    {
        USB_prvPrepareSetup(pxUSB);
    }
}

/**
 * @brief Clear a STALL condition on an endpoint
 * @param pxUSB: pointer to the USB handle structure
 * @param ucEpAddress: endpoint number
 */
void USB_vEpClearStall(USB_HandleType * pxUSB, uint8_t ucEpAddress)
{
    USB_OTG_GenEndpointType * pxDEP = USB_EPR(pxUSB, ucEpAddress);

    pxDEP->DxEPCTL.b.STALL = 0;

    /* INTERRUPT || BULK -> set DATA0 PID */
    if (pxDEP->DxEPCTL.b.EPTYP >= USB_EP_TYPE_BULK)
    {
        pxDEP->DxEPCTL.b.SD0PID_SEVNFRM = 1;
    }
}

/**
 * @brief Clears the current data content of an active endpoint.
 * @param pxUSB: pointer to the USB handle structure
 * @param ucEpAddress: endpoint number
 */
void USB_vEpFlush(USB_HandleType * pxUSB, uint8_t ucEpAddress)
{
    if (ucEpAddress > 0x7F)
    {
        USB_prvFlushTxFifo(pxUSB, ucEpAddress & 0xF);
    }
    else
    {
        USB_prvFlushRxFifo(pxUSB);
    }
}

/**
 * @brief Initiates data reception on the OUT endpoint.
 * @param pxUSB: pointer to the USB handle structure
 * @param ucEpAddress: endpoint address
 * @param pucData: pointer to the data buffer
 * @param usLength: amount of data bytes to transfer
 */
void USB_vEpReceive(
        USB_HandleType *    pxUSB,
        uint8_t             ucEpAddress,
        uint8_t *           pucData,
        uint16_t            usLength)
{
    USB_EndPointHandleType * pxEP = &pxUSB->EP.OUT[ucEpAddress];

    /* setup transfer */
    pxEP->Transfer.Data       = pucData;
    pxEP->Transfer.Progress   = usLength;
    pxEP->Transfer.Length     = 0;

    USB_prvEpReceive(pxUSB, ucEpAddress);
}

/**
 * @brief Initiates data transmission on the IN endpoint.
 * @param pxUSB: pointer to the USB handle structure
 * @param ucEpAddress: endpoint address
 * @param pucData: pointer to the data buffer
 * @param usLength: amount of data bytes to transfer
 */
void USB_vEpSend(
        USB_HandleType *    pxUSB,
        uint8_t             ucEpAddress,
        const uint8_t *     pucData,
        uint16_t            usLength)
{
    uint8_t ucEpNum = ucEpAddress & 0xF;
    USB_EndPointHandleType * pxEP = &pxUSB->EP.IN[ucEpNum];

    /* setup and start the transfer */
    pxEP->Transfer.Data       = (uint8_t*)pucData;
    pxEP->Transfer.Progress   = usLength;
    pxEP->Transfer.Length     = usLength;

    USB_prvEpSend(pxUSB, ucEpNum);
}

/**
 * @brief USB interrupt handler that provides event-driven peripheral management
 *        and handle callbacks.
 * @param pxUSB: pointer to the USB handle structure
 */
void USB_vDevIRQHandler(USB_HandleType * pxUSB)
{
    uint32_t ulGINT = pxUSB->Inst->GINTSTS.w & pxUSB->Inst->GINTMSK.w;

    if (ulGINT != 0)
    {
        union {
            struct {
                uint16_t IEPINT;
                uint16_t OEPINT;
            }b;
            uint32_t w;
        }xDAINT = { .w = pxUSB->Inst->DAINT.w & pxUSB->Inst->DAINTMSK.w };

        /* Rx FIFO level reached */
        if ((ulGINT & USB_OTG_GINTSTS_RXFLVL) != 0)
        {
            uint32_t ulGRXSTSP  = pxUSB->Inst->GRXSTSP.w;
            uint16_t usDataCount= (ulGRXSTSP & USB_OTG_GRXSTSP_BCNT_Msk)
                                            >> USB_OTG_GRXSTSP_BCNT_Pos;
            uint8_t  ucEpNum    = (ulGRXSTSP & USB_OTG_GRXSTSP_EPNUM_Msk)
                                            >> USB_OTG_GRXSTSP_EPNUM_Pos;
            USB_EndPointHandleType * pxEP = &pxUSB->EP.OUT[ucEpNum];

            switch (ulGRXSTSP & USB_OTG_GRXSTSP_PKTSTS_Msk)
            {
                case STS_DATA_UPDT:
                    /* Data packet received */
                    USB_prvReadFifo(pxUSB, pxEP->Transfer.Data, usDataCount);
                    pxEP->Transfer.Length += usDataCount;
                    pxEP->Transfer.Data += usDataCount;
                    break;

                case STS_SETUP_UPDT:
                    /* Setup packet received */
                    USB_prvReadFifo(pxUSB, (uint8_t *)&pxUSB->Setup,
                            sizeof(pxUSB->Setup));
                    break;

                default:
                    break;
            }
        }

        /* OUT endpoint interrupts */
        if ((ulGINT & USB_OTG_GINTSTS_OEPINT) != 0)
        {
            uint8_t ucEpNum;

            /* Handle individual endpoint interrupts */
            for (ucEpNum = 0; xDAINT.b.OEPINT != 0; ucEpNum++, xDAINT.b.OEPINT >>= 1)
            {
                if ((xDAINT.b.OEPINT & 1) != 0)
                {
                    USB_prvOutEpEventHandler(pxUSB, ucEpNum);
                }
            }
        }

        /* IN endpoint interrupts */
        if ((ulGINT & USB_OTG_GINTSTS_IEPINT) != 0)
        {
            uint8_t ucEpNum;

            /* Handle individual endpoint interrupts */
            for (ucEpNum = 0; xDAINT.b.IEPINT != 0; ucEpNum++, xDAINT.b.IEPINT >>= 1)
            {
                if ((xDAINT.b.IEPINT & 1) != 0)
                {
                    USB_prvInEpEventHandler(pxUSB, ucEpNum);
                }
            }
        }

        /* Handle Reset Interrupt */
        if ((ulGINT & USB_OTG_GINTSTS_USBRST) != 0)
        {
            /* Clear IT flag */
            USB_FLAG_CLEAR(pxUSB, USBRST);

            pxUSB->LinkState = USB_LINK_STATE_ACTIVE;

            /* Stop any ongoing Remote Wakeup signaling and EP0 transfers */
            USB_REG_BIT(pxUSB,DCTL,RWUSIG) = 0;
            USB_prvFlushRxFifo(pxUSB);
            USB_prvFlushTxFifo(pxUSB, 0);

            /* Clear EP interrupt flags */
            USB_prvClearEpInts(pxUSB);

            /* Set default address (0) */
            USB_vSetAddress(pxUSB, 0);
        }

        /* Handle Enumeration done Interrupt */
        if ((ulGINT & USB_OTG_GINTSTS_ENUMDNE) != 0)
        {
            USB_SpeedType eSpeed = USB_SPEED_FULL;

            /* Clear IT flag */
            USB_FLAG_CLEAR(pxUSB, ENUMDNE);

            /* Clear global IN NAK */
            USB_REG_BIT(pxUSB,DCTL,CGINAK) = 1;

#ifdef USB_OTG_HS
            /* High speed enumerated */
            if (pxUSB->Inst->DSTS.b.ENUMSPD == 0)
            {
                eSpeed = USB_SPEED_HIGH;
                pxUSB->Inst->GUSBCFG.b.TRDT = 9;
            }
            else
#endif
            {
                /* Full speed enumeration */
                uint32_t ulTRDT;

                /* Get most suitable value depending on AHB frequency */
                ulTRDT = 224000000 / RCC_ulClockFreq_Hz(HCLK);
                if (ulTRDT < 6)
                {
                    ulTRDT = 6;
                }
                pxUSB->Inst->GUSBCFG.b.TRDT = ulTRDT;
            }

            /* Notify device handler */
            USB_vResetCallback(pxUSB, eSpeed);
        }

        /* Handle Resume Interrupt */
        if ((ulGINT & USB_OTG_GINTSTS_WKUINT) != 0)
        {
            /* Stop any ongoing Remote Wakeup signaling */
            USB_REG_BIT(pxUSB,DCTL,RWUSIG) = 0;

            USB_FLAG_CLEAR(pxUSB, WKUINT);

            XPD_SAFE_CALLBACK(pxUSB->Callbacks.Resume, pxUSB);

            /* LPM state is changed after Resume callback
             * -> possible to determine exited suspend level */
            pxUSB->LinkState = USB_LINK_STATE_ACTIVE;
        }

#ifdef USB_OTG_GLPMCFG_LPMEN
        /* Handle L1 suspend request */
        if ((ulGINT & USB_OTG_GINTSTS_LPMINT) != 0)
        {
            USB_FLAG_CLEAR(pxUSB, LPMINT);

            /* Set the target Link State */
            pxUSB->LinkState = USB_LINK_STATE_SLEEP;
            XPD_SAFE_CALLBACK(pxUSB->Callbacks.Suspend, pxUSB);
        }
#endif

        /* Handle Suspend Interrupt */
        if ((ulGINT & USB_OTG_GINTSTS_USBSUSP) != 0)
        {
            USB_FLAG_CLEAR(pxUSB, USBSUSP);

            if (USB_REG_BIT(pxUSB,DSTS,SUSPSTS) != 0)
            {
                /* Set the target Link State */
                pxUSB->LinkState = USB_LINK_STATE_SUSPEND;
                XPD_SAFE_CALLBACK(pxUSB->Callbacks.Suspend, pxUSB);
            }
        }

        /* Handle SOF Interrupt */
        if ((ulGINT & USB_OTG_GINTSTS_SOF) != 0)
        {
            USB_FLAG_CLEAR(pxUSB, SOF);

            XPD_SAFE_CALLBACK(pxUSB->Callbacks.SOF, pxUSB);
        }
    }
}

/**
 * @brief Activates remote wake-up signaling.
 * @param pxUSB: pointer to the USB handle structure
 * @return
 */
void USB_vSetRemoteWakeup(USB_HandleType * pxUSB)
{
    if (USB_REG_BIT(pxUSB,DSTS,SUSPSTS) != 0)
    {
        /* Activate Remote wakeup signaling
         * If LPM is supported and in L1 Sleep state,
         * this bit is cleared automatically */
        USB_REG_BIT(pxUSB,DCTL,RWUSIG) = 1;
    }
}

/**
 * @brief Deactivates remote wake-up signaling.
 * @param pxUSB: pointer to the USB handle structure
 */
void USB_vClearRemoteWakeup(USB_HandleType * pxUSB)
{
    /* Deactivate Resume signal after 1 - 15 ms */
    USB_REG_BIT(pxUSB,DCTL,RWUSIG) = 0;
}

/**
 * @brief Executes Battery Charger Detection algorithm.
 * @note  This function shall be executed before the USB peripheral is connected
 *        (started) with DP pull-up.
 * @param pxUSB: pointer to the USB handle structure
 * @return The determined upstream port type
 */
USB_ChargerType USB_eChargerDetect(USB_HandleType * pxUSB)
{
#ifndef USB_OTG_GCCFG_BCDEN
    USB_ChargerType eDetection = USB_BCD_NOT_SUPPORTED;
#else
    /* USB power is always available before the data lines are contacted */
    USB_ChargerType eDetection = USB_BCD_NO_DATA_CONTACT;

    /* Enable Battery Charger Detection */
    USB_REG_BIT(pxUSB,GCCFG,BCDEN) = 1;

    /* Check if device is connected */
    if (USB_REG_BIT(pxUSB,DCTL,SDIS) != 0)
    {
        /* Adjust depending on the speed of plugging in the device */
        uint32_t ulTimeout = 1000;

        USB_REG_BIT(pxUSB,GCCFG,DCDEN) = 1;

        /* Data Contact Detect: determines contact of data lines
         * to the bus powering entity */
        if (XPD_OK == XPD_eWaitForDiff((void*)&pxUSB->Inst->GCCFG.w,
                USB_OTG_GCCFG_DCDET, 0, &ulTimeout))
        {
            /* Bus electric stabilization */
            XPD_vDelay_ms(100);

            /* Primary Detection:
             * distinguishes between a Standard Downstream Port
             * and Charging Ports */
            pxUSB->Inst->GCCFG.w = (pxUSB->Inst->GCCFG.w &
                    ~USB_OTG_GCCFG_DCDEN) | USB_OTG_GCCFG_PDEN;

            /* Bus electric stabilization */
            XPD_vDelay_ms(100);

            /* Check the results of PD */
            switch (pxUSB->Inst->GCCFG.w & (USB_OTG_GCCFG_PS2DET | USB_OTG_GCCFG_PDET))
            {
                case USB_OTG_GCCFG_PS2DET:
                    /* D- is externally pulled high during Primary Detection
                     * PS2 port or proprietary charger */
                    eDetection = USB_BCD_PS2_PROPRIETARY_PORT;
                    break;

                case USB_OTG_GCCFG_PDET:
                {
                    /* Secondary Detection:
                     * distinguishes between Charging Downstream Port
                     * and Dedicated Charging Port */
                    pxUSB->Inst->GCCFG.w = (pxUSB->Inst->GCCFG.w &
                            ~USB_OTG_GCCFG_PDEN) | USB_OTG_GCCFG_SDEN;

                    /* Bus electric stabilization */
                    XPD_vDelay_ms(100);

                    if (USB_REG_BIT(pxUSB,GCCFG,SDET) != 0)
                    {
                        /* Dedicated Downstream Port DCP */
                        eDetection = USB_BCD_DEDICATED_CHARGING_PORT;
                    }
                    else
                    {
                        /* Charging Downstream Port CDP */
                        eDetection = USB_BCD_CHARGING_DOWNSTREAM_PORT;
                    }
                    break;
                }

                default:
                    /* No downstream side support for PD
                     * Standard Downstream Port SDP */
                    eDetection = USB_BCD_STANDARD_DOWNSTREAM_PORT;
                    break;
            }
        }
    }
    /* Disable Battery Charger Detection */
    CLEAR_BIT(pxUSB->Inst->GCCFG.w,
            USB_OTG_GCCFG_BCDEN | USB_OTG_GCCFG_DCDEN |
            USB_OTG_GCCFG_PDEN | USB_OTG_GCCFG_SDEN);
#endif
    return eDetection;
}

/**
 * @brief Determines the USB peripheral's current bus speed.
 * @param pxUSB: pointer to the USB handle structure
 * @return The current USB device speed
 */
USB_SpeedType USB_eDevSpeed(USB_HandleType * pxUSB)
{
    USB_SpeedType eSpeed;
#ifdef USB_OTG_HS
    /* High speed enumerated */
    if (pxUSB->Inst->DSTS.b.ENUMSPD == 0)
    {
        eSpeed = USB_SPEED_HIGH;
    }
    else
#endif
    {
        eSpeed = USB_SPEED_FULL;
    }
    return eSpeed;
}

/**
 * @brief Configure peripheral FIFO allocation for endpoints
 *        after device initialization and before starting the USB operation.
 * @param pxUSB: pointer to the USB handle structure
 */
__weak void USB_vAllocateEPs(USB_HandleType * pxUSB)
{
    XPD_ReturnType eResult = XPD_OK;
    uint8_t ucEpNum;
    uint8_t ucEpCount = USB_ENDPOINT_COUNT(pxUSB);
    uint32_t ulMinFifoSizeVal = 16;
    uint32_t ulFifoSize = ulMinFifoSizeVal * sizeof(uint32_t);
    uint32_t ulFifoOffset;
    uint32_t ulFifoLimit = USB_TOTAL_FIFO_SIZE(pxUSB);

    /* Configure the global Receive FIFO based on the largest requested OUT EP size */
    for (ucEpNum = 0; ucEpNum < ucEpCount; ucEpNum++)
    {
        if (pxUSB->EP.OUT[ucEpNum].MaxPacketSize > ulFifoSize)
        {
            ulFifoSize = pxUSB->EP.OUT[ucEpNum].MaxPacketSize;
        }
    }

    /* FIFO sizes are in words */
    ulFifoSize = (ulFifoSize + 3) >> 2;

    /* Global RX FIFO according to RM0431: */
    ulFifoOffset = 10           /* to receive SETUP packets on the control endpoint */
            + (ulFifoSize + 1)  /* each packet gets status info as well */
            + (ucEpCount * 2)   /* transfer complete status is also stored with the last packet */
            + 1;                /* for Global OUT NAK */
    pxUSB->Inst->GRXFSIZ = ulFifoOffset;

    /* EP0 TX FIFO */
    ulFifoSize = (pxUSB->EP.IN[0].MaxPacketSize + 3) >> 2;
    if (ulFifoSize < ulMinFifoSizeVal)
    {   ulFifoSize = ulMinFifoSizeVal; }

    pxUSB->Inst->DIEPTXF0_HNPTXFSIZ.w =
            (ulFifoSize   << USB_OTG_DIEPTXF_INEPTXFD_Pos) |
            (ulFifoOffset << USB_OTG_DIEPTXF_INEPTXSA_Pos);

    for (ucEpNum = 1; ucEpNum < ucEpCount; ucEpNum++)
    {
        /* Increase offset with the FIFO size */
        ulFifoOffset += ulFifoSize;

        /* FIFO sizes are in words */
        ulFifoSize = (pxUSB->EP.IN[ucEpNum].MaxPacketSize + 3) >> 2;
        if (ulFifoSize < ulMinFifoSizeVal)
        {   ulFifoSize = ulMinFifoSizeVal; }

        /* EPx TX FIFOs */
        pxUSB->Inst->DIEPTXF[ucEpNum - 1].w =
                (ulFifoSize   << USB_OTG_DIEPTXF_INEPTXFD_Pos) |
                (ulFifoOffset << USB_OTG_DIEPTXF_INEPTXSA_Pos);
    }

    /* Total FIFO use shouldn't exceed available size */
    if (ulFifoLimit < (ulFifoOffset + ulFifoSize))
    {
        eResult = XPD_ERROR;
    }

    (void) eResult;
}

/** @} */

/** @} */

#endif /* defined(USB_OTG_FS) */
