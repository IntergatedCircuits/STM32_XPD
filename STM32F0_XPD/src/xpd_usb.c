/**
  ******************************************************************************
  * @file    xpd_usb.c
  * @author  Benedek Kupper
  * @version 0.3
  * @date    2018-01-28
  * @brief   STM32 eXtensible Peripheral Drivers Universal Serial Bus Module
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
#include <xpd_usb.h>
#include <xpd_rcc.h>
#include <xpd_utils.h>

#if defined(USB)

/** @addtogroup USB
 * @{ */

/* The packet buffer memory SRAM access scheme differs in earlier versions */
#ifdef USB_LPMCSR_LPMEN
typedef struct {
         uint16_t TX_ADDR;
    __IO uint16_t TX_COUNT;
         uint16_t RX_ADDR;
    __IO uint16_t RX_COUNT;
}USB_BufferDescriptorType;

typedef uint16_t USB_PacketAddressType;
#else
typedef struct {
         uint32_t TX_ADDR;
    __IO uint32_t TX_COUNT;
         uint32_t RX_ADDR;
    __IO uint32_t RX_COUNT;
}USB_BufferDescriptorType;

typedef uint32_t USB_PacketAddressType;
#endif

#define USB_BTABLE_VALUE            0

#ifdef USB_BTABLE_VALUE
/* Optimized BufferDescriptorTable access */
#define USB_EP_BDT ((USB_BufferDescriptorType *)(USB_PMAADDR + USB_BTABLE_VALUE))
#else
#define USB_EP_BDT ((USB_BufferDescriptorType *)(USB_PMAADDR + USB->BTABLE))
#endif

#define USB_RXCNT_NUM_BLOCK_Pos     10U
#define USB_RXCNT_NUM_BLOCK_Msk     (0x1FU << USB_RXCNT_NUM_BLOCK_Pos)
#define USB_RXCNT_BL_SIZE           0x8000

#define USB_TOGGLE(EP_ID, BIT_NAME) \
    (USB->EPR[EP_ID].w = (USB->EPR[EP_ID].w & USB_EPREG_MASK)\
            | USB_EP_CTR_RX | USB_EP_CTR_TX | USB_EP_ ## BIT_NAME)

/* if the DTOG bit is not 1, write a 1 to it to toggle back to 0 */
#define USB_TOGGLE_CLEAR(EP_ID, BIT_NAME) \
do { if (USB_REG_BIT(pxUSB,EPR[EP_ID],BIT_NAME) != 0) USB_TOGGLE(EP_ID, BIT_NAME); }while(0)

#define USB_EPRX_TX_DTOG1    (USB_EPRX_DTOG1 | USB_EPTX_DTOG1)
#define USB_EPRX_TX_DTOG2    (USB_EPRX_DTOG2 | USB_EPTX_DTOG2)
#define USB_EPRX_TX_DTOGMASK (USB_EPRX_DTOGMASK | USB_EPTX_DTOGMASK)
#define USB_EP_RX_TX_STALL   (USB_EP_RX_STALL | USB_EP_TX_STALL)

#define USB_EP_SET_STATUS(EP_ID, DIRECTION, STATUS) \
    (USB->EPR[EP_ID].w = ((USB->EPR[EP_ID].w & USB_EP##DIRECTION##_DTOGMASK)\
    ^ USB_EP_##DIRECTION##_##STATUS) | USB_EP_CTR_RX | USB_EP_CTR_TX)

#define USB_EP_FLAG_CLEAR(EP_ID, FLAG_NAME)     \
        (USB->EPR[EP_ID].w &= USB_EPREG_MASK & (~USB_EP_ ## FLAG_NAME))

#define USB_GET_EP_AT(HANDLE, NUMBER)   (((NUMBER) > 0x7F) ?            \
        (&(HANDLE)->EP.IN[(NUMBER) & 0xF]) :                            \
        (&(HANDLE)->EP.OUT[NUMBER]))

#define USB_EP_DOUBLE_BUFFERED(ENDPOINT)  ((ENDPOINT)->Type == USB_EP_TYPE_ISOCHRONOUS)

static const uint16_t usb_ausEpTypeRemap[4] = {
    USB_EP_CONTROL,
    USB_EP_ISOCHRONOUS,
    USB_EP_BULK,
    USB_EP_INTERRUPT
};

/* Writes user data to USB endpoint packet memory */
static void USB_prvWritePMA(uint8_t * pucSrcBuf, uint16_t usPmaAddress, uint16_t usDataCount)
{
    USB_PacketAddressType * pxDst = (USB_PacketAddressType *)USB_PMAADDR + (usPmaAddress / 2);
    uint16_t usWCount;

    /* Assemble halfwords and copy them to packet memory */
    for (usWCount = (usDataCount + 1) / 2; usWCount > 0; usWCount--)
    {
        *pxDst = ((uint16_t)(pucSrcBuf[1]) << 8) | (uint16_t)(pucSrcBuf[0]);
        pxDst++;
        pucSrcBuf += 2;
    }
}

/* Reads USB endpoint data from packet memory */
static void USB_prvReadPMA(uint8_t * pucDstBuf, uint16_t usPmaAddress, uint16_t usDataCount)
{
    USB_PacketAddressType * pxSrc = (USB_PacketAddressType *)USB_PMAADDR + (usPmaAddress / 2);
    uint16_t usWCount;

    /* Copy each halfword into the byte buffer */
    for (usWCount = usDataCount / 2; usWCount > 0; usWCount--)
    {
        uint16_t usData = *pxSrc;
        *pucDstBuf = usData;
        pucDstBuf++;
        *pucDstBuf = usData >> 8;
        pucDstBuf++;
        pxSrc++;
    }

    /* The last, unaligned byte is filled if exists */
    if ((usDataCount & 1) != 0)
    {
        *pucDstBuf = (uint8_t)*pxSrc;
    }
}

/* Setting RX_COUNT requires special conversion */
static uint16_t USB_prvConvertRxCount(uint16_t usRxCount)
{
    uint16_t usBlocks;
    if(usRxCount > 62)
    {
        /* Rx count to blocks of 32 */
        usBlocks = ((usRxCount + 31) >> 5) - 1;
        usBlocks = USB_RXCNT_BL_SIZE | (usBlocks << USB_RXCNT_NUM_BLOCK_Pos);
    }
    else
    {
        /* Rx count to blocks of 2 */
        usBlocks = (usRxCount + 1) >> 1;
        usBlocks = (usBlocks << USB_RXCNT_NUM_BLOCK_Pos);
    }
    return usBlocks;
}

/* Determines the next packet size based on the transfer progress and the EP MPS */
static uint16_t USB_prvNextPacketSize(USB_EndPointHandleType * pxEP)
{
    uint16_t usPacketLength;

    /* Multi packet transfer */
    if (pxEP->Transfer.Progress > pxEP->MaxPacketSize)
    {
        pxEP->Transfer.Progress -= pxEP->MaxPacketSize;
        usPacketLength = pxEP->MaxPacketSize;
    }
    else
    {
        usPacketLength = pxEP->Transfer.Progress;
        pxEP->Transfer.Progress = 0;
    }

    return usPacketLength;
}

/* Handle OUT EP transfer */
static void USB_prvReceivePacket(USB_HandleType * pxUSB, USB_EndPointHandleType * pxEP)
{
    uint16_t usPacketLength = USB_prvConvertRxCount(USB_prvNextPacketSize(pxEP));

    /* Double buffering */
    if (USB_EP_DOUBLE_BUFFERED(pxEP) && ((USB->EPR[pxEP->RegId].w & USB_EP_DTOG_RX) == 0))
    {
        /* Set endpoint buffer 0 count */
        USB_EP_BDT[pxEP->RegId].TX_COUNT = usPacketLength;
    }
    else
    {
        /*Set RX buffer count */
        USB_EP_BDT[pxEP->RegId].RX_COUNT = usPacketLength;
    }

    USB_EP_SET_STATUS(pxEP->RegId, RX, VALID);
}

/* Handle IN EP transfer */
static void USB_prvTransmitPacket(USB_HandleType * pxUSB, USB_EndPointHandleType * pxEP)
{
    uint16_t usPmaAddress = USB_EP_BDT[pxEP->RegId].TX_ADDR;
    uint16_t usPacketLength = USB_prvNextPacketSize(pxEP);

    if (!USB_EP_DOUBLE_BUFFERED(pxEP))
    {
        USB_EP_BDT[pxEP->RegId].TX_COUNT = usPacketLength;

        /* Write the data to the packet memory */
        USB_prvWritePMA(pxEP->Transfer.Data, usPmaAddress, usPacketLength);

        /* Validate Tx endpoint */
        USB_EP_SET_STATUS(pxEP->RegId, TX, VALID);
    }
    else /* Double buffered endpoint */
    {
        /* Use buffer 1 when DTOG == 1 */
        if ((USB->EPR[pxEP->RegId].w & USB_EP_DTOG_TX) != 0)
        {
            USB_EP_BDT[pxEP->RegId].RX_COUNT = usPacketLength;
            usPmaAddress = USB_EP_BDT[pxEP->RegId].RX_ADDR;
        }
        else
        {
            USB_EP_BDT[pxEP->RegId].TX_COUNT = usPacketLength;
        }

        /* Write the data to the packet memory */
        USB_prvWritePMA(pxEP->Transfer.Data, usPmaAddress, usPacketLength);

        /* Toggle SW_BUF flag to clear NAK status (DTOG == SW_BUF) */
        if (USB->EPR[pxEP->RegId].b.DTOG_TX == USB->EPR[pxEP->RegId].b.DTOG_RX)
        {
            USB_TOGGLE(pxEP->RegId, DTOG_RX);
        }
    }
}

/* Opens EP0 bidirectional dedicated control endpoint */
static void USB_prvCtrlEpOpen(USB_HandleType * pxUSB)
{
    /* Configure EP0 type; address */
    USB->EPR[0].w = (USB->EPR[0].w & USB_EP_T_MASK)
            | USB_EP_CONTROL;

    USB->EPR[0].w = (USB->EPR[0].w & USB_EPREG_MASK)
            | USB_EP_CTR_RX | USB_EP_CTR_TX | 0;

    /* IN direction */
    {
        /*Set the endpoint Transmit buffer address */
        USB_EP_BDT[0].TX_COUNT = 0;

        USB_TOGGLE_CLEAR(0, DTOG_TX);

        /* Configure NAK status for the Endpoint */
        USB_EP_SET_STATUS(0, TX, NAK);
    }
    /* OUT direction */
    {
        /* Set the endpoint Receive buffer address and counter */
        USB_EP_BDT[0].RX_COUNT = USB_prvConvertRxCount(pxUSB->EP.OUT[0].MaxPacketSize);

        USB_TOGGLE_CLEAR(0, DTOG_RX);

        /* Configure VALID status for the Endpoint */
        USB_EP_SET_STATUS(0, RX, VALID);
    }
}

/** @defgroup USB_Exported_Functions USB Exported Functions
 * @{ */

/**
 * @brief Initializes the USB peripheral using the setup configuration
 * @param pxUSB: pointer to the USB handle structure
 * @param pxConfig: USB setup configuration
 */
void USB_vInit(USB_HandleType * pxUSB, const USB_InitType * pxConfig)
{
    /* Enable peripheral clock */
    RCC_vClockEnable(RCC_POS_USB);

    /* Initialize handle variables */
    pxUSB->EP.OUT[0].MaxPacketSize =
    pxUSB->EP.IN [0].MaxPacketSize = USBD_EP0_MAX_PACKET_SIZE;
    pxUSB->EP.OUT[0].Type =
    pxUSB->EP.IN [0].Type = USB_EP_TYPE_CONTROL;
    pxUSB->LinkState = USB_LINK_STATE_OFF;

    /* Initialize peripheral device */
    /* FRES = 1 */
    USB->CNTR.w = USB_CNTR_FRES;

    /* FRES = 0 */
    USB->CNTR.w = 0;

    /*Clear pending interrupts */
    USB->ISTR.w = 0;

    /* Set Btable Address */
    USB->BTABLE = USB_BTABLE_VALUE;

#ifdef USB_LPMCSR_LPMEN
    /* Set Link Power Management feature (L1 sleep mode support) */
    if (pxConfig->LPM != DISABLE)
    {
        SET_BIT(USB->LPMCSR.w, USB_LPMCSR_LPMEN | USB_LPMCSR_LPMACK);
    }
    else
    {
        CLEAR_BIT(USB->LPMCSR.w, USB_LPMCSR_LPMEN | USB_LPMCSR_LPMACK);
    }
#endif

    /* Initialize dependencies (pins, IRQ lines) */
    XPD_SAFE_CALLBACK(pxUSB->Callbacks.DepInit, pxUSB);
}

/**
 * @brief Restores the USB peripheral to its default inactive state
 * @param pxUSB: pointer to the USB handle structure
 */
void USB_vDeinit(USB_HandleType * pxUSB)
{
    /* Stop device if not done so far */
    USB_vStop_IT(pxUSB);

    /* Switch-off device */
    USB->CNTR.w = USB_CNTR_FRES | USB_CNTR_PDWN;

    /* Deinitialize dependencies */
    XPD_SAFE_CALLBACK(pxUSB->Callbacks.DepDeinit, pxUSB);

    /* Peripheral clock disabled */
    RCC_vClockDisable(RCC_POS_USB);
}

/**
 * @brief Starts the USB device operation with necessary interrupts.
 * @param pxUSB: pointer to the USB handle structure
 */
void USB_vStart_IT(USB_HandleType * pxUSB)
{
    /*Set interrupt mask */
    uint32_t ulCNTR = USB_CNTR_CTRM | USB_CNTR_WKUPM | USB_CNTR_SUSPM | USB_CNTR_RESETM;
#ifdef USB_LPMCSR_LPMEN
    /* Set Link Power Management feature (L1 sleep mode support) */
    if (USB_REG_BIT(pxUSB, LPMCSR, LPMEN) != 0)
    {
        ulCNTR |= USB_CNTR_L1REQM;
    }
#endif
    /* Apply interrupts selection */
    USB->CNTR.w |= ulCNTR;

    /* Activate DP line pull up */
#ifdef USB_BCDR_DPPU
    USB_REG_BIT(pxUSB,BCDR, DPPU) = 1;
#else
    XPD_SAFE_CALLBACK(pxUSB->Callbacks.ConnectCtrl, ENABLE);
#endif
}

/**
 * @brief Disconnects the device from the USB host and disables all interrupts.
 * @param pxUSB: pointer to the USB handle structure
 */
void USB_vStop_IT(USB_HandleType * pxUSB)
{
    /* Disable all interrupts */
    CLEAR_BIT(USB->CNTR.w, 0xFF80);

    /* Clear interrupt status register */
    USB->ISTR.w = 0;

    /* Deactivate DP line pull up */
#ifdef USB_BCDR_DPPU
    USB_REG_BIT(pxUSB,BCDR, DPPU) = 0;
#else
    XPD_SAFE_CALLBACK(pxUSB->Callbacks.ConnectCtrl, DISABLE);
#endif
    pxUSB->LinkState = USB_LINK_STATE_OFF;
}

/**
 * @brief Sets the USB device address.
 * @param pxUSB: pointer to the USB handle structure
 * @param Address: new device address
 */
void USB_vSetAddress(USB_HandleType * pxUSB, uint8_t ucAddress)
{
    USB->DADDR.w = USB_DADDR_EF | ucAddress;
}

/**
 * @brief Sets endpoint buffers and opens the default control endpoint.
 * @param pxUSB: pointer to the USB handle structure
 */
void USB_vCtrlEpOpen(USB_HandleType * pxUSB)
{
    /* Allocate packet memory for all used endpoints based on MPS */
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
    USB_EndPointHandleType * pxEP = USB_GET_EP_AT(pxUSB, ucEpAddress);
    uint8_t ucEpNum = ucEpAddress & 0xF;

    pxEP->MaxPacketSize = usMaxPacketSize;
    pxEP->Type          = eType;

    /* Configure EP type */
    USB->EPR[pxEP->RegId].w = (USB->EPR[pxEP->RegId].w & USB_EP_T_MASK)
            | usb_ausEpTypeRemap[pxEP->Type];

    /* Configure EP address */
    USB->EPR[pxEP->RegId].w = (USB->EPR[pxEP->RegId].w & USB_EPREG_MASK)
            | USB_EP_CTR_RX | USB_EP_CTR_TX | ucEpNum;

    /* Double buffer */
    if (USB_EP_DOUBLE_BUFFERED(pxEP))
    {
        /* Set the endpoint as double buffered */
        USB->EPR[pxEP->RegId].w = (USB->EPR[pxEP->RegId].w & USB_EPREG_MASK)
            | USB_EP_CTR_RX | USB_EP_CTR_TX | USB_EP_KIND;

        /* Clear the data toggle bits for the endpoint IN/OUT */
        USB_TOGGLE_CLEAR(pxEP->RegId, DTOG_RX);
        USB_TOGGLE_CLEAR(pxEP->RegId, DTOG_TX);

        /* Initially no data */
        USB_EP_BDT[pxEP->RegId].TX_COUNT =
        USB_EP_BDT[pxEP->RegId].RX_COUNT = 0;

        if (ucEpAddress > 0x7F)
        {
            /* DTOG == SW_BUF == 0 result in NAK */
            USB_EP_SET_STATUS(pxEP->RegId, TX, VALID);
            /* Disable unused direction */
            USB_EP_SET_STATUS(pxEP->RegId, RX, DIS);
        }
        else
        {
            /* Set SW_BUF flag */
            USB_TOGGLE(pxEP->RegId, DTOG_TX);

            /* Configure VALID status for the Endpoint */
            USB_EP_SET_STATUS(pxEP->RegId, RX, VALID);
            /* Disable unused direction */
            USB_EP_SET_STATUS(pxEP->RegId, TX, DIS);
        }
    }
    /* Configure NAK status for the Endpoint */
    else if (ucEpAddress > 0x7F)
    {
        USB_TOGGLE_CLEAR(pxEP->RegId, DTOG_TX);
        USB_EP_SET_STATUS(pxEP->RegId, TX, NAK);
    }
    else
    {
        USB_TOGGLE_CLEAR(pxEP->RegId, DTOG_RX);
        USB_EP_SET_STATUS(pxEP->RegId, RX, NAK);
    }
}

/**
 * @brief Closes an active endpoint (EP0 shall not be closed).
 * @param pxUSB: pointer to the USB handle structure
 * @param ucEpAddress: endpoint address
 */
void USB_vEpClose(USB_HandleType * pxUSB, uint8_t ucEpAddress)
{
    USB_EndPointHandleType * pxEP = USB_GET_EP_AT(pxUSB, ucEpAddress);

    if (ucEpAddress > 0x7F)
    {
        /* Configure DISABLE status for the Endpoint*/
        USB_TOGGLE_CLEAR(pxEP->RegId, DTOG_TX);
        USB_EP_SET_STATUS(pxEP->RegId, TX, DIS);

        if (USB_EP_DOUBLE_BUFFERED(pxEP))
        {
            /* Disable other half of EPnR as well */
            USB_TOGGLE_CLEAR(pxEP->RegId, DTOG_RX);
            USB_TOGGLE(pxEP->RegId, DTOG_RX);
            USB_EP_SET_STATUS(pxEP->RegId, RX, DIS);
        }
    }
    else
    {
        /* Configure DISABLE status for the Endpoint*/
        USB_TOGGLE_CLEAR(pxEP->RegId, DTOG_RX);
        USB_EP_SET_STATUS(pxEP->RegId, RX, DIS);

        if (USB_EP_DOUBLE_BUFFERED(pxEP))
        {
            /* Disable other half of EPnR as well */
            USB_TOGGLE_CLEAR(pxEP->RegId, DTOG_TX);
            USB_TOGGLE(pxEP->RegId, DTOG_TX);
            USB_EP_SET_STATUS(pxEP->RegId, TX, DIS);
        }
    }
}

/**
 * @brief Set a STALL condition on an endpoint (not supported for Isochronous).
 * @param pxUSB: pointer to the USB handle structure
 * @param ucEpAddress: endpoint address
 */
void USB_vEpSetStall(USB_HandleType * pxUSB, uint8_t ucEpAddress)
{
    USB_EndPointHandleType * pxEP = USB_GET_EP_AT(pxUSB, ucEpAddress);

    if (ucEpAddress > 0x7F)
    {
        USB_EP_SET_STATUS(pxEP->RegId, TX, STALL);
    }
    else
    {
        USB_EP_SET_STATUS(pxEP->RegId, RX, STALL);
    }
}

/**
 * @brief Clear a STALL condition on an endpoint.
 * @param pxUSB: pointer to the USB handle structure
 * @param ucEpAddress: endpoint address
 */
void USB_vEpClearStall(USB_HandleType * pxUSB, uint8_t ucEpAddress)
{
    USB_EndPointHandleType * pxEP = USB_GET_EP_AT(pxUSB, ucEpAddress);

    if (ucEpAddress > 0x7F)
    {
        USB_TOGGLE_CLEAR(pxEP->RegId, DTOG_TX);
        USB_EP_SET_STATUS(pxEP->RegId, TX, NAK);
    }
    else
    {
        USB_TOGGLE_CLEAR(pxEP->RegId, DTOG_RX);
        USB_EP_SET_STATUS(pxEP->RegId, RX, VALID);
    }
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
    USB_EndPointHandleType * pxEP = &pxUSB->EP.IN[ucEpAddress & 0xF];

    /* setup the transfer */
    pxEP->Transfer.Data       = (uint8_t*)pucData;
    pxEP->Transfer.Progress   = usLength;
    pxEP->Transfer.Length     = usLength;

    USB_prvTransmitPacket(pxUSB, pxEP);
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

    USB_prvReceivePacket(pxUSB, pxEP);
}

/**
 * @brief Activates remote wake-up signaling.
 * @param pxUSB: pointer to the USB handle structure
 */
void USB_vSetRemoteWakeup(USB_HandleType * pxUSB)
{
#ifdef USB_CNTR_L1RESUME
    if (pxUSB->LinkState == USB_LINK_STATE_SLEEP)
    {
        /* Send LPM L1 Resume signal - Cleared by hardware */
        USB_REG_BIT(pxUSB,CNTR,L1RESUME) = 1;
    }
    else
#endif
    if (pxUSB->LinkState == USB_LINK_STATE_SUSPEND)
    {
        /* Activate Resume signal
         * Wait 1 - 15 ms before disabling */
        USB_REG_BIT(pxUSB,CNTR,RESUME) = 1;
    }
}

/**
 * @brief Deactivates remote wake-up signaling.
 * @param pxUSB: pointer to the USB handle structure
 */
void USB_vClearRemoteWakeup(USB_HandleType * pxUSB)
{
    /* Deactivate Resume signal after 1 - 15 ms */
    USB_REG_BIT(pxUSB,CNTR,RESUME) = 0;
}

/**
 * @brief USB interrupt handler that provides event-driven peripheral management.
 *        and handle callbacks.
 * @param pxUSB: pointer to the USB handle structure
 */
void USB_vIRQHandler(USB_HandleType * pxUSB)
{
    uint16_t usISTR;

    /* loop while Endpoint interrupts are present */
    for (usISTR = USB->ISTR.w; (usISTR & USB_ISTR_CTR) != 0; usISTR = USB->ISTR.w)
    {
        /* Read highest priority endpoint number */
        uint16_t usEpId  = usISTR & USB_ISTR_EP_ID;
        uint16_t usEpReg = USB->EPR[usEpId].w;
        uint8_t  ucEpNum = usEpReg & USB_EPADDR_FIELD;
        uint16_t usDataCount;

        /* OUT data received */
        if ((usEpReg & USB_EP_CTR_RX) != 0)
        {
            USB_EndPointHandleType *pxEP = &pxUSB->EP.OUT[ucEpNum];

            /* Get SETUP Packet (EP0 only) */
            if ((usEpReg & USB_EP_SETUP) != 0)
            {
                uint8_t* pSetup = (uint8_t*)&pxUSB->Setup;

                /* Clear RX complete flag */
                USB_EP_FLAG_CLEAR(0, CTR_RX);

                USB_prvReadPMA(pSetup, USB_EP_BDT[usEpId].RX_ADDR,
                        sizeof(pxUSB->Setup));

                /* Process SETUP Packet */
                USB_vSetupCallback(pxUSB);
            }
            else
            {
                /* Get Data packet */
                uint16_t usPmaAddress = USB_EP_BDT[usEpId].RX_ADDR;
                usDataCount = USB_EP_BDT[usEpId].RX_COUNT & 0x3FF;

                /* Clear RX complete flag */
                USB_EP_FLAG_CLEAR(usEpId, CTR_RX);

                /* Double buffering */
                if (USB_EP_DOUBLE_BUFFERED(pxEP))
                {
                    if ((usEpReg & USB_EP_DTOG_RX) != 0)
                    {
                        /* read from endpoint buffer 0 */
                        usPmaAddress = USB_EP_BDT[usEpId].TX_ADDR;
                        usDataCount  = USB_EP_BDT[usEpId].TX_COUNT & 0x3FF;
                    }

                    /* Switch the reception buffer by toggling SW_BUF flag */
                    USB_TOGGLE(usEpId, DTOG_TX);
                }

                USB_prvReadPMA(pxEP->Transfer.Data, usPmaAddress, usDataCount);

                pxEP->Transfer.Length += usDataCount;
                pxEP->Transfer.Data += usDataCount;

                /* If the last packet of the data, transfer is complete
                 * TODO if Length % MaxPacketSize == 0 the transfer will hang without ZLP */
                if ((pxEP->Transfer.Progress == 0) ||
                    (usDataCount < pxEP->MaxPacketSize))
                {
                    /* Reception finished */
                    USB_vDataOutCallback(pxUSB, pxEP);

                    if (ucEpNum == 0)
                    {
                        /* Keep EP0 ready to receive next setup */
                        USB_EP_BDT[0].RX_COUNT =
                                USB_prvConvertRxCount(pxEP->MaxPacketSize);
                        USB_EP_SET_STATUS(0, RX, VALID);
                    }
                }
                else
                {
                    /* Continue data reception */
                    USB_prvReceivePacket(pxUSB, pxEP);
                }
            }
        }

        /* IN data sent */
        if ((usEpReg & USB_EP_CTR_TX) != 0)
        {
            USB_EndPointHandleType *pxEP = &pxUSB->EP.IN[ucEpNum];

            /* Clear TX complete flag */
            USB_EP_FLAG_CLEAR(usEpId, CTR_TX);

            /* Double buffering */
            if ((USB_EP_DOUBLE_BUFFERED(pxEP)) && ((usEpReg & USB_EP_DTOG_TX) == 0))
            {
                /* written from endpoint 1 buffer */
                usDataCount = USB_EP_BDT[usEpId].RX_COUNT & 0x3FF;
            }
            else
            {
                /* written from endpoint 0 (Tx) buffer */
                usDataCount = USB_EP_BDT[usEpId].TX_COUNT & 0x3FF;
            }
            pxEP->Transfer.Data += usDataCount;

            /* If the last packet of the data */
            if (pxEP->Transfer.Progress == 0)
            {
                /* Transmission complete */
                USB_vDataInCallback(pxUSB, pxEP);
            }
            else
            {
                /* Continue data transmission */
                USB_prvTransmitPacket(pxUSB, pxEP);
            }
        }
    }

    /* Handle device reset */
    if ((usISTR & USB_ISTR_RESET) != 0)
    {
        USB_FLAG_CLEAR(pxUSB, RESET);

        /* Clear any ongoing Remote Wakeup signaling */
        CLEAR_BIT(USB->CNTR.w, USB_CNTR_RESUME);
        pxUSB->LinkState = USB_LINK_STATE_ACTIVE;

        /* Set default address (0) */
        USB_vSetAddress(pxUSB, 0);

        /* Notify device handler */
        USB_vResetCallback(pxUSB, USB_SPEED_FULL);
    }

    /* Handle wakeup signal */
    if ((usISTR & USB_ISTR_WKUP) != 0)
    {
        /* Release low-power mode, clear any ongoing Remote Wakeup signaling */
        CLEAR_BIT(USB->CNTR.w, USB_CNTR_FSUSP | USB_CNTR_LPMODE | USB_CNTR_RESUME);

        USB_FLAG_CLEAR(pxUSB, WKUP);

        XPD_SAFE_CALLBACK(pxUSB->Callbacks.Resume, pxUSB);

        /* LPM state is changed after Resume callback
         * -> possible to determine exited suspend level */
        pxUSB->LinkState = USB_LINK_STATE_ACTIVE;
    }

#ifdef USB_ISTR_L1REQ
    /* Handle L1 suspend request */
    if ((usISTR & USB_ISTR_L1REQ) != 0)
    {
        USB_FLAG_CLEAR(pxUSB, L1REQ);

        /* Force suspend and low-power mode before going to L1 state */
        SET_BIT(USB->CNTR.w, USB_CNTR_FSUSP | USB_CNTR_LPMODE);

        /* Set the target Link State */
        pxUSB->LinkState = USB_LINK_STATE_SLEEP;
        XPD_SAFE_CALLBACK(pxUSB->Callbacks.Suspend, pxUSB);
    }
#endif

    /* Handle suspend request */
    if ((usISTR & USB_ISTR_SUSP) != 0)
    {
        USB_FLAG_CLEAR(pxUSB, SUSP);

        /* Force low-power mode in the macrocell */
        SET_BIT(USB->CNTR.w, USB_CNTR_FSUSP | USB_CNTR_LPMODE);

        if (USB_FLAG_STATUS(pxUSB, WKUP) == 0)
        {
            /* Set the target Link State */
            pxUSB->LinkState = USB_LINK_STATE_SUSPEND;
            XPD_SAFE_CALLBACK(pxUSB->Callbacks.Suspend, pxUSB);
        }
    }

    /* Handle Start Of Frame signal (if enabled) */
    if ((USB_REG_BIT(pxUSB,CNTR,SOFM) != 0) && ((usISTR & USB_ISTR_SOF) != 0))
    {
        USB_FLAG_CLEAR(pxUSB, SOF);
        XPD_SAFE_CALLBACK(pxUSB->Callbacks.SOF, pxUSB);
    }
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
#ifndef USB_BCDR_BCDEN
    USB_ChargerType eDetection = USB_BCD_NOT_SUPPORTED;
#else
    /* USB power is always available before the data lines are contacted */
    USB_ChargerType eDetection = USB_BCD_NO_DATA_CONTACT;

    /* Enable Battery Charger Detection */
    SET_BIT(USB->BCDR.w, USB_BCDR_BCDEN | USB_BCDR_DCDEN);
    {
        uint32_t ulTimeout = 1000; /* Adjust depending on the speed of plugging in the device */

        /* Data Contact Detect: determines contact of data lines to the bus powering entity */
        if (XPD_OK == XPD_eWaitForDiff((void*)&USB->BCDR.w, USB_BCDR_DCDET, 0, &ulTimeout))
        {
            /* Bus electric stabilization */
            XPD_vDelay_ms(300);

            /* Primary Detection:
             * distinguishes between a Standard Downstream Port and Charging Ports */
            USB->BCDR.w = (USB->BCDR.w & ~USB_BCDR_DCDEN) | USB_BCDR_PDEN;

            /* Bus electric stabilization */
            XPD_vDelay_ms(300);

            /* Check the results of PD */
            switch (USB->BCDR.w & (USB_BCDR_PS2DET | USB_BCDR_PDET))
            {
                case USB_BCDR_PS2DET:
                    /* D- is externally pulled high during Primary Detection
                     * PS2 port or proprietary charger */
                    eDetection = USB_BCD_PS2_PROPRIETARY_PORT;
                    break;

                case USB_BCDR_PDET:
                    /* Secondary Detection:
                     * distinguishes between Charging Downstream Port and Dedicated Charging Port */
                    USB->BCDR.w = (USB->BCDR.w & ~USB_BCDR_PDEN) | USB_BCDR_SDEN;

                    /* Bus electric stabilization */
                    XPD_vDelay_ms(300);

                    if (USB_REG_BIT(pxUSB,BCDR,SDET) != 0)
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

                default:
                    /* No downstream side support for PD
                     * Standard Downstream Port SDP */
                    eDetection = USB_BCD_STANDARD_DOWNSTREAM_PORT;
                    break;
            }
        }
    }
    /* Disable Battery Charger Detection */
    CLEAR_BIT(USB->BCDR.w, USB_BCDR_BCDEN | USB_BCDR_DCDEN | USB_BCDR_PDEN | USB_BCDR_SDEN);
#endif
    return eDetection;
}

/**
 * @brief Configure EPnR assignment and packet memory allocation for all endpoints
 *        based on the handle's Endpoint setup.
 * @param pxUSB: pointer to the USB handle structure
 */
__weak void USB_vAllocateEPs(USB_HandleType * pxUSB)
{
    XPD_ReturnType eResult = XPD_ERROR;
    USB_EndPointHandleType *pxEP, *pxEP2;
    uint32_t ulEpNum;
    uint16_t usPmaTail;
    uint8_t  ucRegId = 0;

    /* Init endpoints structures */
    for (ulEpNum = 0; ulEpNum < USBD_MAX_EP_COUNT; ulEpNum++)
    {
        pxEP = &pxUSB->EP.OUT[ulEpNum];
        pxEP2 = &pxUSB->EP.IN[ulEpNum];

        /* Only consider used EPs */
        if (pxEP->MaxPacketSize > 0)
        {
            pxEP->RegId = ucRegId++;

            if (pxEP2->MaxPacketSize > 0)
            {
                /* If IN-OUT endpoints with the same address and type
                 * are both single buffer, one EPnR can manage both */
                if (!USB_EP_DOUBLE_BUFFERED(pxEP) &&
                    !USB_EP_DOUBLE_BUFFERED(pxEP2) &&
                    (pxEP->Type == pxEP2->Type))
                {
                    pxEP2->RegId = pxEP->RegId;
                }
                else
                {
                    pxEP2->RegId = ucRegId++;
                }
            }
        }
        else if (pxEP2->MaxPacketSize > 0)
        {
            pxEP2->RegId = ucRegId++;
        }
    }

    /* Ensure that endpoints can be fitted in EP regs */
    if (ucRegId < USBD_MAX_EP_COUNT)
    {
        /* Reserve place for BTABLE */
        usPmaTail = ucRegId * sizeof(USB_BufferDescriptorType);

        /* EP0 is half-duplex, IN and OUT can share the memory */
        pxEP = &pxUSB->EP.IN[0];
        USB_EP_BDT[pxEP->RegId].TX_ADDR = usPmaTail;
        USB_EP_BDT[pxEP->RegId].RX_ADDR = usPmaTail;
        usPmaTail += (pxEP->MaxPacketSize + 1) & (~1);

        /* Allocate packet memory for all endpoints (unused ones' MPS = 0) */
        for (ulEpNum = 1; ulEpNum < USBD_MAX_EP_COUNT; ulEpNum++)
        {
            pxEP = &pxUSB->EP.IN[ulEpNum];
            if (pxEP->MaxPacketSize > 0)
            {
                /* The PMA allocation must be 16 bit aligned */
                uint16_t usMPS = (pxEP->MaxPacketSize + 1) & (~1);

                /* Set TX_ADDR or RX_ADDR depending on direction */
                USB_EP_BDT[pxEP->RegId].TX_ADDR = usPmaTail;
                usPmaTail += usMPS;

                /* Allocate double buffer */
                if (USB_EP_DOUBLE_BUFFERED(pxEP))
                {
                    /* Set RX_ADDR or TX_ADDR as well */
                    USB_EP_BDT[pxEP->RegId].RX_ADDR = usPmaTail;
                    usPmaTail += usMPS;
                }
            }

            pxEP = &pxUSB->EP.OUT[ulEpNum];
            if (pxEP->MaxPacketSize > 0)
            {
                /* The PMA allocation must be 16 bit aligned */
                uint16_t usMPS = (pxEP->MaxPacketSize + 1) & (~1);

                /* Set TX_ADDR or RX_ADDR depending on direction */
                USB_EP_BDT[pxEP->RegId].RX_ADDR = usPmaTail;
                usPmaTail += usMPS;

                /* Allocate double buffer */
                if (USB_EP_DOUBLE_BUFFERED(pxEP))
                {
                    /* Set RX_ADDR or TX_ADDR as well */
                    USB_EP_BDT[pxEP->RegId].TX_ADDR = usPmaTail;
                    usPmaTail += usMPS;
                }
            }
        }

#ifdef USB_LPMCSR_LPMEN
        /* TODO: usPmaTail shall not exceed 1024 (or 768 if CAN is enabled) */
#else
        /* TODO: usPmaTail shall not exceed 512 */
#endif
        {
            eResult = XPD_OK;
        }
    }
    else
    {
        /* If the EP needs were more than what can be provided
         * by the peripheral, return error */
        eResult = XPD_ERROR;
    }

    (void) eResult;
}

/** @} */

/** @} */

#endif /* defined(USB) */
