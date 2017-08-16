/**
 ******************************************************************************
 * @file    usbd_cdc.c
 * @author  Benedek Kupper
 * @version V0.1
 * @date    2017-04-16
 * @brief   This file provides the high layer firmware functions to manage the
 *          following functionalities of the USB CDC Class:
 *           - Initialization and Configuration of high and low layer
 *           - Enumeration as CDC Device (and enumeration for each implemented memory interface)
 *           - OUT/IN data transfer
 *           - Command IN transfer (class requests management)
 *           - Error management
 *
 *  @verbatim
 *
 *          ===================================================================
 *                                CDC Class Driver Description
 *          ===================================================================
 *           This driver manages the "Universal Serial Bus Class Definitions for Communications Devices
 *           Revision 1.2 November 16, 2007" and the sub-protocol specification of "Universal Serial Bus
 *           Communications Class Subclass Specification for PSTN Devices Revision 1.2 February 9, 2007"
 *           This driver implements the following aspects of the specification:
 *             - Device descriptor management
 *             - Configuration descriptor management
 *             - Enumeration as CDC device with 2 data endpoints (IN and OUT) and 1 command endpoint (IN)
 *             - Requests management (as described in section 6.2 in specification)
 *             - Abstract Control Model compliant
 *             - Union Functional collection (using 1 IN endpoint for control)
 *             - Data interface class
 *
 *           These aspects may be enriched or modified for a specific user application.
 *
 *            This driver doesn't implement the following aspects of the specification
 *            (but it is possible to manage these features with some modifications on this driver):
 *             - Any class-specific aspect relative to communication classes should be managed by user application.
 *             - All communication classes other than PSTN are not managed
 *
 *  @endverbatim
 *
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; COPYRIGHT 2015 STMicroelectronics</center></h2>
 *
 * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
 * You may not use this file except in compliance with the License.
 * You may obtain a copy of the License at:
 *
 *        http://www.st.com/software_license_agreement_liberty_v2
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 ******************************************************************************
 */

#include "usbd_cdc.h"
#include "usbd_desc.h"
#include "usbd_ctlreq.h"
#include "usbd_conf.h"

/** @addtogroup STM32_USB_DEVICE_LIBRARY
 * @{ */

/** @defgroup USBD_CDC 
 * @brief USB Communication Device Class module
 * @{ */

static uint8_t USBD_CDC_Init(USBD_HandleTypeDef *pdev, uint8_t cfgidx);

static uint8_t USBD_CDC_DeInit(USBD_HandleTypeDef *pdev, uint8_t cfgidx);

static uint8_t USBD_CDC_Setup(USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req);

static uint8_t USBD_CDC_DataIn(USBD_HandleTypeDef *pdev, uint8_t epnum);

static uint8_t USBD_CDC_DataOut(USBD_HandleTypeDef *pdev, uint8_t epnum);

static uint8_t USBD_CDC_EP0_RxReady(USBD_HandleTypeDef *pdev);

static uint8_t *USBD_CDC_GetFSCfgDesc(uint16_t *length);

#ifdef DEVICE_HS
static uint8_t *USBD_CDC_GetHSCfgDesc(uint16_t *length);
#endif

static uint8_t *USBD_CDC_GetDeviceQualifierDescriptor(uint16_t *length);

/** @defgroup USBD_CDC_Private_Variables
 * @{ */

/* USB Standard Device Descriptor */
__ALIGN_BEGIN static const uint8_t USBD_CDC_DeviceQualifierDesc[USB_LEN_DEV_QUALIFIER_DESC] __ALIGN_END =
{
    USB_LEN_DEV_QUALIFIER_DESC,     /* bLength */
    USB_DESC_TYPE_DEVICE_QUALIFIER, /* bDescriptorType */
    0x00, 0x02,                     /* bcdUSB */
    0x02,                           /* bDeviceClass */
    0x00,                           /* bDeviceSubClass */
    0x00,                           /* bDeviceProtocol */
    0x40,                           /* bMaxPacketSize */
    0x01,                           /* bNumConfigurations */
    0x00,                           /* bReserved */
};

/* CDC interface class callbacks structure */
const USBD_ClassTypeDef USBD_CDC = {
    USBD_CDC_Init,
    USBD_CDC_DeInit,
    USBD_CDC_Setup,
    NULL, /* EP0_TxSent, */
    USBD_CDC_EP0_RxReady,
    USBD_CDC_DataIn,
    USBD_CDC_DataOut,
    NULL,
    NULL,
    NULL,
#ifdef DEVICE_HS
    USBD_CDC_GetHSCfgDesc,
#else
    NULL,
#endif
    USBD_CDC_GetFSCfgDesc,
    NULL, /* USBD_CDC_GetOtherSpeedCfgDesc, */
    USBD_CDC_GetDeviceQualifierDescriptor
};

#ifdef DEVICE_HS
/* USB CDC device Configuration Descriptor */
__ALIGN_BEGIN static const uint8_t USBD_CDC_CfgHSDesc[USB_CDC_CONFIG_DESC_SIZ] __ALIGN_END =
{
    /* Configuration Descriptor */
    0x09,                           /* bLength: Configuration Descriptor size */
    USB_DESC_TYPE_CONFIGURATION,    /* bDescriptorType: Configuration */
    USB_CDC_CONFIG_DESC_SIZ,0x00,   /* wTotalLength:no of returned bytes */
    0x02,                           /* bNumInterfaces: 2 interface */
    0x01,                           /* bConfigurationValue: Configuration value */
    0x00,                           /* iConfiguration: Index of string descriptor describing the configuration */
    0x80 | (USBD_SELF_POWERED << 6),/* bmAttributes: self powered */
    USBD_MAX_POWER_mA / 2,          /* MaxPower x mA */

    /* Interface Descriptor */
    0x09,                       /* bLength: Interface Descriptor size */
    USB_DESC_TYPE_INTERFACE,    /* bDescriptorType: Interface */
    /* Interface descriptor type */
    0x00,   /* bInterfaceNumber: Number of Interface */
    0x00,   /* bAlternateSetting: Alternate setting */
    0x01,   /* bNumEndpoints: One endpoints used */
    0x02,   /* bInterfaceClass: Communication Interface Class */
    0x02,   /* bInterfaceSubClass: Abstract Control Model */
    0x01,   /* bInterfaceProtocol: Common AT commands */
    0x00,   /* iInterface: */

    /* Header Functional Descriptor */
    0x05,   /* bLength: Endpoint Descriptor size */
    0x24,   /* bDescriptorType: CS_INTERFACE */
    0x00,   /* bDescriptorSubtype: Header Func Desc */
    0x10,   /* bcdCDC: spec release number */
    0x01,

    /* Call Management Functional Descriptor */
    0x05,   /* bFunctionLength */
    0x24,   /* bDescriptorType: CS_INTERFACE */
    0x01,   /* bDescriptorSubtype: Call Management Func Desc */
    0x00,   /* bmCapabilities: D0+D1 */
    0x01,   /* bDataInterface: 1 */

    /* ACM Functional Descriptor */
    0x04,   /* bFunctionLength */
    0x24,   /* bDescriptorType: CS_INTERFACE */
    0x02,   /* bDescriptorSubtype: Abstract Control Management desc */
    0x02,   /* bmCapabilities */

    /* Union Functional Descriptor */
    0x05,   /* bFunctionLength */
    0x24,   /* bDescriptorType: CS_INTERFACE */
    0x06,   /* bDescriptorSubtype: Union func desc */
    0x00,   /* bMasterInterface: Communication class interface */
    0x01,   /* bSlaveInterface0: Data Class Interface */

    /* Endpoint 2 Descriptor */
    0x07,                       /* bLength: Endpoint Descriptor size */
    USB_DESC_TYPE_ENDPOINT,     /* bDescriptorType: Endpoint */
    CDC_CMD_EP,                 /* bEndpointAddress */
    0x03,                       /* bmAttributes: Interrupt */
    LOBYTE(CDC_CMD_PACKET_SIZE),/* wMaxPacketSize: */
    HIBYTE(CDC_CMD_PACKET_SIZE),
#ifdef CDC_CMD_INTR_INTERVAL
    CDC_CMD_INTR_INTERVAL,      /* bInterval: */
#else
    0x80,                       /* bInterval: */
#endif /* CDC_CMD_INTR_INTERVAL */

    /* Data class interface descriptor */
    0x09,   /* bLength: Endpoint Descriptor size */
    USB_DESC_TYPE_INTERFACE,  /* bDescriptorType: */
    0x01,   /* bInterfaceNumber: Number of Interface */
    0x00,   /* bAlternateSetting: Alternate setting */
    0x02,   /* bNumEndpoints: Two endpoints used */
    0x0A,   /* bInterfaceClass: CDC */
    0x00,   /* bInterfaceSubClass: */
    0x00,   /* bInterfaceProtocol: */
    0x00,   /* iInterface: */

    /* Endpoint OUT Descriptor */
    0x07,   /* bLength: Endpoint Descriptor size */
    USB_DESC_TYPE_ENDPOINT,      /* bDescriptorType: Endpoint */
    CDC_OUT_EP,                        /* bEndpointAddress */
    0x02,                              /* bmAttributes: Bulk */
    LOBYTE(CDC_DATA_HS_MAX_PACKET_SIZE),  /* wMaxPacketSize: */
    HIBYTE(CDC_DATA_HS_MAX_PACKET_SIZE),
    0x00,                              /* bInterval: ignore for Bulk transfer */

    /* Endpoint IN Descriptor */
    0x07,   /* bLength: Endpoint Descriptor size */
    USB_DESC_TYPE_ENDPOINT,      /* bDescriptorType: Endpoint */
    CDC_IN_EP,                         /* bEndpointAddress */
    0x02,                              /* bmAttributes: Bulk */
    LOBYTE(CDC_DATA_HS_MAX_PACKET_SIZE),  /* wMaxPacketSize: */
    HIBYTE(CDC_DATA_HS_MAX_PACKET_SIZE),
    0x00                               /* bInterval: ignore for Bulk transfer */
};
#endif

/* USB CDC device Configuration Descriptor */
__ALIGN_BEGIN static const uint8_t USBD_CDC_CfgFSDesc[USB_CDC_CONFIG_DESC_SIZ] __ALIGN_END =
{
    /* Configuration Descriptor */
    0x09,                           /* bLength: Configuration Descriptor size */
    USB_DESC_TYPE_CONFIGURATION,    /* bDescriptorType: Configuration */
    USB_CDC_CONFIG_DESC_SIZ,0x00,   /* wTotalLength:no of returned bytes */
    0x02,                           /* bNumInterfaces: 2 interface */
    0x01,                           /* bConfigurationValue: Configuration value */
    0x00,                           /* iConfiguration: Index of string descriptor describing the configuration */
    0x80 | (USBD_SELF_POWERED << 6),/* bmAttributes: self powered */
    USBD_MAX_POWER_mA / 2,          /* MaxPower x mA */

    /* Interface Descriptor */
    0x09,                       /* bLength: Interface Descriptor size */
    USB_DESC_TYPE_INTERFACE,    /* bDescriptorType: Interface */
    /* Interface descriptor type */
    0x00,   /* bInterfaceNumber: Number of Interface */
    0x00,   /* bAlternateSetting: Alternate setting */
    0x01,   /* bNumEndpoints: One endpoints used */
    0x02,   /* bInterfaceClass: Communication Interface Class */
    0x02,   /* bInterfaceSubClass: Abstract Control Model */
    0x01,   /* bInterfaceProtocol: Common AT commands */
    0x00,   /* iInterface: */

    /* Header Functional Descriptor */
    0x05,   /* bLength: Endpoint Descriptor size */
    0x24,   /* bDescriptorType: CS_INTERFACE */
    0x00,   /* bDescriptorSubtype: Header Func Desc */
    0x10,   /* bcdCDC: spec release number */
    0x01,

    /* Call Management Functional Descriptor */
    0x05,   /* bFunctionLength */
    0x24,   /* bDescriptorType: CS_INTERFACE */
    0x01,   /* bDescriptorSubtype: Call Management Func Desc */
    0x00,   /* bmCapabilities: D0+D1 */
    0x01,   /* bDataInterface: 1 */

    /* ACM Functional Descriptor */
    0x04,   /* bFunctionLength */
    0x24,   /* bDescriptorType: CS_INTERFACE */
    0x02,   /* bDescriptorSubtype: Abstract Control Management desc */
    0x02,   /* bmCapabilities */

    /* Union Functional Descriptor */
    0x05,   /* bFunctionLength */
    0x24,   /* bDescriptorType: CS_INTERFACE */
    0x06,   /* bDescriptorSubtype: Union func desc */
    0x00,   /* bMasterInterface: Communication class interface */
    0x01,   /* bSlaveInterface0: Data Class Interface */

    /* Endpoint 2 Descriptor */
    0x07,                       /* bLength: Endpoint Descriptor size */
    USB_DESC_TYPE_ENDPOINT,     /* bDescriptorType: Endpoint */
    CDC_CMD_EP,                 /* bEndpointAddress */
    0x03,                       /* bmAttributes: Interrupt */
    LOBYTE(CDC_CMD_PACKET_SIZE),/* wMaxPacketSize: */
    HIBYTE(CDC_CMD_PACKET_SIZE),
#ifdef CDC_CMD_INTR_INTERVAL
    CDC_CMD_INTR_INTERVAL,      /* bInterval: */
#else
    0x80,                       /* bInterval: */
#endif /* CDC_CMD_INTR_INTERVAL */

    /* Data class interface descriptor */
    0x09,                   /* bLength: Endpoint Descriptor size */
    USB_DESC_TYPE_INTERFACE,/* bDescriptorType: */
    0x01,                   /* bInterfaceNumber: Number of Interface */
    0x00,                   /* bAlternateSetting: Alternate setting */
    0x02,                   /* bNumEndpoints: Two endpoints used */
    0x0A,                   /* bInterfaceClass: CDC */
    0x00,                   /* bInterfaceSubClass: */
    0x00,                   /* bInterfaceProtocol: */
    0x00,                   /* iInterface: */

    /* Endpoint OUT Descripto r*/
    0x07,                               /* bLength: Endpoint Descriptor size */
    USB_DESC_TYPE_ENDPOINT,             /* bDescriptorType: Endpoint */
    CDC_OUT_EP,                         /* bEndpointAddress */
    0x02,                               /* bmAttributes: Bulk */
    LOBYTE(CDC_DATA_FS_MAX_PACKET_SIZE),/* wMaxPacketSize: */
    HIBYTE(CDC_DATA_FS_MAX_PACKET_SIZE),
    0x00,                               /* bInterval: ignore for Bulk transfer */

    /* Endpoint IN Descriptor */
    0x07,                               /* bLength: Endpoint Descriptor size */
    USB_DESC_TYPE_ENDPOINT,             /* bDescriptorType: Endpoint */
    CDC_IN_EP,                          /* bEndpointAddress */
    0x02,                               /* bmAttributes: Bulk */
    LOBYTE(CDC_DATA_FS_MAX_PACKET_SIZE),/* wMaxPacketSize: */
    HIBYTE(CDC_DATA_FS_MAX_PACKET_SIZE),
    0x00                                /* bInterval: ignore for Bulk transfer */
};

/** @} */

/** @defgroup USBD_CDC_Private_Functions
 * @{ */

/**
 * @brief  (Re)Initialize the CDC interface
 * @param  pdev: device instance
 * @param  cfgidx: Configuration index
 * @retval status
 */
static uint8_t USBD_CDC_Init(USBD_HandleTypeDef *pdev, uint8_t cfgidx)
{
    uint16_t in_packet_size, out_packet_size;

#ifdef DEVICE_HS
    if (pdev->dev_speed == USBD_SPEED_HIGH)
    {
        in_packet_size  = CDC_DATA_HS_IN_PACKET_SIZE;
        out_packet_size = CDC_DATA_HS_OUT_PACKET_SIZE;
    }
    else
#endif
    {
        in_packet_size  = CDC_DATA_FS_IN_PACKET_SIZE;
        out_packet_size = CDC_DATA_FS_OUT_PACKET_SIZE;
    }

    /* Open EP IN */
    USBD_LL_OpenEP(pdev,
        CDC_IN_EP,
        USBD_EP_TYPE_BULK,
        in_packet_size);

    /* Open EP OUT */
    USBD_LL_OpenEP(pdev,
        CDC_OUT_EP,
        USBD_EP_TYPE_BULK,
        out_packet_size);

#if (CDC_AT_COMMAND_SUPPORT == 1)
    /* Open Command IN EP */
    USBD_LL_OpenEP(pdev,
        CDC_CMD_EP,
        USBD_EP_TYPE_INTR,
        CDC_CMD_PACKET_SIZE);
#endif

    pdev->pClassData = USBD_malloc(sizeof(USBD_CDC_HandleTypeDef));

    if (pdev->pClassData != NULL)
    {
        USBD_CDC_HandleTypeDef *hcdc = (USBD_CDC_HandleTypeDef*) pdev->pClassData;
        USBD_CDC_ItfTypeDef *itf     = (USBD_CDC_ItfTypeDef*) pdev->pUserData;

        /* Initialize transfer states */
        hcdc->TxLength = 0;

        /* Initialize CDC Interface components */
        if (itf->Init != NULL)
        {
            itf->Init();
        }
    }

    return USBD_OK;
}

/**
 * @brief  Deinitialize the CDC interface
 * @param  pdev: device instance
 * @param  cfgidx: Configuration index
 * @retval status
 */
static uint8_t USBD_CDC_DeInit(USBD_HandleTypeDef *pdev, uint8_t cfgidx)
{
    /* Close EP IN */
    USBD_LL_CloseEP(pdev,
    CDC_IN_EP);

    /* Close EP OUT */
    USBD_LL_CloseEP(pdev,
    CDC_OUT_EP);

#if (CDC_AT_COMMAND_SUPPORT == 1)
    /* Close Command IN EP */
    USBD_LL_CloseEP(pdev,
    CDC_CMD_EP);
#endif

    /* DeInit CDC Interface components */
    if (pdev->pClassData != NULL)
    {
        USBD_CDC_ItfTypeDef *itf     = (USBD_CDC_ItfTypeDef*) pdev->pUserData;

        if (itf->DeInit != NULL)
        {
            itf->DeInit();
        }

        USBD_free(pdev->pClassData);
        pdev->pClassData = NULL;
    }

    return USBD_OK;
}

/**
 * @brief  Handle the CDC specific commands
 * @param  pdev: instance
 * @param  req: CDC command request
 * @retval status
 */
static uint8_t USBD_CDC_Setup(USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req)
{
    USBD_CDC_HandleTypeDef *hcdc = (USBD_CDC_HandleTypeDef*) pdev->pClassData;
    USBD_CDC_ItfTypeDef *itf     = (USBD_CDC_ItfTypeDef*) pdev->pUserData;

    switch (req->bmRequest & USB_REQ_TYPE_MASK)
    {
    case USB_REQ_TYPE_CLASS:
        if (req->wLength)
        {
            if (req->bmRequest & 0x80)
            {
                if (itf->Control != NULL)
                {
                    itf->Control(req->bRequest, (uint8_t *) hcdc->data, req->wLength);

                    USBD_CtlSendData(pdev, (uint8_t *) hcdc->data, req->wLength);
                }
            }
            else
            {
                hcdc->CmdOpCode = req->bRequest;
                hcdc->CmdLength = req->wLength;

                USBD_CtlPrepareRx(pdev, (uint8_t *) hcdc->data, req->wLength);
            }
        }
        else if (itf->Control != NULL)
        {
            itf->Control(req->bRequest, (uint8_t*) req, 0);
        }
        break;

    case USB_REQ_TYPE_STANDARD:
        switch (req->bRequest)
        {
        case USB_REQ_GET_INTERFACE:
            {
                uint8_t ifalt = 0;
                USBD_CtlSendData(pdev, &ifalt, 1);
            }
            break;

        case USB_REQ_SET_INTERFACE:
            break;
        }
        break;

    default:
        break;
    }
    return USBD_OK;
}

/**
 * @brief  Data sent on non-control IN endpoint
 * @param  pdev: device instance
 * @param  epnum: endpoint number
 * @retval status
 */
static uint8_t USBD_CDC_DataIn(USBD_HandleTypeDef *pdev, uint8_t epnum)
{
    USBD_CDC_HandleTypeDef *hcdc = (USBD_CDC_HandleTypeDef*) pdev->pClassData;
    USBD_CDC_ItfTypeDef *itf     = (USBD_CDC_ItfTypeDef*) pdev->pUserData;

    if (hcdc != NULL)
    {
        /* If there has been a valid transmit request */
        if (hcdc->TxLength > 0)
        {
            /* Set the transmit state to: free to transmit */
            uint16_t txLen = hcdc->TxLength;
            hcdc->TxLength = 0;

            if (itf->Transmitted != NULL)
            {
                /* Provide callback on successful transmission */
                itf->Transmitted(hcdc->TxBuffer, txLen);
            }
        }
    }

    return USBD_OK;
}

/**
 * @brief  Data received on non-control OUT endpoint
 * @param  pdev: device instance
 * @param  epnum: endpoint number
 * @retval status
 */
static uint8_t USBD_CDC_DataOut(USBD_HandleTypeDef *pdev, uint8_t epnum)
{
    USBD_CDC_HandleTypeDef *hcdc = (USBD_CDC_HandleTypeDef*) pdev->pClassData;
    USBD_CDC_ItfTypeDef *itf     = (USBD_CDC_ItfTypeDef*) pdev->pUserData;

    /* USB data will be immediately processed, this allows next USB traffic being
     NAKed till the end of the application transfer */
    if ((itf->Received != NULL) && (hcdc != NULL))
    {
        /* Provide callback on successful reception */
        itf->Received(hcdc->RxBuffer, USBD_LL_GetRxDataSize(pdev, epnum));
    }

    return USBD_OK;
}

/**
 * @brief  Setup endpoint data processing
 * @param  pdev: device instance
 * @retval status
 */
static uint8_t USBD_CDC_EP0_RxReady(USBD_HandleTypeDef *pdev)
{
    USBD_CDC_HandleTypeDef *hcdc = (USBD_CDC_HandleTypeDef*) pdev->pClassData;
    USBD_CDC_ItfTypeDef *itf     = (USBD_CDC_ItfTypeDef*) pdev->pUserData;

    if ((itf->Control != NULL) && (hcdc != NULL) && (hcdc->CmdOpCode != 0xFF))
    {
        /* Provide callback to CMD handler */
        itf->Control(hcdc->CmdOpCode, (uint8_t *) hcdc->data, hcdc->CmdLength);
        hcdc->CmdOpCode = 0xFF;
    }

    return USBD_OK;
}

/**
 * @brief  Returns the configuration descriptor
 * @param  length: pointer to the data length
 * @retval pointer to descriptor buffer
 */
static uint8_t *USBD_CDC_GetFSCfgDesc(uint16_t *length)
{
    *length = sizeof(USBD_CDC_CfgFSDesc);
    return USBD_CDC_CfgFSDesc;
}

#ifdef DEVICE_HS
/**
 * @brief  USBD_CDC_GetHSCfgDesc
 *         Return configuration descriptor
 * @param  length : pointer data length
 * @retval pointer to descriptor buffer
 */
static uint8_t *USBD_CDC_GetHSCfgDesc(uint16_t *length)
{
    *length = sizeof(USBD_CDC_CfgHSDesc);
    return USBD_CDC_CfgHSDesc;
}
#endif

/**
 * @brief  Returns the Device Qualifier descriptor
 * @param  length: pointer to the data length
 * @retval pointer to descriptor buffer
 */
static uint8_t *USBD_CDC_GetDeviceQualifierDescriptor(uint16_t *length)
{
    *length = sizeof(USBD_CDC_DeviceQualifierDesc);
    return USBD_CDC_DeviceQualifierDesc;
}

/**
 * @brief  Sets the CDC user interface to the handler
 * @param  pdev: device instance
 * @param  fops: CDC Interface callback
 * @retval status
 */
uint8_t USBD_CDC_RegisterInterface(USBD_HandleTypeDef *pdev, const USBD_CDC_ItfTypeDef *fops)
{
    uint8_t ret = USBD_FAIL;

    if (fops != NULL)
    {
        pdev->pUserData = fops;
        ret = USBD_OK;
    }

    return ret;
}

/**
 * @brief  Transmits user data through the CDC IN endpoint
 * @param  pdev: device instance
 * @param  pbuff: Tx Buffer
 * @param  length: Tx data length
 * @retval USBD_BUSY if IN transfer is already ongoing, otherwise USBD_OK
 */
uint8_t USBD_CDC_Transmit(USBD_HandleTypeDef *pdev, uint8_t *pbuff, uint16_t length)
{
    USBD_CDC_HandleTypeDef *hcdc = (USBD_CDC_HandleTypeDef*) pdev->pClassData;
    uint8_t retval = USBD_FAIL;

    if (hcdc != NULL)
    {
        retval = USBD_BUSY;

        /* If the transmit state is: free to transmit */
        if (hcdc->TxLength == 0)
        {
            /* Tx Transfer in progress */
            hcdc->TxBuffer = pbuff;
            hcdc->TxLength = length;

            /* Transmit next packet */
            retval = USBD_LL_Transmit(pdev, CDC_IN_EP, pbuff, length);
        }
    }
    return retval;
}

/**
 * @brief  Initiates reception of user data through the CDC OUT endpoint
 * @param  pdev: device instance
 * @param  pbuff: Rx Buffer
 * @param  length: Rx data length
 * @retval status
 */
uint8_t USBD_CDC_Receive(USBD_HandleTypeDef *pdev, uint8_t *pbuff, uint16_t length)
{
    USBD_CDC_HandleTypeDef *hcdc = (USBD_CDC_HandleTypeDef*) pdev->pClassData;
    uint8_t retval = USBD_FAIL;

    if (hcdc != NULL)
    {
        hcdc->RxBuffer = pbuff;

        /* Prepare Out endpoint to receive next packet */
        retval = USBD_LL_PrepareReceive(pdev, CDC_OUT_EP, hcdc->RxBuffer, length);
    }
    return retval;
}

/**
 * @brief  Sets the input as command data on the CMD IN endpoint
 * @param  pdev: device instance
 * @param  pbuff: Tx Buffer
 * @retval status
 */
uint8_t USBD_CDC_SendCommand(USBD_HandleTypeDef *pdev, uint8_t *pbuff)
{
    USBD_CDC_HandleTypeDef *hcdc = (USBD_CDC_HandleTypeDef*) pdev->pClassData;
    uint8_t retval = USBD_FAIL;

#if (CDC_AT_COMMAND_SUPPORT == 1)
    if (hcdc != NULL)
    {
        /* Transmit command packet */
        retval = USBD_LL_Transmit(pdev, CDC_CMD_EP, pbuff, CDC_CMD_PACKET_SIZE);
    }
#endif
    return retval;
}

/** @} */

/** @} */

/** @} */
