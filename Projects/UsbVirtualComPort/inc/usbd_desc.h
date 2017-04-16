/**
  ******************************************************************************
  * @file           : usbd_desc.h
  * @version        : v1.0_Cube
  * @brief          : Header for usbd_desc file.
  ******************************************************************************
  * COPYRIGHT(c) 2016 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  * 1. Redistributions of source code must retain the above copyright notice,
  * this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  * this list of conditions and the following disclaimer in the documentation
  * and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of its contributors
  * may be used to endorse or promote products derived from this software
  * without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
*/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __USBD_DESC__H__
#define __USBD_DESC__H__

#ifdef __cplusplus
 extern "C" {
#endif
/* Includes ------------------------------------------------------------------*/
#include "usbd_def.h"

/** @addtogroup STM32_USB_OTG_DEVICE_LIBRARY
  * @{
  */
  
/** @defgroup USB_DESC
  * @brief general defines for the usb device library file
  * @{
  */ 

/** @defgroup USB_DESC_Exported_Defines
  * @{
  */

/**
  * @}
  */ 

/** @defgroup USBD_DESC_Exported_TypesDefinitions
  * @{
  */

typedef struct
{
    uint8_t  bLength;               /*!< Size of the Descriptor in Bytes (18 bytes) */
    uint8_t  bDescriptorType;       /*!< Device Descriptor (0x01) */
    uint16_t bcdUSB;                /*!< USB Specification Number which device complies to */
    uint8_t  bDeviceClass;          /*!< Class Code (Assigned by USB Org)
                                        If equal to Zero, each interface specifies it’s own class code
                                        If equal to 0xFF, the class code is vendor specified.
                                        Otherwise field is valid Class Code. */
    uint8_t  bDeviceSubClass;       /*!< Subclass Code (Assigned by USB Org) */
    uint8_t  bDeviceProtocol;       /*!< Protocol Code (Assigned by USB Org) */
    uint8_t  bMaxPacketSize;        /*!< Maximum Packet Size for Zero Endpoint. Valid Sizes are 8, 16, 32, 64 */
    uint16_t idVendor;              /*!< Vendor ID (Assigned by USB Org) */
    uint16_t idProduct;             /*!< Product ID (Assigned by Manufacturer) */
    uint16_t bcdDevice;             /*!< Device Release Number */
    uint8_t  iManufacturer;         /*!< Index of Manufacturer String Descriptor */
    uint8_t  iProduct;              /*!< Index of Product String Descriptor */
    uint8_t  iSerialNumber;         /*!< Index of Serial Number String Descriptor */
    uint8_t  bNumConfigurations;    /*!< Number of Possible Configurations */
}USB_DeviceDescriptorType;

typedef struct
{
    uint8_t  bLength;               /*!< Size of Descriptor in Bytes */
    uint8_t  bDescriptorType;       /*!< Configuration Descriptor (0x02) */
    uint16_t wTotalLength;          /*!< Total length in bytes of data returned */
    uint8_t  bNumInterfaces;        /*!< Number of Interfaces */
    uint8_t  bConfigurationValue;   /*!< Value to use as an argument to select this configuration */
    uint8_t  iConfiguration;        /*!< Index of String Descriptor describing this configuration */
    uint8_t  bmAttributes;          /*!< 0b1[Self Powered][Remote Wakeup]00000 */
    uint8_t  bMaxPower;             /*!< Maximum Power Consumption in 2mA units */
}USB_ConfigDescType;

typedef struct
{
    uint8_t  bLength;           /*!< Size of Descriptor in Bytes */
    uint8_t  bDescriptorType;   /*!< String  Descriptor (0x03) */
    uint16_t wLANGID;           /*!< Supported Language Code Zero (e.g. 0x0409 English - United States) */
}USB_LangIDDescType;

typedef struct
{
    uint8_t  bLength;               /*!< Size of Descriptor in Bytes (9 Bytes) */
    uint8_t  bDescriptorType;       /*!< Interface Descriptor (0x04) */
    uint8_t  bInterfaceNumber;      /*!< Number of Interface */
    uint8_t  bAlternateSetting;     /*!< Value used to select alternative setting */
    uint8_t  bNumEndpoints;         /*!< Number of Endpoints used for this interface */
    uint8_t  bInterfaceClass;       /*!< Class Code (Assigned by USB Org) */
    uint8_t  bInterfaceSubClass;    /*!< Subclass Code (Assigned by USB Org) */
    uint8_t  bInterfaceProtocol;    /*!< Protocol Code (Assigned by USB Org) */
    uint8_t  iInterface;            /*!< Index of String Descriptor Describing this interface */
}USB_InterfaceDescType;

typedef struct
{
    uint8_t  bLength;           /*!< Size of Descriptor in Bytes (7 Bytes) */
    uint8_t  bDescriptorType;   /*!< Interface Descriptor (0x05) */
    uint8_t  bEndpointAddress;  /*!< Endpoint Address 0b[0=Out / 1=In]000[Endpoint Number] */
    uint8_t  bmAttributes;     /*!< Bits 0..1 Transfer Type
                                        00 = Control
                                        01 = Isochronous
                                        10 = Bulk
                                        11 = Interrupt
                                    Bits 2..7 are reserved. If Isochronous endpoint,
                                    Bits 3..2 = Synchronisation Type (Iso Mode)
                                        00 = No Synchonisation
                                        01 = Asynchronous
                                        10 = Adaptive
                                        11 = Synchronous
                                    Bits 5..4 = Usage Type (Iso Mode)
                                        00 = Data Endpoint
                                        01 = Feedback Endpoint
                                        10 = Explicit Feedback Data Endpoint
                                        11 = Reserved */
    uint16_t wMaxPacketSize;    /*!< Maximum Packet Size this endpoint is capable of sending or receiving */
    uint8_t  bInterval;         /*!< Interval for polling endpoint data transfers. Value in frame counts.
                                     Ignored for Bulk & Control Endpoints. Isochronous must equal 1 and
                                     field may range from 1 to 255 for interrupt endpoints. */
}USB_EndpointDescType;

typedef struct
{
    uint8_t  bLength;               /*!< Size of Descriptor in Bytes */
    uint8_t  bDescriptorType;       /*!< Device Qualifier Descriptor (0x06) */
    uint16_t bcdUSB;                /*!< USB Specification Number which device complies to */
    uint8_t  bDeviceClass;          /*!< Class Code (Assigned by USB Org)
                                        If equal to Zero, each interface specifies it’s own class code
                                        If equal to 0xFF, the class code is vendor specified.
                                        Otherwise field is valid Class Code. */
    uint8_t  bDeviceSubClass;       /*!< Subclass Code (Assigned by USB Org) */
    uint8_t  bDeviceProtocol;       /*!< Protocol Code (Assigned by USB Org) */
    uint8_t  bMaxPacketSize;        /*!< Maximum Packet Size for Zero Endpoint. Valid Sizes are 8, 16, 32, 64 */
    uint8_t  bNumConfigurations;    /*!< Number of Possible Configurations */
    uint8_t  bReserved;             /*!< Keep 0 */
}USB_DeviceQualifierDescType;

/**
  * @}
  */ 

/** @defgroup USBD_DESC_Exported_Macros
  * @{
  */ 
/**
  * @}
  */ 

/** @defgroup USBD_DESC_Exported_Variables
  * @{
  */ 
extern const USBD_DescriptorsTypeDef FS_Desc;
/**
  * @}
  */ 

/** @defgroup USBD_DESC_Exported_FunctionsPrototype
  * @{
  */ 
  
/**
  * @}
  */ 
#ifdef __cplusplus
}
#endif

#endif /* __USBD_DESC_H */

/**
  * @}
  */ 

/**
* @}
*/ 
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
