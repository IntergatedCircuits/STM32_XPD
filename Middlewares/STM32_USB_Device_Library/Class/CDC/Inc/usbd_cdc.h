/**
 ******************************************************************************
 * @file    usbd_cdc.h
 * @author  Benedek Kupper
 * @version V0.1
 * @date    2017-04-16
 * @brief   header file for the usbd_cdc.c file.
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
#ifndef __USB_CDC_H
#define __USB_CDC_H

#ifdef __cplusplus
extern "C"
{
#endif

/* Includes ------------------------------------------------------------------*/
#include  "usbd_ioreq.h"

/** @addtogroup STM32_USB_DEVICE_LIBRARY
 * @{
 */

/** @defgroup usbd_cdc
 * @brief This file is the Header file for usbd_cdc.c
 * @{
 */

/** @defgroup usbd_cdc_Exported_Defines
 * @{
 */
#define CDC_IN_EP                                   0x81  /* EP1 for data IN */
#define CDC_OUT_EP                                  0x01  /* EP1 for data OUT */
#define CDC_CMD_EP                                  0x82  /* EP2 for CDC commands */

/* CDC Endpoints parameters: you can fine tune these values depending on the needed baudrates and performance. */
#define CDC_DATA_HS_MAX_PACKET_SIZE                 512  /* Endpoint IN & OUT Packet size */
#define CDC_DATA_FS_MAX_PACKET_SIZE                 64  /* Endpoint IN & OUT Packet size */
#define CDC_CMD_PACKET_SIZE                         8  /* Control Endpoint Packet size */ 

#define USB_CDC_CONFIG_DESC_SIZ                     67

#define CDC_DATA_HS_IN_PACKET_SIZE                  CDC_DATA_HS_MAX_PACKET_SIZE
#define CDC_DATA_HS_OUT_PACKET_SIZE                 CDC_DATA_HS_MAX_PACKET_SIZE

#define CDC_DATA_FS_IN_PACKET_SIZE                  CDC_DATA_FS_MAX_PACKET_SIZE
#define CDC_DATA_FS_OUT_PACKET_SIZE                 CDC_DATA_FS_MAX_PACKET_SIZE

/*---------------------------------------------------------------------*/
/*  CDC definitions                                                    */
/*---------------------------------------------------------------------*/
#define CDC_SEND_ENCAPSULATED_COMMAND               0x00
#define CDC_GET_ENCAPSULATED_RESPONSE               0x01
#define CDC_SET_COMM_FEATURE                        0x02
#define CDC_GET_COMM_FEATURE                        0x03
#define CDC_CLEAR_COMM_FEATURE                      0x04
#define CDC_SET_LINE_CODING                         0x20
#define CDC_GET_LINE_CODING                         0x21
#define CDC_SET_CONTROL_LINE_STATE                  0x22
#define CDC_SEND_BREAK                              0x23

/**
 * @}
 */

/** @defgroup USBD_CORE_Exported_TypesDefinitions
 * @{
 */

/**
 * @}
 */

typedef struct
{
    uint8_t  bLength;               /*!< Size of Descriptor in Bytes */
    uint8_t  bDescriptorType;       /*!< CS_INTERFACE Descriptor (0x24) */
    uint8_t  bDescriptorSubtype;    /*!< Header Functional Descriptor (0x00) */
    uint16_t bcdCDC;                /*!< USB Specification Number which device complies to */
}USB_CS_HeaderFunctionalDescType;

typedef struct
{
    uint8_t  bFunctionLength;       /*!< Size of Function in Bytes */
    uint8_t  bDescriptorType;       /*!< CS_INTERFACE Descriptor (0x24) */
    uint8_t  bDescriptorSubtype;    /*!< Call Management Functional Descriptor (0x01) */
    uint8_t  bmCapabilities;        /*!<  */
    uint8_t  bDataInterface;        /*!<  */
}USB_CS_CallMgmtFunctionalDescType;

typedef struct
{
    uint8_t  bFunctionLength;       /*!< Size of Function in Bytes */
    uint8_t  bDescriptorType;       /*!< CS_INTERFACE Descriptor (0x24) */
    uint8_t  bDescriptorSubtype;    /*!< Abstract Control Management Descriptor (0x02) */
    uint8_t  bmCapabilities;        /*!<  */
}USB_CS_ACMFunctionalDescType;

typedef struct
{
    uint8_t  bFunctionLength;       /*!< Size of Function in Bytes */
    uint8_t  bDescriptorType;       /*!< CS_INTERFACE Descriptor (0x24) */
    uint8_t  bDescriptorSubtype;    /*!< Union Functional Descriptor (0x06) */
    uint8_t  bMasterInterface;      /*!< Communication class interface */
    uint8_t  bSlaveInterface0;      /*!< Data Class Interface */
}USB_CS_UnionFunctionalDescType;

typedef struct _USBD_CDC_Itf
{
    void (*Init)(void);
    void (*DeInit)(void);
    void (*Control)(uint8_t, uint8_t *, uint16_t);
    void (*Received)(uint8_t *, uint32_t);
    void (*Transmitted)(uint8_t *, uint32_t);
} USBD_CDC_ItfTypeDef;

typedef struct
{
#ifdef DEVICE_HS
    uint32_t data[CDC_DATA_HS_MAX_PACKET_SIZE / 4]; /* Force 32bits alignment */
#else
    uint32_t data[CDC_DATA_FS_MAX_PACKET_SIZE / 4]; /* Force 32bits alignment */
#endif
    uint8_t *TxBuffer;
    uint8_t *RxBuffer;
    uint16_t TxLength;
    uint16_t RxLength;
    uint8_t CmdOpCode;
    uint8_t CmdLength;
    USBD_StatusTypeDef TxState;
} USBD_CDC_HandleTypeDef;

/** @defgroup USBD_CORE_Exported_Macros
 * @{
 */

/**
 * @}
 */

/** @defgroup USBD_CORE_Exported_Variables
 * @{
 */

extern const USBD_ClassTypeDef USBD_CDC;
#define USBD_CDC_CLASS    &USBD_CDC
/**
 * @}
 */

/** @defgroup USB_CORE_Exported_Functions
 * @{
 */
uint8_t USBD_CDC_RegisterInterface(USBD_HandleTypeDef *pdev,
        const USBD_CDC_ItfTypeDef *fops);

uint8_t USBD_CDC_Transmit(USBD_HandleTypeDef *pdev, uint8_t *pbuff, uint16_t length);

uint8_t USBD_CDC_Receive(USBD_HandleTypeDef *pdev, uint8_t *pbuff, uint16_t length);
/**
 * @}
 */

#ifdef __cplusplus
}
#endif

#endif  /* __USB_CDC_H */
/**
 * @}
 */

/**
 * @}
 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
