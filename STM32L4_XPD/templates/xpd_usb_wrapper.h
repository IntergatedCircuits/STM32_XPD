/**
  ******************************************************************************
  * @file    xpd_usb_wrapper.h
  * @author  Benedek Kupper
  * @version 0.1
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
#ifndef __XPD_USB_WRAPPER_H_
#define __XPD_USB_WRAPPER_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <xpd_common.h>

#if defined(USB) || defined(USB_OTG_FS)
/** @defgroup USB
 * @{ */

#define USBD_EP0_MAX_PACKET_SIZE  64

#if   defined(USB)
#define USBD_MAX_EP_COUNT         8 /* Theoretical maximum */
#elif defined(USB_OTG_HS)
#define USBD_MAX_EP_COUNT         USB_OTG_HS_MAX_IN_ENDPOINTS
#elif defined(USB_OTG_FS)
#define USBD_MAX_EP_COUNT         USB_OTG_FS_MAX_IN_ENDPOINTS
#endif

/** @defgroup USB_Exported_Types USB Exported Types
 * @{ */

/** @brief USB device speed types */
typedef enum
{
    USB_SPEED_FULL = 0, /*!< Default USB device speed */
    USB_SPEED_HIGH = 1, /*!< High speed is only available with special PHY in HS core */
}USB_SpeedType;

/** @brief USB peripheral PHYsical layer selection */
typedef enum {
    USB_PHY_EMBEDDED_FS = 0, /*!< Full-Speed PHY embedded in chip */
    USB_PHY_ULPI        = 1, /*!< ULPI interface to external High-Speed PHY */
    USB_PHY_EMBEDDED_HS = 2, /*!< High-Speed PHY embedded in chip */
}USB_PHYType;

/** @brief Device Link Power Management (LPM) state */
typedef enum
{
    USB_LINK_STATE_OFF      = 3, /*!< Device disconnected from bus */
    USB_LINK_STATE_SUSPEND  = 2, /*!< Device suspended */
    USB_LINK_STATE_SLEEP    = 1, /*!< Device in L1 sleep mode */
    USB_LINK_STATE_ACTIVE   = 0, /*!< Device connected and active */
}USB_LinkStateType;

/** @brief USB configuration structure */
typedef struct
{
#if defined(USB_OTG_HS)
    USB_PHYType     PHY;    /*!< USB PHYsical layer selection */
#endif
#if defined(USB_OTG_GLPMCFG_LPMEN) || defined(USB_LPMCSR_LPMEN)
    FunctionalState LPM;    /*!< Link Power Management L1 sleep mode support */
#endif
#if defined(USB_OTG_GAHBCFG_DMAEN)
    FunctionalState DMA;    /*!< Use dedicated DMA for data transfer */
#endif
}USB_InitType;

/** @brief USB Endpoint types */
typedef enum
{
    USB_EP_TYPE_CONTROL     = 0, /*!< Control endpoint type */
    USB_EP_TYPE_ISOCHRONOUS = 1, /*!< Isochronous endpoint type */
    USB_EP_TYPE_BULK        = 2, /*!< Bulk endpoint type */
    USB_EP_TYPE_INTERRUPT   = 3  /*!< Interrupt endpoint type */
}USB_EndPointType;

/** @brief USB endpoint handle structure */
typedef struct
{
    struct {
        uint8_t *Data;                  /*!< Current data element of transfer */
        uint16_t Length;                /*!< Represents the actual transferred length */
        uint16_t Progress;              /*!< Progress of the transfer */
    }Transfer;                          /*!< Endpoint data transfer context */
    uint16_t            MaxPacketSize;  /*!< Endpoint Max packet size */
    USB_EndPointType    Type;           /*!< Endpoint type */
#ifdef USB
    uint8_t             RegId;          /*!< Endpoint register ID */
#endif
}USB_EndPointHandleType;

/** @brief USB Handle structure */
typedef struct
{
#ifdef USB_OTG_FS
    USB_OTG_TypeDef * Inst;                 /*!< The address of the peripheral instance used by the handle */
#endif
    struct {
        XPD_HandleCallbackType DepInit;     /*!< Initialize module dependencies */
        XPD_HandleCallbackType DepDeinit;   /*!< Restore module dependencies */
        XPD_HandleCallbackType Suspend;     /*!< Suspend request */
        XPD_HandleCallbackType Resume;      /*!< Resume request */
        XPD_HandleCallbackType SOF;         /*!< Start Of Frame */
#if !defined(USB_BCDR_DPPU) && !defined(USB_OTG_FS)
        XPD_CtrlCallbackType   ConnectCtrl; /*!< Callback to set USB device bus line connection state */
#endif
    }Callbacks;                                         /*   Handle Callbacks */
    uint8_t                     Setup[8];               /*!< Setup packet buffer */
    struct {
        USB_EndPointHandleType  IN[USBD_MAX_EP_COUNT];  /*!< IN endpoint status */
        USB_EndPointHandleType  OUT[USBD_MAX_EP_COUNT]; /*!< OUT endpoint status */
    }EP;                                                /*   Endpoint management */
    USB_SpeedType               Speed;                  /*!< Currently available speed */
    USB_LinkStateType           LinkState;              /*!< Device link status */
}USB_HandleType;

/** @} */

extern void     USB_vResetCallback      (USB_HandleType *pxUSB,
                                         USB_SpeedType eSpeed);

extern void     USB_vSetupCallback      (USB_HandleType *pxUSB);

extern void     USB_vDataInCallback     (USB_HandleType *pxUSB,
                                         USB_EndPointHandleType *pxEP);
extern void     USB_vDataOutCallback    (USB_HandleType *pxUSB,
                                         USB_EndPointHandleType *pxEP);

/** @} */

#endif /* defined(USB) || defined(USB_OTG_FS) */

#ifdef __cplusplus
}
#endif

#endif /* __XPD_USB_WRAPPER_H_ */
