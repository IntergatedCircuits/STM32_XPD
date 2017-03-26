/**
  ******************************************************************************
  * @file    xpd_usb.h
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
#ifndef XPD_USB_H_
#define XPD_USB_H_

#include "xpd_common.h"
#include "xpd_config.h"

#ifdef USB
/** @defgroup USB
 * @{ */

/** @defgroup USB_Exported_Types USB Exported Types
 * @{ */

/** @brief USB configuration structure */
typedef struct
{
    FunctionalState SOF;                /*!< StartOfFrame signal output activation */
    FunctionalState LowPowerMode;       /*!< Low power mode activation */
    FunctionalState LinkPowerMgmt;      /*!< Link Power Management activation */
}USB_InitType;

/** @brief USB Endpoint types */
typedef enum
{
    USB_EP_TYPE_CONTROL     = 0, /*!< Control endpoint type */
    USB_EP_TYPE_ISOCHRONOUS = 1, /*!< Isochronous endpoint type */
    USB_EP_TYPE_BULK        = 2, /*!< Bulk endpoint type */
    USB_EP_TYPE_INTERRUPT   = 3  /*!< Interrupt endpoint type */
}USB_EP_PacketType;

/** @brief USB Endpoint management structure */
typedef struct
{
    uint32_t            MaxPacketSize;  /*!< Endpoint Max packet size [0..65536] */
    boolean_t           Stalled;        /*!< Endpoint stall status */
    USB_EP_PacketType   Type;           /*!< Endpoint type */
    uint16_t            PMAAddress0;    /*!< PMA Address [0..1024] */
    uint16_t            PMAAddress1;    /*!< Secondary PMA Address (for double buffering) [0..1024] */
    FunctionalState     DoubleBuffer;   /*!< Double buffer configuration */
    DataStreamType      Transfer;       /*!< Endpoint data transfer context */
}USB_EndPointType;

/** @brief USB Handle structure */
typedef struct
{
    void *                      User;                           /*!< Pointer to upper stack handler */
    struct {
#ifndef USB_BCDR_DPPU
        XPD_CtrlFnType ConnectionStateCtrl; /*!< Callback to set USB device bus line connection state */
#endif
        int (*SetupStage)       (void *, uint8_t *);            /*!< SETUP packet received */
        int (*DataOutStage)     (void *, uint8_t, uint8_t *);   /*!< OUT data received */
        int (*DataInStage)      (void *, uint8_t, uint8_t *);   /*!< IN data transmitted */
        int (*SOF)              (void *);                       /*!< StartOfFrame signal */
        int (*Reset)            (void *);                       /*!< Device reset */
        int (*Suspend)          (void *);                       /*!< Suspend request */
        int (*Resume)           (void *);                       /*!< Resume request */
        int (*Connected)        (void *);                       /*!< Device connected to bus */
        int (*Disconnected)     (void *);                       /*!< Device disconnected from bus */
        int (*ISOOUTIncomplete) (void *, uint8_t);              /*!< Isochronous OUT transfer complete */
        int (*ISOINIncomplete)  (void *, uint8_t);              /*!< Isochronous IN transfer complete */
    }Callbacks;                                                 /*   Handle Callbacks */
    uint32_t                    Setup[12];                      /*!< Setup packet buffer */
    struct {
        USB_EndPointType        IN[8];                          /*!< IN endpoint status */
        USB_EndPointType        OUT[8];                         /*!< OUT endpoint status */
    }EP;                                                        /*   Endpoint management */
    volatile uint8_t            DeviceAddress;                  /*!< USB Address */
}USB_HandleType;


/** @} */

/** @defgroup USB_Exported_Macros USB Exported Macros
 * @{ */

/**
 * @brief  USB Handle initializer macro
 * @param  INSTANCE: specifies the USB peripheral instance.
 */
#define         NEW_USB_HANDLE(INSTANCE)    \
     {.Callbacks = {NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL}}

#ifdef USB_BB
#define USB_REG_BIT(_REG_NAME_, _BIT_NAME_) (USB_BB->_REG_NAME_._BIT_NAME_)
#else
#define USB_REG_BIT(_REG_NAME_, _BIT_NAME_) (USB->_REG_NAME_.b._BIT_NAME_)
#endif /* USB_BB */

#define         XPD_USB_ClearFlag(FLAG_NAME) \
    (USB_REG_BIT(ISTR,FLAG_NAME) = 0)

#define         XPD_USB_GetFlag(FLAG_NAME) \
    (USB_REG_BIT(ISTR,FLAG_NAME))

/** @brief USB Wake up line number */
#define USB_WAKEUP_EXTI_LINE            18

/** @} */

/** @addtogroup USB_Exported_Functions
 * @{ */
XPD_ReturnType  XPD_USB_Init                    (USB_HandleType * husb, USB_InitType * Config);
XPD_ReturnType  XPD_USB_Deinit                  (USB_HandleType * husb);
void            XPD_USB_Start                   (USB_HandleType * husb);
void            XPD_USB_Stop                    (USB_HandleType * husb);
void            XPD_USB_DevConnect              (USB_HandleType * husb);
void            XPD_USB_DevDisconnect           (USB_HandleType * husb);

void            XPD_USB_ActivateRemoteWakeup    (USB_HandleType * husb);
void            XPD_USB_DeActivateRemoteWakeup  (USB_HandleType * husb);

void            XPD_USB_SetAddress              (USB_HandleType * husb, uint8_t Address);

void            XPD_USB_EP_Open                 (USB_HandleType * husb, uint8_t EpNumber,
                                                 USB_EP_PacketType Type, uint16_t MaxPacketSize);
void            XPD_USB_EP_Close                (USB_HandleType * husb, uint8_t EpNumber);
void            XPD_USB_PMAConfig               (USB_HandleType * husb, uint8_t EpNumber,
                                                 uint32_t PmaAdress, FunctionalState DoubleBuffer);

void            XPD_USB_EP_Receive              (USB_HandleType * husb, uint8_t EpNumber,
                                                 uint8_t * Data, uint16_t Length);
uint16_t        XPD_USB_EP_GetRxCount           (USB_HandleType * husb, uint8_t EpNumber);

void            XPD_USB_EP_Transmit             (USB_HandleType * husb, uint8_t EpNumber,
                                                 uint8_t * Data, uint16_t Length);

void            XPD_USB_EP_SetStall             (USB_HandleType * husb, uint8_t EpNumber);
void            XPD_USB_EP_ClearStall           (USB_HandleType * husb, uint8_t EpNumber);

void            XPD_USB_EP_Flush                (USB_HandleType * husb, uint8_t EpNumber);


void            XPD_USB_IRQHandler              (USB_HandleType * husb);
/** @} */

/** @} */

#define XPD_USB_API
#include "xpd_rcc_pc.h"
#include "xpd_syscfg.h"
#undef XPD_USB_API

#endif /* USB */

#endif /* XPD_USB_H_ */
