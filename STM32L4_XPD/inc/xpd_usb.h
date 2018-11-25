/**
  ******************************************************************************
  * @file    xpd_usb.h
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
#ifndef __XPD_USB_H_
#define __XPD_USB_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <xpd_common.h>

#if   defined(USB_OTG_FS)
#include <xpd_usb_otg.h>

#define USB_vInit           USB_vDevInit
#define USB_vDeinit         USB_vDevDeinit
#define USB_vStart_IT       USB_vDevStart_IT
#define USB_vStop_IT        USB_vDevStop_IT
#define USB_vIRQHandler     USB_vDevIRQHandler

#elif defined(USB)
#include <xpd_usb_wrapper.h>

/** @defgroup USB
 * @{ */

/** @defgroup USB_Exported_Types USB Exported Types
 * @{ */

/** @brief USB Power Source types */
typedef enum
{
    USB_BCD_NO_DATA_CONTACT          = 0,   /*!< Data lines are floating on the device side */
    USB_BCD_STANDARD_DOWNSTREAM_PORT = 1,   /*!< Standard Downstream Port */
    USB_BCD_CHARGING_DOWNSTREAM_PORT = 2,   /*!< Charging Downstream Port detected */
    USB_BCD_DEDICATED_CHARGING_PORT  = 3,   /*!< Dedicated Charging Port detected */
    USB_BCD_PS2_PROPRIETARY_PORT     = 4,   /*!< PS2 or proprietary charging port detected */
    USB_BCD_NOT_SUPPORTED            = 0xFF /*!< Battery Charge Detection is not supported on the device */
}USB_ChargerType;

/** @defgroup USB_Exported_Macros USB Exported Macros
 * @{ */

/**
 * @brief USB Instance to handle binder macro
 * @param HANDLE: specifies the peripheral handle.
 * @param INSTANCE: specifies the USB peripheral instance.
 */
#define         USB_INST2HANDLE(HANDLE,INSTANCE)    ((void)INSTANCE)

#ifdef USB_BB
/**
 * @brief USB register bit accessing macro
 * @param HANDLE: specifies the peripheral handle.
 * @param REG_NAME: specifies the register name.
 * @param BIT_NAME: specifies the register bit name.
 */
#define         USB_REG_BIT(HANDLE, REG_NAME, BIT_NAME) \
    (USB_BB->REG_NAME.BIT_NAME)

#else
/**
 * @brief USB register bit accessing macro
 * @param HANDLE: specifies the peripheral handle.
 * @param REG_NAME: specifies the register name.
 * @param BIT_NAME: specifies the register bit name.
 */
#define         USB_REG_BIT(HANDLE, REG_NAME, BIT_NAME) \
    (USB->REG_NAME.b.BIT_NAME)

#endif /* USB_BB */

/**
 * @brief  Enable the specified USB interrupt.
 * @param  HANDLE: specifies the USB Handle.
 * @param  IT_NAME: specifies the interrupt to enable.
 *         This parameter can be one of the following values:
 *            @arg ESOF:    Expected start of frame
 *            @arg SOF:     Start of frame
 *            @arg RESET:   Reset
 *            @arg SUSP:    Suspend
 *            @arg WKUP:    Wake up
 *            @arg L1REQ:   L1 sleep mode request
 *            @arg ERR:     Error
 *            @arg PMAOVR:  Packet memory overrun
 *            @arg CTR:     Correct transfer
 */
#define         USB_IT_ENABLE( HANDLE, IT_NAME)         \
    (USB_REG_BIT(HANDLE,CNTR,IT_NAME##M) = 1)

/**
 * @brief  Disable the specified USB interrupt.
 * @param  HANDLE: specifies the USB Handle.
 * @param  IT_NAME: specifies the interrupt to disable.
 *         This parameter can be one of the following values:
 *            @arg ESOF:    Expected start of frame
 *            @arg SOF:     Start of frame
 *            @arg RESET:   Reset
 *            @arg SUSP:    Suspend
 *            @arg WKUP:    Wake up
 *            @arg L1REQ:   L1 sleep mode request
 *            @arg ERR:     Error
 *            @arg PMAOVR:  Packet memory overrun
 *            @arg CTR:     Correct transfer
 */
#define         USB_IT_DISABLE( HANDLE, IT_NAME)        \
    (USB_REG_BIT(HANDLE,CNTR,IT_NAME##M) = 0)

/**
 * @brief  Get the specified USB flag.
 * @param  HANDLE: specifies the USB Handle.
 * @param  FLAG_NAME: specifies the flag to return.
 *         This parameter can be one of the following values:
 *            @arg ESOF:    Expected start of frame
 *            @arg SOF:     Start of frame
 *            @arg RESET:   Reset
 *            @arg SUSP:    Suspend
 *            @arg WKUP:    Wake up
 *            @arg L1REQ:   L1 sleep mode request
 *            @arg ERR:     Error
 *            @arg PMAOVR:  Packet memory overrun
 *            @arg CTR:     Correct transfer
 */
#define         USB_FLAG_STATUS( HANDLE, FLAG_NAME)     \
    (USB_REG_BIT(HANDLE,ISTR,FLAG_NAME))

/**
 * @brief  Clear the specified USB flag.
 * @param  HANDLE: specifies the USB Handle.
 * @param  FLAG_NAME: specifies the flag to clear.
 *         This parameter can be one of the following values:
 *            @arg ESOF:    Expected start of frame
 *            @arg SOF:     Start of frame
 *            @arg RESET:   Reset
 *            @arg SUSP:    Suspend
 *            @arg WKUP:    Wake up
 *            @arg ERR:     Error
 *            @arg PMAOVR:  Packet memory overrun
 *            @arg CTR:     Correct transfer
 */
#define         USB_FLAG_CLEAR(HANDLE, FLAG_NAME)       \
    (USB_REG_BIT(HANDLE,ISTR,FLAG_NAME) = 0)

/** @brief USB Wake up line number */
#define         USB_WAKEUP_EXTI_LINE            18

/** @} */

/** @addtogroup USB_Exported_Functions
 * @{ */
void            USB_vInit               (USB_HandleType * pxUSB, const USB_InitType * pxConfig);
void            USB_vDeinit             (USB_HandleType * pxUSB);

void            USB_vStart_IT           (USB_HandleType * pxUSB);
void            USB_vStop_IT            (USB_HandleType * pxUSB);

void            USB_vSetAddress         (USB_HandleType * pxUSB, uint8_t ucAddress);
void            USB_vCtrlEpOpen         (USB_HandleType * pxUSB);

void            USB_vEpOpen             (USB_HandleType * pxUSB, uint8_t ucEpAddress,
                                         USB_EndPointType eType, uint16_t usMaxPacketSize);
void            USB_vEpClose            (USB_HandleType * pxUSB, uint8_t ucEpAddress);
void            USB_vEpSetStall         (USB_HandleType * pxUSB, uint8_t ucEpAddress);
void            USB_vEpClearStall       (USB_HandleType * pxUSB, uint8_t ucEpAddress);

void            USB_vEpSend             (USB_HandleType * pxUSB, uint8_t ucEpAddress,
                                         const uint8_t * pucData, uint16_t usLength);
void            USB_vEpReceive          (USB_HandleType * pxUSB, uint8_t ucEpAddress,
                                         uint8_t * pucData, uint16_t usLength);

void            USB_vSetRemoteWakeup    (USB_HandleType * pxUSB);
void            USB_vClearRemoteWakeup  (USB_HandleType * pxUSB);

void            USB_vIRQHandler         (USB_HandleType * pxUSB);

USB_ChargerType USB_eChargerDetect      (USB_HandleType * pxUSB);

/* Used internally, has a weak definition */
void            USB_vAllocateEPs        (USB_HandleType * pxUSB);
/** @} */

/** @} */

#define XPD_USB_API
#include <xpd_rcc_pc.h>
#include <xpd_syscfg.h>
#undef XPD_USB_API

#endif /* defined(USB) */

#ifdef __cplusplus
}
#endif

#endif /* __XPD_USB_H_ */
