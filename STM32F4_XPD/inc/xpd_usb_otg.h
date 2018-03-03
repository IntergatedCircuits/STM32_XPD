/**
  ******************************************************************************
  * @file    xpd_usb_otg.h
  * @author  Benedek Kupper
  * @version 0.1
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
#ifndef __XPD_USB_OTG_H_
#define __XPD_USB_OTG_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <xpd_common.h>

#if defined(USB_OTG_FS)
#include <xpd_usb_wrapper.h>

/** @defgroup USB
 * @{ */

/** @defgroup USB_Exported_Types USB Exported Types
 * @{ */

/** @brief USB Power Source types */
typedef enum
{
    USB_BCD_NO_DATA_CONTACT          = 0, /*!< Data lines are floating on the device side */
    USB_BCD_STANDARD_DOWNSTREAM_PORT = 1, /*!< Standard Downstream Port */
    USB_BCD_CHARGING_DOWNSTREAM_PORT = 2, /*!< Charging Downstream Port detected */
    USB_BCD_DEDICATED_CHARGING_PORT  = 3, /*!< Dedicated Charging Port detected */
    USB_BCD_PS2_PROPRIETARY_PORT     = 4, /*!< PS2 or proprietary charging port detected */
    USB_BCD_NOT_SUPPORTED            = 0xFF /*!< Battery Charge Detection is not supported on the device */
}USB_ChargerType;
/** @} */


/** @defgroup USB_Exported_Macros USB Exported Macros
 * @{ */

/**
 * @brief USB OTG Instance to handle binder macro
 * @param HANDLE: specifies the peripheral handle.
 * @param INSTANCE: specifies the USB OTG peripheral instance.
 */
#define         USB_INST2HANDLE(HANDLE,INSTANCE)                \
    ((HANDLE)->Inst    = (USB_OTG_TypeDef*)(INSTANCE))

/**
 * @brief USB register bit accessing macro
 * @param HANDLE: specifies the peripheral handle.
 * @param REG: specifies the register name.
 * @param BIT: specifies the register bit name.
 */
#define USB_REG_BIT(HANDLE, REG, BIT) ((HANDLE)->Inst->REG.b.BIT)

/**
 * @brief  Enable the specified USB interrupt.
 * @param  HANDLE: specifies the USB Handle.
 * @param  IT_NAME: specifies the interrupt to enable.
 *         This parameter can be one of the following values:
 *            @arg MMIS:    Mode mismatch
 *            @arg SOF:     Start of frame
 */
#define         USB_IT_ENABLE( HANDLE, IT_NAME)         \
    (USB_REG_BIT(HANDLE,GINTMSK,IT_NAME) = 1)

/**
 * @brief  Disable the specified USB interrupt.
 * @param  HANDLE: specifies the USB Handle.
 * @param  IT_NAME: specifies the interrupt to disable.
 *         This parameter can be one of the following values:
 *            @arg MMIS:    Mode mismatch
 *            @arg SOF:     Start of frame
 */
#define         USB_IT_DISABLE( HANDLE, IT_NAME)        \
    (USB_REG_BIT(HANDLE,GINTMSK,IT_NAME) = 1)

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
 *            @arg ERR:     Error
 *            @arg PMAOVR:  DMA overrun
 *            @arg CTR:     Correct transfer
 */
#define         USB_FLAG_STATUS( HANDLE, FLAG_NAME)     \
    (USB_REG_BIT(HANDLE,GINTSTS,FLAG_NAME))

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
 *            @arg PMAOVR:  DMA overrun
 *            @arg CTR:     Correct transfer
 */
#define         USB_FLAG_CLEAR(HANDLE, FLAG_NAME)    \
    ((HANDLE)->Inst->GINTSTS.w = USB_OTG_GINTSTS_##FLAG_NAME)

/** @brief USB OTG FS Wake up line number */
#define USB_OTG_FS_WAKEUP_EXTI_LINE     18

/** @brief USB OTG HS Wake up line number */
#define USB_OTG_HS_WAKEUP_EXTI_LINE     20


#define USB_OTG_GINTMSK_SOF             USB_OTG_GINTMSK_SOFM
#define USB_OTG_GINTMSK_MMIS            USB_OTG_GINTMSK_MMISM
#define USB_OTG_GINTMSK_RXFLVL          USB_OTG_GINTMSK_RXFLVLM
#define USB_OTG_GINTMSK_NPTXFE          USB_OTG_GINTMSK_NPTXFEM
#define USB_OTG_GINTMSK_GINAKEFF        USB_OTG_GINTMSK_GINAKEFFM
#define USB_OTG_GINTMSK_GONAKEFF        USB_OTG_GINTMSK_GONAKEFFM
#define USB_OTG_GINTMSK_ESUSP           USB_OTG_GINTMSK_ESUSPM
#define USB_OTG_GINTMSK_USBSUSP         USB_OTG_GINTMSK_USBSUSPM
#define USB_OTG_GINTMSK_ENUMDNE         USB_OTG_GINTMSK_ENUMDNEM
#define USB_OTG_GINTMSK_ISOODRP         USB_OTG_GINTMSK_ISOODRPM
#define USB_OTG_GINTMSK_EOPF            USB_OTG_GINTMSK_EOPFM
#define USB_OTG_GINTMSK_EPMIS           USB_OTG_GINTMSK_EPMISM
#define USB_OTG_GINTMSK_IISOIXFR        USB_OTG_GINTMSK_IISOIXFRM
#define USB_OTG_GINTMSK_PXFRM_IISOOXFR  USB_OTG_GINTMSK_PXFRM_IISOOXFRM
#define USB_OTG_GINTMSK_FSUSP           USB_OTG_GINTMSK_FSUSPM
#define USB_OTG_GINTMSK_PRTI            USB_OTG_GINTMSK_PRTIM
#define USB_OTG_GINTMSK_HCI             USB_OTG_GINTMSK_HCIM
#define USB_OTG_GINTMSK_PTXFE           USB_OTG_GINTMSK_PTXFEM
#define USB_OTG_GINTMSK_CIDSCHG         USB_OTG_GINTMSK_CIDSCHGM
#define USB_OTG_GINTMSK_SRQI            USB_OTG_GINTMSK_SRQIM
#define USB_OTG_GINTMSK_WUI             USB_OTG_GINTMSK_WUIM



void            USB_vDevInit            (USB_HandleType * pxUSB, const USB_InitType * pxConfig);
void            USB_vDevDeinit          (USB_HandleType * pxUSB);

void            USB_vDevStart_IT        (USB_HandleType * pxUSB);
void            USB_vDevStop_IT         (USB_HandleType * pxUSB);

void            USB_vSetAddress         (USB_HandleType * pxUSB, uint8_t ucAddress);

void            USB_vEpOpen             (USB_HandleType * pxUSB, uint8_t ucEpAddress,
                                         USB_EndPointType eType, uint16_t usMaxPacketSize);
void            USB_vEpClose            (USB_HandleType * pxUSB, uint8_t ucEpAddress);
void            USB_vEpSetStall         (USB_HandleType * pxUSB, uint8_t ucEpAddress);
void            USB_vEpClearStall       (USB_HandleType * pxUSB, uint8_t ucEpAddress);

void            USB_vEpSend             (USB_HandleType * pxUSB, uint8_t ucEpAddress,
                                         const uint8_t * pucData, uint16_t usLength);
void            USB_vEpReceive          (USB_HandleType * pxUSB, uint8_t ucEpAddress,
                                         uint8_t * pucData, uint16_t usLength);
void            USB_vEpFlush            (USB_HandleType * pxUSB, uint8_t ucEpAddress);

void            USB_vSetRemoteWakeup    (USB_HandleType * pxUSB);
void            USB_vClearRemoteWakeup  (USB_HandleType * pxUSB);

USB_SpeedType   USB_eDevSpeed           (USB_HandleType * pxUSB);

USB_ChargerType USB_eChargerDetect      (USB_HandleType * pxUSB);

void            USB_vDevIRQHandler      (USB_HandleType * pxUSB);

/* Used internally, has a weak definition */
void            USB_vAllocateEPs        (USB_HandleType * pxUSB);

/**
 * @brief Sets the USB PHY clock status.
 * @param pxUSB: pointer to the USB handle structure
 * @param NewState: Enable or disable PHY clock
 */
__STATIC_INLINE void USB_vPhyClockCtrl(USB_HandleType * pxUSB, FunctionalState NewState)
{
    USB_REG_BIT(pxUSB, PCGCCTL, STOPCLK) = ~NewState;
}

/** @} */

#define XPD_USB_API
#include <xpd_rcc_pc.h>
#include <xpd_syscfg.h>
#undef XPD_USB_API

#endif /* USB_OTG_FS */

#ifdef __cplusplus
}
#endif

#endif /* __XPD_USB_OTG_H_ */
