/**
  ******************************************************************************
  * @file    xpd_usb.h
  * @author  Benedek Kupper
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
#ifndef __XPD_USB_H_
#define __XPD_USB_H_

#include "xpd_common.h"
#include "xpd_config.h"

#ifdef USB_OTG_FS
/** @defgroup USB
 * @{ */

/** @defgroup USB_Exported_Types USB Exported Types
 * @{ */

/** @brief USB device speed types */
typedef enum
{
    USB_SPEED_FULL = 0, /*!< Default USB device speed */
#ifdef USB_OTG_HS
    USB_SPEED_HIGH = 1, /*!< High speed is only available with ULPI physical interface in HS core */
#endif
}USB_SpeedType;

/** @brief USB configuration structure */
typedef struct
{
    USB_SpeedType   Speed;              /*!< Device speed selection */
    FunctionalState SOF;                /*!< StartOfFrame signal interrupt and callback activation */
    FunctionalState LowPowerMode;       /*!< Low power mode activation */
#ifdef USB_OTG_GLPMCFG_LPMEN
    FunctionalState LinkPowerMgmt;      /*!< Link Power Management L1 sleep mode support */
#endif
#if (USB_DATA_WORD_ALIGNED == 1) && defined(USB_OTG_GAHBCFG_DMAEN)
    FunctionalState DMA;                /*!< Use dedicated DMA for data transfer */
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

/** @brief USB Endpoint management structure */
typedef struct
{
    DataStreamType      Transfer;       /*!< Endpoint data transfer context */
    uint16_t            FifoSize;       /*!< Data FIFO size */
    uint16_t            MaxPacketSize;  /*!< Endpoint Max packet size [0..512] */
    USB_EndPointType    Type;           /*!< Endpoint type */
    boolean_t           Stalled;        /*!< Endpoint stall status */
}USB_EndPointHandleType;

/** @brief Device Link power state */
typedef enum
{
    USB_LPM_L0 = 0,     /*!< Device connected and active */
#ifdef USB_OTG_GLPMCFG_LPMEN
    USB_LPM_L1 = 1,     /*!< Device in L1 sleep mode */
#endif
    USB_LPM_L2 = 2,     /*!< Device suspended */
    USB_LPM_L3 = 3,     /*!< Device disconnected from bus */
}USB_LinkStateType;

#ifdef USB_OTG_GCCFG_BCDEN
/** @brief USB Power Source types */
typedef enum
{
    USB_BCD_NO_DATA_CONTACT          = 0, /*!< Data lines are floating on the device side */
    USB_BCD_STANDARD_DOWNSTREAM_PORT = 1, /*!< Standard Downstream Port */
    USB_BCD_CHARGING_DOWNSTREAM_PORT = 2, /*!< Charging Downstream Port detected */
    USB_BCD_DEDICATED_CHARGING_PORT  = 3, /*!< Dedicated Charging Port detected */
    USB_BCD_PS2_PROPRIETARY_PORT     = 4, /*!< PS2 or proprietary charging port detected */
}USB_ChargerType;
#endif

/** @brief USB Handle structure */
typedef struct
{
    USB_OTG_TypeDef * Inst;                                     /*!< The address of the peripheral instance used by the handle */
#ifdef USB_OTG_BB
    USB_OTG_BitBand_TypeDef * Inst_BB;                          /*!< The address of the peripheral instance in the bit-band region */
#endif
    void *                      User;                           /*!< Pointer to upper stack handler */
    struct {
        XPD_HandleCallbackType DepInit;                         /*!< Callback to initialize module dependencies */
        XPD_HandleCallbackType DepDeinit;                       /*!< Callback to restore module dependencies */
        int (*SetupStage)       (void *, uint8_t *);            /*!< SETUP packet received */
        int (*DataOutStage)     (void *, uint8_t, uint8_t *);   /*!< OUT data received */
        int (*DataInStage)      (void *, uint8_t, uint8_t *);   /*!< IN data transmitted */
        int (*SOF)              (void *);                       /*!< StartOfFrame signal */
        int (*Reset)            (void *);                       /*!< Device reset */
        int (*Suspend)          (void *);                       /*!< Suspend request */
        int (*Resume)           (void *);                       /*!< Resume request */
        int (*Connected)        (void *);                       /*!< Device connected to bus */
        int (*Disconnected)     (void *);                       /*!< Device disconnected from bus */
    }Callbacks;                                                 /*   Handle Callbacks */
    uint32_t                    Setup[12];                      /*!< Setup packet buffer */
    struct {
        USB_EndPointHandleType  IN[6];                          /*!< IN endpoint status */
        USB_EndPointHandleType  OUT[6];                         /*!< OUT endpoint status */
    }EP;                                                        /*   Endpoint management */
    uint8_t                     DeviceAddress;                  /*!< USB Address */
    FunctionalState             LowPowerMode;                   /*!< Low power mode activation */
    USB_SpeedType               Speed;                          /*!< Currently available speed */
    USB_LinkStateType           LinkState;                      /*!< Device link status */
#if (USB_DATA_WORD_ALIGNED == 1) && defined(USB_OTG_GAHBCFG_DMAEN)
    FunctionalState             DMA;                            /*!< DMA activation */
#endif
}USB_HandleType;

/** @} */

/** @defgroup USB_Exported_Macros USB Exported Macros
 * @{ */

/**
 * @brief  USB Handle initializer macro
 * @param  INSTANCE: specifies the USB peripheral instance.
 * @param  INIT_FN: specifies the dependency initialization function to call back.
 * @param  DEINIT_FN: specifies the dependency deinitialization function to call back.
 */
#define         NEW_USB_HANDLE(INSTANCE,INIT_FN,DEINIT_FN)          \
     {  .Inst = (INSTANCE), .LinkState = USB_LPM_L3,                \
        .Callbacks.DepInit = (INIT_FN), .Callbacks.DepDeinit = (DEINIT_FN)}

#ifdef USB_OTG_BB
#define USB_REG_BIT(HANDLE, REG, BIT) ((HANDLE)->Inst_BB->REG.BIT)
#else
#define USB_REG_BIT(HANDLE, REG, BIT) ((HANDLE)->Inst->REG.b.BIT)
#endif /* USB_OTG_BB */

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
#define         XPD_USB_GetFlag( HANDLE, FLAG_NAME)     \
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
#define         XPD_USB_ClearFlag(HANDLE, FLAG_NAME)    \
    ((HANDLE)->Inst->GINTSTS.w = USB_OTG_GINTSTS_##FLAG_NAME)

/** @brief USB OTG FS Wake up line number */
#define USB_OTG_FS_WAKEUP_EXTI_LINE     18

/** @brief USB OTG HS Wake up line number */
#define USB_OTG_HS_WAKEUP_EXTI_LINE     20

/** @brief USB Wake up line initialization values */
#define         USB_WAKEUP_EXTI_INIT                    \
    {   .ITCallback = NULL,                             \
        .Edge       = EDGE_RISING,                      \
        .Reaction   = REACTION_IT }

/**
 * @brief  Macro to access the BESL value in the peripheral.
 * @param  HANDLE: specifies the USB Handle.
 */
#define         XPD_USB_BESL(HANDLE)                ((HANDLE)->Inst->GLPMCFG.b.BESL)

/** @} */

/** @addtogroup USB_Exported_Functions
 * @{ */
XPD_ReturnType  XPD_USB_Init                    (USB_HandleType * husb, const USB_InitType * Config);
XPD_ReturnType  XPD_USB_Deinit                  (USB_HandleType * husb);

void            XPD_USB_Start                   (USB_HandleType * husb);
void            XPD_USB_Stop                    (USB_HandleType * husb);

void            XPD_USB_SetAddress              (USB_HandleType * husb, uint8_t Address);

void            XPD_USB_EP_BufferInit           (USB_HandleType * husb, uint8_t EpAddress,
                                                 uint16_t BufferSize);

void            XPD_USB_EP_Open                 (USB_HandleType * husb, uint8_t EpAddress,
                                                 USB_EndPointType Type, uint16_t MaxPacketSize);
void            XPD_USB_EP_Close                (USB_HandleType * husb, uint8_t EpAddress);

void            XPD_USB_EP_Receive              (USB_HandleType * husb, uint8_t EpAddress,
                                                 uint8_t * Data, uint16_t Length);
uint16_t        XPD_USB_EP_GetRxCount           (USB_HandleType * husb, uint8_t EpAddress);

void            XPD_USB_EP_Transmit             (USB_HandleType * husb, uint8_t EpAddress,
                                                 uint8_t * Data, uint16_t Length);
void            XPD_USB_EP_Flush                (USB_HandleType * husb, uint8_t EpAddress);

void            XPD_USB_EP_SetStall             (USB_HandleType * husb, uint8_t EpAddress);
void            XPD_USB_EP_ClearStall           (USB_HandleType * husb, uint8_t EpAddress);

void            XPD_USB_IRQHandler              (USB_HandleType * husb);

void            XPD_USB_PHY_ClockCtrl           (USB_HandleType * husb, FunctionalState NewState);

void            XPD_USB_ActivateRemoteWakeup    (USB_HandleType * husb);
void            XPD_USB_DeActivateRemoteWakeup  (USB_HandleType * husb);

#ifdef USB_OTG_GCCFG_BCDEN
USB_ChargerType XPD_USB_ChargerDetect           (USB_HandleType * husb);
#endif
/** @} */

/** @} */

#define XPD_USB_API
#include "xpd_rcc_pc.h"
#undef XPD_USB_API

#endif /* USB_OTG_FS */

#endif /* __XPD_USB_H_ */
