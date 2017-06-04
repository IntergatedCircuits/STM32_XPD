/**
  ******************************************************************************
  * @file    xpd_usb.h
  * @author  Benedek Kupper
  * @version V0.2
  * @date    2017-05-04
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

#ifdef USB
/** @defgroup USB
 * @{ */

/** @defgroup USB_Exported_Types USB Exported Types
 * @{ */

/** @brief USB device speed types */
typedef enum
{
    USB_SPEED_FULL = 0, /*!< Default USB device speed */
}USB_SpeedType;

/** @brief USB configuration structure */
typedef struct
{
    USB_SpeedType   Speed;              /*!< Device speed selection */
    FunctionalState SOF;                /*!< StartOfFrame signal interrupt and callback activation */
    FunctionalState LowPowerMode;       /*!< Low power mode activation */
#ifdef USB_LPMCSR_LMPEN
    FunctionalState LinkPowerMgmt;      /*!< Link Power Management L1 sleep mode support */
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
    uint16_t            PacketAddress;  /*!< PMA Address [0..1024] */
    uint16_t            MaxPacketSize;  /*!< Endpoint Max packet size [0..512] */
    USB_EndPointType    Type;           /*!< Endpoint type */
    boolean_t           Stalled;        /*!< Endpoint stall status */
    uint8_t             RegId;          /*!< Endpoint register ID */
    FunctionalState     DoubleBuffer;   /*!< Double buffer configuration */
}USB_EndPointHandleType;

/** @brief Device Link power state */
typedef enum
{
    USB_LPM_L0 = 0,     /*!< Device connected and active */
#ifdef USB_LPMCSR_LMPEN
    USB_LPM_L1 = 1,     /*!< Device in L1 sleep mode */
#endif
    USB_LPM_L2 = 2,     /*!< Device suspended */
    USB_LPM_L3 = 3,     /*!< Device disconnected from bus */
}USB_LinkStateType;

#ifdef USB_BCDR_BCDEN
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
    void *                      User;                           /*!< Pointer to upper stack handler */
    struct {
        XPD_HandleCallbackType DepInit;                         /*!< Callback to initialize module dependencies */
        XPD_HandleCallbackType DepDeinit;                       /*!< Callback to restore module dependencies */
#ifndef USB_BCDR_DPPU
        XPD_CtrlFnType ConnectionStateCtrl;                     /*!< Callback to set USB device bus line connection state */
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
    }Callbacks;                                                 /*   Handle Callbacks */
    uint32_t                    Setup[12];                      /*!< Setup packet buffer */
    struct {
        USB_EndPointHandleType  IN[8];                          /*!< IN endpoint status */
        USB_EndPointHandleType  OUT[8];                         /*!< OUT endpoint status */
    }EP;                                                        /*   Endpoint management */
    uint16_t                    BdtSize;                        /*!< PMA Buffer Descriptor Table size */
    uint16_t                    PmaOffset;                      /*!< End of Packet Memory */
    uint8_t                     DeviceAddress;                  /*!< USB Address */
    FunctionalState             LowPowerMode;                   /*!< Low power mode configuration */
    USB_LinkStateType           LinkState;                      /*!< Device link status */
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
     {  .User = NULL, .LinkState = USB_LPM_L3,                      \
        .Callbacks.DepInit = (INIT_FN), .Callbacks.DepDeinit = (DEINIT_FN)}

#ifdef USB_BB
/**
 * @brief USB register bit accessing macro
 * @param HANDLE: specifies the peripheral handle.
 * @param REG: specifies the register name.
 * @param BIT: specifies the register bit name.
 */
#define USB_REG_BIT(HANDLE, REG_NAME, BIT_NAME) (USB_BB->REG_NAME.BIT_NAME)

#else
/**
 * @brief USB register bit accessing macro
 * @param HANDLE: specifies the peripheral handle.
 * @param REG: specifies the register name.
 * @param BIT: specifies the register bit name.
 */
#define USB_REG_BIT(HANDLE, REG_NAME, BIT_NAME) (USB->REG_NAME.b.BIT_NAME)

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
#define         XPD_USB_EnableIT( HANDLE, IT_NAME)      \
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
#define         XPD_USB_DisableIT( HANDLE, IT_NAME)     \
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
#define         XPD_USB_GetFlag( HANDLE, FLAG_NAME)     \
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
#define         XPD_USB_ClearFlag(HANDLE, FLAG_NAME)    \
    (USB_REG_BIT(HANDLE,ISTR,FLAG_NAME) = 0)

/** @brief USB Wake up line number */
#define         USB_WAKEUP_EXTI_LINE            18

/** @brief USB Wake up line initialization values */
#define         USB_WAKEUP_EXTI_INIT                    \
    {   .ITCallback = NULL,                             \
        .Edge       = EDGE_RISING,                      \
        .Reaction   = REACTION_IT }

/**
 * @brief  Macro to access the BESL value in the peripheral.
 * @param  HANDLE: specifies the USB Handle.
 */
#define         XPD_USB_BESL(HANDLE)                (USB->LPMCSR.b.BESL)

/** @brief Compatibility macro */
#define         XPD_USB_EP_Flush(HANDLE,EP_NUMBER)  ((void)0)

/** @brief Compatibility macro */
#define         XPD_USB_PHY_ClockCtrl(HANDLE,SWITCH)((void)0)

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

void            XPD_USB_EP_SetStall             (USB_HandleType * husb, uint8_t EpAddress);
void            XPD_USB_EP_ClearStall           (USB_HandleType * husb, uint8_t EpAddress);

void            XPD_USB_IRQHandler              (USB_HandleType * husb);

void            XPD_USB_ActivateRemoteWakeup    (USB_HandleType * husb);
void            XPD_USB_DeactivateRemoteWakeup  (USB_HandleType * husb);

#ifdef USB_BCDR_BCDEN
USB_ChargerType XPD_USB_ChargerDetect           (USB_HandleType * husb);
#endif
/** @} */

/** @} */

#define XPD_USB_API
#include "xpd_rcc_pc.h"
#include "xpd_syscfg.h"
#undef XPD_USB_API

#endif /* USB */

#endif /* __XPD_USB_H_ */
