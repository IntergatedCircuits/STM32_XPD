/**
  ******************************************************************************
  * @file    xpd_pwr.h
  * @author  Benedek Kupper
  * @version V0.1
  * @date    2017-01-28
  * @brief   STM32 eXtensible Peripheral Drivers Power Module
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
#ifndef __XPD_PWR_H_
#define __XPD_PWR_H_

#include "xpd_common.h"
#include "xpd_config.h"

/** @defgroup PWR
 * @{ */

/** @defgroup PWR_Core PWR Core
 * @{ */

/** @defgroup PWR_Exported_Types PWR Exported Types
 * @{ */

/** @brief PWR regulator types */
typedef enum
{
    PWR_MAINREGULATOR     = 0, /*!< Main regulator ON in Sleep/Stop mode */
    PWR_LOWPOWERREGULATOR = 1, /*!< Low Power regulator ON in Sleep/Stop mode */
#ifdef PWR_CR_UDEN
    PWR_MAINREGULATOR_UNDERDRIVE_ON     = PWR_CR_MRUDS,     /*!< Main regulator ON in Underdrive mode */
    PWR_LOWPOWERREGULATOR_UNDERDRIVE_ON = PWR_CR_LPUDS | 1, /*!< Low Power regulator ON in Underdrive mode */
#endif
}PWR_RegulatorType;

/** @} */

/** @defgroup PWR_Exported_Macros PWR Exported Macros
 * @{ */

/**
 * @brief  Get the specified PWR flag.
 * @param  FLAG_NAME: specifies the flag to return.
 *         This parameter can be one of the following values:
 *            @arg WUF:         Wake up flag
 *            @arg SBF:         Standby flag
 *            @arg PVDO:        Power Voltage Detector output flag
 *            @arg BRR:         Backup regulator ready flag
 *            @arg VOSRDY:      Regulator voltage scaling output selection ready flag
 */
#define         XPD_PWR_GetFlag(FLAG_NAME)      \
    (PWR_REG_BIT(CSR,FLAG_NAME))

/**
 * @brief  Clear the specified PWR flag.
 * @param  FLAG_NAME: specifies the flag to clear.
 *         This parameter can be one of the following values:
 *            @arg WUF:         Wake up flag
 *            @arg SBF:         Standby flag
 */
#define         XPD_PWR_ClearFlag(FLAG_NAME)    \
    (PWR_REG_BIT(CR,C##FLAG_NAME) = 1)

#ifdef PWR_BB
#define PWR_REG_BIT(_REG_NAME_, _BIT_NAME_) (PWR_BB->_REG_NAME_._BIT_NAME_)
#else
#define PWR_REG_BIT(_REG_NAME_, _BIT_NAME_) (PWR->_REG_NAME_.b._BIT_NAME_)
#endif

/** @} */

/** @addtogroup PWR_Exported_Functions PWR Exported Functions
 * @{ */
void            XPD_PWR_SleepMode           (ReactionType WakeUpOn);
void            XPD_PWR_StopMode            (ReactionType WakeUpOn, PWR_RegulatorType Regulator);
void            XPD_PWR_StandbyMode         (void);

XPD_ReturnType  XPD_PWR_BackupRegulatorCtrl (FunctionalState NewState);

void            XPD_PWR_WakeUpPin_Enable    (uint8_t WakeUpPin);
void            XPD_PWR_WakeUpPin_Disable   (uint8_t WakeUpPin);
#ifdef PWR_CSR_WUPP
void            XPD_PWR_WakeUpPin_Polarity  (uint8_t WakeUpPin, EdgeType RisingOrFalling);
#endif

/**
 * @brief Send Event on Pending bit enables disabled interrupts to wake up
 *        a system from WaitForEvent.
 * @param NewState: the new SEVONPEND value to set
 */
__STATIC_INLINE void XPD_PWR_SEVONPEND(FunctionalState NewState)
{
    SCB->SCR.b.SEVONPEND = NewState;
}

/**
 * @brief Sleep on Exit bit enables to enter sleep mode
 *        on return from an ISR to Thread mode.
 * @param NewState: the new SLEEPONEXIT value to set
 */
__STATIC_INLINE void XPD_PWR_SLEEPONEXIT(FunctionalState NewState)
{
    SCB->SCR.b.SLEEPONEXIT = NewState;
}

/**
 * @brief Sleep Deep bit enables to enter deep sleep mode.
 * @param NewState: the new SLEEPONEXIT value to set
 */
__STATIC_INLINE void XPD_PWR_SLEEPDEEP(FunctionalState NewState)
{
    SCB->SCR.b.SLEEPDEEP = NewState;
}

/** @} */

/** @} */

/** @defgroup PWR_Peripherals PWR Peripherals
 * @{ */

/** @defgroup PWR_Peripherals_Exported_Functions PWR Peripherals Exported Functions
 * @{ */

/**
 * @brief Enables or disables access to the backup domain
 *        (RTC registers, RTC backup data registers when present).
 * @param NewState: the new backup access state to set
 * @note  If the HSE divided by 32 is used as the RTC clock, the
 *         Backup Domain Access should be kept enabled.
 */
__STATIC_INLINE void XPD_PWR_BackupAccess(FunctionalState NewState)
{
    PWR_REG_BIT(CR,DBP) = NewState;
}

#ifdef PWR_CR_FPDS
/**
 * @brief Sets the Flash Power Down state in Stop mode.
 * @param NewState: the new Flash power down state to set
 */
__STATIC_INLINE void XPD_PWR_FlashPowerDown(FunctionalState NewState)
{
    PWR_REG_BIT(CR, FPDS) = NewState;
}
#endif /* PWR_CR_FPDS */

/** @} */

/** @} */

#ifdef PWR_CR_PLS
#include "xpd_exti.h"

/** @defgroup PWR_Voltage_Detector PWR Voltage Detector
 * @{ */

/** @defgroup PWR_PVD_Exported_Types PWR PVD Exported Types
 * @{ */

/** @brief PVD levels */
typedef enum
{
    PWR_PVDLEVEL_2V0 = 0, /*!< 2.0V voltage detector level */
    PWR_PVDLEVEL_2V1 = 1, /*!< 2.1V voltage detector level */
    PWR_PVDLEVEL_2V3 = 2, /*!< 2.3V voltage detector level */
    PWR_PVDLEVEL_2V5 = 3, /*!< 2.5V voltage detector level */
    PWR_PVDLEVEL_2V6 = 4, /*!< 2.6V voltage detector level */
    PWR_PVDLEVEL_2V7 = 5, /*!< 2.7V voltage detector level */
    PWR_PVDLEVEL_2V8 = 6, /*!< 2.8V voltage detector level */
    PWR_PVDLEVEL_2V9 = 7  /*!< 2.9V voltage detector level */
} PWR_PVDLevelType;

/** @brief PVD configuration structure */
typedef struct
{
    PWR_PVDLevelType Level; /*!< Voltage detector level to trigger reaction */
    EXTI_InitType    ExtI;  /*!< External interrupt configuration */
} PWR_PVD_InitType;

/** @} */

/** @defgroup PWR_PVD_Exported_Macros PWR PVD Exported Macros
 * @{ */

/** @brief PVD EXTI line number */
#define PWR_PVD_EXTI_LINE               16
/** @} */

/** @defgroup PWR_PVD_Exported_Functions PWR PVD Exported Functions
 * @{ */

/**
 * @brief Configures the voltage threshold monitoring by the Power Voltage Detector(PVD).
 * @param Config: configuration structure that contains the monitored voltage level
 *         and the EXTI configuration.
 */
__STATIC_INLINE void XPD_PWR_PVD_Init(const PWR_PVD_InitType * Config)
{
    /* Set PLS bits according to PVDLevel value */
    PWR->CR.b.PLS = Config->Level;

    /* External interrupt line 16 Connected to the PVD EXTI Line */
    XPD_EXTI_Init(PWR_PVD_EXTI_LINE, &Config->ExtI);
}

/**
 * @brief Enables the Power Voltage Detector (PVD).
 */
__STATIC_INLINE void XPD_PWR_PVD_Enable(void)
{
    PWR_REG_BIT(CR,PVDE) = ENABLE;
}

/**
 * @brief Disables the Power Voltage Detector (PVD).
 */
__STATIC_INLINE void XPD_PWR_PVD_Disable(void)
{
    PWR_REG_BIT(CR,PVDE) = DISABLE;
}

/** @} */

/** @} */
#endif /* PWR_CR_PLS */

#ifdef PWR_CR_VOS
/** @defgroup PWR_Regulator_Voltage_Scaling PWR Regulator Voltage Scaling
 * @{ */

/** @defgroup PWR_Regulator_Voltage_Scaling_Exported_Types PWR Regulator Voltage Scaling Exported Types
 * @{ */

/** @brief Regulator Voltage Scaling modes */
typedef enum
{
#ifdef PWR_CR_VOS_1
    PWR_REGVOLT_SCALE1 = 3, /*!< Scale 1 mode(default value at reset): the maximum value of fHCLK is 168 MHz. It can be extended to
                                 180 MHz by activating the over-drive mode. */
    PWR_REGVOLT_SCALE2 = 2, /*!< Scale 2 mode: the maximum value of fHCLK is 144 MHz. It can be extended to
                                 168 MHz by activating the over-drive mode. */
    PWR_REGVOLT_SCALE3 = 1  /*!< Scale 3 mode: the maximum value of fHCLK is 120 MHz. */
#else
    PWR_REGVOLT_SCALE1 = 1, /*!< Scale 1 mode(default value at reset): the maximum value of fHCLK = 168 MHz. */
    PWR_REGVOLT_SCALE2 = 0  /*!< Scale 2 mode: the maximum value of fHCLK = 144 MHz. */
#endif
}PWR_RegVoltScaleType;

/** @} */

/** @addtogroup PWR_Regulator_Voltage_Scaling_Exported_Functions
 * @{ */
XPD_ReturnType  XPD_PWR_VoltageScaleConfig  (PWR_RegVoltScaleType Scaling);
#if defined(PWR_CR_MRLVDS) && defined(PWR_CR_LPLVDS)
void            XPD_PWR_RegLowVoltageConfig (PWR_RegulatorType Regulator, FunctionalState NewState);
#endif

/**
 * @brief Returns the current Regulator Voltage Scaling configuration.
 * @return The active voltage scaling
 */
__STATIC_INLINE PWR_RegVoltScaleType XPD_PWR_GetVoltageScale(void)
{
    return PWR->CR.b.VOS;
}

/** @} */

/** @} */
#endif /* PWR_CR_VOS */

#ifdef PWR_CR_ODEN
/** @defgroup PWR_OverDrive_Mode PWR OverDrive Mode
 * @{ */

/** @addtogroup PWR_OverDrive_Mode_Exported_Functions
 * @{ */
XPD_ReturnType  XPD_PWR_OverDrive_Enable    (void);
XPD_ReturnType  XPD_PWR_OverDrive_Disable   (void);
/** @} */

/** @} */
#endif /* PWR_CR_ODEN */

/** @} */

#endif /* __XPD_PWR_H_ */
