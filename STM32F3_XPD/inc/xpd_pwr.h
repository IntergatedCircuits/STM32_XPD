/**
  ******************************************************************************
  * @file    xpd_pwr.h
  * @author  Benedek Kupper
  * @version V0.1
  * @date    2016-11-01
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
 *            @arg VREFINTRDYF: VREFINT reference voltage ready
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

void            XPD_PWR_WakeUpPin_Enable    (uint8_t WakeUpPin);
void            XPD_PWR_WakeUpPin_Disable   (uint8_t WakeUpPin);

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
    PWR_PVDLEVEL_2V2 = 0, /*!< 2.2V voltage detector level */
    PWR_PVDLEVEL_2V3 = 1, /*!< 2.3V voltage detector level */
    PWR_PVDLEVEL_2V4 = 2, /*!< 2.4V voltage detector level */
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

/** @} */

#endif /* __XPD_PWR_H_ */
